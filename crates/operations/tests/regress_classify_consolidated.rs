//! Regression: `remus_operations::classify` must agree with the ground-truth
//! ray caster, and its winding number must be correct on curved geometry.
//!
//! Two defects motivated consolidating this module onto `remus_check`:
//!
//! 1. `operations::classify` was a ~1000-line near-duplicate of
//!    `check::classify` that never read `inner_wires`. Because the WASM
//!    bindings call the *operations* copy, fixing the hole bug in `check`
//!    alone left every JS consumer on the broken path. It was also wrong in a
//!    way `check` never was: solid material read as `Outside`.
//!
//! 2. `operations::compute_winding_number` was not a winding number at all —
//!    it was a single-ray crossing count returning exactly 1.0 or 0.0, despite
//!    docs promising "generalized winding numbers (robust to gaps, uses
//!    tessellation)". The real thing sums signed solid angles over a
//!    watertight tessellation, which is what `winding_number` now does.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{
    PointClassification, classify_point, classify_point_robust, classify_point_winding,
    winding_number,
};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.05;
const TOL: f64 = 1e-6;

/// Flange r45 h12 fused with a coaxial hub r24 h30, then a bolt hole
/// r3.5 h18 cut through the flange at (34, 0). Its planar caps carry inner
/// wires, which is what the duplicated classifier mishandled.
fn make_flange_with_bolt_hole(topo: &mut Topology) -> SolidId {
    let flange = primitives::make_cylinder(topo, 45.0, 12.0).unwrap();
    let hub = primitives::make_cylinder(topo, 24.0, 30.0).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, flange, hub).unwrap();
    let bolt = primitives::make_cylinder(topo, 3.5, 18.0).unwrap();
    transform_solid(topo, bolt, &Mat4::translation(34.0, 0.0, -3.0)).unwrap();
    boolean(topo, BooleanOp::Cut, fused, bolt).unwrap()
}

/// Probes and their true classification.
fn flange_probes() -> Vec<(Point3, bool, &'static str)> {
    vec![
        (
            Point3::new(34.0, 0.0, -1.0),
            false,
            "below, under bolt hole",
        ),
        (Point3::new(10.0, 0.0, -1.0), false, "below, under hub"),
        (Point3::new(15.0, 0.0, -1.0), false, "below"),
        (Point3::new(20.0, 0.0, -1.0), false, "below"),
        (Point3::new(30.0, 0.0, -1.0), false, "below"),
        (Point3::new(0.0, -30.0, -1.0), false, "below -y"),
        (Point3::new(34.0, 0.0, 6.0), false, "in the bolt hole"),
        (Point3::new(0.0, 0.0, 40.0), false, "above everything"),
        (
            Point3::new(30.0, 0.0, 20.0),
            false,
            "beside hub, above flange",
        ),
        (Point3::new(0.0, 0.0, 6.0), true, "hub/flange core"),
        (Point3::new(0.0, 0.0, 20.0), true, "hub above flange"),
        (Point3::new(40.0, 0.0, 6.0), true, "flange rim"),
        (Point3::new(-34.0, 0.0, 6.0), true, "flange opposite bolt"),
        (Point3::new(0.0, -34.0, 6.0), true, "flange -y"),
        (Point3::new(38.5, 0.0, 6.0), true, "just outside bolt wall"),
        (Point3::new(23.0, 0.0, 25.0), true, "hub wall high"),
    ]
}

/// The WASM-facing ray caster must match the ground truth on holed faces.
#[test]
fn operations_ray_caster_handles_face_holes() {
    let mut topo = Topology::new();
    let solid = make_flange_with_bolt_hole(&mut topo);

    for (p, want_inside, label) in flange_probes() {
        let got = classify_point(&topo, solid, p, DEFLECTION, TOL).unwrap();
        let want = if want_inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(got, want, "probe {p:?} ({label})");
    }
}

/// The winding classifier must agree with the ray caster on the same solid.
#[test]
fn operations_winding_matches_ray_caster() {
    let mut topo = Topology::new();
    let solid = make_flange_with_bolt_hole(&mut topo);

    for (p, want_inside, label) in flange_probes() {
        let w = winding_number(&topo, solid, p, DEFLECTION).unwrap();
        // A real winding number is decisive, not marginal: ~1 or ~0.
        if want_inside {
            assert!(w > 0.9, "interior probe {p:?} ({label}) winding {w}");
        } else {
            assert!(w < 0.1, "exterior probe {p:?} ({label}) winding {w}");
        }

        let want = if want_inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(
            classify_point_winding(&topo, solid, p, DEFLECTION, TOL).unwrap(),
            want,
            "winding probe {p:?} ({label})"
        );
        assert_eq!(
            classify_point_robust(&topo, solid, p, DEFLECTION, TOL).unwrap(),
            want,
            "robust probe {p:?} ({label})"
        );
    }
}

/// The old fan-of-boundary-loop winding read interior points of a plain
/// cylinder as Outside. Nothing about this solid involves holes — it is purely
/// the curved-face defect.
#[test]
fn winding_is_correct_on_curved_faces() {
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();

    for p in [
        Point3::new(0.0, 0.0, 10.0),
        Point3::new(9.0, 0.0, 1.0),
        Point3::new(0.0, -9.0, 19.0),
        Point3::new(5.0, 5.0, 10.0),
        Point3::new(-9.5, 0.0, 10.0),
    ] {
        let w = winding_number(&topo, cyl, p, DEFLECTION).unwrap();
        assert!(w > 0.9, "cylinder interior {p:?} winding {w}");
    }

    for p in [
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(11.0, 0.0, 10.0),
        Point3::new(0.0, 0.0, 21.0),
        Point3::new(20.0, 20.0, 10.0),
    ] {
        let w = winding_number(&topo, cyl, p, DEFLECTION).unwrap();
        assert!(w < 0.1, "cylinder exterior {p:?} winding {w}");
    }
}

/// Spheres and tori exercise pole and seam handling that a boundary-loop fan
/// cannot represent at all.
#[test]
fn winding_is_correct_on_sphere_and_torus() {
    let mut topo = Topology::new();
    let sph = primitives::make_sphere(&mut topo, 8.0, 32).unwrap();
    assert!(winding_number(&topo, sph, Point3::new(0.0, 0.0, 0.0), DEFLECTION).unwrap() > 0.9);
    assert!(winding_number(&topo, sph, Point3::new(7.5, 0.0, 0.0), DEFLECTION).unwrap() > 0.9);
    assert!(winding_number(&topo, sph, Point3::new(20.0, 0.0, 0.0), DEFLECTION).unwrap() < 0.1);

    let mut topo2 = Topology::new();
    let tor = primitives::make_torus(&mut topo2, 10.0, 3.0, 32).unwrap();
    // Inside the tube, not at the doughnut's centre.
    assert!(winding_number(&topo2, tor, Point3::new(10.0, 0.0, 0.0), DEFLECTION).unwrap() > 0.9);
    // The hole in the middle is empty space.
    assert!(winding_number(&topo2, tor, Point3::new(0.0, 0.0, 0.0), DEFLECTION).unwrap() < 0.1);
    assert!(winding_number(&topo2, tor, Point3::new(30.0, 0.0, 0.0), DEFLECTION).unwrap() < 0.1);
}

/// A hollow solid's cavity is outside the material for every classifier.
#[test]
fn hollow_solid_cavity_is_outside() {
    let mut topo = Topology::new();
    let outer = primitives::make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let inner = primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    transform_solid(&mut topo, inner, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, inner).unwrap();
    assert_eq!(topo.solid(hollow).unwrap().inner_shells().len(), 1);

    let cavity = Point3::new(1.5, 1.5, 1.5);
    let material = Point3::new(0.5, 0.5, 0.5);
    assert!(winding_number(&topo, hollow, cavity, DEFLECTION).unwrap() < 0.1);
    assert!(winding_number(&topo, hollow, material, DEFLECTION).unwrap() > 0.9);

    for classify in [
        classify_point,
        classify_point_winding,
        classify_point_robust,
    ] {
        assert_eq!(
            classify(&topo, hollow, cavity, DEFLECTION, TOL).unwrap(),
            PointClassification::Outside
        );
        assert_eq!(
            classify(&topo, hollow, material, DEFLECTION, TOL).unwrap(),
            PointClassification::Inside
        );
    }
}
