//! Regression: `classify_point` must not count a ray crossing where the ray
//! passes through a hole in a face.
//!
//! `remus_check::util::face_polygon` returns only a face's OUTER wire, and
//! the classifier's containment tests used it alone. A GFA boolean result
//! whose planar caps carry inner wires — the r24 hub circle absorbed into the
//! flange's z=0 and z=12 annuli, and the r3.5 bolt hole punched through both
//! — therefore reported a crossing for every ray that travelled up one of
//! those holes. The extra crossing flipped the parity, so exterior probes
//! below the part classified as `Inside`.
//!
//! Pristine primitives were unaffected: `make_cylinder` produces caps with no
//! inner wires, so nothing exercised the missing hole test.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Flange r45 h12 fused with a coaxial hub r24 h30, then a bolt hole
/// r3.5 h18 cut through the flange at (34, 0).
fn make_flange_with_bolt_hole(topo: &mut Topology) -> SolidId {
    let flange = primitives::make_cylinder(topo, 45.0, 12.0).unwrap();
    let hub = primitives::make_cylinder(topo, 24.0, 30.0).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, flange, hub).unwrap();

    let bolt = primitives::make_cylinder(topo, 3.5, 18.0).unwrap();
    transform_solid(topo, bolt, &Mat4::translation(34.0, 0.0, -3.0)).unwrap();
    boolean(topo, BooleanOp::Cut, fused, bolt).unwrap()
}

#[test]
fn boolean_result_has_holed_planar_caps() {
    let mut topo = Topology::new();
    let solid = make_flange_with_bolt_hole(&mut topo);

    // The setup only exercises the bug if the result really does carry faces
    // with inner wires — otherwise the probes below prove nothing.
    let holed = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&fid| !topo.face(fid).unwrap().inner_wires().is_empty())
        .count();
    assert!(
        holed >= 2,
        "expected the z=0 and z=12 caps to carry inner wires, found {holed} holed faces"
    );
}

#[test]
fn points_below_the_part_classify_outside() {
    let mut topo = Topology::new();
    let solid = make_flange_with_bolt_hole(&mut topo);
    let opts = ClassifyOptions::default();

    // 1mm below the z=0 face. Each of these once read `Inside` because a ray
    // crossing the bottom cap inside the bolt hole (or inside the absorbed
    // r24 hub circle) was counted as material.
    let probes = [
        Point3::new(34.0, 0.0, -1.0),
        Point3::new(10.0, 0.0, -1.0),
        Point3::new(15.0, 0.0, -1.0),
        Point3::new(20.0, 0.0, -1.0),
        Point3::new(30.0, 0.0, -1.0),
        Point3::new(0.0, -30.0, -1.0),
        Point3::new(40.0, 0.0, -1.0),
        Point3::new(0.0, 30.0, -1.0),
        Point3::new(-30.0, 0.0, -1.0),
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(-20.0, -20.0, -0.5),
    ];
    for p in probes {
        assert_eq!(
            classify_point(&topo, solid, p, &opts).unwrap(),
            PointClassification::Outside,
            "probe {p:?} is below the part"
        );
    }
}

#[test]
fn bolt_hole_interior_classifies_outside() {
    let mut topo = Topology::new();
    let solid = make_flange_with_bolt_hole(&mut topo);
    let opts = ClassifyOptions::default();

    // The bolt hole is a through-hole: its interior is air, at every height.
    //
    // Ray casting only. `classify_point_winding` (and `classify_point_robust`,
    // which defers to it) still misreads points here: its fan triangulation of
    // a cylindrical face's boundary loop is not that face's surface, so curved
    // solids give a wrong solid-angle sum regardless of hole handling. That is
    // a separate, pre-existing limitation — `classify_point` is the ground
    // truth classifier.
    for z in [0.5, 3.0, 6.0, 9.0, 11.5] {
        let p = Point3::new(34.0, 0.0, z);
        assert_eq!(
            classify_point(&topo, solid, p, &opts).unwrap(),
            PointClassification::Outside,
            "probe {p:?} is inside the bolt hole"
        );
    }
}

#[test]
fn material_still_classifies_inside() {
    let mut topo = Topology::new();
    let solid = make_flange_with_bolt_hole(&mut topo);
    let opts = ClassifyOptions::default();

    // Subtracting holes must not over-reject: real material still reads
    // Inside, including right next to the bolt hole wall.
    let probes = [
        Point3::new(0.0, 0.0, 6.0),
        Point3::new(0.0, 0.0, 20.0),
        Point3::new(40.0, 0.0, 6.0),
        Point3::new(30.0, 0.0, 6.0),
        Point3::new(-34.0, 0.0, 6.0),
        Point3::new(0.0, -34.0, 6.0),
        Point3::new(38.5, 0.0, 6.0),
        Point3::new(20.0, 0.0, 2.0),
        Point3::new(23.0, 0.0, 25.0),
        Point3::new(0.0, 23.0, 28.0),
    ];
    for p in probes {
        assert_eq!(
            classify_point(&topo, solid, p, &opts).unwrap(),
            PointClassification::Inside,
            "probe {p:?} is inside material"
        );
    }
}

/// Pristine primitives were always classified correctly; keep that true.
#[test]
fn pristine_cylinder_unaffected() {
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, 45.0, 12.0).unwrap();
    let opts = ClassifyOptions::default();

    for p in [
        Point3::new(34.0, 0.0, -1.0),
        Point3::new(10.0, 0.0, -1.0),
        Point3::new(0.0, -30.0, -1.0),
        Point3::new(0.0, 0.0, 13.0),
    ] {
        assert_eq!(
            classify_point(&topo, cyl, p, &opts).unwrap(),
            PointClassification::Outside,
            "probe {p:?}"
        );
    }
    for p in [
        Point3::new(34.0, 0.0, 6.0),
        Point3::new(0.0, 0.0, 6.0),
        Point3::new(0.0, -30.0, 1.0),
    ] {
        assert_eq!(
            classify_point(&topo, cyl, p, &opts).unwrap(),
            PointClassification::Inside,
            "probe {p:?}"
        );
    }
}
