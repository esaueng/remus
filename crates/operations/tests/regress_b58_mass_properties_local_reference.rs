//! B58: Gauss mass properties must not depend on where the body sits.
//!
//! `mass_properties` and `solid_center_of_mass` reach the boundary-trimmed
//! Gauss integrator in `remus_check::properties`. It used to integrate P·n and
//! the first and second moments about the world origin and move the tensor to
//! the centroid afterwards (Huygens). For a small body far from the origin the
//! second moments then cancel terms of order |offset|²·L³ against an L⁵
//! answer: a 1e-3 frustum moved by the B26 harness offset (13, −7, 5) read its
//! inertia tensor 25× off and its centroid 5.8e-4 body lengths away, against
//! the same frustum in place.
//!
//! Each body here is measured in place and again as a translated copy. The
//! offset is the harness offset scaled with the body, so every scale sits the
//! same ~1.3e4 body lengths from the origin. Every exact arm of the integrator
//! is on the path: planar Green closed forms (box, cone and cylinder caps),
//! untrimmed quadric quadrature (sphere, cone), the full torus,
//! polygon-trimmed cylinder bands (the cross-drilled shaft), and NURBS
//! quadrature (the box converted to B-spline carriers).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_2;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::heal::convert_to_bspline;
use remus_operations::measure::{mass_properties, solid_center_of_mass};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Mass, centroid (in body lengths) and tensor must agree to this, relative.
const REL: f64 = 1e-9;

/// The B26 harness offset, for a body of unit-1e-3 scale.
const OFFSET: [f64; 3] = [13.0, -7.0, 5.0];

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

type Build = fn(&mut Topology, f64) -> SolidId;

fn box_body(topo: &mut Topology, s: f64) -> SolidId {
    make_box(topo, s, 2.0 * s, 3.0 * s).unwrap()
}

/// The box with every plane converted to a bilinear B-spline carrier: NURBS
/// quadrature over trimmed faces.
fn bspline_box_body(topo: &mut Topology, s: f64) -> SolidId {
    let body = box_body(topo, s);
    assert!(convert_to_bspline(topo, body).unwrap() > 0);
    body
}

fn sphere_body(topo: &mut Topology, s: f64) -> SolidId {
    make_sphere(topo, s, 16).unwrap()
}

fn frustum_body(topo: &mut Topology, s: f64) -> SolidId {
    make_cone(topo, s, 0.5 * s, 2.0 * s).unwrap()
}

fn pointed_cone_body(topo: &mut Topology, s: f64) -> SolidId {
    make_cone(topo, s, 0.0, 2.0 * s).unwrap()
}

fn torus_body(topo: &mut Topology, s: f64) -> SolidId {
    make_torus(topo, 2.0 * s, 0.5 * s, 16).unwrap()
}

/// An r = 3, h = 30 shaft with an equal-radius bore through its side, scaled
/// by `s / 10`: its walls are cylinder bands trimmed by ellipse arcs.
fn cross_drilled_body(topo: &mut Topology, s: f64) -> SolidId {
    const R: f64 = 3.0;
    const H: f64 = 30.0;
    let shaft = make_cylinder(topo, R, H).unwrap();
    let len = H + 4.0 * R;
    let bore = make_cylinder(topo, R, len).unwrap();
    transform_solid(topo, bore, &Mat4::rotation_y(FRAC_PI_2)).unwrap();
    transform_solid(topo, bore, &Mat4::translation(-len / 2.0, 0.0, H / 2.0)).unwrap();
    let body = boolean(topo, BooleanOp::Cut, shaft, bore).unwrap();
    let k = s / 10.0;
    transform_solid(topo, body, &Mat4::scale(k, k, k)).unwrap();
    body
}

const BODIES: [(&str, Build); 7] = [
    ("box", box_body),
    ("B-spline box", bspline_box_body),
    ("sphere", sphere_body),
    ("frustum", frustum_body),
    ("pointed cone", pointed_cone_body),
    ("torus", torus_body),
    ("cross-drilled shaft", cross_drilled_body),
];

fn max_abs(values: &[f64]) -> f64 {
    values.iter().fold(0.0_f64, |m, v| m.max(v.abs()))
}

#[test]
fn b58_mass_properties_do_not_depend_on_placement() {
    let mut failures = Vec::new();
    for (name, build) in BODIES {
        for s in SCALES {
            let mut topo = Topology::new();
            let body = build(&mut topo, s);
            let k = s * 1e3;
            let offset = Vec3::new(OFFSET[0] * k, OFFSET[1] * k, OFFSET[2] * k);
            let moved = copy_and_transform_solid(
                &mut topo,
                body,
                &Mat4::translation(offset.x(), offset.y(), offset.z()),
            )
            .unwrap();

            let here = mass_properties(&topo, body).unwrap();
            let there = mass_properties(&topo, moved).unwrap();
            let length = here.mass.abs().cbrt();

            let mass = (there.mass - here.mass).abs() / here.mass.abs();
            let shift = |c: Point3| c - offset;
            let centroid = (shift(there.center) - here.center).length() / length;
            let deltas: Vec<f64> = here
                .inertia
                .iter()
                .zip(there.inertia)
                .map(|(a, b)| b - a)
                .collect();
            let tensor = max_abs(&deltas) / max_abs(&here.inertia);

            let com_here = solid_center_of_mass(&topo, body, 0.1).unwrap();
            let com_there = solid_center_of_mass(&topo, moved, 0.1).unwrap();
            let com = (shift(com_there) - com_here).length() / length;

            for (what, err) in [
                ("mass", mass),
                ("centroid", centroid),
                ("tensor", tensor),
                ("solid_center_of_mass", com),
            ] {
                if err.is_nan() || err > REL {
                    failures.push(format!("{name} at scale {s}: {what} moved by {err:.2e}"));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "mass properties depend on placement:\n{}",
        failures.join("\n")
    );
}
