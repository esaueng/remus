//! Regression coverage for curved-edge extrema in `remus_check` solid bounds.
//!
//! Both shapes use closed analytic edges with one seam vertex. Vertex-only
//! bounds therefore collapse each rim to that vertex and under-bound the
//! solid; the expected boxes require interior curve extrema.

#![allow(clippy::expect_used, clippy::unwrap_used)]

use remus_check::properties::bounding_box;
use remus_math::vec::{Point3, Vec3};
use remus_operations::extrude::extrude;
use remus_operations::primitives::make_cone;
use remus_topology::Topology;
use remus_topology::builder::{make_ellipse_edge_with_ref, make_face_from_wire};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

const EPS: f64 = 1e-9;

#[track_caller]
fn assert_box(topo: &Topology, solid: SolidId, expected_min: Point3, expected_max: Point3) {
    let aabb = bounding_box(topo, solid).unwrap();
    for (actual, expected) in [
        (aabb.min.x(), expected_min.x()),
        (aabb.min.y(), expected_min.y()),
        (aabb.min.z(), expected_min.z()),
        (aabb.max.x(), expected_max.x()),
        (aabb.max.y(), expected_max.y()),
        (aabb.max.z(), expected_max.z()),
    ] {
        assert!(
            (actual - expected).abs() < EPS,
            "got {actual}, expected {expected}"
        );
    }
}

#[test]
fn conical_frustum_box_includes_closed_circle_rims() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 5.0, 2.0, 7.0).unwrap();

    assert_box(
        &topo,
        solid,
        Point3::new(-5.0, -5.0, 0.0),
        Point3::new(5.0, 5.0, 7.0),
    );
}

#[test]
fn extruded_rotated_ellipse_keeps_exact_analytic_box() {
    let mut topo = Topology::new();
    let angle = std::f64::consts::FRAC_PI_6;
    let edge = make_ellipse_edge_with_ref(
        &mut topo,
        Point3::new(-4.0, 7.0, -2.0),
        Vec3::new(0.0, 0.0, 1.0),
        6.0,
        2.0,
        Vec3::new(angle.cos(), angle.sin(), 0.0),
        1e-7,
    )
    .unwrap();
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let face = make_face_from_wire(&mut topo, wire).unwrap();
    let solid = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
    let x_extent = 28.0_f64.sqrt();
    let y_extent = 12.0_f64.sqrt();

    assert_box(
        &topo,
        solid,
        Point3::new(-4.0 - x_extent, 7.0 - y_extent, -2.0),
        Point3::new(-4.0 + x_extent, 7.0 + y_extent, 2.0),
    );
}
