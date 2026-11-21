//! Loft connections: which vertex of each section the correspondence starts
//! from. A ruled loft between parallel sections moves every section vertex
//! linearly, so its section area is quadratic in height and the prismatoid
//! formula `h/6·(A₀ + 4·A_m + A₁)` is exact, with `A_m` the area of the
//! polygon of corresponding vertices' midpoints.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::solid_edges;
use remus_topology::face::{Face, FaceId, FaceSurface};

use super::*;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// A 2 × 2 square about the Z axis at height `z`, counter-clockwise from
/// (−1, −1).
fn square(topo: &mut Topology, z: f64) -> FaceId {
    let wire = make_polygon_wire(
        topo,
        &[
            p(-1.0, -1.0, z),
            p(1.0, -1.0, z),
            p(1.0, 1.0, z),
            p(-1.0, 1.0, z),
        ],
        1e-7,
    )
    .unwrap();
    let normal = Vec3::new(0.0, 0.0, 1.0);
    topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d: z }))
}

fn volume(topo: &Topology, solid: remus_topology::solid::SolidId) -> f64 {
    crate::measure::solid_volume(topo, solid, 0.05).unwrap()
}

/// Whether the solid has a straight edge joining `a` and `b`.
fn joins(topo: &Topology, solid: remus_topology::solid::SolidId, a: Point3, b: Point3) -> bool {
    solid_edges(topo, solid).unwrap().into_iter().any(|e| {
        let edge = topo.edge(e).unwrap();
        let (s, t) = (
            topo.vertex(edge.start()).unwrap().point(),
            topo.vertex(edge.end()).unwrap().point(),
        );
        ((s - a).length() < 1e-9 && (t - b).length() < 1e-9)
            || ((s - b).length() < 1e-9 && (t - a).length() < 1e-9)
    })
}

#[test]
fn connecting_matching_corners_is_the_plain_loft() {
    let mut topo = Topology::new();
    let (a, b) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
    let starts = [p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)];
    let solid = loft_with_connections(&mut topo, &[a, b], &starts, false).unwrap();
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    assert!((volume(&topo, solid) - 40.0).abs() < 1e-9);
    assert!(joins(&topo, solid, p(-1.0, -1.0, 0.0), p(-1.0, -1.0, 10.0)));
}

#[test]
fn shifting_a_connection_by_one_corner_twists_the_loft_a_quarter_turn() {
    let mut topo = Topology::new();
    let (a, b) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
    // (−1, −1) at the bottom connects to (1, −1) at the top, and so on round.
    let starts = [p(-1.0, -1.0, 0.0), p(1.0, -1.0, 10.0)];
    let solid = loft_with_connections(&mut topo, &[a, b], &starts, false).unwrap();
    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    assert!(joins(&topo, solid, p(-1.0, -1.0, 0.0), p(1.0, -1.0, 10.0)));
    assert!(!joins(
        &topo,
        solid,
        p(-1.0, -1.0, 0.0),
        p(-1.0, -1.0, 10.0)
    ));
    // Midpoints (0, −1), (1, 0), (0, 1), (−1, 0): a diamond of area 2.
    let exact = 10.0 / 6.0 * (4.0 + 4.0 * 2.0 + 4.0);
    let measured = volume(&topo, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );
}

#[test]
fn connection_refusals_are_typed_and_leave_the_topology_unchanged() {
    for (label, starts, phrase) in [
        (
            "a point that is not a vertex",
            vec![p(0.0, -1.0, 0.0), p(1.0, -1.0, 10.0)],
            "is not a vertex of section 0",
        ),
        (
            "one point for two sections",
            vec![p(-1.0, -1.0, 0.0)],
            "one point per section",
        ),
    ] {
        let mut topo = Topology::new();
        let (a, b) = (square(&mut topo, 0.0), square(&mut topo, 10.0));
        let faces = topo.num_faces();
        let error = loft_with_connections(&mut topo, &[a, b], &starts, false).expect_err(label);
        let crate::OperationsError::InvalidInput { reason } = &error else {
            panic!("{label}: untyped {error:?}");
        };
        assert!(reason.contains(phrase), "{label}: {reason}");
        assert_eq!(topo.num_faces(), faces, "{label}: faces left behind");
    }
}
