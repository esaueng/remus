//! RFC 0008 F4: points on a full-turn lateral wall are on the boundary.
//!
//! The on-boundary trim test projected a curved face's 3D outer polygon onto
//! a plane. For a full-turn wall (rim, seam, rim, seam) that projection is
//! degenerate, so `(0, 3, 4)` on the wall of an r 3 cylinder classified
//! `Outside` while `(±3, 0, 4)` happened to pass.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{
    ClassifyOptions, PointClassification, PreparedSolid, classify_point, is_point_on_boundary,
};
use remus_math::vec::Point3;
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-6;

fn assert_on_boundary(topo: &Topology, solid: SolidId, points: &[Point3], expected: bool) {
    let prepared = PreparedSolid::prepare(topo, solid).unwrap();
    let options = ClassifyOptions::default();
    for &p in points {
        assert_eq!(
            prepared.is_point_on_boundary(p, TOL).unwrap(),
            expected,
            "prepared at {p:?}"
        );
        assert_eq!(
            prepared.is_point_on_boundary_unpruned(p, TOL).unwrap(),
            expected,
            "unpruned at {p:?}"
        );
        assert_eq!(
            is_point_on_boundary(topo, solid, p, TOL).unwrap(),
            expected,
            "one-shot at {p:?}"
        );
        if expected {
            assert_eq!(
                prepared.classify_point(p, &options).unwrap(),
                PointClassification::OnBoundary,
                "prepared class at {p:?}"
            );
            assert_eq!(
                classify_point(topo, solid, p, &options).unwrap(),
                PointClassification::OnBoundary,
                "one-shot class at {p:?}"
            );
        }
    }
}

/// `(radius at height z, z)` samples around a wall, the seam included.
fn ring(radius: impl Fn(f64) -> f64, heights: &[f64], offset: f64) -> Vec<Point3> {
    let mut points = Vec::new();
    for k in 0..24 {
        let angle = f64::from(k) * std::f64::consts::TAU / 24.0;
        for &z in heights {
            let r = radius(z) + offset;
            points.push(Point3::new(r * angle.cos(), r * angle.sin(), z));
        }
    }
    points
}

#[test]
fn rfc_0008_f4_repro_points() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 3.0, 8.0).unwrap();
    assert_on_boundary(
        &topo,
        solid,
        &[
            Point3::new(0.0, 3.0, 4.0),
            Point3::new(3.0, 0.0, 4.0),
            Point3::new(-3.0, 0.0, 4.0),
            Point3::new(0.0, -3.0, 4.0),
        ],
        true,
    );
}

#[test]
fn cylinder_wall_points_are_on_the_boundary_all_the_way_round() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 3.0, 8.0).unwrap();
    assert_on_boundary(&topo, solid, &ring(|_| 3.0, &[0.5, 4.0, 7.5], 0.0), true);
    // Off the wall, or on its carrier beyond the caps.
    assert_on_boundary(&topo, solid, &ring(|_| 3.0, &[4.0], 1e-3), false);
    assert_on_boundary(&topo, solid, &ring(|_| 3.0, &[4.0], -1e-3), false);
    assert_on_boundary(&topo, solid, &ring(|_| 3.0, &[-0.5, 8.5], 0.0), false);
}

#[test]
fn cone_wall_points_are_on_the_boundary_all_the_way_round() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 3.0, 1.0, 6.0).unwrap();
    let radius = |z: f64| 3.0 - z / 3.0;
    assert_on_boundary(&topo, solid, &ring(radius, &[0.5, 3.0, 5.5], 0.0), true);
    assert_on_boundary(&topo, solid, &ring(radius, &[-0.5, 6.5], 0.0), false);
}
