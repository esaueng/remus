//! A planar face move across a blend seam must rebuild the seam on its exact
//! tangent line. The plane–cylinder branch used to come from √(r² − d²), which
//! turned ~1e-14 of rounding at tangency into a ~1e-7 offset: the fillet arc's
//! tangent vertex left its trim, and the next move of the same face refused
//! with "misses its parameter authority".

#![allow(
    clippy::disallowed_methods,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic
)]

use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{face_vertices, solid_edges, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

/// The highest upward planar face strictly below `below`.
fn top_face_below(topo: &Topology, solid: SolidId, below: f64) -> FaceId {
    let mut best: Option<(FaceId, f64)> = None;
    for face in solid_faces(topo, solid).unwrap() {
        let Some(normal) = topo.face(face).unwrap().effective_plane_normal() else {
            continue;
        };
        if normal.dot(Vec3::new(0.0, 0.0, 1.0)) < 1.0 - 1e-9 {
            continue;
        }
        let vertices = face_vertices(topo, face).unwrap();
        let z = topo.vertex(vertices[0]).unwrap().point().z();
        if z < below && best.is_none_or(|(_, best_z)| z > best_z) {
            best = Some((face, z));
        }
    }
    best.expect("top face").0
}

/// Largest distance between a circular edge's trim endpoints and its vertices.
fn worst_arc_endpoint_residual(topo: &Topology, solid: SolidId) -> f64 {
    let mut worst = 0.0_f64;
    for edge in solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(edge).unwrap();
        if !matches!(edge.curve(), EdgeCurve::Circle(_)) {
            continue;
        }
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let (t0, t1) = edge.strict_domain().unwrap();
        for (parameter, vertex) in [(t0, start), (t1, end)] {
            let on_curve = edge.curve().evaluate_with_endpoints(parameter, start, end);
            worst = worst.max((on_curve - vertex).length());
        }
    }
    worst
}

#[test]
fn moving_a_face_past_a_blend_seam_keeps_the_seam_exact_for_the_next_move() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, 6.65).unwrap();
    let post = make_cylinder(&mut topo, 6.0, 28.0).unwrap();
    let union = boolean(&mut topo, BooleanOp::Fuse, block, post).unwrap();
    let corner = solid_edges(&topo, union)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let edge = topo.edge(edge).unwrap();
            [edge.start(), edge.end()].iter().all(|&vertex| {
                let point = topo.vertex(vertex).unwrap().point();
                (point.x() - 30.0).abs() < 1e-9 && point.y().abs() < 1e-9
            })
        })
        .expect("x-max/y-min vertical edge");
    let rounded = fillet_v2(&mut topo, union, &[corner], 7.7).unwrap().solid;

    let mut solid = rounded;
    for (distance, below) in [(16.0, 27.0), (1.0, 27.0), (-9.0, 27.9)] {
        let face = top_face_below(&topo, solid, below);
        solid = move_faces(&mut topo, solid, &[face], distance)
            .unwrap_or_else(|error| panic!("move by {distance}: {error:?}"));
        validate_solid(&topo, solid).unwrap();
        let residual = worst_arc_endpoint_residual(&topo, solid);
        assert!(
            residual < 1e-12,
            "move by {distance} left an arc endpoint {residual} off its trim"
        );
    }
}
