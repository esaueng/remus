//! A moving shoulder blend must not pass through the fixed boss rim.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Vec3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{
    face_edges, face_vertices, solid_edges, solid_entity_counts, solid_faces,
};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

fn top_at(topo: &Topology, solid: SolidId, height: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .expect("planar top at requested height")
}

fn shoulder() -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 30.0, 18.0, 26.0).unwrap();
    let boss = make_cylinder(&mut topo, 6.0, 35.0).unwrap();
    let blank = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let cap = top_at(&topo, blank, 35.0);
    let rim = face_edges(&topo, cap).unwrap();
    let rounded = fillet_cascade(&mut topo, blank, &rim, 1.8).unwrap().solid;
    let perimeter: Vec<_> = solid_edges(&topo, rounded)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let at = |a: f64, b: f64| (a - b).abs() < 1e-7;
            let top = at(a.z(), 26.0) && at(b.z(), 26.0);
            match edge.curve() {
                EdgeCurve::Circle(_) => top,
                EdgeCurve::Line => {
                    (top && ((at(a.x(), 0.0) && at(b.x(), 0.0))
                        || (at(a.y(), 0.0) && at(b.y(), 0.0))))
                        || ((a.z() - b.z()).abs() > 25.0
                            && ((at(a.x(), 6.0)
                                && at(b.x(), 6.0)
                                && at(a.y(), 0.0)
                                && at(b.y(), 0.0))
                                || (at(a.y(), 6.0)
                                    && at(b.y(), 6.0)
                                    && at(a.x(), 0.0)
                                    && at(b.x(), 0.0))))
                }
                _ => false,
            }
        })
        .collect();
    assert_eq!(perimeter.len(), 5);
    let solid = fillet_cascade(&mut topo, rounded, &perimeter, 5.0)
        .unwrap()
        .solid;
    let top = top_at(&topo, solid, 26.0);
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
    assert!((face_area(&topo, top, 0.01).unwrap() - 311.097_938_647_583_36).abs() < 0.01);
    (topo, solid, top)
}

#[test]
fn shoulder_cannot_cross_the_fixed_cap_blend() {
    // The shoulder's R5 spring reaches z=31. The fixed R1.8 boss rim
    // starts at z=33.2, leaving 2.2 mm of travel before they meet.
    let (mut topo, solid, top) = shoulder();
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let faces = solid_faces(&topo, solid).unwrap();
    let volume = solid_volume(&topo, solid, 0.01).unwrap();
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[top]).unwrap();
    for distance in [2.3, 3.3, 5.0] {
        let refusal = move_faces(&mut topo, solid, &[top], distance)
            .expect_err("the moving blend must not cross the fixed rim");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{refusal}"
        );
        let refusal = prepared
            .move_faces(&mut topo, distance)
            .expect_err("prepared moves must enforce the same clearance");
        assert!(
            refusal
                .to_string()
                .contains("moving blend reaches fixed rim face"),
            "{refusal}"
        );
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
        assert_eq!(solid_faces(&topo, solid).unwrap(), faces);
        assert_eq!(
            solid_volume(&topo, solid, 0.01).unwrap().to_bits(),
            volume.to_bits()
        );
    }
    for distance in [-7.0, -1.51, 1.0, 2.0] {
        let moved = move_faces(&mut topo, solid, &[top], distance)
            .unwrap_or_else(|error| panic!("distance {distance}: {error}"));
        assert!(validate_solid(&topo, moved).unwrap().is_valid());
        let moved_top = top_at(&topo, moved, 26.0 + distance);
        assert!(
            (face_area(&topo, moved_top, 0.01).unwrap() - face_area(&topo, top, 0.01).unwrap())
                .abs()
                < 1e-5
        );
        assert_eq!(solid_entity_counts(&topo, moved).unwrap(), counts);
        top_at(&topo, moved, 35.0);
    }
}
