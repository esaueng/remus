//! Differential oracle for the edge-concavity crossing memo on the imported
//! hammer holder, whose NURBS and torus blends dominate its probe rays.
//!
//! Edge relations and feature recognition on the holder, then on the results
//! of direct edits and a cut, are identical with the memo off and on, with
//! the memo carried across the edits as a session carries it.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::step::reader::read_step;
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::feature_recognition::recognize_features;
use remus_operations::measure::face_area;
use remus_operations::primitives::make_box;
use remus_operations::push_pull::move_faces;
use remus_operations::query::{
    disable_thread_crossing_memo, enable_thread_crossing_memo, solid_edge_relations,
    thread_crossing_memo_stats,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// The planar face with outward normal along `axis` whose area is nearest
/// `area`.
fn planar_face(topo: &Topology, solid: SolidId, axis: usize, area: f64) -> FaceId {
    let mut best: Option<(FaceId, f64)> = None;
    for face in solid_faces(topo, solid).unwrap() {
        if let FaceSurface::Plane { normal, .. } = topo.face(face).unwrap().surface()
            && normal.0[axis] > 0.99
        {
            let gap = (face_area(topo, face, 0.05).unwrap() - area).abs();
            if best.is_none_or(|(_, g)| gap < g) {
                best = Some((face, gap));
            }
        }
    }
    best.unwrap().0
}

/// The holder and the bodies a session derives from it: the opening wall
/// moved in by 6 mm, then moved back out by 4, and a slot cut into the base.
fn bodies() -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let source = read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).unwrap()[0];
    let wall = planar_face(&topo, source, 0, 1045.93);
    let moved = move_faces(&mut topo, source, &[wall], -6.0).unwrap();
    let wall = planar_face(&topo, moved, 0, 1045.93);
    let back = move_faces(&mut topo, moved, &[wall], 4.0).unwrap();
    let tool = make_box(&mut topo, 4.0, 12.0, 20.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(10.0, 44.0, 0.0)).unwrap();
    let slotted = boolean(&mut topo, BooleanOp::Cut, back, tool).unwrap();
    (topo, vec![source, moved, back, slotted])
}

fn analysis(topo: &Topology, solid: SolidId) -> String {
    format!(
        "{:#?}\n{:#?}",
        solid_edge_relations(topo, solid, None).unwrap(),
        recognize_features(topo, solid, 0.08).unwrap()
    )
}

#[test]
fn crossing_memo_reproduces_holder_analysis_across_edits() {
    disable_thread_crossing_memo();
    let (topo, bodies) = bodies();
    let expected: Vec<String> = bodies.iter().map(|&body| analysis(&topo, body)).collect();

    enable_thread_crossing_memo();
    let mut hits = 0;
    for (index, (&body, expected)) in bodies.iter().zip(&expected).enumerate() {
        assert_eq!(&analysis(&topo, body), expected, "body {index}");
        let stats = thread_crossing_memo_stats().unwrap();
        if index > 0 {
            // Every edit leaves most blends untouched.
            assert!(stats.hits > hits, "body {index} found no remembered count");
        }
        hits = stats.hits;
    }
    disable_thread_crossing_memo();
}
