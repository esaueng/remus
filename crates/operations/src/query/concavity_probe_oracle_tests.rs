//! Oracle for [`ConcavityProbe`]: one prepared context per solid against the
//! one-shot [`edge_concavity_from_faces`] it replaces in the rolling-ball
//! engine and the convex material gate, on every manifold edge of a corpus
//! of primitives, boolean results, a filleted box and imported STEP bodies
//! (analytic and NURBS fillet plates, the hammer holder). Verdicts and
//! errors are compared through their `Debug` text.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_topology::explorer::solid_edges;

use super::*;
use crate::boolean::{BooleanOp, boolean};
use crate::primitives::{make_box, make_cylinder};
use crate::transform::transform_solid;

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

fn step(topo: &mut Topology, text: &str) -> SolidId {
    remus_io::step::reader::read_step(text, topo).unwrap()[0]
}

/// Bodies with convex, concave, tangent and out-of-scale edges.
fn corpus() -> Vec<(&'static str, Topology, SolidId)> {
    let mut cases = Vec::new();

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 3.0, 2.0).unwrap();
    cases.push(("box", topo, solid));

    let mut topo = Topology::new();
    let mut plate = make_box(&mut topo, 60.0, 40.0, 10.0).unwrap();
    for (x, y) in [(10.0, 10.0), (30.0, 20.0), (50.0, 30.0)] {
        let hole = make_cylinder(&mut topo, 4.0, 30.0).unwrap();
        translate(&mut topo, hole, x, y, -10.0);
        plate = boolean(&mut topo, BooleanOp::Cut, plate, hole).unwrap();
    }
    cases.push(("holed plate", topo, plate));

    let mut topo = Topology::new();
    let base = make_box(&mut topo, 30.0, 20.0, 5.0).unwrap();
    let upright = make_box(&mut topo, 5.0, 20.0, 20.0).unwrap();
    let bracket = boolean(&mut topo, BooleanOp::Fuse, base, upright).unwrap();
    cases.push(("L bracket", topo, bracket));

    let mut topo = Topology::new();
    let block = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let corner = make_box(&mut topo, 10.0, 10.0, 20.0).unwrap();
    translate(&mut topo, corner, 10.0, 10.0, -5.0);
    let notched = boolean(&mut topo, BooleanOp::Cut, block, corner).unwrap();
    cases.push(("notch", topo, notched));

    let mut topo = Topology::new();
    let pad = make_box(&mut topo, 30.0, 30.0, 5.0).unwrap();
    let boss = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    translate(&mut topo, boss, 15.0, 15.0, 0.0);
    let bossed = boolean(&mut topo, BooleanOp::Fuse, pad, boss).unwrap();
    cases.push(("boss", topo, bossed));

    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, cube).unwrap();
    let rounded = crate::blend_ops::fillet_v2(&mut topo, cube, &edges, 1.0)
        .unwrap()
        .solid;
    cases.push(("filleted box", topo, rounded));

    for (name, text) in [
        (
            "analytic fillet plate",
            include_str!("../../../io/tests/data/openzcad_e_analytic_fillet_plate.step"),
        ),
        (
            "NURBS fillet plate",
            include_str!("../../../io/tests/data/openzcad_e_nurbs_fillet_plate.step"),
        ),
        (
            "hammer holder",
            include_str!("../../../io/tests/data/shapr3d_hammer_holder.step"),
        ),
    ] {
        let mut topo = Topology::new();
        let solid = step(&mut topo, text);
        cases.push((name, topo, solid));
    }
    cases
}

#[test]
fn prepared_probe_matches_one_shot_concavity() {
    let mut verdicts = [0_usize; 4];
    let mut prepared = 0;
    let cases = corpus();
    for (name, topo, solid) in &cases {
        let (topo, solid) = (topo, *solid);
        let adjacency = topo.build_adjacency(solid).unwrap();
        let mut probe = ConcavityProbe::new(topo, solid);
        let edges = solid_edges(topo, solid).unwrap();
        // The hammer holder's one-shot classifier is slow; a third of its
        // edges still covers every surface type it carries.
        let stride = if edges.len() > 200 { 3 } else { 1 };
        for &edge in edges.iter().step_by(stride) {
            let faces = adjacency.faces_for_edge(edge).to_vec();
            if faces.len() != 2 {
                continue;
            }
            for step in [0.05, 0.5, 2.0, 40.0] {
                let expected =
                    edge_concavity_from_faces(topo, solid, edge, faces[0], faces[1], step);
                let got = probe.concavity(edge, faces[0], faces[1], step);
                assert_eq!(
                    format!("{got:?}"),
                    format!("{expected:?}"),
                    "{name}: edge {edge:?} probe {step}"
                );
                if let Ok(verdict) = expected {
                    verdicts[match verdict {
                        EdgeConcavity::Convex => 0,
                        EdgeConcavity::Concave => 1,
                        EdgeConcavity::Tangent => 2,
                        EdgeConcavity::Unknown => 3,
                    }] += 1;
                }
            }
            // A self-seam pair and an invalid probe take the early exits.
            for (a, b, step) in [
                (faces[0], faces[0], 0.5),
                (faces[0], faces[1], 0.0),
                (faces[0], faces[1], f64::NAN),
            ] {
                assert_eq!(
                    format!("{:?}", probe.concavity(edge, a, b, step)),
                    format!(
                        "{:?}",
                        edge_concavity_from_faces(topo, solid, edge, a, b, step)
                    ),
                    "{name}: edge {edge:?} early exit"
                );
            }
        }
        // No corpus face has an unsampleable trim, so every solid that
        // reached the classifier must have been prepared. The filleted box
        // is tangent at every edge and never reaches it.
        match probe.classifier {
            ProbeClassifier::Prepared(_) => prepared += 1,
            ProbeClassifier::Unprepared => {}
            ProbeClassifier::OneShot => panic!("{name} fell back to the one-shot classifier"),
        }
    }
    // The prepared classifier must be the one answering, and every verdict
    // must occur, or the equality above covers less than it seems to.
    assert_eq!(
        prepared,
        cases.len() - 1,
        "solids answered by the prepared path"
    );
    assert!(
        verdicts.iter().all(|&count| count > 0),
        "verdict counts (convex, concave, tangent, unknown): {verdicts:?}"
    );
}
