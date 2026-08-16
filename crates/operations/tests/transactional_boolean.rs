//! The transactional boolean contract (RFC 0002 / Issue 9).
//!
//! `boolean_transacted` = stage → validate → commit / roll back: a failed
//! or invalid result restores the topology exactly, and handles allocated
//! by the failed attempt can never alias later entities.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean, boolean_transacted};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::make_box;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;

fn two_overlapping_boxes(
    topo: &mut Topology,
) -> (brepkit_topology::SolidId, brepkit_topology::SolidId) {
    let a = make_box(topo, 2.0, 2.0, 2.0).unwrap();
    let b = make_box(topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(topo, b, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    (a, b)
}

#[test]
fn valid_result_commits_and_matches_the_untransacted_path() {
    let mut topo_a = Topology::new();
    let (a1, a2) = two_overlapping_boxes(&mut topo_a);
    let plain = boolean(&mut topo_a, BooleanOp::Fuse, a1, a2).unwrap();

    let mut topo_b = Topology::new();
    let (b1, b2) = two_overlapping_boxes(&mut topo_b);
    let transacted = boolean_transacted(&mut topo_b, BooleanOp::Fuse, b1, b2).unwrap();

    let v_plain = solid_volume(&topo_a, plain, 0.1).unwrap();
    let v_transacted = solid_volume(&topo_b, transacted, 0.1).unwrap();
    assert!(
        (v_plain - v_transacted).abs() < 1e-9,
        "transacted commit must be the same result as the plain path"
    );
    // Closed-form: 2*8 - 1 = 15.
    assert!((v_transacted - 15.0).abs() < 1e-6);
}

#[test]
fn failure_rolls_back_to_the_exact_pre_state() {
    let mut topo = Topology::new();
    let (a, b) = two_overlapping_boxes(&mut topo);
    let _consumed = boolean_transacted(&mut topo, BooleanOp::Fuse, a, b).unwrap();

    let vertices = topo.num_vertices();
    let edges = topo.num_edges();
    let faces = topo.num_faces();
    let solids = topo.num_solids();
    let slots = topo.allocated_slot_count();

    // Cutting a solid from itself is an algebraic empty result: a typed
    // failure that must leave no trace.
    let err = boolean_transacted(&mut topo, BooleanOp::Cut, a, a);
    assert!(err.is_err(), "A \\ A must refuse");

    assert_eq!(topo.num_vertices(), vertices);
    assert_eq!(topo.num_edges(), edges);
    assert_eq!(topo.num_faces(), faces);
    assert_eq!(topo.num_solids(), solids);
    assert!(
        topo.allocated_slot_count() >= slots,
        "rollback preserves the high-water mark; slots are never reclaimed"
    );

    // The topology is still fully usable after the rollback.
    let (c, d) = two_overlapping_boxes(&mut topo);
    boolean_transacted(&mut topo, BooleanOp::Cut, c, d).unwrap();
}
