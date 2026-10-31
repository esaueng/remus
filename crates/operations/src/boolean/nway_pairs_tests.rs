//! Candidate-pair pruning for the N-way cluster fuse.
//!
//! The pruned pave filler must deposit exactly what the all-pairs pipeline
//! deposits (oracle: the whole arena is identical), and a large sparse
//! lattice must stay on the single N-way arrangement instead of tripping the
//! source-pair budget into the sequential fallback.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use remus_math::context::OperationContext;
use remus_math::mat::Mat4;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::{cluster_candidate_pairs, fuse_cluster};
use crate::primitives::{make_box, make_cylinder};
use crate::transform::transform_solid;

/// A `side × side` lattice at unit pitch; `size > 1` overlaps neighbours,
/// `size == 1` makes them touch face to face.
fn box_lattice(side: usize, size: f64) -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::new();
    for row in 0..side {
        for col in 0..side {
            let solid = make_box(&mut topo, size, size, 1.0).unwrap();
            transform_solid(
                &mut topo,
                solid,
                &Mat4::translation(col as f64, row as f64, 0.0),
            )
            .unwrap();
            solids.push(solid);
        }
    }
    (topo, solids)
}

fn cylinder_row(count: usize, radius: f64, pitch: f64) -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::new();
    for index in 0..count {
        let solid = make_cylinder(&mut topo, radius, 2.0).unwrap();
        transform_solid(
            &mut topo,
            solid,
            &Mat4::translation(index as f64 * pitch, 0.0, 0.0),
        )
        .unwrap();
        solids.push(solid);
    }
    (topo, solids)
}

#[test]
fn candidate_pairs_reproduce_the_all_pairs_arena() {
    let cases = [
        ("overlapping 3x3", box_lattice(3, 1.01)),
        ("overlapping 6x6", box_lattice(6, 1.01)),
        ("touching 4x4", box_lattice(4, 1.0)),
        ("cylinder row", cylinder_row(6, 0.6, 1.0)),
    ];
    let context = OperationContext::new();
    for (name, (topo, solids)) in cases {
        let pairs = cluster_candidate_pairs(&topo, &solids, context.tolerance.linear)
            .candidates()
            .expect("finite bounds");
        let all = solids.len() * (solids.len() - 1) / 2;
        assert!(
            pairs.len() < all,
            "{name}: nothing pruned ({} of {all})",
            pairs.len()
        );

        let mut all_pairs = topo.clone();
        let mut pruned = topo.clone();
        let reference = remus_algo::gfa::fuse_n_with_context(&mut all_pairs, &solids, &context);
        let candidate =
            remus_algo::gfa::fuse_n_with_candidate_pairs(&mut pruned, &solids, &pairs, &context);
        assert_eq!(
            format!("{candidate:?}"),
            format!("{reference:?}"),
            "{name}: result differs"
        );
        assert_eq!(
            format!("{pruned:?}"),
            format!("{all_pairs:?}"),
            "{name}: arena differs"
        );
    }
}

#[test]
fn candidate_pairs_grow_with_contacts_not_with_all_pairs() {
    let tolerance = OperationContext::new().tolerance.linear;
    let count = |side| {
        let (topo, solids) = box_lattice(side, 1.01);
        cluster_candidate_pairs(&topo, &solids, tolerance)
            .candidates()
            .expect("finite bounds")
            .len()
    };
    // Each interior box overlaps its eight neighbours: four pairs per box.
    let (small, large) = (count(10), count(20));
    assert!(small <= 4 * 100, "{small} pairs for 100 boxes");
    assert!(large <= 4 * 400, "{large} pairs for 400 boxes");
    assert!(
        large * 10 <= small * 45,
        "4x the boxes gave {large} vs {small} pairs: super-linear"
    );
}

#[test]
fn large_sparse_cluster_fuses_in_one_arrangement() {
    // 121 boxes: 7260 source pairs, over the 4096-pair budget, so the
    // all-pairs N-way fuse refuses and the cluster used to drop to the
    // O(n²) sequential left-fold.
    let (topo, solids) = box_lattice(11, 1.01);
    let context = OperationContext::new();
    let mut all_pairs = topo.clone();
    assert!(
        remus_algo::gfa::fuse_n_with_context(&mut all_pairs, &solids, &context).is_err(),
        "the all-pairs budget no longer refuses this lattice; pick a larger one"
    );

    let pairs = cluster_candidate_pairs(&topo, &solids, context.tolerance.linear)
        .candidates()
        .expect("finite bounds");
    let mut direct = topo.clone();
    let nway = remus_algo::gfa::fuse_n_with_candidate_pairs(&mut direct, &solids, &pairs, &context)
        .expect("one arrangement");

    let mut fused = topo;
    let result = fuse_cluster(&mut fused, &solids).unwrap();
    let faces =
        |topo: &Topology, solid| remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert_eq!(
        faces(&fused, result).len(),
        faces(&direct, nway).len(),
        "fuse_cluster did not take the N-way arrangement"
    );
    crate::measure::test_support::assert_volume_near(&fused, result, 11.01 * 11.01, 1e-9);
}

#[test]
fn dense_cluster_stops_collecting_at_the_pair_budget() {
    // 92 coincident-overlapping boxes: 4186 interacting pairs, one past what
    // the N-way fuse accepts. Collection must stop at the budget instead of
    // materializing every pair, and report the cluster as over budget.
    let mut topo = Topology::new();
    let mut solids = Vec::new();
    for index in 0..92 {
        let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let shift = f64::from(index) * 1e-3;
        transform_solid(&mut topo, solid, &Mat4::translation(shift, shift, 0.0)).unwrap();
        solids.push(solid);
    }
    const { assert!(92 * 91 / 2 > remus_algo::gfa::MAX_NWAY_SOURCE_PAIRS) };
    let tolerance = OperationContext::new().tolerance.linear;
    assert!(matches!(
        cluster_candidate_pairs(&topo, &solids, tolerance),
        super::ClusterPairs::OverBudget
    ));
    // One fewer pair than the budget still yields the full candidate list.
    let within = &solids[..91];
    const { assert!(91 * 90 / 2 <= remus_algo::gfa::MAX_NWAY_SOURCE_PAIRS) };
    assert_eq!(
        cluster_candidate_pairs(&topo, within, tolerance)
            .candidates()
            .expect("finite bounds")
            .len(),
        91 * 90 / 2
    );
}
