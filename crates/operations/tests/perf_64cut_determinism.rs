//! Regression gate for 64-cut boolean determinism.
//!
//! HashMap iteration nondeterminism in the GFA boolean pipeline previously
//! drove wide per-iteration variance on `bench_boolean_64_holes`, with unlucky
//! runs reaching the mesh fallback. Geometry-affecting assembly maps now use
//! the fixed-seed deterministic hasher, and mesh fallback has bounded work.
//!
//! Keep this test active: count divergence is a release-blocking signal that
//! topology construction has become order-dependent again.

#![allow(clippy::unwrap_used, clippy::print_stdout)]

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::primitives;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::explorer;
use brepkit_topology::solid::SolidId;

/// Run a 64-cut sequence, snapshotting (face, edge, vertex) counts after
/// each cut. Returns a Vec of 64 snapshots.
fn run_64_cut_snapshot() -> Vec<(usize, usize, usize)> {
    let mut topo = Topology::new();
    let mut result: SolidId = primitives::make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    let mut snapshots = Vec::with_capacity(64);
    for row in 0..8 {
        for col in 0..8 {
            let cyl = primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
            let mat = Mat4::translation(
                6.0 + f64::from(col) * 12.0,
                6.0 + f64::from(row) * 12.0,
                -5.0,
            );
            transform_solid(&mut topo, cyl, &mat).unwrap();
            result = boolean(&mut topo, BooleanOp::Cut, result, cyl).unwrap();
            let (f, e, v) = explorer::solid_entity_counts(&topo, result).unwrap();
            snapshots.push((f, e, v));
        }
    }
    snapshots
}

/// Run the 64-cut sequence twice in one process and report the first
/// cut where the (face, edge, vertex) count snapshots diverge between
/// runs.
#[test]
fn diverge_first_cut() {
    let a = run_64_cut_snapshot();
    let b = run_64_cut_snapshot();
    let mut divergence: Option<usize> = None;
    for (i, (sa, sb)) in a.iter().zip(b.iter()).enumerate() {
        if sa != sb {
            println!(
                "DIVERGE at cut {i}: A=(f={},e={},v={}) vs B=(f={},e={},v={})",
                sa.0, sa.1, sa.2, sb.0, sb.1, sb.2
            );
            for j in i.saturating_sub(3)..=i {
                println!(
                    "  cut {j}: A=(f={},e={},v={}) B=(f={},e={},v={})",
                    a[j].0, a[j].1, a[j].2, b[j].0, b[j].1, b[j].2
                );
            }
            divergence = Some(i);
            break;
        }
    }
    // Fail loudly so CI gets an unambiguous signal at the first divergent cut.
    assert!(
        divergence.is_none(),
        "64-cut sequence is nondeterministic — first divergence at cut {} (full trace above)",
        divergence.unwrap_or(0)
    );
    println!("No divergence in 64 cuts — runs are deterministic");
}
