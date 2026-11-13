//! Fuse (union) scaling benchmarks.
//!
//! Measures `fuse_all` (balanced pair-wise reduction) vs sequential left-fold
//! for grids of adjacent boxes. Exercises the Fuse code path in `boolean()`.
//! Also measures the lineage-carrying `fuse_regions` against `fuse_all` on a
//! staggered chain of boxes.
//!
//! Run with: `cargo bench -p remus-operations --bench fuse_perf`

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean, fuse_regions};
use remus_operations::compound_ops::fuse_all;
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::compound::Compound;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build an N×N grid of unit boxes, each touching its neighbors (shared edges).
/// Returns (topology, vec_of_solid_ids).
fn build_box_grid(n: usize) -> (Topology, Vec<remus_topology::solid::SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::with_capacity(n * n);
    for row in 0..n {
        for col in 0..n {
            let bx = primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            let mat = Mat4::translation(col as f64, row as f64, 0.0);
            transform_solid(&mut topo, bx, &mat).unwrap();
            solids.push(bx);
        }
    }
    (topo, solids)
}

/// Build an N×N grid of slightly overlapping boxes (0.01 overlap on each edge).
fn build_overlapping_box_grid(n: usize) -> (Topology, Vec<remus_topology::solid::SolidId>) {
    let mut topo = Topology::new();
    let size = 1.01; // slight overlap
    let mut solids = Vec::with_capacity(n * n);
    for row in 0..n {
        for col in 0..n {
            let bx = primitives::make_box(&mut topo, size, size, 1.0).unwrap();
            let mat = Mat4::translation(col as f64, row as f64, 0.0);
            transform_solid(&mut topo, bx, &mat).unwrap();
            solids.push(bx);
        }
    }
    (topo, solids)
}

// ---------------------------------------------------------------------------
// Benchmark: fuse_all (balanced) vs sequential left-fold
// ---------------------------------------------------------------------------

fn bench_fuse_balanced(c: &mut Criterion) {
    let mut group = c.benchmark_group("fuse_balanced");
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(5));

    // N=144 is past the N-way source-pair budget when every pair is
    // counted (10 296 > 4 096); its sequential twin would take seconds.
    for &side in &[2, 3, 4, 5, 12] {
        let n = side * side;
        group.bench_function(format!("balanced_N={n}"), |b| {
            let (base_topo, solids) = build_overlapping_box_grid(side);
            b.iter(|| {
                let mut topo = base_topo.clone();
                let cid = topo.add_compound(Compound::new(solids.clone()));
                black_box(fuse_all(&mut topo, cid).unwrap());
            });
        });
        if side > 5 {
            continue;
        }

        group.bench_function(format!("sequential_N={n}"), |b| {
            let (base_topo, solids) = build_overlapping_box_grid(side);
            b.iter(|| {
                let mut topo = base_topo.clone();
                let mut result = solids[0];
                for &s in &solids[1..] {
                    result = boolean(&mut topo, BooleanOp::Fuse, result, s).unwrap();
                }
                black_box(result);
            });
        });
    }

    group.finish();
}

fn bench_fuse_touching_boxes(c: &mut Criterion) {
    let mut group = c.benchmark_group("fuse_touching");
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(5));

    for &side in &[2, 3, 4, 12] {
        let n = side * side;
        group.bench_function(format!("balanced_N={n}"), |b| {
            let (base_topo, solids) = build_box_grid(side);
            b.iter(|| {
                let mut topo = base_topo.clone();
                let cid = topo.add_compound(Compound::new(solids.clone()));
                black_box(fuse_all(&mut topo, cid).unwrap());
            });
        });
    }

    group.finish();
}

/// Build `n` boxes staggered along a diagonal with no two faces coplanar.
/// Each overlaps its next two neighbours, so the union is one body and most
/// edges are cut more than once: the qualified domain of `fuse_regions`
/// (`tests/nary_fuse_regions.rs` holds the same layout to its oracles).
fn build_staggered_chain(n: usize) -> (Topology, Vec<remus_topology::solid::SolidId>) {
    let mut topo = Topology::new();
    let mut solids = Vec::with_capacity(n);
    for index in 0..n {
        let step = index as f64;
        let bx = primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let mat = Mat4::translation(0.875 * step, 0.375 * step, 0.25 * step);
        transform_solid(&mut topo, bx, &mat).unwrap();
        solids.push(bx);
    }
    (topo, solids)
}

/// The lineage-carrying N-ary fuse against the single-solid `fuse_all` on
/// the same operands: what regions, total lineage and the pre-export
/// qualification cost on top of the shared arrangement.
fn bench_fuse_regions(c: &mut Criterion) {
    let mut group = c.benchmark_group("fuse_regions");
    group.sample_size(10);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(5));

    for &n in &[3, 8] {
        group.bench_function(format!("regions_N={n}"), |b| {
            let (base_topo, solids) = build_staggered_chain(n);
            b.iter(|| {
                let mut topo = base_topo.clone();
                black_box(fuse_regions(&mut topo, &solids).unwrap());
            });
        });
        group.bench_function(format!("fuse_all_N={n}"), |b| {
            let (base_topo, solids) = build_staggered_chain(n);
            b.iter(|| {
                let mut topo = base_topo.clone();
                let cid = topo.add_compound(Compound::new(solids.clone()));
                black_box(fuse_all(&mut topo, cid).unwrap());
            });
        });
    }

    group.finish();
}

// ===========================================================================
// Criterion setup
// ===========================================================================

criterion_group!(
    fuse_scaling,
    bench_fuse_balanced,
    bench_fuse_touching_boxes,
    bench_fuse_regions,
);

criterion_main!(fuse_scaling);
