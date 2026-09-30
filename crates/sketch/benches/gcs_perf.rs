//! Criterion sketch benchmarks (PERF-S01 baseline coverage + PERF-S02 scaling).
//!
//! Mirrors the pinned `scripts/performance/sketch/workloads.json` fixtures at
//! 10/100/1000 parameters. No wall-time gates: criterion reports
//! distributions; fixture/outcome identity is gated by
//! `crates/sketch/tests/gcs_perf_identity.rs` and the runner's per-sample
//! validation. Run with `cargo bench -p remus-sketch --bench gcs_perf`.
//!
//! PERF-S02 additions: `mixed_sizes` (one coupled chain beside independent
//! pairs — a mixed block-size distribution) and `independent_solved_large`
//! (2000/5000/10000 parameters, which the dense path refuses under the
//! runner's 256 MiB Jacobian budget and the component path solves
//! block-wise).

#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use remus_sketch::{Constraint, GcsSystem, PointData};

const TOL: f64 = 1e-10;
const MAX_ITER: usize = 100;
const INCONSISTENT_MAX_ITER: usize = 50;

fn build_independent_under(n_params: usize) -> GcsSystem {
    let mut sys = GcsSystem::new();
    for i in 0..n_params / 2 {
        let ax = 10.0 * i as f64;
        let anchor = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .unwrap();
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
    }
    sys
}

fn build_independent_solved(n_params: usize) -> GcsSystem {
    let mut sys = GcsSystem::new();
    for i in 0..n_params / 2 {
        let ax = 10.0 * i as f64;
        let anchor = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .unwrap();
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    }
    sys
}

fn build_coupled_chain(n_params: usize) -> GcsSystem {
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap(),
    );
    for i in 1..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .unwrap(),
        );
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    }
    sys
}

fn build_redundant(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_solved(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for chunk in ids.chunks(2) {
        sys.add_constraint(Constraint::Distance(chunk[0], chunk[1], 5.0))
            .unwrap();
    }
    sys
}

fn build_inconsistent(n_params: usize) -> GcsSystem {
    let mut sys = build_independent_under(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    sys.add_constraint(Constraint::Distance(ids[0], ids[1], 6.0))
        .unwrap();
    sys
}

/// One coupled chain over half the parameter budget plus independent solved
/// pairs over the rest: a mixed block-size distribution (one large banded
/// block beside many tiny blocks).
fn build_mixed_sizes(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(4) && n_params >= 100);
    let chain_params = n_params / 2;
    let mut sys = GcsSystem::new();
    let n_pts = chain_params / 2 + 1;
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .unwrap(),
    );
    for i in 1..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .unwrap(),
        );
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).unwrap();
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .unwrap();
        sys.add_constraint(Constraint::Horizontal(line)).unwrap();
    }
    for i in 0..n_params / 4 {
        let ax = 1000.0 + 10.0 * i as f64;
        let anchor = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .unwrap();
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    }
    sys
}

fn bench_scaling(c: &mut Criterion, name: &str, build: fn(usize) -> GcsSystem) {
    let mut group = c.benchmark_group(format!("sketch/{name}"));
    group.sample_size(20);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    for size in [10_usize, 100, 1000] {
        group.bench_with_input(BenchmarkId::new("solve", size), &size, |b, &size| {
            b.iter_batched(
                || build(size),
                |mut sys| black_box(sys.solve(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
        group.bench_with_input(BenchmarkId::new("detailed", size), &size, |b, &size| {
            b.iter_batched(
                || build(size),
                |mut sys| black_box(sys.solve_detailed(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

fn bench_fixed_100(c: &mut Criterion, name: &str, build: fn(usize) -> GcsSystem, max_iter: usize) {
    let mut group = c.benchmark_group(format!("sketch/{name}"));
    group.sample_size(20);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    group.bench_function("solve", |b| {
        b.iter_batched(
            || build(100),
            |mut sys| black_box(sys.solve(max_iter, TOL)).unwrap(),
            criterion::BatchSize::SmallInput,
        );
    });
    group.bench_function("detailed", |b| {
        b.iter_batched(
            || build(100),
            |mut sys| black_box(sys.solve_detailed(max_iter, TOL)).unwrap(),
            criterion::BatchSize::SmallInput,
        );
    });
    group.finish();
}

fn bench_drag(c: &mut Criterion) {
    let mut group = c.benchmark_group("sketch/drag_100");
    group.sample_size(20);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    group.bench_function("solve_step", |b| {
        b.iter_batched(
            || {
                let mut sys = build_coupled_chain(100);
                sys.solve(MAX_ITER, TOL).unwrap();
                let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
                ids.sort_by_key(|id| id.index());
                (sys, ids[25])
            },
            |(mut sys, target)| {
                {
                    let slot = sys.point_mut(target).unwrap();
                    slot.x += 0.5;
                    slot.y += 0.25;
                }
                black_box(sys.solve(MAX_ITER, TOL)).unwrap();
            },
            criterion::BatchSize::SmallInput,
        );
    });
    group.bench_function("detailed_step", |b| {
        b.iter_batched(
            || {
                let mut sys = build_coupled_chain(100);
                sys.solve(MAX_ITER, TOL).unwrap();
                let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
                ids.sort_by_key(|id| id.index());
                (sys, ids[25])
            },
            |(mut sys, target)| {
                {
                    let slot = sys.point_mut(target).unwrap();
                    slot.x += 0.5;
                    slot.y += 0.25;
                }
                black_box(sys.solve_detailed(MAX_ITER, TOL)).unwrap();
            },
            criterion::BatchSize::SmallInput,
        );
    });
    group.finish();
}

fn sketch_perf(c: &mut Criterion) {
    bench_scaling(c, "independent_under", build_independent_under);
    bench_scaling(c, "independent_solved", build_independent_solved);
    bench_scaling(c, "coupled_chain", build_coupled_chain);
    bench_fixed_100(c, "redundant_100", build_redundant, MAX_ITER);
    bench_fixed_100(
        c,
        "inconsistent_100",
        build_inconsistent,
        INCONSISTENT_MAX_ITER,
    );
    bench_drag(c);
    bench_mixed_sizes(c);
    bench_large_independent(c);
    bench_ellipse_scaling(c, "ellipse_driven", build_ellipse_driven);
    bench_ellipse_scaling(c, "ellipse_mixed", build_ellipse_mixed);
    bench_s04_coupled(c);
    bench_s04_variants(c);
}

/// B75 ellipse workloads: `n_params` counts free solver parameters.
/// A fixed-center driven ellipse carries 3 (a, b, phi).
fn build_ellipse_driven(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(3) && n_params >= 3);
    let mut sys = GcsSystem::new();
    for i in 0..n_params / 3 {
        let ax = 20.0 * i as f64;
        let center = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let ell = sys.add_ellipse(center, 1.0, 1.0, 0.0).unwrap();
        sys.add_constraint(Constraint::EllipseAxisA(ell, 6.0))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAxisB(ell, 2.5))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAngle(ell, 0.7))
            .unwrap();
    }
    sys
}

/// Mixed components: disjoint distance pairs beside driven ellipses, so the
/// component path handles heterogeneous blocks in one solve.
///
/// `n_params` must be a multiple of 30: 3/5 fall on distance pairs (2 params
/// each) and 2/5 on driven ellipses (3 params each) at a far offset, so a
/// shared trust region would couple unrelated scales.
fn build_ellipse_mixed(n_params: usize) -> GcsSystem {
    assert!(n_params.is_multiple_of(30) && n_params >= 30);
    let mut sys = GcsSystem::new();
    // 3/5 of params: classic anchor/free distance pairs (2 params each).
    // Pair count is (3n/5)/2 = 3n/10.
    for i in 0..(3 * n_params / 10) {
        let ax = 10.0 * i as f64;
        let anchor = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let free = sys
            .add_point(PointData {
                x: ax + 1.0,
                y: 1.0,
                fixed: false,
            })
            .unwrap();
        sys.add_constraint(Constraint::Distance(anchor, free, 5.0))
            .unwrap();
        sys.add_constraint(Constraint::FixY(free, 4.0)).unwrap();
    }
    // 2/5 of params: driven ellipses (3 params each) at a far offset, so a
    // shared trust region would couple unrelated scales.
    // Ellipse count is (2n/5)/3 = 2n/15.
    for i in 0..(2 * n_params / 5 / 3) {
        let ax = 1.0e4 + 20.0 * i as f64;
        let center = sys
            .add_point(PointData {
                x: ax,
                y: 0.0,
                fixed: true,
            })
            .unwrap();
        let ell = sys.add_ellipse(center, 1.0, 1.0, 0.0).unwrap();
        sys.add_constraint(Constraint::EllipseAxisA(ell, 6.0))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAxisB(ell, 2.5))
            .unwrap();
        sys.add_constraint(Constraint::EllipseAngle(ell, 0.7))
            .unwrap();
    }
    sys
}

/// Ellipse scaling: driven ellipses at 9/99/999 params, mixed at 30/300/990.
fn bench_ellipse_scaling(c: &mut Criterion, name: &str, build: fn(usize) -> GcsSystem) {
    let sizes: &[usize] = if name == "ellipse_mixed" {
        &[30, 300, 990]
    } else {
        &[9, 99, 999]
    };
    let mut group = c.benchmark_group(format!("sketch/{name}"));
    group.sample_size(20);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    for size in sizes.iter().copied() {
        group.bench_with_input(BenchmarkId::new("solve", size), &size, |b, &size| {
            b.iter_batched(
                || build(size),
                |mut sys| black_box(sys.solve(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
        group.bench_with_input(BenchmarkId::new("detailed", size), &size, |b, &size| {
            b.iter_batched(
                || build(size),
                |mut sys| black_box(sys.solve_detailed(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

/// Mixed block-size distribution at 100/1000 parameters.
fn bench_mixed_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("sketch/mixed_sizes");
    group.sample_size(20);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    for size in [100_usize, 1000] {
        group.bench_with_input(BenchmarkId::new("solve", size), &size, |b, &size| {
            b.iter_batched(
                || build_mixed_sizes(size),
                |mut sys| black_box(sys.solve(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
        group.bench_with_input(BenchmarkId::new("detailed", size), &size, |b, &size| {
            b.iter_batched(
                || build_mixed_sizes(size),
                |mut sys| black_box(sys.solve_detailed(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

/// Independent-pair scaling past the dense path's 256 MiB Jacobian budget
/// (10000 params = 800 MB dense; component blocks stay 2x2).
fn bench_large_independent(c: &mut Criterion) {
    let mut group = c.benchmark_group("sketch/independent_solved_large");
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    for size in [2000_usize, 5000, 10_000] {
        group.bench_with_input(BenchmarkId::new("solve", size), &size, |b, &size| {
            b.iter_batched(
                || build_independent_solved(size),
                |mut sys| black_box(sys.solve(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
        group.bench_with_input(BenchmarkId::new("detailed", size), &size, |b, &size| {
            b.iter_batched(
                || build_independent_solved(size),
                |mut sys| black_box(sys.solve_detailed(MAX_ITER, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
    }
    group.finish();
}

// ── PERF-S04 bounded-slice workloads ─────────────────────────────────────
//
// All builders are deterministic (no RNG), closed-form starts near the
// solution (same style as the S01 chain: exact grid plus a small alternating
// perturb), and pinned by `gcs_perf_identity`-style dimension/outcome gates
// in `crates/sketch/tests/s04_identity.rs` (added with the slice).
//
// * `s04_grid`: 2D point grid, Distance + Horizontal/Vertical per edge.
//   Overdetermined (`m > n`), full column rank, point-only (banded).
// * `s04_linkage`: chain plus triangular bracing Distance(p_k,p_{k+2})=2.
//   Overdetermined, full column rank, point-only (banded).
// * `s04_tangent`: horizontal lines tangent to fixed-center circles
//   (Distance + TangentLineCircle + CircleRadius per segment). Square,
//   full rank, mixes point and radius params (wide band → dense fallback,
//   correctness preserved, no win claimed).
// * `s04_ellipse_chain`: chain points driving ellipses via PointOnEllipse
//   plus axis drives (Distance + 3 axis + PointOnEllipse per segment).
//   Square, full rank, mixes points and ellipse scalars (wide band → dense
//   fallback, correctness preserved).
// Variants reuse the chain base: `s04_chain_redundant` (extra Distance),
// `s04_chain_inconsistent` (contradictory Distance), `s04_chain_under`
// (drop Horizontal → m < n), `s04_chain_ill` (near-coincident start →
// ill-conditioned first iterate, still converges).

fn build_s04_grid(n_params: usize) -> GcsSystem {
    // Smallest grid with >= n_params free params (2 per free point).
    // W chosen so W*H >= n_pts; last row may be partial (still connected).
    let n_pts_needed = n_params / 2 + 1;
    let w = ((n_pts_needed as f64).sqrt().ceil() as usize).max(2);
    let mut sys = GcsSystem::new();
    let mut grid: Vec<Vec<_>> = Vec::new();
    let mut count = 0_usize;
    for r in 0.. {
        if count >= n_pts_needed {
            break;
        }
        let mut row = Vec::new();
        for c in 0..w {
            if count >= n_pts_needed {
                break;
            }
            let fixed = r == 0 && c == 0;
            let x = c as f64 + 0.1 * f64::from((c % 2) as u8);
            let y = r as f64 + 0.1 * f64::from((r % 2) as u8);
            row.push(sys.add_point(PointData { x, y, fixed }).expect("finite"));
            count += 1;
        }
        grid.push(row);
    }
    for (r, row) in grid.iter().enumerate() {
        for (c, &p) in row.iter().enumerate() {
            if c + 1 < row.len() {
                let q = row[c + 1];
                let line = sys.add_line(p, q).expect("valid");
                sys.add_constraint(Constraint::Distance(p, q, 1.0))
                    .expect("valid");
                sys.add_constraint(Constraint::Horizontal(line))
                    .expect("valid");
            }
            if r + 1 < grid.len() && c < grid[r + 1].len() {
                let q = grid[r + 1][c];
                let line = sys.add_line(p, q).expect("valid");
                sys.add_constraint(Constraint::Distance(p, q, 1.0))
                    .expect("valid");
                sys.add_constraint(Constraint::Vertical(line))
                    .expect("valid");
            }
        }
    }
    sys
}

fn build_s04_linkage(n_params: usize) -> GcsSystem {
    // Interleaved bracing so the scaled-diagonal band stays small (trailing
    // bracing blocks rows far from their columns and force dense fallback).
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("finite"),
    );
    for i in 1..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .expect("finite"),
        );
        let w = &pts[(i - 1)..=i];
        let line = sys.add_line(w[0], w[1]).expect("valid");
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .expect("valid");
        sys.add_constraint(Constraint::Horizontal(line))
            .expect("valid");
        if i >= 2 {
            sys.add_constraint(Constraint::Distance(pts[i - 2], pts[i], 2.0))
                .expect("valid");
        }
    }
    sys
}

fn build_s04_tangent(n_params: usize) -> GcsSystem {
    // 3 params per segment (new point 2 + new radius 1); 3 rows per segment.
    assert!(n_params.is_multiple_of(3) && n_params >= 30);
    let segs = n_params / 3;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(segs + 1);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("finite"),
    );
    for i in 1..=segs {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.2 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .expect("finite"),
        );
    }
    for (k, w) in pts.windows(2).enumerate() {
        let line = sys.add_line(w[0], w[1]).expect("valid");
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .expect("valid");
        let center = sys
            .add_point(PointData {
                x: k as f64,
                y: 1.0,
                fixed: true,
            })
            .expect("finite");
        let circ = sys.add_circle(center, 1.2).expect("valid");
        sys.add_constraint(Constraint::TangentLineCircle(line, circ))
            .expect("valid");
        sys.add_constraint(Constraint::CircleRadius(circ, 1.0))
            .expect("valid");
    }
    sys
}

fn build_s04_ellipse_chain(n_params: usize) -> GcsSystem {
    // 5 params per segment (new point 2 + new ellipse 3); 5 rows per segment.
    assert!(n_params.is_multiple_of(5) && n_params >= 50);
    let segs = n_params / 5;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(segs + 1);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("finite"),
    );
    for i in 1..=segs {
        pts.push(
            sys.add_point(PointData {
                x: 4.0 * i as f64,
                y: 0.3 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .expect("finite"),
        );
    }
    for (k, w) in pts.windows(2).enumerate() {
        let _line = sys.add_line(w[0], w[1]).expect("valid");
        sys.add_constraint(Constraint::Distance(w[0], w[1], 4.0))
            .expect("valid");
        // Ellipse centered at the left point (fixed for k=0, free after).
        let ell = sys.add_ellipse(w[0], 3.0, 1.5, 0.1).expect("valid ellipse");
        sys.add_constraint(Constraint::EllipseAxisA(ell, 4.0))
            .expect("valid");
        sys.add_constraint(Constraint::EllipseAxisB(ell, 2.0))
            .expect("valid");
        sys.add_constraint(Constraint::EllipseAngle(ell, 0.0))
            .expect("valid");
        sys.add_constraint(Constraint::PointOnEllipse(w[1], ell))
            .expect("valid");
        let _ = k;
    }
    sys
}

fn build_s04_chain_redundant(n_params: usize) -> GcsSystem {
    let mut sys = build_coupled_chain(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    for w in ids.windows(2) {
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .expect("valid");
    }
    sys
}

fn build_s04_chain_inconsistent(n_params: usize) -> GcsSystem {
    let mut sys = build_coupled_chain(n_params);
    let mut ids: Vec<_> = sys.points().map(|(id, _)| id).collect();
    ids.sort_by_key(|id| id.index());
    sys.add_constraint(Constraint::Distance(ids[0], ids[1], 2.0))
        .expect("valid");
    sys
}

fn build_s04_chain_under(n_params: usize) -> GcsSystem {
    // Chain with Horizontal dropped: Distance only (m = n/2 < n).
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("finite"),
    );
    for i in 1..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .expect("finite"),
        );
    }
    for w in pts.windows(2) {
        let _line = sys.add_line(w[0], w[1]).expect("valid");
        sys.add_constraint(Constraint::Distance(w[0], w[1], 1.0))
            .expect("valid");
    }
    sys
}

fn build_s04_chain_ill(n_params: usize) -> GcsSystem {
    // Near-coincident start: first free point almost on the anchor so the
    // first Distance gradient is ill-conditioned, then the chain continues.
    let n_pts = n_params / 2 + 1;
    let mut sys = GcsSystem::new();
    let mut pts = Vec::with_capacity(n_pts);
    pts.push(
        sys.add_point(PointData {
            x: 0.0,
            y: 0.0,
            fixed: true,
        })
        .expect("finite"),
    );
    pts.push(
        sys.add_point(PointData {
            x: 1e-9,
            y: 0.0,
            fixed: false,
        })
        .expect("finite"),
    );
    for i in 2..n_pts {
        pts.push(
            sys.add_point(PointData {
                x: i as f64,
                y: 0.5 * f64::from((i % 2) as u8),
                fixed: false,
            })
            .expect("finite"),
        );
    }
    for w in pts.windows(2) {
        let line = sys.add_line(w[0], w[1]).expect("valid");
        let target = if w[0] == pts[0] && w[1] == pts[1] {
            1.0
        } else {
            1.0
        };
        sys.add_constraint(Constraint::Distance(w[0], w[1], target))
            .expect("valid");
        sys.add_constraint(Constraint::Horizontal(line))
            .expect("valid");
    }
    sys
}

#[allow(clippy::too_many_lines)]
fn bench_s04_coupled(c: &mut Criterion) {
    // Coupled families at 100/300 params (300 keeps dense baselines bounded;
    // 1000-param cells live in the paired release-bench report, not here).
    let mut group = c.benchmark_group("sketch/s04_coupled");
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(300));
    group.measurement_time(Duration::from_secs(1));
    for (name, build) in [
        ("chain", build_coupled_chain as fn(usize) -> GcsSystem),
        ("grid", build_s04_grid),
        ("linkage", build_s04_linkage),
        ("tangent", build_s04_tangent),
        ("ellipse_chain", build_s04_ellipse_chain),
    ] {
        for size in [99_usize, 300] {
            // Tangent needs multiples of 3, ellipse multiples of 5, grid any.
            let size = if name == "tangent" {
                (size / 3) * 3
            } else if name == "ellipse_chain" {
                (size / 5) * 5
            } else {
                (size / 2) * 2
            };
            if size < 30 {
                continue;
            }
            group.bench_with_input(
                BenchmarkId::new(format!("{name}_solve"), size),
                &size,
                |b, &size| {
                    b.iter_batched(
                        || build(size),
                        |mut sys| black_box(sys.solve(MAX_ITER, TOL)).unwrap(),
                        criterion::BatchSize::SmallInput,
                    );
                },
            );
        }
    }
    group.finish();
}

fn bench_s04_variants(c: &mut Criterion) {
    let mut variants = c.benchmark_group("sketch/s04_variants");
    variants.sample_size(10);
    variants.warm_up_time(Duration::from_millis(300));
    variants.measurement_time(Duration::from_secs(1));
    for (name, build, iters) in [
        (
            "chain_redundant",
            build_s04_chain_redundant as fn(usize) -> GcsSystem,
            MAX_ITER,
        ),
        (
            "chain_inconsistent",
            build_s04_chain_inconsistent as fn(usize) -> GcsSystem,
            INCONSISTENT_MAX_ITER,
        ),
        (
            "chain_under",
            build_s04_chain_under as fn(usize) -> GcsSystem,
            MAX_ITER,
        ),
        (
            "chain_ill",
            build_s04_chain_ill as fn(usize) -> GcsSystem,
            MAX_ITER,
        ),
    ] {
        variants.bench_function(format!("{name}_100_solve"), |b| {
            b.iter_batched(
                || build(100),
                |mut sys| black_box(sys.solve(iters, TOL)).unwrap(),
                criterion::BatchSize::SmallInput,
            );
        });
    }
    variants.finish();
}

criterion_group!(benches, sketch_perf);
criterion_main!(benches);
