//! Criterion sketch benchmarks (PERF-S01 baseline coverage).
//!
//! Mirrors the pinned `scripts/performance/sketch/workloads.json` fixtures at
//! 10/100/1000 parameters. No wall-time gates: criterion reports
//! distributions; fixture/outcome identity is gated by
//! `crates/sketch/tests/gcs_perf_identity.rs` and the runner's per-sample
//! validation. Run with `cargo bench -p remus-sketch --bench gcs_perf`.

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
}

criterion_group!(benches, sketch_perf);
criterion_main!(benches);
