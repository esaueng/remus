//! PERF-Q02 benefit probe: persistent classification cache vs baselines.
//!
//! Three regimes on one box solid (small model) plus a cache-thrashing
//! workload (more solids than the 16-entry bound):
//!
//! - repeated: 10 000 points, no edits (amortised preparation wins).
//! - occasional edits: 10 batches of 100 points with an unrelated allocation
//!   between batches (whole-topology invalidation forces one rebuild per
//!   batch; still wins over per-ray rebuilds).
//! - constant edits: an unrelated allocation before every query (every lookup
//!   misses; management cost must not be claimed as a win).
//! - thrashing: 32 solids round-robin with capacity 16 (FIFO evicts; still
//!   correct, hit rate near zero).
//!
//! Reports hit rate, rebuilds, retained bytes, and preparation counters
//! (`perf-counters` feature) alongside wall-clock. Run:
//! `cargo bench -p remus-operations --bench classify_cache_perf`
//! with `--features perf-counters` for the counter cells.

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};
use remus_check::classify::{ClassificationCache, ClassifyOptions, PreparedSolid, classify_point};
use remus_math::vec::Point3;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn halton(mut i: u32, base: u32) -> f64 {
    let (mut f, mut r) = (1.0_f64, 0.0_f64);
    while i > 0 {
        f /= f64::from(base);
        r += f * f64::from(i % base);
        i /= base;
    }
    r
}

fn halton_box(n: u32) -> Vec<Point3> {
    (1..=n)
        .map(|i| {
            Point3::new(
                11.0f64.mul_add(halton(i, 2), -1.0),
                11.0f64.mul_add(halton(i, 3), -1.0),
                11.0f64.mul_add(halton(i, 5), -1.0),
            )
        })
        .collect()
}

fn bench_repeated(c: &mut Criterion) {
    let mut group = c.benchmark_group("cache/repeated-10000");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let points = halton_box(10_000);
    let options = ClassifyOptions::default();

    group.bench_function("oneshot", |b| {
        b.iter(|| {
            let mut inside = 0usize;
            for &p in &points {
                if black_box(classify_point(&topo, solid, p, &options).unwrap())
                    == remus_check::classify::PointClassification::Inside
                {
                    inside += 1;
                }
            }
            black_box(inside)
        });
    });

    group.bench_function("borrowed-prepared", |b| {
        b.iter(|| {
            let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
            black_box(
                prepared
                    .classify_points(&points, &options)
                    .unwrap()
                    .iter()
                    .filter(|r| **r == remus_check::classify::PointClassification::Inside)
                    .count(),
            )
        });
    });

    group.bench_function("persistent-cache", |b| {
        b.iter(|| {
            let mut cache = ClassificationCache::new();
            black_box(
                cache
                    .classify_points(&topo, solid, &points, &options)
                    .unwrap()
                    .iter()
                    .filter(|r| **r == remus_check::classify::PointClassification::Inside)
                    .count(),
            );
            // Second batch hits: preparation amortised across calls.
            black_box(
                cache
                    .classify_points(&topo, solid, &points, &options)
                    .unwrap()
                    .len(),
            );
            black_box(cache.stats());
        });
    });

    group.finish();
}

fn bench_occasional_edits(c: &mut Criterion) {
    let mut group = c.benchmark_group("cache/occasional-edits-10x100");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));

    let points = halton_box(100);
    let options = ClassifyOptions::default();

    group.bench_function("oneshot", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            let mut total = 0usize;
            for _ in 0..10 {
                for &p in &points {
                    if black_box(classify_point(&topo, solid, p, &options).unwrap())
                        == remus_check::classify::PointClassification::Inside
                    {
                        total += 1;
                    }
                }
                // Unrelated edit: new box elsewhere bumps the generation.
                let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            }
            black_box(total)
        });
    });

    group.bench_function("persistent-cache", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            let mut cache = ClassificationCache::new();
            let mut total = 0usize;
            for _ in 0..10 {
                total += black_box(
                    cache
                        .classify_points(&topo, solid, &points, &options)
                        .unwrap()
                        .iter()
                        .filter(|r| **r == remus_check::classify::PointClassification::Inside)
                        .count(),
                );
                let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            }
            black_box((total, cache.stats()));
        });
    });

    group.finish();
}

fn bench_constant_edits(c: &mut Criterion) {
    let mut group = c.benchmark_group("cache/constant-edits-200");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));

    let points = halton_box(200);
    let options = ClassifyOptions::default();

    group.bench_function("oneshot", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            let mut total = 0usize;
            for &p in &points {
                let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
                if black_box(classify_point(&topo, solid, p, &options).unwrap())
                    == remus_check::classify::PointClassification::Inside
                {
                    total += 1;
                }
            }
            black_box(total)
        });
    });

    // No speedup is claimed here: every lookup misses by design.
    group.bench_function("persistent-cache", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            let mut cache = ClassificationCache::new();
            let mut total = 0usize;
            for &p in &points {
                let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
                if black_box(cache.classify_point(&topo, solid, p, &options).unwrap())
                    == remus_check::classify::PointClassification::Inside
                {
                    total += 1;
                }
            }
            black_box((total, cache.stats()));
        });
    });

    group.finish();
}

fn bench_thrashing(c: &mut Criterion) {
    let mut group = c.benchmark_group("cache/thrashing-32-solids");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));

    let options = ClassifyOptions::default();
    let point = Point3::new(5.0, 5.0, 5.0);

    group.bench_function("persistent-cache-cap16", |b| {
        b.iter(|| {
            let mut topo = Topology::new();
            let mut solids = Vec::new();
            for _ in 0..32 {
                solids.push(make_box(&mut topo, 10.0, 10.0, 10.0).unwrap());
            }
            let mut cache = ClassificationCache::with_capacity(16);
            let mut inside = 0usize;
            // Round-robin exceeds capacity: FIFO thrashes but stays correct.
            for _ in 0..2 {
                for &s in &solids {
                    if black_box(cache.classify_point(&topo, s, point, &options).unwrap())
                        == remus_check::classify::PointClassification::Inside
                    {
                        inside += 1;
                    }
                }
            }
            black_box((inside, cache.stats()));
        });
    });

    group.finish();
}

#[allow(clippy::unused_self)]
fn _assert_solid(_: &mut Criterion, _: &Topology, _: SolidId) {}

criterion_group!(
    benches,
    bench_repeated,
    bench_occasional_edits,
    bench_constant_edits,
    bench_thrashing
);
criterion_main!(benches);
