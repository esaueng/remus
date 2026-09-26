//! Point-classification reuse benchmarks (PERF-Q01).
//!
//! Compares the one-shot path (bounds, BVH and trim polygons rebuilt per ray)
//! against [`remus_check::classify::PreparedSolid`] (built once per solid)
//! over 1, 100 and 10000 points on one solid, per the PERF-Q01 acceptance
//! matrix. The deterministic preparation reduction itself is guarded by the
//! `perf-counters` test in `remus-check`; this bench pins the wall-clock
//! shape, including single-point overhead.
//!
//! Run locally: `cargo bench -p remus-operations --bench classify_perf`

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, criterion_group, criterion_main};

use remus_check::classify::{ClassifyOptions, PreparedSolid, classify_point};
use remus_math::vec::Point3;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Deterministic low-discrepancy sample in [0, 1).
fn halton(mut i: u32, base: u32) -> f64 {
    let (mut f, mut r) = (1.0_f64, 0.0_f64);
    while i > 0 {
        f /= f64::from(base);
        r += f * f64::from(i % base);
        i /= base;
    }
    r
}

fn halton_box(n: u32, lo: Point3, hi: Point3) -> Vec<Point3> {
    (1..=n)
        .map(|i| {
            Point3::new(
                (hi.x() - lo.x()).mul_add(halton(i, 2), lo.x()),
                (hi.y() - lo.y()).mul_add(halton(i, 3), lo.y()),
                (hi.z() - lo.z()).mul_add(halton(i, 5), lo.z()),
            )
        })
        .collect()
}

fn count_inside(results: &[remus_check::classify::PointClassification]) -> usize {
    results
        .iter()
        .filter(|r| **r == remus_check::classify::PointClassification::Inside)
        .count()
}

/// One-shot loop vs prepared batch over the same points on the same solid.
fn bench_shape(
    group: &mut criterion::BenchmarkGroup<criterion::measurement::WallTime>,
    name: &str,
    topo: &Topology,
    solid: SolidId,
    points: &[Point3],
) {
    let options = ClassifyOptions::default();
    group.bench_function(format!("{name}/oneshot"), |bencher| {
        bencher.iter(|| {
            let mut inside = 0usize;
            for &point in points {
                if black_box(classify_point(topo, solid, point, &options).unwrap())
                    == remus_check::classify::PointClassification::Inside
                {
                    inside += 1;
                }
            }
            black_box(inside)
        });
    });
    let prepared = PreparedSolid::prepare(topo, solid).unwrap();
    group.bench_function(format!("{name}/prepared"), |bencher| {
        bencher.iter(|| {
            black_box(count_inside(black_box(
                &prepared.classify_points(points, &options).unwrap(),
            )))
        });
    });
}

fn bench_classify(c: &mut Criterion) {
    let mut group = c.benchmark_group("classify");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(2));

    // Box: the 1 / 100 / 10000 acceptance comparison on one solid.
    {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        for n in [1u32, 100, 10_000] {
            let points = halton_box(
                n,
                Point3::new(-1.0, -1.0, -1.0),
                Point3::new(11.0, 11.0, 11.0),
            );
            bench_shape(&mut group, &format!("box-{n}"), &topo, solid, &points);
        }
        // Single-point overhead: full preparation plus one query vs one
        // one-shot query, on the identical point. Preparation builds every
        // face's trims eagerly, so a lone query pays work the one-shot path
        // may never need.
        let point = halton_box(
            1,
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(11.0, 11.0, 11.0),
        )[0];
        let options = ClassifyOptions::default();
        group.bench_function("box-1/oneshot-single", |bencher| {
            bencher.iter(|| {
                black_box(classify_point(&topo, solid, black_box(point), &options).unwrap());
            });
        });
        group.bench_function("box-1/prepare-plus-single", |bencher| {
            bencher.iter(|| {
                let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
                black_box(prepared.classify_point(black_box(point), &options).unwrap());
            });
        });
    }

    // Curved solids: fewer points (heavier narrow phase per query).
    {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
        let points = halton_box(
            1000,
            Point3::new(-6.0, -6.0, -1.0),
            Point3::new(6.0, 6.0, 11.0),
        );
        bench_shape(&mut group, "cylinder-1000", &topo, solid, &points);
    }

    {
        let mut topo = Topology::new();
        let solid = make_sphere(&mut topo, 5.0, 32).unwrap();
        let points = halton_box(
            300,
            Point3::new(-6.0, -6.0, -6.0),
            Point3::new(6.0, 6.0, 6.0),
        );
        bench_shape(&mut group, "sphere-300", &topo, solid, &points);
    }

    group.finish();
}

criterion_group!(benches, bench_classify);
criterion_main!(benches);
