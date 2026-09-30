#![allow(
    missing_docs,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss
)]

//! PERF-N01 interpolation scaling: compact band storage vs the historical
//! n-by-n allocation. Measures `interpolate` wall time plus the exact stored
//! coefficient counts (`n*n` dense vs `n*(3*bw+1)` compact) at
//! 128/512/2048/8192 points, degrees 1/3/5, uniform / clustered /
//! near-singular spacing.

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use remus_math::nurbs::fitting::interpolate;
use remus_math::vec::Point3;

fn make_points(n: usize, kind: &str) -> Vec<Point3> {
    match kind {
        "uniform" => (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                Point3::new(
                    t * 10.0,
                    (t * std::f64::consts::TAU).sin(),
                    (t * std::f64::consts::TAU * 0.5).cos() * 0.5,
                )
            })
            .collect(),
        "clustered" => (0..n)
            .map(|i| {
                let u = i as f64 / (n - 1) as f64;
                let t = u.powi(4);
                Point3::new(t * 10.0, (t * std::f64::consts::TAU).sin(), t * t)
            })
            .collect(),
        "near-singular" => {
            let mut pts: Vec<Point3> = (0..n)
                .map(|i| {
                    let t = i as f64 / (n - 1) as f64;
                    Point3::new(t * 10.0, (t * std::f64::consts::TAU).sin(), 0.0)
                })
                .collect();
            if n >= 3 {
                let m = n / 2;
                let prev = pts[m - 1];
                pts[m] = Point3::new(prev.x() + 1e-12, prev.y() + 1e-12, 0.0);
            }
            pts
        }
        _ => unreachable!(),
    }
}

fn bench_interpolation(c: &mut Criterion) {
    let mut group = c.benchmark_group("fitting_interpolation");
    group.throughput(Throughput::Elements(1));
    // Keep the 8192-point case affordable on shared runners: fewer samples.
    group.sample_size(10);

    for &n in &[128usize, 512, 2048, 8192] {
        for &degree in &[1usize, 3, 5] {
            for kind in ["uniform", "clustered", "near-singular"] {
                // 8192 x degree-1/5 x clustered variants are the slowest
                // allocators; restrict them to the representative degree-3
                // uniform/clustered pair to keep the bench under ~2 min.
                if n == 8192 && !(degree == 3 && kind != "near-singular") {
                    continue;
                }
                // 2048 keeps the full matrix; 512 and below are fast.
                if n == 2048 && degree == 5 && kind == "near-singular" {
                    continue;
                }
                let pts = make_points(n, kind);
                // Sanity: the fixture must interpolate before benchmarking.
                assert!(interpolate(&pts, degree).is_ok());
                let id = BenchmarkId::new(
                    format!("n{n}_deg{degree}_{kind}"),
                    format!(
                        "stored_dense_{}_compact_{}",
                        n * n,
                        n * (3 * degree.max(1) + 1)
                    ),
                );
                group.bench_with_input(id, &degree, |bencher, &deg| {
                    bencher.iter(|| black_box(interpolate(black_box(&pts), deg).unwrap()));
                });
            }
        }
    }
    group.finish();
}

fn criterion_config() -> Criterion {
    Criterion::default()
        .warm_up_time(Duration::from_millis(300))
        .measurement_time(Duration::from_secs(2))
}

criterion_group! {
    name = benches;
    config = criterion_config();
    targets = bench_interpolation
}
criterion_main!(benches);
