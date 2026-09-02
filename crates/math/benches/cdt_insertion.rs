#![allow(
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::unwrap_used,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BatchSize, BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use remus_math::cdt::Cdt;
use remus_math::vec::Point2;

fn deterministic_points(count: usize) -> Vec<Point2> {
    const PHI_CONJUGATE: f64 = 0.618_033_988_749_894_9;
    const SQRT_TWO_CONJUGATE: f64 = 0.414_213_562_373_095_15;
    (0..count)
        .map(|index| {
            let i = index as f64 + 1.0;
            Point2::new(
                (i * PHI_CONJUGATE).fract() * 1000.0,
                (i * SQRT_TWO_CONJUGATE).fract() * 1000.0,
            )
        })
        .collect()
}

fn bench_cdt_insertion(c: &mut Criterion) {
    let mut group = c.benchmark_group("cdt_insertion");
    for count in [1_000_usize, 10_000] {
        let points = deterministic_points(count);
        group.throughput(Throughput::Elements(count as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(count),
            &count,
            |bencher, &count| {
                bencher.iter_batched(
                    || {
                        Cdt::with_capacity(
                            (Point2::new(0.0, 0.0), Point2::new(1000.0, 1000.0)),
                            count,
                        )
                    },
                    |mut cdt| {
                        black_box(
                            cdt.insert_points_hilbert(black_box(&points))
                                .expect("benchmark points must triangulate"),
                        );
                    },
                    BatchSize::SmallInput,
                );
            },
        );
    }
    group.finish();
}

fn criterion_config() -> Criterion {
    Criterion::default()
        .sample_size(10)
        .warm_up_time(Duration::from_millis(300))
        .measurement_time(Duration::from_secs(1))
}

criterion_group! {
    name = benches;
    config = criterion_config();
    targets = bench_cdt_insertion
}
criterion_main!(benches);
