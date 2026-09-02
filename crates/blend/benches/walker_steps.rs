#![allow(clippy::cast_possible_truncation, clippy::expect_used, missing_docs)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use remus_blend::benchmark_plane_pair_walk;

fn bench_walker(c: &mut Criterion) {
    let repetitions = 16_usize;
    let sections_per_walk =
        benchmark_plane_pair_walk(1).expect("walker benchmark fixture must converge");
    let mut group = c.benchmark_group("blend_walker");
    group.throughput(Throughput::Elements(
        sections_per_walk.saturating_mul(repetitions) as u64,
    ));
    group.bench_function("plane_pair_steps", |bencher| {
        bencher.iter(|| {
            black_box(
                benchmark_plane_pair_walk(black_box(repetitions))
                    .expect("walker benchmark fixture must converge"),
            )
        });
    });
    group.finish();
}

fn criterion_config() -> Criterion {
    Criterion::default()
        .sample_size(20)
        .warm_up_time(Duration::from_millis(300))
        .measurement_time(Duration::from_secs(1))
}

criterion_group! {
    name = benches;
    config = criterion_config();
    targets = bench_walker
}
criterion_main!(benches);
