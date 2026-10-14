//! Isolated carrier-solver timing; whole recognition is measured separately.
#![allow(clippy::unwrap_used, clippy::expect_used, missing_docs)]

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use remus_math::analytic_intersection::intersect_line_torus;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use std::hint::black_box;
use std::time::Duration;

fn torus_roots(c: &mut Criterion) {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 3.0).unwrap();
    let mut group = c.benchmark_group("torus_ray_roots");
    group
        .sample_size(20)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(2));
    group.throughput(Throughput::Elements(64));
    for (name, offset) in [
        ("crossings", 0.0),
        ("misses", 6.0),
        ("near_grazes", 3.0 - 1e-8),
    ] {
        let rays: Vec<_> = (0..64)
            .map(|i| {
                let angle = f64::from(i) * std::f64::consts::TAU / 64.0;
                let (sine, cosine) = angle.sin_cos();
                (
                    Point3::new(-15.0 * cosine, -15.0 * sine, offset),
                    Vec3::new(cosine, sine, 0.0),
                )
            })
            .collect();
        group.bench_function(name, |b| {
            b.iter(|| {
                for &(origin, direction) in &rays {
                    black_box(intersect_line_torus(
                        black_box(&torus),
                        black_box(origin),
                        black_box(direction),
                    ));
                }
            });
        });
    }
    group.finish();
}
criterion_group!(benches, torus_roots);
criterion_main!(benches);
