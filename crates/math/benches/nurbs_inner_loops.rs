#![allow(
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::unwrap_used,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use remus_math::nurbs::basis::{basis_funs_into, ders_basis_funs_into, find_span};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::predicates::{point_in_polygon, winding_number};
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point2, Point3, Vec3};

fn clamped_knots(degree: usize) -> Vec<f64> {
    let mut knots = vec![0.0; degree + 1];
    knots.extend(vec![1.0; degree + 1]);
    knots
}

fn curve(degree: usize) -> NurbsCurve {
    let scale = degree as f64;
    let control_points = (0..=degree)
        .map(|index| {
            let t = index as f64 / scale;
            Point3::new(t * 10.0, (t * std::f64::consts::TAU).sin(), t * t)
        })
        .collect::<Vec<_>>();
    let weights = (0..=degree)
        .map(|index| 1.0 + 0.05 * (index % 3) as f64)
        .collect();
    NurbsCurve::new(degree, clamped_knots(degree), control_points, weights)
        .expect("benchmark curve must be valid")
}

fn surface(degree: usize) -> NurbsSurface {
    let scale = degree as f64;
    let control_points = (0..=degree)
        .map(|u_index| {
            (0..=degree)
                .map(|v_index| {
                    let u = u_index as f64 / scale;
                    let v = v_index as f64 / scale;
                    Point3::new(u * 4.0, v * 4.0, (u * 2.0).sin() * (v * 2.0).cos())
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let weights = (0..=degree)
        .map(|u_index| {
            (0..=degree)
                .map(|v_index| 1.0 + 0.02 * ((u_index + v_index) % 5) as f64)
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    NurbsSurface::new(
        degree,
        degree,
        clamped_knots(degree),
        clamped_knots(degree),
        control_points,
        weights,
    )
    .expect("benchmark surface must be valid")
}

fn bench_nurbs(c: &mut Criterion) {
    let mut group = c.benchmark_group("nurbs");
    group.throughput(Throughput::Elements(1));

    for degree in [3_usize, 9] {
        let knots = clamped_knots(degree);
        let span = find_span(degree + 1, degree, 0.37, &knots);
        let mut basis_output = vec![0.0; degree + 1];
        group.bench_with_input(
            BenchmarkId::new("basis", format!("degree{degree}")),
            &degree,
            |bencher, &degree| {
                bencher.iter(|| {
                    basis_funs_into(
                        black_box(span),
                        black_box(0.37),
                        degree,
                        black_box(&knots),
                        black_box(&mut basis_output),
                    );
                    black_box(&basis_output);
                });
            },
        );

        let mut derivative_output = vec![0.0; 3 * (degree + 1)];
        group.bench_with_input(
            BenchmarkId::new("basis_derivatives", format!("degree{degree}")),
            &degree,
            |bencher, &degree| {
                bencher.iter(|| {
                    ders_basis_funs_into(
                        black_box(span),
                        black_box(0.37),
                        degree,
                        2,
                        black_box(&knots),
                        black_box(&mut derivative_output),
                    );
                    black_box(&derivative_output);
                });
            },
        );

        let curve = curve(degree);
        group.bench_with_input(
            BenchmarkId::new("curve_evaluate", format!("degree{degree}")),
            &degree,
            |bencher, _| bencher.iter(|| black_box(curve.evaluate(black_box(0.37)))),
        );
        group.bench_with_input(
            BenchmarkId::new("curve_derivatives", format!("degree{degree}")),
            &degree,
            |bencher, _| bencher.iter(|| black_box(curve.derivatives(black_box(0.37), 2))),
        );

        let surface = surface(degree);
        group.bench_with_input(
            BenchmarkId::new("surface_evaluate", format!("degree{degree}")),
            &degree,
            |bencher, _| {
                bencher.iter(|| black_box(surface.evaluate(black_box(0.37), black_box(0.61))));
            },
        );
        group.bench_with_input(
            BenchmarkId::new("surface_derivatives", format!("degree{degree}")),
            &degree,
            |bencher, _| {
                bencher
                    .iter(|| black_box(surface.derivatives(black_box(0.37), black_box(0.61), 2)));
            },
        );
    }

    group.finish();
}

fn bench_flamegraph_hot_paths(c: &mut Criterion) {
    let mut group = c.benchmark_group("flamegraph_hot");
    group.throughput(Throughput::Elements(1));
    let cylinder =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0)
            .expect("benchmark cylinder must be valid");
    group.bench_function("analytic_cylinder_evaluate", |bencher| {
        bencher.iter(|| black_box(cylinder.evaluate(black_box(1.23), black_box(4.56))));
    });
    group.bench_function("analytic_cylinder_project_point", |bencher| {
        bencher.iter(|| black_box(cylinder.project_point(black_box(Point3::new(1.5, 1.25, 4.56)))));
    });

    let polygon = (0..64)
        .map(|index| {
            let angle = std::f64::consts::TAU * index as f64 / 64.0;
            Point2::new(angle.cos() * 10.0, angle.sin() * 10.0)
        })
        .collect::<Vec<_>>();
    group.bench_function("winding_number_64", |bencher| {
        bencher.iter(|| winding_number(black_box(Point2::new(0.25, -0.75)), black_box(&polygon)));
    });
    group.bench_function("point_in_polygon_64", |bencher| {
        bencher.iter(|| point_in_polygon(black_box(Point2::new(0.25, -0.75)), black_box(&polygon)));
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
    targets = bench_nurbs, bench_flamegraph_hot_paths
}
criterion_main!(benches);
