//! Equivalent-geometry evaluation and qualified reduction/fitting benchmarks.
#![allow(
    clippy::cast_precision_loss,
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use remus_math::context::{FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::nurbs::cubic_fit::{CubicFitOptions, fit_cubic_curve};
use remus_math::nurbs::knot_ops::{
    curve_knot_insert, surface_knot_insert_u, surface_knot_insert_v,
};
use remus_math::nurbs::reduction::{simplify_curve, simplify_surface};
use remus_math::nurbs::reuse::ReductionOptions;
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_math::vec::Point3;

fn arc() -> NurbsCurve {
    NurbsCurve::new(
        2,
        vec![0., 0., 0., 1., 1., 1.],
        vec![
            Point3::new(1., 0., 0.),
            Point3::new(1., 1., 0.),
            Point3::new(0., 1., 0.),
        ],
        vec![1., std::f64::consts::FRAC_1_SQRT_2, 1.],
    )
    .unwrap()
}

fn cylinder() -> NurbsSurface {
    let source = arc();
    NurbsSurface::new(
        2,
        1,
        source.knots().to_vec(),
        vec![0., 0., 1., 1.],
        source
            .control_points()
            .iter()
            .map(|p| vec![Point3::new(p.x(), p.y(), 0.), Point3::new(p.x(), p.y(), 3.)])
            .collect(),
        source.weights().iter().map(|&w| vec![w; 2]).collect(),
    )
    .unwrap()
}

fn bench_truck_reuse(c: &mut Criterion) {
    let tolerance = 1e-8;
    let options = ReductionOptions {
        tolerance,
        max_work: 10_000_000,
    };
    let context = OperationContext::new()
        .with_fallback(FallbackPolicy::AllowApproximate { budget: tolerance })
        .with_budgets(
            WorkBudgets::new()
                .with_subdivision_depth(20)
                .with_segments(4096),
        );

    let curve = arc();
    let surface = cylinder();
    let mut refined_curve = curve.clone();
    let mut refined_surface = surface.clone();
    for i in 1..=8 {
        let knot = i as f64 / 9.;
        refined_curve = curve_knot_insert(&refined_curve, knot, 1).unwrap();
        refined_surface = surface_knot_insert_u(&refined_surface, knot, 1).unwrap();
        refined_surface = surface_knot_insert_v(&refined_surface, knot, 1).unwrap();
    }
    let reduced_curve = simplify_curve(&refined_curve, &options, &context).unwrap();
    let reduced_surface = simplify_surface(&refined_surface, &options, &context).unwrap();
    assert_eq!(reduced_curve.removed_knots, 8);
    assert_eq!(reduced_surface.removed_knots, 16);
    assert!(reduced_curve.deviation_bound <= tolerance);
    assert!(reduced_surface.deviation_bound <= tolerance);
    // Qualify output equivalence before any timings, including every parameter
    // that the evaluation benchmarks use. These are independent checks of
    // accepted certified results, not the acceptance mechanism itself.
    for i in 0..128 {
        let t = (i as f64 + 0.5) / 128.;
        assert!(
            (refined_curve.evaluate(t) - reduced_curve.geometry.evaluate(t)).length() <= tolerance
        );
        for j in 0..8 {
            let v = (j as f64 + 0.5) / 8.;
            assert!(
                (refined_surface.evaluate(t, v) - reduced_surface.geometry.evaluate(t, v)).length()
                    <= tolerance
            );
        }
    }

    {
        let mut evaluation = c.benchmark_group("truck_reuse/equivalent_evaluation");
        evaluation.sample_size(10);
        evaluation.warm_up_time(Duration::from_secs(1));
        evaluation.measurement_time(Duration::from_secs(2));
        evaluation.throughput(Throughput::Elements(128));
        for (name, candidate) in [
            ("original", &curve),
            ("refined", &refined_curve),
            ("simplified", &reduced_curve.geometry),
        ] {
            evaluation.bench_with_input(
                BenchmarkId::new(
                    "curve",
                    format!("{name}_{}cp", candidate.control_points().len()),
                ),
                candidate,
                |b, candidate| {
                    b.iter(|| {
                        for i in 0..128 {
                            black_box(candidate.evaluate(black_box((i as f64 + 0.5) / 128.)));
                        }
                    });
                },
            );
        }
        evaluation.throughput(Throughput::Elements(1024));
        for (name, candidate) in [
            ("original", &surface),
            ("refined", &refined_surface),
            ("simplified", &reduced_surface.geometry),
        ] {
            let count = candidate.control_points().len() * candidate.control_points()[0].len();
            evaluation.bench_with_input(
                BenchmarkId::new("surface", format!("{name}_{count}cp")),
                candidate,
                |b, candidate| {
                    b.iter(|| {
                        for i in 0..128 {
                            for j in 0..8 {
                                black_box(candidate.evaluate(
                                    black_box((i as f64 + 0.5) / 128.),
                                    black_box((j as f64 + 0.5) / 8.),
                                ));
                            }
                        }
                    });
                },
            );
        }
        evaluation.finish();
    }

    let mut construction = c.benchmark_group("truck_reuse/qualified_construction");
    construction.sample_size(10);
    construction.warm_up_time(Duration::from_secs(1));
    construction.measurement_time(Duration::from_secs(2));
    construction.bench_function("curve_simplify_11_to_3cp", |b| {
        b.iter(|| {
            black_box(simplify_curve(black_box(&refined_curve), &options, &context).unwrap());
        });
    });
    construction.bench_function("surface_simplify_110_to_6cp", |b| {
        b.iter(|| {
            black_box(simplify_surface(black_box(&refined_surface), &options, &context).unwrap());
        });
    });
    let fitting_options = CubicFitOptions {
        position_tolerance: 1e-4,
        derivative_tolerance: 1e-3,
        max_work: 10_000_000,
        max_depth: 20,
        max_segments: 4096,
    };
    let fitting_context = context
        .clone()
        .with_fallback(FallbackPolicy::AllowApproximate { budget: 1e-4 });
    let fitted = fit_cubic_curve(&curve, &fitting_options, &fitting_context).unwrap();
    assert!(fitted.position_bound <= 1e-4 && fitted.derivative_bound <= 1e-3);
    construction.bench_function("rational_quarter_arc_to_certified_cubics", |b| {
        b.iter(|| {
            black_box(
                fit_cubic_curve(black_box(&curve), &fitting_options, &fitting_context).unwrap(),
            );
        });
    });
    construction.finish();
}

criterion_group!(benches, bench_truck_reuse);
criterion_main!(benches);
