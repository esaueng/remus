#![allow(
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used,
    missing_docs
)]

use std::hint::black_box;
use std::time::Duration;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use remus_math::context::{OperationContext, WorkBudgets};
use remus_math::nurbs::bezier_clip::curve_curve_intersect_full;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::intersection::intersect_nurbs_nurbs_with_context;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;

#[derive(Clone, Copy)]
enum CylinderAxis {
    X,
    Z,
}

fn rational_cylinder(axis: CylinderAxis) -> NurbsSurface {
    let radius = 1.0;
    let low = -2.0;
    let high = 2.0;
    let diagonal_weight = std::f64::consts::FRAC_1_SQRT_2;
    let profile = [
        (radius, 0.0, 1.0),
        (radius, radius, diagonal_weight),
        (0.0, radius, 1.0),
        (-radius, radius, diagonal_weight),
        (-radius, 0.0, 1.0),
        (-radius, -radius, diagonal_weight),
        (0.0, -radius, 1.0),
        (radius, -radius, diagonal_weight),
        (radius, 0.0, 1.0),
    ];
    let row = |station: f64| {
        profile
            .iter()
            .map(|&(first, second, _)| match axis {
                CylinderAxis::X => Point3::new(station, first, second),
                CylinderAxis::Z => Point3::new(first, second, station),
            })
            .collect::<Vec<_>>()
    };
    let weights = profile
        .iter()
        .map(|&(_, _, weight)| weight)
        .collect::<Vec<_>>();
    let quarter = std::f64::consts::FRAC_PI_2;
    let half = std::f64::consts::PI;
    let three_quarters = 3.0 * quarter;
    let full = std::f64::consts::TAU;

    NurbsSurface::new(
        1,
        2,
        vec![low, low, high, high],
        vec![
            0.0,
            0.0,
            0.0,
            quarter,
            quarter,
            half,
            half,
            three_quarters,
            three_quarters,
            full,
            full,
            full,
        ],
        vec![row(low), row(high)],
        vec![weights.clone(), weights],
    )
    .expect("benchmark cylinder must be valid")
}

fn saddle_surface() -> NurbsSurface {
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.5, 0.25),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![
                Point3::new(0.5, 0.0, -0.25),
                Point3::new(0.5, 0.5, 0.0),
                Point3::new(0.5, 1.0, 0.25),
            ],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.5, -0.25),
                Point3::new(1.0, 1.0, 0.0),
            ],
        ],
        vec![vec![1.0; 3]; 3],
    )
    .expect("benchmark saddle must be valid")
}

fn tilted_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, -0.25), Point3::new(0.0, 1.0, -0.25)],
            vec![Point3::new(1.0, 0.0, 0.25), Point3::new(1.0, 1.0, 0.25)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .expect("benchmark tilted surface must be valid")
}

fn clipping_curves() -> (NurbsCurve, NurbsCurve) {
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let first = NurbsCurve::new(
        3,
        knots.clone(),
        vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(-0.25, 1.25, 0.0),
            Point3::new(0.25, -1.25, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        vec![1.0; 4],
    )
    .expect("first clipping curve must be valid");
    let second = NurbsCurve::new(
        3,
        knots,
        vec![
            Point3::new(-1.0, 0.8, 0.0),
            Point3::new(-0.25, -1.0, 0.0),
            Point3::new(0.25, 1.0, 0.0),
            Point3::new(1.0, -0.8, 0.0),
        ],
        vec![1.0; 4],
    )
    .expect("second clipping curve must be valid");
    (first, second)
}

fn seeding_context() -> OperationContext {
    OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_segments(0)
            .with_subdivision_depth(5),
    )
}

fn marching_context() -> OperationContext {
    OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_march_steps(80)
            .with_queue_size(24)
            .with_segments(6)
            .with_subdivision_depth(5),
    )
}

fn bench_intersections(c: &mut Criterion) {
    let cylinder_z = rational_cylinder(CylinderAxis::Z);
    let cylinder_x = rational_cylinder(CylinderAxis::X);
    let saddle = saddle_surface();
    let tilted = tilted_surface();
    let seed_only = seeding_context();
    let march = marching_context();

    let quadric_probe =
        intersect_nurbs_nurbs_with_context(&cylinder_z, &cylinder_x, 10, 0.1, &march)
            .expect("quadric benchmark fixture must intersect");
    assert!(
        !quadric_probe.is_empty(),
        "quadric benchmark fixture must produce a curve"
    );
    let nurbs_probe = intersect_nurbs_nurbs_with_context(&saddle, &tilted, 10, 0.05, &march)
        .expect("NURBS benchmark fixture must intersect");
    assert!(
        !nurbs_probe.is_empty(),
        "NURBS benchmark fixture must produce a curve"
    );

    {
        let mut ssi = c.benchmark_group("ssi");
        ssi.throughput(Throughput::Elements(1));
        ssi.bench_function("quadric_seed", |bencher| {
            bencher.iter(|| {
                black_box(
                    intersect_nurbs_nurbs_with_context(
                        black_box(&cylinder_z),
                        black_box(&cylinder_x),
                        10,
                        0.1,
                        black_box(&seed_only),
                    )
                    .expect("quadric seeding must succeed"),
                )
            });
        });
        ssi.bench_function("quadric_march", |bencher| {
            bencher.iter(|| {
                black_box(
                    intersect_nurbs_nurbs_with_context(
                        black_box(&cylinder_z),
                        black_box(&cylinder_x),
                        10,
                        0.1,
                        black_box(&march),
                    )
                    .expect("quadric marching must succeed"),
                )
            });
        });
        ssi.bench_function("nurbs_seed", |bencher| {
            bencher.iter(|| {
                black_box(
                    intersect_nurbs_nurbs_with_context(
                        black_box(&saddle),
                        black_box(&tilted),
                        10,
                        0.05,
                        black_box(&seed_only),
                    )
                    .expect("NURBS seeding must succeed"),
                )
            });
        });
        ssi.bench_function("nurbs_march", |bencher| {
            bencher.iter(|| {
                black_box(
                    intersect_nurbs_nurbs_with_context(
                        black_box(&saddle),
                        black_box(&tilted),
                        10,
                        0.05,
                        black_box(&march),
                    )
                    .expect("NURBS marching must succeed"),
                )
            });
        });
        ssi.finish();
    }

    let (first, second) = clipping_curves();
    c.bench_function("bezier_clip/cubic_pair", |bencher| {
        bencher.iter(|| {
            black_box(
                curve_curve_intersect_full(black_box(&first), black_box(&second), 1e-8)
                    .expect("Bezier clipping must succeed"),
            )
        });
    });
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
    targets = bench_intersections
}
criterion_main!(benches);
