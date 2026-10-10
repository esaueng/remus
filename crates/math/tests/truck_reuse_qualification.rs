//! Independently generated analytic fixtures for Apache-compatible NURBS reuse.
//!
//! Dense comparisons below are a separate regression oracle. Acceptance must
//! still use the implementation's whole-domain certificates, never these samples.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::float_cmp
)]

use remus_math::context::{CancellationToken, FallbackPolicy, OperationContext, WorkBudgets};
use remus_math::nurbs::cubic_fit::{CubicFitOptions, fit_cubic_curve};
use remus_math::nurbs::knot_ops::{
    curve_knot_insert, surface_knot_insert_u, surface_knot_insert_v,
};
use remus_math::nurbs::reduction::{
    simplify_curve, simplify_surface, surface_knot_remove_u, surface_knot_remove_v,
};
use remus_math::nurbs::reuse::ReductionOptions;
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_math::vec::{Point3, Vec3};

fn context(budget: f64) -> OperationContext {
    OperationContext::new()
        .with_fallback(FallbackPolicy::AllowApproximate { budget })
        .with_budgets(
            WorkBudgets::new()
                .with_subdivision_depth(20)
                .with_segments(4096),
        )
}

fn options(tolerance: f64) -> ReductionOptions {
    ReductionOptions {
        tolerance,
        max_work: 10_000_000,
    }
}

fn cubic_options(position_tolerance: f64, derivative_tolerance: f64) -> CubicFitOptions {
    CubicFitOptions {
        position_tolerance,
        derivative_tolerance,
        max_work: 10_000_000,
        max_depth: 20,
        max_segments: 4096,
    }
}

fn plane() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0., 0., 1., 1.],
        vec![0., 0., 1., 1.],
        vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 2.)],
            vec![Point3::new(1., 0., 3.), Point3::new(1., 1., 5.)],
        ],
        vec![vec![1.; 2]; 2],
    )
    .unwrap()
}

fn paraboloid() -> NurbsSurface {
    let quadratic = [0., 0., 1.];
    let points = (0..3)
        .map(|u| {
            (0..3)
                .map(|v| {
                    Point3::new(
                        u as f64 / 2.,
                        v as f64 / 2.,
                        quadratic[u] + 2. * quadratic[v],
                    )
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(
        2,
        2,
        vec![0., 0., 0., 1., 1., 1.],
        vec![0., 0., 0., 1., 1., 1.],
        points,
        vec![vec![1.; 3]; 3],
    )
    .unwrap()
}

fn quarter_cylinder() -> NurbsSurface {
    let arc = [(1., 0.), (1., 1.), (0., 1.)];
    let weights = [1., std::f64::consts::FRAC_1_SQRT_2, 1.];
    NurbsSurface::new(
        2,
        1,
        vec![0., 0., 0., 1., 1., 1.],
        vec![0., 0., 1., 1.],
        arc.iter()
            .map(|&(x, y)| vec![Point3::new(x, y, 0.), Point3::new(x, y, 3.)])
            .collect(),
        weights.iter().map(|&w| vec![w; 2]).collect(),
    )
    .unwrap()
}

fn analytic_cylinder(u: f64, v: f64) -> Point3 {
    let first = (1. - u).powi(2);
    let middle = std::f64::consts::SQRT_2 * u * (1. - u);
    let last = u * u;
    let denominator = first + middle + last;
    Point3::new(
        (first + middle) / denominator,
        (middle + last) / denominator,
        3. * v,
    )
}

fn refine_surface(surface: &NurbsSurface) -> NurbsSurface {
    let (ua, ub) = surface.domain_u();
    let (va, vb) = surface.domain_v();
    let mut refined = surface.clone();
    for fraction in [0.25, 0.5, 0.75] {
        refined = surface_knot_insert_u(&refined, ua + fraction * (ub - ua), 1).unwrap();
        refined = surface_knot_insert_v(&refined, va + fraction * (vb - va), 1).unwrap();
    }
    refined
}

fn assert_surface_bound(original: &NurbsSurface, result: &NurbsSurface, bound: f64) {
    assert!(bound.is_finite() && bound >= 0.);
    assert_eq!(original.domain_u(), result.domain_u());
    assert_eq!(original.domain_v(), result.domain_v());
    let (ua, ub) = original.domain_u();
    let (va, vb) = original.domain_v();
    let coordinate_scale = original
        .control_points()
        .iter()
        .flatten()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1_f64, f64::max);
    let evaluation_roundoff = 128. * f64::EPSILON * coordinate_scale;
    for i in 0..=43 {
        for j in 0..=37 {
            let u = ua + (ub - ua) * i as f64 / 43.;
            let v = va + (vb - va) * j as f64 / 37.;
            let measured = (original.evaluate(u, v) - result.evaluate(u, v)).length();
            assert!(
                measured <= bound + evaluation_roundoff,
                "surface bound {bound:e} misses {measured:e} at ({u},{v})"
            );
        }
    }
}

fn assert_curve_bound(original: &NurbsCurve, result: &NurbsCurve, bound: f64) {
    assert!(bound.is_finite() && bound >= 0.);
    assert_eq!(original.domain(), result.domain());
    let (a, b) = original.domain();
    let scale = original
        .control_points()
        .iter()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1_f64, f64::max);
    for i in 0..=1009 {
        let t = a + (b - a) * i as f64 / 1009.;
        let measured = (original.evaluate(t) - result.evaluate(t)).length();
        assert!(measured <= bound + 128. * f64::EPSILON * scale);
    }
}

#[test]
fn surface_reduction_retains_three_independent_analytic_geometries() {
    type AnalyticFixture = (NurbsSurface, fn(f64, f64) -> Point3);
    let fixtures: [AnalyticFixture; 3] = [
        (plane(), |u, v| Point3::new(u, v, 3. * u + 2. * v)),
        (paraboloid(), |u, v| Point3::new(u, v, u * u + 2. * v * v)),
        (quarter_cylinder(), analytic_cylinder),
    ];
    for (base, oracle) in fixtures {
        let refined = refine_surface(&base);
        let snapshot = refined.clone();
        let reduced = simplify_surface(&refined, &options(1e-8), &context(1e-8)).unwrap();
        assert_eq!(refined, snapshot);
        assert_eq!(reduced.removed_knots, 6);
        assert!(reduced.deviation_bound <= 1e-8);
        assert!(reduced.work_used > 0 && reduced.work_used <= 10_000_000);
        assert_eq!(
            reduced.geometry.control_points().len(),
            base.control_points().len()
        );
        assert_eq!(
            reduced.geometry.control_points()[0].len(),
            base.control_points()[0].len()
        );
        assert_surface_bound(&refined, &reduced.geometry, reduced.deviation_bound);
        for i in 0..=31 {
            for j in 0..=29 {
                let u = i as f64 / 31.;
                let v = j as f64 / 29.;
                assert!((reduced.geometry.evaluate(u, v) - oracle(u, v)).length() < 1e-8);
            }
        }
    }
}

#[test]
fn both_axes_remove_repeated_interior_knots_one_at_a_time() {
    let base = paraboloid();
    let refined_u = surface_knot_insert_u(&base, 0.375, 2).unwrap();
    let refined = surface_knot_insert_v(&refined_u, 0.625, 2).unwrap();
    let mut result = refined;
    for _ in 0..2 {
        let removed =
            surface_knot_remove_u(&result, 0.375, &options(1e-8), &context(1e-8)).unwrap();
        assert_eq!(removed.removed_knots, 1);
        assert_surface_bound(&result, &removed.geometry, removed.deviation_bound);
        result = removed.geometry;
    }
    for _ in 0..2 {
        let removed =
            surface_knot_remove_v(&result, 0.625, &options(1e-8), &context(1e-8)).unwrap();
        assert_eq!(removed.removed_knots, 1);
        assert_surface_bound(&result, &removed.geometry, removed.deviation_bound);
        result = removed.geometry;
    }
    assert_eq!(result.control_points().len(), 3);
    assert_eq!(result.control_points()[0].len(), 3);
}

fn transform_surface(surface: &NurbsSurface, scale: f64, weight_factor: f64) -> NurbsSurface {
    NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        surface.knots_u().iter().map(|k| -2. + 7. * k).collect(),
        surface.knots_v().iter().map(|k| 3. + 2. * k).collect(),
        surface
            .control_points()
            .iter()
            .map(|row| {
                row.iter()
                    .map(|p| {
                        Point3::new(
                            1e4 + scale * p.x(),
                            -2e4 + scale * p.y(),
                            3e4 + scale * p.z(),
                        )
                    })
                    .collect()
            })
            .collect(),
        surface
            .weights()
            .iter()
            .map(|row| row.iter().map(|w| weight_factor * w).collect())
            .collect(),
    )
    .unwrap()
}

#[test]
fn rational_reduction_preserves_placement_domain_and_common_weight_scaling() {
    for scale in [1e-3, 1., 1e3] {
        for factor in [1e-200, 1., 1e200] {
            // Refinement happens before rescaling to isolate reduction from
            // the older insertion routine's unnormalized homogeneous products.
            let original = transform_surface(&refine_surface(&quarter_cylinder()), scale, factor);
            let reduced = simplify_surface(&original, &options(1e-5), &context(1e-5)).unwrap();
            assert_eq!(reduced.removed_knots, 6);
            assert!(reduced.deviation_bound <= 1e-5);
            assert_surface_bound(&original, &reduced.geometry, reduced.deviation_bound);
        }
    }
}

fn kink_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0., 0., 0.5, 1., 1.],
        vec![0., 0., 1., 1.],
        [0., 1., 0.]
            .iter()
            .enumerate()
            .map(|(i, &z)| {
                vec![
                    Point3::new(i as f64 / 2., 0., z),
                    Point3::new(i as f64 / 2., 1., z),
                ]
            })
            .collect(),
        vec![vec![1.; 2]; 3],
    )
    .unwrap()
}

#[test]
fn nonremovable_knot_refuses_and_leaves_surface_immutable() {
    let original = kink_surface();
    let snapshot = original.clone();
    assert!(surface_knot_remove_u(&original, 0.5, &options(1e-5), &context(1e-5)).is_err());
    assert_eq!(original, snapshot);
    let result = simplify_surface(&original, &options(1e-5), &context(1e-5)).unwrap();
    assert_eq!(result.removed_knots, 0);
    assert_eq!(result.geometry, original);
    assert_eq!(result.deviation_bound, 0.);
}

#[test]
fn surface_failures_are_immutable_for_invalid_budget_policy_and_cancellation() {
    let original = refine_surface(&paraboloid());
    let snapshot = original.clone();
    let cancelled = CancellationToken::new();
    cancelled.cancel();
    let cancelled_context = context(1e-8).with_cancellation(cancelled);
    assert!(simplify_surface(&original, &options(1e-8), &cancelled_context).is_err());
    assert_eq!(original, snapshot);
    let exact = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    // Identity is permitted under ExactOnly; a candidate whose only proof
    // has positive deviation must not be returned as an exact reduction.
    let exact_result = simplify_surface(&original, &options(1e-8), &exact).unwrap();
    assert_eq!(exact_result.removed_knots, 0);
    assert_eq!(exact_result.geometry, original);
    assert_eq!(exact_result.deviation_bound, 0.);
    assert!(surface_knot_remove_u(&original, 0.5, &options(1e-8), &exact).is_err());
    assert_eq!(original, snapshot);
    for tolerance in [f64::NAN, f64::INFINITY, -1.] {
        assert!(simplify_surface(&original, &options(tolerance), &context(1e-8)).is_err());
        assert_eq!(original, snapshot);
    }
    let exhausted = ReductionOptions {
        tolerance: 1e-8,
        max_work: 1,
    };
    assert!(simplify_surface(&original, &exhausted, &context(1e-8)).is_err());
    assert_eq!(original, snapshot);
}

fn rational_arc() -> NurbsCurve {
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

#[test]
fn curve_simplification_counts_all_removals_and_bounds_against_original() {
    let mut original = rational_arc();
    for knot in [0.125, 0.25, 0.5, 0.75, 0.875] {
        original = curve_knot_insert(&original, knot, 1).unwrap();
    }
    let result = simplify_curve(&original, &options(1e-8), &context(1e-8)).unwrap();
    assert_eq!(result.removed_knots, 5);
    assert_eq!(result.geometry.control_points().len(), 3);
    assert!(result.deviation_bound <= 1e-8);
    assert_curve_bound(&original, &result.geometry, result.deviation_bound);
}

#[test]
fn approximate_simplification_uses_one_original_to_final_budget() {
    let base = paraboloid();
    let refined = refine_surface(&base);
    let mut points = refined.control_points().to_vec();
    for (i, row) in points.iter_mut().enumerate() {
        for (j, point) in row.iter_mut().enumerate() {
            *point = Point3::new(
                point.x(),
                point.y(),
                point.z() + ((i + j) % 3) as f64 * 2e-5,
            );
        }
    }
    let original = NurbsSurface::new(
        refined.degree_u(),
        refined.degree_v(),
        refined.knots_u().to_vec(),
        refined.knots_v().to_vec(),
        points,
        refined.weights().to_vec(),
    )
    .unwrap();
    let result = simplify_surface(&original, &options(1e-3), &context(1e-3)).unwrap();
    assert!(
        result.removed_knots >= 2,
        "must exercise an aggregate certificate"
    );
    assert!(result.deviation_bound <= 1e-3);
    assert_surface_bound(&original, &result.geometry, result.deviation_bound);
}

fn hidden_interior_curve() -> NurbsCurve {
    // y(t) = t²(1-t)²(t-1/2)³. Position and first derivative are zero
    // at both endpoints and the midpoint, so endpoint-Hermite interpolation
    // and a midpoint-only position/derivative check both miss its interior.
    let ys = [0., 0., -1. / 168., 3. / 280., -3. / 280., 1. / 168., 0., 0.];
    NurbsCurve::new(
        7,
        [vec![0.; 8], vec![1.; 8]].concat(),
        ys.iter()
            .enumerate()
            .map(|(i, &y)| Point3::new(i as f64 / 7., y, 0.))
            .collect(),
        vec![1.; 8],
    )
    .unwrap()
}

fn hidden_interior_value(t: f64) -> Point3 {
    Point3::new(t, t.powi(2) * (1. - t).powi(2) * (t - 0.5).powi(3), 0.)
}

fn hidden_interior_derivative(t: f64) -> Vec3 {
    Vec3::new(
        1.,
        2. * t * (1. - t) * (1. - 2. * t) * (t - 0.5).powi(3)
            + 3. * t.powi(2) * (1. - t).powi(2) * (t - 0.5).powi(2),
        0.,
    )
}

#[test]
fn cubic_fitting_detects_interior_error_invisible_at_endpoints_and_midpoint() {
    let original = hidden_interior_curve();
    let options = cubic_options(1e-6, 1e-5);
    for t in [0., 0.5, 1.] {
        assert!(original.evaluate(t).y().abs() < 1e-17);
        assert!(original.derivatives(t, 1)[1].y().abs() < 1e-16);
    }
    let result = fit_cubic_curve(&original, &options, &context(1e-6)).unwrap();
    assert!(result.approximate);
    assert!(
        result.segments > 1,
        "a whole-domain check must split this curve"
    );
    assert!(result.position_bound <= options.position_tolerance);
    assert!(result.derivative_bound <= options.derivative_tolerance);
    assert!(result.work_used > 0 && result.work_used <= options.max_work);
    for i in 0..=1009 {
        let t = i as f64 / 1009.;
        let position_error = (result.curve.evaluate(t) - hidden_interior_value(t)).length();
        let derivative_error =
            (result.curve.derivatives(t, 1)[1] - hidden_interior_derivative(t)).length();
        assert!(
            position_error <= result.position_bound + 1e-13,
            "position {t}: {position_error:e}"
        );
        assert!(
            derivative_error <= result.derivative_bound + 1e-12,
            "derivative {t}: {derivative_error:e}"
        );
    }
}

#[test]
fn cubic_fit_retains_parameter_domain_endpoints_and_endpoint_tangents() {
    let source = rational_arc();
    let source = NurbsCurve::new(
        source.degree(),
        source.knots().iter().map(|t| -4. + 10. * t).collect(),
        source.control_points().to_vec(),
        source.weights().to_vec(),
    )
    .unwrap();
    let options = cubic_options(1e-4, 1e-3);
    let result = fit_cubic_curve(&source, &options, &context(1e-4)).unwrap();
    assert_eq!(source.domain(), result.curve.domain());
    assert_curve_bound(&source, &result.curve, result.position_bound);
    for t in [-4., 6.] {
        assert!((source.evaluate(t) - result.curve.evaluate(t)).length() < 1e-12);
        assert!((source.derivatives(t, 1)[1] - result.curve.derivatives(t, 1)[1]).length() < 1e-12);
    }
    for i in 0..=1009 {
        let t = -4. + 10. * i as f64 / 1009.;
        let derivative_error =
            (source.derivatives(t, 1)[1] - result.curve.derivatives(t, 1)[1]).length();
        assert!(derivative_error <= result.derivative_bound + 1e-12);
    }
}

#[test]
fn cubic_fitting_refuses_forbidden_approximation_and_exhausted_work_without_mutation() {
    let source = hidden_interior_curve();
    let snapshot = source.clone();
    let options = cubic_options(1e-8, 1e-8);
    let cancelled = CancellationToken::new();
    cancelled.cancel();
    for ctx in [
        OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
        context(1e-8).with_cancellation(cancelled),
    ] {
        assert!(fit_cubic_curve(&source, &options, &ctx).is_err());
        assert_eq!(source, snapshot);
    }
    for limits in [
        CubicFitOptions {
            max_work: 1,
            ..options
        },
        CubicFitOptions {
            max_depth: 0,
            ..options
        },
        CubicFitOptions {
            max_segments: 1,
            ..options
        },
    ] {
        assert!(fit_cubic_curve(&source, &limits, &context(1e-8)).is_err());
        assert_eq!(source, snapshot);
    }
}
