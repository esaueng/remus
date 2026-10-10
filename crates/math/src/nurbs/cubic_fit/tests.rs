//! Independent analytic oracles; no Truck tests or fixtures are copied.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use super::*;
use crate::context::{FallbackPolicy, WorkBudgets};

fn generous_context() -> OperationContext {
    OperationContext::new()
        .with_budgets(
            WorkBudgets::new()
                .with_subdivision_depth(12)
                .with_segments(1024),
        )
        .with_fallback(FallbackPolicy::AllowApproximate { budget: 1.0 })
}

fn bezier(degree: usize, points: Vec<Point3>, domain: (f64, f64)) -> NurbsCurve {
    let mut knots = vec![domain.0; degree + 1];
    knots.extend(std::iter::repeat_n(domain.1, degree + 1));
    let weights = vec![1.0; points.len()];
    NurbsCurve::new(degree, knots, points, weights).expect("valid independent oracle")
}

fn assert_point(actual: Point3, expected: Point3, tolerance: f64) {
    assert!(
        (actual - expected).length() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance}"
    );
}

fn assert_vector(actual: Vec3, expected: Vec3, tolerance: f64) {
    assert!(
        (actual - expected).length() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance}"
    );
}

#[test]
fn affine_line_retains_original_parameter_and_analytic_derivative() {
    let curve = bezier(
        1,
        vec![Point3::new(-3.0, 6.0, 5.0), Point3::new(9.0, -12.0, 5.0)],
        (-2.0, 4.0),
    );
    let before = curve.clone();
    let options = CubicFitOptions::new(1e-10, 1e-10);
    let fit = fit_cubic_curve(&curve, &options, &generous_context()).expect("certified line");
    assert_eq!(curve, before);
    assert_eq!(fit.curve.domain(), (-2.0, 4.0));
    assert_eq!(fit.curve.degree(), 3);
    assert_eq!(fit.segments, 1);
    assert!(fit.approximate);
    assert!(fit.position_bound <= options.position_tolerance);
    assert!(fit.derivative_bound <= options.derivative_tolerance);
    assert!(fit.work_used <= options.max_work);
    for t in [-2.0, -1.5, 0.0, 0.25, 1.0, 3.5, 4.0] {
        assert_point(
            fit.curve.evaluate(t),
            Point3::new(1.0 + 2.0 * t, -3.0 * t, 5.0),
            2e-13,
        );
        assert_vector(
            fit.curve.derivatives(t, 1)[1],
            Vec3::new(2.0, -3.0, 0.0),
            2e-13,
        );
    }
}

#[test]
fn parabola_matches_closed_form_position_and_derivative() {
    let curve = bezier(
        2,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        (0.0, 1.0),
    );
    let options = CubicFitOptions::new(1e-10, 1e-10);
    let fit = fit_cubic_curve(&curve, &options, &generous_context()).expect("certified parabola");
    assert_eq!(fit.segments, 1);
    for t in [0.0, 0.01, 0.125, 0.33, 0.5, 0.79, 1.0] {
        assert_point(fit.curve.evaluate(t), Point3::new(t, t * t, 0.0), 2e-14);
        assert_vector(
            fit.curve.derivatives(t, 1)[1],
            Vec3::new(1.0, 2.0 * t, 0.0),
            2e-14,
        );
    }
}

#[test]
fn cubic_matches_closed_form_and_endpoint_tangents() {
    let curve = bezier(
        3,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0 / 3.0, 0.0, 0.0),
            Point3::new(2.0 / 3.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        (0.0, 1.0),
    );
    let fit = fit_cubic_curve(
        &curve,
        &CubicFitOptions::new(1e-10, 1e-10),
        &generous_context(),
    )
    .expect("certified cubic");
    assert_eq!(fit.segments, 1);
    for t in [0.0, 0.1, 0.3, 0.5, 0.9, 1.0] {
        assert_point(fit.curve.evaluate(t), Point3::new(t, t * t * t, 0.0), 2e-14);
        assert_vector(
            fit.curve.derivatives(t, 1)[1],
            Vec3::new(1.0, 3.0 * t * t, 0.0),
            2e-14,
        );
    }
}

#[test]
fn smooth_multispan_parabola_is_certified_after_piece_concatenation() {
    // The independently derived B-spline controls for C(t)=(t,t²,0),
    // with the quadratic blossom evaluated at knots 0, 1/2, and 1.
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.25, 0.0, 0.0),
            Point3::new(0.75, 0.5, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        vec![1.0; 4],
    )
    .expect("analytic two-span parabola");
    let options = CubicFitOptions::new(1e-9, 1e-9);
    let fit =
        fit_cubic_curve(&curve, &options, &generous_context()).expect("certified two-span fit");
    assert_eq!(fit.segments, 2);
    assert_eq!(fit.curve.control_points().len(), 7);
    assert_eq!(
        fit.curve.knots(),
        &[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]
    );
    for t in [0.0, 0.1, 0.25, 0.49, 0.5, 0.51, 0.75, 0.9, 1.0] {
        assert_point(fit.curve.evaluate(t), Point3::new(t, t * t, 0.0), 2e-14);
        assert_vector(
            fit.curve.derivatives(t, 1)[1],
            Vec3::new(1.0, 2.0 * t, 0.0),
            2e-14,
        );
    }
}

#[test]
fn narrow_nonzero_span_preserves_domain_and_derivative_units() {
    let width = 2.0_f64.powi(-40);
    let curve = bezier(
        2,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        (1.0, 1.0 + width),
    );
    let options = CubicFitOptions::new(1e-8, 1e4);
    let fit = fit_cubic_curve(&curve, &options, &generous_context()).expect("narrow span fit");
    assert_eq!(fit.curve.domain(), curve.domain());
    for fraction in [0.0, 0.125, 0.5, 0.875, 1.0] {
        let t = 1.0 + width * fraction;
        assert_point(
            fit.curve.evaluate(t),
            Point3::new(fraction, fraction * fraction, 0.0),
            1e-13,
        );
        assert_vector(
            fit.curve.derivatives(t, 1)[1],
            Vec3::new(1.0 / width, 2.0 * fraction / width, 0.0),
            0.01,
        );
    }
}

#[test]
fn derivative_tolerance_can_force_subdivision_independently_of_position() {
    // Quartic C(t)=(t,t^4,0), with an independently written Bernstein net.
    let curve = bezier(
        4,
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.25, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(0.75, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        (0.0, 1.0),
    );
    let fit = fit_cubic_curve(
        &curve,
        &CubicFitOptions::new(0.5, 1e-3),
        &generous_context(),
    )
    .expect("derivative-qualified adaptive fit");
    assert!(fit.segments > 1);
    assert!(fit.derivative_bound <= 1e-3);
    for t in [0.0, 0.13, 0.3, 0.51, 0.7, 0.93, 1.0] {
        assert_vector(
            fit.curve.derivatives(t, 1)[1],
            Vec3::new(1.0, 4.0 * t * t * t, 0.0),
            fit.derivative_bound,
        );
    }
}

#[test]
fn nondifferentiable_interior_is_refused_without_mutation() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        vec![1.0; 3],
    )
    .expect("valid corner");
    let before = curve.clone();
    assert!(matches!(
        fit_cubic_curve(
            &curve,
            &CubicFitOptions::new(0.01, 0.01),
            &generous_context()
        ),
        Err(ReuseError::Unsupported { .. })
    ));
    assert_eq!(curve, before);
}

#[test]
fn nonfinite_and_nonpositive_tolerances_are_typed_errors() {
    let curve = bezier(
        1,
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        (0.0, 1.0),
    );
    for (position, derivative) in [
        (0.0, 1.0),
        (-1.0, 1.0),
        (f64::INFINITY, 1.0),
        (f64::NAN, 1.0),
        (1.0, 0.0),
        (1.0, -1.0),
        (1.0, f64::INFINITY),
        (1.0, f64::NAN),
    ] {
        assert!(matches!(
            fit_cubic_curve(
                &curve,
                &CubicFitOptions::new(position, derivative),
                &generous_context()
            ),
            Err(ReuseError::InvalidOptions { .. })
        ));
    }
}

#[test]
fn underflowing_weight_normalization_is_refused_before_endpoint_evaluation() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        vec![f64::from_bits(1), f64::MAX],
    )
    .expect("finite positive-weight source");
    let before = curve.clone();
    assert!(matches!(
        fit_cubic_curve(
            &curve,
            &CubicFitOptions::new(0.001, 0.001),
            &generous_context()
        ),
        Err(ReuseError::BoundUnavailable { .. })
    ));
    assert_eq!(curve, before);
}

#[test]
fn smallest_positive_span_is_refused_before_basis_overflow_can_panic() {
    let width = f64::from_bits(1);
    let curve = bezier(
        1,
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        (0.0, width),
    );
    let before = curve.clone();
    assert!(matches!(
        fit_cubic_curve(
            &curve,
            &CubicFitOptions::new(0.001, 0.001),
            &generous_context()
        ),
        Err(ReuseError::BoundUnavailable { .. })
    ));
    assert_eq!(curve, before);
}

#[test]
fn endpoint_quotient_overflow_is_a_typed_refusal() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(f64::MAX, 0.0, 0.0)],
        vec![1.0, 2.0],
    )
    .expect("finite endpoint data but unrepresentable start derivative");
    assert!(matches!(
        fit_cubic_curve(
            &curve,
            &CubicFitOptions::new(0.001, 0.001),
            &generous_context()
        ),
        Err(ReuseError::BoundUnavailable { .. })
    ));
}
