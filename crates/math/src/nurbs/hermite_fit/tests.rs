#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;

/// The involute of a circle of radius `rb`, its derivative, and the exact
/// fourth-derivative norm `rb·√(9 + t²)` (see the gear profile module).
fn involute(rb: f64) -> impl Fn(f64) -> (Point3, Vec3) {
    move |t: f64| {
        let (s, c) = t.sin_cos();
        (
            Point3::new(rb * t.mul_add(s, c), rb * (-t).mul_add(c, s), 0.0),
            Vec3::new(rb * t * c, rb * t * s, 0.0),
        )
    }
}

/// Largest parameter-matched error over a dense uniform sampling.
fn sampled_error(fit: &HermiteFit, f: &impl Fn(f64) -> (Point3, Vec3), t0: f64, t1: f64) -> f64 {
    (0..=20_000)
        .map(|i| {
            let t = t0 + (t1 - t0) * f64::from(i) / 20_000.0;
            (fit.curve.evaluate(t) - f(t).0).length()
        })
        .fold(0.0, f64::max)
}

#[test]
fn cubic_polynomial_is_reproduced_with_one_span_and_zero_bound() {
    let f = |t: f64| {
        (
            Point3::new(t * t * t - 2.0 * t, 0.5 * t * t, 3.0),
            Vec3::new(3.0 * t * t - 2.0, t, 0.0),
        )
    };
    let fit = fit_cubic_hermite(f, -1.0, 2.0, 0.0, 1e-9, 8).unwrap();
    assert_eq!(fit.spans, 1);
    assert!(fit.remainder_bound <= 0.0);
    assert!(sampled_error(&fit, &f, -1.0, 2.0) < 1e-13);
}

#[test]
fn involute_error_never_exceeds_the_proven_bound() {
    for &(rb, t1, tol) in &[
        (9.396_926_f64, 0.7_f64, 1e-4),
        (46.984_631, 0.474, 1e-6),
        (0.5, 1.2, 1e-9),
        (500.0, 0.3, 1e-7),
    ] {
        let f = involute(rb);
        let bound = rb * t1.mul_add(t1, 9.0).sqrt();
        let fit = fit_cubic_hermite(&f, 0.0, t1, bound, tol, 4096).unwrap();
        let measured = sampled_error(&fit, &f, 0.0, t1);
        assert!(
            fit.remainder_bound <= tol,
            "bound {} > tol {tol}",
            fit.remainder_bound
        );
        assert!(
            measured <= fit.remainder_bound + 1e-14 * rb,
            "rb {rb}: measured {measured:e} exceeds proven {:e}",
            fit.remainder_bound
        );
        // The bound is not vacuous: the true error is within a few orders of it.
        assert!(
            measured > fit.remainder_bound * 1e-3,
            "rb {rb}: bound far too loose"
        );
    }
}

#[test]
fn fit_interpolates_end_points_and_tangent_directions_exactly() {
    let f = involute(10.0);
    let fit = fit_cubic_hermite(&f, 0.1, 0.6, 10.0 * 9.36_f64.sqrt(), 1e-8, 512).unwrap();
    let points = fit.curve.control_points();
    assert_eq!(points[0], f(0.1).0);
    assert_eq!(points[points.len() - 1], f(0.6).0);
    assert_eq!(fit.curve.domain(), (0.1, 0.6));
    assert_eq!(points.len(), 2 * fit.spans + 2);
    let end_tangent = fit.curve.tangent(0.6).unwrap();
    let expected = f(0.6).1.normalize().unwrap();
    assert!((end_tangent - expected).length() < 1e-12);
}

#[test]
fn interior_junctions_are_c1() {
    let f = involute(3.0);
    let fit = fit_cubic_hermite(&f, 0.0, 1.0, 3.0 * 10.0_f64.sqrt(), 1e-9, 512).unwrap();
    assert!(fit.spans > 2);
    for i in 1..fit.spans {
        #[allow(clippy::cast_precision_loss)]
        let t = i as f64 / fit.spans as f64;
        let before = fit.curve.derivatives(t - 1e-9, 1)[1];
        let after = fit.curve.derivatives(t + 1e-9, 1)[1];
        assert!((before - after).length() < 1e-6, "derivative jump at {t}");
        assert!((fit.curve.evaluate(t) - f(t).0).length() <= fit.remainder_bound + 1e-14);
    }
}

#[test]
fn tighter_tolerance_needs_more_spans() {
    let f = involute(20.0);
    let bound = 20.0 * 9.25_f64.sqrt();
    let coarse = fit_cubic_hermite(&f, 0.0, 0.5, bound, 1e-3, 4096).unwrap();
    let fine = fit_cubic_hermite(&f, 0.0, 0.5, bound, 1e-9, 4096).unwrap();
    assert!(fine.spans > coarse.spans);
    assert!(fine.remainder_bound < coarse.remainder_bound);
}

#[test]
fn span_cap_refuses_instead_of_growing_without_bound() {
    let f = involute(20.0);
    let error = fit_cubic_hermite(&f, 0.0, 0.5, 70.0, 1e-15, 16).unwrap_err();
    assert!(matches!(
        error,
        MathError::ResourceLimitExceeded {
            resource: "hermite_spans",
            limit: 16,
            ..
        }
    ));
}

#[test]
fn invalid_arguments_refuse() {
    let f = involute(1.0);
    assert!(fit_cubic_hermite(&f, 1.0, 1.0, 1.0, 1e-6, 8).is_err());
    assert!(fit_cubic_hermite(&f, 0.0, f64::NAN, 1.0, 1e-6, 8).is_err());
    assert!(fit_cubic_hermite(&f, 0.0, 1.0, -1.0, 1e-6, 8).is_err());
    assert!(fit_cubic_hermite(&f, 0.0, 1.0, 1.0, 0.0, 8).is_err());
    let poisoned = |_t: f64| (Point3::new(f64::NAN, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
    assert!(fit_cubic_hermite(poisoned, 0.0, 1.0, 0.0, 1e-6, 8).is_err());
}
