//! `approximate_lspia` returns its best iterate and says whether it
//! converged (kernel review 2026-10-14, MATH-9).
//!
//! LSPIA's least-squares residual falls monotonically, but its maximum
//! deviation need not. The loop returned whatever its last update produced,
//! unmeasured, as `Ok`: on thirty collinear points with one spike it handed
//! back a maximum deviation of 3.613 after sixty updates when 3.560 had been
//! reached after one. It now measures every iterate and returns the best,
//! with its deviation and a `converged` flag.
//!
//! The oracle here uses only the public API: each returned curve is
//! re-measured at the chord-length parameters computed below, and because
//! a larger iteration budget only adds candidates, the returned deviation
//! can never grow with it.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::fitting::approximate_lspia;
use remus_math::vec::Point3;

fn spike() -> Vec<Point3> {
    (0..30)
        .map(|i| Point3::new(i as f64, if i == 15 { 5.0 } else { 0.0 }, 0.0))
        .collect()
}

fn chord_length_params(points: &[Point3]) -> Vec<f64> {
    let mut acc = vec![0.0];
    for w in points.windows(2) {
        acc.push(acc[acc.len() - 1] + (w[1] - w[0]).length());
    }
    let total = acc[acc.len() - 1];
    acc.iter().map(|d| d / total).collect()
}

fn max_deviation(points: &[Point3], curve: &NurbsCurve) -> f64 {
    points
        .iter()
        .zip(chord_length_params(points))
        .map(|(&q, u)| (q - curve.evaluate(u)).length())
        .fold(0.0, f64::max)
}

#[test]
fn deviation_never_grows_with_the_iteration_budget() {
    let pts = spike();
    let mut previous = f64::INFINITY;
    for max_iterations in 0..=60 {
        let fit = approximate_lspia(&pts, 3, 8, 0.0, max_iterations).unwrap();
        assert!(!fit.converged);
        assert!(fit.iteration <= max_iterations);
        let measured = max_deviation(&pts, &fit.curve);
        assert!(
            (measured - fit.max_error).abs() <= 1e-12,
            "reported {} but measured {measured} at budget {max_iterations}",
            fit.max_error
        );
        assert!(
            fit.max_error <= previous,
            "budget {max_iterations} returned {} after {previous}",
            fit.max_error
        );
        previous = fit.max_error;
    }
    // Main returned 3.6126 here; the best iterate is the first update.
    assert!((previous - 3.5603).abs() < 1e-4, "{previous}");
}

#[test]
fn convergence_is_reported() {
    let pts = spike();
    let fit = approximate_lspia(&pts, 3, 8, 3.6, 60).unwrap();
    assert!(fit.converged);
    assert!(fit.max_error < 3.6);
    let fit = approximate_lspia(&pts, 3, 8, 3.5, 60).unwrap();
    assert!(!fit.converged);
    assert!(fit.max_error >= 3.5);
}
