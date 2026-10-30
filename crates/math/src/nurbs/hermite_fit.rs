//! Certified piecewise cubic Hermite fitting of a parametric curve.
//!
//! [`fit_cubic_hermite`] interpolates a curve `f(t)` and its derivative at
//! uniformly spaced parameters and returns the result as one C¹ cubic
//! B-spline with double interior knots. The B-spline is parameterized by the
//! same `t` as `f`, so its distance from `f` at every parameter is bounded by
//! the classical Hermite remainder:
//!
//! ```text
//! |e_i(t)| = |f_i''''(ξ)| · (t − a)²(t − b)² / 4!  ≤  h⁴/384 · sup|f_i''''|
//! ```
//!
//! per component on a span `[a, b]` of width `h`. With a caller-supplied
//! bound `M ≥ sup ‖f''''‖` over the whole domain, every component obeys
//! `|f_i''''| ≤ M`, so the Euclidean error is at most `√3 · h⁴/384 · M`.
//! That is a proof in exact arithmetic, not a sampled estimate: the caller
//! is responsible for `M` being a true bound (a closed-form fourth
//! derivative is the intended source). Floating-point evaluation of the
//! control points adds error of order `ε · ‖f‖`, which the bound does not
//! include.
//!
//! The parameter-matched error bounds the geometric (closest-point)
//! deviation from above, so the bound also certifies the fitted curve's
//! Hausdorff distance to `f` over the domain.

use crate::MathError;
use crate::nurbs::NurbsCurve;
use crate::vec::{Point3, Vec3};

/// A certified cubic Hermite fit (see the [module docs](self)).
#[derive(Debug, Clone)]
pub struct HermiteFit {
    /// The fitted C¹ cubic B-spline, parameterized like the source curve.
    pub curve: NurbsCurve,
    /// Proven upper bound on `‖curve(t) − f(t)‖` over the whole domain, in
    /// exact arithmetic.
    pub remainder_bound: f64,
    /// Number of uniform Hermite spans.
    pub spans: usize,
}

/// Fit `f` over `[t0, t1]` with uniform cubic Hermite spans until the
/// remainder bound is at most `tolerance`.
///
/// `sample(t)` returns the point `f(t)` and the derivative `f'(t)`.
/// `fourth_derivative_bound` must bound `‖f''''(t)‖` over `[t0, t1]`; a
/// zero bound (a polynomial of degree at most three) yields one span and a
/// zero remainder.
///
/// # Errors
///
/// - [`MathError::ParameterOutOfRange`] when the domain is empty or
///   non-finite, the bound is negative or non-finite, or the tolerance is
///   not positive and finite;
/// - [`MathError::ResourceLimitExceeded`] when meeting `tolerance` needs
///   more than `max_spans` spans;
/// - [`MathError::InvalidControlPointValue`] when `sample` returns a
///   non-finite value.
pub fn fit_cubic_hermite(
    sample: impl Fn(f64) -> (Point3, Vec3),
    t0: f64,
    t1: f64,
    fourth_derivative_bound: f64,
    tolerance: f64,
    max_spans: usize,
) -> Result<HermiteFit, MathError> {
    if !t0.is_finite() || !t1.is_finite() || t1 <= t0 {
        return Err(MathError::ParameterOutOfRange {
            value: t1,
            min: t0,
            max: f64::INFINITY,
        });
    }
    if !fourth_derivative_bound.is_finite() || fourth_derivative_bound < 0.0 {
        return Err(MathError::ParameterOutOfRange {
            value: fourth_derivative_bound,
            min: 0.0,
            max: f64::INFINITY,
        });
    }
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return Err(MathError::ParameterOutOfRange {
            value: tolerance,
            min: f64::MIN_POSITIVE,
            max: f64::INFINITY,
        });
    }

    let width = t1 - t0;
    let spans = span_count(width, fourth_derivative_bound, tolerance, max_spans)?;
    #[allow(clippy::cast_precision_loss)]
    let h = width / spans as f64;
    let remainder_bound = remainder_bound(h, fourth_derivative_bound);

    #[allow(clippy::cast_precision_loss)]
    let knot = |i: usize| {
        if i == spans {
            t1
        } else {
            t0 + width * (i as f64 / spans as f64)
        }
    };
    let mut control_points = Vec::with_capacity(2 * spans + 2);
    let mut knots = Vec::with_capacity(2 * spans + 6);
    knots.extend([t0; 4]);
    let (mut point, mut derivative) = sample(t0);
    control_points.push(point);
    for i in 0..spans {
        let (a, b) = (knot(i), knot(i + 1));
        let third = (b - a) / 3.0;
        let (next_point, next_derivative) = sample(b);
        control_points.push(point + derivative * third);
        control_points.push(next_point - next_derivative * third);
        if i + 1 < spans {
            knots.extend([b; 2]);
        }
        point = next_point;
        derivative = next_derivative;
    }
    control_points.push(point);
    knots.extend([t1; 4]);

    let weights = vec![1.0; control_points.len()];
    let curve = NurbsCurve::new(3, knots, control_points, weights)?;
    Ok(HermiteFit {
        curve,
        remainder_bound,
        spans,
    })
}

/// `√3 · h⁴/384 · M`: the Euclidean form of the per-component remainder.
fn remainder_bound(h: f64, fourth_derivative_bound: f64) -> f64 {
    3.0_f64.sqrt() * h.powi(4) / 384.0 * fourth_derivative_bound
}

/// Smallest uniform span count whose remainder bound meets `tolerance`.
fn span_count(
    width: f64,
    fourth_derivative_bound: f64,
    tolerance: f64,
    max_spans: usize,
) -> Result<usize, MathError> {
    if fourth_derivative_bound <= 0.0 {
        return Ok(1);
    }
    let h_max = (384.0 * tolerance / (3.0_f64.sqrt() * fourth_derivative_bound)).powf(0.25);
    let estimate = (width / h_max).ceil();
    let limit_error = || MathError::ResourceLimitExceeded {
        resource: "hermite_spans",
        limit: max_spans,
        actual: usize::MAX,
    };
    #[allow(clippy::cast_precision_loss)]
    if !estimate.is_finite() || estimate > max_spans as f64 {
        return Err(limit_error());
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let first = (estimate as usize).max(1);
    // `powf` and the division round; step up until the bound itself holds.
    #[allow(clippy::cast_precision_loss)]
    (first..=max_spans)
        .find(|&spans| remainder_bound(width / spans as f64, fourth_derivative_bound) <= tolerance)
        .ok_or_else(limit_error)
}

#[cfg(test)]
mod tests;
