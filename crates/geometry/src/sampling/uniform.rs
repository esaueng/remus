//! Uniform parameter-space curve sampling.

use remus_math::traits::ParametricCurve;
use remus_math::vec::Point3;

/// Sample `n` evenly-spaced points in parameter space over `[t_start, t_end]`.
///
/// - `n == 0` returns an empty `Vec`.
/// - `n == 1` returns a single point at `t_start`.
/// - `n >= 2` returns points including both endpoints.
#[must_use]
pub fn sample_uniform<C: ParametricCurve>(
    curve: &C,
    t_start: f64,
    t_end: f64,
    n: usize,
) -> Vec<Point3> {
    sample_uniform_with_params(curve, t_start, t_end, n)
        .into_iter()
        .map(|(_, p)| p)
        .collect()
}

/// Sample `n` evenly-spaced `(t, Point3)` pairs over `[t_start, t_end]`.
///
/// - `n == 0` returns an empty `Vec`.
/// - `n == 1` returns `vec![(t_start, curve(t_start))]`.
/// - `n >= 2` returns pairs including both endpoints.
#[must_use]
pub fn sample_uniform_with_params<C: ParametricCurve>(
    curve: &C,
    t_start: f64,
    t_end: f64,
    n: usize,
) -> Vec<(f64, Point3)> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(t_start, curve.evaluate(t_start))];
    }
    let step = (t_end - t_start) / (n - 1) as f64;
    (0..n)
        .map(|i| {
            let t = if i == n - 1 {
                t_end // avoid floating-point overshoot on last point
            } else {
                t_start + i as f64 * step
            };
            (t, curve.evaluate(t))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::vec::{Point3, Vec3};
    use std::f64::consts::TAU;

    fn unit_circle() -> Circle3D {
        Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
    }

    #[test]
    fn zero_samples_returns_empty() {
        let c = unit_circle();
        let pts = sample_uniform(&c, 0.0, TAU, 0);
        assert!(pts.is_empty());
    }

    #[test]
    fn one_sample_returns_start() {
        let c = unit_circle();
        let pts = sample_uniform(&c, 0.0, TAU, 1);
        assert_eq!(pts.len(), 1);
        // t=0 must lie on the unit circle (radius == 1).
        let r =
            (pts[0].x() * pts[0].x() + pts[0].y() * pts[0].y() + pts[0].z() * pts[0].z()).sqrt();
        assert!((r - 1.0).abs() < 1e-12, "point not on unit circle: r={r}");
    }

    #[test]
    fn four_samples_on_unit_circle() {
        let c = unit_circle();
        let pairs = sample_uniform_with_params(&c, 0.0, TAU, 4);
        assert_eq!(pairs.len(), 4);

        // All points should lie on the unit circle.
        for (_, p) in &pairs {
            let r = (p.x() * p.x() + p.y() * p.y() + p.z() * p.z()).sqrt();
            assert!((r - 1.0).abs() < 1e-12, "point not on unit circle: r={r}");
        }

        // First and last parameter values must be endpoints.
        assert!((pairs[0].0 - 0.0).abs() < 1e-12);
        assert!((pairs[3].0 - TAU).abs() < 1e-12);

        // First point (t=0) and last point (t=TAU) must coincide (full circle).
        let p0 = pairs[0].1;
        let p3 = pairs[3].1;
        let dist =
            ((p0.x() - p3.x()).powi(2) + (p0.y() - p3.y()).powi(2) + (p0.z() - p3.z()).powi(2))
                .sqrt();
        assert!(
            dist < 1e-12,
            "endpoints should coincide on full circle: dist={dist}"
        );
    }

    #[test]
    fn params_cover_full_range() {
        let c = unit_circle();
        let pairs = sample_uniform_with_params(&c, 0.0, TAU, 5);
        assert_eq!(pairs.len(), 5);
        assert!((pairs[0].0 - 0.0).abs() < 1e-12);
        assert!((pairs[4].0 - TAU).abs() < 1e-12);
    }

    #[test]
    fn params_are_evenly_spaced_over_an_asymmetric_range() {
        // t in [1.1, 4.7] with n = 7: step = 3.6 / 6 = 0.6, so the documented
        // evenly-spaced parameters are 1.1, 1.7, 2.3, 2.9, 3.5, 4.1, 4.7.
        let c = unit_circle();
        let expected = [1.1, 1.7, 2.3, 2.9, 3.5, 4.1, 4.7];
        let pairs = sample_uniform_with_params(&c, 1.1, 4.7, 7);
        assert_eq!(pairs.len(), expected.len());
        for (i, (t, p)) in pairs.iter().enumerate() {
            assert!(
                (t - expected[i]).abs() < 1e-12,
                "parameter {i} is {t}, expected {}",
                expected[i]
            );
            // The point must be the curve evaluated at that same parameter.
            let q = c.evaluate(expected[i]);
            let d = ((p.x() - q.x()).powi(2) + (p.y() - q.y()).powi(2) + (p.z() - q.z()).powi(2))
                .sqrt();
            assert!(
                d < 1e-12,
                "point {i} is not curve({}): dist={d}",
                expected[i]
            );
        }
    }

    #[test]
    fn last_param_is_snapped_exactly_to_t_end() {
        // The doc comment promises the final parameter is t_end itself, not the
        // accumulated t_start + (n-1)*step. For this range the two differ:
        // 1.1 + 6*((4.7 - 1.1)/6) is one ulp below 4.7.
        let t_start = 1.1_f64;
        let t_end = 4.7_f64;
        let n = 7;
        let step = (t_end - t_start) / 6.0;
        assert_ne!(
            (t_start + 6.0 * step).to_bits(),
            t_end.to_bits(),
            "fixture is degenerate: the unsnapped value already equals t_end"
        );

        let c = unit_circle();
        let pairs = sample_uniform_with_params(&c, t_start, t_end, n);
        assert_eq!(
            pairs.last().unwrap().0.to_bits(),
            t_end.to_bits(),
            "final parameter must be snapped to t_end, got {}",
            pairs.last().unwrap().0
        );
    }
}
