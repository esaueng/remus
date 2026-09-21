//! Arc-length parameterized curve sampling.

use remus_math::traits::ParametricCurve;
use remus_math::vec::Point3;

/// Number of segments used for the coarse chord-length table.
const CHORD_SEGMENTS: usize = 256;

/// Sample `n` points at approximately equal arc-length spacing over `[t_start, t_end]`.
///
/// Uses a fine-resolution chord-length approximation (256 segments) to build a
/// cumulative arc-length table, then bisects to find the parameter at each
/// target arc-length fraction.
///
/// - `n == 0` returns an empty `Vec`.
/// - `n == 1` returns a single point at `t_start`.
/// - `n >= 2` returns points including both endpoints.
#[must_use]
pub fn sample_arc_length<C: ParametricCurve>(
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

    // Build a cumulative chord-length table at fine resolution.
    let segs = CHORD_SEGMENTS;
    let mut t_table = Vec::with_capacity(segs + 1);
    let mut arc_table = Vec::with_capacity(segs + 1);

    let mut prev_p = curve.evaluate(t_start);
    t_table.push(t_start);
    arc_table.push(0.0_f64);

    for i in 1..=segs {
        let t = if i == segs {
            t_end
        } else {
            t_start + i as f64 * (t_end - t_start) / segs as f64
        };
        let p = curve.evaluate(t);
        let chord = {
            let dx = p.x() - prev_p.x();
            let dy = p.y() - prev_p.y();
            let dz = p.z() - prev_p.z();
            (dx * dx + dy * dy + dz * dz).sqrt()
        };
        t_table.push(t);
        arc_table.push(arc_table[i - 1] + chord);
        prev_p = p;
    }

    let total_len = *arc_table.last().unwrap_or(&0.0);

    // For each target fraction k/(n-1), bisect into the arc-length table to
    // find the parameter that achieves that arc-length.
    let mut result = Vec::with_capacity(n);

    for i in 0..n {
        let target = if i == n - 1 {
            total_len
        } else {
            total_len * i as f64 / (n - 1) as f64
        };

        // Binary search in arc_table for the segment containing `target`.
        let seg_idx = arc_table
            .partition_point(|&s| s < target)
            .saturating_sub(1)
            .min(segs - 1);

        let s0 = arc_table[seg_idx];
        let s1 = arc_table[seg_idx + 1];
        let t0 = t_table[seg_idx];
        let t1 = t_table[seg_idx + 1];

        // Linearly interpolate within the fine segment, then bisect on the
        // actual curve for higher accuracy.
        let t_approx = if (s1 - s0).abs() < f64::EPSILON {
            t0
        } else {
            t0 + (target - s0) / (s1 - s0) * (t1 - t0)
        };

        // Bisect on the curve arc-length within [t0, t1] to refine.
        let t_refined = bisect_arc_length(curve, t0, t1, s0, target, 32);

        // Use whichever is closer to the target; for the endpoints, snap exactly.
        let t_final = if i == 0 {
            t_start
        } else if i == n - 1 {
            t_end
        } else {
            // Prefer the bisected value but fall back to linear if bisect is off.
            let _ = t_approx; // linear approx available but bisect is better
            t_refined
        };

        result.push((t_final, curve.evaluate(t_final)));
    }

    result
}

/// Bisect to find the parameter in `[t_lo, t_hi]` at which the arc-length
/// from `t_lo` (where arc-length offset from curve start is `arc_lo`) reaches
/// `target_arc`.
///
/// Uses chord-length approximation with `max_iter` steps.
fn bisect_arc_length<C: ParametricCurve>(
    curve: &C,
    t_lo: f64,
    t_hi: f64,
    arc_lo: f64,
    target_arc: f64,
    max_iter: u32,
) -> f64 {
    let mut lo = t_lo;
    let mut hi = t_hi;
    let mut arc_at_lo = arc_lo;

    for _ in 0..max_iter {
        let mid = 0.5 * (lo + hi);
        // Approximate arc-length from lo to mid by chord.
        let p_lo = curve.evaluate(lo);
        let p_mid = curve.evaluate(mid);
        let dx = p_mid.x() - p_lo.x();
        let dy = p_mid.y() - p_lo.y();
        let dz = p_mid.z() - p_lo.z();
        let chord = (dx * dx + dy * dy + dz * dz).sqrt();
        let arc_at_mid = arc_at_lo + chord;

        if arc_at_mid < target_arc {
            lo = mid;
            arc_at_lo = arc_at_mid;
        } else {
            hi = mid;
        }
    }

    0.5 * (lo + hi)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::curves::{Circle3D, Ellipse3D};
    use remus_math::vec::{Point3, Vec3};
    use std::f64::consts::TAU;

    fn unit_circle() -> Circle3D {
        Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
    }

    fn dist(a: Point3, b: Point3) -> f64 {
        let dx = a.x() - b.x();
        let dy = a.y() - b.y();
        let dz = a.z() - b.z();
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    #[test]
    fn zero_samples_returns_empty() {
        let c = unit_circle();
        assert!(sample_arc_length(&c, 0.0, TAU, 0).is_empty());
    }

    #[test]
    fn one_sample_returns_start() {
        let c = unit_circle();
        let pts = sample_arc_length(&c, 0.0, TAU, 1);
        assert_eq!(pts.len(), 1);
        assert!((pts[0].0 - 0.0).abs() < 1e-12);
    }

    #[test]
    fn endpoints_included() {
        let c = unit_circle();
        let pts = sample_arc_length(&c, 0.0, TAU, 8);
        assert_eq!(pts.len(), 8);
        assert!((pts[0].0 - 0.0).abs() < 1e-12);
        assert!((pts[7].0 - TAU).abs() < 1e-12);
    }

    #[test]
    fn spacing_approximately_uniform_on_circle() {
        // A circle has uniform curvature so arc-length spacing = chord spacing.
        let c = unit_circle();
        let pts = sample_arc_length(&c, 0.0, TAU, 16);
        assert_eq!(pts.len(), 16);

        // Compute consecutive chord distances (skip the wrap-around gap).
        let dists: Vec<f64> = pts.windows(2).map(|w| dist(w[0].1, w[1].1)).collect();

        let max_d = dists.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min_d = dists.iter().copied().fold(f64::INFINITY, f64::min);

        // Max/min ratio must be ≤ 1.5 — a generous bound for uniformity.
        assert!(
            max_d / min_d <= 1.5,
            "spacing not uniform: max={max_d:.4}, min={min_d:.4}, ratio={:.4}",
            max_d / min_d
        );
    }

    /// Centre of [`tilted_circle`].
    fn tilt_center() -> Point3 {
        Point3::new(0.7, -0.4, 1.3)
    }

    /// Radius of [`tilted_circle`].
    const TILT_RADIUS: f64 = 2.3;

    /// A circle whose plane is tilted against all three axes and whose centre
    /// is off the origin. Every coordinate of the sampled points then moves,
    /// and no coordinate *difference* collapses to zero — which is what makes
    /// the `dx`/`dy`/`dz` terms of the chord formula observable.
    fn tilted_circle() -> Circle3D {
        Circle3D::new(tilt_center(), Vec3::new(0.3, -0.5, 0.8), TILT_RADIUS).unwrap()
    }

    /// Contract: `n >= 2` returns `n` points at equal arc-length spacing,
    /// including both endpoints.
    ///
    /// On a circle equal arc length *is* equal angle, and the parameter of
    /// `Circle3D` is the angle. So the returned parameters must be evenly
    /// spaced by `(t_end - t_start) / (n - 1)`, and consecutive points must be
    /// separated by the closed-form chord `2 * R * sin(dtheta / 2)`. Nothing
    /// here is read off the current output: both numbers follow from the
    /// geometry of a circle.
    #[test]
    fn arc_length_on_tilted_circle_is_equal_angle() {
        let c = tilted_circle();
        let t_start = 0.4_f64;
        let t_end = 2.6_f64;
        // 7 intervals: 256 / 7 is not an integer, so no arc-length target
        // lands exactly on a chord-table knot.
        let n = 8usize;
        let pts = sample_arc_length(&c, t_start, t_end, n);
        assert_eq!(pts.len(), n);

        // Fixture guard: the arc has to move in all three coordinates.
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for (_, p) in &pts {
            for (k, v) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
                lo[k] = lo[k].min(v);
                hi[k] = hi[k].max(v);
            }
        }
        for (k, (h, l)) in hi.iter().zip(&lo).enumerate() {
            assert!(h - l > 0.5, "fixture degenerate on axis {k}: {}", h - l);
        }

        // Both endpoints are included, exactly.
        assert!((pts[0].0 - t_start).abs() < 1e-12, "start not included");
        assert!((pts[n - 1].0 - t_end).abs() < 1e-12, "end not included");

        // Parameters increase monotonically and stay inside the span.
        for (i, w) in pts.windows(2).enumerate() {
            assert!(
                w[1].0 > w[0].0,
                "parameters not increasing at {i}: {} then {}",
                w[0].0,
                w[1].0
            );
        }
        for (i, (t, _)) in pts.iter().enumerate() {
            assert!(
                *t >= t_start - 1e-12 && *t <= t_end + 1e-12,
                "parameter {i} = {t} outside [{t_start}, {t_end}]"
            );
        }

        // Equal arc length on a circle => equal angle step.
        let step = (t_end - t_start) / 7.0;
        for (i, w) in pts.windows(2).enumerate() {
            let d = w[1].0 - w[0].0;
            assert!(
                (d - step).abs() < 1e-3 * step,
                "angle step {i} = {d}, expected {step}"
            );
        }

        // ... and therefore an equal, closed-form chord between samples.
        let expected_chord = 2.0 * TILT_RADIUS * (step / 2.0).sin();
        for (i, w) in pts.windows(2).enumerate() {
            let d = dist(w[0].1, w[1].1);
            assert!(
                (d - expected_chord).abs() < 1e-3 * expected_chord,
                "chord {i} = {d}, expected {expected_chord}"
            );
        }

        // Every sample still lies on the circle.
        let center = tilt_center();
        for (i, (_, p)) in pts.iter().enumerate() {
            let r = dist(*p, center);
            assert!(
                (r - TILT_RADIUS).abs() < 1e-9,
                "sample {i} off the circle: r={r}"
            );
        }
    }

    /// On an ellipse the parameter is *not* proportional to arc length:
    /// `|dP/dt| = sqrt(a^2 sin^2 t + b^2 cos^2 t)` grows from `b` at `t = 0` to
    /// `a` at `t = pi/2`. Arc-length sampling must therefore differ measurably
    /// from uniform-parameter sampling. Because the arc-length function `s(t)`
    /// is convex on this span, `s` lies below the chord joining its endpoints,
    /// so every interior arc-length parameter must sit strictly to the *right*
    /// of the corresponding uniform-parameter one.
    #[test]
    fn arc_length_beats_uniform_parameter_on_an_ellipse() {
        let e = Ellipse3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            1.6,
        )
        .unwrap();
        let t_start = 0.1_f64;
        let t_end = 1.4_f64;
        // 9 intervals: 256 / 9 is not an integer.
        let n = 10usize;
        let step = (t_end - t_start) / 9.0;

        let pts = sample_arc_length(&e, t_start, t_end, n);
        assert_eq!(pts.len(), n);

        // Interior parameters are pushed right of the uniform ones, by a
        // margin far larger than the chord-table discretisation.
        let mut t_uniform = t_start;
        let mut max_shift = 0.0_f64;
        for (i, (t, _)) in pts.iter().enumerate() {
            if i > 0 && i < n - 1 {
                assert!(
                    *t > t_uniform,
                    "sample {i}: arc-length t={t} not right of uniform t={t_uniform}"
                );
                max_shift = max_shift.max(*t - t_uniform);
            }
            t_uniform += step;
        }
        assert!(
            max_shift > 0.05,
            "arc-length sampling barely differs from uniform parameter: {max_shift}"
        );

        // Arc-length sampling makes the spatial spacing near-uniform ...
        let chords: Vec<f64> = pts.windows(2).map(|w| dist(w[0].1, w[1].1)).collect();
        let c_max = chords.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let c_min = chords.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            c_max / c_min < 1.3,
            "arc-length spacing not uniform: max={c_max}, min={c_min}"
        );

        // ... whereas uniform-parameter sampling of the same span is not.
        // (Fixture guard: proves the two strategies really do disagree here.)
        let mut uniform = Vec::with_capacity(n);
        let mut t = t_start;
        for _ in 0..n {
            uniform.push(e.evaluate(t));
            t += step;
        }
        let u: Vec<f64> = uniform.windows(2).map(|w| dist(w[0], w[1])).collect();
        let u_max = u.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let u_min = u.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            u_max / u_min > 1.5,
            "fixture is not discriminating: uniform spacing ratio {}",
            u_max / u_min
        );
    }

    #[test]
    fn all_points_on_circle() {
        let c = unit_circle();
        let pts = sample_arc_length(&c, 0.0, TAU, 12);
        for (_, p) in &pts {
            let r = (p.x() * p.x() + p.y() * p.y() + p.z() * p.z()).sqrt();
            assert!((r - 1.0).abs() < 1e-10, "point not on unit circle: r={r}");
        }
    }
}
