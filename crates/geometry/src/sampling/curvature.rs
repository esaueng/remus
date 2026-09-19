//! Curvature-adaptive curve sampling for NURBS curves.

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::Point3;

/// Maximum recursion depth to prevent infinite subdivision on degenerate curves.
const MAX_DEPTH: u32 = 20;

/// Compute the curvature κ at parameter `t` on a NURBS curve.
///
/// κ = |C' × C''| / |C'|³
///
/// Returns `0.0` when the first derivative is near-zero (degenerate).
fn curvature_at(curve: &NurbsCurve, t: f64) -> f64 {
    let ders = curve.derivatives(t, 2);
    if ders.len() < 3 {
        return 0.0;
    }
    let cp = ders[1]; // C'(t)
    let cpp = ders[2]; // C''(t)
    let cp_len = cp.length();
    if cp_len < f64::EPSILON {
        return 0.0;
    }
    let cross = cp.cross(cpp);
    cross.length() / (cp_len * cp_len * cp_len)
}

/// Chord length between two `Point3` values.
fn chord(a: Point3, b: Point3) -> f64 {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let dz = b.z() - a.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Recursively subdivide `[t_a, t_b]` if the curvature×interval estimate
/// exceeds `tolerance`.
///
/// `p_a` and `p_b` are pre-evaluated curve points at the interval endpoints.
/// New interior points (excluding `p_a`) are appended to `out`.
#[allow(clippy::too_many_arguments)]
fn subdivide(
    curve: &NurbsCurve,
    t_a: f64,
    p_a: Point3,
    t_b: f64,
    p_b: Point3,
    tolerance: f64,
    depth: u32,
    out: &mut Vec<(f64, Point3)>,
) {
    if depth >= MAX_DEPTH {
        return;
    }

    // Sample the midpoint before deciding whether to stop. Endpoint-only
    // curvature and a single chord can both be near zero for a strongly bowed
    // interval, which would incorrectly accept the whole curve as one segment.
    let t_m = 0.5 * (t_a + t_b);
    let p_m = curve.evaluate(t_m);
    let interval_len = chord(p_a, p_m) + chord(p_m, p_b);

    let kappa_a = curvature_at(curve, t_a);
    let kappa_m = curvature_at(curve, t_m);
    let kappa_b = curvature_at(curve, t_b);
    let kappa_max = kappa_a.max(kappa_m).max(kappa_b);

    if kappa_max * interval_len <= tolerance {
        // Angular change is within tolerance — no further subdivision needed.
        return;
    }

    subdivide(curve, t_a, p_a, t_m, p_m, tolerance, depth + 1, out);
    out.push((t_m, p_m));
    subdivide(curve, t_m, p_m, t_b, p_b, tolerance, depth + 1, out);
}

/// Curvature-adaptive sampling for NURBS curves.
///
/// Subdivides intervals where the product of curvature and interval arc-length
/// exceeds `tolerance` (roughly: angular change per segment ≤ tolerance).
///
/// Always returns at least the two endpoints. If `tolerance` is non-positive,
/// only the two endpoints are returned.
#[must_use]
pub fn sample_curvature(
    curve: &NurbsCurve,
    t_start: f64,
    t_end: f64,
    tolerance: f64,
) -> Vec<(f64, Point3)> {
    let p_start = curve.evaluate(t_start);
    let p_end = curve.evaluate(t_end);

    let mut out = Vec::new();
    out.push((t_start, p_start));

    if tolerance > 0.0 {
        subdivide(
            curve, t_start, p_start, t_end, p_end, tolerance, 0, &mut out,
        );
    }

    out.push((t_end, p_end));
    out
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::vec::Point3;

    /// Cubic Bezier with varying curvature: tightly curved near t=0, flatter near t=1.
    /// Control polygon: (0,0,0) → (0.1, 1, 0) → (0.9, 1, 0) → (4, 0, 0)
    fn varying_curvature_bezier() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.1, 1.0, 0.0),
                Point3::new(0.9, 1.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .expect("valid bezier")
    }

    /// Quarter circle as rational NURBS degree 2.
    fn quarter_circle_nurbs() -> NurbsCurve {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        )
        .expect("valid quarter circle")
    }

    #[test]
    fn endpoints_always_included() {
        let c = varying_curvature_bezier();
        let pts = sample_curvature(&c, 0.0, 1.0, 0.1);
        assert!(!pts.is_empty());
        assert!((pts.first().unwrap().0 - 0.0).abs() < 1e-12);
        assert!((pts.last().unwrap().0 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn non_positive_tolerance_returns_two_endpoints() {
        let c = varying_curvature_bezier();
        let pts_zero = sample_curvature(&c, 0.0, 1.0, 0.0);
        assert_eq!(pts_zero.len(), 2);
        let pts_neg = sample_curvature(&c, 0.0, 1.0, -1.0);
        assert_eq!(pts_neg.len(), 2);
    }

    #[test]
    fn recursion_limit_stops_without_appending_points() {
        let curve = varying_curvature_bezier();
        let mut out = Vec::new();
        subdivide(
            &curve,
            0.0,
            curve.evaluate(0.0),
            1.0,
            curve.evaluate(1.0),
            f64::MIN_POSITIVE,
            MAX_DEPTH,
            &mut out,
        );
        assert!(out.is_empty());
    }

    #[test]
    fn parameters_sorted() {
        let c = varying_curvature_bezier();
        let pts = sample_curvature(&c, 0.0, 1.0, 0.05);
        for w in pts.windows(2) {
            assert!(
                w[0].0 < w[1].0,
                "parameters not sorted: {} >= {}",
                w[0].0,
                w[1].0
            );
        }
    }

    #[test]
    fn high_curvature_produces_more_points_than_low() {
        // Tight bezier (control points near each other) → high curvature at interior.
        let tight = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.1, 1.0, 0.0),
                Point3::new(0.1, 0.0, 0.0),
            ],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .expect("valid");

        // Flat bezier (nearly linear).
        let flat = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.01, 0.0),
                Point3::new(2.0, 0.01, 0.0),
                Point3::new(3.0, 0.0, 0.0),
            ],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .expect("valid");

        let tol = 0.1;
        let pts_tight = sample_curvature(&tight, 0.0, 1.0, tol);
        let pts_flat = sample_curvature(&flat, 0.0, 1.0, tol);

        assert!(
            pts_tight.len() > pts_flat.len(),
            "expected more points for tight curve ({}) than flat curve ({})",
            pts_tight.len(),
            pts_flat.len()
        );
    }

    #[test]
    fn quarter_circle_sample_on_unit_circle() {
        // Verify all sampled points lie on the unit circle.
        let c = quarter_circle_nurbs();
        let pts = sample_curvature(&c, 0.0, 1.0, 0.05);
        assert!(pts.len() >= 2);
        for (_, p) in &pts {
            let r = (p.x() * p.x() + p.y() * p.y() + p.z() * p.z()).sqrt();
            assert!((r - 1.0).abs() < 1e-6, "point not on unit circle: r={r:.8}");
        }
    }

    /// Cubic Bezier whose first derivative vanishes at t = 0 (triple start
    /// control point), i.e. the degenerate case `curvature_at` guards against.
    fn cusp_start_bezier() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
            ],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .expect("valid cusp bezier")
    }

    #[test]
    fn curvature_of_unit_radius_arc_is_one() {
        // kappa = |C' x C''| / |C'|^3 is the geometric curvature, so it is
        // independent of the parameterisation: on an arc of radius R it is
        // exactly 1/R. The rational quadratic here is the unit quarter circle,
        // so kappa == 1 at every parameter (|C'| is 1.41 at t=0, so the mutated
        // denominators genuinely differ from the real one).
        let c = quarter_circle_nurbs();
        for t in [0.0, 0.17, 0.4, 0.63, 0.9, 1.0] {
            let k = curvature_at(&c, t);
            assert!(
                (k - 1.0).abs() < 1e-9,
                "curvature at t={t} is {k}, expected 1.0 on a unit-radius arc"
            );
        }
    }

    #[test]
    fn curvature_is_zero_where_the_first_derivative_vanishes() {
        // Contract: a near-zero first derivative returns 0.0 rather than
        // dividing by |C'|^3 == 0 and yielding NaN.
        let c = cusp_start_bezier();
        let k = curvature_at(&c, 0.0);
        assert!(
            k.is_finite(),
            "curvature must stay finite at a degenerate point, got {k}"
        );
        assert!(k.abs() < 1e-12, "expected zero curvature, got {k}");
    }

    #[test]
    fn chord_matches_the_euclidean_closed_form() {
        // (3, 4, 12) is a Pythagorean quadruple: the distance is exactly 13.
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = Point3::new(4.0, 6.0, 15.0);
        let d = chord(a, b);
        assert!((d - 13.0).abs() < 1e-12, "chord is {d}, expected 13");
        let reversed = chord(b, a);
        assert!(
            (reversed - 13.0).abs() < 1e-12,
            "chord is not symmetric: {reversed}"
        );
    }

    #[test]
    fn stop_rule_compares_curvature_times_interval_length() {
        // Unit quarter circle: kappa == 1 everywhere. The two half-chords about
        // t = 0.5 sum to 1.531, which exceeds a tolerance of 1.0, so the
        // interval is split; each half's own chord sum is 0.78, which does not,
        // so neither half is split again. Exactly one interior point.
        // (kappa + len = 2.53 and kappa / len = 0.65 both decide differently.)
        let c = quarter_circle_nurbs();
        let mut out = Vec::new();
        subdivide(
            &c,
            0.0,
            c.evaluate(0.0),
            1.0,
            c.evaluate(1.0),
            1.0,
            MAX_DEPTH - 2,
            &mut out,
        );
        assert_eq!(
            out.len(),
            1,
            "kappa*len = 1.53 > 1.0 splits once, then 1*0.78 <= 1.0 stops; got {} points",
            out.len()
        );
    }

    #[test]
    fn subdivide_increments_depth_on_both_branches() {
        // Entering two levels below MAX_DEPTH, the recursion may split at most
        // twice whatever the tolerance, so at most three interior points can be
        // appended. With a 1e-6 tolerance on a unit arc every allowed split
        // does happen, so the count is exactly three.
        let c = quarter_circle_nurbs();
        let mut out = Vec::new();
        subdivide(
            &c,
            0.0,
            c.evaluate(0.0),
            1.0,
            c.evaluate(1.0),
            1e-6,
            MAX_DEPTH - 2,
            &mut out,
        );
        assert_eq!(
            out.len(),
            3,
            "depth budget allows exactly three interior points, got {}",
            out.len()
        );
    }
}
