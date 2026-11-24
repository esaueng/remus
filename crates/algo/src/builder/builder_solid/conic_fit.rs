//! Certified "fitted copy" test for a NURBS edge against a circle or ellipse
//! arc sharing its endpoints.
//!
//! The duplicate-edge merge may unify a conic edge with a NURBS edge only when
//! the NURBS is a fitted copy of the arc: every point of its trimmed range lies
//! within the fit band of the arc. Sampling cannot show that (a lens narrower
//! than the sample spacing passes), so the bound here is proven span by span:
//!
//! * The trimmed NURBS is cut into rational Bézier spans, and spans are halved
//!   (homogeneous de Casteljau) until each one is decided.
//! * In the conic's frame `(x, y, z)` with semi-axes `a, b`, the implicit
//!   `F = x²/a² + y²/b² − 1` along a span is `G(t) / W(t)²`, where
//!   `G = X²/a² + Y²/b² − W²` is a polynomial in homogeneous coordinates. Its
//!   Bernstein coefficients bound it, and `W ≥ min wᵢ > 0`, so
//!   `|F| ≤ max|Gₖ| / min wᵢ²` over the whole span.
//! * For a circle `|ρ − r| ≤ r (1 − √(1 − |F|))` exactly. For an ellipse the in-plane
//!   distance to the curve is at most `|F| / g`, where `g` bounds `|∇F|` from
//!   below on the span's box grown by that distance (gradient flow from the
//!   point reaches `F = 0` within that length).
//! * The out-of-plane offset is at most the control points' largest `|z|`
//!   (positive weights keep the span in its control hull).
//! * The branch: in the scaled chart `(x/a, y/b)` the conic's parameter is the
//!   polar angle. Each span's control points must lie in a wedge narrower than
//!   a right angle (so the hull misses the centre and its angles are the
//!   control points' angles), unwrapped continuously from the end the NURBS
//!   starts at, and inside the arc's parameter range widened by
//!   `band / max(a, b)` (at most `band` of arc length).
//!
//! The answer is `true` only when every span is certified within the band. A
//! span shown to leave the band, a non-positive weight, or a spent work budget
//! all answer `false`: the caller keeps the edges apart.

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::decompose::curve_to_bezier_segments;
use remus_math::vec::{Point3, Vec3};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Halvings allowed below one Bézier span.
const MAX_DEPTH: usize = 30;
/// Spans examined (accepted, split or refuted) before giving up.
const MAX_SPANS: usize = 4096;

/// A conic arc in its own frame.
#[derive(Clone, Copy)]
pub(super) struct ConicArc {
    center: Point3,
    u: Vec3,
    v: Vec3,
    n: Vec3,
    a: f64,
    b: f64,
    /// Arc parameter range, `lo < hi`.
    lo: f64,
    hi: f64,
}

impl ConicArc {
    pub(super) fn circle(circle: &Circle3D, t0: f64, t1: f64) -> Self {
        Self {
            center: circle.center(),
            u: circle.u_axis(),
            v: circle.v_axis(),
            n: circle.normal(),
            a: circle.radius(),
            b: circle.radius(),
            lo: t0.min(t1),
            hi: t0.max(t1),
        }
    }

    pub(super) fn ellipse(ellipse: &Ellipse3D, t0: f64, t1: f64) -> Self {
        Self {
            center: ellipse.center(),
            u: ellipse.u_axis(),
            v: ellipse.v_axis(),
            n: ellipse.normal(),
            a: ellipse.semi_major(),
            b: ellipse.semi_minor(),
            lo: t0.min(t1),
            hi: t0.max(t1),
        }
    }

    fn eval(&self, t: f64) -> Point3 {
        self.center + self.u * (self.a * t.cos()) + self.v * (self.b * t.sin())
    }

    fn local(&self, p: Point3) -> [f64; 3] {
        let d = p - self.center;
        [d.dot(self.u), d.dot(self.v), d.dot(self.n)]
    }

    // Exact: a circle frame is built with `a` and `b` from one radius.
    #[allow(clippy::float_cmp)]
    fn is_circle(&self) -> bool {
        self.a == self.b
    }
}

/// A homogeneous rational Bézier span in the conic frame: `[wx, wy, wz, w]`.
type Span = Vec<[f64; 4]>;

/// Whether the NURBS `curve` over `[s0, s1]` lies within `band` of `arc`, on
/// the arc's branch. See the module documentation for the bound.
pub(super) fn nurbs_certified_on_conic(
    arc: &ConicArc,
    curve: &NurbsCurve,
    s0: f64,
    s1: f64,
    band: f64,
) -> bool {
    let (smin, smax) = (s0.min(s1), s0.max(s1));
    if !(arc.a.is_finite()
        && arc.b.is_finite()
        && arc.a > 0.0
        && arc.b > 0.0
        && arc.lo.is_finite()
        && arc.hi.is_finite()
        && arc.hi > arc.lo
        && arc.hi - arc.lo <= TAU + 1e-9
        && smin.is_finite()
        && smax > smin
        && band > 0.0)
    {
        return false;
    }
    if curve.weights().iter().any(|w| !(w.is_finite() && *w > 0.0)) {
        return false;
    }
    let Some(spans) = trimmed_spans(arc, curve, smin, smax) else {
        return false;
    };
    let Some(first) = spans.first().and_then(|s| s.first()) else {
        return false;
    };
    // The NURBS starts at one end of the arc; unwrap its angle from there.
    let start = Point3::new(
        first[0] / first[3],
        first[1] / first[3],
        first[2] / first[3],
    );
    let at = |t: f64| {
        let q = arc.local(arc.eval(t));
        ((start.x() - q[0]).powi(2) + (start.y() - q[1]).powi(2) + (start.z() - q[2]).powi(2))
            .sqrt()
    };
    let (d_lo, d_hi) = (at(arc.lo), at(arc.hi));
    let mut phi = if d_lo <= d_hi { arc.lo } else { arc.hi };
    let start_gap = d_lo.min(d_hi);
    if start_gap.is_nan() || start_gap > band {
        return false;
    }

    let scale = arc.a.max(arc.b);
    let eta = band / scale;
    let (lo, hi) = (arc.lo - eta, arc.hi + eta);
    // Rounding in the frame change and the halvings: relative to the
    // coordinates' magnitude.
    let magnitude = scale
        + Vec3::new(arc.center.x(), arc.center.y(), arc.center.z()).length()
        + spans
            .iter()
            .flatten()
            .map(|h| (h[0].abs() + h[1].abs() + h[2].abs()) / h[3])
            .fold(0.0, f64::max);
    let slack = 1e3 * f64::EPSILON * magnitude;
    if !(slack.is_finite() && slack < 0.5 * band) {
        return false;
    }
    let limit = band - slack;

    let mut examined = 0usize;
    for span in spans {
        let mut stack: Vec<(Span, usize)> = vec![(span, 0)];
        while let Some((span, depth)) = stack.pop() {
            examined += 1;
            if examined > MAX_SPANS {
                return false;
            }
            match judge(arc, &span, phi, lo, hi, limit) {
                Verdict::Inside(end_phi) => phi = end_phi,
                Verdict::Undecided if depth < MAX_DEPTH => {
                    let (left, right) = split(&span, 0.5);
                    if refuted(arc, &right[0], phi, lo, hi, band) {
                        return false;
                    }
                    stack.push((right, depth + 1));
                    stack.push((left, depth + 1));
                }
                Verdict::Undecided => return false,
            }
        }
    }
    true
}

enum Verdict {
    /// Certified within the band; carries the unwrapped angle at the span end.
    Inside(f64),
    Undecided,
}

fn judge(arc: &ConicArc, span: &[[f64; 4]], phi: f64, lo: f64, hi: f64, limit: f64) -> Verdict {
    // Branch: control points in a wedge narrower than a right angle, inside
    // the widened parameter range.
    let mut dmin = f64::INFINITY;
    let mut dmax = f64::NEG_INFINITY;
    let mut d_last = 0.0;
    for h in span {
        let (sx, sy) = (h[0] / arc.a, h[1] / arc.b);
        if !(sx.is_finite() && sy.is_finite()) || (sx == 0.0 && sy == 0.0) {
            return Verdict::Undecided;
        }
        let d = wrap(sy.atan2(sx) - phi);
        dmin = dmin.min(d);
        dmax = dmax.max(d);
        d_last = d;
    }
    if !(dmax - dmin < FRAC_PI_2 && phi + dmin >= lo && phi + dmax <= hi) {
        return Verdict::Undecided;
    }

    // Out of plane.
    let w_min = span.iter().map(|h| h[3]).fold(f64::INFINITY, f64::min);
    if !(w_min.is_finite() && w_min > 0.0) {
        return Verdict::Undecided;
    }
    let z_max = span.iter().map(|h| (h[2] / h[3]).abs()).fold(0.0, f64::max);
    if z_max.is_nan() || z_max >= limit {
        return Verdict::Undecided;
    }

    // |F| over the span from G's Bernstein coefficients.
    let Some(g_max) = implicit_bound(arc, span) else {
        return Verdict::Undecided;
    };
    let f_max = g_max / (w_min * w_min);
    let in_plane = if arc.is_circle() {
        // |ρ − r| = r²|F| / (ρ + r) with ρ ≥ r − |ρ − r|, so
        // |ρ − r| ≤ r (1 − √(1 − |F|)).
        if f_max.is_nan() || f_max >= 1.0 {
            return Verdict::Undecided;
        }
        arc.a * f_max / (1.0 + (1.0 - f_max).sqrt())
    } else {
        let mut xl = f64::INFINITY;
        let mut xh = f64::NEG_INFINITY;
        let mut yl = f64::INFINITY;
        let mut yh = f64::NEG_INFINITY;
        for h in span {
            let (x, y) = (h[0] / h[3], h[1] / h[3]);
            xl = xl.min(x);
            xh = xh.max(x);
            yl = yl.min(y);
            yh = yh.max(y);
        }
        let grad_floor = |grow: f64| {
            let mx = interval_abs_min(xl - grow, xh + grow);
            let my = interval_abs_min(yl - grow, yh + grow);
            2.0 * ((mx / (arc.a * arc.a)).powi(2) + (my / (arc.b * arc.b)).powi(2)).sqrt()
        };
        let g0 = grad_floor(0.0);
        if g0.is_nan() || g0 <= 0.0 {
            return Verdict::Undecided;
        }
        let reach = 2.0 * f_max / g0;
        let g = grad_floor(reach);
        let d = f_max / g;
        if !(g > 0.0 && d <= reach) {
            return Verdict::Undecided;
        }
        d
    };
    let dist = in_plane.hypot(z_max);
    if dist.is_finite() && dist <= limit {
        Verdict::Inside(phi + d_last)
    } else {
        Verdict::Undecided
    }
}

/// A rigorous refutation at an on-curve point `h` (a span end): the point is
/// farther than `band` from the conic, or its angle has left the widened
/// range by more than the wedge test could reconcile.
fn refuted(arc: &ConicArc, h: &[f64; 4], phi: f64, lo: f64, hi: f64, band: f64) -> bool {
    let (x, y, z) = (h[0] / h[3], h[1] / h[3], h[2] / h[3]);
    if !(x.is_finite() && y.is_finite() && z.is_finite()) {
        return true;
    }
    if z.abs() > band {
        return true;
    }
    // Moving `band` changes F by at most band·max|∇F| on the band ball.
    let f = (x / arc.a).powi(2) + (y / arc.b).powi(2) - 1.0;
    let grad_cap = 2.0
        * (((x.abs() + band) / (arc.a * arc.a)).powi(2)
            + ((y.abs() + band) / (arc.b * arc.b)).powi(2))
        .sqrt();
    if f.abs() > 2.0 * band * grad_cap {
        return true;
    }
    // An on-curve angle a quarter turn outside the arc cannot be unwrapped
    // back into it by a later wedge.
    // Only for arcs shorter than a half turn: every on-arc angle is then
    // within π of `phi`, so the wrapped offset is the true one.
    if hi - lo >= PI {
        return false;
    }
    let d = wrap((y / arc.b).atan2(x / arc.a) - phi);
    let theta = phi + d;
    theta < lo - FRAC_PI_2 * 0.5 || theta > hi + FRAC_PI_2 * 0.5
}

/// Upper bound of `|X²/a² + Y²/b² − W²|` over a span, from its Bernstein
/// coefficients, with a rounding allowance.
fn implicit_bound(arc: &ConicArc, span: &[[f64; 4]]) -> Option<f64> {
    let p = span.len().checked_sub(1)?;
    let m = 2 * p;
    let binom = |n: usize, k: usize| -> f64 {
        let mut c = 1.0;
        for i in 0..k {
            #[allow(clippy::cast_precision_loss)]
            {
                c = c * (n - i) as f64 / (i + 1) as f64;
            }
        }
        c
    };
    let (ia, ib) = (1.0 / arc.a, 1.0 / arc.b);
    let mut worst: f64 = 0.0;
    for k in 0..=m {
        let mut sum = 0.0;
        let mut abs = 0.0;
        let i_lo = k.saturating_sub(p);
        for i in i_lo..=k.min(p) {
            let j = k - i;
            let c = binom(p, i) * binom(p, j);
            let (hi_, hj) = (span[i], span[j]);
            let xx = hi_[0] * ia * hj[0] * ia;
            let yy = hi_[1] * ib * hj[1] * ib;
            let ww = hi_[3] * hj[3];
            sum += c * (xx + yy - ww);
            abs += c * (xx.abs() + yy.abs() + ww.abs());
        }
        let norm = binom(m, k);
        #[allow(clippy::cast_precision_loss)]
        let rounding = 4.0 * (m + 4) as f64 * f64::EPSILON * abs;
        let bound = (sum.abs() + rounding) / norm;
        if !bound.is_finite() {
            return None;
        }
        worst = worst.max(bound);
    }
    Some(worst)
}

fn interval_abs_min(lo: f64, hi: f64) -> f64 {
    if lo <= 0.0 && hi >= 0.0 {
        0.0
    } else {
        lo.abs().min(hi.abs())
    }
}

/// `d` wrapped to `(-π, π]`.
fn wrap(d: f64) -> f64 {
    let w = d - TAU * (d / TAU).round();
    if w <= -PI { w + TAU } else { w }
}

/// Homogeneous de Casteljau split at local parameter `t`.
fn split(span: &[[f64; 4]], t: f64) -> (Span, Span) {
    let n = span.len();
    let mut work: Span = span.to_vec();
    let mut left = Vec::with_capacity(n);
    let mut right = Vec::with_capacity(n);
    left.push(work[0]);
    right.push(work[n - 1]);
    for level in 1..n {
        for i in 0..n - level {
            for c in 0..4 {
                work[i][c] = (1.0 - t) * work[i][c] + t * work[i + 1][c];
            }
        }
        left.push(work[0]);
        right.push(work[n - 1 - level]);
    }
    right.reverse();
    (left, right)
}

/// The rational Bézier spans of `curve` restricted to `[smin, smax]`, in
/// parameter order, as homogeneous points in the conic frame.
fn trimmed_spans(arc: &ConicArc, curve: &NurbsCurve, smin: f64, smax: f64) -> Option<Vec<Span>> {
    let (d0, d1) = curve.domain();
    if smin < d0 - 1e-12 || smax > d1 + 1e-12 {
        return None;
    }
    let segments = curve_to_bezier_segments(curve).ok()?;
    let mut spans = Vec::new();
    for seg in &segments {
        let (u0, u1) = seg.domain();
        let len = u1 - u0;
        if len.is_nan() || len <= 0.0 || u1 <= smin || u0 >= smax {
            continue;
        }
        let ta = ((smin.max(u0) - u0) / len).clamp(0.0, 1.0);
        let tb = ((smax.min(u1) - u0) / len).clamp(0.0, 1.0);
        if tb - ta <= 1e-15 {
            continue;
        }
        let mut span: Span = seg
            .control_points()
            .iter()
            .zip(seg.weights())
            .map(|(&p, &w)| {
                let q = arc.local(p);
                [w * q[0], w * q[1], w * q[2], w]
            })
            .collect();
        if tb < 1.0 {
            span = split(&span, tb).0;
        }
        if ta > 0.0 {
            span = split(&span, ta / tb).1;
        }
        spans.push(span);
    }
    (!spans.is_empty()).then_some(spans)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;

    fn quarter_circle_nurbs(r: f64) -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(r, 0.0, 0.0),
                Point3::new(r, r, 0.0),
                Point3::new(0.0, r, 0.0),
            ],
            vec![1.0, FRAC_1_SQRT_2, 1.0],
        )
        .unwrap()
    }

    fn circle(r: f64) -> Circle3D {
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            r,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn exact_rational_quarter_certifies_either_direction() {
        let arc = ConicArc::circle(&circle(5.0), 0.0, FRAC_PI_2);
        let curve = quarter_circle_nurbs(5.0);
        assert!(nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4));
        let reversed = curve.reversed();
        let (a, b) = reversed.domain();
        assert!(nurbs_certified_on_conic(&arc, &reversed, a, b, 1e-4));
    }

    #[test]
    fn complementary_arc_is_not_certified() {
        // The quarter from π/2 to π lies on the circle but off the arc.
        let arc = ConicArc::circle(&circle(5.0), 0.0, FRAC_PI_2);
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 5.0, 0.0),
                Point3::new(-5.0, 5.0, 0.0),
                Point3::new(-5.0, 0.0, 0.0),
            ],
            vec![1.0, FRAC_1_SQRT_2, 1.0],
        )
        .unwrap();
        assert!(!nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4));
    }

    #[test]
    fn out_of_plane_offset_is_charged() {
        let arc = ConicArc::circle(&circle(5.0), 0.0, FRAC_PI_2);
        let mut cps = quarter_circle_nurbs(5.0).control_points().to_vec();
        cps[1] = Point3::new(5.0, 5.0, 4e-4);
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            cps,
            vec![1.0, FRAC_1_SQRT_2, 1.0],
        )
        .unwrap();
        // At t = 0.5 the offset is 0.5·w₁·z₁ / (0.5 + 0.5·w₁) ≈ 1.66e-4.
        assert!(!nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4));
    }

    #[test]
    fn split_reproduces_the_span() {
        let arc = ConicArc::circle(&circle(5.0), 0.0, FRAC_PI_2);
        let curve = quarter_circle_nurbs(5.0);
        let spans = trimmed_spans(&arc, &curve, 0.25, 0.75).unwrap();
        assert_eq!(spans.len(), 1);
        let h0 = spans[0][0];
        let p = curve.evaluate(0.25);
        assert!((h0[0] / h0[3] - p.x()).abs() < 1e-12);
        assert!((h0[1] / h0[3] - p.y()).abs() < 1e-12);
        let h2 = spans[0][2];
        let q = curve.evaluate(0.75);
        assert!((h2[0] / h2[3] - q.x()).abs() < 1e-12);
        assert!((h2[1] / h2[3] - q.y()).abs() < 1e-12);
    }
}
