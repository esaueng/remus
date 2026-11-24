//! Certified "fitted copy" test for a NURBS edge against a circle or ellipse
//! arc sharing its endpoints.
//!
//! The duplicate-edge merge may unify a conic edge with a NURBS edge only when
//! the NURBS is a fitted copy of the arc: every point of its trimmed range lies
//! within the fit band of the arc. Sampling cannot show that (a lens narrower
//! than the sample spacing passes), so the bound here is proven span by span:
//!
//! * The trimmed NURBS is cut into rational Bézier spans, and spans are halved
//!   (homogeneous de Casteljau) until each one is decided. Every nonempty
//!   overlap of a span with the trim is bounded, padded outward so rounding
//!   never drops a sliver of it, and each span's weights are normalized to a
//!   largest of 1 before the homogeneous products are formed; a product that
//!   would be subnormal or overflow declines.
//! * The bound runs in units of `max(a, b)`. A conic with
//!   `min(a, b) / max(a, b)` below `MIN_AXIS_RATIO` is declined: its squared
//!   gradient leaves the range f64 can bound.
//! * In the conic's frame `(x, y, z)` with semi-axes `a, b`, the implicit
//!   `F = x²/a² + y²/b² − 1` along a span is `G(t) / W(t)²`, where
//!   `G = X²/a² + Y²/b² − W²` is a polynomial in homogeneous coordinates. Its
//!   Bernstein coefficients bound it, and `W ≥ min wᵢ > 0`, so
//!   `|F| ≤ max|Gₖ| / min wᵢ²` over the whole span.
//! * The branch: in the scaled chart `(x/a, y/b)` the conic's parameter is the
//!   polar angle. Each span's control points must lie in a wedge narrower than
//!   a right angle (so the hull misses the centre and its angles are the
//!   control points' angles), unwrapped continuously from the end the NURBS
//!   starts at, and inside the arc's parameter range widened by
//!   `band / max(a, b)`.
//! * One budget: the in-plane distance to the *trimmed* arc is at most the
//!   distance to a conic point plus the arc length from it back into the
//!   arc (the angle past the arc's ends times `max(a, b)`, the conic's
//!   largest speed). The conic point is either the one at the span point's
//!   own scaled angle, `|p| |s − 1| / s` away with `s² = 1 + F` (for a
//!   circle `r (1 − √(1 − |F|))`), or, for an ellipse, where gradient flow
//!   from the point reaches `F = 0`: within `|F| / g`, where `g` bounds
//!   `|∇F|` from below on the span's box grown by that distance, at an angle
//!   between that grown box's corners.
//! * The out-of-plane offset is at most the control points' largest `|z|`
//!   (positive weights keep the span in its control hull); it adds to the
//!   in-plane distance in quadrature, and the sum must stay within the band
//!   less a rounding allowance relative to the world coordinates.
//!
//! The answer is `true` only when every span is certified within the band. A
//! span shown to leave the band, a non-positive weight, a non-finite or
//! underflowing quantity, or a spent work budget all answer `false`: the
//! caller keeps the edges apart.

use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::decompose::curve_to_bezier_segments;
use remus_math::vec::{Point3, Vec3};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Halvings allowed below one Bézier span.
const MAX_DEPTH: usize = 30;
/// Spans examined (accepted, split or refuted) before giving up.
const MAX_SPANS: usize = 4096;
/// Smallest semi-axis ratio `min(a, b) / max(a, b)` the bound accepts: past
/// it the squared gradient leaves the range f64 can bound.
const MIN_AXIS_RATIO: f64 = 1e-6;
/// Parameter padding on each side of a trimmed Bézier piece: covers the
/// rounding of the trim's map into the span's local parameter, so the piece
/// bounded holds the whole trimmed overlap.
const TRIM_PAD: f64 = 4.0 * f64::EPSILON;

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
    // Work in units of the larger semi-axis; a conic flatter than f64 can
    // bound (the squared gradient's range) is declined.
    let scale = arc.a.max(arc.b);
    if !(arc.a.min(arc.b) >= MIN_AXIS_RATIO * scale && scale.is_finite()) {
        return false;
    }
    if curve
        .weights()
        .iter()
        .any(|w| !(w.is_finite() && *w >= f64::MIN_POSITIVE))
    {
        return false;
    }
    let Some(spans) = trimmed_spans(arc, curve, smin, smax) else {
        return false;
    };
    // Rounding in the frame change, the weight normalization and the
    // halvings: relative to the coordinates' world magnitude.
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

    // The unit frame: semi-axes in (0, 1], band and coordinates divided by
    // `scale`.
    let unit = ConicArc {
        a: arc.a / scale,
        b: arc.b / scale,
        ..*arc
    };
    let band = band / scale;
    let limit = band - slack / scale;
    let mut unit_spans = Vec::with_capacity(spans.len());
    for span in spans {
        let mut scaled = span;
        for h in &mut scaled {
            for c in &mut h[..3] {
                *c /= scale;
                if !c.is_finite() || (*c != 0.0 && c.abs() < f64::MIN_POSITIVE) {
                    return false;
                }
            }
        }
        unit_spans.push(scaled);
    }
    if !(band.is_finite() && limit.is_normal() && limit > 0.0) {
        return false;
    }

    let Some(first) = unit_spans.first().and_then(|s| s.first()) else {
        return false;
    };
    // The NURBS starts at one end of the arc; unwrap its angle from there.
    let start = [
        first[0] / first[3],
        first[1] / first[3],
        first[2] / first[3],
    ];
    let at = |t: f64| {
        let q = [unit.a * t.cos(), unit.b * t.sin(), 0.0];
        ((start[0] - q[0]).powi(2) + (start[1] - q[1]).powi(2) + (start[2] - q[2]).powi(2)).sqrt()
    };
    let (d_lo, d_hi) = (at(unit.lo), at(unit.hi));
    let mut phi = if d_lo <= d_hi { unit.lo } else { unit.hi };
    let start_gap = d_lo.min(d_hi);
    if start_gap.is_nan() || start_gap > band {
        return false;
    }

    // In the unit frame the widening `band / max(a, b)` is the band itself.
    let eta = band;
    let (lo, hi) = (unit.lo - eta, unit.hi + eta);

    let mut examined = 0usize;
    for span in unit_spans {
        let mut stack: Vec<(Span, usize)> = vec![(span, 0)];
        while let Some((span, depth)) = stack.pop() {
            examined += 1;
            if examined > MAX_SPANS {
                return false;
            }
            match judge(&unit, &span, phi, lo, hi, limit) {
                Verdict::Inside(end_phi) => phi = end_phi,
                Verdict::Undecided if depth < MAX_DEPTH => {
                    let (left, right) = split(&span, 0.5);
                    if refuted(&unit, &right[0], phi, lo, hi, band) {
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
    // the widened parameter range. The hull's angles are the control
    // points' angles.
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
    let Some(w_sq) = mul(w_min, w_min) else {
        return Verdict::Undecided;
    };
    let z_max = span.iter().map(|h| (h[2] / h[3]).abs()).fold(0.0, f64::max);
    if !z_max.is_finite() || z_max >= limit {
        return Verdict::Undecided;
    }

    // |F| over the span from G's Bernstein coefficients.
    let Some(g_max) = implicit_bound(arc, span) else {
        return Verdict::Undecided;
    };
    let f_max = g_max / w_sq;
    if !(f_max.is_finite() && f_max < 1.0) {
        return Verdict::Undecided;
    }
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
    if !(xl.is_finite() && xh.is_finite() && yl.is_finite() && yh.is_finite()) {
        return Verdict::Undecided;
    }
    // Arc length past the trimmed arc's ends for an angle range: the
    // conic's speed in its parameter is at most max(a, b) = 1.
    let overshoot = |amin: f64, amax: f64| (arc.lo - amin).max(amax - arc.hi).max(0.0);

    // One budget: a point of the span is within `in_plane` of a point of the
    // trimmed arc (a conic point it reaches, then along the conic back into
    // the arc), and `z_max` off its plane.
    //
    // (a) Radially in the scaled chart: a point p with scaled radius s is
    // `|p| |s − 1| / s` from the conic point at its own parameter, and
    // `s ≥ √(1 − |F|)`, `|s − 1| ≤ |F| / (1 + s)`.
    let smin = (1.0 - f_max).sqrt();
    let radial = if arc.is_circle() {
        arc.a * f_max / (1.0 + smin)
    } else {
        let p_max = xl.abs().max(xh.abs()).hypot(yl.abs().max(yh.abs()));
        p_max * f_max / (smin * (1.0 + smin))
    };
    let mut in_plane = radial + overshoot(phi + dmin, phi + dmax);
    // (b) Ellipses: gradient flow from the point reaches F = 0 within `d`,
    // inside the box grown by `d`, whose corners bound the reached angles.
    if !arc.is_circle()
        && let Some(by_flow) = flow_distance(arc, [xl, xh, yl, yh], f_max, phi)
            .map(|(d, amin, amax)| d + overshoot(amin, amax))
    {
        in_plane = in_plane.min(by_flow);
    }
    let dist = in_plane.hypot(z_max);
    if dist.is_finite() && dist <= limit {
        Verdict::Inside(phi + d_last)
    } else {
        Verdict::Undecided
    }
}

/// The gradient-flow distance bound for an ellipse span with box
/// `[xl, xh, yl, yh]` and `|F| ≤ f_max`, with the unwrapped angle range of
/// the box grown by that distance (where the flow can end). `None` when the
/// bound does not close or the grown box reaches the centre.
fn flow_distance(arc: &ConicArc, bbox: [f64; 4], f_max: f64, phi: f64) -> Option<(f64, f64, f64)> {
    let [xl, xh, yl, yh] = bbox;
    let grad_floor = |grow: f64| {
        let mx = interval_abs_min(xl - grow, xh + grow);
        let my = interval_abs_min(yl - grow, yh + grow);
        2.0 * ((mx / (arc.a * arc.a)).powi(2) + (my / (arc.b * arc.b)).powi(2)).sqrt()
    };
    let g0 = grad_floor(0.0);
    if !(g0.is_finite() && g0 > 0.0) {
        return None;
    }
    let reach = 2.0 * f_max / g0;
    let g = grad_floor(reach);
    if !(reach.is_finite() && g.is_finite() && g > 0.0) {
        return None;
    }
    let d = f_max / g;
    if !(d.is_finite() && d <= reach) {
        return None;
    }
    let (bx0, bx1, by0, by1) = (
        (xl - d) / arc.a,
        (xh + d) / arc.a,
        (yl - d) / arc.b,
        (yh + d) / arc.b,
    );
    if bx0 <= 0.0 && bx1 >= 0.0 && by0 <= 0.0 && by1 >= 0.0 {
        return None;
    }
    // A box clear of the centre spans its angles between its corners.
    let mut amin = f64::INFINITY;
    let mut amax = f64::NEG_INFINITY;
    for (x, y) in [(bx0, by0), (bx1, by0), (bx0, by1), (bx1, by1)] {
        let a = wrap(y.atan2(x) - phi);
        amin = amin.min(a);
        amax = amax.max(a);
    }
    (amax - amin < FRAC_PI_2).then_some((d, phi + amin, phi + amax))
}

/// `a · b`, or `None` when the product is not finite or loses its precision
/// to underflow (subnormal, or zero from nonzero factors).
fn mul(a: f64, b: f64) -> Option<f64> {
    let p = a * b;
    if !p.is_finite() {
        return None;
    }
    if p == 0.0 {
        return (a == 0.0 || b == 0.0).then_some(p);
    }
    (p.abs() >= f64::MIN_POSITIVE).then_some(p)
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
            let xx = mul(mul(hi_[0], ia)?, mul(hj[0], ia)?)?;
            let yy = mul(mul(hi_[1], ib)?, mul(hj[1], ib)?)?;
            let ww = mul(hi_[3], hj[3])?;
            sum += c * (xx + yy - ww);
            abs += c * (xx.abs() + yy.abs() + ww.abs());
        }
        let norm = binom(m, k);
        #[allow(clippy::cast_precision_loss)]
        let rounding = 4.0 * (m + 4) as f64 * f64::EPSILON * abs;
        let bound = (sum.abs() + rounding) / norm;
        // An allowance lost to underflow no longer covers the rounding.
        if !bound.is_finite() || (abs > 0.0 && rounding < f64::MIN_POSITIVE) {
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
        // The overlap is nonempty: bound a piece that holds all of it, never
        // skip it (a sliver's weights can carry the curve anywhere).
        let ta = ((smin.max(u0) - u0) / len - TRIM_PAD).clamp(0.0, 1.0);
        let tb = ((smax.min(u1) - u0) / len + TRIM_PAD).clamp(0.0, 1.0);
        if ta.is_nan() || tb.is_nan() || tb <= ta {
            return None;
        }
        // Weights normalized to a largest of 1, so the homogeneous products
        // below keep their precision.
        let w_max = seg.weights().iter().copied().fold(0.0, f64::max);
        if !(w_max.is_finite() && w_max >= f64::MIN_POSITIVE) {
            return None;
        }
        let mut span: Span = Vec::with_capacity(seg.weights().len());
        for (&p, &w) in seg.control_points().iter().zip(seg.weights()) {
            let w = w / w_max;
            if w.is_nan() || w < f64::MIN_POSITIVE {
                return None;
            }
            let q = arc.local(p);
            span.push([mul(w, q[0])?, mul(w, q[1])?, mul(w, q[2])?, w]);
        }
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

    fn unit_circle() -> Circle3D {
        circle(1.0)
    }

    fn line_nurbs(cps: [Point3; 3], w: f64) -> NurbsCurve {
        NurbsCurve::new(1, vec![0.0, 0.0, 0.5, 1.0, 1.0], cps.to_vec(), vec![w; 3]).unwrap()
    }

    /// Review round 2, case 1 (#1974): the trim reaches `s/2` into the second
    /// span, whose weights carry the curve 0.2 off the plane within that
    /// sliver. A nonempty overlap is bounded, never skipped.
    #[test]
    fn a_sliver_of_a_trimmed_span_is_bounded() {
        let s = 2f64.powi(-51);
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(0.0, 1.0, -1.0),
            ],
            vec![1.0, FRAC_1_SQRT_2, 1.0, 1.0 / s, 2.0 * (1.0 - s) / (s * s)],
        )
        .unwrap();
        let off = curve.evaluate(0.5 + s / 4.0).z().abs();
        assert!(off > 0.1, "the sliver leaves the plane: {off}");
        let arc = ConicArc::circle(&unit_circle(), 0.0, FRAC_PI_2);
        assert!(!nurbs_certified_on_conic(
            &arc,
            &curve,
            0.0,
            0.5 + s / 2.0,
            1e-4
        ));
        // The first span alone is the exact quarter.
        assert!(nurbs_certified_on_conic(&arc, &curve, 0.0, 0.5, 1e-4));
    }

    /// Review round 2, case 2 (#1974): weights of 2.3e-162 make the
    /// homogeneous products subnormal. The polyline bends 0.01 off the circle
    /// and must be judged as it is with unit weights.
    #[test]
    fn subnormal_homogeneous_products_do_not_certify() {
        let arc = ConicArc::circle(&unit_circle(), 0.0, 0.01);
        let cps = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.01, 0.005, 0.0),
            Point3::new(0.01f64.cos(), 0.01f64.sin(), 0.0),
        ];
        for w in [1.0, 2.3e-162, 1e-300, 1e300] {
            let curve = line_nurbs(cps, w);
            assert!(
                !nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4),
                "weights {w:e}"
            );
        }
    }

    /// Review round 2, case 3 (#1974): semi-axes 1 and 1e-155 overflow the
    /// squared gradient. A polyline at least 0.01 off the ellipse must not
    /// certify, at the origin or a million units away.
    #[test]
    fn a_needle_ellipse_does_not_overflow_into_a_certificate() {
        for offset in [0.0, 1e6] {
            let c = Point3::new(offset, 0.0, 0.0);
            let ellipse = Ellipse3D::new_with_ref(
                c,
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
                1e-155,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let (t0, t1) = (std::f64::consts::FRAC_PI_6, FRAC_PI_2);
            let arc = ConicArc::ellipse(&ellipse, t0, t1);
            let curve = line_nurbs(
                [
                    ellipse.evaluate(t0),
                    c + Vec3::new(1.01, 1.1e-155, 0.0),
                    ellipse.evaluate(t1),
                ],
                1.0,
            );
            assert!(
                !nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4),
                "offset {offset:e}"
            );
        }
    }

    /// Review round 2, case 4 (#1974): the middle point is 0.8e-4 behind the
    /// arc's start in angle and 0.8e-4 outside the circle, 1.131e-4 from the
    /// trimmed arc. The angular overshoot and the radial gap share one band.
    #[test]
    fn angular_overshoot_and_radial_gap_share_the_band() {
        let arc = ConicArc::circle(&unit_circle(), 0.0, 0.001);
        let mid = Point3::new(1.00008, -0.00008, 0.0);
        let curve = line_nurbs(
            [
                Point3::new(1.0, 0.0, 0.0),
                mid,
                Point3::new(0.001f64.cos(), 0.001f64.sin(), 0.0),
            ],
            1.0,
        );
        let from_arc = (mid - Point3::new(1.0, 0.0, 0.0)).length();
        assert!(from_arc > 1.13e-4, "{from_arc:e}");
        assert!(!nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4));
        // Inside the arc's range the same radial gap alone is a fitted copy.
        let arc = ConicArc::circle(&unit_circle(), -0.001, 0.001);
        let curve = line_nurbs(
            [
                Point3::new((-0.001f64).cos(), (-0.001f64).sin(), 0.0),
                Point3::new(1.00008, 0.0, 0.0),
                Point3::new(0.001f64.cos(), 0.001f64.sin(), 0.0),
            ],
            1.0,
        );
        assert!(nurbs_certified_on_conic(&arc, &curve, 0.0, 1.0, 1e-4));
    }
}
