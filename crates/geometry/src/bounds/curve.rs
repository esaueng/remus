//! Conservative bounds over finite curve spans.
//!
//! Every function returns a [`FiniteBound`]: either a
//! certified conservative box (usable for pruning) or an explicitly unknown
//! fallback that never prunes. See [`super`] for the proof sketches.
//!
//! All spans use set semantics: a reversed trim `(t0, t1)` with `t0 > t1`
//! bounds the same point set as the ascending span, because traversal
//! direction does not change which points the edge covers.

use std::f64::consts::TAU;

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::Point3;

use super::{FiniteBound, box_of_points};

/// Roundoff allowance for recognizing an anchored full turn.
///
/// Full-turn trims are recorded anchored at a seam vertex (e.g.
/// `[2.8, 2.8 + 2π]`), and the subtraction can sit one ULP above `TAU`.
/// Mirrors the allowance in `remus-topology`'s periodic domain validation.
const FULL_TURN_ROUNDOFF: f64 = 4.0 * f64::EPSILON * TAU;

/// Conservative box of a line segment.
#[must_use]
pub fn line_segment_bounds(p0: Point3, p1: Point3) -> FiniteBound {
    match box_of_points(&[p0, p1]) {
        Some(aabb) => FiniteBound::conservative(aabb),
        None => FiniteBound::infinite_unknown("non_finite_input"),
    }
}

/// Conservative box of a circle arc over `[t0, t1]` (radians, ascending or
/// reversed, anchored or wrapped past `2π`).
#[must_use]
pub fn circle_arc_bounds(circle: &Circle3D, t0: f64, t1: f64) -> FiniteBound {
    if !carrier_is_finite(
        circle.center(),
        circle.radius(),
        &[circle.u_axis(), circle.v_axis()],
    ) {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    let Some((lo, hi)) = ascending_span(t0, t1) else {
        return FiniteBound::infinite_unknown("non_finite_span");
    };
    if hi - lo >= TAU - FULL_TURN_ROUNDOFF {
        return FiniteBound::conservative(circle_full_extent(
            circle.center(),
            circle.u_axis(),
            circle.v_axis(),
            circle.radius(),
            circle.radius(),
        ));
    }
    sinusoid_span_bounds(
        &SinusoidCarrier {
            center: circle.center(),
            u_axis: circle.u_axis(),
            v_axis: circle.v_axis(),
            s_a: circle.radius(),
            s_b: circle.radius(),
        },
        lo,
        hi,
        |t| circle.evaluate(t),
    )
}

/// Conservative box of an ellipse arc over `[t0, t1]`.
///
/// Parameterized as `C + a·cos(t)·U + b·sin(t)·V`; each world coordinate is
/// still a single sinusoid, with amplitude
/// `R = hypot(a·U_e, b·V_e)` and phase `atan2(b·V_e, a·U_e)`.
#[must_use]
pub fn ellipse_arc_bounds(ellipse: &Ellipse3D, t0: f64, t1: f64) -> FiniteBound {
    if !carrier_is_finite(
        ellipse.center(),
        ellipse.semi_major(),
        &[ellipse.u_axis(), ellipse.v_axis()],
    ) || !ellipse.semi_minor().is_finite()
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    let Some((lo, hi)) = ascending_span(t0, t1) else {
        return FiniteBound::infinite_unknown("non_finite_span");
    };
    if hi - lo >= TAU - FULL_TURN_ROUNDOFF {
        return FiniteBound::conservative(circle_full_extent(
            ellipse.center(),
            ellipse.u_axis(),
            ellipse.v_axis(),
            ellipse.semi_major(),
            ellipse.semi_minor(),
        ));
    }
    sinusoid_span_bounds(
        &SinusoidCarrier {
            center: ellipse.center(),
            u_axis: ellipse.u_axis(),
            v_axis: ellipse.v_axis(),
            s_a: ellipse.semi_major(),
            s_b: ellipse.semi_minor(),
        },
        lo,
        hi,
        |t| ellipse.evaluate(t),
    )
}

/// Conservative box of a parabola arc over the finite span `[t0, t1]`.
///
/// `P(t) = V + (t²/4f)·A + t·U`, so each world coordinate is quadratic in
/// `t` with at most one stationary point `t* = −U_e·2f/A_e`.
#[must_use]
pub fn parabola_arc_bounds(parabola: &Parabola3D, t0: f64, t1: f64) -> FiniteBound {
    if !carrier_is_finite(
        parabola.vertex(),
        parabola.focal_length(),
        &[parabola.axis_dir(), parabola.u_axis()],
    ) || parabola.focal_length() <= 0.0
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    let Some((lo, hi)) = ascending_span(t0, t1) else {
        return FiniteBound::infinite_unknown("non_finite_span");
    };
    let f = parabola.focal_length();
    let v = parabola.vertex();
    let a = parabola.axis_dir();
    let u = parabola.u_axis();
    // Per-axis quadratic coefficients: coord(t) = c0 + c1·t + c2·t².
    let axes = [
        (v.x(), u.x(), a.x()),
        (v.y(), u.y(), a.y()),
        (v.z(), u.z(), a.z()),
    ];
    let mut points = vec![parabola.evaluate(lo), parabola.evaluate(hi)];
    for (c0, c1, c2raw) in axes {
        let _ = c0;
        let c2 = c2raw / (4.0 * f);
        if c2 != 0.0 {
            let star = -c1 / (2.0 * c2);
            if star >= lo && star <= hi {
                points.push(parabola.evaluate(star));
            }
        }
    }
    match box_of_points(&points) {
        Some(aabb) => FiniteBound::conservative(aabb),
        None => FiniteBound::infinite_unknown("non_finite_evaluation"),
    }
}

/// Conservative box of a hyperbola arc over the finite span `[t0, t1]`.
///
/// `P(t) = C + a·cosh(t)·U + b·sinh(t)·V`, so each world coordinate has the
/// form `C_e + a·U_e·cosh(t) + b·V_e·sinh(t)` with at most one stationary
/// point at `tanh(t*) = −b·V_e / (a·U_e)` (present only when the right-hand
/// side lies strictly inside `(−1, 1)`).
#[must_use]
pub fn hyperbola_arc_bounds(hyperbola: &Hyperbola3D, t0: f64, t1: f64) -> FiniteBound {
    if !carrier_is_finite(
        hyperbola.center(),
        hyperbola.semi_major(),
        &[hyperbola.u_axis(), hyperbola.v_axis()],
    ) || !hyperbola.semi_minor().is_finite()
        || hyperbola.semi_major() <= 0.0
        || hyperbola.semi_minor() <= 0.0
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    let Some((lo, hi)) = ascending_span(t0, t1) else {
        return FiniteBound::infinite_unknown("non_finite_span");
    };
    let a = hyperbola.semi_major();
    let b = hyperbola.semi_minor();
    let u = hyperbola.u_axis();
    let vv = hyperbola.v_axis();
    let comps = [(u.x(), vv.x()), (u.y(), vv.y()), (u.z(), vv.z())];
    let mut points = vec![hyperbola.evaluate(lo), hyperbola.evaluate(hi)];
    for (ue, ve) in comps {
        if ue != 0.0 {
            let rhs = -(b * ve) / (a * ue);
            if rhs > -1.0 && rhs < 1.0 {
                let star = 0.5 * ((1.0 + rhs) / (1.0 - rhs)).ln();
                if star >= lo && star <= hi {
                    points.push(hyperbola.evaluate(star));
                }
            }
        }
    }
    match box_of_points(&points) {
        Some(aabb) => FiniteBound::conservative(aabb),
        None => FiniteBound::infinite_unknown("non_finite_evaluation"),
    }
}

/// Conservative box of a NURBS curve over `[t0, t1]`.
///
/// The span is clipped to the curve domain; the box is the control hull of
/// the control points whose basis support overlaps the clipped span. With
/// positive finite weights the span is a convex combination of those points,
/// so the hull contains it. A reversed span bounds the same set. A span that
/// clips to empty, or a curve that fails validation, refuses with `Unknown`.
#[must_use]
pub fn nurbs_curve_bounds(curve: &NurbsCurve, t0: f64, t1: f64) -> FiniteBound {
    if !t0.is_finite() || !t1.is_finite() {
        return FiniteBound::infinite_unknown("non_finite_span");
    }
    if curve.validate().is_err() {
        return FiniteBound::infinite_unknown("invalid_curve");
    }
    let cps = curve.control_points();
    if !super::all_points_finite(cps) {
        return FiniteBound::infinite_unknown("non_finite_control_net");
    }
    let (d0, d1) = curve.domain();
    // No overlap at all (before clamping hides it): empty span.
    if t0.max(t1) < d0 || t0.min(t1) > d1 {
        let whole_hull = match box_of_points(cps) {
            Some(hull) => hull,
            None => return FiniteBound::infinite_unknown("non_finite_control_net"),
        };
        return FiniteBound::unknown(whole_hull, "empty_span");
    }
    let lo = t0.min(t1).clamp(d0, d1);
    let hi = t0.max(t1).clamp(d0, d1);
    let whole_hull = match box_of_points(cps) {
        Some(hull) => hull,
        None => return FiniteBound::infinite_unknown("non_finite_control_net"),
    };
    if lo > hi {
        return FiniteBound::unknown(whole_hull, "empty_span");
    }
    let degree = curve.degree();
    let knots = curve.knots();
    let span_lo = span_index(knots, degree, cps.len(), lo);
    let span_hi = span_index(knots, degree, cps.len(), hi);
    let first = span_lo.saturating_sub(degree);
    let last = span_hi.min(cps.len().saturating_sub(1));
    let active = &cps[first..=last];
    match box_of_points(active) {
        Some(aabb) => FiniteBound::conservative(aabb),
        None => FiniteBound::unknown(whole_hull, "empty_span"),
    }
}

/// Normalize a trim to an ascending finite span, or `None` when non-finite.
fn ascending_span(t0: f64, t1: f64) -> Option<(f64, f64)> {
    if !t0.is_finite() || !t1.is_finite() {
        return None;
    }
    Some((t0.min(t1), t0.max(t1)))
}

/// Whether a conic carrier's defining data is finite.
fn carrier_is_finite(anchor: Point3, scale: f64, axes: &[remus_math::vec::Vec3]) -> bool {
    anchor.x().is_finite()
        && anchor.y().is_finite()
        && anchor.z().is_finite()
        && scale.is_finite()
        && axes
            .iter()
            .all(|v| v.x().is_finite() && v.y().is_finite() && v.z().is_finite())
}

/// Whole-curve extent of `C + s_a·cos(t)·U + s_b·sin(t)·V`.
///
/// Per axis the amplitude is `hypot(s_a·U_e, s_b·V_e)`; for a circle
/// `s_a = s_b = radius`, for an ellipse the semi-axes.
fn circle_full_extent(
    center: Point3,
    u_axis: remus_math::vec::Vec3,
    v_axis: remus_math::vec::Vec3,
    s_a: f64,
    s_b: f64,
) -> remus_math::aabb::Aabb3 {
    let ext = [
        (s_a * u_axis.x()).hypot(s_b * v_axis.x()),
        (s_a * u_axis.y()).hypot(s_b * v_axis.y()),
        (s_a * u_axis.z()).hypot(s_b * v_axis.z()),
    ];
    remus_math::aabb::Aabb3 {
        min: Point3::new(
            center.x() - ext[0],
            center.y() - ext[1],
            center.z() - ext[2],
        ),
        max: Point3::new(
            center.x() + ext[0],
            center.y() + ext[1],
            center.z() + ext[2],
        ),
    }
}

/// A sinusoid carrier `C + s_a·cos(t)·U + s_b·sin(t)·V` shared by the
/// circle and ellipse span bounders.
struct SinusoidCarrier {
    /// Curve anchor point.
    center: Point3,
    /// First frame direction (carries the `s_a` extent).
    u_axis: remus_math::vec::Vec3,
    /// Second frame direction (carries the `s_b` extent).
    v_axis: remus_math::vec::Vec3,
    /// Extent along `u_axis` (radius for circles, semi-major for ellipses).
    s_a: f64,
    /// Extent along `v_axis` (radius for circles, semi-minor for ellipses).
    s_b: f64,
}

/// Box a sinusoid arc over `[lo, hi]` (`lo <= hi`, span strictly below a
/// full turn).
///
/// Per axis the coordinate is `c + R·cos(t − φ)`; extrema over the span sit
/// at the ends or at the phases `φ` / `φ + π` congruent into the span. The
/// extreme *values* `c ± R` are used directly (no trigonometry at large
/// anchors), and the whole box is padded for endpoint argument-reduction
/// noise (see [`trig_anchor_margin`]).
fn sinusoid_span_bounds(
    carrier: &SinusoidCarrier,
    lo: f64,
    hi: f64,
    evaluate: impl Fn(f64) -> Point3,
) -> FiniteBound {
    let SinusoidCarrier {
        center,
        u_axis,
        v_axis,
        s_a,
        s_b,
    } = *carrier;
    let comps = [
        (center.x(), u_axis.x(), v_axis.x()),
        (center.y(), u_axis.y(), v_axis.y()),
        (center.z(), u_axis.z(), v_axis.z()),
    ];
    let p_lo = evaluate(lo);
    let p_hi = evaluate(hi);
    let ends = [p_lo, p_hi];
    if !super::all_points_finite(&ends) {
        return FiniteBound::infinite_unknown("non_finite_evaluation");
    }
    let mut lo_pt = Point3::new(
        p_lo.x().min(p_hi.x()),
        p_lo.y().min(p_hi.y()),
        p_lo.z().min(p_hi.z()),
    );
    let mut hi_pt = Point3::new(
        p_lo.x().max(p_hi.x()),
        p_lo.y().max(p_hi.y()),
        p_lo.z().max(p_hi.z()),
    );
    for (axis, (ce, ue, ve)) in comps.iter().enumerate() {
        let a = s_a * ue;
        let b = s_b * ve;
        let r = a.hypot(b);
        if r == 0.0 {
            continue;
        }
        let phi = b.atan2(a);
        let set = |pt: Point3, v: f64| -> Point3 {
            match axis {
                0 => Point3::new(v, pt.y(), pt.z()),
                1 => Point3::new(pt.x(), v, pt.z()),
                _ => Point3::new(pt.x(), pt.y(), v),
            }
        };
        if phase_in_span(phi, lo, hi) {
            hi_pt = set(hi_pt, ce + r);
        }
        if phase_in_span(phi + std::f64::consts::PI, lo, hi) {
            lo_pt = set(lo_pt, ce - r);
        }
    }
    let margin = trig_anchor_margin(lo, hi, s_a.abs().max(s_b.abs()));
    if !margin.is_finite() {
        return FiniteBound::infinite_unknown("anchor_out_of_range");
    }
    let aabb = remus_math::aabb::Aabb3 {
        min: Point3::new(lo_pt.x() - margin, lo_pt.y() - margin, lo_pt.z() - margin),
        max: Point3::new(hi_pt.x() + margin, hi_pt.y() + margin, hi_pt.z() + margin),
    };
    FiniteBound::conservative(aabb)
}

/// Whether the phase `phi` (any congruent representative) has a congruent
/// angle inside `[lo, hi]`.
fn phase_in_span(phi: f64, lo: f64, hi: f64) -> bool {
    // Shift `phi` by whole turns to land at or just past `lo`.
    let k = ((lo - phi) / TAU).ceil();
    let t = phi + k * TAU;
    t <= hi
}

/// Padding for endpoint trigonometry at large parameter anchors.
///
/// `sin`/`cos` argument reduction loses roughly `|t|·ε` of absolute
/// precision; the endpoint evaluations can therefore sit slightly off the
/// true arc. The margin covers that first-order noise with headroom. Anchors
/// beyond `1e12` are outside every kernel trim convention (recorded trims
/// sit near the curve's natural domain or a nearby seam anchor); callers
/// needing larger anchors must establish their own reduction guarantees.
fn trig_anchor_margin(lo: f64, hi: f64, scale: f64) -> f64 {
    const MAX_ANCHOR: f64 = 1e12;
    if lo.abs() > MAX_ANCHOR || hi.abs() > MAX_ANCHOR || !scale.is_finite() {
        return f64::INFINITY;
    }
    (lo.abs() + hi.abs()) * f64::EPSILON * scale.abs() * 8.0
}

/// Knot-span index for parameter `t`: the largest `i` with
/// `knots[i] <= t`, clamped to the valid span range `[degree, n - 1]`
/// where `n` is the control-point count.
pub(crate) fn span_index(knots: &[f64], degree: usize, num_points: usize, t: f64) -> usize {
    let last = num_points.saturating_sub(1);
    let mut span = degree;
    while span < last && knots.get(span + 1).is_some_and(|&k| k <= t) {
        span += 1;
    }
    span.min(last)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::vec::Vec3;

    fn circle(radius: f64) -> Circle3D {
        // Pinned frame: u = +X, v = +Y, so angle 0 is (r, 0, 0) and π/2 is
        // (0, r, 0). (`Circle3D::new` derives an arbitrary perpendicular.)
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            radius,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn quarter_arc_box_is_exact() {
        // Quarter circle from angle 0 to π/2: x ∈ [0, 1], y ∈ [0, 1].
        let bound = circle_arc_bounds(&circle(1.0), 0.0, std::f64::consts::FRAC_PI_2);
        assert!(bound.is_prunable());
        let aabb = bound.aabb();
        assert!((aabb.min.x() - 0.0).abs() < 1e-12, "min.x={}", aabb.min.x());
        assert!((aabb.max.x() - 1.0).abs() < 1e-12, "max.x={}", aabb.max.x());
        assert!((aabb.min.y() - 0.0).abs() < 1e-12, "min.y={}", aabb.min.y());
        assert!((aabb.max.y() - 1.0).abs() < 1e-12, "max.y={}", aabb.max.y());
    }

    #[test]
    fn reversed_trim_bounds_the_same_set() {
        let fwd = circle_arc_bounds(&circle(2.0), 0.3, 1.9);
        let rev = circle_arc_bounds(&circle(2.0), 1.9, 0.3);
        assert!(fwd.is_prunable() && rev.is_prunable());
        for (a, b) in [
            (fwd.aabb().min.x(), rev.aabb().min.x()),
            (fwd.aabb().max.y(), rev.aabb().max.y()),
        ] {
            assert!((a - b).abs() < 1e-12, "{a} vs {b}");
        }
    }

    #[test]
    fn anchored_full_turn_takes_whole_extent() {
        let c = circle(3.0);
        let bound = circle_arc_bounds(&c, 2.8, 2.8 + TAU);
        assert!(bound.is_prunable());
        let aabb = bound.aabb();
        assert!((aabb.min.x() + 3.0).abs() < 1e-9);
        assert!((aabb.max.x() - 3.0).abs() < 1e-9);
        assert!((aabb.min.y() + 3.0).abs() < 1e-9);
        assert!((aabb.max.y() - 3.0).abs() < 1e-9);
    }

    /// Arc strictly between axis extrema: the box must equal the endpoint
    /// box (no spurious extreme leaks in), proving endpoints-away-from-extrema
    /// handling.
    #[test]
    fn endpoints_away_from_extrema_box_endpoints() {
        // Arc [0.2, 0.5] on the unit circle: monotonic in both x (falling)
        // and y (rising); no phase 0, π/2, π, 3π/2 lies inside.
        let c = circle(1.0);
        let (t0, t1) = (0.2, 0.5);
        let bound = circle_arc_bounds(&c, t0, t1);
        assert!(bound.is_prunable());
        let (p0, p1) = (c.evaluate(t0), c.evaluate(t1));
        let aabb = bound.aabb();
        assert!(
            (aabb.min.x() - p1.x()).abs() < 1e-9,
            "min.x={}",
            aabb.min.x()
        );
        assert!(
            (aabb.max.x() - p0.x()).abs() < 1e-9,
            "max.x={}",
            aabb.max.x()
        );
        assert!(
            (aabb.min.y() - p0.y()).abs() < 1e-9,
            "min.y={}",
            aabb.min.y()
        );
        assert!(
            (aabb.max.y() - p1.y()).abs() < 1e-9,
            "max.y={}",
            aabb.max.y()
        );
    }

    /// A very short arc: the box stays tiny and still contains the span.
    #[test]
    fn very_short_arc_stays_tight() {
        let c = circle(2.0);
        let bound = circle_arc_bounds(&c, 1.0, 1.0 + 1e-9);
        assert!(bound.is_prunable());
        let aabb = bound.aabb();
        let diag = (aabb.max - aabb.min).length();
        assert!(diag < 1e-6, "short-arc box diagonal {diag}");
        for i in 0..=8 {
            let t = 1.0 + 1e-9 * f64::from(i) / 8.0;
            assert!(aabb.contains_point(c.evaluate(t)));
        }
    }

    /// Tilted carrier: the per-axis amplitude/phase derivation is
    /// orientation-agnostic. The box must contain dense samples and stay
    /// tight against them (proving interior extrema were captured, not just
    /// endpoints).
    #[test]
    fn tilted_circle_arc_captures_interior_extrema() {
        let tilted =
            Circle3D::new(Point3::new(1.0, -2.0, 3.0), Vec3::new(1.0, 1.0, 1.0), 2.5).unwrap();
        let (t0, t1) = (0.0, std::f64::consts::PI);
        let bound = circle_arc_bounds(&tilted, t0, t1);
        assert!(bound.is_prunable());
        assert_tight_against_samples(&bound, |t| tilted.evaluate(t), t0, t1, 720, 1e-9);
    }

    /// Seam-crossing arc expressed as an anchored span past `2π`.
    #[test]
    fn seam_crossing_anchored_span() {
        let c = circle(1.0);
        // Arc from 5.5 rad sweeping 1.0 rad forward, crossing the seam at 2π.
        let (t0, t1) = (5.5, 5.5 + 1.0);
        let bound = circle_arc_bounds(&c, t0, t1);
        assert!(bound.is_prunable());
        assert_tight_against_samples(&bound, |t| c.evaluate(t), t0, t1, 360, 1e-9);
        // The seam point (angle 0 ≡ 2π ≈ 6.28 ∈ [5.5, 6.5]) has x = 1: the
        // box must reach it.
        assert!((bound.aabb().max.x() - 1.0).abs() < 1e-9);
    }

    /// Ellipse arc away from extrema plus a tilted-ellipse tightness check.
    #[test]
    fn ellipse_arcs() {
        let e = Ellipse3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            4.0,
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        // Arc [0.1, 0.4]: x falling from ~4, y rising from ~0.1; the y
        // maximum (phase π/2) is outside, so max.y is the endpoint value.
        let bound = ellipse_arc_bounds(&e, 0.1, 0.4);
        assert!(bound.is_prunable());
        assert!((bound.aabb().max.y() - e.evaluate(0.4).y()).abs() < 1e-9);
        assert!((bound.aabb().max.x() - e.evaluate(0.1).x()).abs() < 1e-9);

        let tilted = Ellipse3D::new(
            Point3::new(-1.0, 2.0, 0.5),
            Vec3::new(0.0, 1.0, 1.0),
            3.0,
            1.0,
        )
        .unwrap();
        let bound = ellipse_arc_bounds(&tilted, 0.5, 4.0);
        assert!(bound.is_prunable());
        assert_tight_against_samples(&bound, |t| tilted.evaluate(t), 0.5, 4.0, 720, 1e-8);
    }

    /// Parabola vertex inside the span (minimum captured) and outside the
    /// span (endpoint box only).
    #[test]
    fn parabola_vertex_in_and_out_of_span() {
        let p = Parabola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.25,
        )
        .unwrap();
        // Span [-1, 1]: y = t² ranges over [0, 1]; the vertex t = 0 is the
        // y minimum and x is odd (no interior extreme).
        let bound = parabola_arc_bounds(&p, -1.0, 1.0);
        assert!(bound.is_prunable());
        assert!(
            (bound.aabb().min.y() - 0.0).abs() < 1e-12,
            "min.y={}",
            bound.aabb().min.y()
        );
        assert!(
            (bound.aabb().max.y() - 1.0).abs() < 1e-12,
            "max.y={}",
            bound.aabb().max.y()
        );
        assert!((bound.aabb().min.x() + 1.0).abs() < 1e-12);
        assert!((bound.aabb().max.x() - 1.0).abs() < 1e-12);

        // Span [1, 2]: vertex outside; y ∈ [1, 4] from the endpoints.
        let bound = parabola_arc_bounds(&p, 2.0, 1.0);
        assert!(bound.is_prunable());
        assert!((bound.aabb().min.y() - 1.0).abs() < 1e-12);
        assert!((bound.aabb().max.y() - 4.0).abs() < 1e-12);
    }

    /// Hyperbola symmetric span: x minimum at the vertex t = 0, y odd.
    #[test]
    fn hyperbola_symmetric_span() {
        let h = Hyperbola3D::with_axes(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0,
            3.0,
        )
        .unwrap();
        let bound = hyperbola_arc_bounds(&h, -1.0, 1.0);
        assert!(bound.is_prunable());
        // x = 2·cosh(t): minimum 2 at t = 0; y = 3·sinh(t): odd.
        assert!(
            (bound.aabb().min.x() - 2.0).abs() < 1e-12,
            "min.x={}",
            bound.aabb().min.x()
        );
        assert!((bound.aabb().max.x() - 2.0 * 1.0_f64.cosh()).abs() < 1e-9);
        assert!((bound.aabb().min.y() + 3.0 * 1.0_f64.sinh()).abs() < 1e-9);
        assert!((bound.aabb().max.y() - 3.0 * 1.0_f64.sinh()).abs() < 1e-9);
        // Reversed span bounds the same set.
        let rev = hyperbola_arc_bounds(&h, 1.0, -1.0);
        assert!((rev.aabb().min.x() - bound.aabb().min.x()).abs() < 1e-12);
    }

    /// Large translation: the vertex extreme survives far from the origin.
    #[test]
    fn parabola_large_translation() {
        let vertex = Point3::new(1e6, -2e6, 3e6);
        let p = Parabola3D::with_axes(
            vertex,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            0.25,
        )
        .unwrap();
        let bound = parabola_arc_bounds(&p, -1.0, 1.0);
        assert!(bound.is_prunable());
        // y minimum is exactly the vertex y; allow scaled floating error.
        assert!(
            (bound.aabb().min.y() - vertex.y()).abs() < 1e-6,
            "min.y={} vs {}",
            bound.aabb().min.y(),
            vertex.y()
        );
        assert_tight_against_samples(&bound, |t| p.evaluate(t), -1.0, 1.0, 200, 1e-3);
    }

    /// Multiple scales: the quarter-arc box is exact from microns to
    /// megameters (relative tolerance).
    #[test]
    fn circle_scales() {
        for radius in [1e-6, 1.0, 1e6] {
            let c = Circle3D::new_with_ref(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                radius,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let bound = circle_arc_bounds(&c, 0.0, std::f64::consts::FRAC_PI_2);
            assert!(bound.is_prunable());
            let rel = radius;
            assert!((bound.aabb().max.x() - radius).abs() < 1e-9 * rel);
            assert!(bound.aabb().min.x().abs() < 1e-9 * rel + 1e-18);
        }
    }

    /// Nonuniform positive weights: the span stays inside the active control
    /// hull, and the full domain reproduces the whole hull.
    #[test]
    fn nurbs_nonuniform_weights_hull() {
        // Rational quarter circle (exact) with non-1 weights.
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        )
        .unwrap();
        let (d0, d1) = curve.domain();
        let bound = nurbs_curve_bounds(&curve, d0, d1);
        assert!(bound.is_prunable());
        // Hull of the three control points.
        assert!((bound.aabb().min.x() - 0.0).abs() < 1e-12);
        assert!((bound.aabb().max.x() - 1.0).abs() < 1e-12);
        assert!((bound.aabb().min.y() - 0.0).abs() < 1e-12);
        assert!((bound.aabb().max.y() - 1.0).abs() < 1e-12);
        // The arc itself stays on the circle (inside the hull, as proven).
        for i in 0..=40 {
            let t = d0 + (d1 - d0) * f64::from(i) / 40.0;
            let p = curve.evaluate(t);
            assert!((p.x().hypot(p.y()) - 1.0).abs() < 1e-9);
            assert!(bound.aabb().contains_point(p));
        }
    }

    /// A sub-span uses the active control hull, strictly tighter than the
    /// whole hull when later spans stick out.
    #[test]
    fn nurbs_subspan_uses_active_hull() {
        // Degree-2 curve with 4 control points and 2 spans; the last control
        // point sticks far out in +z.
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(3.0, 0.0, 100.0),
            ],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .unwrap();
        let (d0, d1) = curve.domain();
        assert!((d0 - 0.0).abs() < 1e-15 && (d1 - 1.0).abs() < 1e-15);
        // First span [0, 0.5] involves control points 0..=2 only.
        let sub = nurbs_curve_bounds(&curve, 0.0, 0.4);
        assert!(sub.is_prunable());
        assert!(
            sub.aabb().max.z() < 100.0,
            "sub-span must exclude the far control point, got {}",
            sub.aabb().max.z()
        );
        for i in 0..=40 {
            let t = 0.4 * f64::from(i) / 40.0;
            assert!(sub.aabb().contains_point(curve.evaluate(t)));
        }
        // Full domain covers everything.
        let full = nurbs_curve_bounds(&curve, d0, d1);
        assert!((full.aabb().max.z() - 100.0).abs() < 1e-12);
    }

    /// A span outside the domain clips to empty: unknown, but non-empty.
    #[test]
    fn nurbs_empty_span_is_unknown() {
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let (_, d1) = curve.domain();
        let bound = nurbs_curve_bounds(&curve, d1 + 1.0, d1 + 2.0);
        assert!(!bound.is_prunable());
        assert!(bound.aabb().contains_point(Point3::new(0.5, 0.0, 0.0)));
    }

    /// Containment plus tightness against dense samples.
    ///
    /// Sampling supplements the closed-form proof: containment must hold
    /// exactly, and the box must hug the samples within `tight` plus the
    /// sampling sagitta `(span/samples)²·scale` (a missed interior extreme
    /// would leave a scale-visible gap far above that resolution floor).
    fn assert_tight_against_samples(
        bound: &FiniteBound,
        evaluate: impl Fn(f64) -> Point3,
        t0: f64,
        t1: f64,
        samples: usize,
        tight: f64,
    ) {
        let (mut smin, mut smax) = (
            Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        );
        for i in 0..=samples {
            // Pin the ends exactly: recomputing an end as `t0 + span`
            // can differ by one ULP from the caller's endpoint, which the
            // bound boxes exactly.
            #[allow(clippy::cast_precision_loss)]
            let t = if i == 0 {
                t0
            } else if i == samples {
                t1
            } else {
                t0 + (t1 - t0) * (i as f64) / (samples as f64)
            };
            let p = evaluate(t);
            assert!(
                bound.aabb().contains_point(p),
                "box must contain sample {p:?}"
            );
            smin = Point3::new(
                smin.x().min(p.x()),
                smin.y().min(p.y()),
                smin.z().min(p.z()),
            );
            smax = Point3::new(
                smax.x().max(p.x()),
                smax.y().max(p.y()),
                smax.z().max(p.z()),
            );
        }
        let aabb = bound.aabb();
        #[allow(clippy::cast_precision_loss)]
        let step = (t1 - t0).abs() / samples as f64;
        let diag = (aabb.max - aabb.min).length();
        let slack = tight + step * step * diag;
        for (lo, slo, hi, shi) in [
            (aabb.min.x(), smin.x(), aabb.max.x(), smax.x()),
            (aabb.min.y(), smin.y(), aabb.max.y(), smax.y()),
            (aabb.min.z(), smin.z(), aabb.max.z(), smax.z()),
        ] {
            assert!(lo <= slo + slack, "lo {lo} vs samples {slo}");
            assert!(hi >= shi - slack, "hi {hi} vs samples {shi}");
            assert!(slo - lo < slack, "box loose below samples: {slo} vs {lo}");
            assert!(hi - shi < slack, "box loose above samples: {hi} vs {shi}");
        }
    }

    #[test]
    fn non_finite_span_is_unknown_and_nonempty() {
        let bound = circle_arc_bounds(&circle(1.0), f64::NAN, 1.0);
        assert!(!bound.is_prunable());
        // Unknown must never mean an empty box: the whole-space fallback has
        // zero distance to any query point.
        let d = bound
            .aabb()
            .distance_squared_to_point(Point3::new(5.0, 5.0, 5.0));
        assert!(d <= 0.0, "unknown fallback must never prune, got {d}");
    }
}

/// Randomized qualification: random spans must be prunable and contain
/// their samples. Sampling supplements the closed-form proofs (it is not
/// the proof); its value here is breadth over anchors, orientations, and
/// scales the hand-picked fixtures do not cover.
#[cfg(test)]
mod proptests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use proptest::prelude::*;
    use remus_math::vec::Vec3;

    use super::*;

    /// Arbitrary finite scale covering microns to megameters.
    fn scale_strategy() -> impl Strategy<Value = f64> {
        prop_oneof![1e-6..1e-3, 1e-3..1.0, 1.0..10.0, 10.0..1e6,]
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        #[test]
        fn random_circle_arcs_contain_samples(
            cx in -10.0f64..10.0,
            cy in -10.0f64..10.0,
            cz in -10.0f64..10.0,
            nx in -1.0f64..1.0,
            ny in -1.0f64..1.0,
            nz in -1.0f64..1.0,
            radius in 0.01f64..10.0,
            t0 in -TAU..TAU,
            span in 0.001f64..TAU,
        ) {
            let normal = Vec3::new(nx, ny, nz);
            prop_assume!(normal.length_squared() > 1e-6);
            let center = Point3::new(cx, cy, cz);
            let circle = Circle3D::new(center, normal, radius).unwrap();
            let bound = circle_arc_bounds(&circle, t0, t0 + span);
            prop_assert!(bound.is_prunable());
            for i in 0..=32 {
                #[allow(clippy::cast_precision_loss)]
                let t = if i == 0 {
                    t0
                } else if i == 32 {
                    t0 + span
                } else {
                    t0 + span * (i as f64) / 32.0
                };
                prop_assert!(
                    bound.aabb().contains_point(circle.evaluate(t)),
                    "arc sample outside box"
                );
            }
        }

        #[test]
        fn random_ellipse_arcs_contain_samples(
            semi_major in 0.1f64..10.0,
            ratio in 0.05f64..1.0,
            t0 in -TAU..TAU,
            span in 0.001f64..TAU,
        ) {
            let semi_minor = semi_major * ratio;
            let ellipse = Ellipse3D::new(
                Point3::new(1.0, -1.0, 0.5),
                Vec3::new(0.0, 1.0, 1.0),
                semi_major,
                semi_minor,
            )
            .unwrap();
            let bound = ellipse_arc_bounds(&ellipse, t0, t0 + span);
            prop_assert!(bound.is_prunable());
            for i in 0..=32 {
                #[allow(clippy::cast_precision_loss)]
                let t = if i == 0 {
                    t0
                } else if i == 32 {
                    t0 + span
                } else {
                    t0 + span * (i as f64) / 32.0
                };
                prop_assert!(bound.aabb().contains_point(ellipse.evaluate(t)));
            }
        }

        #[test]
        fn random_parabola_spans_contain_samples(
            t0 in -3.0f64..3.0,
            span in 0.001f64..3.0,
        ) {
            let parabola = Parabola3D::with_axes(
                Point3::new(-2.0, 5.0, 1.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 1.0, 0.0),
                1.7,
            )
            .unwrap();
            let (lo, hi) = (t0.min(t0 + span), t0.max(t0 + span));
            let bound = parabola_arc_bounds(&parabola, lo, hi);
            prop_assert!(bound.is_prunable());
            for i in 0..=24 {
                #[allow(clippy::cast_precision_loss)]
                let t = if i == 0 {
                    lo
                } else if i == 24 {
                    hi
                } else {
                    lo + (hi - lo) * (i as f64) / 24.0
                };
                prop_assert!(bound.aabb().contains_point(parabola.evaluate(t)));
            }
        }

        #[test]
        fn random_hyperbola_spans_contain_samples(
            t0 in -2.0f64..2.0,
            span in 0.001f64..2.0,
        ) {
            let hyperbola = Hyperbola3D::with_axes(
                Point3::new(3.0, -1.0, 7.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                2.0,
                3.0,
            )
            .unwrap();
            let (lo, hi) = (t0.min(t0 + span), t0.max(t0 + span));
            let bound = hyperbola_arc_bounds(&hyperbola, lo, hi);
            prop_assert!(bound.is_prunable());
            for i in 0..=24 {
                #[allow(clippy::cast_precision_loss)]
                let t = if i == 0 {
                    lo
                } else if i == 24 {
                    hi
                } else {
                    lo + (hi - lo) * (i as f64) / 24.0
                };
                prop_assert!(bound.aabb().contains_point(hyperbola.evaluate(t)));
            }
        }

        #[test]
        fn random_circle_scales_stay_prunable(scale in scale_strategy()) {
            let circle = Circle3D::new_with_ref(
                Point3::new(scale, -scale, 0.5 * scale),
                Vec3::new(0.0, 0.0, 1.0),
                scale,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap();
            let bound = circle_arc_bounds(&circle, 0.3, 2.1);
            prop_assert!(bound.is_prunable());
            prop_assert!(bound.aabb().contains_point(circle.evaluate(0.3)));
            prop_assert!(bound.aabb().contains_point(circle.evaluate(2.1)));
            prop_assert!(bound.aabb().contains_point(circle.evaluate(1.2)));
        }
    }
}
