//! Bezier clipping for curve-curve intersection (Sederberg-Nishita 1990).
//!
//! Decomposes NURBS curves into Bezier segments, then uses recursive
//! fat-line clipping to find all intersection points.
//!
//! Every depth works on the CURRENT windows: both are blossomed out of
//! their parent segments, the fat line and the distance polygon come from
//! those sub-segments, and rational windows are clipped through their
//! weighted distance numerators. Termination, Newton polishing, duplicate
//! merging and the overlap/tangent-contact split are judged in model
//! space, so the result does not depend on how parameter speed relates to
//! the model scale (roadmap B10).

#![allow(clippy::similar_names, clippy::suspicious_operation_groupings)]

use crate::MathError;
use crate::nurbs::curve::NurbsCurve;
use crate::nurbs::decompose::curve_to_bezier_segments;
use crate::vec::{Point3, Vec3};

/// Maximum recursion depth for Bezier clipping.
const MAX_DEPTH: usize = 50;

/// Maximum Newton refinement iterations.
const MAX_NEWTON: usize = 10;

/// Threshold ratio: if clipping removes less than 40%, subdivide instead.
const CLIP_THRESHOLD: f64 = 0.6;

/// A curve-curve intersection result.
#[derive(Debug, Clone, Copy)]
pub struct CurveCurveHit {
    /// Parameter on the first curve.
    pub u1: f64,
    /// Parameter on the second curve.
    pub u2: f64,
    /// The intersection point.
    pub point: Point3,
}

/// A coincident/overlapping interval between two curves.
#[derive(Debug, Clone, Copy)]
pub struct CurveCurveOverlap {
    /// Start parameter on the first curve.
    pub u1_start: f64,
    /// End parameter on the first curve.
    pub u1_end: f64,
    /// Start parameter on the second curve.
    pub u2_start: f64,
    /// End parameter on the second curve.
    pub u2_end: f64,
}

/// Complete result of a curve-curve intersection, including both point
/// intersections and overlapping (coincident) intervals.
#[derive(Debug, Clone)]
pub struct CurveCurveResult {
    /// Isolated intersection points.
    pub hits: Vec<CurveCurveHit>,
    /// Coincident curve intervals (shared sub-arcs).
    pub overlaps: Vec<CurveCurveOverlap>,
}

/// Find all intersections between two NURBS curves using Bezier clipping.
///
/// Returns intersection parameters and points, accurate to `tolerance`.
///
/// # Errors
///
/// Returns an error if curve decomposition fails.
pub fn curve_curve_intersect(
    curve1: &NurbsCurve,
    curve2: &NurbsCurve,
    tolerance: f64,
) -> Result<Vec<CurveCurveHit>, MathError> {
    let result = curve_curve_intersect_full(curve1, curve2, tolerance)?;
    Ok(result.hits)
}

/// Find all intersections between two NURBS curves, including overlapping
/// (coincident) intervals.
///
/// Use this instead of [`curve_curve_intersect`] when you need to detect
/// shared sub-arcs between curves.
///
/// `tolerance` is a model-space distance: a point hit is reported only
/// where the two curves come within `tolerance` of each other, and hits
/// closer than `tolerance` (or joined by a stretch along which the curves
/// stay within `tolerance`) are one contact. Hit points are polished to
/// floating-point precision relative to the model scale, not merely to
/// `tolerance`.
///
/// # Errors
///
/// Returns an error if curve decomposition fails.
pub fn curve_curve_intersect_full(
    curve1: &NurbsCurve,
    curve2: &NurbsCurve,
    tolerance: f64,
) -> Result<CurveCurveResult, MathError> {
    let segments1 = curve_to_bezier_segments(curve1)?;
    let segments2 = curve_to_bezier_segments(curve2)?;

    let mut out = ClipOutput {
        tolerance,
        hits: Vec::new(),
        overlaps: Vec::new(),
    };

    for seg1 in &segments1 {
        // Control-point boxes contain their (positive-weight) segments;
        // pad by the reporting tolerance like the recursive early exit.
        let aabb1 = seg1.aabb().expanded(tolerance);
        for seg2 in &segments2 {
            if !aabb1.intersects(seg2.aabb()) {
                continue;
            }

            let (u1_lo, u1_hi) = seg1.domain();
            let (u2_lo, u2_hi) = seg2.domain();

            bezier_clip_recurse(
                ClipSide::new(seg1, u1_lo, u1_hi),
                ClipSide::new(seg2, u2_lo, u2_hi),
                false,
                0,
                &mut out,
            );
        }
    }

    let ClipOutput {
        mut hits,
        mut overlaps,
        ..
    } = out;
    merge_duplicate_hits(&mut hits, curve1, curve2, tolerance);
    merge_overlaps(&mut overlaps, curve1, tolerance);
    // Remove point hits that fall within an overlap interval. The
    // interval test runs in curve1's parameter space, so the model-space
    // tolerance is converted through the local parameter speed.
    if !overlaps.is_empty() {
        hits.retain(|h| {
            let slack = param_tolerance(curve1, h.u1, tolerance);
            !overlaps
                .iter()
                .any(|o| h.u1 >= o.u1_start - slack && h.u1 <= o.u1_end + slack)
        });
    }
    Ok(CurveCurveResult { hits, overlaps })
}

/// Convert a model-space `tolerance` into a parameter-space slack on
/// `curve` at `u`, through the local parameter speed `|C'(u)|`.
fn param_tolerance(curve: &NurbsCurve, u: f64, tolerance: f64) -> f64 {
    let speed = curve.derivatives(u, 1).get(1).map_or(0.0, |d| d.length());
    if speed.is_finite() && speed > 0.0 {
        tolerance / speed
    } else {
        0.0
    }
}

/// Accumulated output of one recursive clipping run.
struct ClipOutput {
    /// Model-space reporting tolerance.
    tolerance: f64,
    /// Point hits, always stored as `(curve1, curve2)` parameters.
    hits: Vec<CurveCurveHit>,
    /// Coincident intervals, always stored as `(curve1, curve2)` parameters.
    overlaps: Vec<CurveCurveOverlap>,
}

impl ClipOutput {
    /// Record a hit found with the recursion's `(a, b)` roles, undoing a
    /// role swap so `u1` always belongs to the first input curve.
    fn push_hit(&mut self, swapped: bool, u_a: f64, u_b: f64, point: Point3) {
        let (u1, u2) = if swapped { (u_b, u_a) } else { (u_a, u_b) };
        self.hits.push(CurveCurveHit { u1, u2, point });
    }

    /// Record an overlap found with the recursion's `(a, b)` roles.
    fn push_overlap(&mut self, swapped: bool, a: ClipSide<'_>, b: ClipSide<'_>) {
        let (first, second) = if swapped { (b, a) } else { (a, b) };
        self.overlaps.push(CurveCurveOverlap {
            u1_start: first.lo,
            u1_end: first.hi,
            u2_start: second.lo,
            u2_end: second.hi,
        });
    }
}

/// One side of a clipping pair: a single-span Bezier segment and the
/// parameter window of it still under consideration.
#[derive(Clone, Copy)]
struct ClipSide<'s> {
    seg: &'s NurbsCurve,
    lo: f64,
    hi: f64,
}

impl<'s> ClipSide<'s> {
    const fn new(seg: &'s NurbsCurve, lo: f64, hi: f64) -> Self {
        Self { seg, lo, hi }
    }

    const fn with_window(self, lo: f64, hi: f64) -> Self {
        Self::new(self.seg, lo, hi)
    }

    fn span(self) -> f64 {
        self.hi - self.lo
    }

    fn mid(self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    /// Map a local sub-window parameter `t` in `[0, 1]` to the segment's
    /// native parameter, pinning the ends so a full-window clip is exact.
    fn at(self, t: f64) -> f64 {
        if t <= 0.0 {
            self.lo
        } else if t >= 1.0 {
            self.hi
        } else {
            t.mul_add(self.span(), self.lo).clamp(self.lo, self.hi)
        }
    }

    /// Narrow the window to the local sub-interval `[t0, t1]`.
    fn narrowed(self, t0: f64, t1: f64) -> Self {
        let lo = self.at(t0);
        let hi = self.at(t1).max(lo);
        self.with_window(lo, hi)
    }

    /// Whether the window can no longer be split in floating point.
    fn at_param_floor(self) -> bool {
        let magnitude = self.lo.abs().max(self.hi.abs()).max(f64::MIN_POSITIVE);
        self.span() <= 64.0 * f64::EPSILON * magnitude
    }
}

/// Rational Bezier control data of a [`ClipSide`] window, recomputed from
/// the parent segment at every clipping depth.
///
/// Sederberg-Nishita clipping is only sound (and only converges) when the
/// fat line and the distance polygon describe the CURRENT sub-curve.
/// Re-using the parent segment's control polygon while narrowing only the
/// parameter window turns every clip into a fixed centred shrink toward
/// the window midpoint, which excludes an off-centre root within a few
/// levels (B10).
struct SubSegment {
    /// Cartesian control points of the window.
    pts: Vec<Point3>,
    /// Weights of the window (positive because the parent's are).
    weights: Vec<f64>,
}

impl SubSegment {
    /// Extract the window by blossoming the parent segment's homogeneous
    /// control points: control point `i` of the window `[s0, s1]` is the
    /// blossom `b(s1^i, s0^(p-i))`. Each window is taken directly from the
    /// parent, so rounding does not accumulate across depths.
    fn new(side: ClipSide<'_>) -> Option<Self> {
        let (a, b) = side.seg.domain();
        let width = b - a;
        if width.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return None;
        }
        let s0 = ((side.lo - a) / width).clamp(0.0, 1.0);
        let s1 = ((side.hi - a) / width).clamp(0.0, 1.0);
        let cps = side.seg.control_points();
        let ws = side.seg.weights();
        let n = cps.len();
        if n < 2 || ws.len() != n {
            return None;
        }
        let degree = n - 1;
        let homogeneous: Vec<[f64; 4]> = cps
            .iter()
            .zip(ws)
            .map(|(c, &w)| [c.x() * w, c.y() * w, c.z() * w, w])
            .collect();

        let mut pts = Vec::with_capacity(n);
        let mut weights = Vec::with_capacity(n);
        let mut scratch = homogeneous.clone();
        for i in 0..=degree {
            scratch.copy_from_slice(&homogeneous);
            for level in 0..degree {
                let t = if level < i { s1 } else { s0 };
                for j in 0..(degree - level) {
                    let next = scratch[j + 1];
                    for (k, value) in scratch[j].iter_mut().enumerate() {
                        *value = t.mul_add(next[k] - *value, *value);
                    }
                }
            }
            let h = scratch[0];
            if !(h[3].is_finite() && h[3] > 0.0) {
                return None;
            }
            pts.push(Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]));
            weights.push(h[3]);
        }
        Some(Self { pts, weights })
    }

    /// Control-point box: contains the window's curve (positive weights).
    fn aabb(&self) -> crate::aabb::Aabb3 {
        crate::aabb::Aabb3::from_points(self.pts.iter().copied())
    }

    /// Diagonal of the control-point box: the window's 3D size bound.
    fn extent(&self) -> f64 {
        let b = self.aabb();
        (b.max - b.min).length()
    }

    /// Largest absolute control-point coordinate: the rounding scale.
    fn magnitude(&self) -> f64 {
        self.pts
            .iter()
            .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
            .fold(0.0, f64::max)
    }

    /// Largest control-point distance from the chord line: zero for a
    /// straight window.
    fn flatness(&self) -> f64 {
        let p0 = self.pts[0];
        let chord = self.pts[self.pts.len() - 1] - p0;
        let len = chord.length();
        if len.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater) {
            return self.extent();
        }
        let dir = chord * (1.0 / len);
        self.pts
            .iter()
            .map(|&p| {
                let v = p - p0;
                (v - dir * v.dot(dir)).length()
            })
            .fold(0.0, f64::max)
    }
}

/// Outcome of clipping one window against the other's fat line.
enum Clip {
    /// The window cannot meet the fat line: no intersection here.
    Empty,
    /// Local sub-interval `[t0, t1]` of `[0, 1]` that may still intersect.
    Interval(f64, f64),
}

/// Unit normal of `a`'s fat line: perpendicular to `a`'s chord, pointing
/// toward `a`'s farthest control point (the in-plane normal for a planar
/// window). A straight `a` borrows the direction toward `b` so the slab
/// still separates the pair. Soundness does not depend on the choice:
/// any unit direction gives a valid slab; the choice only sets how thin
/// `a`'s slab is.
fn fat_line_normal(a: &SubSegment, b: &SubSegment) -> Option<Vec3> {
    let p0 = a.pts[0];
    let mut chord = a.pts[a.pts.len() - 1] - p0;
    let a_extent = a.extent();
    if chord.length() <= a_extent * 1e-9 {
        // Closed or collapsed window: the chord is undefined, so take the
        // direction to the farthest control point instead.
        chord = a
            .pts
            .iter()
            .map(|&p| p - p0)
            .max_by(|u, v| u.length_squared().total_cmp(&v.length_squared()))?;
    }
    let chord_len = chord.length();
    if !(chord_len.is_finite() && chord_len > 0.0) {
        return None;
    }
    let dir = chord * (1.0 / chord_len);
    let farthest_perp = |pts: &[Point3]| {
        pts.iter()
            .map(|&p| {
                let v = p - p0;
                v - dir * v.dot(dir)
            })
            .max_by(|u, v| u.length_squared().total_cmp(&v.length_squared()))
    };
    let mut normal = farthest_perp(&a.pts)?;
    if normal.length() <= chord_len * 1e-9 {
        normal = farthest_perp(&b.pts)?;
    }
    if normal.length() <= chord_len * 1e-15 {
        // Both windows lie on one line: any perpendicular will do.
        let axis = if dir.x().abs() <= dir.y().abs() && dir.x().abs() <= dir.z().abs() {
            Vec3::new(1.0, 0.0, 0.0)
        } else if dir.y().abs() <= dir.z().abs() {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        normal = dir.cross(axis);
    }
    let len = normal.length();
    if !(len.is_finite() && len > 0.0) {
        return None;
    }
    Some(normal * (1.0 / len))
}

/// Clip window `b` against the fat line of window `a`.
///
/// The fat line is the slab `{P : (P - a0)·m in [d_min, d_max]}` spanned
/// by `a`'s own control points (it contains `a`'s curve because the
/// weights are positive), widened by `pad` to absorb rounding. The signed
/// distance is the AFFINE functional `(P - a0)·m`, so for a rational `b`
/// the condition `d(t) >= d_min` holds exactly where the numerator
/// `sum w_i B_i(t) (d_i - d_min)` is non-negative (and likewise for
/// `d_max`): each one-sided test is the convex hull of the weighted
/// control distances against zero, and the clip is the intersection of
/// the two intervals.
fn clip_to_fat_line(a: &SubSegment, b: &SubSegment, pad: f64) -> Clip {
    let Some(m) = fat_line_normal(a, b) else {
        return Clip::Interval(0.0, 1.0);
    };
    let p0 = a.pts[0];
    let (mut d_min, mut d_max) = a
        .pts
        .iter()
        .map(|&p| (p - p0).dot(m))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| {
            (lo.min(d), hi.max(d))
        });
    d_min -= pad;
    d_max += pad;

    #[allow(clippy::cast_precision_loss)]
    let degree = (b.pts.len() - 1) as f64;
    let weighted = |bound: f64| -> Vec<(f64, f64)> {
        b.pts
            .iter()
            .zip(&b.weights)
            .enumerate()
            .map(|(i, (&p, &w))| {
                #[allow(clippy::cast_precision_loss)]
                let t = i as f64 / degree;
                (t, w * ((p - p0).dot(m) - bound))
            })
            .collect()
    };
    let Some((lo0, lo1)) = convex_hull_clip(&weighted(d_min), 0.0, f64::MAX) else {
        return Clip::Empty;
    };
    let Some((hi0, hi1)) = convex_hull_clip(&weighted(d_max), -f64::MAX, 0.0) else {
        return Clip::Empty;
    };
    let t0 = lo0.max(hi0).max(0.0);
    let t1 = lo1.min(hi1).min(1.0);
    if t0 > t1 {
        Clip::Empty
    } else {
        Clip::Interval(t0, t1)
    }
}

/// Clip the convex hull of a set of (t, d) points against the horizontal
/// band `[d_min, d_max]`. Returns the parameter interval `(t_lo, t_hi)`
/// where the convex hull lies within the band, or `None` if no overlap.
fn convex_hull_clip(pts: &[(f64, f64)], d_min: f64, d_max: f64) -> Option<(f64, f64)> {
    // Build upper and lower convex hulls of the (t, d) polygon.
    let upper = upper_hull(pts);
    let lower = lower_hull(pts);

    // The feasible region is { t : hull column at t overlaps [d_min, d_max] }.
    // For a convex hull it is a single interval whose endpoints are either
    // hull-edge crossings with the band boundaries or hull vertices lying
    // inside the band. A vertex counts only if it is inside the FULL band:
    // testing a single bound here once let a vertex far above d_max extend
    // t_lo to the start of the interval, turning every clip against a
    // zero-thickness fat line into a no-op.
    let mut t_lo = f64::INFINITY;
    let mut t_hi = f64::NEG_INFINITY;

    for hull in [&upper, &lower] {
        for window in hull.windows(2) {
            let (t0, d0) = window[0];
            let (t1, d1) = window[1];
            for d in [d_min, d_max] {
                if (d0 - d) * (d1 - d) <= 0.0 {
                    let dd = d1 - d0;
                    if dd.abs() < 1e-30 {
                        // Edge lies on the line: its whole t-range is feasible.
                        t_lo = t_lo.min(t0.min(t1));
                        t_hi = t_hi.max(t0.max(t1));
                    } else {
                        let t = t0 + (d - d0) * (t1 - t0) / dd;
                        t_lo = t_lo.min(t);
                        t_hi = t_hi.max(t);
                    }
                }
            }
        }
        for &(t, di) in hull {
            if di >= d_min && di <= d_max {
                t_lo = t_lo.min(t);
                t_hi = t_hi.max(t);
            }
        }
    }

    if t_lo > t_hi {
        return None;
    }

    Some((t_lo, t_hi))
}

/// Compute the upper convex hull of a set of (t, d) points sorted by t.
fn upper_hull(pts: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut hull = Vec::with_capacity(pts.len());
    for &p in pts {
        while hull.len() >= 2 && cross_2d(hull[hull.len() - 2], hull[hull.len() - 1], p) >= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull
}

/// Compute the lower convex hull of a set of (t, d) points sorted by t.
fn lower_hull(pts: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut hull = Vec::with_capacity(pts.len());
    for &p in pts {
        while hull.len() >= 2 && cross_2d(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull
}

/// 2D cross product for convex hull computation.
fn cross_2d(o: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - o.0).mul_add(b.1 - o.1, -((a.1 - o.1) * (b.0 - o.0)))
}

/// Recursion depth at which we start checking for overlap instead of
/// continuing to subdivide fruitlessly.
const OVERLAP_CHECK_DEPTH: usize = 8;

/// Fat line thickness below which we consider the curve degenerate
/// (collinear control points). Triggers immediate overlap detection.
const DEGENERATE_FAT_LINE: f64 = 1e-12;

/// Number of samples for approximate Hausdorff distance check.
const HAUSDORFF_SAMPLES: usize = 5;

/// Fat-line padding relative to the coordinate magnitude. It absorbs the
/// rounding of the blossomed control points so a root lying exactly on a
/// slab boundary (a crossing at a window end, a tangent contact) is not
/// clipped away by a few ULPs. It is far below any modelling tolerance
/// and does not widen what counts as an intersection.
const CLIP_NOISE_PAD: f64 = 1e-12;

/// Recursive Bezier clipping core.
///
/// `swapped` records whether `a` is the second input curve, so hits and
/// overlaps are reported in `(curve1, curve2)` order whatever the role
/// alternation depth.
#[allow(clippy::too_many_lines)]
fn bezier_clip_recurse(
    a: ClipSide<'_>,
    b: ClipSide<'_>,
    swapped: bool,
    depth: usize,
    out: &mut ClipOutput,
) {
    let tolerance = out.tolerance;
    let (Some(sub_a), Some(sub_b)) = (SubSegment::new(a), SubSegment::new(b)) else {
        return;
    };

    // Base case: both windows are within the model-space tolerance (or
    // cannot be split further). Termination is judged on the windows' 3D
    // size, never on raw parameter spans: a parameter span means a
    // different model distance on every curve and scale.
    let a_done = sub_a.extent() <= tolerance || a.at_param_floor();
    let b_done = sub_b.extent() <= tolerance || b.at_param_floor();
    if a_done && b_done {
        emit_point_hit(a, b, swapped, out);
        return;
    }

    if depth >= MAX_DEPTH {
        // Before giving up, check for coincident overlap.
        if check_overlap_aligned(a, b, swapped, out) {
            return;
        }
        // Not coincident: report the polished point only if it verifies.
        emit_point_hit(a, b, swapped, out);
        return;
    }

    // Degenerate-AABB early exit. The boxes are the windows' control-point
    // boxes (they contain the curves); when a clip collapses a window to
    // zero width a box degenerates to a point, and an exact test can
    // reject a true intersection whose boxes are one ULP apart. Pad by the
    // intersection tolerance so a branch is only discarded when the
    // curves are provably farther apart than the reporting tolerance.
    if !sub_a.aabb().expanded(tolerance).intersects(sub_b.aabb()) {
        return;
    }

    // Early overlap detection: if both fat lines are degenerate (near-zero
    // thickness), the curves are collinear. Check for overlap immediately
    // instead of subdividing 2^30 times.
    if depth <= 2
        && sub_a.flatness() < DEGENERATE_FAT_LINE
        && sub_b.flatness() < DEGENERATE_FAT_LINE
        && check_overlap(a, b, swapped, out)
    {
        return;
    }

    let pad = CLIP_NOISE_PAD * sub_a.magnitude().max(sub_b.magnitude());

    // Clip B against A's fat line.
    let Clip::Interval(tb0, tb1) = clip_to_fat_line(&sub_a, &sub_b, pad) else {
        return;
    };
    let b_clipped = b.narrowed(tb0, tb1);
    if tb1 - tb0 < CLIP_THRESHOLD {
        // Good clip: recurse with swapped roles (clip A against B next).
        bezier_clip_recurse(b_clipped, a, !swapped, depth + 1, out);
        return;
    }

    // Clip A against the (possibly narrowed) B's fat line.
    let sub_b_clipped = if tb0 > 0.0 || tb1 < 1.0 {
        match SubSegment::new(b_clipped) {
            Some(s) => s,
            None => return,
        }
    } else {
        sub_b
    };
    let Clip::Interval(ta0, ta1) = clip_to_fat_line(&sub_b_clipped, &sub_a, pad) else {
        return;
    };
    let a_clipped = a.narrowed(ta0, ta1);
    if ta1 - ta0 < CLIP_THRESHOLD {
        bezier_clip_recurse(a_clipped, b_clipped, swapped, depth + 1, out);
        return;
    }

    // Neither clip was effective. At high depth, check for overlap before
    // subdividing further — coincident curves will never clip effectively.
    if depth >= OVERLAP_CHECK_DEPTH {
        if check_overlap_aligned(a_clipped, b_clipped, swapped, out) {
            return;
        }
        // A window lying wholly within tolerance of the other curve, yet
        // not coincident to second order, is one tangent contact: every
        // hit the pair could still produce is merged into one contact
        // later, so refine a single point instead of tiling the whole
        // tolerance well down to tolerance-sized windows.
        if let Some((u, v)) = tolerance_contact(a_clipped, b_clipped, tolerance) {
            if let Some(hit) = newton_refine(a.seg, b.seg, u, v, tolerance) {
                out.push_hit(swapped, hit.u1, hit.u2, hit.point);
            }
            return;
        }
    }

    // Subdivide the window that is larger in model space (and still
    // splittable).
    let split_a = if a_done {
        false
    } else if b_done {
        true
    } else {
        sub_a.extent() >= sub_b_clipped.extent()
    };
    if split_a {
        let mid = a_clipped.mid();
        bezier_clip_recurse(
            a_clipped.with_window(a_clipped.lo, mid),
            b_clipped,
            swapped,
            depth + 1,
            out,
        );
        bezier_clip_recurse(
            a_clipped.with_window(mid, a_clipped.hi),
            b_clipped,
            swapped,
            depth + 1,
            out,
        );
    } else {
        let mid = b_clipped.mid();
        bezier_clip_recurse(
            a_clipped,
            b_clipped.with_window(b_clipped.lo, mid),
            swapped,
            depth + 1,
            out,
        );
        bezier_clip_recurse(
            a_clipped,
            b_clipped.with_window(mid, b_clipped.hi),
            swapped,
            depth + 1,
            out,
        );
    }
}

/// Polish the window pair's midpoint with Newton and record it only if
/// the curves verifiably meet within the reporting tolerance there.
fn emit_point_hit(a: ClipSide<'_>, b: ClipSide<'_>, swapped: bool, out: &mut ClipOutput) {
    if let Some(hit) = newton_refine(a.seg, b.seg, a.mid(), b.mid(), out.tolerance) {
        out.push_hit(swapped, hit.u1, hit.u2, hit.point);
    }
}

/// Overlap check on the stretch the two windows actually share.
///
/// Sound fat-line clips do not keep coincident windows aligned: a clip
/// against a window that is a prefix of the other stops where the slab
/// ends, not where the shared stretch ends, so the pair keeps one
/// window's tail that the other does not cover. A Hausdorff test on such
/// a misaligned pair always fails and the pair would subdivide down to
/// the tolerance. Trim each window to the parameters of the window ends
/// (its own and the other's, projected) that lie on the other curve, then
/// run the Hausdorff test on the aligned pair.
fn check_overlap_aligned(
    a: ClipSide<'_>,
    b: ClipSide<'_>,
    swapped: bool,
    out: &mut ClipOutput,
) -> bool {
    let tolerance = out.tolerance;
    let on_other = 10.0 * tolerance;
    let mut a_params = Vec::with_capacity(4);
    let mut b_params = Vec::with_capacity(4);
    for u in [a.lo, a.hi] {
        let (v, dist) = project_onto_window(b, a.seg.evaluate(u));
        if dist <= on_other {
            a_params.push(u);
            b_params.push(v);
        }
    }
    for v in [b.lo, b.hi] {
        let (u, dist) = project_onto_window(a, b.seg.evaluate(v));
        if dist <= on_other {
            a_params.push(u);
            b_params.push(v);
        }
    }
    let (Some(a_shared), Some(b_shared)) =
        (shared_window(a, &a_params), shared_window(b, &b_params))
    else {
        return false;
    };
    check_overlap(a_shared, b_shared, swapped, out)
}

/// Samples per window for the tolerance-contact test.
const CONTACT_SAMPLES: usize = 6;

/// If one window lies wholly within `tolerance` of the other curve's
/// window (checked at [`CONTACT_SAMPLES`] + 1 points, each projected onto
/// the other window), return the closest sampled parameter pair as a
/// refinement start, in `(a, b)` order.
fn tolerance_contact(a: ClipSide<'_>, b: ClipSide<'_>, tolerance: f64) -> Option<(f64, f64)> {
    let within = |own: ClipSide<'_>, other: ClipSide<'_>| -> Option<(f64, f64, f64)> {
        let mut best: Option<(f64, f64, f64)> = None;
        for i in 0..=CONTACT_SAMPLES {
            #[allow(clippy::cast_precision_loss)]
            let u = own.at(i as f64 / CONTACT_SAMPLES as f64);
            let (v, dist) = project_onto_window(other, own.seg.evaluate(u));
            if dist > tolerance {
                return None;
            }
            if best.is_none_or(|(_, _, d)| dist < d) {
                best = Some((u, v, dist));
            }
        }
        best
    };
    if let Some((u, v, _)) = within(a, b) {
        return Some((u, v));
    }
    within(b, a).map(|(v, u, _)| (u, v))
}

/// The sub-window of `side` spanned by `params`, if it has positive length.
fn shared_window<'s>(side: ClipSide<'s>, params: &[f64]) -> Option<ClipSide<'s>> {
    let lo = params.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = params.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    (lo < hi).then(|| side.with_window(lo.max(side.lo), hi.min(side.hi)))
}

/// Closest point of `side`'s window to `p`: coarse sampling followed by
/// Newton on `(C(u) - p)·C'(u) = 0`, clamped to the window. Returns the
/// parameter and the distance.
fn project_onto_window(side: ClipSide<'_>, p: Point3) -> (f64, f64) {
    const SAMPLES: usize = 16;
    let mut best_u = side.lo;
    let mut best_d = f64::INFINITY;
    for i in 0..=SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let u = side.at(i as f64 / SAMPLES as f64);
        let d = (side.seg.evaluate(u) - p).length();
        if d < best_d {
            best_u = u;
            best_d = d;
        }
    }
    let mut u = best_u;
    for _ in 0..MAX_NEWTON {
        let ders = side.seg.derivatives(u, 2);
        let (Some(&c1), Some(&c2)) = (ders.get(1), ders.get(2)) else {
            break;
        };
        let r = side.seg.evaluate(u) - p;
        let g = r.dot(c1);
        let dg = c1.dot(c1) + r.dot(c2);
        if !(dg.is_finite() && dg > 0.0) {
            break;
        }
        let next = (u - g / dg).clamp(side.lo, side.hi);
        let d = (side.seg.evaluate(next) - p).length();
        if d < best_d {
            best_u = next;
            best_d = d;
            u = next;
        } else {
            break;
        }
    }
    (best_u, best_d)
}

/// Check if two curve segments are coincident over the given parameter
/// intervals. Samples points on curve A and checks their distance to
/// curve B. If the maximum distance (approximate Hausdorff distance)
/// is below tolerance, emits an overlap and returns `true`.
fn check_overlap(a: ClipSide<'_>, b: ClipSide<'_>, swapped: bool, out: &mut ClipOutput) -> bool {
    let tolerance = out.tolerance;
    let (seg_a, seg_b) = (a.seg, b.seg);
    let (u_a_lo, u_a_hi, u_b_lo, u_b_hi) = (a.lo, a.hi, b.lo, b.hi);
    let span_a = u_a_hi - u_a_lo;
    let span_b = u_b_hi - u_b_lo;

    // Don't classify tiny intervals as overlaps — those are point
    // intersections where both curves happen to be close near a crossing.
    // Overlap requires the curves to be coincident over a meaningful arc
    // length, so the 3D extent must exceed a multiple of tolerance.
    let pa_lo = seg_a.evaluate(u_a_lo);
    let pa_hi = seg_a.evaluate(u_a_hi);
    let arc_a = (pa_hi - pa_lo).length();
    if arc_a < tolerance * 50.0 && span_a < tolerance * 100.0 && span_b < tolerance * 100.0 {
        return false;
    }

    // Sample points on A and find closest points on B (symmetric Hausdorff).
    let mut max_dist = 0.0_f64;
    #[allow(clippy::cast_precision_loss)]
    for i in 0..=HAUSDORFF_SAMPLES {
        let t_a = u_a_lo + (u_a_hi - u_a_lo) * (i as f64) / (HAUSDORFF_SAMPLES as f64);
        let pa = seg_a.evaluate(t_a);

        let mut best_dist = f64::MAX;
        #[allow(clippy::cast_precision_loss)]
        for j in 0..=HAUSDORFF_SAMPLES {
            let t_b = u_b_lo + (u_b_hi - u_b_lo) * (j as f64) / (HAUSDORFF_SAMPLES as f64);
            let pb = seg_b.evaluate(t_b);
            best_dist = best_dist.min((pa - pb).length());
        }
        max_dist = max_dist.max(best_dist);
    }

    #[allow(clippy::cast_precision_loss)]
    for i in 0..=HAUSDORFF_SAMPLES {
        let t_b = u_b_lo + (u_b_hi - u_b_lo) * (i as f64) / (HAUSDORFF_SAMPLES as f64);
        let pb = seg_b.evaluate(t_b);

        let mut best_dist = f64::MAX;
        #[allow(clippy::cast_precision_loss)]
        for j in 0..=HAUSDORFF_SAMPLES {
            let t_a = u_a_lo + (u_a_hi - u_a_lo) * (j as f64) / (HAUSDORFF_SAMPLES as f64);
            let pa = seg_a.evaluate(t_a);
            best_dist = best_dist.min((pb - pa).length());
        }
        max_dist = max_dist.max(best_dist);
    }

    if max_dist < tolerance * 10.0 && coincident_to_second_order(a, b, tolerance) {
        out.push_overlap(swapped, a, b);
        true
    } else {
        false
    }
}

/// Unit tangent and curvature vector of `curve` at `u`.
fn tangent_and_curvature(curve: &NurbsCurve, u: f64) -> Option<(Vec3, Vec3)> {
    let ders = curve.derivatives(u, 2);
    let (&d1, &d2) = (ders.get(1)?, ders.get(2)?);
    let speed_sq = d1.length_squared();
    if !(speed_sq.is_finite() && speed_sq > 0.0) {
        return None;
    }
    let tangent = d1 * (1.0 / speed_sq.sqrt());
    let normal_part = d2 - tangent * d2.dot(tangent);
    Some((tangent, normal_part * (1.0 / speed_sq)))
}

/// Second-order coincidence test for a window pair that already passed
/// the Hausdorff test.
///
/// A tangent contact is within any tolerance of the other curve over a
/// stretch of length ~ sqrt(tolerance / relative curvature), which grows
/// with the model scale and as the tolerance loosens, so a Hausdorff test
/// alone reports tangent contacts as overlaps. Coincident curves also
/// share their tangent direction and curvature vector; a tangent contact
/// does not (its relative curvature is non-zero). Accept the overlap only
/// if the tangent-angle and curvature differences at the window middle
/// would separate the curves by no more than `tolerance` across the
/// parent segments' size.
fn coincident_to_second_order(a: ClipSide<'_>, b: ClipSide<'_>, tolerance: f64) -> bool {
    let u = a.mid();
    let (v, _) = project_onto_window(b, a.seg.evaluate(u));
    let (Some((ta, ka)), Some((tb, kb))) = (
        tangent_and_curvature(a.seg, u),
        tangent_and_curvature(b.seg, v),
    ) else {
        // No usable differential geometry (a degenerate parameterization):
        // fall back to the Hausdorff verdict.
        return true;
    };
    let size = SubSegment::new(ClipSide::new(a.seg, a.seg.domain().0, a.seg.domain().1))
        .map_or(0.0, |s| s.extent())
        .max(
            SubSegment::new(ClipSide::new(b.seg, b.seg.domain().0, b.seg.domain().1))
                .map_or(0.0, |s| s.extent()),
        );
    let direction_gap = ta.cross(tb).length() * size;
    let curvature_gap = (ka - kb).length() * size * size / 8.0;
    direction_gap <= tolerance && curvature_gap <= tolerance
}

/// Newton-Raphson refinement for a curve-curve intersection.
///
/// Given approximate parameters `(u1, u2)`, refine toward the exact
/// intersection with a 2x2 least-squares (Gauss-Newton) step from 3D.
///
/// Convergence and acceptance are separate. The iteration polishes until
/// the gap reaches floating-point resolution RELATIVE TO THE MODEL SCALE
/// (or stops improving), so hits are exact at every scale; the result is
/// then accepted only if the best gap reached is within `tolerance`. The
/// start point itself counts as an iterate, so a failed or singular step
/// (tangent contact) can never make the answer worse.
fn newton_refine(
    curve_a: &NurbsCurve,
    curve_b: &NurbsCurve,
    mut u1: f64,
    mut u2: f64,
    tolerance: f64,
) -> Option<CurveCurveHit> {
    let (a_lo, a_hi) = curve_a.domain();
    let (b_lo, b_hi) = curve_b.domain();

    let mut pa = curve_a.evaluate(u1);
    let mut f = pa - curve_b.evaluate(u2);
    let mut best = (f.length(), u1, u2, pa);

    for _ in 0..MAX_NEWTON {
        let magnitude = pa.x().abs().max(pa.y().abs()).max(pa.z().abs());
        if best.0 <= 64.0 * f64::EPSILON * magnitude {
            break;
        }

        let da = curve_a.derivatives(u1, 1);
        let db = curve_b.derivatives(u2, 1);
        let t1 = da[1]; // tangent of curve A
        let t2 = db[1]; // tangent of curve B

        // Solve the 3x2 system [t1 | -t2] * [du1, du2]^T = -f
        // via normal equations: J^T J delta = -J^T f
        let j11 = t1.dot(t1);
        let j12 = -t1.dot(t2);
        let j22 = t2.dot(t2);

        let r1 = -t1.dot(f);
        let r2 = t2.dot(f);

        // Scale-free singularity test: det / (|t1|^2 |t2|^2) = sin^2 of
        // the tangent angle.
        let det = j11 * j22 - j12 * j12;
        if !(det.is_finite() && det > f64::EPSILON * f64::EPSILON * j11 * j22) {
            break; // Parallel tangents (tangent contact) at this point.
        }

        let du1 = (j22 * r1 - j12 * r2) / det;
        let du2 = (-j12).mul_add(r1, j11 * r2) / det;

        u1 = (u1 + du1).clamp(a_lo, a_hi);
        u2 = (u2 + du2).clamp(b_lo, b_hi);
        pa = curve_a.evaluate(u1);
        f = pa - curve_b.evaluate(u2);
        let gap = f.length();
        if gap < best.0 {
            best = (gap, u1, u2, pa);
        } else {
            break; // No further progress at this precision.
        }
    }

    let (gap, u1, u2, point) = best;
    (gap <= tolerance).then_some(CurveCurveHit { u1, u2, point })
}

/// Whether two hits are the same contact: their points coincide within
/// `tolerance`, or the curves stay within `tolerance` of each other all
/// along the stretch between them (a tangent or near-tangent contact
/// found from several windows). Both tests are in model space.
fn same_contact(
    curve1: &NurbsCurve,
    curve2: &NurbsCurve,
    h: &CurveCurveHit,
    k: &CurveCurveHit,
    tolerance: f64,
) -> bool {
    if (h.point - k.point).length() <= tolerance {
        return true;
    }
    [0.25_f64, 0.5, 0.75].iter().all(|&s| {
        let u1 = s.mul_add(k.u1 - h.u1, h.u1);
        let u2 = s.mul_add(k.u2 - h.u2, h.u2);
        (curve1.evaluate(u1) - curve2.evaluate(u2)).length() <= tolerance
    })
}

/// Merge hits that are the same contact (see [`same_contact`]), keeping
/// the representative with the smallest gap between the two curves.
fn merge_duplicate_hits(
    hits: &mut Vec<CurveCurveHit>,
    curve1: &NurbsCurve,
    curve2: &NurbsCurve,
    tolerance: f64,
) {
    if hits.len() <= 1 {
        return;
    }

    hits.sort_by(|a, b| a.u1.total_cmp(&b.u1).then(a.u2.total_cmp(&b.u2)));

    let mut merged: Vec<(CurveCurveHit, f64)> = Vec::with_capacity(hits.len());
    for hit in hits.iter() {
        let gap = (curve1.evaluate(hit.u1) - curve2.evaluate(hit.u2)).length();
        if let Some(slot) = merged
            .iter_mut()
            .find(|(kept, _)| same_contact(curve1, curve2, kept, hit, tolerance))
        {
            if gap < slot.1 {
                *slot = (*hit, gap);
            }
        } else {
            merged.push((*hit, gap));
        }
    }

    *hits = merged.into_iter().map(|(hit, _)| hit).collect();
}

/// Merge adjacent or overlapping overlap intervals.
fn merge_overlaps(overlaps: &mut Vec<CurveCurveOverlap>, curve1: &NurbsCurve, tolerance: f64) {
    if overlaps.len() <= 1 {
        return;
    }
    overlaps.sort_by(|a, b| a.u1_start.total_cmp(&b.u1_start));

    let mut merged = Vec::with_capacity(overlaps.len());
    merged.push(overlaps[0]);

    for ov in overlaps.iter().skip(1) {
        if let Some(last) = merged.last_mut() {
            let slack = param_tolerance(curve1, last.u1_end, tolerance);
            if ov.u1_start <= last.u1_end + slack {
                // Extend the existing interval.
                last.u1_end = last.u1_end.max(ov.u1_end);
                last.u2_start = last.u2_start.min(ov.u2_start);
                last.u2_end = last.u2_end.max(ov.u2_end);
            } else {
                merged.push(*ov);
            }
        }
    }

    *overlaps = merged;
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    /// Create a degree-1 NURBS line from p0 to p1.
    fn make_line(p0: Point3, p1: Point3) -> NurbsCurve {
        NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![p0, p1], vec![1.0, 1.0])
            .expect("valid line")
    }

    #[test]
    fn two_lines_one_intersection() {
        // Line 1: (0,0,0) to (2,2,0) — the diagonal
        // Line 2: (0,2,0) to (2,0,0) — the anti-diagonal
        // They cross at (1,1,0).
        let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0));
        let c2 = make_line(Point3::new(0.0, 2.0, 0.0), Point3::new(2.0, 0.0, 0.0));

        let hits = curve_curve_intersect(&c1, &c2, 1e-10).expect("no error");
        assert_eq!(hits.len(), 1, "expected 1 hit, got {}", hits.len());

        let hit = &hits[0];
        assert!((hit.point.x() - 1.0).abs() < 1e-6, "x: {}", hit.point.x());
        assert!((hit.point.y() - 1.0).abs() < 1e-6, "y: {}", hit.point.y());
        assert!((hit.u1 - 0.5).abs() < 1e-6, "u1: {}", hit.u1);
        assert!((hit.u2 - 0.5).abs() < 1e-6, "u2: {}", hit.u2);
    }

    #[test]
    fn two_quarter_circles_intersection() {
        let w = std::f64::consts::FRAC_1_SQRT_2;

        // Arc 1: quarter circle centered at origin, from (1,0) to (0,1).
        let c1 = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        )
        .expect("valid arc1");

        // Arc 2: quarter circle centered at (1, 1), from (1,0) to (0,1).
        // This arc has radius 1 centered at (1,1), sweeping from 270 deg to 180 deg.
        // Control points for rational quadratic: start=(1,0), mid=(0,0) with weight w, end=(0,1).
        let c2 = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![1.0, w, 1.0],
        )
        .expect("valid arc2");

        let hits = curve_curve_intersect(&c1, &c2, 1e-8).expect("no error");
        assert!(
            !hits.is_empty(),
            "expected at least one intersection between overlapping arcs"
        );

        // Verify each hit lies on both curves.
        for hit in &hits {
            let p1 = c1.evaluate(hit.u1);
            let p2 = c2.evaluate(hit.u2);
            let dist = (p1 - p2).length();
            assert!(
                dist < 1e-4,
                "hit not on both curves: dist={dist}, u1={}, u2={}",
                hit.u1,
                hit.u2
            );
        }
    }

    #[test]
    fn disjoint_curves_no_hits() {
        // Two lines far apart.
        let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let c2 = make_line(Point3::new(0.0, 10.0, 0.0), Point3::new(1.0, 10.0, 0.0));

        let hits = curve_curve_intersect(&c1, &c2, 1e-10).expect("no error");
        assert!(hits.is_empty(), "expected no hits for disjoint curves");
    }

    #[test]
    fn parallel_lines_no_hits() {
        // Two parallel lines close but not touching.
        let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let c2 = make_line(Point3::new(0.0, 0.1, 0.0), Point3::new(1.0, 0.1, 0.0));

        let hits = curve_curve_intersect(&c1, &c2, 1e-10).expect("no error");
        assert!(hits.is_empty(), "expected no hits for parallel lines");
    }

    /// Evaluate a hull's piecewise-linear envelope at `t`, or `None` if `t`
    /// lies outside the hull's parameter span.
    fn envelope_at(hull: &[(f64, f64)], t: f64) -> Option<f64> {
        for w in hull.windows(2) {
            let (t0, d0) = w[0];
            let (t1, d1) = w[1];
            if t >= t0 && t <= t1 {
                if (t1 - t0).abs() < 1e-30 {
                    return Some(d0.max(d1));
                }
                return Some(d0 + (d1 - d0) * (t - t0) / (t1 - t0));
            }
        }
        None
    }

    /// Brute-force the feasible interval: the t-range where the hull's
    /// column overlaps the band.
    fn brute_force_feasible(pts: &[(f64, f64)], d_min: f64, d_max: f64) -> Option<(f64, f64)> {
        const N: usize = 20_001;
        let upper = upper_hull(pts);
        let lower = lower_hull(pts);
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for i in 0..N {
            #[allow(clippy::cast_precision_loss)]
            let t = i as f64 / (N - 1) as f64;
            let (Some(u), Some(l)) = (envelope_at(&upper, t), envelope_at(&lower, t)) else {
                continue;
            };
            if l.max(u) >= d_min && l.min(u) <= d_max {
                lo = lo.min(t);
                hi = hi.max(t);
            }
        }
        (lo <= hi).then_some((lo, hi))
    }

    #[test]
    fn hull_clip_matches_brute_force() {
        // `convex_hull_clip` must return EXACTLY the t-interval where the
        // hull column overlaps [d_min, d_max]:
        //
        //   - too narrow silently drops real intersections;
        //   - too wide is a no-op clip, which degrades Sederberg-Nishita to
        //     plain bisection (the defect fixed in #8: the vertex test
        //     checked one band bound, so a vertex above d_max extended t_lo
        //     back to the start of the interval).
        //
        // `perpendicular_lines_dense_scan` pins the end-to-end symptom for
        // straight lines; this pins the clip primitive itself over arbitrary
        // hull geometry.

        // Sampling resolution is 5e-5; allow a comfortable margin.
        const TOL: f64 = 1e-3;

        let mut seed: u64 = 0x2545_F491_4F6C_DD1D;
        let mut next_unit = move || {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            #[allow(clippy::cast_precision_loss)]
            let v = (seed >> 33) as f64 / f64::from(1u32 << 31);
            v - 1.0
        };

        let mut checked = 0_usize;
        for n_pts in 2..=6_u32 {
            for trial in 0..400 {
                let pts: Vec<(f64, f64)> = (0..n_pts)
                    .map(|i| (f64::from(i) / f64::from(n_pts - 1), next_unit() * 2.0))
                    .collect();
                // The band always straddles 0, matching fat_line's bounds.
                let half = next_unit().abs() * 1.5;
                let d_min = if trial % 2 == 0 { -half } else { 0.0 };
                let (d_min, d_max) = (d_min, half);

                let got = convex_hull_clip(&pts, d_min, d_max);
                let expected = brute_force_feasible(&pts, d_min, d_max);
                checked += 1;

                match (got, expected) {
                    (_, None) => {
                        // Nothing feasible at sample resolution: any interval
                        // returned must be a sub-sample sliver.
                        if let Some((lo, hi)) = got {
                            assert!(
                                hi - lo < TOL,
                                "clip returned [{lo}, {hi}] where none is feasible; \
                                 pts={pts:?} band=[{d_min}, {d_max}]"
                            );
                        }
                    }
                    (None, Some((lo, hi))) => panic!(
                        "clip returned None but [{lo}, {hi}] is feasible; \
                         pts={pts:?} band=[{d_min}, {d_max}]"
                    ),
                    (Some((got_lo, got_hi)), Some((exp_lo, exp_hi))) => {
                        assert!(
                            got_lo <= exp_lo + TOL && got_hi >= exp_hi - TOL,
                            "clip [{got_lo}, {got_hi}] is NARROWER than feasible \
                             [{exp_lo}, {exp_hi}] (drops intersections); \
                             pts={pts:?} band=[{d_min}, {d_max}]"
                        );
                        assert!(
                            got_lo >= exp_lo - TOL && got_hi <= exp_hi + TOL,
                            "clip [{got_lo}, {got_hi}] is LOOSER than feasible \
                             [{exp_lo}, {exp_hi}] (no-op clip, degrades to bisection); \
                             pts={pts:?} band=[{d_min}, {d_max}]"
                        );
                    }
                }
            }
        }
        assert!(checked >= 2000, "expected a full sweep, checked={checked}");
    }

    #[test]
    fn perpendicular_lines_dense_scan() {
        // Regression: the convex-hull clip once admitted hull vertices that
        // satisfied only one band bound, so clips against a zero-thickness
        // fat line never shrank the interval and the fallback bisection
        // pruned the true hit on 1-ulp point-AABB mismatches. Roughly 2.6%
        // of crossing positions returned zero hits, including the proptest
        // seeds 0.2484656653399068 and 0.46366748772885985.
        let mut u_values: Vec<f64> = (0..=800)
            .map(|i| 0.1 + 0.8 * f64::from(i) / 800.0)
            .collect();
        u_values.push(0.248_465_665_339_906_8);
        u_values.push(0.463_667_487_728_859_85);
        for u in u_values {
            let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
            let target = c1.evaluate(u);
            let c2 = make_line(
                Point3::new(target.x(), -1.0, 0.0),
                Point3::new(target.x(), 1.0, 0.0),
            );
            let hits = curve_curve_intersect(&c1, &c2, 1e-8).expect("no error");
            assert!(
                hits.iter().any(|h| (h.point - target).length() < 1e-4),
                "no hit near target for u={u}, hits: {hits:?}"
            );
        }
    }

    use proptest::prelude::*;

    proptest! {
        #[test]
        fn prop_known_intersection(u_param in 0.1f64..0.9) {
            // Build curve1 as a line from (0,0,0) to (2,0,0).
            let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
            let target = c1.evaluate(u_param);

            // Build curve2 as a line passing through that point vertically.
            let c2 = make_line(
                Point3::new(target.x(), -1.0, 0.0),
                Point3::new(target.x(), 1.0, 0.0),
            );

            let hits = curve_curve_intersect(&c1, &c2, 1e-8).expect("no error");
            prop_assert!(!hits.is_empty(), "expected hit near u={u_param}");

            // At least one hit should be near the target point.
            let near = hits.iter().any(|h| (h.point - target).length() < 1e-4);
            prop_assert!(near, "no hit near target {:?}, hits: {:?}", target, hits);
        }
    }

    #[test]
    fn overlapping_lines_detected() {
        // Two collinear lines that share the interval [0.5, 1.5] on x.
        // Line 1: (0,0,0) → (2,0,0)
        // Line 2: (1,0,0) → (3,0,0)
        let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
        let c2 = make_line(Point3::new(1.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));

        let result = curve_curve_intersect_full(&c1, &c2, 1e-8).expect("no error");
        assert!(
            !result.overlaps.is_empty(),
            "expected overlap, got {} hits and {} overlaps",
            result.hits.len(),
            result.overlaps.len()
        );

        // The overlap on c1 should span roughly [0.5, 1.0] (u-space).
        let ov = &result.overlaps[0];
        assert!(ov.u1_start < 0.55, "u1_start too high: {}", ov.u1_start);
        assert!(ov.u1_end > 0.95, "u1_end too low: {}", ov.u1_end);
    }

    #[test]
    fn identical_curves_full_overlap() {
        let c1 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));
        let c2 = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));

        let result = curve_curve_intersect_full(&c1, &c2, 1e-8).expect("no error");
        assert!(
            !result.overlaps.is_empty(),
            "identical curves should produce overlap, got {} hits",
            result.hits.len()
        );
    }

    /// Rational quadratic arc with the standard 90-degree middle weight.
    fn quarter_arc(p0: Point3, p1: Point3, p2: Point3, knot_hi: f64) -> NurbsCurve {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, knot_hi, knot_hi, knot_hi],
            vec![p0, p1, p2],
            vec![1.0, w, 1.0],
        )
        .expect("valid arc")
    }

    /// B10 minimized seed: two unit quarter-arcs crossing off their
    /// parameter midpoints. Re-using the parent control polygon at every
    /// depth turned each clip into a fixed shrink toward the window
    /// midpoint, excluded the root by depth 7, and the pair was pruned
    /// with no hit. The closed form is (0.75, -sqrt(1 - 0.75^2)).
    #[test]
    fn b10_minimized_seed_off_centre_arc_crossing() {
        // Unit circle at the origin, angles -90..0 degrees.
        let a = quarter_arc(
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            1.0,
        );
        // Unit circle at (1.5, 0), angles 180..270 degrees.
        let b = quarter_arc(
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(0.5, -1.0, 0.0),
            Point3::new(1.5, -1.0, 0.0),
            1.0,
        );
        let want = Point3::new(0.75, -(1.0_f64 - 0.5625).sqrt(), 0.0);
        let result = curve_curve_intersect_full(&a, &b, 1e-7).expect("no error");
        assert!(result.overlaps.is_empty());
        assert_eq!(result.hits.len(), 1, "hits: {:?}", result.hits);
        let hit = result.hits[0];
        assert!((a.evaluate(hit.u1) - want).length() < 1e-14);
        assert!((b.evaluate(hit.u2) - want).length() < 1e-14);
        // The crossing is off-centre on both arcs (the old clip's fixed
        // point was 0.5 on each).
        assert!((hit.u1 - 0.5).abs() > 0.03 && (hit.u2 - 0.5).abs() > 0.03);
    }

    /// Blossomed windows are the parent curve restricted to the window.
    #[test]
    fn sub_segment_window_matches_parent() {
        let seg = quarter_arc(
            Point3::new(3.0, -1.0, 2.0),
            Point3::new(5.0, 4.0, -1.0),
            Point3::new(-2.0, 6.0, 1.0),
            4.0,
        );
        for (lo, hi) in [(0.0, 4.0), (0.3, 1.7), (2.9, 3.1), (1.0, 1.0 + 1e-9)] {
            let sub = SubSegment::new(ClipSide::new(&seg, lo, hi)).expect("window");
            let window =
                NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], sub.pts, sub.weights)
                    .expect("valid window");
            for i in 0..=8 {
                let t = f64::from(i) / 8.0;
                let gap = (window.evaluate(t) - seg.evaluate(t.mul_add(hi - lo, lo))).length();
                assert!(gap < 1e-12, "window [{lo},{hi}] t={t}: gap {gap:.3e}");
            }
        }
    }

    /// The recursion alternates which curve it clips; hits must still be
    /// reported with `u1` on the first curve and `u2` on the second, which
    /// only shows when the two parameter domains differ.
    #[test]
    fn hit_parameters_follow_input_order_across_role_swaps() {
        let line = make_line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0));
        let arc = quarter_arc(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            10.0,
        );
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let want = Point3::new(s, s, 0.0);
        for (c1, c2) in [(&line, &arc), (&arc, &line)] {
            let hits = curve_curve_intersect(c1, c2, 1e-9).expect("no error");
            assert_eq!(hits.len(), 1, "hits: {hits:?}");
            assert!((c1.evaluate(hits[0].u1) - want).length() < 1e-12);
            assert!((c2.evaluate(hits[0].u2) - want).length() < 1e-12);
        }
    }

    /// A tangent contact is within tolerance of the other curve over a
    /// stretch that grows with the model scale; it must still be reported
    /// as one point hit on the contact, never as an overlap and never as
    /// scattered fragments.
    #[test]
    fn tangent_arcs_one_contact_at_every_scale() {
        for scale in [1e-3, 1.0, 1e3] {
            // Arc of the circle of radius `scale` at the origin (-90..0
            // degrees) and its mirror across x = scale: externally tangent
            // at (scale, 0) with a shared tangent line.
            let a = quarter_arc(
                Point3::new(0.0, -scale, 0.0),
                Point3::new(scale, -scale, 0.0),
                Point3::new(scale, 0.0, 0.0),
                1.0,
            );
            let b = quarter_arc(
                Point3::new(2.0 * scale, -scale, 0.0),
                Point3::new(scale, -scale, 0.0),
                Point3::new(scale, 0.0, 0.0),
                1.0,
            );
            for tol in [1e-7, 1e-9] {
                let result = curve_curve_intersect_full(&a, &b, tol).expect("no error");
                assert!(
                    result.overlaps.is_empty(),
                    "scale {scale} tol {tol}: overlap"
                );
                assert_eq!(result.hits.len(), 1, "scale {scale} tol {tol}");
                let p = a.evaluate(result.hits[0].u1);
                assert!((p - Point3::new(scale, 0.0, 0.0)).length() <= 1e-6 * scale);
            }
        }
    }

    /// The `bezier_clip/cubic_pair` bench pair: two x-monotone cubic
    /// S-curves crossing three times. The pre-B10 clip returned no hits at
    /// all here. The oracle counts sign changes of the y-difference of the
    /// two graphs on a dense x grid, independent of the clipper.
    #[test]
    fn wavy_cubic_pair_three_crossings() {
        let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let a = NurbsCurve::new(
            3,
            knots.clone(),
            vec![
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(-0.25, 1.25, 0.0),
                Point3::new(0.25, -1.25, 0.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            vec![1.0; 4],
        )
        .expect("valid cubic");
        let b = NurbsCurve::new(
            3,
            knots,
            vec![
                Point3::new(-1.0, 0.8, 0.0),
                Point3::new(-0.25, -1.0, 0.0),
                Point3::new(0.25, 1.0, 0.0),
                Point3::new(1.0, -0.8, 0.0),
            ],
            vec![1.0; 4],
        )
        .expect("valid cubic");

        let graph = |c: &NurbsCurve| -> Vec<Point3> {
            (0..=4000)
                .map(|i| c.evaluate(f64::from(i) / 4000.0))
                .collect()
        };
        let (pa, pb) = (graph(&a), graph(&b));
        let y_at = |pts: &[Point3], x: f64| {
            let i = pts.partition_point(|q| q.x() < x).clamp(1, pts.len() - 1);
            let (q0, q1) = (pts[i - 1], pts[i]);
            (q1.y() - q0.y()).mul_add((x - q0.x()) / (q1.x() - q0.x()), q0.y())
        };
        let mut sign_changes = 0;
        let mut prev = y_at(&pa, -1.0) - y_at(&pb, -1.0);
        for i in 1..=2000 {
            let x = f64::from(i).mul_add(1.0 / 1000.0, -1.0);
            let d = y_at(&pa, x) - y_at(&pb, x);
            if d.signum() != prev.signum() {
                sign_changes += 1;
            }
            prev = d;
        }
        assert_eq!(sign_changes, 3, "oracle must certify 3 crossings");

        let result = curve_curve_intersect_full(&a, &b, 1e-8).expect("no error");
        assert!(result.overlaps.is_empty());
        assert_eq!(result.hits.len(), 3, "hits: {:?}", result.hits);
        for hit in &result.hits {
            assert!((a.evaluate(hit.u1) - b.evaluate(hit.u2)).length() < 1e-12);
        }
    }
}
