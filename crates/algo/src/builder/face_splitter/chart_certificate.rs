//! Certified chart enclosures of exact edge curves, and the noding
//! certificate of a chart arrangement built from them.
//!
//! [`super::chart_arrangement`] traces a face partition from pieces of exact
//! curves (lines, circle and ellipse arcs, positive-weight NURBS) lifted into a
//! 2D chart: a plane's frame, or the `(u, v)` angles of a torus or cylinder.
//! Sampled polylines can miss a crossing of the exact curves between two
//! samples, or a touch of a seam or rim. This module proves instead that the
//! exact curves meet only where the arrangement has a node, or declines.
//!
//! # Spans
//!
//! Every piece is cut into spans, each carrying:
//!
//! - a **3D hull** of the exact span: the control points of a positive-weight
//!   rational Bezier span (the curve lies in their convex hull), the
//!   chord-and-tangent triangle of a circle or ellipse arc of at most `π/2`
//!   (an affine image of the unit-circle case), or a segment's ends. Hull
//!   points are expressed in the chart's own frame.
//! - **chart ranges** of the hull: exact angular ranges of the hull vertices
//!   about the axis (a convex set missing the axis subtends less than `π`; its
//!   extreme angles are at vertices), the exact distance range from the axis
//!   (point-to-hull distance), and for a torus the angle range of the
//!   `(ρ − R, z)` box about the tube centre. A hull touching the axis or the
//!   tube centre is singular for the chart: the span is subdivided, and a span
//!   that stays singular declines.
//! - a **velocity cone**: an interval-arithmetic enclosure (outward rounded)
//!   of the chart velocity `Dφ(c(t)) c'(t)` over the span, from the hull
//!   ranges and an enclosure of `c'` (the hodograph hull of a Bezier span; for
//!   a rational span `c' ∝ A' − c w'` with `w > 0`; the derivative arc of a
//!   conic). The cone is the angle interval of that box; it must miss the
//!   origin and be at most `π/2` wide.
//!
//! The chart image of the span then lies in `(S + K) ∩ (E − K)`, where `S`
//! and `E` are the lifted chart images of the span ends and `K` the cone: the
//! displacement from either end is an integral of velocities in `K`. That
//! convex quadrilateral is the span's **enclosure**; its width across the
//! chord is second order in the span length, so no projection or Lipschitz
//! estimate of a 3D chord is involved. Lifting is exact too: a span whose
//! angular ranges are under `π/2` wide changes each angle by less than `π`, so
//! unwrapping its end next to its start picks the continuous lift.
//!
//! # The certificate
//!
//! Two spans whose enclosures are disjoint (beyond a rounding guard) share no
//! point. Two spans that touch are accepted only at one *apex*:
//!
//! - consecutive spans of one piece, at their common break point `B`: the
//!   earlier span lies in `B − K₁`, the later in `B + K₂`, which meet only at
//!   `B` when the two direction intervals are disjoint;
//! - two piece ends at one node `N`: each end span lies in its end point `E`
//!   plus its cone, and `E` misses `N` by the node merge `d`. Outside the
//!   ball of radius `ε = d·(1 + 1/sin β)` about `N` the direction from `N`
//!   stays within `β` of the cone; with the two leaving sectors a gap `3β`
//!   apart, the curves can meet only inside that ball, which is the node.
//!
//! A pair touching anywhere else, or at two apexes, is a conflict: both spans
//! are bisected and the pair is tested again, up to a fixed budget of rounds,
//! depth and spans. A real crossing, touch or tangency away from a node never
//! separates, so it exhausts the budget and the arrangement declines.
//!
//! The certified polyline of a piece runs through its node, its break points
//! and its other node. Every polyline segment lies in its span's enclosure
//! (the end enclosures include the node), and the chord directions lie in the
//! cones (widened by the node mismatch at an end), so the polylines satisfy
//! the same certificate: they form a plane graph with the same rotation
//! system at every node and break point as the exact curves, and every point
//! outside the enclosures has the same winding number about an exact cycle
//! and its polyline. Regions, holes and interior points traced on the
//! polylines are therefore regions, holes and interior points of the exact
//! partition, provided each interior point lies outside every enclosure.

use std::cmp::Ordering;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use remus_math::compact_sort::CompactSort;

use remus_math::nurbs::curve_to_bezier_segments;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::edge::EdgeCurve;

use super::super::plane_frame::PlaneFrame;
use super::chart_arrangement::{GraphEdge, Node};
use crate::periodic_chart::ChartCarrier;

/// Decline with the reason's source line in the debug log.
macro_rules! decline {
    () => {{
        log::debug!("chart_certificate: declined at line {}", line!());
        return None;
    }};
}

/// Widest cone and angular range a span may carry.
const MAX_WIDTH: f64 = FRAC_PI_2;
/// Widest arc a conic span may cover (its hull triangle needs `< π`).
const MAX_ARC: f64 = FRAC_PI_4;
/// Refinement rounds before declining.
const MAX_ROUNDS: usize = 40;
/// Bisections of one span before declining.
const MAX_DEPTH: u32 = 48;
/// Total spans of one arrangement before declining.
const MAX_SPANS: usize = 250_000;
/// Candidate enclosure pairs across all refinement rounds. Coincident
/// columns otherwise make the x-sorted overlap scan quadratic in the span cap.
const MAX_PAIR_CHECKS: usize = 2_000_000;
/// Floor of a cone's width, so its enclosure is a proper quadrilateral.
const MIN_CONE: f64 = 1e-10;
/// Angle padding covering the rounding of `atan2`.
const ANGLE_PAD: f64 = 1e-13;
/// Highest NURBS degree whose hull is handled.
const MAX_DEGREE: usize = 9;

// ---------------------------------------------------------------------------
// Interval arithmetic (outward rounded)
// ---------------------------------------------------------------------------

/// A closed interval with outward-rounded operations.
#[derive(Clone, Copy, Debug)]
pub(super) struct Iv {
    lo: f64,
    hi: f64,
}

impl Iv {
    /// The interval `[lo, hi]`, widened by one ulp each way.
    fn new(lo: f64, hi: f64) -> Self {
        Self {
            lo: lo.next_down(),
            hi: hi.next_up(),
        }
    }

    fn add(self, o: Self) -> Self {
        Self {
            lo: (self.lo + o.lo).next_down(),
            hi: (self.hi + o.hi).next_up(),
        }
    }

    fn sub(self, o: Self) -> Self {
        Self {
            lo: (self.lo - o.hi).next_down(),
            hi: (self.hi - o.lo).next_up(),
        }
    }

    fn sub_f(self, x: f64) -> Self {
        self.sub(Self { lo: x, hi: x })
    }

    fn mul(self, o: Self) -> Self {
        let c = [
            self.lo * o.lo,
            self.lo * o.hi,
            self.hi * o.lo,
            self.hi * o.hi,
        ];
        let lo = c.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = c.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        Self {
            lo: lo.next_down(),
            hi: hi.next_up(),
        }
    }

    /// The square, never negative.
    fn sqr(self) -> Self {
        let (a, b) = (self.lo.abs(), self.hi.abs());
        if self.lo <= 0.0 && self.hi >= 0.0 {
            Self {
                lo: 0.0,
                hi: (a.max(b) * a.max(b)).next_up(),
            }
        } else {
            let (m, n) = (a.min(b), a.max(b));
            Self {
                lo: (m * m).next_down().max(0.0),
                hi: (n * n).next_up(),
            }
        }
    }

    /// The quotient by an interval of one strict sign; `None` otherwise.
    fn div(self, o: Self) -> Option<Self> {
        if !(o.lo > 0.0 || o.hi < 0.0) {
            return None;
        }
        let c = [
            self.lo / o.lo,
            self.lo / o.hi,
            self.hi / o.lo,
            self.hi / o.hi,
        ];
        let lo = c.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = c.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        Some(Self {
            lo: lo.next_down(),
            hi: hi.next_up(),
        })
    }

    fn contains_zero(self) -> bool {
        self.lo <= 0.0 && self.hi >= 0.0
    }

    fn finite(self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }
}

/// Bounding interval of values, padded by `pad`.
fn range_of(values: impl Iterator<Item = f64>, pad: f64) -> Iv {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for x in values {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    Iv::new(lo - pad, hi + pad)
}

// ---------------------------------------------------------------------------
// Small 2D helpers
// ---------------------------------------------------------------------------

fn cross2(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn seg_dist(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let ab = (b.0 - a.0, b.1 - a.1);
    let len2 = ab.0 * ab.0 + ab.1 * ab.1;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * ab.0 + (p.1 - a.1) * ab.1) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let q = (a.0 + ab.0 * t - p.0, a.1 + ab.1 * t - p.1);
    q.0.hypot(q.1)
}

/// Whether `p` lies in the closed triangle `abc` (any orientation). A
/// degenerate triangle holds nothing: its points lie on its edges, which
/// the caller measures as segments.
fn in_triangle(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    if cross2((b.0 - a.0, b.1 - a.1), (c.0 - a.0, c.1 - a.1)) == 0.0 {
        return false;
    }
    let d1 = cross2((b.0 - a.0, b.1 - a.1), (p.0 - a.0, p.1 - a.1));
    let d2 = cross2((c.0 - b.0, c.1 - b.1), (p.0 - b.0, p.1 - b.1));
    let d3 = cross2((a.0 - c.0, a.1 - c.1), (p.0 - c.0, p.1 - c.1));
    let neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(neg && pos)
}

/// Distance from the origin to the convex hull of `pts` (0 when inside).
fn origin_hull_distance(pts: &[(f64, f64)]) -> f64 {
    let o = (0.0, 0.0);
    let n = pts.len();
    for i in 0..n {
        for j in i + 1..n {
            for k in j + 1..n {
                if in_triangle(o, pts[i], pts[j], pts[k]) {
                    return 0.0;
                }
            }
        }
    }
    let mut d = f64::INFINITY;
    for i in 0..n {
        d = d.min(pts[i].0.hypot(pts[i].1));
        for j in i + 1..n {
            d = d.min(seg_dist(o, pts[i], pts[j]));
        }
    }
    d
}

/// Angle interval subtended at the origin by a point set missing it (the
/// caller guarantees the set's hull misses the origin), padded.
fn angle_range(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    let r = *pts.first()?;
    let base = r.1.atan2(r.0);
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for &p in pts {
        let d = cross2(r, p).atan2(r.0 * p.0 + r.1 * p.1);
        lo = lo.min(d);
        hi = hi.max(d);
    }
    Some((base + lo - ANGLE_PAD, base + hi + ANGLE_PAD))
}

/// Angle interval of the box `x × y`, which must miss the origin.
fn box_angle_range(x: Iv, y: Iv) -> Option<(f64, f64)> {
    if !(x.finite() && y.finite()) || (x.contains_zero() && y.contains_zero()) {
        return None;
    }
    angle_range(&[(x.lo, y.lo), (x.hi, y.lo), (x.hi, y.hi), (x.lo, y.hi)])
}

/// Circular gap between two angle intervals each narrower than `π`
/// (negative when they overlap).
fn angular_gap(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (wa, wb) = (a.1 - a.0, b.1 - b.0);
    let d = (b.0 - a.0).rem_euclid(TAU);
    (d - wa).min(TAU - wb - d)
}

/// The convex hull of a few points, counter-clockwise (monotone chain).
fn convex_hull(mut pts: Vec<Point2>) -> Vec<Point2> {
    pts.compact_sort_by(|a, b| a.x().total_cmp(&b.x()).then(a.y().total_cmp(&b.y())));
    pts.dedup_by(|a, b| a.x().total_cmp(&b.x()).is_eq() && a.y().total_cmp(&b.y()).is_eq());
    if pts.len() < 3 {
        return pts;
    }
    let turn = |o: Point2, a: Point2, b: Point2| {
        (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x())
    };
    let mut lower: Vec<Point2> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && turn(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<Point2> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && turn(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Whether two convex polygons (counter-clockwise, possibly degenerate)
/// come within `guard` of each other (separating-axis test).
fn polygons_touch(a: &[Point2], b: &[Point2], guard: f64) -> bool {
    let mut axes: Vec<(f64, f64)> = Vec::with_capacity(a.len() + b.len() + 2);
    for poly in [a, b] {
        let n = poly.len();
        for i in 0..n {
            let (p, q) = (poly[i], poly[(i + 1) % n]);
            let e = (q.x() - p.x(), q.y() - p.y());
            if e.0 != 0.0 || e.1 != 0.0 {
                axes.push((-e.1, e.0));
                if n < 3 {
                    axes.push(e);
                }
            }
        }
    }
    if a.len() == 1 && b.len() == 1 {
        axes.push((b[0].x() - a[0].x(), b[0].y() - a[0].y()));
    }
    for ax in axes {
        let len = ax.0.hypot(ax.1);
        if len == 0.0 || !len.is_finite() {
            continue;
        }
        let proj = |poly: &[Point2]| {
            poly.iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                    let s = (p.x() * ax.0 + p.y() * ax.1) / len;
                    (lo.min(s), hi.max(s))
                })
        };
        let (pa, pb) = (proj(a), proj(b));
        if pa.1 < pb.0 - guard || pb.1 < pa.0 - guard {
            return false;
        }
    }
    true
}

/// Distance from `p` to a convex counter-clockwise polygon (0 inside).
pub(super) fn polygon_distance(p: Point2, poly: &[Point2]) -> f64 {
    let n = poly.len();
    let q = (p.x(), p.y());
    if n >= 3 {
        let inside = (0..n).all(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            cross2((b.x() - a.x(), b.y() - a.y()), (q.0 - a.x(), q.1 - a.y())) >= 0.0
        });
        if inside {
            return 0.0;
        }
    }
    let mut d = f64::INFINITY;
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        d = d.min(seg_dist(q, (a.x(), a.y()), (b.x(), b.y())));
    }
    d
}

// ---------------------------------------------------------------------------
// The chart map
// ---------------------------------------------------------------------------

/// A chart of a face's carrier, in the carrier's own frame.
#[derive(Clone, Debug)]
pub(super) enum ChartMap {
    /// A plane frame: chart `(x, y)`.
    Plane { origin: Point3, axes: [Vec3; 3] },
    /// A cylinder: chart `(atan2(y, x), z)`.
    Cylinder { origin: Point3, axes: [Vec3; 3] },
    /// A ring torus: chart `(atan2(y, x), atan2(z, ρ − R))`.
    Torus {
        origin: Point3,
        axes: [Vec3; 3],
        major: f64,
    },
}

/// What a span's hull gives in the chart.
struct Ranges {
    /// Local-frame box of the hull.
    lbox: [Iv; 3],
    /// Distance range from the axis (torus and cylinder).
    rho: Iv,
    /// Raw angular ranges of the chart coordinates that are angles.
    ang: [Option<(f64, f64)>; 2],
}

impl ChartMap {
    /// The chart of a plane frame.
    pub(super) fn plane(frame: &PlaneFrame) -> Self {
        let (u, v) = (frame.u_axis(), frame.v_axis());
        Self::Plane {
            origin: frame.origin(),
            axes: [u, v, u.cross(v)],
        }
    }

    /// The chart of a torus or cylinder carrier.
    pub(super) fn of_carrier(carrier: &ChartCarrier) -> Self {
        match carrier {
            ChartCarrier::Torus(t) => Self::Torus {
                origin: t.center(),
                axes: [t.x_axis(), t.y_axis(), t.z_axis()],
                major: t.major_radius(),
            },
            ChartCarrier::Cylinder(c) => Self::Cylinder {
                origin: c.origin(),
                axes: [c.x_axis(), c.y_axis(), c.axis()],
            },
        }
    }

    const fn frame(&self) -> (Point3, &[Vec3; 3]) {
        match self {
            Self::Plane { origin, axes }
            | Self::Cylinder { origin, axes }
            | Self::Torus { origin, axes, .. } => (*origin, axes),
        }
    }

    /// Which chart coordinates are angles.
    pub(super) const fn angular(&self) -> [bool; 2] {
        match self {
            Self::Plane { .. } => [false, false],
            Self::Cylinder { .. } => [true, false],
            Self::Torus { .. } => [true, true],
        }
    }

    fn local(&self, p: Point3) -> [f64; 3] {
        let (o, axes) = self.frame();
        let d = p - o;
        [d.dot(axes[0]), d.dot(axes[1]), d.dot(axes[2])]
    }

    fn local_vec(&self, v: Vec3) -> [f64; 3] {
        let (_, axes) = self.frame();
        [v.dot(axes[0]), v.dot(axes[1]), v.dot(axes[2])]
    }

    /// Raw chart point (angles in `(−π, π]`) of a local point.
    fn chart_raw(&self, l: [f64; 3]) -> Option<(f64, f64)> {
        let out = match self {
            Self::Plane { .. } => (l[0], l[1]),
            Self::Cylinder { .. } => (l[1].atan2(l[0]), l[2]),
            Self::Torus { major, .. } => (l[1].atan2(l[0]), l[2].atan2(l[0].hypot(l[1]) - major)),
        };
        (out.0.is_finite() && out.1.is_finite()).then_some(out)
    }

    /// Lift a raw chart point next to a lifted one.
    fn unwrap_near(&self, raw: (f64, f64), near: Point2) -> Point2 {
        let [au, av] = self.angular();
        let fix = |x: f64, n: f64, angular: bool| {
            if angular {
                x + TAU * ((n - x) / TAU).round()
            } else {
                x
            }
        };
        Point2::new(fix(raw.0, near.x(), au), fix(raw.1, near.y(), av))
    }

    /// The chart ranges of a hull, or `None` where the chart is singular.
    fn ranges(&self, hull: &[[f64; 3]]) -> Option<Ranges> {
        let scale = hull
            .iter()
            .map(|p| p[0].abs().max(p[1].abs()).max(p[2].abs()))
            .fold(0.0, f64::max);
        let pad = 1e-14 * (1.0 + scale);
        let lbox = [
            range_of(hull.iter().map(|p| p[0]), pad),
            range_of(hull.iter().map(|p| p[1]), pad),
            range_of(hull.iter().map(|p| p[2]), pad),
        ];
        if !lbox.iter().all(|b| b.finite()) {
            return None;
        }
        match self {
            Self::Plane { .. } => Some(Ranges {
                lbox,
                rho: Iv { lo: 0.0, hi: 0.0 },
                ang: [None, None],
            }),
            Self::Cylinder { .. } | Self::Torus { .. } => {
                let xy: Vec<(f64, f64)> = hull.iter().map(|p| (p[0], p[1])).collect();
                let near = origin_hull_distance(&xy);
                if near <= 8.0 * pad {
                    return None;
                }
                let far = xy.iter().map(|p| p.0.hypot(p.1)).fold(0.0, f64::max);
                let rho = Iv::new(near - pad, far + pad);
                let u = angle_range(&xy)?;
                let v = if let Self::Torus { major, .. } = self {
                    Some(box_angle_range(rho.sub_f(*major), lbox[2])?)
                } else {
                    None
                };
                Some(Ranges {
                    lbox,
                    rho,
                    ang: [Some(u), v],
                })
            }
        }
    }

    /// The velocity cone (angle interval of the chart velocity) of a span
    /// with hull ranges `r` and local velocity enclosure `vel`.
    fn cone(&self, r: &Ranges, vel: [Iv; 3]) -> Option<(f64, f64)> {
        let [x, y, z] = r.lbox;
        let [dx, dy, dz] = vel;
        let (du, dv) = match self {
            Self::Plane { .. } => (dx, dy),
            Self::Cylinder { .. } | Self::Torus { .. } => {
                let num = x.mul(dy).sub(y.mul(dx));
                let du = num.div(r.rho.sqr())?;
                if let Self::Torus { major, .. } = self {
                    let drho = x.mul(dx).add(y.mul(dy)).div(r.rho)?;
                    let s = r.rho.sub_f(*major);
                    let den = s.sqr().add(z.sqr());
                    let dv = s.mul(dz).sub(z.mul(drho)).div(den)?;
                    (du, dv)
                } else {
                    (du, dz)
                }
            }
        };
        let (lo, hi) = box_angle_range(du, dv)?;
        let w = hi - lo;
        if w < MIN_CONE {
            let m = 0.5 * (lo + hi);
            return Some((m - 0.5 * MIN_CONE, m + 0.5 * MIN_CONE));
        }
        Some((lo, hi))
    }
}

// ---------------------------------------------------------------------------
// Span geometry (local frame)
// ---------------------------------------------------------------------------

/// The exact geometry of one span, in the chart's local frame.
#[derive(Clone, Debug)]
enum Geom {
    /// A segment.
    Seg([f64; 3], [f64; 3]),
    /// `c + a cos t + b sin t`, `t` from `t0` to `t1` (either order).
    Arc {
        c: [f64; 3],
        a: [f64; 3],
        b: [f64; 3],
        t0: f64,
        t1: f64,
    },
    /// A rational Bezier span in homogeneous coordinates `(w x, w y, w z, w)`.
    Bez(Vec<[f64; 4]>),
}

fn lerp3(p: [f64; 3], q: [f64; 3], s: f64) -> [f64; 3] {
    [
        p[0] + (q[0] - p[0]) * s,
        p[1] + (q[1] - p[1]) * s,
        p[2] + (q[2] - p[2]) * s,
    ]
}

fn dehom(q: [f64; 4]) -> [f64; 3] {
    [q[0] / q[3], q[1] / q[3], q[2] / q[3]]
}

fn hom_lerp(a: [f64; 4], b: [f64; 4], t: f64) -> [f64; 4] {
    hom_mix(a, b, 1.0 - t, t)
}

fn hom_mix(a: [f64; 4], b: [f64; 4], weight_a: f64, weight_b: f64) -> [f64; 4] {
    if weight_b <= 0.0 {
        return a;
    }
    if weight_a <= 0.0 {
        return b;
    }
    std::array::from_fn(|i| weight_b.mul_add(b[i], weight_a * a[i]))
}

/// Split a homogeneous Bezier at `s` (de Casteljau).
fn bez_split(cps: &[[f64; 4]], s: f64) -> (Vec<[f64; 4]>, Vec<[f64; 4]>) {
    bez_split_weights(cps, 1.0 - s, s)
}

/// The two coefficients are supplied independently when trimming on a knot
/// span: subtracting a rounded ratio near one loses the small coefficient.
fn bez_split_weights(
    cps: &[[f64; 4]],
    weight_a: f64,
    weight_b: f64,
) -> (Vec<[f64; 4]>, Vec<[f64; 4]>) {
    let n = cps.len();
    let mut work = cps.to_vec();
    let mut left = Vec::with_capacity(n);
    let mut right = Vec::with_capacity(n);
    left.push(work[0]);
    right.push(work[n - 1]);
    for level in 1..n {
        for i in 0..n - level {
            work[i] = hom_mix(work[i], work[i + 1], weight_a, weight_b);
        }
        left.push(work[0]);
        right.push(work[n - 1 - level]);
    }
    right.reverse();
    (left, right)
}

fn bez_eval(cps: &[[f64; 4]], s: f64) -> [f64; 3] {
    let mut work = cps.to_vec();
    let n = work.len();
    for level in 1..n {
        for i in 0..n - level {
            work[i] = hom_lerp(work[i], work[i + 1], s);
        }
    }
    dehom(work[0])
}

impl Geom {
    fn conic_at(c: [f64; 3], a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
        let (s, co) = t.sin_cos();
        [
            c[0] + a[0] * co + b[0] * s,
            c[1] + a[1] * co + b[1] * s,
            c[2] + a[2] * co + b[2] * s,
        ]
    }

    fn at(&self, s: f64) -> [f64; 3] {
        match self {
            Self::Seg(p, q) => lerp3(*p, *q, s),
            Self::Arc { c, a, b, t0, t1 } => Self::conic_at(*c, *a, *b, t0 + (t1 - t0) * s),
            Self::Bez(cps) => {
                if s <= 0.0 {
                    dehom(cps[0])
                } else if s >= 1.0 {
                    dehom(cps[cps.len() - 1])
                } else {
                    bez_eval(cps, s)
                }
            }
        }
    }

    fn start(&self) -> [f64; 3] {
        self.at(0.0)
    }

    fn end(&self) -> [f64; 3] {
        self.at(1.0)
    }

    fn split(&self) -> (Self, Self) {
        match self {
            Self::Seg(p, q) => {
                let m = lerp3(*p, *q, 0.5);
                (Self::Seg(*p, m), Self::Seg(m, *q))
            }
            Self::Arc { c, a, b, t0, t1 } => {
                let m = 0.5 * (t0 + t1);
                (
                    Self::Arc {
                        c: *c,
                        a: *a,
                        b: *b,
                        t0: *t0,
                        t1: m,
                    },
                    Self::Arc {
                        c: *c,
                        a: *a,
                        b: *b,
                        t0: m,
                        t1: *t1,
                    },
                )
            }
            Self::Bez(cps) => {
                let (l, r) = bez_split(cps, 0.5);
                (Self::Bez(l), Self::Bez(r))
            }
        }
    }

    /// Points whose convex hull contains the span; `None` when the span is
    /// too wide (an arc) or not a positive-weight Bezier.
    fn hull(&self) -> Option<Vec<[f64; 3]>> {
        match self {
            Self::Seg(p, q) => Some(vec![*p, *q]),
            Self::Arc { c, a, b, t0, t1 } => {
                let w = (t1 - t0).abs();
                if w > MAX_ARC {
                    return None;
                }
                let m = 0.5 * (t0 + t1);
                let k = 1.0 / (0.5 * w).cos();
                let t = Self::conic_at([0.0; 3], *a, *b, m);
                Some(vec![
                    Self::conic_at(*c, *a, *b, *t0),
                    Self::conic_at(*c, *a, *b, *t1),
                    [c[0] + t[0] * k, c[1] + t[1] * k, c[2] + t[2] * k],
                ])
            }
            Self::Bez(cps) => {
                if cps.iter().any(|q| !(q[3] > 0.0 && q[3].is_finite())) {
                    return None;
                }
                Some(cps.iter().map(|&q| dehom(q)).collect())
            }
        }
    }

    /// An enclosure of the span's velocity (up to a positive factor) in the
    /// traversal direction, given its hull ranges.
    fn velocity(&self, r: &Ranges) -> Option<[Iv; 3]> {
        let pad = |v: &[[f64; 3]]| {
            let s = v
                .iter()
                .map(|p| p[0].abs().max(p[1].abs()).max(p[2].abs()))
                .fold(0.0, f64::max);
            let pad = 1e-14 * s;
            [
                range_of(v.iter().map(|p| p[0]), pad),
                range_of(v.iter().map(|p| p[1]), pad),
                range_of(v.iter().map(|p| p[2]), pad),
            ]
        };
        match self {
            Self::Seg(p, q) => Some(pad(&[[q[0] - p[0], q[1] - p[1], q[2] - p[2]]])),
            Self::Arc { a, b, t0, t1, .. } => {
                // The derivative `−a sin t + b cos t` is the same conic a
                // quarter turn on; its arc's hull is the same triangle.
                let sign = if t1 >= t0 { 1.0 } else { -1.0 };
                let w = (t1 - t0).abs();
                let k = 1.0 / (0.5 * w).cos();
                let d = |t: f64, f: f64| {
                    let (s, c) = t.sin_cos();
                    [
                        sign * f * (-a[0] * s + b[0] * c),
                        sign * f * (-a[1] * s + b[1] * c),
                        sign * f * (-a[2] * s + b[2] * c),
                    ]
                };
                Some(pad(&[d(*t0, 1.0), d(*t1, 1.0), d(0.5 * (t0 + t1), k)]))
            }
            Self::Bez(cps) => {
                // c' = (A' − c w') / w with w > 0: the direction of A' − c w'.
                let diffs: Vec<[f64; 4]> = cps
                    .windows(2)
                    .map(|q| {
                        [
                            q[1][0] - q[0][0],
                            q[1][1] - q[0][1],
                            q[1][2] - q[0][2],
                            q[1][3] - q[0][3],
                        ]
                    })
                    .collect();
                if diffs.is_empty() {
                    return None;
                }
                let a = pad(&diffs.iter().map(|d| [d[0], d[1], d[2]]).collect::<Vec<_>>());
                let wmax = diffs.iter().map(|d| d[3].abs()).fold(0.0, f64::max);
                if wmax == 0.0 {
                    return Some(a);
                }
                let wd = range_of(diffs.iter().map(|d| d[3]), 1e-14 * wmax);
                Some([
                    a[0].sub(r.lbox[0].mul(wd)),
                    a[1].sub(r.lbox[1].mul(wd)),
                    a[2].sub(r.lbox[2].mul(wd)),
                ])
            }
        }
    }
}

/// A span is usable when its hull is valid, its chart ranges are regular and
/// narrow, and its velocity cone is regular and narrow.
fn analyse(map: &ChartMap, g: &Geom) -> Option<(f64, f64)> {
    let hull = g.hull()?;
    let r = map.ranges(&hull)?;
    for a in r.ang.iter().flatten() {
        if a.1 - a.0 > MAX_WIDTH {
            return None;
        }
    }
    let cone = map.cone(&r, g.velocity(&r)?)?;
    (cone.1 - cone.0 <= MAX_WIDTH).then_some(cone)
}

/// Whether the certificate handles this curve: a circle or ellipse arc, or a
/// NURBS curve of degree 1 to 9 with finite positive weights. Lines are
/// handled where a chart takes them; hyperbolic and parabolic arcs and other
/// NURBS are outside the family.
pub(super) fn exact_curve_supported(curve: &EdgeCurve) -> bool {
    match curve {
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => true,
        EdgeCurve::NurbsCurve(n) => {
            (1..=MAX_DEGREE).contains(&n.degree())
                && n.weights().iter().all(|w| *w > 0.0 && w.is_finite())
        }
        EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => false,
    }
}

// ---------------------------------------------------------------------------
// Pieces as spans
// ---------------------------------------------------------------------------

/// One Bezier span of a NURBS curve: its knot interval and homogeneous
/// local-frame control points.
type BezierSpan = (f64, f64, Vec<[f64; 4]>);
type BezierSpans = Vec<BezierSpan>;

fn same_parameter(a: f64, b: f64) -> bool {
    a.partial_cmp(&b) == Some(Ordering::Equal)
}

fn same_point(a: [f64; 3], b: [f64; 3]) -> bool {
    if !a.iter().chain(&b).all(|c| c.is_finite()) {
        return false;
    }
    let scale = a.iter().chain(&b).fold(1.0_f64, |s, c| s.max(c.abs()));
    a.into_iter()
        .zip(b)
        .fold(0.0_f64, |d, (x, y)| d.hypot(x - y))
        <= 1e-12 * scale
}

/// Bezier spans of NURBS curves, cached by curve identity.
#[derive(Default)]
struct BezierCache {
    entries: Vec<((usize, usize, usize), BezierSpans)>,
}

impl BezierCache {
    fn spans(
        &mut self,
        map: &ChartMap,
        n: &remus_math::nurbs::NurbsCurve,
    ) -> Option<&[BezierSpan]> {
        let key = (
            n.control_points().as_ptr() as usize,
            n.control_points().len(),
            n.knots().as_ptr() as usize,
        );
        if let Some(i) = self.entries.iter().position(|(k, _)| *k == key) {
            return Some(&self.entries[i].1);
        }
        if n.degree() == 0 || n.degree() > MAX_DEGREE {
            return None;
        }
        // Only positive weights: a rational Bezier lies in the hull of its
        // control points only then.
        if n.weights().iter().any(|w| !(*w > 0.0 && w.is_finite())) {
            return None;
        }
        // The shared decomposer compares knots with an absolute 1e-15
        // epsilon. Distinct knots inside that band must not be identified
        // for a certificate of the original curve.
        if n.knots()
            .windows(2)
            .any(|w| w[1] > w[0] && w[1] - w[0] <= 1e-15)
        {
            return None;
        }
        let (d0, d1) = n.domain();
        if !(d0.is_finite() && d1.is_finite() && d0 < d1) {
            return None;
        }
        let segments = curve_to_bezier_segments(n).ok()?;
        let mut out = Vec::with_capacity(segments.len());
        let mut covered = d0;
        let mut previous_end = None;
        for seg in &segments {
            let (k0, k1) = seg.domain();
            if !same_parameter(k0, covered) || !k1.is_finite() || k1 <= k0 || k1 > d1 {
                return None;
            }
            let weight_scale = seg.weights().iter().copied().fold(0.0_f64, f64::max);
            let cps: Vec<[f64; 4]> = seg
                .control_points()
                .iter()
                .zip(seg.weights())
                .map(|(p, &w)| {
                    let l = map.local(*p);
                    let w = w / weight_scale;
                    [l[0] * w, l[1] * w, l[2] * w, w]
                })
                .collect();
            if cps.len() != n.degree() + 1
                || cps
                    .iter()
                    .any(|q| !q.iter().all(|c| c.is_finite()) || !q[3].is_normal() || q[3] <= 0.0)
            {
                return None;
            }
            let first = dehom(cps[0]);
            let last = dehom(cps[cps.len() - 1]);
            if !same_point(first, map.local(n.evaluate(k0)))
                || !same_point(last, map.local(n.evaluate(k1)))
                || previous_end.is_some_and(|p| !same_point(p, first))
            {
                return None;
            }
            covered = k1;
            previous_end = Some(last);
            out.push((k0, k1, cps));
        }
        if !same_parameter(covered, d1) || out.is_empty() {
            return None;
        }
        self.entries.push((key, out));
        Some(&self.entries.last()?.1)
    }
}

/// The exact geometry of a piece as spans in traversal order.
fn piece_geoms(map: &ChartMap, cache: &mut BezierCache, e: &GraphEdge) -> Option<Vec<Geom>> {
    let p = &e.piece;
    let (ta, tb) = (p.t_a, p.t_b);
    match &p.curve {
        EdgeCurve::Line => Some(vec![Geom::Seg(map.local(p.p3_a), map.local(p.p3_b))]),
        EdgeCurve::Circle(c) => {
            let r = c.radius();
            let a = map.local_vec(c.u_axis() * r);
            let b = map.local_vec(c.v_axis() * r);
            conic_geoms(map.local(c.center()), a, b, ta, tb)
        }
        EdgeCurve::Ellipse(el) => {
            let a = map.local_vec(el.u_axis() * el.semi_major());
            let b = map.local_vec(el.v_axis() * el.semi_minor());
            conic_geoms(map.local(el.center()), a, b, ta, tb)
        }
        EdgeCurve::NurbsCurve(n) => {
            if !(ta.is_finite() && tb.is_finite()) || (tb - ta).abs() < f64::MIN_POSITIVE {
                return None;
            }
            let (lo, hi) = (ta.min(tb), ta.max(tb));
            let (d0, d1) = n.domain();
            let slack = 1e-12 * (1.0 + d0.abs().max(d1.abs()));
            if lo < d0 - slack || hi > d1 + slack {
                return None;
            }
            let spans = cache.spans(map, n)?;
            let mut out = Vec::new();
            let mut covered = lo.max(d0);
            for (k0, k1, cps) in spans {
                if *k1 <= lo || *k0 >= hi {
                    continue;
                }
                let from = lo.max(*k0);
                let to = hi.min(*k1);
                if !same_parameter(from, covered) {
                    return None;
                }
                let len = k1 - k0;
                let kept = to - from;
                if !len.is_finite() || !kept.is_finite() || kept / len <= 1e-14 {
                    // A tiny parameter fraction can contain an arbitrarily
                    // large geometric excursion under positive reweighting.
                    return None;
                }
                let mut seg = cps.clone();
                if to < *k1 {
                    let (a, b) = ((k1 - to) / len, (to - k0) / len);
                    if !(a > 0.0 && b > 0.0) {
                        return None;
                    }
                    seg = bez_split_weights(&seg, a, b).0;
                }
                if from > *k0 {
                    let span = to - k0;
                    let (a, b) = ((to - from) / span, (from - k0) / span);
                    if !(a > 0.0 && b > 0.0) {
                        return None;
                    }
                    seg = bez_split_weights(&seg, a, b).1;
                }
                out.push(Geom::Bez(seg));
                covered = to;
            }
            if out.is_empty() || !same_parameter(covered, hi.min(d1)) {
                return None;
            }
            if ta > tb {
                out.reverse();
                for g in &mut out {
                    if let Geom::Bez(cps) = g {
                        cps.reverse();
                    }
                }
            }
            Some(out)
        }
        // Hyperbolic and parabolic arcs are outside the qualified family.
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    }
}

fn conic_geoms(c: [f64; 3], a: [f64; 3], b: [f64; 3], ta: f64, tb: f64) -> Option<Vec<Geom>> {
    if !(ta.is_finite() && tb.is_finite()) || (tb - ta).abs() < f64::MIN_POSITIVE {
        return None;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = ((tb - ta).abs() / MAX_ARC).ceil().max(1.0) as usize;
    if n > 4096 {
        return None;
    }
    #[allow(clippy::cast_precision_loss)]
    Some(
        (0..n)
            .map(|k| Geom::Arc {
                c,
                a,
                b,
                t0: ta + (tb - ta) * (k as f64) / (n as f64),
                t1: ta + (tb - ta) * ((k + 1) as f64) / (n as f64),
            })
            .collect(),
    )
}

/// One certified span.
#[derive(Clone, Debug)]
struct Span {
    geom: Geom,
    depth: u32,
    /// Lifted chart images of the span's ends.
    s: Point2,
    e: Point2,
    cone: (f64, f64),
}

/// Bisect a geometry until it is usable.
fn usable(
    map: &ChartMap,
    g: Geom,
    depth: u32,
    out: &mut Vec<(Geom, u32, (f64, f64))>,
) -> Option<()> {
    if let Some(cone) = analyse(map, &g) {
        out.push((g, depth, cone));
        return Some(());
    }
    if depth >= MAX_DEPTH || out.len() > MAX_SPANS {
        return None;
    }
    let (l, r) = g.split();
    usable(map, l, depth + 1, out)?;
    usable(map, r, depth + 1, out)
}

/// A piece's spans and how its curve ends miss its nodes.
struct PieceSpans {
    spans: Vec<Span>,
    /// Chart distance from the curve's start (end) to node_a (node_b).
    d_a: f64,
    d_b: f64,
}

/// Settings of one certificate.
pub(super) struct CertConfig {
    /// Largest chart distance from a curve end to its node.
    pub(super) mismatch_cap: f64,
    /// Largest node ball radius.
    pub(super) ball_cap: f64,
}

/// A certified piece: its polyline and leaving angles.
pub(super) struct CertifiedEdge {
    /// node_a, break points, node_b.
    pub(super) poly: Vec<Point2>,
    /// A denser polyline in the same enclosures, for the emitted pcurve.
    pub(super) dense: Vec<Point2>,
    /// Leaving angle at node_a, and at node_b along the reversed piece.
    pub(super) leave_a: f64,
    pub(super) leave_b: f64,
}

/// The certified arrangement.
pub(super) struct Certificate {
    pub(super) edges: Vec<CertifiedEdge>,
    /// Every span's enclosure, as a convex polygon.
    pub(super) enclosures: Vec<Vec<Point2>>,
    /// The largest node ball any contact was confined to.
    pub(super) max_ball: f64,
}

/// Which apex two touching spans share.
enum Apex {
    /// Consecutive spans `k`, `k + 1` of one piece.
    Break { edge: usize, k: usize },
    /// Span ends at one node.
    Node {
        node: usize,
        a: (usize, usize, bool),
        b: (usize, usize, bool),
    },
}

struct Ctx<'a> {
    map: &'a ChartMap,
    edges: &'a [GraphEdge],
    nodes: &'a [Node],
    pieces: Vec<PieceSpans>,
    cfg: &'a CertConfig,
    guard: f64,
    max_ball: f64,
}

impl Ctx<'_> {
    fn last(&self, i: usize) -> usize {
        self.pieces[i].spans.len() - 1
    }

    /// Node ends a span carries: `(node, leaving)`.
    fn ends(&self, i: usize, k: usize) -> Vec<(usize, bool)> {
        let mut out = Vec::new();
        if k == 0 {
            out.push((self.edges[i].node_a, true));
        }
        if k == self.last(i) {
            out.push((self.edges[i].node_b, false));
        }
        out
    }

    /// The span's polyline segment ends.
    fn poly_ends(&self, i: usize, k: usize) -> (Point2, Point2) {
        let sp = &self.pieces[i].spans[k];
        let a = if k == 0 {
            self.nodes[self.edges[i].node_a].uv
        } else {
            sp.s
        };
        let b = if k == self.last(i) {
            self.nodes[self.edges[i].node_b].uv
        } else {
            sp.e
        };
        (a, b)
    }

    /// The cone, widened at a piece end so it also holds the polyline chord
    /// (whose end is the node, not the curve end).
    fn widened(&self, i: usize, k: usize) -> Option<(f64, f64)> {
        let sp = &self.pieces[i].spans[k];
        let mut miss = 0.0;
        if k == 0 {
            miss += self.pieces[i].d_a;
        }
        if k == self.last(i) {
            miss += self.pieces[i].d_b;
        }
        if miss == 0.0 {
            return Some(sp.cone);
        }
        let chord = (sp.e - sp.s).length();
        if miss.partial_cmp(&(0.5 * chord)) != Some(Ordering::Less) {
            return None;
        }
        let d = (miss / chord).asin() + ANGLE_PAD;
        Some((sp.cone.0 - d, sp.cone.1 + d))
    }

    /// Convex enclosure of a span (including its node at a piece end).
    fn enclosure(&self, i: usize, k: usize) -> Option<Vec<Point2>> {
        let sp = &self.pieces[i].spans[k];
        let mut pts = quad(sp.s, sp.e, sp.cone)?;
        let (a, b) = self.poly_ends(i, k);
        if k == 0 {
            pts.push(a);
        }
        if k == self.last(i) {
            pts.push(b);
        }
        Some(convex_hull(pts))
    }

    /// Check the one apex two touching spans share.
    fn apex_ok(&mut self, apex: &Apex) -> bool {
        match *apex {
            Apex::Break { edge, k } => {
                let (Some(w1), Some(w2)) = (self.widened(edge, k), self.widened(edge, k + 1))
                else {
                    return false;
                };
                angular_gap((w1.0 + PI, w1.1 + PI), w2) > ANGLE_PAD
            }
            Apex::Node { node, a, b } => {
                let n = self.nodes[node].uv;
                let side = |ctx: &Self, (i, k, leaving): (usize, usize, bool)| {
                    let w = ctx.widened(i, k)?;
                    let dir = if leaving { w } else { (w.0 + PI, w.1 + PI) };
                    let miss = if leaving {
                        ctx.pieces[i].d_a
                    } else {
                        ctx.pieces[i].d_b
                    };
                    let (pa, pb) = ctx.poly_ends(i, k);
                    let far = if leaving { pb } else { pa };
                    Some((dir, miss, (far - n).length()))
                };
                let (Some(sa), Some(sb)) = (side(self, a), side(self, b)) else {
                    return false;
                };
                let gap = angular_gap(sa.0, sb.0);
                if gap <= ANGLE_PAD {
                    return false;
                }
                let beta = gap / 3.0;
                let ball = sa.1.max(sb.1) * (1.0 + 1.0 / beta.sin());
                if !(ball <= self.cfg.ball_cap && sa.2 > ball && sb.2 > ball) {
                    return false;
                }
                self.max_ball = self.max_ball.max(ball);
                true
            }
        }
    }

    /// Split span `k` of piece `i` in two, lifting the midpoint.
    fn split(&self, i: usize, k: usize) -> Option<(Span, Span)> {
        let sp = &self.pieces[i].spans[k];
        if sp.depth >= MAX_DEPTH {
            return None;
        }
        let (l, r) = sp.geom.split();
        let mid = self.map.unwrap_near(self.map.chart_raw(l.end())?, sp.s);
        let cl = analyse(self.map, &l)?;
        let cr = analyse(self.map, &r)?;
        Some((
            Span {
                geom: l,
                depth: sp.depth + 1,
                s: sp.s,
                e: mid,
                cone: cl,
            },
            Span {
                geom: r,
                depth: sp.depth + 1,
                s: mid,
                e: sp.e,
                cone: cr,
            },
        ))
    }
}

/// `(S + K) ∩ (E − K)` for the cone `K`: the span's chart enclosure, as the
/// quadrilateral `S, V₁, E, V₂`.
fn quad(s: Point2, e: Point2, cone: (f64, f64)) -> Option<Vec<Point2>> {
    let d = (e.x() - s.x(), e.y() - s.y());
    let (mut lo, mut hi) = cone;
    for _ in 0..8 {
        let (dl, dh) = ((lo.cos(), lo.sin()), (hi.cos(), hi.sin()));
        let det = cross2(dl, dh);
        if det > 0.0 {
            // S + p·dl = E − q·dh  and  S + p'·dh = E − q'·dl.
            let p1 = cross2(d, dh) / det;
            let q1 = cross2(dl, d) / det;
            let p2 = cross2(dl, d) / det;
            let q2 = cross2(d, dh) / det;
            let ok = p1 >= 0.0 && q1 >= 0.0 && p2 >= 0.0 && q2 >= 0.0;
            if ok || d.0.hypot(d.1) == 0.0 {
                let v1 = Point2::new(s.x() + p1.max(0.0) * dl.0, s.y() + p1.max(0.0) * dl.1);
                let v2 = Point2::new(s.x() + p2.max(0.0) * dh.0, s.y() + p2.max(0.0) * dh.1);
                return Some(vec![s, v1, e, v2]);
            }
        }
        // The chord sits a rounding error outside the cone: widen it.
        let w = (hi - lo).max(MIN_CONE);
        lo -= w;
        hi += w;
        if hi - lo > MAX_WIDTH + 0.5 {
            break;
        }
    }
    None
}

/// Certify the arrangement of `edges` (pieces between `nodes`) in `map`.
///
/// Returns each piece's certified polyline and leaving angles, and every
/// span's enclosure; `None` when the exact curves cannot be proven to meet
/// only at nodes within the budget.
#[allow(clippy::too_many_lines)]
pub(super) fn certify(
    map: &ChartMap,
    edges: &[GraphEdge],
    nodes: &[Node],
    cfg: &CertConfig,
) -> Option<Certificate> {
    let mut pairs_left = MAX_PAIR_CHECKS;
    certify_with_pair_budget(map, edges, nodes, cfg, &mut pairs_left)
}

#[allow(clippy::too_many_lines)]
fn certify_with_pair_budget(
    map: &ChartMap,
    edges: &[GraphEdge],
    nodes: &[Node],
    cfg: &CertConfig,
    pairs_left: &mut usize,
) -> Option<Certificate> {
    let mut cache = BezierCache::default();
    let mut pieces = Vec::with_capacity(edges.len());
    let mut total = 0usize;
    for e in edges {
        let mut geoms = Vec::new();
        for g in piece_geoms(map, &mut cache, e)? {
            if usable(map, g, 0, &mut geoms).is_none() {
                decline!();
            }
        }
        // A piece closing on one node needs three spans, so its first and
        // last spans are not also consecutive.
        while e.node_a == e.node_b && geoms.len() < 3 {
            let (g, d, _) = geoms.remove(0);
            let (l, r) = g.split();
            let mut tmp = Vec::new();
            usable(map, l, d + 1, &mut tmp)?;
            usable(map, r, d + 1, &mut tmp)?;
            for (k, x) in tmp.into_iter().enumerate() {
                geoms.insert(k, x);
            }
        }
        // Lift the chain from node_a.
        let na = nodes[e.node_a].uv;
        let nb = nodes[e.node_b].uv;
        let start = map.unwrap_near(map.chart_raw(geoms[0].0.start())?, na);
        let d_a = (start - na).length();
        let mut spans = Vec::with_capacity(geoms.len());
        let mut cur = start;
        for (g, depth, cone) in geoms {
            let end = map.unwrap_near(map.chart_raw(g.end())?, cur);
            spans.push(Span {
                geom: g,
                depth,
                s: cur,
                e: end,
                cone,
            });
            cur = end;
        }
        let d_b = (cur - nb).length();
        if !(d_a <= cfg.mismatch_cap && d_b <= cfg.mismatch_cap) {
            log::debug!(
                "chart_certificate: curve ends miss their nodes by {d_a:e}, {d_b:e} (cap {:e})",
                cfg.mismatch_cap
            );
            decline!();
        }
        total += spans.len();
        if total > MAX_SPANS {
            decline!();
        }
        pieces.push(PieceSpans { spans, d_a, d_b });
    }
    if total > MAX_SPANS {
        decline!();
    }
    let scale = nodes
        .iter()
        .map(|n| n.uv.x().abs().max(n.uv.y().abs()))
        .fold(1.0, f64::max);
    let mut ctx = Ctx {
        map,
        edges,
        nodes,
        pieces,
        cfg,
        guard: 1e-12 * scale,
        max_ball: 0.0,
    };

    for round in 0..=MAX_ROUNDS {
        // Every span with its enclosure and box.
        let mut flat: Vec<(usize, usize, Vec<Point2>, [f64; 4])> = Vec::new();
        for i in 0..ctx.pieces.len() {
            for k in 0..ctx.pieces[i].spans.len() {
                let Some(poly) = ctx.enclosure(i, k) else {
                    decline!();
                };
                let bb = poly
                    .iter()
                    .fold([f64::MAX, f64::MIN, f64::MAX, f64::MIN], |b, p| {
                        [
                            b[0].min(p.x()),
                            b[1].max(p.x()),
                            b[2].min(p.y()),
                            b[3].max(p.y()),
                        ]
                    });
                flat.push((i, k, poly, bb));
            }
        }
        let mut order: Vec<usize> = (0..flat.len()).collect();
        order.compact_sort_by(|&a, &b| flat[a].3[0].total_cmp(&flat[b].3[0]));
        let mut marked: Vec<Vec<bool>> = ctx
            .pieces
            .iter()
            .map(|p| vec![false; p.spans.len()])
            .collect();
        let mut conflicts = 0usize;
        let g = ctx.guard;
        for (oi, &x) in order.iter().enumerate() {
            let bx = flat[x].3;
            for &y in &order[oi + 1..] {
                let by = flat[y].3;
                if by[0] > bx[1] + g {
                    break;
                }
                if *pairs_left == 0 {
                    decline!();
                }
                *pairs_left -= 1;
                if by[2] > bx[3] + g || bx[2] > by[3] + g {
                    continue;
                }
                let (i, k) = (flat[x].0, flat[x].1);
                let (j, l) = (flat[y].0, flat[y].1);
                if !polygons_touch(&flat[x].2, &flat[y].2, g) {
                    continue;
                }
                // The apexes the two spans share.
                let mut apexes = Vec::new();
                if i == j && k.abs_diff(l) == 1 {
                    apexes.push(Apex::Break {
                        edge: i,
                        k: k.min(l),
                    });
                }
                for (na, la) in ctx.ends(i, k) {
                    for (nb, lb) in ctx.ends(j, l) {
                        if na == nb {
                            apexes.push(Apex::Node {
                                node: na,
                                a: (i, k, la),
                                b: (j, l, lb),
                            });
                        }
                    }
                }
                let ok = apexes.len() == 1 && ctx.apex_ok(&apexes[0]);
                if !ok {
                    conflicts += 1;
                    marked[i][k] = true;
                    marked[j][l] = true;
                }
            }
        }
        if conflicts == 0 {
            log::debug!(
                "chart_certificate: certified {} spans in {round} rounds (node ball {:e})",
                flat.len(),
                ctx.max_ball
            );
            return Some(finish(&ctx, flat.into_iter().map(|f| f.2).collect()));
        }
        if round == MAX_ROUNDS {
            break;
        }
        // Bisect every conflicting span.
        let mut count = 0usize;
        for i in 0..ctx.pieces.len() {
            let mut next = Vec::with_capacity(ctx.pieces[i].spans.len());
            for k in 0..ctx.pieces[i].spans.len() {
                if marked[i][k] {
                    let Some((l, r)) = ctx.split(i, k) else {
                        decline!();
                    };
                    next.push(l);
                    next.push(r);
                } else {
                    next.push(ctx.pieces[i].spans[k].clone());
                }
            }
            count += next.len();
            ctx.pieces[i].spans = next;
        }
        if count > MAX_SPANS {
            decline!();
        }
        log::trace!("chart_certificate: round {round}: {conflicts} conflicts, {count} spans");
    }
    log::debug!("chart_certificate: conflicts remain after {MAX_ROUNDS} rounds");
    None
}

/// Assemble the certified polylines and leaving angles.
fn finish(ctx: &Ctx<'_>, enclosures: Vec<Vec<Point2>>) -> Certificate {
    let mid = |w: (f64, f64)| 0.5 * (w.0 + w.1);
    let mut out = Vec::with_capacity(ctx.pieces.len());
    for (i, p) in ctx.pieces.iter().enumerate() {
        let last = p.spans.len() - 1;
        let na = ctx.nodes[ctx.edges[i].node_a].uv;
        let nb = ctx.nodes[ctx.edges[i].node_b].uv;
        let mut poly = vec![na];
        poly.extend(p.spans[..last].iter().map(|s| s.e));
        poly.push(nb);
        // Dense samples inside each span's enclosure, lifted next to the
        // span start (the span's angles vary by under `π/2`).
        let per = 64usize.div_ceil(p.spans.len()).saturating_sub(1);
        let mut dense = vec![na];
        for (k, s) in p.spans.iter().enumerate() {
            if !matches!(s.geom, Geom::Seg(..)) {
                for j in 1..=per {
                    #[allow(clippy::cast_precision_loss)]
                    let f = j as f64 / (per + 1) as f64;
                    if let Some(raw) = ctx.map.chart_raw(s.geom.at(f)) {
                        dense.push(ctx.map.unwrap_near(raw, s.s));
                    }
                }
            }
            dense.push(if k == last { nb } else { s.e });
        }
        let wa = ctx.widened(i, 0).unwrap_or(p.spans[0].cone);
        let wb = ctx.widened(i, last).unwrap_or(p.spans[last].cone);
        out.push(CertifiedEdge {
            poly,
            dense,
            leave_a: mid(wa),
            leave_b: mid(wb) + PI,
        });
    }
    Certificate {
        edges: out,
        enclosures,
        max_ball: ctx.max_ball,
    }
}

#[cfg(test)]
#[path = "chart_certificate_tests.rs"]
mod tests;
