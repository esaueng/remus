//! Certified `(u, v)` lift of one boundary loop of a ring-torus face.
//!
//! Every boundary edge is cut into pieces whose carrier-angle range is
//! *proved*, never read from samples:
//!
//! - a NURBS edge is cut at its knots into the rational Bezier spans that
//!   cover its authoritative domain (each control point is a blossom of the
//!   stored net, so every step is a convex combination and positive weights
//!   stay positive); a circle edge is cut into exact rational quadratic arcs
//!   of at most a quarter turn; a line edge is its own degree-1 span;
//! - with positive weights a span lies in the convex hull of its control
//!   points. Its `u` range is the arc of directions those points span about
//!   the torus axis, provided that arc is under π (the hull then misses the
//!   axis). Its distance `ρ` from the axis is bounded by the Bernstein
//!   coefficients of `ρ²·w²` over those of `w²` (exact for a parallel: the
//!   ratio is then constant), its height `z` by the control heights, and its
//!   `v` range is the arc of directions the rectangle
//!   `[ρ_min − R, ρ_max − R] × [z_min, z_max]` spans about the tube centre,
//!   provided the rectangle misses that centre;
//! - a span whose hull meets the axis or the tube centre, or whose arc is
//!   wider than a quarter turn, is split in half by exact de Casteljau
//!   subdivision of its homogeneous net, and so on to a depth limit.
//!
//! Consecutive pieces are chained into one continuous lift by their shared
//! end point: each piece's whole path lies in its own arc (under π wide), so
//! the lifted angle of its end point is the representative inside that arc,
//! and exactly one `2π` shift of the next piece puts its start within the
//! junction's gap allowance of it. The shift needed to close the loop is its
//! winding. No sample position ever decides a branch, and parameter speed
//! (knot spacing, a positive-weight reparametrization) changes nothing: the
//! pieces are the same point sets however fast the parameter runs.
//!
//! Tightness is refined, never assumed. The hull of the lifted pieces always
//! contains the lifted loop; the lifted end points are points of it. A piece
//! whose interval reaches further past the attained extremes than
//! [`TIGHTNESS`] is subdivided again, up to [`MAX_PIECES`] pieces. Running out
//! of budget leaves a looser hull, still a sound one.
//!
//! Rounding is covered by explicit allowances proportional to the
//! coordinate magnitudes involved (a few hundred ulps of the frame's scale);
//! junction gaps (vertex tolerances) by the angle a gap can subtend. Those
//! are the only widenings; nothing is approximated.

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::wire::{OrientedEdge, WireId};

use crate::error::AlgoError;

const TAU: f64 = std::f64::consts::TAU;
const PI: f64 = std::f64::consts::PI;

/// Widest angle range one piece may carry. Well under π, so a piece's arc
/// never aliases and the chaining shift is unique.
const MAX_PIECE_WIDTH: f64 = std::f64::consts::FRAC_PI_2;

/// How far the hull may reach past the attained extremes before the pieces
/// responsible are subdivided again (radians).
pub(super) const TIGHTNESS: f64 = 1e-10;

/// Piece budget per loop; beyond it the hull is left as it stands (sound).
const MAX_PIECES: usize = 4096;

/// Subdivision depth below which a piece that still meets the axis or the
/// tube centre makes the loop uncertain.
const MAX_DEPTH: u32 = 60;

/// Refinement rounds per loop.
const MAX_ROUNDS: usize = 128;

/// Highest span degree handled (the Bernstein product tables stay small).
const MAX_DEGREE: usize = 24;

/// Rounding allowance, in units of `f64::EPSILON` times the magnitude of the
/// coordinates involved.
const ROUNDING_ULPS: f64 = 256.0;

/// Smallest angular allowance added to every bound (radians).
const MIN_ANGLE_PAD: f64 = 1e-14;

/// A homogeneous control point `[w·x, w·y, w·z, w]` in the torus frame.
type Hom = [f64; 4];

/// The torus frame every piece is expressed in.
pub(super) struct Carrier {
    center: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    major: f64,
    /// `|centre| + R + r`: the magnitude world coordinates were rounded at.
    scale: f64,
    /// `+1` when `∂u × ∂v` points along the surface normal, `-1` otherwise.
    parameter_sense: f64,
}

impl Carrier {
    /// The frame of a ring torus (`R > r > 0`), `None` for any other.
    pub(super) fn new(torus: &ToroidalSurface) -> Option<Self> {
        let (major, minor) = (torus.major_radius(), torus.minor_radius());
        if !(major.is_finite() && minor.is_finite() && minor > 0.0 && major > minor) {
            return None;
        }
        let center = torus.center();
        let (x, y, z) = (torus.x_axis(), torus.y_axis(), torus.z_axis());
        // At (u, v) = (0, 0): ∂u ∝ Y and ∂v ∝ Z, so ∂u × ∂v ∝ Y × Z.
        let sense = y.cross(z).dot(torus.normal(0.0, 0.0));
        if !sense.is_finite() || sense.abs() < 0.5 {
            return None;
        }
        Some(Self {
            center,
            x,
            y,
            z,
            major,
            scale: center.x().abs() + center.y().abs() + center.z().abs() + major + minor,
            parameter_sense: sense.signum(),
        })
    }

    /// `+1` when a loop running counter-clockwise in `(u, v)` runs
    /// counter-clockwise about the surface normal.
    pub(super) const fn parameter_sense(&self) -> f64 {
        self.parameter_sense
    }

    fn hom(&self, p: Point3, w: f64) -> Hom {
        let d = p - self.center;
        [d.dot(self.x) * w, d.dot(self.y) * w, d.dot(self.z) * w, w]
    }
}

/// The certified lift of one closed boundary loop.
#[derive(Debug, Clone, Copy)]
pub(super) struct LoopLift {
    /// Lifted `u` interval holding the whole loop.
    pub(super) u: (f64, f64),
    /// Lifted `v` interval holding the whole loop.
    pub(super) v: (f64, f64),
    /// Lifted `u` interval spanned by points of the loop (a subset of `u`).
    pub(super) attained_u: (f64, f64),
    /// Lifted `v` interval spanned by points of the loop (a subset of `v`).
    pub(super) attained_v: (f64, f64),
    /// Net turns of the loop in `u` and `v`.
    pub(super) winding: (i64, i64),
    /// Signed `(u, v)` area of the lifted loop (counter-clockwise positive),
    /// meaningful only for winding `(0, 0)`.
    pub(super) area: f64,
    /// A bound on the error of [`Self::area`].
    pub(super) area_error: f64,
}

impl LoopLift {
    /// Whether the area's sign is certified and equals `sign`.
    pub(super) fn area_has_sign(&self, sign: f64) -> bool {
        self.winding == (0, 0)
            && self.area.is_finite()
            && self.area_error.is_finite()
            && self.area * sign > self.area_error
    }
}

/// The proved angle ranges of one piece, in the piece's own branch.
#[derive(Debug, Clone, Copy)]
struct Enclosure {
    u: (f64, f64),
    v: (f64, f64),
    start: (f64, f64),
    end: (f64, f64),
    /// Rounding allowance on the end-point angles.
    pad: (f64, f64),
}

#[derive(Debug, Clone)]
struct Piece {
    cps: Vec<Hom>,
    depth: u32,
    enclosure: Enclosure,
}

/// `angle` reduced to `(-π, π]`.
fn wrap_pi(angle: f64) -> f64 {
    let reduced = angle - TAU * (angle / TAU).round();
    if reduced <= -PI {
        reduced + TAU
    } else {
        reduced
    }
}

fn point_of(h: &Hom) -> [f64; 3] {
    [h[0] / h[3], h[1] / h[3], h[2] / h[3]]
}

fn lerp(a: &Hom, b: &Hom, t: f64) -> Hom {
    // Blossom extraction reaches exact span ends. Preserve those values;
    // subtracting a large-weight neighbour can erase the smaller endpoint.
    if t == 0.0 {
        return *a;
    }
    if t >= 1.0 {
        return *b;
    }
    let s = 1.0 - t;
    [
        t.mul_add(b[0], s * a[0]),
        t.mul_add(b[1], s * a[1]),
        t.mul_add(b[2], s * a[2]),
        t.mul_add(b[3], s * a[3]),
    ]
}

/// `C(n, k)` as a float (exact for the degrees handled).
fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n - k);
    let mut value = 1.0_f64;
    for i in 0..k {
        value = value * (n - i) as f64 / (i + 1) as f64;
    }
    value
}

/// The directions of `points` about the origin as an interval relative to
/// the direction of `reference`, or `None` when they do not fit in an arc of
/// at most [`MAX_PIECE_WIDTH`] (which also means they may surround it).
fn direction_arc(reference: (f64, f64), points: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let base = reference.1.atan2(reference.0);
    let (mut lo, mut hi) = (0.0_f64, 0.0_f64);
    for &(a, b) in points {
        if !(a.is_finite() && b.is_finite() && a.hypot(b) > 0.0) {
            return None;
        }
        let offset = wrap_pi(b.atan2(a) - base);
        lo = lo.min(offset);
        hi = hi.max(offset);
    }
    (hi - lo <= MAX_PIECE_WIDTH).then_some((base, lo, hi))
}

/// The proved enclosure of one rational Bezier piece, `None` when its hull
/// meets the torus axis or the tube centre, or spans too wide an arc.
fn enclose(cps: &[Hom], carrier: &Carrier) -> Option<Enclosure> {
    let n = cps.len().checked_sub(1)?;
    if n == 0 || n > MAX_DEGREE {
        return None;
    }
    let mut magnitude = 0.0_f64;
    let mut points = Vec::with_capacity(cps.len());
    for h in cps {
        if !(h.iter().all(|c| c.is_finite()) && h[3] > 0.0) {
            return None;
        }
        let p = point_of(h);
        if !p.iter().all(|c| c.is_finite()) {
            return None;
        }
        magnitude = magnitude.max(p[0].abs()).max(p[1].abs()).max(p[2].abs());
        points.push(p);
    }
    let (first, last) = (points[0], points[n]);
    // Absolute allowance for the frame change and the arithmetic below.
    let linear_pad = ROUNDING_ULPS * f64::EPSILON * (carrier.scale + magnitude);

    // u: directions of the control points about the axis.
    let xy: Vec<(f64, f64)> = points.iter().map(|p| (p[0], p[1])).collect();
    let axis_distance = xy
        .iter()
        .map(|&(a, b)| a.hypot(b))
        .fold(f64::INFINITY, f64::min);
    if axis_distance.is_nan() || axis_distance <= 4.0 * linear_pad {
        return None;
    }
    let (u_base, u_lo, u_hi) = direction_arc((first[0], first[1]), &xy)?;
    let u_pad = linear_pad / axis_distance + MIN_ANGLE_PAD;
    let u_end = u_base + wrap_pi(last[1].atan2(last[0]) - u_base);

    // ρ² = (X² + Y²)/W² as a ratio of degree-2n Bernstein polynomials whose
    // denominator has positive coefficients.
    let gamma = 8.0 * (2 * n + 4) as f64 * f64::EPSILON;
    let (mut rho2_lo, mut rho2_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for k in 0..=2 * n {
        let (mut num, mut abs, mut den) = (0.0_f64, 0.0_f64, 0.0_f64);
        let scale = binomial(2 * n, k);
        for i in k.saturating_sub(n)..=k.min(n) {
            let j = k - i;
            let c = binomial(n, i) * binomial(n, j) / scale;
            let (a, b) = (&cps[i], &cps[j]);
            num += c * (a[0] * b[0] + a[1] * b[1]);
            abs += c * ((a[0] * b[0]).abs() + (a[1] * b[1]).abs());
            den += c * a[3] * b[3];
        }
        // The relative rounding bound below assumes normal arithmetic.
        // Underflowed coefficient sums cannot certify a smaller enclosure.
        if !(den > 0.0 && den.is_normal() && num.is_finite() && abs.is_normal()) {
            return None;
        }
        let ratio = num / den;
        let error = gamma * (abs / den + ratio.abs());
        rho2_lo = rho2_lo.min(ratio - error);
        rho2_hi = rho2_hi.max(ratio + error);
    }
    let s_lo = rho2_lo.max(0.0).sqrt() - carrier.major - linear_pad;
    let s_hi = rho2_hi.sqrt() - carrier.major + linear_pad;
    let z_lo = points.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min) - linear_pad;
    let z_hi = points
        .iter()
        .map(|p| p[2])
        .fold(f64::NEG_INFINITY, f64::max)
        + linear_pad;
    if !(s_lo.is_finite() && s_hi.is_finite() && z_lo.is_finite() && z_hi.is_finite()) {
        return None;
    }
    // Distance from the tube centre (the origin of the (s, z) half-plane) to
    // the rectangle; it must miss it.
    let gap_s = if s_lo > 0.0 {
        s_lo
    } else if s_hi < 0.0 {
        -s_hi
    } else {
        0.0
    };
    let gap_z = if z_lo > 0.0 {
        z_lo
    } else if z_hi < 0.0 {
        -z_hi
    } else {
        0.0
    };
    let tube_distance = gap_s.hypot(gap_z);
    if tube_distance.is_nan() || tube_distance <= 4.0 * linear_pad {
        return None;
    }
    let tube_point = |p: &[f64; 3]| (p[0].hypot(p[1]) - carrier.major, p[2]);
    let corners = [(s_lo, z_lo), (s_hi, z_lo), (s_lo, z_hi), (s_hi, z_hi)];
    let start_sz = tube_point(&first);
    let (v_base, mut v_lo, mut v_hi) = direction_arc(start_sz, &corners)?;
    let end_sz = tube_point(&last);
    let v_end_offset = wrap_pi(end_sz.1.atan2(end_sz.0) - v_base);
    // The end points lie in the rectangle up to rounding; keep them inside.
    v_lo = v_lo.min(v_end_offset).min(0.0);
    v_hi = v_hi.max(v_end_offset).max(0.0);
    let v_pad = 4.0 * f64::EPSILON + MIN_ANGLE_PAD;
    let (u_lo, u_hi) = (u_lo.min(0.0), u_hi.max(u_end - u_base));
    Some(Enclosure {
        u: (u_base + u_lo - u_pad, u_base + u_hi + u_pad),
        v: (v_base + v_lo - v_pad, v_base + v_hi + v_pad),
        start: (u_base, v_base),
        end: (u_end, v_base + v_end_offset),
        pad: (u_pad, v_pad + linear_pad / tube_distance),
    })
}

/// Exact de Casteljau halving of a homogeneous Bezier net.
fn split(cps: &[Hom]) -> (Vec<Hom>, Vec<Hom>) {
    let n = cps.len() - 1;
    let mut work = cps.to_vec();
    let mut left = Vec::with_capacity(n + 1);
    let mut right = Vec::with_capacity(n + 1);
    left.push(work[0]);
    right.push(work[n]);
    for r in 1..=n {
        for i in 0..=n - r {
            work[i] = lerp(&work[i], &work[i + 1], 0.5);
        }
        left.push(work[0]);
        right.push(work[n - r]);
    }
    right.reverse();
    (left, right)
}

/// Appends `cps` to `out` as enclosed pieces, halving where needed. Returns
/// `false` when some part stays unresolved at [`MAX_DEPTH`] or the budget
/// runs out.
fn resolve(cps: Vec<Hom>, depth: u32, carrier: &Carrier, out: &mut Vec<Piece>) -> bool {
    if out.len() >= MAX_PIECES {
        return false;
    }
    if let Some(enclosure) = enclose(&cps, carrier) {
        out.push(Piece {
            cps,
            depth,
            enclosure,
        });
        return true;
    }
    if depth >= MAX_DEPTH {
        return false;
    }
    let (left, right) = split(&cps);
    resolve(left, depth + 1, carrier, out) && resolve(right, depth + 1, carrier, out)
}

/// Exact rational quadratic arcs of at most a quarter turn covering the
/// circle parameters `t0 → t1`.
fn circle_spans(circle: &Circle3D, t0: f64, t1: f64, carrier: &Carrier) -> Option<Vec<Vec<Hom>>> {
    let sweep = t1 - t0;
    if !sweep.is_finite() || sweep.abs() > TAU * (1.0 + 1e-12) || !circle.radius().is_finite() {
        return None;
    }
    let count = (sweep.abs() / std::f64::consts::FRAC_PI_2).ceil().max(1.0) as usize;
    let (center, ua, va, radius) = (
        circle.center(),
        circle.u_axis(),
        circle.v_axis(),
        circle.radius(),
    );
    let at = |angle: f64, scale: f64| center + (ua * angle.cos() + va * angle.sin()) * scale;
    let mut spans = Vec::with_capacity(count);
    for k in 0..count {
        let a = sweep.mul_add(k as f64 / count as f64, t0);
        let b = if k + 1 == count {
            t1
        } else {
            sweep.mul_add((k + 1) as f64 / count as f64, t0)
        };
        let half = 0.5 * (b - a);
        let weight = half.cos();
        if weight.is_nan() || weight <= 0.0 {
            return None;
        }
        spans.push(vec![
            carrier.hom(at(a, radius), 1.0),
            carrier.hom(at(0.5 * (a + b), radius / weight), weight),
            carrier.hom(at(b, radius), 1.0),
        ]);
    }
    Some(spans)
}

/// The blossom of the homogeneous net `pw` at `args` on knot span `i`.
fn blossom(knots: &[f64], pw: &[Hom], i: usize, degree: usize, args: &[f64]) -> Option<Hom> {
    let mut d: Vec<Hom> = pw[i - degree..=i].to_vec();
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let k = i - degree + j;
            let (left, right) = (knots[k], knots[k + degree + 1 - r]);
            let span = right - left;
            if !span.is_finite() || span <= 0.0 {
                return None;
            }
            let arg = args[r - 1];
            let alpha = ((arg - left) / span).clamp(0.0, 1.0);
            d[j] = if alpha <= 0.5 {
                lerp(&d[j - 1], &d[j], alpha)
            } else {
                // Compute the small coefficient directly. Subtracting an
                // alpha near one loses relative accuracy that disparate
                // homogeneous weights can amplify into a geometric error.
                let beta = ((right - arg) / span).clamp(0.0, 1.0);
                lerp(&d[j], &d[j - 1], beta)
            };
        }
    }
    Some(d[degree])
}

/// The rational Bezier spans covering `[t0, t1]` (either order) of a NURBS
/// curve with finite data and positive weights, in the direction `t0 → t1`.
fn nurbs_spans(curve: &NurbsCurve, t0: f64, t1: f64, carrier: &Carrier) -> Option<Vec<Vec<Hom>>> {
    let degree = curve.degree();
    let (knots, points, weights) = (curve.knots(), curve.control_points(), curve.weights());
    if degree == 0
        || degree > MAX_DEGREE
        || points.len() <= degree
        || weights.len() != points.len()
        || knots.len() != points.len() + degree + 1
        || !knots.iter().all(|k| k.is_finite())
        || knots.windows(2).any(|w| w[1] < w[0])
        || !weights.iter().all(|w| w.is_finite() && *w > 0.0)
        || !(t0.is_finite() && t1.is_finite())
    {
        return None;
    }
    // A common weight factor changes no point of the curve. Remove it
    // before homogeneous products can overflow or underflow.
    let weight_scale = weights.iter().copied().fold(0.0_f64, f64::max);
    let pw: Vec<Hom> = points
        .iter()
        .zip(weights)
        .map(|(&p, &w)| carrier.hom(p, w / weight_scale))
        .collect();
    // Losing a positive weight here changes the curve before its hull is
    // bounded. Decline instead of subdividing that altered control net.
    if pw.iter().any(|h| !h[3].is_normal()) {
        return None;
    }
    let (a, b) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    let mut spans = Vec::new();
    for i in degree..points.len() {
        let (lo, hi) = (knots[i].max(a), knots[i + 1].min(b));
        if knots[i] >= knots[i + 1] || lo >= hi {
            continue;
        }
        let mut args = vec![lo; degree];
        let mut net = Vec::with_capacity(degree + 1);
        for m in 0..=degree {
            if m > 0 {
                args[degree - m] = hi;
            }
            net.push(blossom(knots, &pw, i, degree, &args)?);
        }
        spans.push(net);
    }
    if spans.is_empty() {
        return None;
    }
    if t0 > t1 {
        spans.reverse();
        for span in &mut spans {
            span.reverse();
        }
    }
    Some(spans)
}

/// The spans of one oriented boundary edge, in traversal order; `None` for a
/// carrier this lift does not certify (ellipse, hyperbola, parabola, invalid
/// NURBS data).
fn edge_spans(
    topo: &Topology,
    oe: &OrientedEdge,
    carrier: &Carrier,
) -> Result<Option<Vec<Vec<Hom>>>, AlgoError> {
    let edge = topo.edge(oe.edge())?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (t0, t1) =
        super::super::super::helpers::authoritative_edge_domain(edge, oe.edge(), "torus lift")?;
    let spans = match edge.curve() {
        EdgeCurve::Line => Some(vec![vec![carrier.hom(start, 1.0), carrier.hom(end, 1.0)]]),
        EdgeCurve::Circle(circle) => circle_spans(circle, t0, t1, carrier),
        EdgeCurve::NurbsCurve(curve) => nurbs_spans(curve, t0, t1, carrier),
        // B24: exhaustive over `EdgeCurve` — conics other than the circle
        // are not certified here; the caller falls back to a sound bound.
        EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    };
    Ok(spans.map(|mut spans| {
        if !oe.is_forward() {
            spans.reverse();
            for span in &mut spans {
                span.reverse();
            }
        }
        spans
    }))
}

/// Shifts of every piece onto one continuous lift, the loop's winding and
/// the largest junction allowance.
struct Chain {
    shifts: Vec<(f64, f64)>,
    winding: (i64, i64),
    slack: (f64, f64),
    /// Area error contributed by the junction gaps.
    junction_area_error: f64,
}

/// The `2π` shifts that carry `next`'s start onto `prev`'s lifted end, and
/// the angle allowance of the gap between them; `None` when the gap is too
/// wide to bound or no shift fits.
fn junction(prev: &Piece, prev_shift: (f64, f64), next: &Piece, major: f64) -> Option<[f64; 4]> {
    let p = point_of(&prev.cps[prev.cps.len() - 1]);
    let q = point_of(&next.cps[0]);
    let gap = ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt();
    let axis = p[0].hypot(p[1]).min(q[0].hypot(q[1]));
    let tube = (p[0].hypot(p[1]) - major)
        .hypot(p[2])
        .min((q[0].hypot(q[1]) - major).hypot(q[2]));
    // Both maps (to the axis direction and to the tube direction) are
    // 1-Lipschitz, so a gap of `d` seen from distance `D >= 4d` subtends
    // less than `2d / D`.
    if !(gap.is_finite() && 4.0 * gap <= axis && 4.0 * gap <= tube) {
        return None;
    }
    let slack = [
        2.0 * gap / axis + prev.enclosure.pad.0 + next.enclosure.pad.0,
        2.0 * gap / tube + prev.enclosure.pad.1 + next.enclosure.pad.1,
    ];
    let lifted_end = (
        prev.enclosure.end.0 + prev_shift.0,
        prev.enclosure.end.1 + prev_shift.1,
    );
    let mut shift = [0.0; 2];
    for (axis_index, (end, start)) in [
        (lifted_end.0, next.enclosure.start.0),
        (lifted_end.1, next.enclosure.start.1),
    ]
    .into_iter()
    .enumerate()
    {
        let k = ((end - start) / TAU).round();
        let lifted = k.mul_add(TAU, start);
        let miss = (end - lifted).abs();
        if miss.is_nan() || miss > slack[axis_index] {
            return None;
        }
        shift[axis_index] = k * TAU;
    }
    Some([shift[0], shift[1], slack[0], slack[1]])
}

fn chain(pieces: &[Piece], major: f64) -> Option<Chain> {
    let mut shifts = Vec::with_capacity(pieces.len());
    shifts.push((0.0, 0.0));
    let mut slack = (0.0_f64, 0.0_f64);
    let mut junction_slack = Vec::with_capacity(pieces.len());
    for i in 1..pieces.len() {
        let [su, sv, au, av] = junction(&pieces[i - 1], shifts[i - 1], &pieces[i], major)?;
        shifts.push((su, sv));
        slack = (slack.0.max(au), slack.1.max(av));
        junction_slack.push((au, av));
    }
    let last = pieces.len() - 1;
    let [su, sv, au, av] = junction(&pieces[last], shifts[last], &pieces[0], major)?;
    slack = (slack.0.max(au), slack.1.max(av));
    junction_slack.push((au, av));
    // Each junction may move the lifted boundary by its allowance; bound the
    // area it can sweep by that allowance times the loop's extent.
    let (mut u_lo, mut u_hi, mut v_lo, mut v_hi) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for (piece, shift) in pieces.iter().zip(&shifts) {
        u_lo = u_lo.min(piece.enclosure.u.0 + shift.0);
        u_hi = u_hi.max(piece.enclosure.u.1 + shift.0);
        v_lo = v_lo.min(piece.enclosure.v.0 + shift.1);
        v_hi = v_hi.max(piece.enclosure.v.1 + shift.1);
    }
    let junction_area_error = junction_slack
        .iter()
        .map(|&(au, av)| au * (v_hi - v_lo + av) + av * (u_hi - u_lo + au))
        .sum();
    Some(Chain {
        shifts,
        winding: ((su / TAU).round() as i64, (sv / TAU).round() as i64),
        slack,
        junction_area_error,
    })
}

fn summarize(pieces: &[Piece], chain: &Chain) -> LoopLift {
    let inf = f64::INFINITY;
    let (mut u, mut v) = ((inf, -inf), (inf, -inf));
    let (mut au, mut av) = ((inf, -inf), (inf, -inf));
    let (mut area, mut area_error, mut magnitude) = (0.0_f64, chain.junction_area_error, 0.0_f64);
    let mut previous_end: Option<(f64, f64)> = None;
    for (piece, &(su, sv)) in pieces.iter().zip(&chain.shifts) {
        let e = &piece.enclosure;
        u = (u.0.min(e.u.0 + su), u.1.max(e.u.1 + su));
        v = (v.0.min(e.v.0 + sv), v.1.max(e.v.1 + sv));
        let start = (e.start.0 + su, e.start.1 + sv);
        let end = (e.end.0 + su, e.end.1 + sv);
        for (a, b) in [start, end] {
            au = (au.0.min(a), au.1.max(a));
            av = (av.0.min(b), av.1.max(b));
        }
        // Chords of the junction gap and of the piece; area = -∮ v du.
        let mut chord = |from: (f64, f64), to: (f64, f64)| {
            let term = -0.5 * (from.1 + to.1) * (to.0 - from.0);
            magnitude += term.abs();
            area += term;
        };
        if let Some(prev) = previous_end {
            chord(prev, start);
        }
        chord(start, end);
        previous_end = Some(end);
        // Between its ends a degree-n piece changes direction in u at most
        // 2n - 2 times (u' ∝ XY' − YX', of degree 2n − 2), so its u travel is
        // at most (2n − 1)·width_u, and ∫(v − v_mid) du differs from the
        // chord's by at most n·width_u·width_v.
        let degree = (piece.cps.len() - 1) as f64;
        area_error += degree * (e.u.1 - e.u.0) * (e.v.1 - e.v.0);
    }
    let first = &pieces[0].enclosure;
    if let Some(prev) = previous_end {
        let closing = (
            first.start.0 + (chain.winding.0 as f64) * TAU,
            first.start.1 + (chain.winding.1 as f64) * TAU,
        );
        let term = -0.5 * (prev.1 + closing.1) * (closing.0 - prev.0);
        magnitude += term.abs();
        area += term;
    }
    area_error += 16.0 * (pieces.len() as f64 + 2.0) * f64::EPSILON * magnitude;
    let (du, dv) = chain.slack;
    LoopLift {
        u: (u.0 - du, u.1 + du),
        v: (v.0 - dv, v.1 + dv),
        attained_u: au,
        attained_v: av,
        winding: chain.winding,
        area,
        area_error,
    }
}

/// `spans` closed into one chain: wherever a span does not end exactly
/// where the next (cyclically) starts (two edges meeting at a vertex within
/// its tolerance, or two knot spans' rounded end points), the straight
/// segment between the two points is inserted. The loop is then a closed
/// path of certified pieces; a gap is never covered by widening the whole
/// hull. `None` for an empty loop.
fn bridge_gaps(spans: Vec<Vec<Hom>>) -> Option<Vec<Vec<Hom>>> {
    let unit = |h: &Hom| {
        let p = point_of(h);
        [p[0], p[1], p[2], 1.0]
    };
    let same = |a: &Hom, b: &Hom| {
        point_of(a)
            .iter()
            .zip(point_of(b))
            .all(|(x, y)| x.to_bits() == y.to_bits())
    };
    let starts: Vec<Hom> = spans
        .iter()
        .map(|span| span.first().map(unit))
        .collect::<Option<_>>()?;
    let count = starts.len();
    if count == 0 {
        return None;
    }
    let mut out = Vec::with_capacity(2 * count);
    for (k, span) in spans.into_iter().enumerate() {
        let end = unit(span.last()?);
        let next = starts[(k + 1) % count];
        out.push(span);
        if !same(&end, &next) {
            out.push(vec![end, next]);
        }
    }
    Some(out)
}

/// Refine selected pieces, retaining the certified parent when a child
/// cannot be resolved within the subdivision limits.
fn refine_pieces(pieces: Vec<Piece>, refine: Vec<bool>, carrier: &Carrier) -> Vec<Piece> {
    let count = pieces.len();
    debug_assert!(count <= MAX_PIECES);
    debug_assert_eq!(refine.len(), count);
    let mut next = Vec::with_capacity((count * 2).min(MAX_PIECES));
    for (index, (piece, refine)) in pieces.into_iter().zip(refine).enumerate() {
        if refine {
            // Reserve one slot for every unvisited parent. A failed split
            // must retain those parents as well as this certified piece.
            let reserved = count - index - 1;
            let mark = next.len();
            if mark + reserved + 2 > MAX_PIECES {
                next.push(Piece {
                    depth: MAX_DEPTH,
                    ..piece
                });
                continue;
            }
            let (left, right) = split(&piece.cps);
            let ok = resolve(left, piece.depth + 1, carrier, &mut next)
                && resolve(right, piece.depth + 1, carrier, &mut next);
            if !ok || next.len() + reserved > MAX_PIECES {
                // The budget ran out or a half stayed unresolved: keep
                // the coarser, already certified piece, and never try
                // to split it again.
                next.truncate(mark);
                next.push(Piece {
                    depth: MAX_DEPTH,
                    ..piece
                });
            }
        } else {
            next.push(piece);
        }
    }
    next
}

/// The certified lift of the loop `wire` on the torus `carrier`, `Ok(None)`
/// when it cannot be certified (an unsupported or invalid edge carrier, a
/// piece whose hull keeps meeting the axis or the tube centre, a junction
/// gap too wide to bound, or the piece budget exhausted before every piece
/// resolved). Topology lookups and authoritative edge domains propagate
/// their errors.
pub(super) fn lift_loop(
    topo: &Topology,
    wire: WireId,
    carrier: &Carrier,
) -> Result<Option<LoopLift>, AlgoError> {
    let mut spans: Vec<Vec<Hom>> = Vec::new();
    for oe in topo.wire(wire)?.edges() {
        let Some(edge) = edge_spans(topo, oe, carrier)? else {
            return Ok(None);
        };
        spans.extend(edge);
    }
    let Some(spans) = bridge_gaps(spans) else {
        return Ok(None);
    };
    let mut pieces = Vec::new();
    for span in spans {
        if !resolve(span, 0, carrier, &mut pieces) {
            return Ok(None);
        }
    }
    let mut best = None;
    for _ in 0..MAX_ROUNDS {
        let Some(chain) = chain(&pieces, carrier.major) else {
            return Ok(best);
        };
        let lift = summarize(&pieces, &chain);
        best = Some(lift);
        let mut refine = vec![false; pieces.len()];
        let mut any = false;
        for (k, (piece, &(su, sv))) in pieces.iter().zip(&chain.shifts).enumerate() {
            let e = &piece.enclosure;
            let loose = e.u.0 + su < lift.attained_u.0 - TIGHTNESS
                || e.u.1 + su > lift.attained_u.1 + TIGHTNESS
                || e.v.0 + sv < lift.attained_v.0 - TIGHTNESS
                || e.v.1 + sv > lift.attained_v.1 + TIGHTNESS;
            if loose && piece.depth < MAX_DEPTH {
                refine[k] = true;
                any = true;
            }
        }
        // An undecided area sign: refine the pieces carrying more than their
        // share of the error.
        let undecided = lift.area.is_nan() || lift.area.abs() <= lift.area_error;
        if lift.winding == (0, 0) && undecided {
            let share = lift.area_error / pieces.len() as f64;
            for (k, piece) in pieces.iter().enumerate() {
                let e = &piece.enclosure;
                let error = (piece.cps.len() - 1) as f64 * (e.u.1 - e.u.0) * (e.v.1 - e.v.0);
                if error >= share && piece.depth < MAX_DEPTH {
                    refine[k] = true;
                    any = true;
                }
            }
        }
        if !any {
            return Ok(best);
        }
        pieces = refine_pieces(pieces, refine, carrier);
    }
    Ok(match chain(&pieces, carrier.major) {
        Some(chain) => Some(summarize(&pieces, &chain)),
        None => best,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::{
        Carrier, MAX_PIECES, NurbsCurve, Piece, Point3, ToroidalSurface, enclose, nurbs_spans,
        point_of, refine_pieces,
    };

    #[test]
    fn unrepresentable_weight_normalization_declines_the_lift() {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 2.0).unwrap();
        let carrier = Carrier::new(&torus).unwrap();
        let curve = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(12.0, 0.0, 0.0),
                Point3::new(12.0, 1.0, 0.0),
                Point3::new(11.0, 2.0, 0.0),
            ],
            vec![1e308, 1e-308, 1e308],
        )
        .unwrap();
        assert!(nurbs_spans(&curve, 0.0, 1.0, &carrier).is_none());
    }

    #[test]
    fn trimmed_arc_extraction_keeps_the_small_blossom_coefficient() {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 2.0).unwrap();
        let carrier = Carrier::new(&torus).unwrap();
        let half = 0.1_f64;
        let end = 0.2_f64;
        let mu = 1e-12;
        let curve = NurbsCurve::new(
            2,
            vec![-1e12, -1e12, -1e12, 1.0, 1.0, 1.0],
            vec![
                Point3::new(12.0, 0.0, 0.0),
                Point3::new(12.0, 12.0 * half.tan(), 0.0),
                Point3::new(12.0 * end.cos(), 12.0 * end.sin(), 0.0),
            ],
            vec![1.0, half.cos() * mu, mu * mu],
        )
        .unwrap();
        // The trim starts approximately halfway round this exact circle.
        // Computing its small coefficient as 1 - alpha loses significant
        // digits when alpha is near one on a long knot span.
        let expected = curve.evaluate(0.0);
        let midpoint = Point3::new(12.0 * half.cos(), 12.0 * half.sin(), 0.0);
        assert!((expected - midpoint).length() < 1e-9);
        let spans = nurbs_spans(&curve, 0.0, 1.0, &carrier).unwrap();
        let actual = point_of(&spans[0][0]);
        let expected = point_of(&carrier.hom(expected, 1.0));
        let distance = (Point3::new(actual[0], actual[1], actual[2])
            - Point3::new(expected[0], expected[1], expected[2]))
        .length();
        assert!(
            distance < 1e-10,
            "extracted trim endpoint moved by {distance}"
        );
    }

    #[test]
    fn refinement_respects_piece_budget_without_losing_arc_coverage() {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 2.0).unwrap();
        let carrier = Carrier::new(&torus).unwrap();
        // An exact rational quadratic arc of the outer equator, 0 to 0.2.
        let half = 0.1_f64;
        let end = 0.2_f64;
        let cps = vec![
            [12.0, 0.0, 0.0, 1.0],
            [12.0 * half.cos(), 12.0 * half.sin(), 0.0, half.cos()],
            [12.0 * end.cos(), 12.0 * end.sin(), 0.0, 1.0],
        ];
        let piece = Piece {
            enclosure: enclose(&cps, &carrier).unwrap(),
            cps,
            depth: 0,
        };
        for count in [MAX_PIECES - 2, MAX_PIECES - 1, MAX_PIECES] {
            let refined = refine_pieces(vec![piece.clone(); count], vec![true; count], &carrier);
            assert!(
                refined.len() <= MAX_PIECES,
                "{count} parents produced {} pieces, budget {MAX_PIECES}",
                refined.len()
            );
            // Subdivision may stop at its budget, but must retain every
            // parent's arc rather than discard the unvisited remainder.
            let swept: f64 = refined
                .iter()
                .map(|p| {
                    let first = p.cps.first().unwrap();
                    let last = p.cps.last().unwrap();
                    last[1].atan2(last[0]) - first[1].atan2(first[0])
                })
                .sum();
            assert!((swept - end * count as f64).abs() < 1e-8);
        }
    }
}
