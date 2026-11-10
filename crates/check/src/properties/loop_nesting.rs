//! Certified nesting of a planar face's inner loops.
//!
//! A planar face's inner wires alternate void and material by nesting depth:
//! a hole, an island inside it, a hole inside that island. The measurement
//! paths subtract a loop at even depth and add one back at odd depth, so the
//! depth decides the sign of an exact moment and must not be guessed.
//! [`inner_loop_nesting`] answers it as a certificate or not at all:
//!
//! - **The loop is the integrated loop.** Each inner wire is read the way the
//!   exact Green's-theorem integration reads it: a line edge between its
//!   vertices, a circle, ellipse, parabola or hyperbola edge over its strict
//!   trim, a NURBS edge over its trim (its carrier domain only for a closed
//!   edge with no trim). No carrier span outside a trim is ever looked at.
//! - **Every curved piece is enclosed.** A curved edge is cut into pieces,
//!   each held with its chord and a certified chord deviation `δ`: the piece
//!   lies within `δ` of its chord. A conic arc lies in the triangle of its
//!   chord and its end tangents' intersection (in closed form: the centre
//!   plus `(C(m) − centre) / cos h`, `cosh h` for a hyperbola, or
//!   `2·C(m) − (C(t₀) + C(t₁)) / 2` for a parabola), a rational Bézier piece
//!   with positive weights in the hull of its control points; `δ` is the
//!   distance of that apex, or of the farthest control point, from the chord.
//! - **Containment is exact parity.** Loop `A` is inside loop `B` when a
//!   point on `A` (a curve point, not a vertex: two loops of a degenerate face
//!   may share one) is inside `B`. Once the probe is farther than `δ` plus a
//!   rounding margin from every piece of `B`, each piece and its chord bound a
//!   region (inside the piece's convex enclosure) that does not hold the probe,
//!   so the probe's parity against `B` is its parity against `B`'s chord
//!   polygon, which exact `orient2d` decides. A piece the probe is too close
//!   to is bisected; a probe within the margin of `B` itself (touching loops)
//!   is abandoned for the next probe on `A`.
//! - **Nesting is reported only for disjoint loops.** Valid face loops do not
//!   cross, so one decided probe proves `A ⊂ B`. That premise is certified,
//!   not assumed, before any depth is reported: every pair of loops whose
//!   boxes overlap must have every pair of pieces separated by more than their
//!   deviations plus the margin, bisecting the fatter piece until they are.
//!   Loops that cross or touch (an invalid face) therefore never report
//!   nesting; at most they read [`InnerLoopNesting::Flat`] or
//!   [`InnerLoopNesting::Undecided`], and every caller keeps its
//!   subtract-every-hole arithmetic for both.
//!
//! Cost: building costs a few curve evaluations per edge. A loop is a
//! container candidate of another only when the other's on-curve points lie
//! in its box, so loops side by side cost one box comparison per pair,
//! `O(n²)` comparisons for `n` inner loops and nothing else. Containment and
//! disjointness work (one unit per piece visit or bisection) is bounded by
//! [`NESTING_WORK_BUDGET`] per face; past it the face is
//! [`InnerLoopNesting::Undecided`]. Everything is deterministic: no hashing,
//! no sorting, loops visited in face order.

use std::f64::consts::FRAC_PI_4;

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::frame::Frame3;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::predicates::orient2d;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::FaceId;
use remus_topology::wire::WireId;

#[cfg(test)]
mod tests;

/// How a planar face's inner loops nest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InnerLoopNesting {
    /// No inner loop lies inside another: every inner wire is a hole. Also
    /// the answer for a face with fewer than two inner wires.
    Flat,
    /// Each inner wire's nesting depth, in the face's inner-wire order, with
    /// at least one positive: the number of other inner loops that contain
    /// it. Every containment and the disjointness of all loops whose boxes
    /// overlap are certified.
    Nested(Vec<usize>),
    /// Nesting could not be certified: a boundary the reader declines, a
    /// probe that stays ambiguous, loops that touch or cross, or an exhausted
    /// work budget. Callers keep their subtract-every-hole arithmetic.
    Undecided,
}

impl InnerLoopNesting {
    /// The certified depths, or `None` unless some loop is certified nested.
    #[must_use]
    pub fn nested_depths(&self) -> Option<&[usize]> {
        match self {
            Self::Nested(depths) => Some(depths),
            Self::Flat | Self::Undecided => None,
        }
    }
}

/// Piece visits and bisections one face's nesting may spend.
pub const NESTING_WORK_BUDGET: usize = 1 << 20;

/// Bisections allowed below one initial piece.
const MAX_SPLITS: u32 = 52;

/// Probes tried on a loop before its containment is left undecided.
const PROBES_PER_LOOP: usize = 4;

/// Rounding margin, relative to the face's coordinate magnitude.
const MARGIN_RELATIVE: f64 = 1e-12;

/// Initial parameter sweep of one circle or ellipse piece.
const CONIC_SWEEP: f64 = FRAC_PI_4;

/// Initial parameter span of one hyperbola piece.
const HYPERBOLA_SPAN: f64 = 0.5;

/// Initial pieces one edge may produce.
const MAX_EDGE_PIECES: usize = 4096;

/// A gap between consecutive edges longer than this share of the loop's
/// diagonal means the wire is not chained as stored.
const MAX_GAP_RELATIVE: f64 = 1e-4;

/// The nesting of `face_id`'s inner loops, projected along `normal`.
///
/// `normal` only picks the projection plane; pass the normal of the frame the
/// caller integrates in. Never fails: a missing entity, an unreadable edge or
/// any doubt is [`InnerLoopNesting::Undecided`].
#[must_use]
pub fn inner_loop_nesting(topo: &Topology, face_id: FaceId, normal: Vec3) -> InnerLoopNesting {
    let Ok(face) = topo.face(face_id) else {
        return InnerLoopNesting::Undecided;
    };
    let wires = face.inner_wires();
    if wires.len() < 2 {
        return InnerLoopNesting::Flat;
    }
    let mut work = NESTING_WORK_BUDGET;
    nesting_of(topo, wires, normal, &mut work).unwrap_or(InnerLoopNesting::Undecided)
}

type P2 = [f64; 2];

/// Axis-aligned box `[x_min, y_min, x_max, y_max]`.
type Box2 = [f64; 4];

const EMPTY_BOX: Box2 = [
    f64::INFINITY,
    f64::INFINITY,
    f64::NEG_INFINITY,
    f64::NEG_INFINITY,
];

/// The in-plane frame every loop is projected into.
struct Plane {
    origin: Point3,
    x: Vec3,
    y: Vec3,
}

impl Plane {
    fn project(&self, p: Point3) -> P2 {
        let d = p - self.origin;
        [d.dot(self.x), d.dot(self.y)]
    }
}

/// What a piece is a chord of.
#[derive(Clone)]
enum Carrier<'a> {
    /// A line edge or the gap between two edges: the chord itself.
    Straight,
    /// A circle, ellipse or hyperbola arc over `(t0, t1)`; `center` is the
    /// conic's projected centre.
    Central {
        curve: Central<'a>,
        center: P2,
        t0: f64,
        t1: f64,
    },
    /// A parabola arc over `(t0, t1)`.
    Parabola {
        curve: &'a Parabola3D,
        t0: f64,
        t1: f64,
    },
    /// A rational Bézier piece: projected homogeneous control points
    /// `(w·x, w·y, w)` with positive weights.
    Bezier(Vec<[f64; 3]>),
}

/// A conic with a centre: its arcs lie in their end tangents' triangle,
/// whose apex is `centre + (C(m) − centre) / shrink(h)`.
#[derive(Clone, Copy)]
enum Central<'a> {
    Circle(&'a Circle3D),
    Ellipse(&'a Ellipse3D),
    Hyperbola(&'a Hyperbola3D),
}

impl Central<'_> {
    fn evaluate(self, t: f64) -> Point3 {
        match self {
            Self::Circle(c) => c.evaluate(t),
            Self::Ellipse(e) => e.evaluate(t),
            Self::Hyperbola(h) => h.evaluate(t),
        }
    }

    fn center(self) -> Point3 {
        match self {
            Self::Circle(c) => c.center(),
            Self::Ellipse(e) => e.center(),
            Self::Hyperbola(h) => h.center(),
        }
    }

    /// The tangent triangle's apex scale for an arc of half-span `half`:
    /// `cos` on the circle and ellipse (`|half| < π/2`), `cosh` on the
    /// hyperbola.
    fn shrink(self, half: f64) -> f64 {
        match self {
            Self::Circle(_) | Self::Ellipse(_) => half.cos(),
            Self::Hyperbola(_) => half.cosh(),
        }
    }

    /// Initial parameter span of one piece.
    const fn span(self) -> f64 {
        match self {
            Self::Circle(_) | Self::Ellipse(_) => CONIC_SWEEP,
            Self::Hyperbola(_) => HYPERBOLA_SPAN,
        }
    }
}

/// A chord `a → b` of a boundary piece the piece lies within `delta` of.
#[derive(Clone)]
struct Piece<'a> {
    a: P2,
    b: P2,
    delta: f64,
    splits: u32,
    carrier: Carrier<'a>,
}

/// One inner wire as a closed chain of pieces.
struct Loop<'a> {
    pieces: Vec<Piece<'a>>,
    /// Holds the whole loop: chords grown by their deviations.
    bounds: Box2,
    /// Holds the loop's own sampled points (on-curve points only).
    points: Box2,
    /// Points on the loop to test against other loops.
    probes: Vec<P2>,
}

/// Why a decision was not reached.
enum Fail {
    /// The probe is within the margin of the loop: try another probe.
    Ambiguous,
    /// The work budget or the arithmetic gave out: the face is undecided.
    Exhausted,
}

struct Ctx<'p> {
    plane: &'p Plane,
    margin: f64,
    work: usize,
}

impl Ctx<'_> {
    fn spend(&mut self) -> Result<(), Fail> {
        self.work = self.work.checked_sub(1).ok_or(Fail::Exhausted)?;
        Ok(())
    }
}

/// [`inner_loop_nesting`] over `wires`, spending at most `work`; `work` is
/// left holding what was not spent.
fn nesting_of(
    topo: &Topology,
    wires: &[WireId],
    normal: Vec3,
    work: &mut usize,
) -> Option<InnerLoopNesting> {
    let first_edge = topo.wire(wires[0]).ok()?.edges().first()?.edge();
    let origin = topo
        .vertex(topo.edge(first_edge).ok()?.start())
        .ok()?
        .point();
    let frame = Frame3::from_normal(origin, normal).ok()?;
    let plane = Plane {
        origin,
        x: frame.x,
        y: frame.y,
    };
    let loops = wires
        .iter()
        .map(|&wire| read_loop(topo, wire, &plane))
        .collect::<Option<Vec<_>>>()?;

    let origin_magnitude = Vec3::new(origin.x(), origin.y(), origin.z()).length();
    let reach = loops
        .iter()
        .flat_map(|l| l.bounds)
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let margin = MARGIN_RELATIVE * (origin_magnitude + reach);
    if !margin.is_finite() || margin <= 0.0 {
        return None;
    }
    let mut ctx = Ctx {
        plane: &plane,
        margin,
        work: *work,
    };
    let nesting = decide(&mut ctx, &loops);
    *work = ctx.work;
    nesting
}

fn decide(ctx: &mut Ctx<'_>, loops: &[Loop<'_>]) -> Option<InnerLoopNesting> {
    let margin = ctx.margin;

    let n = loops.len();
    let mut depths = vec![0usize; n];
    for (a, inner) in loops.iter().enumerate() {
        for (b, outer) in loops.iter().enumerate() {
            // A loop inside `outer` has every point in `outer`'s box.
            if a == b || !box_holds(outer.bounds, inner.points, margin) {
                continue;
            }
            match loop_holds(ctx, outer, &inner.probes) {
                Ok(true) => depths[a] += 1,
                Ok(false) => {}
                Err(_) => return None,
            }
        }
    }
    if depths.iter().all(|&depth| depth == 0) {
        return Some(InnerLoopNesting::Flat);
    }
    // A decided probe proves containment only for loops that do not meet.
    for (a, first) in loops.iter().enumerate() {
        for second in &loops[a + 1..] {
            if boxes_overlap(first.bounds, second.bounds, margin) {
                loops_disjoint(ctx, first, second).ok()?;
            }
        }
    }
    Some(InnerLoopNesting::Nested(depths))
}

// ── Reading ────────────────────────────────────────────────────────────────

fn read_loop<'a>(topo: &'a Topology, wire_id: WireId, plane: &Plane) -> Option<Loop<'a>> {
    let wire = topo.wire(wire_id).ok()?;
    let mut edges: Vec<Vec<Piece<'a>>> = Vec::with_capacity(wire.edges().len());
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge()).ok()?;
        let pieces = edge_pieces(topo, edge, plane)?;
        if pieces.is_empty() {
            return None;
        }
        edges.push(pieces);
    }
    if edges.is_empty() {
        return None;
    }

    // Orient each edge so it starts where the previous one ended. Parity
    // ignores direction; only the junctions the gaps close need it.
    let ends = |pieces: &[Piece<'_>]| (pieces[0].a, pieces[pieces.len() - 1].b);
    let mut reversed = vec![false; edges.len()];
    if edges.len() > 1 {
        let (s0, e0) = ends(&edges[0]);
        let (s1, e1) = ends(&edges[1]);
        let near = |p: P2| distance(p, s1).min(distance(p, e1));
        reversed[0] = near(s0) < near(e0);
    }
    let mut exit = {
        let (s, e) = ends(&edges[0]);
        if reversed[0] { s } else { e }
    };
    for (k, pieces) in edges.iter().enumerate().skip(1) {
        let (s, e) = ends(pieces);
        reversed[k] = distance(e, exit) < distance(s, exit);
        exit = if reversed[k] { s } else { e };
    }

    let mut pieces: Vec<Piece<'a>> = Vec::new();
    let mut gaps: Vec<(P2, P2)> = Vec::new();
    let mut points = EMPTY_BOX;
    let mut first_entry = None;
    let mut last_exit: Option<P2> = None;
    for (edge, flip) in edges.into_iter().zip(reversed) {
        let (s, e) = ends(&edge);
        let (entry, exit) = if flip { (e, s) } else { (s, e) };
        if let Some(previous) = last_exit {
            gaps.push((previous, entry));
        }
        first_entry.get_or_insert(entry);
        last_exit = Some(exit);
        for piece in &edge {
            points = grow(points, piece.a, 0.0);
            points = grow(points, piece.b, 0.0);
        }
        pieces.extend(edge);
    }
    gaps.push((last_exit?, first_entry?));

    // Probes: curve points spread over the loop's own pieces (not the gaps,
    // and not a zero-length line).
    let own: Vec<&Piece<'a>> = pieces
        .iter()
        .filter(|p| !matches!(p.carrier, Carrier::Straight) || !same_point(p.a, p.b))
        .collect();
    let step = own.len().div_ceil(PROBES_PER_LOOP).max(1);
    let mut probes = Vec::with_capacity(PROBES_PER_LOOP);
    for piece_in in own.into_iter().step_by(step) {
        let probe = midpoint(plane, piece_in)?;
        points = grow(points, probe, 0.0);
        probes.push(probe);
    }
    if probes.is_empty() {
        return None;
    }

    let mut bounds = EMPTY_BOX;
    for piece_in in &pieces {
        bounds = grow(bounds, piece_in.a, piece_in.delta);
        bounds = grow(bounds, piece_in.b, piece_in.delta);
    }
    let diagonal = distance([bounds[0], bounds[1]], [bounds[2], bounds[3]]);
    if !diagonal.is_finite() {
        return None;
    }
    for (from, to) in gaps {
        if same_point(from, to) {
            continue;
        }
        if distance(from, to) > MAX_GAP_RELATIVE * diagonal {
            return None;
        }
        // A gap is a straight chord of itself, inside the box already.
        pieces.push(straight(from, to));
    }
    Some(Loop {
        pieces,
        bounds,
        points,
        probes,
    })
}

/// One edge's initial pieces in increasing parameter order, consecutive
/// pieces sharing their junction points exactly.
fn edge_pieces<'a>(topo: &'a Topology, edge: &'a Edge, plane: &Plane) -> Option<Vec<Piece<'a>>> {
    let curve = edge.curve();
    match curve {
        EdgeCurve::Line => {
            let a = plane.project(topo.vertex(edge.start()).ok()?.point());
            let b = plane.project(topo.vertex(edge.end()).ok()?.point());
            Some(vec![straight(a, b)])
        }
        EdgeCurve::Circle(c) => central_pieces(edge, Central::Circle(c), plane),
        EdgeCurve::Ellipse(e) => central_pieces(edge, Central::Ellipse(e), plane),
        EdgeCurve::Hyperbola(h) => central_pieces(edge, Central::Hyperbola(h), plane),
        EdgeCurve::Parabola(parabola) => {
            let (t0, t1) = edge.strict_domain().ok()?;
            // A parabola turns less than a half turn over any span.
            parameter_pieces(plane, t0, t1, f64::INFINITY, |t0, t1| Carrier::Parabola {
                curve: parabola,
                t0,
                t1,
            })
        }
        EdgeCurve::NurbsCurve(nc) => nurbs_pieces(edge, nc, plane),
    }
}

fn central_pieces<'a>(edge: &Edge, curve: Central<'a>, plane: &Plane) -> Option<Vec<Piece<'a>>> {
    let (t0, t1) = edge.strict_domain().ok()?;
    let center = plane.project(curve.center());
    parameter_pieces(plane, t0, t1, curve.span(), |t0, t1| Carrier::Central {
        curve,
        center,
        t0,
        t1,
    })
}

/// Cut `(t0, t1)` into pieces of parameter span at most `span`.
fn parameter_pieces<'a>(
    plane: &Plane,
    t0: f64,
    t1: f64,
    span: f64,
    carrier: impl Fn(f64, f64) -> Carrier<'a>,
) -> Option<Vec<Piece<'a>>> {
    let count = ((t1 - t0).abs() / span).ceil();
    if !count.is_finite() || count > MAX_EDGE_PIECES as f64 {
        return None;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let count = (count as usize).max(1);
    #[allow(clippy::cast_precision_loss)]
    let dt = (t1 - t0) / count as f64;
    #[allow(clippy::cast_precision_loss)]
    let at = |i: usize| {
        if i == count {
            t1
        } else {
            dt.mul_add(i as f64, t0)
        }
    };
    let mut pieces = Vec::with_capacity(count);
    let mut a = point_at(plane, &carrier(t0, t1), t0)?;
    for i in 0..count {
        let (s0, s1) = (at(i), at(i + 1));
        let piece_carrier = carrier(s0, s1);
        let b = point_at(plane, &piece_carrier, s1)?;
        pieces.push(piece(plane, piece_carrier, a, b, 0)?);
        a = b;
    }
    Some(pieces)
}

/// A NURBS edge over its trim as rational Bézier pieces.
fn nurbs_pieces<'a>(edge: &Edge, nc: &NurbsCurve, plane: &Plane) -> Option<Vec<Piece<'a>>> {
    // The span the Green integration integrates (`oriented_nurbs_trim`).
    let (t0, t1) = match edge.strict_domain() {
        Ok(trim) => trim,
        Err(_) if edge.start() == edge.end() && edge.trim().is_none() => nc.domain(),
        Err(_) => return None,
    };
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let segments = remus_math::nurbs::decompose::curve_to_bezier_segments(nc).ok()?;
    if segments.len() > MAX_EDGE_PIECES {
        return None;
    }
    let mut pieces = Vec::new();
    let mut previous_end: Option<P2> = None;
    for segment in segments {
        let (s0, s1) = segment.domain();
        if s1 <= lo || s0 >= hi || s1 <= s0 {
            continue;
        }
        let scale = segment.weights().iter().fold(0.0_f64, |m, &w| m.max(w));
        if !scale.is_finite() || scale <= 0.0 {
            return None;
        }
        let mut control = Vec::with_capacity(segment.control_points().len());
        for (&p, &w) in segment.control_points().iter().zip(segment.weights()) {
            let w = w / scale;
            if !w.is_finite() || w <= 0.0 {
                return None;
            }
            let q = plane.project(p);
            control.push([q[0] * w, q[1] * w, w]);
        }
        let (mut from, to) = (s0, s1);
        if lo > from {
            control = de_casteljau(&control, (lo - from) / (to - from)).1;
            from = lo;
        }
        if hi < to {
            control = de_casteljau(&control, (hi - from) / (to - from)).0;
        }
        let a = previous_end.unwrap_or_else(|| dehomogenize(control[0]));
        let b = dehomogenize(control[control.len() - 1]);
        previous_end = Some(b);
        pieces.push(piece(plane, Carrier::Bezier(control), a, b, 0)?);
    }
    Some(pieces)
}

// ── Pieces ─────────────────────────────────────────────────────────────────

const fn straight<'a>(a: P2, b: P2) -> Piece<'a> {
    Piece {
        a,
        b,
        delta: 0.0,
        splits: 0,
        carrier: Carrier::Straight,
    }
}

/// The projected curve point at parameter `t`.
fn point_at(plane: &Plane, carrier: &Carrier<'_>, t: f64) -> Option<P2> {
    let p = match carrier {
        Carrier::Central { curve, .. } => curve.evaluate(t),
        Carrier::Parabola { curve, .. } => curve.evaluate(t),
        Carrier::Straight | Carrier::Bezier(_) => return None,
    };
    let q = plane.project(p);
    (q[0].is_finite() && q[1].is_finite()).then_some(q)
}

/// A piece over `a → b` with its certified chord deviation.
fn piece<'a>(plane: &Plane, carrier: Carrier<'a>, a: P2, b: P2, splits: u32) -> Option<Piece<'a>> {
    let delta = match &carrier {
        Carrier::Straight => 0.0,
        Carrier::Central {
            curve,
            center,
            t0,
            t1,
        } => {
            let shrink = curve.shrink(0.5 * (t1 - t0));
            if shrink.is_nan() || shrink <= 0.0 {
                return None;
            }
            let m = point_at(plane, &carrier, 0.5 * (t0 + t1))?;
            let apex = [
                center[0] + (m[0] - center[0]) / shrink,
                center[1] + (m[1] - center[1]) / shrink,
            ];
            segment_distance(apex, a, b)
        }
        Carrier::Parabola { t0, t1, .. } => {
            let m = point_at(plane, &carrier, 0.5 * (t0 + t1))?;
            let apex = [
                2.0f64.mul_add(m[0], -0.5 * (a[0] + b[0])),
                2.0f64.mul_add(m[1], -0.5 * (a[1] + b[1])),
            ];
            segment_distance(apex, a, b)
        }
        Carrier::Bezier(control) => control.iter().fold(0.0_f64, |d, &q| {
            d.max(segment_distance(dehomogenize(q), a, b))
        }),
    };
    (delta.is_finite() && a.iter().chain(&b).all(|v| v.is_finite())).then_some(Piece {
        a,
        b,
        delta,
        splits,
        carrier,
    })
}

/// Bisect a curved piece; the halves share the new junction exactly.
fn split<'a>(plane: &Plane, piece_in: &Piece<'a>) -> Option<(Piece<'a>, Piece<'a>)> {
    let splits = piece_in.splits + 1;
    match &piece_in.carrier {
        Carrier::Straight => None,
        Carrier::Central {
            curve,
            center,
            t0,
            t1,
        } => {
            let m = 0.5 * (t0 + t1);
            let left = Carrier::Central {
                curve: *curve,
                center: *center,
                t0: *t0,
                t1: m,
            };
            let right = Carrier::Central {
                curve: *curve,
                center: *center,
                t0: m,
                t1: *t1,
            };
            let mid = point_at(plane, &left, m)?;
            Some((
                piece(plane, left, piece_in.a, mid, splits)?,
                piece(plane, right, mid, piece_in.b, splits)?,
            ))
        }
        Carrier::Parabola { curve, t0, t1 } => {
            let m = 0.5 * (t0 + t1);
            let left = Carrier::Parabola {
                curve,
                t0: *t0,
                t1: m,
            };
            let right = Carrier::Parabola {
                curve,
                t0: m,
                t1: *t1,
            };
            let mid = point_at(plane, &left, m)?;
            Some((
                piece(plane, left, piece_in.a, mid, splits)?,
                piece(plane, right, mid, piece_in.b, splits)?,
            ))
        }
        Carrier::Bezier(control) => {
            let (left, right) = de_casteljau(control, 0.5);
            let mid = dehomogenize(left[left.len() - 1]);
            Some((
                piece(plane, Carrier::Bezier(left), piece_in.a, mid, splits)?,
                piece(plane, Carrier::Bezier(right), mid, piece_in.b, splits)?,
            ))
        }
    }
}

/// A point on the piece's own curve near its middle.
fn midpoint(plane: &Plane, piece_in: &Piece<'_>) -> Option<P2> {
    match piece_in.carrier {
        Carrier::Straight => Some([
            0.5 * (piece_in.a[0] + piece_in.b[0]),
            0.5 * (piece_in.a[1] + piece_in.b[1]),
        ]),
        _ => split(plane, piece_in).map(|(left, _)| left.b),
    }
}

/// Split homogeneous Bézier control points at local parameter `s`.
fn de_casteljau(control: &[[f64; 3]], s: f64) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let mut row = control.to_vec();
    let mut left = Vec::with_capacity(control.len());
    let mut right = Vec::with_capacity(control.len());
    left.push(row[0]);
    right.push(row[row.len() - 1]);
    for level in 1..control.len() {
        for i in 0..control.len() - level {
            for k in 0..3 {
                row[i][k] = (row[i + 1][k] - row[i][k]).mul_add(s, row[i][k]);
            }
        }
        left.push(row[0]);
        right.push(row[control.len() - level - 1]);
    }
    right.reverse();
    (left, right)
}

fn dehomogenize(q: [f64; 3]) -> P2 {
    [q[0] / q[2], q[1] / q[2]]
}

// ── Decisions ──────────────────────────────────────────────────────────────

/// Whether `outer` holds the loop the probes lie on, from the first probe
/// that is decided.
fn loop_holds(ctx: &mut Ctx<'_>, outer: &Loop<'_>, probes: &[P2]) -> Result<bool, Fail> {
    'probe: for &p in probes {
        if !box_holds(outer.bounds, [p[0], p[1], p[0], p[1]], ctx.margin) {
            // Outside the box that holds the loop, so not on it either.
            return Ok(false);
        }
        let mut inside = false;
        for piece_in in &outer.pieces {
            match crossing_parity(ctx, piece_in, p) {
                Ok(crosses) => inside ^= crosses,
                Err(Fail::Ambiguous) => continue 'probe,
                Err(Fail::Exhausted) => return Err(Fail::Exhausted),
            }
        }
        return Ok(inside);
    }
    Err(Fail::Ambiguous)
}

/// The parity a ray from `p` along `+x` crosses this piece with.
fn crossing_parity(ctx: &mut Ctx<'_>, piece_in: &Piece<'_>, p: P2) -> Result<bool, Fail> {
    ctx.spend()?;
    if segment_distance(p, piece_in.a, piece_in.b) > piece_in.delta + ctx.margin {
        // The piece and its chord bound a region inside the piece's convex
        // enclosure, which misses `p`: they cross the ray with equal parity.
        return Ok(crosses(p, piece_in.a, piece_in.b));
    }
    if piece_in.delta <= ctx.margin || piece_in.splits >= MAX_SPLITS {
        return Err(Fail::Ambiguous);
    }
    let (left, right) = split(ctx.plane, piece_in).ok_or(Fail::Exhausted)?;
    Ok(crossing_parity(ctx, &left, p)? ^ crossing_parity(ctx, &right, p)?)
}

/// Exact half-open crossing of segment `a → b` by the ray from `p` along
/// `+x`.
fn crosses(p: P2, a: P2, b: P2) -> bool {
    if (a[1] > p[1]) == (b[1] > p[1]) {
        return false;
    }
    let side = orient2d(point2(a), point2(b), point2(p));
    if b[1] > a[1] { side > 0.0 } else { side < 0.0 }
}

/// Certify that no point of `first` is within the margin of `second`.
fn loops_disjoint(ctx: &mut Ctx<'_>, first: &Loop<'_>, second: &Loop<'_>) -> Result<(), Fail> {
    for a in &first.pieces {
        ctx.spend()?;
        if !boxes_overlap(piece_box(a), second.bounds, ctx.margin) {
            continue;
        }
        for b in &second.pieces {
            pieces_apart(ctx, a, b)?;
        }
    }
    Ok(())
}

fn pieces_apart(ctx: &mut Ctx<'_>, a: &Piece<'_>, b: &Piece<'_>) -> Result<(), Fail> {
    ctx.spend()?;
    let reach = a.delta + b.delta + ctx.margin;
    if !boxes_overlap(chord_box(a), chord_box(b), reach)
        || segments_distance(a.a, a.b, b.a, b.b) > reach
    {
        return Ok(());
    }
    let (fat, thin, fat_first) = if a.delta >= b.delta {
        (a, b, true)
    } else {
        (b, a, false)
    };
    if fat.delta <= ctx.margin || fat.splits >= MAX_SPLITS {
        return Err(Fail::Ambiguous);
    }
    let (left, right) = split(ctx.plane, fat).ok_or(Fail::Exhausted)?;
    for half in [&left, &right] {
        if fat_first {
            pieces_apart(ctx, half, thin)?;
        } else {
            pieces_apart(ctx, thin, half)?;
        }
    }
    Ok(())
}

// ── Planar helpers ─────────────────────────────────────────────────────────

/// Whether two junctions are the same floats (shared, not merely close).
fn same_point(p: P2, q: P2) -> bool {
    p[0].to_bits() == q[0].to_bits() && p[1].to_bits() == q[1].to_bits()
}

fn point2(p: P2) -> Point2 {
    Point2::new(p[0], p[1])
}

fn distance(p: P2, q: P2) -> f64 {
    (p[0] - q[0]).hypot(p[1] - q[1])
}

fn segment_distance(p: P2, a: P2, b: P2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length2 = dx.mul_add(dx, dy * dy);
    let t = if length2 > 0.0 {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    distance(p, [dx.mul_add(t, a[0]), dy.mul_add(t, a[1])])
}

/// Distance between segments `a0 a1` and `b0 b1`: zero when they meet (as
/// exact orientation signs decide), else the nearest endpoint distance.
fn segments_distance(a0: P2, a1: P2, b0: P2, b1: P2) -> f64 {
    let sign = |a: P2, b: P2, c: P2| {
        let o = orient2d(point2(a), point2(b), point2(c));
        i8::from(o > 0.0) - i8::from(o < 0.0)
    };
    if sign(a0, a1, b0) * sign(a0, a1, b1) <= 0 && sign(b0, b1, a0) * sign(b0, b1, a1) <= 0 {
        return 0.0;
    }
    segment_distance(a0, b0, b1)
        .min(segment_distance(a1, b0, b1))
        .min(segment_distance(b0, a0, a1))
        .min(segment_distance(b1, a0, a1))
}

fn grow(bounds: Box2, p: P2, by: f64) -> Box2 {
    [
        bounds[0].min(p[0] - by),
        bounds[1].min(p[1] - by),
        bounds[2].max(p[0] + by),
        bounds[3].max(p[1] + by),
    ]
}

fn chord_box(piece_in: &Piece<'_>) -> Box2 {
    grow(grow(EMPTY_BOX, piece_in.a, 0.0), piece_in.b, 0.0)
}

fn piece_box(piece_in: &Piece<'_>) -> Box2 {
    grow(
        grow(EMPTY_BOX, piece_in.a, piece_in.delta),
        piece_in.b,
        piece_in.delta,
    )
}

/// Whether `outer`, grown by `margin`, holds `inner`.
fn box_holds(outer: Box2, inner: Box2, margin: f64) -> bool {
    inner[0] >= outer[0] - margin
        && inner[1] >= outer[1] - margin
        && inner[2] <= outer[2] + margin
        && inner[3] <= outer[3] + margin
}

/// Whether the boxes come within `gap` of each other.
fn boxes_overlap(a: Box2, b: Box2, gap: f64) -> bool {
    a[0] <= b[2] + gap && b[0] <= a[2] + gap && a[1] <= b[3] + gap && b[1] <= a[3] + gap
}
