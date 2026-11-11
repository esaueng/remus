//! Certified nesting of a planar face's inner loops.
//!
//! A planar face's inner wires alternate void and material by nesting depth:
//! a hole, an island inside it, a hole inside that island. The measurement
//! paths subtract a loop at even depth and add one back at odd depth, so the
//! depth decides the sign of an exact moment and must not be guessed.
//!
//! # The contract
//!
//! [`inner_loop_nesting`] ends in exactly one of four answers, and the type
//! makes every caller handle each:
//!
//! - [`InnerLoopNesting::Flat`]: no inner loop has a point inside another.
//!   Every candidate pair is decided by an exact parity probe, and loops
//!   whose boxes do not nest cannot nest. Callers subtract every inner wire,
//!   in order, exactly as before nesting was read.
//! - [`InnerLoopNesting::Nested`]: each loop's depth, certified. Every
//!   containment is an exact parity probe, and the loops are certified
//!   pairwise disjoint (see below), which is what makes one probe per loop a
//!   proof. Callers alternate signs by depth parity.
//! - [`InnerLoopNesting::Invalid`]: two loops are certified to come within
//!   their edges' tolerances of each other (touching or crossing), so the
//!   face is invalid. Callers keep the subtract-every-hole arithmetic, the
//!   answer such faces always had. Invalid input never newly fails.
//! - [`InnerLoopNesting::Undecided`]: none of the above could be certified.
//!   Callers must not fall back to subtract-every-hole: the remus-check
//!   integrators return [`crate::CheckError::UndecidedLoopNesting`], and the
//!   operations layer measures the face on its mesh instead.
//!
//! # How a loop is read
//!
//! - **The loop is the integrated loop.** Each inner wire is read the way the
//!   exact Green's-theorem integration reads it: a line edge between its
//!   vertices, a circle, ellipse, parabola or hyperbola edge over its strict
//!   trim, a NURBS edge over its trim (its carrier domain only for a closed
//!   edge with no trim). Only the knot spans inside the trim are read: their
//!   rational Bézier control points come straight from the blossom of the
//!   span's own `p + 1` control points, so a carrier span outside the trim
//!   costs nothing and changes nothing.
//! - **Local frame.** Every coordinate is taken relative to one origin of the
//!   face's own (the first vertex of its first inner wire) before anything
//!   is rounded, and every margin and error bound is computed from those
//!   local magnitudes. A rigid translation of the face therefore leaves the
//!   decision alone except through the rounding of the translated input
//!   itself; [`super::face_cache`] reuses a translated reading only when the
//!   two faces' nesting answers agree.
//! - **Every piece is enclosed, with its evaluation error.** A curved edge is
//!   cut into pieces, each held with its chord, a certified geometric chord
//!   deviation `g` (the piece lies within `g` of the chord in exact
//!   arithmetic) and an evaluation error bound `e` (every computed point of
//!   the edge is within `e` of the exact curve). The enclosure is the chord
//!   grown by `g + e`.
//!   - A conic arc of parameter half-span `h` around `m` lies in the triangle
//!     of its chord and its end tangents' intersection. With the conic
//!     written `c + o(t)` about its centre (vertex for a parabola), the apex
//!     sits `o(m) · sin²h / cos h` from the chord midpoint on an ellipse
//!     (`sinh²h / cosh h` on a hyperbola, `2 |a| h²` on a parabola
//!     `c + a t² + b t`). That is a product of local quantities, so a shallow
//!     arc of a huge circle keeps its sagitta instead of losing it to the
//!     difference of two carrier-scale points.
//!   - A rational Bézier piece with positive weights lies in the hull of its
//!     control points; `g` is their largest distance from the chord.
//!   - `e` is bounded from the carrier's own magnitude in the local frame:
//!     `16 ε (|c| + |a| + |b|)` for a conic, a multiple of `ε · max|P| ·
//!     w_max / w_min` over the used control points for a NURBS edge. A carrier
//!     whose centre or control points lie far from the trimmed span therefore
//!     carries a wide enclosure, and when that is wider than the gap it has to
//!     certify the answer is `Undecided`, never `Nested`.
//!   - A circle arc additionally carries its circle as an annulus (centre and
//!     radius bounds), so two circular arcs whose circles are certified apart
//!     or nested are disjoint in one comparison, at any gap the annulus can
//!     resolve.
//!
//! # Decisions
//!
//! - **Containment is exact parity.** Loop `A` is inside loop `B` when a
//!   computed point `p` of `A` is: `p` is within `e_A` of the exact curve, so
//!   it is decided only once it is farther than `g + e + e_A` plus a rounding
//!   margin from every piece of `B`. Each piece and its chord then bound a
//!   region inside the piece's enclosure that misses the disc around `p`, so
//!   the exact point's parity against `B` is `p`'s parity against `B`'s chord
//!   polygon, which exact `orient2d` with a half-open rule decides. A piece
//!   too close is bisected; a probe still undecided when the piece is flat is
//!   dropped for the next of up to four probes.
//! - **Disjointness makes the probes proofs.** Before `Nested` is reported,
//!   each loop and its innermost container, and every two loops with the same
//!   container whose boxes overlap, are certified disjoint: every pair of
//!   pieces separated by more than their enclosures plus the margin,
//!   bisecting the fatter piece until they are. Those pairs are enough: with
//!   them certified, every two loops of the face are disjoint (a loop is
//!   inside its container's region, siblings' regions are disjoint), so each
//!   probe decided the loop it lies on.
//! - **Contact is certified, not guessed.** Where two pieces cannot be
//!   separated once both are flat, they are bisected by length until an
//!   upper bound on the distance between the exact curves (chord distance,
//!   half the chord lengths and both evaluation errors) falls below the sum
//!   of the edges' tolerances: the face is then [`InnerLoopNesting::Invalid`].
//!   When neither separation nor contact can be certified (the loops are
//!   closer than the enclosures resolve, yet farther apart than their
//!   tolerances), the answer is [`NestingUndecided::BelowResolution`].
//!
//! # Cost
//!
//! One work unit is a piece read, a box comparison, a piece visit by a probe
//! or a piece-pair visit. Loops are paired by a sweep over their boxes, so
//! loops side by side cost box comparisons with their neighbours only, and a
//! loop is probed only against loops whose boxes hold it: `N` separated
//! hole-and-island pairs cost `O(N)` containment decisions. A face may spend
//! [`nesting_work_limit`] of its piece count: [`NESTING_BASE_WORK`] plus
//! [`NESTING_WORK_PER_PIECE`] per piece it reads; past that it is
//! [`NestingUndecided::WorkLimit`]. Separating two nearly coincident curves
//! that are not both circles costs about `5 / √γ` units at relative gap `γ`
//! (about three million at `γ = 1e-9`). Everything is deterministic: no
//! hashing, sorts by box coordinate with index tie-breaks.

use std::cell::Cell;
use std::f64::consts::FRAC_PI_4;

use remus_math::compact_sort::CompactSort;
use remus_math::frame::Frame3;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::predicates::orient2d;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::wire::WireId;

#[cfg(test)]
mod cache_tests;
#[cfg(test)]
mod tests;

/// How a planar face's inner loops nest. See the [module docs](self) for
/// what each answer certifies and what callers do with it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InnerLoopNesting {
    /// No inner loop has a point inside another inner loop: every inner wire
    /// is a hole. Also the answer for a face with fewer than two inner wires.
    Flat,
    /// Each inner wire's certified nesting depth, in the face's inner-wire
    /// order, with at least one positive: the number of other inner loops
    /// that contain it.
    Nested(Vec<usize>),
    /// Two inner loops are certified to touch or cross within their edges'
    /// tolerances: the face is invalid, and keeps the subtract-every-hole
    /// arithmetic it always had.
    Invalid,
    /// The nesting could not be certified. Never resolved by subtracting every
    /// hole.
    Undecided(NestingUndecided),
}

/// Why [`InnerLoopNesting::Undecided`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NestingUndecided {
    /// The face is missing, not a plane, or its normal is degenerate.
    NotPlanar,
    /// A boundary edge or wire the reader cannot enclose (a non-positive
    /// NURBS weight, an invalid trim, a wire whose edges do not chain, a
    /// non-finite coordinate).
    Unreadable,
    /// Two loops are closer than their enclosures can resolve, yet not
    /// certified to be within their tolerances of each other.
    BelowResolution,
    /// The face spent its [`nesting_work_limit`].
    WorkLimit,
}

impl std::fmt::Display for NestingUndecided {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::NotPlanar => "the face is not a readable plane",
            Self::Unreadable => "an inner wire cannot be enclosed",
            Self::BelowResolution => "two inner loops are closer than their enclosures resolve",
            Self::WorkLimit => "the nesting work limit was spent",
        })
    }
}

/// An [`InnerLoopNesting`] with what it cost.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NestingReport {
    /// The answer.
    pub nesting: InnerLoopNesting,
    /// Pieces read from the inner wires.
    pub pieces: usize,
    /// Work units spent, reading included.
    pub work: usize,
    /// Work units the face was allowed.
    pub work_limit: usize,
}

/// Work every face may spend whatever its size.
pub const NESTING_BASE_WORK: usize = 1 << 23;

/// Work a face may spend per piece it reads.
pub const NESTING_WORK_PER_PIECE: usize = 1 << 12;

/// The work a face whose inner wires read as `pieces` pieces may spend.
#[must_use]
pub const fn nesting_work_limit(pieces: usize) -> usize {
    NESTING_BASE_WORK.saturating_add(NESTING_WORK_PER_PIECE.saturating_mul(pieces))
}

thread_local! {
    /// A test and diagnostic override of [`nesting_work_limit`].
    static WORK_LIMIT_OVERRIDE: Cell<Option<usize>> = const { Cell::new(None) };
}

/// Run `f` with every nesting decision on this thread limited to `limit`
/// work units instead of [`nesting_work_limit`]. For tests and diagnostics
/// that need an [`InnerLoopNesting::Undecided`] face; the previous limit is
/// restored when `f` returns or unwinds.
#[doc(hidden)]
pub fn with_nesting_work_limit<R>(limit: usize, f: impl FnOnce() -> R) -> R {
    struct Restore(Option<usize>);
    impl Drop for Restore {
        fn drop(&mut self) {
            let previous = self.0;
            WORK_LIMIT_OVERRIDE.with(|cell| cell.set(previous));
        }
    }
    let previous = WORK_LIMIT_OVERRIDE.with(|cell| cell.replace(Some(limit)));
    let _restore = Restore(previous);
    f()
}

/// The nesting of `face_id`'s inner loops, projected along the face's
/// stored plane normal.
#[must_use]
pub fn inner_loop_nesting(topo: &Topology, face_id: FaceId) -> InnerLoopNesting {
    inner_loop_nesting_report(topo, face_id).nesting
}

/// [`inner_loop_nesting`] with its cost.
#[must_use]
pub fn inner_loop_nesting_report(topo: &Topology, face_id: FaceId) -> NestingReport {
    let Ok(face) = topo.face(face_id) else {
        return report(
            InnerLoopNesting::Undecided(NestingUndecided::NotPlanar),
            0,
            0,
            0,
        );
    };
    let wires = face.inner_wires();
    if wires.len() < 2 {
        return report(InnerLoopNesting::Flat, 0, 0, 0);
    }
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        return report(
            InnerLoopNesting::Undecided(NestingUndecided::NotPlanar),
            0,
            0,
            0,
        );
    };
    nesting_of(topo, wires, *normal)
}

const fn report(
    nesting: InnerLoopNesting,
    pieces: usize,
    work: usize,
    limit: usize,
) -> NestingReport {
    NestingReport {
        nesting,
        pieces,
        work,
        work_limit: limit,
    }
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

const EPS: f64 = f64::EPSILON;

/// Bisections allowed below one initial piece.
const MAX_SPLITS: u32 = 60;

/// Probes tried on a loop before its containment is left undecided.
const PROBES_PER_LOOP: usize = 4;

/// Rounding margin of the planar arithmetic, relative to the local reach.
const MARGIN_ULPS: f64 = 256.0;

/// Relative slack on a computed geometric chord deviation.
const GEOM_SLACK: f64 = 32.0 * EPS;

/// Initial parameter sweep of one circle or ellipse piece.
const ELLIPTIC_SPAN: f64 = FRAC_PI_4;

/// Initial parameter span of one hyperbola piece.
const HYPERBOLIC_SPAN: f64 = 0.5;

/// Initial pieces one conic edge may produce.
const MAX_EDGE_PIECES: f64 = 65_536.0;

/// A gap between consecutive edges longer than this share of the loop's
/// diagonal means the wire is not chained as stored.
const MAX_GAP_RELATIVE: f64 = 1e-4;

/// The in-plane frame every loop is projected into: the local origin and
/// two in-plane axes.
struct Plane {
    origin: Point3,
    x: Vec3,
    y: Vec3,
}

impl Plane {
    /// `p − origin` in the frame, with its rounding bound.
    fn point(&self, p: Point3) -> (P2, f64) {
        let d = p - self.origin;
        let size = d.x().abs() + d.y().abs() + d.z().abs();
        ([d.dot(self.x), d.dot(self.y)], 8.0 * EPS * size)
    }

    fn vector(&self, v: Vec3) -> P2 {
        [v.dot(self.x), v.dot(self.y)]
    }
}

/// How a conic's offset from its centre runs with the parameter.
#[derive(Debug, Clone, Copy)]
enum ConicKind {
    /// `a cos t + b sin t`: circle and ellipse.
    Elliptic,
    /// `a cosh t + b sinh t`.
    Hyperbolic,
    /// `a t² + b t` about the vertex.
    Parabolic,
}

/// A circle carrier as an annulus: every point of the exact projected circle
/// lies between `inner` and `outer` of a centre within `centre_error` of
/// `centre`.
#[derive(Debug, Clone, Copy)]
struct Annulus {
    centre: P2,
    centre_error: f64,
    inner: f64,
    outer: f64,
}

/// A conic in the local frame: `c + a·f(t) + b·g(t)`.
#[derive(Debug, Clone, Copy)]
struct Conic2 {
    kind: ConicKind,
    c: P2,
    a: P2,
    b: P2,
    annulus: Option<Annulus>,
}

impl Conic2 {
    fn offset(&self, t: f64) -> P2 {
        let (f, g) = match self.kind {
            ConicKind::Elliptic => (t.cos(), t.sin()),
            ConicKind::Hyperbolic => (t.cosh(), t.sinh()),
            ConicKind::Parabolic => (t * t, t),
        };
        [
            self.a[0].mul_add(f, self.b[0] * g),
            self.a[1].mul_add(f, self.b[1] * g),
        ]
    }

    fn point(&self, t: f64) -> P2 {
        let o = self.offset(t);
        [self.c[0] + o[0], self.c[1] + o[1]]
    }

    /// The certified distance of the arc over `(t0, t1)` from its exact chord:
    /// the tangent triangle's apex height, written as a product of local
    /// quantities.
    fn chord_deviation(&self, t0: f64, t1: f64) -> f64 {
        let h = 0.5 * (t1 - t0);
        let m = 0.5 * (t0 + t1);
        match self.kind {
            ConicKind::Elliptic => {
                let (s, c) = h.sin_cos();
                norm(self.offset(m)) * (s * s / c)
            }
            ConicKind::Hyperbolic => {
                let s = h.sinh();
                norm(self.offset(m)) * (s * s / h.cosh())
            }
            ConicKind::Parabolic => 2.0 * norm(self.a) * h * h,
        }
    }

    const fn span(&self) -> f64 {
        match self.kind {
            ConicKind::Elliptic => ELLIPTIC_SPAN,
            ConicKind::Hyperbolic => HYPERBOLIC_SPAN,
            // A parabola's end tangents meet over any span.
            ConicKind::Parabolic => f64::INFINITY,
        }
    }
}

/// What a piece is a chord of.
#[derive(Debug, Clone)]
enum Carrier {
    /// A line edge or the gap between two edges: the chord itself.
    Straight,
    /// A conic arc over `(t0, t1)`.
    Conic { conic: Conic2, t0: f64, t1: f64 },
    /// A rational Bézier piece: local homogeneous control points
    /// `(w·x, w·y, w)` with positive weights.
    Bezier(Vec<[f64; 3]>),
}

/// A chord `a → b` of a boundary piece and its enclosure.
#[derive(Debug, Clone)]
struct Piece {
    a: P2,
    b: P2,
    /// Certified distance of the exact piece from its exact chord.
    geom: f64,
    /// Bound on the distance of every computed point of the piece (chord
    /// ends, probes, control points) from the exact curve.
    err: f64,
    /// The edge's tolerance.
    tol: f64,
    splits: u32,
    carrier: Carrier,
}

impl Piece {
    /// The enclosure half-width: the exact piece lies within it of the
    /// computed chord.
    fn delta(&self) -> f64 {
        self.geom.mul_add(1.0 + GEOM_SLACK, self.err)
    }

    const fn curved(&self) -> bool {
        !matches!(self.carrier, Carrier::Straight)
    }

    fn length(&self) -> f64 {
        distance(self.a, self.b)
    }
}

/// One inner wire as a closed chain of pieces.
struct Loop {
    pieces: Vec<Piece>,
    /// Holds the whole loop: chords grown by their enclosures.
    bounds: Box2,
    /// Holds the loop's computed on-curve points.
    points: Box2,
    /// The largest evaluation error of the loop's points.
    err: f64,
    /// Points on the loop to test against other loops, with their errors.
    probes: Vec<(P2, f64)>,
}

/// Why a decision was not reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Fail {
    /// Closer than the enclosures resolve, contact not certified.
    Ambiguous,
    /// Two pieces certified within their tolerances of each other.
    Contact,
    /// The work limit is spent.
    Exhausted,
}

struct Ctx {
    margin: f64,
    work: usize,
    limit: usize,
}

impl Ctx {
    fn spend(&mut self, units: usize) -> Result<(), Fail> {
        self.work = self.work.saturating_add(units);
        if self.work > self.limit {
            Err(Fail::Exhausted)
        } else {
            Ok(())
        }
    }
}

/// [`inner_loop_nesting`] over `wires`, projected along `normal`.
pub(crate) fn nesting_of(topo: &Topology, wires: &[WireId], normal: Vec3) -> NestingReport {
    let unreadable = |pieces| {
        report(
            InnerLoopNesting::Undecided(NestingUndecided::Unreadable),
            pieces,
            pieces,
            nesting_work_limit(pieces),
        )
    };
    let Some(origin) = first_vertex(topo, wires[0]) else {
        return unreadable(0);
    };
    let Ok(frame) = Frame3::from_normal(origin, normal) else {
        return report(
            InnerLoopNesting::Undecided(NestingUndecided::NotPlanar),
            0,
            0,
            0,
        );
    };
    let plane = Plane {
        origin,
        x: frame.x,
        y: frame.y,
    };
    let mut loops = Vec::with_capacity(wires.len());
    let mut pieces = 0usize;
    for &wire in wires {
        let Some(l) = read_loop(topo, wire, &plane) else {
            return unreadable(pieces);
        };
        pieces += l.pieces.len();
        loops.push(l);
    }
    let reach = loops
        .iter()
        .flat_map(|l| l.bounds)
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let margin = MARGIN_ULPS * EPS * reach;
    if !margin.is_finite() || margin <= 0.0 {
        return unreadable(pieces);
    }
    let limit = WORK_LIMIT_OVERRIDE
        .with(Cell::get)
        .unwrap_or_else(|| nesting_work_limit(pieces));
    let mut ctx = Ctx {
        margin,
        work: 0,
        limit,
    };
    let nesting = match ctx.spend(pieces) {
        Ok(()) => decide(&mut ctx, &loops),
        Err(_) => InnerLoopNesting::Undecided(NestingUndecided::WorkLimit),
    };
    report(nesting, pieces, ctx.work.min(limit), limit)
}

fn first_vertex(topo: &Topology, wire: WireId) -> Option<Point3> {
    let first_edge = topo.wire(wire).ok()?.edges().first()?.edge();
    Some(
        topo.vertex(topo.edge(first_edge).ok()?.start())
            .ok()?
            .point(),
    )
}

const fn undecided(fail: Fail) -> InnerLoopNesting {
    match fail {
        Fail::Contact => InnerLoopNesting::Invalid,
        Fail::Ambiguous => InnerLoopNesting::Undecided(NestingUndecided::BelowResolution),
        Fail::Exhausted => InnerLoopNesting::Undecided(NestingUndecided::WorkLimit),
    }
}

#[allow(clippy::too_many_lines)]
fn decide(ctx: &mut Ctx, loops: &[Loop]) -> InnerLoopNesting {
    let n = loops.len();
    let pairs = match overlapping_pairs(ctx, loops) {
        Ok(pairs) => pairs,
        Err(fail) => return undecided(fail),
    };

    // Containment, probed only where the inner loop's points lie in the
    // outer loop's box.
    let mut containers: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut ambiguous = Vec::new();
    for &(i, j) in &pairs {
        for (inner, outer) in [(i, j), (j, i)] {
            let (inside, holder) = (&loops[inner], &loops[outer]);
            if !box_holds(holder.bounds, inside.points, ctx.margin + inside.err) {
                continue;
            }
            match loop_holds(ctx, holder, &inside.probes) {
                Ok(true) => containers[inner].push(outer),
                Ok(false) => {}
                Err(Fail::Ambiguous) => ambiguous.push((inner, outer)),
                Err(fail) => return undecided(fail),
            }
        }
    }
    // A pair no probe could decide is either certified in contact (an
    // invalid face) or undecided: either loop may hold the other.
    if !ambiguous.is_empty() {
        let mut verdict = Fail::Ambiguous;
        for &(inner, outer) in &ambiguous {
            match loops_disjoint(ctx, &loops[inner], &loops[outer]) {
                Err(Fail::Contact) => return InnerLoopNesting::Invalid,
                Err(Fail::Exhausted) => {
                    verdict = Fail::Exhausted;
                    break;
                }
                Ok(()) | Err(Fail::Ambiguous) => {}
            }
        }
        return undecided(verdict);
    }
    let depths: Vec<usize> = containers.iter().map(Vec::len).collect();
    if depths.iter().all(|&depth| depth == 0) {
        return InnerLoopNesting::Flat;
    }

    // The containment tree: a loop's innermost container is the one of
    // depth one less, and its containers are that container's plus itself.
    let mut parent: Vec<Option<usize>> = vec![None; n];
    let mut consistent = true;
    for a in 0..n {
        if depths[a] == 0 {
            continue;
        }
        let mut found = None;
        for &p in &containers[a] {
            if depths[p] + 1 == depths[a] {
                if found.is_some() {
                    consistent = false;
                }
                found = Some(p);
            }
        }
        let Some(p) = found else {
            consistent = false;
            continue;
        };
        let mut above: Vec<usize> = containers[a].iter().copied().filter(|&q| q != p).collect();
        let mut theirs = containers[p].clone();
        above.compact_sort_unstable();
        theirs.compact_sort_unstable();
        consistent &= above == theirs;
        parent[a] = Some(p);
    }

    // Disjointness of each loop and its container and of overlapping
    // siblings proves every pair disjoint; without a consistent tree, every
    // overlapping pair is searched for the contact that must exist.
    let mut checks: Vec<(usize, usize)> = Vec::new();
    if consistent {
        for (a, p) in parent.iter().enumerate() {
            if let Some(p) = *p {
                checks.push((a, p));
            }
        }
        checks.extend(
            pairs
                .iter()
                .copied()
                .filter(|&(i, j)| parent[i] == parent[j]),
        );
    } else {
        checks.extend_from_slice(&pairs);
    }
    let mut verdict = None;
    for (i, j) in checks {
        match loops_disjoint(ctx, &loops[i], &loops[j]) {
            Ok(()) => {}
            Err(Fail::Contact) => return InnerLoopNesting::Invalid,
            Err(Fail::Ambiguous) => verdict = Some(Fail::Ambiguous),
            Err(Fail::Exhausted) => return undecided(Fail::Exhausted),
        }
    }
    match (consistent, verdict) {
        (true, None) => InnerLoopNesting::Nested(depths),
        (_, Some(fail)) => undecided(fail),
        // Every overlapping pair disjoint yet the containments not nested:
        // the probes cannot all be right, so nothing is certified.
        (false, None) => InnerLoopNesting::Undecided(NestingUndecided::BelowResolution),
    }
}

/// Every pair of loops whose boxes come within the margin of each other, by
/// a sweep over the boxes' `x` ranges.
fn overlapping_pairs(ctx: &mut Ctx, loops: &[Loop]) -> Result<Vec<(usize, usize)>, Fail> {
    let margin = ctx.margin;
    let mut order: Vec<usize> = (0..loops.len()).collect();
    order.compact_sort_by(|&i, &j| {
        loops[i].bounds[0]
            .total_cmp(&loops[j].bounds[0])
            .then(i.cmp(&j))
    });
    let mut active: Vec<usize> = Vec::new();
    let mut pairs = Vec::new();
    for &i in &order {
        let x0 = loops[i].bounds[0];
        ctx.spend(active.len().max(1))?;
        active.retain(|&j| loops[j].bounds[2] + margin >= x0);
        for &j in &active {
            if boxes_overlap(loops[i].bounds, loops[j].bounds, margin) {
                pairs.push((j.min(i), j.max(i)));
            }
        }
        active.push(i);
    }
    Ok(pairs)
}

// ── Reading ────────────────────────────────────────────────────────────────

fn read_loop(topo: &Topology, wire_id: WireId, plane: &Plane) -> Option<Loop> {
    let wire = topo.wire(wire_id).ok()?;
    let mut edges: Vec<Vec<Piece>> = Vec::with_capacity(wire.edges().len());
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
    let ends = |pieces: &[Piece]| (pieces[0].a, pieces[pieces.len() - 1].b);
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

    let mut pieces: Vec<Piece> = Vec::new();
    // A gap's chord, its error and its tolerance.
    let mut gaps: Vec<(P2, P2, f64, f64)> = Vec::new();
    let mut points = EMPTY_BOX;
    let mut first: Option<(P2, f64, f64)> = None;
    let mut last: Option<(P2, f64, f64)> = None;
    for (edge, flip) in edges.into_iter().zip(reversed) {
        let (s, e) = ends(&edge);
        let (entry, exit) = if flip { (e, s) } else { (s, e) };
        let (err, tol) = (edge[0].err, edge[0].tol);
        if let Some((previous, previous_err, previous_tol)) = last {
            gaps.push((
                previous,
                entry,
                err.max(previous_err),
                tol.max(previous_tol),
            ));
        }
        first.get_or_insert((entry, err, tol));
        last = Some((exit, err, tol));
        for piece in &edge {
            points = grow(points, piece.a, 0.0);
            points = grow(points, piece.b, 0.0);
        }
        pieces.extend(edge);
    }
    let ((exit, exit_err, exit_tol), (entry, entry_err, entry_tol)) = (last?, first?);
    gaps.push((
        exit,
        entry,
        exit_err.max(entry_err),
        exit_tol.max(entry_tol),
    ));

    // Probes: curve points spread over the loop's own pieces (not the gaps,
    // and not a zero-length line).
    let own: Vec<&Piece> = pieces
        .iter()
        .filter(|p| p.curved() || !same_point(p.a, p.b))
        .collect();
    let step = own.len().div_ceil(PROBES_PER_LOOP).max(1);
    let mut probes = Vec::with_capacity(PROBES_PER_LOOP);
    for piece_in in own.into_iter().step_by(step) {
        let probe = midpoint(piece_in);
        points = grow(points, probe, 0.0);
        probes.push((probe, piece_in.err));
    }
    if probes.is_empty() {
        return None;
    }

    let mut bounds = EMPTY_BOX;
    let mut err = 0.0_f64;
    for piece_in in &pieces {
        let delta = piece_in.delta();
        bounds = grow(bounds, piece_in.a, delta);
        bounds = grow(bounds, piece_in.b, delta);
        err = err.max(piece_in.err);
    }
    let diagonal = distance([bounds[0], bounds[1]], [bounds[2], bounds[3]]);
    if !diagonal.is_finite() || !err.is_finite() {
        return None;
    }
    for (from, to, gap_err, gap_tol) in gaps {
        if same_point(from, to) {
            continue;
        }
        if distance(from, to) > MAX_GAP_RELATIVE * diagonal {
            return None;
        }
        // A gap is a straight chord of itself, inside the box already.
        pieces.push(straight(from, to, gap_err, gap_tol));
    }
    Some(Loop {
        pieces,
        bounds,
        points,
        err,
        probes,
    })
}

/// One edge's initial pieces in increasing parameter order, consecutive
/// pieces sharing their junction points exactly.
fn edge_pieces(topo: &Topology, edge: &Edge, plane: &Plane) -> Option<Vec<Piece>> {
    let start = topo.vertex(edge.start()).ok()?;
    let end = topo.vertex(edge.end()).ok()?;
    let tol = edge.effective_tolerance(start.tolerance().max(end.tolerance()));
    if !tol.is_finite() || tol < 0.0 {
        return None;
    }
    match edge.curve() {
        EdgeCurve::Line => {
            let (a, ea) = plane.point(start.point());
            let (b, eb) = plane.point(end.point());
            Some(vec![straight(a, b, ea.max(eb), tol)])
        }
        EdgeCurve::Circle(c) => {
            let (centre, centre_err) = plane.point(c.center());
            let r = c.radius();
            let (u, v) = (plane.vector(c.u_axis()), plane.vector(c.v_axis()));
            let conic = Conic2 {
                kind: ConicKind::Elliptic,
                c: centre,
                a: scale(u, r),
                b: scale(v, r),
                annulus: Some(annulus(centre, centre_err, u, v, r)),
            };
            conic_pieces(edge, conic, centre_err, 2.0 * r, tol)
        }
        EdgeCurve::Ellipse(e) => {
            let (centre, centre_err) = plane.point(e.center());
            let conic = Conic2 {
                kind: ConicKind::Elliptic,
                c: centre,
                a: scale(plane.vector(e.u_axis()), e.semi_major()),
                b: scale(plane.vector(e.v_axis()), e.semi_minor()),
                annulus: None,
            };
            conic_pieces(
                edge,
                conic,
                centre_err,
                e.semi_major() + e.semi_minor(),
                tol,
            )
        }
        EdgeCurve::Hyperbola(h) => {
            let (centre, centre_err) = plane.point(h.center());
            let (t0, t1) = edge.strict_domain().ok()?;
            let conic = Conic2 {
                kind: ConicKind::Hyperbolic,
                c: centre,
                a: scale(plane.vector(h.u_axis()), h.semi_major()),
                b: scale(plane.vector(h.v_axis()), h.semi_minor()),
                annulus: None,
            };
            let reach = (h.semi_major() + h.semi_minor()) * t0.abs().max(t1.abs()).cosh();
            conic_pieces(edge, conic, centre_err, reach, tol)
        }
        EdgeCurve::Parabola(p) => {
            let (vertex, vertex_err) = plane.point(p.vertex());
            let (t0, t1) = edge.strict_domain().ok()?;
            let conic = Conic2 {
                kind: ConicKind::Parabolic,
                c: vertex,
                a: scale(plane.vector(p.axis_dir()), 0.25 / p.focal_length()),
                b: plane.vector(p.u_axis()),
                annulus: None,
            };
            let t = t0.abs().max(t1.abs());
            let reach = norm1(conic.a).mul_add(t * t, norm1(conic.b) * t);
            conic_pieces(edge, conic, vertex_err, reach, tol)
        }
        EdgeCurve::NurbsCurve(nc) => nurbs_pieces(edge, nc, plane, tol),
    }
}

/// The annulus a projected circle `centre + r (cos t·u + sin t·v)` lies in.
fn annulus(centre: P2, centre_error: f64, u: P2, v: P2, r: f64) -> Annulus {
    // The squared stretch of `[u v]` lies within Gershgorin's bounds of
    // `[u v]ᵀ[u v]`, widened for the rounding of `u` and `v` themselves.
    let (p, q, s) = (dot(u, u), dot(v, v), dot(u, v).abs());
    let high = p.max(q) + s + 16.0 * EPS;
    let low = (p.min(q) - s - 16.0 * EPS).max(0.0);
    Annulus {
        centre,
        centre_error,
        inner: r * low.sqrt() * (1.0 - 8.0 * EPS),
        outer: r * high.sqrt() * (1.0 + 8.0 * EPS),
    }
}

/// A conic edge over its strict trim. `centre_error` is the rounding of the
/// projected centre and `reach` bounds `|a·f(t)| + |b·g(t)|` over the trim.
fn conic_pieces(
    edge: &Edge,
    conic: Conic2,
    centre_error: f64,
    reach: f64,
    tol: f64,
) -> Option<Vec<Piece>> {
    let (t0, t1) = edge.strict_domain().ok()?;
    let c = norm1(conic.c);
    // Every computed point: the centre's projection, the offset's products
    // and sums, the rounding of `cos`/`sin` (or `cosh`/`sinh`), and the final
    // sum, each bounded by the local magnitudes.
    let err = centre_error + 16.0 * EPS * (c + reach);
    if !err.is_finite() || !reach.is_finite() {
        return None;
    }
    let count = ((t1 - t0).abs() / conic.span()).ceil();
    if !count.is_finite() || count > MAX_EDGE_PIECES {
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
    let mut a = conic.point(t0);
    for i in 0..count {
        let (s0, s1) = (at(i), at(i + 1));
        let b = conic.point(s1);
        pieces.push(conic_piece(conic, s0, s1, a, b, err, tol, 0)?);
        a = b;
    }
    Some(pieces)
}

#[allow(clippy::too_many_arguments)]
fn conic_piece(
    conic: Conic2,
    t0: f64,
    t1: f64,
    a: P2,
    b: P2,
    err: f64,
    tol: f64,
    splits: u32,
) -> Option<Piece> {
    let geom = conic.chord_deviation(t0, t1);
    finite_piece(Piece {
        a,
        b,
        geom,
        err,
        tol,
        splits,
        carrier: Carrier::Conic { conic, t0, t1 },
    })
}

/// A NURBS edge over its trim as rational Bézier pieces, one per knot span
/// inside the trim. Each piece's control points are the blossom of its
/// span's `p + 1` control points at the trimmed span ends, so nothing
/// outside the trim is read.
fn nurbs_pieces(edge: &Edge, nc: &NurbsCurve, plane: &Plane, tol: f64) -> Option<Vec<Piece>> {
    // The span the Green integration integrates (`oriented_nurbs_trim`).
    let (t0, t1) = match edge.strict_domain() {
        Ok(trim) => trim,
        Err(_) if edge.start() == edge.end() && edge.trim().is_none() => nc.domain(),
        Err(_) => return None,
    };
    let p = nc.degree();
    let knots = nc.knots();
    let points = nc.control_points();
    let weights = nc.weights();
    let n = points.len();
    if p == 0 || n <= p || knots.len() != n + p + 1 || weights.len() != n {
        return None;
    }
    let (lo, hi) = (t0.min(t1).max(knots[p]), t0.max(t1).min(knots[n]));
    if lo.is_nan() || hi.is_nan() || lo >= hi {
        return None;
    }
    // The first span holding `lo`.
    let mut k = knots[p..=n].partition_point(|&u| u <= lo) + p - 1;
    k = k.clamp(p, n - 1);

    let mut spans: Vec<(usize, f64, f64)> = Vec::new();
    while k < n && knots[k] < hi {
        let (a, b) = (knots[k].max(lo), knots[k + 1].min(hi));
        if b > a {
            spans.push((k, a, b));
        }
        k += 1;
    }
    if spans.is_empty() {
        return None;
    }
    // The used control points: their local magnitude and weight spread
    // bound every rounding below.
    let (first, last) = (spans[0].0 - p, spans[spans.len() - 1].0);
    let mut magnitude = 0.0_f64;
    let (mut w_min, mut w_max) = (f64::INFINITY, 0.0_f64);
    let mut local = Vec::with_capacity(last - first + 1);
    for (&point, &w) in points[first..=last].iter().zip(&weights[first..=last]) {
        let (q, rounding) = plane.point(point);
        magnitude = magnitude.max(rounding / (8.0 * EPS));
        if !w.is_finite() || w <= 0.0 {
            return None;
        }
        w_min = w_min.min(w);
        w_max = w_max.max(w);
        local.push((q, w));
    }
    let w_scale = w_max;
    #[allow(clippy::cast_precision_loss)]
    let levels = (p as f64) * f64::from(MAX_SPLITS + 2);
    // Projection, homogenisation, `p` blossom levels and later up to
    // `MAX_SPLITS` de Casteljau levels of `p` steps each, every one a convex
    // combination, then the division by a weight at least `w_min`.
    let err = 8.0f64.mul_add(levels, 16.0) * EPS * magnitude * (w_max / w_min);
    if !err.is_finite() {
        return None;
    }
    let homogeneous: Vec<[f64; 3]> = local
        .iter()
        .map(|&(q, w)| {
            let w = w / w_scale;
            [q[0] * w, q[1] * w, w]
        })
        .collect();

    let mut pieces = Vec::with_capacity(spans.len());
    let mut previous_end: Option<P2> = None;
    for (span, a, b) in spans {
        let window = &homogeneous[span - p - first..=span - first];
        let control: Vec<[f64; 3]> = (0..=p)
            .map(|i| blossom(window, knots, span, p, a, b, i))
            .collect();
        if control.iter().any(|q| q[2].is_nan() || q[2] <= 0.0) {
            return None;
        }
        let start = previous_end.unwrap_or_else(|| dehomogenize(control[0]));
        let end = dehomogenize(control[p]);
        previous_end = Some(end);
        pieces.push(bezier_piece(control, start, end, err, tol, 0)?);
    }
    Some(pieces)
}

/// The Bézier control point `i` of span `k` restricted to `(a, b)`: the
/// blossom at `a` repeated `p − i` times and `b` repeated `i` times, by de
/// Boor's triangle over the span's own control points `window`.
fn blossom(
    window: &[[f64; 3]],
    knots: &[f64],
    k: usize,
    p: usize,
    a: f64,
    b: f64,
    i: usize,
) -> [f64; 3] {
    let mut d = window.to_vec();
    for r in 1..=p {
        let t = if r <= p - i { a } else { b };
        for j in (r..=p).rev() {
            let g = k - p + j;
            let (lo, hi) = (knots[g], knots[g + p + 1 - r]);
            let alpha = (t - lo) / (hi - lo);
            for c in 0..3 {
                d[j][c] = (d[j][c] - d[j - 1][c]).mul_add(alpha, d[j - 1][c]);
            }
        }
    }
    d[p]
}

fn bezier_piece(
    control: Vec<[f64; 3]>,
    a: P2,
    b: P2,
    err: f64,
    tol: f64,
    splits: u32,
) -> Option<Piece> {
    let geom = control.iter().fold(0.0_f64, |d, &q| {
        d.max(segment_distance(dehomogenize(q), a, b))
    });
    finite_piece(Piece {
        a,
        b,
        geom,
        // The control points and chord ends both carry the edge's error.
        err: 2.0 * err,
        tol,
        splits,
        carrier: Carrier::Bezier(control),
    })
}

fn finite_piece(piece: Piece) -> Option<Piece> {
    (piece.geom.is_finite()
        && piece.err.is_finite()
        && piece.a.iter().chain(&piece.b).all(|v| v.is_finite()))
    .then_some(piece)
}

// ── Pieces ─────────────────────────────────────────────────────────────────

const fn straight(a: P2, b: P2, err: f64, tol: f64) -> Piece {
    Piece {
        a,
        b,
        geom: 0.0,
        err,
        tol,
        splits: 0,
        carrier: Carrier::Straight,
    }
}

/// Bisect a curved piece; the halves share the new junction exactly.
fn split(piece_in: &Piece) -> Option<(Piece, Piece)> {
    let splits = piece_in.splits + 1;
    match &piece_in.carrier {
        Carrier::Straight => None,
        Carrier::Conic { conic, t0, t1 } => {
            let m = 0.5 * (t0 + t1);
            let mid = conic.point(m);
            let (err, tol) = (piece_in.err, piece_in.tol);
            Some((
                conic_piece(*conic, *t0, m, piece_in.a, mid, err, tol, splits)?,
                conic_piece(*conic, m, *t1, mid, piece_in.b, err, tol, splits)?,
            ))
        }
        Carrier::Bezier(control) => {
            let (left, right) = de_casteljau(control, 0.5);
            let mid = dehomogenize(left[left.len() - 1]);
            let (err, tol) = (0.5 * piece_in.err, piece_in.tol);
            Some((
                bezier_piece(left, piece_in.a, mid, err, tol, splits)?,
                bezier_piece(right, mid, piece_in.b, err, tol, splits)?,
            ))
        }
    }
}

/// A computed point on the piece's own curve near its middle.
fn midpoint(piece_in: &Piece) -> P2 {
    match &piece_in.carrier {
        Carrier::Straight => [
            0.5 * (piece_in.a[0] + piece_in.b[0]),
            0.5 * (piece_in.a[1] + piece_in.b[1]),
        ],
        Carrier::Conic { conic, t0, t1 } => conic.point(0.5 * (t0 + t1)),
        Carrier::Bezier(control) => {
            let (left, _) = de_casteljau(control, 0.5);
            dehomogenize(left[left.len() - 1])
        }
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
fn loop_holds(ctx: &mut Ctx, outer: &Loop, probes: &[(P2, f64)]) -> Result<bool, Fail> {
    'probe: for &(p, radius) in probes {
        if !box_holds(outer.bounds, [p[0], p[1], p[0], p[1]], ctx.margin + radius) {
            // Outside the box that holds the loop, so not on it either.
            return Ok(false);
        }
        let mut inside = false;
        for piece_in in &outer.pieces {
            match crossing_parity(ctx, piece_in, p, radius) {
                Ok(crosses) => inside ^= crosses,
                Err(Fail::Ambiguous) => continue 'probe,
                Err(fail) => return Err(fail),
            }
        }
        return Ok(inside);
    }
    Err(Fail::Ambiguous)
}

/// The parity a ray from `p` along `+x` crosses this piece with, valid for
/// every point within `radius` of `p`.
fn crossing_parity(ctx: &mut Ctx, piece_in: &Piece, p: P2, radius: f64) -> Result<bool, Fail> {
    ctx.spend(1)?;
    let reach = piece_in.delta() + ctx.margin + radius;
    let clear = p[1] + reach < piece_in.a[1].min(piece_in.b[1])
        || p[1] - reach > piece_in.a[1].max(piece_in.b[1])
        || p[0] - reach > piece_in.a[0].max(piece_in.b[0]);
    if clear || segment_distance(p, piece_in.a, piece_in.b) > reach {
        // The piece and its chord bound a region inside the piece's convex
        // enclosure, which misses the disc around `p`: they cross the ray
        // with equal parity.
        return Ok(crosses(p, piece_in.a, piece_in.b));
    }
    if piece_in.geom <= ctx.margin || piece_in.splits >= MAX_SPLITS {
        return Err(Fail::Ambiguous);
    }
    let (left, right) = split(piece_in).ok_or(Fail::Ambiguous)?;
    Ok(crossing_parity(ctx, &left, p, radius)? ^ crossing_parity(ctx, &right, p, radius)?)
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

/// Certify that the two loops are disjoint, or that they come within their
/// tolerances of each other. Pieces are paired by a sweep over their boxes.
fn loops_disjoint(ctx: &mut Ctx, first: &Loop, second: &Loop) -> Result<(), Fail> {
    let margin = ctx.margin;
    let boxes = |l: &Loop| -> Vec<Box2> { l.pieces.iter().map(piece_box).collect() };
    let (boxes_a, boxes_b) = (boxes(first), boxes(second));
    // Pieces of either loop by their boxes' left edges; `true` marks `first`.
    let mut order: Vec<(bool, usize)> = (0..boxes_a.len())
        .map(|i| (true, i))
        .chain((0..boxes_b.len()).map(|i| (false, i)))
        .collect();
    ctx.spend(order.len())?;
    let box_of = |&(side, i): &(bool, usize)| if side { boxes_a[i] } else { boxes_b[i] };
    order.compact_sort_by(|x, y| {
        box_of(x)[0]
            .total_cmp(&box_of(y)[0])
            .then(x.0.cmp(&y.0))
            .then(x.1.cmp(&y.1))
    });
    let mut active_a: Vec<usize> = Vec::new();
    let mut active_b: Vec<usize> = Vec::new();
    for entry in order {
        let (side, i) = entry;
        let own_box = box_of(&entry);
        let (others, other_boxes) = if side {
            (&mut active_b, &boxes_b)
        } else {
            (&mut active_a, &boxes_a)
        };
        ctx.spend(others.len().max(1))?;
        others.retain(|&j| other_boxes[j][2] + margin >= own_box[0]);
        for &j in others.iter() {
            if !boxes_overlap(own_box, other_boxes[j], margin) {
                continue;
            }
            if side {
                pieces_apart(ctx, &first.pieces[i], &second.pieces[j])?;
            } else {
                pieces_apart(ctx, &first.pieces[j], &second.pieces[i])?;
            }
        }
        if side {
            active_a.push(i);
        } else {
            active_b.push(i);
        }
    }
    Ok(())
}

fn pieces_apart(ctx: &mut Ctx, a: &Piece, b: &Piece) -> Result<(), Fail> {
    ctx.spend(1)?;
    let reach = a.delta() + b.delta() + ctx.margin;
    if !boxes_overlap(chord_box(a), chord_box(b), reach) {
        return Ok(());
    }
    let chords = segments_distance(a.a, a.b, b.a, b.b);
    if chords > reach || circles_apart(a, b, ctx.margin) {
        return Ok(());
    }
    // Refine the fatter enclosure while that can still separate them.
    let (fat, thin, fat_first) = if a.geom >= b.geom {
        (a, b, true)
    } else {
        (b, a, false)
    };
    if fat.geom > ctx.margin && fat.splits < MAX_SPLITS {
        return halves(ctx, fat, thin, fat_first);
    }
    // Both flat: certify contact, or shorten a curved piece while that can.
    // Every chord point of a curved piece is within half its length of a
    // computed curve point; a straight chord is its curve.
    let half = |p: &Piece| if p.curved() { 0.5 * p.length() } else { 0.0 };
    let tolerance = a.tol + b.tol;
    let floor = chords + a.err + b.err + ctx.margin;
    if floor + half(a) + half(b) < tolerance {
        return Err(Fail::Contact);
    }
    if floor >= tolerance {
        return Err(Fail::Ambiguous);
    }
    let (long, short, long_first) = if half(a) >= half(b) {
        (a, b, true)
    } else {
        (b, a, false)
    };
    if !long.curved() || long.splits >= MAX_SPLITS {
        return Err(Fail::Ambiguous);
    }
    halves(ctx, long, short, long_first)
}

/// [`pieces_apart`] for both halves of `piece_in` against `other`, keeping
/// the argument order.
fn halves(ctx: &mut Ctx, piece_in: &Piece, other: &Piece, piece_first: bool) -> Result<(), Fail> {
    let (left, right) = split(piece_in).ok_or(Fail::Ambiguous)?;
    for half in [&left, &right] {
        if piece_first {
            pieces_apart(ctx, half, other)?;
        } else {
            pieces_apart(ctx, other, half)?;
        }
    }
    Ok(())
}

/// Whether two circular arcs lie on circles certified apart or nested.
fn circles_apart(a: &Piece, b: &Piece, margin: f64) -> bool {
    let annulus = |p: &Piece| match &p.carrier {
        Carrier::Conic { conic, .. } => conic.annulus,
        Carrier::Straight | Carrier::Bezier(_) => None,
    };
    let (Some(x), Some(y)) = (annulus(a), annulus(b)) else {
        return false;
    };
    let d = distance(x.centre, y.centre);
    let slack = x.centre_error + y.centre_error + 4.0 * EPS * d + margin;
    d - slack > x.outer + y.outer || d + slack + x.outer < y.inner || d + slack + y.outer < x.inner
}

// ── Planar helpers ─────────────────────────────────────────────────────────

/// Whether two junctions are the same floats (shared, not merely close).
fn same_point(p: P2, q: P2) -> bool {
    p[0].to_bits() == q[0].to_bits() && p[1].to_bits() == q[1].to_bits()
}

fn point2(p: P2) -> Point2 {
    Point2::new(p[0], p[1])
}

fn scale(v: P2, s: f64) -> P2 {
    [v[0] * s, v[1] * s]
}

fn dot(u: P2, v: P2) -> f64 {
    u[0].mul_add(v[0], u[1] * v[1])
}

fn norm(v: P2) -> f64 {
    v[0].hypot(v[1])
}

fn norm1(v: P2) -> f64 {
    v[0].abs() + v[1].abs()
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

fn chord_box(piece_in: &Piece) -> Box2 {
    grow(grow(EMPTY_BOX, piece_in.a, 0.0), piece_in.b, 0.0)
}

fn piece_box(piece_in: &Piece) -> Box2 {
    let delta = piece_in.delta();
    grow(grow(EMPTY_BOX, piece_in.a, delta), piece_in.b, delta)
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
