//! Whether a planar face's boundary loops respect each other.
//!
//! A face's inner loops must lie inside its outer loop, and two inner loops
//! must not cross. A blend whose contact line runs past a boss's foot breaks
//! the first: the rebuilt outer loop crosses the boss's base circle, leaving
//! two open slivers, yet every count-based check (closed shell, Euler,
//! manifold edges) still passes.
//!
//! The reading is the certified nesting reader's: each loop is a chain of
//! enclosed pieces (see the [parent module](super)). A defect is reported
//! only from points whose side of another loop is certified, and only from
//! points farther from that loop than both loops' edge tolerances plus the
//! caller's tolerance, so loops that touch or run tangent within tolerance
//! are never reported. Anything the reader cannot read or decide within its
//! work limit reports nothing: this is a cheap one-sided check, not a proof
//! that a face is valid.

use std::collections::BTreeMap;

use remus_math::frame::Frame3;
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};

use super::{
    Box2, Ctx, EPS, Fail, Loop, MARGIN_ULPS, P2, Piece, Plane, box_holds, crossing_parity,
    first_vertex, midpoint, norm1, overlapping_pairs, read_loop, same_point, split,
};

/// Work one face may spend: a piece read, a probe against a piece, a
/// bisection.
const CONTAINMENT_WORK_LIMIT: usize = 1 << 21;

/// Points probed on each loop, spread over its edges.
const PROBES_PER_LOOP: usize = 64;

/// Most probes taken from one edge piece.
const MAX_PROBES_PER_PIECE: usize = 16;

/// A violation of a planar face's loop containment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopContainmentDefect {
    /// A point of the inner loop at this index of `Face::inner_wires` is
    /// certified outside the outer loop, farther from it than the
    /// tolerances: the loops cross, or the inner loop lies outside.
    InnerOutsideOuter {
        /// Index into `Face::inner_wires`.
        inner: usize,
    },
    /// Inner loop `a` has points certified both inside and outside inner
    /// loop `b`, so the two cross. Indices into `Face::inner_wires`.
    InnerLoopsCross {
        /// The loop whose points straddle `b`.
        a: usize,
        /// The loop crossed.
        b: usize,
    },
}

impl std::fmt::Display for LoopContainmentDefect {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InnerOutsideOuter { inner } => write!(
                f,
                "inner loop {inner} leaves the outer loop (the loops cross, or the inner loop \
                 lies outside)"
            ),
            Self::InnerLoopsCross { a, b } => write!(f, "inner loops {a} and {b} cross"),
        }
    }
}

/// The first certified loop-containment defect of a planar face, if any.
///
/// `tolerance` is added to the edges' own tolerances: a probe point must be
/// farther than all of them from a loop before its side of that loop counts.
/// Returns `None` for a non-planar face, a face without inner loops, an
/// unreadable boundary, or a face whose decision ran out of work.
#[must_use]
pub fn face_loop_containment_defect(
    topo: &Topology,
    face_id: FaceId,
    tolerance: f64,
) -> Option<LoopContainmentDefect> {
    let face = topo.face(face_id).ok()?;
    if face.inner_wires().is_empty() || !tolerance.is_finite() {
        return None;
    }
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        return None;
    };
    let origin = first_vertex(topo, face.outer_wire())?;
    let frame = Frame3::from_normal(origin, *normal).ok()?;
    let plane = Plane {
        origin,
        x: frame.x,
        y: frame.y,
    };
    let domains = BTreeMap::new();
    let mut loops = Vec::with_capacity(face.inner_wires().len() + 1);
    let mut pieces = 0usize;
    for &wire in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
        let l = read_loop(topo, wire, &plane, &domains)?;
        pieces += l.pieces.len();
        loops.push(l);
    }
    let reach = loops
        .iter()
        .flat_map(|l| l.bounds)
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    let margin = MARGIN_ULPS * EPS * reach;
    if !margin.is_finite() || margin <= 0.0 {
        return None;
    }
    let mut ctx = Ctx {
        margin,
        work: 0,
        limit: CONTAINMENT_WORK_LIMIT,
    };
    ctx.spend(pieces).ok()?;
    // A spent work limit (the only failure `decide` returns) reports nothing.
    decide(&mut ctx, &loops, tolerance.max(0.0)).unwrap_or_default()
}

fn decide(
    ctx: &mut Ctx,
    loops: &[Loop],
    tolerance: f64,
) -> Result<Option<LoopContainmentDefect>, Fail> {
    let probes: Vec<Vec<Probe>> = loops.iter().map(dense_probes).collect();
    let tolerances: Vec<f64> = loops.iter().map(loop_tolerance).collect();

    let outer = &loops[0];
    for (index, own) in probes.iter().enumerate().skip(1) {
        for probe in own {
            let radius = probe.err + probe.tol + tolerances[0] + tolerance;
            if side(ctx, outer, probe.point, radius)? == Some(false) {
                return Ok(Some(LoopContainmentDefect::InnerOutsideOuter {
                    inner: index - 1,
                }));
            }
        }
    }

    let inner = &loops[1..];
    for (i, j) in overlapping_pairs(ctx, inner)? {
        for (a, b) in [(i, j), (j, i)] {
            let (mut inside, mut outside) = (false, false);
            for probe in &probes[a + 1] {
                let radius = probe.err + probe.tol + tolerances[b + 1] + tolerance;
                match side(ctx, &inner[b], probe.point, radius)? {
                    Some(true) => inside = true,
                    Some(false) => outside = true,
                    None => {}
                }
                if inside && outside {
                    return Ok(Some(LoopContainmentDefect::InnerLoopsCross { a, b }));
                }
            }
        }
    }
    Ok(None)
}

/// Whether `p` is inside `l`, certified for every point within `radius` of
/// it; `None` when `p` is too close to `l` to tell.
fn side(ctx: &mut Ctx, l: &Loop, p: P2, radius: f64) -> Result<Option<bool>, Fail> {
    let point: Box2 = [p[0], p[1], p[0], p[1]];
    if !box_holds(l.bounds, point, ctx.margin + radius) {
        return Ok(Some(false));
    }
    let mut inside = false;
    for piece_in in &l.pieces {
        match crossing_parity(ctx, piece_in, p, radius) {
            Ok(crosses) => inside ^= crosses,
            Err(Fail::Ambiguous) => return Ok(None),
            Err(fail) => return Err(fail),
        }
    }
    Ok(Some(inside))
}

/// A computed point on a loop, its evaluation error and its edge's
/// tolerance.
struct Probe {
    point: P2,
    err: f64,
    tol: f64,
}

/// The largest edge tolerance of a loop.
fn loop_tolerance(l: &Loop) -> f64 {
    l.pieces.iter().fold(0.0_f64, |m, p| m.max(p.tol))
}

/// About [`PROBES_PER_LOOP`] points spread over the loop's own edges: the
/// midpoints of each curved piece's halves, quarters, … and evenly spaced
/// points along each line.
fn dense_probes(l: &Loop) -> Vec<Probe> {
    let own = &l.pieces[..l.own];
    let per_piece = PROBES_PER_LOOP
        .div_ceil(own.len().max(1))
        .clamp(1, MAX_PROBES_PER_PIECE);
    let mut probes = Vec::with_capacity(own.len() * per_piece);
    for piece_in in own {
        if piece_in.curved() {
            curved_probes(piece_in, per_piece, &mut probes);
        } else if !same_point(piece_in.a, piece_in.b) {
            let rounding = 4.0 * EPS * (norm1(piece_in.a) + norm1(piece_in.b));
            #[allow(clippy::cast_precision_loss)]
            for k in 0..per_piece {
                let s = (k as f64 + 0.5) / per_piece as f64;
                let point = [
                    (piece_in.b[0] - piece_in.a[0]).mul_add(s, piece_in.a[0]),
                    (piece_in.b[1] - piece_in.a[1]).mul_add(s, piece_in.a[1]),
                ];
                probes.push(Probe {
                    point,
                    err: piece_in.err + rounding,
                    tol: piece_in.tol,
                });
            }
        }
    }
    probes
}

/// Midpoints of `piece_in` bisected into `count` (rounded up to a power of
/// two) leaves; a piece that cannot be bisected gives its own midpoint.
fn curved_probes(piece_in: &Piece, count: usize, out: &mut Vec<Probe>) {
    if count > 1
        && let Some((left, right)) = split(piece_in)
    {
        let half = count.div_ceil(2);
        curved_probes(&left, half, out);
        curved_probes(&right, half, out);
        return;
    }
    out.push(Probe {
        point: midpoint(piece_in),
        err: piece_in.err,
        tol: piece_in.tol,
    });
}

#[cfg(test)]
mod tests;
