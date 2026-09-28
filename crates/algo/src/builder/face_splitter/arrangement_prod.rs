//! Production adapter: qualified planar faces onto the provenance-preserving
//! UV arrangement (O2.3b/c).
//!
//! The isolated core in [`super::arrangement`] subdivides certified
//! line/circle uses in one chart. This module builds those uses from
//! production topology — face loops/coedges, section edges, native trims —
//! and runs the core. It owns no geometry invention:
//!
//! - Boundary uses come from the face's authoritative loops (outer then
//!   inner), cross-checked edge-for-edge against the wire the established
//!   splitter consumes. Any loop/wire mismatch defers to the established
//!   path.
//! - Circle pcurves are rebuilt analytically from the 3D carrier projected
//!   into the split frame. The NURBS-fit pcurves production carries for
//!   plane curves are never fed to the core.
//! - The pcurve-angle to 3D-angle correspondence is derived per use from
//!   the projected carrier axes (orientation sign) anchored at the
//!   traversal start. A carrier parameter is never assumed to be an angle,
//!   and a rational (NURBS) parameter never reaches the core: only
//!   [`EdgeCurve::Circle`] (native angle parameters) is accepted.
//! - Uses are passed in traversal order (possibly descending); the core
//!   normalizes direction itself while preserving the end pairing, so no
//!   interval is ever silently rebased.
//! - Arcs crossing the angle branch cut are split there into exact
//!   sub-uses, since the core only represents canonical `[0, 2π]`
//!   branches. Full circles are split the same way unless they already
//!   start at angle zero.
//! - Section endpoints landing on another use's interior split that use at
//!   the exact projected parameter with a shared certificate — the same
//!   join the established splitter performs with weld bands, but with exact
//!   subspans and core residual verification instead of snapping.
//! - Endpoint identity comes from authoritative topology vertices for
//!   boundary uses, and from tolerance-scoped correspondence derivation
//!   (residual-verified by the core) for computed section endpoints. No
//!   proximity merge is ever manufactured: unresolvable contacts refuse.
//! - Every section endpoint must anchor to the face boundary through the
//!   shared-certificate graph (or close on itself as an interior ring).
//!   Dangling spurs are out-of-domain: the established path owns them.
//!
//! Contract: `Ok(None)` means out-of-domain — the caller must run the
//! established path. `Err` means the input passed qualification but the
//! core refused it; that is an internal error on claimed input and must
//! propagate, never degrade into an unsplit face or a fallback.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use remus_math::context::OperationContext;
use remus_math::curves2d::{Circle2D, Curve2D, Line2D};
use remus_math::predicates::orient2d;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::face_loop::LoopId;

use super::super::plane_frame::PlaneFrame;
use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::arrangement::{
    Arrangement, ArrangementError, ArrangementInput, BoundarySource, CurveSource, CurveUse,
    ParamDomain, build_arrangement,
};
use crate::ds::Rank;
use crate::error::AlgoError;

/// Angular band deciding full-circle versus partial-arc trims (radians).
const FULL_CIRCLE_BAND: f64 = 1e-9;
/// In-plane gate band scale (multiples of the linear tolerance).
const PLANE_GATE_SCALE: f64 = 100.0;
/// End-proximity margin for interior-cut parameters (absolute, in the
/// use's native pcurve parameter units).
const END_MARGIN: f64 = 1e-9;
/// Maximum input uses for one exact arrangement. Pair scans scale
/// quadratically; larger faces stay on the established chord path.
const MAX_ARRANGEMENT_USES: usize = 256;

/// One coedge traversal in a loop, resolved to vertex identities.
struct LoopUse {
    coedge: remus_topology::coedge::CoedgeId,
    edge_id: remus_topology::edge::EdgeId,
    start_vid: remus_topology::vertex::VertexId,
    end_vid: remus_topology::vertex::VertexId,
    start_pt: Point3,
    end_pt: Point3,
}

/// One canonical circle piece's range/source pair, in traversal order.
struct RawPiece {
    range: [f64; 2],
    source: [f64; 2],
}

/// One exact transverse crossing between two line uses.
struct Crossing {
    first: usize,
    second: usize,
    first_param: f64,
    second_param: f64,
    point: Point2,
}

/// Production inputs collected for one qualified planar face, ready for the
/// isolated core. `parents` is indexed by the input position stored in each
/// use's [`CurveSource::source_edge_idx`]; the M3 emitter consumes it to
/// rebuild exact wire edges with lineage.
#[derive(Debug, Clone)]
pub(super) struct PlanarInputs {
    /// Certified uses in deterministic (boundary-then-section) order.
    pub uses: Vec<CurveUse>,
    /// Per-input-position parent data for emission.
    pub parents: Vec<ParentInfo>,
    /// Certificates minted for canonical branch cuts (angle zero).
    /// These subdivide uses for the core's chart but mark no geometric
    /// event; emission merges across them so neighboring faces never
    /// see a T-junction there.
    pub branch_certs: BTreeSet<u64>,
}

/// Parent data for one adapter input use.
#[derive(Debug, Clone)]
pub(super) struct ParentInfo {
    /// Authoritative parent pcurve in the split frame.
    pub pcurve: Curve2D,
    /// Traversal start/end in 3D.
    pub endpoints_3d: [Point3; 2],
    /// Store-space topology edge index for boundary uses.
    pub source_topo_edge: Option<usize>,
    /// Pave block for section uses (cross-face sharing).
    pub pave_block_id: Option<usize>,
    /// Whether this input is a section (false = face boundary).
    pub is_section: bool,
}

/// Collect certified arrangement uses for a planar face.
///
/// Returns `Ok(None)` when the face is outside the qualified domain; see
/// the module contract. The returned uses are fully gated: pcurve/3D
/// consistency, closed-loop connectivity, section anchoring, and exact
/// subspans are verified here so a core refusal later is an internal
/// error, not a gate miss.
pub(super) fn collect_planar_uses(
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    frame: &PlaneFrame,
    tol: &Tolerance,
    context: &OperationContext,
) -> Result<Option<PlanarInputs>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
    context.check_cancelled().map_err(cancelled)?;
    let face = match topo.face(face_id) {
        Ok(face) => face,
        Err(_) => return Ok(None),
    };
    let FaceSurface::Plane { .. } = face.surface() else {
        return Ok(None);
    };
    if !tol.linear.is_finite() || tol.linear <= 0.0 {
        return Ok(None);
    }
    let Some(loop_ids) = topo.loops_of_face(face_id) else {
        return Ok(None);
    };
    if loop_ids.is_empty() {
        return Ok(None);
    }
    let _ = rank;

    let mut collector = Collector::new(frame, tol.linear);
    // Outer loop first, then holes: `loops_of_face` guarantees that order.
    for (loop_position, &loop_id) in loop_ids.iter().enumerate() {
        context.check_cancelled().map_err(cancelled)?;
        if !collector.push_loop(topo, face_id, loop_id, loop_position as u64) {
            return Ok(None);
        }
    }
    if !Collector::loops_match_wires(topo, face_id) {
        return Ok(None);
    }
    for section in sections {
        context.check_cancelled().map_err(cancelled)?;
        if !collector.push_section(section) {
            return Ok(None);
        }
    }
    // Interior concurrency first: shared-cert splits become range
    // ends that the endpoint prejoin below then treats as joined.
    if !collector.resolve_line_concurrency(context)? {
        return Ok(None);
    }
    if !collector.prejoin_endpoints(context)? {
        return Ok(None);
    }
    if !collector.section_ends_resolved() {
        return Ok(None);
    }
    Ok(collector.finish())
}

/// Run the isolated core on collected uses.
///
/// Returns the typed core error unchanged: the caller decides whether a
/// refusal is out-of-domain (recorded-unsupported contact classes) or an
/// internal error on claimed input.
pub(super) fn run_planar_arrangement(
    inputs: &PlanarInputs,
    context: &OperationContext,
) -> Result<Arrangement, ArrangementError> {
    build_arrangement(&ArrangementInput {
        uses: &inputs.uses,
        domain: ParamDomain::Plane,
        context,
    })
}

fn collect_failed(error: ArrangementError) -> AlgoError {
    AlgoError::FaceSplitFailed(format!("provenance arrangement refused: {error:?}"))
}

/// Whether a native line parameter lies strictly inside its use interval.
///
/// Contacts within `margin` of an endpoint are certified wire joints (G1
/// chains, section anchors, T-junction touches), never degeneracy.
fn strictly_inside_line(t: f64, range: [f64; 2], margin: f64) -> bool {
    let lo = range[0].min(range[1]);
    let hi = range[0].max(range[1]);
    t > lo + margin && t < hi - margin
}

/// Whether a circle angle lies strictly inside its use interval.
///
/// Canonical pieces stay within one turn; a full-turn use has no endpoints,
/// so every contact on it is interior.
fn strictly_inside_angle(t: f64, range: [f64; 2], margin: f64) -> bool {
    let lo = range[0].min(range[1]);
    let hi = range[0].max(range[1]);
    if hi - lo >= TAU - 1e-9 {
        return true;
    }
    let shifted = lo + (t - lo).rem_euclid(TAU);
    shifted > lo + margin && shifted < hi - margin
}

/// Whether two line uses overlap coincidentally over more than a point.
///
/// Transverse crossings and parallel-distinct lines are the core's domain;
/// only a coincident run creates the unrefinable overlap the core cannot
/// emit. A point touch is a certified joint, not an overlap.
fn line_pair_is_degenerate(a: &Line2D, ra: [f64; 2], b: &Line2D, rb: [f64; 2], tol: f64) -> bool {
    let da = a.tangent(0.0);
    let db = b.tangent(0.0);
    let la = da.length();
    let lb = db.length();
    if la < 1e-15 || lb < 1e-15 {
        return false;
    }
    let cross = (da.x() * db.y() - da.y() * db.x()).abs() / (la * lb);
    if cross > 1e-9 {
        return false;
    }
    if b.distance_to_point(a.evaluate(0.0)) > tol {
        return false;
    }
    let pa0 = b.project(a.evaluate(ra[0]));
    let pa1 = b.project(a.evaluate(ra[1]));
    let overlap = pa0.max(pa1).min(rb[0].max(rb[1])) - pa0.min(pa1).max(rb[0].min(rb[1]));
    overlap * lb > tol
}

/// Whether a line use touches a circle use tangentially at an interior
/// point of both. Secant crossings and misses are the core's domain.
fn line_circle_is_degenerate(
    line: &Line2D,
    line_range: [f64; 2],
    circle: &Circle2D,
    circle_range: [f64; 2],
    tol: f64,
) -> bool {
    let radius = circle.radius();
    if radius < 1e-15 {
        return false;
    }
    if (line.distance_to_point(circle.center()) - radius).abs() > tol {
        return false;
    }
    let scale = line.tangent(0.0).length().max(1e-15);
    let foot = line.project(circle.center());
    if !strictly_inside_line(foot, line_range, tol / scale) {
        return false;
    }
    strictly_inside_angle(
        circle.project(line.evaluate(foot)),
        circle_range,
        tol / radius,
    )
}

/// Whether two circle uses touch tangentially or coincide at an interior
/// point of both. Secant crossings, misses, and disjoint containments are
/// the core's domain.
fn circle_pair_is_degenerate(
    a: &Circle2D,
    ra: [f64; 2],
    b: &Circle2D,
    rb: [f64; 2],
    tol: f64,
) -> bool {
    let (r1, r2) = (a.radius(), b.radius());
    if r1 < 1e-15 || r2 < 1e-15 {
        return false;
    }
    let delta = b.center() - a.center();
    let dist = delta.length();
    if dist <= tol {
        if (r1 - r2).abs() > tol {
            return false;
        }
        let (lo1, hi1) = (ra[0].min(ra[1]), ra[0].max(ra[1]));
        let (lo2, hi2) = (rb[0].min(rb[1]), rb[0].max(rb[1]));
        if hi1 - lo1 >= TAU - 1e-9 || hi2 - lo2 >= TAU - 1e-9 {
            return true;
        }
        return hi1.min(hi2) - lo1.max(lo2) > tol / r1.max(r2);
    }
    if dist < 1e-15 {
        return false;
    }
    let unit = delta * (1.0 / dist);
    // External tangency (contact toward the other center), then internal
    // tangency (contact on the major side: toward when `a` is major, away
    // when `b` is major).
    let cases = [
        (1.0, r1 + r2),
        (if r1 >= r2 { 1.0 } else { -1.0 }, (r1 - r2).abs()),
    ];
    for (sign, expected) in cases {
        if (dist - expected).abs() > tol {
            continue;
        }
        let contact = a.center() + unit * (sign * r1);
        if strictly_inside_angle(a.project(contact), ra, tol / r1)
            && strictly_inside_angle(b.project(contact), rb, tol / r2)
        {
            return true;
        }
    }
    false
}

/// Emission refusal for a wire run spanning input uses with no kept vertex
/// between them: the event topology cannot be emitted with exact provenance.
/// Geometric (a grazing contact merges the joint vertex away), so the caller
/// declines to the established path. Kept in a constant because the `try_`
/// mapping matches on it — rewording here requires updating the mapping,
/// and the tangent-boss batch contract pins the pairing end to end.
const MIXED_USE_RUN: &str = "mixed-use run";

/// Whether an emission failure is the geometric mixed-use refusal (decline)
/// rather than an internal error (propagate) or cancellation (propagate).
fn is_mixed_use_run(error: &AlgoError) -> bool {
    matches!(error, AlgoError::FaceSplitFailed(detail) if detail.contains(MIXED_USE_RUN))
}

/// Whether two certified uses meet in an interior tangent contact or a
/// coincident overlap.
fn uses_pair_is_degenerate(a: &CurveUse, b: &CurveUse, tol: f64) -> bool {
    match (&a.pcurve, &b.pcurve) {
        (Curve2D::Line(la), Curve2D::Line(lb)) => {
            line_pair_is_degenerate(la, a.range, lb, b.range, tol)
        }
        (Curve2D::Line(line), Curve2D::Circle(circle)) => {
            line_circle_is_degenerate(line, a.range, circle, b.range, tol)
        }
        (Curve2D::Circle(circle), Curve2D::Line(line)) => {
            line_circle_is_degenerate(line, b.range, circle, a.range, tol)
        }
        (Curve2D::Circle(ca), Curve2D::Circle(cb)) => {
            circle_pair_is_degenerate(ca, a.range, cb, b.range, tol)
        }
        _ => false,
    }
}

/// Whether any two certified uses meet in an interior tangent contact or a
/// coincident overlap.
///
/// Tangent contacts and coincident overlaps are out-of-domain for the
/// exact-event core (recorded-unsupported classes): a tangent crossing has
/// no transverse refinement, so the core refuses with
/// `IntersectionRefinementFailed`. That refusal is geometric, not an
/// adapter bug — declining here routes the face to the established path
/// exactly like the syntactic out-of-domain gates. Only interior contacts
/// decline: endpoint touches are certified wire joints (G1 chains,
/// T-junction anchors), which the core owns.
fn uses_have_degenerate_contact(uses: &[CurveUse], tol: f64) -> bool {
    for i in 0..uses.len() {
        for j in (i + 1)..uses.len() {
            if uses_pair_is_degenerate(&uses[i], &uses[j], tol) {
                return true;
            }
        }
    }
    false
}

/// Input-proportional budget policy for one arrangement run. The core
/// charges every pair scan, refinement, seed search, graph walk, and
/// quotient step against `march_steps`, caps inputs/events at
/// `queue_size`, and caps emitted edges at `segments`.
fn arrangement_context(parent: &OperationContext, uses: usize) -> OperationContext {
    let n = uses.saturating_add(8);
    let floor = remus_math::context::WorkBudgets::new()
        .with_march_steps(64_usize.saturating_mul(n).saturating_mul(n))
        .with_queue_size(8_usize.saturating_mul(n).saturating_mul(n))
        .with_segments(4_usize.saturating_mul(n).saturating_mul(n));
    let current = parent.budgets;
    parent.clone().with_budgets(
        remus_math::context::WorkBudgets::new()
            .with_march_steps(current.march_steps.max(floor.march_steps))
            .with_queue_size(current.queue_size.max(floor.queue_size))
            .with_segments(current.segments.max(floor.segments))
            .with_branches_per_direction(current.branches_per_direction)
            .with_newton_iterations(current.newton_iterations)
            .with_subdivision_depth(current.subdivision_depth),
    )
}

/// Split a qualified planar face through the provenance-preserving
/// arrangement.
///
/// Returns `Ok(None)` when the face is out-of-domain (the caller runs the
/// established path). Geometric refusals — tangent/overlap contacts and
/// unrefinable grazing crossings — likewise decline; any other core
/// refusal on qualified input is an internal error and propagates. Construction is atomic: collection, arrangement,
/// and emission are pure and allocate no topology, so any failure leaves
/// the caller's topology untouched.
#[allow(clippy::too_many_arguments)]
pub(super) fn try_split_plane_face_by_provenance_arrangement(
    topo: &Topology,
    face_id: FaceId,
    sections: &[SectionEdge],
    rank: Rank,
    frame: &PlaneFrame,
    tol: &Tolerance,
    context: &OperationContext,
    split_registry: Option<&mut std::collections::HashMap<usize, Vec<Point3>>>,
) -> Result<Option<Vec<SplitSubFace>>, AlgoError> {
    let face = match topo.face(face_id) {
        Ok(face) => face,
        Err(_) => return Ok(None),
    };
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    let Some(inputs) = collect_planar_uses(topo, face_id, sections, rank, frame, tol, context)?
    else {
        return Ok(None);
    };

    if inputs.uses.len() > MAX_ARRANGEMENT_USES {
        return Ok(None);
    }

    if uses_have_degenerate_contact(&inputs.uses, tol.linear) {
        log::debug!(
            "provenance arrangement declined face={face_id:?}: interior tangent/overlap contact"
        );
        return Ok(None);
    }
    // The pipeline's budgets are SSI-calibrated (a handful of steps);
    // the arrangement is a different cost model with quadratic pair
    // scans. The adapter translates with an input-proportional policy
    // floor, keeping the caller's cancellation token and every larger
    // allowance: budgets stay finite and exhaustion stays typed, while
    // default contexts can actually run the core. Measured: 10 uses
    // consume ~700 march steps; the floor carries 8x headroom.
    let child = arrangement_context(context, inputs.uses.len());
    match run_planar_arrangement(&inputs, &child) {
        Err(
            ArrangementError::AmbiguousContact
            | ArrangementError::AmbiguousOverlap
            | ArrangementError::IntersectionRefinementFailed,
        ) => {
            // Geometric refusals (M1 contract): tangent contacts, coincident
            // overlaps, and unrefinable crossings are out-of-domain, so the
            // caller runs the established path. A tangent or grazing
            // crossing has no transverse refinement — the core's twin
            // events land within roundoff with no endpoint certificate to
            // adopt — and that outcome is geometric, not an adapter bug:
            // real models carry dust-scale grazing caps (a boss arc poking
            // 0.001 past a wall with crossings 0.28 apart). Every other
            // refusal is an internal error on claimed input and propagates.
            // Declining never produces wrong geometry (the established path
            // is the shipped baseline); a correspondence regression would
            // show up as lost engagement in the differential suite, which
            // pins improvement on transverse input.
            log::debug!(
                "provenance arrangement declined face={face_id:?} err=geometric uses={}",
                inputs.uses.len()
            );
            Ok(None)
        }
        Err(error) => {
            log::debug!(
                "provenance arrangement refused face={face_id:?} err={error:?} uses={}",
                inputs.uses.len()
            );
            Err(AlgoError::FaceSplitFailed(format!(
                "provenance arrangement refused: {error:?}"
            )))
        }
        Ok(arrangement) => {
            record_section_breaks(&inputs, &arrangement, frame, split_registry);
            match emit_planar_subfaces(
                &inputs,
                &arrangement,
                surface,
                reversed,
                face_id,
                rank,
                frame,
                context,
            ) {
                Err(error) if is_mixed_use_run(&error) => {
                    log::debug!(
                        "provenance arrangement declined face={face_id:?}: {MIXED_USE_RUN}"
                    );
                    Ok(None)
                }
                result => result.map(Some),
            }
        }
    }
}

/// Record section interior break points for curved neighbors.
///
/// Mirrors the established plane-arrangement contract: interior vertices
/// where distinct uses meet (crossings, joints) are recorded per pave
/// block as exact frame-evaluated points, so curved faces sharing the
/// section curve pre-split at identical points instead of meeting the
/// plane side in T-junctions. Canonical subdivisions (cardinal and
/// branch cuts: single-use vertices with no event) are deliberately NOT
/// recorded — neighbors share no event there, and recording them would
/// hand curved faces splits the plane side itself merges away. Use ends
/// are excluded (neighbors already share those events).
fn record_section_breaks(
    inputs: &PlanarInputs,
    arrangement: &Arrangement,
    frame: &PlaneFrame,
    split_registry: Option<&mut std::collections::HashMap<usize, Vec<Point3>>>,
) {
    let Some(registry) = split_registry else {
        return;
    };
    let by_id: BTreeMap<u64, &CurveUse> = arrangement
        .sources
        .iter()
        .map(|u| (u.source.use_id, u))
        .collect();
    // Distinct input uses meeting at each vertex, with half indices
    // for certificate resolution.
    let mut vertex_uses: BTreeMap<usize, BTreeSet<u64>> = BTreeMap::new();
    let mut vertex_halves: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (hi, half) in arrangement.half_edges.iter().enumerate() {
        vertex_uses
            .entry(half.from)
            .or_default()
            .insert(half.source.use_id);
        vertex_uses
            .entry(half.to)
            .or_default()
            .insert(half.source.use_id);
        vertex_halves.entry(half.from).or_default().push(hi);
        vertex_halves.entry(half.to).or_default().push(hi);
    }
    // Whether every incidence at a vertex carries one branch-cut
    // certificate (a canonical subdivision with no event).
    let is_branch_cut = |vertex: usize| -> bool {
        let Some(halves) = vertex_halves.get(&vertex) else {
            return false;
        };
        let mut cert = None;
        for &hi in halves {
            let half = &arrangement.half_edges[hi];
            let Some(use_data) = by_id.get(&half.source.use_id) else {
                return false;
            };
            let param = if half.from == vertex {
                half.range[0]
            } else {
                half.range[1]
            };
            let mut found = None;
            for side in 0..2 {
                if param.total_cmp(&use_data.range[side]).is_eq() {
                    found = Some(use_data.endpoints[side]);
                    break;
                }
            }
            let Some(cert_here) = found else {
                return false;
            };
            if cert.is_none() {
                cert = Some(cert_here);
            }
            if cert != Some(cert_here) {
                return false;
            }
        }
        cert.is_some_and(|c| inputs.branch_certs.contains(&c))
    };
    for half in arrangement.half_edges.iter().step_by(2) {
        let Some(position) = half.source.source_edge_idx else {
            continue;
        };
        let (Some(parent), Some(source)) =
            (inputs.parents.get(position), by_id.get(&half.source.use_id))
        else {
            continue;
        };
        if !parent.is_section {
            continue;
        }
        let Some(pave_block) = half.source.pave_block_id.or(parent.pave_block_id) else {
            continue;
        };
        // Interior vertices where distinct uses meet: ends are shared
        // events already, and single-use vertices are canonical
        // subdivisions with no event for neighbors to share.
        for (vertex, param) in [(half.from, half.range[0]), (half.to, half.range[1])] {
            if param.total_cmp(&source.range[0]).is_eq()
                || param.total_cmp(&source.range[1]).is_eq()
            {
                continue;
            }
            let Some(users) = vertex_uses.get(&vertex) else {
                continue;
            };
            if users.len() < 2 || is_branch_cut(vertex) {
                continue;
            }
            let uv = arrangement.vertices[vertex].uv;
            let point = frame.evaluate(uv.x(), uv.y());
            let entry = registry.entry(pave_block).or_default();
            if !entry.contains(&point) {
                entry.push(point);
            }
        }
    }
}

/// Convert material regions into production subfaces.
///
/// Every disconnected material region is preserved with its holes; each
/// region carries its certified interior seed as the classification
/// point, so downstream classification consumes arrangement-certified
/// interiors instead of re-searching the surface. Holes attach as inner
/// wires in traced order. New edges retain source provenance
/// (`source_edge_idx`, full-span `pave_block_id`, boundary
/// `source_topo_edge`) and strict native parameter domains; subspans of
/// a split section drop the pave block exactly like the established
/// splitter's split pieces, leaving cross-face sharing to the
/// position-based duplicate-edge merge.
#[allow(clippy::too_many_arguments)]
pub(super) fn emit_planar_subfaces(
    inputs: &PlanarInputs,
    arrangement: &Arrangement,
    surface: FaceSurface,
    reversed: bool,
    parent: FaceId,
    rank: Rank,
    frame: &PlaneFrame,
    context: &OperationContext,
) -> Result<Vec<SplitSubFace>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
    let carriers: BTreeMap<u64, &CurveUse> = arrangement
        .sources
        .iter()
        .map(|use_data| (use_data.source.use_id, use_data))
        .collect();
    let mut subfaces = Vec::new();
    for region in &arrangement.regions {
        context.check_cancelled().map_err(cancelled)?;
        if !region.material {
            continue;
        }
        let outer = emit_cycle_wire(
            inputs,
            arrangement,
            &carriers,
            &arrangement.cycles[region.outer].edges,
            context,
        )?;
        if outer.is_empty() {
            return Err(collect_failed(ArrangementError::OpenRegion));
        }
        let mut inner_wires = Vec::new();
        for hole in &region.holes {
            context.check_cancelled().map_err(cancelled)?;
            let wire = emit_cycle_wire(
                inputs,
                arrangement,
                &carriers,
                &arrangement.cycles[*hole].edges,
                context,
            )?;
            if wire.is_empty() {
                return Err(collect_failed(ArrangementError::OpenRegion));
            }
            inner_wires.push(wire);
        }
        subfaces.push(SplitSubFace {
            surface: surface.clone(),
            outer_wire: outer,
            inner_wires,
            reversed,
            parent,
            rank,
            precomputed_interior: Some(frame.evaluate(region.interior.x(), region.interior.y())),
        });
    }
    Ok(subfaces)
}

/// Build one wire from a traced half-edge cycle, preserving order.
///
/// Consecutive halves of one use joined at a pass-through vertex merge
/// into a single wire edge: the vertex carries no branch (exactly four
/// incident halves of one use: the two pieces and their twins), so it is
/// an artifact of cardinal subdivision, not a shared event. Merging
/// restores established-like segmentation — split at crossings and
/// endpoints only — so neighboring faces tessellate against identical
/// boundary vertices instead of T-junctions. Vertices shared across
/// uses (crossings, endpoints, section feet) never merge.
fn emit_cycle_wire(
    inputs: &PlanarInputs,
    arrangement: &Arrangement,
    carriers: &BTreeMap<u64, &CurveUse>,
    cycle: &[usize],
    context: &OperationContext,
) -> Result<Vec<OrientedPCurveEdge>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
    // Per-vertex incidence across the whole arrangement.
    let mut incidence: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (hi, half) in arrangement.half_edges.iter().enumerate() {
        incidence.entry(half.from).or_default().push(hi);
        incidence.entry(half.to).or_default().push(hi);
    }
    // Certificate carried by one half-edge end at a vertex, if that end
    // coincides with an input-use endpoint.
    let end_cert = |half_idx: usize, at_from: bool| -> Option<u64> {
        let half = &arrangement.half_edges[half_idx];
        let use_data = carriers.get(&half.source.use_id)?;
        let param = if at_from {
            half.range[0]
        } else {
            half.range[1]
        };
        for side in 0..2 {
            if param.total_cmp(&use_data.range[side]).is_eq() {
                return Some(use_data.endpoints[side]);
            }
        }
        None
    };
    let mergeable = |vertex: usize| -> bool {
        let Some(halves) = incidence.get(&vertex) else {
            return false;
        };
        // Branch-cut force merge below handles non-quad arity.
        if halves.len() == 4 {
            let use_id = arrangement.half_edges[halves[0]].source.use_id;
            if halves
                .iter()
                .all(|h| arrangement.half_edges[*h].source.use_id == use_id)
            {
                let mut undirected: Vec<usize> = halves.iter().map(|h| h / 2).collect();
                undirected.sort_unstable();
                undirected.dedup();
                if undirected.len() == 2 {
                    return true;
                }
            }
        }
        // Canonical branch cuts mark no geometric event: merge across
        // them whenever every incidence at the vertex carries that one
        // branch certificate. A real event coinciding with the cut
        // contributes foreign certificates and keeps the vertex.
        let mut certs: Vec<u64> = Vec::new();
        for &half_idx in halves {
            let half = &arrangement.half_edges[half_idx];
            let at_from = half.from == vertex;
            let Some(cert) = end_cert(half_idx, at_from) else {
                return false;
            };
            certs.push(cert);
        }
        certs.windows(2).all(|w| w[0] == w[1])
            && certs
                .first()
                .is_some_and(|c| inputs.branch_certs.contains(c))
    };
    let mut wire = Vec::with_capacity(cycle.len());
    // Kept vertices break runs; mergeable pass-throughs do not. A use
    // change always lands on a kept vertex (mixed incidence), so every
    // run is single-use. Cyclic runs wrap past the cycle end.
    let mut kept: Vec<bool> = Vec::with_capacity(cycle.len());
    for &half_idx in cycle {
        context.check_cancelled().map_err(cancelled)?;
        kept.push(!mergeable(arrangement.half_edges[half_idx].from));
    }
    let Some(first) = kept.iter().position(|k| *k) else {
        // Whole cycle merges: one edge (single use by the merge rule).
        if !cycle.is_empty() {
            wire.push(emit_wire_run(
                inputs,
                arrangement,
                carriers,
                cycle,
                context,
            )?);
        }
        return Ok(wire);
    };
    // Kept boundaries in cyclic order starting at `first`, plus a
    // sentinel one full turn on; each consecutive pair bounds a run.
    let mut boundaries = vec![first];
    for step in 1..cycle.len() {
        let pos = (first + step) % cycle.len();
        if kept[pos] {
            boundaries.push(first + step);
        }
    }
    boundaries.push(first + cycle.len());
    for pair in boundaries.windows(2) {
        context.check_cancelled().map_err(cancelled)?;
        let run: Vec<usize> = (pair[0]..pair[1]).map(|k| cycle[k % cycle.len()]).collect();
        if !run.is_empty() {
            wire.push(emit_wire_run(inputs, arrangement, carriers, &run, context)?);
        }
    }
    Ok(wire)
}

/// Build one production wire edge from a run of same-use halves.
///
/// Geometry comes from the run's exact endpoints (3D) and chart vertices
/// (UV); the carrier and lineage come from the originating input use.
/// A single-half run is the common case; longer runs merge cardinal
/// pass-throughs back into one exact subspan. Circle edges carry their
/// exact native subspan with the traversal-oriented forward flag; lines
/// carry no trim, matching production convention.
#[allow(clippy::too_many_lines)]
fn emit_wire_run(
    inputs: &PlanarInputs,
    arrangement: &Arrangement,
    carriers: &BTreeMap<u64, &CurveUse>,
    run: &[usize],
    context: &OperationContext,
) -> Result<OrientedPCurveEdge, AlgoError> {
    let internal = |detail: &str| {
        AlgoError::FaceSplitFailed(format!("provenance arrangement emission failed: {detail}"))
    };
    let tolerance = context.tolerance.linear;
    let first = &arrangement.half_edges[*run.first().ok_or_else(|| internal("empty run"))?];
    let last = &arrangement.half_edges[*run.last().ok_or_else(|| internal("empty run"))?];
    if first.source.use_id != last.source.use_id {
        return Err(internal(MIXED_USE_RUN));
    }
    let position = first
        .source
        .source_edge_idx
        .ok_or_else(|| internal("missing source index"))?;
    let parent = inputs
        .parents
        .get(position)
        .ok_or_else(|| internal("stale source index"))?;
    let carrier = carriers
        .get(&first.source.use_id)
        .ok_or_else(|| internal("stale use id"))?;
    let start_uv = arrangement.vertices[first.from].uv;
    let end_uv = arrangement.vertices[last.to].uv;
    // Combined native subspan across the run, in traversal order. Runs
    // are single-use by construction; every member is checked.
    let mut source_span = first.source_range;
    for &half_idx in &run[1..] {
        let half = &arrangement.half_edges[half_idx];
        if half.source.use_id != first.source.use_id {
            return Err(internal(MIXED_USE_RUN));
        }
        source_span[1] = half.source_range[1];
    }
    let (pcurve, trim, forward) = match &carrier.curve_3d {
        EdgeCurve::Line => {
            let direction = end_uv - start_uv;
            if direction.length() <= f64::EPSILON {
                return Err(internal("degenerate line sub-edge"));
            }
            let Ok(line) = Line2D::new(start_uv, direction) else {
                return Err(internal("degenerate line sub-edge"));
            };
            (Curve2D::Line(line), None, true)
        }
        EdgeCurve::Circle(_) => {
            let Curve2D::Circle(_) = parent.pcurve else {
                return Err(internal("circle sub-edge without circle pcurve"));
            };
            if !source_span[0].is_finite() || !source_span[1].is_finite() {
                return Err(internal("non-finite circle subspan"));
            }
            (
                parent.pcurve.clone(),
                Some((source_span[0], source_span[1])),
                source_span[1] >= source_span[0],
            )
        }
        _ => return Err(internal("non-line/circle carrier")),
    };
    // Pave-block sharing only for full-span sections: subspans drop it
    // exactly like the established splitter's split pieces, so vertex
    // resolution never snaps a piece to its parent's unsplit endpoints.
    // Untouched uses pass through bitwise-identical, so exact comparison
    // is the correct untouched detector (not a tolerance comparison).
    // A fully re-merged run recovers its parent span (either traversal
    // orientation) and keeps the block.
    let run_range = [first.range[0], last.range[1]];
    let reversed = [run_range[1], run_range[0]];
    let run_source = source_span;
    let reversed_source = [source_span[1], source_span[0]];
    let full_span = (same_interval(carrier.range, run_range)
        && same_interval(carrier.source_range, run_source))
        || (same_interval(carrier.range, reversed)
            && same_interval(carrier.source_range, reversed_source));
    let pave_block_id = if full_span {
        first.source.pave_block_id
    } else {
        None
    };
    // The carried trim must reproduce the wire endpoints through the
    // carrier. This duplicates the downstream preflight at emission
    // time so a pairing bug fails here with provenance attached,
    // never as a silent weld or a far-away topology error.
    if let (EdgeCurve::Circle(circle), Some((t0, t1))) = (&carrier.curve_3d, trim) {
        let start_ok = (circle.evaluate(t0) - first.endpoints_3d[0]).length() <= tolerance
            && (circle.evaluate(t1) - last.endpoints_3d[1]).length() <= tolerance;
        if !start_ok {
            return Err(internal("circle subspan endpoints mismatch carrier"));
        }
    }
    Ok(OrientedPCurveEdge {
        curve_3d: carrier.curve_3d.clone(),
        trim,
        pcurve,
        start_uv,
        end_uv,
        start_3d: first.endpoints_3d[0],
        end_3d: last.endpoints_3d[1],
        forward,
        source_edge_idx: first.source.source_edge_idx,
        pave_block_id,
        source_topo_edge: parent.source_topo_edge,
    })
}

/// One circle piece in traversal order: analytic pcurve, canonical range,
/// paired native source interval, and both ends' certificates.
struct CirclePiece {
    pcurve: Circle2D,
    range: [f64; 2],
    source_range: [f64; 2],
    endpoints_3d: [Point3; 2],
    endpoints: [u64; 2],
}

struct Collector<'a> {
    frame: &'a PlaneFrame,
    tol: f64,
    uses: Vec<CurveUse>,
    parents: Vec<ParentInfo>,
    /// Boundary vertex positions in assignment order, for section-endpoint
    /// correspondence derivation.
    cert_positions: Vec<(Point3, u64)>,
    next_fresh: u64,
    /// Certificates minted for canonical branch cuts.
    branch_certs: BTreeSet<u64>,
}

impl<'a> Collector<'a> {
    fn new(frame: &'a PlaneFrame, tol: f64) -> Self {
        Self {
            frame,
            tol,
            uses: Vec::new(),
            parents: Vec::new(),
            cert_positions: Vec::new(),
            next_fresh: u64::MAX,
            branch_certs: BTreeSet::new(),
        }
    }

    fn fresh_cert(&mut self) -> u64 {
        let id = self.next_fresh;
        self.next_fresh = self.next_fresh.saturating_sub(1);
        id
    }

    /// Push one loop. Returns `false` when the loop is outside the
    /// qualified domain.
    fn push_loop(
        &mut self,
        topo: &Topology,
        face_id: FaceId,
        loop_id: LoopId,
        loop_key: u64,
    ) -> bool {
        let face_loop = match topo.face_loop(loop_id) {
            Ok(loop_data) => loop_data,
            Err(_) => return false,
        };
        if face_loop.coedges().is_empty() {
            return false;
        }
        // First pass: resolve every coedge and verify the loop closes
        // head-to-tail by vertex identity. An open chain defers: the core
        // would refuse it as an invalid boundary, but an unclosable wire
        // is established-path territory, not an internal error.
        let mut chain: Vec<LoopUse> = Vec::new();
        for &coedge_id in face_loop.coedges() {
            let coedge = match topo.coedge(coedge_id) {
                Ok(coedge) => coedge,
                Err(_) => return false,
            };
            if coedge.parent_loop() != loop_id {
                return false;
            }
            let edge_id = coedge.edge();
            let forward = coedge.is_forward();
            let edge = match topo.edge(edge_id) {
                Ok(edge) => edge,
                Err(_) => return false,
            };
            let (start_vid, end_vid) = if forward {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            let start_pt = match topo.vertex(start_vid) {
                Ok(vertex) => vertex.point(),
                Err(_) => return false,
            };
            let end_pt = match topo.vertex(end_vid) {
                Ok(vertex) => vertex.point(),
                Err(_) => return false,
            };
            chain.push(LoopUse {
                coedge: coedge_id,
                edge_id,
                start_vid,
                end_vid,
                start_pt,
                end_pt,
            });
        }
        for (i, link) in chain.iter().enumerate() {
            let next = &chain[(i + 1) % chain.len()];
            if link.end_vid != next.start_vid {
                return false;
            }
        }
        // Second pass: build certified uses from the closed chain.
        for link in &chain {
            let edge = match topo.edge(link.edge_id) {
                Ok(edge) => edge,
                Err(_) => return false,
            };
            // Closed uses require one authoritative vertex: two distinct
            // coincident vertices are not claimed as one chart event here.
            if (link.start_pt - link.end_pt).length() <= self.tol && link.start_vid != link.end_vid
            {
                return false;
            }
            let boundary = Some(BoundarySource {
                face: face_id,
                boundary_loop: loop_id,
                coedge: link.coedge,
            });
            if !self.push_use(
                edge.curve(),
                edge.trim(),
                link.start_pt,
                link.end_pt,
                link.start_vid.index() as u64,
                link.end_vid.index() as u64,
                Some(loop_key),
                boundary,
                Some(link.edge_id.index()),
                None,
                false,
            ) {
                return false;
            }
            self.cert_positions
                .push((link.start_pt, link.start_vid.index() as u64));
            // A closed use contributes one chart vertex, not two.
            if link.start_vid != link.end_vid {
                self.cert_positions
                    .push((link.end_pt, link.end_vid.index() as u64));
            }
        }
        true
    }

    /// The loops must reproduce the wire the established splitter consumes,
    /// edge-for-edge in order. Anything else defers to the established path.
    fn loops_match_wires(topo: &Topology, face_id: FaceId) -> bool {
        let Ok(face) = topo.face(face_id) else {
            return false;
        };
        let Some(loop_ids) = topo.loops_of_face(face_id) else {
            return false;
        };
        let mut wire_ids = vec![face.outer_wire()];
        wire_ids.extend(face.inner_wires().iter().copied());
        if wire_ids.len() != loop_ids.len() {
            return false;
        }
        for (&wire_id, &loop_id) in wire_ids.iter().zip(loop_ids.iter()) {
            let (Ok(wire), Ok(face_loop)) = (topo.wire(wire_id), topo.face_loop(loop_id)) else {
                return false;
            };
            if wire.edges().len() != face_loop.coedges().len() {
                return false;
            }
            for (oriented, &coedge_id) in wire.edges().iter().zip(face_loop.coedges().iter()) {
                let Ok(coedge) = topo.coedge(coedge_id) else {
                    return false;
                };
                if coedge.edge() != oriented.edge() || coedge.is_forward() != oriented.is_forward()
                {
                    return false;
                }
            }
        }
        true
    }

    /// Push one section use. Returns `false` when the section is outside
    /// the qualified domain.
    ///
    /// Section pcurves are rebuilt in the split frame by [`Self::push_use`];
    /// the stored rank pcurves may live in a different frame, and NURBS-fit
    /// circles are never authoritative. The trim is the carried authority;
    /// only line and circle carriers qualify.
    fn push_section(&mut self, section: &SectionEdge) -> bool {
        if !matches!(section.curve_3d, EdgeCurve::Line | EdgeCurve::Circle(_)) {
            return false;
        }
        let start_cert = self.cert_for_computed_endpoint(section.start);
        let end_cert = if (section.start - section.end).length() <= self.tol {
            start_cert
        } else {
            self.cert_for_computed_endpoint(section.end)
        };
        self.push_use(
            &section.curve_3d,
            section.trim,
            section.start,
            section.end,
            start_cert,
            end_cert,
            None,
            None,
            None,
            section.pave_block_id,
            true,
        )
    }

    /// Correspondence derivation for a computed section endpoint: adopt the
    /// first authoritative vertex within tolerance, else mint a fresh
    /// certificate. Deterministic in assignment order; the core verifies
    /// every merge residual, so a wrong adoption refuses rather than welds.
    fn cert_for_computed_endpoint(&mut self, point: Point3) -> u64 {
        for (position, cert) in &self.cert_positions {
            if (*position - point).length() <= self.tol {
                return *cert;
            }
        }
        let cert = self.fresh_cert();
        self.cert_positions.push((point, cert));
        cert
    }

    /// Resolve interior concurrent line crossings: three or more line
    /// uses meeting at one interior point would re-derive that event
    /// once per pair, diverging by float noise and refusing as an
    /// unresolvable near-miss. Splitting every involved use at its own
    /// exact crossing parameter with one shared certificate turns the
    /// cluster into certified ends, which the core's dust-twin adoption
    /// merges into a single vertex. Returns `Ok(false)` for degenerate
    /// clusters (an end near the cluster without its certificate);
    /// circle-involved interior multiways stay deferred to the
    /// established path.
    fn resolve_line_concurrency(&mut self, context: &OperationContext) -> Result<bool, AlgoError> {
        let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
        // Exact transverse crossings between line uses.
        let mut crossings: Vec<Crossing> = Vec::new();
        for i in 0..self.uses.len() {
            for j in i + 1..self.uses.len() {
                context.check_cancelled().map_err(cancelled)?;
                let (Some(ti), Some(tj), point) =
                    line_crossing(&self.uses[i], &self.uses[j], self.tol)
                else {
                    continue;
                };
                crossings.push(Crossing {
                    first: i,
                    second: j,
                    first_param: ti,
                    second_param: tj,
                    point,
                });
            }
        }
        // Deterministic greedy clustering within tolerance.
        crossings.sort_by(|a, b| {
            a.point
                .x()
                .total_cmp(&b.point.x())
                .then(a.point.y().total_cmp(&b.point.y()))
        });
        let mut clusters: Vec<(Point2, Vec<(usize, f64)>)> = Vec::new();
        for crossing in &crossings {
            context.check_cancelled().map_err(cancelled)?;
            let mut placed = false;
            for (center, members) in &mut clusters {
                if (*center - crossing.point).length() <= self.tol {
                    members.push((crossing.first, crossing.first_param));
                    members.push((crossing.second, crossing.second_param));
                    placed = true;
                    break;
                }
            }
            if !placed {
                clusters.push((
                    crossing.point,
                    vec![
                        (crossing.first, crossing.first_param),
                        (crossing.second, crossing.second_param),
                    ],
                ));
            }
        }
        // Only multi-use clusters need shared certificates; pairs are
        // found exactly by the core with no duplicate computation.
        let mut cuts: BTreeMap<usize, Vec<(f64, u64)>> = BTreeMap::new();
        for (center, members) in &clusters {
            context.check_cancelled().map_err(cancelled)?;
            let mut distinct: Vec<usize> = members.iter().map(|(u, _)| *u).collect();
            distinct.sort_unstable();
            distinct.dedup();
            if distinct.len() < 3 {
                continue;
            }
            // Adopt an authoritative certificate at the cluster point
            // when one exists; otherwise mint a fresh one.
            let mut cluster_cert = None;
            for (position, cert) in &self.cert_positions {
                if (self.frame.project(*position) - *center).length() <= self.tol {
                    cluster_cert = Some(*cert);
                    break;
                }
            }
            let cluster_cert = cluster_cert.unwrap_or_else(|| self.fresh_cert());
            // Degenerate near-coincidence (an end at the cluster without
            // its certificate) defers rather than misbuilds.
            for use_data in &self.uses {
                for end in 0..2 {
                    if use_data.endpoints[end] != cluster_cert
                        && (eval_pcurve(use_data, use_data.range[end]) - *center).length()
                            <= self.tol
                    {
                        return Ok(false);
                    }
                }
            }
            // Split every involved use at its own exact parameter (one
            // split per use; same-event duplicates merge within
            // roundoff, distinct crossings stay distinct).
            let mut per_use: BTreeMap<usize, Vec<f64>> = BTreeMap::new();
            for (idx, param) in members {
                per_use.entry(*idx).or_default().push(*param);
            }
            for (idx, mut params) in per_use {
                params.sort_by(f64::total_cmp);
                let mut kept: Vec<f64> = Vec::new();
                for param in params {
                    if kept.last().is_none_or(|prev: &f64| {
                        (self.uses[idx].pcurve.evaluate(param)
                            - self.uses[idx].pcurve.evaluate(*prev))
                        .length()
                            > roundoff(self.uses[idx].pcurve.evaluate(param))
                    }) {
                        kept.push(param);
                    }
                }
                for param in kept {
                    cuts.entry(idx).or_default().push((param, cluster_cert));
                }
            }
        }
        if cuts.is_empty() {
            return Ok(true);
        }
        self.apply_cuts(cuts)
    }

    /// Apply collected per-target cuts in ascending target order, tracking
    /// the cumulative index shift that splitting inserts. A dust-scale
    /// cut table defers the whole face (see [`Self::split_use_at`]).
    fn apply_cuts(
        &mut self,
        mut cuts: BTreeMap<usize, Vec<(f64, u64)>>,
    ) -> Result<bool, AlgoError> {
        let targets: Vec<usize> = cuts.keys().copied().collect();
        let mut shift = 0usize;
        for target in targets {
            let mut list = cuts.remove(&target).unwrap_or_default();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            list.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-12);
            if !self.split_use_at(target + shift, &list)? {
                return Ok(false);
            }
            shift += list.len();
        }
        Ok(true)
    }

    /// Split uses at section endpoints landing on another use's interior.
    ///
    /// The core refines exact pair crossings but never snaps: an endpoint
    /// that misses its support by clip noise would dangle and refuse. The
    /// join is therefore established here with a shared certificate and
    /// exact subspans; the core re-derives the same event and verifies the
    /// residual.
    fn prejoin_endpoints(&mut self, context: &OperationContext) -> Result<bool, AlgoError> {
        let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
        // Section endpoints in deterministic order.
        let mut endpoints: Vec<(usize, usize)> = Vec::new();
        for (idx, _) in self.uses.iter().enumerate() {
            if self.parents[idx].is_section {
                endpoints.push((idx, 0));
                endpoints.push((idx, 1));
            }
        }
        // Candidate splits: (target use, parameter, shared cert). End-end
        // joining needs no action (certificates are already shared), but
        // a joint sitting on a third use's interior must split that use
        // too: otherwise the joint's incidences plus the third use's pair
        // hit form an uncertified multiway the core must refuse. One
        // split per (target, event): an event is one point, so further
        // claims collapse onto the first.
        let mut candidates: Vec<(usize, f64, u64)> = Vec::new();
        for (idx, side) in endpoints {
            context.check_cancelled().map_err(cancelled)?;
            let cert = self.uses[idx].endpoints[side];
            let uv = eval_pcurve(&self.uses[idx], self.uses[idx].range[side]);
            for (other_idx, other) in self.uses.iter().enumerate() {
                if other_idx == idx {
                    continue;
                }
                if let Some(parameter) = interior_parameter(other, uv, self.tol)
                    && !near_use_end(other, parameter, uv)
                    && !candidates
                        .iter()
                        .any(|(t, _, c)| *t == other_idx && *c == cert)
                {
                    candidates.push((other_idx, parameter, cert));
                }
            }
        }
        let mut cuts: BTreeMap<usize, Vec<(f64, u64)>> = BTreeMap::new();
        for (target, parameter, cert) in candidates {
            cuts.entry(target).or_default().push((parameter, cert));
        }
        // Splits insert uses, shifting later targets: tracked centrally
        // so every cut lands on its original target.
        self.apply_cuts(cuts)
    }

    /// No section endpoint may dangle: every certificate appearing on a
    /// section end must be incident to at least two use ends, so the trace
    /// can always continue through it. A degree-one foot is a spur, which
    /// the core would refuse as a bridge — out-of-domain here, where the
    /// established path owns spurs. Degree-two joints are pass-throughs
    /// (multi-window clip chains, bent spans); higher degrees are
    /// crossings and corners.
    fn section_ends_resolved(&self) -> bool {
        let mut degree: BTreeMap<u64, usize> = BTreeMap::new();
        for use_data in &self.uses {
            for cert in use_data.endpoints {
                *degree.entry(cert).or_default() += 1;
            }
        }
        self.uses.iter().enumerate().all(|(idx, use_data)| {
            if !self.parents[idx].is_section {
                return true;
            }
            use_data
                .endpoints
                .iter()
                .all(|cert| degree.get(cert).copied().unwrap_or(0) >= 2)
        })
    }

    /// Core use constructor shared by boundary and section inputs.
    ///
    /// Returns `Ok(false)` for out-of-domain input (unsupported curve,
    /// off-plane arc, missing/degenerate trim). All pcurve/3D consistency
    /// is verified so the core only ever sees gated input. Ranges are
    /// passed in traversal order; the core normalizes direction while
    /// preserving the end pairing.
    #[allow(clippy::too_many_arguments, clippy::too_many_lines)]
    fn push_use(
        &mut self,
        curve_3d: &EdgeCurve,
        trim: Option<(f64, f64)>,
        start_3d: Point3,
        end_3d: Point3,
        start_cert: u64,
        mut end_cert: u64,
        boundary_loop: Option<u64>,
        boundary: Option<BoundarySource>,
        source_topo_edge: Option<usize>,
        pave_block_id: Option<usize>,
        is_section: bool,
    ) -> bool {
        if !start_3d.x().is_finite()
            || !start_3d.y().is_finite()
            || !start_3d.z().is_finite()
            || !end_3d.x().is_finite()
            || !end_3d.y().is_finite()
            || !end_3d.z().is_finite()
        {
            return false;
        }
        let closed = (start_3d - end_3d).length() <= self.tol;
        match curve_3d {
            EdgeCurve::Line => {
                if (end_3d - start_3d).length() <= self.tol {
                    return false;
                }
                if !on_plane(self.frame, start_3d, self.tol)
                    || !on_plane(self.frame, end_3d, self.tol)
                {
                    return false;
                }
                let start_uv = self.frame.project(start_3d);
                let end_uv = self.frame.project(end_3d);
                let direction = end_uv - start_uv;
                let length = direction.length();
                if length <= self.tol {
                    return false;
                }
                let Ok(line) = Line2D::new(start_uv, direction) else {
                    return false;
                };
                let pcurve = Curve2D::Line(line);
                self.push_certified_use(
                    pcurve.clone(),
                    [0.0, length],
                    EdgeCurve::Line,
                    [0.0, 1.0],
                    [start_3d, end_3d],
                    [start_cert, end_cert],
                    boundary_loop,
                    boundary,
                    pave_block_id,
                    ParentInfo {
                        pcurve,
                        endpoints_3d: [start_3d, end_3d],
                        source_topo_edge,
                        pave_block_id,
                        is_section,
                    },
                );
                true
            }
            EdgeCurve::Circle(circle) => {
                if closed {
                    end_cert = start_cert;
                }
                let Some(pieces) = self.circle_pieces(
                    circle,
                    trim,
                    start_3d,
                    end_3d,
                    closed,
                    [start_cert, end_cert],
                ) else {
                    return false;
                };
                for piece in pieces {
                    self.push_certified_use(
                        Curve2D::Circle(piece.pcurve.clone()),
                        piece.range,
                        curve_3d.clone(),
                        piece.source_range,
                        piece.endpoints_3d,
                        piece.endpoints,
                        boundary_loop,
                        boundary.clone(),
                        pave_block_id,
                        ParentInfo {
                            pcurve: Curve2D::Circle(piece.pcurve),
                            endpoints_3d: piece.endpoints_3d,
                            source_topo_edge,
                            pave_block_id,
                            is_section,
                        },
                    );
                }
                true
            }
            _ => false,
        }
    }

    /// Push one certified use with its emission parent. The input position
    /// becomes the use id, the source-edge index, and (for sections) the
    /// section tag, keeping all three in lockstep.
    #[allow(clippy::too_many_arguments)]
    fn push_certified_use(
        &mut self,
        pcurve: Curve2D,
        range: [f64; 2],
        curve_3d: EdgeCurve,
        source_range: [f64; 2],
        endpoints_3d: [Point3; 2],
        endpoints: [u64; 2],
        boundary_loop: Option<u64>,
        boundary: Option<BoundarySource>,
        pave_block_id: Option<usize>,
        parent: ParentInfo,
    ) {
        let position = self.parents.len();
        self.uses.push(CurveUse {
            source: CurveSource {
                use_id: position as u64,
                boundary,
                section: parent.is_section.then_some(position),
                source_edge_idx: Some(position),
                pave_block_id,
            },
            pcurve,
            range,
            curve_3d,
            source_range,
            endpoints_3d,
            endpoints,
            boundary_loop,
        });
        self.parents.push(parent);
    }

    /// Authoritative circle pieces in traversal order: analytic pcurve,
    /// canonical `[0, 2π]` ranges, and native source intervals with the
    /// exact angle correspondence. Branch-crossing arcs (including full
    /// circles that do not start at angle zero) become two exact sub-uses
    /// sharing a fresh certificate at the cut.
    fn circle_pieces(
        &mut self,
        circle: &remus_math::curves::Circle3D,
        trim: Option<(f64, f64)>,
        start_3d: Point3,
        end_3d: Point3,
        closed: bool,
        certs: [u64; 2],
    ) -> Option<Vec<CirclePiece>> {
        // Without a carried trim the traversal span is unknowable: the
        // endpoint reconstruction always returns the CCW complement and
        // would silently claim the wrong arc for CW uses.
        let (trim_lo, trim_hi) = trim?;
        if !trim_lo.is_finite() || !trim_hi.is_finite() {
            return None;
        }
        // The trim is stored native; traversal runs start to end. Verify
        // the carried trim actually spans start to end: the carrier
        // evaluated at the trim ends must reproduce both endpoints. This
        // rejects stale trims instead of sweeping the wrong arc.
        //
        // Closed uses canonicalize to a CCW-native full span anchored at
        // the traversal start's own angle. The trim proves full span but
        // its branch says nothing about where traversal starts, so
        // anchoring to the trim branch would pair the wrong ends (the
        // core verifies source ends against endpoint positions). Sense
        // is unobservable for a closed loop; the set covers the carrier.
        let (native_start, native_end) = if closed {
            if (trim_hi - trim_lo).abs() < TAU - FULL_CIRCLE_BAND
                || (trim_hi - trim_lo).abs() > TAU + FULL_CIRCLE_BAND
            {
                return None;
            }
            let raw = circle.project(start_3d);
            if !raw.is_finite() {
                return None;
            }
            let anchor = raw.rem_euclid(TAU);
            if (circle.evaluate(anchor) - start_3d).length() > self.tol {
                return None;
            }
            (anchor, anchor + TAU)
        } else {
            let carrier_start = circle.evaluate(trim_lo);
            let carrier_end = circle.evaluate(trim_hi);
            let forward_match = (carrier_start - start_3d).length() <= self.tol
                && (carrier_end - end_3d).length() <= self.tol;
            let reverse_match = (carrier_start - end_3d).length() <= self.tol
                && (carrier_end - start_3d).length() <= self.tol;
            if forward_match {
                (trim_lo, trim_hi)
            } else if reverse_match {
                (trim_hi, trim_lo)
            } else {
                return None;
            }
        };
        let span = native_end - native_start;
        if span.abs() <= FULL_CIRCLE_BAND || span.abs() > TAU + FULL_CIRCLE_BAND {
            return None;
        }
        // The circle must lie in the face plane: centre on the plane and a
        // non-degenerate projected axis pair, else the UV projection is not
        // an isometry and angles are meaningless.
        let center_uv = self.frame.project(circle.center());
        if !on_plane(self.frame, circle.center(), self.tol) {
            return None;
        }
        let u_axis = circle.u_axis();
        let v_axis = circle.v_axis();
        let u2d = self.frame.project(circle.center() + u_axis) - center_uv;
        let v2d = self.frame.project(circle.center() + v_axis) - center_uv;
        if u2d.length() <= 1e-12 || v2d.length() <= 1e-12 {
            return None;
        }
        // Orientation sign: +1 when the carrier angle runs CCW in UV.
        let orientation = if u2d.x() * v2d.y() - u2d.y() * v2d.x() >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let Ok(pcurve) = Circle2D::new(center_uv, circle.radius()) else {
            return None;
        };
        if circle.radius() <= self.tol {
            return None;
        }
        // Anchor the correspondence at the traversal start. `rem_euclid`
        // can return exactly TAU through rounding; TAU is the branch cut,
        // canonically identical to angle zero.
        let start_angle = {
            let raw = angle_of(center_uv, self.frame.project(start_3d));
            if raw >= TAU { 0.0 } else { raw }
        };
        let signed_span = orientation * span;
        // Directed traversal interval in UV angles (possibly descending
        // for CW uses, possibly leaving `[0, 2π]`). A full turn from a
        // nonzero start always crosses the cut exactly once; a partial
        // arc crosses it at most once since |span| <= 2π.
        let end_angle = start_angle + signed_span;
        let crosses = !(0.0..=TAU).contains(&end_angle);
        // (range, source) per piece, each in traversal order. Sources
        // come from the directed chart (uniform angle motion, exact
        // fractions); ranges use canonical `[0, 2π]` representatives of
        // the same geometric angles. A traversal starting exactly on
        // the cut with a negative span takes one descending canonical
        // piece instead of a degenerate cut pair.
        let cut_source = |cut_directed: f64| -> f64 {
            native_start + span * (cut_directed - start_angle) / (end_angle - start_angle)
        };
        let pieces_raw: Vec<RawPiece> = if start_angle == 0.0 && signed_span < 0.0 {
            vec![RawPiece {
                range: [TAU, end_angle + TAU],
                source: [native_start, native_end],
            }]
        } else if !crosses {
            vec![RawPiece {
                range: [start_angle, end_angle],
                source: [native_start, native_end],
            }]
        } else {
            let cut = if signed_span > 0.0 { TAU } else { 0.0 };
            let restart = if signed_span > 0.0 { 0.0 } else { TAU };
            let canon_end = if signed_span > 0.0 {
                end_angle - TAU
            } else {
                end_angle + TAU
            };
            let source_cut = cut_source(cut);
            vec![
                RawPiece {
                    range: [start_angle, cut],
                    source: [native_start, source_cut],
                },
                RawPiece {
                    range: [restart, canon_end],
                    source: [source_cut, native_end],
                },
            ]
        };
        let cut_cert = self.fresh_cert();
        if pieces_raw.len() == 2 {
            // A genuine branch split (not the degenerate single-piece
            // path): record the cut certificate so emission can merge
            // across it. The cut marks no geometric event, so keeping it
            // as a wire vertex would hand neighboring faces a
            // T-junction there.
            self.branch_certs.insert(cut_cert);
        }
        let mut pieces = Vec::new();
        for (piece_idx, raw) in pieces_raw.iter().enumerate() {
            // Canonical window check: every piece must lie in `[0, 2π]`.
            if raw.range[0] < 0.0 || raw.range[0] > TAU || raw.range[1] < 0.0 || raw.range[1] > TAU
            {
                return None;
            }
            let first = piece_idx == 0;
            let last = piece_idx + 1 == pieces_raw.len();
            let range = raw.range;
            let source_sub = raw.source;
            let start_3d_piece = if first {
                start_3d
            } else {
                circle.evaluate(source_sub[0])
            };
            let end_3d_piece = if last {
                end_3d
            } else {
                circle.evaluate(source_sub[1])
            };
            let start_cert_piece = if first { certs[0] } else { cut_cert };
            let end_cert_piece = if last { certs[1] } else { cut_cert };
            pieces.push(CirclePiece {
                pcurve: pcurve.clone(),
                range,
                source_range: source_sub,
                endpoints_3d: [start_3d_piece, end_3d_piece],
                endpoints: [start_cert_piece, end_cert_piece],
            });
        }
        // Verify every piece: the derived range must reproduce both
        // ends, and the arc midpoint must lie in the plane.
        for piece in &pieces {
            let pcurve_eval = |t: f64| {
                Point2::new(
                    center_uv.x() + circle.radius() * t.cos(),
                    center_uv.y() + circle.radius() * t.sin(),
                )
            };
            let far_uv = pcurve_eval(piece.range[1]);
            let expected = self.frame.project(piece.endpoints_3d[1]);
            if (far_uv - expected).length() > self.tol {
                return None;
            }
            let near_uv = pcurve_eval(piece.range[0]);
            let expected_near = self.frame.project(piece.endpoints_3d[0]);
            if (near_uv - expected_near).length() > self.tol {
                return None;
            }
            let mid_source = (piece.source_range[0] + piece.source_range[1]) * 0.5;
            if !on_plane(self.frame, circle.evaluate(mid_source), self.tol) {
                return None;
            }
        }
        Some(pieces)
    }

    /// Split target use at the given interior parameters, sharing each
    /// cut's certificate. Source subspans divide affinely, which is exact
    /// for lines (fraction space) and circles (uniform angle motion).
    /// Splits a use at interior cut parameters with shared certificates.
    ///
    /// Returns `Ok(false)` (defer to the established weld-based paths)
    /// when two distinct cuts land within endpoint margin of each other:
    /// near-coincident distinct events belong to tolerance welding, not
    /// to exact subdivision. Only a truly inconsistent cut table is an
    /// internal error.
    fn split_use_at(&mut self, target: usize, cuts: &[(f64, u64)]) -> Result<bool, AlgoError> {
        if cuts.is_empty() {
            return Ok(true);
        }
        let parent = self.uses[target].clone();
        let parent_info = self.parents[target].clone();
        let mut bounds = vec![parent.range[0]];
        for (parameter, _) in cuts {
            bounds.push(*parameter);
        }
        bounds.push(parent.range[1]);
        // Certificates keyed by cut parameter.
        let cert_at = |parameter: f64| -> Result<u64, AlgoError> {
            if (parameter - parent.range[0]).abs() <= END_MARGIN {
                return Ok(parent.endpoints[0]);
            }
            if (parameter - parent.range[1]).abs() <= END_MARGIN {
                return Ok(parent.endpoints[1]);
            }
            cuts.iter()
                .find(|(cut, _)| (*cut - parameter).abs() <= 1e-12)
                .map(|(_, cert)| *cert)
                .ok_or_else(|| collect_failed(ArrangementError::OpenRegion))
        };
        let mut pieces: Vec<CurveUse> = Vec::new();
        let mut infos: Vec<ParentInfo> = Vec::new();
        for window in bounds.windows(2) {
            let (lo, hi) = (window[0], window[1]);
            if hi - lo <= END_MARGIN {
                return Ok(false);
            }
            let fraction = |t: f64| (t - parent.range[0]) / (parent.range[1] - parent.range[0]);
            let source_at = |t: f64| {
                parent.source_range[0]
                    + (parent.source_range[1] - parent.source_range[0]) * fraction(t)
            };
            let start_cert = cert_at(lo)?;
            let end_cert = cert_at(hi)?;
            let start_3d = point_on_parent(&parent, &parent_info, lo);
            let end_3d = point_on_parent(&parent, &parent_info, hi);
            pieces.push(CurveUse {
                source: CurveSource {
                    use_id: 0,
                    boundary: parent.source.boundary.clone(),
                    section: parent.source.section,
                    source_edge_idx: Some(0),
                    pave_block_id: parent.source.pave_block_id,
                },
                pcurve: parent.pcurve.clone(),
                range: [lo, hi],
                curve_3d: parent.curve_3d.clone(),
                source_range: [source_at(lo), source_at(hi)],
                endpoints_3d: [start_3d, end_3d],
                endpoints: [start_cert, end_cert],
                boundary_loop: parent.boundary_loop,
            });
            infos.push(ParentInfo {
                pcurve: parent.pcurve.clone(),
                endpoints_3d: [start_3d, end_3d],
                source_topo_edge: parent_info.source_topo_edge,
                pave_block_id: parent_info.pave_block_id,
                is_section: parent_info.is_section,
            });
        }
        // Replace the parent with its pieces, preserving input order.
        self.uses.splice(target..=target, pieces);
        self.parents.splice(target..=target, infos);
        self.renumber_uses();
        Ok(true)
    }

    /// Restore use-id and source-edge-index consistency after splits.
    fn renumber_uses(&mut self) {
        for (idx, use_data) in self.uses.iter_mut().enumerate() {
            use_data.source.use_id = idx as u64;
            use_data.source.source_edge_idx = Some(idx);
        }
    }

    fn finish(mut self) -> Option<PlanarInputs> {
        if self.uses.is_empty() {
            return None;
        }
        self.renumber_uses();
        // Section tags track input positions; re-derive them after splits.
        for (idx, use_data) in self.uses.iter_mut().enumerate() {
            if self.parents[idx].is_section {
                use_data.source.section = Some(idx);
            }
        }
        Some(PlanarInputs {
            uses: self.uses,
            parents: self.parents,
            branch_certs: self.branch_certs,
        })
    }
}

/// Frame round-trip: an in-plane point projects and evaluates back to
/// itself within the gate band.
fn on_plane(frame: &PlaneFrame, point: Point3, tol: f64) -> bool {
    let uv = frame.project(point);
    if !uv.x().is_finite() || !uv.y().is_finite() {
        return false;
    }
    (frame.evaluate(uv.x(), uv.y()) - point).length() <= tol * PLANE_GATE_SCALE
}

fn angle_of(center: Point2, point: Point2) -> f64 {
    (point.y() - center.y())
        .atan2(point.x() - center.x())
        .rem_euclid(TAU)
}

/// Pcurve evaluation for a collected use (the core's private helper is
/// not visible here; this is the same single dispatch).
fn eval_pcurve(use_data: &CurveUse, t: f64) -> Point2 {
    use_data.pcurve.evaluate(t)
}

/// Bitwise interval equality for untouched-use detection.
fn same_interval(a: [f64; 2], b: [f64; 2]) -> bool {
    a[0].total_cmp(&b[0]).is_eq() && a[1].total_cmp(&b[1]).is_eq()
}

fn roundoff(p: Point2) -> f64 {
    64.0 * f64::EPSILON * (1.0 + p.x().abs() + p.y().abs())
}

/// Exact transverse crossing between two line uses, if their interiors
/// meet transversely. Endpoint touches are not crossings (they already
/// carry certificates); parallel and near-parallel pairs are skipped
/// (the core's overlap handling owns them). Any exact solve will do:
/// the core's dust-twin adoption absorbs float noise at certified ends.
fn line_crossing(a: &CurveUse, b: &CurveUse, tol: f64) -> (Option<f64>, Option<f64>, Point2) {
    let (Curve2D::Line(la), Curve2D::Line(lb)) = (&a.pcurve, &b.pcurve) else {
        return (None, None, Point2::new(0.0, 0.0));
    };
    let (a0, a1) = (a.pcurve.evaluate(a.range[0]), a.pcurve.evaluate(a.range[1]));
    let (b0, b1) = (b.pcurve.evaluate(b.range[0]), b.pcurve.evaluate(b.range[1]));
    let ar = orient2d(a0, a1, b0);
    let as_ = orient2d(a0, a1, b1);
    let bp = orient2d(b0, b1, a0);
    let bq = orient2d(b0, b1, a1);
    // Strict straddle on both sides: transverse interior crossing.
    let straddle_a = (ar > 0.0 && as_ < 0.0) || (ar < 0.0 && as_ > 0.0);
    let straddle_b = (bp > 0.0 && bq < 0.0) || (bp < 0.0 && bq > 0.0);
    if !straddle_a || !straddle_b {
        return (None, None, Point2::new(0.0, 0.0));
    }
    let d = la.direction();
    let e = lb.direction();
    let den = d.x() * e.y() - d.y() * e.x();
    if den.abs() < 1e-12 {
        return (None, None, Point2::new(0.0, 0.0));
    }
    let delta = Point2::new(b0.x() - a0.x(), b0.y() - a0.y());
    let ta = (delta.x() * e.y() - delta.y() * e.x()) / den;
    let tb = (delta.x() * d.y() - delta.y() * d.x()) / den;
    // Line2D parameters are arc-length from each pcurve origin, and
    // every line pcurve is origin-based, so solve offsets are native
    // parameters directly (even on split sub-pieces).
    let (pa, pb) = (ta, tb);
    let in_a = pa > a.range[0] + END_MARGIN && pa < a.range[1] - END_MARGIN;
    let in_b = pb > b.range[0] + END_MARGIN && pb < b.range[1] - END_MARGIN;
    if !in_a || !in_b {
        return (None, None, Point2::new(0.0, 0.0));
    }
    if (la.evaluate(ta) - lb.evaluate(tb)).length() > tol {
        return (None, None, Point2::new(0.0, 0.0));
    }
    (Some(pa), Some(pb), la.evaluate(ta))
}

/// Exact parameter where `point` meets `use_data`'s curve, if strictly
/// interior to the use. Tangential grazes are reported like crossings:
/// the core refuses ambiguous contacts, so surfacing the candidate is
/// honest — the refusal then carries the true geometry.
fn interior_parameter(use_data: &CurveUse, point: Point2, tol: f64) -> Option<f64> {
    match &use_data.pcurve {
        Curve2D::Line(line) => {
            let parameter = line.project(point);
            let closest = line.evaluate(parameter);
            if (closest - point).length() > tol {
                return None;
            }
            if parameter > use_data.range[0] + END_MARGIN
                && parameter < use_data.range[1] - END_MARGIN
            {
                Some(parameter)
            } else {
                None
            }
        }
        Curve2D::Circle(circle) => {
            let radial = (point - circle.center()).length() - circle.radius();
            if radial.abs() > tol {
                return None;
            }
            // The use never wraps the branch cut, so at most one lift
            // brings the angle into the use's branch.
            let mut angle = angle_of(circle.center(), point);
            if angle < use_data.range[0] {
                angle += TAU;
            }
            if angle > use_data.range[0] + END_MARGIN && angle < use_data.range[1] - END_MARGIN {
                Some(angle)
            } else {
                None
            }
        }
        _ => None,
    }
}

/// Whether a cut parameter touches a use end (then it is an end-touch,
/// not an interior split).
fn near_use_end(use_data: &CurveUse, parameter: f64, cut_uv: Point2) -> bool {
    // A cut admitted by `interior_parameter` already clears END_MARGIN;
    // this guards the roundoff band at the ends for large coordinates.
    let _ = parameter;
    let lo_uv = eval_pcurve(use_data, use_data.range[0]);
    let hi_uv = eval_pcurve(use_data, use_data.range[1]);
    (lo_uv - cut_uv).length() <= roundoff(cut_uv) || (hi_uv - cut_uv).length() <= roundoff(cut_uv)
}

/// Exact 3D point of a parent use at a native pcurve parameter: carrier
/// evaluation for circles, endpoint lerp for lines — the same computation
/// the core performs, so emission and refinement agree.
fn point_on_parent(parent: &CurveUse, info: &ParentInfo, parameter: f64) -> Point3 {
    match &parent.curve_3d {
        EdgeCurve::Line => {
            let fraction = (parameter - parent.range[0]) / (parent.range[1] - parent.range[0]);
            info.endpoints_3d[0] + (info.endpoints_3d[1] - info.endpoints_3d[0]) * fraction
        }
        EdgeCurve::Circle(_) => {
            let fraction = (parameter - parent.range[0]) / (parent.range[1] - parent.range[0]);
            let source = parent.source_range[0]
                + (parent.source_range[1] - parent.source_range[0]) * fraction;
            parent.curve_3d.evaluate_with_endpoints(
                source,
                info.endpoints_3d[0],
                info.endpoints_3d[1],
            )
        }
        _ => info.endpoints_3d[0],
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::vec::{Vec2, Vec3};
    use remus_topology::edge::Edge;
    use remus_topology::face::Face;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    const TOL: f64 = 1e-7;

    fn test_context() -> OperationContext {
        OperationContext::new().with_budgets(
            remus_math::context::WorkBudgets::new()
                .with_march_steps(2_000_000)
                .with_queue_size(20_000)
                .with_segments(10_000),
        )
    }

    fn plane_frame() -> PlaneFrame {
        PlaneFrame::from_normal_and_point(Vec3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, 0.0))
    }

    fn plane_frame_at(point: Point3) -> PlaneFrame {
        PlaneFrame::from_normal_and_point(Vec3::new(0.0, 0.0, 1.0), point)
    }

    fn square_topology(size: f64) -> (Topology, FaceId) {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(size, 0.0, 0.0),
            Point3::new(size, size, 0.0),
            Point3::new(0.0, size, 0.0),
        ];
        let mut vids = Vec::new();
        for corner in corners {
            vids.push(topo.add_vertex(Vertex::new(corner, TOL)));
        }
        let mut edges = Vec::new();
        for i in 0..4 {
            edges.push(topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line)));
        }
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        (topo, face)
    }

    fn line_section(start: Point3, end: Point3) -> SectionEdge {
        use remus_math::curves2d::Line2D;
        use remus_math::vec::Vec2;
        let dir = Vec2::new(end.x() - start.x(), end.y() - start.y());
        let len = dir.length().max(1e-12);
        let pcurve = Curve2D::Line(
            Line2D::new(
                Point2::new(start.x(), start.y()),
                Vec2::new(dir.x() / len, dir.y() / len),
            )
            .unwrap(),
        );
        SectionEdge {
            curve_3d: EdgeCurve::Line,
            trim: None,
            pcurve_a: pcurve.clone(),
            pcurve_b: pcurve,
            start,
            end,
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: Some(7),
        }
    }

    fn collect(topo: &Topology, face: FaceId, sections: &[SectionEdge]) -> Option<PlanarInputs> {
        collect_planar_uses(
            topo,
            face,
            sections,
            Rank::A,
            &plane_frame(),
            &Tolerance::default(),
            &test_context(),
        )
        .unwrap()
    }

    fn collect_in(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
        frame: &PlaneFrame,
    ) -> Option<PlanarInputs> {
        collect_planar_uses(
            topo,
            face,
            sections,
            Rank::A,
            frame,
            &Tolerance::default(),
            &test_context(),
        )
        .unwrap()
    }

    fn run(inputs: &PlanarInputs) -> Arrangement {
        run_planar_arrangement(inputs, &test_context()).unwrap()
    }

    fn material_count(arrangement: &Arrangement) -> usize {
        arrangement.regions.iter().filter(|r| r.material).count()
    }

    fn material_area(arrangement: &Arrangement) -> f64 {
        arrangement
            .regions
            .iter()
            .filter(|r| r.material)
            .map(|r| r.area)
            .sum()
    }

    #[test]
    fn crossing_lines_yield_four_material_regions() {
        let (topo, face) = square_topology(2.0);
        // Sections pre-clipped to the face, as the production clipper does.
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let inputs = collect(&topo, face, &sections).expect("qualified planar X");
        // 4 boundary uses split at the four T feet, plus 2 sections.
        assert_eq!(inputs.uses.len(), 10);
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 4);
        assert!((material_area(&arrangement) - 4.0).abs() < 1e-9);
        for half in &arrangement.half_edges {
            assert!(half.source_range[0].is_finite() && half.source_range[1].is_finite());
        }
    }

    #[test]
    fn t_junction_between_sections_partitions_three_ways() {
        let (topo, face) = square_topology(2.0);
        // A full-span vertical plus a half-span horizontal forming a T at
        // (1, 1): the horizontal's foot splits the vertical with a shared
        // certificate, and every endpoint anchors to the boundary.
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let inputs = collect(&topo, face, &sections).expect("qualified planar T");
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 3);
        assert!((material_area(&arrangement) - 4.0).abs() < 1e-9);
    }

    #[test]
    fn dangling_spur_section_defers_to_established_path() {
        let (topo, face) = square_topology(2.0);
        // A single section ending in the face interior partitions nothing;
        // the established path owns spurs.
        let sections = [line_section(
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        )];
        assert!(collect(&topo, face, &sections).is_none());
    }

    #[test]
    fn disc_with_chord_keeps_major_arc_source_span() {
        use std::f64::consts::PI;
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let start = circle.evaluate(0.0);
        let v = topo.add_vertex(Vertex::new(start, TOL));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let eid = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        // A chord at y = 1, pre-clipped to the disc: minor cap above,
        // major remnant below.
        let root = 3.0_f64.sqrt();
        let sections = [line_section(
            Point3::new(-root, 1.0, 0.0),
            Point3::new(root, 1.0, 0.0),
        )];
        let inputs = collect(&topo, face, &sections).expect("qualified disc");
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 2);
        let cap = 4.0 * PI / 3.0 - root;
        let mut areas: Vec<f64> = arrangement
            .regions
            .iter()
            .filter(|r| r.material)
            .map(|r| r.area)
            .collect();
        areas.sort_by(f64::total_cmp);
        assert!((areas[0] - cap).abs() < 1e-9, "cap {areas:?}");
        assert!((areas[1] - (4.0 * PI - cap)).abs() < 1e-9);
        // The full circle source is fully covered by its subspans.
        let covered: f64 = arrangement
            .half_edges
            .iter()
            .step_by(2)
            .filter(|e| e.source.boundary.is_some())
            .map(|e| (e.source_range[1] - e.source_range[0]).abs())
            .sum();
        assert!((covered - TAU).abs() < 1e-9, "covered {covered}");
    }

    #[test]
    fn branch_crossing_arc_splits_exactly_at_angle_zero() {
        use std::f64::consts::PI;
        // An arc from 3π/2 to π/2 (CCW through angle zero): the adapter
        // must split it at the cut into two canonical pieces whose native
        // subspans rejoin exactly.
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 5.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let a = circle.evaluate(3.0 * PI / 2.0);
        let b = circle.evaluate(PI / 2.0);
        let va = topo.add_vertex(Vertex::new(a, TOL));
        let vb = topo.add_vertex(Vertex::new(b, TOL));
        let vc = topo.add_vertex(Vertex::new(center, TOL));
        let mut edge = Edge::new(va, vb, EdgeCurve::Circle(circle));
        edge.set_trim(Some((3.0 * PI / 2.0, 5.0 * PI / 2.0)));
        let eid = topo.add_edge(edge);
        let l1 = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let l2 = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(eid, true),
                    OrientedEdge::new(l1, true),
                    OrientedEdge::new(l2, true),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 5.0,
            },
        ));
        let inputs = collect_in(&topo, face, &[], &plane_frame_at(center))
            .expect("qualified branch crosser");
        let arc_uses: Vec<_> = inputs
            .uses
            .iter()
            .filter(|u| matches!(u.curve_3d, EdgeCurve::Circle(_)))
            .collect();
        assert_eq!(arc_uses.len(), 2);
        // Native subspans rejoin to the parent trim, in order.
        let mut spans: Vec<[f64; 2]> = arc_uses.iter().map(|u| u.source_range).collect();
        spans.sort_by(|a, b| a[0].total_cmp(&b[0]));
        assert!((spans[0][1] - spans[1][0]).abs() < 1e-12);
        let total: f64 = spans.iter().map(|s| (s[1] - s[0]).abs()).sum();
        assert!((total - PI).abs() < 1e-9, "total {total}");
        // Both pieces are canonical branches joined at the cut.
        for use_data in &arc_uses {
            assert!(use_data.range[0] >= 0.0 && use_data.range[1] <= TAU);
        }
        let cut_count = arc_uses
            .iter()
            .flat_map(|u| [u.range[0], u.range[1]])
            .filter(|t| t.total_cmp(&0.0).is_eq() || t.total_cmp(&TAU).is_eq())
            .count();
        assert_eq!(cut_count, 2, "one shared branch cut");
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 1);
    }

    #[test]
    fn reversed_circle_use_preserves_traversal_source_span() {
        use std::f64::consts::PI;
        // A CW (reversed-coedge) quarter arc: pcurve range runs with the
        // traversal while the native source runs high to low; the core
        // normalizes direction preserving the end pairing.
        let mut topo = Topology::new();
        let center = Point3::new(5.0, 5.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let a = circle.evaluate(0.0);
        let b = circle.evaluate(PI / 2.0);
        let va = topo.add_vertex(Vertex::new(a, TOL));
        let vb = topo.add_vertex(Vertex::new(b, TOL));
        let vc = topo.add_vertex(Vertex::new(center, TOL));
        let mut edge = Edge::new(va, vb, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, PI / 2.0)));
        let eid = topo.add_edge(edge);
        let l1 = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let l2 = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        // Traverse arc reversed (b -> a, CW), then va -> vc -> vb.
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(eid, false),
                    OrientedEdge::new(l2, false),
                    OrientedEdge::new(l1, false),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let inputs = collect(&topo, face, &[]).expect("qualified reversed arc");
        let arc_use = inputs
            .uses
            .iter()
            .find(|u| matches!(u.curve_3d, EdgeCurve::Circle(_)))
            .expect("arc use");
        // Traversal order: pcurve high-to-low with the native source.
        assert!((arc_use.range[0] - PI / 2.0).abs() < 1e-12);
        assert!(arc_use.range[1].abs() < 1e-12);
        assert!((arc_use.source_range[0] - PI / 2.0).abs() < 1e-12);
        assert!(arc_use.source_range[1].abs() < 1e-12);
        // End pairing is preserved: range end i matches source end i.
        for end in 0..2 {
            let uv = eval_pcurve(arc_use, arc_use.range[end]);
            let expected = plane_frame().project(arc_use.endpoints_3d[end]);
            assert!((uv - expected).length() < 1e-12);
        }
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 1);
    }

    #[test]
    fn square_hole_yields_holed_material_region() {
        let mut topo = Topology::new();
        let outer = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ];
        let hole_pts = [
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
            Point3::new(3.0, 3.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
        ];
        let mut wire_of = |points: &[Point3; 4]| {
            let mut vids = Vec::new();
            for point in points {
                vids.push(topo.add_vertex(Vertex::new(*point, TOL)));
            }
            let mut edges = Vec::new();
            for i in 0..4 {
                edges.push(topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line)));
            }
            topo.add_wire(
                Wire::new(
                    edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                    true,
                )
                .unwrap(),
            )
        };
        let outer_wire = wire_of(&outer);
        let hole_wire = wire_of(&hole_pts);
        let face = topo.add_face(Face::new(
            outer_wire,
            vec![hole_wire],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let inputs = collect(&topo, face, &[]).expect("qualified holed face");
        assert_eq!(inputs.uses.len(), 8);
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 1);
        assert!((material_area(&arrangement) - 12.0).abs() < 1e-9);
        assert_eq!(arrangement.regions.iter().filter(|r| r.material).count(), 1);
        let region = arrangement.regions.iter().find(|r| r.material).unwrap();
        assert_eq!(region.holes.len(), 1);
    }

    #[test]
    fn nurbs_boundary_defers_to_established_path() {
        // One boundary edge carried as NURBS: outside the line/circle
        // domain, so the adapter defers without touching dispatch.
        let start = Point3::new(0.0, 0.0, 0.0);
        let corner = Point3::new(2.0, 0.0, 0.0);
        let end = Point3::new(2.0, 2.0, 0.0);
        let last = Point3::new(0.0, 2.0, 0.0);
        let nurbs = remus_math::nurbs::fitting::interpolate(&[start, corner, end], 1).unwrap();
        let mut topo = Topology::new();
        let va = topo.add_vertex(Vertex::new(start, TOL));
        let vb = topo.add_vertex(Vertex::new(corner, TOL));
        let vc = topo.add_vertex(Vertex::new(end, TOL));
        let vd = topo.add_vertex(Vertex::new(last, TOL));
        let n0 = topo.add_edge(Edge::new(va, vb, EdgeCurve::NurbsCurve(nurbs)));
        let n1 = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let n2 = topo.add_edge(Edge::new(vc, vd, EdgeCurve::Line));
        let n3 = topo.add_edge(Edge::new(vd, va, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                [n0, n1, n2, n3]
                    .iter()
                    .map(|e| OrientedEdge::new(*e, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        assert!(collect(&topo, face, &[]).is_none());
    }

    #[test]
    fn open_wire_face_defers_to_established_path() {
        // A face whose boundary wire is not closed derives no authoritative
        // loop; the adapter defers rather than guessing connectivity.
        let mut topo = Topology::new();
        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), TOL));
        let b = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), TOL));
        let c = topo.add_vertex(Vertex::new(Point3::new(2.0, 2.0, 0.0), TOL));
        let e0 = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
        let open_wire = topo.add_wire(
            Wire::new(
                vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
                false,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            open_wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        assert!(collect(&topo, face, &[]).is_none());
    }

    #[test]
    fn tangent_section_refuses_as_internal_error() {
        // A tangent segment with a free end is a spur and defers (see
        // `dangling_spur_section_defers_to_established_path`). The
        // anchored tangent configuration is a closed ring section tangent
        // to the boundary: an inscribed circle touching the disc rim at
        // exactly one point. Both ends share the touch certificate, the
        // touch splits the boundary, and the core refuses the tangent
        // contact with equal limiting tangents.
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let start = circle.evaluate(0.0);
        let v = topo.add_vertex(Vertex::new(start, TOL));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let eid = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        // Inscribed circle of radius 1 about (1, 0, 0), touching (2, 0).
        let touch = Point3::new(2.0, 0.0, 0.0);
        let inner =
            Circle3D::new(Point3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let proto = line_section(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let sections = [SectionEdge {
            curve_3d: EdgeCurve::Circle(inner),
            trim: Some((0.0, TAU)),
            pcurve_a: proto.pcurve_a.clone(),
            pcurve_b: proto.pcurve_b,
            start: touch,
            end: touch,
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: Some(11),
        }];
        let inputs = collect(&topo, face, &sections).expect("tangent is gated in");
        let error = run_planar_arrangement(&inputs, &test_context()).unwrap_err();
        assert!(
            format!("{error:?}").contains("AmbiguousContact"),
            "unexpected {error:?}"
        );
    }

    #[test]
    fn exhausted_budget_fails_atomically() {
        let (topo, face) = square_topology(2.0);
        let sections = [line_section(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        )];
        let inputs = collect(&topo, face, &sections).expect("qualified");
        let starved = OperationContext::new().with_budgets(
            remus_math::context::WorkBudgets::new()
                .with_march_steps(0)
                .with_queue_size(0)
                .with_segments(0),
        );
        let error = run_planar_arrangement(&inputs, &starved).unwrap_err();
        assert!(
            format!("{error:?}").contains("WorkBudgetExceeded"),
            "unexpected {error:?}"
        );
    }

    #[test]
    fn cancelled_context_fails_atomically() {
        let (topo, face) = square_topology(2.0);
        let sections = [line_section(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        )];
        let inputs = collect(&topo, face, &sections).expect("qualified");
        let token = remus_math::context::CancellationToken::new();
        token.cancel();
        let cancelled = OperationContext::new().with_cancellation(token);
        let error = run_planar_arrangement(&inputs, &cancelled).unwrap_err();
        assert!(
            format!("{error:?}").contains("Cancelled"),
            "unexpected {error:?}"
        );
    }

    #[test]
    fn ellipse_section_defers_to_established_path() {
        let (topo, face) = square_topology(2.0);
        let ellipse = remus_math::curves::Ellipse3D::new(
            Point3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.5,
            0.25,
        )
        .unwrap();
        let proto = line_section(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let sections = [SectionEdge {
            curve_3d: EdgeCurve::Ellipse(ellipse),
            trim: Some((0.0, 1.0)),
            pcurve_a: proto.pcurve_a.clone(),
            pcurve_b: proto.pcurve_b,
            start: Point3::new(0.5, 1.0, 0.0),
            end: Point3::new(1.5, 1.0, 0.0),
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: None,
        }];
        assert!(collect(&topo, face, &sections).is_none());
    }

    #[test]
    fn non_plane_face_defers_to_established_path() {
        let (mut topo, face) = square_topology(2.0);
        let cylinder = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
        )
        .unwrap();
        // A second face sharing the square boundary but carried on a
        // cylinder surface: the adapter only reads planes, so it defers
        // before any loop work.
        let outer = topo.face(face).unwrap().outer_wire();
        let cylinder_face = topo.add_face(Face::new(
            outer,
            Vec::new(),
            remus_topology::face::FaceSurface::Cylinder(cylinder),
        ));
        assert!(collect(&topo, cylinder_face, &[]).is_none());
        // The original plane face is unaffected and still qualifies.
        assert!(collect(&topo, face, &[]).is_some());
    }
    #[test]
    fn arc_starting_on_branch_cut_canonicalizes_to_zero() {
        use std::f64::consts::PI;
        // `atan2` angles infinitesimally below zero map through
        // `rem_euclid` to exactly TAU (rounding). A traversal starting
        // there must canonicalize to angle zero, not emit a degenerate
        // `[TAU, TAU]` first piece (which the core refuses as an
        // invalid boundary). Production witness: the 2-tangency
        // box fuse, whose section arc starts on the cut.
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let tiny = -5e-17_f64;
        let a = circle.evaluate(tiny);
        let b = circle.evaluate(tiny + PI / 2.0);
        let va = topo.add_vertex(Vertex::new(a, TOL));
        let vb = topo.add_vertex(Vertex::new(b, TOL));
        let vc = topo.add_vertex(Vertex::new(center, TOL));
        let mut edge = Edge::new(va, vb, EdgeCurve::Circle(circle));
        edge.set_trim(Some((tiny, tiny + PI / 2.0)));
        let eid = topo.add_edge(edge);
        let l1 = topo.add_edge(Edge::new(vb, vc, EdgeCurve::Line));
        let l2 = topo.add_edge(Edge::new(vc, va, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(eid, true),
                    OrientedEdge::new(l1, true),
                    OrientedEdge::new(l2, true),
                ],
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let inputs = collect(&topo, face, &[]).expect("qualified cut-start arc");
        let arc_use = inputs
            .uses
            .iter()
            .find(|u| matches!(u.curve_3d, EdgeCurve::Circle(_)))
            .expect("arc use");
        assert!(arc_use.range[0].total_cmp(&0.0).is_eq());
        assert!((arc_use.range[1] - PI / 2.0).abs() < 1e-9);
        let arrangement = run(&inputs);
        assert_eq!(material_count(&arrangement), 1);
    }

    fn established_boundary(
        topo: &Topology,
        face: FaceId,
        frame: &PlaneFrame,
    ) -> Vec<OrientedPCurveEdge> {
        let face_data = topo.face(face).unwrap();
        let wire_pts = super::super::collect_wire_points(topo, face_data.outer_wire());
        super::super::conversion::boundary_edges_to_pcurve(
            topo,
            face_data.outer_wire(),
            face_data.surface(),
            &wire_pts,
            Some(frame),
        )
        .unwrap()
    }

    fn established_crossing(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
        frame: &PlaneFrame,
    ) -> Option<Vec<SplitSubFace>> {
        let face_data = topo.face(face).unwrap();
        let boundary = established_boundary(topo, face, frame);
        super::super::special_cases::try_split_crossing_plane_face(
            face_data.surface(),
            &boundary,
            sections,
            Rank::A,
            false,
            face,
            frame,
            &Tolerance::default(),
        )
        .unwrap()
    }

    fn established_disk(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
        frame: &PlaneFrame,
    ) -> Option<Vec<SplitSubFace>> {
        let face_data = topo.face(face).unwrap();
        let boundary = established_boundary(topo, face, frame);
        super::super::special_cases::try_split_disk_by_chords(
            face_data.surface(),
            &boundary,
            sections,
            Rank::A,
            false,
            face,
            frame,
            Tolerance::default().linear,
        )
        .unwrap()
    }

    /// Split-level differential: same region count, same total area, and
    /// matching region centroids (each side within tolerance). Areas are
    /// measured through dense carrier sampling in the shared split frame,
    /// so analytic arcs compare by their true geometry, not by chorded
    /// polygons.
    fn assert_same_partition(
        label: &str,
        new_faces: &[SplitSubFace],
        old_faces: &[SplitSubFace],
        frame: &PlaneFrame,
        area_tol: f64,
    ) {
        assert_eq!(
            new_faces.len(),
            old_faces.len(),
            "{label}: region count new={} old={}",
            new_faces.len(),
            old_faces.len()
        );
        let total = |faces: &[SplitSubFace]| {
            faces
                .iter()
                .map(|s| sampled_wire_moments(&s.outer_wire, frame).0.abs())
                .sum::<f64>()
        };
        let (new_total, old_total) = (total(new_faces), total(old_faces));
        assert!(
            (new_total - old_total).abs() <= area_tol,
            "{label}: area new={new_total} old={old_total}"
        );
        // Dense area centroids converge regardless of how each side
        // segments its arcs (cardinal splits, midpoint lens cuts).
        let mut unmatched: Vec<Point2> = old_faces
            .iter()
            .map(|s| sampled_wire_moments(&s.outer_wire, frame).1)
            .collect();
        for sub in new_faces {
            let center = sampled_wire_moments(&sub.outer_wire, frame).1;
            let hit = unmatched
                .iter()
                .position(|o| (*o - center).length() <= 1e-4);
            assert!(hit.is_some(), "{label}: unmatched new region at {center:?}");
            unmatched.remove(hit.unwrap());
        }
    }

    /// True UV area and area centroid of a wire by dense carrier sampling
    /// (4096 per arc) in the shared split frame. Lines contribute
    /// endpoints; circles are walked in traversal order through their
    /// carried trim.
    fn sampled_wire_moments(wire: &[OrientedPCurveEdge], frame: &PlaneFrame) -> (f64, Point2) {
        const SAMPLES: usize = 4096;
        let mut points: Vec<Point2> = Vec::new();
        for edge in wire {
            match &edge.curve_3d {
                EdgeCurve::Line => {
                    if points.is_empty() {
                        points.push(edge.start_uv);
                    }
                    points.push(edge.end_uv);
                }
                EdgeCurve::Circle(circle) => {
                    let Some((t0, t1)) = edge.trim else {
                        if points.is_empty() {
                            points.push(edge.start_uv);
                        }
                        points.push(edge.end_uv);
                        continue;
                    };
                    // Traversal runs start to end: pick the trim direction
                    // whose carrier endpoints reproduce them.
                    let direct = (circle.evaluate(t0) - edge.start_3d).length()
                        + (circle.evaluate(t1) - edge.end_3d).length();
                    let reversed = (circle.evaluate(t0) - edge.end_3d).length()
                        + (circle.evaluate(t1) - edge.start_3d).length();
                    let (from, to) = if direct <= reversed {
                        (t0, t1)
                    } else {
                        (t1, t0)
                    };
                    for k in 0..=SAMPLES {
                        #[allow(clippy::cast_precision_loss)]
                        let f = k as f64 / SAMPLES as f64;
                        let p3 = circle.evaluate(from + (to - from) * f);
                        points.push(frame.project(p3));
                    }
                }
                _ => {
                    if points.is_empty() {
                        points.push(edge.start_uv);
                    }
                    points.push(edge.end_uv);
                }
            }
        }
        let mut area = 0.0;
        let mut centroid = Point2::new(0.0, 0.0);
        for pair in points.windows(2) {
            let cross = pair[0].x() * pair[1].y() - pair[1].x() * pair[0].y();
            area += cross;
            centroid = Point2::new(
                centroid.x() + (pair[0].x() + pair[1].x()) * cross,
                centroid.y() + (pair[0].y() + pair[1].y()) * cross,
            );
        }
        area *= 0.5;
        if area.abs() > f64::EPSILON {
            centroid = Point2::new(centroid.x() / (6.0 * area), centroid.y() / (6.0 * area));
        }
        (area, centroid)
    }

    #[test]
    fn differential_x_matches_crossing_helper() {
        let (topo, face) = square_topology(2.0);
        let frame = plane_frame();
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let new_faces = split_emit(&topo, face, &sections).expect("adapter X");
        let old_faces = established_crossing(&topo, face, &sections, &frame).expect("helper X");
        assert_same_partition("X", &new_faces, &old_faces, &frame, 1e-9);
    }

    #[test]
    fn differential_t_matches_crossing_helper() {
        let (topo, face) = square_topology(2.0);
        let frame = plane_frame();
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let new_faces = split_emit(&topo, face, &sections).expect("adapter T");
        let old_faces = established_crossing(&topo, face, &sections, &frame).expect("helper T");
        assert_same_partition("T", &new_faces, &old_faces, &frame, 1e-9);
    }

    #[test]
    fn differential_diameter_matches_disk_helper() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let eid = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let frame = plane_frame();
        let sections = [line_section(
            Point3::new(-2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        )];
        let new_faces = split_emit(&topo, face, &sections).expect("adapter diameter");
        let old_faces = established_disk(&topo, face, &sections, &frame).expect("helper diameter");
        assert_same_partition("diameter", &new_faces, &old_faces, &frame, 1e-6);
    }

    fn partition_signature(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
        frame: &PlaneFrame,
    ) -> (usize, f64) {
        let inputs = collect_planar_uses(
            topo,
            face,
            sections,
            Rank::A,
            frame,
            &Tolerance::default(),
            &test_context(),
        )
        .unwrap()
        .expect("qualified");
        let arrangement = run(&inputs);
        let count = arrangement.regions.iter().filter(|r| r.material).count();
        let area: f64 = arrangement
            .regions
            .iter()
            .filter(|r| r.material)
            .map(|r| r.area)
            .sum();
        (count, area)
    }

    #[test]
    fn section_permutations_and_reversals_agree() {
        let (topo, face) = square_topology(2.0);
        let frame = plane_frame();
        let a = line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0));
        let b = line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0));
        let rev = |s: &SectionEdge| {
            let mut r = s.clone();
            std::mem::swap(&mut r.start, &mut r.end);
            r
        };
        let variants = [
            vec![a.clone(), b.clone()],
            vec![b.clone(), a.clone()],
            vec![rev(&a), b.clone()],
            vec![a.clone(), rev(&b)],
            vec![rev(&b), rev(&a)],
        ];
        for sections in &variants {
            let (count, area) = partition_signature(&topo, face, sections, &frame);
            assert_eq!(count, 4);
            assert!((area - 4.0).abs() < 1e-9, "area {area}");
        }
    }

    #[test]
    fn rigid_placement_keeps_partition() {
        use remus_math::mat::Mat4;
        // Rotate 37 degrees about z and translate off-origin, including
        // out of the z = 0 plane frame origin used elsewhere.
        let rotation =
            Mat4::rotation_z(0.645_771_823_237_901_9) * Mat4::translation(7.5, -3.25, 10.0);
        let map = |p: Point3| rotation.mul_point(p);
        let corners = [
            map(Point3::new(0.0, 0.0, 0.0)),
            map(Point3::new(2.0, 0.0, 0.0)),
            map(Point3::new(2.0, 2.0, 0.0)),
            map(Point3::new(0.0, 2.0, 0.0)),
        ];
        let mut topo = Topology::new();
        let mut vids = Vec::new();
        for corner in corners {
            vids.push(topo.add_vertex(Vertex::new(corner, TOL)));
        }
        let mut edges = Vec::new();
        for i in 0..4 {
            edges.push(topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line)));
        }
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane { normal, d: 10.0 },
        ));
        let wire_pts =
            super::super::collect_wire_points(&topo, topo.face(face).unwrap().outer_wire());
        let frame = PlaneFrame::from_plane_face(normal, &wire_pts);
        let sections = [
            line_section(
                map(Point3::new(1.0, 0.0, 0.0)),
                map(Point3::new(1.0, 2.0, 0.0)),
            ),
            line_section(
                map(Point3::new(0.0, 1.0, 0.0)),
                map(Point3::new(2.0, 1.0, 0.0)),
            ),
        ];
        let (count, area) = partition_signature(&topo, face, &sections, &frame);
        assert_eq!(count, 4);
        assert!((area - 4.0).abs() < 1e-9, "area {area}");
    }

    #[test]
    fn scale_keeps_partition_topology() {
        for scale in [1e-3, 1e3] {
            let (mut topo, _) = (Topology::new(), None::<FaceId>);
            let corners = [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0 * scale, 0.0, 0.0),
                Point3::new(2.0 * scale, 2.0 * scale, 0.0),
                Point3::new(0.0, 2.0 * scale, 0.0),
            ];
            let mut vids = Vec::new();
            for corner in corners {
                vids.push(topo.add_vertex(Vertex::new(corner, TOL)));
            }
            let mut edges = Vec::new();
            for i in 0..4 {
                edges.push(topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line)));
            }
            let wire = topo.add_wire(
                Wire::new(
                    edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                    true,
                )
                .unwrap(),
            );
            let face = topo.add_face(Face::new(
                wire,
                Vec::new(),
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            ));
            let sections = [
                line_section(
                    Point3::new(1.0 * scale, 0.0, 0.0),
                    Point3::new(1.0 * scale, 2.0 * scale, 0.0),
                ),
                line_section(
                    Point3::new(0.0, 1.0 * scale, 0.0),
                    Point3::new(2.0 * scale, 1.0 * scale, 0.0),
                ),
            ];
            let (count, area) = partition_signature(&topo, face, &sections, &plane_frame());
            assert_eq!(count, 4, "scale {scale}");
            assert!(
                (area - 4.0 * scale * scale).abs() < 1e-6 * scale * scale,
                "scale {scale} area {area}"
            );
        }
    }

    #[test]
    fn thin_retained_strip_survives() {
        let (topo, face) = square_topology(2.0);
        let frame = plane_frame();
        // Two full-width sections a micron apart: the strip between them
        // is retained whole, not welded away.
        let sections = [
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
            line_section(
                Point3::new(0.0, 1.0 + 1e-6, 0.0),
                Point3::new(2.0, 1.0 + 1e-6, 0.0),
            ),
        ];
        let (count, area) = partition_signature(&topo, face, &sections, &frame);
        assert_eq!(count, 3);
        assert!((area - 4.0).abs() < 1e-9, "area {area}");
        let inputs = collect(&topo, face, &sections).expect("qualified thin");
        let arrangement = run(&inputs);
        let mut areas: Vec<f64> = arrangement
            .regions
            .iter()
            .filter(|r| r.material)
            .map(|r| r.area)
            .collect();
        areas.sort_by(f64::total_cmp);
        assert!((areas[0] - 2e-6).abs() < 2e-6 * 0.01 + 1e-12, "{areas:?}");
    }

    #[test]
    fn reversed_wire_traversal_agrees() {
        // Same square traversed clockwise instead of counter-clockwise.
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        let mut vids = Vec::new();
        for corner in corners {
            vids.push(topo.add_vertex(Vertex::new(corner, TOL)));
        }
        let mut edges = Vec::new();
        for i in 0..4 {
            edges.push(topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line)));
        }
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let frame = plane_frame();
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let (count, area) = partition_signature(&topo, face, &sections, &frame);
        assert_eq!(count, 4);
        assert!((area - 4.0).abs() < 1e-9, "area {area}");
    }

    #[test]
    fn three_section_star_extends_beyond_helper() {
        // Three lines through the center at 60 degrees: the established
        // crossing helper handles two or four sections only and defers
        // (None). The arrangement partitions all six slices; each is
        // verified against its closed-form area.
        use std::f64::consts::PI;
        let (topo, face) = square_topology(2.0);
        let frame = plane_frame();
        let center = Point3::new(1.0, 1.0, 0.0);
        let mut sections = Vec::new();
        for k in 0..3 {
            let angle = k as f64 * PI / 3.0;
            let dir = Point3::new(angle.cos(), angle.sin(), 0.0);
            sections.push(line_section(
                Point3::new(center.x() - dir.x() * 2.0, center.y() - dir.y() * 2.0, 0.0),
                Point3::new(center.x() + dir.x() * 2.0, center.y() + dir.y() * 2.0, 0.0),
            ));
        }
        // Clip to the square like production does: keep the in-square span.
        let clipped: Vec<SectionEdge> = sections
            .iter()
            .map(|s| {
                let clip =
                    |p: Point3| Point3::new(p.x().clamp(0.0, 2.0), p.y().clamp(0.0, 2.0), 0.0);
                line_section(clip(s.start), clip(s.end))
            })
            .collect();
        let helper = established_crossing(&topo, face, &clipped, &frame);
        assert!(helper.is_none(), "helper owns two/four-section stars only");
        let (count, area) = partition_signature(&topo, face, &clipped, &frame);
        assert_eq!(count, 6);
        assert!((area - 4.0).abs() < 1e-9, "area {area}");
    }

    fn split_emit(
        topo: &Topology,
        face: FaceId,
        sections: &[SectionEdge],
    ) -> Option<Vec<SplitSubFace>> {
        try_split_plane_face_by_provenance_arrangement(
            topo,
            face,
            sections,
            Rank::A,
            &plane_frame(),
            &Tolerance::default(),
            &test_context(),
            None,
        )
        .unwrap()
    }

    /// Every wire closes head-to-tail within the operation tolerance.
    fn assert_closed(wire: &[OrientedPCurveEdge]) {
        assert!(!wire.is_empty());
        for pair in wire.windows(2) {
            assert!(
                (pair[0].end_3d - pair[1].start_3d).length() <= 1e-7,
                "wire gap {:?} -> {:?}",
                pair[0].end_3d,
                pair[1].start_3d
            );
        }
        let (first, last) = (&wire[0], &wire[wire.len() - 1]);
        assert!((last.end_3d - first.start_3d).length() <= 1e-7);
    }

    fn uv_polygon_area(wire: &[OrientedPCurveEdge]) -> f64 {
        let mut area = 0.0;
        for edge in wire {
            area += edge.start_uv.x() * edge.end_uv.y() - edge.end_uv.x() * edge.start_uv.y();
        }
        area * 0.5
    }

    fn point_in_uv_polygon(point: Point2, wire: &[OrientedPCurveEdge]) -> bool {
        let mut inside = false;
        for edge in wire {
            let (a, b) = (edge.start_uv, edge.end_uv);
            if (a.y() > point.y()) != (b.y() > point.y()) {
                let x = a.x() + (point.y() - a.y()) / (b.y() - a.y()) * (b.x() - a.x());
                if x > point.x() {
                    inside = !inside;
                }
            }
        }
        inside
    }

    #[test]
    fn emit_crossing_lines_produces_four_classified_subfaces() {
        let (topo, face) = square_topology(2.0);
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(0.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let subfaces = split_emit(&topo, face, &sections).expect("qualified planar X");
        assert_eq!(subfaces.len(), 4);
        let mut total = 0.0;
        for sub in &subfaces {
            assert_closed(&sub.outer_wire);
            assert!(sub.inner_wires.is_empty());
            assert_eq!(sub.parent, face);
            assert_eq!(sub.rank, Rank::A);
            assert!(!sub.reversed);
            assert!(matches!(sub.surface, FaceSurface::Plane { .. }));
            total += uv_polygon_area(&sub.outer_wire).abs();
            // Split section pieces drop the pave block like the
            // established splitter's split pieces do.
            for edge in &sub.outer_wire {
                if matches!(edge.curve_3d, EdgeCurve::Line) && edge.source_topo_edge.is_none() {
                    assert!(
                        edge.pave_block_id.is_none(),
                        "split section piece must not keep its pave block"
                    );
                }
            }
            // The certified interior classifies inside its own region.
            let interior = sub.precomputed_interior.expect("certified seed");
            assert!((interior.z()).abs() < 1e-12);
            let uv = plane_frame().project(interior);
            assert!(point_in_uv_polygon(uv, &sub.outer_wire));
        }
        assert!((total - 4.0).abs() < 1e-9, "total {total}");
    }

    #[test]
    fn emit_t_junction_keeps_shared_section_provenance() {
        let (topo, face) = square_topology(2.0);
        let sections = [
            line_section(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0)),
            line_section(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
        ];
        let subfaces = split_emit(&topo, face, &sections).expect("qualified planar T");
        assert_eq!(subfaces.len(), 3);
        // The shared T foot (1, 1) appears as a wire vertex on every
        // region touching it.
        let mut foot_uses = 0;
        for sub in &subfaces {
            assert_closed(&sub.outer_wire);
            for edge in &sub.outer_wire {
                for end in [edge.start_3d, edge.end_3d] {
                    if (end - Point3::new(1.0, 1.0, 0.0)).length() < 1e-9 {
                        foot_uses += 1;
                    }
                }
            }
        }
        assert!(foot_uses >= 6, "T foot shared, got {foot_uses}");
    }

    #[test]
    fn emit_disc_chord_keeps_analytic_trims() {
        let mut topo = Topology::new();
        let center = Point3::new(0.0, 0.0, 0.0);
        let circle = Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let eid = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let root = 3.0_f64.sqrt();
        let sections = [line_section(
            Point3::new(-root, 1.0, 0.0),
            Point3::new(root, 1.0, 0.0),
        )];
        let subfaces = split_emit(&topo, face, &sections).expect("qualified disc");
        assert_eq!(subfaces.len(), 2);
        // Every circle sub-edge carries an exact native subspan; the
        // boundary spans rejoin to the full turn (major arc preserved).
        let mut covered = 0.0;
        for sub in &subfaces {
            assert_closed(&sub.outer_wire);
            for edge in &sub.outer_wire {
                if matches!(edge.curve_3d, EdgeCurve::Circle(_)) {
                    let (t0, t1) = edge.trim.expect("circle sub-edge trim");
                    assert!(t0.is_finite() && t1.is_finite() && (t1 - t0).abs() > 1e-12);
                    if edge.source_topo_edge.is_some() {
                        covered += (t1 - t0).abs();
                    }
                }
            }
        }
        assert!((covered - TAU).abs() < 1e-9, "covered {covered}");
        // Both regions keep analytic circle edges (never chorded lines).
        for sub in &subfaces {
            assert!(
                sub.outer_wire
                    .iter()
                    .any(|e| matches!(e.curve_3d, EdgeCurve::Circle(_))),
                "disc region lost its arc"
            );
        }
    }

    #[test]
    fn emit_holed_face_carries_hole_and_interior() {
        let mut topo = Topology::new();
        let mut wire_of = |points: &[Point3; 4]| {
            let mut vids = Vec::new();
            for point in points {
                vids.push(topo.add_vertex(Vertex::new(*point, TOL)));
            }
            let mut edges = Vec::new();
            for i in 0..4 {
                edges.push(topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line)));
            }
            topo.add_wire(
                Wire::new(
                    edges.iter().map(|e| OrientedEdge::new(*e, true)).collect(),
                    true,
                )
                .unwrap(),
            )
        };
        let outer = wire_of(&[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        ]);
        let hole = wire_of(&[
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
            Point3::new(3.0, 3.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
        ]);
        let face = topo.add_face(Face::new(
            outer,
            vec![hole],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let subfaces = split_emit(&topo, face, &[]).expect("qualified holed face");
        assert_eq!(subfaces.len(), 1);
        let sub = &subfaces[0];
        assert_closed(&sub.outer_wire);
        assert_eq!(sub.inner_wires.len(), 1);
        assert_closed(&sub.inner_wires[0]);
        let interior = sub.precomputed_interior.expect("certified seed");
        let uv = plane_frame().project(interior);
        assert!(point_in_uv_polygon(uv, &sub.outer_wire));
        assert!(!point_in_uv_polygon(uv, &sub.inner_wires[0]));
    }

    #[test]
    fn emission_is_atomic_on_internal_refusal() {
        // The tangent ring passes qualification but the core refuses it
        // at run level; the try_ entry maps recorded-unsupported contacts
        // back to out-of-domain, so emission never runs, no partial
        // subface escapes, and the input topology is untouched.
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let v = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
        let mut edge = Edge::new(v, v, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        let eid = topo.add_edge(edge);
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(eid, true)], true).unwrap());
        let face = topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let inner =
            Circle3D::new(Point3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let proto = line_section(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
        let sections = [SectionEdge {
            curve_3d: EdgeCurve::Circle(inner),
            trim: Some((0.0, TAU)),
            pcurve_a: proto.pcurve_a.clone(),
            pcurve_b: proto.pcurve_b,
            start: Point3::new(2.0, 0.0, 0.0),
            end: Point3::new(2.0, 0.0, 0.0),
            start_uv_a: None,
            end_uv_a: None,
            start_uv_b: None,
            end_uv_b: None,
            target_face: None,
            pave_block_id: Some(11),
        }];
        let before = topo.num_faces();
        let deferred = try_split_plane_face_by_provenance_arrangement(
            &topo,
            face,
            &sections,
            Rank::A,
            &plane_frame(),
            &Tolerance::default(),
            &test_context(),
            None,
        )
        .unwrap();
        assert!(
            deferred.is_none(),
            "recorded-unsupported tangency defers to the established path"
        );
        assert_eq!(topo.num_faces(), before, "no topology allocated on refusal");
    }

    fn line_use(pcurve: Line2D, range: [f64; 2], id: u64) -> CurveUse {
        CurveUse {
            source: CurveSource {
                use_id: id,
                boundary: None,
                section: None,
                source_edge_idx: None,
                pave_block_id: None,
            },
            pcurve: Curve2D::Line(pcurve),
            range,
            curve_3d: EdgeCurve::Line,
            source_range: range,
            endpoints_3d: [Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)],
            endpoints: [id * 2, id * 2 + 1],
            boundary_loop: None,
        }
    }

    fn circle_use(pcurve: Circle2D, range: [f64; 2], id: u64) -> CurveUse {
        let mut use_data = line_use(
            Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
            [0.0, 1.0],
            id,
        );
        use_data.pcurve = Curve2D::Circle(pcurve);
        use_data.range = range;
        use_data
    }

    fn uv_line(ox: f64, oy: f64, dx: f64, dy: f64) -> Line2D {
        Line2D::new(Point2::new(ox, oy), Vec2::new(dx, dy)).unwrap()
    }

    fn uv_circle(cx: f64, cy: f64, r: f64) -> Circle2D {
        Circle2D::new(Point2::new(cx, cy), r).unwrap()
    }

    #[test]
    fn tangent_line_and_circle_decline() {
        let line = line_use(uv_line(0.0, 0.0, 1.0, 0.0), [-10.0, 10.0], 0);
        let circle = circle_use(uv_circle(0.0, 5.0, 5.0), [0.0, TAU], 1);
        assert!(uses_have_degenerate_contact(&[line, circle], TOL));
    }

    #[test]
    fn secant_line_and_circle_proceed() {
        let line = line_use(uv_line(0.0, 0.0, 1.0, 0.0), [-10.0, 10.0], 0);
        let circle = circle_use(uv_circle(0.0, 1.0, 5.0), [0.0, TAU], 1);
        assert!(!uses_have_degenerate_contact(&[line, circle], TOL));
    }

    #[test]
    fn endpoint_touch_does_not_decline() {
        // Tangent contact at the line's endpoint is a certified joint.
        let line = line_use(uv_line(0.0, 0.0, 1.0, 0.0), [0.0, 10.0], 0);
        let circle = circle_use(uv_circle(0.0, 5.0, 5.0), [0.0, TAU], 1);
        assert!(!uses_have_degenerate_contact(&[line, circle], TOL));
    }

    #[test]
    fn grazing_line_and_circle_proceed_to_core() {
        // A 0.001 dust-scale graze is not exact tangency: the classifier
        // stays out and the core's refinement refusal declines instead.
        let line = line_use(uv_line(-40.0, 0.0, 1.0, 0.0), [0.0, 40.0], 0);
        let circle = circle_use(uv_circle(-20.0, -9.999, 10.0), [0.0, TAU], 1);
        assert!(!uses_have_degenerate_contact(&[line, circle], TOL));
    }

    #[test]
    fn coincident_lines_decline_but_parallel_distinct_proceed() {
        let a = || line_use(uv_line(0.0, 0.0, 1.0, 0.0), [0.0, 10.0], 0);
        let b = line_use(uv_line(0.0, 0.0, 1.0, 0.0), [5.0, 15.0], 1);
        assert!(uses_have_degenerate_contact(&[a(), b], TOL));
        let c = line_use(uv_line(0.0, 1.0, 1.0, 0.0), [0.0, 10.0], 2);
        assert!(!uses_have_degenerate_contact(&[a(), c], TOL));
        // Collinear endpoint touch is a joint, not an overlap.
        let d = line_use(uv_line(0.0, 0.0, 1.0, 0.0), [10.0, 20.0], 3);
        assert!(!uses_have_degenerate_contact(&[a(), d], TOL));
    }

    #[test]
    fn tangent_circles_decline_but_secant_proceed() {
        let a = || circle_use(uv_circle(0.0, 0.0, 5.0), [0.0, TAU], 0);
        let external = circle_use(uv_circle(10.0, 0.0, 5.0), [0.0, TAU], 1);
        assert!(uses_have_degenerate_contact(&[a(), external], TOL));
        let secant = circle_use(uv_circle(8.0, 0.0, 5.0), [0.0, TAU], 2);
        assert!(!uses_have_degenerate_contact(&[a(), secant], TOL));
        // Internal tangency: r=10 about origin, r=5 about (5, 0).
        let big = circle_use(uv_circle(0.0, 0.0, 10.0), [0.0, TAU], 3);
        let small = circle_use(uv_circle(5.0, 0.0, 5.0), [0.0, TAU], 4);
        assert!(uses_have_degenerate_contact(&[big, small], TOL));
        // Concentric distinct radii never touch.
        let concentric = circle_use(uv_circle(0.0, 0.0, 3.0), [0.0, TAU], 5);
        assert!(!uses_have_degenerate_contact(&[a(), concentric], TOL));
    }
}
