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

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use remus_math::context::OperationContext;
use remus_math::curves2d::{Circle2D, Curve2D, Line2D};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::face_loop::LoopId;

use super::super::plane_frame::PlaneFrame;
use super::super::split_types::{OrientedPCurveEdge, SectionEdge, SplitSubFace};
use super::arrangement::{
    Arrangement, ArrangementError, ArrangementHalfEdge, ArrangementInput, BoundarySource,
    CurveSource, CurveUse, ParamDomain, build_arrangement,
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
    collector.prejoin_endpoints(context)?;
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
/// established path). A core refusal on qualified input is an internal
/// error and propagates. Construction is atomic: collection, arrangement,
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
    // The pipeline's budgets are SSI-calibrated (a handful of steps);
    // the arrangement is a different cost model with quadratic pair
    // scans. The adapter translates with an input-proportional policy
    // floor, keeping the caller's cancellation token and every larger
    // allowance: budgets stay finite and exhaustion stays typed, while
    // default contexts can actually run the core. Measured: 10 uses
    // consume ~700 march steps; the floor carries 8x headroom.
    let child = arrangement_context(context, inputs.uses.len());
    match run_planar_arrangement(&inputs, &child) {
        Err(ArrangementError::AmbiguousContact | ArrangementError::AmbiguousOverlap) => {
            // Recorded-unsupported contact classes (M1 contract): tangent
            // contacts and coincident overlaps are out-of-domain, so the
            // caller runs the established path. Every other refusal is an
            // internal error on claimed input and propagates. A genuine
            // adapter bug cannot hide here: only exact geometric tangency
            // or collinearity produces these errors, never a macroscopic
            // correspondence failure (those refuse as refinement, open
            // region, or non-manifold errors instead).
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
            let subfaces = emit_planar_subfaces(
                &inputs,
                &arrangement,
                surface,
                reversed,
                face_id,
                rank,
                frame,
                context,
            )?;
            Ok(Some(subfaces))
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
fn emit_cycle_wire(
    inputs: &PlanarInputs,
    arrangement: &Arrangement,
    carriers: &BTreeMap<u64, &CurveUse>,
    cycle: &[usize],
    context: &OperationContext,
) -> Result<Vec<OrientedPCurveEdge>, AlgoError> {
    let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
    let mut wire = Vec::with_capacity(cycle.len());
    for &half_idx in cycle {
        context.check_cancelled().map_err(cancelled)?;
        let half = &arrangement.half_edges[half_idx];
        wire.push(emit_wire_edge(
            inputs,
            arrangement,
            carriers,
            half,
            context,
        )?);
    }
    Ok(wire)
}

/// Build one production wire edge from an arrangement half-edge.
///
/// Geometry comes from the half-edge's exact endpoints (3D) and chart
/// vertices (UV); the carrier and lineage come from the originating
/// input use. Circle sub-edges carry their exact native subspan with
/// the traversal-oriented forward flag; lines carry no trim, matching
/// production convention.
fn emit_wire_edge(
    inputs: &PlanarInputs,
    arrangement: &Arrangement,
    carriers: &BTreeMap<u64, &CurveUse>,
    half: &ArrangementHalfEdge,
    context: &OperationContext,
) -> Result<OrientedPCurveEdge, AlgoError> {
    let internal = |detail: &str| {
        AlgoError::FaceSplitFailed(format!("provenance arrangement emission failed: {detail}"))
    };
    let tolerance = context.tolerance.linear;
    let position = half
        .source
        .source_edge_idx
        .ok_or_else(|| internal("missing source index"))?;
    let parent = inputs
        .parents
        .get(position)
        .ok_or_else(|| internal("stale source index"))?;
    let carrier = carriers
        .get(&half.source.use_id)
        .ok_or_else(|| internal("stale use id"))?;
    let start_uv = arrangement.vertices[half.from].uv;
    let end_uv = arrangement.vertices[half.to].uv;
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
            let source_range = half.source_range;
            if !source_range[0].is_finite() || !source_range[1].is_finite() {
                return Err(internal("non-finite circle subspan"));
            }
            (
                parent.pcurve.clone(),
                Some((source_range[0], source_range[1])),
                source_range[1] >= source_range[0],
            )
        }
        _ => return Err(internal("non-line/circle carrier")),
    };
    // Pave-block sharing only for full-span sections: subspans drop it
    // exactly like the established splitter's split pieces, so vertex
    // resolution never snaps a piece to its parent's unsplit endpoints.
    // Untouched uses pass through bitwise-identical, so exact comparison
    // is the correct untouched detector (not a tolerance comparison).
    let full_span = same_interval(carrier.range, half.range)
        && same_interval(carrier.source_range, half.source_range);
    let pave_block_id = if full_span {
        half.source.pave_block_id
    } else {
        None
    };
    // The carried trim must reproduce the wire endpoints through the
    // carrier. This duplicates the downstream preflight at emission
    // time so a pairing bug fails here with provenance attached,
    // never as a silent weld or a far-away topology error.
    if let (EdgeCurve::Circle(circle), Some((t0, t1))) = (&carrier.curve_3d, trim) {
        let start_ok = (circle.evaluate(t0) - half.endpoints_3d[0]).length() <= tolerance
            && (circle.evaluate(t1) - half.endpoints_3d[1]).length() <= tolerance;
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
        start_3d: half.endpoints_3d[0],
        end_3d: half.endpoints_3d[1],
        forward,
        source_edge_idx: half.source.source_edge_idx,
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

    /// Split uses at section endpoints landing on another use's interior.
    ///
    /// The core refines exact pair crossings but never snaps: an endpoint
    /// that misses its support by clip noise would dangle and refuse. The
    /// join is therefore established here with a shared certificate and
    /// exact subspans; the core re-derives the same event and verifies the
    /// residual.
    fn prejoin_endpoints(&mut self, context: &OperationContext) -> Result<(), AlgoError> {
        let cancelled = |_| AlgoError::FaceSplitFailed("planar arrangement cancelled".into());
        // Section endpoints in deterministic order.
        let mut endpoints: Vec<(usize, usize)> = Vec::new();
        for (idx, _) in self.uses.iter().enumerate() {
            if self.parents[idx].is_section {
                endpoints.push((idx, 0));
                endpoints.push((idx, 1));
            }
        }
        // (target use, parameter, shared cert).
        let mut cuts: BTreeMap<usize, Vec<(f64, u64)>> = BTreeMap::new();
        for (idx, side) in endpoints {
            context.check_cancelled().map_err(cancelled)?;
            let cert = self.uses[idx].endpoints[side];
            // An endpoint sharing its certificate with any other use end
            // is already joined — certificates are unique per event, so
            // equality alone is the certificate (positions are verified
            // by the core, not re-welded here).
            let joined = self
                .uses
                .iter()
                .enumerate()
                .any(|(other_idx, other)| other_idx != idx && other.endpoints.contains(&cert));
            if joined {
                continue;
            }
            let uv = eval_pcurve(&self.uses[idx], self.uses[idx].range[side]);
            for (other_idx, other) in self.uses.iter().enumerate() {
                if other_idx == idx {
                    continue;
                }
                if let Some(parameter) = interior_parameter(other, uv, self.tol)
                    && !near_use_end(other, parameter, uv)
                {
                    cuts.entry(other_idx).or_default().push((parameter, cert));
                }
            }
        }
        let targets: Vec<usize> = cuts.keys().copied().collect();
        // Splits insert uses, shifting later targets: track the
        // cumulative offset so every cut lands on its original target.
        let mut shift = 0usize;
        for target in targets {
            let mut list = cuts.remove(&target).unwrap_or_default();
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            list.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-12);
            self.split_use_at(target + shift, &list)?;
            shift += list.len();
        }
        Ok(())
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
    fn split_use_at(&mut self, target: usize, cuts: &[(f64, u64)]) -> Result<(), AlgoError> {
        if cuts.is_empty() {
            return Ok(());
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
                return Err(collect_failed(ArrangementError::OpenRegion));
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
        Ok(())
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
    use remus_math::vec::Vec3;
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
        )
        .unwrap();
        assert!(
            deferred.is_none(),
            "recorded-unsupported tangency defers to the established path"
        );
        assert_eq!(topo.num_faces(), before, "no topology allocated on refusal");
    }
}
