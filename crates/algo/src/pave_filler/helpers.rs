//! Shared helper functions for PaveFiller phases.
//!
//! Extracted from phase_ee, phase_ef, and phase_ve to eliminate
//! duplicated vertex-lookup and pave-insertion logic.

use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeId};
use remus_topology::vertex::VertexId;

use crate::ds::{GfaArena, Pave};
use crate::error::AlgoError;

/// Resolve the stored parameter authority for a topology edge.
///
/// PaveFiller must never reconstruct a curved edge's branch from its endpoint
/// positions: periodic seams and major/reversed spans are not recoverable from
/// those points alone. Lines retain their intrinsic endpoint-local `[0, 1]`
/// domain through [`Edge::strict_domain`].
pub(super) fn authoritative_edge_domain(
    edge: &Edge,
    edge_id: EdgeId,
    stage: &'static str,
) -> Result<(f64, f64), AlgoError> {
    edge.strict_domain().map_err(|error| {
        AlgoError::IntersectionFailed(format!(
            "{stage} edge {edge_id:?} lacks authoritative parameter range: {error}"
        ))
    })
}

/// Validate a complete edge set before a phase starts mutating its arena.
pub(super) fn validate_edge_domains(
    topo: &Topology,
    edges: &[EdgeId],
    stage: &'static str,
) -> Result<(), AlgoError> {
    for &edge_id in edges {
        let edge = topo.edge(edge_id)?;
        authoritative_edge_domain(edge, edge_id, stage)?;
    }
    Ok(())
}

/// Return the part of a vertex ball that widens an operation beyond its
/// global linear floor.
pub(super) fn vertex_tolerance_excess(
    topo: &Topology,
    vertex_id: VertexId,
    floor: f64,
) -> Result<f64, AlgoError> {
    let value = topo.vertex(vertex_id)?.tolerance();
    tolerance_excess(value, floor, "vertex")
}

/// Return the part of an edge tube that widens an operation beyond its
/// global linear floor.
pub(super) fn edge_tolerance_excess(
    topo: &Topology,
    edge_id: EdgeId,
    floor: f64,
) -> Result<f64, AlgoError> {
    let edge = topo.edge(edge_id)?;
    let start_tolerance = topo.vertex(edge.start())?.tolerance();
    let end_tolerance = topo.vertex(edge.end())?.tolerance();
    tolerance_excess(start_tolerance, floor, "vertex")?;
    tolerance_excess(end_tolerance, floor, "vertex")?;
    let vertex_tolerance = start_tolerance.max(end_tolerance);
    tolerance_excess(edge.effective_tolerance(vertex_tolerance), floor, "edge")
}

/// Add tolerance contributions and reject a non-finite acceptance band.
pub(super) fn tolerance_band(
    floor: f64,
    contributions: impl IntoIterator<Item = f64>,
) -> Result<f64, AlgoError> {
    let mut band = floor;
    for contribution in contributions {
        band += contribution;
    }
    if !band.is_finite() || band.is_sign_negative() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue {
            entity: "predicate band",
            value: band,
        }
        .into());
    }
    Ok(band)
}

fn tolerance_excess(value: f64, floor: f64, entity: &'static str) -> Result<f64, AlgoError> {
    if !value.is_finite() || value.is_sign_negative() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue { entity, value }.into());
    }
    Ok((value - floor).max(0.0))
}

/// Find a vertex near the given point among all pave block vertices.
///
/// Returns the resolved (same-domain canonical) vertex first encountered within
/// the operation floor or the candidate vertex's wider tolerance ball,
/// scanning pave blocks in `edge_pave_blocks` order
/// (ascending `EdgeId`, start-before-end). When the arena's spatial index is
/// available (built after Phase VV) the lookup is O(1) and returns the exact
/// same vertex; otherwise it falls back to the linear scan.
pub(super) fn find_nearby_pave_vertex(
    topo: &Topology,
    arena: &GfaArena,
    point: Point3,
    tol: Tolerance,
) -> Option<VertexId> {
    if let Some(index) = &arena.pave_vertex_index {
        return index.find_with_entry_radius(point);
    }
    for pbs in arena.edge_pave_blocks.values() {
        for &pb_id in pbs {
            if let Some(pb) = arena.pave_blocks.get(pb_id) {
                for vid in [pb.start.vertex, pb.end.vertex] {
                    crate::perf::bump_pave_vertex_probe();
                    let resolved = arena.resolve_vertex(vid);
                    if let Ok(v) = topo.vertex(resolved)
                        && (v.point() - point).length() <= v.tolerance().max(tol.linear)
                    {
                        return Some(resolved);
                    }
                }
            }
        }
    }
    None
}

/// Widened variant of [`find_nearby_pave_vertex`] for tangential contacts.
///
/// A grazing crossing's solved position is only accurate to
/// `sqrt(2 * r * residual)`, so the exact junction vertex can sit microns
/// outside the linear tolerance. This scans every pave-block endpoint within
/// `radius` and returns the nearest candidate that passes `accept` (the
/// caller checks genuine curve/surface incidence, which is what makes the
/// widened radius safe). If the accepted candidates span more than one
/// distinct position (beyond `tol_linear` of each other), the contact is
/// ambiguous — two different junctions inside the window — and `None` is
/// returned so the caller keeps the solved point rather than merging
/// distinct junctions. The spatial index uses cells at least as large as the
/// maximum widened radius, keeping this lookup bounded to a 3x3x3 stencil.
pub(super) fn find_nearby_pave_vertex_widened(
    arena: &GfaArena,
    point: Point3,
    radius: f64,
    tol_linear: f64,
    accept: impl Fn(Point3) -> bool,
) -> Option<VertexId> {
    arena
        .pave_vertex_index
        .as_ref()?
        .find_unambiguous_within(point, radius, tol_linear, accept)
}

/// Add a pave to the appropriate pave block of an edge.
///
/// Finds the pave block whose parameter range contains the pave's
/// parameter (with a small guard band) and adds the extra pave to it.
pub(super) fn add_pave_to_edge(arena: &mut GfaArena, edge_id: EdgeId, pave: Pave) {
    if let Some(pb_ids) = arena.edge_pave_blocks.get(&edge_id) {
        let pb_ids_copy: Vec<_> = pb_ids.clone();
        for pb_id in pb_ids_copy {
            if let Some(pb) = arena.pave_blocks.get_mut(pb_id)
                && pb.contains_parameter_interior(pave.parameter, 1e-10)
            {
                pb.add_extra_pave(pave);
            }
        }
    }
}
