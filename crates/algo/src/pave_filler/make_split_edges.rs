//! Create topology edges from finalized pave blocks.
//!
//! This is the single point where `&mut Topology` is written during the
//! PaveFiller pipeline. For each leaf pave block that does not yet have a
//! `split_edge`, a new [`Edge`] is created in the topology.
//!
//! **CommonBlock-aware:** When a PaveBlock belongs to a CommonBlock, a single
//! edge is created for the entire group. All PBs in the CB reference the
//! same edge entity.

use std::collections::HashSet;

use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};

use crate::ds::{CommonBlockId, GfaArena, PaveBlockId};
use crate::error::AlgoError;

/// Create topology edges for all leaf pave blocks.
///
/// For each pave block without children (leaf) and without an existing
/// `split_edge`, a new [`Edge`] is created in the topology. The curve
/// type is inherited from the original edge.
///
/// **CommonBlock handling:** When a PB belongs to a CommonBlock that has
/// already been processed, the existing edge is reused. When processing
/// a CB for the first time, the canonical PB creates the edge and all
/// members share it.
///
/// # Errors
///
/// Returns [`AlgoError`] if a topology lookup fails.
pub fn perform(topo: &mut Topology, arena: &mut GfaArena) -> Result<(), AlgoError> {
    // Track processed CommonBlocks to avoid creating duplicate edges
    let mut processed_cbs: HashSet<CommonBlockId> = HashSet::new();

    let leaf_ids: Vec<_> = arena
        .pave_blocks
        .iter()
        .filter(|(_, pb)| pb.children.is_empty() && pb.split_edge.is_none())
        .map(|(id, _)| id)
        .collect();

    for pb_id in leaf_ids {
        if let Some(&cb_id) = arena.pb_to_cb.get(&pb_id) {
            if !processed_cbs.insert(cb_id) {
                // CB already processed — reuse its split edge
                let split_edge = arena.common_blocks.get(cb_id).and_then(|cb| cb.split_edge);
                if let Some(edge_id) = split_edge
                    && let Some(pb) = arena.pave_blocks.get_mut(pb_id)
                {
                    pb.split_edge = Some(edge_id);
                }
                continue;
            }

            // First PB in this CB — use canonical PB to create the edge
            let canonical_pb_id = arena
                .common_blocks
                .get(cb_id)
                .and_then(|cb| cb.pave_blocks.first().copied())
                .unwrap_or(pb_id);

            let edge_id = create_split_edge(topo, arena, canonical_pb_id)?;

            // Set split_edge on the CB and ALL PBs in the group
            let all_pbs: Vec<PaveBlockId> = arena
                .common_blocks
                .get(cb_id)
                .map(|cb| cb.pave_blocks.clone())
                .unwrap_or_default();

            if let Some(cb) = arena.common_blocks.get_mut(cb_id) {
                cb.split_edge = Some(edge_id);
            }
            for &member_pb in &all_pbs {
                if let Some(pb) = arena.pave_blocks.get_mut(member_pb) {
                    pb.split_edge = Some(edge_id);
                }
            }

            log::debug!(
                "MakeSplitEdges: created edge {edge_id:?} for CommonBlock {cb_id:?} \
                 ({} PaveBlocks)",
                all_pbs.len()
            );
        } else {
            let edge_id = create_split_edge(topo, arena, pb_id)?;
            if let Some(pb) = arena.pave_blocks.get_mut(pb_id) {
                pb.split_edge = Some(edge_id);
            }
            log::debug!("MakeSplitEdges: created edge {edge_id:?} for pave block {pb_id:?}");
        }
    }

    Ok(())
}

/// Create a single split edge from a pave block's data.
fn create_split_edge(
    topo: &mut Topology,
    arena: &GfaArena,
    pb_id: PaveBlockId,
) -> Result<EdgeId, AlgoError> {
    let (original_edge_id, start_vertex, end_vertex, t_start, t_end) = {
        let pb = arena.pave_blocks.get(pb_id).ok_or_else(|| {
            AlgoError::FaceSplitFailed(format!(
                "MakeSplitEdges: pave block {pb_id:?} not found in arena"
            ))
        })?;
        let start_v = arena.resolve_vertex(pb.start.vertex);
        let end_v = arena.resolve_vertex(pb.end.vertex);
        (
            pb.original_edge,
            start_v,
            end_v,
            pb.start.parameter,
            pb.end.parameter,
        )
    };

    let curve = topo.edge(original_edge_id)?.curve().clone();
    // The pave block has the exact sub-span on the original curve in hand;
    // record it so the domain never has to be reconstructed by endpoint
    // projection (RFC 0002, Stage 3). Per curve type:
    // - `Line` carries no stored geometry — the sub-edge's line re-anchors
    //   to its new vertices with domain [0, 1], so parameters measured on
    //   the ORIGINAL edge do not apply to it: no trim.
    // - Angular curves (circle, ellipse) evaluate any angle, so a block
    //   that wraps the parameter seam is stored unwrapped by one period
    //   (t_end + 2π), keeping the span forward.
    // - Other curves store only a forward span; a wrapped span keeps the
    //   legacy projection path.
    let trim = match &curve {
        EdgeCurve::Line => None,
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => {
            let span = if t_end > t_start {
                t_end
            } else {
                t_end + std::f64::consts::TAU
            };
            Some((t_start, span))
        }
        EdgeCurve::NurbsCurve(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
            (t_end > t_start).then_some((t_start, t_end))
        }
    };
    let mut new_edge = Edge::new(start_vertex, end_vertex, curve);
    new_edge.set_trim(trim);
    Ok(topo.add_edge(new_edge))
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod trim_tests {
    use remus_math::curves::Circle3D;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;

    use crate::ds::{GfaArena, Pave, PaveBlock};

    use super::create_split_edge;

    fn arc_split_fixture(t_start: f64, t_end: f64, curve: EdgeCurve) -> Topology {
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(-1.0, 0.0, 0.0), 1e-7));
        let _edge = topo.add_edge(Edge::new(v0, v1, curve));
        let _ = (t_start, t_end);
        topo
    }

    fn circle() -> EdgeCurve {
        EdgeCurve::Circle(
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap(),
        )
    }

    fn split(topo: &mut Topology, t_start: f64, t_end: f64) -> remus_topology::EdgeId {
        let edge_id = topo.edge_id_from_index(0).unwrap();
        let v0 = topo.vertex_id_from_index(0).unwrap();
        let v1 = topo.vertex_id_from_index(1).unwrap();
        let mut arena = GfaArena::new();
        let pb = arena.pave_blocks.alloc(PaveBlock::new(
            edge_id,
            Pave::new(v0, t_start),
            Pave::new(v1, t_end),
        ));
        create_split_edge(topo, &arena, pb).unwrap()
    }

    #[test]
    fn forward_arc_span_is_recorded_exactly() {
        let mut topo = arc_split_fixture(0.5, 2.5, circle());
        let eid = split(&mut topo, 0.5, 2.5);
        assert_eq!(topo.edge(eid).unwrap().trim(), Some((0.5, 2.5)));
    }

    #[test]
    fn wrapped_arc_span_is_unwrapped_by_one_period() {
        const TAU: f64 = std::f64::consts::TAU;
        let mut topo = arc_split_fixture(5.5, 1.0, circle());
        let eid = split(&mut topo, 5.5, 1.0);
        assert_eq!(topo.edge(eid).unwrap().trim(), Some((5.5, 1.0 + TAU)));
    }

    #[test]
    fn line_sub_edges_store_no_trim() {
        // A line re-anchors to its new vertices with domain [0, 1]; the
        // original-edge parameters do not apply.
        let mut topo = arc_split_fixture(0.25, 0.75, EdgeCurve::Line);
        let eid = split(&mut topo, 0.25, 0.75);
        assert_eq!(topo.edge(eid).unwrap().trim(), None);
    }
}
