//! Map original edges to their split images (leaf pave blocks).
//!
//! After the PaveFiller has split edges at intersection points, each
//! original edge maps to one or more leaf pave blocks, each of which
//! has a `split_edge` pointing to the new topology edge.

use std::collections::HashMap;

use remus_topology::edge::EdgeId;

use crate::ds::GfaArena;

/// For each original edge, collect its leaf pave block split-edge IDs,
/// sorted by start parameter along the original edge's directed domain.
///
/// Returns a map from original edge ID to a list of new edge IDs that
/// replace it. Edges with no splits map to their original split-edge
/// (the single leaf pave block). The split edges are in parameter order
/// so that wire reconstruction can iterate them in sequence.
#[must_use]
pub fn fill_edge_images(arena: &GfaArena) -> HashMap<EdgeId, Vec<EdgeId>> {
    let mut images: HashMap<EdgeId, Vec<EdgeId>> = HashMap::new();

    for (&original_edge, pb_ids) in &arena.edge_pave_blocks {
        let leaves = arena.collect_leaf_pave_blocks(pb_ids);

        let mut split_with_param: Vec<(f64, EdgeId)> = Vec::new();

        for &leaf_id in &leaves {
            if let Some(pb) = arena.pave_blocks.get(leaf_id)
                && let Some(se) = pb.split_edge
            {
                split_with_param.push((pb.start.parameter, se));
            }
        }

        if split_with_param.is_empty() {
            images.insert(original_edge, vec![original_edge]);
            continue;
        }

        // Preserve the source edge's directed parameter order. A descending
        // stored trim is authoritative reverse traversal, so sorting those
        // children numerically upward would invert the rebuilt wire.
        let descending = leaves_direction_is_descending(arena, &leaves);
        split_with_param.sort_by(|a, b| {
            let order = a.0.total_cmp(&b.0);
            if descending { order.reverse() } else { order }
        });

        let split_edges: Vec<EdgeId> = split_with_param.into_iter().map(|(_, eid)| eid).collect();
        images.insert(original_edge, split_edges);
    }

    images
}

fn leaves_direction_is_descending(arena: &GfaArena, leaves: &[crate::ds::PaveBlockId]) -> bool {
    leaves.iter().find_map(|&leaf_id| {
        let pb = arena.pave_blocks.get(leaf_id)?;
        let (start, end) = pb.parameter_range();
        match start.total_cmp(&end) {
            std::cmp::Ordering::Less => Some(false),
            std::cmp::Ordering::Greater => Some(true),
            std::cmp::Ordering::Equal => None,
        }
    }) == Some(true)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::vec::Point3;
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;

    use super::fill_edge_images;
    use crate::ds::GfaArena;

    #[test]
    fn descending_split_images_keep_source_traversal_order() {
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), 1e-7));
        let original = topo.add_edge(Edge::new(v0, v2, EdgeCurve::Line));
        let first = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let second = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));

        let mut arena = GfaArena::new();
        let first_pb = arena.init_edge_pave_block(original, v0, 5.0, v1, 3.0);
        let second_pb = arena.init_edge_pave_block(original, v1, 3.0, v2, 1.0);
        arena.pave_blocks.get_mut(first_pb).unwrap().split_edge = Some(first);
        arena.pave_blocks.get_mut(second_pb).unwrap().split_edge = Some(second);

        assert_eq!(fill_edge_images(&arena)[&original], vec![first, second]);
    }
}
