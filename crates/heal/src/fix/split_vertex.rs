//! Split common vertex — split vertices shared by non-adjacent edges.
//!
//! In a manifold solid, each vertex is typically connected to a modest
//! number of edges (3+). Vertices connected to an unreasonably large
//! number of edges may indicate over-connected topology that should be
//! split into separate vertices. This module detects such vertices and
//! splits them into separate connected groups based on face adjacency.

use std::collections::{HashMap, HashSet, VecDeque};

use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};

use super::FixResult;
use super::config::FixConfig;
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Maximum number of edges a vertex should be connected to before
/// it is flagged as over-connected.
const MAX_VERTEX_EDGES: usize = 20;

/// Vertex descendants allocated by completed common-vertex splits.
#[derive(Debug, Default)]
pub struct SplitVertexHistory {
    /// Each retained source and all of its resulting vertex identities.
    pub vertices: Vec<(VertexId, Vec<VertexId>)>,
}

/// Split common vertices and retain the actual allocated descendants.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn fix_split_common_vertex_with_history(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<(FixResult, SplitVertexHistory), HealError> {
    if config.fix_split_common_vertex == super::config::FixMode::Off {
        return Ok((FixResult::ok(), SplitVertexHistory::default()));
    }
    let mut history = SplitVertexHistory::default();
    let result = fix_split_common_vertex_impl(topo, solid_id, ctx, Some(&mut history))?;
    Ok((result, history))
}

/// Split vertices that are shared by too many non-adjacent edges.
///
/// For each over-connected vertex (more than `MAX_VERTEX_EDGES` edge
/// connections), edges are grouped by face adjacency. If multiple
/// disconnected groups exist, the vertex is duplicated and edges in
/// each group (after the first) are reassigned to a fresh vertex at
/// the same position.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn fix_split_common_vertex(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    if config.fix_split_common_vertex == super::config::FixMode::Off {
        return Ok(FixResult::ok());
    }
    fix_split_common_vertex_impl(topo, solid_id, ctx, None)
}

fn fix_split_common_vertex_impl(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
    mut history: Option<&mut SplitVertexHistory>,
) -> Result<FixResult, HealError> {
    // Walk outer + inner (cavity) shells. Over-connection counting
    // is solid-scoped: a vertex can be over-connected via a mix of
    // outer-shell and inner-shell edges, and outer-shell-only would
    // miss those cases.
    let face_ids = solid_faces(topo, solid_id)?;

    let mut vertex_edge_count: HashMap<usize, (VertexId, usize)> = HashMap::new();

    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge())?;

                let start_vid = edge.start();
                vertex_edge_count
                    .entry(start_vid.index())
                    .or_insert_with(|| (start_vid, 0))
                    .1 += 1;

                let end_vid = edge.end();
                vertex_edge_count
                    .entry(end_vid.index())
                    .or_insert_with(|| (end_vid, 0))
                    .1 += 1;
            }
        }
    }

    let mut over_connected: Vec<VertexId> = Vec::new();
    for &(vid, count) in vertex_edge_count.values() {
        if count > MAX_VERTEX_EDGES {
            over_connected.push(vid);
        }
    }

    if over_connected.is_empty() {
        return Ok(FixResult::ok());
    }

    let mut total_splits = 0usize;

    for &vertex_id in &over_connected {
        let splits = split_vertex(topo, vertex_id, solid_id, ctx, history.as_deref_mut())?;
        total_splits += splits;
    }

    if total_splits == 0 {
        ctx.warn(format!(
            "detected {} over-connected vertices (>{MAX_VERTEX_EDGES} edges) but all are single-group — no splits needed",
            over_connected.len()
        ));
        return Ok(FixResult::ok());
    }

    ctx.info(format!(
        "split {total_splits} over-connected vertices into separate groups"
    ));

    Ok(FixResult::changed(
        Status::DONE5,
        super::RepairActionKind::CommonVertexSplit,
        total_splits,
    ))
}

/// Split a single over-connected vertex into separate groups.
///
/// Groups edges by face adjacency: edges that share a face belong to the
/// same group. For each group after the first, a new vertex is created at
/// the same position and all edges in that group are updated.
///
/// Returns the number of new vertices created (0 if only one group).
fn split_vertex(
    topo: &mut Topology,
    vertex_id: VertexId,
    solid_id: SolidId,
    ctx: &mut HealContext,
    history: Option<&mut SplitVertexHistory>,
) -> Result<usize, HealError> {
    // Walk outer + inner (cavity) shells (see top-level comment in
    // `fix_split_common_vertex`).
    let face_ids = solid_faces(topo, solid_id)?;

    let mut edge_faces: HashMap<usize, HashSet<usize>> = HashMap::new();
    let mut vertex_edges: Vec<EdgeId> = Vec::new();
    let mut vertex_edges_set: HashSet<usize> = HashSet::new();

    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let eid = oe.edge();
                let edge = topo.edge(eid)?;

                edge_faces
                    .entry(eid.index())
                    .or_default()
                    .insert(fid.index());

                if (edge.start() == vertex_id || edge.end() == vertex_id)
                    && vertex_edges_set.insert(eid.index())
                {
                    vertex_edges.push(eid);
                }
            }
        }
    }

    if vertex_edges.len() <= 1 {
        return Ok(0);
    }

    // Two edges belong to the same group if they share at least one face.
    // Use union-find via BFS on the edge adjacency graph.
    let n = vertex_edges.len();
    let mut groups: Vec<i32> = vec![-1; n];
    let mut group_id = 0i32;

    for start_idx in 0..n {
        if groups[start_idx] >= 0 {
            continue;
        }
        groups[start_idx] = group_id;
        let mut queue = VecDeque::new();
        queue.push_back(start_idx);

        while let Some(current) = queue.pop_front() {
            let current_eid = vertex_edges[current];
            let current_faces = match edge_faces.get(&current_eid.index()) {
                Some(f) => f,
                None => continue,
            };

            for neighbor_idx in 0..n {
                if groups[neighbor_idx] >= 0 {
                    continue;
                }
                let neighbor_eid = vertex_edges[neighbor_idx];
                let neighbor_faces = match edge_faces.get(&neighbor_eid.index()) {
                    Some(f) => f,
                    None => continue,
                };

                let shares_face = current_faces.iter().any(|f| neighbor_faces.contains(f));
                if shares_face {
                    groups[neighbor_idx] = group_id;
                    queue.push_back(neighbor_idx);
                }
            }
        }

        group_id += 1;
    }

    #[allow(clippy::cast_sign_loss)]
    let num_groups = group_id as usize;
    if num_groups <= 1 {
        return Ok(0);
    }

    let vertex_data = topo.vertex(vertex_id)?;
    let position = vertex_data.point();
    let vtx_tolerance = vertex_data.tolerance();

    let mut group_edges: HashMap<usize, Vec<(EdgeId, bool, bool)>> = HashMap::new();
    for (i, &eid) in vertex_edges.iter().enumerate() {
        #[allow(clippy::cast_sign_loss)]
        let g = groups[i] as usize;
        if g == 0 {
            continue;
        }
        let edge = topo.edge(eid)?;
        let is_start = edge.start() == vertex_id;
        let is_end = edge.end() == vertex_id;
        group_edges
            .entry(g)
            .or_default()
            .push((eid, is_start, is_end));
    }

    let mut descendants = vec![vertex_id];
    let mut new_vertices_created = 0usize;
    for edges in group_edges.values() {
        let new_vid = topo.add_vertex(Vertex::new(position, vtx_tolerance));

        for &(eid, update_start, update_end) in edges {
            let edge = topo.edge_mut(eid)?;
            if update_start {
                edge.set_start(new_vid);
            }
            if update_end {
                edge.set_end(new_vid);
            }
        }

        descendants.push(new_vid);
        new_vertices_created += 1;
    }

    ctx.info(format!(
        "vertex (index {}) split into {} groups ({} new vertices)",
        vertex_id.index(),
        num_groups,
        new_vertices_created
    ));

    if let Some(history) = history {
        descendants.sort_by_key(|id| id.index());
        history.vertices.push((vertex_id, descendants));
    }
    Ok(new_vertices_created)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::wire::{OrientedEdge, Wire};

    #[test]
    fn completed_splits_record_actual_descendants_across_shells() {
        // Disconnected triangle fans exercise lineage, not closed-solid validity.
        let mut topo = Topology::new();
        let source = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let mut faces = Vec::new();
        for i in 0..11 {
            let x = f64::from(i + 1);
            let a = topo.add_vertex(Vertex::new(Point3::new(x, 0.0, 0.0), 1e-7));
            let b = topo.add_vertex(Vertex::new(Point3::new(x, 1.0, 0.0), 1e-7));
            let mut edges = Vec::new();
            for (start, end) in [(source, a), (a, b), (b, source)] {
                let edge = topo.add_edge(Edge::new(start, end, EdgeCurve::Line));
                edges.push(OrientedEdge::new(edge, true));
            }
            let wire = topo.add_wire(Wire::new(edges, true).unwrap());
            faces.push(topo.add_face(Face::new(
                wire,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            )));
        }
        let inner = topo.add_shell(Shell::new(faces.split_off(6)).unwrap());
        let outer = topo.add_shell(Shell::new(faces).unwrap());
        let solid = topo.add_solid(Solid::new(outer, vec![inner]));
        let disabled = FixConfig {
            fix_split_common_vertex: super::super::config::FixMode::Off,
            ..Default::default()
        };
        let before_vertices = remus_topology::explorer::solid_vertices(&topo, solid).unwrap();
        let (disabled_result, disabled_history) = fix_split_common_vertex_with_history(
            &mut topo,
            solid,
            &mut HealContext::new(),
            &disabled,
        )
        .unwrap();
        assert_eq!(disabled_result.actions_taken, 0);
        assert!(disabled_history.vertices.is_empty());
        assert_eq!(
            fix_split_common_vertex(&mut topo, solid, &mut HealContext::new(), &disabled)
                .unwrap()
                .actions_taken,
            0
        );
        assert_eq!(
            remus_topology::explorer::solid_vertices(&topo, solid).unwrap(),
            before_vertices
        );
        let mut pipeline_topo = topo.clone();
        let mut process = crate::pipeline::process::HealProcess::new();
        process.add_step("split_common_vertex");
        process.add_step("split_common_vertex");
        let (_, reports, steps) = process
            .execute_with_history(&mut pipeline_topo, solid)
            .unwrap();
        assert_eq!(reports[0].actions_taken, 10);
        assert_eq!(reports[1].actions_taken, 0);
        let key = remus_topology::journal::EntityKey::vertex(source.index());
        let claims = steps[0].replacements.entity_history().unwrap();
        assert_eq!(claims[&key].len(), 11);
        assert!(claims[&key].contains(&key));
        assert!(
            claims[&key]
                .iter()
                .all(|target| steps[0].result.contains(target))
        );
        assert!(steps[1].replacements.entity_history().unwrap().is_empty());
        let mut ordinary_topo = topo.clone();
        let (_, ordinary_reports) = process.execute(&mut ordinary_topo, solid).unwrap();
        assert_eq!(ordinary_reports[0].actions_taken, reports[0].actions_taken);
        assert_eq!(
            remus_topology::explorer::solid_vertices(&ordinary_topo, solid)
                .unwrap()
                .len(),
            remus_topology::explorer::solid_vertices(&pipeline_topo, solid)
                .unwrap()
                .len(),
        );
        let (report, history) = fix_split_common_vertex_with_history(
            &mut topo,
            solid,
            &mut HealContext::new(),
            &FixConfig::default(),
        )
        .unwrap();
        assert_eq!(report.actions_taken, 10);
        assert_eq!(history.vertices.len(), 1);
        let (original, targets) = &history.vertices[0];
        assert_eq!(*original, source);
        assert_eq!(targets.len(), 11);
        assert!(targets.contains(&source));
        let live = remus_topology::explorer::solid_vertices(&topo, solid).unwrap();
        for target in targets {
            assert!(live.contains(target));
            assert_eq!(
                topo.vertex(*target).unwrap().point(),
                topo.vertex(source).unwrap().point()
            );
        }
        let mut records = crate::reshape::ReShape::new();
        records
            .record_applied_vertex_split(source, targets.clone())
            .unwrap();
        let before: Vec<_> = remus_topology::explorer::solid_edges(&topo, solid)
            .unwrap()
            .iter()
            .map(|id| {
                let edge = topo.edge(*id).unwrap();
                (*id, edge.start(), edge.end())
            })
            .collect();
        records.apply(&mut topo, solid).unwrap();
        for (id, start, end) in before {
            let edge = topo.edge(id).unwrap();
            assert_eq!((edge.start(), edge.end()), (start, end));
        }
        let (repeat, history) = fix_split_common_vertex_with_history(
            &mut topo,
            solid,
            &mut HealContext::new(),
            &FixConfig::default(),
        )
        .unwrap();
        assert_eq!(repeat.actions_taken, 0);
        assert!(history.vertices.is_empty());
    }
}
