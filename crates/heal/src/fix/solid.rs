//! Solid fixing — top-level fix orchestration.
//!
//! Orchestrates shell, face, wire, and edge fixes, plus solid-level
//! repairs such as coincident vertex merging and small face removal.

use remus_math::compact_sort::CompactSort;
use std::collections::HashMap;

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use super::FixResult;
use super::config::FixConfig;
use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Fix a solid: orchestrates shell, face, wire, and edge fixes.
///
/// 1. Fixes the outer shell (face-level + orientation).
/// 2. If `config.fix_coincident_vertices` permits, merges coincident
///    vertices across the solid.
/// 3. If `config.fix_small_faces` permits, removes degenerate small faces.
/// 4. If `config.fix_duplicate_faces` permits, removes geometrically
///    coincident duplicate faces.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups or resource checks fail. Errors
/// restore the topology and the context's pending repairs and messages.
pub fn fix_solid(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    let reshape = ctx.reshape.clone();
    let messages = ctx.messages.len();
    let result = remus_topology::transaction::run_transacted(topo, |topo| {
        fix_solid_inner(topo, solid_id, ctx, config)
    });
    if result.is_err() {
        ctx.reshape = reshape;
        ctx.messages.truncate(messages);
    }
    result
}

fn fix_solid_inner(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
    config: &FixConfig,
) -> Result<FixResult, HealError> {
    let mut result = FixResult::ok();

    let solid = topo.solid(solid_id)?;
    let shell_id = solid.outer_shell();

    let shell_result = super::shell::fix_shell(topo, shell_id, ctx, config)?;
    result.merge(&shell_result);

    let solid = topo.solid(solid_id)?;
    let inner_shells: Vec<_> = solid.inner_shells().to_vec();
    for shell_id in inner_shells {
        let inner_result = super::shell::fix_shell(topo, shell_id, ctx, config)?;
        result.merge(&inner_result);
    }

    // Always detected (cheap), applied per config mode.
    let should_merge = config.fix_coincident_vertices.should_fix(true);
    if should_merge {
        let merge_result = merge_coincident_vertices(topo, solid_id, ctx)?;
        result.merge(&merge_result);
    }

    let should_fix_small = config.fix_small_faces.should_fix(true);
    if should_fix_small {
        let small_result = super::small_face::fix_small_faces(topo, solid_id, ctx, config)?;
        result.merge(&small_result);
    }

    let should_fix_dupes = config.fix_duplicate_faces.should_fix(true);
    if should_fix_dupes {
        let dup_result = super::duplicate_faces::fix_duplicate_faces(topo, solid_id, ctx)?;
        result.merge(&dup_result);
    }

    Ok(result)
}

/// Merge coincident vertices in every shell of a solid.
///
/// For each pair of vertices within `ctx.tolerance.linear` distance,
/// the higher-index vertex is replaced by the lower-index one. Edge
/// start/end references are updated via the `ReShape` replacement tracker.
///
/// Each shell (outer plus every cavity) is merged on its own, as
/// [`fix_small_faces`](super::small_face::fix_small_faces) does: a cavity
/// vertex never merges into an outer-shell vertex.
///
/// Ported from `remus-operations` `heal::merge_coincident_vertices`.
fn merge_coincident_vertices(
    topo: &mut Topology,
    solid_id: SolidId,
    ctx: &mut HealContext,
) -> Result<FixResult, HealError> {
    let solid_data = topo.solid(solid_id)?;
    let shell_ids: Vec<ShellId> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();

    let mut merged_count = 0usize;
    for shell_id in shell_ids {
        merged_count += merge_shell_coincident_vertices(topo, shell_id, ctx)?;
    }

    if merged_count == 0 {
        return Ok(FixResult::ok());
    }

    ctx.info(format!("merged {merged_count} coincident vertices"));

    Ok(FixResult::changed(
        Status::DONE2,
        super::RepairActionKind::CoincidentVertexMerged,
        merged_count,
    ))
}

/// Merge coincident vertices within one shell; returns the merge count.
#[allow(clippy::too_many_lines)]
fn merge_shell_coincident_vertices(
    topo: &mut Topology,
    shell_id: ShellId,
    ctx: &mut HealContext,
) -> Result<usize, HealError> {
    let tol = ctx.tolerance.linear;

    let face_ids: Vec<_> = topo.shell(shell_id)?.faces().to_vec();

    let mut vertex_ids: Vec<VertexId> = Vec::new();
    let mut positions: Vec<Point3> = Vec::new();
    let mut seen = std::collections::HashSet::new();

    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge())?;
                for &vid in &[edge.start(), edge.end()] {
                    if seen.insert(vid.index()) {
                        let point = topo.vertex(vid)?.point();
                        vertex_ids.push(vid);
                        positions.push(point);
                    }
                }
            }
        }
    }

    // Build merge map: higher-index merges into lower-index (canonical).
    // Candidate discovery is a conservative spatial index
    // (`vertex_merge::plan_coincident_merges`); the exact predicate, tolerance
    // policy, canonical representative, deterministic order, and
    // nontransitive semantics equal the legacy all-pairs scan (PERF-H01).
    let plan = super::vertex_merge::plan_coincident_merges(&positions, tol);
    debug_assert_eq!(plan.merge_target.len(), vertex_ids.len());
    let mut merge_to: HashMap<usize, VertexId> = HashMap::new();
    let mut merged_count = 0usize;

    for (j, target) in plan.merge_target.iter().enumerate() {
        if let Some(i) = *target {
            merge_to.insert(vertex_ids[j].index(), vertex_ids[i]);
            merged_count += 1;
        }
    }

    if merged_count == 0 {
        return Ok(0);
    }

    for (&from_idx, &to_vid) in &merge_to {
        if let Some(&from_vid) = vertex_ids.iter().find(|v| v.index() == from_idx) {
            ctx.reshape.replace_vertex(from_vid, to_vid);
        }
    }

    // Also apply directly to edges (snapshot then allocate pattern).
    let mut edge_ids = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                edge_ids.push(oe.edge());
            }
        }
    }
    edge_ids.compact_sort_by_key(|e| e.index());
    edge_ids.dedup_by_key(|e| e.index());

    let updates: Vec<_> = edge_ids
        .iter()
        .filter_map(|&eid| {
            let edge = topo.edge(eid).ok()?;
            let new_start = merge_to
                .get(&edge.start().index())
                .copied()
                .unwrap_or_else(|| edge.start());
            let new_end = merge_to
                .get(&edge.end().index())
                .copied()
                .unwrap_or_else(|| edge.end());
            if new_start != edge.start() || new_end != edge.end() {
                Some((eid, new_start, new_end))
            } else {
                None
            }
        })
        .collect();

    for (eid, new_start, new_end) in updates {
        let edge = topo.edge_mut(eid)?;
        // `set_start`/`set_end`, never a whole-`Edge` rebuild: an explicit
        // trim (RFC 0002, Stage 3) and an edge-specific tolerance are not
        // recoverable from the endpoints, and a vertex merge changes neither
        // the curve nor the parameter interval on it.
        edge.set_start(new_start);
        edge.set_end(new_end);
    }

    Ok(merged_count)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_topology::Topology;

    use super::merge_coincident_vertices;
    use crate::context::HealContext;
    use crate::test_fixtures::{broken_junctions, cavity_shell, hollow_block, open_corner_gap};

    /// The same 1e-9 corner gap, opened on either shell of a hollow block,
    /// must be merged either way.
    #[test]
    fn coincident_vertices_merge_in_the_cavity_as_on_the_outer_shell() {
        for in_cavity in [false, true] {
            let mut topo = Topology::new();
            let solid = hollow_block(&mut topo);
            let shell = if in_cavity {
                cavity_shell(&topo, solid)
            } else {
                topo.solid(solid).unwrap().outer_shell()
            };
            let (corner, twin) = open_corner_gap(&mut topo, shell, 1e-9);
            assert_eq!(broken_junctions(&topo, shell), 2);

            let mut ctx = HealContext::new();
            let result = merge_coincident_vertices(&mut topo, solid, &mut ctx).unwrap();

            assert_eq!(result.actions_taken, 1, "in_cavity = {in_cavity}");
            assert_eq!(
                ctx.reshape.resolve_vertex(twin),
                ctx.reshape.resolve_vertex(corner),
                "in_cavity = {in_cavity}: corner and twin resolve to one vertex"
            );
            assert_eq!(broken_junctions(&topo, shell), 0, "in_cavity = {in_cavity}");
        }
    }
}
