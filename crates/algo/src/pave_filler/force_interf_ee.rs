//! Post-split EE overlap detection — creates CommonBlocks for coincident
//! leaf PaveBlocks from different original edges.
//!
//! Runs after `make_blocks` (which splits PaveBlocks at extra paves),
//! iterating leaf PaveBlocks to find pairs with matching 3D endpoints
//! and compatible curve geometry.

use std::collections::{HashMap, HashSet};

use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};

use crate::ds::{GfaArena, PaveBlockId};
use crate::error::AlgoError;

#[derive(Debug, Clone, Copy)]
struct LeafData {
    pave_block: PaveBlockId,
    edge: EdgeId,
    start: remus_math::vec::Point3,
    end: remus_math::vec::Point3,
    start_excess: f64,
    end_excess: f64,
}

/// Detect overlapping leaf PaveBlocks and group them into CommonBlocks.
///
/// Two leaf PaveBlocks from different original edges overlap if:
/// 1. Their start/end vertex positions are within tolerance
/// 2. Their edge curves have compatible geometry (same line direction,
///    same circle, etc.)
///
/// # Errors
///
/// Returns [`AlgoError`] if topology lookups fail.
#[allow(clippy::too_many_lines)]
pub fn perform(topo: &Topology, tol: Tolerance, arena: &mut GfaArena) -> Result<(), AlgoError> {
    let all_edge_pbs: Vec<(EdgeId, Vec<PaveBlockId>)> = arena
        .edge_pave_blocks
        .iter()
        .map(|(&eid, pbs)| (eid, arena.collect_leaf_pave_blocks(pbs)))
        .collect();

    let mut leaf_data = Vec::new();
    let mut max_excess = 0.0_f64;

    for (orig_edge, leaf_pbs) in &all_edge_pbs {
        for &pb_id in leaf_pbs {
            let pb = match arena.pave_blocks.get(pb_id) {
                Some(pb) => pb,
                None => continue,
            };
            let sv = arena.resolve_vertex(pb.start.vertex);
            let ev = arena.resolve_vertex(pb.end.vertex);
            let start_pos = topo.vertex(sv)?.point();
            let end_pos = topo.vertex(ev)?.point();
            let edge_excess = super::helpers::edge_tolerance_excess(topo, *orig_edge, tol.linear)?;
            let start_excess = edge_excess.max(super::helpers::vertex_tolerance_excess(
                topo, sv, tol.linear,
            )?);
            let end_excess = edge_excess.max(super::helpers::vertex_tolerance_excess(
                topo, ev, tol.linear,
            )?);
            max_excess = max_excess.max(start_excess).max(end_excess);
            leaf_data.push(LeafData {
                pave_block: pb_id,
                edge: *orig_edge,
                start: start_pos,
                end: end_pos,
                start_excess,
                end_excess,
            });
        }
    }

    // Find overlapping pairs. A naive scan is O(n²) over leaf PaveBlocks,
    // which explodes on solids with many edges (a shelled, lip-fused bin can
    // reach thousands of leaf blocks). Two blocks can only overlap if BOTH
    // endpoints coincide within tolerance, so we spatially hash each block by
    // the quantized cell of its (unordered) endpoint pair and only compare
    // blocks that share a candidate cell — collapsing the scan to ~O(n).
    let mut overlap_map: HashMap<PaveBlockId, Vec<PaveBlockId>> = HashMap::new();
    let n = leaf_data.len();

    // Cell size covers the largest possible pair band, so matching block
    // midpoints cannot fall beyond the immediate 3x3x3 neighborhood.
    let max_pair_band = super::helpers::tolerance_band(tol.linear, [max_excess, max_excess])?;
    let cell = (max_pair_band * 4.0).max(f64::MIN_POSITIVE);
    if !cell.is_finite() {
        return Err(remus_topology::TopologyError::InvalidToleranceValue {
            entity: "predicate band",
            value: cell,
        }
        .into());
    }
    let key = |p: remus_math::vec::Point3| -> (i64, i64, i64) {
        (
            (p.x() / cell).floor() as i64,
            (p.y() / cell).floor() as i64,
            (p.z() / cell).floor() as i64,
        )
    };
    let midpoint = |a: remus_math::vec::Point3, b: remus_math::vec::Point3| {
        remus_math::vec::Point3::new(
            f64::midpoint(a.x(), b.x()),
            f64::midpoint(a.y(), b.y()),
            f64::midpoint(a.z(), b.z()),
        )
    };

    // Bucket each leaf block by its midpoint cell.
    let mut buckets: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (i, leaf) in leaf_data.iter().enumerate() {
        buckets
            .entry(key(midpoint(leaf.start, leaf.end)))
            .or_default()
            .push(i);
    }

    // For each block, gather candidate partners from its own cell plus the
    // 3×3×3 neighborhood (each block lives in exactly one bucket, so a pair is
    // visited once via `j > i`), and run the exact same fwd/rev endpoint +
    // curve-compatibility test as the naive scan.
    for i in 0..n {
        let leaf_i = leaf_data[i];
        let mid_i = midpoint(leaf_i.start, leaf_i.end);
        let (kx, ky, kz) = key(mid_i);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(cands) = buckets.get(&(kx + dx, ky + dy, kz + dz)) else {
                        continue;
                    };
                    for &j in cands {
                        if j <= i {
                            continue;
                        }
                        let leaf_j = leaf_data[j];

                        if leaf_i.edge == leaf_j.edge {
                            continue;
                        }
                        if arena.pb_to_cb.contains_key(&leaf_i.pave_block)
                            && arena.pb_to_cb.get(&leaf_i.pave_block)
                                == arena.pb_to_cb.get(&leaf_j.pave_block)
                        {
                            continue;
                        }

                        let fwd_start_band = super::helpers::tolerance_band(
                            tol.linear,
                            [leaf_i.start_excess, leaf_j.start_excess],
                        )?;
                        let fwd_end_band = super::helpers::tolerance_band(
                            tol.linear,
                            [leaf_i.end_excess, leaf_j.end_excess],
                        )?;
                        let rev_start_band = super::helpers::tolerance_band(
                            tol.linear,
                            [leaf_i.start_excess, leaf_j.end_excess],
                        )?;
                        let rev_end_band = super::helpers::tolerance_band(
                            tol.linear,
                            [leaf_i.end_excess, leaf_j.start_excess],
                        )?;
                        let fwd_match = (leaf_i.start - leaf_j.start).length() <= fwd_start_band
                            && (leaf_i.end - leaf_j.end).length() <= fwd_end_band;
                        let rev_match = (leaf_i.start - leaf_j.end).length() <= rev_start_band
                            && (leaf_i.end - leaf_j.start).length() <= rev_end_band;
                        if !fwd_match && !rev_match {
                            continue;
                        }

                        let curve_i = topo.edge(leaf_i.edge)?.curve();
                        let curve_j = topo.edge(leaf_j.edge)?.curve();
                        if !curves_compatible(curve_i, curve_j, tol) {
                            continue;
                        }

                        overlap_map
                            .entry(leaf_i.pave_block)
                            .or_default()
                            .push(leaf_j.pave_block);
                        overlap_map
                            .entry(leaf_j.pave_block)
                            .or_default()
                            .push(leaf_i.pave_block);
                    }
                }
            }
        }
    }

    // Build transitive closure and create CommonBlocks
    let mut visited: HashSet<PaveBlockId> = HashSet::new();

    for leaf in &leaf_data {
        let pb_id = leaf.pave_block;
        if visited.contains(&pb_id) || !overlap_map.contains_key(&pb_id) {
            continue;
        }

        // BFS to find connected component
        let mut group = Vec::new();
        let mut queue = vec![pb_id];
        while let Some(current) = queue.pop() {
            if !visited.insert(current) {
                continue;
            }
            group.push(current);
            if let Some(neighbors) = overlap_map.get(&current) {
                for &nb in neighbors {
                    if !visited.contains(&nb) {
                        queue.push(nb);
                    }
                }
            }
        }

        if group.len() >= 2 {
            let cb_id = arena.create_common_block(group.clone());
            log::debug!(
                "ForceInterfEE: created CommonBlock {cb_id:?} with {} PaveBlocks",
                group.len()
            );
        }
    }

    Ok(())
}

/// Check if two edge curves are geometrically compatible (same type + parameters).
fn curves_compatible(a: &EdgeCurve, b: &EdgeCurve, tol: Tolerance) -> bool {
    // Exhaustive match — no wildcards per CLAUDE.md convention.
    // Adding a new EdgeCurve variant will require updating this match.
    match (a, b) {
        (EdgeCurve::Line, EdgeCurve::Line) => true,
        (EdgeCurve::Circle(ca), EdgeCurve::Circle(cb)) => {
            (ca.radius() - cb.radius()).abs() < tol.linear
                && (ca.center() - cb.center()).length() < tol.linear
                && ca.normal().dot(cb.normal()).abs() > 1.0 - tol.angular
        }
        (EdgeCurve::Ellipse(ea), EdgeCurve::Ellipse(eb)) => {
            (ea.semi_major() - eb.semi_major()).abs() < tol.linear
                && (ea.semi_minor() - eb.semi_minor()).abs() < tol.linear
                && (ea.center() - eb.center()).length() < tol.linear
                && ea.normal().dot(eb.normal()).abs() > 1.0 - tol.angular
        }
        // Same-type conic coincidence: identical placement implies
        // identical point sets, since both parameterizations are injective
        // over the whole real line.
        (EdgeCurve::Hyperbola(ha), EdgeCurve::Hyperbola(hb)) => {
            (ha.semi_major() - hb.semi_major()).abs() < tol.linear
                && (ha.semi_minor() - hb.semi_minor()).abs() < tol.linear
                && (ha.center() - hb.center()).length() < tol.linear
                && ha.normal().dot(hb.normal()).abs() > 1.0 - tol.angular
                // Sign matters here: `Hyperbola3D` models a single branch,
                // so an anti-parallel real axis is the OTHER branch.
                && ha.u_axis().dot(hb.u_axis()) > 1.0 - tol.angular
        }
        (EdgeCurve::Parabola(pa), EdgeCurve::Parabola(pb)) => {
            (pa.focal_length() - pb.focal_length()).abs() < tol.linear
                && (pa.vertex() - pb.vertex()).length() < tol.linear
                && pa.axis_dir().dot(pb.axis_dir()) > 1.0 - tol.angular
                // A parabola is symmetric about its axis, so only the PLANE
                // matters, not the sign of its normal.
                && pa.normal().dot(pb.normal()).abs() > 1.0 - tol.angular
        }
        // NurbsCurve overlap detection deferred — needs parametric comparison.
        (EdgeCurve::NurbsCurve(_), EdgeCurve::NurbsCurve(_)) => false,
        // Different curve types cannot be geometrically coincident. Every
        // left-hand variant is listed rather than using `_`, so adding an
        // `EdgeCurve` variant still makes the compiler flag this site.
        (
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_),
            _,
        ) => false,
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::vec::Point3;
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    use super::*;

    fn parallel_blocks(gap: f64, edge_tolerance: Option<f64>) -> (Topology, GfaArena) {
        let mut topo = Topology::new();
        let a0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let a1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let b0 = topo.add_vertex(Vertex::new(Point3::new(0.0, gap, 0.0), 1e-7));
        let b1 = topo.add_vertex(Vertex::new(Point3::new(1.0, gap, 0.0), 1e-7));
        let edge_a = topo.add_edge(Edge::with_tolerance(
            a0,
            a1,
            EdgeCurve::Line,
            edge_tolerance,
        ));
        let edge_b = topo.add_edge(Edge::with_tolerance(
            b0,
            b1,
            EdgeCurve::Line,
            edge_tolerance,
        ));
        let mut arena = GfaArena::new();
        arena.init_edge_pave_block(edge_a, a0, 0.0, a1, 1.0);
        arena.init_edge_pave_block(edge_b, b0, 0.0, b1, 1.0);
        (topo, arena)
    }

    #[test]
    fn declared_tubes_group_nearby_compatible_blocks() {
        let (topo, mut arena) = parallel_blocks(5e-7, Some(1e-4));
        perform(&topo, Tolerance::default(), &mut arena).unwrap();
        assert_eq!(arena.common_blocks.iter().count(), 1);
    }

    #[test]
    fn compatible_blocks_outside_default_tolerance_stay_separate() {
        let (topo, mut arena) = parallel_blocks(5e-7, None);
        perform(&topo, Tolerance::default(), &mut arena).unwrap();
        assert_eq!(arena.common_blocks.iter().count(), 0);
    }

    #[test]
    fn declared_tubes_do_not_group_blocks_beyond_their_combined_band() {
        let (topo, mut arena) = parallel_blocks(3e-4, Some(1e-4));
        perform(&topo, Tolerance::default(), &mut arena).unwrap();
        assert_eq!(arena.common_blocks.iter().count(), 0);
    }
}
