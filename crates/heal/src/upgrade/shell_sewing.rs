//! Shell sewing — close an open shell by sharing coincident free edges.
//!
//! A shell assembled patch-by-patch (a mesh import, a set of separately
//! built faces) is geometrically closed but topologically open: each face
//! carries its own copy of every boundary edge, so no edge is shared and
//! every one of them reads as free. Sewing repairs that by making the two
//! faces on either side of a seam reference **one** edge.
//!
//! Two coincident endpoints are not evidence that two edges carry the same
//! curve — a chord and the arc it subtends share both — so every candidate
//! pair is sampled along its interior before it is merged, and a pair that
//! disagrees is declined rather than reported as sewn.

use std::collections::{HashMap, HashSet};

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::pcurve::PCurve;
use remus_topology::shell::ShellId;
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, WireId};

use crate::HealError;

/// Interior samples used to decide whether two coincident free edges carry
/// the same 3D curve. The endpoints are already known to match, so only the
/// interior carries information.
const CURVE_SAMPLES: u32 = 7;

/// What a sewing pass did, and what it refused to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SewReport {
    /// Free-edge pairs merged into one shared edge. Each merge removes two
    /// free edges from the shell.
    pub sewn: usize,
    /// Coincident free-edge pairs that were **not** merged: the curves
    /// between the shared endpoints disagreed, the partner was ambiguous, or
    /// the merge would have collided in the pcurve registry. A non-zero
    /// count means the shell is still open on purpose.
    pub declined: usize,
}

/// Sew coincident free boundary edges in a shell.
///
/// Returns the number of edges sewn — that is, the number of free-edge pairs
/// collapsed onto a single shared edge. Zero means nothing was sewn, whether
/// because there was nothing to sew or because every candidate was declined;
/// [`sew_shell_report`] distinguishes the two.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn sew_shell(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<usize, HealError> {
    Ok(sew_shell_report(topo, shell_id, tolerance)?.sewn)
}

/// Sew coincident free boundary edges in a shell, reporting declines.
///
/// Each pair of free edges whose endpoints coincide within `tolerance` is
/// sampled along its interior; only a pair that agrees everywhere is merged.
/// The dropped edge's wire uses are rewritten onto the retained edge (with
/// the traversal sense flipped when the retained edge runs the other way),
/// its pcurves are re-keyed onto the retained edge, and its vertices are
/// merged into the retained edge's, so the wires on both sides stay
/// connected chains.
///
/// A free edge with more than one geometrically valid partner is left alone
/// rather than paired arbitrarily.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail.
pub fn sew_shell_report(
    topo: &mut Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<SewReport, HealError> {
    let (wire_ids, usage) = survey_shell(topo, shell_id)?;

    let mut free_ids: Vec<EdgeId> = usage
        .iter()
        .filter(|&(_, &count)| count == 1)
        .map(|(&eid, _)| eid)
        .collect();
    free_ids.sort_unstable();

    if free_ids.len() < 2 {
        return Ok(SewReport::default());
    }

    let mut free = Vec::with_capacity(free_ids.len());
    for id in free_ids {
        let edge = topo.edge(id)?;
        let (start, end) = (edge.start(), edge.end());
        free.push(FreeEdge {
            id,
            start_pos: topo.vertex(start)?.point(),
            end_pos: topo.vertex(end)?.point(),
        });
    }

    let (plans, declined) = plan_merges(topo, &free, tolerance)?;
    if plans.is_empty() {
        return Ok(SewReport { sewn: 0, declined });
    }

    apply_merges(topo, shell_id, &wire_ids, &plans)?;

    Ok(SewReport {
        sewn: plans.len(),
        declined,
    })
}

/// A boundary edge used by exactly one face wire, with its endpoints
/// snapshotted before any merging moves them.
struct FreeEdge {
    id: EdgeId,
    start_pos: Point3,
    end_pos: Point3,
}

/// One planned merge: `drop` disappears from every wire in favour of `keep`.
struct Merge {
    keep: EdgeId,
    drop: EdgeId,
    /// `true` when `keep` traces `drop`'s curve backwards, so every use of
    /// `drop` flips its traversal sense when rewritten onto `keep`.
    reversed: bool,
}

/// Every distinct wire in the shell, and how many oriented-edge uses each
/// edge has across it.
///
/// Uses are counted per face traversal, matching
/// [`crate::analysis::free_bounds::find_free_bounds`], so the two agree on
/// which edges are free.
fn survey_shell(
    topo: &Topology,
    shell_id: ShellId,
) -> Result<(Vec<WireId>, HashMap<EdgeId, usize>), HealError> {
    let face_ids = topo.shell(shell_id)?.faces().to_vec();
    let mut wire_ids = Vec::new();
    let mut seen = HashSet::new();
    let mut usage: HashMap<EdgeId, usize> = HashMap::new();

    for face_id in face_ids {
        let face = topo.face(face_id)?;
        let face_wires: Vec<WireId> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for wire_id in face_wires {
            for oe in topo.wire(wire_id)?.edges() {
                *usage.entry(oe.edge()).or_insert(0) += 1;
            }
            if seen.insert(wire_id) {
                wire_ids.push(wire_id);
            }
        }
    }

    Ok((wire_ids, usage))
}

/// Pair up free edges that share both endpoints *and* the curve between
/// them. Returns the merges to perform and the number of coincident pairs
/// declined.
fn plan_merges(
    topo: &Topology,
    free: &[FreeEdge],
    tolerance: f64,
) -> Result<(Vec<Merge>, usize), HealError> {
    let tol_sq = tolerance * tolerance;
    // An edge is consumed once it has been sewn, or once it has been ruled
    // out as an arbitrary choice among several valid partners.
    let mut consumed = vec![false; free.len()];
    let mut plans = Vec::new();
    let mut declined = 0;

    for i in 0..free.len() {
        if consumed[i] {
            continue;
        }

        let mut candidates: Vec<(usize, bool)> = Vec::new();
        let mut any_coincident = false;

        for j in (i + 1)..free.len() {
            if consumed[j] {
                continue;
            }
            let (fwd_ok, rev_ok) = endpoints_coincide(&free[i], &free[j], tol_sq);
            if !fwd_ok && !rev_ok {
                continue;
            }
            any_coincident = true;
            // Shared endpoints prove nothing about the span between them.
            if fwd_ok && curves_agree(topo, &free[i], &free[j], false, tolerance)? {
                candidates.push((j, false));
            } else if rev_ok && curves_agree(topo, &free[i], &free[j], true, tolerance)? {
                candidates.push((j, true));
            }
        }

        if candidates.len() == 1 {
            let (j, reversed) = candidates[0];
            if pcurve_keys_available(topo, free[i].id, free[j].id, reversed) {
                consumed[i] = true;
                consumed[j] = true;
                plans.push(Merge {
                    keep: free[i].id,
                    drop: free[j].id,
                    reversed,
                });
            } else {
                // Merging would put two uses of one edge on one face in the
                // same direction, which is not a manifold boundary.
                log::debug!(
                    "sew_shell: declining {:?}/{:?} — pcurve use key already occupied",
                    free[i].id,
                    free[j].id
                );
                declined += 1;
            }
        } else if candidates.len() > 1 {
            // More than one valid partner is a non-manifold junction. Picking
            // one would be arbitrary, so pick none — and consume the whole
            // group, or the leftovers would pair off by iteration order.
            log::debug!(
                "sew_shell: declining {:?} — {} coincident partners",
                free[i].id,
                candidates.len()
            );
            consumed[i] = true;
            for &(j, _) in &candidates {
                consumed[j] = true;
            }
            declined += 1;
        } else if any_coincident {
            log::debug!(
                "sew_shell: declining {:?} — endpoints match a partner but the curves do not",
                free[i].id
            );
            declined += 1;
        }
    }

    Ok((plans, declined))
}

/// Whether two free edges share both endpoints, forwards and/or reversed.
///
/// A closed edge (`start == end`) satisfies both; the interior sampling in
/// [`curves_agree`] is what settles the direction.
fn endpoints_coincide(a: &FreeEdge, b: &FreeEdge, tol_sq: f64) -> (bool, bool) {
    let fwd = (a.start_pos - b.start_pos).length_squared() < tol_sq
        && (a.end_pos - b.end_pos).length_squared() < tol_sq;
    let rev = (a.start_pos - b.end_pos).length_squared() < tol_sq
        && (a.end_pos - b.start_pos).length_squared() < tol_sq;
    (fwd, rev)
}

/// Whether two free edges describe the same 3D curve between their shared
/// endpoints, sampling `b` backwards when `reversed`.
fn curves_agree(
    topo: &Topology,
    a: &FreeEdge,
    b: &FreeEdge,
    reversed: bool,
    tolerance: f64,
) -> Result<bool, HealError> {
    let edge_a = topo.edge(a.id)?;
    let edge_b = topo.edge(b.id)?;
    let (a0, a1) = edge_a
        .strict_domain()
        .map_err(crate::error::upgrade_edge_domain)?;
    let (b0, b1) = edge_b
        .strict_domain()
        .map_err(crate::error::upgrade_edge_domain)?;

    for k in 1..=CURVE_SAMPLES {
        let frac = f64::from(k) / f64::from(CURVE_SAMPLES + 1);
        let frac_b = if reversed { 1.0 - frac } else { frac };
        let pa =
            edge_a
                .curve()
                .evaluate_with_endpoints(a0 + (a1 - a0) * frac, a.start_pos, a.end_pos);
        let pb =
            edge_b
                .curve()
                .evaluate_with_endpoints(b0 + (b1 - b0) * frac_b, b.start_pos, b.end_pos);
        if (pa - pb).length() > tolerance {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Whether every pcurve use of `drop` can be re-keyed onto `keep` without
/// displacing a use already stored there.
fn pcurve_keys_available(topo: &Topology, keep: EdgeId, drop: EdgeId, reversed: bool) -> bool {
    topo.pcurves_for_edge(drop)
        .iter()
        .all(|(face, forward, _)| {
            topo.pcurve_oriented(keep, *face, *forward != reversed)
                .is_none()
        })
}

/// Carry out the planned merges: pcurves, then wire uses, then vertices.
fn apply_merges(
    topo: &mut Topology,
    shell_id: ShellId,
    wire_ids: &[WireId],
    plans: &[Merge],
) -> Result<(), HealError> {
    // 1. Move each dropped edge's pcurves onto the retained edge. A pcurve's
    //    `[t_start, t_end]` tracks its edge's natural direction, so a
    //    reversed merge swaps the bounds as well as the use's sense.
    for merge in plans {
        let uses: Vec<(FaceId, bool, PCurve)> = topo
            .pcurves_for_edge(merge.drop)
            .into_iter()
            .map(|(face, forward, pcurve)| (face, forward, pcurve.clone()))
            .collect();
        for (face, forward, pcurve) in uses {
            topo.remove_pcurve_oriented(merge.drop, face, forward);
            let (pcurve, forward) = if merge.reversed {
                (
                    PCurve::new(pcurve.curve().clone(), pcurve.t_end(), pcurve.t_start()),
                    !forward,
                )
            } else {
                (pcurve, forward)
            };
            topo.set_pcurve_oriented(merge.keep, face, forward, pcurve);
        }
    }

    // 2. Rewrite every wire use of a dropped edge onto its retained twin.
    //    This — not rewriting the dropped edge's endpoints — is what makes
    //    two faces share one edge.
    let redirect: HashMap<EdgeId, (EdgeId, bool)> = plans
        .iter()
        .map(|merge| (merge.drop, (merge.keep, merge.reversed)))
        .collect();
    for &wire_id in wire_ids {
        let old: Vec<OrientedEdge> = topo.wire(wire_id)?.edges().to_vec();
        let mut changed = false;
        let new: Vec<OrientedEdge> = old
            .iter()
            .map(|oe| match redirect.get(&oe.edge()) {
                Some(&(keep, reversed)) => {
                    changed = true;
                    OrientedEdge::new(keep, oe.is_forward() != reversed)
                }
                None => *oe,
            })
            .collect();
        if changed {
            let closed = topo.wire(wire_id)?.is_closed();
            let replacement = remus_topology::wire::Wire::new(new, closed)?;
            topo.replace_boundary_wire(wire_id, replacement)?;
        }
    }

    // 3. Merge the dropped edges' vertices into the retained ones. Without
    //    this the neighbouring edges in each rewritten wire still terminate
    //    at their own copies and the chain is broken.
    let mut vertex_map: HashMap<VertexId, VertexId> = HashMap::new();
    for merge in plans {
        let keep = topo.edge(merge.keep)?;
        let (keep_start, keep_end) = if merge.reversed {
            (keep.end(), keep.start())
        } else {
            (keep.start(), keep.end())
        };
        let dropped = topo.edge(merge.drop)?;
        let (drop_start, drop_end) = (dropped.start(), dropped.end());
        union_vertices(&mut vertex_map, drop_start, keep_start);
        union_vertices(&mut vertex_map, drop_end, keep_end);
    }
    apply_vertex_map(topo, wire_ids, &vertex_map)?;

    // 4. Any face whose loops were derived now has a stale derivation
    //    (RFC 0002, Stage 1 keeps loops and wires in agreement).
    for face_id in topo.shell(shell_id)?.faces().to_vec() {
        if topo.loops_of_face(face_id).is_some() {
            topo.build_face_loops(face_id)?;
        }
    }

    Ok(())
}

/// Repoint every edge still referenced by the shell at its merged vertices.
fn apply_vertex_map(
    topo: &mut Topology,
    wire_ids: &[WireId],
    vertex_map: &HashMap<VertexId, VertexId>,
) -> Result<(), HealError> {
    if vertex_map.is_empty() {
        return Ok(());
    }

    let mut seen = HashSet::new();
    let mut edge_ids = Vec::new();
    for &wire_id in wire_ids {
        for oe in topo.wire(wire_id)?.edges() {
            if seen.insert(oe.edge()) {
                edge_ids.push(oe.edge());
            }
        }
    }

    let mut updates = Vec::new();
    for edge_id in edge_ids {
        let edge = topo.edge(edge_id)?;
        let start = resolve_vertex(vertex_map, edge.start());
        let end = resolve_vertex(vertex_map, edge.end());
        if start != edge.start() || end != edge.end() {
            updates.push((edge_id, start, end));
        }
    }

    for (edge_id, start, end) in updates {
        // `set_start`/`set_end` rather than rebuilding through `Edge::new`:
        // an edge's explicit trim and edge-specific tolerance are not
        // recoverable from its endpoints.
        let edge = topo.edge_mut(edge_id)?;
        edge.set_start(start);
        edge.set_end(end);
    }

    Ok(())
}

/// Follow a vertex through the merge chain to its surviving representative.
///
/// Links are only ever made root-to-root, so the map is a forest and the walk
/// cannot cycle. It is bounded by the map's own size — an exact bound, unlike
/// a fixed cap, which would silently return a mid-chain vertex and split a
/// point that had been merged.
fn resolve_vertex(vertex_map: &HashMap<VertexId, VertexId>, mut vertex: VertexId) -> VertexId {
    for _ in 0..vertex_map.len() {
        match vertex_map.get(&vertex) {
            Some(&next) => vertex = next,
            None => break,
        }
    }
    vertex
}

/// Record that `from` and `to` are the same point, linking their roots.
fn union_vertices(vertex_map: &mut HashMap<VertexId, VertexId>, from: VertexId, to: VertexId) {
    let from = resolve_vertex(vertex_map, from);
    let to = resolve_vertex(vertex_map, to);
    if from != to {
        vertex_map.insert(from, to);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    use remus_math::curves::Circle3D;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    const TOL: f64 = 1e-7;

    fn circle_edge(
        topo: &mut Topology,
        start: VertexId,
        end: VertexId,
        circle: Circle3D,
    ) -> EdgeId {
        let start_parameter = circle.project(topo.vertex(start).unwrap().point());
        let canonical_end = circle.project(topo.vertex(end).unwrap().point());
        let end_parameter = if start == end {
            start_parameter + std::f64::consts::TAU
        } else if canonical_end <= start_parameter {
            canonical_end + std::f64::consts::TAU
        } else {
            canonical_end
        };
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some((start_parameter, end_parameter)));
        topo.add_edge(edge)
    }

    /// Build a planar quad face from four corner points, allocating fresh
    /// vertices and edges for every corner. Independent allocation is the
    /// point: it is what a mesh import or a patch-by-patch build produces,
    /// and it is the input sewing exists to repair.
    fn quad_face(topo: &mut Topology, pts: [Point3; 4], normal: Vec3, d: f64) -> FaceId {
        let vs: Vec<_> = pts
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
            .collect();
        let es: Vec<_> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        let wire = Wire::new(
            es.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
    }

    /// Six faces of a unit cube, each built from its own vertices and edges,
    /// assembled into one shell. Geometrically closed, topologically 24
    /// separate free edges — exactly the shape `sew_shell` is meant to close.
    fn disjoint_cube_shell(topo: &mut Topology) -> ShellId {
        let p = Point3::new;
        let faces = vec![
            // bottom, -Z
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 0.0),
                    p(0.0, 1.0, 0.0),
                    p(1.0, 1.0, 0.0),
                    p(1.0, 0.0, 0.0),
                ],
                Vec3::new(0.0, 0.0, -1.0),
                0.0,
            ),
            // top, +Z
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 1.0),
                    p(1.0, 0.0, 1.0),
                    p(1.0, 1.0, 1.0),
                    p(0.0, 1.0, 1.0),
                ],
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
            ),
            // front, -Y
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 0.0),
                    p(1.0, 0.0, 0.0),
                    p(1.0, 0.0, 1.0),
                    p(0.0, 0.0, 1.0),
                ],
                Vec3::new(0.0, -1.0, 0.0),
                0.0,
            ),
            // back, +Y
            quad_face(
                topo,
                [
                    p(0.0, 1.0, 0.0),
                    p(0.0, 1.0, 1.0),
                    p(1.0, 1.0, 1.0),
                    p(1.0, 1.0, 0.0),
                ],
                Vec3::new(0.0, 1.0, 0.0),
                1.0,
            ),
            // left, -X
            quad_face(
                topo,
                [
                    p(0.0, 0.0, 0.0),
                    p(0.0, 0.0, 1.0),
                    p(0.0, 1.0, 1.0),
                    p(0.0, 1.0, 0.0),
                ],
                Vec3::new(-1.0, 0.0, 0.0),
                0.0,
            ),
            // right, +X
            quad_face(
                topo,
                [
                    p(1.0, 0.0, 0.0),
                    p(1.0, 1.0, 0.0),
                    p(1.0, 1.0, 1.0),
                    p(1.0, 0.0, 1.0),
                ],
                Vec3::new(1.0, 0.0, 0.0),
                1.0,
            ),
        ];
        topo.add_shell(Shell::new(faces).unwrap())
    }

    fn free_edge_count(topo: &Topology, shell_id: ShellId) -> usize {
        crate::analysis::free_bounds::find_free_bounds(topo, shell_id)
            .unwrap()
            .iter()
            .map(Vec::len)
            .sum()
    }

    /// Every wire in the shell must be a connected closed chain: each edge's
    /// oriented end vertex is the next edge's oriented start vertex.
    fn assert_wires_chain(topo: &Topology, shell_id: ShellId) {
        for &fid in topo.shell(shell_id).unwrap().faces() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                let wire = topo.wire(wid).unwrap();
                remus_topology::validation::validate_wire_closed(wire, topo).unwrap_or_else(|e| {
                    panic!("wire {wid:?} on face {fid:?} is not a valid closed chain: {e}")
                });
                // Identity-level check: position-coincident but distinct
                // vertex IDs are accepted by `validate_wire_closed`, and
                // sewing is exactly the operation that must remove them.
                let oes = wire.edges();
                for k in 0..oes.len() {
                    let cur = oes[k];
                    let next = oes[(k + 1) % oes.len()];
                    let cur_e = topo.edge(cur.edge()).unwrap();
                    let next_e = topo.edge(next.edge()).unwrap();
                    assert_eq!(
                        cur.oriented_end(cur_e),
                        next.oriented_start(next_e),
                        "wire {wid:?} on face {fid:?}: edge {:?} ends at a different vertex than \
                         edge {:?} starts",
                        cur.edge(),
                        next.edge()
                    );
                }
            }
        }
    }

    #[test]
    fn sew_shell_closes_a_disjoint_cube_shell() {
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);

        let before = free_edge_count(&topo, shell_id);
        assert_eq!(before, 24, "every edge of the disjoint cube starts free");

        let sewn = sew_shell(&mut topo, shell_id, 1e-6).unwrap();

        let after = free_edge_count(&topo, shell_id);

        assert_eq!(
            after, 0,
            "sew_shell reported {sewn} edges sewn but left {after} free edges"
        );
        assert_eq!(sewn, 12, "a cube has 12 edges to sew");

        let shell = topo.shell(shell_id).unwrap();
        remus_topology::validation::validate_shell_closed(shell, &topo)
            .expect("sewn cube shell must be a closed 2-manifold");
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_leaves_every_wire_a_connected_chain() {
        // Independent of the free-edge count: whatever `sew_shell` decides to
        // do, it must not leave a wire whose consecutive edges terminate at
        // different vertices. Rewriting one edge's endpoints in isolation
        // does exactly that.
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);
        assert_wires_chain(&topo, shell_id);

        sew_shell(&mut topo, shell_id, 1e-6).unwrap();

        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_preserves_trim_and_tolerance_of_retained_edges() {
        // Sewing rewrites wire membership and merges coincident vertices. It
        // must not rebuild edges from scratch on the way: an edge's explicit
        // trim (RFC 0002) and edge-specific tolerance are not recoverable
        // from its endpoints.
        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);

        // Stamp every edge in the shell with a distinguishable trim and
        // tolerance. A `Line` reads its domain as [0, 1]; the trim below is
        // deliberately different so a drop is visible.
        let mut edge_ids = Vec::new();
        for &fid in topo.shell(shell_id).unwrap().faces() {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                edge_ids.push(oe.edge());
            }
        }
        for &eid in &edge_ids {
            let e = topo.edge_mut(eid).unwrap();
            e.set_trim(Some((0.0, 1.0)));
            e.set_tolerance(Some(3.5e-8)).unwrap();
        }

        sew_shell(&mut topo, shell_id, 1e-6).unwrap();

        // Check the edges that survived into the sewn wires.
        let mut checked = 0;
        for &fid in topo.shell(shell_id).unwrap().faces() {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                assert_eq!(
                    e.trim(),
                    Some((0.0, 1.0)),
                    "edge {:?} lost its explicit trim during sewing",
                    oe.edge()
                );
                assert_eq!(
                    e.tolerance(),
                    Some(3.5e-8),
                    "edge {:?} lost its edge-specific tolerance during sewing",
                    oe.edge()
                );
                checked += 1;
            }
        }
        assert_eq!(checked, 24, "all 24 edge uses inspected");
    }

    /// A quad face whose edges are supplied by the caller, so a test can
    /// hand it a specific curve or reuse a specific vertex.
    fn quad_face_from_edges(
        topo: &mut Topology,
        edges: [EdgeId; 4],
        normal: Vec3,
        d: f64,
    ) -> FaceId {
        let wire = Wire::new(
            edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
    }

    fn line_loop(
        topo: &mut Topology,
        pts: [Point3; 4],
    ) -> ([EdgeId; 4], Vec<remus_topology::vertex::VertexId>) {
        let vs: Vec<_> = pts
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
            .collect();
        let es: Vec<EdgeId> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        ([es[0], es[1], es[2], es[3]], vs)
    }

    #[test]
    fn sew_shell_flips_the_traversal_sense_on_a_reversed_match() {
        // Two coplanar squares meeting along x = 1, each wound CCW. The
        // shared segment is traversed (1,0,0)->(1,1,0) by the left face and
        // (1,1,0)->(1,0,0) by the right one — the ordinary manifold case,
        // and a reversed endpoint match.
        let p = Point3::new;
        let mut topo = Topology::new();
        let n = Vec3::new(0.0, 0.0, 1.0);

        let (left_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let (right_edges, _) = line_loop(
            &mut topo,
            [
                p(1.0, 0.0, 0.0),
                p(2.0, 0.0, 0.0),
                p(2.0, 1.0, 0.0),
                p(1.0, 1.0, 0.0),
            ],
        );
        let seam_left = left_edges[1]; // (1,0,0) -> (1,1,0)
        let seam_right = right_edges[3]; // (1,1,0) -> (1,0,0)

        let fl = quad_face_from_edges(&mut topo, left_edges, n, 0.0);
        let fr = quad_face_from_edges(&mut topo, right_edges, n, 0.0);
        let shell_id = topo.add_shell(Shell::new(vec![fl, fr]).unwrap());

        assert_eq!(free_edge_count(&topo, shell_id), 8);
        let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();
        assert_eq!(
            report,
            SewReport {
                sewn: 1,
                declined: 0
            }
        );
        assert_eq!(free_edge_count(&topo, shell_id), 6, "one seam closed");

        // The retained edge appears in both wires, and the face that lost its
        // copy now traverses the retained edge backwards.
        let keep = seam_left.min(seam_right);
        let dropped = seam_left.max(seam_right);
        let mut senses = Vec::new();
        for fid in [fl, fr] {
            let wid = topo.face(fid).unwrap().outer_wire();
            for oe in topo.wire(wid).unwrap().edges() {
                assert_ne!(oe.edge(), dropped, "dropped edge still referenced");
                if oe.edge() == keep {
                    senses.push(oe.is_forward());
                }
            }
        }
        assert_eq!(senses.len(), 2, "the retained edge is used by both faces");
        assert_ne!(
            senses[0], senses[1],
            "a reversed match must flip the rewritten use's sense"
        );
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_declines_when_the_curves_between_shared_endpoints_disagree() {
        // Both edges run between (0,0,0) and (1,0,0); one is the chord, the
        // other the semicircular arc. Coincident endpoints, different curves.
        let p = Point3::new;
        let mut topo = Topology::new();

        let (top_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let v_a = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let v_b = topo.add_vertex(Vertex::new(p(1.0, -1.0, 0.0), TOL));
        let v_c = topo.add_vertex(Vertex::new(p(0.0, -1.0, 0.0), TOL));
        let v_d = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let arc = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let bottom_edges = [
            topo.add_edge(Edge::new(v_a, v_b, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_b, v_c, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_c, v_d, EdgeCurve::Line)),
            // (0,0,0) -> (1,0,0) along the arc, not the chord.
            circle_edge(&mut topo, v_d, v_a, arc),
        ];

        let n = Vec3::new(0.0, 0.0, 1.0);
        let ft = quad_face_from_edges(&mut topo, top_edges, n, 0.0);
        let fb = quad_face_from_edges(&mut topo, bottom_edges, n, 0.0);
        let shell_id = topo.add_shell(Shell::new(vec![ft, fb]).unwrap());

        let before = free_edge_count(&topo, shell_id);
        let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();

        assert_eq!(
            report,
            SewReport {
                sewn: 0,
                declined: 1
            },
            "the chord and the arc must not be reported as sewn"
        );
        assert_eq!(
            free_edge_count(&topo, shell_id),
            before,
            "declining must leave the shell exactly as it was"
        );
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shell_declines_an_ambiguous_partner() {
        // Three faces meet along one segment. Any two of them could be sewn;
        // choosing a pair would be an arbitrary answer to a non-manifold
        // junction, so none of the three is touched.
        let p = Point3::new;
        let mut topo = Topology::new();
        let fans = [
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, -1.0, 0.0),
                p(0.0, -1.0, 0.0),
            ],
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 0.0, 1.0),
                p(0.0, 0.0, 1.0),
            ],
        ];
        let mut faces = Vec::new();
        for pts in fans {
            let (edges, _) = line_loop(&mut topo, pts);
            faces.push(quad_face_from_edges(
                &mut topo,
                edges,
                Vec3::new(0.0, 0.0, 1.0),
                0.0,
            ));
        }
        let shell_id = topo.add_shell(Shell::new(faces).unwrap());

        let before = free_edge_count(&topo, shell_id);
        let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();

        assert_eq!(report.sewn, 0, "an ambiguous junction must not be sewn");
        assert_eq!(report.declined, 1);
        assert_eq!(free_edge_count(&topo, shell_id), before);
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shells_operator_closes_the_shell_through_the_pipeline() {
        // The consumer-reachable path: `sew_shells` is registered in the
        // operator registry and named in the WASM heal bindings, so a JS
        // caller reaches this code by name.
        use crate::pipeline::process::HealProcess;

        let mut topo = Topology::new();
        let shell_id = disjoint_cube_shell(&mut topo);
        let solid_id = topo.add_solid(remus_topology::solid::Solid::new(shell_id, vec![]));

        assert_eq!(free_edge_count(&topo, shell_id), 24);

        let mut process = HealProcess::new();
        process.add_step("sew_shells");
        let (_, results) = process.execute(&mut topo, solid_id).unwrap();

        assert_eq!(results.len(), 1);
        assert_eq!(results[0].actions_taken, 12);
        assert!(
            results[0].status.contains(crate::status::Status::DONE1),
            "a successful sew must report DONE1, got {:?}",
            results[0].status
        );
        assert!(
            !results[0].status.is_fail(),
            "nothing was declined, so no FAIL flag belongs here"
        );
        assert_eq!(
            free_edge_count(&topo, shell_id),
            0,
            "the pipeline operator must actually close the shell"
        );
        assert_wires_chain(&topo, shell_id);
    }

    #[test]
    fn sew_shells_operator_reports_fail_when_it_declines() {
        // Reporting DONE on a shell it could not close is the defect this
        // whole change exists to remove.
        use crate::pipeline::process::HealProcess;

        let p = Point3::new;
        let mut topo = Topology::new();

        let (top_edges, _) = line_loop(
            &mut topo,
            [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ],
        );
        let v_a = topo.add_vertex(Vertex::new(p(1.0, 0.0, 0.0), TOL));
        let v_b = topo.add_vertex(Vertex::new(p(1.0, -1.0, 0.0), TOL));
        let v_c = topo.add_vertex(Vertex::new(p(0.0, -1.0, 0.0), TOL));
        let v_d = topo.add_vertex(Vertex::new(p(0.0, 0.0, 0.0), TOL));
        let arc = Circle3D::new(p(0.5, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5).unwrap();
        let bottom_edges = [
            topo.add_edge(Edge::new(v_a, v_b, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_b, v_c, EdgeCurve::Line)),
            topo.add_edge(Edge::new(v_c, v_d, EdgeCurve::Line)),
            circle_edge(&mut topo, v_d, v_a, arc),
        ];
        let n = Vec3::new(0.0, 0.0, 1.0);
        let ft = quad_face_from_edges(&mut topo, top_edges, n, 0.0);
        let fb = quad_face_from_edges(&mut topo, bottom_edges, n, 0.0);
        let shell_id = topo.add_shell(Shell::new(vec![ft, fb]).unwrap());
        let solid_id = topo.add_solid(remus_topology::solid::Solid::new(shell_id, vec![]));

        let mut process = HealProcess::new();
        process.add_step("sew_shells");
        let (_, results) = process.execute(&mut topo, solid_id).unwrap();

        assert_eq!(results[0].actions_taken, 0);
        assert!(
            results[0].status.is_fail(),
            "declining every candidate must not be reported as success, got {:?}",
            results[0].status
        );
        assert!(!results[0].status.is_done());
    }

    #[test]
    fn sew_shell_closes_a_seam_of_closed_circular_edges() {
        // Closed edges (`start == end`) are the periodic case: both endpoint
        // orientations match trivially, so only the interior sampling can
        // tell whether the two rims run the same way. Two discs sharing one
        // rim is the smallest shape that exercises it.
        let p = Point3::new;
        let mut topo = Topology::new();
        let circle = Circle3D::new(p(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();

        let mut disc = |normal: Vec3| {
            let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), TOL));
            let rim = circle_edge(&mut topo, seam, seam, circle.clone());
            let wire = Wire::new(vec![OrientedEdge::new(rim, true)], true).unwrap();
            let wid = topo.add_wire(wire);
            topo.add_face(Face::new(
                wid,
                vec![],
                FaceSurface::Plane { normal, d: 0.0 },
            ))
        };
        let top = disc(Vec3::new(0.0, 0.0, 1.0));
        let bottom = disc(Vec3::new(0.0, 0.0, -1.0));
        let shell_id = topo.add_shell(Shell::new(vec![top, bottom]).unwrap());

        assert_eq!(free_edge_count(&topo, shell_id), 2, "two unshared rims");

        let report = sew_shell_report(&mut topo, shell_id, 1e-6).unwrap();
        assert_eq!(
            report,
            SewReport {
                sewn: 1,
                declined: 0
            }
        );
        assert_eq!(
            free_edge_count(&topo, shell_id),
            0,
            "the closed rim must end up shared by both discs"
        );
        remus_topology::validation::validate_shell_closed(topo.shell(shell_id).unwrap(), &topo)
            .expect("sewn rim leaves a closed shell");
        assert_wires_chain(&topo, shell_id);
    }
}
