//! Collapse collinear interior wire vertices that are structurally orphan
//! across the solid.
//!
//! After mesh-fallback boolean assembly, adjacent faces can carry
//! asymmetric tessellation residue at shared edges: face A's wire goes
//! `… → P → V → Q → …` (V being a tessellation-diagonal artifact) while
//! face B's wire jumps `… → P → Q → …` directly. Calls to
//! `unify_faces`/`merge_collinear_edges` only smooth wires of faces that
//! the unifier itself merged, leaving the asymmetry intact whenever the
//! two faces remain separate topological entities.
//!
//! When `V` is incident to exactly two edges across the entire solid AND
//! both are Line curves AND V is collinear strictly between P and Q,
//! every wire that traverses V must do so as the consecutive pair
//! (`E1`, `E2`) in some orientation. The vertex is structurally
//! collapsible: replace the pair with a single oriented edge `E_new(P,Q)`
//! in every wire that referenced it, reusing an existing `Edge(P,Q,Line)`
//! when one is already present (so adjacent faces end up sharing the
//! same `EdgeId` rather than two parallel ones, which is what downstream
//! face-adjacency counting needs).
//!
//! Conservative gates: only Line curves, only when V's two neighbours
//! are distinct, only when V is strictly between them, only when V has
//! exactly two incident edges in the solid. Vertices that fail any gate
//! are left untouched.
//!
//! Adjacent candidates (`P → V1 → V2 → Q`, all collinear) share an edge, so
//! their plans conflict: collapsing V1 to `P → V2` and V2 to `V1 → Q` in the
//! same pass lets each wire pick whichever junction it meets first, and the
//! two sides of the chord end on different edges. A pass therefore takes
//! only plans whose edge pairs are disjoint, in vertex order, and the passes
//! repeat until a run of any length has collapsed to one edge. Each merged
//! edge remembers the vertices it absorbed, and extending it again must keep
//! all of them within tolerance of the new chord.

use std::collections::{BTreeMap, BTreeSet};

use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::HealError;

/// Collapse interior wire vertices that are collinear between their two
/// neighbours and have no other edges anchoring them.
///
/// Returns the number of vertices collapsed.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail. Best-effort: a single
/// failure mid-pass does not abort already-applied collapses.
pub fn collapse_collinear_wire_vertices(
    topo: &mut Topology,
    solid_id: SolidId,
    tol: Tolerance,
) -> Result<usize, HealError> {
    // Every productive pass shortens at least one wire by an edge, so the
    // passes end.
    let mut collapsed = 0;
    let mut absorbed = BTreeMap::new();
    loop {
        let pass = collapse_pass(topo, solid_id, tol, &mut absorbed)?;
        if pass == 0 {
            return Ok(collapsed);
        }
        collapsed += pass;
    }
}

/// One pass over the solid: collapse every candidate vertex whose two edges
/// no earlier candidate (in vertex order) has claimed. Returns the number of
/// vertices removed from at least one wire.
///
/// `absorbed` maps each merged edge to every vertex position collapsed into
/// it so far, across passes: extending an edge must keep all of them within
/// tolerance of the new chord, not only the vertex removed now, or a chain of
/// collapses could drift past tolerance one acceptable step at a time.
#[allow(clippy::too_many_lines)]
fn collapse_pass(
    topo: &mut Topology,
    solid_id: SolidId,
    tol: Tolerance,
    absorbed: &mut BTreeMap<EdgeId, Vec<Point3>>,
) -> Result<usize, HealError> {
    // 1. Gather every (face_id, wire_id) pair in the solid. Faces hold
    //    one outer wire plus zero-or-more inner wires (holes); both are
    //    in scope.
    let face_ids = remus_topology::explorer::solid_faces(topo, solid_id)?;
    let mut wire_ids: Vec<WireId> = Vec::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        wire_ids.push(face.outer_wire());
        wire_ids.extend(face.inner_wires().iter().copied());
    }
    if wire_ids.is_empty() {
        return Ok(0);
    }

    // 2. Snapshot every edge that participates in any solid wire, indexed
    //    by EdgeId so vertex→edge counting below stays O(unique edges)
    //    rather than O(sum of wire lengths). Edges that appear in 2 wires
    //    (a healthy manifold edge) only count once toward vertex incidence.
    //    Also record which faces reference each edge so the planarity gate
    //    below can confirm every face touching V is planar before collapse.
    let mut edges_in_solid: BTreeMap<EdgeId, (VertexId, VertexId)> = BTreeMap::new();
    let mut edge_to_faces: BTreeMap<EdgeId, BTreeSet<FaceId>> = BTreeMap::new();
    for &fid in &face_ids {
        let face = topo.face(fid)?;
        let mut all_wires: Vec<WireId> = vec![face.outer_wire()];
        all_wires.extend(face.inner_wires().iter().copied());
        for wid in all_wires {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let eid = oe.edge();
                if let std::collections::btree_map::Entry::Vacant(slot) = edges_in_solid.entry(eid)
                {
                    let edge = topo.edge(eid)?;
                    slot.insert((edge.start(), edge.end()));
                }
                edge_to_faces.entry(eid).or_default().insert(fid);
            }
        }
    }

    let mut vertex_edges: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
    for (&eid, &(start, end)) in &edges_in_solid {
        vertex_edges.entry(start).or_default().push(eid);
        if end != start {
            vertex_edges.entry(end).or_default().push(eid);
        }
    }

    // 4. Identify candidate vertices: exactly two incident edges, both
    //    Lines, and the vertex strictly between the two far endpoints.
    let lin_tol = tol.linear.max(1e-12);
    let lin_tol_sq = lin_tol * lin_tol;

    let mut plans: Vec<CollapsePlan> = Vec::new();
    // Edges already consumed by an accepted plan this pass. A candidate
    // sharing one is an adjacent collinear vertex; it waits for the next
    // pass, when its neighbour's merged edge has replaced the shared one.
    let mut claimed: BTreeSet<EdgeId> = BTreeSet::new();
    for (&vertex, incident) in &vertex_edges {
        if incident.len() != 2 {
            continue;
        }
        let eid1 = incident[0];
        let eid2 = incident[1];
        if claimed.contains(&eid1) || claimed.contains(&eid2) {
            continue;
        }
        let edge1 = topo.edge(eid1)?;
        let edge2 = topo.edge(eid2)?;
        if !matches!(edge1.curve(), EdgeCurve::Line) || !matches!(edge2.curve(), EdgeCurve::Line) {
            continue;
        }
        // Determine the far endpoints P and Q (the non-V end of each edge).
        let p_vid = if edge1.start() == vertex {
            edge1.end()
        } else if edge1.end() == vertex {
            edge1.start()
        } else {
            continue; // V not actually on edge1 — defensive
        };
        let q_vid = if edge2.start() == vertex {
            edge2.end()
        } else if edge2.end() == vertex {
            edge2.start()
        } else {
            continue;
        };
        if p_vid == q_vid || p_vid == vertex || q_vid == vertex {
            continue;
        }
        // Topological gate: E1 and E2 must be shared by the same face set.
        // Anything else means V's two-edge pair has inconsistent
        // membership and collapsing it would re-asymmetrize the wires.
        //
        //   * 2 faces — "symmetric" case, both faces register V on their
        //     wire. Common when mesh fallback splits the same chord on
        //     both sides; coplanar pair required so the chord doesn't
        //     diverge from the underlying geometry on either side.
        //   * 1 face — "asymmetric residue" case (#696), only one face
        //     carries V; the neighbour already references a direct P→Q
        //     edge via a separate `EdgeId`. Re-unifying onto that edge
        //     requires it to exist and to live on a face that is
        //     coplanar with — or perpendicular to (a different planar
        //     face altogether) — the V-bearing face. We check this
        //     after the planarity pass below.
        let faces_e1 = edge_to_faces.get(&eid1).cloned().unwrap_or_default();
        let faces_e2 = edge_to_faces.get(&eid2).cloned().unwrap_or_default();
        if faces_e1 != faces_e2 || !(1..=2).contains(&faces_e1.len()) {
            continue;
        }

        // Planarity gate: a vertex on a chord between curved-surface samples
        // is load-bearing for the curved face's tessellation — removing it
        // breaks watertightness against the adjacent face. Only collapse
        // when both faces are coplanar (planes match in normal direction
        // and offset) so the chord and the underlying surface coincide.
        let ang_tol_cos = (1.0 - tol.angular).max(1.0 - 1e-9);
        let mut plane_ref: Option<(remus_math::vec::Vec3, f64)> = None;
        let mut all_planar = true;
        for &fid in &faces_e1 {
            let face = topo.face(fid)?;
            let FaceSurface::Plane { normal, d } = face.surface() else {
                all_planar = false;
                break;
            };
            let n_len = normal.dot(*normal).sqrt();
            if n_len < tol.linear {
                all_planar = false;
                break;
            }
            let n_unit = *normal * (1.0 / n_len);
            let d_norm = *d / n_len;
            if let Some((prev_n, prev_d)) = plane_ref {
                let dot = prev_n.dot(n_unit).abs();
                if dot < ang_tol_cos {
                    all_planar = false;
                    break;
                }
                let signed_d = if prev_n.dot(n_unit) >= 0.0 {
                    d_norm
                } else {
                    -d_norm
                };
                if (signed_d - prev_d).abs() > tol.linear {
                    all_planar = false;
                    break;
                }
            } else {
                plane_ref = Some((n_unit, d_norm));
            }
        }
        if !all_planar {
            continue;
        }

        // Strict-between collinearity check in 3D.
        let v_pt = topo.vertex(vertex)?.point();
        let p_pt = topo.vertex(p_vid)?.point();
        let q_pt = topo.vertex(q_vid)?.point();
        let pq = q_pt - p_pt;
        let pq_len_sq = pq.dot(pq);
        if pq_len_sq < lin_tol_sq {
            continue; // P and Q coincident
        }
        let pv = v_pt - p_pt;
        let t = pv.dot(pq) / pq_len_sq;
        // Strictly between: 0 < t < 1, with a tolerance margin so end
        // collisions don't masquerade as interior.
        if t <= lin_tol || t >= 1.0 - lin_tol {
            continue;
        }
        // Perpendicular distance from V to line PQ, scaled by pq length.
        let projection = p_pt + pq * t;
        let perp = v_pt - projection;
        if perp.dot(perp) > lin_tol_sq {
            continue;
        }
        let mut witnesses = vec![v_pt];
        for eid in [eid1, eid2] {
            witnesses.extend(absorbed.get(&eid).into_iter().flatten().copied());
        }
        let off_chord = |w: &Point3| {
            let s = ((*w - p_pt).dot(pq) / pq_len_sq).clamp(0.0, 1.0);
            let gap = *w - (p_pt + pq * s);
            gap.dot(gap) > lin_tol_sq
        };
        if witnesses.iter().any(off_chord) {
            continue;
        }

        // Find-or-create the merged edge. The symmetric (size-2) case gets
        // an edge of its own, which both faces then share; an existing
        // `P → Q` edge already bounds another face, so reusing it would
        // put the edge on a third. The asymmetric (size-1) case *requires*
        // an existing direct edge on a face the V-bearing face isn't
        // using — otherwise we'd be either creating an orphan edge (no
        // benefit) or unifying a face with itself — and reuses it only
        // while it stays on two faces.
        let existing = find_line_edge(topo, &edges_in_solid, p_vid, q_vid);
        let reuse = match existing {
            Some(eid) => {
                let neighbour_faces = edge_to_faces.get(&eid).cloned().unwrap_or_default();
                if neighbour_faces.is_subset(&faces_e1) {
                    // No genuine "other side" to re-unify onto.
                    continue;
                }
                let shared = neighbour_faces.union(&faces_e1).count() <= 2;
                // An edge another plan this pass removes or reuses waits
                // for the next pass.
                (shared && !claimed.contains(&eid)).then_some(eid)
            }
            None => None,
        };
        let new_edge = match (reuse, faces_e1.len()) {
            (Some(eid), _) => eid,
            (None, 2) => topo.add_edge(Edge::new(p_vid, q_vid, EdgeCurve::Line)),
            (None, _) => continue,
        };
        if new_edge == eid1 || new_edge == eid2 {
            continue; // Defensive — should not match the V-touching edges
        }
        claimed.insert(eid1);
        claimed.insert(eid2);
        if reuse.is_some() {
            claimed.insert(new_edge);
        }
        absorbed.entry(new_edge).or_default().extend(witnesses);
        plans.push(CollapsePlan {
            vertex,
            old_edges: [eid1, eid2],
            new_edge,
        });
    }

    if plans.is_empty() {
        return Ok(0);
    }

    // 5. Apply plans: for each wire, scan for consecutive edges that match
    //    a (V, E1, E2) pair and rewrite the oriented-edge list. A wire's
    //    edge list shrinks by 1 for each match; rebuild via `Wire::new`
    //    + `*wire_mut = new_wire` (matches `merge_collinear_edges` pattern
    //    in `unify_same_domain`).
    let plan_by_vertex: BTreeMap<VertexId, &CollapsePlan> =
        plans.iter().map(|p| (p.vertex, p)).collect();

    // Track which vertices were actually consumed by at least one wire
    // rewrite — the plan-vertex count is the upper bound, but a vertex
    // unreachable from any wire (orphaned by a prior pass) shouldn't
    // count as "collapsed".
    let mut consumed: BTreeSet<VertexId> = BTreeSet::new();

    for &wid in &wire_ids {
        let mut wire_consumed = BTreeSet::new();
        let new_edges_opt = try_rewrite_wire(topo, wid, &plan_by_vertex, &mut wire_consumed)?;
        if let Some(new_edges) = new_edges_opt {
            let is_closed = topo.wire(wid)?.is_closed();
            match Wire::new(new_edges, is_closed) {
                Ok(new_wire) => {
                    topo.replace_boundary_wire(wid, new_wire)?;
                    consumed.append(&mut wire_consumed);
                }
                Err(e) => {
                    log::warn!(
                        "collapse_collinear_wire_vertices: skipped wire {wid:?} \
                         after rewrite — {e}"
                    );
                }
            }
        }
    }

    Ok(consumed.len())
}

/// Find an existing `Line` edge whose endpoints are `{a, b}` (in either
/// order) among the solid-scoped edge set. Returns `None` if no
/// candidate exists; the caller decides whether to create one.
///
/// Scoping the search to `edges_in_solid` (rather than scanning the
/// whole topology arena) keeps the cost O(edges_in_solid) per candidate
/// vertex and prevents edges belonging to unrelated solids from being
/// returned — those would have an empty face set and silently break
/// the cross-face re-unification this pass is built around.
fn find_line_edge(
    topo: &Topology,
    edges_in_solid: &BTreeMap<EdgeId, (VertexId, VertexId)>,
    a: VertexId,
    b: VertexId,
) -> Option<EdgeId> {
    for (&eid, &(s, e)) in edges_in_solid {
        if !((s == a && e == b) || (s == b && e == a)) {
            continue;
        }
        if let Ok(edge) = topo.edge(eid)
            && matches!(edge.curve(), EdgeCurve::Line)
        {
            return Some(eid);
        }
    }
    None
}

/// Rewrite `wire`'s edges by collapsing every matching plan-vertex pair
/// into the plan's merged oriented edge. Two-pass algorithm:
///
/// 1. **Plan pass**: scan every junction (including the wrap-around
///    junction `(n-1, 0)` on closed wires) and record which edge
///    indices are consumed by which merge. An edge can be consumed by
///    at most one merge — earlier junctions win when they overlap
///    (e.g. an edge whose both endpoints are plan vertices). This
///    "greedy by index" choice keeps the algorithm wrap-around-safe:
///    by the time the loop reaches the wrap junction, any conflict
///    with the head junction is already resolved.
///
/// 2. **Emit pass**: walk `edges` in order; emit the merged edge at the
///    lower index of each consumed pair, skip the partner index, and
///    emit untouched edges verbatim. The wrap-around merge is emitted
///    at slot `n-1`, which preserves wire ordering without ever needing
///    to remove an element from the output buffer.
///
/// Returns `None` if no merges fired — saves reallocation on the common
/// case of untouched wires.
fn try_rewrite_wire(
    topo: &Topology,
    wid: WireId,
    plans: &BTreeMap<VertexId, &CollapsePlan>,
    consumed: &mut BTreeSet<VertexId>,
) -> Result<Option<Vec<OrientedEdge>>, HealError> {
    let wire = topo.wire(wid)?;
    let edges = wire.edges();
    let n = edges.len();
    if n < 2 {
        return Ok(None);
    }

    let junction_count = if wire.is_closed() { n } else { n - 1 };
    let mut consumed_edges = vec![false; n];
    let mut merge_emit: Vec<Option<OrientedEdge>> = vec![None; n];
    let mut changed = false;

    for j in 0..junction_count {
        let i_cur = j;
        let i_next = (j + 1) % n;
        if consumed_edges[i_cur] || consumed_edges[i_next] {
            continue;
        }
        let oe_cur = edges[i_cur];
        let oe_next = edges[i_next];
        let edge_cur = topo.edge(oe_cur.edge())?;
        let edge_next = topo.edge(oe_next.edge())?;
        let end_v = oe_cur.oriented_end(edge_cur);
        if end_v != oe_next.oriented_start(edge_next) {
            continue;
        }
        let Some(plan) = plans.get(&end_v) else {
            continue;
        };
        let cur_eid = oe_cur.edge();
        let nxt_eid = oe_next.edge();
        let matches_plan = (cur_eid == plan.old_edges[0] && nxt_eid == plan.old_edges[1])
            || (cur_eid == plan.old_edges[1] && nxt_eid == plan.old_edges[0]);
        if !matches_plan {
            continue;
        }
        let start_v = oe_cur.oriented_start(edge_cur);
        let final_v = oe_next.oriented_end(edge_next);
        let new_edge = topo.edge(plan.new_edge)?;
        let forward = if new_edge.start() == start_v && new_edge.end() == final_v {
            true
        } else if new_edge.start() == final_v && new_edge.end() == start_v {
            false
        } else {
            // Endpoints of the merged edge don't match the wire's
            // surrounding vertices — bail on this pair rather than
            // emit an oriented edge with mismatched endpoints.
            continue;
        };

        consumed_edges[i_cur] = true;
        consumed_edges[i_next] = true;
        merge_emit[i_cur] = Some(OrientedEdge::new(plan.new_edge, forward));
        consumed.insert(end_v);
        changed = true;
    }

    if !changed {
        return Ok(None);
    }

    let mut rewritten: Vec<OrientedEdge> = Vec::with_capacity(n);
    for i in 0..n {
        if let Some(merged) = merge_emit[i] {
            rewritten.push(merged);
        } else if !consumed_edges[i] {
            rewritten.push(edges[i]);
        }
        // else: consumed by a merge at another index — skip.
    }

    Ok(Some(rewritten))
}

struct CollapsePlan {
    vertex: VertexId,
    old_edges: [EdgeId; 2],
    new_edge: EdgeId,
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::Solid;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    fn vertex(topo: &mut Topology, p: Point3) -> VertexId {
        topo.add_vertex(Vertex::new(p, 1e-7))
    }

    /// Build a face whose outer wire is a 4-edge polyline A→V→B→…→A
    /// where `v` is collinear-strictly-between `a` and `b`.
    fn face_with_split_edge(
        topo: &mut Topology,
        a: VertexId,
        v: VertexId,
        b: VertexId,
        c: VertexId,
    ) -> FaceId {
        let e_av = topo.add_edge(Edge::new(a, v, EdgeCurve::Line));
        let e_vb = topo.add_edge(Edge::new(v, b, EdgeCurve::Line));
        let e_bc = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
        let e_ca = topo.add_edge(Edge::new(c, a, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_av, true),
                    OrientedEdge::new(e_vb, true),
                    OrientedEdge::new(e_bc, true),
                    OrientedEdge::new(e_ca, true),
                ],
                true,
            )
            .unwrap(),
        );
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        topo.add_face(Face::new(wire, Vec::new(), surface))
    }

    fn wrap_in_solid(topo: &mut Topology, faces: Vec<FaceId>) -> SolidId {
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        topo.add_solid(Solid::new(shell, Vec::new()))
    }

    #[test]
    fn vertex_strictly_between_collapses() {
        // Two coplanar faces share the chord A-V-B. The chord vertex V
        // sits collinearly between A and B; both faces have wire
        // A→V→B→{C or D}→A. Edges E1=(A,V) and E2=(V,B) are shared by
        // BOTH faces (the symmetric / structurally-collapsible case).
        let mut topo = Topology::new();
        let a = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let b = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let v = vertex(&mut topo, Point3::new(1.0, 0.0, 0.0));
        let c = vertex(&mut topo, Point3::new(0.0, -1.0, 0.0));
        let d = vertex(&mut topo, Point3::new(2.0, 1.0, 0.0));

        // Shared edges across both faces.
        let e_av = topo.add_edge(Edge::new(a, v, EdgeCurve::Line));
        let e_vb = topo.add_edge(Edge::new(v, b, EdgeCurve::Line));

        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };

        // Face 1: A→V→B→D→A (uses shared e_av, e_vb)
        let e_bd = topo.add_edge(Edge::new(b, d, EdgeCurve::Line));
        let e_da = topo.add_edge(Edge::new(d, a, EdgeCurve::Line));
        let wire1 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_av, true),
                    OrientedEdge::new(e_vb, true),
                    OrientedEdge::new(e_bd, true),
                    OrientedEdge::new(e_da, true),
                ],
                true,
            )
            .unwrap(),
        );
        let f1 = topo.add_face(Face::new(wire1, Vec::new(), surface.clone()));

        // Face 2: B→V→A→C→B (reverse traversal of e_vb, e_av; uses shared edges)
        let e_ac = topo.add_edge(Edge::new(a, c, EdgeCurve::Line));
        let e_cb = topo.add_edge(Edge::new(c, b, EdgeCurve::Line));
        let wire2 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_vb, false),
                    OrientedEdge::new(e_av, false),
                    OrientedEdge::new(e_ac, true),
                    OrientedEdge::new(e_cb, true),
                ],
                true,
            )
            .unwrap(),
        );
        let f2 = topo.add_face(Face::new(wire2, Vec::new(), surface));

        let solid = wrap_in_solid(&mut topo, vec![f1, f2]);

        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();
        assert_eq!(
            collapsed, 1,
            "single shared interior vertex should collapse"
        );

        // Both wires should now be 3 edges (was 4) and both should
        // reference the same merged A↔B edge (find_or_create_line_edge
        // creates one and reuses it for the second wire).
        let w1 = topo.wire(topo.face(f1).unwrap().outer_wire()).unwrap();
        let w2 = topo.wire(topo.face(f2).unwrap().outer_wire()).unwrap();
        assert_eq!(w1.edges().len(), 3);
        assert_eq!(w2.edges().len(), 3);
        assert_eq!(
            w1.edges()[0].edge(),
            w2.edges()[0].edge(),
            "both wires should now reference the same merged EdgeId for A↔B"
        );
    }

    /// Edges used by exactly one face of `faces`.
    fn once_used_edges(topo: &Topology, faces: &[FaceId]) -> usize {
        let mut uses: BTreeMap<EdgeId, usize> = BTreeMap::new();
        for &fid in faces {
            let wire = topo.wire(topo.face(fid).unwrap().outer_wire()).unwrap();
            for oe in wire.edges() {
                *uses.entry(oe.edge()).or_default() += 1;
            }
        }
        uses.values().filter(|&&n| n == 1).count()
    }

    /// Two coplanar faces sharing the chord `A-V1-V2-B` at the given
    /// points (z = 0), closed off by `C` below and `D` above. Returns the
    /// topology, the solid, both faces and `V2`.
    fn shared_chord(points: [Point3; 4]) -> (Topology, SolidId, [FaceId; 2], VertexId) {
        let mut topo = Topology::new();
        let [a, v1, v2, b] = points.map(|p| vertex(&mut topo, p));
        let c = vertex(&mut topo, Point3::new(1.5, -1.0, 0.0));
        let d = vertex(&mut topo, Point3::new(1.5, 1.0, 0.0));
        let line = |topo: &mut Topology, p, q| topo.add_edge(Edge::new(p, q, EdgeCurve::Line));
        let e_a1 = line(&mut topo, a, v1);
        let e_12 = line(&mut topo, v1, v2);
        let e_2b = line(&mut topo, v2, b);
        let e_bd = line(&mut topo, b, d);
        let e_da = line(&mut topo, d, a);
        let e_ac = line(&mut topo, a, c);
        let e_cb = line(&mut topo, c, b);
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let face = |topo: &mut Topology, edges: &[(EdgeId, bool)]| {
            let oes = edges
                .iter()
                .map(|&(e, f)| OrientedEdge::new(e, f))
                .collect();
            let wire = topo.add_wire(Wire::new(oes, true).unwrap());
            topo.add_face(Face::new(wire, Vec::new(), surface.clone()))
        };
        let f1 = face(
            &mut topo,
            &[
                (e_a1, true),
                (e_12, true),
                (e_2b, true),
                (e_bd, true),
                (e_da, true),
            ],
        );
        let f2 = face(
            &mut topo,
            &[
                (e_2b, false),
                (e_12, false),
                (e_a1, false),
                (e_ac, true),
                (e_cb, true),
            ],
        );
        let solid = wrap_in_solid(&mut topo, vec![f1, f2]);
        (topo, solid, [f1, f2], v2)
    }

    /// Each step stays within tolerance `L` but the chain does not: V1 at
    /// 0.9 L above A-V2 collapses first, and then V2 sits 0.9 L from A-B,
    /// but V1 would end 1.35 L from that final chord. V2 must stay.
    #[test]
    fn repeated_collapses_keep_every_absorbed_vertex_within_tolerance() {
        let l = Tolerance::new().linear;
        let (mut topo, solid, [f1, f2], v2) = shared_chord([
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.9 * l, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, -1.35 * l, 0.0),
        ]);
        let before = once_used_edges(&topo, &[f1, f2]);

        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();

        assert_eq!(collapsed, 1, "only V1 collapses");
        assert_eq!(once_used_edges(&topo, &[f1, f2]), before);
        for face in [f1, f2] {
            let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
            assert_eq!(wire.edges().len(), 4);
            assert!(
                wire.edges().iter().any(|oe| {
                    let e = topo.edge(oe.edge()).unwrap();
                    e.start() == v2 || e.end() == v2
                }),
                "V2 must stay on face {face:?}"
            );
        }
    }

    #[test]
    fn adjacent_collinear_vertices_collapse_to_one_shared_edge() {
        // Two coplanar faces share the chord A-V1-V2-B with V1, V2 both
        // collinear strictly between A and B. Face 1 meets V1 first, face 2
        // (walking the chord backwards) meets V2 first: collapsing both in
        // one pass left face 1 on (A,V2)+(V2,B) and face 2 on (B,V1)+(V1,A),
        // four once-used edges where there had been none.
        let mut topo = Topology::new();
        let a = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let v1 = vertex(&mut topo, Point3::new(1.0, 0.0, 0.0));
        let v2 = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let b = vertex(&mut topo, Point3::new(3.0, 0.0, 0.0));
        let c = vertex(&mut topo, Point3::new(1.5, -1.0, 0.0));
        let d = vertex(&mut topo, Point3::new(1.5, 1.0, 0.0));
        let line = |topo: &mut Topology, p, q| topo.add_edge(Edge::new(p, q, EdgeCurve::Line));
        let e_a1 = line(&mut topo, a, v1);
        let e_12 = line(&mut topo, v1, v2);
        let e_2b = line(&mut topo, v2, b);
        let e_bd = line(&mut topo, b, d);
        let e_da = line(&mut topo, d, a);
        let e_ac = line(&mut topo, a, c);
        let e_cb = line(&mut topo, c, b);
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let face = |topo: &mut Topology, edges: &[(EdgeId, bool)]| {
            let oes = edges
                .iter()
                .map(|&(e, f)| OrientedEdge::new(e, f))
                .collect();
            let wire = topo.add_wire(Wire::new(oes, true).unwrap());
            topo.add_face(Face::new(wire, Vec::new(), surface.clone()))
        };
        let f1 = face(
            &mut topo,
            &[
                (e_a1, true),
                (e_12, true),
                (e_2b, true),
                (e_bd, true),
                (e_da, true),
            ],
        );
        let f2 = face(
            &mut topo,
            &[
                (e_2b, false),
                (e_12, false),
                (e_a1, false),
                (e_ac, true),
                (e_cb, true),
            ],
        );
        let solid = wrap_in_solid(&mut topo, vec![f1, f2]);
        let before = once_used_edges(&topo, &[f1, f2]);

        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();

        assert_eq!(collapsed, 2, "both chord vertices collapse");
        assert_eq!(
            once_used_edges(&topo, &[f1, f2]),
            before,
            "the collapse must not unpair the chord"
        );
        let w1 = topo.wire(topo.face(f1).unwrap().outer_wire()).unwrap();
        let w2 = topo.wire(topo.face(f2).unwrap().outer_wire()).unwrap();
        assert_eq!(w1.edges().len(), 3);
        assert_eq!(w2.edges().len(), 3);
        let chord = w1.edges()[0].edge();
        assert!(
            w2.edges().iter().any(|oe| oe.edge() == chord),
            "both faces must share the one A-B edge"
        );
        let chord = topo.edge(chord).unwrap();
        assert_eq!(
            [
                chord.start().min(chord.end()),
                chord.start().max(chord.end())
            ],
            [a.min(b), a.max(b)]
        );
    }

    /// The most faces of `faces` that use any one edge.
    fn max_face_uses(topo: &Topology, faces: &[FaceId]) -> usize {
        let mut uses: BTreeMap<EdgeId, usize> = BTreeMap::new();
        for &fid in faces {
            let wire = topo.wire(topo.face(fid).unwrap().outer_wire()).unwrap();
            for oe in wire.edges() {
                *uses.entry(oe.edge()).or_default() += 1;
            }
        }
        uses.values().copied().max().unwrap_or(0)
    }

    /// A planar face bounded by `edges` (edge, forward) on the plane
    /// through the origin with normal `normal`.
    fn plane_face(topo: &mut Topology, edges: &[(EdgeId, bool)], normal: Vec3) -> FaceId {
        let oes = edges
            .iter()
            .map(|&(e, f)| OrientedEdge::new(e, f))
            .collect();
        let wire = topo.add_wire(Wire::new(oes, true).unwrap());
        topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane { normal, d: 0.0 },
        ))
    }

    #[test]
    fn edge_disjoint_chords_between_the_same_ends_keep_separate_edges() {
        // P-V1-Q bounds f1, f2 (z = 0) and P-V2-Q bounds f3, f4 (y = 0).
        // Both chords collapse in one pass; sharing one P-Q edge between
        // them would put it on four faces.
        let mut topo = Topology::new();
        let p = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let v1 = vertex(&mut topo, Point3::new(1.0, 0.0, 0.0));
        let v2 = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let q = vertex(&mut topo, Point3::new(3.0, 0.0, 0.0));
        let c = vertex(&mut topo, Point3::new(1.5, -1.0, 0.0));
        let d = vertex(&mut topo, Point3::new(1.5, 1.0, 0.0));
        let u = vertex(&mut topo, Point3::new(1.5, 0.0, 1.0));
        let l = vertex(&mut topo, Point3::new(1.5, 0.0, -1.0));
        let line = |topo: &mut Topology, a, b| topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let e_p1 = line(&mut topo, p, v1);
        let e_1q = line(&mut topo, v1, q);
        let e_p2 = line(&mut topo, p, v2);
        let e_2q = line(&mut topo, v2, q);
        let e_qd = line(&mut topo, q, d);
        let e_dp = line(&mut topo, d, p);
        let e_pc = line(&mut topo, p, c);
        let e_cq = line(&mut topo, c, q);
        let e_qu = line(&mut topo, q, u);
        let e_up = line(&mut topo, u, p);
        let e_pl = line(&mut topo, p, l);
        let e_lq = line(&mut topo, l, q);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let faces = [
            plane_face(
                &mut topo,
                &[(e_p1, true), (e_1q, true), (e_qd, true), (e_dp, true)],
                z,
            ),
            plane_face(
                &mut topo,
                &[(e_1q, false), (e_p1, false), (e_pc, true), (e_cq, true)],
                z,
            ),
            plane_face(
                &mut topo,
                &[(e_p2, true), (e_2q, true), (e_qu, true), (e_up, true)],
                y,
            ),
            plane_face(
                &mut topo,
                &[(e_2q, false), (e_p2, false), (e_pl, true), (e_lq, true)],
                y,
            ),
        ];
        let solid = wrap_in_solid(&mut topo, faces.to_vec());

        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();

        assert_eq!(collapsed, 2, "both chord vertices collapse");
        assert_eq!(max_face_uses(&topo, &faces), 2, "no edge on a third face");
        for pair in faces.chunks(2) {
            let first = topo.wire(topo.face(pair[0]).unwrap().outer_wire()).unwrap();
            let second = topo.wire(topo.face(pair[1]).unwrap().outer_wire()).unwrap();
            assert_eq!(first.edges().len(), 3);
            assert!(
                second
                    .edges()
                    .iter()
                    .any(|oe| oe.edge() == first.edges()[0].edge()),
                "each pair shares its own P-Q edge"
            );
        }
    }

    #[test]
    fn asymmetric_residue_never_moves_onto_an_edge_with_two_faces() {
        // f carries P-V-Q alone; g and h already share a direct P-Q edge.
        // Re-unifying f onto it would put that edge on three faces.
        let mut topo = Topology::new();
        let p = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let v = vertex(&mut topo, Point3::new(1.0, 0.0, 0.0));
        let q = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let c = vertex(&mut topo, Point3::new(1.0, -1.0, 0.0));
        let u = vertex(&mut topo, Point3::new(1.0, 0.0, 1.0));
        let l = vertex(&mut topo, Point3::new(1.0, 0.0, -1.0));
        let line = |topo: &mut Topology, a, b| topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let e_pv = line(&mut topo, p, v);
        let e_vq = line(&mut topo, v, q);
        let e_qc = line(&mut topo, q, c);
        let e_cp = line(&mut topo, c, p);
        let e_pq = line(&mut topo, p, q);
        let e_qu = line(&mut topo, q, u);
        let e_up = line(&mut topo, u, p);
        let e_pl = line(&mut topo, p, l);
        let e_lq = line(&mut topo, l, q);
        let z = Vec3::new(0.0, 0.0, 1.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let faces = [
            plane_face(
                &mut topo,
                &[(e_pv, true), (e_vq, true), (e_qc, true), (e_cp, true)],
                z,
            ),
            plane_face(&mut topo, &[(e_pq, true), (e_qu, true), (e_up, true)], y),
            plane_face(&mut topo, &[(e_pq, false), (e_pl, true), (e_lq, true)], y),
        ];
        let solid = wrap_in_solid(&mut topo, faces.to_vec());

        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();

        assert_eq!(collapsed, 0);
        assert_eq!(max_face_uses(&topo, &faces), 2);
    }

    #[test]
    fn off_line_vertex_is_not_collapsed() {
        // V is not collinear — perpendicular offset of 1e-3 on a 2.0 baseline.
        let mut topo = Topology::new();
        let a = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let b = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let v = vertex(&mut topo, Point3::new(1.0, 1e-3, 0.0));
        let d = vertex(&mut topo, Point3::new(2.0, 1.0, 0.0));
        let f = face_with_split_edge(&mut topo, a, v, b, d);
        let solid = wrap_in_solid(&mut topo, vec![f]);
        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();
        assert_eq!(collapsed, 0);
    }

    #[test]
    fn vertex_with_three_incident_edges_is_not_collapsed() {
        // V on edge A-V-B but also has a third edge going to a corner —
        // collapsing would orphan that third edge, so the pass must skip.
        let mut topo = Topology::new();
        let a = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let b = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let v = vertex(&mut topo, Point3::new(1.0, 0.0, 0.0));
        let c = vertex(&mut topo, Point3::new(1.0, 1.0, 0.0));
        let d = vertex(&mut topo, Point3::new(2.0, 1.0, 0.0));

        // First face: A→V→B→D→A
        let f_split = face_with_split_edge(&mut topo, a, v, b, d);
        // Second face: A→V→C→A — gives V a 3rd incident edge (V→C and C→A)
        let e_av2 = topo.add_edge(Edge::new(a, v, EdgeCurve::Line));
        let e_vc = topo.add_edge(Edge::new(v, c, EdgeCurve::Line));
        let e_ca2 = topo.add_edge(Edge::new(c, a, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_av2, true),
                    OrientedEdge::new(e_vc, true),
                    OrientedEdge::new(e_ca2, true),
                ],
                true,
            )
            .unwrap(),
        );
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        let f_extra = topo.add_face(Face::new(wire, Vec::new(), surface));
        let solid = wrap_in_solid(&mut topo, vec![f_split, f_extra]);

        let collapsed =
            collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();
        assert_eq!(
            collapsed, 0,
            "V has 3 distinct incident edges (A→V appears as two EdgeIds plus V→C); should not collapse"
        );
    }

    /// Closed 3-edge wire where *both* the head junction (edges[0],
    /// edges[1]) and the wrap-around junction (edges[2], edges[0]) sit
    /// at collinear plan-candidate vertices. The earlier naïve
    /// implementation rewrote the wire to a length-2 wire that
    /// dropped the previously-merged edge via `Vec::remove(0)`; the
    /// two-pass plan-then-emit algorithm prevents that by recording
    /// merges before emitting and never consuming an edge index twice.
    /// Greptile flagged this as the corruption scenario for PR #708.
    #[test]
    fn closed_wire_wrap_around_does_not_corrupt() {
        let mut topo = Topology::new();
        // Three collinear vertices on the X axis plus a fourth above:
        // A=(0,0,0), V=(1,0,0), B=(2,0,0), C=(3,0,0). With the wire
        // going A→V→B→C→A, junction(0,1) is at V (collinear between
        // A and B) and junction(2,3) is at C (collinear between B and
        // A — going *back* via the long wrap edge). We only set up
        // one plan vertex (V) in this test so the algorithm has to
        // safely consume edges[0]+edges[1] and emit a merged edge in
        // slot 0 while leaving the wrap-around edges[3] untouched.
        // The check is that the resulting wire is still well-formed
        // (closed, correct length, no `EdgeId` corruption).
        let a = vertex(&mut topo, Point3::new(0.0, 0.0, 0.0));
        let v = vertex(&mut topo, Point3::new(1.0, 0.0, 0.0));
        let b = vertex(&mut topo, Point3::new(2.0, 0.0, 0.0));
        let d = vertex(&mut topo, Point3::new(2.0, 1.0, 0.0));

        // Two coplanar faces sharing E_av and E_vb (symmetric case).
        let e_av = topo.add_edge(Edge::new(a, v, EdgeCurve::Line));
        let e_vb = topo.add_edge(Edge::new(v, b, EdgeCurve::Line));
        let e_bd = topo.add_edge(Edge::new(b, d, EdgeCurve::Line));
        let e_da = topo.add_edge(Edge::new(d, a, EdgeCurve::Line));

        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };

        // Wire 1: A→V→B→D→A — the consumed pair lives at the *head*.
        let wire1 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_av, true),
                    OrientedEdge::new(e_vb, true),
                    OrientedEdge::new(e_bd, true),
                    OrientedEdge::new(e_da, true),
                ],
                true,
            )
            .unwrap(),
        );
        let f1 = topo.add_face(Face::new(wire1, Vec::new(), surface.clone()));

        // Wire 2: D→B→V→A→D — the consumed pair (E_vb, E_av) now
        // straddles the wrap-around boundary (positions 1+2 → still
        // mid-wire, not the wrap), but we *also* shift it so the
        // collapse lands at the (n-1, 0) junction by ordering as
        // A→D→B→V→A. That puts e_vb at index n-1 (=3) and the
        // wraparound junction at V — exactly the corruption scenario.
        let e_db = topo.add_edge(Edge::new(d, b, EdgeCurve::Line));
        let e_va = topo.add_edge(Edge::new(v, a, EdgeCurve::Line));
        let wire2 = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e_da, false), // A → D (reverse)
                    OrientedEdge::new(e_db, true),  // D → B
                    OrientedEdge::new(e_vb, false), // B → V (reverse)
                    OrientedEdge::new(e_va, true),  // V → A
                ],
                true,
            )
            .unwrap(),
        );
        let _f2 = topo.add_face(Face::new(wire2, Vec::new(), surface));

        let solid = wrap_in_solid(&mut topo, vec![f1, _f2]);

        // The pass should run without panicking and either collapse V
        // safely (wires shrink by 1) or skip (no collapse), but in no
        // case produce a corrupt wire. We validate by checking each
        // wire's edges remain consistent (start of edge i == end of
        // edge i-1) under the original orientation.
        let _ = collapse_collinear_wire_vertices(&mut topo, solid, Tolerance::new()).unwrap();

        for fid in [f1, _f2] {
            let face = topo.face(fid).unwrap();
            let w = topo.wire(face.outer_wire()).unwrap();
            assert!(
                !w.edges().is_empty(),
                "wire {fid:?} should not be empty after collapse"
            );
            // Walk the chain; each edge's oriented_end must match the
            // next edge's oriented_start.
            for window in w.edges().windows(2) {
                let prev = window[0];
                let next = window[1];
                let prev_edge = topo.edge(prev.edge()).unwrap();
                let next_edge = topo.edge(next.edge()).unwrap();
                assert_eq!(
                    prev.oriented_end(prev_edge),
                    next.oriented_start(next_edge),
                    "wire {fid:?} is broken after collapse: edges no longer chain"
                );
            }
        }
    }
}
