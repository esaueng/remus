//! G1 continuity chain expansion for fillet edge propagation.
//!
//! Given a set of seed edges, iteratively expands along manifold edges
//! that are tangent-continuous at the shared vertex and continue the same
//! smooth ridgeline: either both edges lie between the same two faces, or
//! the faces change across a tangent seam (a planar wall running into the
//! cylindrical band of an earlier fillet) so that the outward normals on
//! each side agree at the vertex.

use std::collections::{HashMap, HashSet};

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::builder_utils::{cylinder_of, plane_of};

/// Cosine threshold shared by the edge-tangent and face-normal continuity
/// tests: about 10 degrees of deviation. A true G1 joint scores 1.0 exactly.
const G1_COS_THRESHOLD: f64 = 0.985;

/// Outward unit normal of face `fid` at the surface point `point`.
///
/// Returns `None` when the surface cannot be evaluated there (a NURBS
/// projection that fails, a degenerate normal), which callers treat as
/// "not tangent-continuous".
fn outward_normal_at(topo: &Topology, fid: FaceId, point: Point3) -> Option<Vec3> {
    let face = topo.face(fid).ok()?;
    let surface = face.surface();
    let raw = match surface {
        FaceSurface::Plane { normal, .. } => *normal,
        other => {
            let (u, v) = other.project_point(point)?;
            other.normal(u, v)
        }
    };
    let oriented = if face.is_reversed() { -raw } else { raw };
    oriented.normalize().ok()
}

/// Whether two faces carry one and the same underlying surface, so that the
/// edge between them is a topological split rather than a geometric seam.
///
/// Only planes and cylinders are recognised; every other pairing is treated
/// as a genuine change of surface.
fn same_underlying_surface(topo: &Topology, a: FaceId, b: FaceId, tol: Tolerance) -> bool {
    let (Ok(fa), Ok(fb)) = (topo.face(a), topo.face(b)) else {
        return false;
    };
    let cos_parallel = 1.0 - tol.angular.max(1e-9);
    if let (Some((na, da)), Some((nb, db))) = (plane_of(fa.surface()), plane_of(fb.surface())) {
        let (Ok(ua), Ok(ub)) = (na.normalize(), nb.normalize()) else {
            return false;
        };
        let dot = ua.dot(ub);
        // Same plane whichever way each face's stored normal points.
        return (dot > cos_parallel && (da - db).abs() <= tol.linear)
            || (dot < -cos_parallel && (da + db).abs() <= tol.linear);
    }
    if let (Some(ca), Some(cb)) = (cylinder_of(fa.surface()), cylinder_of(fb.surface())) {
        let (Ok(axis_a), Ok(axis_b)) = (ca.axis().normalize(), cb.axis().normalize()) else {
            return false;
        };
        if axis_a.dot(axis_b).abs() < cos_parallel {
            return false;
        }
        if (ca.radius() - cb.radius()).abs() > tol.linear {
            return false;
        }
        let between = cb.origin() - ca.origin();
        let off_axis = between - axis_a * between.dot(axis_a);
        return off_axis.length() <= tol.linear;
    }
    false
}

/// Whether the two faces bounding one edge continue the two faces bounding
/// another edge smoothly at `point`: the outward normals match pairwise
/// (either pairing, since face order within a pair is arbitrary), and every
/// face that changes across the vertex changes to a different surface.
///
/// This is what makes a ridgeline continue across a tangent seam: a box edge
/// runs onto the arc where the wall turns into an earlier fillet's cylinder.
/// Edges whose faces meet at a crease do not qualify, so a fillet never
/// propagates around a sharp corner. Nor does a split of one surface into
/// two faces (collinear edges between coplanar faces, as a fuse can leave
/// behind): that boundary is topological, callers select the segments they
/// mean, and the engine has always ended a fillet there.
fn faces_continue_at(
    topo: &Topology,
    current: &[FaceId],
    neighbor: &[FaceId],
    point: Point3,
    tol: Tolerance,
) -> bool {
    let normals = |faces: &[FaceId]| -> Option<(Vec3, Vec3)> {
        Some((
            outward_normal_at(topo, faces[0], point)?,
            outward_normal_at(topo, faces[1], point)?,
        ))
    };
    let (Some((a1, a2)), Some((b1, b2))) = (normals(current), normals(neighbor)) else {
        return false;
    };
    let same = |x: Vec3, y: Vec3| x.dot(y) > G1_COS_THRESHOLD;
    let pairing = if same(a1, b1) && same(a2, b2) {
        [(current[0], neighbor[0]), (current[1], neighbor[1])]
    } else if same(a1, b2) && same(a2, b1) {
        [(current[0], neighbor[1]), (current[1], neighbor[0])]
    } else {
        return false;
    };
    pairing.iter().all(|&(from, to)| {
        from.index() == to.index() || !same_underlying_surface(topo, from, to, tol)
    })
}

/// Sample the tangent of an edge curve at normalized parameter `t` in `[0, 1]`.
///
/// Maps from the `[0, 1]` interval to the curve's native parameter range,
/// accounting for circle/ellipse angle wrapping.
fn sample_edge_tangent(
    curve: &EdgeCurve,
    p_start: Point3,
    p_end: Point3,
    domain: (f64, f64),
    t: f64,
) -> Vec3 {
    let parameter = (domain.1 - domain.0).mul_add(t, domain.0);
    curve.tangent_with_endpoints(parameter, p_start, p_end)
}

/// Expand a seed edge set by G1 (tangent-continuity) chain propagation.
///
/// Starting from `seed_edges`, iteratively adds any manifold edge that:
/// 1. Shares a vertex with an edge already in the set.
/// 2. Continues the same ridgeline: it has the same pair of adjacent faces,
///    or its faces are tangent to the current edge's faces at the shared
///    vertex (the outward normals agree pairwise, < 10 deg deviation). The
///    second form is how a fillet seed on a box edge runs onto the arc where
///    that edge meets an earlier fillet's cylindrical band, and out again
///    onto the edge beyond it.
/// 3. Is tangent-continuous at the shared vertex (< 10 deg deviation).
///
/// # Errors
///
/// Returns `BlendError::Topology` if any topology lookup fails.
#[allow(clippy::too_many_lines)]
pub fn expand_g1_chain(
    topo: &Topology,
    solid: SolidId,
    seed_edges: &[EdgeId],
    tol: Tolerance,
) -> Result<Vec<EdgeId>, crate::BlendError> {
    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    let shell_face_ids: Vec<FaceId> = shell.faces().to_vec();

    let mut edge_to_faces: HashMap<usize, Vec<FaceId>> = HashMap::new();
    let mut vertex_to_edges: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    let mut edge_ids: HashMap<usize, EdgeId> = HashMap::new();

    for &fid in &shell_face_ids {
        let face = topo.face(fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for wid in wire_ids {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let eid = oe.edge();
                edge_to_faces.entry(eid.index()).or_default().push(fid);
                edge_ids.insert(eid.index(), eid);
                let edge = topo.edge(eid)?;
                vertex_to_edges
                    .entry(edge.start().index())
                    .or_default()
                    .push(eid);
                vertex_to_edges
                    .entry(edge.end().index())
                    .or_default()
                    .push(eid);
            }
        }
    }
    // Deduplicate vertex_to_edges (each edge appears once per adjacent face).
    for edges in vertex_to_edges.values_mut() {
        edges.sort_unstable_by_key(|e: &EdgeId| e.index());
        edges.dedup_by_key(|e: &mut EdgeId| e.index());
    }

    let mut expanded: HashSet<usize> = seed_edges.iter().map(|e| e.index()).collect();
    let mut queue: Vec<EdgeId> = seed_edges.to_vec();

    while let Some(current) = queue.pop() {
        // Face pair for current edge (sorted for comparison).
        let Some(cf) = edge_to_faces.get(&current.index()) else {
            continue;
        };
        if cf.len() != 2 {
            continue;
        }
        let (cf1, cf2) = {
            let (a, b) = (cf[0].index(), cf[1].index());
            if a < b { (a, b) } else { (b, a) }
        };

        let cur_edge = topo.edge(current)?;
        let cur_start = topo.vertex(cur_edge.start())?.point();
        let cur_end = topo.vertex(cur_edge.end())?.point();
        let cur_domain = cur_edge.strict_domain().map_err(crate::edge_domain_input)?;

        for &shared_vid in &[cur_edge.start(), cur_edge.end()] {
            // "Away from vertex" tangent for the current edge at this vertex.
            let t_cur = {
                let t_raw = if shared_vid == cur_edge.start() {
                    // Forward tangent at start points away from vertex -- correct sign.
                    sample_edge_tangent(cur_edge.curve(), cur_start, cur_end, cur_domain, 0.0)
                } else {
                    // Forward tangent at end points INTO vertex; negate for "away".
                    -sample_edge_tangent(cur_edge.curve(), cur_start, cur_end, cur_domain, 1.0)
                };
                let len = t_raw.length();
                if len < tol.linear {
                    continue;
                }
                t_raw * (1.0 / len)
            };

            let Some(neighbors) = vertex_to_edges.get(&shared_vid.index()) else {
                continue;
            };
            for &nb in neighbors {
                if expanded.contains(&nb.index()) {
                    continue;
                }
                // Must be manifold (exactly 2 adjacent faces).
                let Some(nf) = edge_to_faces.get(&nb.index()) else {
                    continue;
                };
                if nf.len() != 2 {
                    continue;
                }
                // Must continue the same ridgeline: the same face pair, or a
                // face pair tangent to it at the shared vertex.
                let (nf1, nf2) = {
                    let (a, b) = (nf[0].index(), nf[1].index());
                    if a < b { (a, b) } else { (b, a) }
                };
                let shared_point = topo.vertex(shared_vid)?.point();
                if (cf1, cf2) != (nf1, nf2) && !faces_continue_at(topo, cf, nf, shared_point, tol) {
                    continue;
                }

                // "Away from vertex" tangent for the neighbor edge at the shared vertex.
                let nb_edge = topo.edge(nb)?;
                let nb_start = topo.vertex(nb_edge.start())?.point();
                let nb_end = topo.vertex(nb_edge.end())?.point();
                let nb_domain = nb_edge.strict_domain().map_err(crate::edge_domain_input)?;
                let t_nb = {
                    let t_raw = if shared_vid == nb_edge.start() {
                        sample_edge_tangent(nb_edge.curve(), nb_start, nb_end, nb_domain, 0.0)
                    } else {
                        -sample_edge_tangent(nb_edge.curve(), nb_start, nb_end, nb_domain, 1.0)
                    };
                    let len = t_raw.length();
                    if len < tol.linear {
                        continue;
                    }
                    t_raw * (1.0 / len)
                };

                // G1 continuity: "away" tangents must be anti-parallel (< ~10 deg deviation).
                // cos(170 deg) ~ -0.985.  This is strict: a true G1 joint has dot = -1.0.
                if t_cur.dot(t_nb) < -G1_COS_THRESHOLD {
                    expanded.insert(nb.index());
                    queue.push(nb);
                }
            }
        }
    }

    let mut result: Vec<EdgeId> = expanded
        .iter()
        .filter_map(|idx| edge_ids.get(idx).copied())
        .collect();
    result.sort_unstable_by_key(|e| e.index());
    Ok(result)
}

/// Group seed edges into G1 chains, each ordered head-to-tail.
///
/// [`expand_g1_chain`] answers "which edges belong to the ridgeline", but
/// returns them as an index-sorted set covering every seed at once. A blend
/// spine needs something stronger: one entry per ridgeline, with the edges in
/// traversal order, so arc length accumulates monotonically along the chain.
///
/// Seeds that land on the same ridgeline collapse into a single chain. A
/// chain that is not a simple path (a branch point, which G1 face-pair
/// matching should already exclude) is returned in its expansion order rather
/// than dropped, leaving the caller's existing behaviour intact.
///
/// # Errors
///
/// Returns [`crate::BlendError::Topology`] if any topology lookup fails.
pub fn g1_chains(
    topo: &Topology,
    solid: SolidId,
    seed_edges: &[EdgeId],
    tol: Tolerance,
) -> Result<Vec<Vec<EdgeId>>, crate::BlendError> {
    let mut claimed: HashSet<usize> = HashSet::new();
    let mut chains: Vec<Vec<EdgeId>> = Vec::new();

    for &seed in seed_edges {
        if claimed.contains(&seed.index()) {
            continue;
        }
        let members = expand_g1_chain(topo, solid, &[seed], tol)?;
        // An edge that belongs to no face of this solid expands to nothing.
        // Keep it as its own chain so the caller still has something to
        // attribute the resulting failure to, rather than silently dropping
        // the request.
        let members = if members.is_empty() {
            vec![seed]
        } else {
            members
        };
        for member in &members {
            claimed.insert(member.index());
        }
        chains.push(order_chain(topo, members)?);
    }

    Ok(chains)
}

/// Order a set of connected edges head-to-tail.
///
/// Walks from a free end (a vertex touched by exactly one edge of the set);
/// a closed loop has no free end, so the lowest-indexed vertex starts it.
/// Returns the input order unchanged if the set does not form a simple path.
fn order_chain(topo: &Topology, edges: Vec<EdgeId>) -> Result<Vec<EdgeId>, crate::BlendError> {
    if edges.len() <= 1 {
        return Ok(edges);
    }

    let mut incident: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    let mut vertices: HashMap<usize, remus_topology::vertex::VertexId> = HashMap::new();
    for &eid in &edges {
        let edge = topo.edge(eid)?;
        for vid in [edge.start(), edge.end()] {
            incident.entry(vid.index()).or_default().push(eid);
            vertices.insert(vid.index(), vid);
        }
    }
    for list in incident.values_mut() {
        list.sort_unstable_by_key(|e: &EdgeId| e.index());
    }

    // Prefer a free end so an open chain runs end to end; fall back to the
    // lowest vertex index for a closed loop. `min()` keeps this deterministic.
    let start_index = incident
        .iter()
        .filter(|(_, list)| list.len() == 1)
        .map(|(index, _)| *index)
        .min()
        .or_else(|| incident.keys().copied().min());
    let Some(start_index) = start_index else {
        return Ok(edges);
    };
    let Some(&start_vertex) = vertices.get(&start_index) else {
        return Ok(edges);
    };

    let mut ordered: Vec<EdgeId> = Vec::with_capacity(edges.len());
    let mut used: HashSet<usize> = HashSet::new();
    let mut current = start_vertex;
    while ordered.len() < edges.len() {
        let Some(candidates) = incident.get(&current.index()) else {
            break;
        };
        let Some(&next) = candidates.iter().find(|eid| !used.contains(&eid.index())) else {
            break;
        };
        used.insert(next.index());
        ordered.push(next);
        let edge = topo.edge(next)?;
        current = if edge.start() == current {
            edge.end()
        } else {
            edge.start()
        };
    }

    if ordered.len() == edges.len() {
        Ok(ordered)
    } else {
        Ok(edges)
    }
}
