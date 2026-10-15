//! Oracle for the shared [`G1ChainIndex`]: [`expand_g1_chain`],
//! [`expand_g1_chain_with`] and [`g1_chains`] against the per-call expansion
//! as it stood before the index was built once per call, kept verbatim
//! (std hash collections included).
//!
//! The corpus is the unit cube and cubes with one to four edges already
//! rounded by [`FilletBuilder`], whose bands meet the planar faces along
//! tangent seams, so ridgelines cross face pairs through `faces_continue_at`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{HashMap, HashSet};

use remus_topology::edge::Edge;
use remus_topology::test_utils::make_unit_cube_manifold;
use remus_topology::vertex::Vertex;

use super::*;
use crate::fillet_builder::FilletBuilder;

/// `expand_g1_chain` before the shared index, verbatim.
#[allow(clippy::too_many_lines)]
fn reference_expand_g1_chain(
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

/// `g1_chains` before the shared index, verbatim.
fn reference_g1_chains(
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
        let members = reference_expand_g1_chain(topo, solid, &[seed], tol)?;
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
        chains.push(reference_order_chain(topo, members)?);
    }

    Ok(chains)
}

/// `order_chain` before the deterministic-hash migration, verbatim.
fn reference_order_chain(
    topo: &Topology,
    edges: Vec<EdgeId>,
) -> Result<Vec<EdgeId>, crate::BlendError> {
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

fn debug<T: std::fmt::Debug>(value: &T) -> String {
    format!("{value:?}")
}

/// The unit cube, and the cube with each listed edge selection rounded.
fn corpus() -> Vec<(Topology, SolidId)> {
    let mut cases = Vec::new();
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edges = remus_topology::explorer::solid_edges(&topo, cube).unwrap();
    cases.push((topo.clone(), cube));
    let selections: [&[usize]; 6] = [&[0], &[3], &[0, 2], &[1, 6], &[0, 2, 8, 10], &[4, 5]];
    for selection in selections {
        for radius in [0.1, 0.25] {
            let mut rounded = topo.clone();
            let picked: Vec<EdgeId> = selection.iter().map(|&i| edges[i]).collect();
            let mut builder = FilletBuilder::new(&mut rounded, cube);
            builder.add_edges(&picked, radius);
            if let Ok(result) = builder.build()
                && !result.is_partial
            {
                cases.push((rounded, result.solid));
            }
        }
    }
    cases
}

#[test]
fn shared_index_matches_per_call_expansion() {
    let tol = Tolerance::new();
    let cases = corpus();
    assert!(cases.len() >= 8, "only {} corpus solids built", cases.len());
    let (mut seam_chains, mut checked) = (0, 0);
    for (topo, solid) in &cases {
        let edges = remus_topology::explorer::solid_edges(topo, *solid).unwrap();
        let index = G1ChainIndex::build(topo, *solid).unwrap();

        for &edge in &edges {
            let expected = reference_expand_g1_chain(topo, *solid, &[edge], tol).unwrap();
            assert_eq!(
                expand_g1_chain(topo, *solid, &[edge], tol).unwrap(),
                expected
            );
            assert_eq!(
                expand_g1_chain_with(&index, &[edge], tol).unwrap(),
                expected
            );
            let mut faces: Vec<usize> = expected
                .iter()
                .flat_map(|e| index.edge_to_faces[&e.index()].iter().map(|f| f.index()))
                .collect();
            faces.sort_unstable();
            faces.dedup();
            if faces.len() > 2 {
                seam_chains += 1;
            }
            checked += 1;
        }

        let mut reversed = edges.clone();
        reversed.reverse();
        let pairs: Vec<EdgeId> = edges
            .iter()
            .zip(edges.iter().skip(1))
            .flat_map(|(&a, &b)| [a, b])
            .collect();
        // A stray edge on no face expands to nothing and stays its own chain.
        let mut stray_topo = topo.clone();
        let a = stray_topo.add_vertex(Vertex::new(Point3::new(5.0, 5.0, 5.0), 1e-7));
        let b = stray_topo.add_vertex(Vertex::new(Point3::new(6.0, 5.0, 5.0), 1e-7));
        let stray = stray_topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let with_duplicates = [edges[0], edges[0], stray, edges[edges.len() - 1]];
        for seeds in [
            &edges[..],
            &reversed[..],
            &pairs[..],
            &with_duplicates[..],
            &[][..],
        ] {
            assert_eq!(
                expand_g1_chain(&stray_topo, *solid, seeds, tol).unwrap(),
                reference_expand_g1_chain(&stray_topo, *solid, seeds, tol).unwrap()
            );
            assert_eq!(
                g1_chains(&stray_topo, *solid, seeds, tol).unwrap(),
                reference_g1_chains(&stray_topo, *solid, seeds, tol).unwrap()
            );
        }
    }
    // Without ridgelines that cross a tangent seam the corpus would never
    // reach `faces_continue_at`, and the equality above would be thin.
    assert!(
        seam_chains > 20,
        "{seam_chains} chains crossed a tangent seam"
    );
    assert!(checked > 100, "checked {checked} seeds");
}

#[test]
fn errors_match_per_call_expansion() {
    let tol = Tolerance::new();
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let edge = remus_topology::explorer::solid_edges(&topo, cube).unwrap()[0];
    let mut other = Topology::new();
    let _ = make_unit_cube_manifold(&mut other);
    let missing = make_unit_cube_manifold(&mut other);

    assert_eq!(
        debug(&expand_g1_chain(&topo, missing, &[edge], tol)),
        debug(&reference_expand_g1_chain(&topo, missing, &[edge], tol))
    );
    assert!(G1ChainIndex::build(&topo, missing).is_err());
    assert_eq!(
        debug(&g1_chains(&topo, missing, &[edge], tol)),
        debug(&reference_g1_chains(&topo, missing, &[edge], tol))
    );
    // No seed, no lookup: both return an empty grouping for a missing solid.
    assert_eq!(
        debug(&g1_chains(&topo, missing, &[], tol)),
        debug(&reference_g1_chains(&topo, missing, &[], tol))
    );
}
