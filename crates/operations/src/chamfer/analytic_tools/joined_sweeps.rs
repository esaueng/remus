//! Exact piecewise-analytic sweep tools joined across identical end sections.
//! Tier: core. No surface or boundary curve is tessellated.

use super::Tool;
use crate::OperationsError;
use remus_blend::{
    BlendResult,
    g1_chain::{G1ChainIndex, expand_g1_chain_with},
};
use remus_math::{
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
    explorer::solid_faces,
    face::FaceId,
    shell::Shell,
    solid::{Solid, SolidId},
};
use std::collections::{BTreeMap, BTreeSet};

struct Cap {
    tool: usize,
    face: FaceId,
    normal: Vec3,
    points: Vec<Point3>,
}

pub(super) fn build(
    topo: &mut Topology,
    solid: SolidId,
    seeds: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<BlendResult, OperationsError> {
    let tol = Tolerance::new();
    let index = G1ChainIndex::build(topo, solid)?;
    let adjacency = topo.build_adjacency(solid)?;
    let mut distances = BTreeMap::new();
    for &seed in seeds {
        if distances.contains_key(&seed) {
            continue;
        }
        let chain: Vec<_> = expand_g1_chain_with(&index, &[seed], tol)?
            .into_iter()
            .filter(|edge| seeds.contains(edge))
            .collect();
        let first = adjacency.faces_for_edge(seed);
        let rail = first
            .iter()
            .copied()
            .find(|face| {
                chain
                    .iter()
                    .all(|&edge| adjacency.faces_for_edge(edge).contains(face))
            })
            .ok_or_else(|| crate::chamfer::unsupported("analytic chain has no common rail"))?;
        let along = if first[0] == rail { d1 } else { d2 };
        let across = if first[0] == rail { d2 } else { d1 };
        for edge in chain {
            let pair = adjacency.faces_for_edge(edge);
            if pair.len() != 2 {
                return Err(crate::chamfer::unsupported(
                    "analytic chain is not manifold",
                ));
            }
            distances.insert(
                edge,
                if pair[0] == rail {
                    [along, across]
                } else {
                    [across, along]
                },
            );
        }
    }
    let mut pieces = Vec::new();
    for &seed in seeds {
        let pair = adjacency.faces_for_edge(seed);
        let piece = super::make_tool(
            topo,
            solid,
            seed,
            [pair[0], pair[1]],
            distances[&seed],
            false,
        )?
        .ok_or_else(|| {
            crate::chamfer::unsupported("analytic chain contains an unsupported sweep")
        })?;
        if piece.concave {
            return Err(crate::chamfer::unsupported(
                "joined sweeps require convex ridges",
            ));
        }
        pieces.push(piece);
    }
    let mut caps = Vec::new();
    for (tool, piece) in pieces.iter().enumerate() {
        for face in solid_faces(topo, piece.solid)? {
            let data = topo.face(face)?;
            let Some(normal) = data.effective_plane_normal() else {
                continue;
            };
            if !data.inner_wires().is_empty() {
                continue;
            }
            let edges = topo.wire(data.outer_wire())?.edges();
            if edges.len() != 3
                || edges.iter().any(|edge| {
                    topo.edge(edge.edge())
                        .is_ok_and(|edge| !matches!(edge.curve(), EdgeCurve::Line))
                })
            {
                continue;
            }
            let points = edges
                .iter()
                .map(|edge| {
                    Ok(topo
                        .vertex(edge.oriented_start(topo.edge(edge.edge())?))?
                        .point())
                })
                .collect::<Result<Vec<_>, OperationsError>>()?;
            caps.push(Cap {
                tool,
                face,
                normal,
                points,
            });
        }
    }
    let mut parent: Vec<_> = (0..pieces.len()).collect();
    let mut remove = BTreeSet::new();
    for (i, a) in caps.iter().enumerate() {
        if remove.contains(&a.face) {
            continue;
        }
        let matches: Vec<_> = caps
            .iter()
            .enumerate()
            .skip(i + 1)
            .filter(|(_, b)| {
                a.tool != b.tool
                    && !remove.contains(&b.face)
                    && a.normal.dot(b.normal) < -1.0 + tol.angular
                    && a.points.iter().all(|p| {
                        b.points
                            .iter()
                            .filter(|q| (**q - *p).length() <= tol.linear)
                            .count()
                            == 1
                    })
            })
            .collect();
        let [(_, b)] = matches.as_slice() else {
            continue;
        };
        remove.insert(a.face);
        remove.insert(b.face);
        let ra = root(&mut parent, a.tool);
        let rb = root(&mut parent, b.tool);
        parent[rb] = ra;
    }
    if remove.is_empty() {
        return Err(crate::chamfer::unsupported(
            "no identical analytic sweep sections",
        ));
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..pieces.len() {
        groups.entry(root(&mut parent, i)).or_default().push(i);
    }
    let mut tools = Vec::new();
    for group in groups.into_values() {
        if group.len() == 1 {
            let seed = seeds[group[0]];
            let pair = adjacency.faces_for_edge(seed);
            tools.push(
                super::make_tool(
                    topo,
                    solid,
                    seed,
                    [pair[0], pair[1]],
                    distances[&seed],
                    true,
                )?
                .ok_or_else(|| crate::chamfer::unsupported("missing standalone sweep"))?,
            );
            continue;
        }
        let mut faces = Vec::new();
        for piece in group {
            faces.extend(
                solid_faces(topo, pieces[piece].solid)?
                    .into_iter()
                    .filter(|face| !remove.contains(face)),
            );
        }
        let shell = topo.add_shell(Shell::new(faces)?);
        let rough = topo.add_solid(Solid::new(shell, Vec::new()));
        // Isolate the graph before sewing: the source pieces still own their
        // original faces and must not be modified through shared wires.
        let joined = crate::copy::copy_solid(topo, rough)?;
        let shell = topo.solid(joined)?.outer_shell();
        let sewing = remus_heal::upgrade::shell_sewing::sew_shell_report(topo, shell, tol.linear)?;
        let report = crate::validate::validate_solid(topo, joined)?;
        if !report.is_valid() {
            log::debug!("joined sweep sewing={sewing:?}, validation={report:?}");
            return Err(crate::chamfer::unsupported(
                "joined analytic sweep is not a valid closed tool",
            ));
        }
        tools.push(Tool {
            solid: joined,
            concave: false,
            supports: Vec::new(),
            radial: true,
            plane: None,
        });
    }
    log::debug!(
        "joined {} analytic sweeps into {} tools",
        pieces.len(),
        tools.len()
    );
    let caps = super::corner_tools_with_distances(topo, solid, seeds, &distances)?;
    let mut refusal = None;
    for order in 0..tools.len() * 2 {
        let mut ordered = tools.clone();
        if order % 2 == 1 {
            ordered.reverse();
        }
        let shift = (order / 2) % ordered.len();
        ordered.rotate_left(shift);
        let mut all = caps.clone();
        all.extend(ordered);
        match remus_topology::transaction::run_transacted(topo, |t| {
            super::apply_tools(t, solid, seeds, &all)
        }) {
            Ok(result) => return Ok(result),
            Err(error) => {
                log::debug!("joined sweep order {order} refused: {error}");
                refusal = Some(error);
            }
        }
    }
    Err(refusal.unwrap_or_else(|| crate::chamfer::unsupported("empty joined sweep selection")))
}

fn root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}
