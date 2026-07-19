//! Final shell and solid assembly from offset faces and wire loops.

use brepkit_math::vec::Vec3;
use brepkit_topology::Topology;
use brepkit_topology::edge::{Edge, EdgeCurve, EdgeId};
use brepkit_topology::face::{Face, FaceId, FaceSurface};
use brepkit_topology::shell::Shell;
use brepkit_topology::solid::{Solid, SolidId};
use brepkit_topology::vertex::VertexId;
use brepkit_topology::wire::{OrientedEdge, Wire};

use crate::data::{OffsetData, OffsetStatus};
use crate::error::OffsetError;

/// Assemble the final offset solid from trimmed offset faces, joint
/// faces, and wire loops.
///
/// For each non-excluded offset face that has reconstructed wire loops,
/// a new [`Face`] is created with the offset surface and wires. All
/// new faces (including any joint faces from Phase 6) are collected
/// into a [`Shell`], which is then wrapped in a [`Solid`].
///
/// # Errors
///
/// Returns [`OffsetError::AssemblyFailed`] if no faces could be
/// assembled or the shell construction fails.
pub fn assemble_solid(topo: &mut Topology, data: &OffsetData) -> Result<SolidId, OffsetError> {
    let mut new_faces = Vec::new();
    let has_openings = !data.excluded_faces.is_empty();
    let mut offset_face_ids = data.offset_faces.keys().copied().collect::<Vec<_>>();
    offset_face_ids.sort_by_key(|face_id| face_id.index());

    for face_id in &offset_face_ids {
        let offset_face = &data.offset_faces[face_id];
        if offset_face.status == OffsetStatus::Excluded {
            continue;
        }

        let wires = data
            .face_wires
            .get(face_id)
            .ok_or_else(|| OffsetError::AssemblyFailed {
                reason: format!(
                    "offset face {} has no reconstructed wire loops",
                    face_id.index()
                ),
            })?;

        if wires.is_empty() {
            return Err(OffsetError::AssemblyFailed {
                reason: format!("offset face {} has an empty wire-loop set", face_id.index()),
            });
        }

        let outer_wire = wires[0];
        let inner_wires = wires[1..].to_vec();

        // A thick solid contains the original outer skin and an offset inner
        // skin, so the offset faces must oppose their source faces.
        let result_reversed = topo.face(offset_face.original)?.is_reversed() ^ has_openings;
        let face = if result_reversed {
            Face::new_reversed(outer_wire, inner_wires, offset_face.surface.clone())
        } else {
            Face::new(outer_wire, inner_wires, offset_face.surface.clone())
        };
        let face_id = topo.add_face(face);
        new_faces.push(face_id);
    }

    if has_openings {
        // Retain a cloned outer skin for every non-excluded source face. The
        // clone prevents shell orientation from mutating faces owned by the
        // caller's original solid while safely reusing its shared edges.
        for face_id in &offset_face_ids {
            let offset_face = &data.offset_faces[face_id];
            if offset_face.status == OffsetStatus::Done {
                new_faces.push(topo.add_face(topo.face(offset_face.original)?.clone()));
            }
        }
        let wall_faces = build_wall_faces(topo, data)?;
        new_faces.extend(wall_faces);
    }

    for &joint_face in &data.joint_faces {
        new_faces.push(joint_face);
    }

    if new_faces.is_empty() {
        return Err(OffsetError::AssemblyFailed {
            reason: "no faces could be assembled for the offset solid".to_string(),
        });
    }

    let shell = Shell::new(new_faces)?;
    let shell_id = topo.add_shell(shell);

    let solid = Solid::new(shell_id, vec![]);
    let solid_id = topo.add_solid(solid);
    orient_shell_faces(topo, shell_id)?;

    Ok(solid_id)
}

/// Make adjacent faces traverse every shared edge in opposite directions.
fn orient_shell_faces(
    topo: &mut Topology,
    shell_id: brepkit_topology::shell::ShellId,
) -> Result<(), OffsetError> {
    use std::collections::{HashMap, VecDeque};

    let face_ids = topo.shell(shell_id)?.faces().to_vec();
    let face_reversed = face_ids
        .iter()
        .map(|&face_id| topo.face(face_id).map(Face::is_reversed))
        .collect::<Result<Vec<_>, _>>()?;
    let mut edge_faces: HashMap<usize, Vec<(usize, bool)>> = HashMap::new();
    let mut face_edges = vec![Vec::new(); face_ids.len()];
    for (face_index, &face_id) in face_ids.iter().enumerate() {
        let face = topo.face(face_id)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                let edge_index = oriented.edge().index();
                edge_faces
                    .entry(edge_index)
                    .or_default()
                    .push((face_index, oriented.is_forward()));
                face_edges[face_index].push((edge_index, oriented.is_forward()));
            }
        }
    }

    let mut visited = vec![false; face_ids.len()];
    let mut flip = vec![false; face_ids.len()];
    for start in 0..face_ids.len() {
        if visited[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        while let Some(current) = queue.pop_front() {
            for &(edge_index, current_forward) in &face_edges[current] {
                let current_effective = current_forward != (face_reversed[current] ^ flip[current]);
                for &(neighbor, neighbor_forward) in &edge_faces[&edge_index] {
                    if neighbor == current || visited[neighbor] {
                        continue;
                    }
                    visited[neighbor] = true;
                    let neighbor_effective = neighbor_forward != face_reversed[neighbor];
                    flip[neighbor] = current_effective == neighbor_effective;
                    queue.push_back(neighbor);
                }
            }
        }
    }

    for (face_index, &face_id) in face_ids.iter().enumerate() {
        if flip[face_index] {
            let face = topo.face_mut(face_id)?;
            face.set_reversed(!face.is_reversed());
        }
    }
    Ok(())
}

/// Build wall faces connecting excluded face edges to offset vertices.
///
/// For each edge of an excluded face, we compute where the adjacent
/// non-excluded face's offset surface is at the original edge vertices,
/// create new vertices there, and build a quad wall face connecting
/// the original edge to the offset positions.
fn build_wall_faces(topo: &mut Topology, data: &OffsetData) -> Result<Vec<FaceId>, OffsetError> {
    use std::collections::HashMap;

    let mut wall_faces = Vec::new();
    let mut connector_edges: HashMap<(usize, usize), EdgeId> = HashMap::new();

    for &excluded_face_id in &data.excluded_faces {
        let outer_wire_id = topo.face(excluded_face_id)?.outer_wire();
        let wire_edges: Vec<_> = topo.wire(outer_wire_id)?.edges().to_vec();

        for oriented_edge in &wire_edges {
            let edge_id = oriented_edge.edge();

            let edge = topo.edge(edge_id)?;
            let p0_id = if oriented_edge.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            let p1_id = if oriented_edge.is_forward() {
                edge.end()
            } else {
                edge.start()
            };

            let p0 = topo.vertex(p0_id)?.point();
            let p1 = topo.vertex(p1_id)?.point();

            let excl_face = topo.face(excluded_face_id)?;
            match excl_face.surface() {
                FaceSurface::Plane { .. } => {}
                FaceSurface::Cylinder(_)
                | FaceSurface::Cone(_)
                | FaceSurface::Sphere(_)
                | FaceSurface::Torus(_)
                | FaceSurface::Nurbs(_) => {
                    return Err(OffsetError::InvalidInput {
                        reason: format!(
                            "wall generation for excluded non-planar face {} is not supported",
                            excluded_face_id.index()
                        ),
                    });
                }
            }

            let offset_edges = data
                .boundary_offset_edges
                .get(&edge_id.index())
                .ok_or_else(|| OffsetError::AssemblyFailed {
                    reason: format!(
                        "excluded boundary edge {} has no reconstructed offset edge",
                        edge_id.index()
                    ),
                })?;
            if offset_edges.len() != 1 {
                return Err(OffsetError::AssemblyFailed {
                    reason: format!(
                        "excluded boundary edge {} reconstructed into {} edges; planar wall assembly requires exactly one",
                        edge_id.index(),
                        offset_edges.len()
                    ),
                });
            }
            let offset_edge_id = offset_edges[0];
            let offset_edge = topo.edge(offset_edge_id)?;
            let offset_start = offset_edge.start();
            let offset_end = offset_edge.end();
            let offset_start_point = topo.vertex(offset_start)?.point();
            let offset_end_point = topo.vertex(offset_end)?.point();

            // Preserve correspondence along the original boundary direction.
            // The offset edge may have been created in either orientation.
            let aligned = point_distance_sq(p0, offset_start_point)
                + point_distance_sq(p1, offset_end_point)
                <= point_distance_sq(p0, offset_end_point)
                    + point_distance_sq(p1, offset_start_point);
            let (q0_id, q1_id, q0, q1) = if aligned {
                (
                    offset_start,
                    offset_end,
                    offset_start_point,
                    offset_end_point,
                )
            } else {
                (
                    offset_end,
                    offset_start,
                    offset_end_point,
                    offset_start_point,
                )
            };

            let (connector_1, connector_1_forward) =
                cached_line_edge(topo, &mut connector_edges, p1_id, q1_id)?;
            let offset_forward = edge_orientation_from(topo, offset_edge_id, q1_id)?;
            let (connector_0, connector_0_forward) =
                cached_line_edge(topo, &mut connector_edges, q0_id, p0_id)?;
            let wall_edges = vec![
                *oriented_edge,
                OrientedEdge::new(connector_1, connector_1_forward),
                OrientedEdge::new(offset_edge_id, offset_forward),
                OrientedEdge::new(connector_0, connector_0_forward),
            ];

            let face_id = make_wall_quad(topo, wall_edges, p0, p1, q1, q0)?.ok_or_else(|| {
                OffsetError::AssemblyFailed {
                    reason: format!(
                        "degenerate wall quad for excluded face {} edge {}",
                        excluded_face_id.index(),
                        edge_id.index()
                    ),
                }
            })?;
            wall_faces.push(face_id);
        }
    }

    Ok(wall_faces)
}

/// Create a planar quad wall face from 4 vertices (p0→p1→p2→p3).
///
/// Returns `None` if the quad is degenerate (zero area).
#[allow(clippy::too_many_arguments)]
fn make_wall_quad(
    topo: &mut Topology,
    wall_edges: Vec<OrientedEdge>,
    p0: brepkit_math::vec::Point3,
    p1: brepkit_math::vec::Point3,
    _p2: brepkit_math::vec::Point3,
    p3: brepkit_math::vec::Point3,
) -> Result<Option<FaceId>, OffsetError> {
    let edge_a = p1 - p0;
    let edge_b = p3 - p0;
    let cross = edge_a.cross(edge_b);
    let len = cross.length();
    // Degenerate quad (zero area cross product).
    if len < 1e-15 {
        return Ok(None);
    }
    let normal = cross * (1.0 / len);
    let d = normal.dot(Vec3::new(p0.x(), p0.y(), p0.z()));

    let wire = Wire::new(wall_edges, true)?;
    let wire_id = topo.add_wire(wire);
    let face = Face::new(wire_id, vec![], FaceSurface::Plane { normal, d });
    Ok(Some(topo.add_face(face)))
}

fn point_distance_sq(a: brepkit_math::vec::Point3, b: brepkit_math::vec::Point3) -> f64 {
    let delta = a - b;
    delta.dot(delta)
}

fn edge_orientation_from(
    topo: &Topology,
    edge_id: EdgeId,
    from: VertexId,
) -> Result<bool, OffsetError> {
    let edge = topo.edge(edge_id)?;
    if edge.start() == from {
        Ok(true)
    } else if edge.end() == from {
        Ok(false)
    } else {
        Err(OffsetError::AssemblyFailed {
            reason: format!(
                "vertex {} is not an endpoint of edge {}",
                from.index(),
                edge_id.index()
            ),
        })
    }
}

fn cached_line_edge(
    topo: &mut Topology,
    cache: &mut std::collections::HashMap<(usize, usize), EdgeId>,
    from: VertexId,
    to: VertexId,
) -> Result<(EdgeId, bool), OffsetError> {
    let key = if from.index() < to.index() {
        (from.index(), to.index())
    } else {
        (to.index(), from.index())
    };
    let edge_id = if let Some(&existing) = cache.get(&key) {
        existing
    } else {
        let created = topo.add_edge(Edge::new(from, to, EdgeCurve::Line));
        cache.insert(key, created);
        created
    };
    Ok((edge_id, edge_orientation_from(topo, edge_id, from)?))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use crate::data::{OffsetData, OffsetOptions};
    use brepkit_topology::Topology;

    fn run_full_pipeline(topo: &mut Topology, solid: SolidId, distance: f64) -> SolidId {
        let mut data = OffsetData::new(distance, OffsetOptions::default(), vec![]);
        crate::analyse::analyse_edges(topo, solid, &mut data).unwrap();
        crate::offset::build_offset_faces(topo, solid, &mut data).unwrap();
        crate::inter3d::intersect_faces_3d(topo, solid, &mut data).unwrap();
        crate::inter2d::intersect_pcurves_2d(topo, solid, &mut data).unwrap();
        crate::loops::build_wire_loops(topo, &mut data).unwrap();
        assemble_solid(topo, &data).unwrap()
    }

    #[test]
    fn box_offset_produces_valid_solid() {
        let mut topo = Topology::new();
        let solid = brepkit_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let result = run_full_pipeline(&mut topo, solid, 0.5);

        let shell_id = topo.solid(result).unwrap().outer_shell();
        let shell = topo.shell(shell_id).unwrap();
        assert_eq!(shell.faces().len(), 6, "offset box should have 6 faces");
    }

    #[test]
    fn box_offset_faces_have_wires() {
        let mut topo = Topology::new();
        let solid = brepkit_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let result = run_full_pipeline(&mut topo, solid, 0.5);

        let shell_id = topo.solid(result).unwrap().outer_shell();
        let shell = topo.shell(shell_id).unwrap();
        for &fid in shell.faces() {
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            assert_eq!(wire.edges().len(), 4, "each face should have 4 edges");
        }
    }

    #[test]
    fn box_offset_end_to_end() {
        let mut topo = Topology::new();
        let solid = brepkit_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let result = crate::offset_solid(
            &mut topo,
            solid,
            0.5,
            OffsetOptions {
                remove_self_intersections: false,
                ..Default::default()
            },
        )
        .unwrap();

        let shell_id = topo.solid(result).unwrap().outer_shell();
        let shell = topo.shell(shell_id).unwrap();
        assert_eq!(shell.faces().len(), 6);
    }

    fn run_thick_pipeline(
        topo: &mut Topology,
        solid: SolidId,
        distance: f64,
        exclude: Vec<FaceId>,
    ) -> SolidId {
        let mut data = OffsetData::new(distance, OffsetOptions::default(), exclude);
        crate::analyse::analyse_edges(topo, solid, &mut data).unwrap();
        crate::offset::build_offset_faces(topo, solid, &mut data).unwrap();
        crate::inter3d::intersect_faces_3d(topo, solid, &mut data).unwrap();
        crate::inter2d::intersect_pcurves_2d(topo, solid, &mut data).unwrap();
        crate::loops::build_wire_loops(topo, &mut data).unwrap();
        assemble_solid(topo, &data).unwrap()
    }

    /// Thick solid with excluded face.
    ///
    /// Currently the wire loop builder doesn't handle faces adjacent to
    /// excluded faces (their loop is incomplete because the shared edge
    /// has no intersection). This test documents the current limitation.
    #[test]
    fn thick_solid_with_excluded_face() {
        let mut topo = Topology::new();
        let solid = brepkit_topology::test_utils::make_unit_cube_manifold(&mut topo);
        let shell_id = topo.solid(solid).unwrap().outer_shell();
        let faces: Vec<_> = topo.shell(shell_id).unwrap().faces().to_vec();

        let exclude = vec![faces[0]];
        let result = run_thick_pipeline(&mut topo, solid, -0.1, exclude);

        let result_shell = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        assert!(
            result_shell.faces().len() >= 9,
            "thick solid should have at least 9 faces (5 offset + 4 walls), got {}",
            result_shell.faces().len()
        );
    }
}
