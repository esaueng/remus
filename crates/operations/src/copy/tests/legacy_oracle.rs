//! The map-based solid copy paths that `CopyPlan` replaced, kept verbatim
//! as differential oracles: every copy must leave the arena `Debug`-identical
//! (allocation order, tombstones, pcurve registry, coedge authority, mutation
//! ticks) and return identical entity maps.

use std::collections::HashMap;

use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::attributes::EntityAttributes;
use remus_topology::coedge::PeriodicWinding;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::transaction::run_transacted;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::copy::CopiedSolidEntities;
use crate::transform::{TransformPolicy, TransformRecorder};

/// Verbatim from the parent commit apart from item visibility; the
/// snapshot and restore helpers they share with `copy_face` are live.
#[allow(clippy::too_many_lines)]
mod legacy {
    use std::collections::HashMap;

    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceId, FaceSurface};
    use remus_topology::shell::Shell;
    use remus_topology::solid::{Solid, SolidId};
    use remus_topology::vertex::{Vertex, VertexId};
    use remus_topology::wire::{OrientedEdge, Wire, WireId};

    use crate::copy::{
        CoedgeAuthoritySnap, CopiedSolidEntities, EdgeSnap, VertexSnap, WireSnap,
        remapped_authority_edge, restore_face_coedge_authority, snapshot_face_coedge_authority,
    };
    use crate::transform::TransformRecorder;

    struct FaceSnap {
        old_index: usize,
        outer_wire_index: usize,
        inner_wire_indices: Vec<usize>,
        surface: FaceSurface,
        reversed: bool,
        attributes: Option<remus_topology::attributes::EntityAttributes>,
    }

    struct ShellSnap {
        faces: Vec<FaceSnap>,
    }

    fn remapped_authority_face(
        face_map: &HashMap<usize, FaceId>,
        snapshot: &CoedgeAuthoritySnap,
    ) -> Result<FaceId, crate::OperationsError> {
        face_map
            .get(&snapshot.face_index)
            .copied()
            .ok_or_else(|| remus_topology::TopologyError::NonManifold {
                reason: format!(
                    "copy plan has no target face for authoritative source face index {}",
                    snapshot.face_index
                ),
            })
            .map_err(Into::into)
    }

    pub(super) fn copy_solid_between_with_entity_map(
        source: &Topology,
        destination: &mut Topology,
        solid_id: SolidId,
    ) -> Result<CopiedSolidEntities, crate::OperationsError> {
        let solid = source.solid(solid_id)?;
        let solid_attributes = source.attributes().solid(solid_id).cloned();
        let shell_ids: Vec<_> = std::iter::once(solid.outer_shell())
            .chain(solid.inner_shells().iter().copied())
            .collect();
        let mut vertices = Vec::new();
        let mut edges = Vec::new();
        let mut wires = Vec::new();
        let mut shells = Vec::new();
        let mut coedge_authority = Vec::new();
        let mut seen_vertices = std::collections::HashSet::new();
        let mut seen_edges = std::collections::HashSet::new();
        let mut seen_wires = std::collections::HashSet::new();

        for shell_id in shell_ids {
            let shell = source.shell(shell_id)?;
            let mut faces = Vec::new();
            for &face_id in shell.faces() {
                let face = source.face(face_id)?;
                coedge_authority.extend(snapshot_face_coedge_authority(source, face_id)?);
                for wire_id in
                    std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                {
                    if !seen_wires.insert(wire_id.index()) {
                        continue;
                    }
                    let wire = source.wire(wire_id)?;
                    let mut edge_refs = Vec::new();
                    for oriented in wire.edges() {
                        let edge_id = oriented.edge();
                        edge_refs.push((edge_id.index(), oriented.is_forward()));
                        if !seen_edges.insert(edge_id.index()) {
                            continue;
                        }
                        let edge = source.edge(edge_id)?;
                        for vertex_id in [edge.start(), edge.end()] {
                            if seen_vertices.insert(vertex_id.index()) {
                                let vertex = source.vertex(vertex_id)?;
                                vertices.push(VertexSnap {
                                    old_index: vertex_id.index(),
                                    point: vertex.point(),
                                    tol: vertex.tolerance(),
                                });
                            }
                        }
                        edges.push(EdgeSnap {
                            old_index: edge_id.index(),
                            start_index: edge.start().index(),
                            end_index: edge.end().index(),
                            curve: edge.curve().clone(),
                            tolerance: edge.tolerance(),
                            trim: edge.trim(),
                        });
                    }
                    wires.push(WireSnap {
                        old_index: wire_id.index(),
                        edges: edge_refs,
                        closed: wire.is_closed(),
                    });
                }
                faces.push(FaceSnap {
                    old_index: face_id.index(),
                    outer_wire_index: face.outer_wire().index(),
                    inner_wire_indices: face
                        .inner_wires()
                        .iter()
                        .map(|wire| wire.index())
                        .collect(),
                    surface: face.surface().clone(),
                    reversed: face.is_reversed(),
                    attributes: source.attributes().face(face_id).cloned(),
                });
            }
            shells.push(ShellSnap { faces });
        }

        destination.reserve(
            vertices.len(),
            edges.len(),
            wires.len(),
            shells.iter().map(|shell| shell.faces.len()).sum(),
            shells.len(),
            1,
        );
        let mut vertex_map = HashMap::new();
        for vertex in vertices {
            vertex_map.insert(
                vertex.old_index,
                destination.add_vertex(Vertex::new(vertex.point, vertex.tol)),
            );
        }
        let mut edge_map = HashMap::new();
        for edge in edges {
            edge_map.insert(
                edge.old_index,
                destination.add_edge({
                    let mut copied = Edge::with_tolerance(
                        vertex_map[&edge.start_index],
                        vertex_map[&edge.end_index],
                        edge.curve,
                        edge.tolerance,
                    );
                    copied.set_trim(edge.trim);
                    copied
                }),
            );
        }
        let mut wire_map = HashMap::new();
        for wire in wires {
            let oriented = wire
                .edges
                .into_iter()
                .map(|(edge, forward)| OrientedEdge::new(edge_map[&edge], forward))
                .collect();
            wire_map.insert(
                wire.old_index,
                destination.add_wire(
                    Wire::new(oriented, wire.closed).map_err(crate::OperationsError::Topology)?,
                ),
            );
        }
        let mut new_shells = Vec::new();
        let mut face_map = HashMap::new();
        for shell in shells {
            let mut new_faces = Vec::new();
            for face in shell.faces {
                let outer = wire_map[&face.outer_wire_index];
                let inner = face
                    .inner_wire_indices
                    .iter()
                    .map(|index| wire_map[index])
                    .collect();
                let new_face = if face.reversed {
                    Face::new_reversed(outer, inner, face.surface)
                } else {
                    Face::new(outer, inner, face.surface)
                };
                let new_face_id = destination.add_face(new_face);
                if let Some(attributes) = face.attributes {
                    destination.set_face_attributes(new_face_id, attributes)?;
                }
                face_map.insert(face.old_index, new_face_id);
                new_faces.push(new_face_id);
            }
            new_shells.push(
                destination
                    .add_shell(Shell::new(new_faces).map_err(crate::OperationsError::Topology)?),
            );
        }
        for snapshot in coedge_authority {
            let face = remapped_authority_face(&face_map, &snapshot)?;
            let edge = remapped_authority_edge(&edge_map, &snapshot)?;
            restore_face_coedge_authority(destination, face, edge, snapshot)?;
        }
        let outer = new_shells[0];
        let inner = new_shells[1..].to_vec();
        let copied = destination.add_solid(Solid::new(outer, inner));
        if let Some(attributes) = solid_attributes {
            destination.set_solid_attributes(copied, attributes)?;
        }
        Ok(CopiedSolidEntities {
            solid: copied,
            face_map,
            edge_map,
            vertex_map,
        })
    }

    #[allow(clippy::too_many_lines)]
    pub(super) fn copy_solid_with_entity_map(
        topo: &mut Topology,
        solid_id: SolidId,
    ) -> Result<CopiedSolidEntities, crate::OperationsError> {
        let solid = topo.solid(solid_id)?;
        let solid_attributes = topo.attributes().solid(solid_id).cloned();
        let outer_shell_id = solid.outer_shell();
        let inner_shell_ids: Vec<_> = solid.inner_shells().to_vec();

        let all_shell_ids: Vec<_> = std::iter::once(outer_shell_id)
            .chain(inner_shell_ids.iter().copied())
            .collect();

        let mut vertex_snaps: Vec<VertexSnap> = Vec::new();
        let mut edge_snaps: Vec<EdgeSnap> = Vec::new();
        let mut wire_snaps: Vec<WireSnap> = Vec::new();
        let mut shell_snaps: Vec<ShellSnap> = Vec::new();
        let mut coedge_authority_snaps = Vec::new();

        let mut seen_vertices = std::collections::HashSet::new();
        let mut seen_edges = std::collections::HashSet::new();
        let mut seen_wires = std::collections::HashSet::new();

        for &shell_id in &all_shell_ids {
            let shell = topo.shell(shell_id)?;
            let mut face_snaps = Vec::new();

            for &face_id in shell.faces() {
                let face = topo.face(face_id)?;
                coedge_authority_snaps.extend(snapshot_face_coedge_authority(topo, face_id)?);
                let surface = face.surface().clone();
                let outer_wire_index = face.outer_wire().index();
                let inner_wire_indices: Vec<usize> =
                    face.inner_wires().iter().map(|w| w.index()).collect();

                for wire_id_val in
                    std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                {
                    if !seen_wires.insert(wire_id_val.index()) {
                        continue;
                    }
                    let wire = topo.wire(wire_id_val)?;
                    let mut edge_refs = Vec::new();

                    for oe in wire.edges() {
                        let edge_idx = oe.edge().index();
                        edge_refs.push((edge_idx, oe.is_forward()));

                        if !seen_edges.insert(edge_idx) {
                            continue;
                        }
                        let edge = topo.edge(oe.edge())?;
                        let start_idx = edge.start().index();
                        let end_idx = edge.end().index();

                        for &vid_idx in &[start_idx, end_idx] {
                            if seen_vertices.insert(vid_idx) {
                                let vid = if vid_idx == start_idx {
                                    edge.start()
                                } else {
                                    edge.end()
                                };
                                let v = topo.vertex(vid)?;
                                vertex_snaps.push(VertexSnap {
                                    old_index: vid_idx,
                                    point: v.point(),
                                    tol: v.tolerance(),
                                });
                            }
                        }

                        edge_snaps.push(EdgeSnap {
                            old_index: edge_idx,
                            start_index: start_idx,
                            end_index: end_idx,
                            curve: edge.curve().clone(),
                            tolerance: edge.tolerance(),
                            trim: edge.trim(),
                        });
                    }

                    wire_snaps.push(WireSnap {
                        old_index: wire_id_val.index(),
                        edges: edge_refs,
                        closed: wire.is_closed(),
                    });
                }

                face_snaps.push(FaceSnap {
                    old_index: face_id.index(),
                    outer_wire_index,
                    inner_wire_indices,
                    surface,
                    reversed: face.is_reversed(),
                    attributes: topo.attributes().face(face_id).cloned(),
                });
            }

            shell_snaps.push(ShellSnap { faces: face_snaps });
        }

        topo.reserve(
            vertex_snaps.len(),
            edge_snaps.len(),
            wire_snaps.len(),
            shell_snaps
                .iter()
                .map(|s| s.faces.len())
                .fold(0usize, usize::saturating_add),
            shell_snaps.len(),
            1,
        );

        let mut vertex_map: HashMap<usize, VertexId> = HashMap::new();
        for vsnap in &vertex_snaps {
            let new_vid = topo.add_vertex(Vertex::new(vsnap.point, vsnap.tol));
            vertex_map.insert(vsnap.old_index, new_vid);
        }

        let mut edge_map: HashMap<usize, remus_topology::edge::EdgeId> = HashMap::new();
        for esnap in &edge_snaps {
            let new_start = vertex_map[&esnap.start_index];
            let new_end = vertex_map[&esnap.end_index];
            let copied_edge = topo.add_edge({
                let mut copied =
                    Edge::with_tolerance(new_start, new_end, esnap.curve.clone(), esnap.tolerance);
                copied.set_trim(esnap.trim);
                copied
            });
            edge_map.insert(esnap.old_index, copied_edge);
        }

        let mut wire_map: HashMap<usize, WireId> = HashMap::new();
        for wsnap in &wire_snaps {
            let new_edges: Vec<OrientedEdge> = wsnap
                .edges
                .iter()
                .map(|&(edge_idx, fwd)| OrientedEdge::new(edge_map[&edge_idx], fwd))
                .collect();
            let new_wire =
                Wire::new(new_edges, wsnap.closed).map_err(crate::OperationsError::Topology)?;
            wire_map.insert(wsnap.old_index, topo.add_wire(new_wire));
        }

        let mut new_shell_ids = Vec::new();
        let mut copied_face_ids: HashMap<usize, FaceId> = HashMap::new();
        for ssnap in &shell_snaps {
            let mut new_face_ids = Vec::new();
            for fsnap in &ssnap.faces {
                let new_outer = wire_map[&fsnap.outer_wire_index];
                let new_inner: Vec<WireId> = fsnap
                    .inner_wire_indices
                    .iter()
                    .map(|idx| wire_map[idx])
                    .collect();
                let new_face = if fsnap.reversed {
                    Face::new_reversed(new_outer, new_inner, fsnap.surface.clone())
                } else {
                    Face::new(new_outer, new_inner, fsnap.surface.clone())
                };
                let new_fid = topo.add_face(new_face);
                if let Some(attributes) = fsnap.attributes.clone() {
                    topo.set_face_attributes(new_fid, attributes)?;
                }
                copied_face_ids.insert(fsnap.old_index, new_fid);
                new_face_ids.push(new_fid);
            }
            let new_shell = Shell::new(new_face_ids).map_err(crate::OperationsError::Topology)?;
            new_shell_ids.push(topo.add_shell(new_shell));
        }
        for snapshot in coedge_authority_snaps {
            let face = remapped_authority_face(&copied_face_ids, &snapshot)?;
            let edge = remapped_authority_edge(&edge_map, &snapshot)?;
            restore_face_coedge_authority(topo, face, edge, snapshot)?;
        }

        let new_outer = new_shell_ids[0];
        let new_inner: Vec<_> = new_shell_ids[1..].to_vec();

        let new_solid = topo.add_solid(Solid::new(new_outer, new_inner));
        if let Some(attributes) = solid_attributes {
            topo.set_solid_attributes(new_solid, attributes)?;
        }
        Ok(CopiedSolidEntities {
            solid: new_solid,
            face_map: copied_face_ids,
            edge_map,
            vertex_map,
        })
    }

    #[allow(clippy::too_many_lines)]
    pub(super) fn copy_and_transform_solid_impl(
        topo: &mut Topology,
        solid_id: SolidId,
        matrix: &remus_math::mat::Mat4,
        normal_matrix: &remus_math::mat::Mat4,
        recorder: &mut TransformRecorder,
    ) -> Result<SolidId, crate::OperationsError> {
        let chart_reversing = crate::transform::linear_determinant(matrix) < 0.0;
        let certificates = crate::transform::translation_edge_certificates(
            topo,
            &remus_topology::explorer::solid_edges(topo, solid_id)?,
            matrix,
        )?;

        // Read phase mirrors copy_solid.
        let solid = topo.solid(solid_id)?;
        let solid_attributes = topo.attributes().solid(solid_id).cloned();
        let outer_shell_id = solid.outer_shell();
        let inner_shell_ids: Vec<_> = solid.inner_shells().to_vec();

        let all_shell_ids: Vec<_> = std::iter::once(outer_shell_id)
            .chain(inner_shell_ids.iter().copied())
            .collect();

        let mut vertex_snaps: Vec<VertexSnap> = Vec::new();
        let mut edge_snaps: Vec<EdgeSnap> = Vec::new();
        let mut wire_snaps: Vec<WireSnap> = Vec::new();
        let mut shell_snaps: Vec<ShellSnap> = Vec::new();
        let mut coedge_authority_snaps = Vec::new();

        let mut seen_vertices = std::collections::HashSet::new();
        let mut seen_edges = std::collections::HashSet::new();
        let mut seen_wires = std::collections::HashSet::new();

        for &shell_id in &all_shell_ids {
            let shell = topo.shell(shell_id)?;
            let mut face_snaps = Vec::new();

            for &face_id in shell.faces() {
                let face = topo.face(face_id)?;
                coedge_authority_snaps.extend(snapshot_face_coedge_authority(topo, face_id)?);
                let surface = face.surface().clone();
                let outer_wire_index = face.outer_wire().index();
                let inner_wire_indices: Vec<usize> =
                    face.inner_wires().iter().map(|w| w.index()).collect();

                for wire_id_val in
                    std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
                {
                    if !seen_wires.insert(wire_id_val.index()) {
                        continue;
                    }
                    let wire = topo.wire(wire_id_val)?;
                    let mut edge_refs = Vec::new();

                    for oe in wire.edges() {
                        let edge_idx = oe.edge().index();
                        edge_refs.push((edge_idx, oe.is_forward()));

                        if !seen_edges.insert(edge_idx) {
                            continue;
                        }
                        let edge = topo.edge(oe.edge())?;
                        let start_idx = edge.start().index();
                        let end_idx = edge.end().index();

                        for &vid_idx in &[start_idx, end_idx] {
                            if seen_vertices.insert(vid_idx) {
                                let vid = if vid_idx == start_idx {
                                    edge.start()
                                } else {
                                    edge.end()
                                };
                                let v = topo.vertex(vid)?;
                                vertex_snaps.push(VertexSnap {
                                    old_index: vid_idx,
                                    point: v.point(),
                                    tol: v.tolerance(),
                                });
                            }
                        }

                        edge_snaps.push(EdgeSnap {
                            old_index: edge_idx,
                            start_index: start_idx,
                            end_index: end_idx,
                            curve: edge.curve().clone(),
                            tolerance: edge.tolerance(),
                            trim: edge.trim(),
                        });
                    }

                    wire_snaps.push(WireSnap {
                        old_index: wire_id_val.index(),
                        edges: edge_refs,
                        closed: wire.is_closed(),
                    });
                }

                face_snaps.push(FaceSnap {
                    old_index: face_id.index(),
                    outer_wire_index,
                    inner_wire_indices,
                    surface,
                    reversed: face.is_reversed(),
                    attributes: topo.attributes().face(face_id).cloned(),
                });
            }

            shell_snaps.push(ShellSnap { faces: face_snaps });
        }

        topo.reserve(
            vertex_snaps.len(),
            edge_snaps.len(),
            wire_snaps.len(),
            shell_snaps
                .iter()
                .map(|s| s.faces.len())
                .fold(0usize, usize::saturating_add),
            shell_snaps.len(),
            1,
        );

        let mut vertex_map: HashMap<usize, VertexId> = HashMap::new();
        for vsnap in &vertex_snaps {
            let new_point = matrix.mul_point(vsnap.point);
            let new_vid = topo.add_vertex(Vertex::new(new_point, vsnap.tol));
            vertex_map.insert(vsnap.old_index, new_vid);
        }

        let mut edge_map: HashMap<usize, remus_topology::edge::EdgeId> = HashMap::new();
        for esnap in &edge_snaps {
            let new_start = vertex_map[&esnap.start_index];
            let new_end = vertex_map[&esnap.end_index];
            // Shared with `transform::transform_edges` — including its exact trim
            // policy: retain where the map provably preserves the
            // parameterization, remap the handled Circle→Ellipse axis swap,
            // drop otherwise (RFC 0002).
            let from = esnap.curve.type_tag();
            let (new_curve, new_trim) =
                crate::transform::transform_edge_curve_with_trim(&esnap.curve, esnap.trim, matrix)?;
            let to = new_curve.as_ref().map_or(from, |curve| curve.type_tag());
            let mut copied_edge = Edge::with_tolerance(
                new_start,
                new_end,
                new_curve.unwrap_or(EdgeCurve::Line),
                esnap.tolerance,
            );
            copied_edge.set_trim(new_trim);
            let copied = topo.add_edge(copied_edge);
            recorder.record_edge(copied, from, to);
            edge_map.insert(esnap.old_index, copied);
        }

        crate::transform::restore_translation_certificates(
            topo,
            certificates
                .into_iter()
                .map(|(id, tolerance, budget)| (edge_map[&id.index()], tolerance, budget))
                .collect(),
        )?;

        // Wires carry no geometry to transform.
        let mut wire_map: HashMap<usize, WireId> = HashMap::new();
        for wsnap in &wire_snaps {
            let new_edges: Vec<OrientedEdge> = wsnap
                .edges
                .iter()
                .map(|&(edge_idx, fwd)| OrientedEdge::new(edge_map[&edge_idx], fwd))
                .collect();
            let new_wire =
                Wire::new(new_edges, wsnap.closed).map_err(crate::OperationsError::Topology)?;
            wire_map.insert(wsnap.old_index, topo.add_wire(new_wire));
        }

        let mut new_shell_ids = Vec::new();
        let mut copied_face_ids = HashMap::new();
        let mut chart_replaced_faces = Vec::new();
        for ssnap in &shell_snaps {
            let mut new_face_ids = Vec::new();
            for fsnap in &ssnap.faces {
                let new_outer = wire_map[&fsnap.outer_wire_index];
                let new_inner: Vec<WireId> = fsnap
                    .inner_wire_indices
                    .iter()
                    .map(|idx| wire_map[idx])
                    .collect();

                // Copy the surface verbatim; the shared transformer below rewrites
                // it once the face exists. The old inline math here diverged from
                // `transform_solid` — it never scaled cylinder/sphere/torus radii
                // and had no anisotropic-scale handling at all, so "equivalent to
                // copy_solid followed by transform_solid" was untrue for any
                // scaling matrix.
                let new_surface = fsnap.surface.clone();

                let new_face = if fsnap.reversed {
                    Face::new_reversed(new_outer, new_inner, new_surface)
                } else {
                    Face::new(new_outer, new_inner, new_surface)
                };
                let new_fid = topo.add_face(new_face);
                // Vertices and edge curves were written at their transformed
                // positions above, which is exactly the state
                // `transform_face_surface` expects (its non-uniform branches map
                // boundary probes back through the inverse).
                recorder.set_refusal_origin(topo.face_id_from_index(fsnap.old_index));
                crate::transform::transform_face_surface_recorded(
                    topo,
                    new_fid,
                    matrix,
                    normal_matrix,
                    recorder,
                )?;
                recorder.set_refusal_origin(None);
                if matches!(topo.face(new_fid)?.surface(), FaceSurface::Nurbs(_))
                    && (!matches!(&fsnap.surface, FaceSurface::Nurbs(_)) || chart_reversing)
                {
                    chart_replaced_faces.push(new_fid);
                }
                if let Some(attributes) = fsnap.attributes.clone() {
                    topo.set_face_attributes(new_fid, attributes)?;
                }
                copied_face_ids.insert(fsnap.old_index, new_fid);
                new_face_ids.push(new_fid);
            }
            let new_shell = Shell::new(new_face_ids).map_err(crate::OperationsError::Topology)?;
            new_shell_ids.push(topo.add_shell(new_shell));
        }

        for snapshot in coedge_authority_snaps {
            let face = remapped_authority_face(&copied_face_ids, &snapshot)?;
            let edge = remapped_authority_edge(&edge_map, &snapshot)?;
            restore_face_coedge_authority(topo, face, edge, snapshot)?;
        }
        for face in chart_replaced_faces {
            let uses: Vec<_> = topo
                .pcurves_for_face(face)
                .into_iter()
                .map(|(edge, forward, _)| (edge, forward))
                .collect();
            for (edge, forward) in uses {
                topo.remove_pcurve_oriented(edge, face, forward)?;
            }
        }

        let new_outer = new_shell_ids[0];
        let new_inner: Vec<_> = new_shell_ids[1..].to_vec();

        let copied = topo.add_solid(Solid::new(new_outer, new_inner));
        if let Some(attributes) = solid_attributes {
            topo.set_solid_attributes(copied, attributes)?;
        }
        Ok(copied)
    }
}

fn legacy_copy_and_transform(
    topo: &mut Topology,
    solid: SolidId,
    matrix: &Mat4,
    policy: TransformPolicy,
) -> Result<(SolidId, String), crate::OperationsError> {
    crate::transform::reject_degenerate_transform(matrix)?;
    let normal_matrix = matrix.inverse()?.transpose();
    let determinant = crate::transform::linear_determinant(matrix);
    let mut recorder = TransformRecorder::new(
        determinant < 0.0,
        matches!(policy, TransformPolicy::AllowApproximate),
    );
    let copied = run_transacted(topo, |live| {
        legacy::copy_and_transform_solid_impl(live, solid, matrix, &normal_matrix, &mut recorder)
    })?;
    let report = recorder.into_report(determinant, crate::transform::is_similarity(matrix));
    Ok((copied, format!("{report:?}")))
}

/// The returned entities with every map sorted.
fn entities(copied: Result<CopiedSolidEntities, crate::OperationsError>) -> String {
    let copied = match copied {
        Ok(copied) => copied,
        Err(error) => return format!("{error:?}"),
    };
    let sorted = |mut map: Vec<(usize, usize)>| {
        map.sort_unstable();
        map
    };
    format!(
        "{:?} {:?} {:?} {:?}",
        copied.solid,
        sorted(
            copied
                .face_map
                .iter()
                .map(|(&k, v)| (k, v.index()))
                .collect()
        ),
        sorted(
            copied
                .edge_map
                .iter()
                .map(|(&k, v)| (k, v.index()))
                .collect()
        ),
        sorted(
            copied
                .vertex_map
                .iter()
                .map(|(&k, v)| (k, v.index()))
                .collect()
        ),
    )
}

fn face_map(map: Result<(SolidId, HashMap<usize, usize>), crate::OperationsError>) -> String {
    map.map(|(solid, map)| {
        let mut map: Vec<_> = map.into_iter().collect();
        map.sort_unstable();
        format!("{solid:?} {map:?}")
    })
    .unwrap_or_else(|error| format!("{error:?}"))
}

/// Runs `new` and `old` on clones of `fixture` and asserts identical
/// results, `Debug`-identical arenas, and identical cache invalidation.
fn assert_same(
    label: &str,
    fixture: &Topology,
    new: impl FnOnce(&mut Topology) -> String,
    old: impl FnOnce(&mut Topology) -> String,
) {
    let mut planned = fixture.clone();
    let mut mapped = fixture.clone();
    let planned_generation = planned.cache_generation();
    let mapped_generation = mapped.cache_generation();
    assert_eq!(new(&mut planned), old(&mut mapped), "{label}");
    assert_eq!(format!("{planned:?}"), format!("{mapped:?}"), "{label}");
    assert_eq!(
        planned.cache_generation() - planned_generation,
        mapped.cache_generation() - mapped_generation,
        "{label}: cache invalidations"
    );
}

/// Stores a lifted pcurve and a nonzero periodic winding on `face`'s uses.
fn add_authority(topo: &mut Topology, face: FaceId) {
    let coedges = crate::copy::face_coedges(topo, face).unwrap();
    let pcurve = PCurve::new(
        Curve2D::Line(Line2D::new(Point2::new(2.0, 3.0), Vec2::new(1.0, 0.0)).unwrap()),
        4.0,
        5.0,
    );
    topo.set_coedge_pcurve(coedges[0], pcurve).unwrap();
    topo.set_coedge_periodic_winding(coedges[coedges.len() - 1], PeriodicWinding::new(3, -2))
        .unwrap();
}

fn plane() -> FaceSurface {
    FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 0.0,
    }
}

#[allow(clippy::too_many_lines)]
fn copy_fixtures() -> Vec<(&'static str, Topology, SolidId)> {
    let mut fixtures = Vec::new();
    let mut add = |name, build: &dyn Fn(&mut Topology) -> SolidId| {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        fixtures.push((name, topo, solid));
    };
    add("box", &|t| {
        crate::primitives::make_box(t, 2.0, 3.0, 4.0).unwrap()
    });
    add("attributed box", &|t| {
        let solid = crate::primitives::make_box(t, 2.0, 3.0, 4.0).unwrap();
        let named = |name: &str| EntityAttributes {
            name: Some(name.to_owned()),
            ..EntityAttributes::default()
        };
        t.set_solid_attributes(solid, named("solid")).unwrap();
        for (i, face) in solid_faces(t, solid)
            .unwrap()
            .into_iter()
            .take(2)
            .enumerate()
        {
            t.set_face_attributes(face, named(&format!("face {i}")))
                .unwrap();
        }
        solid
    });
    add("cylinder", &|t| {
        let solid = crate::primitives::make_cylinder(t, 1.5, 4.0).unwrap();
        for face in solid_faces(t, solid).unwrap() {
            add_authority(t, face);
        }
        solid
    });
    add("sphere", &|t| {
        crate::primitives::make_sphere(t, 2.0, 16).unwrap()
    });
    add("torus", &|t| {
        crate::primitives::make_torus(t, 3.0, 1.0, 16).unwrap()
    });
    add("hollow box", &|t| {
        let outer = crate::primitives::make_box(t, 3.0, 3.0, 3.0).unwrap();
        let inner = crate::primitives::make_box(t, 1.0, 1.0, 1.0).unwrap();
        crate::transform::transform_solid(t, inner, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
        let hollow =
            crate::boolean::boolean(t, crate::boolean::BooleanOp::Cut, outer, inner).unwrap();
        assert_eq!(t.solid(hollow).unwrap().inner_shells().len(), 1);
        hollow
    });
    add("holed plate", &|t| {
        let plate = crate::primitives::make_box(t, 4.0, 4.0, 1.0).unwrap();
        let bore = crate::primitives::make_cylinder(t, 1.0, 3.0).unwrap();
        crate::transform::transform_solid(t, bore, &Mat4::translation(2.0, 2.0, -1.0)).unwrap();
        let holed =
            crate::boolean::boolean(t, crate::boolean::BooleanOp::Cut, plate, bore).unwrap();
        assert!(
            solid_faces(t, holed).unwrap().into_iter().any(|face| !t
                .face(face)
                .unwrap()
                .inner_wires()
                .is_empty())
        );
        holed
    });
    add("hammer holder", &|t| {
        let step = include_str!("../../../../io/tests/data/shapr3d_hammer_holder.step");
        remus_io::step::reader::read_step(step, t).unwrap()[0]
    });
    // `Shell::new` accepts a face listed twice: the map-based copy restored
    // both visits' authority onto the last copy and left the first bare.
    add("face listed twice", &|t| {
        let source = crate::primitives::make_box(t, 1.0, 2.0, 3.0).unwrap();
        let mut faces = solid_faces(t, source).unwrap();
        add_authority(t, faces[0]);
        faces.push(faces[0]);
        let shell = t.add_shell(Shell::new(faces).unwrap());
        t.add_solid(Solid::new(shell, vec![]))
    });
    add("inner shell repeats the outer", &|t| {
        let source = crate::primitives::make_box(t, 1.0, 2.0, 3.0).unwrap();
        add_authority(t, solid_faces(t, source).unwrap()[2]);
        let shell = t.solid(source).unwrap().outer_shell();
        t.add_solid(Solid::new(shell, vec![shell]))
    });
    // One face uses an edge twice in the same direction: the per-snapshot
    // search refuses it as ambiguous after the copy is allocated.
    add("ambiguous use", &|t| {
        let a = t.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let b = t.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let there = t.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let back = t.add_edge(Edge::new(b, a, EdgeCurve::Line));
        let uses = vec![
            OrientedEdge::new(there, true),
            OrientedEdge::new(back, true),
            OrientedEdge::new(there, true),
        ];
        let wire = t.add_wire(Wire::new(uses, true).unwrap());
        let face = t.add_face(Face::new(wire, vec![], plane()));
        let source = crate::primitives::make_box(t, 1.0, 1.0, 1.0).unwrap();
        let mut faces = solid_faces(t, source).unwrap();
        add_authority(t, faces[1]);
        faces.insert(1, face);
        let shell = t.add_shell(Shell::new(faces).unwrap());
        t.add_solid(Solid::new(shell, vec![]))
    });
    fixtures
}

fn copy_matrices() -> [(&'static str, Mat4); 4] {
    [
        ("translate", Mat4::translation(10.0, -3.0, 7.5)),
        (
            "rotate",
            Mat4::translation(1.0, 2.0, 3.0) * Mat4::rotation_z(0.7) * Mat4::rotation_x(0.3),
        ),
        ("mirror", Mat4::scale(-1.0, 1.0, 1.0)),
        ("anisotropic scale", Mat4::scale(1.0, 2.0, 3.0)),
    ]
}

#[test]
fn copy_plan_matches_map_based_copy() {
    let mut destination = Topology::new();
    crate::primitives::make_box(&mut destination, 1.0, 1.0, 1.0).unwrap();
    for (name, fixture, solid) in copy_fixtures() {
        assert_same(
            &format!("{name}: copy_solid_with_entity_map"),
            &fixture,
            |t| entities(crate::copy::copy_solid_with_entity_map(t, solid)),
            |t| entities(legacy::copy_solid_with_entity_map(t, solid)),
        );
        assert_same(
            &format!("{name}: copy_solid"),
            &fixture,
            |t| format!("{:?}", crate::copy::copy_solid(t, solid)),
            |t| {
                format!(
                    "{:?}",
                    legacy::copy_solid_with_entity_map(t, solid).map(|c| c.solid)
                )
            },
        );
        assert_same(
            &format!("{name}: copy_solid_with_face_map"),
            &fixture,
            |t| face_map(crate::copy::copy_solid_with_face_map(t, solid)),
            |t| {
                face_map(legacy::copy_solid_with_entity_map(t, solid).map(|c| {
                    let map = c.face_map.into_iter().map(|(k, v)| (k, v.index()));
                    (c.solid, map.collect())
                }))
            },
        );
        // Between arenas: the source is shared, the destination cloned.
        assert_same(
            &format!("{name}: copy_solid_between_with_entity_map"),
            &destination,
            |t| {
                entities(crate::copy::copy_solid_between_with_entity_map(
                    &fixture, t, solid,
                ))
            },
            |t| {
                entities(legacy::copy_solid_between_with_entity_map(
                    &fixture, t, solid,
                ))
            },
        );
        assert_same(
            &format!("{name}: copy_solid_between"),
            &destination,
            |t| format!("{:?}", crate::copy::copy_solid_between(&fixture, t, solid)),
            |t| {
                let copied = legacy::copy_solid_between_with_entity_map(&fixture, t, solid);
                format!("{:?}", copied.map(|c| c.solid))
            },
        );
        for (what, matrix) in copy_matrices() {
            let policy = TransformPolicy::AllowApproximate;
            assert_same(
                &format!("{name}: copy_and_transform_solid / {what}"),
                &fixture,
                |t| {
                    format!(
                        "{:?}",
                        crate::copy::copy_and_transform_solid(t, solid, &matrix)
                    )
                },
                |t| {
                    let copied = legacy_copy_and_transform(t, solid, &matrix, policy);
                    format!("{:?}", copied.map(|(solid, _)| solid))
                },
            );
            assert_same(
                &format!("{name}: copy_and_transform_solid_detailed / {what}"),
                &fixture,
                |t| {
                    let copied =
                        crate::copy::copy_and_transform_solid_detailed(t, solid, &matrix, policy);
                    format!(
                        "{:?}",
                        copied.map(|(solid, report)| (solid, format!("{report:?}")))
                    )
                },
                |t| format!("{:?}", legacy_copy_and_transform(t, solid, &matrix, policy)),
            );
        }
    }
}

/// The repeated-face and ambiguous fixtures exercise the paths they claim.
#[test]
fn copy_oracle_fixtures_reach_their_edge_cases() {
    let fixtures = copy_fixtures();
    let find = |wanted: &str| {
        let (_, topo, solid) = fixtures.iter().find(|(name, ..)| *name == wanted).unwrap();
        (topo.clone(), *solid)
    };

    let (mut topo, solid) = find("face listed twice");
    let copied = crate::copy::copy_solid(&mut topo, solid).unwrap();
    let shell = topo.solid(copied).unwrap().outer_shell();
    let faces = topo.shell(shell).unwrap().faces().to_vec();
    assert_eq!(faces.len(), 7);
    assert_ne!(faces[0], faces[6]);
    assert!(
        topo.pcurves_for_face(faces[0]).is_empty(),
        "the first visit stays bare"
    );
    assert_eq!(
        topo.pcurves_for_face(faces[6]).len(),
        1,
        "the last visit carries the authority"
    );

    let (mut topo, solid) = find("ambiguous use");
    let error = crate::copy::copy_solid(&mut topo, solid).unwrap_err();
    assert!(
        format!("{error:?}").contains("does not contain exactly one forward use"),
        "{error:?}"
    );
}
