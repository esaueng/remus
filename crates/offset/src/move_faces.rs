//! Topology-preserving planar face moves.

use std::collections::{HashMap, HashSet};

use remus_math::analytic_intersection::{
    AnalyticSurface, ExactIntersectionCurve, exact_plane_analytic,
};
use remus_math::curves::{Circle3D, Ellipse3D};
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{
    edge_to_face_map, face_wires, solid_edges, solid_entity_counts, solid_faces, solid_vertices,
};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::data::{OffsetData, OffsetFace, OffsetOptions, OffsetStatus};
use crate::error::OffsetError;

/// A topology-preserving face move and its construction-derived face map.
#[derive(Debug)]
pub struct MoveFacesResult {
    /// Edited solid.
    pub solid: SolidId,
    /// Source face index to the one result face that carries it.
    pub face_map: HashMap<usize, FaceId>,
}

/// Move a coplanar group of planar faces by a signed distance along their
/// common outward normal.
///
/// The selected surfaces move while every other support surface stays fixed.
/// Each source edge is then rebuilt from the intersection of its two support
/// surfaces, so adjacent faces extend or shrink without changing the source
/// adjacency graph.
///
/// This first-phase implementation accepts planar selected faces and planar
/// or cylindrical adjacent faces. Other selected/support surface
/// configurations are refused before a result is returned.
///
/// # Errors
///
/// Returns [`OffsetError::UnsupportedMoveFace`] for unsupported surfaces and
/// [`OffsetError::TopologyChange`] when the requested distance collapses,
/// splits, or otherwise changes a source face, wire, or edge.
pub fn move_faces(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<SolidId, OffsetError> {
    Ok(move_faces_with_face_map(topo, solid, faces, distance)?.solid)
}

/// [`move_faces`] with the exact source-to-result face correspondence.
///
/// # Errors
///
/// Returns the same typed refusals as [`move_faces`].
pub fn move_faces_with_face_map(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<MoveFacesResult, OffsetError> {
    let snapshot = topo.clone();
    let result = move_faces_impl(topo, solid, faces, distance);
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

/// Replace one face's support surface and re-limit its existing adjacency.
///
/// The qualified cell accepts plane-to-plane and coaxial
/// cylinder-to-cylinder replacements whose neighboring supports are planar
/// or cylindrical. Every source edge is rebuilt from the new pair of support
/// surfaces, while the face, wire, edge, vertex, and shell counts remain
/// unchanged.
///
/// # Errors
///
/// Returns a typed refusal naming the source face or adjacency when the
/// replacement is unsupported, loses an edge, changes a wire, or opens the
/// shell. Failure restores the exact pre-call topology.
pub fn replace_surface_with_face_map(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    replacement: FaceSurface,
) -> Result<MoveFacesResult, OffsetError> {
    let snapshot = topo.clone();
    let result = replace_surface_impl(topo, solid, face, replacement);
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

#[allow(clippy::too_many_lines)]
fn replace_surface_impl(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    replacement: FaceSurface,
) -> Result<MoveFacesResult, OffsetError> {
    let options = OffsetOptions::default();
    let source_faces = solid_faces(topo, solid)?;
    if !source_faces.contains(&face) {
        return Err(OffsetError::FaceNotInSolid { face, solid });
    }
    let replacement = validate_replacement(topo, face, replacement, options.tolerance)?;
    let displacement = replacement_displacement(topo, face, &replacement)?;
    if displacement <= options.tolerance.linear {
        return Err(OffsetError::InvalidInput {
            reason: format!("replacement surface for face {} is unchanged", face.index()),
        });
    }

    let selected = HashSet::from([face.index()]);
    let source_counts = solid_entity_counts(topo, solid)?;
    let source_shell_sizes = shell_face_counts(topo, solid)?;
    let source_wire_shapes = source_faces
        .iter()
        .map(|&source_face| Ok((source_face, wire_shape(topo, source_face)?)))
        .collect::<Result<HashMap<_, _>, OffsetError>>()?;
    let source_edge_faces = edge_to_face_map(topo, solid)?;
    validate_source_edges(topo, &source_edge_faces, &selected)?;
    validate_replacement_clearance(
        topo,
        face,
        &replacement,
        &source_edge_faces,
        options.tolerance,
    )?;

    let marker = displacement.max(2.0 * options.tolerance.linear);
    let mut data = OffsetData::new(marker, options, Vec::new());
    populate_replacement_surfaces(topo, solid, face, replacement, marker, &mut data)?;
    let all_surfaces = data.offset_faces.clone();
    let relevant_faces = move_neighborhood(&source_edge_faces, &selected);
    data.offset_faces
        .retain(|candidate, _| relevant_faces.contains(&candidate.index()));
    crate::inter3d::intersect_faces_3d(topo, solid, &mut data)?;
    data.offset_faces = all_surfaces;
    crate::inter2d::intersect_pcurves_2d(topo, solid, &mut data)?;
    restore_exact_plane_cylinder_edges::<true>(topo, Vec3::new(0.0, 0.0, 0.0), 0.0, &mut data)?;
    validate_rebuilt_edges::<true, _>(topo, &source_edge_faces, &selected, &data)?;
    build_topology_preserving_wires::<true, _>(
        topo,
        solid,
        Vec3::new(0.0, 0.0, 0.0),
        0.0,
        &source_edge_faces,
        &mut data,
    )?;
    validate_rebuilt_wires(topo, &source_wire_shapes, &data)?;

    let result = crate::assemble::assemble_solid_with_face_map(topo, &data)?;
    super::validate_offset_result(topo, result.solid)?;
    validate_result_topology(
        topo,
        result.solid,
        source_counts,
        source_shell_sizes.as_slice(),
    )?;
    validate_replacement_face_map(topo, &source_faces, result.solid, &result.face_map)?;
    Ok(MoveFacesResult {
        solid: result.solid,
        face_map: result.face_map,
    })
}

fn validate_replacement_face_map(
    topo: &Topology,
    source_faces: &[FaceId],
    result: SolidId,
    face_map: &HashMap<usize, FaceId>,
) -> Result<(), OffsetError> {
    let source_indices: HashSet<_> = source_faces.iter().copied().map(FaceId::index).collect();
    let result_indices: HashSet<_> = solid_faces(topo, result)?
        .into_iter()
        .map(FaceId::index)
        .collect();
    let mapped_indices: HashSet<_> = face_map.values().copied().map(FaceId::index).collect();
    if face_map.len() != source_indices.len()
        || face_map.keys().copied().collect::<HashSet<_>>() != source_indices
        || mapped_indices.len() != face_map.len()
        || mapped_indices != result_indices
    {
        return Err(OffsetError::AssemblyFailed {
            reason: format!(
                "replace-surface face map is not one-to-one ({} source faces, {} map entries, {} distinct mapped faces, {} result faces)",
                source_indices.len(),
                face_map.len(),
                mapped_indices.len(),
                result_indices.len()
            ),
        });
    }
    Ok(())
}

fn move_faces_impl(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<MoveFacesResult, OffsetError> {
    let options = OffsetOptions::default();
    if !distance.is_finite() || distance.abs() <= options.tolerance.linear {
        return Err(OffsetError::InvalidInput {
            reason: "move-face distance must be non-zero and finite".into(),
        });
    }
    if faces.is_empty() {
        return Err(OffsetError::InvalidInput {
            reason: "move-face requires at least one selected face".into(),
        });
    }

    let source_faces = solid_faces(topo, solid)?;
    let source_face_set: HashSet<_> = source_faces.iter().map(|face| face.index()).collect();
    let mut selected = HashSet::with_capacity(faces.len());
    for &face in faces {
        if !source_face_set.contains(&face.index()) {
            return Err(OffsetError::FaceNotInSolid { face, solid });
        }
        if !selected.insert(face.index()) {
            return Err(OffsetError::InvalidInput {
                reason: format!(
                    "move-face selection contains face {} more than once",
                    face.index()
                ),
            });
        }
    }

    let reference = faces[0];
    let (reference_normal, reference_d) = effective_plane(topo, reference)?;
    for &face in faces.iter().skip(1) {
        let (normal, d) = effective_plane(topo, face)?;
        if normal.dot(reference_normal) < 1.0 - options.tolerance.angular {
            return Err(OffsetError::MoveGroupMismatch {
                reference,
                face,
                reason: "selected faces do not have the same outward normal".into(),
            });
        }
        if !options.tolerance.approx_eq(d, reference_d) {
            return Err(OffsetError::MoveGroupMismatch {
                reference,
                face,
                reason: "selected faces are parallel but not coplanar".into(),
            });
        }
    }

    let source_counts = solid_entity_counts(topo, solid)?;
    let source_shell_sizes = shell_face_counts(topo, solid)?;
    let source_wire_shapes = source_faces
        .iter()
        .map(|&face| Ok((face, wire_shape(topo, face)?)))
        .collect::<Result<HashMap<_, _>, OffsetError>>()?;
    let source_edge_faces = edge_to_face_map(topo, solid)?;
    validate_source_edges(topo, &source_edge_faces, &selected)?;

    let mut data = OffsetData::new(distance, options, Vec::new());
    populate_surfaces(topo, solid, &selected, distance, &mut data)?;
    let all_surfaces = data.offset_faces.clone();
    let relevant_faces = move_neighborhood(&source_edge_faces, &selected);
    data.offset_faces
        .retain(|face, _| relevant_faces.contains(&face.index()));
    crate::inter3d::intersect_faces_3d(topo, solid, &mut data)?;
    data.offset_faces = all_surfaces;
    crate::inter2d::intersect_pcurves_2d(topo, solid, &mut data)?;
    restore_exact_plane_cylinder_edges::<false>(topo, reference_normal, distance, &mut data)?;
    validate_rebuilt_edges::<false, _>(topo, &source_edge_faces, &selected, &data)?;
    build_topology_preserving_wires::<false, _>(
        topo,
        solid,
        reference_normal,
        distance,
        &source_edge_faces,
        &mut data,
    )?;
    validate_rebuilt_wires(topo, &source_wire_shapes, &data)?;

    let result = crate::assemble::assemble_solid_with_face_map(topo, &data)?;
    super::validate_offset_result(topo, result.solid)?;
    validate_result_topology(
        topo,
        result.solid,
        source_counts,
        source_shell_sizes.as_slice(),
    )?;
    Ok(MoveFacesResult {
        solid: result.solid,
        face_map: result.face_map,
    })
}

fn move_neighborhood<V: std::ops::Deref<Target = [FaceId]>>(
    edge_faces: &std::collections::BTreeMap<usize, V>,
    selected: &HashSet<usize>,
) -> HashSet<usize> {
    let mut relevant = selected.clone();
    for faces in edge_faces.values() {
        if faces.iter().any(|face| selected.contains(&face.index())) {
            relevant.extend(faces.iter().map(|face| face.index()));
        }
    }
    relevant
}

fn restore_exact_plane_cylinder_edges<const REPLACEMENT: bool>(
    topo: &mut Topology,
    move_normal: Vec3,
    distance: f64,
    data: &mut OffsetData,
) -> Result<(), OffsetError> {
    for index in 0..data.intersections.len() {
        let intersection = &data.intersections[index];
        let face_a = data.offset_faces.get(&intersection.face_a).ok_or_else(|| {
            OffsetError::TopologyChange {
                face: Some(intersection.face_a),
                edge: Some(intersection.original_edge),
                reason: "first support surface is unavailable".into(),
            }
        })?;
        let face_b = data.offset_faces.get(&intersection.face_b).ok_or_else(|| {
            OffsetError::TopologyChange {
                face: Some(intersection.face_b),
                edge: Some(intersection.original_edge),
                reason: "second support surface is unavailable".into(),
            }
        })?;
        let (normal, plane_d, cylinder) = match (&face_a.surface, &face_b.surface) {
            (FaceSurface::Plane { normal, d }, FaceSurface::Cylinder(cylinder))
            | (FaceSurface::Cylinder(cylinder), FaceSurface::Plane { normal, d }) => {
                (*normal, *d, cylinder.clone())
            }
            _ => continue,
        };
        let shift = if REPLACEMENT {
            Vec3::new(0.0, 0.0, 0.0)
        } else if face_a.distance != 0.0 || face_b.distance != 0.0 {
            move_normal * distance
        } else {
            Vec3::new(0.0, 0.0, 0.0)
        };
        if let Some(edge) = exact_plane_cylinder_edge::<REPLACEMENT>(
            topo,
            intersection.original_edge,
            normal,
            plane_d,
            &cylinder,
            shift,
            data.options.tolerance.linear,
        )? {
            data.intersections[index].new_edges = vec![edge];
        }
    }
    Ok(())
}

fn exact_plane_cylinder_edge<const REPLACEMENT: bool>(
    topo: &mut Topology,
    source_edge: EdgeId,
    normal: Vec3,
    plane_d: f64,
    cylinder: &CylindricalSurface,
    shift: Vec3,
    tolerance: f64,
) -> Result<Option<EdgeId>, OffsetError> {
    let source = topo.edge(source_edge)?;
    let source_curve = source.curve().clone();
    let source_start = source.start();
    let source_end = source.end();
    let source_tolerance = source.tolerance();
    let source_range = source
        .strict_domain()
        .map_err(|error| OffsetError::TopologyChange {
            face: None,
            edge: Some(source_edge),
            reason: format!("source edge has no valid parameter authority: {error}"),
        })?;

    if matches!(source_curve, EdgeCurve::Line) {
        return parallel_plane_cylinder_edge(
            topo,
            source_start,
            source_end,
            source_tolerance,
            normal,
            plane_d,
            cylinder,
            shift,
            tolerance,
        );
    }

    let exact = exact_plane_analytic(AnalyticSurface::Cylinder(cylinder), normal, plane_d)?;
    let curve = match (&source_curve, exact.as_slice()) {
        (EdgeCurve::Circle(source_circle), [ExactIntersectionCurve::Circle(circle)]) => {
            EdgeCurve::Circle(Circle3D::new_with_ref(
                circle.center(),
                source_circle.normal(),
                circle.radius(),
                source_circle.u_axis(),
            )?)
        }
        (EdgeCurve::Ellipse(source_ellipse), [ExactIntersectionCurve::Ellipse(ellipse)]) => {
            EdgeCurve::Ellipse(Ellipse3D::with_axes(
                ellipse.center(),
                source_ellipse.normal(),
                ellipse.semi_major(),
                ellipse.semi_minor(),
                source_ellipse.u_axis(),
                source_ellipse.v_axis(),
            )?)
        }
        (EdgeCurve::Circle(_), [ExactIntersectionCurve::Ellipse(ellipse)]) if REPLACEMENT => {
            EdgeCurve::Ellipse(ellipse.clone())
        }
        _ => return Ok(None),
    };
    if REPLACEMENT && source_curve.type_tag() != curve.type_tag() {
        return Ok(Some(add_reparameterized_curve_edge(
            topo,
            source_edge,
            source_start,
            source_end,
            source_tolerance,
            source_range,
            curve,
            tolerance,
        )?));
    }
    Ok(Some(add_projected_curve_edge(
        topo,
        source_edge,
        source_start,
        source_end,
        source_tolerance,
        &source_curve,
        source_range,
        curve,
        shift,
        tolerance,
    )?))
}

#[allow(clippy::too_many_arguments)]
fn add_reparameterized_curve_edge(
    topo: &mut Topology,
    source_edge: EdgeId,
    source_start: VertexId,
    source_end: VertexId,
    edge_tolerance: Option<f64>,
    source_range: (f64, f64),
    curve: EdgeCurve,
    tolerance: f64,
) -> Result<EdgeId, OffsetError> {
    let start_vertex = topo.vertex(source_start)?;
    let start_point = start_vertex.point();
    let start_tolerance = start_vertex.tolerance();
    let end_vertex = topo.vertex(source_end)?;
    let end_point = end_vertex.point();
    let end_tolerance = end_vertex.tolerance();
    let vertex_tolerance =
        replacement_vertex_tolerance(start_tolerance, end_tolerance, edge_tolerance, tolerance)
            .map_err(|reason| OffsetError::TopologyChange {
                face: None,
                edge: Some(source_edge),
                reason,
            })?;

    let project = |point| match &curve {
        EdgeCurve::Circle(circle) => Some(circle.project(point)),
        EdgeCurve::Ellipse(ellipse) => Some(ellipse.project(point)),
        _ => None,
    };
    let start_parameter = project(start_point).ok_or_else(|| OffsetError::TopologyChange {
        face: None,
        edge: Some(source_edge),
        reason: "changed intersection curve has no exact parameter projector".into(),
    })?;
    let source_span = source_range.1 - source_range.0;
    let end_parameter = if source_start == source_end {
        start_parameter + source_span.signum() * std::f64::consts::TAU
    } else {
        let raw_end = project(end_point).ok_or_else(|| OffsetError::TopologyChange {
            face: None,
            edge: Some(source_edge),
            reason: "changed intersection curve end has no exact parameter projector".into(),
        })?;
        let delta = if source_span.is_sign_negative() {
            -((start_parameter - raw_end).rem_euclid(std::f64::consts::TAU))
        } else {
            (raw_end - start_parameter).rem_euclid(std::f64::consts::TAU)
        };
        start_parameter + delta
    };
    if !start_parameter.is_finite()
        || !end_parameter.is_finite()
        || (end_parameter - start_parameter).abs() <= f64::EPSILON
    {
        return Err(OffsetError::TopologyChange {
            face: None,
            edge: Some(source_edge),
            reason: "changed intersection curve has no finite non-zero trim".into(),
        });
    }

    let new_start = curve.evaluate_with_endpoints(start_parameter, start_point, end_point);
    let new_end = curve.evaluate_with_endpoints(end_parameter, start_point, end_point);
    let start = topo.add_vertex(Vertex::new(new_start, vertex_tolerance));
    let end = if source_start == source_end {
        start
    } else {
        topo.add_vertex(Vertex::new(new_end, vertex_tolerance))
    };
    let mut edge = Edge::with_tolerance(start, end, curve, edge_tolerance);
    edge.set_trim(Some((start_parameter, end_parameter)));
    edge.strict_domain()
        .map_err(|error| OffsetError::TopologyChange {
            face: None,
            edge: Some(source_edge),
            reason: format!("changed intersection curve has invalid trim authority: {error}"),
        })?;
    Ok(topo.add_edge(edge))
}

#[allow(clippy::too_many_arguments)]
fn parallel_plane_cylinder_edge(
    topo: &mut Topology,
    source_start: VertexId,
    source_end: VertexId,
    edge_tolerance: Option<f64>,
    normal: Vec3,
    plane_d: f64,
    cylinder: &CylindricalSurface,
    shift: Vec3,
    tolerance: f64,
) -> Result<Option<EdgeId>, OffsetError> {
    let axis = cylinder.axis();
    let perpendicular = normal - axis * normal.dot(axis);
    let perpendicular_length = perpendicular.length();
    if perpendicular_length <= tolerance {
        return Ok(None);
    }
    let radial_normal = perpendicular * (1.0 / perpendicular_length);
    let radial_distance = (plane_d - normal.dot(cylinder.origin() - Point3::new(0.0, 0.0, 0.0)))
        / perpendicular_length;
    if radial_distance.abs() > cylinder.radius() + tolerance {
        return Ok(None);
    }
    let foot = cylinder.origin() + radial_normal * radial_distance;
    let branch_direction = axis.cross(radial_normal).normalize()?;
    let half_span = cylinder
        .radius()
        .mul_add(cylinder.radius(), -(radial_distance * radial_distance))
        .max(0.0)
        .sqrt();
    let branches = [
        foot + branch_direction * half_span,
        foot - branch_direction * half_span,
    ];
    let source_start_vertex = topo.vertex(source_start)?;
    let source_start_point = source_start_vertex.point() + shift;
    let source_start_tolerance = source_start_vertex.tolerance();
    let source_end_vertex = topo.vertex(source_end)?;
    let source_end_point = source_end_vertex.point() + shift;
    let source_end_tolerance = source_end_vertex.tolerance();
    let vertex_tolerance = replacement_vertex_tolerance(
        source_start_tolerance,
        source_end_tolerance,
        edge_tolerance,
        tolerance,
    )
    .map_err(|reason| OffsetError::TopologyChange {
        face: None,
        edge: None,
        reason,
    })?;
    let midpoint = source_start_point + (source_end_point - source_start_point) * 0.5;
    let branch = branches
        .into_iter()
        .min_by(|first, second| {
            distance_to_line(midpoint, *first, axis)
                .total_cmp(&distance_to_line(midpoint, *second, axis))
        })
        .ok_or_else(|| OffsetError::TopologyChange {
            face: None,
            edge: None,
            reason: "plane-cylinder intersection has no line branch".into(),
        })?;
    let project = |point: Point3| branch + axis * (point - branch).dot(axis);
    let start = topo.add_vertex(Vertex::new(project(source_start_point), vertex_tolerance));
    let end = topo.add_vertex(Vertex::new(project(source_end_point), vertex_tolerance));
    Ok(Some(topo.add_edge(Edge::with_tolerance(
        start,
        end,
        EdgeCurve::Line,
        edge_tolerance,
    ))))
}

#[allow(clippy::too_many_arguments)]
fn add_projected_curve_edge(
    topo: &mut Topology,
    source_edge: EdgeId,
    source_start: VertexId,
    source_end: VertexId,
    edge_tolerance: Option<f64>,
    source_curve: &EdgeCurve,
    source_range: (f64, f64),
    curve: EdgeCurve,
    shift: Vec3,
    tolerance: f64,
) -> Result<EdgeId, OffsetError> {
    let source_start_vertex = topo.vertex(source_start)?;
    let source_end_vertex = topo.vertex(source_end)?;
    let source_start_point = source_start_vertex.point();
    let source_end_point = source_end_vertex.point();
    let vertex_tolerance = replacement_vertex_tolerance(
        source_start_vertex.tolerance(),
        source_end_vertex.tolerance(),
        edge_tolerance,
        tolerance,
    )
    .map_err(|reason| OffsetError::TopologyChange {
        face: None,
        edge: Some(source_edge),
        reason,
    })?;

    let midpoint_parameter = (source_range.1 - source_range.0).mul_add(0.5, source_range.0);
    if !midpoint_parameter.is_finite() {
        return Err(OffsetError::TopologyChange {
            face: None,
            edge: Some(source_edge),
            reason: "source analytic edge midpoint parameter is not finite".into(),
        });
    }
    for (label, parameter, expected) in [
        ("start", source_range.0, source_start_point),
        ("end", source_range.1, source_end_point),
    ] {
        let actual =
            source_curve.evaluate_with_endpoints(parameter, source_start_point, source_end_point);
        let residual = (actual - expected).length();
        if !residual.is_finite() || residual > vertex_tolerance {
            return Err(OffsetError::TopologyChange {
                face: None,
                edge: Some(source_edge),
                reason: format!(
                    "source analytic edge {label} misses its parameter authority by {residual} \
                     (tolerance {vertex_tolerance})"
                ),
            });
        }
    }
    let start_point = curve.evaluate_with_endpoints(
        source_range.0,
        source_start_point + shift,
        source_end_point + shift,
    );
    let midpoint = curve.evaluate_with_endpoints(
        midpoint_parameter,
        source_start_point + shift,
        source_end_point + shift,
    );
    let end_point = curve.evaluate_with_endpoints(
        source_range.1,
        source_start_point + shift,
        source_end_point + shift,
    );
    for (label, point) in [
        ("start", start_point),
        ("midpoint", midpoint),
        ("end", end_point),
    ] {
        if point.0.iter().any(|coordinate| !coordinate.is_finite()) {
            return Err(OffsetError::TopologyChange {
                face: None,
                edge: Some(source_edge),
                reason: format!("projected analytic edge {label} is not finite"),
            });
        }
    }

    let mut authority_probe =
        Edge::with_tolerance(source_start, source_end, curve.clone(), edge_tolerance);
    authority_probe.set_trim(Some(source_range));
    authority_probe
        .strict_domain()
        .map_err(|error| OffsetError::TopologyChange {
            face: None,
            edge: Some(source_edge),
            reason: format!("projected analytic edge has invalid parameter authority: {error}"),
        })?;

    let start = topo.add_vertex(Vertex::new(start_point, vertex_tolerance));
    let end = if source_start == source_end {
        start
    } else {
        topo.add_vertex(Vertex::new(end_point, vertex_tolerance))
    };
    let mut edge = Edge::with_tolerance(start, end, curve, edge_tolerance);
    edge.set_trim(Some(source_range));
    Ok(topo.add_edge(edge))
}

fn replacement_vertex_tolerance(
    start_tolerance: f64,
    end_tolerance: f64,
    edge_tolerance: Option<f64>,
    operation_floor: f64,
) -> Result<f64, String> {
    if !start_tolerance.is_finite()
        || start_tolerance < 0.0
        || !end_tolerance.is_finite()
        || end_tolerance < 0.0
        || edge_tolerance.is_some_and(|value| !value.is_finite() || value < 0.0)
        || !operation_floor.is_finite()
        || operation_floor < 0.0
    {
        return Err(format!(
            "source edge has invalid tolerance authority (start {start_tolerance}, end \
             {end_tolerance}, edge {edge_tolerance:?}, operation floor {operation_floor})"
        ));
    }
    Ok(edge_tolerance.unwrap_or_else(|| start_tolerance.max(end_tolerance).max(operation_floor)))
}

fn distance_to_line(point: Point3, origin: Point3, direction: Vec3) -> f64 {
    let delta = point - origin;
    (delta - direction * delta.dot(direction)).length()
}

#[allow(clippy::too_many_lines)]
fn build_topology_preserving_wires<
    const REPLACEMENT: bool,
    V: std::ops::Deref<Target = [FaceId]>,
>(
    topo: &mut Topology,
    solid: SolidId,
    move_normal: Vec3,
    distance: f64,
    source_edge_faces: &std::collections::BTreeMap<usize, V>,
    data: &mut OffsetData,
) -> Result<(), OffsetError> {
    let source_edges = solid_edges(topo, solid)?;
    let source_vertices = solid_vertices(topo, solid)?;
    let mut preliminary = HashMap::new();
    for intersection in &data.intersections {
        let [replacement] = intersection.new_edges.as_slice() else {
            return Err(OffsetError::TopologyChange {
                face: Some(intersection.face_a),
                edge: Some(intersection.original_edge),
                reason: format!(
                    "support-surface intersection produced {} replacement edges",
                    intersection.new_edges.len()
                ),
            });
        };
        preliminary.insert(intersection.original_edge.index(), *replacement);
    }

    let selected_vertices: HashSet<_> = data
        .offset_faces
        .iter()
        .filter(|(_, face)| face.distance != 0.0)
        .map(|(&face, _)| remus_topology::explorer::face_vertices(topo, face))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .map(remus_topology::arena::Id::index)
        .collect();

    let mut incident_edges: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &edge_id in &source_edges {
        let edge = topo.edge(edge_id)?;
        incident_edges
            .entry(edge.start().index())
            .or_default()
            .push(edge_id);
        if edge.end() != edge.start() {
            incident_edges
                .entry(edge.end().index())
                .or_default()
                .push(edge_id);
        }
    }

    let mut vertex_map = HashMap::with_capacity(source_vertices.len());
    for source_vertex in source_vertices {
        let source = topo.vertex(source_vertex)?;
        let point = if selected_vertices.contains(&source_vertex.index()) {
            let predicted = if REPLACEMENT {
                project_to_changed_surface(
                    source.point(),
                    incident_edges
                        .get(&source_vertex.index())
                        .map_or(&[][..], Vec::as_slice),
                    source_edge_faces,
                    data,
                )?
            } else {
                source.point() + move_normal * distance
            };
            rebuild_vertex_point::<REPLACEMENT, _>(
                topo,
                source_vertex,
                predicted,
                incident_edges
                    .get(&source_vertex.index())
                    .map_or(&[][..], Vec::as_slice),
                &preliminary,
                source_edge_faces,
                data,
            )?
        } else {
            source.point()
        };
        vertex_map.insert(
            source_vertex.index(),
            topo.add_vertex(Vertex::new(point, source.tolerance())),
        );
    }

    let mut edge_map = HashMap::with_capacity(source_edges.len());
    for source_edge in source_edges {
        let source = topo.edge(source_edge)?;
        let start = vertex_map[&source.start().index()];
        let end = vertex_map[&source.end().index()];
        if source.start() != source.end()
            && (topo.vertex(start)?.point() - topo.vertex(end)?.point()).length()
                <= data.options.tolerance.linear
        {
            return Err(OffsetError::TopologyChange {
                face: source_edge_faces
                    .get(&source_edge.index())
                    .and_then(|faces| faces.first().copied()),
                edge: Some(source_edge),
                reason: "move collapsed a non-degenerate source edge".into(),
            });
        }
        let replacement = preliminary.get(&source_edge.index()).copied();
        let curve = replacement.map_or_else(
            || Ok::<_, OffsetError>(source.curve().clone()),
            |replacement| Ok(topo.edge(replacement)?.curve().clone()),
        )?;
        let mut rebuilt = Edge::with_tolerance(start, end, curve, source.tolerance());
        rebuilt.set_trim(if REPLACEMENT && let Some(replacement) = replacement {
            topo.edge(replacement)?.trim()
        } else {
            source.trim()
        });
        edge_map.insert(source_edge.index(), topo.add_edge(rebuilt));
    }

    let mut wire_map: HashMap<usize, WireId> = HashMap::new();
    let mut faces: Vec<_> = data.offset_faces.keys().copied().collect();
    faces.sort_by_key(|face| face.index());
    for face in faces {
        let source_wires = face_wires(topo, face)?;
        let mut rebuilt_wires = Vec::with_capacity(source_wires.len());
        for source_wire in source_wires {
            let rebuilt = if let Some(&wire) = wire_map.get(&source_wire.index()) {
                wire
            } else {
                let source = topo.wire(source_wire)?;
                let edges = source
                    .edges()
                    .iter()
                    .map(|oriented| {
                        OrientedEdge::new(edge_map[&oriented.edge().index()], oriented.is_forward())
                    })
                    .collect();
                let wire = topo.add_wire(Wire::new(edges, source.is_closed())?);
                wire_map.insert(source_wire.index(), wire);
                wire
            };
            rebuilt_wires.push(rebuilt);
        }
        data.face_wires.insert(face, rebuilt_wires);
    }
    Ok(())
}

fn project_to_changed_surface<V: std::ops::Deref<Target = [FaceId]>>(
    point: Point3,
    incident_edges: &[EdgeId],
    source_edge_faces: &std::collections::BTreeMap<usize, V>,
    data: &OffsetData,
) -> Result<Point3, OffsetError> {
    let changed = incident_edges.iter().find_map(|edge| {
        source_edge_faces.get(&edge.index()).and_then(|faces| {
            faces.iter().find_map(|face| {
                data.offset_faces
                    .get(face)
                    .filter(|candidate| candidate.distance != 0.0)
            })
        })
    });
    let Some(changed) = changed else {
        return Ok(point);
    };
    project_point_to_surface(point, &changed.surface, changed.original)
}

fn project_point_to_surface(
    point: Point3,
    surface: &FaceSurface,
    face: FaceId,
) -> Result<Point3, OffsetError> {
    match surface {
        FaceSurface::Plane { normal, d } => {
            let length_sq = normal.dot(*normal);
            if !length_sq.is_finite() || length_sq <= f64::EPSILON {
                return Err(OffsetError::InvalidInput {
                    reason: "replacement plane has a zero or non-finite normal".into(),
                });
            }
            let origin = Point3::new(0.0, 0.0, 0.0);
            Ok(point + *normal * ((*d - normal.dot(point - origin)) / length_sq))
        }
        FaceSurface::Cylinder(cylinder) => {
            let (u, v) = cylinder.project_point(point);
            Ok(cylinder.evaluate(u, v))
        }
        other => Err(OffsetError::UnsupportedMoveFace {
            face,
            surface_type: other.type_tag(),
            reason: "replacement vertex projection supports planes and cylinders only".into(),
        }),
    }
}

#[allow(clippy::too_many_arguments)]
fn rebuild_vertex_point<const REPLACEMENT: bool, V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    source_vertex: VertexId,
    predicted: Point3,
    incident_edges: &[EdgeId],
    preliminary: &HashMap<usize, EdgeId>,
    source_edge_faces: &std::collections::BTreeMap<usize, V>,
    data: &OffsetData,
) -> Result<Point3, OffsetError> {
    let curves: Vec<_> = incident_edges
        .iter()
        .filter_map(|edge| {
            preliminary.get(&edge.index()).copied().or_else(|| {
                if !REPLACEMENT {
                    return Some(*edge);
                }
                let faces = source_edge_faces.get(&edge.index())?;
                let selected_self_seam = faces.iter().all(|face| {
                    data.offset_faces
                        .get(face)
                        .is_some_and(|surface| surface.distance != 0.0)
                });
                (!selected_self_seam).then_some(*edge)
            })
        })
        .collect();

    let mut candidates = Vec::new();
    candidates.extend(intersect_incident_lines_with_moved_planes(
        topo,
        incident_edges,
        source_edge_faces,
        data,
    )?);
    for (index, &first) in curves.iter().enumerate() {
        for &second in curves.iter().skip(index + 1) {
            if let Some(point) = intersect_replacement_lines(topo, first, second, data)? {
                candidates.push(point);
            }
        }
    }
    candidates.sort_by(|a, b| distance_sq(*a, predicted).total_cmp(&distance_sq(*b, predicted)));

    let point = if let Some(point) = candidates
        .into_iter()
        .find(|&point| point_on_curves(topo, point, &curves, data.options.tolerance.linear))
    {
        point
    } else if curves.is_empty() {
        predicted
    } else {
        let mut point = predicted;
        for _ in 0..24 {
            let before = point;
            for &edge in &curves {
                point = project_to_edge(topo, edge, point)?;
            }
            if distance_sq(point, before) <= data.options.tolerance.linear_sq() {
                break;
            }
        }
        point
    };

    if !point_on_curves(topo, point, &curves, data.options.tolerance.linear) {
        return Err(OffsetError::TopologyChange {
            face: None,
            edge: incident_edges.first().copied(),
            reason: format!(
                "moved source vertex {} does not lie on every rebuilt incident edge",
                source_vertex.index()
            ),
        });
    }
    validate_vertex_supports(
        source_vertex,
        point,
        incident_edges,
        source_edge_faces,
        data,
    )?;
    Ok(point)
}

fn intersect_incident_lines_with_moved_planes<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    incident_edges: &[EdgeId],
    source_edge_faces: &std::collections::BTreeMap<usize, V>,
    data: &OffsetData,
) -> Result<Vec<Point3>, OffsetError> {
    let mut moved_planes = Vec::new();
    for edge in incident_edges {
        let Some(faces) = source_edge_faces.get(&edge.index()) else {
            continue;
        };
        for face in faces.iter() {
            let Some(offset_face) = data.offset_faces.get(face) else {
                continue;
            };
            if offset_face.distance == 0.0 {
                continue;
            }
            if let FaceSurface::Plane { normal, d } = offset_face.surface {
                moved_planes.push((normal, d));
            }
        }
    }

    let origin = Point3::new(0.0, 0.0, 0.0);
    let mut candidates = Vec::new();
    for edge in incident_edges {
        let edge = topo.edge(*edge)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            continue;
        }
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let direction = end - start;
        for &(normal, d) in &moved_planes {
            let denominator = normal.dot(direction);
            if denominator.abs() <= f64::EPSILON * direction.length().max(1.0) {
                continue;
            }
            let parameter = (d - normal.dot(start - origin)) / denominator;
            candidates.push(start + direction * parameter);
        }
    }
    Ok(candidates)
}

fn intersect_replacement_lines(
    topo: &Topology,
    first: EdgeId,
    second: EdgeId,
    data: &OffsetData,
) -> Result<Option<Point3>, OffsetError> {
    let first_edge = topo.edge(first)?;
    let second_edge = topo.edge(second)?;
    if !matches!(first_edge.curve(), EdgeCurve::Line)
        || !matches!(second_edge.curve(), EdgeCurve::Line)
    {
        return Ok(None);
    }
    let p1 = topo.vertex(first_edge.start())?.point();
    let p2 = topo.vertex(first_edge.end())?.point();
    let q1 = topo.vertex(second_edge.start())?.point();
    let q2 = topo.vertex(second_edge.end())?.point();
    let u = p2 - p1;
    let v = q2 - q1;
    let w = p1 - q1;
    let a = u.dot(u);
    let b = u.dot(v);
    let c = v.dot(v);
    let d = u.dot(w);
    let e = v.dot(w);
    let denominator = a.mul_add(c, -(b * b));
    if denominator.abs() <= f64::EPSILON * a.max(c).max(1.0) {
        return Ok(None);
    }
    let s = b.mul_add(e, -(c * d)) / denominator;
    let t = a.mul_add(e, -(b * d)) / denominator;
    let on_first = p1 + u * s;
    let on_second = q1 + v * t;
    if (on_first - on_second).length() > data.options.tolerance.linear {
        return Ok(None);
    }
    Ok(Some(Point3::new(
        0.5 * (on_first.x() + on_second.x()),
        0.5 * (on_first.y() + on_second.y()),
        0.5 * (on_first.z() + on_second.z()),
    )))
}

fn point_on_curves(topo: &Topology, point: Point3, curves: &[EdgeId], tolerance: f64) -> bool {
    curves.iter().all(|&edge| {
        project_to_edge(topo, edge, point)
            .is_ok_and(|projected| (projected - point).length() <= tolerance)
    })
}

fn project_to_edge(topo: &Topology, edge_id: EdgeId, point: Point3) -> Result<Point3, OffsetError> {
    let edge = topo.edge(edge_id)?;
    match edge.curve() {
        EdgeCurve::Line => {
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let direction = end - start;
            let length_sq = direction.dot(direction);
            if length_sq <= f64::EPSILON {
                return Ok(start);
            }
            Ok(start + direction * ((point - start).dot(direction) / length_sq))
        }
        EdgeCurve::Circle(circle) => Ok(circle.evaluate(circle.project(point))),
        EdgeCurve::Ellipse(ellipse) => Ok(ellipse.evaluate(ellipse.project(point))),
        EdgeCurve::Hyperbola(hyperbola) => Ok(hyperbola.evaluate(hyperbola.project(point))),
        EdgeCurve::Parabola(parabola) => Ok(parabola.evaluate(parabola.project(point))),
        EdgeCurve::NurbsCurve(_) => Err(OffsetError::TopologyChange {
            face: None,
            edge: Some(edge_id),
            reason: "move-face vertex reconstruction does not approximate NURBS edges".into(),
        }),
    }
}

fn validate_vertex_supports<V: std::ops::Deref<Target = [FaceId]>>(
    source_vertex: VertexId,
    point: Point3,
    incident_edges: &[EdgeId],
    source_edge_faces: &std::collections::BTreeMap<usize, V>,
    data: &OffsetData,
) -> Result<(), OffsetError> {
    let mut faces = HashSet::new();
    for edge in incident_edges {
        if let Some(adjacent) = source_edge_faces.get(&edge.index()) {
            faces.extend(adjacent.iter().copied());
        }
    }
    let tolerance = data.options.tolerance;
    for face in faces {
        let Some(offset_face) = data.offset_faces.get(&face) else {
            continue;
        };
        let residual = match &offset_face.surface {
            FaceSurface::Plane { normal, d } => {
                let vector = point - Point3::new(0.0, 0.0, 0.0);
                (normal.dot(vector) - d).abs()
            }
            FaceSurface::Cylinder(cylinder) => {
                let delta = point - cylinder.origin();
                let radial = delta - cylinder.axis() * delta.dot(cylinder.axis());
                (radial.length() - cylinder.radius()).abs()
            }
            other => {
                return Err(OffsetError::UnsupportedMoveFace {
                    face,
                    surface_type: other.type_tag(),
                    reason: format!(
                        "surface constrains moved source vertex {}",
                        source_vertex.index()
                    ),
                });
            }
        };
        let scale = [point.x().abs(), point.y().abs(), point.z().abs(), 1.0]
            .into_iter()
            .fold(1.0_f64, f64::max);
        if residual > tolerance.linear.max(tolerance.relative * scale) {
            return Err(OffsetError::TopologyChange {
                face: Some(face),
                edge: incident_edges.first().copied(),
                reason: format!(
                    "moved source vertex {} misses its support surface by {residual}",
                    source_vertex.index()
                ),
            });
        }
    }
    Ok(())
}

fn distance_sq(first: Point3, second: Point3) -> f64 {
    let delta = first - second;
    delta.dot(delta)
}

fn effective_plane(topo: &Topology, face: FaceId) -> Result<(Vec3, f64), OffsetError> {
    let face_data = topo.face(face)?;
    let FaceSurface::Plane { normal, d } = face_data.surface() else {
        return Err(OffsetError::UnsupportedMoveFace {
            face,
            surface_type: face_data.surface().type_tag(),
            reason: "phase 4.1 moves planar faces only".into(),
        });
    };
    if face_data.is_reversed() {
        Ok((-*normal, -*d))
    } else {
        Ok((*normal, *d))
    }
}

fn validate_replacement(
    topo: &Topology,
    face: FaceId,
    replacement: FaceSurface,
    tolerance: remus_math::tolerance::Tolerance,
) -> Result<FaceSurface, OffsetError> {
    let source = topo.face(face)?.surface();
    match (source, replacement) {
        (
            FaceSurface::Plane {
                normal: source_normal,
                ..
            },
            FaceSurface::Plane { normal, d },
        ) => {
            if normal.0.iter().any(|value| !value.is_finite()) || !d.is_finite() {
                return Err(OffsetError::InvalidInput {
                    reason: "replacement plane must be finite".into(),
                });
            }
            let length = normal.length();
            if length <= f64::EPSILON {
                return Err(OffsetError::InvalidInput {
                    reason: "replacement plane normal must be non-zero".into(),
                });
            }
            let normal = normal * (1.0 / length);
            let source_normal = source_normal.normalize()?;
            if normal.dot(source_normal) <= tolerance.angular {
                return Err(OffsetError::UnsupportedMoveFace {
                    face,
                    surface_type: "plane",
                    reason: "replacement plane reverses or turns through the source face".into(),
                });
            }
            Ok(FaceSurface::Plane {
                normal,
                d: d / length,
            })
        }
        (FaceSurface::Cylinder(source), FaceSurface::Cylinder(replacement)) => {
            if !replacement.radius().is_finite()
                || replacement.radius() <= tolerance.linear
                || replacement
                    .origin()
                    .0
                    .iter()
                    .any(|value| !value.is_finite())
                || replacement.axis().0.iter().any(|value| !value.is_finite())
            {
                return Err(OffsetError::InvalidInput {
                    reason: "replacement cylinder must be finite with a positive radius".into(),
                });
            }
            if source.axis().dot(replacement.axis()) < 1.0 - tolerance.angular {
                return Err(OffsetError::UnsupportedMoveFace {
                    face,
                    surface_type: "cylinder",
                    reason: "qualified replacement cylinders preserve the source axis direction"
                        .into(),
                });
            }
            let delta = replacement.origin() - source.origin();
            let radial = delta - source.axis() * delta.dot(source.axis());
            if radial.length() > tolerance.linear {
                return Err(OffsetError::UnsupportedMoveFace {
                    face,
                    surface_type: "cylinder",
                    reason: "qualified replacement cylinders are coaxial with the source".into(),
                });
            }
            Ok(FaceSurface::Cylinder(replacement))
        }
        (source, replacement) => Err(OffsetError::UnsupportedMoveFace {
            face,
            surface_type: replacement.type_tag(),
            reason: format!(
                "qualified replace-surface does not change {} to {}",
                source.type_tag(),
                replacement.type_tag()
            ),
        }),
    }
}

fn replacement_displacement(
    topo: &Topology,
    face: FaceId,
    replacement: &FaceSurface,
) -> Result<f64, OffsetError> {
    let vertices = remus_topology::explorer::face_vertices(topo, face)?;
    if vertices.is_empty() {
        return Err(OffsetError::TopologyChange {
            face: Some(face),
            edge: None,
            reason: "replacement face has no boundary vertices".into(),
        });
    }
    vertices.into_iter().try_fold(0.0_f64, |maximum, vertex| {
        let point = topo.vertex(vertex)?.point();
        let projected = project_point_to_surface(point, replacement, face)?;
        Ok(maximum.max((projected - point).length()))
    })
}

fn validate_replacement_clearance<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    selected: FaceId,
    replacement: &FaceSurface,
    edge_faces: &std::collections::BTreeMap<usize, V>,
    tolerance: remus_math::tolerance::Tolerance,
) -> Result<(), OffsetError> {
    let selected_face = topo.face(selected)?;
    let selected_vertices: HashSet<_> = remus_topology::explorer::face_vertices(topo, selected)?
        .into_iter()
        .map(VertexId::index)
        .collect();
    if matches!(replacement, FaceSurface::Cylinder(_)) && !selected_face.is_reversed() {
        return Err(OffsetError::UnsupportedMoveFace {
            face: selected,
            surface_type: "cylinder",
            reason: "qualified cylinder replacement is limited to inward-facing bore walls".into(),
        });
    }

    for (&edge_index, faces) in edge_faces {
        if faces.contains(&selected) {
            continue;
        }
        let edge_id =
            topo.edge_id_from_index(edge_index)
                .ok_or_else(|| OffsetError::TopologyChange {
                    face: Some(selected),
                    edge: None,
                    reason: format!("source edge {edge_index} is unavailable"),
                })?;
        let edge = topo.edge(edge_id)?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let scale = [
            start.x().abs(),
            start.y().abs(),
            start.z().abs(),
            end.x().abs(),
            end.y().abs(),
            end.z().abs(),
            1.0,
        ]
        .into_iter()
        .fold(1.0_f64, f64::max);
        let linear = tolerance.linear.max(tolerance.relative * scale);
        let clear = match replacement {
            FaceSurface::Plane { normal, d } => {
                let orientation = if selected_face.is_reversed() {
                    -1.0
                } else {
                    1.0
                };
                let normal = *normal * orientation;
                let d = *d * orientation;
                let origin = Point3::new(0.0, 0.0, 0.0);
                let center_value = |point: Point3| normal.dot(point - origin) - d;
                let maximum = match edge.curve() {
                    EdgeCurve::Line => [
                        (!selected_vertices.contains(&edge.start().index()))
                            .then(|| center_value(start)),
                        (!selected_vertices.contains(&edge.end().index()))
                            .then(|| center_value(end)),
                    ]
                    .into_iter()
                    .flatten()
                    .fold(f64::NEG_INFINITY, f64::max),
                    EdgeCurve::Circle(circle) => {
                        let radial = circle.radius()
                            * normal
                                .dot(circle.u_axis())
                                .hypot(normal.dot(circle.v_axis()));
                        center_value(circle.center()) + radial
                    }
                    EdgeCurve::Ellipse(ellipse) => {
                        let u = ellipse.semi_major() * normal.dot(ellipse.u_axis());
                        let v = ellipse.semi_minor() * normal.dot(ellipse.v_axis());
                        center_value(ellipse.center()) + u.hypot(v)
                    }
                    other => {
                        return Err(OffsetError::TopologyChange {
                            face: Some(selected),
                            edge: Some(edge_id),
                            reason: format!(
                                "cannot prove replacement-plane clearance against a nonadjacent {} edge",
                                other.type_tag()
                            ),
                        });
                    }
                };
                maximum <= linear
            }
            FaceSurface::Cylinder(cylinder) => match edge.curve() {
                EdgeCurve::Line => {
                    minimum_segment_axis_distance(start, end, cylinder)
                        >= cylinder.radius() - linear
                }
                other => {
                    return Err(OffsetError::TopologyChange {
                        face: Some(selected),
                        edge: Some(edge_id),
                        reason: format!(
                            "cannot prove replacement-bore clearance against a nonadjacent {} edge",
                            other.type_tag()
                        ),
                    });
                }
            },
            _ => unreachable!("replacement validation limits the surface variants"),
        };
        if !clear {
            return Err(OffsetError::TopologyChange {
                face: Some(selected),
                edge: Some(edge_id),
                reason: "replacement surface crosses a nonadjacent source boundary".into(),
            });
        }
    }
    Ok(())
}

fn minimum_segment_axis_distance(start: Point3, end: Point3, cylinder: &CylindricalSurface) -> f64 {
    let axis = cylinder.axis();
    let offset = start - cylinder.origin();
    let direction = end - start;
    let radial_offset = offset - axis * offset.dot(axis);
    let radial_direction = direction - axis * direction.dot(axis);
    let denominator = radial_direction.dot(radial_direction);
    let parameter = if denominator <= f64::EPSILON {
        0.0
    } else {
        (-radial_offset.dot(radial_direction) / denominator).clamp(0.0, 1.0)
    };
    (radial_offset + radial_direction * parameter).length()
}

fn populate_replacement_surfaces(
    topo: &Topology,
    solid: SolidId,
    selected: FaceId,
    replacement: FaceSurface,
    marker: f64,
    data: &mut OffsetData,
) -> Result<(), OffsetError> {
    let solid_data = topo.solid(solid)?;
    for shell_id in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        let faces = topo.shell(shell_id)?.faces().to_vec();
        data.shell_faces.push(faces.clone());
        for face_id in faces {
            let face = topo.face(face_id)?;
            data.offset_faces.insert(
                face_id,
                OffsetFace {
                    original: face_id,
                    surface: if face_id == selected {
                        replacement.clone()
                    } else {
                        face.surface().clone()
                    },
                    distance: if face_id == selected { marker } else { 0.0 },
                    status: OffsetStatus::Done,
                },
            );
        }
    }
    Ok(())
}

fn populate_surfaces(
    topo: &Topology,
    solid: SolidId,
    selected: &HashSet<usize>,
    distance: f64,
    data: &mut OffsetData,
) -> Result<(), OffsetError> {
    let solid_data = topo.solid(solid)?;
    for shell_id in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        let faces = topo.shell(shell_id)?.faces().to_vec();
        data.shell_faces.push(faces.clone());
        for face_id in faces {
            let face = topo.face(face_id)?;
            let mut surface = face.surface().clone();
            let face_distance = if selected.contains(&face_id.index()) {
                let FaceSurface::Plane { d, .. } = &mut surface else {
                    return Err(OffsetError::UnsupportedMoveFace {
                        face: face_id,
                        surface_type: face.surface().type_tag(),
                        reason: "phase 4.1 moves planar faces only".into(),
                    });
                };
                let signed = if face.is_reversed() {
                    -distance
                } else {
                    distance
                };
                *d += signed;
                signed
            } else {
                0.0
            };
            data.offset_faces.insert(
                face_id,
                OffsetFace {
                    original: face_id,
                    surface,
                    distance: face_distance,
                    status: OffsetStatus::Done,
                },
            );
        }
    }
    Ok(())
}

fn validate_source_edges<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    edge_faces: &std::collections::BTreeMap<usize, V>,
    selected: &HashSet<usize>,
) -> Result<(), OffsetError> {
    for (&edge_index, faces) in edge_faces {
        let edge =
            topo.edge_id_from_index(edge_index)
                .ok_or_else(|| OffsetError::TopologyChange {
                    face: None,
                    edge: None,
                    reason: format!("source edge {edge_index} is unavailable"),
                })?;
        if faces.len() != 2 {
            return Err(OffsetError::TopologyChange {
                face: faces.first().copied(),
                edge: Some(edge),
                reason: format!("source edge has {} face uses, expected 2", faces.len()),
            });
        }
        if faces[0] == faces[1] {
            continue;
        }
        let boundary = selected.contains(&faces[0].index()) ^ selected.contains(&faces[1].index());
        if !boundary {
            continue;
        }
        let neighbor = if selected.contains(&faces[0].index()) {
            faces[1]
        } else {
            faces[0]
        };
        let neighbor_face = topo.face(neighbor)?;
        if !matches!(
            neighbor_face.surface(),
            FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
        ) {
            return Err(OffsetError::UnsupportedMoveFace {
                face: neighbor,
                surface_type: neighbor_face.surface().type_tag(),
                reason: format!(
                    "face is adjacent to selected move boundary edge {}",
                    edge.index()
                ),
            });
        }
    }
    Ok(())
}

fn validate_rebuilt_edges<const ALLOW_CONIC_CHANGE: bool, V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    source_edge_faces: &std::collections::BTreeMap<usize, V>,
    selected: &HashSet<usize>,
    data: &OffsetData,
) -> Result<(), OffsetError> {
    for (&edge_index, faces) in source_edge_faces {
        if faces.len() == 2 && faces[0] == faces[1] {
            continue;
        }
        let edge =
            topo.edge_id_from_index(edge_index)
                .ok_or_else(|| OffsetError::TopologyChange {
                    face: faces.first().copied(),
                    edge: None,
                    reason: format!("source edge {edge_index} is unavailable"),
                })?;
        let matches: Vec<_> = data
            .intersections
            .iter()
            .filter(|intersection| intersection.original_edge == edge)
            .collect();
        let affected = faces.iter().any(|face| selected.contains(&face.index()));
        if !affected && matches.is_empty() {
            continue;
        }
        if matches.len() != 1 || matches[0].new_edges.len() != 1 {
            return Err(OffsetError::TopologyChange {
                face: faces.first().copied(),
                edge: Some(edge),
                reason: format!(
                    "support-surface intersection produced {} records and {} replacement edges",
                    matches.len(),
                    matches.first().map_or(0, |item| item.new_edges.len())
                ),
            });
        }
        let source_curve = topo.edge(edge)?.curve();
        let source_kind = source_curve.type_tag();
        let replacement = matches[0].new_edges[0];
        let replacement_curve = topo.edge(replacement)?.curve();
        let replacement_kind = replacement_curve.type_tag();
        let qualified_conic_change = matches!(
            (source_curve, replacement_curve),
            (EdgeCurve::Circle(_), EdgeCurve::Ellipse(_))
        );
        if source_kind != replacement_kind && !(ALLOW_CONIC_CHANGE && qualified_conic_change) {
            return Err(OffsetError::TopologyChange {
                face: faces.first().copied(),
                edge: Some(edge),
                reason: format!(
                    "intersection curve changed from {source_kind} to {replacement_kind}"
                ),
            });
        }
    }
    Ok(())
}

fn validate_rebuilt_wires(
    topo: &Topology,
    source_wire_shapes: &HashMap<FaceId, Vec<usize>>,
    data: &OffsetData,
) -> Result<(), OffsetError> {
    for (&face, expected) in source_wire_shapes {
        let Some(wires) = data.face_wires.get(&face) else {
            return Err(OffsetError::TopologyChange {
                face: Some(face),
                edge: None,
                reason: "support-surface intersections did not form a closed face".into(),
            });
        };
        let mut actual = wires
            .iter()
            .map(|&wire| topo.wire(wire).map(|item| item.edges().len()))
            .collect::<Result<Vec<_>, _>>()?;
        actual.sort_unstable();
        if &actual != expected {
            return Err(OffsetError::TopologyChange {
                face: Some(face),
                edge: None,
                reason: format!("wire edge counts changed from {expected:?} to {actual:?}"),
            });
        }
    }
    Ok(())
}

fn validate_result_topology(
    topo: &Topology,
    result: SolidId,
    source_counts: (usize, usize, usize),
    source_shell_sizes: &[usize],
) -> Result<(), OffsetError> {
    let result_counts = solid_entity_counts(topo, result)?;
    if result_counts != source_counts {
        return Err(OffsetError::TopologyChange {
            face: None,
            edge: None,
            reason: format!(
                "entity counts changed from {source_counts:?} to {result_counts:?} (F, E, V)"
            ),
        });
    }
    let result_shell_sizes = shell_face_counts(topo, result)?;
    if result_shell_sizes != source_shell_sizes {
        return Err(OffsetError::TopologyChange {
            face: None,
            edge: None,
            reason: format!(
                "shell face counts changed from {source_shell_sizes:?} to {result_shell_sizes:?}"
            ),
        });
    }
    Ok(())
}

fn shell_face_counts(topo: &Topology, solid: SolidId) -> Result<Vec<usize>, OffsetError> {
    let solid_data = topo.solid(solid)?;
    std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .map(|shell| Ok(topo.shell(shell)?.faces().len()))
        .collect()
}

fn wire_shape(topo: &Topology, face: FaceId) -> Result<Vec<usize>, OffsetError> {
    let mut shape = face_wires(topo, face)?
        .into_iter()
        .map(|wire| Ok(topo.wire(wire)?.edges().len()))
        .collect::<Result<Vec<_>, OffsetError>>()?;
    shape.sort_unstable();
    Ok(shape)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use super::*;

    fn assert_edge_oracles(topo: &Topology, edge_id: EdgeId, expected_range: (f64, f64)) {
        let edge = topo.edge(edge_id).unwrap();
        let range = edge.strict_domain().expect("explicit projected authority");
        assert_eq!(range.0.to_bits(), expected_range.0.to_bits());
        assert_eq!(range.1.to_bits(), expected_range.1.to_bits());
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        assert!(
            (edge.curve().evaluate_with_endpoints(range.0, start, end) - start).length() < 1e-9
        );
        assert!((edge.curve().evaluate_with_endpoints(range.1, start, end) - end).length() < 1e-9);
    }

    #[test]
    fn projected_curves_preserve_anchored_seam_and_reversed_branch() {
        let shift = Vec3::new(7.0, -3.0, 2.0);

        let mut topo = Topology::new();
        let source_circle = Circle3D::new_with_ref(
            Point3::new(10.0, 20.0, 30.0),
            Vec3::new(0.0, 0.0, 1.0),
            4.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let circle_range = (2.8, 2.8 + std::f64::consts::TAU);
        let seam = topo.add_vertex(Vertex::new(source_circle.evaluate(circle_range.0), 1e-9));
        let mut source =
            Edge::with_tolerance(seam, seam, EdgeCurve::Circle(source_circle.clone()), None);
        source.set_trim(Some(circle_range));
        let source_id = topo.add_edge(source);
        let result_circle = Circle3D::new_with_ref(
            source_circle.center() + shift,
            source_circle.normal(),
            6.0,
            source_circle.u_axis(),
        )
        .unwrap();
        let result_curve = EdgeCurve::Circle(result_circle);
        let projected = add_projected_curve_edge(
            &mut topo,
            source_id,
            seam,
            seam,
            None,
            &EdgeCurve::Circle(source_circle),
            circle_range,
            result_curve,
            shift,
            1e-7,
        )
        .expect("anchored closed circle");
        assert_edge_oracles(&topo, projected, circle_range);
        let projected_circle_edge = topo.edge(projected).unwrap();
        assert_eq!(projected_circle_edge.tolerance(), None);
        assert_eq!(
            topo.vertex(projected_circle_edge.start())
                .unwrap()
                .tolerance()
                .to_bits(),
            1e-7_f64.to_bits(),
            "an inherited edge must retain the operation tolerance floor"
        );
        let circle_midpoint = (circle_range.0 + circle_range.1) * 0.5;
        let expected_circle_midpoint = Point3::new(
            17.0 + 6.0 * circle_midpoint.cos(),
            17.0 + 6.0 * circle_midpoint.sin(),
            32.0,
        );
        let projected_edge = topo.edge(projected).unwrap();
        let projected_start = topo.vertex(projected_edge.start()).unwrap().point();
        assert!(
            (projected_start
                - Point3::new(
                    17.0 + 6.0 * circle_range.0.cos(),
                    17.0 + 6.0 * circle_range.0.sin(),
                    32.0,
                ))
            .length()
                < 1e-12
        );
        assert!(
            (projected_edge.curve().evaluate_with_endpoints(
                circle_midpoint,
                projected_start,
                projected_start,
            ) - expected_circle_midpoint)
                .length()
                < 1e-12
        );

        let source_ellipse = Ellipse3D::with_axes(
            Point3::new(-8.0, 5.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            5.0,
            2.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let ellipse_range = (2.4, 0.6);
        let ellipse_start =
            topo.add_vertex(Vertex::new(source_ellipse.evaluate(ellipse_range.0), 5e-4));
        let ellipse_end =
            topo.add_vertex(Vertex::new(source_ellipse.evaluate(ellipse_range.1), 2e-4));
        let mut source = Edge::with_tolerance(
            ellipse_start,
            ellipse_end,
            EdgeCurve::Ellipse(source_ellipse.clone()),
            None,
        );
        source.set_trim(Some(ellipse_range));
        let source_id = topo.add_edge(source);
        let result_ellipse = Ellipse3D::with_axes(
            source_ellipse.center() + shift,
            source_ellipse.normal(),
            7.5,
            3.0,
            source_ellipse.u_axis(),
            source_ellipse.v_axis(),
        )
        .unwrap();
        let result_curve = EdgeCurve::Ellipse(result_ellipse);
        let projected = add_projected_curve_edge(
            &mut topo,
            source_id,
            ellipse_start,
            ellipse_end,
            None,
            &EdgeCurve::Ellipse(source_ellipse),
            ellipse_range,
            result_curve,
            shift,
            1e-7,
        )
        .expect("reversed ellipse branch");
        assert_edge_oracles(&topo, projected, ellipse_range);
        let projected_ellipse_edge = topo.edge(projected).unwrap();
        assert_eq!(projected_ellipse_edge.tolerance(), None);
        for vertex in [projected_ellipse_edge.start(), projected_ellipse_edge.end()] {
            assert_eq!(
                topo.vertex(vertex).unwrap().tolerance().to_bits(),
                5e-4_f64.to_bits(),
                "an inherited edge must retain the maximum endpoint tolerance"
            );
        }
        let ellipse_midpoint = (ellipse_range.0 + ellipse_range.1) * 0.5;
        let expected_ellipse_midpoint = Point3::new(
            -1.0 + 7.5 * ellipse_midpoint.cos(),
            2.0 + 3.0 * ellipse_midpoint.sin(),
            3.0,
        );
        let projected_edge = topo.edge(projected).unwrap();
        let projected_start = topo.vertex(projected_edge.start()).unwrap().point();
        let projected_end = topo.vertex(projected_edge.end()).unwrap().point();
        assert!(
            (projected_start
                - Point3::new(
                    -1.0 + 7.5 * ellipse_range.0.cos(),
                    2.0 + 3.0 * ellipse_range.0.sin(),
                    3.0,
                ))
            .length()
                < 1e-12
        );
        assert!(
            (projected_end
                - Point3::new(
                    -1.0 + 7.5 * ellipse_range.1.cos(),
                    2.0 + 3.0 * ellipse_range.1.sin(),
                    3.0,
                ))
            .length()
                < 1e-12
        );
        assert!(
            (projected_edge.curve().evaluate_with_endpoints(
                ellipse_midpoint,
                projected_start,
                projected_end,
            ) - expected_ellipse_midpoint)
                .length()
                < 1e-12
        );
    }
}
