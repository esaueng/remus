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
    let snapshot = topo.clone();
    let result = move_faces_impl(topo, solid, faces, distance);
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

fn move_faces_impl(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<SolidId, OffsetError> {
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
    restore_exact_plane_cylinder_edges(topo, reference_normal, distance, &mut data)?;
    validate_rebuilt_edges(topo, &source_edge_faces, &selected, &data)?;
    build_topology_preserving_wires(
        topo,
        solid,
        reference_normal,
        distance,
        &source_edge_faces,
        &mut data,
    )?;
    validate_rebuilt_wires(topo, &source_wire_shapes, &data)?;

    let result = crate::assemble::assemble_solid(topo, &data)?;
    super::validate_offset_result(topo, result)?;
    validate_result_topology(topo, result, source_counts, source_shell_sizes.as_slice())?;
    Ok(result)
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

fn restore_exact_plane_cylinder_edges(
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
        let shift = if face_a.distance != 0.0 || face_b.distance != 0.0 {
            move_normal * distance
        } else {
            Vec3::new(0.0, 0.0, 0.0)
        };
        if let Some(edge) = exact_plane_cylinder_edge(
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

fn exact_plane_cylinder_edge(
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
        _ => return Ok(None),
    };
    Ok(Some(add_projected_curve_edge(
        topo,
        source_start,
        source_end,
        source_tolerance,
        curve,
        shift,
        tolerance,
    )?))
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
    let source_start_point = topo.vertex(source_start)?.point() + shift;
    let source_end_point = topo.vertex(source_end)?.point() + shift;
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
    let vertex_tolerance = edge_tolerance.unwrap_or(tolerance);
    let start = topo.add_vertex(Vertex::new(project(source_start_point), vertex_tolerance));
    let end = topo.add_vertex(Vertex::new(project(source_end_point), vertex_tolerance));
    Ok(Some(topo.add_edge(Edge::with_tolerance(
        start,
        end,
        EdgeCurve::Line,
        edge_tolerance,
    ))))
}

fn add_projected_curve_edge(
    topo: &mut Topology,
    source_start: VertexId,
    source_end: VertexId,
    edge_tolerance: Option<f64>,
    curve: EdgeCurve,
    shift: Vec3,
    tolerance: f64,
) -> Result<EdgeId, OffsetError> {
    let source_start_point = topo.vertex(source_start)?.point() + shift;
    let start_point = project_to_curve(&curve, source_start_point);
    let vertex_tolerance = edge_tolerance.unwrap_or(tolerance);
    let start = topo.add_vertex(Vertex::new(start_point, vertex_tolerance));
    let end = if source_start == source_end {
        start
    } else {
        let source_end_point = topo.vertex(source_end)?.point() + shift;
        let end_point = project_to_curve(&curve, source_end_point);
        topo.add_vertex(Vertex::new(end_point, vertex_tolerance))
    };
    Ok(topo.add_edge(Edge::with_tolerance(start, end, curve, edge_tolerance)))
}

fn project_to_curve(curve: &EdgeCurve, point: Point3) -> Point3 {
    match curve {
        EdgeCurve::Circle(circle) => circle.evaluate(circle.project(point)),
        EdgeCurve::Ellipse(ellipse) => ellipse.evaluate(ellipse.project(point)),
        _ => point,
    }
}

fn distance_to_line(point: Point3, origin: Point3, direction: Vec3) -> f64 {
    let delta = point - origin;
    (delta - direction * delta.dot(direction)).length()
}

#[allow(clippy::too_many_lines)]
fn build_topology_preserving_wires<V: std::ops::Deref<Target = [FaceId]>>(
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
            let predicted = source.point() + move_normal * distance;
            rebuild_vertex_point(
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
        let curve = preliminary.get(&source_edge.index()).map_or_else(
            || Ok::<_, OffsetError>(source.curve().clone()),
            |replacement| Ok(topo.edge(*replacement)?.curve().clone()),
        )?;
        let mut rebuilt = Edge::with_tolerance(start, end, curve, source.tolerance());
        rebuilt.set_trim(source.trim());
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

#[allow(clippy::too_many_arguments)]
fn rebuild_vertex_point<V: std::ops::Deref<Target = [FaceId]>>(
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
        .map(|edge| preliminary.get(&edge.index()).copied().unwrap_or(*edge))
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

fn validate_rebuilt_edges<V: std::ops::Deref<Target = [FaceId]>>(
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
        let source_kind = topo.edge(edge)?.curve().type_tag();
        let replacement = matches[0].new_edges[0];
        let replacement_kind = topo.edge(replacement)?.curve().type_tag();
        if source_kind != replacement_kind {
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
