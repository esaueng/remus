//! Exact, local reconstruction of analytic wounds.
//!
//! This module contains narrowly qualified topology surgery shared by blend
//! removal and direct edits.  It deliberately does not call the general
//! surface-intersection marcher: every replacement edge has a closed-form 3D
//! carrier. Sampling participates only in a derived spherical UV p-curve,
//! which is rejected unless the strict p-curve validator proves a full-interval
//! error bound at the kernel's standard tolerance.

use std::collections::{BTreeMap, BTreeSet};

use remus_algo::{PlaneFrame, compute_pcurve_on_surface_in_domain};
use remus_math::curves::Circle3D;
use remus_math::curves2d::{Circle2D, Curve2D, NurbsCurve2D};
use remus_math::plane::plane_plane_intersection;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntityKey;
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::OperationsError;
use crate::defeature::DefeatureOutcome;
use crate::resize_blend::ResizeBlendError;

/// The 3D edge is analytic and authoritative.  A non-planar p-curve is a
/// derived parameter-space aid and must stay within this measured deviation.

#[derive(Clone, Copy)]
struct UnitPlane {
    normal: Vec3,
    d: f64,
}

struct SphereEndPlan {
    band: FaceId,
    supports: [FaceId; 2],
    springs: [EdgeId; 2],
    planar_cross: EdgeId,
    sphere_cross: EdgeId,
    planar_end: FaceId,
    sphere_end: FaceId,
    planar_boundaries: [EdgeId; 2],
    sphere_boundaries: [EdgeId; 2],
    planar_vertices: [VertexId; 2],
    sphere_vertices: [VertexId; 2],
    planar_corner: Point3,
    sphere_corner: Point3,
}

fn reconstruction(reason: impl Into<String>) -> OperationsError {
    ResizeBlendError::ReconstructionFailed {
        reason: reason.into(),
    }
    .into()
}

fn unit_plane(topo: &Topology, face: FaceId) -> Result<Option<UnitPlane>, OperationsError> {
    let FaceSurface::Plane { normal, d } = topo.face(face)?.surface() else {
        return Ok(None);
    };
    let length = normal.length();
    if length <= Tolerance::new().angular {
        return Err(reconstruction(format!(
            "face {} has a degenerate plane normal",
            face.index()
        )));
    }
    Ok(Some(UnitPlane {
        normal: *normal * (1.0 / length),
        d: *d / length,
    }))
}

fn edge_vertices(topo: &Topology, edge: EdgeId) -> Result<[VertexId; 2], OperationsError> {
    let edge = topo.edge(edge)?;
    Ok([edge.start(), edge.end()])
}

fn shared_edges(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    first: FaceId,
    second: FaceId,
) -> Result<Vec<EdgeId>, OperationsError> {
    let mut result = Vec::new();
    for edge in remus_topology::explorer::face_edges(topo, first)? {
        let faces = adjacency.faces_for_edge(edge);
        if faces.contains(&first) && faces.contains(&second) {
            result.push(edge);
        }
    }
    result.sort_unstable_by_key(|edge| edge.index());
    result.dedup();
    Ok(result)
}

fn other_face(
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    edge: EdgeId,
    excluded: FaceId,
) -> Option<FaceId> {
    let mut faces: Vec<_> = adjacency
        .faces_for_edge(edge)
        .iter()
        .copied()
        .filter(|face| *face != excluded)
        .collect();
    faces.sort_unstable_by_key(|face| face.index());
    faces.dedup();
    let [face] = faces.as_slice() else {
        return None;
    };
    Some(*face)
}

fn shared_vertex(first: [VertexId; 2], second: [VertexId; 2]) -> Option<VertexId> {
    let mut common = first
        .into_iter()
        .filter(|vertex| second.contains(vertex))
        .collect::<Vec<_>>();
    common.sort_unstable_by_key(|vertex| vertex.index());
    common.dedup();
    let [vertex] = common.as_slice() else {
        return None;
    };
    Some(*vertex)
}

fn incident_pair_edge(
    topo: &Topology,
    solid_edges: &[EdgeId],
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    vertex: VertexId,
    first: FaceId,
    second: FaceId,
    excluded: &BTreeSet<EdgeId>,
) -> Result<Option<EdgeId>, OperationsError> {
    let mut candidates = Vec::new();
    for &edge_id in solid_edges {
        if excluded.contains(&edge_id) {
            continue;
        }
        let edge = topo.edge(edge_id)?;
        if edge.start() != vertex && edge.end() != vertex {
            continue;
        }
        let faces = adjacency.faces_for_edge(edge_id);
        if faces.contains(&first) && faces.contains(&second) {
            candidates.push(edge_id);
        }
    }
    candidates.sort_unstable_by_key(|edge| edge.index());
    candidates.dedup();
    let [edge] = candidates.as_slice() else {
        return Ok(None);
    };
    Ok(Some(*edge))
}

fn triple_plane_corner(a: UnitPlane, b: UnitPlane, c: UnitPlane) -> Option<Point3> {
    let bc = b.normal.cross(c.normal);
    let det = a.normal.dot(bc);
    if det.abs() < crate::defeature::MIN_PLANE_TRIPLE_DET {
        return None;
    }
    let ca = c.normal.cross(a.normal);
    let ab = a.normal.cross(b.normal);
    let value = (bc * a.d + ca * b.d + ab * c.d) * (1.0 / det);
    Some(Point3::new(value.x(), value.y(), value.z()))
}

fn line_sphere_roots(origin: Point3, direction: Vec3, center: Point3, radius: f64) -> Vec<Point3> {
    let offset = origin - center;
    let a = direction.dot(direction);
    if !a.is_finite() || a <= Tolerance::new().angular * Tolerance::new().angular {
        return Vec::new();
    }
    let closest_parameter = -offset.dot(direction) / a;
    let closest_offset = offset + direction * closest_parameter;
    let span_squared = radius.mul_add(radius, -closest_offset.dot(closest_offset));
    let scale_squared = radius
        .mul_add(radius, closest_offset.dot(closest_offset))
        .max(offset.dot(offset))
        .max(1.0);
    let arithmetic_bound = 64.0 * f64::EPSILON * scale_squared;
    let geometric_bound = Tolerance::new().linear * Tolerance::new().linear;
    let tangent_bound = geometric_bound + arithmetic_bound;
    if span_squared <= tangent_bound {
        return Vec::new();
    }
    let closest = origin + direction * closest_parameter;
    let parameter_span = (span_squared / a).sqrt();
    let mut points = vec![
        closest - direction * parameter_span,
        closest + direction * parameter_span,
    ];
    points.sort_by(|left, right| {
        left.x()
            .total_cmp(&right.x())
            .then_with(|| left.y().total_cmp(&right.y()))
            .then_with(|| left.z().total_cmp(&right.z()))
    });
    points
}

fn certify_plane_sphere_circle(
    circle: &Circle3D,
    plane: UnitPlane,
    sphere: &remus_math::surfaces::SphericalSurface,
) -> bool {
    let tol = Tolerance::new();
    if 1.0 - circle.normal().dot(plane.normal).abs() > tol.angular {
        return false;
    }
    if (plane.normal.dot(Vec3::new(
        circle.center().x(),
        circle.center().y(),
        circle.center().z(),
    )) - plane.d)
        .abs()
        > tol.linear
    {
        return false;
    }
    let signed = circle.normal().dot(sphere.center() - circle.center());
    let projected = sphere.center() - circle.normal() * signed;
    if (projected - circle.center()).length() > tol.linear {
        return false;
    }
    let expected_sq = sphere.radius().mul_add(sphere.radius(), -(signed * signed));
    expected_sq >= 0.0
        && (circle.radius() * circle.radius() - expected_sq).abs()
            <= tol.linear * sphere.radius().max(1.0)
}

fn classify_sphere_end(
    topo: &Topology,
    solid: SolidId,
    band: FaceId,
    supports: [FaceId; 2],
) -> Result<Option<SphereEndPlan>, OperationsError> {
    if supports[0] == supports[1]
        || !remus_topology::explorer::solid_faces(topo, solid)?.contains(&band)
    {
        return Ok(None);
    }
    let Some(support_plane0) = unit_plane(topo, supports[0])? else {
        return Ok(None);
    };
    let Some(support_plane1) = unit_plane(topo, supports[1])? else {
        return Ok(None);
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(band)?.surface() else {
        return Ok(None);
    };
    if !topo.face(band)?.inner_wires().is_empty() {
        return Ok(None);
    }
    let adjacency = topo.build_adjacency(solid)?;
    let mut springs = Vec::new();
    for support in supports {
        let contacts = shared_edges(topo, &adjacency, band, support)?;
        let [contact] = contacts.as_slice() else {
            return Ok(None);
        };
        if !matches!(topo.edge(*contact)?.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        springs.push(*contact);
    }
    let springs: [EdgeId; 2] = springs
        .try_into()
        .map_err(|_| reconstruction("sphere termination lost a support contact"))?;
    let band_edges = topo.wire(topo.face(band)?.outer_wire())?.edges();
    if band_edges.len() != 4 {
        return Ok(None);
    }
    let spring_set: BTreeSet<_> = springs.into_iter().collect();
    let crosses: Vec<_> = band_edges
        .iter()
        .map(OrientedEdge::edge)
        .filter(|edge| !spring_set.contains(edge))
        .collect();
    let [first_cross, second_cross] = crosses.as_slice() else {
        return Ok(None);
    };
    let Some(first_end) = other_face(&adjacency, *first_cross, band) else {
        return Ok(None);
    };
    let Some(second_end) = other_face(&adjacency, *second_cross, band) else {
        return Ok(None);
    };
    let (planar_cross, planar_end, sphere_cross, sphere_end) = match (
        topo.face(first_end)?.surface(),
        topo.face(second_end)?.surface(),
    ) {
        (FaceSurface::Plane { .. }, FaceSurface::Sphere(_)) => {
            (*first_cross, first_end, *second_cross, second_end)
        }
        (FaceSurface::Sphere(_), FaceSurface::Plane { .. }) => {
            (*second_cross, second_end, *first_cross, first_end)
        }
        _ => return Ok(None),
    };
    let Some(planar_end_plane) = unit_plane(topo, planar_end)? else {
        return Ok(None);
    };
    let FaceSurface::Sphere(sphere) = topo.face(sphere_end)?.surface() else {
        return Ok(None);
    };
    let cross_edges: BTreeSet<_> = [planar_cross, sphere_cross]
        .into_iter()
        .chain(springs)
        .collect();
    let solid_edges = remus_topology::explorer::solid_edges(topo, solid)?;
    let mut planar_vertices = Vec::new();
    let mut sphere_vertices = Vec::new();
    let mut planar_boundaries = Vec::new();
    let mut sphere_boundaries = Vec::new();
    for (index, support) in supports.into_iter().enumerate() {
        let spring_vertices = edge_vertices(topo, springs[index])?;
        let Some(planar_vertex) =
            shared_vertex(spring_vertices, edge_vertices(topo, planar_cross)?)
        else {
            return Ok(None);
        };
        let Some(sphere_vertex) =
            shared_vertex(spring_vertices, edge_vertices(topo, sphere_cross)?)
        else {
            return Ok(None);
        };
        if planar_vertex == sphere_vertex {
            return Ok(None);
        }
        let Some(planar_boundary) = incident_pair_edge(
            topo,
            &solid_edges,
            &adjacency,
            planar_vertex,
            support,
            planar_end,
            &cross_edges,
        )?
        else {
            return Ok(None);
        };
        let Some(sphere_boundary) = incident_pair_edge(
            topo,
            &solid_edges,
            &adjacency,
            sphere_vertex,
            support,
            sphere_end,
            &cross_edges,
        )?
        else {
            return Ok(None);
        };
        if !matches!(topo.edge(planar_boundary)?.curve(), EdgeCurve::Line) {
            return Ok(None);
        }
        let EdgeCurve::Circle(circle) = topo.edge(sphere_boundary)?.curve() else {
            return Ok(None);
        };
        if topo.edge(sphere_boundary)?.strict_domain().is_err()
            || !certify_plane_sphere_circle(circle, [support_plane0, support_plane1][index], sphere)
        {
            return Ok(None);
        }
        planar_vertices.push(planar_vertex);
        sphere_vertices.push(sphere_vertex);
        planar_boundaries.push(planar_boundary);
        sphere_boundaries.push(sphere_boundary);
    }
    let planar_vertices: [VertexId; 2] = planar_vertices
        .try_into()
        .map_err(|_| reconstruction("sphere termination lost planar vertices"))?;
    let sphere_vertices: [VertexId; 2] = sphere_vertices
        .try_into()
        .map_err(|_| reconstruction("sphere termination lost sphere vertices"))?;
    let planar_boundaries: [EdgeId; 2] = planar_boundaries
        .try_into()
        .map_err(|_| reconstruction("sphere termination lost planar boundaries"))?;
    let sphere_boundaries: [EdgeId; 2] = sphere_boundaries
        .try_into()
        .map_err(|_| reconstruction("sphere termination lost sphere boundaries"))?;

    // Every terminal vertex is the simple three-face corner that will be
    // replaced.  A point-contacting fourth face would crack if silently left.
    for vertex in planar_vertices.into_iter().chain(sphere_vertices) {
        let mut users = Vec::new();
        for &edge in &solid_edges {
            let data = topo.edge(edge)?;
            if data.start() == vertex || data.end() == vertex {
                users.extend(adjacency.faces_for_edge(edge).iter().copied());
            }
        }
        users.sort_unstable_by_key(|face| face.index());
        users.dedup();
        if users.len() != 3 || !users.contains(&band) {
            return Ok(None);
        }
    }

    let planar_corner = triple_plane_corner(support_plane0, support_plane1, planar_end_plane)
        .ok_or_else(|| reconstruction("sphere-ended band has no planar sharp corner"))?;
    let Some((line_origin, line_direction)) = plane_plane_intersection(
        support_plane0.normal,
        support_plane0.d,
        support_plane1.normal,
        support_plane1.d,
        Tolerance::new().angular,
    ) else {
        return Ok(None);
    };
    let candidates = line_sphere_roots(
        line_origin,
        line_direction,
        sphere.center(),
        sphere.radius(),
    );
    if candidates.is_empty() {
        return Err(reconstruction(
            "sharp support line does not intersect the spherical termination",
        ));
    }
    let terminal_points = [
        topo.vertex(sphere_vertices[0])?.point(),
        topo.vertex(sphere_vertices[1])?.point(),
    ];
    let planar_points = [
        topo.vertex(planar_vertices[0])?.point(),
        topo.vertex(planar_vertices[1])?.point(),
    ];
    let travel0 = (terminal_points[0] - planar_points[0])
        .normalize()
        .map_err(|error| reconstruction(format!("degenerate first spring: {error}")))?;
    let travel1 = (terminal_points[1] - planar_points[1])
        .normalize()
        .map_err(|error| reconstruction(format!("degenerate second spring: {error}")))?;
    if travel0.dot(travel1) < 1.0 - Tolerance::new().angular {
        return Ok(None);
    }
    let wound_direction = (travel0 + travel1)
        .normalize()
        .map_err(|error| reconstruction(format!("opposed spring directions: {error}")))?;
    let radial_dot = |point: Point3| {
        (point - sphere.center())
            .normalize()
            .map(|normal| normal.dot(wound_direction))
    };
    let reference_side = terminal_points
        .iter()
        .map(|point| radial_dot(*point))
        .collect::<Result<Vec<_>, _>>()?;
    if reference_side
        .iter()
        .any(|side| side.abs() <= Tolerance::new().angular)
        || reference_side[0].is_sign_positive() != reference_side[1].is_sign_positive()
    {
        return Ok(None);
    }
    let mut admissible = Vec::new();
    for candidate in candidates {
        // The retained sharp edge must advance from the planar cap toward the
        // old spherical end, never through the body to the disconnected far
        // sphere root.  Each old plane/sphere trim must also extend to the
        // candidate along its existing parameter direction by less than half
        // a circle.  These are construction-side proofs, not a nearest-root
        // heuristic.
        if (candidate - planar_corner).dot(wound_direction) <= Tolerance::new().linear {
            continue;
        }
        // Stay on the same spherical cap hemisphere as both old terminals;
        // the opposite line/sphere root belongs to a disconnected continuation
        // of the carrier, even though it lies on the same infinite sphere.
        let candidate_side = radial_dot(candidate)?;
        if candidate_side.abs() <= Tolerance::new().angular
            || candidate_side.is_sign_positive() != reference_side[0].is_sign_positive()
        {
            continue;
        }
        if locally_extends_circle_edge(topo, sphere_boundaries[0], sphere_vertices[0], candidate)?
            && locally_extends_circle_edge(
                topo,
                sphere_boundaries[1],
                sphere_vertices[1],
                candidate,
            )?
        {
            admissible.push(candidate);
        }
    }
    let [sphere_corner] = admissible.as_slice() else {
        return Err(reconstruction(format!(
            "sphere termination has {} construct-valid sharp roots; expected exactly one",
            admissible.len()
        )));
    };
    let sphere_corner = *sphere_corner;

    let axis = cylinder
        .axis()
        .normalize()
        .map_err(|error| reconstruction(format!("invalid blend cylinder axis: {error}")))?;
    for (index, spring) in springs.into_iter().enumerate() {
        let edge = topo.edge(spring)?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let direction = (end - start)
            .normalize()
            .map_err(|error| reconstruction(format!("degenerate spring: {error}")))?;
        if 1.0 - direction.dot(axis).abs() > Tolerance::new().angular {
            return Ok(None);
        }
        for point in [start, end] {
            let radial = (point - cylinder.origin()) - axis * (point - cylinder.origin()).dot(axis);
            if (radial.length() - cylinder.radius()).abs() > Tolerance::new().linear
                || ([support_plane0, support_plane1][index]
                    .normal
                    .dot(Vec3::new(point.x(), point.y(), point.z()))
                    - [support_plane0, support_plane1][index].d)
                    .abs()
                    > Tolerance::new().linear
            {
                return Ok(None);
            }
        }
    }

    let patch_span = terminal_points
        .into_iter()
        .chain(planar_points)
        .fold(0.0_f64, |span, point| {
            span.max((point - terminal_points[0]).length())
        });
    let max_displacement =
        patch_span.max(cylinder.radius()) * crate::defeature::MAX_HEAL_DISPLACEMENT_FACTOR;
    for (old, new) in planar_vertices
        .into_iter()
        .map(|vertex| (vertex, planar_corner))
        .chain(
            sphere_vertices
                .into_iter()
                .map(|vertex| (vertex, sphere_corner)),
        )
    {
        if (topo.vertex(old)?.point() - new).length() > max_displacement {
            return Err(reconstruction(
                "sphere termination sharp corner lies outside the local blend patch",
            ));
        }
    }

    Ok(Some(SphereEndPlan {
        band,
        supports,
        springs,
        planar_cross,
        sphere_cross,
        planar_end,
        sphere_end,
        planar_boundaries,
        sphere_boundaries,
        planar_vertices,
        sphere_vertices,
        planar_corner,
        sphere_corner,
    }))
}

fn extended_circle_trim(
    circle: &Circle3D,
    old_domain: (f64, f64),
    replace_start: bool,
    new_point: Point3,
) -> Result<(f64, f64), OperationsError> {
    let tau = std::f64::consts::TAU;
    let angular = Tolerance::new().angular;
    let projected = circle.project(new_point);
    let old_span = old_domain.1 - old_domain.0;
    let old_terminal = if replace_start {
        old_domain.0
    } else {
        old_domain.1
    };
    let mut candidates = Vec::new();
    for shift in -3..=3 {
        let candidate = projected + f64::from(shift) * tau;
        let domain = if replace_start {
            (candidate, old_domain.1)
        } else {
            (old_domain.0, candidate)
        };
        let span = domain.1 - domain.0;
        let extends = if old_span.is_sign_positive() {
            span > 0.0
                && if replace_start {
                    candidate <= old_domain.0 + angular
                } else {
                    candidate >= old_domain.1 - angular
                }
        } else {
            span < 0.0
                && if replace_start {
                    candidate >= old_domain.0 - angular
                } else {
                    candidate <= old_domain.1 + angular
                }
        };
        if extends && span.abs() <= tau + angular {
            candidates.push(((candidate - old_terminal).abs(), domain));
        }
    }
    candidates.sort_by(|left, right| left.0.total_cmp(&right.0));
    let Some((_, domain)) = candidates.first().copied() else {
        return Err(reconstruction(
            "extended sphere boundary has no unambiguous periodic trim",
        ));
    };
    Ok(domain)
}

fn locally_extends_circle_edge(
    topo: &Topology,
    edge_id: EdgeId,
    old_vertex: VertexId,
    candidate: Point3,
) -> Result<bool, OperationsError> {
    let edge = topo.edge(edge_id)?;
    let EdgeCurve::Circle(circle) = edge.curve() else {
        return Ok(false);
    };
    let replace_start = if edge.start() == old_vertex {
        true
    } else if edge.end() == old_vertex {
        false
    } else {
        return Ok(false);
    };
    let old_domain = edge.strict_domain().map_err(|error| {
        reconstruction(format!(
            "sphere boundary edge {} has no authoritative domain: {error}",
            edge_id.index()
        ))
    })?;
    let Ok(new_domain) = extended_circle_trim(circle, old_domain, replace_start, candidate) else {
        return Ok(false);
    };
    let extension = (new_domain.1 - new_domain.0).abs() - (old_domain.1 - old_domain.0).abs();
    if extension < -Tolerance::new().angular
        || extension > std::f64::consts::PI - Tolerance::new().angular
    {
        return Ok(false);
    }
    let parameter = if replace_start {
        new_domain.0
    } else {
        new_domain.1
    };
    Ok((circle.evaluate(parameter) - candidate).length() <= Tolerance::new().linear)
}

fn replacement_edge(
    topo: &mut Topology,
    source: EdgeId,
    old_vertex: VertexId,
    new_vertex: VertexId,
) -> Result<EdgeId, OperationsError> {
    let source_edge = topo.edge(source)?;
    let (start, end, replace_start) = if source_edge.start() == old_vertex {
        (new_vertex, source_edge.end(), true)
    } else if source_edge.end() == old_vertex {
        (source_edge.start(), new_vertex, false)
    } else {
        return Err(reconstruction(format!(
            "edge {} does not reach terminal vertex {}",
            source.index(),
            old_vertex.index()
        )));
    };
    let curve = source_edge.curve().clone();
    let tolerance = source_edge.tolerance();
    let old_domain = source_edge.strict_domain().map_err(|error| {
        reconstruction(format!(
            "source edge {} has no authoritative domain: {error}",
            source.index()
        ))
    })?;
    let new_point = topo.vertex(new_vertex)?.point();
    let mut result = Edge::with_tolerance(start, end, curve.clone(), tolerance);
    if let EdgeCurve::Circle(circle) = &curve {
        let domain = extended_circle_trim(circle, old_domain, replace_start, new_point)?;
        result.set_trim(Some(domain));
    }
    let result = topo.add_edge(result);
    topo.edge(result)?.strict_domain().map_err(|error| {
        reconstruction(format!(
            "replacement edge {} has no authoritative domain: {error}",
            result.index()
        ))
    })?;
    Ok(result)
}

fn mapped_vertex(
    vertex: VertexId,
    planar: &[VertexId; 2],
    sphere: &[VertexId; 2],
    planar_corner: VertexId,
    sphere_corner: VertexId,
) -> VertexId {
    if planar.contains(&vertex) {
        planar_corner
    } else if sphere.contains(&vertex) {
        sphere_corner
    } else {
        vertex
    }
}

#[allow(clippy::too_many_arguments)]
fn splice_face(
    topo: &mut Topology,
    face: FaceId,
    springs: &[EdgeId; 2],
    crosses: &[EdgeId; 2],
    replacements: &BTreeMap<EdgeId, EdgeId>,
    planar_vertices: &[VertexId; 2],
    sphere_vertices: &[VertexId; 2],
    planar_corner: VertexId,
    sphere_corner: VertexId,
    sharp: EdgeId,
) -> Result<(), OperationsError> {
    let face_data = topo.face(face)?;
    let old_wire = face_data.outer_wire();
    let inner = face_data.inner_wires().to_vec();
    let sequence = topo.wire(old_wire)?.edges().to_vec();
    let wound: BTreeSet<_> = springs
        .iter()
        .chain(crosses.iter())
        .chain(replacements.keys())
        .copied()
        .collect();
    if !sequence.iter().any(|edge| wound.contains(&edge.edge())) {
        return Err(reconstruction(format!(
            "face {} has no sphere-termination wound on its outer wire",
            face.index()
        )));
    }
    let mut rebuilt = Vec::with_capacity(sequence.len());
    for oriented in sequence {
        let edge_id = oriented.edge();
        if crosses.contains(&edge_id) {
            continue;
        }
        if springs.contains(&edge_id) {
            let edge = topo.edge(edge_id)?;
            let old_start = oriented.oriented_start(edge);
            let old_end = oriented.oriented_end(edge);
            let target_start = mapped_vertex(
                old_start,
                planar_vertices,
                sphere_vertices,
                planar_corner,
                sphere_corner,
            );
            let target_end = mapped_vertex(
                old_end,
                planar_vertices,
                sphere_vertices,
                planar_corner,
                sphere_corner,
            );
            let sharp_edge = topo.edge(sharp)?;
            let forward = sharp_edge.start() == target_start && sharp_edge.end() == target_end;
            let reverse = sharp_edge.start() == target_end && sharp_edge.end() == target_start;
            if !forward && !reverse {
                return Err(reconstruction("spring does not map to the sharp edge"));
            }
            rebuilt.push(OrientedEdge::new(sharp, forward));
            continue;
        }
        if let Some(&replacement) = replacements.get(&edge_id) {
            rebuilt.push(OrientedEdge::new(replacement, oriented.is_forward()));
        } else {
            rebuilt.push(oriented);
        }
    }
    let wire = topo.add_wire(Wire::new(rebuilt, true)?);
    topo.set_face_boundary_wires(face, wire, inner)?;
    Ok(())
}

fn register_pcurve(
    topo: &mut Topology,
    edge_id: EdgeId,
    face_id: FaceId,
) -> Result<(), OperationsError> {
    let face = topo.face(face_id)?;
    let surface = face.surface().clone();
    let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();
    let mut use_orientation = None;
    for wire in &wire_ids {
        for oriented in topo.wire(*wire)?.edges() {
            if oriented.edge() == edge_id
                && use_orientation.replace(oriented.is_forward()).is_some()
            {
                return Err(reconstruction(format!(
                    "edge {} occurs more than once on face {}",
                    edge_id.index(),
                    face_id.index()
                )));
            }
        }
    }
    let Some(forward) = use_orientation else {
        return Err(reconstruction(format!(
            "edge {} is absent from face {}",
            edge_id.index(),
            face_id.index()
        )));
    };
    let outer = face.outer_wire();
    let wire_points: Vec<_> = topo
        .wire(outer)?
        .edges()
        .iter()
        .map(|oriented| {
            let edge = topo.edge(oriented.edge())?;
            topo.vertex(oriented.oriented_start(edge))
                .map(remus_topology::vertex::Vertex::point)
        })
        .collect::<Result<_, _>>()?;
    let plane_frame = if let FaceSurface::Plane { normal, d } = &surface {
        let denom = normal.dot(*normal);
        let anchor = Point3::new(
            normal.x() * d / denom,
            normal.y() * d / denom,
            normal.z() * d / denom,
        );
        Some(PlaneFrame::from_normal_and_point(*normal, anchor))
    } else {
        None
    };
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let domain = edge.strict_domain().map_err(|error| {
        reconstruction(format!(
            "edge {} has no authoritative domain for p-curve construction: {error}",
            edge_id.index()
        ))
    })?;
    let (curve, edge_start_parameter, edge_end_parameter) = match (&surface, edge.curve()) {
        (FaceSurface::Plane { .. }, EdgeCurve::Line) => {
            let curve = compute_pcurve_on_surface_in_domain(
                edge.curve(),
                start,
                end,
                domain,
                &surface,
                &wire_points,
                plane_frame.as_ref(),
            )?;
            (curve, 0.0, (end - start).length())
        }
        (FaceSurface::Plane { .. }, EdgeCurve::Circle(circle)) => {
            let frame = plane_frame
                .as_ref()
                .ok_or_else(|| reconstruction("planar circle p-curve lost its plane frame"))?;
            let curve_2d = Circle2D::new(frame.project(circle.center()), circle.radius())?;
            let start_2d = frame.project(circle.evaluate(domain.0));
            let end_2d = frame.project(circle.evaluate(domain.1));
            let midpoint_2d = frame.project(circle.evaluate(0.5 * (domain.0 + domain.1)));
            let a0 = curve_2d.project(start_2d);
            let a1 = curve_2d.project(end_2d);
            let mut candidates = (-1..=1)
                .map(|turn| a1 + f64::from(turn) * std::f64::consts::TAU)
                .filter(|candidate| {
                    (*candidate - a0).abs() <= std::f64::consts::TAU + Tolerance::new().angular
                })
                .map(|candidate| {
                    let represented_midpoint = curve_2d.evaluate(0.5 * (a0 + candidate));
                    ((represented_midpoint - midpoint_2d).length(), candidate)
                })
                .collect::<Vec<_>>();
            candidates.sort_by(|left, right| left.0.total_cmp(&right.0));
            let Some((midpoint_error, a1_unwrapped)) = candidates.first().copied() else {
                return Err(reconstruction(
                    "planar circle p-curve has no periodic branch",
                ));
            };
            if midpoint_error > Tolerance::new().linear {
                return Err(reconstruction(format!(
                    "planar circle p-curve branch misses its analytic midpoint by {midpoint_error:.3e}"
                )));
            }
            (Curve2D::Circle(curve_2d), a0, a1_unwrapped)
        }
        (FaceSurface::Sphere(_), EdgeCurve::Circle(_)) => {
            // A general plane/sphere circle is transcendental in the
            // longitude/latitude chart.  The 3D circle remains exact and
            // authoritative; this derived NURBS p-curve is accepted only
            // after the standard SameParameter check below proves it stays
            // within the kernel's default linear tolerance.
            const SEGMENTS: usize = 256;
            let mut parameters = Vec::with_capacity(SEGMENTS + 1);
            let mut points = Vec::with_capacity(SEGMENTS + 1);
            let mut previous_u = None;
            for sample in 0..=SEGMENTS {
                #[allow(clippy::cast_precision_loss)]
                let parameter = sample as f64 / SEGMENTS as f64;
                let edge_parameter = domain.0 + parameter * (domain.1 - domain.0);
                let point = edge
                    .curve()
                    .evaluate_with_endpoints(edge_parameter, start, end);
                let (mut u, v) = surface
                    .project_point(point)
                    .ok_or_else(|| reconstruction("sphere circle p-curve projection failed"))?;
                if let Some(previous) = previous_u {
                    if u - previous > std::f64::consts::PI {
                        u -= std::f64::consts::TAU;
                    } else if u - previous < -std::f64::consts::PI {
                        u += std::f64::consts::TAU;
                    }
                }
                previous_u = Some(u);
                parameters.push(parameter);
                points.push(Point3::new(u, v, 0.0));
            }
            let fitted = remus_math::nurbs::interpolate_with_params(&points, 3, &parameters)?;
            let controls = fitted
                .control_points()
                .iter()
                .map(|point| Point2::new(point.x(), point.y()))
                .collect();
            let curve = Curve2D::Nurbs(NurbsCurve2D::new(
                fitted.degree(),
                fitted.knots().to_vec(),
                controls,
                fitted.weights().to_vec(),
            )?);
            (curve, 0.0, 1.0)
        }
        _ => {
            return Err(reconstruction(format!(
                "exact local-wound p-curve is unavailable for {} on {}",
                edge.curve().type_tag(),
                surface.type_tag()
            )));
        }
    };
    let (t0, t1) = if forward {
        (edge_start_parameter, edge_end_parameter)
    } else {
        (edge_end_parameter, edge_start_parameter)
    };
    topo.set_pcurve_oriented(edge_id, face_id, forward, PCurve::new(curve, t0, t1))?;
    remus_topology::validation::validate_same_parameter(
        topo,
        edge_id,
        face_id,
        forward,
        Tolerance::new().linear,
        1025,
    )?;
    remus_topology::validation::validate_same_parameter_strict(
        topo,
        edge_id,
        face_id,
        forward,
        Tolerance::new().linear,
        1025,
    )
    .map_err(|error| reconstruction(format!("strict p-curve proof failed: {error}")))?;
    remus_topology::validation::validate_same_range_strict(
        topo,
        edge_id,
        face_id,
        forward,
        Tolerance::new().linear,
    )
    .map_err(|error| reconstruction(format!("strict p-curve range proof failed: {error}")))?;
    Ok(())
}

fn certify_result(topo: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    let report = crate::validate::validate_solid(topo, solid)?;
    if !report.is_valid() {
        return Err(reconstruction(format!(
            "sphere termination reconstruction failed validation with {} error(s)",
            report.error_count()
        )));
    }
    let pcurves = remus_topology::validation::validate_solid_pcurve_contracts(
        topo,
        solid,
        Tolerance::new().linear,
        1025,
    )
    .map_err(|error| reconstruction(format!("sphere termination p-curve proof failed: {error}")))?;
    if pcurves.validated_uses != pcurves.stored_pcurves {
        return Err(reconstruction(format!(
            "sphere termination proves {} of {} stored p-curve uses",
            pcurves.validated_uses, pcurves.stored_pcurves
        )));
    }
    let adjacency = topo.build_adjacency(solid)?;
    for edge in remus_topology::explorer::solid_edges(topo, solid)? {
        let mut faces = adjacency.faces_for_edge(edge).to_vec();
        faces.sort_unstable_by_key(|face| face.index());
        faces.dedup();
        if faces.len() != 2 {
            return Err(reconstruction(format!(
                "sphere termination result edge {} has {} incident faces",
                edge.index(),
                faces.len()
            )));
        }
    }
    Ok(())
}

/// Heal one cylindrical blend strip between two planes when one end is
/// planar and the other terminates on a sphere.
///
/// The sharp support line is rebuilt between an exact three-plane corner and
/// the locally unique line/sphere root.  The two existing plane/sphere circle
/// carriers are extended to that root, preserving every unrelated entity.
/// Configurations outside this verified topology return `Ok(None)` so other
/// exact healers may classify them.  No sampled 3D curve is ever created.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn heal_cylinder_plane_band_sphere_end(
    topo: &mut Topology,
    solid: SolidId,
    band_face: FaceId,
    supports: [FaceId; 2],
) -> Result<Option<DefeatureOutcome>, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        heal_cylinder_plane_band_sphere_end_impl(topo, solid, band_face, supports)
    })
}

fn heal_cylinder_plane_band_sphere_end_impl(
    topo: &mut Topology,
    solid: SolidId,
    band_face: FaceId,
    supports: [FaceId; 2],
) -> Result<Option<DefeatureOutcome>, OperationsError> {
    let Some(plan) = classify_sphere_end(topo, solid, band_face, supports)? else {
        return Ok(None);
    };
    let copied = crate::copy::copy_solid_with_entity_map(topo, solid)?;
    let copied_face = |source: FaceId| {
        copied
            .face_map
            .get(&source.index())
            .copied()
            .ok_or_else(|| reconstruction(format!("face {} was not copied", source.index())))
    };
    let copied_edge = |source: EdgeId| {
        copied
            .edge_map
            .get(&source.index())
            .copied()
            .ok_or_else(|| reconstruction(format!("edge {} was not copied", source.index())))
    };
    let copied_vertex = |source: VertexId| {
        copied
            .vertex_map
            .get(&source.index())
            .copied()
            .ok_or_else(|| reconstruction(format!("vertex {} was not copied", source.index())))
    };
    let band = copied_face(plan.band)?;
    let supports = [
        copied_face(plan.supports[0])?,
        copied_face(plan.supports[1])?,
    ];
    let planar_end = copied_face(plan.planar_end)?;
    let sphere_end = copied_face(plan.sphere_end)?;
    let springs = [copied_edge(plan.springs[0])?, copied_edge(plan.springs[1])?];
    let crosses = [
        copied_edge(plan.planar_cross)?,
        copied_edge(plan.sphere_cross)?,
    ];
    let planar_boundaries = [
        copied_edge(plan.planar_boundaries[0])?,
        copied_edge(plan.planar_boundaries[1])?,
    ];
    let sphere_boundaries = [
        copied_edge(plan.sphere_boundaries[0])?,
        copied_edge(plan.sphere_boundaries[1])?,
    ];
    let planar_vertices = [
        copied_vertex(plan.planar_vertices[0])?,
        copied_vertex(plan.planar_vertices[1])?,
    ];
    let sphere_vertices = [
        copied_vertex(plan.sphere_vertices[0])?,
        copied_vertex(plan.sphere_vertices[1])?,
    ];

    let planar_tolerance = planar_vertices
        .iter()
        .map(|vertex| {
            topo.vertex(*vertex)
                .map(remus_topology::vertex::Vertex::tolerance)
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .fold(Tolerance::new().linear, f64::max);
    let sphere_tolerance = sphere_vertices
        .iter()
        .map(|vertex| {
            topo.vertex(*vertex)
                .map(remus_topology::vertex::Vertex::tolerance)
        })
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .fold(Tolerance::new().linear, f64::max);
    let planar_corner = topo.add_vertex(Vertex::new(plan.planar_corner, planar_tolerance));
    let sphere_corner = topo.add_vertex(Vertex::new(plan.sphere_corner, sphere_tolerance));
    let sharp = topo.add_edge(Edge::new(planar_corner, sphere_corner, EdgeCurve::Line));

    let mut replacements = BTreeMap::new();
    for index in 0..2 {
        replacements.insert(
            planar_boundaries[index],
            replacement_edge(
                topo,
                planar_boundaries[index],
                planar_vertices[index],
                planar_corner,
            )?,
        );
        replacements.insert(
            sphere_boundaries[index],
            replacement_edge(
                topo,
                sphere_boundaries[index],
                sphere_vertices[index],
                sphere_corner,
            )?,
        );
    }
    for face in [supports[0], supports[1], planar_end, sphere_end] {
        splice_face(
            topo,
            face,
            &springs,
            &crosses,
            &replacements,
            &planar_vertices,
            &sphere_vertices,
            planar_corner,
            sphere_corner,
            sharp,
        )?;
    }

    register_pcurve(topo, sharp, supports[0])?;
    register_pcurve(topo, sharp, supports[1])?;
    for index in 0..2 {
        let planar = replacements[&planar_boundaries[index]];
        register_pcurve(topo, planar, supports[index])?;
        register_pcurve(topo, planar, planar_end)?;
        let spherical = replacements[&sphere_boundaries[index]];
        register_pcurve(topo, spherical, supports[index])?;
        register_pcurve(topo, spherical, sphere_end)?;
    }

    let old_shell = topo.solid(copied.solid)?.outer_shell();
    let kept_faces: Vec<_> = topo
        .shell(old_shell)?
        .faces()
        .iter()
        .copied()
        .filter(|face| *face != band)
        .collect();
    let shell = topo.add_shell(Shell::new(kept_faces)?);
    let healed = topo.add_solid(Solid::new(shell, Vec::new()));
    certify_result(topo, healed)?;

    let mut face_map = copied.face_map;
    face_map.remove(&plan.band.index());
    let replacement_by_source: BTreeMap<usize, EdgeId> = plan
        .planar_boundaries
        .iter()
        .zip(planar_boundaries)
        .chain(plan.sphere_boundaries.iter().zip(sphere_boundaries))
        .map(|(source, copied_source)| (source.index(), replacements[&copied_source]))
        .collect();
    let spring_sources: BTreeSet<_> = plan.springs.iter().map(|edge| edge.index()).collect();
    let cross_sources: BTreeSet<_> = [plan.planar_cross.index(), plan.sphere_cross.index()]
        .into_iter()
        .collect();
    let live_edges: BTreeSet<_> = remus_topology::explorer::solid_edges(topo, healed)?
        .into_iter()
        .collect();
    let mut copied_edges: Vec<_> = copied.edge_map.into_iter().collect();
    copied_edges.sort_unstable_by_key(|(source, _)| *source);
    let mut boundary_history = Vec::new();
    for (source, copied_edge) in copied_edges {
        let target = if spring_sources.contains(&source) {
            Some(sharp)
        } else if cross_sources.contains(&source) {
            None
        } else if let Some(&replacement) = replacement_by_source.get(&source) {
            Some(replacement)
        } else if live_edges.contains(&copied_edge) {
            Some(copied_edge)
        } else {
            return Err(reconstruction(format!(
                "unplanned disappearance of copied edge {source}"
            )));
        };
        boundary_history.push((
            EntityKey::edge(source),
            target.map(|edge| EntityKey::edge(edge.index())),
        ));
    }
    let planar_source_vertices: BTreeSet<_> = plan
        .planar_vertices
        .iter()
        .map(|vertex| vertex.index())
        .collect();
    let sphere_source_vertices: BTreeSet<_> = plan
        .sphere_vertices
        .iter()
        .map(|vertex| vertex.index())
        .collect();
    let live_vertices: BTreeSet<_> = remus_topology::explorer::solid_vertices(topo, healed)?
        .into_iter()
        .collect();
    let mut copied_vertices: Vec<_> = copied.vertex_map.into_iter().collect();
    copied_vertices.sort_unstable_by_key(|(source, _)| *source);
    for (source, copied_vertex) in copied_vertices {
        let target = if planar_source_vertices.contains(&source) {
            Some(planar_corner)
        } else if sphere_source_vertices.contains(&source) {
            Some(sphere_corner)
        } else if live_vertices.contains(&copied_vertex) {
            Some(copied_vertex)
        } else {
            return Err(reconstruction(format!(
                "unplanned disappearance of copied vertex {source}"
            )));
        };
        boundary_history.push((
            EntityKey::vertex(source),
            target.map(|vertex| EntityKey::vertex(vertex.index())),
        ));
    }
    let history_targets: BTreeSet<_> = boundary_history
        .iter()
        .filter_map(|(_, target)| *target)
        .collect();
    let live_targets: BTreeSet<_> = live_edges
        .iter()
        .map(|edge| EntityKey::edge(edge.index()))
        .chain(
            live_vertices
                .iter()
                .map(|vertex| EntityKey::vertex(vertex.index())),
        )
        .collect();
    if history_targets != live_targets {
        return Err(reconstruction(
            "sphere termination boundary history does not cover the exact result census",
        ));
    }
    Ok(Some(DefeatureOutcome {
        solid: healed,
        face_map,
        boundary_history: Some(boundary_history),
    }))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
    use remus_topology::face::Face;

    use super::*;

    struct Fixture {
        topo: Topology,
        solid: SolidId,
        band: FaceId,
        supports: [FaceId; 2],
        sphere_boundaries: [EdgeId; 2],
    }

    fn vertex(topo: &mut Topology, x: f64, y: f64, z: f64) -> VertexId {
        topo.add_vertex(Vertex::new(Point3::new(x, y, z), Tolerance::new().linear))
    }

    fn line(topo: &mut Topology, start: VertexId, end: VertexId) -> EdgeId {
        topo.add_edge(Edge::new(start, end, EdgeCurve::Line))
    }

    fn minor_circle(
        topo: &mut Topology,
        start: VertexId,
        end: VertexId,
        center: Point3,
        mut normal: Vec3,
        radius: f64,
    ) -> EdgeId {
        let start_point = topo.vertex(start).unwrap().point();
        let end_point = topo.vertex(end).unwrap().point();
        let build =
            |normal| Circle3D::new_with_ref(center, normal, radius, start_point - center).unwrap();
        let mut circle = build(normal);
        let mut end_parameter = circle.project(end_point).rem_euclid(std::f64::consts::TAU);
        if end_parameter > std::f64::consts::PI {
            normal = normal * -1.0;
            circle = build(normal);
            end_parameter = circle.project(end_point).rem_euclid(std::f64::consts::TAU);
        }
        assert!(end_parameter > 1e-9 && end_parameter <= std::f64::consts::PI + 1e-9);
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, end_parameter)));
        topo.add_edge(edge)
    }

    fn face(topo: &mut Topology, edges: Vec<OrientedEdge>, surface: FaceSurface) -> FaceId {
        let wire = topo.add_wire(Wire::new(edges, true).unwrap());
        topo.add_face(Face::new(wire, vec![], surface))
    }

    /// Exact rounded-column body.  The top is a spherical patch; its boundary
    /// uses the five exact plane/sphere or cylinder/sphere circles.  No boolean
    /// or fitted 3D curve participates in the fixture.
    fn sphere_ended_band_fixture(scale: f64) -> Fixture {
        let (r, l, h, sphere_radius) = (2.0 * scale, 4.0 * scale, 3.0 * scale, 8.0 * scale);
        let z_corner = (sphere_radius * sphere_radius - 2.0 * l * l).sqrt();
        let z_side = (sphere_radius * sphere_radius - r * r - l * l).sqrt();
        let z_band = (sphere_radius * sphere_radius - r * r).sqrt();
        let mut topo = Topology::new();

        let b0 = vertex(&mut topo, -l, -l, -h);
        let b1 = vertex(&mut topo, r, -l, -h);
        let b2 = vertex(&mut topo, r, 0.0, -h);
        let b3 = vertex(&mut topo, 0.0, r, -h);
        let b4 = vertex(&mut topo, -l, r, -h);
        let t0 = vertex(&mut topo, -l, -l, z_corner);
        let t1 = vertex(&mut topo, r, -l, z_side);
        let t2 = vertex(&mut topo, r, 0.0, z_band);
        let t3 = vertex(&mut topo, 0.0, r, z_band);
        let t4 = vertex(&mut topo, -l, r, z_side);

        let b01 = line(&mut topo, b0, b1);
        let b12 = line(&mut topo, b1, b2);
        let b23 = minor_circle(
            &mut topo,
            b2,
            b3,
            Point3::new(0.0, 0.0, -h),
            Vec3::new(0.0, 0.0, 1.0),
            r,
        );
        let b34 = line(&mut topo, b3, b4);
        let b40 = line(&mut topo, b4, b0);

        let t01 = minor_circle(
            &mut topo,
            t0,
            t1,
            Point3::new(0.0, -l, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            (sphere_radius * sphere_radius - l * l).sqrt(),
        );
        let t12 = minor_circle(
            &mut topo,
            t1,
            t2,
            Point3::new(r, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            (sphere_radius * sphere_radius - r * r).sqrt(),
        );
        let t23 = minor_circle(
            &mut topo,
            t2,
            t3,
            Point3::new(0.0, 0.0, z_band),
            Vec3::new(0.0, 0.0, 1.0),
            r,
        );
        let t34 = minor_circle(
            &mut topo,
            t3,
            t4,
            Point3::new(0.0, r, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            (sphere_radius * sphere_radius - r * r).sqrt(),
        );
        let t40 = minor_circle(
            &mut topo,
            t4,
            t0,
            Point3::new(-l, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            (sphere_radius * sphere_radius - l * l).sqrt(),
        );

        let v0 = line(&mut topo, b0, t0);
        let v1 = line(&mut topo, b1, t1);
        let v2 = line(&mut topo, b2, t2);
        let v3 = line(&mut topo, b3, t3);
        let v4 = line(&mut topo, b4, t4);

        let bottom = face(
            &mut topo,
            vec![
                OrientedEdge::new(b01, true),
                OrientedEdge::new(b12, true),
                OrientedEdge::new(b23, true),
                OrientedEdge::new(b34, true),
                OrientedEdge::new(b40, true),
            ],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, -1.0),
                d: h,
            },
        );
        let south = face(
            &mut topo,
            vec![
                OrientedEdge::new(b01, false),
                OrientedEdge::new(v0, true),
                OrientedEdge::new(t01, true),
                OrientedEdge::new(v1, false),
            ],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, -1.0, 0.0),
                d: l,
            },
        );
        let support_x = face(
            &mut topo,
            vec![
                OrientedEdge::new(b12, false),
                OrientedEdge::new(v1, true),
                OrientedEdge::new(t12, true),
                OrientedEdge::new(v2, false),
            ],
            FaceSurface::Plane {
                normal: Vec3::new(1.0, 0.0, 0.0),
                d: r,
            },
        );
        let cylinder =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), r)
                .unwrap();
        let band = face(
            &mut topo,
            vec![
                OrientedEdge::new(b23, false),
                OrientedEdge::new(v2, true),
                OrientedEdge::new(t23, true),
                OrientedEdge::new(v3, false),
            ],
            FaceSurface::Cylinder(cylinder),
        );
        let support_y = face(
            &mut topo,
            vec![
                OrientedEdge::new(b34, false),
                OrientedEdge::new(v3, true),
                OrientedEdge::new(t34, true),
                OrientedEdge::new(v4, false),
            ],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: r,
            },
        );
        let west = face(
            &mut topo,
            vec![
                OrientedEdge::new(b40, false),
                OrientedEdge::new(v4, true),
                OrientedEdge::new(t40, true),
                OrientedEdge::new(v0, false),
            ],
            FaceSurface::Plane {
                normal: Vec3::new(-1.0, 0.0, 0.0),
                d: l,
            },
        );
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), sphere_radius).unwrap();
        let sphere_face = face(
            &mut topo,
            vec![
                OrientedEdge::new(t01, false),
                OrientedEdge::new(t40, false),
                OrientedEdge::new(t34, false),
                OrientedEdge::new(t23, false),
                OrientedEdge::new(t12, false),
            ],
            FaceSurface::Sphere(sphere),
        );
        let shell = topo.add_shell(
            Shell::new(vec![
                bottom,
                south,
                support_x,
                band,
                support_y,
                west,
                sphere_face,
            ])
            .unwrap(),
        );
        let solid = topo.add_solid(Solid::new(shell, vec![]));

        // Give the two changed analytic boundaries real per-use p-curve
        // authority.  The healer must replace it with validated p-curves, not
        // accidentally inherit stale authority from the copied coedges.
        for (edge, support) in [(t12, support_x), (t34, support_y)] {
            register_pcurve(&mut topo, edge, support).unwrap();
        }
        Fixture {
            topo,
            solid,
            band,
            supports: [support_x, support_y],
            sphere_boundaries: [t12, t34],
        }
    }

    fn assert_valid(topo: &Topology, solid: SolidId) {
        let report = crate::validate::validate_solid(topo, solid).unwrap();
        assert!(report.is_valid(), "validation errors: {:?}", report.issues);
    }

    fn assert_manifold_vertex_disks(topo: &Topology, solid: SolidId) {
        let adjacency = topo.build_adjacency(solid).unwrap();
        let edges = remus_topology::explorer::solid_edges(topo, solid).unwrap();
        for vertex in remus_topology::explorer::solid_vertices(topo, solid).unwrap() {
            let incident: Vec<_> = edges
                .iter()
                .copied()
                .filter(|edge| {
                    let edge = topo.edge(*edge).unwrap();
                    edge.start() == vertex || edge.end() == vertex
                })
                .collect();
            let faces: BTreeSet<_> = incident
                .iter()
                .flat_map(|edge| adjacency.faces_for_edge(*edge).iter().copied())
                .collect();
            assert_eq!(
                incident.len(),
                faces.len(),
                "vertex {} does not have one closed manifold disk link",
                vertex.index()
            );
        }
    }

    /// Independent material-volume oracle for the healed body.  It integrates
    /// the spherical height above the exact sharp square footprint, rather
    /// than reusing B-rep faces, tessellation, or the operations volume path.
    fn sharp_volume_oracle(scale: f64) -> f64 {
        const N: usize = 400;
        let (lo, hi, bottom, radius) = (-4.0 * scale, 2.0 * scale, 3.0 * scale, 8.0 * scale);
        let step = (hi - lo) / N as f64;
        let mut sum = 0.0;
        for ix in 0..=N {
            let x = lo + ix as f64 * step;
            let wx = if ix == 0 || ix == N {
                1.0
            } else if ix % 2 == 0 {
                2.0
            } else {
                4.0
            };
            for iy in 0..=N {
                let y = lo + iy as f64 * step;
                let wy = if iy == 0 || iy == N {
                    1.0
                } else if iy % 2 == 0 {
                    2.0
                } else {
                    4.0
                };
                sum += wx * wy * ((radius * radius - x * x - y * y).sqrt() + bottom);
            }
        }
        sum * step * step / 9.0
    }

    #[test]
    fn exact_sphere_termination_reconstructs_sharp_line_and_circle_extensions() {
        for scale in [0.25, 1.0, 7.0] {
            let Fixture {
                mut topo,
                solid,
                band,
                supports,
                ..
            } = sphere_ended_band_fixture(scale);
            assert_valid(&topo, solid);
            let before_faces = remus_topology::explorer::solid_faces(&topo, solid)
                .unwrap()
                .len();
            let outcome = heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
                .unwrap_or_else(|error| panic!("scale {scale}: {error}"))
                .expect("qualified sphere end");
            assert_valid(&topo, outcome.solid);
            assert_manifold_vertex_disks(&topo, outcome.solid);
            assert_eq!(
                remus_topology::explorer::solid_faces(&topo, outcome.solid)
                    .unwrap()
                    .len(),
                before_faces - 1
            );
            let sharp_corner =
                Point3::new(2.0 * scale, 2.0 * scale, (64.0_f64 - 8.0).sqrt() * scale);
            let mut sharp_hits = 0;
            let mut extended_circles = 0;
            for edge in remus_topology::explorer::solid_edges(&topo, outcome.solid).unwrap() {
                let data = topo.edge(edge).unwrap();
                let endpoints = [
                    topo.vertex(data.start()).unwrap().point(),
                    topo.vertex(data.end()).unwrap().point(),
                ];
                if matches!(data.curve(), EdgeCurve::Line)
                    && endpoints
                        .iter()
                        .any(|point| (*point - sharp_corner).length() < 1e-8 * scale.max(1.0))
                {
                    sharp_hits += 1;
                }
                if matches!(data.curve(), EdgeCurve::Circle(_))
                    && endpoints
                        .iter()
                        .any(|point| (*point - sharp_corner).length() < 1e-8 * scale.max(1.0))
                {
                    extended_circles += 1;
                    assert!(data.strict_domain().is_ok());
                }
            }
            assert_eq!(
                sharp_hits, 1,
                "one support/support sharp edge reaches sphere"
            );
            assert_eq!(extended_circles, 2, "two exact plane/sphere circles");
            let measured = crate::measure::solid_volume(&topo, outcome.solid, 0.00001 * scale)
                .unwrap()
                .abs();
            let oracle = sharp_volume_oracle(scale);
            assert!(
                (measured - oracle).abs() <= oracle * 2e-6,
                "independent sharp-volume oracle: measured={measured:.12}, oracle={oracle:.12}"
            );
        }
    }

    #[test]
    fn public_resize_blend_removes_exact_sphere_ended_band() {
        let Fixture {
            mut topo,
            solid,
            band,
            ..
        } = sphere_ended_band_fixture(1.0);
        let before_faces = remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len();
        let result = crate::resize_blend::resize_blend(&mut topo, solid, band, 2.0, 0.0)
            .expect("public resize removes the qualified sphere-ended band");
        assert_valid(&topo, result.solid);
        assert!(result.evolution.is_complete());
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, result.solid)
                .unwrap()
                .len(),
            before_faces - 1
        );
    }

    #[test]
    fn journaled_resize_records_total_sphere_termination_history() {
        let Fixture {
            mut topo,
            solid,
            band,
            ..
        } = sphere_ended_band_fixture(1.0);
        let source_boundaries = crate::journal_ops::solid_entity_keys(&topo, solid).unwrap();
        let result = crate::journal_ops::resize_blend_journaled(&mut topo, solid, band, 2.0, 0.0)
            .expect("journaled public resize removes the qualified sphere-ended band");
        assert_valid(&topo, result.solid);
        assert!(result.map.is_complete());
        assert!(result.map.deleted.contains(&band.index()));
        assert_eq!(
            topo.journal().entries().len(),
            1,
            "one atomic public journal entry"
        );
        let target_boundaries = crate::journal_ops::solid_entity_keys(&topo, result.solid).unwrap();
        assert!(source_boundaries.len() > target_boundaries.len());
    }

    #[test]
    fn sphere_termination_is_rigid_motion_and_chart_seam_invariant() {
        let Fixture {
            mut topo,
            solid,
            band,
            supports,
            ..
        } = sphere_ended_band_fixture(1.0);
        let transform = remus_math::mat::Mat4::translation(17.0, -23.0, 31.0)
            * remus_math::mat::Mat4::rotation_x(0.71)
            * remus_math::mat::Mat4::rotation_z(-2.4);
        crate::transform::transform_solid(&mut topo, solid, &transform).unwrap();
        assert_valid(&topo, solid);
        let outcome = heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
            .unwrap()
            .expect("rigidly placed sphere termination remains qualified");
        assert_valid(&topo, outcome.solid);
        for edge in remus_topology::explorer::solid_edges(&topo, outcome.solid).unwrap() {
            for (face, forward, _) in topo.pcurves_for_edge(edge) {
                remus_topology::validation::validate_same_parameter(
                    &topo,
                    edge,
                    face,
                    forward,
                    Tolerance::new().linear,
                    1027,
                )
                .unwrap();
            }
        }
    }

    #[test]
    fn sphere_root_proof_excludes_the_disconnected_far_branch() {
        let fixture = sphere_ended_band_fixture(1.0);
        let plan =
            classify_sphere_end(&fixture.topo, fixture.solid, fixture.band, fixture.supports)
                .unwrap()
                .expect("qualified sphere end");
        let expected = (64.0_f64 - 8.0).sqrt();
        assert!((plan.sphere_corner.z() - expected).abs() < 1e-12);
        let far = Point3::new(2.0, 2.0, -expected);
        let planar = fixture
            .topo
            .vertex(plan.planar_vertices[0])
            .unwrap()
            .point();
        let terminal = fixture
            .topo
            .vertex(plan.sphere_vertices[0])
            .unwrap()
            .point();
        let wound_direction = (terminal - planar).normalize().unwrap();
        let local_trim = locally_extends_circle_edge(
            &fixture.topo,
            plan.sphere_boundaries[0],
            plan.sphere_vertices[0],
            far,
        )
        .unwrap();
        let advances_from_cap =
            (far - plan.planar_corner).dot(wound_direction) > Tolerance::new().linear;
        assert!(
            !(local_trim && advances_from_cap),
            "far root must fail the combined local-trim and cap-direction proof"
        );
        assert!(!advances_from_cap, "far root lies behind the planar cap");
    }

    #[test]
    fn sphere_termination_history_pcurves_and_step_round_trip_are_total() {
        let Fixture {
            mut topo,
            solid,
            band,
            supports,
            sphere_boundaries,
        } = sphere_ended_band_fixture(1.0);
        let source_edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
        let source_vertices = remus_topology::explorer::solid_vertices(&topo, solid).unwrap();
        for (edge, support) in sphere_boundaries.into_iter().zip(supports) {
            assert!(topo.pcurve_oriented(edge, support, true).is_some());
        }
        let outcome = heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
            .unwrap()
            .expect("qualified sphere end");
        let history = outcome.boundary_history.as_ref().expect("boundary history");
        let sources: BTreeSet<_> = history.iter().map(|(source, _)| *source).collect();
        assert_eq!(sources.len(), source_edges.len() + source_vertices.len());
        let targets: BTreeSet<_> = history.iter().filter_map(|(_, target)| *target).collect();
        let expected_targets: BTreeSet<_> =
            remus_topology::explorer::solid_edges(&topo, outcome.solid)
                .unwrap()
                .into_iter()
                .map(|edge| EntityKey::edge(edge.index()))
                .chain(
                    remus_topology::explorer::solid_vertices(&topo, outcome.solid)
                        .unwrap()
                        .into_iter()
                        .map(|vertex| EntityKey::vertex(vertex.index())),
                )
                .collect();
        assert_eq!(
            targets, expected_targets,
            "history covers every result boundary"
        );
        let result_pcurves: usize = remus_topology::explorer::solid_faces(&topo, outcome.solid)
            .unwrap()
            .into_iter()
            .map(|face| topo.pcurves_for_face(face).len())
            .sum();
        assert_eq!(
            result_pcurves, 10,
            "every one of the ten reconstructed coedge uses carries a p-curve"
        );

        let volume = crate::measure::solid_volume(&topo, outcome.solid, 0.01).unwrap();
        assert!(volume.is_finite() && volume.abs() > 1.0);
        let step = remus_io::step::write_step(&topo, &[outcome.solid]).unwrap();
        let mut reread = Topology::new();
        let reread_solid = remus_io::step::read_step(&step, &mut reread).unwrap()[0];
        assert_valid(&reread, reread_solid);
        let reread_circles = remus_topology::explorer::solid_edges(&reread, reread_solid)
            .unwrap()
            .into_iter()
            .filter(|edge| matches!(reread.edge(*edge).unwrap().curve(), EdgeCurve::Circle(_)))
            .count();
        assert!(
            reread_circles >= 3,
            "STEP round trip preserves exact circular carriers"
        );
        let reread_volume = crate::measure::solid_volume(&reread, reread_solid, 0.01).unwrap();
        assert!((reread_volume - volume).abs() < 1e-6 * volume.abs().max(1.0));
    }

    #[test]
    fn non_circle_sphere_boundary_refuses_without_mutation() {
        let Fixture {
            mut topo,
            solid,
            band,
            supports,
            sphere_boundaries,
        } = sphere_ended_band_fixture(1.0);
        let before = topo.clone();
        topo.edge_mut(sphere_boundaries[0])
            .unwrap()
            .set_curve(EdgeCurve::Line);
        let tampered = topo.clone();
        let result = heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports).unwrap();
        assert!(result.is_none());
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap(),
            remus_topology::explorer::solid_entity_counts(&tampered, solid).unwrap()
        );
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&before, solid).unwrap(),
            remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap()
        );

        let before_public = topo.clone();
        let error = crate::resize_blend::resize_blend(&mut topo, solid, band, 2.0, 0.0)
            .expect_err("public resize must refuse an uncertified sphere boundary");
        assert_eq!(
            crate::resize_blend::resize_blend_failure_code(&error),
            "resize-blend-failed"
        );
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&before_public, solid).unwrap(),
            remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap(),
            "public transaction rolls back every attempted reconstruction"
        );
    }
}
