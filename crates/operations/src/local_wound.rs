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
        (
            FaceSurface::Plane { .. },
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_),
        )
        | (
            FaceSurface::Sphere(_),
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
        )
        | (
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_),
            FaceSurface::Plane { .. }
            | FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_),
        ) => return Ok(None),
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
        (
            FaceSurface::Plane { .. },
            EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        )
        | (
            FaceSurface::Sphere(_),
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        )
        | (
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_),
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_),
        ) => {
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
        normal: Vec3,
        radius: f64,
    ) -> EdgeId {
        minor_circle_with_trim(
            topo,
            start,
            end,
            center,
            normal,
            radius,
            TrimLayout::default(),
        )
    }

    /// How a fixture circle stores its minor-arc trim.  The geometry is the
    /// same exact arc in every layout; only the parameter bookkeeping moves.
    #[derive(Clone, Copy, Default)]
    struct TrimLayout {
        /// Parameter of the start vertex (the reference direction is rotated
        /// so the start lands here instead of at zero).
        offset: f64,
        /// Store the arc on the opposite normal so its parameter decreases
        /// from start to end (a negative-span trim).
        decreasing: bool,
    }

    fn minor_circle_with_trim(
        topo: &mut Topology,
        start: VertexId,
        end: VertexId,
        center: Point3,
        mut normal: Vec3,
        radius: f64,
        layout: TrimLayout,
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
        let sweep = if layout.decreasing {
            circle = build(normal * -1.0);
            -end_parameter
        } else {
            end_parameter
        };
        // Rotate the reference direction by -offset: the start vertex then
        // evaluates at `offset` and the end at `offset + sweep`.
        let (sin, cos) = layout.offset.sin_cos();
        let reference = circle.u_axis() * cos - circle.v_axis() * sin;
        let circle = Circle3D::new_with_ref(center, circle.normal(), radius, reference).unwrap();
        let trim = (layout.offset, layout.offset + sweep);
        assert!((circle.evaluate(trim.0) - start_point).length() < 1e-12 * radius.max(1.0));
        assert!((circle.evaluate(trim.1) - end_point).length() < 1e-12 * radius.max(1.0));
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some(trim));
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
        sphere_ended_band_fixture_with(&FixtureOptions {
            scale,
            ..FixtureOptions::default()
        })
    }

    /// Variations of the same exact body.  Every option changes only how the
    /// identical solid is stored or how tall its planar column is, so each
    /// variant has the same closed-form sharp corner and volume integrand.
    #[derive(Clone, Copy)]
    struct FixtureOptions {
        scale: f64,
        /// Depth of the planar bottom cap below the sphere centre, in units
        /// of `scale` (the original fixture uses 3).
        depth: f64,
        /// Spherical chart axis and longitude reference; `None` is the
        /// world frame.
        sphere_frame: Option<(Vec3, Vec3)>,
        /// Store the two changed plane/sphere circles from the band terminal
        /// outward instead of toward it (the opposite `replace_start` arm).
        reverse_sphere_boundaries: bool,
        /// Parameter bookkeeping of those two circles.
        sphere_boundary_trim: TrimLayout,
        /// Multiply every stored plane equation by this factor.
        plane_storage_scale: f64,
    }

    impl Default for FixtureOptions {
        fn default() -> Self {
            Self {
                scale: 1.0,
                depth: 3.0,
                sphere_frame: None,
                reverse_sphere_boundaries: false,
                sphere_boundary_trim: TrimLayout::default(),
                plane_storage_scale: 1.0,
            }
        }
    }

    fn sphere_ended_band_fixture_with(options: &FixtureOptions) -> Fixture {
        let scale = options.scale;
        let (r, l, h, sphere_radius) =
            (2.0 * scale, 4.0 * scale, options.depth * scale, 8.0 * scale);
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
        let reversed = options.reverse_sphere_boundaries;
        let (t12_start, t12_end) = if reversed { (t2, t1) } else { (t1, t2) };
        let t12 = minor_circle_with_trim(
            &mut topo,
            t12_start,
            t12_end,
            Point3::new(r, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            (sphere_radius * sphere_radius - r * r).sqrt(),
            options.sphere_boundary_trim,
        );
        let t23 = minor_circle(
            &mut topo,
            t2,
            t3,
            Point3::new(0.0, 0.0, z_band),
            Vec3::new(0.0, 0.0, 1.0),
            r,
        );
        let (t34_start, t34_end) = if reversed { (t4, t3) } else { (t3, t4) };
        let t34 = minor_circle_with_trim(
            &mut topo,
            t34_start,
            t34_end,
            Point3::new(0.0, r, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            (sphere_radius * sphere_radius - r * r).sqrt(),
            options.sphere_boundary_trim,
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
                OrientedEdge::new(t12, !reversed),
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
                OrientedEdge::new(t34, !reversed),
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
        let sphere = match options.sphere_frame {
            None => SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), sphere_radius),
            Some((axis, reference)) => SphericalSurface::with_frame(
                Point3::new(0.0, 0.0, 0.0),
                sphere_radius,
                axis,
                reference,
            ),
        }
        .unwrap();
        let sphere_face = face(
            &mut topo,
            vec![
                OrientedEdge::new(t01, false),
                OrientedEdge::new(t40, false),
                OrientedEdge::new(t34, reversed),
                OrientedEdge::new(t23, false),
                OrientedEdge::new(t12, reversed),
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

        // Multiplying by the default factor 1.0 is exact, so the original
        // fixture is unchanged.
        for plane_face in [bottom, south, support_x, support_y, west] {
            let FaceSurface::Plane { normal, d } = *topo.face(plane_face).unwrap().surface() else {
                unreachable!("fixture plane");
            };
            topo.face_mut(plane_face)
                .unwrap()
                .set_surface(FaceSurface::Plane {
                    normal: normal * options.plane_storage_scale,
                    d: d * options.plane_storage_scale,
                });
        }

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
        sharp_volume_oracle_with_depth(scale, 3.0)
    }

    fn sharp_volume_oracle_with_depth(scale: f64, depth: f64) -> f64 {
        const N: usize = 400;
        let (lo, hi, bottom, radius) = (-4.0 * scale, 2.0 * scale, depth * scale, 8.0 * scale);
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

    // ---- B19 survivor tranche: independent oracles for the sphere-end
    // certificates.  Every expected value below is derived from the fixture's
    // closed-form geometry (plane x = r, plane y = r, sphere |p| = R), never
    // from a previous run of the code under test.

    const R_SPHERE: f64 = 8.0;

    fn sphere_corner(scale: f64) -> Point3 {
        Point3::new(2.0 * scale, 2.0 * scale, (64.0_f64 - 8.0).sqrt() * scale)
    }

    fn unit(normal: Vec3, d: f64) -> UnitPlane {
        UnitPlane { normal, d }
    }

    fn plane_residual(plane: UnitPlane, point: Point3) -> f64 {
        plane.normal.dot(Vec3::new(point.x(), point.y(), point.z())) - plane.d
    }

    /// Heal the given fixture variant and check it against the closed-form
    /// sharp body: one support/support line and two plane/sphere circles meet
    /// the exact root, each circle keeps its far vertex and grows by exactly
    /// the angle from the old terminal to the root on its own carrier.
    fn assert_heals_to_sharp_corner(options: &FixtureOptions) -> (Topology, SolidId) {
        let Fixture {
            mut topo,
            solid,
            band,
            supports,
            sphere_boundaries,
        } = sphere_ended_band_fixture_with(options);
        assert_valid(&topo, solid);
        let scale = options.scale;
        let corner = sphere_corner(scale);
        let mut expected_spans = Vec::new();
        for (edge, terminal_y) in sphere_boundaries.into_iter().zip([0.0, 2.0 * scale]) {
            let data = topo.edge(edge).unwrap();
            let EdgeCurve::Circle(circle) = data.curve() else {
                panic!("fixture sphere boundary is a circle");
            };
            let (t0, t1) = data.strict_domain().unwrap();
            // The old terminal is the band vertex on this circle: (r, 0, zb)
            // on x = r, or (0, r, zb) on y = r.
            let zb = (60.0_f64).sqrt() * scale;
            let terminal = if terminal_y == 0.0 {
                Point3::new(2.0 * scale, 0.0, zb)
            } else {
                Point3::new(0.0, 2.0 * scale, zb)
            };
            let a = (terminal - circle.center()).normalize().unwrap();
            let b = (corner - circle.center()).normalize().unwrap();
            let growth = a.dot(b).clamp(-1.0, 1.0).acos();
            expected_spans.push(((t1 - t0).abs() + growth, circle.center()));
        }
        let before_faces = remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len();
        let outcome = heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
            .unwrap_or_else(|error| panic!("heal failed: {error}"))
            .expect("qualified sphere end");
        assert_valid(&topo, outcome.solid);
        assert_manifold_vertex_disks(&topo, outcome.solid);
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, outcome.solid)
                .unwrap()
                .len(),
            before_faces - 1
        );
        let near = |point: Point3| (point - corner).length() < 1e-9 * scale.max(1.0);
        let mut lines = 0;
        let mut spans = Vec::new();
        for edge in remus_topology::explorer::solid_edges(&topo, outcome.solid).unwrap() {
            let data = topo.edge(edge).unwrap();
            let start = topo.vertex(data.start()).unwrap().point();
            let end = topo.vertex(data.end()).unwrap().point();
            if !(near(start) || near(end)) {
                continue;
            }
            match data.curve() {
                EdgeCurve::Line => lines += 1,
                EdgeCurve::Circle(circle) => {
                    let (t0, t1) = data.strict_domain().unwrap();
                    assert!((circle.evaluate(t0) - start).length() < 1e-9 * scale.max(1.0));
                    assert!((circle.evaluate(t1) - end).length() < 1e-9 * scale.max(1.0));
                    spans.push(((t1 - t0).abs(), circle.center()));
                }
                other => panic!("unexpected {} at the sharp corner", other.type_tag()),
            }
        }
        assert_eq!(lines, 1, "one support/support sharp line reaches the root");
        assert_eq!(spans.len(), 2, "two extended plane/sphere circles");
        for (expected, center) in expected_spans {
            let (actual, _) = spans
                .iter()
                .copied()
                .find(|(_, c)| (*c - center).length() < 1e-12 * scale.max(1.0))
                .expect("extended circle keeps its exact carrier");
            assert!(
                (actual - expected).abs() < 1e-9,
                "extended span {actual} != old span + exact growth {expected}"
            );
        }
        (topo, outcome.solid)
    }

    #[test]
    fn unit_plane_normalizes_the_stored_plane_equation() {
        let mut topo = Topology::new();
        let a = vertex(&mut topo, 0.0, 0.0, 3.0);
        let b = vertex(&mut topo, 1.0, 0.0, 3.0);
        let c = vertex(&mut topo, 0.0, 1.0, 3.0);
        let edges = [
            line(&mut topo, a, b),
            line(&mut topo, b, c),
            line(&mut topo, c, a),
        ];
        let plane = face(
            &mut topo,
            edges
                .iter()
                .map(|edge| OrientedEdge::new(*edge, true))
                .collect(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 2.5),
                d: 7.5,
            },
        );
        let unit = unit_plane(&topo, plane).unwrap().unwrap();
        assert!((unit.normal - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-15);
        assert!(
            (unit.d - 3.0).abs() < 1e-15,
            "offset of z = 3 is 3, got {}",
            unit.d
        );
    }

    #[test]
    fn non_unit_plane_storage_heals_to_the_same_sharp_corner() {
        for plane_storage_scale in [2.5, 0.4] {
            assert_heals_to_sharp_corner(&FixtureOptions {
                plane_storage_scale,
                ..FixtureOptions::default()
            });
        }
    }

    #[test]
    fn triple_plane_corner_solves_oblique_planes_and_refuses_near_parallel() {
        // Three oblique unit planes through the known point p.
        let p = Point3::new(1.25, -0.5, 2.0);
        let planes = [
            Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0),
            Vec3::new(-2.0, 1.0, 2.0) * (1.0 / 3.0),
            Vec3::new(0.6, 0.0, 0.8),
        ]
        .map(|normal| unit(normal, normal.dot(Vec3::new(p.x(), p.y(), p.z()))));
        let corner = triple_plane_corner(planes[0], planes[1], planes[2]).expect("transverse");
        assert!((corner - p).length() < 1e-12, "corner {corner:?} != {p:?}");
        for plane in planes {
            assert!(plane_residual(plane, corner).abs() < 1e-12);
        }
        // A third plane within 1e-8 rad of the first gives |det| ~ 1e-8,
        // below the 1e-6 corner gate: no meaningful corner exists.
        let tilt = 1e-8;
        let almost = unit(
            Vec3::new(1.0, 2.0 + tilt, 2.0) * (1.0 / (9.0 + 4.0 * tilt).sqrt()),
            1.0,
        );
        assert!(triple_plane_corner(planes[0], planes[1], almost).is_none());
    }

    fn assert_roots_on_line_and_sphere(
        roots: &[Point3],
        origin: Point3,
        direction: Vec3,
        center: Point3,
        radius: f64,
    ) {
        let unit_direction = direction.normalize().unwrap();
        for root in roots {
            let offset = *root - origin;
            let off_line = (offset - unit_direction * offset.dot(unit_direction)).length();
            assert!(
                off_line < 1e-9 * radius.max(1.0),
                "root off the line by {off_line}"
            );
            let off_sphere = ((*root - center).length() - radius).abs();
            assert!(
                off_sphere < 1e-9 * radius.max(1.0),
                "root off the sphere by {off_sphere}"
            );
        }
    }

    #[test]
    fn line_sphere_roots_are_invariant_to_direction_length() {
        // Line x = 2, y = 2 through the radius-8 sphere: roots z = ±sqrt(56).
        let center = Point3::new(0.0, 0.0, 0.0);
        let origin = Point3::new(2.0, 2.0, -3.0);
        let expected = (56.0_f64).sqrt();
        for length in [1.0, 3.0, 0.25, 1e-7] {
            let direction = Vec3::new(0.0, 0.0, length);
            let roots = line_sphere_roots(origin, direction, center, R_SPHERE);
            assert_eq!(roots.len(), 2, "direction length {length}");
            assert_roots_on_line_and_sphere(&roots, origin, direction, center, R_SPHERE);
            assert!((roots[0].z() + expected).abs() < 1e-12);
            assert!((roots[1].z() - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn line_sphere_roots_separate_distinct_roots_from_tangency() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let direction = Vec3::new(0.0, 0.0, 1.0);
        // Chord half-length 1e-4 (roots 2e-4 apart, far above the 1e-7
        // modeling tolerance): two genuine roots.
        let span = 1e-4;
        let distance = (R_SPHERE * R_SPHERE - span * span).sqrt();
        let origin = Point3::new(distance, 0.0, 0.0);
        let roots = line_sphere_roots(origin, direction, center, R_SPHERE);
        assert_eq!(roots.len(), 2);
        assert_roots_on_line_and_sphere(&roots, origin, direction, center, R_SPHERE);
        assert!(((roots[1] - roots[0]).length() - 2.0 * span).abs() < 1e-9);
        // Chord half-length 5e-8, below the modeling tolerance: tangent.
        let span = 5e-8;
        let distance = (R_SPHERE * R_SPHERE - span * span).sqrt();
        let origin = Point3::new(distance, 0.0, 0.0);
        assert!(line_sphere_roots(origin, direction, center, R_SPHERE).is_empty());
        // At radius 1e6 the squared chord 1e-3 is below the cancellation
        // noise of R^2 - d^2 (~64 eps * 1e12): the roots are not certified.
        let radius: f64 = 1e6;
        let span_squared = 1e-3;
        let origin = Point3::new((radius * radius - span_squared).sqrt(), 0.0, 0.0);
        assert!(line_sphere_roots(origin, direction, center, radius).is_empty());
    }

    fn sphere_r8() -> remus_math::surfaces::SphericalSurface {
        SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), R_SPHERE).unwrap()
    }

    #[test]
    fn plane_sphere_circle_certificate_accepts_the_exact_section() {
        // Plane x = 2 meets |p| = 8 in the circle centred (2,0,0), r^2 = 60.
        let plane = unit(Vec3::new(1.0, 0.0, 0.0), 2.0);
        for normal in [Vec3::new(1.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)] {
            let circle =
                Circle3D::new(Point3::new(2.0, 0.0, 0.0), normal, 60.0_f64.sqrt()).unwrap();
            assert!(certify_plane_sphere_circle(&circle, plane, &sphere_r8()));
        }
        // Plane z = 0 is the great circle, radius 8 (r^2 = 64 differs from 2r).
        let equator =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 8.0).unwrap();
        assert!(certify_plane_sphere_circle(
            &equator,
            unit(Vec3::new(0.0, 0.0, 1.0), 0.0),
            &sphere_r8()
        ));
    }

    #[test]
    fn plane_sphere_circle_certificate_refuses_each_violated_clause() {
        let plane = unit(Vec3::new(1.0, 0.0, 0.0), 2.0);
        let sphere = sphere_r8();
        // 1. A genuine sphere section on a plane tilted 1e-5 rad about y whose
        //    centre still lies on x = 2: centre and radius clauses hold, only
        //    the normal clause (1 - cos 1e-5 = 5e-11 > 1e-12) refuses.
        let tilt: f64 = 1e-5;
        let tilted = Vec3::new(tilt.cos(), 0.0, tilt.sin());
        let distance = 2.0 / tilt.cos();
        let tilted_circle = Circle3D::new(
            Point3::new(0.0, 0.0, 0.0) + tilted * distance,
            tilted,
            (R_SPHERE * R_SPHERE - distance * distance).sqrt(),
        )
        .unwrap();
        assert!((plane_residual(plane, tilted_circle.center())).abs() < 1e-12);
        assert!(!certify_plane_sphere_circle(&tilted_circle, plane, &sphere));
        // 2. The exact section of the parallel plane x = 2 + 1e-4: it lies on
        //    the sphere, but not on the support plane.
        let shifted = 2.0 + 1e-4;
        let off_plane = Circle3D::new(
            Point3::new(shifted, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            (R_SPHERE * R_SPHERE - shifted * shifted).sqrt(),
        )
        .unwrap();
        assert!(!certify_plane_sphere_circle(&off_plane, plane, &sphere));
        // 3. In-plane circle with the section radius but a centre 1e-4 off the
        //    sphere axis: plane and radius clauses hold, it is not on the sphere.
        let off_axis = Circle3D::new(
            Point3::new(2.0, 1e-4, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            60.0_f64.sqrt(),
        )
        .unwrap();
        assert!(!certify_plane_sphere_circle(&off_axis, plane, &sphere));
        // 4. Correct centre and plane, radius 1e-4 too large.
        let wrong_radius = Circle3D::new(
            Point3::new(2.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            60.0_f64.sqrt() + 1e-4,
        )
        .unwrap();
        assert!(!certify_plane_sphere_circle(&wrong_radius, plane, &sphere));
    }

    /// Circle of radius 3 about the z axis, u axis +x: `evaluate(t)` is the
    /// point at polar angle t, so expected parameters are exact angles.
    fn polar_circle() -> Circle3D {
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    #[test]
    fn extended_circle_trim_grows_the_named_terminal_by_the_exact_angle() {
        let circle = polar_circle();
        let tau = std::f64::consts::TAU;
        // (old domain, replace_start, angle of the new terminal, expected).
        // Domains are placed on shifted periods and in both parameter
        // directions; each expectation is the unique continuation of the
        // named terminal that keeps the other terminal fixed.
        let cases = [
            ((0.0, 1.0), false, 1.5, (0.0, 1.5)),
            ((0.0, 1.0), true, -0.25, (-0.25, 1.0)),
            (
                (2.0 * tau + 0.5, 2.0 * tau + 1.0),
                false,
                1.75,
                (2.0 * tau + 0.5, 2.0 * tau + 1.75),
            ),
            (
                (2.0 * tau + 0.5, 2.0 * tau + 1.0),
                true,
                0.25,
                (2.0 * tau + 0.25, 2.0 * tau + 1.0),
            ),
            ((-5.0, -4.0), false, -3.5, (-5.0, -3.5)),
            ((-5.0, -4.0), true, -5.25, (-5.25, -4.0)),
            // Decreasing parameter: the end grows downward, the start upward.
            ((4.0, 3.0), false, 2.5, (4.0, 2.5)),
            ((4.0, 3.0), true, 4.5, (4.5, 3.0)),
            ((-3.0, -4.0), false, -4.75, (-3.0, -4.75)),
            ((-3.0, -4.0), true, -2.5, (-2.5, -4.0)),
            // Growth across the principal-angle seam of `project`.
            ((2.5, 3.0), false, 3.5, (2.5, 3.5)),
            ((-2.5, -3.0), false, -3.5, (-2.5, -3.5)),
            // A zero-length growth is the old domain itself.
            ((0.5, 2.0), false, 2.0, (0.5, 2.0)),
        ];
        for (old, replace_start, angle, expected) in cases {
            let point = circle.evaluate(angle);
            let actual = extended_circle_trim(&circle, old, replace_start, point)
                .unwrap_or_else(|error| panic!("{old:?} {replace_start} {angle}: {error}"));
            assert!(
                (actual.0 - expected.0).abs() < 1e-12 && (actual.1 - expected.1).abs() < 1e-12,
                "{old:?} replace_start={replace_start} angle={angle}: got {actual:?}, expected {expected:?}"
            );
        }
    }

    #[test]
    fn extended_circle_trim_refuses_retraction_and_overturn() {
        let circle = polar_circle();
        // A point strictly inside the old arc would shorten it: no
        // continuation of that terminal reaches it within one turn.
        for (old, replace_start, angle) in [
            ((0.0, 2.0), false, 1.0),
            ((0.0, 2.0), true, 1.0),
            ((2.0, 0.0), false, 1.0),
            ((2.0, 0.0), true, 1.0),
            ((-1.0, 5.0), false, 4.0),
            ((5.0, -1.0), true, 4.0),
        ] {
            assert!(
                extended_circle_trim(&circle, old, replace_start, circle.evaluate(angle)).is_err(),
                "{old:?} replace_start={replace_start} angle={angle}"
            );
        }
    }

    fn circle_edge_topology(
        domain: (f64, f64),
    ) -> (Topology, EdgeId, VertexId, VertexId, Circle3D) {
        let circle = polar_circle();
        let mut topo = Topology::new();
        let start = topo.add_vertex(Vertex::new(
            circle.evaluate(domain.0),
            Tolerance::new().linear,
        ));
        let end = topo.add_vertex(Vertex::new(
            circle.evaluate(domain.1),
            Tolerance::new().linear,
        ));
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
        edge.set_trim(Some(domain));
        let edge = topo.add_edge(edge);
        (topo, edge, start, end, circle)
    }

    #[test]
    fn locally_extends_circle_edge_accepts_growth_below_half_a_turn() {
        use std::f64::consts::PI;
        for (domain, grow_end, angle) in [
            ((1.0, 2.0), true, 2.5),
            ((1.0, 2.0), false, 0.25),
            ((7.0, 6.0), true, 5.0),
            ((7.0, 6.0), false, 7.5),
            // Zero growth: the root is the old terminal itself.
            ((1.0, 2.0), true, 2.0),
            ((-4.0, -3.5), false, -4.0),
            // Just under half a turn of growth.
            ((1.0, 2.0), true, 2.0 + PI - 1e-3),
        ] {
            let (topo, edge, start, end, circle) = circle_edge_topology(domain);
            let old_vertex = if grow_end { end } else { start };
            assert!(
                locally_extends_circle_edge(&topo, edge, old_vertex, circle.evaluate(angle))
                    .unwrap(),
                "{domain:?} grow_end={grow_end} angle={angle}"
            );
        }
    }

    #[test]
    fn locally_extends_circle_edge_refuses_non_local_roots() {
        use std::f64::consts::PI;
        let (topo, edge, start, end, circle) = circle_edge_topology((1.0, 2.0));
        // More than half a turn of growth reaches the far branch.
        assert!(
            !locally_extends_circle_edge(&topo, edge, end, circle.evaluate(2.0 + PI + 1e-3))
                .unwrap()
        );
        assert!(
            !locally_extends_circle_edge(&topo, edge, start, circle.evaluate(1.0 - PI - 1e-3))
                .unwrap()
        );
        // A root inside the arc would retract it.
        assert!(!locally_extends_circle_edge(&topo, edge, end, circle.evaluate(1.5)).unwrap());
        // A point 1e-4 off the carrier is not on the extended trim.
        let off = circle.evaluate(2.5) + Vec3::new(0.0, 0.0, 1e-4);
        assert!(!locally_extends_circle_edge(&topo, edge, end, off).unwrap());
        // A vertex that is not an endpoint of the edge names no terminal.
        let mut topo = topo;
        let stray = topo.add_vertex(Vertex::new(circle.evaluate(3.0), Tolerance::new().linear));
        assert!(!locally_extends_circle_edge(&topo, edge, stray, circle.evaluate(2.5)).unwrap());
    }

    #[test]
    fn shifted_decreasing_and_reversed_trims_heal_to_the_same_extension() {
        for reverse_sphere_boundaries in [false, true] {
            for (offset, decreasing) in [(2.0, false), (-5.0, false), (9.0, true), (-1.5, true)] {
                assert_heals_to_sharp_corner(&FixtureOptions {
                    reverse_sphere_boundaries,
                    sphere_boundary_trim: TrimLayout { offset, decreasing },
                    ..FixtureOptions::default()
                });
            }
        }
    }

    #[test]
    fn sphere_chart_seam_in_either_longitude_direction_heals() {
        // Rotate the spherical chart so its longitude seam sweeps across the
        // two extended circles, with the chart axis both up and down (the
        // latter reverses the longitude direction of the same arcs).
        for axis in [Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0)] {
            for step in 0..12 {
                let angle = f64::from(step) * std::f64::consts::TAU / 12.0;
                let reference = Vec3::new(angle.cos(), angle.sin(), 0.0);
                assert_heals_to_sharp_corner(&FixtureOptions {
                    sphere_frame: Some((axis, reference)),
                    ..FixtureOptions::default()
                });
            }
        }
    }

    #[test]
    fn deep_column_far_root_is_excluded_by_the_cap_hemisphere_alone() {
        // With the planar cap at z = -10 both line/sphere roots z = ±sqrt(56)
        // advance from the cap and both are local extensions of the old
        // circles (the far one by 165 degrees); only the cap-hemisphere proof
        // rejects the disconnected far root.
        let options = FixtureOptions {
            depth: 10.0,
            ..FixtureOptions::default()
        };
        let fixture = sphere_ended_band_fixture_with(&options);
        let plan =
            classify_sphere_end(&fixture.topo, fixture.solid, fixture.band, fixture.supports)
                .unwrap()
                .expect("qualified sphere end");
        let far = Point3::new(2.0, 2.0, -(56.0_f64).sqrt());
        let travel = (fixture
            .topo
            .vertex(plan.sphere_vertices[0])
            .unwrap()
            .point()
            - fixture
                .topo
                .vertex(plan.planar_vertices[0])
                .unwrap()
                .point())
        .normalize()
        .unwrap();
        assert!(
            (far - plan.planar_corner).dot(travel) > 1.0,
            "far root advances from the cap"
        );
        for (edge, vertex) in plan.sphere_boundaries.into_iter().zip(plan.sphere_vertices) {
            assert!(locally_extends_circle_edge(&fixture.topo, edge, vertex, far).unwrap());
        }
        assert!((plan.sphere_corner - sphere_corner(1.0)).length() < 1e-12);

        let (topo, solid) = assert_heals_to_sharp_corner(&options);
        let measured = crate::measure::solid_volume(&topo, solid, 0.00001)
            .unwrap()
            .abs();
        let oracle = sharp_volume_oracle_with_depth(1.0, 10.0);
        assert!(
            (measured - oracle).abs() <= oracle * 2e-6,
            "independent sharp-volume oracle: measured={measured:.12}, oracle={oracle:.12}"
        );
    }

    /// Classification must refuse, and the public healer must leave the
    /// arena untouched.
    fn assert_refuses_without_mutation(fixture: Fixture, what: &str) {
        let Fixture {
            mut topo,
            solid,
            band,
            supports,
            ..
        } = fixture;
        assert!(
            classify_sphere_end(&topo, solid, band, supports)
                .unwrap_or_else(|error| panic!("{what}: typed refusal expected, got {error}"))
                .is_none(),
            "{what}: classification must refuse"
        );
        let slots = topo.allocated_slot_count();
        assert!(
            heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
                .unwrap_or_else(|error| panic!("{what}: {error}"))
                .is_none(),
            "{what}: healer must decline"
        );
        assert_eq!(
            topo.allocated_slot_count(),
            slots,
            "{what}: arena unchanged"
        );
    }

    fn vertex_at(topo: &Topology, solid: SolidId, target: Point3) -> VertexId {
        remus_topology::explorer::solid_vertices(topo, solid)
            .unwrap()
            .into_iter()
            .find(|vertex| (topo.vertex(*vertex).unwrap().point() - target).length() < 1e-12)
            .unwrap_or_else(|| panic!("no fixture vertex at {target:?}"))
    }

    #[test]
    fn uncertified_sphere_boundary_circle_refuses() {
        // Replace the x = r boundary carrier by the circle through the same
        // two vertices, in the same plane, whose centre sits 1e-3 along the
        // chord bisector: the trim is exact at both vertices, but the carrier
        // is not the plane/sphere section.
        let mut fixture = sphere_ended_band_fixture(1.0);
        let edge = fixture.sphere_boundaries[0];
        let data = fixture.topo.edge(edge).unwrap();
        let EdgeCurve::Circle(circle) = data.curve().clone() else {
            unreachable!("fixture circle");
        };
        let start = fixture.topo.vertex(data.start()).unwrap().point();
        let end = fixture.topo.vertex(data.end()).unwrap().point();
        let bisector = circle.normal().cross(end - start).normalize().unwrap();
        let center = circle.center() + bisector * 1e-3;
        let moved = Circle3D::new_with_ref(
            center,
            circle.normal(),
            (start - center).length(),
            start - center,
        )
        .unwrap();
        let end_parameter = moved.project(end).rem_euclid(std::f64::consts::TAU);
        assert!(end_parameter < std::f64::consts::PI);
        assert!((moved.evaluate(end_parameter) - end).length() < 1e-12);
        let edge_data = fixture.topo.edge_mut(edge).unwrap();
        edge_data.set_curve(EdgeCurve::Circle(moved));
        edge_data.set_trim(Some((0.0, end_parameter)));
        assert!(fixture.topo.edge(edge).unwrap().strict_domain().is_ok());
        assert_refuses_without_mutation(fixture, "uncertified circle");
    }

    #[test]
    fn opposed_spring_travel_refuses() {
        // Move the y = r terminal below its planar vertex: the two springs
        // then travel in opposite directions and define no wound direction.
        let mut fixture = sphere_ended_band_fixture(1.0);
        let terminal = vertex_at(
            &fixture.topo,
            fixture.solid,
            Point3::new(0.0, 2.0, (60.0_f64).sqrt()),
        );
        fixture
            .topo
            .vertex_mut(terminal)
            .unwrap()
            .set_point(Point3::new(0.0, 2.0, -5.0));
        assert_refuses_without_mutation(fixture, "opposed springs");
    }

    #[test]
    fn terminals_on_opposite_cap_hemispheres_refuse() {
        // Deep column; the y = r terminal is moved to the lower sphere root of
        // its own support line.  It is still on the sphere, the support plane
        // and the band carrier, and still above its planar vertex, but the two
        // old terminals now lie on opposite hemispheres of the wound.
        let mut fixture = sphere_ended_band_fixture_with(&FixtureOptions {
            depth: 10.0,
            ..FixtureOptions::default()
        });
        let zb = (60.0_f64).sqrt();
        let terminal = vertex_at(&fixture.topo, fixture.solid, Point3::new(0.0, 2.0, zb));
        fixture
            .topo
            .vertex_mut(terminal)
            .unwrap()
            .set_point(Point3::new(0.0, 2.0, -zb));
        assert_refuses_without_mutation(fixture, "opposite hemispheres");
    }

    #[test]
    fn band_carrier_violations_refuse() {
        // (a) The band cylinder 1e-4 too large: every spring endpoint is off
        //     the carrier radially, nothing else changes.
        let mut fixture = sphere_ended_band_fixture(1.0);
        fixture
            .topo
            .face_mut(fixture.band)
            .unwrap()
            .set_surface(FaceSurface::Cylinder(
                CylindricalSurface::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0 + 1e-4,
                )
                .unwrap(),
            ));
        assert_refuses_without_mutation(fixture, "radius off carrier");

        // (b) A horizontal cylinder along (1,-1,0)/sqrt 2 through all four
        //     spring endpoints: every endpoint is on the carrier, but the
        //     springs are not generatrices.
        let mut fixture = sphere_ended_band_fixture(1.0);
        let zb = (60.0_f64).sqrt();
        let (mid, half) = (0.5 * (zb - 3.0), 0.5 * (zb + 3.0));
        let axis = Vec3::new(1.0, -1.0, 0.0) * std::f64::consts::FRAC_1_SQRT_2;
        let sideways =
            CylindricalSurface::new(Point3::new(0.0, 0.0, mid), axis, (2.0 + half * half).sqrt())
                .unwrap();
        for point in [
            Point3::new(2.0, 0.0, -3.0),
            Point3::new(2.0, 0.0, zb),
            Point3::new(0.0, 2.0, -3.0),
            Point3::new(0.0, 2.0, zb),
        ] {
            let offset = point - sideways.origin();
            let radial = (offset - axis * offset.dot(axis)).length();
            assert!(
                (radial - sideways.radius()).abs() < 1e-12,
                "endpoint on the sideways carrier"
            );
        }
        fixture
            .topo
            .face_mut(fixture.band)
            .unwrap()
            .set_surface(FaceSurface::Cylinder(sideways));
        assert_refuses_without_mutation(fixture, "springs not generatrices");

        // (c) Rotate the x = r spring 0.01 rad about the band axis: it stays a
        //     generatrix on the carrier and parallel to the other spring, and
        //     its terminal stays on the sphere, but it leaves the x = r
        //     support plane by r (1 - cos 0.01) = 1e-4.
        let mut fixture = sphere_ended_band_fixture(1.0);
        let (sin, cos) = (0.01_f64).sin_cos();
        for z in [-3.0, zb] {
            let vertex = vertex_at(&fixture.topo, fixture.solid, Point3::new(2.0, 0.0, z));
            fixture
                .topo
                .vertex_mut(vertex)
                .unwrap()
                .set_point(Point3::new(2.0 * cos, 2.0 * sin, z));
        }
        assert_refuses_without_mutation(fixture, "spring off its support plane");
    }

    #[test]
    fn terminal_vertex_touched_by_a_fourth_face_refuses() {
        // A tetrahedral lump pinched onto the planar terminal (r, 0, -3):
        // it shares that vertex but no edge, so the terminal is no longer the
        // simple three-face corner the reconstruction replaces.
        let mut fixture = sphere_ended_band_fixture(1.0);
        let pinch = vertex_at(&fixture.topo, fixture.solid, Point3::new(2.0, 0.0, -3.0));
        let topo = &mut fixture.topo;
        let a = vertex(topo, 3.0, -1.0, -4.0);
        let b = vertex(topo, 4.0, 0.0, -4.0);
        let c = vertex(topo, 3.0, 1.0, -4.0);
        let pa = line(topo, pinch, a);
        let pb = line(topo, pinch, b);
        let pc = line(topo, pinch, c);
        let ab = line(topo, a, b);
        let bc = line(topo, b, c);
        let ca = line(topo, c, a);
        let plane = |normal: Vec3| FaceSurface::Plane {
            normal,
            d: normal.dot(Vec3::new(2.0, 0.0, -3.0)),
        };
        let lump = [
            face(
                topo,
                vec![
                    OrientedEdge::new(pa, true),
                    OrientedEdge::new(ab, true),
                    OrientedEdge::new(pb, false),
                ],
                plane(Vec3::new(1.0, -1.0, 1.0)),
            ),
            face(
                topo,
                vec![
                    OrientedEdge::new(pb, true),
                    OrientedEdge::new(bc, true),
                    OrientedEdge::new(pc, false),
                ],
                plane(Vec3::new(1.0, 1.0, 1.0)),
            ),
            face(
                topo,
                vec![
                    OrientedEdge::new(pc, true),
                    OrientedEdge::new(ca, true),
                    OrientedEdge::new(pa, false),
                ],
                plane(Vec3::new(-1.0, 0.0, 1.0)),
            ),
            face(
                topo,
                vec![
                    OrientedEdge::new(ab, false),
                    OrientedEdge::new(ca, false),
                    OrientedEdge::new(bc, false),
                ],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, -1.0),
                    d: 4.0,
                },
            ),
        ];
        let shell = topo.solid(fixture.solid).unwrap().outer_shell();
        let mut faces = topo.shell(shell).unwrap().faces().to_vec();
        faces.extend(lump);
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        fixture.solid = topo.add_solid(Solid::new(shell, vec![]));
        assert_refuses_without_mutation(fixture, "pinched terminal");
    }

    #[test]
    fn sharp_corner_outside_the_local_patch_refuses() {
        // Tilt the planar end to k*y + z = -3s: the planar sharp corner drops
        // to z = -3s - 2sk while the old terminal stays at (2s, 0, -3s).  The
        // displacement bound is 4 * max(patch span, r).
        for (scale, tilt) in [(1.0, 30.0), (0.1, 23.5)] {
            let mut fixture = sphere_ended_band_fixture(scale);
            let displacement = (4.0 * scale * scale + (2.0 * scale * tilt).powi(2)).sqrt();
            // Widest pair of old terminals: (2s, 0, sqrt(60) s) to (0, 2s, -3s).
            let span = (8.0 + (60.0_f64.sqrt() + 3.0).powi(2)).sqrt() * scale;
            let bound = 4.0 * f64::max(span, 2.0 * scale);
            assert!(displacement > bound * 1.02, "{displacement} vs {bound}");
            let bottom = remus_topology::explorer::solid_faces(&fixture.topo, fixture.solid)
                .unwrap()
                .into_iter()
                .find(|face| {
                    matches!(
                        fixture.topo.face(*face).unwrap().surface(),
                        FaceSurface::Plane { normal, .. } if normal.z() < -0.5
                    )
                })
                .unwrap();
            let norm = (1.0 + tilt * tilt).sqrt();
            fixture
                .topo
                .face_mut(bottom)
                .unwrap()
                .set_surface(FaceSurface::Plane {
                    normal: Vec3::new(0.0, tilt / norm, 1.0 / norm),
                    d: -3.0 * scale / norm,
                });
            let Fixture {
                mut topo,
                solid,
                band,
                supports,
                ..
            } = fixture;
            let slots = topo.allocated_slot_count();
            let error = match heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
            {
                Err(error) => error,
                Ok(outcome) => panic!(
                    "scale {scale}: far corner must refuse, got {}",
                    if outcome.is_some() {
                        "a result"
                    } else {
                        "a decline"
                    }
                ),
            };
            assert!(
                error.to_string().contains("outside the local blend patch"),
                "scale {scale}: {error}"
            );
            assert_eq!(topo.allocated_slot_count(), slots);
        }
    }

    #[test]
    fn rotated_bodies_heal_across_planar_chart_seams() {
        // Planar p-curve charts derive their axes from the stored normal, so
        // turning the body about z sweeps each support's circle angles across
        // the principal-angle seam.  Every placement heals to the rotated
        // closed-form corner.
        for reverse_sphere_boundaries in [false, true] {
            for step in 0..12 {
                let angle = f64::from(step) * std::f64::consts::TAU / 12.0 + 0.1;
                let Fixture {
                    mut topo,
                    solid,
                    band,
                    supports,
                    ..
                } = sphere_ended_band_fixture_with(&FixtureOptions {
                    reverse_sphere_boundaries,
                    ..FixtureOptions::default()
                });
                let rotation = remus_math::mat::Mat4::rotation_z(angle);
                crate::transform::transform_solid(&mut topo, solid, &rotation).unwrap();
                let outcome = heal_cylinder_plane_band_sphere_end(&mut topo, solid, band, supports)
                    .unwrap_or_else(|error| panic!("angle {angle}: {error}"))
                    .expect("rotated sphere end stays qualified");
                assert_valid(&topo, outcome.solid);
                let corner = rotation.mul_point(sphere_corner(1.0));
                let reaches = remus_topology::explorer::solid_vertices(&topo, outcome.solid)
                    .unwrap()
                    .into_iter()
                    .any(|vertex| (topo.vertex(vertex).unwrap().point() - corner).length() < 1e-9);
                assert!(reaches, "angle {angle}: sharp corner at the rotated root");
            }
        }
    }

    #[test]
    fn root_that_extends_only_one_boundary_circle_refuses() {
        // Store the y = r boundary as the complementary major arc between the
        // same two vertices (same exact carrier, trim > pi).  Growing that arc
        // past its band terminal to the sharp root would sweep more than half
        // a turn, so the root is local to the x = r circle only.
        let mut fixture = sphere_ended_band_fixture(1.0);
        let edge = fixture.sphere_boundaries[1];
        let data = fixture.topo.edge(edge).unwrap();
        let EdgeCurve::Circle(circle) = data.curve().clone() else {
            unreachable!("fixture circle");
        };
        let (t0, t1) = data.strict_domain().unwrap();
        let minor = t1 - t0;
        assert!(minor.abs() < std::f64::consts::PI);
        // Same carrier on the opposite normal: flipped(-t) == circle(t), so
        // the major arc from the start vertex runs over (-t0, -t0 + (2pi - |minor|)).
        let flipped = Circle3D::new_with_ref(
            circle.center(),
            circle.normal() * -1.0,
            circle.radius(),
            circle.u_axis(),
        )
        .unwrap();
        let major = std::f64::consts::TAU - minor.abs();
        let trim = (-t0, -t0 + major);
        let start = fixture.topo.vertex(data.start()).unwrap().point();
        let end = fixture.topo.vertex(data.end()).unwrap().point();
        assert!((flipped.evaluate(trim.0) - start).length() < 1e-12);
        assert!((flipped.evaluate(trim.1) - end).length() < 1e-12);
        let edge_data = fixture.topo.edge_mut(edge).unwrap();
        edge_data.set_curve(EdgeCurve::Circle(flipped));
        edge_data.set_trim(Some(trim));
        let plan =
            classify_sphere_end(&fixture.topo, fixture.solid, fixture.band, fixture.supports);
        let Err(error) = plan else {
            panic!("a root local to one circle only must not qualify");
        };
        assert!(
            error.to_string().contains("expected exactly one"),
            "{error}"
        );
    }
}
