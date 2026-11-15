//! A planar face and a cylindrical face of one solid that touch along a line
//! inside both faces, with no edge there: the solid has zero thickness along
//! that line (esaueng/remus#1739).
//!
//! A cut that leaves a drilled hole exactly tangent to a step wall keeps both
//! the wall and the hole wall, tangent along a vertical line segment that
//! neither face is trimmed at. Every edge still has two faces and the Euler
//! count balances, so nothing else in [`super::validate_solid`] sees it, and
//! the boolean returned it as `exact`. Faces of a manifold solid meet only on
//! their shared edges and vertices, so a point inside both open faces is a
//! defect whatever the normals do there.
//!
//! The check is analytic and bounded: a plane and a cylinder whose axis lies
//! parallel to the plane at exactly one radius from it are tangent along one
//! line. Faces that share an edge are skipped (a fillet touches each of its
//! supports along their shared edge). Otherwise samples along the line
//! inside both faces' axial extents are tested against each face's trims and
//! kept clear of every edge of both faces, so contact on an edge and contact
//! at isolated points are not reported. Anything the check cannot decide is
//! left unreported, because it adds an error.

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::{Severity, ValidationIssue};

/// Points sampled along each tangent line.
const LINE_SAMPLES: usize = 17;
/// Points sampled along each boundary edge to bound a face's extent.
const EDGE_SAMPLES: usize = 16;

struct PlaneFace {
    id: FaceId,
    normal: Vec3,
    offset: f64,
}

struct CylinderFace {
    id: FaceId,
    origin: Point3,
    axis: Vec3,
    radius: f64,
}

/// Points along every boundary edge of `face`, endpoints included.
fn boundary_samples(topo: &Topology, face: FaceId) -> Option<Vec<Point3>> {
    let face = topo.face(face).ok()?;
    let mut points = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire).ok()?.edges() {
            let edge = topo.edge(oriented.edge()).ok()?;
            let start = topo.vertex(edge.start()).ok()?.point();
            let end = topo.vertex(edge.end()).ok()?.point();
            if matches!(edge.curve(), EdgeCurve::Line) {
                points.extend([start, end]);
                continue;
            }
            let (t0, t1) = edge.strict_domain().ok()?;
            for k in 0..=EDGE_SAMPLES {
                #[allow(clippy::cast_precision_loss)]
                let t = (t1 - t0).mul_add(k as f64 / EDGE_SAMPLES as f64, t0);
                points.push(edge.curve().evaluate_with_endpoints(t, start, end));
            }
        }
    }
    Some(points)
}

/// The edges bounding `face`.
fn boundary_edges(topo: &Topology, face: FaceId) -> Option<Vec<remus_topology::edge::EdgeId>> {
    let face = topo.face(face).ok()?;
    let mut edges = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        edges.extend(topo.wire(wire).ok()?.edges().iter().map(|oe| oe.edge()));
    }
    Some(edges)
}

/// Whether `point` is at least `margin` from every edge in `edges`.
fn clear_of_edges(
    topo: &Topology,
    point: Point3,
    edges: &[remus_topology::edge::EdgeId],
    margin: f64,
) -> Option<bool> {
    for &edge in edges {
        let (distance, _) = remus_check::distance::point_to_edge(topo, point, edge).ok()?;
        if distance <= margin {
            return Some(false);
        }
    }
    Some(true)
}

/// A point on the line where `plane` touches `cylinder` that lies inside both
/// faces and clear of their edges, if the two are tangent along a line.
fn interior_contact(
    topo: &Topology,
    plane: &PlaneFace,
    cylinder: &CylinderFace,
    tol: Tolerance,
    margin: f64,
) -> Option<Point3> {
    // Axis parallel to the plane, at one radius from it.
    if plane.normal.dot(cylinder.axis).abs() > tol.angular {
        return None;
    }
    let axis_distance = plane
        .normal
        .dot(cylinder.origin - Point3::new(0.0, 0.0, 0.0))
        - plane.offset;
    if (axis_distance.abs() - cylinder.radius).abs() > tol.linear {
        return None;
    }
    // Faces that share an edge are skipped: a fillet is tangent to each of
    // its supports exactly along the edge they share, which is every blend
    // in a filleted part, and sampling those would only rediscover the edge.
    // A defect beside a shared edge is left unreported, like any case this
    // check cannot decide.
    let mut edges = boundary_edges(topo, plane.id)?;
    let cylinder_edges = boundary_edges(topo, cylinder.id)?;
    if cylinder_edges.iter().any(|edge| edges.contains(edge)) {
        return None;
    }
    edges.extend(cylinder_edges);

    // The tangent line: the axis moved one radius toward the plane.
    let foot = cylinder.origin - plane.normal * axis_distance;

    // Where both faces reach along the line.
    let span = |points: &[Point3]| {
        points
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                let t = (*p - foot).dot(cylinder.axis);
                (lo.min(t), hi.max(t))
            })
    };
    let (c_lo, c_hi) = span(&boundary_samples(topo, cylinder.id)?);
    let (p_lo, p_hi) = span(&boundary_samples(topo, plane.id)?);
    let (lo, hi) = (c_lo.max(p_lo), c_hi.min(p_hi));
    if hi - lo <= 2.0 * margin {
        return None;
    }

    for k in 1..LINE_SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let t = (hi - lo).mul_add(k as f64 / LINE_SAMPLES as f64, lo);
        let point = foot + cylinder.axis * t;
        let inside_both = remus_check::classify::surface_point_in_face(topo, plane.id, point)
            .ok()?
            && remus_check::classify::surface_point_in_face(topo, cylinder.id, point).ok()?;
        if inside_both && clear_of_edges(topo, point, &edges, margin)? {
            return Some(point);
        }
    }
    None
}

/// Whether `solid` has a plane–cylinder pair tangent along a line inside both
/// faces: the exact boolean gate's reading of [`tangent_line_contact_issues`].
pub(crate) fn has_tangent_line_contact(
    topo: &Topology,
    solid: remus_topology::solid::SolidId,
) -> bool {
    let Ok(faces) = remus_topology::explorer::solid_faces(topo, solid) else {
        return false;
    };
    let scale = crate::measure::solid_bounding_box(topo, solid)
        .map_or(1.0, |bounds| (bounds.max - bounds.min).length());
    !tangent_line_contact_issues(topo, &faces, Tolerance::new(), scale).is_empty()
}

/// Errors for every plane–cylinder pair of `faces` tangent along a line
/// inside both faces. `scale` is the solid's size (its bounding-box
/// diagonal), from which the edge clearance is taken.
pub(super) fn tangent_line_contact_issues(
    topo: &Topology,
    faces: &[FaceId],
    tol: Tolerance,
    scale: f64,
) -> Vec<ValidationIssue> {
    let mut planes = Vec::new();
    let mut cylinders = Vec::new();
    for &id in faces {
        let Ok(face) = topo.face(id) else { continue };
        match face.surface() {
            FaceSurface::Plane { normal, d } => {
                // `d` is the signed distance along the (unit) normal.
                if let Ok(normal) = normal.normalize() {
                    planes.push(PlaneFace {
                        id,
                        normal,
                        offset: *d,
                    });
                }
            }
            FaceSurface::Cylinder(c) => {
                if let Ok(axis) = c.axis().normalize() {
                    cylinders.push(CylinderFace {
                        id,
                        origin: c.origin(),
                        axis,
                        radius: c.radius(),
                    });
                }
            }
            _ => {}
        }
    }
    let margin = (1e-6 * scale).max(100.0 * tol.linear);
    let mut issues = Vec::new();
    for cylinder in &cylinders {
        for plane in &planes {
            if let Some(point) = interior_contact(topo, plane, cylinder, tol, margin) {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "planar face {} and cylindrical face {} touch along a line inside both \
                         faces with no edge there (zero thickness at ({:.6}, {:.6}, {:.6}))",
                        plane.id.index(),
                        cylinder.id.index(),
                        point.x(),
                        point.y(),
                        point.z()
                    ),
                });
            }
        }
    }
    issues
}

#[cfg(test)]
#[path = "tangent_contact_tests.rs"]
mod tests;
