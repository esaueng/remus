//! Vertex geometric validation checks.

use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

/// Check that a vertex lies on its edge's 3D curve within tolerance.
///
/// Evaluates the edge curve at the domain start/end and measures
/// the distance to the vertex position.
pub fn check_vertex_on_curve(
    topo: &Topology,
    vertex_id: VertexId,
    edge_id: EdgeId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let vertex = topo.vertex(vertex_id)?;
    let edge = topo.edge(edge_id)?;
    let pos = vertex.point();

    // For Line edges, the vertices ARE the geometry — always consistent.
    let deviation = match edge.curve() {
        EdgeCurve::Line => return Ok(vec![]),
        EdgeCurve::Circle(c) => {
            let t_closest = c.project(pos);
            (pos - c.evaluate(t_closest)).length()
        }
        EdgeCurve::Ellipse(e) => {
            let t_closest = e.project(pos);
            (pos - e.evaluate(t_closest)).length()
        }
        // `project` is an exact closed-form inverse of the
        // parameterization for both conics, so `pos - evaluate(project(pos))`
        // is the true distance from the vertex to the curve.
        EdgeCurve::Hyperbola(h) => {
            let t_closest = h.project(pos);
            (pos - h.evaluate(t_closest)).length()
        }
        EdgeCurve::Parabola(p) => {
            let t_closest = p.project(pos);
            (pos - p.evaluate(t_closest)).length()
        }
        EdgeCurve::NurbsCurve(nc) => {
            let (t0, t1) = nc.domain();
            let d_start = (pos - nc.evaluate(t0)).length();
            let d_end = (pos - nc.evaluate(t1)).length();
            d_start.min(d_end)
        }
    };

    if deviation > tolerance {
        return Ok(vec![ValidationIssue {
            check: CheckId::VertexOnCurve,
            severity: Severity::Warning,
            entity: EntityRef::Vertex(vertex_id),
            description: format!(
                "vertex deviates {deviation:.2e} from edge curve (tolerance {tolerance:.2e})"
            ),
            deviation: Some(deviation),
        }]);
    }
    Ok(vec![])
}

/// Check that a vertex lies on a face's surface within tolerance.
///
/// Projects the vertex onto the surface and measures deviation.
pub fn check_vertex_on_surface(
    topo: &Topology,
    vertex_id: VertexId,
    face_id: FaceId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let vertex = topo.vertex(vertex_id)?;
    let pos = vertex.point();
    let face = topo.face(face_id)?;

    let deviation = match face.surface() {
        FaceSurface::Plane { normal, d } => {
            let pv = remus_math::vec::Vec3::new(pos.x(), pos.y(), pos.z());
            (normal.dot(pv) - d).abs()
        }
        FaceSurface::Cylinder(s) => {
            let (u, v) = s.project_point(pos);
            (pos - s.evaluate(u, v)).length()
        }
        FaceSurface::Cone(s) => {
            let (u, v) = s.project_point(pos);
            (pos - s.evaluate(u, v)).length()
        }
        FaceSurface::Sphere(s) => {
            let (u, v) = s.project_point(pos);
            (pos - s.evaluate(u, v)).length()
        }
        FaceSurface::Torus(s) => {
            let (u, v) = s.project_point(pos);
            (pos - s.evaluate(u, v)).length()
        }
        FaceSurface::Nurbs(s) => {
            match remus_math::nurbs::projection::project_point_to_surface(s, pos, tolerance) {
                Ok(proj) => proj.distance,
                Err(_) => return Ok(vec![]),
            }
        }
    };

    if deviation > tolerance {
        return Ok(vec![ValidationIssue {
            check: CheckId::VertexOnSurface,
            severity: Severity::Warning,
            entity: EntityRef::Vertex(vertex_id),
            description: format!(
                "vertex deviates {deviation:.2e} from face surface (tolerance {tolerance:.2e})"
            ),
            deviation: Some(deviation),
        }]);
    }
    Ok(vec![])
}
