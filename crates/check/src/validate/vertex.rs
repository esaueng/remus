//! Vertex geometric validation checks.

use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

/// Check that a vertex lies on its edge's 3D curve within tolerance.
///
/// Measures the distance from the vertex to the edge's geometry: the
/// closest point of a conic carrier, or the nearer end of a NURBS edge
/// (its stored trim ends, falling back to the curve's knot domain).
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
        // The edge's ends are its trim ends; the carrier's knot-span ends
        // belong to the edge only when no trim is stored (a shared split
        // curve would otherwise measure the vertex against the sibling's
        // far end).
        EdgeCurve::NurbsCurve(nc) => {
            let (t0, t1) = edge.trim().unwrap_or_else(|| nc.domain());
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

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::nurbs::fitting::interpolate;
    use remus_math::vec::Point3;
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
    use remus_topology::vertex::{Vertex, VertexId};

    use super::check_vertex_on_curve;
    use crate::validate::checks::CheckId;

    const TOL: f64 = 1e-6;

    fn arc_curve() -> NurbsCurve {
        interpolate(
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.6, 0.0),
                Point3::new(2.0, 0.8, 0.0),
                Point3::new(3.0, 0.6, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            3,
        )
        .unwrap()
    }

    /// One carrier split at its mid-parameter into two trimmed edges, the
    /// shape FF section splits emit. Returns the start, mid and end
    /// vertices and the two edges.
    fn split_halves(topo: &mut Topology) -> ([VertexId; 3], [EdgeId; 2]) {
        let curve = arc_curve();
        let (d0, d1) = curve.domain();
        let dm = 0.5 * (d0 + d1);
        let va = topo.add_vertex(Vertex::new(curve.evaluate(d0), 1e-7));
        let vm = topo.add_vertex(Vertex::new(curve.evaluate(dm), 1e-7));
        let vb = topo.add_vertex(Vertex::new(curve.evaluate(d1), 1e-7));
        let first = topo.add_edge(Edge::new(va, vm, EdgeCurve::NurbsCurve(curve.clone())));
        topo.edge_mut(first).unwrap().set_trim(Some((d0, dm)));
        let second = topo.add_edge(Edge::new(vm, vb, EdgeCurve::NurbsCurve(curve)));
        topo.edge_mut(second).unwrap().set_trim(Some((dm, d1)));
        ([va, vm, vb], [first, second])
    }

    #[test]
    fn vertex_at_nurbs_trim_end_is_on_curve() {
        // Regression: the NURBS arm measured against the carrier's knot-span
        // ends, so the split vertex read as ~2 units off both halves.
        let mut topo = Topology::new();
        let ([va, vm, vb], [first, second]) = split_halves(&mut topo);
        for (vertex, edge) in [(va, first), (vm, first), (vm, second), (vb, second)] {
            let issues = check_vertex_on_curve(&topo, vertex, edge, TOL).unwrap();
            assert!(
                issues.is_empty(),
                "vertex at a trim end must lie on its edge: {issues:?}"
            );
        }
    }

    #[test]
    fn vertex_at_carrier_end_outside_trim_is_reported() {
        // The far carrier end is not an end of the trimmed first half: it
        // is reported, with the distance to the NEARER trim end (the split
        // vertex), not waved through because it lies on the carrier.
        let mut topo = Topology::new();
        let ([_, vm, vb], [first, _]) = split_halves(&mut topo);
        let issues = check_vertex_on_curve(&topo, vb, first, TOL).unwrap();
        assert_eq!(issues.len(), 1, "{issues:?}");
        assert_eq!(issues[0].check, CheckId::VertexOnCurve);
        let expected =
            (topo.vertex(vb).unwrap().point() - topo.vertex(vm).unwrap().point()).length();
        let deviation = issues[0].deviation.unwrap();
        assert!(
            (deviation - expected).abs() < 1e-12,
            "deviation {deviation} must be the distance to the split vertex {expected}"
        );
    }

    #[test]
    fn untrimmed_nurbs_edge_falls_back_to_knot_domain() {
        let mut topo = Topology::new();
        let curve = arc_curve();
        let (d0, d1) = curve.domain();
        let va = topo.add_vertex(Vertex::new(curve.evaluate(d0), 1e-7));
        let vb = topo.add_vertex(Vertex::new(curve.evaluate(d1), 1e-7));
        let edge = topo.add_edge(Edge::new(va, vb, EdgeCurve::NurbsCurve(curve)));
        for vertex in [va, vb] {
            let issues = check_vertex_on_curve(&topo, vertex, edge, TOL).unwrap();
            assert!(issues.is_empty(), "{issues:?}");
        }
    }
}
