//! Recognition of bounded rectangular torus patches.
use remus_math::tolerance::Tolerance;
use remus_topology::{Topology, face::FaceId};

/// Recognize a four-arc rectangular torus patch in unwrapped parameter space.
/// Returns each angular interval as (start, span).
pub fn rectangular_torus_domain(
    topo: &Topology,
    face_id: FaceId,
    tol: Tolerance,
) -> Option<((f64, f64), (f64, f64))> {
    use remus_math::vec::Point2;
    use remus_topology::{edge::EdgeCurve, face::FaceSurface};
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Torus(torus) = face.surface() else {
        return None;
    };
    if !face.inner_wires().is_empty() {
        return None;
    }
    let edges = topo.wire(face.outer_wire()).ok()?.edges();
    if edges.len() != 4 {
        return None;
    }
    let angular_tol = tol.angular.max(tol.linear / torus.minor_radius());
    let mut polygon: Vec<Point2> = Vec::new();
    let mut previous: Option<Point2> = None;
    for oe in edges {
        let edge = topo.edge(oe.edge()).ok()?;
        if !matches!(edge.curve(), EdgeCurve::Circle(_)) {
            return None;
        }
        let (mut t0, mut t1) = edge.strict_domain().ok()?;
        if !oe.is_forward() {
            std::mem::swap(&mut t0, &mut t1);
        }
        let a = topo.vertex(edge.start()).ok()?.point();
        let b = topo.vertex(edge.end()).ok()?.point();
        let mut points = Vec::new();
        for k in 0..=8 {
            let p = edge.curve().evaluate_with_endpoints(
                (t1 - t0).mul_add(f64::from(k) / 8.0, t0),
                a,
                b,
            );
            let (mut u, mut v) = face.surface().project_point(p)?;
            let on_surface = face.surface().evaluate(u, v)?;
            if !u.is_finite()
                || !v.is_finite()
                || (on_surface - p).length()
                    > tol.linear.max(edge.tolerance().unwrap_or(tol.linear))
            {
                return None;
            }
            if let Some(last) = previous {
                let tau = std::f64::consts::TAU;
                u += tau * ((last.x() - u) / tau).round();
                v += tau * ((last.y() - v) / tau).round();
            }
            let uv = Point2::new(u, v);
            previous = Some(uv);
            points.push(uv);
        }
        let first = points[0];
        let constant_u = points
            .iter()
            .all(|p| (p.x() - first.x()).abs() <= angular_tol);
        let constant_v = points
            .iter()
            .all(|p| (p.y() - first.y()).abs() <= angular_tol);
        if constant_u == constant_v {
            return None;
        }
        polygon.extend(points);
    }
    let first = polygon.first()?;
    let last = polygon.last()?;
    if (*last - *first).length() > angular_tol {
        return None;
    }
    let u0 = polygon.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
    let u1 = polygon
        .iter()
        .map(|p| p.x())
        .fold(f64::NEG_INFINITY, f64::max);
    let v0 = polygon.iter().map(|p| p.y()).fold(f64::INFINITY, f64::min);
    let v1 = polygon
        .iter()
        .map(|p| p.y())
        .fold(f64::NEG_INFINITY, f64::max);
    if u1 - u0 <= angular_tol || v1 - v0 <= angular_tol {
        return None;
    }
    let centre = Point2::new(f64::midpoint(u0, u1), f64::midpoint(v0, v1));
    if !crate::builder::classify_2d::point_in_polygon_2d(centre, &polygon) {
        return None;
    }
    Some(((u0, u1 - u0), (v0, v1 - v0)))
}
