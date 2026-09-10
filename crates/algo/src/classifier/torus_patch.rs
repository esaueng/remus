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

/// A local torus trim. The carrier is exact; this polygon approximates only
/// trim membership with adaptive sampling. `margin` includes the sampling
/// target and observed projection error; near-trim ray hits remain suspicious.
pub(super) struct TorusTrim {
    pub polygon: Vec<remus_math::vec::Point2>,
    pub centre: remus_math::vec::Point2,
    pub margin: f64,
}

/// Recognize a hole-free local patch (less than half a turn in each angle).
/// Adaptively project authoritative edge trims, rejecting non-finite curves,
/// non-closing loops, and exhausted sampling budgets. Unsupported patches keep
/// the existing classifier path.
pub(super) fn trimmed_torus_polygon(
    topo: &Topology,
    face_id: FaceId,
    tol: Tolerance,
) -> Option<TorusTrim> {
    use remus_math::vec::Point2;
    use remus_topology::face::FaceSurface;
    let face = topo.face(face_id).ok()?;
    let FaceSurface::Torus(torus) = face.surface() else {
        return None;
    };
    if !face.inner_wires().is_empty() || torus.major_radius() <= torus.minor_radius() {
        return None;
    }
    let metric = torus
        .minor_radius()
        .min(torus.major_radius() - torus.minor_radius());
    let margin = tol.angular.max(tol.linear / metric);
    let mut polygon: Vec<Point2> = Vec::new();
    let mut closure_margin = margin;
    let projection_margin = std::cell::Cell::new(margin);
    for oe in topo.wire(face.outer_wire()).ok()?.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        let (mut t0, mut t1) = edge.strict_domain().ok()?;
        if !oe.is_forward() {
            std::mem::swap(&mut t0, &mut t1);
        }
        let a = topo.vertex(edge.start()).ok()?.point();
        let b = topo.vertex(edge.end()).ok()?.point();
        let allowance = tol.linear.max(edge.tolerance().unwrap_or(tol.linear));
        closure_margin = closure_margin.max(allowance / metric);
        let evaluate = |t: f64, anchor: Point2| -> Option<Point2> {
            let p = edge.curve().evaluate_with_endpoints(t, a, b);
            let (u, v) = torus.project_point(p);
            let residual = (torus.evaluate(u, v) - p).length();
            if !u.is_finite() || !v.is_finite() || !residual.is_finite() {
                return None;
            }
            // Imported space curves can deviate from their supporting surface.
            // Account for that projection in classifier uncertainty only; never
            // change the edge tolerance or the geometry acceptance policy.
            projection_margin.set(projection_margin.get().max(residual / metric));
            Some(unwrap_uv(Point2::new(u, v), anchor))
        };
        let start = evaluate(t0, polygon.last().copied().unwrap_or(Point2::new(0.0, 0.0)))?;
        if let Some(last) = polygon.last()
            && (*last - start).length() > closure_margin.max(projection_margin.get())
        {
            return None;
        }
        polygon.push(start);
        // Seed short spans so periodic unwrapping never chooses a complementary
        // arc from a large endpoint-only jump.
        for k in 0..16 {
            let lo = (t1 - t0).mul_add(f64::from(k) / 16.0, t0);
            let hi = (t1 - t0).mul_add(f64::from(k + 1) / 16.0, t0);
            let left = *polygon.last()?;
            let right = evaluate(hi, left)?;
            sample_trim_segment(&evaluate, lo, hi, left, right, margin, 0, &mut polygon)?;
        }
    }
    if (*polygon.last()? - *polygon.first()?).length() > closure_margin.max(projection_margin.get())
    {
        return None;
    }
    let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &polygon {
        lo = Point2::new(lo.x().min(p.x()), lo.y().min(p.y()));
        hi = Point2::new(hi.x().max(p.x()), hi.y().max(p.y()));
    }
    let span = hi - lo;
    if span.x() <= margin
        || span.y() <= margin
        || span.x() >= std::f64::consts::PI
        || span.y() >= std::f64::consts::PI
    {
        return None;
    }
    Some(TorusTrim {
        polygon,
        centre: Point2::new(f64::midpoint(lo.x(), hi.x()), f64::midpoint(lo.y(), hi.y())),
        margin: closure_margin.max(projection_margin.get()) + margin,
    })
}

fn unwrap_uv(
    p: remus_math::vec::Point2,
    anchor: remus_math::vec::Point2,
) -> remus_math::vec::Point2 {
    let tau = std::f64::consts::TAU;
    remus_math::vec::Point2::new(
        p.x() + tau * ((anchor.x() - p.x()) / tau).round(),
        p.y() + tau * ((anchor.y() - p.y()) / tau).round(),
    )
}

#[allow(clippy::too_many_arguments)]
fn sample_trim_segment(
    evaluate: &impl Fn(f64, remus_math::vec::Point2) -> Option<remus_math::vec::Point2>,
    lo: f64,
    hi: f64,
    a: remus_math::vec::Point2,
    b: remus_math::vec::Point2,
    margin: f64,
    depth: u8,
    polygon: &mut Vec<remus_math::vec::Point2>,
) -> Option<()> {
    if polygon.len() >= 16384 || depth >= 20 {
        return None;
    }
    let mut error = 0.0_f64;
    for fraction in [0.25, 0.5, 0.75] {
        let expected = a + (b - a) * fraction;
        let actual = evaluate((hi - lo).mul_add(fraction, lo), expected)?;
        error = error.max((actual - expected).length());
    }
    if error <= margin {
        polygon.push(b);
        return Some(());
    }
    let mid = f64::midpoint(lo, hi);
    let p = evaluate(mid, a + (b - a) * 0.5)?;
    sample_trim_segment(evaluate, lo, mid, a, p, margin, depth + 1, polygon)?;
    sample_trim_segment(evaluate, mid, hi, p, b, margin, depth + 1, polygon)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::{
        curves::Circle3D,
        surfaces::ToroidalSurface,
        vec::{Point3, Vec3},
    };
    use remus_topology::{
        edge::{Edge, EdgeCurve},
        face::{Face, FaceSurface},
        vertex::Vertex,
        wire::{OrientedEdge, Wire},
    };

    #[test]
    fn projected_trim_preserves_split_arcs_winding_and_periodic_seams() {
        for scale in [0.01, 1.0, 100.0] {
            for (u0, v0) in [(0.3, 0.4), (5.8, 5.9)] {
                let u1 = u0 + 1.2;
                let v1 = v0 + 1.0;
                let um = f64::midpoint(u0, u1);
                let t = ToroidalSurface::with_axis(
                    Point3::new(2.0, -3.0, 4.0),
                    12.0 * scale,
                    3.0 * scale,
                    Vec3::new(1.0, 2.0, 3.0),
                )
                .unwrap();
                let major = |v: f64| {
                    Circle3D::new_with_ref(
                        t.center() + t.z_axis() * (t.minor_radius() * v.sin()),
                        t.z_axis(),
                        t.major_radius() + t.minor_radius() * v.cos(),
                        t.x_axis(),
                    )
                    .unwrap()
                };
                let minor = |u: f64| {
                    let radial = t.x_axis() * u.cos() + t.y_axis() * u.sin();
                    Circle3D::new_with_ref(
                        t.center() + radial * t.major_radius(),
                        radial.cross(t.z_axis()),
                        t.minor_radius(),
                        radial,
                    )
                    .unwrap()
                };
                let mut topo = Topology::new();
                let vertices = [(u0, v0), (um, v0), (u1, v0), (u1, v1), (u0, v1)]
                    .map(|(u, v)| topo.add_vertex(Vertex::new(t.evaluate(u, v), 1e-7)));
                let curves = [
                    (major(v0), (u0, um)),
                    (major(v0), (um, u1)),
                    (minor(u1), (v0, v1)),
                    (major(v1), (u1, u0)),
                    (minor(u0), (v1, v0)),
                ];
                let edges: Vec<_> = curves
                    .into_iter()
                    .enumerate()
                    .map(|(i, (c, domain))| {
                        let mut e =
                            Edge::new(vertices[i], vertices[(i + 1) % 5], EdgeCurve::Circle(c));
                        e.set_trim(Some(domain));
                        OrientedEdge::new(topo.add_edge(e), true)
                    })
                    .collect();
                let reverse: Vec<_> = edges
                    .iter()
                    .rev()
                    .map(|e| OrientedEdge::new(e.edge(), false))
                    .collect();
                for winding in [edges, reverse] {
                    let wire = topo.add_wire(Wire::new(winding, true).unwrap());
                    let face =
                        topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(t.clone())));
                    assert!(rectangular_torus_domain(&topo, face, Tolerance::default()).is_none());
                    let trim = trimmed_torus_polygon(&topo, face, Tolerance::default()).unwrap();
                    let middle = unwrap_uv(
                        remus_math::vec::Point2::new(um, f64::midpoint(v0, v1)),
                        trim.centre,
                    );
                    assert!(crate::builder::classify_2d::point_in_polygon_2d(
                        middle,
                        &trim.polygon
                    ));
                    assert!(
                        crate::builder::classify_2d::distance_to_polygon_boundary(
                            middle,
                            &trim.polygon
                        ) > 0.49
                    );
                    let outside = unwrap_uv(
                        remus_math::vec::Point2::new(u1 + 0.1, v0 + 0.5),
                        trim.centre,
                    );
                    assert!(!crate::builder::classify_2d::point_in_polygon_2d(
                        outside,
                        &trim.polygon
                    ));
                    let holed =
                        topo.add_face(Face::new(wire, vec![wire], FaceSurface::Torus(t.clone())));
                    assert!(trimmed_torus_polygon(&topo, holed, Tolerance::default()).is_none());
                }
            }
        }
    }
}
