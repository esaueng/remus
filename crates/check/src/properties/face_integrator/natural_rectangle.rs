//! Natural degree-(2,1) domains witnessed by every complete boundary image.

use remus_math::compact_sort::CompactSort;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;

use crate::CheckError;

fn clamped(knots: &[f64], degree: usize, range: (f64, f64)) -> bool {
    knots
        .iter()
        .take(degree + 1)
        .all(|&x| x.to_bits() == range.0.to_bits())
        && knots
            .iter()
            .rev()
            .take(degree + 1)
            .all(|&x| x.to_bits() == range.1.to_bits())
}

fn close_point(a: Point3, b: Point3, tolerance: f64) -> bool {
    (a - b).length() <= tolerance
}

fn same_curve(a: &NurbsCurve, b: &NurbsCurve, tolerance: f64) -> bool {
    a.degree() == b.degree()
        && a.knots() == b.knots()
        && a.weights() == b.weights()
        && a.control_points().len() == b.control_points().len()
        && a.control_points()
            .iter()
            .zip(b.control_points())
            .all(|(&a, &b)| close_point(a, b, tolerance))
}

/// A quadratic iso section is a minor circular arc, not an arbitrary conic.
fn circular_section_matches(
    iso: &NurbsCurve,
    circle: &remus_math::curves::Circle3D,
    range: (f64, f64),
    tolerance: f64,
) -> bool {
    let points = iso.control_points();
    let weights = iso.weights();
    if iso.degree() != 2
        || points.len() != 3
        || weights.len() != 3
        || !clamped(iso.knots(), 2, iso.domain())
        || weights[0].to_bits() != weights[2].to_bits()
        || weights[0] <= 0.0
    {
        return false;
    }
    let w = weights[1] / weights[0];
    if !(0.05..0.999_999).contains(&w) {
        return false;
    }
    let midpoint = points[0] + (points[2] - points[0]) * 0.5;
    let center = midpoint + (midpoint - points[1]) * (w * w / (1.0 - w * w));
    let r = (points[0] - center).length();
    if !r.is_finite()
        || r <= tolerance
        || (r - (points[2] - center).length()).abs() > tolerance
        || ((midpoint - center).length() - r * w).abs() > tolerance
        || !close_point(center, circle.center(), tolerance)
        || (circle.radius() - r).abs() > tolerance
    {
        return false;
    }
    let a = points[0] - center;
    let b = points[2] - center;
    let angle = a.cross(b).length().atan2(a.dot(b));
    if (angle - (range.1 - range.0).abs()).abs() > 128.0 * f64::EPSILON {
        return false;
    }
    let normal = match a.cross(b).normalize() {
        Ok(n) => n,
        Err(_) => return false,
    };
    if normal.cross(circle.normal()).length() > 128.0 * f64::EPSILON {
        return false;
    }
    let first = circle.evaluate(range.0);
    let last = circle.evaluate(range.1);
    let ends = (close_point(first, points[0], tolerance)
        && close_point(last, points[2], tolerance))
        || (close_point(first, points[2], tolerance) && close_point(last, points[0], tolerance));
    ends && close_point(
        circle.evaluate(f64::midpoint(range.0, range.1)),
        iso.evaluate(f64::midpoint(iso.domain().0, iso.domain().1)),
        tolerance,
    )
}

/// Complete side coverage is a domain witness; no sampled projection decides it.
pub(super) fn is_complete(
    topo: &Topology,
    face: FaceId,
    s: &NurbsSurface,
) -> Result<bool, CheckError> {
    let f = topo.face(face)?;
    if !f.inner_wires().is_empty()
        || s.degree_u() != 2
        || s.degree_v() != 1
        || s.control_points().len() != 3
        || s.control_points()[0].len() < 2
        || s.is_periodic_u()
        || s.is_periodic_v()
        || !clamped(s.knots_u(), 2, s.domain_u())
        || !clamped(s.knots_v(), 1, s.domain_v())
    {
        return Ok(false);
    }
    let points = s.control_points();
    let weights = s.weights();
    let n = points[0].len();
    let magnitude = points
        .iter()
        .flatten()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(1.0_f64, f64::max);
    let tolerance = (128.0 * f64::EPSILON * magnitude).min(1e-7);
    let iso = [
        NurbsCurve::new(
            1,
            s.knots_v().to_vec(),
            points[0].clone(),
            weights[0].clone(),
        )?,
        NurbsCurve::new(
            1,
            s.knots_v().to_vec(),
            points[2].clone(),
            weights[2].clone(),
        )?,
        NurbsCurve::new(
            2,
            s.knots_u().to_vec(),
            points.iter().map(|r| r[0]).collect(),
            weights.iter().map(|r| r[0]).collect(),
        )?,
        NurbsCurve::new(
            2,
            s.knots_u().to_vec(),
            points.iter().map(|r| r[n - 1]).collect(),
            weights.iter().map(|r| r[n - 1]).collect(),
        )?,
    ];
    let mut coverage: [Vec<(f64, f64)>; 4] = std::array::from_fn(|_| Vec::new());
    let wire = topo.wire(f.outer_wire())?;
    if remus_topology::validation::validate_wire_closed(wire, topo).is_err() {
        return Ok(false);
    }
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        let Ok(range) = e.strict_domain() else {
            return Ok(false);
        };
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        if !close_point(
            e.curve().evaluate_with_endpoints(range.0, start, end),
            start,
            tolerance,
        ) || !close_point(
            e.curve().evaluate_with_endpoints(range.1, start, end),
            end,
            tolerance,
        ) {
            return Ok(false);
        }
        let mut claims = Vec::new();
        for (side, curve) in iso.iter().enumerate() {
            match e.curve() {
                EdgeCurve::NurbsCurve(edge_curve) if same_curve(curve, edge_curve, tolerance) => {
                    let (a, b) = curve.domain();
                    let low = range.0.min(range.1);
                    let high = range.0.max(range.1);
                    if low >= a && high <= b && high > low {
                        claims.push((side, (low, high)));
                    }
                }
                EdgeCurve::Circle(circle)
                    if side >= 2 && circular_section_matches(curve, circle, range, tolerance) =>
                {
                    claims.push((side, curve.domain()));
                }
                EdgeCurve::Line
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Circle(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {}
            }
        }
        let [(side, interval)] = claims.as_slice() else {
            return Ok(false);
        };
        coverage[*side].push(*interval);
    }
    for (side, spans) in coverage.iter_mut().enumerate() {
        spans.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let domain = iso[side].domain();
        let mut end = domain.0;
        for &(a, b) in spans.iter() {
            if (a - end).abs() > 128.0 * f64::EPSILON * domain.1.abs().max(1.0) {
                return Ok(false);
            }
            end = b;
        }
        if spans.is_empty()
            || (end - domain.1).abs() > 128.0 * f64::EPSILON * domain.1.abs().max(1.0)
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::cast_precision_loss
    )]
    use super::*;
    use remus_math::curves::Circle3D;
    use remus_math::vec::Vec3;
    use remus_topology::edge::Edge;
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    fn quarter_cylinder(split: bool) -> (Topology, FaceId, NurbsSurface) {
        let mut t = Topology::new();
        let n = 514;
        let z = |j: usize| 4.0 * j as f64 / (n - 1) as f64;
        let cp = vec![
            (0..n).map(|j| Point3::new(2.0, 0.0, z(j))).collect(),
            (0..n).map(|j| Point3::new(2.0, 2.0, z(j))).collect(),
            (0..n).map(|j| Point3::new(0.0, 2.0, z(j))).collect(),
        ];
        let weights = vec![
            vec![1.0; n],
            vec![std::f64::consts::FRAC_1_SQRT_2; n],
            vec![1.0; n],
        ];
        let mut kv = vec![0.0, 0.0];
        kv.extend((1..n - 1).map(|j| j as f64 / (n - 1) as f64));
        kv.extend([1.0, 1.0]);
        let surface = NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            kv.clone(),
            cp.clone(),
            weights.clone(),
        )
        .unwrap();
        let points = [
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 4.0),
            Point3::new(2.0, 0.0, 4.0),
        ];
        let v: Vec<_> = points
            .into_iter()
            .map(|p| t.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let mut add = |a, b, curve, range| {
            let mut e = Edge::new(a, b, curve);
            e.set_trim(Some(range));
            t.add_edge(e)
        };
        let bottom = add(
            v[0],
            v[1],
            EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
            (0.0, std::f64::consts::FRAC_PI_2),
        );
        let right = add(
            v[1],
            v[2],
            EdgeCurve::NurbsCurve(
                NurbsCurve::new(1, kv.clone(), cp[2].clone(), weights[2].clone()).unwrap(),
            ),
            (0.0, 1.0),
        );
        let top = add(
            v[3],
            v[2],
            EdgeCurve::Circle(
                Circle3D::new_with_ref(
                    Point3::new(0.0, 0.0, 4.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    2.0,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
            ),
            (0.0, std::f64::consts::FRAC_PI_2),
        );
        let curve = NurbsCurve::new(1, kv, cp[0].clone(), weights[0].clone()).unwrap();
        let mut wire = vec![
            OrientedEdge::new(bottom, true),
            OrientedEdge::new(right, true),
            OrientedEdge::new(top, false),
        ];
        if split {
            let middle = t.add_vertex(Vertex::new(curve.evaluate(0.37), 1e-7));
            let mut a = Edge::new(v[0], middle, EdgeCurve::NurbsCurve(curve.clone()));
            a.set_trim(Some((0.0, 0.37)));
            let a = t.add_edge(a);
            let mut b = Edge::new(middle, v[3], EdgeCurve::NurbsCurve(curve));
            b.set_trim(Some((0.37, 1.0)));
            let b = t.add_edge(b);
            wire.extend([OrientedEdge::new(b, false), OrientedEdge::new(a, false)]);
        } else {
            let mut e = Edge::new(v[0], v[3], EdgeCurve::NurbsCurve(curve));
            e.set_trim(Some((0.0, 1.0)));
            wire.push(OrientedEdge::new(t.add_edge(e), false));
        }
        let wire = t.add_wire(Wire::new(wire, true).unwrap());
        let face = t.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
        (t, face, surface)
    }

    #[test]
    fn dense_natural_rectangle_and_split_row_match_cylinder_area() {
        for split in [false, true] {
            let (t, face, s) = quarter_cylinder(split);
            assert!(is_complete(&t, face, &s).unwrap());
            let area = super::super::integrate_face_area(&t, face, 5).unwrap();
            assert!((area - 4.0 * std::f64::consts::PI).abs() < 1e-8, "{area}");
        }
    }

    #[test]
    fn holes_partial_sections_and_changed_row_carriers_decline() {
        let (mut t, face, s) = quarter_cylinder(false);
        let outer = t.face(face).unwrap().outer_wire();
        t.set_face_boundary_wires(face, outer, vec![outer]).unwrap();
        assert!(!is_complete(&t, face, &s).unwrap());
        t.set_face_boundary_wires(face, outer, vec![]).unwrap();
        let edge = t.wire(outer).unwrap().edges()[0].edge();
        t.edge_mut(edge)
            .unwrap()
            .set_trim(Some((0.1, std::f64::consts::FRAC_PI_2)));
        assert!(!is_complete(&t, face, &s).unwrap());
        let (t, face, s) = quarter_cylinder(false);
        let mut points = s.control_points().to_vec();
        points[0][2] = points[0][2] + Vec3::new(0.02, 0.0, 0.0);
        let changed = NurbsSurface::new(
            2,
            1,
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            points,
            s.weights().to_vec(),
        )
        .unwrap();
        assert!(!is_complete(&t, face, &changed).unwrap());
    }
}
