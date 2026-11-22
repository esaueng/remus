//! A second clearance proof for planar carriers with trimmed circular edges.

use std::f64::consts::TAU;

use remus_geometry::bounds::curve::circle_arc_bounds;
use remus_math::aabb::Aabb3;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::face_edges;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use crate::OperationsError;

/// Subdivide only to tighten conservative boxes. Reaching either budget
/// declines the proof and keeps the existing whole-body closure check.
const MAX_PIECES: usize = 4096;
const MAX_PAIRS: usize = 65536;

fn point_box(point: Point3) -> Aabb3 {
    Aabb3 {
        min: point,
        max: point,
    }
}

fn append_edge_boxes(
    edge: &Edge,
    a: Point3,
    b: Point3,
    tolerance: f64,
    output: &mut Vec<Aabb3>,
) -> Option<()> {
    match edge.curve() {
        EdgeCurve::Line => output.push(Aabb3::from_points([a, b])),
        EdgeCurve::Circle(circle) => {
            let (start, end) = edge.strict_domain().ok()?;
            let span = (end - start).abs();
            // Large parameter arguments lose phase precision. A trim that
            // disagrees with its vertices is insufficient evidence here.
            let first = circle.evaluate(start);
            let last = circle.evaluate(end);
            let endpoints_match = ((first - a).length() <= tolerance
                && (last - b).length() <= tolerance)
                || ((last - a).length() <= tolerance && (first - b).length() <= tolerance);
            if start.abs() > 2.0 * TAU || end.abs() > 2.0 * TAU || span == 0.0 || !endpoints_match {
                return None;
            }
            let parts = [1_u32, 2, 4, 8, 16, 32, 64]
                .into_iter()
                .find(|&n| span <= f64::from(n) * (TAU / 64.0) + f64::EPSILON * TAU)?;
            let mut left = start;
            for n in 1..=parts {
                let right = if n == parts {
                    end
                } else {
                    (end - start).mul_add(f64::from(n) / f64::from(parts), start)
                };
                let bound = circle_arc_bounds(circle, left, right);
                if !bound.is_prunable() {
                    return None;
                }
                output.push(bound.aabb());
                left = right;
            }
            output.extend([point_box(a), point_box(b)]);
        }
        EdgeCurve::Ellipse(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::NurbsCurve(_) => return None,
    }
    Some(())
}

/// In a coordinate-axis translation, the other two coordinates are fixed.
/// Two boxes separated on either transverse axis stay separated throughout
/// the move. Every remaining pair bounds the permissible axial displacement.
/// A stretched line lies on the translation axis, so only its two endpoints
/// can newly cross a fixed or moving boundary (as in the axial proof).
#[expect(
    clippy::float_cmp,
    reason = "clearance requires exactly axial motion and parallel stretching edges"
)]
pub(super) fn clearance(
    topo: &Topology,
    face: FaceId,
    is_moved: impl Fn(VertexId) -> bool,
    direction: Vec3,
) -> Result<Option<(f64, f64)>, OperationsError> {
    if !matches!(topo.face(face)?.surface(), FaceSurface::Plane { .. }) {
        return Ok(None);
    }
    let Some(axis) = direction.0.iter().position(|v| v.abs() == 1.0) else {
        return Ok(None);
    };
    if (0..3).any(|i| i != axis && direction.0[i] != 0.0) {
        return Ok(None);
    }
    let linear = Tolerance::new().linear;
    // Reserve rounding headroom for both the coordinates and the translation.
    let coordinate_limit = linear / (128.0 * f64::EPSILON);
    let mut moving = Vec::new();
    let mut fixed = Vec::new();
    let mut margin = linear;
    let edges = face_edges(topo, face)?;
    if edges.len() > MAX_PIECES {
        return Ok(None);
    }
    for id in edges {
        let edge = topo.edge(id)?;
        let va = topo.vertex(edge.start())?;
        let vb = topo.vertex(edge.end())?;
        let (a, b) = (va.point(), vb.point());
        for t in [
            va.tolerance(),
            vb.tolerance(),
            edge.tolerance().unwrap_or(0.0),
        ] {
            if !t.is_finite() || t < 0.0 {
                return Ok(None);
            }
            margin = margin.max(t);
        }
        if !a
            .0
            .into_iter()
            .chain(b.0)
            .all(|v| v.abs() < coordinate_limit)
        {
            return Ok(None);
        }
        let start_moves = is_moved(edge.start());
        if start_moves == is_moved(edge.end()) {
            let pieces = if start_moves { &mut moving } else { &mut fixed };
            if append_edge_boxes(edge, a, b, margin, pieces).is_none() {
                return Ok(None);
            }
        } else {
            if !matches!(edge.curve(), EdgeCurve::Line)
                || (0..3).any(|i| i != axis && a.0[i] != b.0[i])
            {
                return Ok(None);
            }
            let (from, to) = if start_moves { (a, b) } else { (b, a) };
            moving.push(point_box(from));
            fixed.push(point_box(to));
        }
        if moving.len() + fixed.len() > MAX_PIECES {
            return Ok(None);
        }
    }
    if moving.is_empty() || fixed.is_empty() || moving.len() * fixed.len() > MAX_PAIRS {
        return Ok(None);
    }
    if !moving.iter().chain(&fixed).all(|b| {
        b.min
            .0
            .into_iter()
            .chain(b.max.0)
            .all(|v| v.abs() < coordinate_limit)
    }) {
        return Ok(None);
    }
    let gap = 2.0 * margin + linear;
    let (mut lower, mut upper) = (-coordinate_limit, coordinate_limit);
    for m in moving {
        for f in &fixed {
            if (0..3).any(|i| {
                i != axis && (m.min.0[i] - f.max.0[i] > gap || f.min.0[i] - m.max.0[i] > gap)
            }) {
                continue;
            }
            if m.min.0[axis] - f.max.0[axis] > gap {
                lower = lower.max(f.max.0[axis] - m.min.0[axis] + gap);
            } else if f.min.0[axis] - m.max.0[axis] > gap {
                upper = upper.min(f.min.0[axis] - m.max.0[axis] - gap);
            } else {
                return Ok(None);
            }
        }
    }
    Ok(Some(if direction.0[axis] > 0.0 {
        (lower, upper)
    } else {
        (-upper, -lower)
    }))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use remus_math::curves::Circle3D;
    use remus_topology::face::Face;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    // A plate with a translating semicircular end, two stretching sides and
    // a fixed round hole. At dx = -2 the round end first touches the hole.
    fn plate(hole_x: f64, trim: bool) -> (Topology, FaceId, [VertexId; 2]) {
        let mut topo = Topology::new();
        let vertex =
            |topo: &mut Topology, x, y| topo.add_vertex(Vertex::new(Point3::new(x, y, 0.0), 1e-7));
        let a = vertex(&mut topo, -10.0, -4.0);
        let b = vertex(&mut topo, 5.0, -4.0);
        let c = vertex(&mut topo, 5.0, 4.0);
        let d = vertex(&mut topo, -10.0, 4.0);
        let seam = vertex(&mut topo, hole_x + 1.0, 0.0);
        let circle = |x, r| {
            Circle3D::new_with_ref(
                Point3::new(x, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                r,
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap()
        };
        let mut end = Edge::new(b, c, EdgeCurve::Circle(circle(5.0, 4.0)));
        if trim {
            end.set_trim(Some((-TAU / 4.0, TAU / 4.0)));
        }
        let outer = [
            Edge::new(a, b, EdgeCurve::Line),
            end,
            Edge::new(c, d, EdgeCurve::Line),
            Edge::new(d, a, EdgeCurve::Line),
        ]
        .into_iter()
        .map(|edge| OrientedEdge::new(topo.add_edge(edge), true))
        .collect();
        let outer = topo.add_wire(Wire::new(outer, true).unwrap());
        let mut inner = Edge::new(seam, seam, EdgeCurve::Circle(circle(hole_x, 1.0)));
        inner.set_trim(Some((0.0, TAU)));
        let hole = topo.add_edge(inner);
        let inner = topo.add_wire(Wire::new(vec![OrientedEdge::new(hole, false)], true).unwrap());
        let face = topo.add_face(Face::new(
            outer,
            vec![inner],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        (topo, face, [b, c])
    }

    #[test]
    fn rounded_end_proves_travel_only_before_hole_contact() {
        let (topo, face, moving) = plate(6.0, true);
        for sign in [-1.0, 1.0] {
            let direction = Vec3::new(sign, 0.0, 0.0);
            let (lo, hi) = clearance(&topo, face, |v| moving.contains(&v), direction)
                .unwrap()
                .unwrap();
            let admits = |dx: f64| lo < dx * sign && dx * sign < hi;
            assert!(admits(-1.0));
            assert!(admits(5.0));
            assert!(!admits(-2.0));
            assert!(!admits(-3.0));
        }
    }

    #[test]
    fn existing_contact_and_missing_arc_authority_retain_mesh_validation() {
        for (x, trim) in [(8.0, true), (6.0, false)] {
            let (topo, face, moving) = plate(x, trim);
            assert!(
                clearance(
                    &topo,
                    face,
                    |v| moving.contains(&v),
                    Vec3::new(1.0, 0.0, 0.0)
                )
                .unwrap()
                .is_none()
            );
        }
    }
}
