//! Closed-form vector area of a translating region's construction boundary.

use std::collections::{BTreeMap, HashSet};

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceId;
use remus_topology::vertex::VertexId;

use crate::OperationsError;

/// Boundary-derived volume guard; this is not a surface-area measurement.
pub(super) struct BoundaryVolumeChange {
    pub volume: f64,
    pub uncertainty: f64,
}

/// A curve's exact vector-area integral, relative to `origin`.
/// Unsupported carriers decline rather than being chorded.
fn edge_area(
    topo: &Topology,
    edge: EdgeId,
    origin: Point3,
) -> Result<Option<(Vec3, f64)>, OperationsError> {
    let edge = topo.edge(edge)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let area = match edge.curve() {
        EdgeCurve::Line => (start - origin).cross(end - origin),
        EdgeCurve::Circle(circle) => {
            let Ok((a, b)) = edge.strict_domain() else {
                return Ok(None);
            };
            let chord = circle.evaluate(b) - circle.evaluate(a);
            (circle.center() - origin).cross(chord)
                + circle.normal() * (circle.radius() * circle.radius() * (b - a))
        }
        EdgeCurve::NurbsCurve(curve) if curve.degree() == 1 => {
            let Ok((a, b)) = edge.strict_domain() else {
                return Ok(None);
            };
            if b <= a {
                return Ok(None);
            }
            let mut parameters = vec![a];
            parameters.extend(curve.knots().iter().copied().filter(|&t| t > a && t < b));
            parameters.push(b);
            let mut sum = Vec3::new(0.0, 0.0, 0.0);
            for pair in parameters.windows(2) {
                sum += (curve.evaluate(pair[0]) - origin).cross(curve.evaluate(pair[1]) - origin);
            }
            sum
        }
        _ => return Ok(None),
    };
    if !area.length().is_finite() {
        return Ok(None);
    }
    let tolerance = edge
        .tolerance()
        .unwrap_or(Tolerance::new().linear)
        .max(topo.vertex(edge.start())?.tolerance())
        .max(topo.vertex(edge.end())?.tolerance());
    let radius = (start - origin).length().max((end - origin).length());
    Ok(Some((
        area * 0.5,
        radius.mul_add(tolerance * 4.0, area.length() * f64::EPSILON * 64.0),
    )))
}

/// Stokes' vector area over effective boundary coedges. Internal coedges must
/// cancel exactly and the remaining directed graph must balance at every
/// vertex. The operation already proved every unmoved carrier invariant under
/// this translation; this boundary therefore measures the swept prism's sign.
pub(super) fn volume_change(
    topo: &Topology,
    faces: &HashSet<FaceId>,
    delta: Vec3,
) -> Result<Option<BoundaryVolumeChange>, OperationsError> {
    #[cfg(test)]
    if super::translation_oracle_tests::FORCE_MESH_VOLUME.with(std::cell::Cell::get) {
        return Ok(None);
    }
    let mut uses = BTreeMap::<EdgeId, Vec<bool>>::new();
    for &face in faces {
        let data = topo.face(face)?;
        for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
            for oe in topo.wire(wire)?.edges() {
                uses.entry(oe.edge())
                    .or_default()
                    .push(oe.is_forward() != data.is_reversed());
            }
        }
    }
    let mut boundary = Vec::new();
    for (edge, directions) in uses {
        match directions.as_slice() {
            [forward] => boundary.push((edge, *forward)),
            [a, b] if a != b => {}
            _ => return Ok(None),
        }
    }
    let Some(&(first, _)) = boundary.first() else {
        return Ok(None);
    };
    let origin = topo.vertex(topo.edge(first)?.start())?.point();
    let mut flow = BTreeMap::<VertexId, i32>::new();
    let mut area = Vec3::new(0.0, 0.0, 0.0);
    let mut error = 0.0;
    for (edge, forward) in boundary {
        let data = topo.edge(edge)?;
        let (start, end) = if forward {
            (data.start(), data.end())
        } else {
            (data.end(), data.start())
        };
        *flow.entry(start).or_default() += 1;
        *flow.entry(end).or_default() -= 1;
        let Some((value, bound)) = edge_area(topo, edge, origin)? else {
            return Ok(None);
        };
        area += if forward { value } else { -value };
        error += bound;
    }
    if flow.values().any(|&count| count != 0) {
        return Ok(None);
    }
    let volume = delta.dot(area);
    let uncertainty = delta.length() * error;
    if !volume.is_finite() || volume.abs() <= uncertainty.max(1e-7) {
        return Ok(None);
    }
    Ok(Some(BoundaryVolumeChange {
        volume,
        uncertainty,
    }))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    use super::*;
    use crate::primitives::{make_box, make_cylinder};
    use remus_math::mat::Mat4;
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_topology::explorer::{face_edges, solid_faces};

    #[test]
    fn rectangle_and_circle_have_closed_form_signed_swept_volumes() {
        for shift in [0.0, 1e7] {
            for circle in [false, true] {
                let mut topo = Topology::new();
                let solid = if circle {
                    make_cylinder(&mut topo, 1.5, 4.0)
                } else {
                    make_box(&mut topo, 2.0, 3.0, 4.0)
                }
                .unwrap();
                crate::transform::transform_solid(
                    &mut topo,
                    solid,
                    &Mat4::translation(shift, -shift, shift),
                )
                .unwrap();
                let face = solid_faces(&topo, solid)
                    .unwrap()
                    .into_iter()
                    .find(|&f| {
                        topo.face(f)
                            .unwrap()
                            .effective_plane_normal()
                            .is_some_and(|n| n.z() > 0.99)
                    })
                    .unwrap();
                for distance in [-0.5, 2.0] {
                    let change =
                        volume_change(&topo, &HashSet::from([face]), Vec3::new(0.0, 0.0, distance))
                            .unwrap()
                            .unwrap();
                    let area = if circle {
                        std::f64::consts::PI * 1.5 * 1.5
                    } else {
                        6.0
                    };
                    assert!((change.volume - area * distance).abs() < area * 1e-10);
                }
            }
        }
    }

    #[test]
    fn polynomial_line_charts_are_integrated_but_higher_degrees_decline() {
        for degree in [1, 2] {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
            let face = solid_faces(&topo, solid)
                .unwrap()
                .into_iter()
                .find(|&f| {
                    topo.face(f)
                        .unwrap()
                        .effective_plane_normal()
                        .is_some_and(|n| n.z() > 0.99)
                })
                .unwrap();
            let edge = face_edges(&topo, face).unwrap()[0];
            let data = topo.edge(edge).unwrap();
            let start = topo.vertex(data.start()).unwrap().point();
            let end = topo.vertex(data.end()).unwrap().point();
            let (knots, points) = if degree == 1 {
                (vec![0.0, 0.0, 1.0, 1.0], vec![start, end])
            } else {
                (
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![start, start + (end - start) * 0.5, end],
                )
            };
            let curve = NurbsCurve::new(degree, knots, points, vec![1.0; degree + 1]).unwrap();
            topo.edge_mut(edge)
                .unwrap()
                .set_curve(EdgeCurve::NurbsCurve(curve));
            topo.edge_mut(edge).unwrap().set_trim(Some((0.0, 1.0)));
            let result =
                volume_change(&topo, &HashSet::from([face]), Vec3::new(0.0, 0.0, 2.0)).unwrap();
            if degree == 1 {
                assert!((result.unwrap().volume - 12.0).abs() < 1e-12);
            } else {
                assert!(result.is_none());
            }
        }
    }
}
