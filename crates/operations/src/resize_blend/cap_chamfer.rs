//! Exact annular chamfers that travel with a planar cap and slide on a bore.

use crate::OperationsError;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::face_edges;
use remus_topology::face::{FaceId, FaceSurface};
use std::f64::consts::TAU;

/// Prove a complete cone band bounded only by the selected cap and one
/// coaxial cylinder. Partial tapers and cones meeting another detail abstain.
pub(super) fn is_axial_cap_chamfer(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    is_selected: impl Fn(FaceId) -> bool,
    face: FaceId,
    direction: Vec3,
) -> Result<bool, OperationsError> {
    let FaceSurface::Cone(cone) = topo.face(face)?.surface() else {
        return Ok(false);
    };
    let tol = Tolerance::new();
    if cone.axis().cross(direction).length() > tol.angular {
        return Ok(false);
    }
    let mut cap_angle = 0.0;
    let mut wall_angle = 0.0;
    let mut carrier = None;
    let mut cap_level: Option<f64> = None;
    let mut wall_level: Option<f64> = None;
    for edge in face_edges(topo, face)? {
        let others = super::distinct_faces(adjacency.faces_for_edge(edge))
            .into_iter()
            .filter(|&other| other != face)
            .collect::<Vec<_>>();
        // A periodic seam belongs to this cone on both sides.
        if others.is_empty() {
            if !matches!(topo.edge(edge)?.curve(), EdgeCurve::Line) {
                return Ok(false);
            }
            continue;
        }
        let [other] = others.as_slice() else {
            return Ok(false);
        };
        let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() else {
            return Ok(false);
        };
        if circle.normal().cross(direction).length() > tol.angular
            || (circle.center() - cone.apex()).cross(direction).length() > tol.linear
        {
            return Ok(false);
        }
        let level = (circle.center() - cone.apex()).dot(direction);
        let angle = crate::measure::edge_length(topo, edge)? / circle.radius();
        if is_selected(*other) {
            if cap_level.is_some_and(|before| !tol.approx_eq(before, level)) {
                return Ok(false);
            }
            cap_level = Some(level);
            cap_angle += angle;
        } else {
            let FaceSurface::Cylinder(cylinder) = topo.face(*other)?.surface() else {
                return Ok(false);
            };
            if carrier.is_some_and(|before| before != *other)
                || cylinder.axis().cross(direction).length() > tol.angular
                || (cylinder.origin() - cone.apex()).cross(direction).length() > tol.linear
                || wall_level.is_some_and(|before| !tol.approx_eq(before, level))
            {
                return Ok(false);
            }
            carrier = Some(*other);
            wall_level = Some(level);
            wall_angle += angle;
        }
    }
    Ok((cap_angle - TAU).abs() <= tol.angular * TAU
        && (wall_angle - TAU).abs() <= tol.angular * TAU
        && cap_level
            .zip(wall_level)
            .is_some_and(|(cap, wall)| (cap - wall).abs() > tol.linear))
}
