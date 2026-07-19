//! Thin wrappers around `brepkit-blend` for the operations API.

use brepkit_blend::BlendResult;
use brepkit_blend::chamfer_builder::ChamferBuilder;
use brepkit_blend::fillet_builder::FilletBuilder;
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::face::FaceSurface;
use brepkit_topology::solid::SolidId;

use crate::OperationsError;

fn validate_complete_blend(
    topo: &Topology,
    operation: &'static str,
    result: &BlendResult,
) -> Result<(), OperationsError> {
    if result.is_partial {
        return Err(OperationsError::PartialResult {
            operation,
            succeeded: result.succeeded.len(),
            failed: result.failed.len(),
        });
    }
    let report = brepkit_check::validate::validate_solid(
        topo,
        result.solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )?;
    if !report.is_valid() {
        let summary = report
            .issues
            .iter()
            .filter(|issue| issue.severity == brepkit_check::validate::Severity::Error)
            .take(3)
            .map(|issue| issue.description.as_str())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} postcondition validation failed with {} error(s): {summary}",
                report.error_count(),
            ),
        });
    }
    Ok(())
}

/// Return whether every requested edge is a manifold line between two planar
/// faces. These inputs are handled by the polygon-rebuilding chamfer path,
/// which also closes the two end faces of a finite chamfer. The walking
/// builder remains necessary for analytic curved edges and surfaces.
fn is_planar_line_blend(
    topo: &Topology,
    solid: SolidId,
    edges: &[EdgeId],
) -> Result<bool, OperationsError> {
    let adjacency = topo.build_adjacency(solid)?;

    for &edge_id in edges {
        if !matches!(topo.edge(edge_id)?.curve(), EdgeCurve::Line) {
            return Ok(false);
        }

        let faces = adjacency.faces_for_edge(edge_id);
        if faces.len() != 2 {
            return Ok(false);
        }
        for &face_id in faces {
            if !matches!(topo.face(face_id)?.surface(), FaceSurface::Plane { .. }) {
                return Ok(false);
            }
        }
    }

    Ok(true)
}

fn reject_closed_edges(
    topo: &Topology,
    edges: &[EdgeId],
    operation: &'static str,
) -> Result<(), OperationsError> {
    for &edge_id in edges {
        let edge = topo.edge(edge_id)?;
        if edge.start() == edge.end() {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "closed-edge {operation} assembly is not yet supported; refusing to return an invalid solid"
                ),
            });
        }
    }
    Ok(())
}

fn planar_chamfer_result(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<BlendResult, OperationsError> {
    let result_solid = crate::chamfer::chamfer_asymmetric(topo, solid, edges, d1, d2)?;
    let result = BlendResult {
        solid: result_solid,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
    };
    validate_complete_blend(topo, "chamfer", &result)?;
    Ok(result)
}

#[allow(deprecated)]
fn planar_fillet_result(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<BlendResult, OperationsError> {
    let result_solid = crate::fillet::fillet_rolling_ball(topo, solid, edges, radius)?;
    let result = BlendResult {
        solid: result_solid,
        succeeded: edges.to_vec(),
        failed: Vec::new(),
        is_partial: false,
    };
    validate_complete_blend(topo, "fillet", &result)?;
    Ok(result)
}

/// Fillet edges with constant radius (v2 walking-based engine).
///
/// # Errors
/// Returns `OperationsError` if radius is non-positive, edges are empty,
/// or the blend computation fails.
pub fn fillet_v2(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
) -> Result<BlendResult, OperationsError> {
    if radius <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "radius must be positive".into(),
        });
    }
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "no edges specified".into(),
        });
    }
    if is_planar_line_blend(topo, solid, edges)? {
        return planar_fillet_result(topo, solid, edges, radius);
    }
    let mut builder = FilletBuilder::new(topo, solid);
    builder.add_edges(edges, radius);
    let result = builder.build()?;
    validate_complete_blend(topo, "fillet", &result)?;
    Ok(result)
}

/// Chamfer edges with two distances (v2 engine).
///
/// # Errors
/// Returns `OperationsError` if distances are non-positive, edges are empty,
/// or the blend computation fails.
pub fn chamfer_v2(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    d1: f64,
    d2: f64,
) -> Result<BlendResult, OperationsError> {
    if d1 <= 0.0 || d2 <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "distances must be positive".into(),
        });
    }
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "no edges specified".into(),
        });
    }
    reject_closed_edges(topo, edges, "chamfer")?;
    if is_planar_line_blend(topo, solid, edges)? {
        return planar_chamfer_result(topo, solid, edges, d1, d2);
    }
    let mut builder = ChamferBuilder::new(topo, solid);
    builder.add_edges_asymmetric(edges, d1, d2);
    let result = builder.build()?;
    validate_complete_blend(topo, "chamfer", &result)?;
    Ok(result)
}

/// Chamfer edges with distance and angle (v2 engine).
///
/// # Errors
/// Returns `OperationsError` if distance is non-positive, angle is out of
/// range (0, pi/2), edges are empty, or the blend computation fails.
pub fn chamfer_distance_angle(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    distance: f64,
    angle: f64,
) -> Result<BlendResult, OperationsError> {
    if distance <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "distance must be positive".into(),
        });
    }
    if angle <= 0.0 || angle >= std::f64::consts::FRAC_PI_2 {
        return Err(OperationsError::InvalidInput {
            reason: "angle must be between 0 and \u{03c0}/2".into(),
        });
    }
    if edges.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "no edges specified".into(),
        });
    }
    reject_closed_edges(topo, edges, "chamfer")?;
    let d2 = distance * angle.tan();
    if is_planar_line_blend(topo, solid, edges)? {
        return planar_chamfer_result(topo, solid, edges, distance, d2);
    }
    let mut builder = ChamferBuilder::new(topo, solid);
    builder.add_edges_distance_angle(edges, distance, angle);
    let result = builder.build()?;
    validate_complete_blend(topo, "chamfer", &result)?;
    Ok(result)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use brepkit_math::vec::Point3;
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::vertex::Vertex;

    use super::*;

    #[test]
    fn fillet_v2_rejects_all_failed_partial_result() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(10.0, 10.0, 10.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(11.0, 10.0, 10.0), 1e-7));
        let unrelated_edge = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));

        let result = fillet_v2(&mut topo, solid, &[unrelated_edge], 0.2);
        assert!(result.is_err());
        let Err(error) = result else { return };
        assert!(matches!(
            error,
            OperationsError::PartialResult {
                operation: "fillet",
                succeeded: 0,
                failed: 1,
            }
        ));
    }
}
