//! Thin wrappers around `brepkit-blend` for the operations API.

pub use brepkit_blend::BlendError;
use brepkit_blend::BlendResult;
use brepkit_blend::chamfer_builder::ChamferBuilder;
use brepkit_blend::fillet_builder::FilletBuilder;
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::face::FaceSurface;
use brepkit_topology::solid::SolidId;

use crate::OperationsError;

/// Run a blend attempt transactionally: on any failure, roll the arena back
/// to its pre-attempt state.
///
/// The blend engines mutate shared topology in place — `trim_face`'s
/// `propagate_split` rewrites the wires of every face referencing a split
/// edge, and the stitched assembly rewrites cap wires. A failure partway
/// through therefore leaves the INPUT solid mutated: trimmed side faces,
/// arcs where sharp corners were, and free edges from half-applied splits.
/// A caller that reports the failure and keeps using its original solid
/// handle (as the OpenZCAD adapter does) then ships a corrupted body that
/// meshes with holes. Snapshot/restore makes a failed blend a true no-op.
///
/// Handle slots are preserved so IDs handed out before the attempt stay
/// valid after a rollback.
fn transactional<T>(
    topo: &mut Topology,
    attempt: impl FnOnce(&mut Topology) -> Result<T, OperationsError>,
) -> Result<T, OperationsError> {
    let snapshot = topo.clone();
    match attempt(topo) {
        Ok(value) => Ok(value),
        Err(e) => {
            topo.restore_preserving_handle_slots(&snapshot);
            Err(e)
        }
    }
}

/// Classify a blended edge as convex (material on the inside of the dihedral)
/// or concave, by testing a point just inside the inward normal bisector.
///
/// Returns `None` when the edge's neighbourhood cannot be classified
/// (non-manifold edge, degenerate normals, on-boundary sample).
fn edge_is_convex(topo: &Topology, solid: SolidId, edge: EdgeId, probe: f64) -> Option<bool> {
    let adjacency = topo.build_adjacency(solid).ok()?;
    let faces = adjacency.faces_for_edge(edge);
    if faces.len() != 2 {
        return None;
    }
    let e = topo.edge(edge).ok()?;
    let start = topo.vertex(e.start()).ok()?.point();
    let end = topo.vertex(e.end()).ok()?.point();
    let mid = e.curve().evaluate_with_endpoints(
        match e.curve() {
            EdgeCurve::Line => 0.5,
            other => {
                let (t0, t1) = other.domain_with_endpoints(start, end);
                f64::midpoint(t0, t1)
            }
        },
        start,
        end,
    );

    let outward = |fid: brepkit_topology::face::FaceId| {
        let face = topo.face(fid).ok()?;
        let (u, v) = face.surface().project_point(mid)?;
        let n = face.surface().normal(u, v);
        let n = if face.is_reversed() { -n } else { n };
        n.normalize().ok()
    };
    let n1 = outward(faces[0])?;
    let n2 = outward(faces[1])?;
    let bisector = (n1 + n2).normalize().ok()?;

    // Step inward along the bisector. Inside the material ⇒ convex edge.
    let sample = mid - bisector * probe;
    match crate::classify::classify_point_robust(topo, solid, sample, 0.01, 1e-7).ok()? {
        crate::classify::PointClassification::Inside => Some(true),
        crate::classify::PointClassification::Outside => Some(false),
        crate::classify::PointClassification::OnBoundary => None,
    }
}

/// Reject a blend whose volume change is geometrically impossible.
///
/// A blend only moves material inside a tube of radius `size` around each
/// blended edge, so `|Δvolume|` is bounded by `size²·length` per edge plus
/// `2·size³` of end effects. And the sign is fixed by convexity: rounding a
/// convex edge cuts material away, a concave one fills it in. A result that
/// breaks either rule is wrong even when it is a topologically valid closed
/// solid — the failure mode a wrong-side trim produces, which the shell and
/// Euler checks alone accept.
fn validate_blend_volume(
    topo: &Topology,
    operation: &'static str,
    input_solid: SolidId,
    result_solid: SolidId,
    edges: &[EdgeId],
    size: f64,
) -> Result<(), OperationsError> {
    let before = crate::measure::solid_volume(topo, input_solid, 0.1)?;
    let after = crate::measure::solid_volume(topo, result_solid, 0.1)?;
    let delta = after - before;

    let mut budget = 0.0;
    for &edge in edges {
        let e = topo.edge(edge)?;
        let start = topo.vertex(e.start())?.point();
        let end = topo.vertex(e.end())?.point();
        let length = if e.start() == e.end() {
            // Closed edge: use the curve's own extent.
            let (t0, t1) = e.curve().domain_with_endpoints(start, end);
            let mut len = 0.0;
            let mut prev = e.curve().evaluate_with_endpoints(t0, start, end);
            for i in 1..=32 {
                let t = t0 + (t1 - t0) * f64::from(i) / 32.0;
                let p = e.curve().evaluate_with_endpoints(t, start, end);
                len += (p - prev).length();
                prev = p;
            }
            len
        } else {
            (end - start).length()
        };
        budget += size * size * length + 2.0 * size * size * size;
    }

    if delta.abs() > budget {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} changed volume by {delta:+.3}, beyond the {budget:.3} \
                 a blend of this size can move — the result is geometrically wrong"
            ),
        });
    }

    // Sign rule, applied only when every blended edge shares one convexity
    // (a mixed set can legitimately net out either way).
    let convexities: Vec<bool> = edges
        .iter()
        .filter_map(|&e| edge_is_convex(topo, input_solid, e, size * 0.25))
        .collect();
    if convexities.len() == edges.len() && !convexities.is_empty() {
        let all_convex = convexities.iter().all(|&c| c);
        let all_concave = convexities.iter().all(|&c| !c);
        // Allow a hair of tessellation noise either way.
        let noise = budget * 1e-3;
        if all_convex && delta > noise {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "{operation} on convex edges added {delta:+.3} of material; \
                     rounding a convex edge must remove it"
                ),
            });
        }
        if all_concave && delta < -noise {
            return Err(OperationsError::InvalidInput {
                reason: format!(
                    "{operation} on concave edges removed {:.3} of material; \
                     rounding a concave edge must add it",
                    -delta
                ),
            });
        }
    }

    Ok(())
}

/// Per-check error magnitudes of a solid's validation report: the summed
/// deviation (or 1 per issue when absent) of Error-severity issues.
fn error_magnitudes(
    topo: &Topology,
    solid: SolidId,
) -> Result<std::collections::HashMap<brepkit_check::validate::CheckId, f64>, OperationsError> {
    let report = brepkit_check::validate::validate_solid(
        topo,
        solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )?;
    let mut map = std::collections::HashMap::new();
    for issue in &report.issues {
        if issue.severity == brepkit_check::validate::Severity::Error {
            *map.entry(issue.check).or_insert(0.0) += issue.deviation.unwrap_or(1.0);
        }
    }
    Ok(map)
}

/// Validate the blend result against the INPUT solid as a baseline: defects
/// already present in the input (e.g. boolean-inherited orientation quirks
/// on closed circle edges) do not fail the blend; only regressions do.
fn validate_complete_blend(
    topo: &Topology,
    operation: &'static str,
    input_solid: SolidId,
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
    if report.is_valid() {
        return Ok(());
    }
    let after = {
        let mut map = std::collections::HashMap::new();
        for issue in &report.issues {
            if issue.severity == brepkit_check::validate::Severity::Error {
                *map.entry(issue.check).or_insert(0.0) += issue.deviation.unwrap_or(1.0);
            }
        }
        map
    };
    let before = error_magnitudes(topo, input_solid)?;
    let regressed = after
        .iter()
        .any(|(check, &mag)| mag > before.get(check).copied().unwrap_or(0.0));
    if regressed {
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
    validate_complete_blend(topo, "chamfer", solid, &result)?;
    // The fast path gets the same volume guard as the walking path. Closedness
    // and manifoldness alone do not prove a bevel is right: a setback that
    // overruns its face folds the polygon through itself and still validates.
    validate_blend_volume(topo, "chamfer", solid, result_solid, edges, d1.max(d2))?;
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
    validate_complete_blend(topo, "fillet", solid, &result)?;
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
        // The rolling-ball rebuild handles the validated planar classes
        // (simple prisms) and closes multi-edge corner patches. On richer
        // topology (L-shaped side faces, coplanar slivers, holed caps) it
        // emits an open shell; fall through to the walking builder, whose
        // stitched planar assembly handles those shapes. Each attempt is
        // transactional, so the fall-through starts from a clean arena.
        match transactional(topo, |t| planar_fillet_result(t, solid, edges, radius)) {
            Ok(result) => return Ok(result),
            Err(e) => {
                log::warn!("planar fillet fast path failed ({e}); falling back to walking builder");
            }
        }
    }
    transactional(topo, |t| {
        let mut builder = FilletBuilder::new(t, solid);
        builder.add_edges(edges, radius);
        let result = builder.build()?;
        validate_complete_blend(t, "fillet", solid, &result)?;
        validate_blend_volume(t, "fillet", solid, result.solid, edges, radius)?;
        Ok(result)
    })
}

/// Stable machine-readable code for a blend failure.
///
/// Consumers on the far side of the WASM boundary (e.g. the OpenZCAD
/// adapter) receive errors as strings, so the bindings prefix messages with
/// this code to let callers branch on the cause without matching prose.
/// Codes are API: never rename one, only add.
#[must_use]
pub fn blend_failure_code(error: &OperationsError) -> &'static str {
    match error {
        OperationsError::Blend(BlendError::UnsupportedVertexBlend { .. }) => {
            "unsupported-vertex-blend"
        }
        OperationsError::Blend(BlendError::TrimmingFailure { .. }) => "trimming-failure",
        OperationsError::Blend(BlendError::RadiusTooLarge { .. }) => "radius-too-large",
        OperationsError::Blend(BlendError::CornerFailure { .. }) => "corner-failure",
        OperationsError::Blend(BlendError::UnsupportedSurface { .. }) => "unsupported-surface",
        OperationsError::Blend(_) => "blend-failed",
        OperationsError::PartialResult { .. } => "partial-result",
        OperationsError::InvalidInput { .. } => "invalid-input",
        _ => "fillet-failed",
    }
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
    if is_planar_line_blend(topo, solid, edges)? {
        return transactional(topo, |t| planar_chamfer_result(t, solid, edges, d1, d2));
    }
    transactional(topo, |t| {
        let mut builder = ChamferBuilder::new(t, solid);
        builder.add_edges_asymmetric(edges, d1, d2);
        let result = builder.build()?;
        validate_complete_blend(t, "chamfer", solid, &result)?;
        validate_blend_volume(t, "chamfer", solid, result.solid, edges, d1.max(d2))?;
        Ok(result)
    })
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
    let d2 = distance * angle.tan();
    if is_planar_line_blend(topo, solid, edges)? {
        return transactional(topo, |t| {
            planar_chamfer_result(t, solid, edges, distance, d2)
        });
    }
    transactional(topo, |t| {
        let mut builder = ChamferBuilder::new(t, solid);
        builder.add_edges_distance_angle(edges, distance, angle);
        let result = builder.build()?;
        validate_complete_blend(t, "chamfer", solid, &result)?;
        validate_blend_volume(t, "chamfer", solid, result.solid, edges, distance.max(d2))?;
        Ok(result)
    })
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
