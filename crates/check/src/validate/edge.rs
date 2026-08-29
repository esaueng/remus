//! Edge geometric validation checks.

use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceId;
use remus_topology::validation::CurveUseValidationError;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

fn curve_use_error_issue(error: &CurveUseValidationError, edge_id: EdgeId) -> ValidationIssue {
    use remus_math::diagnostic::ToDiagnostic;

    let diagnostic = error.diagnostic();
    ValidationIssue {
        check: CheckId::EdgeSameParameter,
        severity: Severity::Error,
        entity: EntityRef::Edge(edge_id),
        description: format!("{}: {}", diagnostic.code(), diagnostic.message()),
        deviation: None,
    }
}

/// Check that an edge's parameter range is valid (non-degenerate).
pub fn check_edge_range(
    topo: &Topology,
    edge_id: EdgeId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let edge = topo.edge(edge_id)?;
    match edge.curve() {
        EdgeCurve::Line => Ok(vec![]), // Line geometry defined by vertices
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => Ok(vec![]), // Full curves, always valid
        // Hyperbola and parabola branches are unbounded, so the edge's
        // whole extent comes from its vertices. Compare the chord (a
        // length) against the linear tolerance (a length) rather than the
        // parameter span: hyperbola parameters are dimensionless and
        // parabola parameters carry units of length, so a parameter-space
        // threshold would not be scale-invariant across the two.
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
            let p0 = topo.vertex(edge.start())?.point();
            let p1 = topo.vertex(edge.end())?.point();
            let chord = (p1 - p0).length();
            if chord < tolerance {
                return Ok(vec![ValidationIssue {
                    check: CheckId::EdgeRangeValid,
                    severity: Severity::Error,
                    entity: EntityRef::Edge(edge_id),
                    description: format!(
                        "{} edge has zero extent: endpoints coincide (chord {chord:.3e})",
                        edge.curve().type_tag()
                    ),
                    deviation: Some(chord),
                }]);
            }
            Ok(vec![])
        }
        EdgeCurve::NurbsCurve(nc) => {
            let (t0, t1) = nc.domain();
            if (t1 - t0).abs() < tolerance {
                return Ok(vec![ValidationIssue {
                    check: CheckId::EdgeRangeValid,
                    severity: Severity::Error,
                    entity: EntityRef::Edge(edge_id),
                    description: format!("edge NURBS domain [{t0}, {t1}] has zero extent"),
                    deviation: Some((t1 - t0).abs()),
                }]);
            }
            Ok(vec![])
        }
    }
}

/// Check if an edge is degenerate (start == end and near-zero length).
#[allow(clippy::too_many_lines)]
pub fn check_edge_degenerate(
    topo: &Topology,
    edge_id: EdgeId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let edge = topo.edge(edge_id)?;
    if edge.start() != edge.end() {
        return Ok(vec![]);
    }

    // Closed edges (full circles) are not degenerate
    match edge.curve() {
        EdgeCurve::Circle(_) | EdgeCurve::Ellipse(_) => return Ok(vec![]),
        // Unlike circles and ellipses, a hyperbola branch or parabola can
        // never close, so `start == end` always means a zero-extent arc —
        // no length sampling is needed to know that.
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
            return Ok(vec![ValidationIssue {
                check: CheckId::EdgeDegenerate,
                severity: Severity::Warning,
                entity: EntityRef::Edge(edge_id),
                description: format!(
                    "degenerate {} edge: start and end vertex are the same, \
                     but an unbounded conic branch never closes",
                    edge.curve().type_tag()
                ),
                deviation: Some(0.0),
            }]);
        }
        EdgeCurve::Line => {
            let p0 = topo.vertex(edge.start())?.point();
            let p1 = topo.vertex(edge.end())?.point();
            let len = (p1 - p0).length();
            if len < tolerance {
                return Ok(vec![ValidationIssue {
                    check: CheckId::EdgeDegenerate,
                    severity: Severity::Warning,
                    entity: EntityRef::Edge(edge_id),
                    description: format!("degenerate line edge: length {len:.2e}"),
                    deviation: Some(len),
                }]);
            }
        }
        EdgeCurve::NurbsCurve(nc) => {
            let (t0, t1) = nc.domain();
            let n_samples = 10;
            let mut length = 0.0;
            let mut prev = nc.evaluate(t0);
            #[allow(clippy::cast_precision_loss)]
            for i in 1..=n_samples {
                let t = t0 + (t1 - t0) * (i as f64) / (n_samples as f64);
                let curr = nc.evaluate(t);
                length += (curr - prev).length();
                prev = curr;
            }
            if length < tolerance {
                return Ok(vec![ValidationIssue {
                    check: CheckId::EdgeDegenerate,
                    severity: Severity::Warning,
                    entity: EntityRef::Edge(edge_id),
                    description: format!("degenerate NURBS edge: length {length:.2e}"),
                    deviation: Some(length),
                }]);
            }
        }
    }
    Ok(vec![])
}

/// Check that an edge's 3D curve matches its PCurve(surface) within tolerance.
///
/// Samples N points along the edge, evaluates both the 3D curve and the
/// PCurve projected through the surface, and measures the maximum deviation.
#[allow(clippy::cast_precision_loss)]
pub fn check_edge_same_parameter(
    topo: &Topology,
    edge_id: EdgeId,
    face_id: FaceId,
    forward: bool,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let mut issues = Vec::new();
    let report = match remus_topology::validation::check_same_parameter_strict(
        topo, edge_id, face_id, forward, 10,
    ) {
        Ok(report) => report,
        Err(CurveUseValidationError::Topology(error)) => return Err(error.into()),
        Err(error) => return Ok(vec![curve_use_error_issue(&error, edge_id)]),
    };
    if let Some(report) = report
        && report.max_deviation > tolerance
    {
        issues.push(ValidationIssue {
            check: CheckId::EdgeSameParameter,
            severity: Severity::Error,
            entity: EntityRef::Edge(edge_id),
            description: format!(
                "SameParameter deviation {:.2e} exceeds tolerance {tolerance:.2e} on the {} use",
                report.max_deviation,
                if forward { "forward" } else { "reversed" }
            ),
            deviation: Some(report.max_deviation),
        });
    }

    let max_deviation = match remus_topology::validation::check_same_range_strict(
        topo, edge_id, face_id, forward,
    ) {
        Ok(max_deviation) => max_deviation,
        Err(CurveUseValidationError::Topology(error)) => return Err(error.into()),
        Err(error) => return Ok(vec![curve_use_error_issue(&error, edge_id)]),
    };
    if let Some(max_deviation) = max_deviation
        && max_deviation > tolerance
    {
        issues.push(ValidationIssue {
            check: CheckId::EdgeSameParameter,
            severity: Severity::Error,
            entity: EntityRef::Edge(edge_id),
            description: format!(
                "SameRange deviation {max_deviation:.2e} exceeds tolerance {tolerance:.2e} on the {} use",
                if forward { "forward" } else { "reversed" }
            ),
            deviation: Some(max_deviation),
        });
    }

    Ok(issues)
}
