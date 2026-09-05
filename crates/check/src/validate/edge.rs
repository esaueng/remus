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

/// Check that an open edge's curve runs from its start vertex to its end
/// vertex, not the reverse.
///
/// `check_vertex_on_curve` is deliberately direction-blind (it takes the
/// minimum over both curve ends), so a curve authored opposite to the edge's
/// declared endpoints passes every per-endpoint test; the fault then surfaces
/// downstream as a wire- or face-orientation error, pointing at the wrong
/// entity. This check evaluates the curve at both ends of the edge's
/// authoritative domain and compares each against the start and end vertex
/// positions: when the forward correspondence fails but the reversed one
/// holds, the curve data itself is reversed and the issue names the edge.
///
/// Closed edges carry no vertex-level direction signal and are skipped, as
/// are `Line` edges (the vertices are the geometry) and the unbounded conics
/// (their extent comes from the vertices, so there is no independent curve
/// direction to contradict). Untrimmed open circles and ellipses are skipped
/// too: their domain is a caller convention, not stored data, so there is no
/// authoritative direction to check against.
///
/// # Errors
///
/// Returns an error if any topology entity referenced by the edge is missing.
pub fn check_edge_curve_direction(
    topo: &Topology,
    edge_id: EdgeId,
    tolerance: f64,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let edge = topo.edge(edge_id)?;
    if edge.start() == edge.end() {
        return Ok(vec![]);
    }

    let (t0, t1, evaluate): (f64, f64, Box<dyn Fn(f64) -> remus_math::vec::Point3 + '_>) =
        match edge.curve() {
            EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Ok(vec![]);
            }
            EdgeCurve::NurbsCurve(nc) => {
                let (t0, t1) = edge.trim().unwrap_or_else(|| nc.domain());
                (t0, t1, Box::new(|t| nc.evaluate(t)))
            }
            EdgeCurve::Circle(c) => {
                let Some((t0, t1)) = edge.trim() else {
                    return Ok(vec![]);
                };
                (t0, t1, Box::new(|t| c.evaluate(t)))
            }
            EdgeCurve::Ellipse(e) => {
                let Some((t0, t1)) = edge.trim() else {
                    return Ok(vec![]);
                };
                (t0, t1, Box::new(|t| e.evaluate(t)))
            }
        };

    // A reversed span already declares the anomaly in its trim; reporting the
    // curve itself as reversed on top of it would be noise.
    if t1 <= t0 {
        return Ok(vec![]);
    }

    let p_start = topo.vertex(edge.start())?.point();
    let p_end = topo.vertex(edge.end())?.point();
    let c0 = evaluate(t0);
    let c1 = evaluate(t1);

    let forward_ok = (c0 - p_start).length() <= tolerance && (c1 - p_end).length() <= tolerance;
    let reversed_ok = (c0 - p_end).length() <= tolerance && (c1 - p_start).length() <= tolerance;

    if forward_ok || !reversed_ok {
        // Either consistent, or off the curve entirely — the latter belongs
        // to VertexOnCurve, not here.
        return Ok(vec![]);
    }

    let deviation = (c0 - p_start).length().max((c1 - p_end).length());
    Ok(vec![ValidationIssue {
        check: CheckId::EdgeCurveDirection,
        severity: Severity::Error,
        entity: EntityRef::Edge(edge_id),
        description: format!(
            "edge {}'s curve runs against its declared endpoints: \
             evaluate(domain start) coincides with the end vertex \
             (forward deviation {deviation:.2e})",
            edge_id.index(),
        ),
        deviation: Some(deviation),
    }])
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

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::curves::Circle3D;
    use remus_math::nurbs::fitting::interpolate;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;

    use super::{CheckId, check_edge_curve_direction};

    fn interpolated_open_nurbs() -> remus_math::nurbs::curve::NurbsCurve {
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        ];
        interpolate(&pts, 3).unwrap()
    }

    // Issue #269: a curve authored opposite to the edge's declared endpoints
    // passed validation (vertex-on-curve is direction-blind) and surfaced
    // downstream as a wire-orientation fault. The direction check must name
    // the edge instead.
    #[test]
    fn reversed_nurbs_curve_is_flagged_on_the_edge() {
        let mut topo = Topology::new();
        // Curve runs (0,0,0) → (4,0,0); the edge declares the opposite.
        let v_start = topo.add_vertex(Vertex::new(Point3::new(4.0, 0.0, 0.0), 1e-7));
        let v_end = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(
            v_start,
            v_end,
            EdgeCurve::NurbsCurve(interpolated_open_nurbs()),
        ));

        let issues = check_edge_curve_direction(&topo, edge, 1e-4).unwrap();
        assert_eq!(issues.len(), 1, "reversed curve must fire: {issues:?}");
        assert_eq!(issues[0].check, CheckId::EdgeCurveDirection);
    }

    #[test]
    fn consistent_nurbs_curve_is_clean() {
        let mut topo = Topology::new();
        let v_start = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v_end = topo.add_vertex(Vertex::new(Point3::new(4.0, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(Edge::new(
            v_start,
            v_end,
            EdgeCurve::NurbsCurve(interpolated_open_nurbs()),
        ));

        let issues = check_edge_curve_direction(&topo, edge, 1e-4).unwrap();
        assert!(issues.is_empty(), "consistent edge must pass: {issues:?}");
    }

    #[test]
    fn closed_edge_carries_no_direction_signal() {
        let mut topo = Topology::new();
        let v = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let edge = topo.add_edge(Edge::new(v, v, EdgeCurve::Circle(circle)));

        let issues = check_edge_curve_direction(&topo, edge, 1e-4).unwrap();
        assert!(issues.is_empty(), "closed edge is skipped: {issues:?}");
    }

    #[test]
    fn trimmed_circle_arc_running_backwards_is_flagged() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let p_a = Point3::new(1.0, 0.0, 0.0);
        let p_b = Point3::new(0.0, 1.0, 0.0);
        let ta = circle.project(p_a);
        let mut tb = circle.project(p_b);
        if tb <= ta {
            tb += std::f64::consts::TAU;
        }
        // The forward arc (ta → tb) runs from p_a to p_b, but the edge
        // declares the opposite endpoint correspondence.
        let v_start = topo.add_vertex(Vertex::new(p_b, 1e-7));
        let v_end = topo.add_vertex(Vertex::new(p_a, 1e-7));
        let mut edge_data = Edge::new(v_start, v_end, EdgeCurve::Circle(circle));
        edge_data.set_trim(Some((ta, tb)));
        let edge = topo.add_edge(edge_data);

        let issues = check_edge_curve_direction(&topo, edge, 1e-4).unwrap();
        assert_eq!(issues.len(), 1, "reversed arc must fire: {issues:?}");
        assert_eq!(issues[0].check, CheckId::EdgeCurveDirection);
    }
}
