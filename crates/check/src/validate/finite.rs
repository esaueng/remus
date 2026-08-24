//! Non-finite geometry detection.
//!
//! Every other geometric check in this module compares a measured deviation
//! against a tolerance, and a comparison against NaN is always false. A shape
//! carrying NaN or infinite geometry therefore passes each of those checks
//! silently: it measures, validates, and exports as if it were sound. These
//! checks exist so poisoned geometry is reported instead of ignored.
//!
//! The checks sample through the [`EdgeCurve`] and [`FaceSurface`] delegate
//! methods rather than matching on variants, so a new curve or surface type is
//! covered without touching this file.

use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

/// Check that a vertex position has no NaN or infinite coordinate.
///
/// # Errors
///
/// Returns an error if the vertex lookup fails.
pub fn check_vertex_finite(
    topo: &Topology,
    vertex_id: VertexId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let point = topo.vertex(vertex_id)?.point();
    if point.0.iter().all(|c| c.is_finite()) {
        return Ok(vec![]);
    }
    Ok(vec![ValidationIssue {
        check: CheckId::GeometryFinite,
        severity: Severity::Error,
        entity: EntityRef::Vertex(vertex_id),
        description: format!(
            "vertex position is not finite: ({}, {}, {})",
            point.x(),
            point.y(),
            point.z()
        ),
        deviation: None,
    }])
}

/// Check that an edge's curve evaluates to finite points across its domain.
///
/// # Errors
///
/// Returns an error if the edge or its vertex lookups fail.
pub fn check_edge_finite(
    topo: &Topology,
    edge_id: EdgeId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (t0, t1) = edge.domain_with_endpoints(start, end);
    if !t0.is_finite() || !t1.is_finite() {
        return Ok(vec![ValidationIssue {
            check: CheckId::GeometryFinite,
            severity: Severity::Error,
            entity: EntityRef::Edge(edge_id),
            description: format!("edge parameter domain is not finite: [{t0}, {t1}]"),
            deviation: None,
        }]);
    }

    // Endpoints and midpoint: a curve whose defining data is poisoned
    // evaluates to NaN everywhere, so three samples are enough to catch it
    // without paying for a full traversal on every validation run.
    let mid = t0 + (t1 - t0) * 0.5;
    for t in [t0, mid, t1] {
        let point = edge.curve().evaluate_with_endpoints(t, start, end);
        if !point.0.iter().all(|c| c.is_finite()) {
            return Ok(vec![ValidationIssue {
                check: CheckId::GeometryFinite,
                severity: Severity::Error,
                entity: EntityRef::Edge(edge_id),
                description: format!(
                    "edge curve evaluates to a non-finite point at t={t}: ({}, {}, {})",
                    point.x(),
                    point.y(),
                    point.z()
                ),
                deviation: None,
            }]);
        }
    }
    Ok(vec![])
}

/// Check that a face's surface has finite defining geometry.
///
/// # Errors
///
/// Returns an error if the face lookup fails.
pub fn check_face_finite(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let surface = topo.face(face_id)?.surface();

    let issue = |description: String| ValidationIssue {
        check: CheckId::GeometryFinite,
        severity: Severity::Error,
        entity: EntityRef::Face(face_id),
        description,
        deviation: None,
    };

    // A plane's `d` is not reachable through `normal`/`evaluate`, so it is
    // checked directly; every other variant is covered by the samples below.
    if let FaceSurface::Plane { d, .. } = surface
        && !d.is_finite()
    {
        return Ok(vec![issue(format!("plane offset is not finite: {d}"))]);
    }

    // `normal` is defined for every surface variant, including `Plane`, where
    // it returns the stored normal without consulting `(u, v)`.
    let normal = surface.normal(0.0, 0.0);
    if !normal.0.iter().all(|c| c.is_finite()) {
        return Ok(vec![issue(format!(
            "surface normal is not finite: ({}, {}, {})",
            normal.x(),
            normal.y(),
            normal.z()
        ))]);
    }

    // Parametric surfaces are sampled at a parameter every variant accepts;
    // analytic parameterizations are global, and evaluating a NURBS surface
    // outside its knot domain extrapolates finite data to a finite point, so
    // a NaN here always comes from the surface's own data.
    if let Some(point) = surface.evaluate(0.0, 0.0)
        && !point.0.iter().all(|c| c.is_finite())
    {
        return Ok(vec![issue(format!(
            "surface evaluates to a non-finite point: ({}, {}, {})",
            point.x(),
            point.y(),
            point.z()
        ))]);
    }

    Ok(vec![])
}
