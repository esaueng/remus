//! Exact support-surface replacement with topology-preserving re-limitation.

use remus_algo::{PlaneFrame, compute_pcurve_on_surface_in_domain};
use remus_math::curves2d::Curve2D;
use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;

use crate::OperationsError;

/// A re-limited solid and exact source-to-result face correspondence.
#[derive(Debug)]
pub struct ReplaceSurfaceResult {
    /// Edited solid.
    pub solid: SolidId,
    /// Source face index to the one result face derived from it.
    ///
    /// Deterministically hashed: the offset layer builds this with a std map,
    /// whose iteration order varies run to run.
    pub face_map: DetHashMap<usize, FaceId>,
}

/// Replace one face's support surface and rebuild every affected boundary.
///
/// The qualified first cell accepts plane-to-plane replacement and coaxial
/// cylinder radius replacement when every neighbor is planar or cylindrical.
/// It preserves the source adjacency graph, reconstructs intersection-curve
/// trims, and derives a fresh p-curve for every coedge in the result; a circle
/// on a plane gets its exact `Circle2D` image.
///
/// # Errors
///
/// Returns a typed offset error naming the offending source face or edge when
/// re-intersection cannot preserve the adjacency graph. Any error, including
/// p-curve projection or final validation, restores the exact input topology.
pub fn replace_surface(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    replacement: FaceSurface,
) -> Result<ReplaceSurfaceResult, OperationsError> {
    let result = replace_surface_with_entity_map(topo, solid, face, replacement)?;
    Ok(ReplaceSurfaceResult {
        solid: result.solid,
        face_map: result.face_map.into_iter().collect(),
    })
}

pub(crate) fn replace_surface_with_entity_map(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    replacement: FaceSurface,
) -> Result<remus_offset::MoveFacesEntityResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result = remus_offset::replace_surface_with_entity_map(topo, solid, face, replacement)?;
        let result_faces = solid_faces(topo, result.solid)?;
        register_fresh_pcurves(topo, &result_faces)?;

        let report = crate::validate::validate_solid(topo, result.solid)?;
        if !report.is_valid() {
            return Err(remus_offset::OffsetError::TopologyChange {
                face: Some(face),
                edge: None,
                reason: format!(
                    "replace-surface validation failed with {} error(s)",
                    report.error_count()
                ),
            }
            .into());
        }
        Ok(result)
    })
}

/// Shift a straight p-curve on a `u`-periodic cylinder or cone onto the
/// principal branch at its oriented start vertex.
///
/// A fresh coedge carries zero periodic winding, so its p-curve must start
/// where the surface's own projection puts that vertex. Fitting from an edge
/// trim can land a whole period away (an arc trimmed as `[2π, 5π/2]`), which
/// STEP export rejects as a winding/branch disagreement. Seam uses are left
/// alone: their two branches are a period apart by construction.
fn principal_branch(
    curve: Curve2D,
    start_parameter: f64,
    surface: &FaceSurface,
    oriented_start: Point3,
) -> Curve2D {
    use remus_math::vec::Vec2;
    let Curve2D::Line(line) = &curve else {
        return curve;
    };
    if !matches!(surface, FaceSurface::Cylinder(_) | FaceSurface::Cone(_)) {
        return curve;
    }
    let Some((principal, _)) = surface.project_point(oriented_start) else {
        return curve;
    };
    let at_start = line.evaluate(start_parameter);
    let turns = ((at_start.x() - principal) / std::f64::consts::TAU).round();
    if turns == 0.0 || !turns.is_finite() {
        return curve;
    }
    let origin = line.evaluate(0.0);
    let direction = line.evaluate(1.0) - origin;
    remus_math::curves2d::Line2D::new(
        remus_math::vec::Point2::new(origin.x() - turns * std::f64::consts::TAU, origin.y()),
        Vec2::new(direction.x(), direction.y()),
    )
    .map_or(curve, Curve2D::Line)
}

/// The exact p-curve of a circle use on a plane in the canonical plane chart.
///
/// A fitted NURBS chart of an exact circle has no `SameParameter` proof, so a
/// re-trimmed rim would fail strict validation. The circle lies in the plane,
/// so its chart image is a [`Circle2D`] about the projected centre whose
/// angle `t` meets the edge parameter `s` at `s = φ + σ t`: `φ` is the edge
/// angle of the chart `u` axis and `σ` is +1 when the circle turns with the
/// chart. Both endpoints follow from the use's oriented edge domain.
fn exact_plane_circle_pcurve(
    frame: &PlaneFrame,
    circle: &remus_math::curves::Circle3D,
    domain: (f64, f64),
    forward: bool,
) -> Option<PCurve> {
    let (u, v) = (frame.u_axis(), frame.v_axis());
    let (edge_u, edge_v) = (circle.u_axis(), circle.v_axis());
    let tolerance = remus_math::tolerance::Tolerance::new();
    if circle.normal().cross(u.cross(v)).length() > tolerance.angular {
        return None;
    }
    let sigma = if edge_u.cross(edge_v).dot(u.cross(v)) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let phase = u.dot(edge_v).atan2(u.dot(edge_u));
    let curve =
        remus_math::curves2d::Circle2D::new(frame.project(circle.center()), circle.radius())
            .ok()?;
    let (first, last) = if forward {
        (domain.0, domain.1)
    } else {
        (domain.1, domain.0)
    };
    let (start, end) = (sigma * (first - phase), sigma * (last - phase));
    (start.is_finite() && end.is_finite() && (end - start).abs() > f64::EPSILON)
        .then(|| PCurve::new(Curve2D::Circle(curve), start, end))
}

pub(crate) fn register_fresh_pcurves(
    topo: &mut Topology,
    faces: &[FaceId],
) -> Result<(), OperationsError> {
    register_fresh_pcurves_where(topo, faces, |_, _, _| true)
}

/// [`register_fresh_pcurves`] for only the coedges `(edge, face, forward)`
/// that `rebuilt` accepts; every other coedge on `faces` keeps its stored
/// p-curve (or its absence) exactly as it was.
pub(crate) fn register_fresh_pcurves_where(
    topo: &mut Topology,
    faces: &[FaceId],
    rebuilt: impl Fn(remus_topology::edge::EdgeId, FaceId, bool) -> bool,
) -> Result<(), OperationsError> {
    for &face_id in faces {
        let face = topo.face(face_id)?;
        let surface = face.surface().clone();
        let outer_wire = face.outer_wire();
        let wire_ids: Vec<_> = std::iter::once(outer_wire)
            .chain(face.inner_wires().iter().copied())
            .collect();
        let wire_points: Vec<Point3> = topo
            .wire(outer_wire)?
            .edges()
            .iter()
            .map(|oriented| {
                let edge = topo.edge(oriented.edge())?;
                Ok(topo.vertex(edge.start())?.point())
            })
            .collect::<Result<_, remus_topology::TopologyError>>()?;

        // Persistent plane p-curves use the surface's canonical origin. A
        // face-local first-vertex frame cannot be reconstructed by STEP I/O.
        let plane_frame = if let FaceSurface::Plane { normal, d } = &surface {
            Some(PlaneFrame::from_normal_and_point(
                *normal,
                Point3::new(normal.x() * d, normal.y() * d, normal.z() * d),
            ))
        } else {
            None
        };
        for wire_id in wire_ids {
            let uses = topo.wire(wire_id)?.edges().to_vec();
            let mut seen = DetHashSet::default();
            let seam_edges: DetHashSet<_> = uses
                .iter()
                .map(remus_topology::wire::OrientedEdge::edge)
                .filter(|&edge| !seen.insert(edge))
                .collect();
            for oriented in uses {
                let edge_id = oriented.edge();
                if !rebuilt(edge_id, face_id, oriented.is_forward()) {
                    continue;
                }
                let edge = topo.edge(edge_id)?;
                let start = topo.vertex(edge.start())?.point();
                let end = topo.vertex(edge.end())?.point();
                let oriented_start = if oriented.is_forward() { start } else { end };
                let domain = edge.strict_domain().map_err(|error| {
                    remus_offset::OffsetError::TopologyChange {
                        face: Some(face_id),
                        edge: Some(edge_id),
                        reason: format!("result edge has no trim authority: {error}"),
                    }
                })?;
                if let (Some(frame), remus_topology::edge::EdgeCurve::Circle(circle)) =
                    (plane_frame.as_ref(), edge.curve())
                    && let Some(pcurve) =
                        exact_plane_circle_pcurve(frame, circle, domain, oriented.is_forward())
                {
                    topo.set_pcurve_oriented(edge_id, face_id, oriented.is_forward(), pcurve)?;
                    continue;
                }
                let curve = compute_pcurve_on_surface_in_domain(
                    edge.curve(),
                    start,
                    end,
                    domain,
                    &surface,
                    &wire_points,
                    plane_frame.as_ref(),
                )?;
                let end_parameter = match &curve {
                    Curve2D::Line(_) => match (&surface, edge.curve()) {
                        (FaceSurface::Cylinder(_), remus_topology::edge::EdgeCurve::Circle(_)) => {
                            (domain.1 - domain.0).abs()
                        }
                        (_, _) if edge.start() == edge.end() => std::f64::consts::TAU,
                        _ => (end - start).length(),
                    },
                    Curve2D::Circle(_) | Curve2D::Ellipse(_) | Curve2D::Nurbs(_) => 1.0,
                };
                let (start_parameter, end_parameter) = if oriented.is_forward() {
                    (0.0, end_parameter)
                } else {
                    (end_parameter, 0.0)
                };
                let curve = if seam_edges.contains(&edge_id) {
                    curve
                } else {
                    principal_branch(curve, start_parameter, &surface, oriented_start)
                };
                let pcurve = PCurve::new(curve, start_parameter, end_parameter);
                topo.set_pcurve_oriented(edge_id, face_id, oriented.is_forward(), pcurve)?;
            }
        }
    }
    Ok(())
}
