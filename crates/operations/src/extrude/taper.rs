//! Tapered extrusion: a planar profile swept toward a point on its axis (the
//! apex), every section a scaled copy of the profile (straight bevel gears).
//!
//! As for the [twisted extrusion](super::twist), the straight extrusion
//! builds the topology and this pass rewrites geometry only, so every face,
//! edge and vertex keeps its identity, wire and orientation flag, and the
//! construction record stays valid. With the profile at distance `h` from the
//! apex, the far cap at `distance` from the profile is the profile scaled by
//! `s = 1 − distance/h` about the apex.
//!
//! - Far vertices and edges: the homothety `p ↦ apex + s·(p − apex)` of the
//!   profile's, exact for lines, circles and NURBS control points.
//! - Vertical edges stay lines, now through the apex.
//! - Side faces: a line sweeps a plane through the apex; a circle concentric
//!   with the axis sweeps a cone whose vertex is the apex; any other curve
//!   sweeps a ruled NURBS whose rows are the curve and its scaled copy with
//!   the same weights. Every side face is the exact sweep of its edge.
//! - A hole that is one closed circle concentric with the axis (a bore) is
//!   left as built: a straight cylinder through both caps. An outer boundary
//!   that is one such circle tapers into a cone like any other.
//!
//! A side face keeps the straight extrusion's reversed flag, so its new
//! surface keeps the straight surface's natural normal sense: planes and
//! ruled NURBS are flipped to match it, and a cone's natural normal points
//! away from its axis as the cylinder's did.

use remus_math::curves::Circle3D;
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use super::twist::{arc_nurbs, snap};
use super::{ExtrudeConstruction, ExtrudedWire, extrude_impl};
use crate::OperationsError;

/// Relative tolerance for treating a circle as concentric with the axis.
const CONCENTRIC_TOLERANCE: f64 = 1e-9;

/// A tapered extrusion.
#[allow(clippy::redundant_pub_crate)]
pub(crate) struct TaperedExtrusion {
    pub(crate) solid: remus_topology::solid::SolidId,
    pub(crate) construction: ExtrudeConstruction,
}

/// The homothety about the apex, and the axis it tapers along.
struct Taper {
    apex: Point3,
    axis: Vec3,
    scale: f64,
}

impl Taper {
    fn point(&self, p: Point3) -> Point3 {
        self.apex + (p - self.apex) * self.scale
    }

    fn is_concentric(&self, circle: &Circle3D) -> bool {
        let offset = circle.center() - self.apex;
        let across = offset - self.axis * offset.dot(self.axis);
        across.length() <= CONCENTRIC_TOLERANCE * circle.radius().max(1.0)
            && circle.normal().cross(self.axis).length() <= CONCENTRIC_TOLERANCE
    }

    fn curve(&self, curve: &NurbsCurve) -> Result<NurbsCurve, OperationsError> {
        Ok(NurbsCurve::new(
            curve.degree(),
            curve.knots().to_vec(),
            curve
                .control_points()
                .iter()
                .map(|&p| self.point(p))
                .collect(),
            curve.weights().to_vec(),
        )?)
    }
}

/// Extrude the planar `face` by `distance` along `direction`, tapering every
/// section toward `apex`. The taper axis runs through `apex` along
/// `direction`, and the apex must lie beyond the far cap: further than
/// `distance` from the profile plane, on the side `direction` points to.
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] when the profile is not a plane
/// perpendicular to `direction`, the apex does not lie beyond the far cap,
/// or the profile holds a curve this pass cannot sweep (an ellipse or conic,
/// a trimmed NURBS, a closed circle off the axis); the straight extrusion's
/// errors.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn tapered_extrude_impl(
    topo: &mut Topology,
    face: FaceId,
    apex: Point3,
    direction: Vec3,
    distance: f64,
) -> Result<TaperedExtrusion, OperationsError> {
    let axis = direction.normalize()?;
    let perpendicular = match topo.face(face)?.surface() {
        FaceSurface::Plane { normal, .. } => normal.cross(axis).length() < CONCENTRIC_TOLERANCE,
        FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => false,
    };
    if !perpendicular {
        return Err(OperationsError::InvalidInput {
            reason: "a tapered extrusion needs a planar profile perpendicular to its axis"
                .to_owned(),
        });
    }
    let (solid, construction) = extrude_impl(topo, face, axis, distance)?;
    let apex_height = construction
        .outer
        .positions
        .first()
        .map_or(0.0, |&p| (apex - p).dot(axis));
    if !(apex_height > distance && apex_height.is_finite()) {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "a tapered extrusion of length {distance} needs its apex beyond the far cap, \
                 not {apex_height} from the profile"
            ),
        });
    }
    let taper = Taper {
        apex,
        axis,
        scale: 1.0 - distance / apex_height,
    };
    let offset = axis * distance;
    let mut plans = Vec::new();
    if let Some(plan) = plan_wire(
        topo,
        &taper,
        offset,
        &construction.outer,
        &construction.outer_sides,
        false,
    )? {
        plans.push(plan);
    }
    for hole in &construction.holes {
        if let Some(plan) = plan_wire(topo, &taper, offset, &hole.inner, &hole.sides, true)? {
            plans.push(plan);
        }
    }
    for plan in plans {
        apply(topo, plan)?;
    }
    Ok(TaperedExtrusion {
        solid,
        construction,
    })
}

/// One position of a tapered wire, read before any mutation.
struct Position {
    top_vertex: VertexId,
    top_point: Point3,
    top_edge: EdgeId,
    top_curve: EdgeCurve,
    top_trim: Option<(f64, f64)>,
    face: FaceId,
    surface: FaceSurface,
}

fn unsupported(what: &str) -> OperationsError {
    OperationsError::InvalidInput {
        reason: format!("a tapered extrusion cannot sweep {what}"),
    }
}

fn plan_wire(
    topo: &Topology,
    taper: &Taper,
    offset: Vec3,
    wire: &ExtrudedWire,
    sides: &[FaceId],
    hole: bool,
) -> Result<Option<Vec<Position>>, OperationsError> {
    let n = wire.oriented.len();
    if n == 1 {
        let edge = topo.edge(wire.edge_ids[0])?;
        match edge.curve() {
            EdgeCurve::Circle(circle) if taper.is_concentric(circle) => {
                if hole {
                    return Ok(None);
                }
            }
            EdgeCurve::Circle(_)
            | EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Err(unsupported("a closed edge off the axis")),
        }
    }
    // The far curves are translated copies of the profile's: undo the
    // translation, then scale about the apex.
    let far = |q: Point3| taper.point(q - offset);
    let mut positions = Vec::with_capacity(n);
    for i in 0..n {
        let bottom_edge = topo.edge(wire.edge_ids[i])?;
        let start = topo.vertex(bottom_edge.start())?.point();
        let end = topo.vertex(bottom_edge.end())?.point();
        let old = topo.face(sides[i])?.surface().clone();
        let surface = match bottom_edge.curve() {
            EdgeCurve::Line => plane_through_apex(taper.apex, start, end, &old)?,
            EdgeCurve::Circle(circle) if taper.is_concentric(circle) => {
                let to_profile = circle.center() - taper.apex;
                let height = to_profile.length();
                FaceSurface::Cone(ConicalSurface::new(
                    taper.apex,
                    to_profile,
                    height.atan2(circle.radius()),
                )?)
            }
            EdgeCurve::Circle(circle) => {
                let (t0, t1) =
                    bottom_edge
                        .strict_domain()
                        .map_err(|error| OperationsError::InvalidInput {
                            reason: format!("tapered extrusion arc has no domain: {error}"),
                        })?;
                ruled(taper, snap(arc_nurbs(circle, t0, t1)?, start, end)?, &old)?
            }
            EdgeCurve::NurbsCurve(curve) => {
                let domain = curve.domain();
                let full = bottom_edge.trim().is_none_or(|(t0, t1)| {
                    let span = (domain.1 - domain.0).abs().max(1.0);
                    (t0 - domain.0).abs() <= 1e-12 * span && (t1 - domain.1).abs() <= 1e-12 * span
                });
                if !full {
                    return Err(unsupported("a trimmed spline edge"));
                }
                ruled(taper, snap(curve.clone(), start, end)?, &old)?
            }
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Err(unsupported("an ellipse or open conic edge"));
            }
        };
        let top = topo.edge(wire.top_edge_ids[i])?;
        let top_curve = match top.curve() {
            EdgeCurve::Line => EdgeCurve::Line,
            EdgeCurve::Circle(circle) => EdgeCurve::Circle(Circle3D::with_axes(
                far(circle.center()),
                circle.normal(),
                circle.radius() * taper.scale,
                circle.u_axis(),
                circle.v_axis(),
            )?),
            EdgeCurve::NurbsCurve(curve) => EdgeCurve::NurbsCurve(NurbsCurve::new(
                curve.degree(),
                curve.knots().to_vec(),
                curve.control_points().iter().map(|&p| far(p)).collect(),
                curve.weights().to_vec(),
            )?),
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Err(unsupported("a far edge unlike its profile edge"));
            }
        };
        positions.push(Position {
            top_vertex: wire.top_verts[i],
            top_point: taper.point(wire.positions[i]),
            top_edge: wire.top_edge_ids[i],
            top_curve,
            top_trim: top.trim(),
            face: sides[i],
            surface,
        });
    }
    Ok(Some(positions))
}

/// The plane through the apex and the segment `start → end`, with the old
/// plane's normal sense.
fn plane_through_apex(
    apex: Point3,
    start: Point3,
    end: Point3,
    old: &FaceSurface,
) -> Result<FaceSurface, OperationsError> {
    let mut normal = (end - start).cross(apex - start).normalize()?;
    if let FaceSurface::Plane { normal: before, .. } = old
        && normal.dot(*before) < 0.0
    {
        normal = -normal;
    }
    Ok(FaceSurface::Plane {
        normal,
        d: normal.dot(Vec3::new(start.x(), start.y(), start.z())),
    })
}

/// The ruled surface from `curve` (`u = 0`) to its scaled copy (`u = 1`),
/// reversed in `v` when needed so its natural normal keeps the old surface's
/// sense at the profile.
fn ruled(
    taper: &Taper,
    curve: NurbsCurve,
    old: &FaceSurface,
) -> Result<FaceSurface, OperationsError> {
    let build = |curve: &NurbsCurve| -> Result<NurbsSurface, OperationsError> {
        let far = taper.curve(curve)?;
        Ok(NurbsSurface::new(
            1,
            curve.degree(),
            vec![0.0, 0.0, 1.0, 1.0],
            curve.knots().to_vec(),
            vec![
                curve.control_points().to_vec(),
                far.control_points().to_vec(),
            ],
            vec![curve.weights().to_vec(), curve.weights().to_vec()],
        )?)
    };
    let (v0, v1) = curve.domain();
    let middle = f64::midpoint(v0, v1);
    let at = curve.evaluate(middle);
    let old_normal = match old {
        FaceSurface::Plane { normal, .. } => *normal,
        surface => {
            let (u, v) =
                surface
                    .project_point(at)
                    .ok_or_else(|| OperationsError::InvalidInput {
                        reason: "tapered extrusion could not orient a side face".to_owned(),
                    })?;
            surface.normal(u, v)
        }
    };
    let surface = build(&curve)?;
    if surface.normal(0.0, middle)?.dot(old_normal) >= 0.0 {
        return Ok(FaceSurface::Nurbs(surface));
    }
    Ok(FaceSurface::Nurbs(build(&curve.reversed())?))
}

fn apply(topo: &mut Topology, plan: Vec<Position>) -> Result<(), OperationsError> {
    for position in plan {
        topo.vertex_mut(position.top_vertex)?
            .set_point(position.top_point);
        // Replacing the curve drops its trim; the scaled curve keeps the
        // same parameter range.
        let top = topo.edge_mut(position.top_edge)?;
        top.set_curve(position.top_curve);
        top.set_trim(position.top_trim);
        topo.face_mut(position.face)?.set_surface(position.surface);
    }
    Ok(())
}

#[cfg(test)]
mod tests;
