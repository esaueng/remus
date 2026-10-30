//! Twisted extrusion: a planar profile swept by a screw motion about an axis
//! perpendicular to it (helical gear teeth, twisted prisms).
//!
//! A twisted extrusion has exactly the topology of the straight one, so the
//! straight extrusion builds it and this pass only rewrites geometry: every
//! face, edge and vertex keeps its identity, wire and orientation flag, and
//! the construction record stays valid.
//!
//! - Top cap: each profile point `p` moves to `H_p(h)` (the
//!   [`ScrewMotion`] at the full height `h`); the cap plane is unchanged.
//! - Vertical edges become fitted helices `H̃_p` with the edge tolerance
//!   raised to the certified bound.
//! - Side faces from circles concentric with the axis keep their exact
//!   cylinder: the motion maps that cylinder onto itself.
//! - Every other side face becomes the certified swept NURBS of its profile
//!   edge. All fits share one [`ScrewPlan`], so a face's boundary helices are
//!   the vertical edges' curves bit for bit, and its top row is the top
//!   edge's curve.
//! - A wire that is one closed circle concentric with the axis (a bore or a
//!   rim) is left as built: the motion maps it onto itself, and keeping its
//!   seam straight keeps its wall an untrimmed cylinder.
//!
//! A swept face keeps the reversed flag the straight extrusion chose for its
//! wire, so its surface must keep the straight surface's natural normal
//! sense; the profile curve is reversed when the swept surface would not.

use std::f64::consts::FRAC_PI_2;

use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::screw::{ScrewMotion, ScrewPlan};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use super::{ExtrudeConstruction, ExtrudedWire, extrude_impl, reverse_edge_curve};
use crate::OperationsError;

/// Relative tolerance for treating a circle as concentric with the axis.
const CONCENTRIC_TOLERANCE: f64 = 1e-9;

/// A twisted extrusion and its certified geometry report.
#[allow(clippy::redundant_pub_crate)]
pub(crate) struct TwistedExtrusion {
    pub(crate) solid: remus_topology::solid::SolidId,
    pub(crate) construction: ExtrudeConstruction,
    /// Proven bound on the distance from every fitted helix and swept face to
    /// the exact screw sweep (with a rounding allowance).
    pub(crate) deviation_bound: f64,
    /// Uniform Hermite spans along the axis.
    pub(crate) spans: usize,
}

/// Extrude the planar `face` by `distance` along `direction` while turning
/// `rate` radians per unit length about the axis through `axis_origin`
/// (right-handed about `direction` for a positive rate).
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] when the profile is not a plane
/// perpendicular to `direction`, or holds a curve this pass cannot sweep (a
/// closed off-axis circle, an ellipse or conic, a trimmed NURBS); the
/// straight extrusion's errors; math errors when `tolerance` needs more than
/// `max_spans` spans.
#[allow(clippy::too_many_arguments, clippy::redundant_pub_crate)]
pub(crate) fn twisted_extrude_impl(
    topo: &mut Topology,
    face: FaceId,
    axis_origin: Point3,
    direction: Vec3,
    distance: f64,
    rate: f64,
    tolerance: f64,
    max_spans: usize,
) -> Result<TwistedExtrusion, OperationsError> {
    let axis = direction.normalize()?;
    let perpendicular = match topo.face(face)?.surface() {
        FaceSurface::Plane { normal, .. } => normal.cross(axis).length() < CONCENTRIC_TOLERANCE,
        _ => false,
    };
    if !perpendicular {
        return Err(OperationsError::InvalidInput {
            reason: "a twisted extrusion needs a planar profile perpendicular to its axis"
                .to_owned(),
        });
    }
    let (solid, construction) = extrude_impl(topo, face, axis, distance)?;
    let motion = ScrewMotion::new(axis_origin, axis, rate, distance)?;

    let mut plans = Vec::new();
    if let Some(plan) = plan_wire(
        topo,
        &motion,
        &construction.outer,
        &construction.outer_sides,
    )? {
        plans.push(plan);
    }
    for hole in &construction.holes {
        if let Some(plan) = plan_wire(topo, &motion, &hole.inner, &hole.sides)? {
            plans.push(plan);
        }
    }
    let max_radius = plans.iter().map(|plan| plan.max_radius).fold(0.0, f64::max);
    let screw = motion.plan(max_radius, tolerance, max_spans)?;
    // The bound is exact-arithmetic; allow for rounding at the largest
    // coordinate the fits evaluate.
    let deviation_bound =
        (64.0 * f64::EPSILON).mul_add(max_radius + distance, screw.remainder_bound);
    for plan in plans {
        apply(topo, &motion, &screw, plan, deviation_bound)?;
    }
    Ok(TwistedExtrusion {
        solid,
        construction,
        deviation_bound,
        spans: screw.spans,
    })
}

/// What happens to one side face.
enum Side {
    /// A cylinder concentric with the axis: kept as built.
    Keep,
    /// Swept from this profile curve (oriented like its bottom edge, ends
    /// snapped to the edge's vertices).
    Sweep(NurbsCurve),
}

/// One position of a twisted wire, read before any mutation.
struct Position {
    bottom: Point3,
    top_vertex: VertexId,
    vertical: EdgeId,
    top_edge: EdgeId,
    top_curve: EdgeCurve,
    top_trim: Option<(f64, f64)>,
    face: FaceId,
    side: Side,
}

struct WirePlan {
    positions: Vec<Position>,
    max_radius: f64,
}

fn unsupported(what: &str) -> OperationsError {
    OperationsError::InvalidInput {
        reason: format!("a twisted extrusion cannot sweep {what}"),
    }
}

fn concentric(motion: &ScrewMotion, circle: &remus_math::curves::Circle3D) -> bool {
    let scale = circle.radius().max(1.0);
    motion.radius(circle.center()) <= CONCENTRIC_TOLERANCE * scale
        && circle.normal().cross(motion.axis()).length() <= CONCENTRIC_TOLERANCE
}

fn plan_wire(
    topo: &Topology,
    motion: &ScrewMotion,
    wire: &ExtrudedWire,
    sides: &[FaceId],
) -> Result<Option<WirePlan>, OperationsError> {
    let n = wire.oriented.len();
    if n == 1 {
        let edge = topo.edge(wire.edge_ids[0])?;
        return match edge.curve() {
            EdgeCurve::Circle(circle) if concentric(motion, circle) => Ok(None),
            _ => Err(unsupported("a closed edge off the axis")),
        };
    }
    let mut max_radius: f64 = 0.0;
    let mut positions = Vec::with_capacity(n);
    for i in 0..n {
        let bottom_edge = topo.edge(wire.edge_ids[i])?;
        let start = topo.vertex(bottom_edge.start())?.point();
        let end = topo.vertex(bottom_edge.end())?.point();
        let bottom = wire.positions[i];
        max_radius = max_radius.max(motion.radius(bottom));
        let side = match bottom_edge.curve() {
            EdgeCurve::Circle(circle) if concentric(motion, circle) => Side::Keep,
            EdgeCurve::Circle(circle) => {
                let (t0, t1) =
                    bottom_edge
                        .strict_domain()
                        .map_err(|error| OperationsError::InvalidInput {
                            reason: format!("twisted extrusion arc has no domain: {error}"),
                        })?;
                Side::Sweep(snap(arc_nurbs(circle, t0, t1)?, start, end)?)
            }
            EdgeCurve::Line => Side::Sweep(NurbsCurve::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![start, end],
                vec![1.0, 1.0],
            )?),
            EdgeCurve::NurbsCurve(curve) => {
                let domain = curve.domain();
                let full = bottom_edge.trim().is_none_or(|(t0, t1)| {
                    let span = (domain.1 - domain.0).abs().max(1.0);
                    (t0 - domain.0).abs() <= 1e-12 * span && (t1 - domain.1).abs() <= 1e-12 * span
                });
                if !full {
                    return Err(unsupported("a trimmed spline edge"));
                }
                Side::Sweep(snap(curve.clone(), start, end)?)
            }
            _ => return Err(unsupported("an ellipse or open conic edge")),
        };
        if let Side::Sweep(curve) = &side {
            for &p in curve.control_points() {
                max_radius = max_radius.max(motion.radius(p));
            }
        }
        let top = topo.edge(wire.top_edge_ids[i])?;
        let top_curve = match (top.curve(), &side) {
            (EdgeCurve::Line, _) => EdgeCurve::Line,
            (EdgeCurve::Circle(circle), _) => EdgeCurve::Circle(rotate_circle(motion, circle)?),
            (EdgeCurve::NurbsCurve(_), Side::Sweep(curve)) => {
                let moved = NurbsCurve::new(
                    curve.degree(),
                    curve.knots().to_vec(),
                    curve
                        .control_points()
                        .iter()
                        .map(|&p| motion.point(p, motion.height()))
                        .collect(),
                    curve.weights().to_vec(),
                )?;
                let moved = EdgeCurve::NurbsCurve(moved);
                if wire.oriented[i].is_forward() {
                    moved
                } else {
                    reverse_edge_curve(&moved)?
                }
            }
            _ => return Err(unsupported("a top edge unlike its profile edge")),
        };
        positions.push(Position {
            bottom,
            top_vertex: wire.top_verts[i],
            vertical: wire.vertical_edge_ids[i],
            top_edge: wire.top_edge_ids[i],
            top_curve,
            top_trim: top.trim(),
            face: sides[i],
            side,
        });
    }
    Ok(Some(WirePlan {
        positions,
        max_radius,
    }))
}

/// Replace `curve`'s end control points by the edge's vertex positions, so
/// its swept boundary helices are the vertical edges' helices exactly.
fn snap(curve: NurbsCurve, start: Point3, end: Point3) -> Result<NurbsCurve, OperationsError> {
    let mut points = curve.control_points().to_vec();
    let scale = points
        .iter()
        .map(|p| p.0.iter().fold(0.0_f64, |m, c| m.max(c.abs())))
        .fold(1.0, f64::max);
    let last = points.len() - 1;
    if (points[0] - start).length() > 1e-9 * scale || (points[last] - end).length() > 1e-9 * scale {
        return Err(OperationsError::InvalidInput {
            reason: "twisted extrusion edge curve does not end at its vertices".to_owned(),
        });
    }
    points[0] = start;
    points[last] = end;
    Ok(NurbsCurve::new(
        curve.degree(),
        curve.knots().to_vec(),
        points,
        curve.weights().to_vec(),
    )?)
}

/// The arc of `circle` over `[t0, t1]` as rational quadratic Bézier spans
/// of at most a quarter turn each, parameterized over `[0, 1]`.
fn arc_nurbs(
    circle: &remus_math::curves::Circle3D,
    t0: f64,
    t1: f64,
) -> Result<NurbsCurve, OperationsError> {
    let sweep = t1 - t0;
    if !(sweep > 0.0 && sweep.is_finite()) {
        return Err(unsupported("an empty or reversed arc"));
    }
    // At most a quarter turn per span keeps every weight at least cos(π/4).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let spans = (sweep / FRAC_PI_2).ceil().max(1.0) as usize;
    #[allow(clippy::cast_precision_loss)]
    let step = sweep / spans as f64;
    let weight = (0.5 * step).cos();
    let mut points = vec![circle.evaluate(t0)];
    let mut weights = vec![1.0];
    let mut knots = vec![0.0; 3];
    for k in 0..spans {
        #[allow(clippy::cast_precision_loss)]
        let a = t0 + step * k as f64;
        let b = if k + 1 == spans { t1 } else { a + step };
        let middle = circle.evaluate(0.5 * (a + b)) - circle.center();
        points.push(circle.center() + middle * (1.0 / weight));
        weights.push(weight);
        points.push(circle.evaluate(b));
        weights.push(1.0);
        if k + 1 < spans {
            #[allow(clippy::cast_precision_loss)]
            let knot = (k + 1) as f64 / spans as f64;
            knots.extend([knot, knot]);
        }
    }
    knots.extend([1.0; 3]);
    Ok(NurbsCurve::new(2, knots, points, weights)?)
}

/// `circle` turned by the motion's full turn about its axis (and kept at its
/// height: the circle already sits on the top cap).
fn rotate_circle(
    motion: &ScrewMotion,
    circle: &remus_math::curves::Circle3D,
) -> Result<remus_math::curves::Circle3D, OperationsError> {
    let turn = |v: Vec3| motion.turn_vector(v, motion.height());
    Ok(remus_math::curves::Circle3D::with_axes(
        motion.turn_point(circle.center(), motion.height()),
        turn(circle.normal()),
        circle.radius(),
        turn(circle.u_axis()),
        turn(circle.v_axis()),
    )?)
}

fn apply(
    topo: &mut Topology,
    motion: &ScrewMotion,
    screw: &ScrewPlan,
    plan: WirePlan,
    deviation_bound: f64,
) -> Result<(), OperationsError> {
    let height = motion.height();
    for position in plan.positions {
        topo.vertex_mut(position.top_vertex)?
            .set_point(motion.point(position.bottom, height));

        let helix = motion.helix(screw, position.bottom)?;
        let vertical = topo.edge_mut(position.vertical)?;
        let tolerance = vertical.tolerance().unwrap_or(0.0).max(deviation_bound);
        vertical.set_curve(EdgeCurve::NurbsCurve(helix));
        vertical.set_trim(Some((0.0, height)));
        vertical.set_tolerance(Some(tolerance))?;

        let top = topo.edge_mut(position.top_edge)?;
        top.set_curve(position.top_curve);
        top.set_trim(position.top_trim);

        if let Side::Sweep(curve) = position.side {
            let old = topo.face(position.face)?.surface().clone();
            let surface = oriented_sweep(motion, screw, &old, curve)?;
            topo.face_mut(position.face)?
                .set_surface(FaceSurface::Nurbs(surface));
        }
    }
    Ok(())
}

/// The swept surface of `curve`, reversed in `v` when needed so its natural
/// normal keeps the straight surface's sense at the profile.
fn oriented_sweep(
    motion: &ScrewMotion,
    screw: &ScrewPlan,
    old: &FaceSurface,
    curve: NurbsCurve,
) -> Result<remus_math::nurbs::NurbsSurface, OperationsError> {
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
                        reason: "twisted extrusion could not orient a side face".to_owned(),
                    })?;
            surface.normal(u, v)
        }
    };
    let surface = motion.surface(screw, &curve)?;
    if surface.normal(0.0, middle)?.dot(old_normal) >= 0.0 {
        return Ok(surface);
    }
    Ok(motion.surface(screw, &curve.reversed())?)
}

#[cfg(test)]
mod tests;
