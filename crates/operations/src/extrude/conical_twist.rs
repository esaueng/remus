//! Conical twisted extrusion: a planar profile about an axis swept by a
//! screw motion whose radius also changes linearly with the advance (the
//! ring of a tapered thread).
//!
//! Like the twisted extrusion ([`twist`](super::twist)) it has the topology
//! of the straight one, so the straight extrusion builds it and this pass
//! only rewrites geometry. The motion carries every profile point along its
//! own radius ([`ConicalScrewMotion`]), which is not an affine map of the
//! profile, so it only sweeps the pieces whose radial direction is known:
//!
//! - an arc concentric with the axis sweeps a strip of an exact cone, a cone
//!   face while the strip covers less than one turn (a longer one is swept
//!   as a NURBS with the arc's own radial net, which is exact too);
//! - a radial line sweeps a ruled surface, exactly;
//! - a spline edge needs the caller's net of radial directions on its own
//!   control points (`nets`, by edge index): the unit radial direction of
//!   the curve as a spline on the curve's basis. The top edge of every swept
//!   spline gets its net in turn, so the next extrusion stacked on the top
//!   cap continues the same surface.
//!
//! A wire that is one closed circle concentric with the axis is left as
//! built: a straight cylinder wall (a thread ring's seam).
//!
//! Top cap: every profile point moves out along its radius by
//! `radial_rate · height` and turns by `rate · height`; arcs keep their
//! centre with the new radius. Vertical edges become fitted conical helices.
//! All fits share one [`ConicalScrewPlan`], so a face's boundary rows are the
//! vertical edges' curves bit for bit.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::nurbs::NurbsCurve;
use remus_math::nurbs::conical_screw::{ConicalScrewMotion, ConicalScrewPlan};
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;

use super::twist::{TwistedExtrusion, arc_nurbs, snap};
use super::{ExtrudedWire, extrude_impl, reverse_edge_curve};
use crate::OperationsError;

/// Relative tolerance for treating a circle as concentric with the axis and
/// a line as radial.
const CONCENTRIC_TOLERANCE: f64 = 1e-9;

/// The radial direction nets of spline edges, by edge index: one direction
/// per control point of the edge's curve.
#[allow(clippy::redundant_pub_crate)]
pub(crate) type RadialNets = BTreeMap<usize, Vec<Vec3>>;

/// Extrude the planar `face` by `distance` along `direction` while turning
/// `rate` radians and moving `radial_rate` away from the axis through
/// `axis_origin` per unit length.
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] when the profile is not a plane
/// perpendicular to `direction`, holds a curve this pass cannot sweep (an
/// arc off the axis, a line that is not radial, a spline without its net in
/// `nets`, a conic), or would shrink through the axis; the straight
/// extrusion's errors; math errors when `tolerance` needs more than
/// `max_spans` spans.
#[allow(clippy::too_many_arguments, clippy::redundant_pub_crate)]
pub(crate) fn conical_twisted_extrude_impl(
    topo: &mut Topology,
    face: FaceId,
    axis_origin: Point3,
    direction: Vec3,
    distance: f64,
    rate: f64,
    radial_rate: f64,
    tolerance: f64,
    max_spans: usize,
    nets: &mut RadialNets,
) -> Result<TwistedExtrusion, OperationsError> {
    super::nesting::refuse_nested_profile(topo, face, "twisted extrusion")?;
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
            reason: "a twisted extrusion needs a planar profile perpendicular to its axis"
                .to_owned(),
        });
    }
    let (solid, construction) = extrude_impl(topo, face, axis, distance)?;
    let motion = ConicalScrewMotion::new(axis_origin, axis, rate, radial_rate, distance)?;

    let mut plans = Vec::new();
    if let Some(plan) = plan_wire(
        topo,
        &motion,
        &construction.outer,
        &construction.outer_sides,
        nets,
    )? {
        plans.push(plan);
    }
    for hole in &construction.holes {
        if let Some(plan) = plan_wire(topo, &motion, &hole.inner, &hole.sides, nets)? {
            plans.push(plan);
        }
    }
    let reach = plans.iter().map(|plan| plan.reach).fold(0.0, f64::max);
    let longest = plans.iter().map(|plan| plan.longest).fold(1.0, f64::max);
    let screw = motion.plan(reach, longest, tolerance, max_spans)?;
    // The bound is exact-arithmetic; allow for rounding at the largest
    // coordinate the fits evaluate.
    let deviation_bound = (64.0 * f64::EPSILON).mul_add(reach + distance, screw.remainder_bound);
    for plan in plans {
        apply(topo, &motion, &screw, plan, deviation_bound, nets)?;
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
    /// A concentric arc over less than a turn: its cone.
    Cone(ConicalSurface),
    /// Swept from this profile curve (oriented like its bottom edge, ends
    /// snapped to the edge's vertices) carried with this radial net.
    Sweep(NurbsCurve, Vec<Vec3>),
}

/// One position of a wire, read before any mutation.
struct Position {
    bottom: Point3,
    radial: Vec3,
    top_vertex: VertexId,
    vertical: EdgeId,
    top_edge: EdgeId,
    top_curve: EdgeCurve,
    top_trim: Option<(f64, f64)>,
    /// The top edge's own net, for the extrusion stacked on it.
    top_net: Option<Vec<Vec3>>,
    face: FaceId,
    side: Side,
}

struct WirePlan {
    positions: Vec<Position>,
    /// Largest distance from the axis along any path.
    reach: f64,
    /// Longest radial direction.
    longest: f64,
}

fn unsupported(what: &str) -> OperationsError {
    OperationsError::InvalidInput {
        reason: format!("a conical twisted extrusion cannot sweep {what}"),
    }
}

fn concentric(motion: &ConicalScrewMotion, circle: &Circle3D) -> bool {
    let scale = circle.radius().max(1.0);
    motion.radius(circle.center()) <= CONCENTRIC_TOLERANCE * scale
        && circle.normal().cross(motion.axis()).length() <= CONCENTRIC_TOLERANCE
}

/// The cone a concentric arc of `circle` sweeps.
fn swept_cone(
    motion: &ConicalScrewMotion,
    circle: &Circle3D,
) -> Result<ConicalSurface, OperationsError> {
    let slope = motion.radial_rate().abs();
    if slope <= 0.0 {
        return Err(unsupported("a cone without a radial rate"));
    }
    // The cone's axis points the way it widens.
    let widening = motion.axis() * motion.radial_rate().signum();
    let apex = circle.center() - widening * (circle.radius() / slope);
    Ok(ConicalSurface::with_ref_dir(
        apex,
        widening,
        slope.recip().atan(),
        circle.u_axis(),
    )?)
}

#[allow(clippy::too_many_lines)]
fn plan_wire(
    topo: &Topology,
    motion: &ConicalScrewMotion,
    wire: &ExtrudedWire,
    sides: &[FaceId],
    nets: &RadialNets,
) -> Result<Option<WirePlan>, OperationsError> {
    let n = wire.oriented.len();
    if n == 1 {
        let edge = topo.edge(wire.edge_ids[0])?;
        return match edge.curve() {
            EdgeCurve::Circle(circle) if concentric(motion, circle) => Ok(None),
            EdgeCurve::Circle(_)
            | EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => Err(unsupported("a closed edge off the axis")),
        };
    }
    let height = motion.height();
    let grown = motion.radial_rate() * height;
    let mut reach: f64 = 0.0;
    let mut longest: f64 = 1.0;
    let mut positions = Vec::with_capacity(n);
    for i in 0..n {
        let bottom_edge = topo.edge(wire.edge_ids[i])?;
        let start = topo.vertex(bottom_edge.start())?.point();
        let end = topo.vertex(bottom_edge.end())?.point();
        let bottom = wire.positions[i];
        let radial = motion.radial(bottom)?;
        if motion.radius(bottom) + grown <= 0.0 {
            return Err(unsupported("a profile that shrinks through the axis"));
        }
        reach = reach.max(motion.reach(bottom, radial));
        let side = match bottom_edge.curve() {
            EdgeCurve::Circle(circle) => {
                if !concentric(motion, circle) {
                    return Err(unsupported("an arc off the axis"));
                }
                let (t0, t1) =
                    bottom_edge
                        .strict_domain()
                        .map_err(|error| OperationsError::InvalidInput {
                            reason: format!("twisted extrusion arc has no domain: {error}"),
                        })?;
                // A cone face holds the strip only while it covers less
                // than one turn; past that it would cover itself twice.
                let strip = (t1 - t0) + motion.total_turn().abs();
                if strip < TAU * (1.0 - CONCENTRIC_TOLERANCE) {
                    Side::Cone(swept_cone(motion, circle)?)
                } else {
                    let curve = snap(arc_nurbs(circle, t0, t1)?, start, end)?;
                    // Scaling the control net about the centre moves every
                    // arc point along its own radius.
                    let mut net: Vec<Vec3> = curve
                        .control_points()
                        .iter()
                        .map(|&p| {
                            let offset = p - circle.center();
                            (offset - motion.axis() * offset.dot(motion.axis()))
                                * circle.radius().recip()
                        })
                        .collect();
                    let last = net.len() - 1;
                    net[0] = motion.radial(start)?;
                    net[last] = motion.radial(end)?;
                    Side::Sweep(curve, net)
                }
            }
            EdgeCurve::Line => {
                let (from, to) = (motion.radial(start)?, motion.radial(end)?);
                let along = (end - start).normalize()?;
                if from.cross(to).length() > CONCENTRIC_TOLERANCE
                    || along.cross(from).length() > CONCENTRIC_TOLERANCE
                {
                    return Err(unsupported("a line that is not radial"));
                }
                Side::Sweep(
                    NurbsCurve::new(
                        1,
                        vec![0.0, 0.0, 1.0, 1.0],
                        vec![start, end],
                        vec![1.0, 1.0],
                    )?,
                    vec![from, to],
                )
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
                let curve = snap(curve.clone(), start, end)?;
                let mut net = nets
                    .get(&wire.edge_ids[i].index())
                    .filter(|net| net.len() == curve.control_points().len())
                    .cloned()
                    .ok_or_else(|| unsupported("a spline edge without its radial directions"))?;
                // The boundary rows are the vertical edges' paths: the
                // vertices' own radial directions.
                let last = net.len() - 1;
                net[0] = motion.radial(start)?;
                net[last] = motion.radial(end)?;
                Side::Sweep(curve, net)
            }
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Err(unsupported("an ellipse or open conic edge"));
            }
        };
        if let Side::Sweep(curve, net) = &side {
            for (&p, &u) in curve.control_points().iter().zip(net) {
                reach = reach.max(motion.reach(p, u));
                longest = longest.max(u.length());
            }
        }
        let top = topo.edge(wire.top_edge_ids[i])?;
        let forward = wire.oriented[i].is_forward();
        let (top_curve, top_net) = match (top.curve(), &side) {
            (EdgeCurve::Line, _) => (EdgeCurve::Line, None),
            (EdgeCurve::Circle(circle), _) => {
                let radius = circle.radius() + grown;
                if radius <= 0.0 {
                    return Err(unsupported("a profile that shrinks through the axis"));
                }
                let turn = |v: Vec3| motion.turn_vector(v, height);
                (
                    EdgeCurve::Circle(Circle3D::with_axes(
                        circle.center(),
                        turn(circle.normal()),
                        radius,
                        turn(circle.u_axis()),
                        turn(circle.v_axis()),
                    )?),
                    None,
                )
            }
            (EdgeCurve::NurbsCurve(_), Side::Sweep(curve, net)) => {
                let moved = NurbsCurve::new(
                    curve.degree(),
                    curve.knots().to_vec(),
                    curve
                        .control_points()
                        .iter()
                        .zip(net)
                        .map(|(&p, &u)| motion.point(p, u, height))
                        .collect(),
                    curve.weights().to_vec(),
                )?;
                let moved = EdgeCurve::NurbsCurve(moved);
                let mut turned: Vec<Vec3> =
                    net.iter().map(|&u| motion.turn_vector(u, height)).collect();
                if forward {
                    (moved, Some(turned))
                } else {
                    turned.reverse();
                    (reverse_edge_curve(&moved)?, Some(turned))
                }
            }
            (
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_),
                _,
            ) => return Err(unsupported("a top edge unlike its profile edge")),
        };
        positions.push(Position {
            bottom,
            radial,
            top_vertex: wire.top_verts[i],
            vertical: wire.vertical_edge_ids[i],
            top_edge: wire.top_edge_ids[i],
            top_curve,
            top_trim: top.trim(),
            top_net,
            face: sides[i],
            side,
        });
    }
    Ok(Some(WirePlan {
        positions,
        reach,
        longest,
    }))
}

fn apply(
    topo: &mut Topology,
    motion: &ConicalScrewMotion,
    screw: &ConicalScrewPlan,
    plan: WirePlan,
    deviation_bound: f64,
    nets: &mut RadialNets,
) -> Result<(), OperationsError> {
    let height = motion.height();
    for position in plan.positions {
        topo.vertex_mut(position.top_vertex)?
            .set_point(motion.point(position.bottom, position.radial, height));

        let helix = motion.helix(screw, position.bottom, position.radial)?;
        let vertical = topo.edge_mut(position.vertical)?;
        let tolerance = vertical.tolerance().unwrap_or(0.0).max(deviation_bound);
        vertical.set_curve(EdgeCurve::NurbsCurve(helix));
        vertical.set_trim(Some((0.0, height)));
        vertical.set_tolerance(Some(tolerance))?;

        let top = topo.edge_mut(position.top_edge)?;
        top.set_curve(position.top_curve);
        top.set_trim(position.top_trim);
        if let Some(net) = position.top_net {
            nets.insert(position.top_edge.index(), net);
        }

        let old = topo.face(position.face)?.surface().clone();
        let surface = match position.side {
            Side::Cone(cone) => {
                let surface = FaceSurface::Cone(cone);
                // The face keeps the flag the straight extrusion chose for
                // its cylinder, so the cone must face the same way.
                let probe = motion.point(position.bottom, position.radial, 0.0);
                let facing = |surface: &FaceSurface| {
                    surface
                        .project_point(probe)
                        .map(|(u, v)| surface.normal(u, v))
                };
                match (facing(&old), facing(&surface)) {
                    (Some(before), Some(after)) if before.dot(after) > 0.0 => surface,
                    _ => return Err(unsupported("an arc whose cone turns its face over")),
                }
            }
            Side::Sweep(curve, net) => {
                FaceSurface::Nurbs(oriented_sweep(motion, screw, &old, curve, net)?)
            }
        };
        topo.face_mut(position.face)?.set_surface(surface);
    }
    Ok(())
}

/// The swept surface of `curve`, reversed in `v` when needed so its natural
/// normal keeps the straight surface's sense at the profile.
fn oriented_sweep(
    motion: &ConicalScrewMotion,
    screw: &ConicalScrewPlan,
    old: &FaceSurface,
    curve: NurbsCurve,
    mut net: Vec<Vec3>,
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
    let surface = motion.surface(screw, &curve, &net)?;
    if surface.normal(0.0, middle)?.dot(old_normal) >= 0.0 {
        return Ok(surface);
    }
    net.reverse();
    Ok(motion.surface(screw, &curve.reversed(), &net)?)
}

#[cfg(test)]
mod tests;
