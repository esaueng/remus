//! Similar extrusion: a planar profile perpendicular to an axis, carried
//! along the axis while it scales uniformly about it (linearly in the
//! advance) and, optionally, turns about it (a twisted, tapered-by-scale
//! sweep along a straight path).
//!
//! At advance `s ∈ [0, h]` a profile point `p` sits at
//!
//! ```text
//! M_s(p) = o + (p − o)∥ + σ(s)·R(ks)·(p − o)⊥ + s·a,   σ(s) = 1 + (σ₁ − 1)·s / h
//! ```
//!
//! with `R(φ)` the rotation by `φ` about `a`. That is the
//! [`ConicalScrewMotion`] of turn rate `k` and radial rate `τ = (σ₁ − 1)/h`
//! applied to the pair `(p, (p − o)⊥)`: the point's own offset from the axis
//! is its direction. The offset is affine in `p`, so a profile curve
//! `C(v) = Σ R_i(v)·P_i` swept with the net `U_i = (P_i − o)⊥` on its own
//! control points is exactly the sweep of `C` (see the conical screw module):
//! any line, arc or untrimmed spline, on the axis or off it, sweeps exactly.
//!
//! Like the twisted extrusions it has the topology of the straight one, so
//! the straight extrusion builds it and this pass only rewrites geometry:
//!
//! - top cap: every profile point moves to `M_h(p)`, a similarity, so lines
//!   stay lines and arcs stay arcs (radius `σ₁·r`); the cap plane is
//!   unchanged;
//! - without a turn (`k = 0`) the motion is affine in `s`: vertical edges
//!   stay lines, a line sweeps a plane (its top image is parallel to it), an
//!   arc concentric with the axis sweeps its exact cone, and anything else
//!   is the motion's exact ruled NURBS (the cubic fit of a linear path is
//!   that path);
//! - with a turn, vertical edges become fitted conical helices and side
//!   faces the certified swept NURBS (a concentric arc keeps its exact cone
//!   while its strip covers less than one turn). All fits share one
//!   [`ConicalScrewPlan`], so a face's boundary rows are the vertical edges'
//!   curves bit for bit;
//! - a wire that is one closed circle concentric with the axis becomes its
//!   exact cone with a straight seam: the top vertex is placed unturned on
//!   the scaled circle, which the motion maps onto itself.
//!
//! Refused (typed `InvalidInput`): a profile not perpendicular to the axis, an
//! end scale that is not finite or outside `[1e-3, 1e3]`, a closed edge off
//! the axis, a trimmed spline, an ellipse or open conic edge.

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

/// Relative tolerance for treating a circle as concentric with the axis.
const CONCENTRIC_TOLERANCE: f64 = 1e-9;

/// Largest absolute offset of a kept concentric circle's centre from the
/// axis at the far end: a tenth of the modelling tolerance, so the circle
/// kept centred stays on its seam vertex.
const CONCENTRIC_ABSOLUTE: f64 = 1e-8;

/// The end scales a scaled extrusion builds: past them the far section
/// shrinks or grows by more than its own digits carry (a scale of `1e-20`
/// rounds `σ₁ − 1` to `−1` and collapses the cap onto the axis).
pub const MIN_END_SCALE: f64 = 1e-3;
/// See [`MIN_END_SCALE`].
pub const MAX_END_SCALE: f64 = 1e3;

/// Rounding allowance of a cone face, in units of `ε` times the distance
/// from the section to the cone's apex.
const CONE_ROUNDING: f64 = 64.0;

/// The motion and everything that is constant over one extrusion.
struct Similar {
    motion: ConicalScrewMotion,
    origin: Point3,
    axis: Vec3,
    /// `σ₁`, the scale at the far end.
    end_scale: f64,
    /// Whether the profile turns (`k ≠ 0`).
    turns: bool,
}

impl Similar {
    /// `(p − o)⊥`: the point's direction, which is its own offset.
    fn offset(&self, p: Point3) -> Vec3 {
        let d = p - self.origin;
        d - self.axis * d.dot(self.axis)
    }

    fn at(&self, p: Point3, s: f64) -> Point3 {
        self.motion.point(p, self.offset(p), s)
    }

    fn end(&self, p: Point3) -> Point3 {
        self.at(p, self.motion.height())
    }

    /// The turned (not scaled) image of a direction at the far end.
    fn turn(&self, v: Vec3) -> Vec3 {
        self.motion.turn_vector(v, self.motion.height())
    }

    /// Whether `circle` is centred on the axis closely enough that keeping it
    /// centred there stays within the tolerance at the far end too: the
    /// scale magnifies the centre's offset by `σ₁`.
    fn concentric(&self, circle: &Circle3D) -> bool {
        let scale = circle.radius().max(1.0);
        let offset = self.offset(circle.center()).length() * self.end_scale.max(1.0);
        offset <= CONCENTRIC_TOLERANCE * scale
            && offset <= CONCENTRIC_ABSOLUTE
            && circle.normal().cross(self.axis).length() <= CONCENTRIC_TOLERANCE
    }

    /// The exact cone a concentric arc of `circle` sweeps (`σ₁ ≠ 1`).
    fn cone(&self, circle: &Circle3D) -> Result<ConicalSurface, OperationsError> {
        let height = self.motion.height();
        // Radius `r·σ(s)`: slope `r·|σ₁ − 1| / h`, widening along `±a`.
        let growth = self.end_scale - 1.0;
        let slope = circle.radius() * growth.abs() / height;
        if !(slope > 0.0 && slope.is_finite()) {
            return Err(unsupported("an arc whose radius does not change"));
        }
        let widening = self.axis * growth.signum();
        let apex = circle.center() - widening * (circle.radius() / slope);
        Ok(ConicalSurface::with_ref_dir(
            apex,
            widening,
            slope.recip().atan(),
            circle.u_axis(),
        )?)
    }

    /// The top cap's `circle`, which the straight extrusion already carried
    /// the full height along the axis, moved on to its place in the far
    /// section: its centre's offset from the axis scaled by `σ₁` and turned,
    /// its axes turned, its radius scaled by `σ₁`.
    fn end_circle(&self, top: &Circle3D) -> Result<Circle3D, OperationsError> {
        let offset = self.offset(top.center());
        let moved = self.turn(offset) * self.end_scale;
        Ok(Circle3D::with_axes(
            top.center() + (moved - offset),
            self.turn(top.normal()),
            top.radius() * self.end_scale,
            self.turn(top.u_axis()),
            self.turn(top.v_axis()),
        )?)
    }

    fn end_nurbs(&self, curve: &NurbsCurve) -> Result<NurbsCurve, OperationsError> {
        Ok(NurbsCurve::new(
            curve.degree(),
            curve.knots().to_vec(),
            curve
                .control_points()
                .iter()
                .map(|&p| self.end(p))
                .collect(),
            curve.weights().to_vec(),
        )?)
    }

    fn net(&self, curve: &NurbsCurve) -> Vec<Vec3> {
        curve
            .control_points()
            .iter()
            .map(|&p| self.offset(p))
            .collect()
    }
}

/// Extrude the planar `face` by `distance` along `direction` while scaling
/// it uniformly about the axis through `axis_origin` from 1 to `end_scale`
/// and turning it `rate` radians per unit length (see the
/// [module docs](self)).
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] when the profile is not a plane
/// perpendicular to `direction`, `end_scale` is not finite and positive, or
/// the profile holds a curve this pass cannot sweep (a closed edge off the
/// axis, a trimmed spline, an ellipse or open conic); the straight
/// extrusion's errors; math errors when `tolerance` needs more than
/// `max_spans` spans.
#[allow(clippy::too_many_arguments, clippy::redundant_pub_crate)]
pub(crate) fn similar_extrude_impl(
    topo: &mut Topology,
    face: FaceId,
    axis_origin: Point3,
    direction: Vec3,
    distance: f64,
    rate: f64,
    end_scale: f64,
    tolerance: f64,
    max_spans: usize,
) -> Result<TwistedExtrusion, OperationsError> {
    if !(end_scale.is_finite() && (MIN_END_SCALE..=MAX_END_SCALE).contains(&end_scale)) {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "a scaled extrusion's end scale must be between {MIN_END_SCALE} and \
                 {MAX_END_SCALE}, got {end_scale}"
            ),
        });
    }
    super::nesting::refuse_nested_profile(topo, face, "scaled extrusion")?;
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
            reason: "a scaled extrusion needs a planar profile perpendicular to its axis"
                .to_owned(),
        });
    }
    let (solid, construction) = extrude_impl(topo, face, axis, distance)?;
    let similar = Similar {
        motion: ConicalScrewMotion::new(
            axis_origin,
            axis,
            rate,
            (end_scale - 1.0) / distance,
            distance,
        )?,
        origin: axis_origin,
        axis,
        end_scale,
        turns: rate != 0.0,
    };

    let mut plans = vec![plan_wire(
        topo,
        &similar,
        &construction.outer,
        &construction.outer_sides,
    )?];
    for hole in &construction.holes {
        plans.push(plan_wire(topo, &similar, &hole.inner, &hole.sides)?);
    }
    // Every path stays within `max(1, σ₁)` of its start's distance from the
    // axis; directions are those distances.
    let widest = plans.iter().map(|plan| plan.widest).fold(0.0, f64::max);
    let reach = widest * end_scale.max(1.0);
    let screw = similar
        .motion
        .plan(reach, widest.max(1.0), tolerance, max_spans)?;
    // A cone's apex lies `h / |σ₁ − 1|` from the profile; evaluating it
    // loses about `ε` times that much.
    let cone_rounding = if (end_scale - 1.0).abs() > 0.0 {
        CONE_ROUNDING * f64::EPSILON * (distance / (end_scale - 1.0).abs() + reach)
    } else {
        0.0
    };
    let deviation_bound =
        (64.0 * f64::EPSILON).mul_add(reach + distance, screw.remainder_bound) + cone_rounding;
    // Every far-cap edge must keep a length: a far section that shrinks
    // below the digits its coordinates carry would collapse onto a point.
    let mut far_edges = Vec::new();
    for plan in &plans {
        far_edges.extend(plan.positions.iter().map(|position| {
            let (a, b) = position.ends;
            (position.top_edge, (b - a).length() * end_scale)
        }));
    }
    for plan in plans {
        apply(topo, &similar, &screw, plan, deviation_bound)?;
    }
    let linear = remus_math::tolerance::Tolerance::new().linear;
    for (edge, expected) in far_edges {
        let edge = topo.edge(edge)?;
        let (a, b) = (
            topo.vertex(edge.start())?.point(),
            topo.vertex(edge.end())?.point(),
        );
        // The far section is the profile scaled by `σ₁`: each chord between
        // its vertices is `σ₁` times the profile's, to within what the
        // coordinates carry there, and that must be a small part of it.
        let far = a.0.iter().chain(&b.0).fold(0.0_f64, |m, c| m.max(c.abs()));
        let allowance = linear.max(64.0 * f64::EPSILON * far);
        if allowance > 1e-3 * expected || ((a - b).length() - expected).abs() > allowance {
            return Err(OperationsError::InvalidInput {
                reason: "a scaled extrusion's far section is too small to hold at its distance \
                         from the origin: its edges lose their length to rounding (a larger end \
                         scale, or a profile nearer the origin)"
                    .to_owned(),
            });
        }
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
    /// A line swept without a turn: the plane through it and its image.
    Plane,
    /// A concentric arc: its exact cone.
    Cone(ConicalSurface),
    /// Swept from this profile curve (oriented like its bottom edge, ends
    /// snapped to the edge's vertices) with its offsets as directions.
    Sweep(NurbsCurve),
}

/// One position of a wire, read before any mutation.
struct Position {
    bottom: Point3,
    /// The bottom edge's own two ends.
    ends: (Point3, Point3),
    top_vertex: VertexId,
    vertical: EdgeId,
    top_edge: EdgeId,
    top_curve: EdgeCurve,
    top_trim: Option<(f64, f64)>,
    face: FaceId,
    side: Side,
}

/// A wire that is one closed concentric circle: its cone and seam.
struct Ring {
    bottom: Point3,
    top_vertex: VertexId,
    seam: EdgeId,
    top_edge: EdgeId,
    top_curve: Circle3D,
    top_trim: Option<(f64, f64)>,
    face: FaceId,
    cone: ConicalSurface,
}

struct WirePlan {
    positions: Vec<Position>,
    ring: Option<Ring>,
    /// Largest distance of any control point from the axis.
    widest: f64,
}

fn unsupported(what: &str) -> OperationsError {
    OperationsError::InvalidInput {
        reason: format!("a scaled extrusion cannot sweep {what}"),
    }
}

#[allow(clippy::too_many_lines)]
fn plan_wire(
    topo: &Topology,
    similar: &Similar,
    wire: &ExtrudedWire,
    sides: &[FaceId],
) -> Result<WirePlan, OperationsError> {
    let n = wire.oriented.len();
    if n == 1 {
        let edge = topo.edge(wire.edge_ids[0])?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Err(unsupported("a closed edge off the axis"));
        };
        if !similar.concentric(circle) {
            return Err(unsupported("a closed edge off the axis"));
        }
        let top = topo.edge(wire.top_edge_ids[0])?;
        let EdgeCurve::Circle(top_circle) = top.curve() else {
            return Err(unsupported("a top edge unlike its profile edge"));
        };
        // The cap circle keeps its own frame: the motion maps a concentric
        // circle onto itself, so the seam stays a straight generator.
        let top_curve = Circle3D::with_axes(
            top_circle.center(),
            top_circle.normal(),
            top_circle.radius() * similar.end_scale,
            top_circle.u_axis(),
            top_circle.v_axis(),
        )?;
        return Ok(WirePlan {
            positions: Vec::new(),
            ring: Some(Ring {
                bottom: wire.positions[0],
                top_vertex: wire.top_verts[0],
                seam: wire.vertical_edge_ids[0],
                top_edge: wire.top_edge_ids[0],
                top_curve,
                top_trim: top.trim(),
                face: sides[0],
                cone: similar.cone(circle)?,
            }),
            widest: circle.radius(),
        });
    }
    let mut widest: f64 = 0.0;
    let mut positions = Vec::with_capacity(n);
    for i in 0..n {
        let bottom_edge = topo.edge(wire.edge_ids[i])?;
        let start = topo.vertex(bottom_edge.start())?.point();
        let end = topo.vertex(bottom_edge.end())?.point();
        let bottom = wire.positions[i];
        widest = widest.max(similar.offset(bottom).length());
        let side = match bottom_edge.curve() {
            EdgeCurve::Circle(circle) => {
                let (t0, t1) =
                    bottom_edge
                        .strict_domain()
                        .map_err(|error| OperationsError::InvalidInput {
                            reason: format!("scaled extrusion arc has no domain: {error}"),
                        })?;
                // A cone face holds the strip only while it covers less
                // than one turn; past that it would cover itself twice.
                let strip = (t1 - t0) + similar.motion.total_turn().abs();
                if similar.concentric(circle) && strip < TAU * (1.0 - CONCENTRIC_TOLERANCE) {
                    Side::Cone(similar.cone(circle)?)
                } else {
                    Side::Sweep(snap(arc_nurbs(circle, t0, t1)?, start, end)?)
                }
            }
            EdgeCurve::Line if !similar.turns => Side::Plane,
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
            EdgeCurve::Ellipse(_) | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                return Err(unsupported("an ellipse or open conic edge"));
            }
        };
        if let Side::Sweep(curve) = &side {
            for &p in curve.control_points() {
                widest = widest.max(similar.offset(p).length());
            }
        }
        let top = topo.edge(wire.top_edge_ids[i])?;
        let top_curve = match (top.curve(), &side) {
            (EdgeCurve::Line, _) => EdgeCurve::Line,
            (EdgeCurve::Circle(circle), _) => EdgeCurve::Circle(similar.end_circle(circle)?),
            (EdgeCurve::NurbsCurve(_), Side::Sweep(curve)) => {
                let moved = EdgeCurve::NurbsCurve(similar.end_nurbs(curve)?);
                if wire.oriented[i].is_forward() {
                    moved
                } else {
                    reverse_edge_curve(&moved)?
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
            ends: (start, end),
            top_vertex: wire.top_verts[i],
            vertical: wire.vertical_edge_ids[i],
            top_edge: wire.top_edge_ids[i],
            top_curve,
            top_trim: top.trim(),
            face: sides[i],
            side,
        });
    }
    Ok(WirePlan {
        positions,
        ring: None,
        widest,
    })
}

/// `surface`, unless it faces the other way from `old` at `probe`.
fn same_facing(
    old: &FaceSurface,
    surface: FaceSurface,
    probe: Point3,
) -> Result<FaceSurface, OperationsError> {
    let facing = |surface: &FaceSurface| {
        surface
            .project_point(probe)
            .map(|(u, v)| surface.normal(u, v))
    };
    match (facing(old), facing(&surface)) {
        (Some(before), Some(after)) if before.dot(after) > 0.0 => Ok(surface),
        _ => Err(unsupported("an arc whose cone turns its face over")),
    }
}

fn apply(
    topo: &mut Topology,
    similar: &Similar,
    screw: &ConicalScrewPlan,
    plan: WirePlan,
    deviation_bound: f64,
) -> Result<(), OperationsError> {
    let height = similar.motion.height();
    if let Some(ring) = plan.ring {
        // Unturned on the scaled circle: the seam is the cone's generator.
        let top = ring.bottom
            + similar.axis * height
            + similar.offset(ring.bottom) * (similar.end_scale - 1.0);
        topo.vertex_mut(ring.top_vertex)?.set_point(top);
        let seam = topo.edge_mut(ring.seam)?;
        let tolerance = seam.tolerance().unwrap_or(0.0).max(deviation_bound);
        seam.set_tolerance(Some(tolerance))?;
        let top_edge = topo.edge_mut(ring.top_edge)?;
        top_edge.set_curve(EdgeCurve::Circle(ring.top_curve));
        top_edge.set_trim(ring.top_trim);
        let old = topo.face(ring.face)?.surface().clone();
        let surface = same_facing(&old, FaceSurface::Cone(ring.cone), ring.bottom)?;
        topo.face_mut(ring.face)?.set_surface(surface);
    }
    for position in plan.positions {
        let top_point = similar.end(position.bottom);
        topo.vertex_mut(position.top_vertex)?.set_point(top_point);

        let vertical = topo.edge_mut(position.vertical)?;
        let tolerance = vertical.tolerance().unwrap_or(0.0).max(deviation_bound);
        if similar.turns {
            let helix =
                similar
                    .motion
                    .helix(screw, position.bottom, similar.offset(position.bottom))?;
            vertical.set_curve(EdgeCurve::NurbsCurve(helix));
            vertical.set_trim(Some((0.0, height)));
        }
        vertical.set_tolerance(Some(tolerance))?;

        let top = topo.edge_mut(position.top_edge)?;
        top.set_curve(position.top_curve);
        top.set_trim(position.top_trim);

        let old = topo.face(position.face)?.surface().clone();
        let surface = match position.side {
            Side::Plane => {
                let FaceSurface::Plane {
                    normal: old_normal, ..
                } = old
                else {
                    return Err(unsupported("a line whose wall is not a plane"));
                };
                let (a, b) = position.ends;
                // The wall holds the line and its image, which is parallel
                // to it, so the line and one image point span it.
                let mut normal = (b - a).cross(similar.end(a) - a).normalize()?;
                if normal.dot(old_normal) < 0.0 {
                    normal = -normal;
                }
                FaceSurface::Plane {
                    normal,
                    d: crate::dot_normal_point(normal, a),
                }
            }
            Side::Cone(cone) => same_facing(&old, FaceSurface::Cone(cone), position.bottom)?,
            Side::Sweep(curve) => FaceSurface::Nurbs(oriented_sweep(similar, screw, &old, curve)?),
        };
        topo.face_mut(position.face)?.set_surface(surface);
    }
    Ok(())
}

/// The swept surface of `curve`, reversed in `v` when needed so its natural
/// normal keeps the straight surface's sense at the profile.
fn oriented_sweep(
    similar: &Similar,
    screw: &ConicalScrewPlan,
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
                        reason: "scaled extrusion could not orient a side face".to_owned(),
                    })?;
            surface.normal(u, v)
        }
    };
    let surface = similar
        .motion
        .surface(screw, &curve, &similar.net(&curve))?;
    if surface.normal(0.0, middle)?.dot(old_normal) >= 0.0 {
        return Ok(surface);
    }
    let reversed = curve.reversed();
    Ok(similar
        .motion
        .surface(screw, &reversed, &similar.net(&reversed))?)
}
