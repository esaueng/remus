//! The throat of a worm wheel: its tips under the worm cut to a torus.
//!
//! A throated wheel's blank wraps the worm: over the middle of the face
//! width its tip surface is the torus swept by a circle of radius `ρ`
//! centered on the worm axis (at center distance `a`, half way across the
//! face) as the wheel turns, so its tips keep a constant clearance from the
//! worm's root. Every transverse section of the wheel at height `s` is the
//! wheel's tooth profile truncated at
//!
//! ```text
//! R(s) = a − √(ρ² − (s − c)²)
//! ```
//!
//! where `R(s)` is below the outside radius `r_e`; elsewhere the tips stay
//! on the outside cylinder.
//!
//! The wheel is built as the helical wheel with tips out to `r_e`, and each
//! tooth's tip is then rebuilt without a boolean. On the throat the tip land
//! (a cylinder) gives way to an exact torus patch between two exact circles
//! at the heights `c ∓ w` where the torus meets the outside cylinder. Each
//! flank stops at the curve where it meets the torus: the involute of the
//! base circle `r_b` at radius `R(s)`, turned by the twist,
//!
//! ```text
//! Θ(s) = θ₀ + k·s + σ·(inv r_e − inv R(s)),   inv r = t − atan t,
//!        t = √(r²/r_b² − 1)
//! ```
//!
//! (`σ` is +1 on a tooth's plus flank, −1 on its minus flank). That curve
//! is not rational, so it is a certified cubic Hermite fit: its fourth
//! derivative is bounded by carrying derivative bounds of `R`, `t`, `inv`
//! and `Θ` through Faà di Bruno's formula and Leibniz's rule.

use std::f64::consts::PI;

use remus_math::curves::Circle3D;
use remus_math::nurbs::hermite_fit::fit_cubic_hermite;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::jet::{Jet, compose, product, sqrt_jet};
use super::profile::MAX_SPANS_PER_FLANK;
use super::{Frame, GearEnd, GearError, GearFace, GearFaceRole, ToothSide};
use crate::OperationsError;

/// The torus a wheel's tips are cut to, in the wheel's frame.
#[derive(Debug, Clone, Copy)]
pub(super) struct Throat {
    /// Center distance `a`: the worm axis's distance from the wheel axis.
    pub(super) center_distance: f64,
    /// Tube radius `ρ`.
    pub(super) tube_radius: f64,
    /// Height of the worm axis along the wheel axis, `c`.
    pub(super) mid_height: f64,
    /// Half the throat's width along the axis, `w`.
    pub(super) half_width: f64,
    /// Outside (tip cylinder) radius `r_e`.
    pub(super) outside_radius: f64,
    /// Face width `b`.
    pub(super) face_width: f64,
}

/// The rebuilt wheel and the throat curves' proven deviation.
pub(super) struct Throated {
    pub(super) solid: SolidId,
    pub(super) face_roles: Vec<(FaceId, GearFace)>,
    /// Bound on any throat curve's distance from the exact flank and torus
    /// intersection, before the flank's own deviation.
    pub(super) curve_bound: f64,
}

/// A proven bound on the fourth derivative of the throat curve over the
/// throat, `|s − c| ≤ w`, in model units per unit height⁴.
fn throat_curve_bound(throat: &Throat, base_radius: f64, rate: f64) -> f64 {
    let Throat {
        center_distance: a,
        tube_radius: rho,
        half_width: w,
        outside_radius,
        ..
    } = *throat;
    // q = ρ² − (s − c)², over [ρ² − w², ρ²].
    let q: Jet = [rho * rho, 2.0 * w, 2.0, 0.0, 0.0];
    let g = compose(sqrt_jet(w.mul_add(-w, rho * rho), rho * rho), q);
    // R = a − g, over [a − ρ, r_e].
    let r: Jet = [outside_radius, g[1], g[2], g[3], g[4]];
    let rb2 = base_radius * base_radius;
    let r_low = a - rho;
    // p = R²/r_b² − 1 and t = √p.
    let p = product(r, r).map(|v| v / rb2);
    let (p_lo, p_hi) = (
        r_low * r_low / rb2 - 1.0,
        outside_radius * outside_radius / rb2 - 1.0,
    );
    let t = compose(sqrt_jet(p_lo, p_hi), p);
    let (t_lo, t_hi) = (p_lo.sqrt(), p_hi.sqrt());
    // inv(t) = t − atan t: derivatives t²/(1+t²), 2t/(1+t²)²,
    // (2 − 6t²)/(1+t²)³, (24t³ − 24t)/(1+t²)⁴.
    let d = t_lo.mul_add(t_lo, 1.0);
    let inv: Jet = [
        t_hi,
        1.0,
        2.0 * t_hi / (d * d),
        6.0f64.mul_add(t_hi * t_hi, 2.0) / d.powi(3),
        24.0 * t_hi.mul_add(t_hi * t_hi, t_hi) / d.powi(4),
    ];
    let inv_t = compose(inv, t);
    // Θ = θ₀ + k·s ± (inv r_e − inv R).
    let theta: Jet = [PI, rate.abs() + inv_t[1], inv_t[2], inv_t[3], inv_t[4]];
    // (cos Θ, sin Θ): every derivative of the unit circle has norm one.
    let circle = compose([1.0; 5], theta);
    // The height is linear in s, so only R·(cos Θ, sin Θ) bends.
    product(r, circle)[4]
}

/// One flank's throat curve in the wheel's frame: the point at height `s`
/// and its derivative.
struct ThroatCurve<'a> {
    throat: &'a Throat,
    frame: &'a Frame,
    base_radius: f64,
    rate: f64,
    /// Polar angle of the flank's tip at the first cap.
    theta0: f64,
    /// +1 on a plus flank, −1 on a minus flank.
    sign: f64,
}

impl ThroatCurve<'_> {
    fn involute(&self, radius: f64) -> f64 {
        let t = ((radius / self.base_radius).powi(2) - 1.0).max(0.0).sqrt();
        t - t.atan()
    }

    fn sample(&self, s: f64) -> (Point3, Vec3) {
        let Throat {
            center_distance: a,
            tube_radius: rho,
            mid_height: c,
            outside_radius,
            ..
        } = *self.throat;
        let u = s - c;
        let g = u.mul_add(-u, rho * rho).max(0.0).sqrt();
        let r = a - g;
        let dr = u / g;
        let rb2 = self.base_radius * self.base_radius;
        let t = (r * r / rb2 - 1.0).max(0.0).sqrt();
        let dt = r * dr / (rb2 * t);
        let dinv = t * t / t.mul_add(t, 1.0) * dt;
        let theta = self.sign.mul_add(
            self.involute(outside_radius) - self.involute(r),
            self.rate.mul_add(s, self.theta0),
        );
        let dtheta = self.sign.mul_add(-dinv, self.rate);
        let (sin, cos) = theta.sin_cos();
        let (x, y) = (r * cos, r * sin);
        let dx = dr.mul_add(cos, -y * dtheta);
        let dy = dr.mul_add(sin, x * dtheta);
        let f = self.frame;
        (
            f.origin + f.x * x + f.y * y + f.axis * s,
            f.x * dx + f.y * dy + f.axis,
        )
    }
}

fn invalid(reason: impl Into<String>) -> GearError {
    GearError::Operations(OperationsError::InvalidInput {
        reason: reason.into(),
    })
}

/// `v` turned by `angle` about the frame's axis.
fn turn(frame: &Frame, v: Vec3, angle: f64) -> Vec3 {
    let along = frame.axis * v.dot(frame.axis);
    let across = v - along;
    let (sin, cos) = angle.sin_cos();
    along + across * cos + frame.axis.cross(across) * sin
}

/// What one tooth's tip edges are replaced by.
struct ToothSplit {
    tip: FaceId,
    /// Vertical (helix) edge → (lower piece, throat curve, upper piece).
    verticals: Vec<(EdgeId, [EdgeId; 3])>,
    /// The bottom and top tip arcs, and the arcs at `c − w` and `c + w`.
    bottom: EdgeId,
    top: EdgeId,
    low_arc: EdgeId,
    high_arc: EdgeId,
}

/// Rebuild every tooth tip of a helical wheel (tips out to the throat's
/// outside radius) as two cylinder strips and a torus patch, and stop its
/// flanks at the torus.
///
/// `rate` is the twist in radians per unit height and `tolerance` the
/// throat curves' fit tolerance.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn cut_throat(
    topo: &mut Topology,
    solid: SolidId,
    face_roles: &[(FaceId, GearFace)],
    frame: &Frame,
    throat: &Throat,
    base_radius: f64,
    rate: f64,
    tolerance: f64,
    flank_bound: f64,
) -> Result<Throated, GearError> {
    let (s1, s2) = (
        throat.mid_height - throat.half_width,
        throat.mid_height + throat.half_width,
    );
    let height = |p: Point3| (p - frame.origin).dot(frame.axis);
    let linear = 1e-9 * throat.face_width.max(1.0);
    let fourth = throat_curve_bound(throat, base_radius, rate);
    let mut curve_bound: f64 = 0.0;
    let role_of = |face: FaceId| {
        face_roles
            .iter()
            .find(|entry| entry.0 == face)
            .map(|entry| entry.1.role)
    };
    let mut splits = Vec::new();
    for &(tip, role) in face_roles {
        let GearFaceRole::Tip { tooth } = role.role else {
            continue;
        };
        let tip_face = topo.face(tip)?;
        if !matches!(tip_face.surface(), FaceSurface::Cylinder(_)) {
            return Err(invalid(
                "a throated wheel needs cylindrical tip lands; this helix turns too far",
            ));
        }
        let wire = topo.wire(tip_face.outer_wire())?.edges().to_vec();
        let mut bottom = None;
        let mut top = None;
        let mut verticals = Vec::new();
        for oriented in &wire {
            let edge = topo.edge(oriented.edge())?;
            let start = topo.vertex(edge.start())?.point();
            match edge.curve() {
                EdgeCurve::Circle(_) if height(start).abs() <= linear => {
                    bottom = Some(oriented.edge());
                }
                EdgeCurve::Circle(_) => top = Some(oriented.edge()),
                EdgeCurve::NurbsCurve(_) => verticals.push(oriented.edge()),
                EdgeCurve::Line
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {
                    return Err(invalid("unexpected edge on a wheel's tip land"));
                }
            }
        }
        let (Some(bottom), Some(top), 2) = (bottom, top, verticals.len()) else {
            return Err(invalid("a wheel's tip land is not a four-sided strip"));
        };

        // Split each vertical helix at c ∓ w and join the pieces through the
        // throat curve of its flank.
        let mut pieces = Vec::new();
        let mut low_vertex = Vec::new();
        let mut high_vertex = Vec::new();
        for vertical in verticals {
            let side = face_roles
                .iter()
                .filter_map(|&(face, entry)| match entry.role {
                    GearFaceRole::Flank { tooth: t, side } if t == tooth => Some((face, side)),
                    _ => None,
                })
                .find_map(|(face, side)| {
                    let wire = topo.face(face).ok()?.outer_wire();
                    topo.wire(wire)
                        .ok()?
                        .edges()
                        .iter()
                        .any(|e| e.edge() == vertical)
                        .then_some(side)
                })
                .ok_or_else(|| invalid("a tip edge has no flank of its tooth"))?;
            let edge = topo.edge(vertical)?.clone();
            let bottom_point = topo.vertex(edge.start())?.point();
            if height(bottom_point).abs() > linear {
                return Err(invalid(
                    "a wheel's tip helix does not start at its first cap",
                ));
            }
            let vertex_tolerance = edge.tolerance().unwrap_or(0.0).max(flank_bound);
            let radial = bottom_point - frame.origin;
            let at = |s: f64| frame.origin + turn(frame, radial, rate * s) + frame.axis * s;
            let v1 = topo.add_vertex(Vertex::new(at(s1), vertex_tolerance));
            let v2 = topo.add_vertex(Vertex::new(at(s2), vertex_tolerance));

            let mut lower =
                Edge::with_tolerance(edge.start(), v1, edge.curve().clone(), edge.tolerance());
            lower.set_trim(Some((0.0, s1)));
            let mut upper =
                Edge::with_tolerance(v2, edge.end(), edge.curve().clone(), edge.tolerance());
            upper.set_trim(Some((s2, throat.face_width)));

            let curve = ThroatCurve {
                throat,
                frame,
                base_radius,
                rate,
                theta0: radial.dot(frame.y).atan2(radial.dot(frame.x)),
                sign: if side == ToothSide::Plus { 1.0 } else { -1.0 },
            };
            let fit = fit_cubic_hermite(
                |s| curve.sample(s),
                s1,
                s2,
                fourth,
                tolerance,
                MAX_SPANS_PER_FLANK,
            )
            .map_err(|error| match error {
                remus_math::MathError::ResourceLimitExceeded { .. } => GearError::refused(
                    super::GearRefusal::ToleranceTooTight,
                    format!(
                        "the throat curves need more than {MAX_SPANS_PER_FLANK} spans for a \
                         tolerance of {tolerance}"
                    ),
                ),
                other => other.into(),
            })?;
            let bound = (64.0 * f64::EPSILON).mul_add(
                throat.center_distance + throat.face_width,
                fit.remainder_bound,
            );
            curve_bound = curve_bound.max(bound);
            let domain = fit.curve.domain();
            let mut middle = Edge::with_tolerance(
                v1,
                v2,
                EdgeCurve::NurbsCurve(fit.curve),
                Some(bound + flank_bound),
            );
            middle.set_trim(Some(domain));
            let ids = [
                topo.add_edge(lower),
                topo.add_edge(middle),
                topo.add_edge(upper),
            ];
            pieces.push((vertical, ids));
            low_vertex.push((edge.start(), v1));
            high_vertex.push((edge.start(), v2));
        }

        // The tip arc turned to c ∓ w: a circle of the outside cylinder, and
        // so of the torus.
        let arc = topo.edge(bottom)?.clone();
        let EdgeCurve::Circle(circle) = arc.curve() else {
            return Err(invalid("a wheel's tip arc is not a circle"));
        };
        let raised = |s: f64, vertices: &[(VertexId, VertexId)]| -> Result<Edge, GearError> {
            let angle = rate * s;
            let moved = Circle3D::with_axes(
                circle.center() + frame.axis * s,
                circle.normal(),
                circle.radius(),
                turn(frame, circle.u_axis(), angle),
                turn(frame, circle.v_axis(), angle),
            )?;
            let find = |bottom_vertex: VertexId| {
                vertices
                    .iter()
                    .find(|entry| entry.0 == bottom_vertex)
                    .map(|entry| entry.1)
                    .ok_or_else(|| invalid("a tip arc does not end on its tip helices"))
            };
            let mut edge = Edge::with_tolerance(
                find(arc.start())?,
                find(arc.end())?,
                EdgeCurve::Circle(moved),
                arc.tolerance(),
            );
            edge.set_trim(arc.trim());
            Ok(edge)
        };
        let low_arc = raised(s1, &low_vertex)?;
        let high_arc = raised(s2, &high_vertex)?;
        splits.push(ToothSplit {
            tip,
            verticals: pieces,
            bottom,
            top,
            low_arc: topo.add_edge(low_arc),
            high_arc: topo.add_edge(high_arc),
        });
    }

    // Rebuild the tips and flanks, and keep every other face.
    let torus = ToroidalSurface::with_axis(
        frame.origin + frame.axis * throat.mid_height,
        throat.center_distance,
        throat.tube_radius,
        frame.axis,
    )?;
    let mut replaced: Vec<(FaceId, Vec<(FaceId, GearFace)>)> = Vec::new();
    for split in &splits {
        let tip = topo.face(split.tip)?.clone();
        let wire = topo.wire(tip.outer_wire())?.edges().to_vec();
        let tooth = match role_of(split.tip) {
            Some(GearFaceRole::Tip { tooth }) => tooth,
            _ => return Err(invalid("a wheel's tip face lost its role")),
        };
        // Each strip takes the tip wire with its edges substituted; the
        // throat's arcs run the same way as the arcs they replace, so the
        // strips share them in opposite directions.
        let rebuild = |bottom: EdgeId, top: EdgeId, piece: usize| -> Vec<OrientedEdge> {
            wire.iter()
                .map(|oriented| {
                    let edge = oriented.edge();
                    let replacement = if edge == split.bottom {
                        bottom
                    } else if edge == split.top {
                        top
                    } else {
                        split
                            .verticals
                            .iter()
                            .find(|entry| entry.0 == edge)
                            .map_or(edge, |entry| entry.1[piece])
                    };
                    OrientedEdge::new(replacement, oriented.is_forward())
                })
                .collect()
        };
        // The strips keep the tip's winding. A face reversed against the
        // tip's sense flips every edge's effective traversal, so its wire is
        // wound the other way to keep the same effective loop.
        let tip_reversed = tip.is_reversed();
        let mut make = |edges: Vec<OrientedEdge>, surface: FaceSurface, reversed: bool| {
            let edges = if reversed == tip_reversed {
                edges
            } else {
                edges
                    .into_iter()
                    .rev()
                    .map(|e| OrientedEdge::new(e.edge(), !e.is_forward()))
                    .collect()
            };
            let wire = topo.add_wire(Wire::new(edges, true)?);
            Ok::<_, GearError>(topo.add_face(if reversed {
                Face::new_reversed(wire, vec![], surface)
            } else {
                Face::new(wire, vec![], surface)
            }))
        };
        let low = make(
            rebuild(split.bottom, split.low_arc, 0),
            tip.surface().clone(),
            tip.is_reversed(),
        )?;
        let high = make(
            rebuild(split.high_arc, split.top, 2),
            tip.surface().clone(),
            tip.is_reversed(),
        )?;
        // The wheel's outward normal on the throat points toward the worm
        // axis, into the torus tube: against the torus's own normal there.
        let middle = make(
            rebuild(split.low_arc, split.high_arc, 1),
            FaceSurface::Torus(torus.clone()),
            true,
        )?;
        let tip_role = |half| GearFace {
            role: GearFaceRole::Tip { tooth },
            half: Some(half),
        };
        replaced.push((
            split.tip,
            vec![
                (low, tip_role(GearEnd::Start)),
                (middle, GearFaceRole::Throat { tooth }.into()),
                (high, tip_role(GearEnd::End)),
            ],
        ));
    }
    // Flanks: each tip helix becomes its lower piece, throat curve and
    // upper piece, walked the way the helix was.
    let chains: Vec<(EdgeId, [EdgeId; 3])> = splits
        .iter()
        .flat_map(|split| split.verticals.iter().copied())
        .collect();
    for &(face, role) in face_roles {
        if !matches!(role.role, GearFaceRole::Flank { .. }) {
            continue;
        }
        let data = topo.face(face)?.clone();
        let wire = topo.wire(data.outer_wire())?.edges().to_vec();
        if !wire
            .iter()
            .any(|e| chains.iter().any(|chain| chain.0 == e.edge()))
        {
            continue;
        }
        let mut edges = Vec::with_capacity(wire.len() + 2);
        for oriented in wire {
            match chains.iter().find(|chain| chain.0 == oriented.edge()) {
                Some((_, ids)) if oriented.is_forward() => {
                    edges.extend(ids.iter().map(|&id| OrientedEdge::new(id, true)));
                }
                Some((_, ids)) => {
                    edges.extend(ids.iter().rev().map(|&id| OrientedEdge::new(id, false)));
                }
                None => edges.push(oriented),
            }
        }
        let wire = topo.add_wire(Wire::new(edges, true)?);
        let surface = data.surface().clone();
        let rebuilt = topo.add_face(if data.is_reversed() {
            Face::new_reversed(wire, data.inner_wires().to_vec(), surface)
        } else {
            Face::new(wire, data.inner_wires().to_vec(), surface)
        });
        replaced.push((face, vec![(rebuilt, role)]));
    }

    let mut faces = Vec::new();
    let mut roles = Vec::new();
    for face in remus_topology::explorer::solid_faces(topo, solid)? {
        if let Some((_, new)) = replaced.iter().find(|entry| entry.0 == face) {
            for &(id, role) in new {
                faces.push(id);
                roles.push((id, role));
            }
        } else {
            faces.push(face);
            if let Some(entry) = face_roles.iter().find(|entry| entry.0 == face) {
                roles.push(*entry);
            }
        }
    }
    let shell = topo.add_shell(Shell::new(faces)?);
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    Ok(Throated {
        solid,
        face_roles: roles,
        curve_bound,
    })
}
