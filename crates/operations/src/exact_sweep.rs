//! Exact path sweep along a chain of lines and circular arcs.
//!
//! A planar profile is carried **in place** by the rigid motions its path
//! describes: a line edge translates it by the line's vector, a circular arc
//! turns it about the arc's axis by the arc's angle. Along a tangent-continuous
//! chain of lines and arcs those motions are exactly the rotation-minimizing
//! frame (the Frenet frame of a planar arc is rotation-minimizing, and a turn
//! about a circle's axis carries every vector normal to the circle's tangent
//! the way that frame does), so nothing is sampled and nothing is fitted:
//!
//! - every profile edge sweeps into one face per path segment. A line moved
//!   along a line is a [`FaceSurface::Plane`], a circle moved along its own
//!   axis a cylinder, a line turned about an axis it is coplanar with a plane,
//!   cylinder or cone, and a circle turned about an axis in its plane a
//!   torus. Every other pair is the exact rational tensor product of the
//!   edge's NURBS with the motion — linear for a translation, rational
//!   quadratic arc spans (Piegl & Tiller's surface of revolution) for a turn;
//! - every profile vertex sweeps into one rail per segment, a line or an
//!   exact circular arc;
//! - the start cap is the profile's own plane and edges, every joint section
//!   and the end cap are the profile moved rigidly, edge for edge.
//!
//! A closed chain whose motions return the profile to where it started builds
//! a ring with no caps.
//!
//! ## Refusals
//!
//! A path edge that is not a line or a circular arc, a chain whose edges do
//! not meet end to end, a corner where the chain is not tangent-continuous, a
//! non-planar profile, a profile the motion does not cross transversally (it
//! would fold through itself: the profile touches or crosses an arc's axis,
//! or lies along a line), a closed chain that does not bring the profile back
//! to its start, and a chain whose non-adjacent segments pass closer than the
//! profile's reach (the swept body could meet itself). Every refusal leaves
//! the topology unchanged.

use std::f64::consts::FRAC_PI_2;

use remus_math::context::OperationContext;
use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::reuse::{ReuseError, certify_rotational_crossing};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, ToroidalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::helix::in_place::{Loop, is_ccw, read_profile_loop};

/// Largest angle between the incoming and outgoing tangents at a joint that
/// still counts as tangent-continuous, in radians.
const G1_ANGLE_TOLERANCE: f64 = 1e-6;

/// The sharpest corner a mitre takes, in radians (175°): beyond it the
/// bisector plane runs almost along both runs.
const MAX_MITRE_TURN: f64 = 175.0 * std::f64::consts::PI / 180.0;

/// Bounds on the points per arc in the self-clearance check.
const CLEARANCE_POINTS: (usize, usize) = (16, 4096);

/// Relative scale for geometric refusal margins.
const RELATIVE_MARGIN: f64 = 1e-9;

fn invalid(reason: impl Into<String>) -> OperationsError {
    OperationsError::InvalidInput {
        reason: reason.into(),
    }
}

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: "exact sweep",
        reason: reason.into(),
    }
}

/// Turn `v` by `angle` about the unit `axis` (Rodrigues).
fn rotate_vec(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    v * cos + axis.cross(v) * sin + axis * (axis.dot(v) * (1.0 - cos))
}

fn to_vec(p: Point3) -> Vec3 {
    Vec3::new(p.x(), p.y(), p.z())
}

fn to_point(v: Vec3) -> Point3 {
    Point3::new(v.x(), v.y(), v.z())
}

/// A rigid map `p ↦ R·p + t`, `R` stored as the images of the unit axes.
#[derive(Debug, Clone, Copy)]
struct Rigid {
    columns: [Vec3; 3],
    translation: Vec3,
}

impl Rigid {
    const IDENTITY: Self = Self {
        columns: [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ],
        translation: Vec3::new(0.0, 0.0, 0.0),
    };

    fn vec(&self, v: Vec3) -> Vec3 {
        self.columns[0] * v.x() + self.columns[1] * v.y() + self.columns[2] * v.z()
    }

    fn point(&self, p: Point3) -> Point3 {
        to_point(self.vec(to_vec(p)) + self.translation)
    }

    /// `self` after `first`: `p ↦ self(first(p))`.
    fn after(&self, first: &Self) -> Self {
        Self {
            columns: first.columns.map(|c| self.vec(c)),
            translation: self.vec(first.translation) + self.translation,
        }
    }

    fn curve(&self, curve: &EdgeCurve) -> Result<EdgeCurve, OperationsError> {
        Ok(match curve {
            EdgeCurve::Line => EdgeCurve::Line,
            EdgeCurve::Circle(c) => EdgeCurve::Circle(Circle3D::with_axes(
                self.point(c.center()),
                self.vec(c.normal()),
                c.radius(),
                self.vec(c.u_axis()),
                self.vec(c.v_axis()),
            )?),
            EdgeCurve::Ellipse(e) => EdgeCurve::Ellipse(Ellipse3D::with_axes(
                self.point(e.center()),
                self.vec(e.normal()),
                e.semi_major(),
                e.semi_minor(),
                self.vec(e.u_axis()),
                self.vec(e.v_axis()),
            )?),
            EdgeCurve::Parabola(p) => EdgeCurve::Parabola(Parabola3D::with_axes(
                self.point(p.vertex()),
                self.vec(p.axis_dir()),
                self.vec(p.u_axis()),
                p.focal_length(),
            )?),
            EdgeCurve::Hyperbola(h) => EdgeCurve::Hyperbola(Hyperbola3D::with_axes(
                self.point(h.center()),
                self.vec(h.normal()),
                self.vec(h.u_axis()),
                h.semi_major(),
                h.semi_minor(),
            )?),
            EdgeCurve::NurbsCurve(n) => EdgeCurve::NurbsCurve(self.nurbs(n)?),
        })
    }

    fn nurbs(&self, n: &NurbsCurve) -> Result<NurbsCurve, OperationsError> {
        Ok(NurbsCurve::new(
            n.degree(),
            n.knots().to_vec(),
            n.control_points().iter().map(|&p| self.point(p)).collect(),
            n.weights().to_vec(),
        )?)
    }
}

/// One path segment's motion.
#[derive(Debug, Clone, Copy)]
enum Motion {
    /// Move by a vector (a line edge).
    Translate(Vec3),
    /// Turn by `angle > 0` right-handed about the unit `axis` through
    /// `origin` (a circular arc edge).
    Rotate {
        origin: Point3,
        axis: Vec3,
        angle: f64,
    },
}

/// One control row of a motion: where it carries a point, and the weight.
#[derive(Debug, Clone, Copy)]
struct Row {
    fraction: f64,
    /// Radial scale about a turn's axis: 1 on span ends, `1 / cos(α/2)` on
    /// span middles; unused by a translation.
    scale: f64,
    weight: f64,
}

impl Motion {
    /// The rigid map of the whole motion.
    fn rigid(&self) -> Rigid {
        match *self {
            Self::Translate(d) => Rigid {
                translation: d,
                ..Rigid::IDENTITY
            },
            Self::Rotate {
                origin,
                axis,
                angle,
            } => {
                let columns = Rigid::IDENTITY.columns.map(|c| rotate_vec(c, axis, angle));
                let rotation = Rigid {
                    columns,
                    translation: Vec3::new(0.0, 0.0, 0.0),
                };
                let o = to_vec(origin);
                Rigid {
                    columns,
                    translation: o - rotation.vec(o),
                }
            }
        }
    }

    /// The velocity of `p` at the start of the motion (direction only).
    fn velocity(&self, p: Point3) -> Vec3 {
        match *self {
            Self::Translate(d) => d,
            Self::Rotate { origin, axis, .. } => axis.cross(p - origin),
        }
    }

    /// Degree, knots and control rows along the motion.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    fn rows(&self) -> (usize, Vec<f64>, Vec<Row>) {
        match *self {
            Self::Translate(_) => (
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![
                    Row {
                        fraction: 0.0,
                        scale: 1.0,
                        weight: 1.0,
                    },
                    Row {
                        fraction: 1.0,
                        scale: 1.0,
                        weight: 1.0,
                    },
                ],
            ),
            Self::Rotate { angle, .. } => {
                let spans = ((angle / FRAC_PI_2).ceil() as usize).max(1);
                let n = spans as f64;
                let weight = (0.5 * angle / n).cos();
                let mut rows = Vec::with_capacity(2 * spans + 1);
                for k in 0..spans {
                    let k = k as f64;
                    rows.push(Row {
                        fraction: k / n,
                        scale: 1.0,
                        weight: 1.0,
                    });
                    rows.push(Row {
                        fraction: (k + 0.5) / n,
                        scale: 1.0 / weight,
                        weight,
                    });
                }
                rows.push(Row {
                    fraction: 1.0,
                    scale: 1.0,
                    weight: 1.0,
                });
                let mut knots = Vec::with_capacity(2 * spans + 4);
                knots.extend([0.0; 3]);
                for k in 1..spans {
                    let t = k as f64 / n;
                    knots.extend([t, t]);
                }
                knots.extend([1.0; 3]);
                (2, knots, rows)
            }
        }
    }

    /// The control point of `p`'s trajectory on `row`.
    fn control_point(&self, p: Point3, row: &Row) -> Point3 {
        match *self {
            Self::Translate(d) => p + d * row.fraction,
            Self::Rotate {
                origin,
                axis,
                angle,
            } => {
                let d = p - origin;
                let z = d.dot(axis);
                let radial = d - axis * z;
                let (sin, cos) = (angle * row.fraction).sin_cos();
                let turned = radial * cos + axis.cross(radial) * sin;
                origin + axis * z + turned * row.scale
            }
        }
    }

    /// The exact tensor-product surface swept by `curve`: `u` is the curve's
    /// own parameter, `v ∈ [0, 1]` runs along the motion.
    fn nurbs_surface(&self, curve: &NurbsCurve) -> Result<NurbsSurface, OperationsError> {
        let (degree, knots, rows) = self.rows();
        let control_points: Vec<Vec<Point3>> = curve
            .control_points()
            .iter()
            .map(|&p| rows.iter().map(|row| self.control_point(p, row)).collect())
            .collect();
        let weights: Vec<Vec<f64>> = curve
            .weights()
            .iter()
            .map(|&w| rows.iter().map(|row| w * row.weight).collect())
            .collect();
        Ok(NurbsSurface::new(
            curve.degree(),
            degree,
            curve.knots().to_vec(),
            knots,
            control_points,
            weights,
        )?)
    }

    /// The rail one point sweeps: a line, or an exact circular arc.
    fn rail(&self, p: Point3) -> Result<(EdgeCurve, Option<(f64, f64)>), OperationsError> {
        match *self {
            Self::Translate(_) => Ok((EdgeCurve::Line, None)),
            Self::Rotate {
                origin,
                axis,
                angle,
            } => {
                let foot = origin + axis * (p - origin).dot(axis);
                let offset = p - foot;
                let radius = offset.length();
                let u = offset.normalize()?;
                let circle = Circle3D::with_axes(foot, axis, radius, u, axis.cross(u))?;
                Ok((EdgeCurve::Circle(circle), Some((0.0, angle))))
            }
        }
    }
}

/// One path edge, oriented along the chain.
#[derive(Debug, Clone, Copy)]
struct Segment {
    start: Point3,
    end: Point3,
    start_tangent: Vec3,
    end_tangent: Vec3,
    motion: Motion,
}

impl Segment {
    /// The segment's length along the path.
    fn length(&self) -> f64 {
        match self.motion {
            Motion::Translate(d) => d.length(),
            Motion::Rotate {
                origin,
                axis,
                angle,
            } => {
                let d = self.start - origin;
                (d - axis * d.dot(axis)).length() * angle.abs()
            }
        }
    }

    /// `count + 1` points evenly spaced along the segment: consecutive ones
    /// are `length / count` apart along it, so no point of the segment is
    /// farther than half that from one of them.
    #[allow(clippy::cast_precision_loss)]
    fn points(&self, count: usize) -> Vec<Point3> {
        (0..=count)
            .map(|k| {
                let f = k as f64 / count as f64;
                match self.motion {
                    Motion::Translate(d) => self.start + d * f,
                    Motion::Rotate {
                        origin,
                        axis,
                        angle,
                    } => origin + rotate_vec(self.start - origin, axis, angle * f),
                }
            })
            .collect()
    }

    /// Two halves of an arc, meeting tangentially at its middle.
    fn halves(&self) -> Option<(Self, Self)> {
        let Motion::Rotate {
            origin,
            axis,
            angle,
        } = self.motion
        else {
            return None;
        };
        let half = Motion::Rotate {
            origin,
            axis,
            angle: angle / 2.0,
        };
        let middle = origin + rotate_vec(self.start - origin, axis, angle / 2.0);
        let tangent = rotate_vec(self.start_tangent, axis, angle / 2.0);
        Some((
            Self {
                start: self.start,
                end: middle,
                start_tangent: self.start_tangent,
                end_tangent: tangent,
                motion: half,
            },
            Self {
                start: middle,
                end: self.end,
                start_tangent: tangent,
                end_tangent: self.end_tangent,
                motion: half,
            },
        ))
    }

    fn reversed(&self) -> Self {
        let motion = match self.motion {
            Motion::Translate(d) => Motion::Translate(-d),
            Motion::Rotate {
                origin,
                axis,
                angle,
            } => Motion::Rotate {
                origin,
                axis: -axis,
                angle,
            },
        };
        Self {
            start: self.end,
            end: self.start,
            start_tangent: -self.end_tangent,
            end_tangent: -self.start_tangent,
            motion,
        }
    }
}

/// Read one path edge in its natural direction.
fn read_segment(
    topo: &Topology,
    edge_id: EdgeId,
    index: usize,
) -> Result<Segment, OperationsError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    match edge.curve() {
        EdgeCurve::Line => {
            let d = end - start;
            let Ok(t) = d.normalize() else {
                return Err(invalid(format!(
                    "exact sweep path edge {index} has zero length"
                )));
            };
            Ok(Segment {
                start,
                end,
                start_tangent: t,
                end_tangent: t,
                motion: Motion::Translate(d),
            })
        }
        EdgeCurve::Circle(c) => {
            let (a, b) = match edge.strict_domain() {
                Ok(domain) => domain,
                Err(_) => {
                    crate::reconstruct_legacy_edge_domain(topo, edge_id, "exact sweep path edge")?
                }
            };
            let span = b - a;
            if !span.is_finite() || span.abs() <= Tolerance::new().angular {
                return Err(invalid(format!(
                    "exact sweep path edge {index} is a zero-length arc"
                )));
            }
            if span.abs() > std::f64::consts::TAU + Tolerance::new().angular {
                return Err(invalid(format!(
                    "exact sweep path edge {index} turns more than once"
                )));
            }
            let axis = if span > 0.0 { c.normal() } else { -c.normal() };
            let sign = span.signum();
            Ok(Segment {
                start,
                end,
                start_tangent: c.tangent(a) * sign,
                end_tangent: c.tangent(b) * sign,
                motion: Motion::Rotate {
                    origin: c.center(),
                    axis: axis.normalize()?,
                    angle: span.abs(),
                },
            })
        }
        other @ (EdgeCurve::Ellipse(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::NurbsCurve(_)) => Err(unsupported(format!(
            "exact sweep paths are lines and circular arcs; path edge {index} is a {}",
            other.type_tag()
        ))),
    }
}

/// Order the path edges into a chain, whatever order they come in, oriented
/// so each starts where the last ended. Returns the segments and whether the
/// chain closes.
fn read_chain(
    topo: &Topology,
    path: &[EdgeId],
    linear: f64,
) -> Result<(Vec<Segment>, bool), OperationsError> {
    if path.is_empty() {
        return Err(invalid("exact sweep needs at least one path edge"));
    }
    let raw: Vec<Segment> = path
        .iter()
        .enumerate()
        .map(|(i, &e)| read_segment(topo, e, i))
        .collect::<Result<_, _>>()?;
    let scale = raw
        .iter()
        .map(|s| to_vec(s.start).length().max(to_vec(s.end).length()))
        .fold(1.0, f64::max);
    let join = (10.0 * linear).max(RELATIVE_MARGIN * scale);
    let meets = |a: Point3, b: Point3| (a - b).length() <= join;

    // How many edge ends meet at a point: 1 is a free end, 2 a joint.
    let ends_at = |p: Point3| {
        raw.iter()
            .map(|s| usize::from(meets(s.start, p)) + usize::from(meets(s.end, p)))
            .sum::<usize>()
    };
    for (i, s) in raw.iter().enumerate() {
        for point in [s.start, s.end] {
            if ends_at(point) > 2 {
                return Err(invalid(format!(
                    "exact sweep path branches at an end of path edge {i}; a sweep path is \
                     one chain"
                )));
            }
        }
    }
    // Start at a free end when there is one (an open chain), oriented away
    // from it; a loop starts at the first edge as given.
    let first = raw
        .iter()
        .enumerate()
        .find_map(|(i, s)| {
            if ends_at(s.start) == 1 {
                Some((i, *s))
            } else if ends_at(s.end) == 1 {
                Some((i, s.reversed()))
            } else {
                None
            }
        })
        .unwrap_or((0, raw[0]));
    let mut used = vec![false; raw.len()];
    used[first.0] = true;
    let mut chain = Vec::with_capacity(raw.len());
    chain.push(first.1);
    while chain.len() < raw.len() {
        let tail = chain[chain.len() - 1].end;
        let next = raw.iter().enumerate().find_map(|(i, s)| {
            if used[i] {
                None
            } else if meets(s.start, tail) {
                Some((i, *s))
            } else if meets(s.end, tail) {
                Some((i, s.reversed()))
            } else {
                None
            }
        });
        let Some((i, segment)) = next else {
            return Err(invalid(format!(
                "exact sweep path edges do not form one connected chain ({} of {} meet end to \
                 end)",
                chain.len(),
                raw.len()
            )));
        };
        used[i] = true;
        chain.push(segment);
    }
    let closed = meets(chain[chain.len() - 1].end, chain[0].start);
    if closed && chain.len() == 1 {
        // One full-turn arc: two half turns, so the ring has two sections
        // like any other closed chain rather than one section met twice.
        let Some((first, second)) = chain[0].halves() else {
            return Err(invalid(
                "a single straight sweep path edge cannot close on itself",
            ));
        };
        chain = vec![first, second];
    }
    Ok((chain, closed))
}

/// `joints[i]` is the joint at the start of segment `i`; the first is the
/// closure joint of a closed chain and smooth otherwise.
fn chain_joints(chain: &[Segment], closed: bool) -> Result<Vec<Joint>, OperationsError> {
    let corner = |i: usize, before: &Segment, after: &Segment| -> Result<Joint, OperationsError> {
        let (incoming, outgoing) = (before.end_tangent, after.start_tangent);
        let angle = incoming.dot(outgoing).clamp(-1.0, 1.0).acos();
        if angle <= G1_ANGLE_TOLERANCE {
            return Ok(Joint::Smooth);
        }
        let both_straight = matches!(before.motion, Motion::Translate(_))
            && matches!(after.motion, Motion::Translate(_));
        if !both_straight {
            return Err(unsupported(format!(
                "the sweep path turns a corner of {:.4}° at joint {i}, next to an arc; an exact \
                 sweep mitres a corner only between two straight runs (join the arc tangentially)",
                angle.to_degrees()
            )));
        }
        if angle >= MAX_MITRE_TURN {
            return Err(unsupported(format!(
                "the sweep path turns back on itself at joint {i} ({:.4}°); a corner that sharp \
                 has no mitre",
                angle.to_degrees()
            )));
        }
        let axis = incoming.cross(outgoing).normalize()?;
        Ok(Joint::Mitre {
            point: after.start,
            normal: (incoming + outgoing).normalize()?,
            axis,
            angle,
        })
    };
    let mut joints = vec![Joint::Smooth; chain.len()];
    for i in 1..chain.len() {
        joints[i] = corner(i, &chain[i - 1], &chain[i])?;
    }
    if closed {
        joints[0] = corner(0, &chain[chain.len() - 1], &chain[0])?;
    }
    Ok(joints)
}

/// How a path segment meets the one before it.
#[derive(Debug, Clone, Copy)]
enum Joint {
    /// Tangent-continuous: the sections simply continue.
    Smooth,
    /// A corner between two straight runs, mitred on the bisector plane
    /// through `point` with unit `normal` (the sum of the two directions):
    /// the frame turns by `angle` about `axis` at the corner, and both runs'
    /// prisms cut that plane in the same section.
    Mitre {
        point: Point3,
        normal: Vec3,
        axis: Vec3,
        angle: f64,
    },
}

impl Joint {
    /// The rigid turn the frame takes at the corner.
    fn turn(&self) -> Option<Rigid> {
        match *self {
            Self::Smooth => None,
            Self::Mitre {
                point, axis, angle, ..
            } => Some(
                Motion::Rotate {
                    origin: point,
                    axis,
                    angle,
                }
                .rigid(),
            ),
        }
    }

    /// The affine map projecting along `direction` onto the mitre plane,
    /// for a mitred joint.
    fn projection(&self, direction: Vec3) -> Option<Projection> {
        match *self {
            Self::Smooth => None,
            Self::Mitre { point, normal, .. } => Some(Projection {
                point,
                normal,
                direction,
            }),
        }
    }
}

/// Parallel projection along `direction` onto the plane through `point`
/// with `normal`: an affine map, so it carries NURBS exactly.
#[derive(Debug, Clone, Copy)]
struct Projection {
    point: Point3,
    normal: Vec3,
    direction: Vec3,
}

impl Projection {
    /// How far along `direction` `p` lies from the plane (negative: before it).
    fn reach(&self, p: Point3) -> f64 {
        (self.point - p).dot(self.normal) / self.direction.dot(self.normal)
    }

    fn point(&self, p: Point3) -> Point3 {
        p + self.direction * self.reach(p)
    }

    fn nurbs(&self, n: &NurbsCurve) -> Result<NurbsCurve, OperationsError> {
        Ok(NurbsCurve::new(
            n.degree(),
            n.knots().to_vec(),
            n.control_points().iter().map(|&p| self.point(p)).collect(),
            n.weights().to_vec(),
        )?)
    }
}

/// An analytic surface for `curve` (in its section position, from `start` to
/// `end` in its natural direction) moved by `motion`, when there is one.
fn analytic_side(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    motion: &Motion,
    probe: Point3,
    linear: f64,
) -> Result<Option<FaceSurface>, OperationsError> {
    match (*motion, curve) {
        (Motion::Translate(d), EdgeCurve::Line) => {
            let normal = (end - start).cross(d).normalize()?;
            Ok(Some(FaceSurface::Plane {
                normal,
                d: crate::dot_normal_point(normal, start),
            }))
        }
        (Motion::Translate(d), EdgeCurve::Circle(c)) => {
            // The far circle stays on the cylinder only if the run drifts off
            // the circle's axis by no more than the linear tolerance over its
            // whole length; an angular test alone lets a long run drift.
            if (d - c.normal() * d.dot(c.normal())).length() > linear {
                return Ok(None);
            }
            let surface = CylindricalSurface::with_ref_dir(
                c.center(),
                c.normal(),
                c.radius(),
                start - c.center(),
            )?;
            Ok(Some(FaceSurface::Cylinder(surface)))
        }
        (Motion::Rotate { origin, axis, .. }, EdgeCurve::Line) => {
            // Tests on the whole run, not its direction alone, so that a long
            // line cannot drift off the surface by more than `linear`.
            let run = end - start;
            let across = run - axis * run.dot(axis);
            // Coplanar with the axis: the line meets or parallels it.
            let offset = start - origin;
            let normal = axis.cross(run);
            if normal.length() > linear && offset.dot(normal.normalize()?).abs() > linear {
                return Ok(None);
            }
            let foot = |p: Point3| origin + axis * (p - origin).dot(axis);
            let radial = probe - foot(probe);
            if across.length() <= linear {
                let surface = CylindricalSurface::with_ref_dir(
                    foot(start),
                    axis,
                    (start - foot(start)).length(),
                    radial,
                )?;
                return Ok(Some(FaceSurface::Cylinder(surface)));
            }
            if run.dot(axis).abs() <= linear {
                return Ok(Some(FaceSurface::Plane {
                    normal: axis,
                    d: crate::dot_normal_point(axis, start),
                }));
            }
            // Oblique: a cone with its apex where the line meets the axis.
            let (z0, r0) = {
                let d0 = start - origin;
                let z = d0.dot(axis);
                (z, (d0 - axis * z).length())
            };
            let (z1, r1) = {
                let d1 = end - origin;
                let z = d1.dot(axis);
                (z, (d1 - axis * z).length())
            };
            let (dr, dz) = (r1 - r0, z1 - z0);
            if dr.abs() <= linear {
                return Ok(None);
            }
            let z_apex = z0 - r0 * (dz / dr);
            let apex = origin + axis * z_apex;
            let wide_z = if r1 > r0 { z1 } else { z0 };
            let cone_axis = if wide_z >= z_apex { axis } else { -axis };
            let half_angle = dr.abs().atan2(dz.abs());
            if half_angle <= 1e-9 || half_angle >= FRAC_PI_2 - 1e-9 {
                return Ok(None);
            }
            let surface = ConicalSurface::new(apex, cone_axis, half_angle)?;
            Ok(Some(FaceSurface::Cone(surface)))
        }
        (Motion::Rotate { origin, axis, .. }, EdgeCurve::Circle(c)) => {
            // The circle's plane must hold the axis.
            // Off the axis plane by at most `linear` anywhere on the circle.
            let reach = (c.center() - origin).length() + c.radius();
            if c.normal().dot(axis).abs() * reach > linear
                || (origin - c.center()).dot(c.normal()).abs() > linear
            {
                return Ok(None);
            }
            let along = (c.center() - origin).dot(axis);
            let centre = origin + axis * along;
            let major = (c.center() - centre).length();
            if major <= c.radius() + linear {
                return Ok(None);
            }
            let surface = ToroidalSurface::with_axis_and_ref_dir(
                centre,
                major,
                c.radius(),
                axis,
                c.center() - centre,
            )?;
            Ok(Some(FaceSurface::Torus(surface)))
        }
        (
            _,
            EdgeCurve::Ellipse(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::NurbsCurve(_),
        ) => Ok(None),
    }
}

/// Read the crossing sign at the profile samples. For rotations this
/// nominates the sign for the whole-boundary proof.
fn crossing_sign(
    samples: &[Point3],
    normal: Vec3,
    motion: &Motion,
    margin: f64,
    index: usize,
) -> Result<f64, OperationsError> {
    if let Motion::Rotate { origin, axis, .. } = *motion {
        let closest = samples
            .iter()
            .map(|&p| {
                let d = p - origin;
                (d - axis * d.dot(axis)).length()
            })
            .fold(f64::INFINITY, f64::min);
        if closest <= margin {
            return Err(invalid(format!(
                "the sweep profile reaches the axis of path arc {index} (closest boundary point \
                 is {closest} from it); use a larger arc radius or a smaller profile"
            )));
        }
    }
    let (low, high) = samples
        .iter()
        .map(|&p| {
            let v = motion.velocity(p);
            v.dot(normal) / v.length().max(1.0)
        })
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| {
            (lo.min(s), hi.max(s))
        });
    if low > margin {
        Ok(1.0)
    } else if high < -margin {
        Ok(-1.0)
    } else {
        Err(invalid(format!(
            "path segment {index} does not cross the sweep profile's plane transversally \
             (the profile lies along the path, or reaches past an arc's axis): the sweep would \
             fold through itself"
        )))
    }
}

/// Refuse a chain whose non-adjacent segments come within twice the
/// profile's reach of each other, where the swept body could meet itself.
/// The distance used is a certified lower bound, never a sampled estimate.
fn check_clearance(
    chain: &[Segment],
    closed: bool,
    reach: f64,
    margin: f64,
) -> Result<(), OperationsError> {
    let n = chain.len();
    let limit = 2.0 * reach + margin;
    for i in 0..n {
        for j in (i + 2)..n {
            if closed && i == 0 && j == n - 1 {
                continue;
            }
            let closest = distance_lower_bound(&chain[i], &chain[j], reach);
            if closest <= limit {
                return Err(invalid(format!(
                    "path segments {i} and {j} pass within {} of each other, within twice the \
                     sweep profile's reach ({reach}): the swept body could meet itself",
                    closest.max(0.0)
                )));
            }
        }
    }
    Ok(())
}

/// A lower bound on the distance between two path segments. Two lines are
/// measured exactly. An arc is replaced by points no farther apart along it
/// than `reach / 4`, so every point of the arc lies within half that spacing
/// of one of them, and that half spacing is taken off.
fn distance_lower_bound(a: &Segment, b: &Segment, reach: f64) -> f64 {
    let spacing = |s: &Segment| -> (Vec<Point3>, f64) {
        let length = s.length();
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let count = ((length / (0.25 * reach)).ceil() as usize)
            .clamp(CLEARANCE_POINTS.0, CLEARANCE_POINTS.1);
        #[allow(clippy::cast_precision_loss)]
        let slack = 0.5 * length / count as f64;
        (s.points(count), slack)
    };
    match (a.motion, b.motion) {
        (Motion::Translate(_), Motion::Translate(_)) => {
            segment_segment_distance(a.start, a.end, b.start, b.end)
        }
        (Motion::Translate(_), Motion::Rotate { .. }) => {
            let (points, slack) = spacing(b);
            points
                .iter()
                .map(|&p| point_segment_distance(p, a.start, a.end))
                .fold(f64::INFINITY, f64::min)
                - slack
        }
        (Motion::Rotate { .. }, Motion::Translate(_)) => distance_lower_bound(b, a, reach),
        (Motion::Rotate { .. }, Motion::Rotate { .. }) => {
            let (pa, sa) = spacing(a);
            let (pb, sb) = spacing(b);
            pa.iter()
                .flat_map(|&p| pb.iter().map(move |&q| (p - q).length()))
                .fold(f64::INFINITY, f64::min)
                - sa
                - sb
        }
    }
}

/// The distance from `p` to the segment `a`–`b`.
fn point_segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let d = b - a;
    let dd = d.dot(d);
    let t = if dd > 0.0 {
        ((p - a).dot(d) / dd).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + d * t)).length()
}

/// A lower bound on the distance between the segments `p0`–`p1` and
/// `q0`–`q1`. The closest pair is interior to both only when the lines are
/// not parallel; every other case has an endpoint in it. The interior
/// parameters come from cross products (Cramer's rule on `n = u × v`), which
/// keep a crossing at a grazing angle that `a·c − b²` cancels to zero, and a
/// rounding margin scaled by the coordinates is taken off the result.
fn segment_segment_distance(p0: Point3, p1: Point3, q0: Point3, q1: Point3) -> f64 {
    let (u, v, w) = (p1 - p0, q1 - q0, q0 - p0);
    let mut best = point_segment_distance(p0, q0, q1)
        .min(point_segment_distance(p1, q0, q1))
        .min(point_segment_distance(q0, p0, p1))
        .min(point_segment_distance(q1, p0, p1));
    let n = u.cross(v);
    let nn = n.dot(n);
    if nn > 0.0 {
        let s = w.cross(v).dot(n) / nn;
        let t = w.cross(u).dot(n) / nn;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            best = best.min(((p0 + u * s) - (q0 + v * t)).length());
        }
    }
    let scale = [p0, p1, q0, q1]
        .iter()
        .map(|p| p.x().abs().max(p.y().abs()).max(p.z().abs()))
        .fold(0.0, f64::max);
    (best - 64.0 * f64::EPSILON * scale).max(0.0)
}

/// The ruled surface a straight run sweeps from `curve` (its rigid start
/// section), cut at a mitred end by that end's projection: each control
/// point runs along the run from its start (projected, at a mitred start)
/// to its end (projected, at a mitred end, or moved by `d`). Every ruling is
/// parallel to the run, so the surface lies on the run's prism exactly.
fn mitred_run_surface(
    curve: &NurbsCurve,
    start: Option<Projection>,
    end: Option<Projection>,
    d: Vec3,
) -> Result<NurbsSurface, OperationsError> {
    let control_points: Vec<Vec<Point3>> = curve
        .control_points()
        .iter()
        .map(|&p| {
            vec![
                start.map_or(p, |projection| projection.point(p)),
                end.map_or(p + d, |projection| projection.point(p)),
            ]
        })
        .collect();
    let weights: Vec<Vec<f64>> = curve.weights().iter().map(|&w| vec![w, w]).collect();
    Ok(NurbsSurface::new(
        curve.degree(),
        1,
        curve.knots().to_vec(),
        vec![0.0, 0.0, 1.0, 1.0],
        control_points,
        weights,
    )?)
}

/// One section of every profile loop: vertices and edges.
struct Section {
    vertices: Vec<Vec<VertexId>>,
    edges: Vec<Vec<EdgeId>>,
}

/// Sweep the planar face `profile` along the chain of line and circular-arc
/// edges `path`, exactly (see the [module docs](self)).
///
/// The profile stays where it is and is carried by the path's rigid motions,
/// starting from the chain end nearer the profile's plane; it need not sit
/// on the path, nor be square to it. Every edge of the result lies exactly on
/// its faces and every face is an exact plane, cylinder, cone, torus or
/// rational NURBS surface — nothing is sampled, fitted or faceted.
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] for an empty or broken chain, a
/// zero-length edge, a non-planar profile, a profile the motion does not cross
/// transversally, a closed chain that does not bring the profile back to its
/// start, or segments that pass closer than the profile's reach.
/// [`OperationsError::Unsupported`] for a path edge that is not a line or a
/// circular arc, and for a corner that is not tangent-continuous. Every error
/// leaves the topology unchanged.
pub fn sweep_exact(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
) -> Result<SolidId, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| sweep_exact_impl(topo, profile, path))
}

#[allow(clippy::too_many_lines)]
fn sweep_exact_impl(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
) -> Result<SolidId, OperationsError> {
    let linear = Tolerance::new().linear;
    let face = topo.face(profile)?;
    let FaceSurface::Plane { normal, d: plane_d } = *face.surface() else {
        return Err(invalid(format!(
            "an exact sweep needs a planar profile, got a {} face",
            face.surface().type_tag()
        )));
    };
    let normal = normal.normalize()?;
    let outer_wire = face.outer_wire();
    let inner_wires = face.inner_wires().to_vec();
    let mut loops: Vec<Loop> = vec![read_profile_loop(topo, outer_wire, true, "exact sweep")?];
    for wire in inner_wires {
        loops.push(read_profile_loop(topo, wire, false, "exact sweep")?);
    }

    let (mut chain, closed) = read_chain(topo, path, linear)?;
    // Start from the chain end nearer the profile's plane.
    let plane_distance = |p: Point3| (to_vec(p).dot(normal) - plane_d).abs();
    if !closed && plane_distance(chain[chain.len() - 1].end) < plane_distance(chain[0].start) {
        chain = chain.iter().rev().map(Segment::reversed).collect();
    }
    let joints = chain_joints(&chain, closed)?;
    let n = chain.len();

    let samples: Vec<Vec<Point3>> = loops.iter().map(Loop::samples).collect();
    let all: Vec<Point3> = samples.iter().flatten().copied().collect();
    let reach = all
        .iter()
        .map(|&p| (p - chain[0].start).length())
        .fold(0.0, f64::max);
    let scale = all
        .iter()
        .map(|&p| to_vec(p).length())
        .chain(chain.iter().map(|s| to_vec(s.end).length()))
        .fold(1.0, f64::max);
    let margin = (RELATIVE_MARGIN * scale).max(linear);

    // The cumulative maps: `placements[k]` carries the profile to the start
    // of segment k, turned at a mitred corner; `placements[n]` to the end.
    let mut placements = Vec::with_capacity(n + 1);
    placements.push(Rigid::IDENTITY);
    for k in 1..=n {
        let mut placement = chain[k - 1].motion.rigid().after(&placements[k - 1]);
        if k < n
            && let Some(turn) = joints[k].turn()
        {
            placement = turn.after(&placement);
        }
        placements.push(placement);
    }
    // The mitre projections at each segment's start and end, along it.
    let direction = |k: usize| match chain[k].motion {
        Motion::Translate(d) => d.normalize().ok(),
        Motion::Rotate { .. } => None,
    };
    let start_projection = |k: usize| {
        (k > 0 || closed)
            .then(|| direction(k).and_then(|t| joints[k].projection(t)))
            .flatten()
    };
    let end_projection = |k: usize| {
        let next = (k + 1) % n;
        (k + 1 < n || closed)
            .then(|| direction(k).and_then(|t| joints[next].projection(t)))
            .flatten()
    };

    // Transversality, with one crossing sense along the whole chain.
    let context = OperationContext::new();
    let mut sigma = None;
    for (k, segment) in chain.iter().enumerate() {
        let placed: Vec<Point3> = all.iter().map(|&p| placements[k].point(p)).collect();
        let normal = placements[k].vec(normal);
        let sign = crossing_sign(&placed, normal, &segment.motion, margin, k)?;
        if let Motion::Rotate { origin, axis, .. } = segment.motion {
            for piece in loops.iter().flat_map(|lp| &lp.pieces) {
                let curve = placements[k].nurbs(&piece.nurbs)?;
                match certify_rotational_crossing(
                    &curve,
                    origin,
                    axis,
                    normal * sign,
                    margin,
                    &context,
                ) {
                    Ok(true) => {}
                    Err(ReuseError::Math(error)) => return Err(error.into()),
                    _ => {
                        return Err(invalid(format!(
                            "path segment {k} cannot be shown to cross the entire sweep profile transversally"
                        )));
                    }
                }
            }
        }
        if (*sigma.get_or_insert(sign) > 0.0) != (sign > 0.0) {
            return Err(invalid(format!(
                "path segment {k} crosses the sweep profile's plane the other way: the sweep \
                 would fold back through itself"
            )));
        }
    }
    let sigma = sigma.unwrap_or(1.0);
    // A mitre must leave every profile point some run: past it the two
    // corners' planes cross and the run would fold through itself.
    for k in 0..n {
        let (start, end) = (start_projection(k), end_projection(k));
        if start.is_none() && end.is_none() {
            continue;
        }
        let Motion::Translate(d) = chain[k].motion else {
            continue;
        };
        for &p in &all {
            let x = placements[k].point(p);
            let from = start.map_or(0.0, |projection| projection.reach(x));
            let to = end.map_or_else(|| d.length(), |projection| projection.reach(x));
            if to - from <= margin {
                return Err(invalid(format!(
                    "path segment {k} is too short for its mitred corners: the profile would \
                     fold through itself (lengthen the run or shrink the profile)"
                )));
            }
        }
    }
    if closed {
        let mut last = placements[n];
        if let Some(turn) = joints[0].turn() {
            last = turn.after(&last);
        }
        let miss = all
            .iter()
            .map(|&p| (last.point(p) - p).length())
            .fold(0.0, f64::max);
        if miss > 10.0 * margin {
            return Err(invalid(format!(
                "the closed sweep path does not bring the profile back to where it started \
                 (it lands {miss} away)"
            )));
        }
    }
    check_clearance(&chain, closed, reach, margin)?;

    // Sections 0..=n; a closed chain's last section is its first.
    let section_count = if closed { chain.len() } else { chain.len() + 1 };
    let mut sections: Vec<Section> = Vec::with_capacity(section_count);
    for (k, placement) in placements.iter().enumerate().take(section_count) {
        let mut vertices = Vec::with_capacity(loops.len());
        let mut edges = Vec::with_capacity(loops.len());
        for lp in &loops {
            // A mitred joint's section is the rigid one projected onto the
            // corner's plane along the run.
            let projection = if k < n { start_projection(k) } else { None };
            let ids: Vec<VertexId> = lp
                .points
                .iter()
                .map(|&(p, tol)| {
                    let at = placement.point(p);
                    let at = projection.map_or(at, |projection| projection.point(at));
                    topo.add_vertex(Vertex::new(at, tol))
                })
                .collect();
            let mut loop_edges = Vec::with_capacity(lp.pieces.len());
            for (i, piece) in lp.pieces.iter().enumerate() {
                let (ns, ne) = lp.natural_ends(i);
                let (curve, trim) = match projection {
                    Some(projection) if !matches!(piece.cap, EdgeCurve::Line) => {
                        let projected = projection.nurbs(&placement.nurbs(&piece.nurbs)?)?;
                        let domain = projected.domain();
                        (EdgeCurve::NurbsCurve(projected), Some(domain))
                    }
                    Some(_) => (EdgeCurve::Line, piece.trim),
                    None if k == 0 => (piece.cap.clone(), piece.trim),
                    None => (placement.curve(&piece.cap)?, piece.trim),
                };
                let mut edge = Edge::new(ids[ns], ids[ne], curve);
                edge.set_trim(trim);
                loop_edges.push(topo.add_edge(edge));
            }
            vertices.push(ids);
            edges.push(loop_edges);
        }
        sections.push(Section { vertices, edges });
    }
    let section = |k: usize| k % section_count;

    let mut side_faces = Vec::new();
    for (k, segment) in chain.iter().enumerate() {
        let placement = placements[k];
        let (from, to) = (section(k), section(k + 1));
        let section_normal = placement.vec(normal);
        for (l, lp) in loops.iter().enumerate() {
            let ccw = is_ccw(&samples[l], normal);
            // Rails: one per profile vertex.
            let mut rails = Vec::with_capacity(lp.points.len());
            for (i, &(p, _)) in lp.points.iter().enumerate() {
                let (curve, trim) = segment.motion.rail(placement.point(p))?;
                let mut edge = Edge::new(
                    sections[from].vertices[l][i],
                    sections[to].vertices[l][i],
                    curve,
                );
                edge.set_trim(trim);
                rails.push(topo.add_edge(edge));
            }
            for (i, piece) in lp.pieces.iter().enumerate() {
                let (ns, ne) = lp.natural_ends(i);
                let nurbs = placement.nurbs(&piece.nurbs)?;
                let (u0, u1) = nurbs.domain();
                let u_mid = f64::midpoint(u0, u1);
                let probe = nurbs.evaluate(u_mid);
                let tangent = nurbs.tangent(u_mid)?;
                let traversal = if piece.forward { tangent } else { -tangent };
                let side = if ccw == lp.outer { 1.0 } else { -1.0 };
                let outward = traversal.cross(section_normal) * side;
                // The wire below runs counter-clockwise about `∂u × ∂v`.
                let natural = tangent.cross(segment.motion.velocity(probe));
                let start_point = placement.point(lp.points[ns].0);
                let end_point = placement.point(lp.points[ne].0);
                let curve = placement.curve(&piece.cap)?;
                let analytic = analytic_side(
                    &curve,
                    start_point,
                    end_point,
                    &segment.motion,
                    probe,
                    linear,
                )?;
                let (surface, flip, reversed) = match analytic {
                    Some(FaceSurface::Plane { normal: n, .. }) => {
                        let n = if n.dot(outward) < 0.0 { -n } else { n };
                        (
                            FaceSurface::Plane {
                                normal: n,
                                d: crate::dot_normal_point(n, probe),
                            },
                            n.dot(natural) < 0.0,
                            false,
                        )
                    }
                    Some(surface) => {
                        let (u, v) = surface
                            .project_point(probe)
                            .ok_or_else(|| invalid("exact sweep side surface has no chart"))?;
                        let n = surface.normal(u, v);
                        let flip = n.dot(natural) < 0.0;
                        let reversed = n.dot(outward) < 0.0;
                        (surface, flip, reversed)
                    }
                    None => {
                        let surface = match (start_projection(k), end_projection(k)) {
                            (None, None) => segment.motion.nurbs_surface(&nurbs)?,
                            (start, end) => {
                                let Motion::Translate(d) = segment.motion else {
                                    return Err(invalid("a mitred run must be straight"));
                                };
                                mitred_run_surface(&nurbs, start, end, d)?
                            }
                        };
                        let reversed = surface.normal(u_mid, 0.0)?.dot(outward) < 0.0;
                        (FaceSurface::Nurbs(surface), false, reversed)
                    }
                };
                let bottom = sections[from].edges[l][i];
                let top = sections[to].edges[l][i];
                let edges = if flip {
                    vec![
                        OrientedEdge::new(rails[ns], true),
                        OrientedEdge::new(top, true),
                        OrientedEdge::new(rails[ne], false),
                        OrientedEdge::new(bottom, false),
                    ]
                } else {
                    vec![
                        OrientedEdge::new(bottom, true),
                        OrientedEdge::new(rails[ne], true),
                        OrientedEdge::new(top, false),
                        OrientedEdge::new(rails[ns], false),
                    ]
                };
                let wire = topo.add_wire(Wire::new(edges, true)?);
                let mut face = Face::new(wire, vec![], surface);
                if reversed {
                    face.set_reversed(true);
                }
                side_faces.push(topo.add_face(face));
            }
        }
    }

    let mut faces = side_faces;
    if !closed {
        // Cap wires: an outer loop runs counter-clockwise about its cap's
        // outward normal, a hole clockwise. The start cap faces −σ n; the end
        // cap faces σ G(n), and the rigid placement keeps the winding.
        let cap_wire = |topo: &mut Topology,
                        lp: &Loop,
                        edges: &[EdgeId],
                        ccw: bool,
                        along_n: bool|
         -> Result<WireId, OperationsError> {
            let oriented: Vec<OrientedEdge> = if (ccw == along_n) == lp.outer {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .map(|(&e, piece)| OrientedEdge::new(e, piece.forward))
                    .collect()
            } else {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .rev()
                    .map(|(&e, piece)| OrientedEdge::new(e, !piece.forward))
                    .collect()
            };
            Ok(topo.add_wire(Wire::new(oriented, true)?))
        };
        let last = chain.len();
        let mut start_wires = Vec::with_capacity(loops.len());
        let mut end_wires = Vec::with_capacity(loops.len());
        for (l, lp) in loops.iter().enumerate() {
            let ccw = is_ccw(&samples[l], normal);
            start_wires.push(cap_wire(topo, lp, &sections[0].edges[l], ccw, sigma < 0.0)?);
            end_wires.push(cap_wire(
                topo,
                lp,
                &sections[last].edges[l],
                ccw,
                sigma > 0.0,
            )?);
        }
        let start_point = loops[0].points[0].0;
        let start_normal = -normal * sigma;
        let end_normal = placements[last].vec(normal) * sigma;
        let end_point = placements[last].point(start_point);
        faces.insert(
            0,
            topo.add_face(Face::new(
                start_wires[0],
                start_wires[1..].to_vec(),
                FaceSurface::Plane {
                    normal: start_normal,
                    d: crate::dot_normal_point(start_normal, start_point),
                },
            )),
        );
        faces.push(topo.add_face(Face::new(
            end_wires[0],
            end_wires[1..].to_vec(),
            FaceSurface::Plane {
                normal: end_normal,
                d: crate::dot_normal_point(end_normal, end_point),
            },
        )));
    }
    let shell = topo.add_shell(Shell::new(faces)?);
    Ok(topo.add_solid(Solid::new(shell, vec![])))
}

#[cfg(test)]
mod tests;
