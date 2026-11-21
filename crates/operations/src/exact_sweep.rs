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
//! not meet end to end, a corner next to an arc (only a corner between two
//! straight runs is mitred), a corner within about 0.1° of turning straight
//! back (its mitre is too ill-conditioned), a non-planar profile, a profile
//! the motion does not cross transversally (it would fold through itself:
//! the profile touches or crosses an arc's axis, or lies along a line), a run
//! too short for its mitres, a closed chain that does not bring the profile
//! back to its start, and a swept body that could meet itself. Every refusal
//! leaves the topology unchanged.
//!
//! ## Self-clearance
//!
//! The body must not meet itself, and that is certified rather than
//! sampled. Bounds over the profile are taken on its boundary pieces in
//! closed form (circle and ellipse arcs) or by their control points. Each
//! run's centre line is extended over every ruling its mitres leave, an arc
//! of more than a half turn is halved, and consecutive turns about one axis
//! form one rotation. Neighbouring pieces must lie on either side of their
//! joint's section or mitre plane, for as much of an arc as that is certified.
//! The rest of the arc, and every other pair, must stay twice the profile's
//! reach apart, or lie on either side of a plane: an upper bound on `g·x`
//! over one piece's body below a lower bound over the other's, both
//! certified (closed-form profile bounds, an arc's sections at close turns
//! plus the most a sinusoid can rise between two of them). A pair neither
//! clears is halved a bounded number of times before it is refused.

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
use crate::helix::in_place::{Loop, Piece, is_ccw, read_profile_loop};

/// Largest angle between the incoming and outgoing tangents at a joint that
/// still counts as tangent-continuous, in radians.
const G1_ANGLE_TOLERANCE: f64 = 1e-6;

/// The smallest `cos(φ/2)` a mitre takes, for a corner turning by `φ`
/// (about 179.885°). A mitre projects each section along its run onto the
/// bisector plane, which stretches it by `1 / cos(φ/2)`; below this floor
/// that stretch amplifies rounding past the linear tolerance at the scales
/// the kernel models. It is a conditioning floor only: whether a sharp corner
/// folds the body through itself is decided by the certified run-length and
/// clearance checks in [`sweep_exact`], not by the angle.
const MIN_MITRE_COSINE: f64 = 1e-3;

/// Bounds on the points per arc in the self-clearance check.
const CLEARANCE_POINTS: (usize, usize) = (16, 4096);

/// Bounds on the sections an arc is cut into for the separation bound.
const SEPARATION_TURNS: (usize, usize) = (4, 128);

/// Directions tried around a circle in the separating-plane search.
const SEPARATION_DIRECTIONS: usize = 48;

/// How many times a pair of bodies may be halved in the separation
/// certificate before it is refused.
const SEPARATION_DEPTH: usize = 10;

/// The most pairs of bodies the separation certificate tries for one pair
/// of stretches before it refuses: a bound on its work.
const SEPARATION_PAIRS: usize = 128;

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

    /// `Rᵀ·g`, so that `g·(R·p + t) = (Rᵀ·g)·p + g·t`.
    fn transpose_vec(&self, g: Vec3) -> Vec3 {
        Vec3::new(
            self.columns[0].dot(g),
            self.columns[1].dot(g),
            self.columns[2].dot(g),
        )
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

    /// The part of an arc between the turns `from` and `to` (`0 ≤ from < to ≤
    /// angle`), measured from its start.
    fn sub_arc(&self, from: f64, to: f64) -> Option<Self> {
        let Motion::Rotate { origin, axis, .. } = self.motion else {
            return None;
        };
        let at = |turn: f64| origin + rotate_vec(self.start - origin, axis, turn);
        Some(Self {
            start: at(from),
            end: at(to),
            start_tangent: rotate_vec(self.start_tangent, axis, from),
            end_tangent: rotate_vec(self.start_tangent, axis, to),
            motion: Motion::Rotate {
                origin,
                axis,
                angle: to - from,
            },
        })
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
        // `|t_in + t_out| = 2 cos(φ/2)` for unit tangents.
        if 0.5 * (incoming + outgoing).length() < MIN_MITRE_COSINE {
            return Err(unsupported(format!(
                "the sweep path turns back on itself at joint {i} ({:.4}°); a corner that sharp \
                 has no well-conditioned mitre",
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

/// Rounding slack for a bound summed from terms of magnitude `size`.
fn rounding(size: f64) -> f64 {
    64.0 * f64::EPSILON * size
}

/// The largest value of `base + a cos t + b sin t` for `t` in `[lo, hi]`.
fn trig_sup(base: f64, a: f64, b: f64, lo: f64, hi: f64) -> f64 {
    let at = |t: f64| b.mul_add(t.sin(), a.mul_add(t.cos(), base));
    let peak = base + a.hypot(b);
    if hi - lo >= std::f64::consts::TAU {
        return peak;
    }
    // The first angle at or after `lo` where the sinusoid peaks.
    let crest = b.atan2(a);
    let crest = crest + ((lo - crest) / std::f64::consts::TAU).ceil() * std::f64::consts::TAU;
    if crest <= hi {
        peak
    } else {
        at(lo).max(at(hi))
    }
}

/// An upper bound on `g·p` over the points `p` of one profile piece: exact
/// for a circle or ellipse arc, the control points otherwise (a NURBS curve
/// with positive weights lies in their convex hull, so an affine function
/// peaks on one of them).
fn piece_sup(piece: &Piece, g: Vec3) -> f64 {
    let span = piece.trim.map(|(a, b)| (a.min(b), a.max(b)));
    let conic = match &piece.cap {
        EdgeCurve::Circle(c) => span.map(|(lo, hi)| {
            (
                c.center(),
                c.u_axis() * c.radius(),
                c.v_axis() * c.radius(),
                lo,
                hi,
            )
        }),
        EdgeCurve::Ellipse(e) => span.map(|(lo, hi)| {
            (
                e.center(),
                e.u_axis() * e.semi_major(),
                e.v_axis() * e.semi_minor(),
                lo,
                hi,
            )
        }),
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => None,
    };
    if let Some((center, u, v, lo, hi)) = conic {
        let base = g.dot(to_vec(center));
        let size = g.length() * (to_vec(center).length() + u.length() + v.length());
        return trig_sup(base, g.dot(u), g.dot(v), lo, hi) + rounding(size);
    }
    piece
        .nurbs
        .control_points()
        .iter()
        .map(|&p| g.dot(to_vec(p)) + rounding(g.length() * to_vec(p).length()))
        .fold(f64::NEG_INFINITY, f64::max)
}

/// An upper bound on `g·x` over the boundary of the profile placed by
/// `placement`: `g·(R·p + t) = (Rᵀ·g)·p + g·t`. An affine function peaks on
/// a region's boundary, so this bounds the whole profile.
fn profile_sup(loops: &[Loop], placement: &Rigid, g: Vec3) -> f64 {
    let local = placement.transpose_vec(g);
    let on_profile = loops
        .iter()
        .flat_map(|lp| &lp.pieces)
        .map(|piece| piece_sup(piece, local))
        .fold(f64::NEG_INFINITY, f64::max);
    let shift = g.dot(placement.translation);
    on_profile + shift + rounding(g.length() * placement.translation.length())
}

/// A lower bound on `g·x` over the placed profile.
fn profile_inf(loops: &[Loop], placement: &Rigid, g: Vec3) -> f64 {
    -profile_sup(loops, placement, -g)
}

/// An upper bound on the distance from `q` to the profile in its own
/// position: `|p − q|` is convex, so it peaks on the boundary. A circle or
/// ellipse arc is bounded in closed form, `|p − q|² = |c − q|² + |p − c|² +
/// 2 (c − q)·(p − c)` with `|p − c|` at most the major semi-axis; any other
/// piece by its control points.
fn profile_reach(loops: &[Loop], q: Point3) -> f64 {
    let piece_reach = |piece: &Piece| -> f64 {
        let conic = match &piece.cap {
            EdgeCurve::Circle(c) => piece.trim.map(|_| (c.center(), c.radius())),
            EdgeCurve::Ellipse(e) => piece
                .trim
                .map(|_| (e.center(), e.semi_major().max(e.semi_minor()))),
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => None,
        };
        if let Some((center, radius)) = conic {
            let g = center - q;
            let across = piece_sup(piece, g) - g.dot(to_vec(center));
            let squared = 2.0f64.mul_add(across, radius.mul_add(radius, g.dot(g)));
            return squared.max(0.0).sqrt();
        }
        piece
            .nurbs
            .control_points()
            .iter()
            .map(|&p| (p - q).length())
            .fold(0.0, f64::max)
    };
    let reach = loops
        .iter()
        .flat_map(|lp| &lp.pieces)
        .map(piece_reach)
        .fold(0.0, f64::max);
    reach + rounding(reach + to_vec(q).length())
}

/// One stretch of the swept body for the self-clearance certificate: a
/// straight run, its centre line extended over every ruling its mitres
/// leave, or an arc of at most a half turn.
#[derive(Debug, Clone, Copy)]
struct Stretch {
    /// The chain segment it comes from.
    segment: usize,
    /// Consecutive stretches turning about one axis in one sense share a
    /// group: together they are one rotation of the profile.
    group: usize,
    path: Segment,
    /// The profile's placement at the stretch's start. The stretch's body
    /// lies in that placed profile carried along all of `path`: a run's is
    /// moved back over its mitre extension, which spans every ruling.
    placement: Rigid,
}

/// Two arcs turning about the same axis line in the same sense.
fn co_rotational(a: &Segment, b: &Segment, margin: f64) -> bool {
    match (a.motion, b.motion) {
        (
            Motion::Rotate {
                origin: o1,
                axis: a1,
                ..
            },
            Motion::Rotate {
                origin: o2,
                axis: a2,
                ..
            },
        ) => {
            let d = o2 - o1;
            (a1 - a2).length() <= G1_ANGLE_TOLERANCE && (d - a1 * d.dot(a1)).length() <= margin
        }
        _ => false,
    }
}

/// How a body meets the plane of one of its end sections.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Side {
    /// On one side of the plane, meeting it in that section alone.
    Strict,
    /// On one side, within the margin; it may also touch the plane far from
    /// the section (an arc of a half turn lays its other end section in it).
    Weak,
    /// Strictly on one side for this much of its turn from the joint; past
    /// it the section may swing across the plane, so the rest of the arc
    /// must clear the neighbour by the distance check.
    Within(f64),
}

/// Which side of its section plane the body of an arc lies on.
///
/// The section is the profile placed by `placement`, its plane's `normal`
/// pointing the way the profile moves. Turning a section point `x` by `α`
/// about `axis` through `origin` moves it off the plane by
/// `f(α) = A (cos α − 1) + B sin α = 2 sin(α/2) (B cos(α/2) − A sin(α/2))`,
/// with `B = n·(axis × r) > 0` by transversality (`r` the radial offset).
/// `B cos β − A sin β` is a sinusoid with zeros π apart that starts
/// positive, so for a turn of at most π it stays non-negative over
/// `[0, angle/2]` exactly when it is at the end: `f ≥ 0` over the whole turn.
/// That end value is `(x − origin)·(m × axis)` with `m` the normal turned by
/// `−angle/2`: affine in `x`, so its minimum over the profile is bounded on
/// the boundary.
///
/// A profile that is not square to the path swings across the plane once
/// it has turned by `2·atan2(B, A)`, which for a half turn happens at any
/// tilt at all. The same test at a smaller half-turn `β` certifies the first
/// `2β` of the arc, so the largest certified `β` is found by bisection and
/// the arc is [`Side::Within`] that turn; only a profile that cannot be
/// certified for any turn is refused.
#[allow(clippy::too_many_arguments)]
fn arc_side(
    loops: &[Loop],
    placement: &Rigid,
    normal: Vec3,
    origin: Point3,
    axis: Vec3,
    angle: f64,
    margin: f64,
    segment: usize,
) -> Result<Side, OperationsError> {
    let low = |half: f64| {
        let w = rotate_vec(normal, axis, -half).cross(axis);
        profile_inf(loops, placement, w) - to_vec(origin).dot(w)
    };
    let end = low(0.5 * angle);
    if end > margin {
        return Ok(Side::Strict);
    }
    if end >= -margin {
        return Ok(Side::Weak);
    }
    let (mut certified, mut failed) = (0.0, 0.5 * angle);
    for _ in 0..60 {
        let half = f64::midpoint(certified, failed);
        if low(half) > margin {
            certified = half;
        } else {
            failed = half;
        }
    }
    if certified > 0.0 {
        Ok(Side::Within(2.0 * certified))
    } else {
        Err(invalid(format!(
            "the sweep profile, turned along path arc {segment}, swings back across the plane of \
             its section at a joint: the swept body could meet itself (square the profile to the \
             path or shorten the arc)"
        )))
    }
}

/// The certified pieces the self-clearance check works on: every line
/// extended over `extents` (along its unit direction from its start), every
/// arc of more than a half turn halved, and consecutive turns about one axis
/// grouped.
fn stretches(
    chain: &[Segment],
    closed: bool,
    placements: &[Rigid],
    extents: &[(f64, f64)],
    margin: f64,
) -> Result<Vec<Stretch>, OperationsError> {
    let mut out: Vec<Stretch> = Vec::with_capacity(2 * chain.len());
    for (k, segment) in chain.iter().enumerate() {
        let placement = placements[k];
        match segment.motion {
            Motion::Translate(d) => {
                let t = d.normalize()?;
                let (lo, hi) = extents[k];
                let (start, end) = (segment.start + t * lo, segment.start + t * hi);
                out.push(Stretch {
                    segment: k,
                    group: 0,
                    path: Segment {
                        start,
                        end,
                        start_tangent: t,
                        end_tangent: t,
                        motion: Motion::Translate(end - start),
                    },
                    placement: Motion::Translate(t * lo).rigid().after(&placement),
                });
            }
            Motion::Rotate { angle, .. } => {
                let halves = (angle > std::f64::consts::PI + Tolerance::new().angular)
                    .then(|| segment.halves())
                    .flatten();
                if let Some((first, second)) = halves {
                    let middle = first.motion.rigid().after(&placement);
                    for (path, placement) in [(first, placement), (second, middle)] {
                        out.push(Stretch {
                            segment: k,
                            group: 0,
                            path,
                            placement,
                        });
                    }
                } else {
                    out.push(Stretch {
                        segment: k,
                        group: 0,
                        path: *segment,
                        placement,
                    });
                }
            }
        }
    }
    let turn = |s: &Stretch| match s.path.motion {
        Motion::Rotate { angle, .. } => angle,
        Motion::Translate(_) => 0.0,
    };
    let mut angles: Vec<f64> = Vec::with_capacity(out.len());
    for i in 0..out.len() {
        if i > 0 && co_rotational(&out[i - 1].path, &out[i].path, margin) {
            out[i].group = out[i - 1].group;
            let last = angles.len() - 1;
            angles[last] += turn(&out[i]);
        } else {
            out[i].group = angles.len();
            angles.push(turn(&out[i]));
        }
    }
    let m = out.len();
    if closed && m > 1 && out[m - 1].group != out[0].group {
        let (tail, head) = (out[m - 1].group, out[0].group);
        if co_rotational(&out[m - 1].path, &out[0].path, margin) {
            for s in out.iter_mut().filter(|s| s.group == tail) {
                s.group = head;
            }
            angles[head] += angles[tail];
            angles[tail] = 0.0;
        }
    }
    if let Some(i) = angles
        .iter()
        .position(|&a| a > std::f64::consts::TAU + Tolerance::new().angular)
    {
        let segment = out.iter().find(|s| s.group == i).map_or(0, |s| s.segment);
        return Err(invalid(format!(
            "the sweep path turns more than once about the axis of path arc {segment}: the swept \
             body would meet itself"
        )));
    }
    Ok(out)
}

/// Refuse a chain whose swept body could meet itself.
///
/// The body is cut into [`stretches`]. Within one group it is a single turn
/// of a transversal profile by at most a full turn, which is embedded.
/// Consecutive stretches of different groups share their joint section and
/// are separated by its plane: a run's rulings all end (or start) on it, a
/// mitred run's on the bisector plane, and an arc is certified by
/// [`arc_side`] over as much of its turn as it can be (the rest of the arc
/// must clear the neighbour's body by [`bodies_clear`]); where both sides
/// only touch the plane, their far ends must clear each other. Every other
/// pair must clear each other by [`bodies_clear`]: twice the profile's reach
/// apart, measured with a certified lower bound, or certifiably on either
/// side of a plane, never a sampled estimate.
#[allow(clippy::too_many_arguments)]
fn check_clearance(
    chain: &[Segment],
    closed: bool,
    loops: &[Loop],
    placements: &[Rigid],
    extents: &[(f64, f64)],
    normal: Vec3,
    reach: f64,
    margin: f64,
) -> Result<(), OperationsError> {
    let stretches = stretches(chain, closed, placements, extents, margin)?;
    let m = stretches.len();
    let limit = 2.0 * reach + margin;
    let mut joints: Vec<(usize, usize)> = (1..m).map(|j| (j - 1, j)).collect();
    if closed && m > 1 {
        joints.push((m - 1, 0));
    }
    for &(u, v) in &joints {
        let (before, after) = (&stretches[u], &stretches[v]);
        if before.group == after.group {
            continue;
        }
        let incoming = match before.path.motion {
            Motion::Translate(_) => Side::Strict,
            Motion::Rotate {
                origin,
                axis,
                angle,
            } => {
                let end = before.path.motion.rigid().after(&before.placement);
                arc_side(
                    loops,
                    &end,
                    -end.vec(normal),
                    origin,
                    -axis,
                    angle,
                    margin,
                    before.segment,
                )?
            }
        };
        let outgoing = match after.path.motion {
            Motion::Translate(_) => Side::Strict,
            Motion::Rotate {
                origin,
                axis,
                angle,
            } => arc_side(
                loops,
                &after.placement,
                after.placement.vec(normal),
                origin,
                axis,
                angle,
                margin,
                after.segment,
            )?,
        };
        // Past a certified turn, an arc's remainder must clear the whole
        // body on the other side of the joint.
        let remainders = [
            match (incoming, before.path.motion) {
                (Side::Within(turn), Motion::Rotate { angle, .. }) => {
                    before.path.sub_arc(0.0, angle - turn).map(|path| {
                        let rest = Body {
                            path,
                            placement: before.placement,
                        };
                        (rest, after)
                    })
                }
                _ => None,
            },
            match (outgoing, after.path.motion) {
                (
                    Side::Within(turn),
                    Motion::Rotate {
                        origin,
                        axis,
                        angle,
                    },
                ) => after.path.sub_arc(turn, angle).map(|path| {
                    let lead = Motion::Rotate {
                        origin,
                        axis,
                        angle: turn,
                    };
                    let placement = lead.rigid().after(&after.placement);
                    (Body { path, placement }, before)
                }),
                _ => None,
            },
        ];
        for (rest, other) in remainders.into_iter().flatten() {
            if !bodies_clear(loops, &rest, &Body::of(other), reach, limit, margin) {
                let closest = distance_lower_bound(&rest.path, &other.path, reach);
                return Err(invalid(format!(
                    "path segments {} and {} pass within {} of each other beyond the turn their \
                     joint's plane separates, within twice the sweep profile's reach ({reach}), \
                     and no plane certifiably separates them: the swept body could meet itself",
                    before.segment,
                    after.segment,
                    closest.max(0.0)
                )));
            }
        }
        if incoming == Side::Weak && outgoing == Side::Weak {
            let apart = (before.path.start - after.path.end).length();
            if apart <= limit {
                return Err(invalid(format!(
                    "path arcs {} and {} both end a half turn from their joint, {apart} apart, \
                     within twice the sweep profile's reach ({reach}): the swept body could meet \
                     itself",
                    before.segment, after.segment
                )));
            }
        }
    }
    for i in 0..m {
        for j in (i + 1)..m {
            let (a, b) = (&stretches[i], &stretches[j]);
            let neighbours = j == i + 1 || (closed && i == 0 && j == m - 1);
            if a.group == b.group || neighbours {
                continue;
            }
            if !bodies_clear(loops, &Body::of(a), &Body::of(b), reach, limit, margin) {
                let closest = distance_lower_bound(&a.path, &b.path, reach);
                return Err(invalid(format!(
                    "path segments {} and {} pass within {} of each other, within twice the \
                     sweep profile's reach ({reach}), and no plane certifiably separates them: \
                     the swept body could meet itself",
                    a.segment,
                    b.segment,
                    closest.max(0.0)
                )));
            }
        }
    }
    Ok(())
}

/// A piece of the swept body for the separation certificate: the profile
/// placed by `placement` at `path.start`, carried along all of `path` (moved
/// along a run, turned through an arc). It contains the material of the
/// stretch or sub-arc it is made from.
#[derive(Debug, Clone, Copy)]
struct Body {
    path: Segment,
    placement: Rigid,
}

impl Body {
    const fn of(stretch: &Stretch) -> Self {
        Self {
            path: stretch.path,
            placement: stretch.placement,
        }
    }

    /// Two bodies whose union contains this one: the path halved, the second
    /// half's profile carried to the middle.
    fn halves(&self) -> Option<(Self, Self)> {
        let (first, second) = match self.path.motion {
            Motion::Translate(d) => {
                let half = d * 0.5;
                let middle = self.path.start + half;
                let piece = |start: Point3, end: Point3| Segment {
                    start,
                    end,
                    motion: Motion::Translate(half),
                    ..self.path
                };
                (piece(self.path.start, middle), piece(middle, self.path.end))
            }
            Motion::Rotate { .. } => self.path.halves()?,
        };
        let placement = first.motion.rigid().after(&self.placement);
        Some((
            Self {
                path: first,
                placement: self.placement,
            },
            Self {
                path: second,
                placement,
            },
        ))
    }
}

/// An upper bound on `g·x` over a [`Body`]. A run adds the better of its
/// two ends to the placed profile's bound (`g·x` is affine along a ruling).
/// An arc is cut into `count` turns of `h`. Turning one point `x`, `g·x`
/// is a sinusoid `c + A cos(α − φ)` with `A ≤ |g| ρ`, `ρ` bounding the
/// point's distance from the axis (the profile lies within `reach` of the
/// path point, and turning keeps that distance). Over one turn of `h` it
/// exceeds the larger of its two end values by at most `A (1 − cos(h/2)) =
/// 2A sin²(h/4)`: past its crest only when the crest lies inside, and then
/// one end is within `h/2` of it. The sup over the profile of the larger end
/// value is the larger of the two placed sections' bounds, so the sections'
/// bounds at every cut plus that wobble bound the whole arc.
fn body_sup(loops: &[Loop], body: &Body, reach: f64, g: Vec3) -> f64 {
    match body.path.motion {
        Motion::Translate(d) => {
            profile_sup(loops, &body.placement, g)
                + g.dot(d).max(0.0)
                + rounding(g.length() * d.length())
        }
        Motion::Rotate {
            origin,
            axis,
            angle,
        } => {
            let rho = (body.path.start - origin).length() + reach;
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            let count = ((16.0 * rho * angle / reach).ceil() as usize)
                .clamp(SEPARATION_TURNS.0, SEPARATION_TURNS.1);
            #[allow(clippy::cast_precision_loss)]
            let h = angle / count as f64;
            let wobble = 2.0 * rho * (0.25 * h).sin().powi(2) * g.length();
            #[allow(clippy::cast_precision_loss)]
            let sections = (0..=count)
                .map(|i| {
                    let turn = Motion::Rotate {
                        origin,
                        axis,
                        angle: i as f64 * h,
                    };
                    profile_sup(loops, &turn.rigid().after(&body.placement), g)
                })
                .fold(f64::NEG_INFINITY, f64::max);
            sections + wobble + rounding(g.length() * (to_vec(origin).length() + 2.0 * rho))
        }
    }
}

/// A unit vector square to the unit `w`, and the one completing the pair.
fn square_pair(w: Vec3) -> Option<(Vec3, Vec3)> {
    let helper = if w.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let e1 = w.cross(helper).normalize().ok()?;
    Some((e1, w.cross(e1)))
}

/// Whether a plane certifiably separates the bodies `a` and `b` by more
/// than `margin`: some direction `g` with a lower bound on `g·x` over `b`
/// above an upper bound on it over `a` by the margin. Only the found
/// direction's bounds decide, so the search for it needs no guarantee: it
/// tries the directions square to an arc's axis (or to both runs), that axis
/// itself and the line between the paths' middles, then refines the best.
fn separated(loops: &[Loop], a: &Body, b: &Body, reach: f64, margin: f64) -> bool {
    let gap = |g: Vec3| -body_sup(loops, b, reach, -g) - body_sup(loops, a, reach, g);
    let axis_of = |body: &Body| match body.path.motion {
        Motion::Rotate { axis, .. } => Some(axis),
        Motion::Translate(_) => None,
    };
    let direction = |body: &Body| match body.path.motion {
        Motion::Translate(d) => d.normalize().ok(),
        Motion::Rotate { .. } => None,
    };
    let w = axis_of(a).or_else(|| axis_of(b)).or_else(|| {
        let (da, db) = (direction(a)?, direction(b)?);
        da.cross(db)
            .normalize()
            .ok()
            .or_else(|| square_pair(da).map(|p| p.0))
    });
    let Some(w) = w else {
        return false;
    };
    let Some((e1, e2)) = square_pair(w) else {
        return false;
    };
    let middle = |body: &Body| {
        body.path
            .points(2)
            .get(1)
            .copied()
            .unwrap_or(body.path.start)
    };
    let mut extra = vec![w, -w];
    if let Ok(between) = (middle(b) - middle(a)).normalize() {
        extra.push(between);
    }
    if extra.into_iter().any(|g| gap(g) > margin) {
        return true;
    }
    let along = |psi: f64| e1 * psi.cos() + e2 * psi.sin();
    #[allow(clippy::cast_precision_loss)]
    let step = std::f64::consts::TAU / SEPARATION_DIRECTIONS as f64;
    let mut best = (f64::NEG_INFINITY, 0.0);
    for k in 0..SEPARATION_DIRECTIONS {
        #[allow(clippy::cast_precision_loss)]
        let psi = k as f64 * step;
        let value = gap(along(psi));
        if value > margin {
            return true;
        }
        if value > best.0 {
            best = (value, psi);
        }
    }
    let (mut value, mut psi) = best;
    let mut width = 0.5 * step;
    for _ in 0..16 {
        for candidate in [psi - width, psi + width] {
            let v = gap(along(candidate));
            if v > margin {
                return true;
            }
            if v > value {
                (value, psi) = (v, candidate);
            }
        }
        width *= 0.5;
    }
    false
}

/// Whether the bodies `a` and `b` certifiably keep apart: their paths more
/// than `limit` apart, or a plane separating them. Failing both, each half
/// of the longer one must keep apart from the other, down to
/// [`SEPARATION_DEPTH`] halvings and within [`SEPARATION_PAIRS`] pairs tried
/// in all; past either, the pair is refused.
fn bodies_clear(loops: &[Loop], a: &Body, b: &Body, reach: f64, limit: f64, margin: f64) -> bool {
    let mut pending = vec![(*a, *b, 0usize)];
    let mut tried = 0usize;
    while let Some((a, b, depth)) = pending.pop() {
        tried += 1;
        if tried > SEPARATION_PAIRS {
            return false;
        }
        if distance_lower_bound(&a.path, &b.path, reach) > limit
            || separated(loops, &a, &b, reach, margin)
        {
            continue;
        }
        if depth >= SEPARATION_DEPTH {
            return false;
        }
        if a.path.length() >= b.path.length() {
            let Some((first, second)) = a.halves() else {
                return false;
            };
            pending.push((second, b, depth + 1));
            pending.push((first, b, depth + 1));
        } else {
            let Some((first, second)) = b.halves() else {
                return false;
            };
            pending.push((a, second, depth + 1));
            pending.push((a, first, depth + 1));
        }
    }
    true
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
/// transversally, a run too short for its mitres, a closed chain that does
/// not bring the profile back to its start, or a body that could meet itself
/// (see [Self-clearance](self#self-clearance)).
/// [`OperationsError::Unsupported`] for a path edge that is not a line or a
/// circular arc, a corner next to an arc, and a corner too sharp to mitre
/// well-conditioned (`cos(φ/2) < 1e-3`, about 179.885°). Every error leaves
/// the topology unchanged.
pub fn sweep_exact(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
) -> Result<SolidId, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| sweep_exact_impl(topo, profile, path))
}

/// Options of [`sweep_exact_with_options`]. The default is the plain
/// [`sweep_exact`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExactSweepOptions {
    /// Total turn of the profile about the path over its length, in
    /// radians, right-handed about the direction of travel; linear in the
    /// distance travelled. `0` turns nothing.
    pub twist: f64,
    /// Scale of the profile at the far end of the path, about the point
    /// where the path crosses it; linear in the distance travelled. `1`
    /// scales nothing.
    pub end_scale: f64,
}

impl Default for ExactSweepOptions {
    fn default() -> Self {
        Self {
            twist: 0.0,
            end_scale: 1.0,
        }
    }
}

/// Most Hermite spans a twisted sweep may fit along its path.
const MAX_TWIST_SPANS: usize = 4096;

/// [`sweep_exact`] with a twist and a scale along the path.
///
/// With the default options this is exactly [`sweep_exact`]. A twist or a
/// scale is built along a straight path (one line, or collinear lines) with
/// the profile square to it: the profile is carried along the path while it
/// scales uniformly about the path's line and turns about it, both linearly
/// in the distance travelled. Without a twist every face is exact (lines
/// sweep planes, arcs about the path cones, anything else exact ruled
/// NURBS); with a twist the side faces and rails are certified NURBS fits of
/// the screw motion, within a proven bound far below the modelling
/// tolerance, recorded as the edges' tolerance. Nothing is faceted.
///
/// # Errors
///
/// As [`sweep_exact`], and [`OperationsError::InvalidInput`] for a
/// non-finite twist or a non-finite or non-positive end scale, and for a
/// profile curve the scaled or twisted construction cannot carry (a closed
/// curve off the path's line, a trimmed spline, an ellipse or open conic).
/// [`OperationsError::Unsupported`] for a twist or scale along a path that
/// bends (an arc or a corner) or closes, or with a profile that is not square
/// to the path: those have no exact construction. Every error leaves the
/// topology unchanged.
pub fn sweep_exact_with_options(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
    options: &ExactSweepOptions,
) -> Result<SolidId, OperationsError> {
    if !options.twist.is_finite() {
        return Err(invalid(format!(
            "an exact sweep's twist must be finite, got {}",
            options.twist
        )));
    }
    if !(options.end_scale.is_finite() && options.end_scale > 0.0) {
        return Err(invalid(format!(
            "an exact sweep's end scale must be finite and positive, got {}",
            options.end_scale
        )));
    }
    #[allow(clippy::float_cmp)]
    if options.twist == 0.0 && options.end_scale == 1.0 {
        return sweep_exact(topo, profile, path);
    }
    remus_topology::transaction::run_transacted(topo, |topo| {
        straight_sweep_impl(topo, profile, path, options)
    })
}

/// A twisted or scaled sweep along a straight path: a similar (or twisted)
/// extrusion about the path's line.
fn straight_sweep_impl(
    topo: &mut Topology,
    profile: FaceId,
    path: &[EdgeId],
    options: &ExactSweepOptions,
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
    let what = if options.twist == 0.0 {
        "scale"
    } else if (options.end_scale - 1.0).abs() == 0.0 {
        "twist"
    } else {
        "twist and scale"
    };
    let (mut chain, closed) = read_chain(topo, path, linear)?;
    if closed {
        return Err(unsupported(format!(
            "an exact sweep can {what} the profile only along an open straight path, and \
             this path closes on itself"
        )));
    }
    let plane_distance = |p: Point3| (to_vec(p).dot(normal) - plane_d).abs();
    if plane_distance(chain[chain.len() - 1].end) < plane_distance(chain[0].start) {
        chain = chain.iter().rev().map(Segment::reversed).collect();
    }
    let mut travel = Vec3::new(0.0, 0.0, 0.0);
    for (k, segment) in chain.iter().enumerate() {
        let Motion::Translate(d) = segment.motion else {
            return Err(unsupported(format!(
                "an exact sweep can {what} the profile only along a straight path; path \
                 segment {k} is an arc, and a profile turning or scaling along an arc has no \
                 exact surface"
            )));
        };
        travel += d;
    }
    for (k, joint) in chain_joints(&chain, false)?.iter().enumerate() {
        if !matches!(joint, Joint::Smooth) {
            return Err(unsupported(format!(
                "an exact sweep can {what} the profile only along a straight path; the path \
                 turns a corner at joint {k}"
            )));
        }
    }
    let length = travel.length();
    let direction = travel.normalize()?;
    if normal.cross(direction).length() > 1e-9 {
        return Err(unsupported(format!(
            "an exact sweep can {what} the profile only when the profile is square to the \
             path; this one is tilted {:.4}° from square",
            normal
                .dot(direction)
                .abs()
                .clamp(0.0, 1.0)
                .acos()
                .to_degrees()
        )));
    }
    // The path's line crosses the profile's plane here: the centre of the
    // twist and of the scale.
    let start = chain[0].start;
    let along = (plane_d - to_vec(start).dot(normal)) / direction.dot(normal);
    let centre = start + direction * along;
    let rate = options.twist / length;
    #[allow(clippy::float_cmp)]
    let built = if options.end_scale == 1.0 {
        crate::extrude::twist::twisted_extrude_impl(
            topo,
            profile,
            centre,
            direction,
            length,
            rate,
            linear,
            MAX_TWIST_SPANS,
        )?
    } else {
        crate::extrude::similar_twist::similar_extrude_impl(
            topo,
            profile,
            centre,
            direction,
            length,
            rate,
            options.end_scale,
            linear,
            MAX_TWIST_SPANS,
        )?
    };
    Ok(built.solid)
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
    // Every placement carries the path's start point to the path point it
    // stands on, so no profile point is farther than this from its own path
    // point anywhere along the sweep.
    let reach = profile_reach(&loops, chain[0].start);
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
    // A run's ruling through section point `x` runs along it from `from(x)`
    // to `to(x)`: 0 and the run's length, or the reach to a mitre plane. Both
    // are affine in `x`, so their extremes over the profile are certified on
    // its boundary. A mitre must leave every profile point some run (past it
    // the two corners' planes cross and the run would fold through itself),
    // and `extents[k]` spans every ruling, for the clearance check.
    let mut extents = vec![(0.0, 0.0); n];
    for k in 0..n {
        let Motion::Translate(d) = chain[k].motion else {
            continue;
        };
        let length = d.length();
        // `reach(x) = (point − x)·n / (t·n) = offset − x·slope`.
        let affine = |projection: Option<Projection>, otherwise: f64| {
            projection.map_or((otherwise, Vec3::new(0.0, 0.0, 0.0)), |p| {
                let slope = p.normal * (1.0 / p.direction.dot(p.normal));
                (to_vec(p.point).dot(slope), slope)
            })
        };
        let (start, end) = (start_projection(k), end_projection(k));
        let (from_offset, from_slope) = affine(start, 0.0);
        let (to_offset, to_slope) = affine(end, length);
        if start.is_some() || end.is_some() {
            let shortest = (to_offset - from_offset)
                - profile_sup(&loops, &placements[k], to_slope - from_slope);
            if shortest <= margin {
                return Err(invalid(format!(
                    "path segment {k} is too short for its mitred corners: the profile would \
                     fold through itself (lengthen the run or shrink the profile)"
                )));
            }
        }
        let lowest = from_offset - profile_sup(&loops, &placements[k], from_slope);
        let highest = to_offset - profile_inf(&loops, &placements[k], to_slope);
        extents[k] = (lowest.min(0.0), highest.max(length));
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
    check_clearance(
        &chain,
        closed,
        &loops,
        &placements,
        &extents,
        normal * sigma,
        reach,
        margin,
    )?;

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

#[cfg(test)]
mod clearance_tests;

#[cfg(test)]
mod options_tests;
