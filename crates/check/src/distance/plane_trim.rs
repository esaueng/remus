//! Exact planar trim membership for line and circle boundaries.
//!
//! A [`PlanarTrim`] reads a planar face's wires once; [`PlanarTrim::contains`]
//! then decides a point by casting an in-plane ray and counting its algebraic
//! events on each line and circular arc, so a circular rim is tested as the
//! circle itself rather than as its chords. The ray's frame depends only on
//! the plane normal, so each circle's phase is computed once at build.
//! Point-to-face distance builds one per query; the point classifier caches
//! one per prepared face.

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;
use remus_topology::wire::WireId;

use super::edge;
use crate::CheckError;

/// An error a loop raises only once a query reaches it, so a point decided by
/// an earlier loop keeps the answer it had when wires were read per query.
#[derive(Debug, Clone)]
enum Deferred {
    Validation(String),
    Distance(&'static str),
}

impl Deferred {
    fn error(&self) -> CheckError {
        match self {
            Self::Validation(message) => CheckError::ValidationFailed(message.clone()),
            Self::Distance(message) => CheckError::DistanceFailed((*message).into()),
        }
    }
}

/// A circle with the ray-independent part of its ray events: the circle is
/// `c + cos_y·cos t + sin_y·sin t` along the frame's `y`.
#[derive(Debug, Clone)]
struct TrimCircle {
    circle: Circle3D,
    cos_y: f64,
    sin_y: f64,
    amplitude: f64,
    phase: f64,
}

impl TrimCircle {
    fn new(circle: &Circle3D, y: Vec3) -> Self {
        let cos_y = circle.radius() * circle.u_axis().dot(y);
        let sin_y = circle.radius() * circle.v_axis().dot(y);
        Self {
            circle: circle.clone(),
            cos_y,
            sin_y,
            amplitude: cos_y.hypot(sin_y),
            phase: sin_y.atan2(cos_y),
        }
    }
}

#[derive(Debug, Clone)]
enum TrimEdge {
    /// A straight edge's endpoints at its domain ends.
    Segment(Point3, Point3),
    /// A circular arc over its native domain `(a, b)`.
    Arc(TrimCircle, f64, f64),
    /// The first edge that cannot be decided; later edges never matter.
    Refused(Deferred),
}

#[derive(Debug, Clone)]
enum TrimLoop {
    /// One complete circular traversal (`complete_circle_wire`).
    Circle(TrimCircle),
    Edges(Vec<TrimEdge>),
    Refused(Deferred),
}

/// The outer loop and holes of one planar face, seen from one plane normal.
#[derive(Debug, Clone)]
pub struct PlanarTrim {
    normal: Vec3,
    /// The in-plane ray direction and its normal, `None` for a degenerate
    /// plane normal (every query then refuses).
    axes: Option<(Vec3, Vec3)>,
    outer: TrimLoop,
    holes: Vec<TrimLoop>,
}

impl PlanarTrim {
    /// Read the face's wires.
    ///
    /// # Errors
    ///
    /// Returns an error if a referenced topology entity is missing. Edge
    /// kinds and domains that cannot be decided are refused by
    /// [`PlanarTrim::contains`] when a query reaches them.
    pub fn build(topo: &Topology, face_id: FaceId, normal: Vec3) -> Result<Self, CheckError> {
        let face = topo.face(face_id)?;
        // The frame's axes do not depend on its origin.
        let axes = remus_math::frame::Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), normal)
            .ok()
            .map(|frame| (frame.x, frame.y));
        let y = axes.map_or(Vec3::new(0.0, 0.0, 0.0), |(_, y)| y);
        Ok(Self {
            normal,
            axes,
            outer: build_loop(topo, face.outer_wire(), y)?,
            holes: face
                .inner_wires()
                .iter()
                .map(|&hole| build_loop(topo, hole, y))
                .collect::<Result<_, _>>()?,
        })
    }

    /// Whether `point`, on the face's plane, lies in the face. The parity
    /// test visits the outer loop and every hole independently. Nested holes
    /// alternate void and material, so the point is in the face when an even
    /// number of holes contain it.
    ///
    /// # Errors
    ///
    /// Refuses a degenerate normal, edge curves other than lines and
    /// circles, invalid edge domains, and ray events it cannot resolve
    /// exactly.
    pub fn contains(&self, point: Point3) -> Result<bool, CheckError> {
        let Some((x, y)) = self.axes else {
            remus_math::frame::Frame3::from_normal(point, self.normal)?;
            return Err(CheckError::DistanceFailed("degenerate plane normal".into()));
        };
        if !loop_contains(&self.outer, point, x, y)? {
            return Ok(false);
        }
        let mut in_holes = 0usize;
        for hole in &self.holes {
            in_holes += usize::from(loop_contains(hole, point, x, y)?);
        }
        Ok(in_holes.is_multiple_of(2))
    }
}

/// Exact planar trim membership for polygonal and circular boundaries.
///
/// # Errors
///
/// As [`PlanarTrim::build`] and [`PlanarTrim::contains`].
pub fn plane_point_in_face(
    topo: &Topology,
    face_id: FaceId,
    point: Point3,
    normal: Vec3,
) -> Result<bool, CheckError> {
    PlanarTrim::build(topo, face_id, normal)?.contains(point)
}

fn build_loop(topo: &Topology, wire_id: WireId, y: Vec3) -> Result<TrimLoop, CheckError> {
    match complete_circle_wire(topo, wire_id) {
        Ok(Some(circle)) => return Ok(TrimLoop::Circle(TrimCircle::new(circle, y))),
        Ok(None) => {}
        Err(CheckError::ValidationFailed(message)) => {
            return Ok(TrimLoop::Refused(Deferred::Validation(message)));
        }
        Err(error) => return Err(error),
    }
    let mut edges = Vec::new();
    for oriented in topo.wire(wire_id)?.edges() {
        let edge = topo.edge(oriented.edge())?;
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let (a, b) = match edge.strict_domain() {
            Ok(domain) => domain,
            Err(error) => {
                // As `crate::error::edge_domain_validation`.
                edges.push(TrimEdge::Refused(Deferred::Validation(error.to_string())));
                break;
            }
        };
        match edge.curve() {
            curve if edge::is_linear_curve(curve) => edges.push(TrimEdge::Segment(
                curve.evaluate_with_endpoints(a, start, end),
                curve.evaluate_with_endpoints(b, start, end),
            )),
            EdgeCurve::Circle(circle) => {
                edges.push(TrimEdge::Arc(TrimCircle::new(circle, y), a, b));
            }
            EdgeCurve::Line
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_) => {
                edges.push(TrimEdge::Refused(Deferred::Distance(
                    "certified planar trim membership requires line or circle boundaries",
                )));
                break;
            }
        }
    }
    Ok(TrimLoop::Edges(edges))
}

/// Recognize one complete circular traversal without summing approximate
/// interval lengths. Every carrier, parameter junction and topological
/// junction must agree, and the existing closed-edge domain rule checks the
/// final full turn and geometric closure using the original tolerance.
#[allow(clippy::float_cmp)]
pub(super) fn complete_circle_wire(
    topo: &Topology,
    wire_id: WireId,
) -> Result<Option<&Circle3D>, CheckError> {
    let wire = topo.wire(wire_id)?;
    let Some(first) = wire.edges().first() else {
        return Ok(None);
    };
    if !wire.is_closed() {
        return Ok(None);
    }
    let first_edge = topo.edge(first.edge())?;
    let EdgeCurve::Circle(carrier) = first_edge.curve() else {
        return Ok(None);
    };
    let (a, b) = first_edge
        .strict_domain()
        .map_err(crate::error::edge_domain_validation)?;
    let (start, end) = if first.is_forward() { (a, b) } else { (b, a) };
    let increasing = end > start;
    let first_vertex = first.oriented_start(first_edge);
    let mut parameter = start;
    let mut vertex = first_vertex;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return Ok(None);
        };
        if circle.center() != carrier.center()
            || circle.radius() != carrier.radius()
            || circle.u_axis() != carrier.u_axis()
            || circle.v_axis() != carrier.v_axis()
        {
            return Ok(None);
        }
        let (a, b) = edge
            .strict_domain()
            .map_err(crate::error::edge_domain_validation)?;
        let (a, b) = if oriented.is_forward() {
            (a, b)
        } else {
            (b, a)
        };
        if a != parameter || (b > a) != increasing || oriented.oriented_start(edge) != vertex {
            return Ok(None);
        }
        parameter = b;
        vertex = oriented.oriented_end(edge);
    }
    if vertex != first_vertex {
        return Ok(None);
    }
    let mut full_turn = first_edge.clone();
    full_turn.set_end(full_turn.start());
    full_turn.set_trim(Some((start, parameter)));
    Ok(full_turn.strict_domain().is_ok().then_some(carrier))
}

/// Ray-parity membership in one loop, casting along `x` from `point` with
/// `y` the in-plane normal of the ray.
#[allow(clippy::too_many_lines, clippy::float_cmp)] // Exact half-open interval endpoints.
fn loop_contains(trim: &TrimLoop, point: Point3, x: Vec3, y: Vec3) -> Result<bool, CheckError> {
    let edges = match trim {
        TrimLoop::Refused(deferred) => return Err(deferred.error()),
        TrimLoop::Circle(TrimCircle {
            circle,
            amplitude,
            phase,
            ..
        }) => {
            let amplitude = *amplitude;
            let center = circle.center() - point;
            let center_y = center.dot(y);
            if !amplitude.is_finite() || amplitude <= 0.0 || !center_y.is_finite() {
                return Err(CheckError::DistanceFailed(
                    "complete circular trim has no finite ray events".into(),
                ));
            }
            let ordinate = -center_y / amplitude;
            if ordinate.abs() >= 1.0 {
                return Ok(false);
            }
            let angle = ordinate.acos();
            // A complete traversal has each algebraic ray event exactly once.
            // Its arbitrary seam cannot remove or duplicate an event.
            let mut crossings = 0;
            for parameter in [phase - angle, *phase + angle] {
                let abscissa = (circle.evaluate(parameter) - point).dot(x);
                if !abscissa.is_finite() {
                    return Err(CheckError::DistanceFailed(
                        "complete circular trim has no finite ray events".into(),
                    ));
                }
                if abscissa > 0.0 {
                    crossings += 1;
                }
            }
            return Ok(crossings % 2 != 0);
        }
        TrimLoop::Edges(edges) => edges,
    };
    let mut crossings = 0;
    for trim_edge in edges {
        match trim_edge {
            TrimEdge::Refused(deferred) => return Err(deferred.error()),
            TrimEdge::Segment(start, end) => {
                let p = *start - point;
                let q = *end - point;
                let (py, qy) = (p.dot(y), q.dot(y));
                if (py > 0.0) != (qy > 0.0) {
                    let t = -py / (qy - py);
                    if (q.dot(x) - p.dot(x)).mul_add(t, p.dot(x)) > 0.0 {
                        crossings += 1;
                    }
                }
            }
            TrimEdge::Arc(
                TrimCircle {
                    circle,
                    cos_y,
                    sin_y,
                    amplitude,
                    phase,
                },
                a,
                b,
            ) => {
                let (a, b) = (*a, *b);
                let (cos_y, sin_y, amplitude, phase) = (*cos_y, *sin_y, *amplitude, *phase);
                let center = circle.center() - point;
                let ordinate = -center.dot(y) / amplitude;
                // A tangent contributes no parity change.
                if ordinate.abs() >= 1.0 {
                    continue;
                }
                let angle = ordinate.acos();
                let (lo, hi) = (a.min(b), a.max(b));
                let period = std::f64::consts::TAU;
                // Refuse arithmetic-ambiguous partial-arc admission. This is
                // an error enclosure, not a geometric endpoint snap: no
                // uncertain ray event is promoted into a crossing.
                let ordinate_scale = center.x().abs() * y.x().abs()
                    + center.y().abs() * y.y().abs()
                    + center.z().abs() * y.z().abs()
                    + cos_y.abs()
                    + sin_y.abs();
                let angle_roundoff = 64.0
                    * f64::EPSILON
                    * (lo.abs().max(hi.abs()).max(period)
                        + (ordinate_scale / amplitude) / (1.0 - ordinate * ordinate).sqrt());
                if !angle_roundoff.is_finite() || angle_roundoff >= period / 4.0 {
                    return Err(CheckError::DistanceFailed(
                        "circular trim ray-event admission is unresolved".into(),
                    ));
                }
                for raw in [phase - angle, phase + angle] {
                    let t = raw + period * ((lo - raw) / period).ceil();
                    if !t.is_finite()
                        || [t - period, t, t + period].into_iter().any(|candidate| {
                            (candidate - lo).abs() <= angle_roundoff
                                || (candidate - hi).abs() <= angle_roundoff
                        })
                    {
                        return Err(CheckError::DistanceFailed(
                            "circular trim ray-event endpoint is unresolved".into(),
                        ));
                    }
                    if t > hi {
                        continue;
                    }
                    let derivative = (-cos_y).mul_add(t.sin(), sin_y * t.cos());
                    let interior = t > lo && t < hi;
                    let lower_crossing = t == lo && derivative > 0.0;
                    let upper_crossing = t == hi && derivative < 0.0;
                    if (interior || lower_crossing || upper_crossing)
                        && (circle.evaluate(t) - point).dot(x) > 0.0
                    {
                        crossings += 1;
                    }
                }
            }
        }
    }
    Ok(crossings % 2 != 0)
}
