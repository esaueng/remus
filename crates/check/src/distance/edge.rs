//! Distances on authoritative edge intervals.

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};

use crate::CheckError;

/// Algebraic qualification of a clamped degree-one affine NURBS edge.
/// This admits the exact line conversion without sampled recognition.
pub(super) fn is_linear_curve(curve: &EdgeCurve) -> bool {
    match curve {
        EdgeCurve::Line => true,
        EdgeCurve::NurbsCurve(curve) => {
            let weights = curve.weights();
            let knots = curve.knots();
            curve.degree() == 1
                && curve.control_points().len() == 2
                && weights.len() == 2
                && weights[0].is_finite()
                && (f64::MIN_POSITIVE..=f64::MAX / 2.0).contains(&weights[0])
                && weights[0].to_bits() == weights[1].to_bits()
                && knots.len() == 4
                && knots[0].to_bits() == knots[1].to_bits()
                && knots[2].to_bits() == knots[3].to_bits()
        }
        EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => false,
    }
}

/// Minimum distance to a line segment or a circular arc.
///
/// Both interval endpoints and the circular stationary point are candidates.
/// A closed circle retains its full interval; its coincident vertices do not
/// turn it into a zero-length segment. Other carriers require a global curve
/// extremum solver and are refused rather than returning a sampled minimum.
///
/// # Errors
/// Returns an error for missing topology, invalid trim authority, or an
/// unsupported curve carrier.
pub fn point_to_edge(
    topo: &Topology,
    point: Point3,
    edge_id: EdgeId,
) -> Result<(f64, Point3), CheckError> {
    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let (a, b) = edge
        .strict_domain()
        .map_err(crate::error::edge_domain_validation)?;
    let (lo, hi) = (a.min(b), a.max(b));
    let mut best = match edge.curve() {
        curve if is_linear_curve(curve) => {
            let (lower, upper) = if matches!(curve, EdgeCurve::Line) {
                (start, end)
            } else {
                (
                    curve.evaluate_with_endpoints(lo, start, end),
                    curve.evaluate_with_endpoints(hi, start, end),
                )
            };
            let delta = upper - lower;
            let length_sq = delta.length_squared();
            if !length_sq.is_finite() {
                return Err(CheckError::DistanceFailed(
                    "line extremum exceeds the finite arithmetic range".into(),
                ));
            }
            let t = if length_sq == 0.0 {
                0.0
            } else {
                ((point - lower).dot(delta) / length_sq).clamp(0.0, 1.0)
            };
            let closest = lower + delta * t;
            ((point - closest).length(), closest)
        }
        EdgeCurve::Circle(circle) => {
            let mut closest = circle.evaluate(lo);
            let mut distance = (point - closest).length();
            let upper = circle.evaluate(hi);
            if (point - upper).length() < distance {
                closest = upper;
                distance = (point - upper).length();
            }
            let raw = circle.project(point);
            let period = std::f64::consts::TAU;
            let t = raw + period * ((lo - raw) / period).ceil();
            if t <= hi {
                let stationary = circle.evaluate(t);
                let candidate = (point - stationary).length();
                if candidate < distance {
                    closest = stationary;
                    distance = candidate;
                }
            }
            (distance, closest)
        }
        EdgeCurve::Line
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::NurbsCurve(_) => {
            return Err(CheckError::DistanceFailed(
                "a certified edge minimum is currently supported only for line and circle carriers"
                    .into(),
            ));
        }
    };
    // Sewn endpoint vertices belong to the boundary as well as the carrier.
    for vertex in [start, end] {
        let distance = (point - vertex).length();
        if distance < best.0 {
            best = (distance, vertex);
        }
    }
    super::ensure_distance_witness(best.0, point, best.1)?;
    Ok(best)
}

/// Local numerical boundary estimate for NURBS faces and NURBS trim curves,
/// including trims sewn onto an analytic carrier.
/// This is not used to certify a solid-to-solid minimum.
pub(super) fn point_to_edge_estimate(
    topo: &Topology,
    point: Point3,
    edge_id: EdgeId,
) -> Result<(f64, Point3), CheckError> {
    let edge = topo.edge(edge_id)?;
    if is_linear_curve(edge.curve()) || matches!(edge.curve(), EdgeCurve::Circle(_)) {
        return point_to_edge(topo, point, edge_id);
    }
    let (a, b) = edge
        .strict_domain()
        .map_err(crate::error::edge_domain_validation)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let curve = BoundaryCurve {
        curve: edge.curve(),
        start,
        end,
        domain: (a.min(b), a.max(b)),
    };
    let projection =
        remus_geometry::extrema::point_to_curve(point, &curve, curve.domain.0, curve.domain.1);
    let mut best = (projection.distance, projection.point);
    for vertex in [start, end] {
        let distance = (point - vertex).length();
        if distance < best.0 {
            best = (distance, vertex);
        }
    }
    super::ensure_distance_witness(best.0, point, best.1)?;
    Ok(best)
}

struct BoundaryCurve<'a> {
    curve: &'a EdgeCurve,
    start: Point3,
    end: Point3,
    domain: (f64, f64),
}

impl remus_math::traits::ParametricCurve for BoundaryCurve<'_> {
    fn evaluate(&self, t: f64) -> Point3 {
        self.curve.evaluate_with_endpoints(t, self.start, self.end)
    }
    fn tangent(&self, t: f64) -> remus_math::vec::Vec3 {
        self.curve.tangent_with_endpoints(t, self.start, self.end)
    }
    fn domain(&self) -> (f64, f64) {
        self.domain
    }
}

/// Minimum distance between two line segments [p0, p1] and [q0, q1].
///
/// Returns (distance, closest\_on\_seg1, closest\_on\_seg2).
#[allow(clippy::similar_names)]
pub fn segment_segment_distance(
    p0: Point3,
    p1: Point3,
    q0: Point3,
    q1: Point3,
) -> (f64, Point3, Point3) {
    remus_geometry::extrema::segment_segment_distance(p0, p1, q0, q1)
}
