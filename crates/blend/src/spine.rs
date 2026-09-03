// Walking engine infrastructure — used progressively as more blend paths are wired up.
#![allow(dead_code)]
//! Spine: ordered edge chain with arc-length parameterization.
//!
//! A spine represents the guideline along which a fillet or chamfer is
//! computed. It may consist of multiple edges forming a G1-continuous chain.

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;

use crate::BlendError;

/// An ordered chain of edges forming the fillet guideline.
#[derive(Debug, Clone)]
pub struct Spine {
    /// Ordered edge IDs in the chain.
    edges: Vec<EdgeId>,
    /// Cumulative arc-length at each edge boundary.
    /// `params[0] = 0`, `params[i]` = cumulative length through edge `i-1`.
    params: Vec<f64>,
    /// Total arc length of the spine.
    length: f64,
    /// Whether the chain forms a closed loop.
    is_closed: bool,
    /// Per-edge traversal direction: `false` where the chain runs against the
    /// edge's own start→end orientation.
    ///
    /// Edge orientation is a property of the topology, not of the chain, so a
    /// ridgeline assembled from several edges routinely contains some that
    /// point "backwards". Without this the spine samples those edges from the
    /// wrong end and its parameterization jumps around instead of advancing.
    forward: Vec<bool>,
}

/// Arc length used by the walker's global spine parameter.
///
/// A closed rim has coincident topological endpoints, so its chord length is
/// zero even though the curve has a full, authoritative parameter span. Keep
/// circles exact and use a deterministic chordal estimate for the remaining
/// curved carriers; the walker needs a monotone station scale, not a second
/// geometry representation.
fn edge_arc_length(topo: &Topology, edge_id: EdgeId) -> Result<f64, BlendError> {
    const SEGMENTS: u32 = 64;

    let edge = topo.edge(edge_id)?;
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    if matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
        return Ok((end - start).length());
    }
    let (t0, t1) = edge.strict_domain().map_err(crate::edge_domain_input)?;
    if let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve() {
        return Ok(circle.radius() * (t1 - t0).abs());
    }

    let mut previous = edge.curve().evaluate_with_endpoints(t0, start, end);
    let mut length = 0.0;
    for index in 1..=SEGMENTS {
        let fraction = f64::from(index) / f64::from(SEGMENTS);
        let parameter = (t1 - t0).mul_add(fraction, t0);
        let point = edge.curve().evaluate_with_endpoints(parameter, start, end);
        length += (point - previous).length();
        previous = point;
    }
    Ok(length)
}

impl Spine {
    /// Build a spine from a single edge.
    ///
    /// # Errors
    /// Returns `BlendError` if the edge or its vertices cannot be found.
    pub fn from_single_edge(topo: &Topology, edge_id: EdgeId) -> Result<Self, BlendError> {
        let edge = topo.edge(edge_id)?;
        let length = edge_arc_length(topo, edge_id)?;

        Ok(Self {
            edges: vec![edge_id],
            params: vec![0.0, length],
            length,
            is_closed: edge.is_closed(),
            forward: vec![true],
        })
    }

    /// Build a spine from an ordered chain of edges.
    ///
    /// Edges must be G1-continuous (verified by caller).
    ///
    /// # Errors
    /// Returns `BlendError` if any edge or vertex cannot be found.
    pub fn from_chain(topo: &Topology, edges: Vec<EdgeId>) -> Result<Self, BlendError> {
        // Walk the chain to learn which way each edge is traversed. The first
        // edge is entered from whichever end the second edge does NOT touch.
        let mut forward = Vec::with_capacity(edges.len());
        if edges.len() <= 1 {
            forward.resize(edges.len(), true);
        } else {
            let first = topo.edge(edges[0])?;
            let second = topo.edge(edges[1])?;
            let joins = [second.start(), second.end()];
            let first_forward = joins.contains(&first.end());
            forward.push(first_forward);
            let mut current = if first_forward {
                first.end()
            } else {
                first.start()
            };
            for &eid in &edges[1..] {
                let edge = topo.edge(eid)?;
                let fwd = edge.start() == current;
                forward.push(fwd);
                current = if fwd { edge.end() } else { edge.start() };
            }
        }

        let mut params = Vec::with_capacity(edges.len() + 1);
        params.push(0.0);
        let mut cumulative = 0.0;

        for &eid in &edges {
            cumulative += edge_arc_length(topo, eid)?;
            params.push(cumulative);
        }

        let is_closed = if edges.len() == 1 {
            topo.edge(edges[0])?.is_closed()
        } else if edges.len() >= 2 {
            let first = topo.edge(edges[0])?;
            let last = topo.edge(edges[edges.len() - 1])?;
            let first_start = if forward[0] {
                first.start()
            } else {
                first.end()
            };
            let last_end = if forward[forward.len() - 1] {
                last.end()
            } else {
                last.start()
            };
            first_start == last_end
        } else {
            false
        };

        Ok(Self {
            edges,
            params,
            length: cumulative,
            is_closed,
            forward,
        })
    }

    /// Total arc length.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Number of edges in the chain.
    #[must_use]
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Whether the spine forms a closed loop.
    #[must_use]
    pub fn is_closed(&self) -> bool {
        self.is_closed
    }

    /// The edges in order.
    #[must_use]
    pub fn edges(&self) -> &[EdgeId] {
        &self.edges
    }

    /// Map a global spine parameter `s in [0, length]` to `(edge_index, local_t in [0,1])`.
    #[must_use]
    pub fn locate(&self, s: f64) -> (usize, f64) {
        let s_clamped = s.clamp(0.0, self.length);
        for i in 0..self.edges.len() {
            let s0 = self.params[i];
            let s1 = self.params[i + 1];
            if s_clamped <= s1 || i == self.edges.len() - 1 {
                let edge_len = s1 - s0;
                let t = if edge_len > f64::EPSILON {
                    (s_clamped - s0) / edge_len
                } else {
                    0.0
                };
                return (i, t.clamp(0.0, 1.0));
            }
        }
        (self.edges.len() - 1, 1.0)
    }

    /// Evaluate the 3D point on the spine at global parameter `s`.
    ///
    /// For `Line` edges this is linear interpolation. For `Circle`, `Ellipse`,
    /// and `NurbsCurve` edges, the actual curve geometry is evaluated.
    ///
    /// # Errors
    /// Returns `BlendError` if topology lookups fail.
    pub fn evaluate(&self, topo: &Topology, s: f64) -> Result<Point3, BlendError> {
        let (idx, t) = self.locate(s);
        let t = if self.forward.get(idx).copied().unwrap_or(true) {
            t
        } else {
            1.0 - t
        };
        let edge = topo.edge(self.edges[idx])?;
        let p_start = topo.vertex(edge.start())?.point();
        let p_end = topo.vertex(edge.end())?.point();
        let curve = edge.curve();
        let (t0, t1) = edge.strict_domain().map_err(crate::edge_domain_input)?;
        let param = t0 + (t1 - t0) * t;
        Ok(curve.evaluate_with_endpoints(param, p_start, p_end))
    }

    /// Evaluate the tangent direction on the spine at global parameter `s`.
    ///
    /// Returns the unit tangent, or a fallback Z-axis if the edge is degenerate.
    /// For curved edges, evaluates the actual curve tangent.
    ///
    /// # Errors
    /// Returns `BlendError` if topology lookups fail.
    pub fn tangent(&self, topo: &Topology, s: f64) -> Result<Vec3, BlendError> {
        let (idx, t) = self.locate(s);
        let fwd = self.forward.get(idx).copied().unwrap_or(true);
        let t = if fwd { t } else { 1.0 - t };
        let edge = topo.edge(self.edges[idx])?;
        let p_start = topo.vertex(edge.start())?.point();
        let p_end = topo.vertex(edge.end())?.point();
        let curve = edge.curve();
        let (t0, t1) = edge.strict_domain().map_err(crate::edge_domain_input)?;
        let param = t0 + (t1 - t0) * t;
        let tan = curve.tangent_with_endpoints(param, p_start, p_end);
        let tan = if fwd { tan } else { -tan };
        Ok(tan.normalize().unwrap_or(Vec3::new(0.0, 0.0, 1.0)))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_topology::Topology;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::vertex::Vertex;

    use remus_math::curves::Circle3D;

    fn make_line_edge(topo: &mut Topology, a: Point3, b: Point3) -> EdgeId {
        let v0 = topo.add_vertex(Vertex::new(a, 1e-7));
        let v1 = topo.add_vertex(Vertex::new(b, 1e-7));
        topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line))
    }

    #[test]
    fn single_edge_spine_length() {
        let mut topo = Topology::new();
        let eid = make_line_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        );
        let spine = Spine::from_single_edge(&topo, eid).unwrap();
        assert!((spine.length() - 10.0).abs() < 1e-10);
        assert_eq!(spine.edge_count(), 1);
        assert!(!spine.is_closed());
    }

    #[test]
    fn closed_circle_spine_uses_its_authoritative_full_turn() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let start = circle.evaluate(0.0);
        let vertex = topo.add_vertex(Vertex::new(start, 1e-7));
        let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, std::f64::consts::TAU)));
        let edge = topo.add_edge(edge);

        let spine = Spine::from_chain(&topo, vec![edge]).unwrap();

        assert!(spine.is_closed());
        assert!((spine.length() - 4.0 * std::f64::consts::PI).abs() < 1e-12);
        let opposite = Point3::new(-start.x(), -start.y(), -start.z());
        assert!((spine.evaluate(&topo, spine.length() * 0.5).unwrap() - opposite).length() < 1e-12);
    }

    #[test]
    fn reversed_first_edge_still_closes_multi_edge_spine() {
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let reversed_first = topo.add_edge(Edge::new(v1, v0, EdgeCurve::Line));
        let second = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let third = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));

        let spine = Spine::from_chain(&topo, vec![reversed_first, second, third]).unwrap();

        assert!(spine.is_closed());
        assert_eq!(
            spine.evaluate(&topo, 0.0).unwrap(),
            topo.vertex(v0).unwrap().point()
        );
        assert_eq!(
            spine.evaluate(&topo, spine.length()).unwrap(),
            topo.vertex(v0).unwrap().point()
        );
    }

    #[test]
    fn locate_maps_parameter_correctly() {
        let mut topo = Topology::new();
        let eid = make_line_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        );
        let spine = Spine::from_single_edge(&topo, eid).unwrap();
        let (idx, t) = spine.locate(5.0);
        assert_eq!(idx, 0);
        assert!((t - 0.5).abs() < 1e-10);
    }

    #[test]
    fn evaluate_midpoint() {
        let mut topo = Topology::new();
        let eid = make_line_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        );
        let spine = Spine::from_single_edge(&topo, eid).unwrap();
        let mid = spine.evaluate(&topo, 5.0).unwrap();
        assert!((mid - Point3::new(5.0, 0.0, 0.0)).length() < 1e-10);
    }

    #[test]
    fn curved_spine_uses_stored_reversed_major_range_and_refuses_missing_authority() {
        let mut topo = Topology::new();
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let range = (5.5, 0.5);
        let start = topo.add_vertex(Vertex::new(circle.evaluate(range.0), 1e-7));
        let end = topo.add_vertex(Vertex::new(circle.evaluate(range.1), 1e-7));
        let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
        edge.set_trim(Some(range));
        let edge_id = topo.add_edge(edge);
        let spine = Spine::from_single_edge(&topo, edge_id).unwrap();

        let midpoint = spine.evaluate(&topo, spine.length() * 0.5).unwrap();
        let expected = circle.evaluate(f64::midpoint(range.0, range.1));
        assert!((midpoint - expected).length() < 1e-12);
        let complementary =
            circle.evaluate(f64::midpoint(range.0, range.1 + std::f64::consts::TAU));
        assert!((midpoint - complementary).length() > 1.0);

        topo.edge_mut(edge_id).unwrap().set_trim(None);
        let error = spine.evaluate(&topo, spine.length() * 0.5).unwrap_err();
        assert!(matches!(error, BlendError::InvalidInput { .. }));
        assert!(
            error
                .to_string()
                .contains("no authoritative parameter range")
        );
    }
}
