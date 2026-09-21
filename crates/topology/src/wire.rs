//! Wire — an ordered sequence of oriented edges forming a path or loop.

use crate::TopologyError;
use crate::arena;
use crate::edge::{Edge, EdgeId};
use crate::topology::BodyClass;
use crate::vertex::VertexId;

/// Typed handle for a [`Wire`] stored in an [`Arena`](crate::Arena).
pub type WireId = arena::Id<Wire>;

/// An edge reference with an orientation flag.
///
/// When `forward` is `true` the edge is traversed from its start to its end.
/// When `false` the traversal is reversed (end to start).
#[derive(Debug, Clone, Copy)]
pub struct OrientedEdge {
    /// The referenced edge.
    edge: EdgeId,
    /// `true` if the edge is traversed in its natural direction.
    forward: bool,
}

impl OrientedEdge {
    /// Creates a new oriented edge reference.
    #[must_use]
    pub const fn new(edge: EdgeId, forward: bool) -> Self {
        Self { edge, forward }
    }

    /// Returns the referenced edge id.
    #[must_use]
    pub const fn edge(&self) -> EdgeId {
        self.edge
    }

    /// Returns `true` if this edge is traversed in its natural direction.
    #[must_use]
    pub const fn is_forward(&self) -> bool {
        self.forward
    }

    /// Returns the vertex at the start of traversal for this oriented edge.
    ///
    /// When traversed forward the start vertex is `edge.start()`; when reversed
    /// it is `edge.end()`.
    #[must_use]
    pub const fn oriented_start(&self, edge: &Edge) -> VertexId {
        if self.forward {
            edge.start()
        } else {
            edge.end()
        }
    }

    /// Returns the vertex at the end of traversal for this oriented edge.
    ///
    /// When traversed forward the end vertex is `edge.end()`; when reversed
    /// it is `edge.start()`.
    #[must_use]
    pub const fn oriented_end(&self, edge: &Edge) -> VertexId {
        if self.forward {
            edge.end()
        } else {
            edge.start()
        }
    }
}

/// A topological wire: an ordered chain of oriented edges.
///
/// A wire must contain at least one edge. It may be open (a path) or
/// closed (a loop).
#[derive(Debug, Clone)]
pub struct Wire {
    /// The ordered sequence of oriented edges.
    edges: Vec<OrientedEdge>,
    /// Whether this wire forms a closed loop.
    closed: bool,
    /// Dimensional class stored on this wire.
    body_class: BodyClass,
}

impl Wire {
    /// Creates a new wire from a non-empty list of oriented edges.
    ///
    /// The `closed` flag indicates whether the wire forms a closed loop.
    /// Topological validation (e.g. checking that the last edge connects
    /// back to the first) is performed separately via
    /// [`validation::validate_wire_closed`](crate::validation::validate_wire_closed).
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::Empty`] if `edges` is empty.
    pub fn new(edges: Vec<OrientedEdge>, closed: bool) -> Result<Self, TopologyError> {
        if edges.is_empty() {
            return Err(TopologyError::Empty { entity: "wire" });
        }
        Ok(Self {
            edges,
            closed,
            body_class: BodyClass::Wire,
        })
    }

    /// Returns the ordered edges of this wire.
    #[must_use]
    pub fn edges(&self) -> &[OrientedEdge] {
        &self.edges
    }

    /// Returns mutable access to the ordered edges of this wire.
    ///
    /// Allows in-place mutation (reorder, replace) but not removal.
    /// The wire must always contain at least one edge.
    pub fn edges_mut(&mut self) -> &mut [OrientedEdge] {
        &mut self.edges
    }

    /// Returns `true` if this wire forms a closed loop.
    #[must_use]
    pub const fn is_closed(&self) -> bool {
        self.closed
    }

    /// Returns the dimensional class stored on this wire.
    #[must_use]
    pub const fn body_class(&self) -> BodyClass {
        self.body_class
    }

    pub(crate) fn set_body_class(&mut self, body_class: BodyClass) {
        self.body_class = body_class;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::vec::Point3;

    use crate::edge::EdgeCurve;
    use crate::topology::Topology;
    use crate::vertex::Vertex;

    use super::*;

    /// Two distinct live edge handles to build wires from.
    fn two_edges() -> (Topology, EdgeId, EdgeId) {
        let mut topo = Topology::new();
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        (topo, e0, e1)
    }

    #[test]
    fn closed_flag_round_trips_both_ways() {
        let (_topo, e0, e1) = two_edges();
        let open = Wire::new(
            vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
            false,
        )
        .unwrap();
        assert!(!open.is_closed(), "a wire built open reports open");

        let closed = Wire::new(vec![OrientedEdge::new(e0, true)], true).unwrap();
        assert!(closed.is_closed(), "a wire built closed reports closed");
    }

    #[test]
    fn edges_mut_exposes_the_stored_edges_for_in_place_replacement() {
        let (_topo, e0, e1) = two_edges();
        let mut wire = Wire::new(
            vec![OrientedEdge::new(e0, true), OrientedEdge::new(e1, true)],
            false,
        )
        .unwrap();

        {
            let edges = wire.edges_mut();
            assert_eq!(edges.len(), 2, "edges_mut sees the wire's own storage");
            edges[0] = OrientedEdge::new(e1, false);
        }

        // The write through the mutable slice is visible to the shared view.
        assert_eq!(wire.edges().len(), 2);
        assert_eq!(wire.edges()[0].edge(), e1);
        assert!(!wire.edges()[0].is_forward());
        assert_eq!(wire.edges()[1].edge(), e1);
        assert!(wire.edges()[1].is_forward());
    }

    #[test]
    fn set_body_class_replaces_the_stored_class() {
        let (_topo, e0, _e1) = two_edges();
        let mut wire = Wire::new(vec![OrientedEdge::new(e0, true)], false).unwrap();
        assert_eq!(wire.body_class(), BodyClass::Wire);

        wire.set_body_class(BodyClass::Sheet);
        assert_eq!(wire.body_class(), BodyClass::Sheet);

        wire.set_body_class(BodyClass::Solid);
        assert_eq!(wire.body_class(), BodyClass::Solid);
    }
}
