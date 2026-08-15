//! Coedge — one directed use of an edge by one face boundary.
//!
//! A coedge is the per-use identity a shared 3D edge lacks: a periodic
//! face's seam edge is one [`Edge`](crate::edge::Edge) but **two** coedges,
//! one per parameter-space branch, each able to carry its own p-curve. See
//! `docs/design/rfc-0002-coedge-architecture.md`.
//!
//! Stage 1 (this module): coedges are derived from a face's wires on
//! request ([`Topology::build_face_loops`](crate::Topology::build_face_loops))
//! and validated against them. Wires remain the authoritative boundary
//! representation until the RFC's Stage 2 authority flip.

use crate::arena;
use crate::edge::EdgeId;
use crate::face_loop::LoopId;

/// Typed handle for a [`Coedge`] stored in an [`Arena`](crate::Arena).
pub type CoedgeId = arena::Id<Coedge>;

/// One directed use of an edge by one face boundary loop.
#[derive(Debug, Clone)]
pub struct Coedge {
    /// The underlying 3D edge.
    edge: EdgeId,
    /// Traversal orientation relative to the edge's natural direction.
    forward: bool,
    /// The loop this use belongs to.
    parent_loop: LoopId,
}

impl Coedge {
    /// Creates a new coedge use.
    #[must_use]
    pub const fn new(edge: EdgeId, forward: bool, parent_loop: LoopId) -> Self {
        Self {
            edge,
            forward,
            parent_loop,
        }
    }

    /// The underlying 3D edge.
    #[must_use]
    pub const fn edge(&self) -> EdgeId {
        self.edge
    }

    /// `true` if the edge is traversed in its natural direction.
    #[must_use]
    pub const fn is_forward(&self) -> bool {
        self.forward
    }

    /// The loop this use belongs to.
    #[must_use]
    pub const fn parent_loop(&self) -> LoopId {
        self.parent_loop
    }
}
