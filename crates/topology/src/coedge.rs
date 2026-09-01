//! Coedge — one directed use of an edge by one face boundary.
//!
//! A coedge is the per-use identity a shared 3D edge lacks: a periodic
//! face's seam edge is one [`Edge`](crate::edge::Edge) but **two** coedges,
//! one per parameter-space branch, each able to carry its own p-curve. See
//! `docs/design/rfc-0002-coedge-architecture.md`.
//!
//! Coedges are the authoritative per-use boundary records. The owning
//! [`Loop`](crate::face_loop::Loop) defines boundary order; face wires are a
//! compatibility view kept synchronized by topology-owned mutation APIs.
//! Direct legacy `Face`/`Wire` mutation can still diverge until the 2.0g
//! facade-removal gate, and strict validation refuses that state.

use crate::arena;
use crate::edge::EdgeId;
use crate::face_loop::LoopId;
use crate::pcurve::PCurve;

/// Typed handle for a [`Coedge`] stored in an [`Arena`](crate::Arena).
pub type CoedgeId = arena::Id<Coedge>;

/// Integer periodic lifts carried by one surface-boundary use.
///
/// `u` and `v` count full parameter-space periods relative to the pcurve's
/// base branch. Most analytic faces are periodic only in `u`; the second
/// component keeps the representation truthful for tori and periodic NURBS.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PeriodicWinding {
    u: i32,
    v: i32,
}

impl PeriodicWinding {
    /// The zero-lift branch.
    pub const ZERO: Self = Self { u: 0, v: 0 };

    /// Creates explicit periodic lift counts.
    #[must_use]
    pub const fn new(u: i32, v: i32) -> Self {
        Self { u, v }
    }

    /// Number of full `u` periods.
    #[must_use]
    pub const fn u(self) -> i32 {
        self.u
    }

    /// Number of full `v` periods.
    #[must_use]
    pub const fn v(self) -> i32 {
        self.v
    }
}

/// One directed use of an edge by one face boundary loop.
#[derive(Debug, Clone)]
pub struct Coedge {
    /// The underlying 3D edge.
    edge: EdgeId,
    /// Traversal orientation relative to the edge's natural direction.
    forward: bool,
    /// The loop this use belongs to.
    parent_loop: LoopId,
    /// This use's curve in the owning face's parameter space.
    pcurve: Option<PCurve>,
    /// Integer lifts of the pcurve branch on periodic surface directions.
    periodic_winding: PeriodicWinding,
}

impl Coedge {
    /// Creates a new coedge use.
    #[must_use]
    pub const fn new(edge: EdgeId, forward: bool, parent_loop: LoopId) -> Self {
        Self::with_pcurve(edge, forward, parent_loop, None)
    }

    /// Creates a new coedge use with embedded pcurve authority.
    #[must_use]
    pub const fn with_pcurve(
        edge: EdgeId,
        forward: bool,
        parent_loop: LoopId,
        pcurve: Option<PCurve>,
    ) -> Self {
        Self {
            edge,
            forward,
            parent_loop,
            pcurve,
            periodic_winding: PeriodicWinding::ZERO,
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

    /// This use's p-curve, if one is required and stored.
    #[must_use]
    pub const fn pcurve(&self) -> Option<&PCurve> {
        self.pcurve.as_ref()
    }

    /// This use's periodic parameter-space lift counts.
    #[must_use]
    pub const fn periodic_winding(&self) -> PeriodicWinding {
        self.periodic_winding
    }

    /// Replaces this use's p-curve and returns the previous value.
    pub(crate) fn replace_pcurve(&mut self, pcurve: Option<PCurve>) -> Option<PCurve> {
        std::mem::replace(&mut self.pcurve, pcurve)
    }

    /// Replaces this use's periodic winding and returns the previous value.
    pub(crate) fn replace_periodic_winding(&mut self, winding: PeriodicWinding) -> PeriodicWinding {
        std::mem::replace(&mut self.periodic_winding, winding)
    }
}
