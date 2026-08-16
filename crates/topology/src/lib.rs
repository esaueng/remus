//! # brepkit-topology
//!
//! Boundary representation (B-Rep) topological data structures.
//!
//! This is layer L1, depending only on `brepkit-math`.
//!
//! # Architecture
//!
//! All topological entities are stored in a central [`Arena`] and referenced
//! via typed index handles ([`VertexId`], [`EdgeId`], etc.). This avoids
//! reference counting overhead and enables efficient traversal.

pub mod adjacency;
pub mod arena;
pub mod builder;
pub mod coedge;
pub mod compound;
pub mod compsolid;
pub mod edge;
pub mod explorer;
pub mod face;
pub mod face_loop;

pub mod pcurve;
pub mod shell;
pub mod solid;
#[cfg(feature = "test-utils")]
pub mod test_utils;
pub mod topology;
pub mod transaction;
pub mod validation;
pub mod vertex;
pub mod wire;

pub use arena::Arena;
pub use coedge::{Coedge, CoedgeId};
pub use compound::CompoundId;
pub use compsolid::CompSolidId;
pub use edge::EdgeId;
pub use face::FaceId;
pub use face_loop::{Loop, LoopId};
pub use shell::ShellId;
pub use solid::SolidId;
pub use topology::Topology;
pub use vertex::VertexId;
pub use wire::{OrientedEdge, WireId};

/// Errors from topology operations.
#[derive(Debug, thiserror::Error)]
pub enum TopologyError {
    /// A referenced vertex ID does not exist in the arena.
    #[error("vertex {0:?} not found")]
    VertexNotFound(vertex::VertexId),

    /// A referenced edge ID does not exist in the arena.
    #[error("edge {0:?} not found")]
    EdgeNotFound(edge::EdgeId),

    /// A referenced wire ID does not exist in the arena.
    #[error("wire {0:?} not found")]
    WireNotFound(wire::WireId),

    /// A referenced face ID does not exist in the arena.
    #[error("face {0:?} not found")]
    FaceNotFound(face::FaceId),

    /// A referenced shell ID does not exist in the arena.
    #[error("shell {0:?} not found")]
    ShellNotFound(shell::ShellId),

    /// A referenced solid ID does not exist in the arena.
    #[error("solid {0:?} not found")]
    SolidNotFound(solid::SolidId),

    /// A referenced compound ID does not exist in the arena.
    #[error("compound {0:?} not found")]
    CompoundNotFound(compound::CompoundId),

    /// A referenced comp-solid ID does not exist in the arena.
    #[error("compsolid {0:?} not found")]
    CompSolidNotFound(compsolid::CompSolidId),

    /// A wire does not form a closed loop.
    #[error("wire is not closed")]
    WireNotClosed,

    /// The topology is not manifold.
    #[error("non-manifold topology: {reason}")]
    NonManifold {
        /// Description of the manifold violation.
        reason: String,
    },

    /// An empty collection was provided where at least one element is required.
    #[error("empty {entity} — at least one element is required")]
    Empty {
        /// The kind of entity that was empty.
        entity: &'static str,
    },

    /// A wire's edge geometry does not lie within tolerance of any single
    /// plane, so a planar face cannot be constructed from it.
    #[error("wire is not planar")]
    NotPlanar,

    /// A referenced loop ID does not exist in the arena.
    #[error("loop {0:?} not found")]
    LoopNotFound(face_loop::LoopId),

    /// A referenced coedge ID does not exist in the arena.
    #[error("coedge {0:?} not found")]
    CoedgeNotFound(coedge::CoedgeId),

    /// A face's derived loops disagree with its authoritative wires
    /// (RFC 0002, Stage 1 consistency invariant). Divergence is a kernel
    /// bug, not a modeling failure.
    #[error("derived loops of face {face:?} do not match its wires")]
    LoopWireMismatch {
        /// The face whose derivation is stale or wrong.
        face: face::FaceId,
    },

    /// A loop's coedges do not connect end-to-start under their
    /// orientations.
    #[error("loop of face {face:?} is not connected")]
    LoopNotConnected {
        /// The face whose loop fails connectivity.
        face: face::FaceId,
    },

    /// An `(edge, face)` pcurve request is ambiguous because the face uses
    /// the edge twice (a periodic seam). The per-use oriented API must be
    /// used instead; answering with either branch would be arbitrary.
    #[error(
        "pcurve request for edge {edge:?} on face {face:?} is ambiguous: \
         the face uses this edge twice (seam); address the use by orientation"
    )]
    SeamPcurveAmbiguous {
        /// The seam edge.
        edge: edge::EdgeId,
        /// The face using it twice.
        face: face::FaceId,
    },

    /// A pcurve's surface image deviates from the 3D edge beyond tolerance
    /// under the shared parameterization (`SameParameter`).
    #[error(
        "pcurve of edge {edge:?} on face {face:?} deviates {max_deviation} \
         from the 3D curve (limit {tolerance}) at parameter {at_parameter}"
    )]
    SameParameterExceeded {
        /// The edge whose pcurve deviates.
        edge: edge::EdgeId,
        /// The face carrying the pcurve.
        face: face::FaceId,
        /// Largest sampled deviation, in model units.
        max_deviation: f64,
        /// The pcurve parameter where it occurred.
        at_parameter: f64,
        /// The limit that was exceeded.
        tolerance: f64,
    },

    /// A pcurve's endpoints do not map to the edge's bounding vertices
    /// within tolerance (`SameRange`).
    #[error(
        "pcurve of edge {edge:?} on face {face:?} misses the edge's \
         endpoints by {max_deviation} (limit {tolerance})"
    )]
    SameRangeExceeded {
        /// The edge whose pcurve range is wrong.
        edge: edge::EdgeId,
        /// The face carrying the pcurve.
        face: face::FaceId,
        /// Larger of the two endpoint deviations, in model units.
        max_deviation: f64,
        /// The limit that was exceeded.
        tolerance: f64,
    },
}

/// Errors from retiring a solid and its unshared topology.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum DeleteSolidError {
    /// The solid or its topology tree contains an invalid handle.
    #[error(transparent)]
    Topology(#[from] TopologyError),

    /// A live root still references the solid.
    #[error("solid {solid:?} is referenced by live {dependent} {dependent_index}")]
    Referenced {
        /// The solid that cannot be retired.
        solid: solid::SolidId,
        /// The kind of dependent root.
        dependent: &'static str,
        /// The arena index of the dependent root.
        dependent_index: usize,
    },
}

impl brepkit_math::diagnostic::ToDiagnostic for TopologyError {
    fn diagnostic(&self) -> brepkit_math::diagnostic::Diagnostic {
        use brepkit_math::diagnostic::{Diagnostic, FailureCategory};
        let message = self.to_string();
        let entity_not_found = |entity: &'static str, index: usize| {
            Diagnostic::new(FailureCategory::InvalidInput, "entity_not_found", &message)
                .with_detail("entity", entity)
                .with_detail("index", index)
        };
        match self {
            Self::VertexNotFound(id) => entity_not_found("vertex", id.index()),
            Self::EdgeNotFound(id) => entity_not_found("edge", id.index()),
            Self::WireNotFound(id) => entity_not_found("wire", id.index()),
            Self::FaceNotFound(id) => entity_not_found("face", id.index()),
            Self::ShellNotFound(id) => entity_not_found("shell", id.index()),
            Self::SolidNotFound(id) => entity_not_found("solid", id.index()),
            Self::CompoundNotFound(id) => entity_not_found("compound", id.index()),
            Self::CompSolidNotFound(id) => entity_not_found("compsolid", id.index()),
            Self::WireNotClosed => {
                Diagnostic::new(FailureCategory::InvalidTopology, "wire_not_closed", message)
            }
            Self::NonManifold { .. } => {
                Diagnostic::new(FailureCategory::InvalidTopology, "non_manifold", message)
            }
            Self::Empty { entity } => {
                Diagnostic::new(FailureCategory::InvalidInput, "empty_collection", message)
                    .with_detail("entity", *entity)
            }
            Self::NotPlanar => {
                Diagnostic::new(FailureCategory::InvalidTopology, "wire_not_planar", message)
            }
            Self::LoopNotFound(id) => entity_not_found("loop", id.index()),
            Self::CoedgeNotFound(id) => entity_not_found("coedge", id.index()),
            Self::LoopWireMismatch { face } => {
                // Internal, not invalid_topology: only the kernel writes
                // derivations, so divergence is a kernel defect.
                Diagnostic::new(FailureCategory::Internal, "loop_wire_mismatch", message)
                    .with_detail("face", face.index())
            }
            Self::LoopNotConnected { face } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "loop_not_connected",
                message,
            )
            .with_detail("face", face.index()),
            Self::SeamPcurveAmbiguous { edge, face } => Diagnostic::new(
                FailureCategory::InvalidTopology,
                "seam_pcurve_ambiguous",
                message,
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index()),
            Self::SameParameterExceeded {
                edge,
                face,
                max_deviation,
                at_parameter,
                tolerance,
            } => Diagnostic::new(
                FailureCategory::ToleranceViolation,
                "same_parameter_exceeded",
                message,
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("maxDeviation", *max_deviation)
            .with_detail("atParameter", *at_parameter)
            .with_detail("tolerance", *tolerance),
            Self::SameRangeExceeded {
                edge,
                face,
                max_deviation,
                tolerance,
            } => Diagnostic::new(
                FailureCategory::ToleranceViolation,
                "same_range_exceeded",
                message,
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("maxDeviation", *max_deviation)
            .with_detail("tolerance", *tolerance),
        }
    }
}

#[cfg(test)]
mod diagnostic_registry_tests {
    #![allow(clippy::unwrap_used)]

    use brepkit_math::diagnostic::{DetailValue, FailureCategory, ToDiagnostic};

    use super::*;

    #[test]
    fn topology_error_registry_is_pinned() {
        // Stable-code pins: changing any string is a public contract change.
        let mut arena: Arena<vertex::Vertex> = Arena::new();
        let vid = arena.alloc(vertex::Vertex::new(
            brepkit_math::vec::Point3::new(0.0, 0.0, 0.0),
            1e-7,
        ));

        let d = TopologyError::VertexNotFound(vid).diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "entity_not_found");
        assert_eq!(
            d.details()[0],
            ("entity", DetailValue::Text("vertex".into()))
        );

        let d = TopologyError::WireNotClosed.diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidTopology);
        assert_eq!(d.code(), "wire_not_closed");

        let d = TopologyError::NonManifold {
            reason: "shared edge used three times".into(),
        }
        .diagnostic();
        assert_eq!(d.code(), "non_manifold");

        let d = TopologyError::Empty { entity: "wire" }.diagnostic();
        assert_eq!(d.code(), "empty_collection");

        let d = TopologyError::NotPlanar.diagnostic();
        assert_eq!(d.code(), "wire_not_planar");
    }
}
