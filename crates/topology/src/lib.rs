//! # remus-topology
//!
//! Boundary representation (B-Rep) topological data structures.
//!
//! This is layer L1, depending only on `remus-math`.
//!
//! # Architecture
//!
//! All topological entities are stored in a central [`Arena`] and referenced
//! via typed index handles ([`VertexId`], [`EdgeId`], etc.). This avoids
//! reference counting overhead and enables efficient traversal.

pub mod adjacency;
pub mod arena;
pub mod attributes;
pub mod builder;
pub mod coedge;
pub mod compound;
pub mod compsolid;
pub mod edge;
pub mod explorer;
pub mod face;
pub mod face_loop;
pub mod journal;
pub mod naming;

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

    /// A color channel is non-finite or outside `[0, 1]`.
    #[error("invalid color channel {channel}: {value} (expected finite in [0, 1])")]
    InvalidColorChannel {
        /// Which channel (`r`, `g`, or `b`).
        channel: &'static str,
        /// The offending value.
        value: f64,
    },

    /// A tolerance raise violates the validated-setter contract (RFC 0004,
    /// Stage 1): the value is non-finite or negative, so no predicate could
    /// compare against it honestly. The stored value is left unchanged.
    #[error("invalid {entity} tolerance {value}: must be finite and non-negative")]
    InvalidToleranceValue {
        /// Which entity kind the rejected raise targeted.
        entity: &'static str,
        /// The rejected value.
        value: f64,
    },

    /// A vertex's tolerance ball fails the ball-containment invariant
    /// (RFC 0004): an incident edge's curve, evaluated at that edge end's
    /// parameter, lies outside the vertex's ball.
    #[error(
        "edge {edge:?} leaves the ball of vertex {vertex:?}: curve endpoint \
         is {deviation} from the vertex point (ball {tolerance})"
    )]
    VertexBallExceeded {
        /// The vertex whose ball is violated.
        vertex: vertex::VertexId,
        /// The incident edge whose end strays outside the ball.
        edge: edge::EdgeId,
        /// Measured distance from the curve's endpoint evaluation to the
        /// vertex point, in model units.
        deviation: f64,
        /// The ball radius that was claimed.
        tolerance: f64,
    },

    /// An edge use's pcurve deviates from the 3D edge curve beyond the
    /// edge's effective tolerance (`EdgeTube`, RFC 0004 invariant 2).
    #[error(
        "pcurve of edge {edge:?} on face {face:?} deviates {max_deviation} \
         from the 3D curve, beyond the effective tube tolerance {tolerance} \
         at parameter {at_parameter}"
    )]
    EdgeTubeExceeded {
        /// The edge whose tube is violated.
        edge: edge::EdgeId,
        /// The face carrying the pcurve.
        face: face::FaceId,
        /// Largest measured 3D↔p-curve deviation (SameParameter and
        /// SameRange both measured; the larger wins), in model units.
        max_deviation: f64,
        /// Witness parameter of the deviation.
        at_parameter: f64,
        /// The effective tube tolerance that was exceeded.
        tolerance: f64,
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
        /// Largest measured deviation or certified upper bound, in model units.
        max_deviation: f64,
        /// Associated measured witness parameter. A certified upper bound need
        /// not be attained at this parameter.
        at_parameter: f64,
        /// The limit that was exceeded.
        tolerance: f64,
    },

    /// One journal entry makes two claims about one entity (RFC 0003): a
    /// resolver must never have to pick between conflicting events, so the
    /// entry is refused whole.
    #[error("journal entry records entity ordinal {ordinal} twice")]
    JournalDuplicateEvent {
        /// The journal-local ordinal recorded twice.
        ordinal: u64,
    },

    /// A persistent reference matches several entities that resolution
    /// will not pick between (RFC 0003).
    #[error("persistent reference is ambiguous over {candidates} candidates: {reason}")]
    RefAmbiguous {
        /// Number of inseparable candidates.
        candidates: usize,
        /// Why they cannot be separated.
        reason: String,
    },

    /// A persistent reference's target was deleted (RFC 0003).
    #[error("persistent reference dangles: target deleted by operation {deleted_at}")]
    RefDangling {
        /// The journal operation that deleted the last live piece.
        deleted_at: u64,
    },

    /// A persistent reference's lineage crosses an operation whose
    /// records cannot carry it (RFC 0003): a barrier, or an in-scope
    /// entry with no claim about the entity. The operation's records are
    /// a declared capability gap.
    #[error(
        "persistent reference cannot resolve across operation {op} \
         ({operation_kind}): its records do not carry this entity"
    )]
    RefUnresolvedAcrossOperation {
        /// The journal operation the lineage could not cross.
        op: u64,
        /// That operation's stable kind name.
        operation_kind: String,
    },

    /// A persistent reference anchors to an operation the journal does
    /// not contain — never journaled, or truncated by a rollback
    /// (RFC 0003).
    #[error("persistent reference anchors to unknown operation {op}")]
    RefUnknownOperation {
        /// The unknown journal operation id.
        op: u64,
    },

    /// A persistent reference's anchor or a discriminator eliminated
    /// every candidate (RFC 0003).
    #[error("persistent reference matches nothing: {reason}")]
    RefNoMatch {
        /// What eliminated the candidates.
        reason: String,
    },

    /// A serialized journal snapshot violates a journal invariant
    /// (RFC 0003, Stage 5): the snapshot is refused whole rather than
    /// installing history a resolver could mis-follow.
    #[error("journal snapshot invalid: {reason}")]
    JournalSnapshotInvalid {
        /// Which invariant the snapshot violates.
        reason: String,
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

impl remus_math::diagnostic::ToDiagnostic for TopologyError {
    fn diagnostic(&self) -> remus_math::diagnostic::Diagnostic {
        use remus_math::diagnostic::{Diagnostic, FailureCategory};
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
            Self::InvalidColorChannel { channel, value } => Diagnostic::new(
                FailureCategory::InvalidInput,
                "invalid_color_channel",
                message,
            )
            .with_detail("channel", *channel)
            .with_detail("value", *value),
            Self::InvalidToleranceValue { entity, value } => {
                let diagnostic = Diagnostic::new(
                    FailureCategory::InvalidInput,
                    "entity_tolerance_invalid",
                    message,
                )
                .with_detail("entity", *entity);
                if value.is_finite() {
                    diagnostic.with_detail("value", *value)
                } else {
                    diagnostic
                }
            }
            Self::VertexBallExceeded {
                vertex,
                edge,
                deviation,
                tolerance,
            } => Diagnostic::new(
                FailureCategory::ToleranceViolation,
                "vertex_ball_violation",
                message,
            )
            .with_detail("vertex", vertex.index())
            .with_detail("edge", edge.index())
            .with_detail("deviation", *deviation)
            .with_detail("tolerance", *tolerance),
            Self::EdgeTubeExceeded {
                edge,
                face,
                max_deviation,
                at_parameter,
                tolerance,
            } => Diagnostic::new(
                FailureCategory::ToleranceViolation,
                "edge_tube_violation",
                message,
            )
            .with_detail("edge", edge.index())
            .with_detail("face", face.index())
            .with_detail("maxDeviation", *max_deviation)
            .with_detail("atParameter", *at_parameter)
            .with_detail("tolerance", *tolerance),
            Self::JournalDuplicateEvent { ordinal } => Diagnostic::new(
                FailureCategory::InvalidInput,
                "journal_duplicate_event",
                message,
            )
            .with_detail("ordinal", usize::try_from(*ordinal).unwrap_or(usize::MAX)),
            Self::RefAmbiguous { candidates, reason } => {
                Diagnostic::new(FailureCategory::InvalidInput, "ref_ambiguous", message)
                    .with_detail("candidates", *candidates)
                    .with_detail("reason", reason.as_str())
            }
            Self::RefDangling { deleted_at } => {
                Diagnostic::new(FailureCategory::InvalidInput, "ref_dangling", message).with_detail(
                    "deletedAt",
                    usize::try_from(*deleted_at).unwrap_or(usize::MAX),
                )
            }
            Self::RefUnresolvedAcrossOperation { op, operation_kind } => Diagnostic::new(
                // The operation's evolution records are a declared
                // capability gap, not a bad input.
                FailureCategory::Unsupported,
                "ref_unresolved_across_operation",
                message,
            )
            .with_detail("op", usize::try_from(*op).unwrap_or(usize::MAX))
            .with_detail("operationKind", operation_kind.as_str()),
            Self::RefUnknownOperation { op } => Diagnostic::new(
                FailureCategory::InvalidInput,
                "ref_unknown_operation",
                message,
            )
            .with_detail("op", usize::try_from(*op).unwrap_or(usize::MAX)),
            Self::RefNoMatch { reason } => {
                Diagnostic::new(FailureCategory::InvalidInput, "ref_no_match", message)
                    .with_detail("reason", reason.as_str())
            }
            Self::JournalSnapshotInvalid { reason } => Diagnostic::new(
                FailureCategory::InvalidInput,
                "journal_snapshot_invalid",
                message,
            )
            .with_detail("reason", reason.as_str()),
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

    use remus_math::diagnostic::{DetailValue, FailureCategory, ToDiagnostic};

    use super::*;

    #[test]
    fn topology_error_registry_is_pinned() {
        // Stable-code pins: changing any string is a public contract change.
        let mut arena: Arena<vertex::Vertex> = Arena::new();
        let vid = arena.alloc(vertex::Vertex::new(
            remus_math::vec::Point3::new(0.0, 0.0, 0.0),
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

        let d = TopologyError::JournalDuplicateEvent { ordinal: 7 }.diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "journal_duplicate_event");
        assert_eq!(d.details()[0], ("ordinal", DetailValue::Int(7)));

        let d = TopologyError::RefAmbiguous {
            candidates: 2,
            reason: "signature matches two faces".into(),
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "ref_ambiguous");

        let d = TopologyError::RefDangling { deleted_at: 3 }.diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "ref_dangling");
        assert_eq!(d.details()[0], ("deletedAt", DetailValue::Int(3)));

        let d = TopologyError::RefUnresolvedAcrossOperation {
            op: 5,
            operation_kind: "offset_solid".into(),
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::Unsupported);
        assert_eq!(d.code(), "ref_unresolved_across_operation");
        assert_eq!(
            d.details()[1],
            ("operationKind", DetailValue::Text("offset_solid".into()))
        );

        let d = TopologyError::RefUnknownOperation { op: 9 }.diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "ref_unknown_operation");

        let d = TopologyError::RefNoMatch {
            reason: "discriminator surface_type:plane eliminated every candidate".into(),
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "ref_no_match");

        let d = TopologyError::JournalSnapshotInvalid {
            reason: "duplicate ordinal in the index".into(),
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidInput);
        assert_eq!(d.code(), "journal_snapshot_invalid");
    }
}
