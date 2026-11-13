//! Error types for the offset engine.

use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

/// Errors from solid offset operations.
#[derive(Debug, thiserror::Error)]
pub enum OffsetError {
    /// A topology operation failed.
    #[error("topology error: {0}")]
    Topology(#[from] remus_topology::TopologyError),

    /// A math operation failed.
    #[error("math error: {0}")]
    Math(#[from] remus_math::MathError),

    /// The input parameters are invalid.
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// Description of why the input is invalid.
        reason: String,
    },

    /// A requested move references a face outside the edited solid.
    #[error("face {face:?} is not part of solid {solid:?}")]
    FaceNotInSolid {
        /// Face requested by the caller.
        face: FaceId,
        /// Solid being edited.
        solid: SolidId,
    },

    /// A selected face has no exact move-face construction in this phase.
    #[error("move-face does not support face {face:?} ({surface_type}): {reason}")]
    UnsupportedMoveFace {
        /// Face that cannot be moved or re-intersected: the selected face, or
        /// for [`UnsupportedMoveCause::UnsupportedNeighbor`] the neighbour
        /// across the selection boundary.
        face: FaceId,
        /// Surface type reported by `face`.
        surface_type: &'static str,
        /// Typed cause of the refusal, for callers that branch on it.
        cause: UnsupportedMoveCause,
        /// Exact reason the configuration was refused (human-readable).
        reason: String,
    },

    /// Selected planar faces do not form one rigid coplanar group.
    #[error(
        "move-face group mismatch between reference face {reference:?} and face {face:?}: {reason}"
    )]
    MoveGroupMismatch {
        /// First selected face, used as the group reference.
        reference: FaceId,
        /// Selected face that does not match the group.
        face: FaceId,
        /// Exact coplanarity or orientation mismatch.
        reason: String,
    },

    /// The requested move would alter the source adjacency graph.
    #[error("move-face would change topology at face {face:?}, edge {edge:?}: {reason}")]
    TopologyChange {
        /// Source face nearest the detected change, when applicable.
        face: Option<FaceId>,
        /// Source edge nearest the detected change, when applicable.
        edge: Option<EdgeId>,
        /// Exact invariant that did not survive the move.
        reason: String,
    },

    /// Edge analysis could not determine convexity.
    #[error("analysis failed for edge {edge:?}: {reason}")]
    AnalysisFailed {
        /// The edge that could not be classified.
        edge: EdgeId,
        /// Description of the failure.
        reason: String,
    },

    /// Intersection of two offset faces failed.
    #[error("intersection failed between faces {face_a:?} and {face_b:?}: {reason}")]
    IntersectionFailed {
        /// First face in the intersection pair.
        face_a: FaceId,
        /// Second face in the intersection pair.
        face_b: FaceId,
        /// Description of the failure.
        reason: String,
    },

    /// A self-intersection was detected in the offset shell.
    #[error("self-intersection: {reason}")]
    SelfIntersection {
        /// Description of the self-intersection.
        reason: String,
    },

    /// The offset would carry two shells of the solid through each other, or
    /// a cavity through itself, and no qualified removal exists for it.
    #[error("shell interference: {0}")]
    ShellInterference(crate::interference::ShellInterference),

    /// Final shell assembly failed.
    #[error("assembly failed: {reason}")]
    AssemblyFailed {
        /// Description of the assembly failure.
        reason: String,
    },

    /// The offset distance exceeds the local curvature, collapsing the solid.
    #[error("offset distance collapses the solid")]
    CollapsedSolid,
}

/// Typed cause of an [`OffsetError::UnsupportedMoveFace`] refusal.
///
/// [`code`](Self::code) is the stable machine-readable name of each cause.
/// Codes are explicit literals, never derived from `Display` text or variant
/// names; existing codes are never reassigned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnsupportedMoveCause {
    /// No finer typed cause is recorded for this refusal yet; the
    /// human-readable `reason` describes it.
    Unclassified,
    /// A partial (not full-turn) cylindrical wall facing into the material,
    /// such as a blind bore segment: partial-wall resize supports outward
    /// quarter walls with radial planar sides only.
    PartialReversedWall,
    /// The selected face borders a face whose surface type has no exact
    /// move-face re-intersection; the error's `face` and `surface_type` name
    /// that neighbour.
    UnsupportedNeighbor {
        /// Selected face whose move boundary meets the neighbour.
        selected: FaceId,
        /// Surface type of the selected face.
        selected_surface_type: &'static str,
        /// Shared boundary edge between the selected face and the neighbour.
        edge: EdgeId,
    },
}

impl UnsupportedMoveCause {
    /// Stable lowercase snake-case code of this cause.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::Unclassified => "unclassified",
            Self::PartialReversedWall => "partial_reversed_wall",
            Self::UnsupportedNeighbor { .. } => "unsupported_neighbor",
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn unsupported_move_cause_codes_are_pinned() {
        // Stable-code pins: changing any of these strings is a breaking
        // change to the WASM `details.reason` contract.
        assert_eq!(UnsupportedMoveCause::Unclassified.code(), "unclassified");
        assert_eq!(
            UnsupportedMoveCause::PartialReversedWall.code(),
            "partial_reversed_wall"
        );
        let mut topo = remus_topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[0];
        let edge = topo.edge_id_from_index(0).unwrap();
        let cause = UnsupportedMoveCause::UnsupportedNeighbor {
            selected: face,
            selected_surface_type: "cylinder",
            edge,
        };
        assert_eq!(cause.code(), "unsupported_neighbor");
    }
}
