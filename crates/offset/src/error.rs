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
        /// Selected face that cannot be moved.
        face: FaceId,
        /// Surface type reported by the face.
        surface_type: &'static str,
        /// Exact reason the configuration was refused.
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
