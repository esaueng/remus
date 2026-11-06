//! Types carried inside [`OperationsError`](crate::OperationsError) variants.
//!
//! The error enum sits below every operation, so the types its variants embed
//! must too (RFC 0007 stage 0). Each is re-exported at its original path:
//! `heal::HealingReport`, `resize_blend::ResizeBlendError`,
//! `blend_cluster::BlendClusterError`.

/// Machine-readable kind of repair performed by [`heal_solid`](crate::heal::heal_solid).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealingChangeKind {
    /// A wire endpoint gap was closed.
    WireGapClosed,
    /// Coincident vertices were merged.
    VertexMerged,
    /// A degenerate edge was removed.
    DegenerateEdgeRemoved,
    /// A small face was removed.
    SmallFaceRemoved,
    /// A duplicate face was removed.
    DuplicateFaceRemoved,
    /// A face orientation was corrected.
    FaceOrientationFixed,
}

impl HealingChangeKind {
    /// Stable wire name used by structured bindings.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::WireGapClosed => "wire_gap_closed",
            Self::VertexMerged => "vertex_merged",
            Self::DegenerateEdgeRemoved => "degenerate_edge_removed",
            Self::SmallFaceRemoved => "small_face_removed",
            Self::DuplicateFaceRemoved => "duplicate_face_removed",
            Self::FaceOrientationFixed => "face_orientation_fixed",
        }
    }
}

/// One non-empty repair category from a [`HealingReport`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HealingChange {
    /// What repair was performed.
    pub kind: HealingChangeKind,
    /// Number of repairs of this kind.
    pub count: usize,
}

/// Summary of repairs performed by [`heal_solid`](crate::heal::heal_solid).
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct HealingReport {
    /// Number of coincident vertices merged.
    pub vertices_merged: usize,
    /// Number of degenerate edges removed.
    pub degenerate_edges_removed: usize,
    /// Number of face orientations fixed.
    pub orientations_fixed: usize,
    /// Number of wire gaps closed.
    pub wire_gaps_closed: usize,
    /// Number of small faces removed.
    pub small_faces_removed: usize,
    /// Number of duplicate faces removed.
    pub duplicate_faces_removed: usize,
}

impl HealingReport {
    /// Every repair category that changed topology, excluding zero-count no-ops.
    #[must_use]
    pub fn changes(&self) -> Vec<HealingChange> {
        [
            (HealingChangeKind::WireGapClosed, self.wire_gaps_closed),
            (HealingChangeKind::VertexMerged, self.vertices_merged),
            (
                HealingChangeKind::DegenerateEdgeRemoved,
                self.degenerate_edges_removed,
            ),
            (
                HealingChangeKind::SmallFaceRemoved,
                self.small_faces_removed,
            ),
            (
                HealingChangeKind::DuplicateFaceRemoved,
                self.duplicate_faces_removed,
            ),
            (
                HealingChangeKind::FaceOrientationFixed,
                self.orientations_fixed,
            ),
        ]
        .into_iter()
        .filter_map(|(kind, count)| (count > 0).then_some(HealingChange { kind, count }))
        .collect()
    }

    /// Total number of disclosed repairs.
    #[must_use]
    pub const fn total(&self) -> usize {
        self.vertices_merged
            + self.degenerate_edges_removed
            + self.orientations_fixed
            + self.wire_gaps_closed
            + self.small_faces_removed
            + self.duplicate_faces_removed
    }
}

/// Stable, machine-readable refusal from [`resize_blend`](crate::resize_blend::resize_blend).
#[derive(Debug, thiserror::Error)]
pub enum ResizeBlendError {
    /// Caller arguments or handles cannot name a resize request.
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// Refusal detail.
        reason: String,
    },
    /// The selected face is not a supported analytic blend surface.
    #[error("selected face is not an analytic torus/cylinder/sphere blend: {surface}")]
    BandNotAnalytic {
        /// Selected surface type.
        surface: &'static str,
    },
    /// Caller radius does not match the exact band radius.
    #[error("expected radius {expected} mm, but the exact band radius is {actual} mm")]
    RadiusMismatch {
        /// Caller-provided witness.
        expected: f64,
        /// Exact topology measurement.
        actual: f64,
    },
    /// The band closes into or is supported by freeform geometry.
    #[error("blend band touches a freeform face")]
    BandTouchesFreeform,
    /// The support surfaces cannot be reconstructed exactly in this phase.
    #[error("unsupported analytic support pair: {first} x {second}")]
    UnsupportedSupportPair {
        /// First support surface type.
        first: &'static str,
        /// Second support surface type.
        second: &'static str,
    },
    /// The requested radius cannot fit on the recovered sharp feature.
    #[error("radius {radius} mm does not fit on the recovered sharp feature")]
    RadiusTooLarge {
        /// Requested radius.
        radius: f64,
    },
    /// Exact topology analysis or reconstruction could not produce one answer.
    #[error("exact reconstruction refused: {reason}")]
    ReconstructionFailed {
        /// Refusal detail.
        reason: String,
    },
}

impl ResizeBlendError {
    /// Stable code used across the WASM boundary.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::InvalidInput { .. } => "invalid-input",
            Self::BandNotAnalytic { .. } => "blend-band-not-analytic",
            Self::RadiusMismatch { .. } => "blend-radius-mismatch",
            Self::BandTouchesFreeform => "band-touches-freeform",
            Self::UnsupportedSupportPair { .. } => "unsupported-support-pair",
            Self::RadiusTooLarge { .. } => "radius-too-large",
            Self::ReconstructionFailed { .. } => "resize-blend-failed",
        }
    }
}

/// Stable, machine-readable refusal from
/// [`recognize_blend_cluster`](crate::blend_cluster::recognize_blend_cluster).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BlendClusterError {
    /// The seed does not name a face of the solid.
    #[error("invalid blend-cluster seed: {reason}")]
    InvalidSeed {
        /// Refusal detail.
        reason: String,
    },
    /// The seed is neither a blend nor a face a recognized blend rolls on.
    #[error("face {seed} supports no blend and is not one")]
    NoDependentBlends {
        /// Arena index of the seed face.
        seed: usize,
    },
    /// A face the dependent group reaches is blend-like but not proven to be
    /// a rolling-ball blend, so the group cannot be closed.
    #[error("face {face} touches the blend group but is not a recognized blend: {reason}")]
    UnrecognizedMember {
        /// Arena index of the unrecognized face.
        face: usize,
        /// Measured refusal detail.
        reason: String,
    },
    /// A recognized member's sharp edge or vertex has no closed form here.
    #[error("blend member face {face} has no closed-form sharp feature: {reason}")]
    NoSharpForm {
        /// Arena index of one face of the member.
        face: usize,
        /// Refusal detail.
        reason: String,
    },
}

impl BlendClusterError {
    /// Stable code used across the WASM boundary.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::InvalidSeed { .. } => "blend-cluster-invalid-seed",
            Self::NoDependentBlends { .. } => "blend-cluster-no-blends",
            Self::UnrecognizedMember { .. } => "blend-cluster-unrecognized-member",
            Self::NoSharpForm { .. } => "blend-cluster-no-sharp-form",
        }
    }
}
