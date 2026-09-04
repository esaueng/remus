//! Shape fixing — targeted repairs for detected issues.
//!
//! The fix hierarchy mirrors the B-Rep entity tree:
//! `fix_shape` → `fix_solid` → `fix_shell` → `fix_face` → `fix_wire` → `fix_edge`.
//!
//! Each fixer uses analysis results to decide which fixes to apply,
//! controlled by [`FixConfig`] tri-state modes.

pub mod config;
pub mod edge;
pub mod face;
pub mod shell;
pub mod small_face;
pub mod solid;
pub mod split_vertex;
pub mod wire;
pub mod wireframe;

use remus_topology::Topology;
use remus_topology::solid::SolidId;

pub use config::{FixConfig, FixMode};

use crate::HealError;
use crate::context::HealContext;
use crate::status::Status;

/// Machine-readable kind of topology or geometry change made by a fixer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RepairActionKind {
    /// Reordered or reversed a wire's edge uses.
    WireReordered,
    /// Closed a wire endpoint gap by merging vertices.
    WireGapClosed,
    /// Rebuilt a face-use pcurve.
    PcurveRebuilt,
    /// Removed a degenerate edge.
    DegenerateEdgeRemoved,
    /// Removed a small edge.
    SmallEdgeRemoved,
    /// Removed a trailing wire edge.
    WireTailRemoved,
    /// Moved a vertex onto its authoritative curve endpoint.
    VertexMovedToCurve,
    /// Removed a notched/cusp edge.
    NotchedEdgeRemoved,
    /// Corrected the orientation of one face from its boundary winding.
    FaceOrientationFixed,
    /// Reversed one face to make a shell's shared-edge orientation consistent.
    ShellFaceOrientationFixed,
    /// Removed a small face.
    SmallFaceRemoved,
    /// Removed a duplicate face.
    DuplicateFaceRemoved,
    /// Merged coincident vertices.
    CoincidentVertexMerged,
    /// Sewed a free-edge pair.
    FreeEdgePairSewn,
    /// Split an over-connected vertex.
    CommonVertexSplit,
    /// Unified same-domain faces.
    SameDomainFaceUnified,
    /// Unified same-domain edges.
    SameDomainEdgeUnified,
    /// Removed an internal wire.
    InternalWireRemoved,
    /// Converted curve or surface geometry to B-spline representation.
    GeometryConvertedToBspline,
    /// Recognized and replaced NURBS face surfaces with elementary surfaces.
    SurfaceConvertedToElementary,
    /// Recognized and replaced NURBS edge curves with elementary curves.
    CurveConvertedToElementary,
}

impl RepairActionKind {
    /// Stable wire name for structured disclosure.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::WireReordered => "wire_reordered",
            Self::WireGapClosed => "wire_gap_closed",
            Self::PcurveRebuilt => "pcurve_rebuilt",
            Self::DegenerateEdgeRemoved => "degenerate_edge_removed",
            Self::SmallEdgeRemoved => "small_edge_removed",
            Self::WireTailRemoved => "wire_tail_removed",
            Self::VertexMovedToCurve => "vertex_moved_to_curve",
            Self::NotchedEdgeRemoved => "notched_edge_removed",
            Self::FaceOrientationFixed => "face_orientation_fixed",
            Self::ShellFaceOrientationFixed => "shell_face_orientation_fixed",
            Self::SmallFaceRemoved => "small_face_removed",
            Self::DuplicateFaceRemoved => "duplicate_face_removed",
            Self::CoincidentVertexMerged => "coincident_vertex_merged",
            Self::FreeEdgePairSewn => "free_edge_pair_sewn",
            Self::CommonVertexSplit => "common_vertex_split",
            Self::SameDomainFaceUnified => "same_domain_face_unified",
            Self::SameDomainEdgeUnified => "same_domain_edge_unified",
            Self::InternalWireRemoved => "internal_wire_removed",
            Self::GeometryConvertedToBspline => "geometry_converted_to_bspline",
            Self::SurfaceConvertedToElementary => "surface_converted_to_elementary",
            Self::CurveConvertedToElementary => "curve_converted_to_elementary",
        }
    }
}

/// Counted repair disclosure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepairAction {
    /// What changed.
    pub kind: RepairActionKind,
    /// Number of changes of this kind.
    pub count: usize,
}

/// Machine-readable reason a requested repair was declined.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RepairRefusalKind {
    /// A seam has two pcurve uses and projection cannot choose one.
    AmbiguousSeamPcurve,
    /// Moving a vertex to its curve is not implemented by this fixer.
    VertexToleranceRepairUnsupported,
    /// SameParameter repair was requested without a face use.
    SameParameterNeedsFace,
    /// A closure gap exceeded the declared tolerance.
    ClosureGapTooLarge,
    /// Wire self-intersection repair is detection-only.
    SelfIntersectionRepairUnsupported,
    /// Adjacent-edge intersection repair is detection-only.
    IntersectingEdgesRepairUnsupported,
    /// Periodic seam insertion is not implemented.
    MissingSeamRepairUnsupported,
    /// Free edges remained after the bounded sewing attempt.
    FreeEdgesRemain,
}

impl RepairRefusalKind {
    /// Stable wire name for structured disclosure.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::AmbiguousSeamPcurve => "ambiguous_seam_pcurve",
            Self::VertexToleranceRepairUnsupported => "vertex_tolerance_repair_unsupported",
            Self::SameParameterNeedsFace => "same_parameter_needs_face",
            Self::ClosureGapTooLarge => "closure_gap_too_large",
            Self::SelfIntersectionRepairUnsupported => "self_intersection_repair_unsupported",
            Self::IntersectingEdgesRepairUnsupported => "intersecting_edges_repair_unsupported",
            Self::MissingSeamRepairUnsupported => "missing_seam_repair_unsupported",
            Self::FreeEdgesRemain => "free_edges_remain",
        }
    }
}

/// Counted typed refusal disclosure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepairRefusal {
    /// Why the requested repair was declined.
    pub kind: RepairRefusalKind,
    /// Number of affected sites.
    pub count: usize,
}

/// Whether this layer established the validity of a repaired result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixVerification {
    /// The L2 healing crate cannot depend on the sibling check crate, so no
    /// shape-validity verdict was produced. Use the operations-layer verified
    /// wrappers before presenting the result as valid.
    NotPerformed,
}

/// Result of a fix operation.
#[derive(Debug, Clone)]
pub struct FixResult {
    /// Status flags indicating what was done/failed.
    pub status: Status,
    /// Total number of individual repair actions taken.
    pub actions_taken: usize,
    /// Every repair category actually applied.
    pub actions: Vec<RepairAction>,
    /// Every requested repair category that could not be established.
    pub refusals: Vec<RepairRefusal>,
    /// Whether shape validity was established by this layer.
    pub verification: FixVerification,
}

impl FixResult {
    /// Create a result indicating nothing was needed.
    #[must_use]
    pub fn ok() -> Self {
        Self {
            status: Status::OK,
            actions_taken: 0,
            actions: Vec::new(),
            refusals: Vec::new(),
            verification: FixVerification::NotPerformed,
        }
    }

    /// Create a result for one non-empty repair category.
    #[must_use]
    pub fn changed(status: Status, kind: RepairActionKind, count: usize) -> Self {
        if count == 0 {
            return Self::ok();
        }
        Self {
            status,
            actions_taken: count,
            actions: vec![RepairAction { kind, count }],
            refusals: Vec::new(),
            verification: FixVerification::NotPerformed,
        }
    }

    /// Create a result for one typed refusal and no mutation.
    #[must_use]
    pub fn refused(status: Status, kind: RepairRefusalKind, count: usize) -> Self {
        if count == 0 {
            return Self::ok();
        }
        Self {
            status,
            actions_taken: 0,
            actions: Vec::new(),
            refusals: vec![RepairRefusal { kind, count }],
            verification: FixVerification::NotPerformed,
        }
    }

    /// Merge another result into this one.
    pub fn merge(&mut self, other: &Self) {
        self.status = self.status.merge(other.status);
        self.actions_taken += other.actions_taken;
        self.actions.extend_from_slice(&other.actions);
        self.refusals.extend_from_slice(&other.refusals);
    }
}

/// Top-level shape fixer — the main entry point for healing.
///
/// Creates a [`HealContext`], runs the full fix hierarchy
/// (solid → shell → face → wire → edge), applies all recorded
/// changes via [`ReShape`](crate::reshape::ReShape), and returns
/// the (possibly updated) solid ID.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail during healing.
pub fn fix_shape(
    topo: &mut Topology,
    solid_id: SolidId,
    config: &FixConfig,
) -> Result<(SolidId, FixResult), HealError> {
    let mut ctx = HealContext::new();
    let result = solid::fix_solid(topo, solid_id, &mut ctx, config)?;

    let new_solid = ctx.reshape.apply(topo, solid_id)?;

    Ok((new_solid, result))
}

/// Top-level shape fixer with custom tolerance.
///
/// # Errors
///
/// Returns [`HealError`] if entity lookups fail during healing.
pub fn fix_shape_with_tolerance(
    topo: &mut Topology,
    solid_id: SolidId,
    config: &FixConfig,
    tolerance: f64,
) -> Result<(SolidId, FixResult), HealError> {
    let mut ctx = HealContext::with_tolerance(tolerance);
    let result = solid::fix_solid(topo, solid_id, &mut ctx, config)?;

    let new_solid = ctx.reshape.apply(topo, solid_id)?;

    Ok((new_solid, result))
}
