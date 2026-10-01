//! Directional projection of edges onto faces, solids, and sketch planes
//! (P-Class 7.4).
//!
//! **Contract stub.** The public surface below is fixed by
//! `docs/design/p74-curve-projection.md`; every entry point currently
//! returns [`OperationsError::Unsupported`] (wrapped in
//! [`ProjectCurveError::Operations`]) without touching the topology. The
//! acceptance oracles live in `crates/operations/tests/qualify_project_curve.rs`.

use remus_math::frame::Frame3;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;

use crate::OperationsError;

/// Default cap on control points of an approximate (fitted) projection.
pub const DEFAULT_MAX_CONTROL_POINTS: usize = 512;

/// Caller options for [`project_curve_onto_face`] and
/// [`project_curves_onto_solid`].
#[derive(Debug, Clone)]
pub struct ProjectCurveOptions {
    /// Accept a fitted NURBS image where no exact construction exists
    /// (arc onto a non-coaxial curved quadric). `false` refuses with
    /// [`ProjectCurveError::ApproximationRequired`].
    pub allow_approximate: bool,
    /// Absolute deviation bound for approximate images. `None` means
    /// `1e-6 · scale`. Values below `1e-9 · scale`, non-finite, or
    /// non-positive values are refused as
    /// [`ProjectCurveError::InvalidOptions`].
    pub approximation_tolerance: Option<f64>,
    /// Control-point budget per approximate edge (minimum 4).
    pub max_control_points: usize,
    /// Frame for 2D output on planar targets. Its `z` must be parallel to
    /// the target plane normal and its origin must lie on that plane.
    pub plane_frame: Option<Frame3>,
}

impl Default for ProjectCurveOptions {
    fn default() -> Self {
        Self {
            allow_approximate: false,
            approximation_tolerance: None,
            max_control_points: DEFAULT_MAX_CONTROL_POINTS,
            plane_frame: None,
        }
    }
}

/// Geometric quality of a projection result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProjectionQuality {
    /// Every edge carries the closed-form image.
    Exact,
    /// At least one edge is a fitted NURBS.
    Approximate {
        /// Sampled maximum of the deviation metric defined in the design
        /// note. Not a certified bound.
        max_deviation: f64,
    },
}

/// One projected edge.
#[derive(Debug, Clone)]
pub struct ProjectedEdge {
    /// New free edge (owned by the caller; not in any wire or face).
    pub edge: EdgeId,
    /// Face the edge lies on.
    pub face: FaceId,
    /// Increasing sub-interval of the source edge's domain that maps onto
    /// this edge. The edge's start vertex is the image of `source_range.0`.
    pub source_range: (f64, f64),
    /// The edge in `plane_frame` coordinates, when a frame was supplied
    /// for a planar target.
    pub plane_curve: Option<PCurve>,
}

/// Result of projecting one source edge onto one face.
#[derive(Debug, Clone)]
pub struct ProjectedCurves {
    /// Projected edges in source order.
    pub edges: Vec<ProjectedEdge>,
    /// Target face.
    pub face: FaceId,
    /// Exact or disclosed-approximate.
    pub quality: ProjectionQuality,
    /// Some part of the source domain has no image on the face.
    pub clipped: bool,
}

/// Projection of one source edge onto a solid.
#[derive(Debug, Clone)]
pub struct SourceProjection {
    /// The source edge.
    pub source: EdgeId,
    /// Projected edges in source order, possibly on several faces. Empty
    /// when the source misses the solid entirely.
    pub edges: Vec<ProjectedEdge>,
    /// Some part of the source domain has no image on the solid.
    pub clipped: bool,
}

/// Result of [`project_curves_onto_solid`].
#[derive(Debug, Clone)]
pub struct SolidProjection {
    /// One entry per source edge, in input order.
    pub sources: Vec<SourceProjection>,
    /// Worst quality over all edges.
    pub quality: ProjectionQuality,
}

/// Typed refusal from curve projection. Codes from [`Self::code`] are API.
#[derive(Debug, thiserror::Error)]
pub enum ProjectCurveError {
    /// The direction is zero-length or non-finite.
    #[error("projection direction must be finite and non-zero")]
    InvalidDirection,
    /// An option value is out of range.
    #[error("invalid projection options: {reason}")]
    InvalidOptions {
        /// What is wrong.
        reason: String,
    },
    /// The source edge has zero length or invalid geometry.
    #[error("degenerate source edge: {reason}")]
    DegenerateSource {
        /// What is degenerate.
        reason: &'static str,
    },
    /// The source curve type has no projection cell for this target.
    #[error("unsupported source curve {curve} for target {surface}")]
    UnsupportedSourceCurve {
        /// Source curve type tag.
        curve: &'static str,
        /// Target surface type tag (`"plane"` for sketch planes).
        surface: &'static str,
    },
    /// The target face's surface type is not projectable in this slice.
    #[error("unsupported target surface {surface}")]
    UnsupportedTargetSurface {
        /// Target surface type tag.
        surface: &'static str,
    },
    /// A segment parallel to the direction projects to a point.
    #[error("source segment is parallel to the projection direction")]
    SourceParallelToDirection,
    /// The image collapses or folds (a conic seen edge-on, a NURBS image
    /// with a stationary point).
    #[error("projected image is degenerate")]
    DegenerateImage,
    /// The direction is parallel to the target plane or to a cylinder axis.
    #[error("projection direction grazes the target surface")]
    GrazingDirection,
    /// The swept carrier is tangent to the target surface.
    #[error("swept carrier is tangent to the target surface")]
    TangentSection,
    /// The sweep plane passes through the cone apex.
    #[error("sweep plane passes through the cone apex")]
    SectionThroughApex,
    /// The cone section lies in the unresolved band around the parabola.
    #[error("cone section is near-parabolic (A = {a_coefficient:e})")]
    NearParabolicSection {
        /// Conic classifier `A = p² − sin²(half_angle)`.
        a_coefficient: f64,
    },
    /// No exact image exists and approximation was not allowed.
    #[error("no exact projection exists; approximation was not allowed")]
    ApproximationRequired,
    /// The fitted image cannot meet the tolerance within the budget.
    #[error("approximation tolerance {requested:e} unattainable (achieved {achieved:e})")]
    ToleranceUnattainable {
        /// Requested absolute tolerance.
        requested: f64,
        /// Best deviation reached within the control-point budget.
        achieved: f64,
    },
    /// An approximate image would need clipping against the face boundary.
    #[error("approximate projections cannot be clipped in this slice")]
    ApproximateClipUnsupported,
    /// The image runs along a face boundary edge over a positive length.
    #[error("projected image runs along a face boundary")]
    ImageAlongBoundary,
    /// No part of the source has an image on the target.
    #[error("projection is empty")]
    EmptyProjection,
    /// The supplied plane frame does not lie in the planar target.
    #[error("plane frame does not match the target plane")]
    PlaneFrameMismatch,
    /// The constructed image failed the on-face residual post-check.
    #[error("projection residual {measured:e} exceeds bound {bound:e}")]
    ResidualExceeded {
        /// Largest sampled residual.
        measured: f64,
        /// Allowed residual.
        bound: f64,
    },
    /// One source of a multi-source call was refused; nothing was created.
    #[error("source {index} refused: {error}")]
    SourceRefused {
        /// Index into the caller's source slice.
        index: usize,
        /// The refusal for that source.
        #[source]
        error: Box<Self>,
    },
    /// Lookup or kernel failure (stale handles, invalid topology).
    #[error(transparent)]
    Operations(#[from] OperationsError),
}

impl ProjectCurveError {
    /// Stable machine-readable refusal code.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::InvalidDirection => "invalid-direction",
            Self::InvalidOptions { .. } => "invalid-options",
            Self::DegenerateSource { .. } => "degenerate-source",
            Self::UnsupportedSourceCurve { .. } => "unsupported-source-curve",
            Self::UnsupportedTargetSurface { .. } => "unsupported-target-surface",
            Self::SourceParallelToDirection => "source-parallel-to-direction",
            Self::DegenerateImage => "degenerate-image",
            Self::GrazingDirection => "grazing-direction",
            Self::TangentSection => "tangent-section",
            Self::SectionThroughApex => "section-through-apex",
            Self::NearParabolicSection { .. } => "near-parabolic-section",
            Self::ApproximationRequired => "approximation-required",
            Self::ToleranceUnattainable { .. } => "tolerance-unattainable",
            Self::ApproximateClipUnsupported => "approximate-clip-unsupported",
            Self::ImageAlongBoundary => "image-along-boundary",
            Self::EmptyProjection => "empty-projection",
            Self::PlaneFrameMismatch => "plane-frame-mismatch",
            Self::ResidualExceeded { .. } => "residual-exceeded",
            Self::SourceRefused { .. } => "source-refused",
            Self::Operations(_) => "operations",
        }
    }
}

fn pending() -> ProjectCurveError {
    ProjectCurveError::Operations(OperationsError::Unsupported {
        operation: "project_curve",
        reason: "P-Class 7.4 curve projection is specified but not implemented".into(),
    })
}

/// Project `source` along `direction` onto the trimmed region of `face`.
///
/// Creates free edges only; records no journal entry; leaves the source
/// edge and the face unchanged. A refusal leaves the topology unchanged.
///
/// # Errors
///
/// Every refusal listed in the design note, as a typed
/// [`ProjectCurveError`]. The stub always returns
/// [`ProjectCurveError::Operations`] wrapping [`OperationsError::Unsupported`].
pub fn project_curve_onto_face(
    topo: &mut Topology,
    source: EdgeId,
    direction: Vec3,
    face: FaceId,
    options: &ProjectCurveOptions,
) -> Result<ProjectedCurves, ProjectCurveError> {
    let _ = (source, direction, face, options);
    remus_topology::transaction::run_append_only(topo, |_| Err(pending())).map(|(value, _)| value)
}

/// Project every source along `direction` onto the first-hit faces of
/// `solid`. Atomic: any per-source refusal is returned as
/// [`ProjectCurveError::SourceRefused`] and nothing is created.
///
/// # Errors
///
/// See [`project_curve_onto_face`].
pub fn project_curves_onto_solid(
    topo: &mut Topology,
    sources: &[EdgeId],
    direction: Vec3,
    solid: SolidId,
    options: &ProjectCurveOptions,
) -> Result<SolidProjection, ProjectCurveError> {
    let _ = (sources, direction, solid, options);
    remus_topology::transaction::run_append_only(topo, |_| Err(pending())).map(|(value, _)| value)
}

/// Read-only projection of every source along `direction` onto the
/// unbounded plane of `frame`, returned as 2D curves in frame coordinates
/// (one per source, input order).
///
/// # Errors
///
/// See [`project_curve_onto_face`].
pub fn project_curves_onto_plane(
    topo: &Topology,
    sources: &[EdgeId],
    direction: Vec3,
    frame: &Frame3,
) -> Result<Vec<PCurve>, ProjectCurveError> {
    let _ = (topo, sources, direction, frame);
    Err(pending())
}
