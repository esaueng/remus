//! Walking-based fillet and chamfer engine.
//!
//! This crate implements blend surface computation using a
//! Newton-Raphson walking algorithm. It produces G1-continuous fillet
//! and chamfer surfaces for all combinations of analytic and NURBS faces.

#[allow(dead_code)]
pub(crate) mod adaptive_tolerance;
pub(crate) mod analytic;
pub(crate) mod blend_func;
pub(crate) mod builder_utils;
pub mod chamfer_builder;
pub(crate) mod corner;
pub mod fillet_builder;
pub mod g1_chain;
pub mod query;
pub mod radius_law;
pub(crate) mod section;
pub(crate) mod spherical_triangle;
pub(crate) mod spine;
pub(crate) mod stripe;
pub(crate) mod trimmer;
pub(crate) mod walker;

use brepkit_topology::edge::EdgeId;
use brepkit_topology::face::FaceId;
use brepkit_topology::solid::SolidId;
use brepkit_topology::vertex::VertexId;

/// Error type for blend operations.
#[derive(Debug, thiserror::Error)]
pub enum BlendError {
    /// A caller supplied malformed or geometrically inconsistent input.
    #[error("invalid blend input: {reason}")]
    InvalidInput {
        /// Why the input cannot define the requested blend.
        reason: String,
    },

    /// No initial solution found at the spine start.
    #[error("no start solution at edge {edge:?}, t={t}")]
    StartSolutionFailure {
        /// The edge where the start solution failed.
        edge: EdgeId,
        /// The parameter value at the failure point.
        t: f64,
    },

    /// Walker diverged during marching.
    #[error("walking failure at edge {edge:?}, t={t}, residual={residual}")]
    WalkingFailure {
        /// The edge where walking failed.
        edge: EdgeId,
        /// The parameter value at the failure point.
        t: f64,
        /// The residual norm at failure.
        residual: f64,
    },

    /// Generated surface is twisted or self-intersecting.
    #[error("twisted surface on stripe {stripe_idx}")]
    TwistedSurface {
        /// Index of the stripe that is twisted.
        stripe_idx: usize,
    },

    /// Radius too large for the edge geometry.
    #[error("radius too large for edge {edge:?}: max={max_radius}")]
    RadiusTooLarge {
        /// The edge for which the radius is too large.
        edge: EdgeId,
        /// The maximum allowable radius.
        max_radius: f64,
    },

    /// Face trimming failed.
    #[error("trimming failure on face {face:?}")]
    TrimmingFailure {
        /// The face where trimming failed.
        face: FaceId,
    },

    /// Corner solver failed at vertex.
    #[error("corner failure at vertex {vertex:?}")]
    CornerFailure {
        /// The vertex where the corner solver failed.
        vertex: VertexId,
    },

    /// Multiple blend stripes meet at a vertex, which the walking engine's
    /// watertight assembly does not support yet: the corner solver computes
    /// exact vertex-blend geometry, but the stripes are not set back and the
    /// corner faces do not share boundary edges with them, so the assembled
    /// shell can never close. Callers fall back to another engine.
    #[error("unsupported vertex blend at {vertex:?}: {stripes} stripes meet")]
    UnsupportedVertexBlend {
        /// The vertex where multiple stripes meet.
        vertex: VertexId,
        /// How many stripes meet there.
        stripes: usize,
    },

    /// Some of the edges the caller named were never blended.
    ///
    /// A blend must round every edge it was asked to round, or say which ones
    /// it could not. Quietly returning the subset it managed is the worst
    /// outcome available: the caller gets a fresh, valid, watertight handle
    /// whose volume sits inside the plausible envelope and has no way to tell
    /// that a feature it asked for is simply missing. Engines — and
    /// dispatchers that retry on a reduced selection — raise this instead.
    #[error("{} of the edges named were not blended ({edges:?}): {reason}", edges.len())]
    EdgesNotBlended {
        /// The named edges that carry no blend in the result.
        edges: Vec<EdgeId>,
        /// Why they could not be blended.
        reason: String,
    },

    /// Surface type not supported.
    #[error("unsupported surface on face {face:?}: {surface_tag}")]
    UnsupportedSurface {
        /// The face with the unsupported surface.
        face: FaceId,
        /// A description of the unsupported surface type.
        surface_tag: String,
    },

    /// Topology error from underlying operations.
    #[error(transparent)]
    Topology(#[from] brepkit_topology::TopologyError),

    /// Math error from underlying computations.
    #[error(transparent)]
    Math(#[from] brepkit_math::MathError),
}

/// Exact face provenance, recorded by the builder while it assembled the
/// result rather than inferred from the result's geometry afterwards.
///
/// The builders already hold everything needed: which faces the blend touched,
/// which trimmed face replaced each of them, and which two base faces every
/// blend face was built between. Reporting it costs nothing and removes the
/// guesswork from a persistent face reference.
#[derive(Debug, Clone, Default)]
pub struct BlendFaceOrigins {
    /// Input face -> the output face that carries it. An untouched face maps to
    /// itself; a trimmed one maps to its replacement.
    pub survived: Vec<(FaceId, FaceId)>,
    /// Input faces the builder proved do not survive in the result.
    pub deleted: Vec<FaceId>,
    /// A face that did not exist in the input -> the input faces it was built
    /// between (the two base faces of the stripe, for a blend band).
    pub created: Vec<(FaceId, Vec<FaceId>)>,
    /// Output faces the builder created without being able to name a base face
    /// for them. Reported so a consumer sees the face exists and has no origin,
    /// rather than not seeing it at all.
    pub created_unattributed: Vec<FaceId>,
}

/// Result of a blend operation.
pub struct BlendResult {
    /// The resulting solid.
    pub solid: SolidId,
    /// Edges that were successfully blended.
    pub succeeded: Vec<EdgeId>,
    /// Edges that failed with diagnostic info.
    pub failed: Vec<(EdgeId, BlendError)>,
    /// Whether this is a partial result (some edges failed).
    pub is_partial: bool,
    /// Construction-derived face provenance, when the engine that ran could
    /// report it. `None` means the caller must fall back to matching geometry —
    /// and must say so to its own consumer.
    pub face_origins: Option<BlendFaceOrigins>,
}
