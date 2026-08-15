//! Error types for the algo crate.

/// Errors from GFA algorithm operations.
#[derive(Debug, thiserror::Error)]
pub enum AlgoError {
    /// A topology entity was not found in the arena.
    #[error("topology error: {0}")]
    Topology(#[from] remus_topology::TopologyError),

    /// A math operation failed.
    #[error("math error: {0}")]
    Math(#[from] remus_math::MathError),

    /// Intersection computation failed.
    #[error("intersection failed: {0}")]
    IntersectionFailed(String),

    /// Face splitting produced invalid topology.
    #[error("face splitting failed: {0}")]
    FaceSplitFailed(String),

    /// Shell assembly produced non-manifold result.
    #[error("assembly failed: {0}")]
    AssemblyFailed(String),

    /// Classification could not determine inside/outside state.
    #[error("classification failed: {0}")]
    ClassificationFailed(String),

    /// An input carries a curve type the GFA pipeline cannot intersect,
    /// split, or classify yet.
    ///
    /// Raised up front by [`crate::gfa::boolean_with_tolerance`] rather
    /// than deep inside the pipeline, so the operation refuses by name
    /// instead of falling back to a chord or a line and returning a
    /// plausible but wrong solid. `variant` is the
    /// [`EdgeCurve`](remus_topology::edge::EdgeCurve) type tag, e.g.
    /// `"hyperbola"`.
    #[error(
        "unsupported edge curve type `{variant}`: the boolean engine cannot \
         intersect or split this curve yet"
    )]
    UnsupportedCurve {
        /// The `EdgeCurve::type_tag()` of the offending curve.
        variant: &'static str,
    },
}
