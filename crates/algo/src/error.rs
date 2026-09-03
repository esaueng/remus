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

    /// A face-surface pair reached the GFA section writer with no
    /// intersection arm for its combination.
    ///
    /// Returning empty sections here would let classification proceed as if
    /// the faces provably do not meet — a silent wrong answer whenever the
    /// pair in fact intersects. Raised as a typed refusal naming both
    /// surface types instead (P-Class issue 2.1, standing rule R1).
    #[error(
        "unsupported surface pair `{a}` × `{b}`: no intersection arm exists \
         for this face-surface combination"
    )]
    UnsupportedSurfacePair {
        /// `FaceSurface::type_tag()` of face A.
        a: &'static str,
        /// `FaceSurface::type_tag()` of face B.
        b: &'static str,
    },

    /// A 3D edge sample could not be projected into a face surface's UV
    /// parameter space while constructing a pcurve.
    ///
    /// Substituting a synthetic UV point here changes the face partition and
    /// can turn an intersecting pair into a plausible but wrong boolean.
    /// Pcurve construction therefore refuses the operation by name.
    #[error("pcurve UV projection failed on `{surface}` surface during `{stage}`")]
    PcurveProjectionFailed {
        /// `FaceSurface::type_tag()` of the target face.
        surface: &'static str,
        /// Stable projection stage identifying the failed boundary.
        stage: &'static str,
    },

    /// The sheet-split arrangement cannot represent the supplied sheet
    /// exactly in its currently qualified subset.
    #[error("unsupported sheet split: {reason}")]
    UnsupportedSheetSplit {
        /// Stable, actionable explanation of the refused configuration.
        reason: String,
    },
}

impl remus_math::diagnostic::ToDiagnostic for AlgoError {
    fn diagnostic(&self) -> remus_math::diagnostic::Diagnostic {
        use remus_math::diagnostic::{Diagnostic, FailureCategory};
        match self {
            // Wrapper variants delegate: one failure, one code, regardless
            // of which layer reports it.
            Self::Topology(inner) => inner.diagnostic(),
            Self::Math(inner) => inner.diagnostic(),
            // Transitional broad codes: these variants carry only prose, so
            // they classify as `internal` until typed context exists
            // (registry rules in `remus_math::diagnostic`).
            Self::IntersectionFailed(_) => Diagnostic::new(
                FailureCategory::Internal,
                "intersection_failed",
                self.to_string(),
            ),
            Self::FaceSplitFailed(_) => Diagnostic::new(
                FailureCategory::Internal,
                "face_split_failed",
                self.to_string(),
            ),
            Self::AssemblyFailed(_) => Diagnostic::new(
                FailureCategory::Internal,
                "assembly_failed",
                self.to_string(),
            ),
            Self::ClassificationFailed(_) => Diagnostic::new(
                FailureCategory::Internal,
                "classification_failed",
                self.to_string(),
            ),
            Self::UnsupportedCurve { variant } => Diagnostic::new(
                FailureCategory::Unsupported,
                "unsupported_edge_curve",
                self.to_string(),
            )
            .with_detail("curveType", *variant),
            Self::UnsupportedSurfacePair { a, b } => Diagnostic::new(
                FailureCategory::Unsupported,
                "unsupported_surface_pair",
                self.to_string(),
            )
            .with_detail("surfaceA", *a)
            .with_detail("surfaceB", *b),
            Self::PcurveProjectionFailed { surface, stage } => Diagnostic::new(
                FailureCategory::Nonconvergence,
                "pcurve_projection_failed",
                self.to_string(),
            )
            .with_detail("surfaceType", *surface)
            .with_detail("stage", *stage),
            Self::UnsupportedSheetSplit { reason } => Diagnostic::new(
                FailureCategory::Unsupported,
                "unsupported_sheet_split",
                self.to_string(),
            )
            .with_detail("reason", reason.as_str()),
        }
    }
}

#[cfg(test)]
mod diagnostic_registry_tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::diagnostic::{FailureCategory, ToDiagnostic};

    use super::*;

    #[test]
    fn algo_error_registry_is_pinned() {
        let d = AlgoError::UnsupportedCurve {
            variant: "hyperbola",
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::Unsupported);
        assert_eq!(d.code(), "unsupported_edge_curve");

        let d = AlgoError::AssemblyFailed("open shell".into()).diagnostic();
        assert_eq!(d.category(), FailureCategory::Internal);
        assert_eq!(d.code(), "assembly_failed");

        let d = AlgoError::IntersectionFailed("invalid section range".into()).diagnostic();
        assert_eq!(d.category(), FailureCategory::Internal);
        assert_eq!(d.code(), "intersection_failed");

        let d = AlgoError::UnsupportedSurfacePair {
            a: "torus",
            b: "cone",
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::Unsupported);
        assert_eq!(d.code(), "unsupported_surface_pair");

        let d = AlgoError::PcurveProjectionFailed {
            surface: "nurbs",
            stage: "curve_sample",
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::Nonconvergence);
        assert_eq!(d.code(), "pcurve_projection_failed");

        let d = AlgoError::UnsupportedSheetSplit {
            reason: "sheet must contain one cylindrical face".into(),
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::Unsupported);
        assert_eq!(d.code(), "unsupported_sheet_split");
    }

    #[test]
    fn wrapped_errors_delegate_to_the_inner_registry() {
        let d = AlgoError::Math(remus_math::MathError::ConvergenceFailure { iterations: 20 })
            .diagnostic();
        assert_eq!(d.category(), FailureCategory::Nonconvergence);
        assert_eq!(d.code(), "newton_nonconvergence");
    }
}
