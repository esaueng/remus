//! Error types for the check crate.

/// Errors from topology algorithm operations.
#[derive(Debug, thiserror::Error)]
pub enum CheckError {
    /// A referenced topology entity was not found.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),

    /// A math error occurred.
    #[error(transparent)]
    Math(#[from] remus_math::MathError),

    /// Classification could not determine a result.
    #[error("classification failed: {0}")]
    ClassificationFailed(String),

    /// A validation check encountered an internal error.
    #[error("validation error: {0}")]
    ValidationFailed(String),

    /// Numerical integration did not converge.
    #[error("integration did not converge: {0}")]
    IntegrationFailed(String),

    /// Distance computation could not find a result.
    #[error("distance computation failed: {0}")]
    DistanceFailed(String),

    /// Curvature analysis failed: the surface is degenerate at the queried
    /// parameters (apex, pole, spindle parallel) or no valid sample could be
    /// evaluated.
    #[error("curvature analysis failed: {0}")]
    CurvatureFailed(String),
}

pub(crate) fn edge_domain_validation(error: remus_topology::edge::EdgeDomainError) -> CheckError {
    CheckError::ValidationFailed(error.to_string())
}
