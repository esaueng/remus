//! Error types for the heal crate.

/// Errors that can occur during shape healing operations.
#[derive(Debug, thiserror::Error)]
pub enum HealError {
    /// A topology lookup or mutation failed.
    #[error(transparent)]
    Topology(#[from] remus_topology::TopologyError),

    /// A math operation failed.
    #[error(transparent)]
    Math(#[from] remus_math::MathError),

    /// A geometry-layer operation failed.
    #[error(transparent)]
    Geometry(#[from] remus_geometry::GeomError),

    /// Analysis detected an unrecoverable problem.
    #[error("analysis failed: {0}")]
    AnalysisFailed(String),

    /// A fix operation could not be applied.
    #[error("fix failed: {0}")]
    FixFailed(String),

    /// A controlled repair could not meet its declared error budget; the
    /// original data was restored.
    #[error("repair achieved deviation {achieved} but the budget is {budget}")]
    RepairBudgetExceeded {
        /// The deviation the repair achieved.
        achieved: f64,
        /// The declared budget it had to meet.
        budget: f64,
    },

    /// An upgrade operation could not be applied.
    #[error("upgrade failed: {0}")]
    UpgradeFailed(String),

    /// Invalid configuration or parameters.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
}

pub(crate) fn analysis_edge_domain(error: remus_topology::edge::EdgeDomainError) -> HealError {
    HealError::AnalysisFailed(error.to_string())
}

pub(crate) fn fix_edge_domain(error: remus_topology::edge::EdgeDomainError) -> HealError {
    HealError::FixFailed(error.to_string())
}

pub(crate) fn upgrade_edge_domain(error: remus_topology::edge::EdgeDomainError) -> HealError {
    HealError::UpgradeFailed(error.to_string())
}
