//! # remus-sketch
//!
//! 2D parametric geometric constraint solver for sketch-mode design.
//!
//! Provides a production-grade GCS (Geometric Constraint System) with:
//! - **Entities**: Points, Lines, Circles, Arcs, Ellipses with generational arena handles
//! - **Constraints**: 35 constraint types with analytic Jacobians
//! - **Solver**: DogLeg trust-region (globally convergent)
//! - **DOF analysis**: QR-based rank detection
//! - **Diagnostics**: transactional [`GcsSystem::solve_detailed`], reporting
//!   per-constraint residuals, rank data, and a truthful classification
//!
//! # Example
//! ```
//! use remus_sketch::{GcsSystem, PointData, Constraint};
//!
//! let mut sys = GcsSystem::new();
//! let p0 = sys
//!     .add_point(PointData { x: 0.0, y: 0.0, fixed: true })
//!     .unwrap();
//! let p1 = sys
//!     .add_point(PointData { x: 5.0, y: 1.0, fixed: false })
//!     .unwrap();
//! sys.add_constraint(Constraint::Distance(p0, p1, 3.0)).unwrap();
//! let result = sys.solve(100, 1e-10).unwrap();
//! assert!(result.converged);
//! ```

mod gcs;

pub use gcs::{
    ArcData, ArcId, CircleData, CircleId, Constraint, ConstraintEntry, ConstraintId,
    ConstraintResidual, DofAnalysis, EllipseData, EllipseId, GcsLimits, GcsSystem, LineData,
    LineId, PointData, PointId, SolveClassification, SolveDiagnostics, SolveResult, classify_solve,
};

/// Errors from the sketch constraint solver.
#[derive(Debug, thiserror::Error)]
pub enum SketchError {
    /// Dense solve/analysis would exceed the configured resource budget.
    #[error("GCS resource limit exceeded: {resource} requires {actual}, limit {limit}")]
    ResourceLimitExceeded {
        /// Resource budget's stable name.
        resource: &'static str,
        /// Configured maximum.
        limit: usize,
        /// Checked requirement (`usize::MAX` when arithmetic overflows).
        actual: usize,
    },

    /// A GCS entity handle is invalid or stale (entity was removed).
    #[error("invalid or stale GCS entity handle")]
    InvalidHandle,

    /// Cannot remove a GCS entity that is still referenced by other entities or constraints.
    #[error("GCS entity is still in use by other entities or constraints")]
    EntityInUse,

    /// A constraint's numeric argument is outside its permitted domain
    /// (non-finite, or non-positive where a positive magnitude is required).
    #[error("constraint value is not finite or is outside its permitted range")]
    InvalidValue,
}
