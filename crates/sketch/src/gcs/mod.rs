//! Geometric Constraint Solver (GCS) for 2D sketch parametric design.

mod components;
mod constraint;
mod diagnostics;
mod dof;
mod entity;
mod final_eval;
mod qr;
mod solver;
mod sparse;
mod system;

pub use constraint::{Constraint, ConstraintEntry, ConstraintId};
pub use diagnostics::{
    ConstraintResidual, SolveClassification, SolveDiagnostics, classify as classify_solve,
};
pub use dof::DofAnalysis;
pub use entity::{
    ArcData, ArcId, CircleData, CircleId, EllipseData, EllipseId, LineData, LineId, PointData,
    PointId,
};
pub use solver::SolveResult;
pub use system::{GcsLimits, GcsSystem};
