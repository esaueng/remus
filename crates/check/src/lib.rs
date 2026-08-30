//! # remus-check
//!
//! Topology algorithms for remus B-Rep models.
//!
//! This is layer L2, depending on `remus-math`, `remus-topology`,
//! and `remus-geometry`.
//!
//! Five subsystems:
//! - **classify** — point-in-solid classification (ray casting + winding numbers)
//! - **validate** — hierarchical shape validation (geometric + topological checks)
//! - **properties** — geometric properties (volume, area, CoM, inertia tensor)
//! - **distance** — minimum distance and extrema between shapes
//! - **analyze** — surface interrogation (curvature reports, minimum radius)

pub mod analyze;
pub mod classify;
pub mod distance;
pub mod error;
pub mod properties;
pub mod util;
pub mod validate;

pub use error::CheckError;
