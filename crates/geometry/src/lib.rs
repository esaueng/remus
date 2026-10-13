//! # remus-geometry
//!
//! Geometry algorithms for remus B-Rep models.
//!
//! This is layer L1, depending only on `remus-math`.
//!
//! Three subsystems:
//! - **sampling** — adaptive and uniform curve/surface sampling
//! - **extrema** — distance and extrema computation between geometry primitives
//! - **convert** — geometry type conversion (e.g. analytic ↔ NURBS)

// Unit tests may use `sort_by`: test binaries never ship, so the per-call-site
// sort copies that `clippy.toml` disallows in production code do not matter there.
#![cfg_attr(test, allow(clippy::disallowed_methods))]

pub mod bounds;
pub mod convert;
pub mod error;
pub mod extrema;
pub mod sampling;

pub use error::GeomError;
