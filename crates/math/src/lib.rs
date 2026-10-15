//! # remus-math
//!
//! Vector math, matrix transforms, NURBS geometry, and exact geometric
//! predicates for the remus CAD kernel.
//!
//! This is the foundation layer (L0) with no workspace dependencies.

// Unit tests may use `sort_by`: test binaries never ship, so the per-call-site
// sort copies that `clippy.toml` disallows in production code do not matter there.
#![cfg_attr(test, allow(clippy::disallowed_methods))]

/// Errors from math operations.
#[derive(Debug, thiserror::Error)]
pub enum MathError {
    /// NURBS degree is unsupported: zero, or not smaller than the number of
    /// control points.
    #[error(
        "invalid NURBS degree {degree} with {control_points} control points: degree must be at least 1 and less than the control point count"
    )]
    InvalidDegree {
        /// Requested polynomial degree.
        degree: usize,
        /// Number of supplied control points.
        control_points: usize,
    },

    /// Knot vector length does not match control points and degree.
    #[error("invalid knot vector: expected {expected} knots, got {got}")]
    InvalidKnotVector {
        /// Expected number of knots.
        expected: usize,
        /// Actual number of knots.
        got: usize,
    },

    /// A NURBS knot is non-finite or occurs before its predecessor.
    #[error(
        "invalid NURBS knot at index {index}: expected a finite value not less than the previous knot, got {value}"
    )]
    InvalidKnotValue {
        /// Index of the invalid knot.
        index: usize,
        /// Invalid knot value.
        value: f64,
    },

    /// Weights vector length does not match control points.
    #[error("invalid weights: expected {expected} weights, got {got}")]
    InvalidWeights {
        /// Expected number of weights.
        expected: usize,
        /// Actual number of weights.
        got: usize,
    },

    /// A NURBS weight is non-finite or non-positive.
    #[error("invalid NURBS weight at index {index}: expected a finite positive value, got {value}")]
    InvalidWeightValue {
        /// Flattened index of the invalid weight.
        index: usize,
        /// Invalid weight value.
        value: f64,
    },

    /// A NURBS control point has a non-finite coordinate.
    #[error(
        "invalid NURBS control point at index {index}: expected finite coordinates, got ({x}, {y}, {z})"
    )]
    InvalidControlPointValue {
        /// Flattened index of the invalid control point.
        index: usize,
        /// X coordinate as supplied.
        x: f64,
        /// Y coordinate as supplied.
        y: f64,
        /// Z coordinate as supplied.
        z: f64,
    },

    /// Control point grid dimensions are inconsistent.
    #[error(
        "invalid control point grid: expected {expected_rows}x{expected_cols}, got inconsistent dimensions"
    )]
    InvalidControlPointGrid {
        /// Expected number of rows.
        expected_rows: usize,
        /// Expected number of columns.
        expected_cols: usize,
    },

    /// Cannot normalize a zero-length vector.
    #[error("cannot normalize zero vector")]
    ZeroVector,

    /// Matrix is singular and cannot be inverted.
    #[error("singular matrix cannot be inverted")]
    SingularMatrix,

    /// Input collection is empty where at least one element is required.
    #[error("empty input where at least one element is required")]
    EmptyInput,

    /// Parameter is outside the valid range.
    #[error("parameter {value} out of range [{min}, {max}]")]
    ParameterOutOfRange {
        /// The out-of-range value.
        value: f64,
        /// Lower bound of the valid range.
        min: f64,
        /// Upper bound of the valid range.
        max: f64,
    },

    /// Newton iteration did not converge within the allowed iterations.
    #[error("Newton iteration did not converge after {iterations} iterations")]
    ConvergenceFailure {
        /// Number of iterations attempted.
        iterations: usize,
    },

    /// The caller requested cooperative cancellation at a safe checkpoint.
    #[error("operation cancelled")]
    Cancelled,

    /// A fit point coincides with its predecessor, so their chord-length
    /// parameters do not increase and interpolation through both is
    /// singular.
    #[error(
        "fit point {index} coincides with its predecessor: chord-length parameters must increase"
    )]
    CoincidentFitPoints {
        /// Index of the second point of the coincident pair.
        index: usize,
    },

    /// A request exceeds an explicit size bound of a math routine.
    #[error("math resource limit exceeded: {resource} requires {actual}, limit {limit}")]
    ResourceLimitExceeded {
        /// The bounded quantity's stable name.
        resource: &'static str,
        /// The bound.
        limit: usize,
        /// The requested amount (`usize::MAX` when its arithmetic overflows).
        actual: usize,
    },
}

pub mod aabb;
pub mod analytic_intersection;
pub mod bvh;
pub mod cdt;
pub mod chord;
pub mod compact_sort;
pub mod context;
pub mod convex_hull;
pub mod curvature;
pub mod curves;
pub mod curves2d;
pub mod det_hash;
pub mod diagnostic;
pub mod filtered;
pub mod fma;
pub mod frame;
pub mod intersect;
pub mod mat;
pub mod nurbs;
pub mod obb;
pub mod plane;
pub mod polygon2d;
pub mod polygon_boolean;
pub mod polygon_offset;
pub mod predicates;
pub mod quadrature;
pub mod ray_triangle;
pub mod surfaces;
pub mod tolerance;
pub mod traits;
pub mod vec;
