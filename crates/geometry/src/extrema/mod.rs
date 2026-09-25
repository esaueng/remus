//! Distance and extrema computation between geometry primitives.
//!
//! # Result types
//!
//! - [`ExtremaSolution`] — general closest-point result between two entities.
//! - [`CurveProjection`] — closest point on a curve from a query point.
//! - [`SurfaceProjection`] — closest point on a surface from a query point.
//!
//! # Algorithms
//!
//! - [`point_curve`] — point-to-curve projection (analytic fast paths + generic
//!   Newton-Raphson fallback).
//! - [`point_surface`] — point-to-surface projection (analytic fast paths + generic
//!   Newton-Raphson fallback).
//! - [`curve_curve`] — curve-to-curve minimum distance (analytic `line_to_line` +
//!   generic sampler/Newton-Raphson).
//! - [`lipschitz`] — Lipschitz global optimizer and NURBS curve-to-curve distance.
//! - [`segment`] — segment-to-segment minimum distance.

pub mod curve_curve;
pub mod lipschitz;
pub mod point_curve;
pub mod point_surface;
pub mod segment;

pub use curve_curve::{curve_to_curve, line_to_line};
pub use lipschitz::{estimate_curve_curve_lipschitz, nurbs_curve_curve_distance};
pub use point_curve::{point_to_circle, point_to_curve, point_to_line};
pub use point_surface::{
    point_to_cone, point_to_cylinder, point_to_nurbs_surface, point_to_plane, point_to_sphere,
    point_to_surface, point_to_torus,
};
pub use segment::segment_segment_distance;

use remus_math::traits::ParametricCurve;
use remus_math::vec::{Point3, Vec3};

/// Finite-difference step for curve derivatives, relative to the range.
const FD_STEP_REL: f64 = 1e-4;

/// Position, first derivative and second derivative of `curve` at `t`.
///
/// Exact when the carrier provides [`ParametricCurve::derivative_pair`]
/// (NURBS and every analytic conic). Otherwise the derivatives come from
/// central differences: the fourth-order stencil for `C'` (its error sets
/// how exactly a solver's stationary point is located) and the five-point
/// stencil for `C''` (which only sets the Newton convergence rate). The
/// step is relative to the parameter range, and the stencil is shifted
/// inward so every sample stays inside `[t_start, t_end]`; the position is
/// taken at `t` itself.
pub(crate) fn curve_derivatives<C: ParametricCurve>(
    curve: &C,
    t: f64,
    t_start: f64,
    t_end: f64,
) -> (Point3, Vec3, Vec3) {
    if let Some((vel, acc)) = curve.derivative_pair(t)
        && vel.x().is_finite()
        && vel.y().is_finite()
        && vel.z().is_finite()
        && acc.x().is_finite()
        && acc.y().is_finite()
        && acc.z().is_finite()
    {
        return (curve.evaluate(t), vel, acc);
    }
    let h = (t_end - t_start) * FD_STEP_REL;
    let tc = t.clamp(t_start + 2.0 * h, t_end - 2.0 * h);
    let c0 = curve.evaluate(tc);
    let (m2, m1) = (curve.evaluate(tc - 2.0 * h), curve.evaluate(tc - h));
    let (p1, p2) = (curve.evaluate(tc + h), curve.evaluate(tc + 2.0 * h));
    let vel = ((p1 - m1) * 8.0 - (p2 - m2)) * (1.0 / (12.0 * h));
    let acc =
        ((p1 - c0) * 16.0 + (m1 - c0) * 16.0 - (p2 - c0) - (m2 - c0)) * (1.0 / (12.0 * h * h));
    (curve.evaluate(t), vel, acc)
}

/// Result of a distance/extrema computation between two geometric entities.
#[derive(Debug, Clone, Copy)]
pub struct ExtremaSolution {
    /// Minimum distance found.
    pub distance: f64,
    /// Closest point on entity A.
    pub point_a: Point3,
    /// Closest point on entity B.
    pub point_b: Point3,
    /// Parameter on entity A at closest point.
    pub param_a: f64,
    /// Parameter on entity B at closest point.
    pub param_b: f64,
}

/// Result of projecting a point onto a curve.
#[derive(Debug, Clone, Copy)]
pub struct CurveProjection {
    /// Distance from point to closest point on curve.
    pub distance: f64,
    /// Closest point on the curve.
    pub point: Point3,
    /// Parameter value at closest point.
    pub parameter: f64,
}

/// Result of projecting a point onto a surface.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceProjection {
    /// Distance from point to closest point on surface.
    pub distance: f64,
    /// Closest point on the surface.
    pub point: Point3,
    /// U parameter at closest point.
    pub u: f64,
    /// V parameter at closest point.
    pub v: f64,
}
