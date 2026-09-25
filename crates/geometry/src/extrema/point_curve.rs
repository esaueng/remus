//! Point-to-curve projection algorithms.
//!
//! Provides analytic fast paths for common curve types ([`Line3D`], [`Circle3D`])
//! and a generic Newton-Raphson solver for any [`ParametricCurve`].

use remus_math::curves::{Circle3D, Line3D};
use remus_math::traits::ParametricCurve;
use remus_math::vec::Point3;

use super::CurveProjection;

/// Maximum iterations for Newton-Raphson refinement.
const MAX_ITER: usize = 50;

/// Convergence tolerance on the parameter step, relative to the range.
const PARAM_TOL_REL: f64 = 1e-14;

/// Maximum step halvings per Newton iteration.
const MAX_BACKTRACK: usize = 40;

/// Number of uniform samples used in the global search phase.
const N_SAMPLES: usize = 64;

// ── Analytic fast paths ──────────────────────────────────────────────────────

/// Project a point onto a bounded line segment, clamped to `[t_start, t_end]`.
///
/// The line is parameterized as `P(t) = origin + t * direction`, where
/// `direction` is a unit vector. The unconstrained closest parameter is the
/// orthogonal projection; clamping handles the finite-segment case.
///
/// # Examples
///
/// ```
/// use remus_math::curves::Line3D;
/// use remus_math::vec::{Point3, Vec3};
/// use remus_geometry::extrema::point_to_line;
///
/// let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
/// let proj = point_to_line(Point3::new(3.0, 2.0, 0.0), &line, 0.0, 10.0);
/// assert!((proj.parameter - 3.0).abs() < 1e-12);
/// assert!((proj.distance - 2.0).abs() < 1e-12);
/// ```
#[must_use]
pub fn point_to_line(point: Point3, line: &Line3D, t_start: f64, t_end: f64) -> CurveProjection {
    // Unconstrained projection: t = dot(point - origin, direction)
    let t_unclamped = line.project(point);
    let t = t_unclamped.clamp(t_start, t_end);
    let closest = line.evaluate(t);
    let diff = closest - point;
    let distance = (diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z()).sqrt();
    CurveProjection {
        distance,
        point: closest,
        parameter: t,
    }
}

/// Project a point onto a full circle (closed, periodic).
///
/// The circle is parameterized as `P(t) = center + r*(cos(t)*u + sin(t)*v)`.
/// This uses the analytic projection: project the point into the circle's plane,
/// compute `atan2`, then evaluate.
///
/// # Examples
///
/// ```
/// use remus_math::curves::Circle3D;
/// use remus_math::vec::{Point3, Vec3};
/// use remus_geometry::extrema::point_to_circle;
///
/// let circle = Circle3D::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Vec3::new(0.0, 0.0, 1.0),
///     1.0,
/// ).unwrap();
/// let proj = point_to_circle(Point3::new(0.0, 5.0, 0.0), &circle);
/// assert!((proj.distance - 4.0).abs() < 1e-12);
/// ```
#[must_use]
pub fn point_to_circle(point: Point3, circle: &Circle3D) -> CurveProjection {
    let t = circle.project(point);
    let closest = circle.evaluate(t);
    let diff = closest - point;
    let distance = (diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z()).sqrt();
    CurveProjection {
        distance,
        point: closest,
        parameter: t,
    }
}

// ── Generic Newton-Raphson solver ────────────────────────────────────────────

/// Project a point onto a parametric curve over `[t_start, t_end]`.
///
/// **Algorithm:**
/// 1. Sample the curve at `N_SAMPLES` uniformly-spaced parameters to find the
///    global closest sample (avoids local-minimum traps on non-convex curves).
/// 2. Refine the best sample using Newton-Raphson on the stationarity condition
///    `dot(C(t) - P, C'(t)) = 0`, clamping `t` to `[t_start, t_end]` after
///    each step.
///
/// Convergence is declared when the parameter update `|Δt|` drops below
/// `1e-14` of the range, or after 50 iterations (whichever comes first).
///
/// # Examples
///
/// ```
/// use remus_math::curves::Circle3D;
/// use remus_math::vec::{Point3, Vec3};
/// use remus_geometry::extrema::point_to_curve;
/// use std::f64::consts::TAU;
///
/// let circle = Circle3D::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Vec3::new(0.0, 0.0, 1.0),
///     2.0,
/// ).unwrap();
/// let proj = point_to_curve(Point3::new(0.0, 10.0, 0.0), &circle, 0.0, TAU);
/// assert!(proj.distance < 9.0); // closest point on circle is ≤ 8.0 from query
/// ```
#[must_use]
pub fn point_to_curve<C: ParametricCurve>(
    point: Point3,
    curve: &C,
    t_start: f64,
    t_end: f64,
) -> CurveProjection {
    // Degenerate range: evaluate the single point.
    if t_end <= t_start {
        let p = curve.evaluate(t_start);
        return CurveProjection {
            distance: (p - point).length(),
            point: p,
            parameter: t_start,
        };
    }

    // ── Phase 1: global search ───────────────────────────────────────────────
    let step = (t_end - t_start) / (N_SAMPLES - 1) as f64;
    let mut best_t = t_start;
    let mut best_dist_sq = f64::INFINITY;

    for i in 0..N_SAMPLES {
        let t = if i == N_SAMPLES - 1 {
            t_end
        } else {
            t_start + i as f64 * step
        };
        let p = curve.evaluate(t);
        let diff = p - point;
        let d2 = diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z();
        if d2 < best_dist_sq {
            best_dist_sq = d2;
            best_t = t;
        }
    }

    // ── Phase 2: safeguarded Newton refinement ───────────────────────────────
    // Solve f(t) = (C(t) − P)·C'(t) = 0, the stationarity of
    // g(t) = ½|C(t) − P|² (g' = f). `ParametricCurve` exposes positions
    // only up to a tangent of unspecified length, so C' and C'' come from
    // central differences (`curve_derivatives`).
    //
    // The FULL derivative f' = |C'|² + (C − P)·C'' is used. The
    // Gauss-Newton approximation f' ≈ |C'|² drops the curvature term,
    // which is not small at a distance: at an ellipse's minor-axis vertex
    // seen from 4 radii the dropped term equals the kept one, so every
    // step was twice too long and the iterate orbited the minimum; at a
    // hyperbola vertex the step was three times too long and diverged
    // (B10). When f' <= 0 (near a distance maximum) the Gauss-Newton step
    // is used instead: it is always a descent direction for g. Every step
    // is backtracked until the distance does not increase (or the
    // stationarity residual halves), and convergence is judged relative
    // to the parameter range, so the
    // result does not depend on the model or parameter scale.
    let span = t_end - t_start;
    let param_tol = span * PARAM_TOL_REL;
    let dist_sq_at = |t: f64| (curve.evaluate(t) - point).length_squared();
    let mut t = best_t;
    let mut g = dist_sq_at(t);
    for _ in 0..MAX_ITER {
        let (p, vel, acc) = super::curve_derivatives(curve, t, t_start, t_end);
        let diff = p - point;

        let f = diff.dot(vel);
        let vel_sq = vel.length_squared();
        if !(vel_sq.is_finite() && vel_sq > 0.0) || f == 0.0 {
            break;
        }
        let full = vel_sq + diff.dot(acc);
        let mut step = if full.is_finite() && full > 0.0 {
            f / full
        } else {
            f / vel_sq
        };

        // Backtrack until the squared distance does not increase, or the
        // stationarity residual at least halves. Near the minimum g is flat
        // to second order, so comparing g values alone can only locate the
        // foot to ~sqrt(eps); the residual is accurate to eps.
        let stationarity = |t: f64| {
            let (p, vel, _) = super::curve_derivatives(curve, t, t_start, t_end);
            (p - point).dot(vel).abs()
        };
        let mut accepted = None;
        for _ in 0..MAX_BACKTRACK {
            let t_new = (t - step).clamp(t_start, t_end);
            let g_new = dist_sq_at(t_new);
            if g_new <= g || stationarity(t_new) <= 0.5 * f.abs() {
                accepted = Some((t_new, g_new));
                break;
            }
            step *= 0.5;
        }
        let Some((t_new, g_new)) = accepted else {
            break;
        };
        let moved = (t_new - t).abs();
        t = t_new;
        g = g.min(g_new);
        if moved <= param_tol {
            break;
        }
    }

    let closest = curve.evaluate(t);
    let diff = closest - point;
    let distance = (diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z()).sqrt();
    CurveProjection {
        distance,
        point: closest,
        parameter: t,
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::vec::Vec3;
    use std::f64::consts::{PI, TAU};

    // ── point_to_line ────────────────────────────────────────────────────────

    #[test]
    fn line_point_above_midpoint() {
        // Line along X-axis from 0 to 10; query point directly above x=5.
        let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let proj = point_to_line(Point3::new(5.0, 3.0, 0.0), &line, 0.0, 10.0);
        assert!(
            (proj.parameter - 5.0).abs() < 1e-12,
            "param={}",
            proj.parameter
        );
        assert!(
            (proj.distance - 3.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
        assert!((proj.point.x() - 5.0).abs() < 1e-12);
        assert!((proj.point.y()).abs() < 1e-12);
    }

    #[test]
    fn line_point_before_start_clamps() {
        // Query point is "behind" t=0; should clamp to t_start.
        let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let proj = point_to_line(Point3::new(-5.0, 0.0, 0.0), &line, 0.0, 10.0);
        assert!(
            (proj.parameter - 0.0).abs() < 1e-12,
            "param={}",
            proj.parameter
        );
        assert!(
            (proj.distance - 5.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
    }

    #[test]
    fn line_point_past_end_clamps() {
        // Query point is past t_end; should clamp to t_end.
        let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let proj = point_to_line(Point3::new(15.0, 0.0, 0.0), &line, 0.0, 10.0);
        assert!(
            (proj.parameter - 10.0).abs() < 1e-12,
            "param={}",
            proj.parameter
        );
        assert!(
            (proj.distance - 5.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
    }

    #[test]
    fn line_point_on_line_zero_distance() {
        let line = Line3D::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        let proj = point_to_line(Point3::new(1.0, 2.0, 5.0), &line, 0.0, 20.0);
        assert!(proj.distance < 1e-12, "dist={}", proj.distance);
        assert!(
            (proj.parameter - 2.0).abs() < 1e-12,
            "param={}",
            proj.parameter
        );
    }

    // ── point_to_circle ──────────────────────────────────────────────────────

    #[test]
    fn circle_point_on_positive_y_axis() {
        // Unit circle in XY plane; point at (0, 5, 0).
        // The analytic projection must produce the closest point on the circle,
        // which lies in the direction of (0,5,0) from the center.
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let q = Point3::new(0.0, 5.0, 0.0);
        let proj = point_to_circle(q, &circle);
        // Distance must be 5 - 1 = 4.
        assert!(
            (proj.distance - 4.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
        // Closest point must lie on the circle (distance from center = 1).
        let r = (proj.point.x() * proj.point.x()
            + proj.point.y() * proj.point.y()
            + proj.point.z() * proj.point.z())
        .sqrt();
        assert!((r - 1.0).abs() < 1e-12, "closest not on circle: r={r}");
        // Closest point must be collinear with center and query point in-plane.
        // Because q is in the XY plane and the circle is in XY, closest must
        // point in the direction of q: x≈0, y≈1, z≈0.
        assert!(proj.point.x().abs() < 1e-12, "x={}", proj.point.x());
        assert!((proj.point.y() - 1.0).abs() < 1e-12, "y={}", proj.point.y());
    }

    #[test]
    fn circle_point_on_axis_distance_equals_radius() {
        // Point on the circle's axis is equidistant from all points;
        // distance should be the radius.
        let r = 3.0_f64;
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        // Point directly on the axis (center), offset in Z.
        let proj = point_to_circle(Point3::new(0.0, 0.0, 5.0), &circle);
        // The closest point is anywhere on the circle; distance is sqrt(r^2 + 25).
        let expected = (r * r + 25.0_f64).sqrt();
        assert!(
            (proj.distance - expected).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
    }

    #[test]
    fn circle_point_in_plane() {
        // Circle of radius 2 in XY; query point at (3, 0, 0).
        // Closest circle point must be on the circle in the direction of (3,0,0),
        // i.e. at (2,0,0), with distance = 1.
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let proj = point_to_circle(Point3::new(3.0, 0.0, 0.0), &circle);
        assert!(
            (proj.distance - 1.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
        // Closest point must lie on the circle.
        let r = (proj.point.x() * proj.point.x()
            + proj.point.y() * proj.point.y()
            + proj.point.z() * proj.point.z())
        .sqrt();
        assert!((r - 2.0).abs() < 1e-12, "closest not on circle: r={r}");
        // Closest point must be at (2, 0, 0).
        assert!((proj.point.x() - 2.0).abs() < 1e-12, "x={}", proj.point.x());
        assert!(proj.point.y().abs() < 1e-12, "y={}", proj.point.y());
    }

    // ── point_to_curve (generic) ─────────────────────────────────────────────

    #[test]
    fn generic_circle_matches_analytic() {
        // Using the generic solver on a circle should match the analytic result.
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
        let q = Point3::new(0.0, 5.0, 0.0);

        let analytic = point_to_circle(q, &circle);
        let generic = point_to_curve(q, &circle, 0.0, TAU);

        assert!(
            (analytic.distance - generic.distance).abs() < 1e-6,
            "analytic={} generic={}",
            analytic.distance,
            generic.distance
        );
        assert!(
            (analytic.parameter - generic.parameter).abs() < 1e-6,
            "analytic_t={} generic_t={}",
            analytic.parameter,
            generic.parameter
        );
    }

    #[test]
    fn generic_stationarity_condition_satisfied() {
        // After projection, dot(C(t)-P, C'(t)) must be near zero
        // (Karush-Kuhn-Tucker stationarity for interior minimizers).
        // We check using the analytic circle tangent scaled by radius.
        let r = 5.0_f64;
        let circle =
            Circle3D::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 1.0), r).unwrap();
        let q = Point3::new(3.0, 7.0, 3.0);
        let proj = point_to_curve(q, &circle, 0.0, TAU);

        let p = circle.evaluate(proj.parameter);
        // The actual velocity C'(t) for a circle has magnitude r; the unit
        // tangent returned by ParametricCurve::tangent is C'(t)/r.
        // We check dot(C(t)-P, unit_tangent) ≈ 0 (equivalent, just scaled by r).
        let tan = circle.tangent(proj.parameter);
        let diff = p - q;
        let dot = diff.x() * tan.x() + diff.y() * tan.y() + diff.z() * tan.z();
        assert!(dot.abs() < 1e-6, "stationarity violated: dot={dot}");
    }

    #[test]
    fn generic_bounded_domain_clamping() {
        // Half-circle domain [0, π]; query point at t = 3π/2 (outside domain).
        // The closest point on [0, π] should be an endpoint.
        let circle =
            Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        // t = 3π/2 evaluates to (0, -1, 0).
        let q = circle.evaluate(3.0 * PI / 2.0);
        let proj = point_to_curve(q, &circle, 0.0, PI);
        // The closest endpoint should be at t = 0 or t = π (symmetric, both at distance √2).
        assert!(
            proj.parameter < 1e-10 || (proj.parameter - PI).abs() < 1e-10,
            "expected endpoint, got t={}",
            proj.parameter
        );
        assert!(
            (proj.distance - 2.0_f64.sqrt()).abs() < 1e-6,
            "dist={}",
            proj.distance
        );
    }

    // ── Test curves for the generic solver ───────────────────────────────────

    /// Circular helix `C(t) = (r·cos t, r·sin t, pitch·t)`.
    ///
    /// A negative `pitch` descends. Several turns give the distance function
    /// one local minimum per turn, so a solver that starts its refinement on
    /// the wrong turn cannot recover.
    struct Helix {
        radius: f64,
        pitch: f64,
        t_start: f64,
        t_end: f64,
    }

    impl ParametricCurve for Helix {
        fn evaluate(&self, t: f64) -> Point3 {
            Point3::new(self.radius * t.cos(), self.radius * t.sin(), self.pitch * t)
        }

        fn tangent(&self, t: f64) -> Vec3 {
            Vec3::new(-self.radius * t.sin(), self.radius * t.cos(), self.pitch)
                .normalize()
                .unwrap()
        }

        fn domain(&self) -> (f64, f64) {
            (self.t_start, self.t_end)
        }
    }

    /// Ellipse lying in the plane `y = 0`: `C(t) = (a·cos t, 0, b·sin t)`.
    ///
    /// Used for queries that also lie in `y = 0`, so every sampled point has a
    /// zero y-offset from the query.
    struct EllipseXz {
        a: f64,
        b: f64,
    }

    impl ParametricCurve for EllipseXz {
        fn evaluate(&self, t: f64) -> Point3 {
            Point3::new(self.a * t.cos(), 0.0, self.b * t.sin())
        }

        fn tangent(&self, t: f64) -> Vec3 {
            Vec3::new(-self.a * t.sin(), 0.0, self.b * t.cos())
                .normalize()
                .unwrap()
        }

        fn domain(&self) -> (f64, f64) {
            (-PI, PI)
        }
    }

    /// Circular arc whose parameter is offset and scaled:
    /// `C(t) = (r·cos(rate·(t - origin)), r·sin(rate·(t - origin)), 0)`.
    ///
    /// The sweep per unit parameter is `rate`, so the finite-difference step
    /// used by the solver must scale with the *length* of the parameter
    /// domain, not with the magnitude of the parameter values.
    struct ScaledArc {
        radius: f64,
        origin: f64,
        rate: f64,
    }

    impl ScaledArc {
        fn angle(&self, t: f64) -> f64 {
            self.rate * (t - self.origin)
        }
    }

    impl ParametricCurve for ScaledArc {
        fn evaluate(&self, t: f64) -> Point3 {
            let a = self.angle(t);
            Point3::new(self.radius * a.cos(), self.radius * a.sin(), 0.0)
        }

        fn tangent(&self, t: f64) -> Vec3 {
            let a = self.angle(t);
            Vec3::new(-a.sin(), a.cos(), 0.0)
        }

        fn domain(&self) -> (f64, f64) {
            (self.origin, self.origin + ARC_DOMAIN_LEN)
        }
    }

    /// Length of the [`ScaledArc`] parameter domain.
    const ARC_DOMAIN_LEN: f64 = 1.0e-4;

    /// Degenerate curve: every parameter evaluates to the same point, so the
    /// velocity vanishes everywhere.
    struct ConstantCurve {
        position: Point3,
    }

    impl ParametricCurve for ConstantCurve {
        fn evaluate(&self, _t: f64) -> Point3 {
            self.position
        }

        fn tangent(&self, _t: f64) -> Vec3 {
            Vec3::new(1.0, 0.0, 0.0)
        }

        fn domain(&self) -> (f64, f64) {
            (0.0, 2.0)
        }
    }

    // ── Reference helpers (independent of the solver under test) ─────────────

    /// Minimum distance from `point` to `curve` over `[t_start, t_end]`, found
    /// by a dense uniform scan. Ground truth for the "globally closest point"
    /// contract of [`point_to_curve`].
    fn brute_force_min<C: ParametricCurve>(
        point: Point3,
        curve: &C,
        t_start: f64,
        t_end: f64,
    ) -> f64 {
        const STEPS: usize = 20_000;
        let mut best = f64::INFINITY;
        for i in 0..=STEPS {
            let t = (t_end - t_start).mul_add(i as f64 / STEPS as f64, t_start);
            let d = (curve.evaluate(t) - point).length();
            if d < best {
                best = d;
            }
        }
        best
    }

    /// `dot(C(t) - P, unit_tangent(t))` — zero at an interior minimum.
    fn stationarity<C: ParametricCurve>(point: Point3, curve: &C, t: f64) -> f64 {
        let diff = curve.evaluate(t) - point;
        let tan = curve.tangent(t);
        diff.x() * tan.x() + diff.y() * tan.y() + diff.z() * tan.z()
    }

    // ── point_to_line ────────────────────────────────────────────────────────

    #[test]
    fn line_distance_uses_every_component_of_the_offset() {
        // Line along X; the query is offset 3 in Y and 4 in Z from the foot of
        // the perpendicular at t = 5, so the distance is the 3-4-5 hypotenuse.
        let line = Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let proj = point_to_line(Point3::new(5.0, 3.0, 4.0), &line, 0.0, 10.0);
        assert!(
            (proj.parameter - 5.0).abs() < 1e-12,
            "param={}",
            proj.parameter
        );
        assert!(
            (proj.distance - 5.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
        assert!((proj.point.z()).abs() < 1e-12, "z={}", proj.point.z());
    }

    // ── point_to_curve (generic) ─────────────────────────────────────────────

    #[test]
    fn generic_circle_off_plane_query_matches_closed_form() {
        // Circle of radius 3 in the XY plane; the query sits 4 above the plane
        // on the ray at angle 0.7 rad, 5 out from the axis. The closed-form
        // distance is sqrt((5 - 3)^2 + 4^2) = sqrt(20), attained at t = 0.7.
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            3.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let angle = 0.7_f64;
        let query = Point3::new(5.0 * angle.cos(), 5.0 * angle.sin(), 4.0);
        let proj = point_to_curve(query, &circle, 0.0, TAU);

        assert!(
            (proj.distance - 20.0_f64.sqrt()).abs() < 1e-9,
            "dist={}",
            proj.distance
        );
        assert!(
            (proj.parameter - angle).abs() < 1e-6,
            "param={}",
            proj.parameter
        );
        // The reported point must be the curve point at the reported parameter.
        let on_curve = circle.evaluate(proj.parameter);
        assert!(
            (on_curve - proj.point).length() < 1e-9,
            "reported point is not on the curve"
        );
    }

    #[test]
    fn generic_multi_turn_helix_picks_the_closest_turn() {
        // Three turns; the query lies beside the middle turn. Each turn carries
        // its own local minimum, so the global sampling phase decides which one
        // the refinement lands in.
        let helix = Helix {
            radius: 3.0,
            pitch: 0.3,
            t_start: -2.0 * TAU,
            t_end: TAU,
        };
        let query = Point3::new(5.0, 0.0, 0.7);
        let proj = point_to_curve(query, &helix, helix.t_start, helix.t_end);

        let reference = brute_force_min(query, &helix, helix.t_start, helix.t_end);
        assert!(
            proj.distance <= reference + 1e-6,
            "not the global minimum: got {} reference {}",
            proj.distance,
            reference
        );
        let residual = stationarity(query, &helix, proj.parameter);
        assert!(residual.abs() < 1e-6, "stationarity violated: {residual}");
        assert!(
            proj.parameter >= helix.t_start && proj.parameter <= helix.t_end,
            "param out of domain: {}",
            proj.parameter
        );
    }

    #[test]
    fn generic_steep_descending_helix_keeps_interior_minimum() {
        // One turn of a steeply descending helix: the axial velocity dominates
        // the circumferential one, and the minimum is interior to the domain.
        let helix = Helix {
            radius: 2.0,
            pitch: -5.0,
            t_start: 0.0,
            t_end: TAU,
        };
        let query = Point3::new(4.0, 0.0, -10.0);
        let proj = point_to_curve(query, &helix, helix.t_start, helix.t_end);

        let reference = brute_force_min(query, &helix, helix.t_start, helix.t_end);
        assert!(
            proj.distance <= reference + 1e-6,
            "not the global minimum: got {} reference {}",
            proj.distance,
            reference
        );
        let residual = stationarity(query, &helix, proj.parameter);
        assert!(residual.abs() < 1e-6, "stationarity violated: {residual}");
        // The minimum is strictly interior; an endpoint answer is wrong.
        assert!(
            proj.parameter > 1e-3 && proj.parameter < TAU - 1e-3,
            "expected interior minimum, got t={}",
            proj.parameter
        );
    }

    #[test]
    fn generic_curve_coplanar_with_query_finds_global_minimum() {
        // Ellipse in the plane y = 0 with the query also at y = 0: every
        // sampled point has a zero y-offset. The query sits on the minor axis,
        // so the closest point is the near minor-axis vertex (0, 0, 2) at
        // distance 2 - 0.3 = 1.7; the far vertex is a competing local minimum
        // at distance 2.3.
        let ellipse = EllipseXz { a: 5.0, b: 2.0 };
        let query = Point3::new(0.0, 0.0, 0.3);
        let proj = point_to_curve(query, &ellipse, -PI, PI);

        assert!((proj.distance - 1.7).abs() < 1e-9, "dist={}", proj.distance);
        assert!(
            (proj.parameter - PI / 2.0).abs() < 1e-6,
            "param={}",
            proj.parameter
        );
    }

    #[test]
    fn generic_offset_parameter_domain_uses_domain_scaled_step() {
        // The arc sweeps 90 degrees over a parameter domain of length 1e-4
        // placed at t = 100. The query lies on the ray at 22.5 degrees, 3 out
        // from the centre of a radius-2 arc, so the distance is exactly 1.
        let rate = (PI / 2.0) / ARC_DOMAIN_LEN;
        let arc = ScaledArc {
            radius: 2.0,
            origin: 100.0,
            rate,
        };
        let (t_start, t_end) = arc.domain();
        let angle = PI / 8.0;
        let query = Point3::new(3.0 * angle.cos(), 3.0 * angle.sin(), 0.0);
        let proj = point_to_curve(query, &arc, t_start, t_end);

        assert!((proj.distance - 1.0).abs() < 1e-6, "dist={}", proj.distance);
        // Convergence is declared on |dt| < 1e-10, which at this parameter
        // scale bounds the angular residual near 1e-6 — far below the error a
        // mis-scaled difference step produces.
        let residual = stationarity(query, &arc, proj.parameter);
        assert!(residual.abs() < 1e-3, "stationarity violated: {residual}");
        assert!(
            proj.parameter >= t_start && proj.parameter <= t_end,
            "param out of domain: {}",
            proj.parameter
        );
    }

    #[test]
    fn generic_zero_velocity_curve_returns_finite_result() {
        // Degenerate curve: the velocity vanishes, so the Newton phase must
        // bail out instead of dividing by it.
        let curve = ConstantCurve {
            position: Point3::new(1.0, 2.0, 3.0),
        };
        let (t_start, t_end) = curve.domain();
        let proj = point_to_curve(Point3::new(1.0, 2.0, 7.0), &curve, t_start, t_end);

        assert!(proj.distance.is_finite(), "dist={}", proj.distance);
        assert!(
            (proj.distance - 4.0).abs() < 1e-12,
            "dist={}",
            proj.distance
        );
        assert!(
            proj.parameter.is_finite() && proj.parameter >= t_start && proj.parameter <= t_end,
            "param={}",
            proj.parameter
        );
    }
}
