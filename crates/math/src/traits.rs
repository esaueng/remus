//! Parametric geometry traits for unified curve and surface evaluation.
//!
//! These traits provide a common interface for evaluating both analytic
//! geometry types (circles, cylinders, etc.) and NURBS representations.

use crate::context::OperationContext;
use crate::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use crate::nurbs::curve::NurbsCurve;
use crate::nurbs::projection::project_point_to_surface;
use crate::nurbs::surface::NurbsSurface;
use crate::surfaces::{
    ConicalSurface, CylindricalSurface, SphericalSurface, SurfaceOfLinearExtrusion,
    SurfaceOfRevolution, ToroidalSurface,
};
use crate::vec::{Point3, Vec3};

/// Unified interface for parametric surface evaluation.
///
/// Implemented by analytic surfaces ([`CylindricalSurface`], [`ConicalSurface`],
/// [`SphericalSurface`], [`ToroidalSurface`]) and [`NurbsSurface`].
pub trait ParametricSurface {
    /// Evaluate the surface at parameters `(u, v)`.
    fn evaluate(&self, u: f64, v: f64) -> Point3;

    /// Surface normal at parameters `(u, v)`.
    ///
    /// Returns the unit normal. For NURBS surfaces at degenerate points,
    /// implementations should return a best-effort fallback (e.g. `Vec3::Z`).
    fn normal(&self, u: f64, v: f64) -> Vec3;

    /// Project a 3D point onto the surface, returning `(u, v)` parameters.
    fn project_point(&self, point: Point3) -> (f64, f64);

    /// Partial derivative ∂S/∂u at (u, v).
    fn partial_u(&self, u: f64, v: f64) -> Vec3;

    /// Partial derivative ∂S/∂v at (u, v).
    fn partial_v(&self, u: f64, v: f64) -> Vec3;

    /// Both first partials `(∂S/∂u, ∂S/∂v)` at (u, v).
    ///
    /// The default forwards to [`partial_u`](Self::partial_u) and
    /// [`partial_v`](Self::partial_v); implementations whose partials share
    /// one evaluation (NURBS) override it so integrators pay for that
    /// evaluation once. Overrides must return exactly what the two separate
    /// calls would.
    fn partials(&self, u: f64, v: f64) -> (Vec3, Vec3) {
        (self.partial_u(u, v), self.partial_v(u, v))
    }

    /// Position and both first partials `(S, ∂S/∂u, ∂S/∂v)` at (u, v).
    ///
    /// The default evaluates each piece separately. Implementations whose
    /// position and partials share one solve (NURBS: one `derivatives(u, v, 1)`
    /// yields all three) override it so quadrature pays for that solve once.
    /// Overrides must return exactly what the three separate calls would,
    /// except for floating-point reassociation at the rounding level.
    fn point_and_partials(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        let (du, dv) = self.partials(u, v);
        (self.evaluate(u, v), du, dv)
    }

    /// [`point_and_partials`](Self::point_and_partials) with caller-provided
    /// scratch storage for implementations that allocate per call (NURBS).
    /// The default ignores `scratch`. Callers must not share one scratch
    /// across threads.
    fn point_and_partials_with_scratch(
        &self,
        u: f64,
        v: f64,
        _scratch: &mut crate::nurbs::surface::DerivativeScratch,
    ) -> (Point3, Vec3, Vec3) {
        self.point_and_partials(u, v)
    }

    /// [`point_and_partials_with_scratch`](Self::point_and_partials_with_scratch)
    /// reusing the previous call's knot spans when the parameters still lie
    /// in them (NURBS only; the default forwards to the unhinted path).
    /// Returns the position, both partials, and whether each axis's span
    /// hint hit — for census only; results are identical either way. Callers
    /// must use one scratch per surface and must not share one across
    /// threads.
    #[inline]
    fn span_hinted_point_and_partials_with_scratch(
        &self,
        u: f64,
        v: f64,
        scratch: &mut crate::nurbs::surface::DerivativeScratch,
    ) -> (Point3, Vec3, Vec3, bool, bool) {
        let (p, du, dv) = self.point_and_partials_with_scratch(u, v, scratch);
        (p, du, dv, false, false)
    }
}

/// Unified interface for parametric curve evaluation.
///
/// Implemented by analytic curves ([`Circle3D`], [`Ellipse3D`], [`Parabola3D`],
/// [`Hyperbola3D`]) and [`NurbsCurve`].
pub trait ParametricCurve {
    /// Evaluate the curve at parameter `t`.
    fn evaluate(&self, t: f64) -> Point3;

    /// Tangent vector at parameter `t`.
    fn tangent(&self, t: f64) -> Vec3;

    /// Parameter domain as `(t_min, t_max)`.
    fn domain(&self) -> (f64, f64);

    /// Exact first and second parameter derivatives `(C'(t), C''(t))`.
    ///
    /// Generic solvers (projection, curve-curve extrema) need true
    /// derivatives: [`Self::tangent`] has no guaranteed length. Carriers
    /// that know their derivatives return them here; the default `None`
    /// makes a solver fall back to finite differences, which lose accuracy
    /// wherever a piecewise carrier (a NURBS knot) is only `C¹`.
    fn derivative_pair(&self, _t: f64) -> Option<(Vec3, Vec3)> {
        None
    }
}

// ── ParametricSurface implementations ────────────────────────────────

impl ParametricSurface for CylindricalSurface {
    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    #[inline]
    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal(u, v)
    }

    #[inline]
    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.project_point(point)
    }

    #[inline]
    fn partial_u(&self, u: f64, _v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        self.x_axis() * (-self.radius() * sin_u) + self.y_axis() * (self.radius() * cos_u)
    }

    #[inline]
    fn partial_v(&self, _u: f64, _v: f64) -> Vec3 {
        self.axis()
    }
}

impl ParametricSurface for ConicalSurface {
    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    #[inline]
    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal(u, v)
    }

    #[inline]
    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.project_point(point)
    }

    #[inline]
    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let cos_a = self.half_angle().cos();
        self.x_axis() * (-v * cos_a * sin_u) + self.y_axis() * (v * cos_a * cos_u)
    }

    #[inline]
    fn partial_v(&self, u: f64, _v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_a, cos_a) = self.half_angle().sin_cos();
        (self.x_axis() * cos_u + self.y_axis() * sin_u) * cos_a + self.axis() * sin_a
    }
}

impl ParametricSurface for SphericalSurface {
    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    #[inline]
    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal(u, v)
    }

    #[inline]
    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.project_point(point)
    }

    #[inline]
    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let cos_v = v.cos();
        self.x_axis() * (-self.radius() * cos_v * sin_u)
            + self.y_axis() * (self.radius() * cos_v * cos_u)
    }

    #[inline]
    fn partial_v(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_v, cos_v) = v.sin_cos();
        self.x_axis() * (-self.radius() * sin_v * cos_u)
            + self.y_axis() * (-self.radius() * sin_v * sin_u)
            + self.z_axis() * (self.radius() * cos_v)
    }
}

impl ParametricSurface for ToroidalSurface {
    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    #[inline]
    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal(u, v)
    }

    #[inline]
    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.project_point(point)
    }

    #[inline]
    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let cos_v = v.cos();
        let tube_radius = self.major_radius() + self.minor_radius() * cos_v;
        self.x_axis() * (-tube_radius * sin_u) + self.y_axis() * (tube_radius * cos_u)
    }

    #[inline]
    fn partial_v(&self, u: f64, v: f64) -> Vec3 {
        let (sin_u, cos_u) = u.sin_cos();
        let (sin_v, cos_v) = v.sin_cos();
        (self.x_axis() * cos_u + self.y_axis() * sin_u) * (-self.minor_radius() * sin_v)
            + self.z_axis() * (self.minor_radius() * cos_v)
    }
}

impl ParametricSurface for NurbsSurface {
    #[inline]
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate(u, v)
    }

    #[inline]
    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal(u, v)
            .unwrap_or_else(|_| Vec3::new(0.0, 0.0, 1.0))
    }

    #[inline]
    fn project_point(&self, point: Point3) -> (f64, f64) {
        // Use default linear tolerance (1e-7) for the Newton projection.
        if let Ok(proj) = project_point_to_surface(self, point, 1e-7) {
            (proj.u, proj.v)
        } else {
            // Fallback: return domain midpoint if Newton fails.
            let (u0, u1) = self.domain_u();
            let (v0, v1) = self.domain_v();
            ((u0 + u1) * 0.5, (v0 + v1) * 0.5)
        }
    }

    #[inline]
    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        let d = self.derivatives(u, v, 1);
        d[1][0]
    }

    #[inline]
    fn partial_v(&self, u: f64, v: f64) -> Vec3 {
        let d = self.derivatives(u, v, 1);
        d[0][1]
    }

    /// One `derivatives(u, v, 1)` serves both partials; the two entries are
    /// the same values `partial_u` and `partial_v` compute separately.
    #[inline]
    fn partials(&self, u: f64, v: f64) -> (Vec3, Vec3) {
        let d = self.derivatives(u, v, 1);
        (d[1][0], d[0][1])
    }

    /// One `derivatives(u, v, 1)` serves the position and both partials: entry
    /// `[0][0]` is the surface point alongside the two first partials.
    ///
    /// The position takes a different summation path than [`Self::evaluate`]
    /// (homogeneous quotient vs. scaled perspective divide), so the two agree
    /// to ~1e-14 in model units, not bit-identically. That is three orders
    /// below the tightest consumer tolerance (`1e-7` linear) and below the
    /// Gauss-quadrature truncation the caller is already converging.
    #[inline]
    fn point_and_partials(&self, u: f64, v: f64) -> (Point3, Vec3, Vec3) {
        let d = self.derivatives(u, v, 1);
        (
            Point3::new(d[0][0].x(), d[0][0].y(), d[0][0].z()),
            d[1][0],
            d[0][1],
        )
    }

    #[inline]
    fn point_and_partials_with_scratch(
        &self,
        u: f64,
        v: f64,
        scratch: &mut crate::nurbs::surface::DerivativeScratch,
    ) -> (Point3, Vec3, Vec3) {
        let (p, du, dv) = scratch.point_and_partials_from(self, u, v);
        (Point3::new(p.x(), p.y(), p.z()), du, dv)
    }

    /// Span-hinted [`point_and_partials_with_scratch`](Self::point_and_partials_with_scratch):
    /// one `derivatives_into(u, v, 1)` that reuses the previous call's knot
    /// spans when the parameters still lie in them. Returns the position,
    /// both partials, and whether each axis's span hint hit — for census
    /// only; results are identical either way. Callers must use one scratch
    /// per surface and must not share one across threads.
    #[inline]
    fn span_hinted_point_and_partials_with_scratch(
        &self,
        u: f64,
        v: f64,
        scratch: &mut crate::nurbs::surface::DerivativeScratch,
    ) -> (Point3, Vec3, Vec3, bool, bool) {
        let (p, du, dv, hit_u, hit_v) = scratch.span_hinted_point_and_partials_from(self, u, v);
        (Point3::new(p.x(), p.y(), p.z()), du, dv, hit_u, hit_v)
    }
}

impl ParametricSurface for SurfaceOfRevolution {
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate_checked(u, v)
            .unwrap_or_else(|_| Point3::new(f64::NAN, f64::NAN, f64::NAN))
    }

    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal_checked(u, v)
            .unwrap_or_else(|_| Vec3::new(f64::NAN, f64::NAN, f64::NAN))
    }

    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.project_point_checked(
            point,
            self.profile().compatibility_bounds(point),
            &OperationContext::new(),
        )
        .map_or((f64::NAN, f64::NAN), |projection| {
            (projection.u, projection.v)
        })
    }

    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        self.derivatives_checked(u, v)
            .map_or_else(|_| Vec3::new(f64::NAN, f64::NAN, f64::NAN), |d| d.1)
    }

    fn partial_v(&self, u: f64, v: f64) -> Vec3 {
        self.derivatives_checked(u, v)
            .map_or_else(|_| Vec3::new(f64::NAN, f64::NAN, f64::NAN), |d| d.2)
    }
}

impl ParametricSurface for SurfaceOfLinearExtrusion {
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        self.evaluate_checked(u, v)
            .unwrap_or_else(|_| Point3::new(f64::NAN, f64::NAN, f64::NAN))
    }

    fn normal(&self, u: f64, v: f64) -> Vec3 {
        self.normal_checked(u, v)
            .unwrap_or_else(|_| Vec3::new(f64::NAN, f64::NAN, f64::NAN))
    }

    fn project_point(&self, point: Point3) -> (f64, f64) {
        self.project_point_checked(
            point,
            self.profile().compatibility_bounds(point),
            &OperationContext::new(),
        )
        .map_or((f64::NAN, f64::NAN), |projection| {
            (projection.u, projection.v)
        })
    }

    fn partial_u(&self, u: f64, v: f64) -> Vec3 {
        self.derivatives_checked(u, v)
            .map_or_else(|_| Vec3::new(f64::NAN, f64::NAN, f64::NAN), |d| d.1)
    }

    fn partial_v(&self, u: f64, v: f64) -> Vec3 {
        self.derivatives_checked(u, v)
            .map_or_else(|_| Vec3::new(f64::NAN, f64::NAN, f64::NAN), |d| d.2)
    }
}

// ── ParametricCurve implementations ─────────────────────────────────

impl ParametricCurve for Circle3D {
    #[inline]
    fn evaluate(&self, t: f64) -> Point3 {
        self.evaluate(t)
    }

    #[inline]
    fn tangent(&self, t: f64) -> Vec3 {
        self.tangent(t)
    }

    #[inline]
    fn domain(&self) -> (f64, f64) {
        (0.0, std::f64::consts::TAU)
    }

    #[inline]
    fn derivative_pair(&self, t: f64) -> Option<(Vec3, Vec3)> {
        let (sin_t, cos_t) = t.sin_cos();
        let (u, v, r) = (self.u_axis(), self.v_axis(), self.radius());
        Some((
            u * (-r * sin_t) + v * (r * cos_t),
            u * (-r * cos_t) + v * (-r * sin_t),
        ))
    }
}

impl ParametricCurve for Ellipse3D {
    #[inline]
    fn evaluate(&self, t: f64) -> Point3 {
        self.evaluate(t)
    }

    #[inline]
    fn tangent(&self, t: f64) -> Vec3 {
        self.tangent(t)
    }

    #[inline]
    fn domain(&self) -> (f64, f64) {
        (0.0, std::f64::consts::TAU)
    }

    #[inline]
    fn derivative_pair(&self, t: f64) -> Option<(Vec3, Vec3)> {
        let (sin_t, cos_t) = t.sin_cos();
        let (u, v) = (self.u_axis(), self.v_axis());
        let (a, b) = (self.semi_major(), self.semi_minor());
        Some((
            u * (-a * sin_t) + v * (b * cos_t),
            u * (-a * cos_t) + v * (-b * sin_t),
        ))
    }
}

/// The parabola's parameter runs over all reals (`t = 0` is the vertex and
/// `t` carries units of length), so its domain is unbounded. Generic
/// extrema solvers take an explicit finite range; pass the trimmed span of
/// the edge or the region of interest, never this domain directly.
impl ParametricCurve for Parabola3D {
    #[inline]
    fn evaluate(&self, t: f64) -> Point3 {
        self.evaluate(t)
    }

    #[inline]
    fn tangent(&self, t: f64) -> Vec3 {
        self.tangent(t)
    }

    #[inline]
    fn domain(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    #[inline]
    fn derivative_pair(&self, t: f64) -> Option<(Vec3, Vec3)> {
        let two_f = 2.0 * self.focal_length();
        Some((
            self.axis_dir() * (t / two_f) + self.u_axis(),
            self.axis_dir() * (1.0 / two_f),
        ))
    }
}

/// The hyperbola branch `center + a·cosh(t)·u + b·sinh(t)·v` is defined for
/// all real `t`, so its domain is unbounded. Generic extrema solvers take
/// an explicit finite range; pass the trimmed span, never this domain.
impl ParametricCurve for Hyperbola3D {
    #[inline]
    fn evaluate(&self, t: f64) -> Point3 {
        self.evaluate(t)
    }

    #[inline]
    fn tangent(&self, t: f64) -> Vec3 {
        self.tangent(t)
    }

    #[inline]
    fn domain(&self) -> (f64, f64) {
        (f64::NEG_INFINITY, f64::INFINITY)
    }

    #[inline]
    fn derivative_pair(&self, t: f64) -> Option<(Vec3, Vec3)> {
        let (sinh_t, cosh_t) = (t.sinh(), t.cosh());
        let (u, v) = (self.u_axis(), self.v_axis());
        let (a, b) = (self.semi_major(), self.semi_minor());
        Some((
            u * (a * sinh_t) + v * (b * cosh_t),
            u * (a * cosh_t) + v * (b * sinh_t),
        ))
    }
}

impl ParametricCurve for NurbsCurve {
    #[inline]
    fn evaluate(&self, t: f64) -> Point3 {
        self.evaluate(t)
    }

    #[inline]
    fn tangent(&self, t: f64) -> Vec3 {
        self.tangent(t).unwrap_or_else(|_| Vec3::new(1.0, 0.0, 0.0))
    }

    #[inline]
    fn domain(&self) -> (f64, f64) {
        self.domain()
    }

    #[inline]
    fn derivative_pair(&self, t: f64) -> Option<(Vec3, Vec3)> {
        let ders = self.derivatives(t, 2);
        Some((*ders.get(1)?, *ders.get(2)?))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;

    /// `derivative_pair` must be the true derivatives of `evaluate`: check
    /// both against fourth-order central differences of positions.
    fn assert_derivative_pair<C: ParametricCurve>(curve: &C, ts: &[f64], name: &str) {
        let h = 1e-3;
        for &t in ts {
            let (d1, d2) = curve.derivative_pair(t).expect("exact derivatives");
            let p = |k: f64| curve.evaluate(k.mul_add(h, t));
            let fd1 = ((p(1.0) - p(-1.0)) * 8.0 - (p(2.0) - p(-2.0))) * (1.0 / (12.0 * h));
            let c = p(0.0);
            let fd2 = ((p(1.0) - c) * 16.0 + (p(-1.0) - c) * 16.0 - (p(2.0) - c) - (p(-2.0) - c))
                * (1.0 / (12.0 * h * h));
            let e1 = (d1 - fd1).length() / d1.length().max(1.0);
            let e2 = (d2 - fd2).length() / d2.length().max(1.0);
            assert!(
                e1 < 1e-9 && e2 < 1e-5,
                "{name} at t={t}: errors ({e1:.2e}, {e2:.2e})"
            );
        }
    }

    #[test]
    fn conic_and_nurbs_derivative_pairs_match_positions() {
        let n = Vec3::new(0.3, -0.4, 0.866);
        let c = Point3::new(1.0, -2.0, 0.5);
        let ts = [-1.3, -0.2, 0.0, 0.7, 2.9];
        assert_derivative_pair(&Circle3D::new(c, n, 1.7).unwrap(), &ts, "circle");
        assert_derivative_pair(&Ellipse3D::new(c, n, 2.5, 0.8).unwrap(), &ts, "ellipse");
        let axis = Vec3::new(0.0, 0.6, 0.8);
        assert_derivative_pair(&Parabola3D::new(c, axis, 0.9).unwrap(), &ts, "parabola");
        assert_derivative_pair(&Hyperbola3D::new(c, n, 1.5, 0.6).unwrap(), &ts, "hyperbola");
        let nurbs = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, -1.0),
                Point3::new(3.0, -1.0, 2.0),
                Point3::new(4.0, 1.0, 0.0),
            ],
            vec![1.0, 0.7, 1.3, 1.0],
        )
        .unwrap();
        assert_derivative_pair(&nurbs, &[0.1, 0.35, 0.5, 0.8], "nurbs");
    }
}
