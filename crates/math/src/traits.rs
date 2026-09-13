//! Parametric geometry traits for unified curve and surface evaluation.
//!
//! These traits provide a common interface for evaluating both analytic
//! geometry types (circles, cylinders, etc.) and NURBS representations.

use crate::context::OperationContext;
use crate::curves::{Circle3D, Ellipse3D};
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
}

/// Unified interface for parametric curve evaluation.
///
/// Implemented by analytic curves ([`Circle3D`], [`Ellipse3D`]) and [`NurbsCurve`].
pub trait ParametricCurve {
    /// Evaluate the curve at parameter `t`.
    fn evaluate(&self, t: f64) -> Point3;

    /// Tangent vector at parameter `t`.
    fn tangent(&self, t: f64) -> Vec3;

    /// Parameter domain as `(t_min, t_max)`.
    fn domain(&self) -> (f64, f64);
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

    /// Scratch-backed [`point_and_partials`](Self::point_and_partials): one
    /// `derivatives_into(u, v, 1)` into caller-owned storage, so a hot loop
    /// reusing one scratch performs no per-abscissa allocation.
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
}
