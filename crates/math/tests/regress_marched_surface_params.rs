//! Regression coverage for `exact_to_marched` / `exacts_to_marched` surface
//! parameters (`crates/math/src/analytic_intersection.rs`).
//!
//! The exact-to-marched conversion used to emit `param1 = param2 = (0.0, 0.0)`
//! on every sample. These tests pin the corrected guarantees through the
//! public entries that route through the conversion:
//! - `intersect_analytic_analytic` with a `(Plane, quadric)` pair (both
//!   operand orders) for cylinder, sphere, and cone;
//! - `intersect_plane_analytic` with a `(Plane, plane)` pair for the line arm.
//!
//! For every returned sample point, evaluating each supporting surface at its
//! returned UV must recover the sample point (round-trip), UVs must lie in
//! the surfaces' canonical ranges, and line endpoints must respect the
//! documented finite half-extent.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::TAU;

use remus_math::analytic_intersection::{AnalyticSurface, intersect_analytic_analytic};
use remus_math::frame::Frame3;
use remus_math::nurbs::intersection::IntersectionPoint;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

const DEFAULT_TOL: f64 = 1e-9;

/// Evaluate the plane `normal . p = d` at its frame UV, using the same
/// deterministic frame the marcher uses (`origin = normal * d`).
fn eval_plane(normal: Vec3, d: f64, uv: (f64, f64)) -> Point3 {
    let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);
    let frame = Frame3::from_normal(origin, normal).unwrap();
    frame.origin + frame.x * uv.0 + frame.y * uv.1
}

/// Round-trip error of `pt` against the analytic surface's own UV.
fn round_trip_err(surface: &AnalyticSurface<'_>, uv: (f64, f64), pt: Point3) -> f64 {
    let q = match surface {
        AnalyticSurface::Cylinder(c) => c.evaluate(uv.0, uv.1),
        AnalyticSurface::Cone(c) => c.evaluate(uv.0, uv.1),
        AnalyticSurface::Sphere(s) => s.evaluate(uv.0, uv.1),
        AnalyticSurface::Torus(t) => t.evaluate(uv.0, uv.1),
        AnalyticSurface::Plane { normal, d } => eval_plane(*normal, *d, uv),
    };
    (q - pt).length()
}

/// Canonical u-range check for the periodic quadrics: `u` must wrap to
/// `[0, TAU)` so seam crossings stay consistent.
fn assert_canonical_u(surface: &AnalyticSurface<'_>, uv: (f64, f64), scale: f64) {
    let is_periodic = !matches!(surface, AnalyticSurface::Plane { .. });
    if is_periodic {
        assert!(
            uv.0 >= -1e-12 && uv.0 <= TAU + 1e-12,
            "u={} outside canonical [0, TAU] (scale {scale})",
            uv.0
        );
    }
    let _ = scale;
}

/// Assert every sample of every curve round-trips on BOTH supports.
fn assert_samples_carry_valid_params(
    a: &AnalyticSurface<'_>,
    b: &AnalyticSurface<'_>,
    points: &[Vec<IntersectionPoint>],
    scale: f64,
) {
    assert!(!points.is_empty(), "expected at least one curve");
    let mut total = 0;
    for curve_pts in points {
        assert!(
            !curve_pts.is_empty(),
            "marched curve must carry its samples"
        );
        for p in curve_pts {
            total += 1;
            let tol = DEFAULT_TOL * scale.max(1.0);
            let err_a = round_trip_err(a, p.param1, p.point);
            let err_b = round_trip_err(b, p.param2, p.point);
            assert!(
                err_a <= tol,
                "param1 {:?} does not recover {:?} on surface A (err {err_a:.3e}, tol {tol:.3e})",
                p.param1,
                p.point,
            );
            assert!(
                err_b <= tol,
                "param2 {:?} does not recover {:?} on surface B (err {err_b:.3e}, tol {tol:.3e})",
                p.param2,
                p.point,
            );
            assert_canonical_u(a, p.param1, scale);
            assert_canonical_u(b, p.param2, scale);
        }
    }
    assert!(total >= 2, "too few samples ({total})");
}

fn collect_point_lists(
    curves: &[remus_math::nurbs::intersection::IntersectionCurve],
) -> Vec<Vec<IntersectionPoint>> {
    curves.iter().map(|c| c.points.clone()).collect()
}

#[test]
fn plane_cylinder_circle_carries_both_uvs() {
    let cyl =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let (n, d) = (Vec3::new(0.0, 0.0, 1.0), 3.0);
    for (a, b) in [
        (
            AnalyticSurface::Plane { normal: n, d },
            AnalyticSurface::Cylinder(&cyl),
        ),
        (
            AnalyticSurface::Cylinder(&cyl),
            AnalyticSurface::Plane { normal: n, d },
        ),
    ] {
        let curves = intersect_analytic_analytic(a, b, 8).unwrap();
        assert_eq!(curves.len(), 1, "one circle section");
        assert_samples_carry_valid_params(&a, &b, &collect_point_lists(&curves), 3.0);
        for p in &curves[0].points {
            assert!((p.point.z() - 3.0).abs() < 1e-9);
            assert!((p.point.x().hypot(p.point.y()) - 2.0).abs() < 1e-9);
        }
    }
}

#[test]
fn plane_cylinder_placed_and_scaled_carries_both_uvs() {
    // Non-default placement (translated + tilted axis) and a larger scale.
    let axis = Vec3::new(1.0, 1.0, 1.0).normalize().unwrap();
    let cyl = CylindricalSurface::new(Point3::new(4.0, -7.0, 11.0), axis, 13.0).unwrap();
    // Plane perpendicular to the axis through the origin offset: a circle.
    let n = axis;
    let d = n.x() * 4.0 + n.y() * -7.0 + n.z() * 11.0 + 5.0;
    let a = AnalyticSurface::Plane { normal: n, d };
    let b = AnalyticSurface::Cylinder(&cyl);
    let curves = intersect_analytic_analytic(a, b, 8).unwrap();
    assert_eq!(curves.len(), 1, "one circle section");
    assert_samples_carry_valid_params(&a, &b, &collect_point_lists(&curves), 20.0);
}

#[test]
fn plane_sphere_circle_carries_both_uvs() {
    let sphere = SphericalSurface::new(Point3::new(1.0, -2.0, 5.0), 4.0).unwrap();
    let (n, d) = (Vec3::new(0.0, 0.0, 1.0), 6.0);
    for (a, b) in [
        (
            AnalyticSurface::Plane { normal: n, d },
            AnalyticSurface::Sphere(&sphere),
        ),
        (
            AnalyticSurface::Sphere(&sphere),
            AnalyticSurface::Plane { normal: n, d },
        ),
    ] {
        let curves = intersect_analytic_analytic(a, b, 8).unwrap();
        assert_eq!(curves.len(), 1, "one circle section");
        assert_samples_carry_valid_params(&a, &b, &collect_point_lists(&curves), 9.0);
    }
}

#[test]
fn plane_cone_ellipse_carries_both_uvs() {
    let cone = ConicalSurface::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let n = Vec3::new(0.3, 0.0, 1.0).normalize().unwrap();
    let d = n.z() * 5.0;
    for (a, b) in [
        (
            AnalyticSurface::Plane { normal: n, d },
            AnalyticSurface::Cone(&cone),
        ),
        (
            AnalyticSurface::Cone(&cone),
            AnalyticSurface::Plane { normal: n, d },
        ),
    ] {
        let curves = intersect_analytic_analytic(a, b, 8).unwrap();
        assert_eq!(curves.len(), 1, "one ellipse section");
        assert_samples_carry_valid_params(&a, &b, &collect_point_lists(&curves), 8.0);
    }
}

#[test]
fn plane_plane_line_respects_half_extent_and_uvs() {
    use remus_math::analytic_intersection::intersect_plane_analytic;
    // z=0 plane x y=0 plane -> the x-axis, finite segment +/-1.0.
    let curves = intersect_plane_analytic(
        AnalyticSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
        Vec3::new(0.0, 1.0, 0.0),
        0.0,
    )
    .unwrap();
    assert_eq!(curves.len(), 1);
    let pts = &curves[0].points;
    assert_eq!(pts.len(), 2, "line segment carries its two endpoints");
    let a = AnalyticSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1.0),
        d: 0.0,
    };
    let b = AnalyticSurface::Plane {
        normal: Vec3::new(0.0, 1.0, 0.0),
        d: 0.0,
    };
    assert_samples_carry_valid_params(&a, &b, std::slice::from_ref(pts), 1.0);
    // Finite-range semantics: endpoints at +/- line_half_extent (1.0 here)
    // along the line direction from the shared base point.
    let dir = (pts[1].point - pts[0].point).normalize().unwrap();
    assert!(dir.x().abs() > 1.0 - 1e-9, "line runs along x");
    let mid = Point3::new(
        (pts[0].point.x() + pts[1].point.x()) * 0.5,
        (pts[0].point.y() + pts[1].point.y()) * 0.5,
        (pts[0].point.z() + pts[1].point.z()) * 0.5,
    );
    for p in pts {
        let t = (p.point - mid).dot(dir).abs();
        assert!(
            (t - 1.0).abs() < 1e-9,
            "endpoint at |t|={t}, expected half-extent 1.0"
        );
    }
}

#[test]
fn converted_params_are_not_fabricated_zeros() {
    // Geometry whose section stays far from any surface's (0,0) evaluation,
    // so a fabricated (0,0) UV could never round-trip.
    let cyl = CylindricalSurface::new(
        Point3::new(50.0, -30.0, 12.0),
        Vec3::new(0.0, 0.0, 1.0),
        7.0,
    )
    .unwrap();
    let (n, d) = (Vec3::new(0.0, 0.0, 1.0), -40.0);
    let a = AnalyticSurface::Plane { normal: n, d };
    let b = AnalyticSurface::Cylinder(&cyl);
    let curves = intersect_analytic_analytic(a, b, 8).unwrap();
    assert_eq!(curves.len(), 1);
    let mut saw_nonzero = false;
    for p in &curves[0].points {
        // The cylinder's (0,0) evaluates to origin + r*x_axis: nowhere near
        // this translated section, so a zero UV is provably fabricated here.
        let origin_eval = cyl.evaluate(0.0, 0.0);
        assert!(
            (origin_eval - p.point).length() > 1.0,
            "test setup broken: section touches the (0,0) evaluation"
        );
        saw_nonzero |= p.param1 != (0.0, 0.0) || p.param2 != (0.0, 0.0);
        let tol = DEFAULT_TOL * 60.0;
        // Operand order here is (Plane, Cylinder): param1 is the plane UV,
        // param2 the cylinder UV.
        assert!((cyl.evaluate(p.param2.0, p.param2.1) - p.point).length() <= tol);
        assert!((eval_plane(n, d, p.param1) - p.point).length() <= tol);
    }
    assert!(saw_nonzero, "all converted params are still (0,0)");
}

#[test]
fn plane_torus_sampled_chains_carry_both_uvs() {
    // Plane x torus has no closed form: the conversion's `Points` arm refits
    // sampled chains. Those samples must still carry valid UVs on both
    // supports. z=0 through an R=5, r=1 torus gives the R-r and R+r rings.
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0, 1.0).unwrap();
    let (n, d) = (Vec3::new(0.0, 0.0, 1.0), 0.0);
    for (a, b) in [
        (
            AnalyticSurface::Plane { normal: n, d },
            AnalyticSurface::Torus(&torus),
        ),
        (
            AnalyticSurface::Torus(&torus),
            AnalyticSurface::Plane { normal: n, d },
        ),
    ] {
        let curves = intersect_analytic_analytic(a, b, 12).unwrap();
        assert_eq!(curves.len(), 2, "two concentric section rings");
        assert_samples_carry_valid_params(&a, &b, &collect_point_lists(&curves), 6.0);
        // Both rings lie in the plane and on the tube.
        for p in curves.iter().flat_map(|c| &c.points) {
            assert!(
                p.point.z().abs() < 1e-9,
                "ring off the plane: {:?}",
                p.point
            );
            let rho = p.point.x().hypot(p.point.y());
            let tube = ((rho - 5.0).powi(2) + p.point.z().powi(2)).sqrt();
            assert!(
                (tube - 1.0).abs() < 1e-6,
                "ring off the tube: {:?}",
                p.point
            );
        }
    }
}
