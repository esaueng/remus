//! Regression coverage for the direct `intersect_plane_*` routines' plane-side
//! surface parameters (`crates/math/src/analytic_intersection.rs`).
//!
//! The `exact_to_marched` conversion (covered by
//! `regress_marched_surface_params.rs`) carries valid UVs on both supports,
//! but the direct plane×quadric routines — `intersect_plane_cylinder`,
//! `intersect_plane_sphere`, `intersect_plane_cone`, `intersect_plane_torus`,
//! and the `intersect_plane_analytic` quadric arms that delegate to them —
//! emitted `param2 = (0.0, 0.0)` for the plane support on every sample. A
//! fabricated `(0,0)` never round-trips except by accident, so any consumer
//! reading the plane UV (e.g. the marcher's projection helpers, which treat
//! `param1`/`param2` as surface UVs) silently leaves the surface.
//!
//! These tests pin the corrected guarantee through both the direct entries
//! and the `intersect_plane_analytic` dispatch: every sample's `param2`
//! evaluates back to the sample point on the cutting plane, in the same
//! deterministic `Frame3::from_normal` frame (`origin = normal * d`) the
//! marcher uses. `param1` (quadric-native UV) round-trips are pinned too,
//! except for `intersect_plane_sphere`, whose `param1 = (theta, 0.0)` is the
//! pre-existing section-circle parameter convention, not a sphere UV — that
//! older semantic is intentionally left untouched here.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::analytic_intersection::{
    AnalyticSurface, intersect_plane_analytic, intersect_plane_cone, intersect_plane_cylinder,
    intersect_plane_sphere, intersect_plane_torus,
};
use remus_math::frame::Frame3;
use remus_math::nurbs::intersection::IntersectionCurve;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

const TOL: f64 = 1e-9;

/// Evaluate the plane `normal . p = d` at its frame UV, using the same
/// deterministic frame the marcher uses (`origin = normal * d`).
fn eval_plane(normal: Vec3, d: f64, uv: (f64, f64)) -> Point3 {
    let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);
    let frame = Frame3::from_normal(origin, normal).unwrap();
    frame.origin + frame.x * uv.0 + frame.y * uv.1
}

/// Assert every sample's plane UV (`param2`) evaluates back to the sample.
fn assert_plane_uvs(normal: Vec3, d: f64, curves: &[IntersectionCurve]) {
    assert!(!curves.is_empty(), "expected at least one curve");
    let mut total = 0;
    for curve in curves {
        assert!(!curve.points.is_empty(), "curve must carry its samples");
        for p in &curve.points {
            total += 1;
            // A fabricated (0,0) only survives when the section happens to
            // cross the plane frame's origin — the chosen geometries keep
            // their sections far from there, so zeros fail loudly here.
            let err = (eval_plane(normal, d, p.param2) - p.point).length();
            assert!(
                err <= TOL,
                "param2 {:?} does not recover {:?} on the plane (err {err:.3e})",
                p.param2,
                p.point,
            );
        }
    }
    assert!(total >= 2, "too few samples ({total})");
}

#[test]
fn direct_plane_cylinder_carries_plane_uv() {
    let cyl =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    // Tilted plane: an ellipse section, kept clear of the frame origin.
    let normal = Vec3::new(0.3, 0.0, 1.0).normalize().unwrap();
    let d = normal.z() * 7.0;
    let curves = intersect_plane_cylinder(&cyl, normal, d).unwrap();
    assert_plane_uvs(normal, d, &curves);
    // param1 is the native cylinder UV: pin the existing behavior.
    for p in curves.iter().flat_map(|c| &c.points) {
        let err = (cyl.evaluate(p.param1.0, p.param1.1) - p.point).length();
        assert!(err <= TOL, "cylinder param1 does not round-trip");
    }
}

#[test]
fn direct_plane_sphere_carries_plane_uv() {
    let sphere = SphericalSurface::new(Point3::new(1.0, -2.0, 5.0), 4.0).unwrap();
    let (normal, d) = (Vec3::new(0.0, 0.0, 1.0), 6.0);
    let curves = intersect_plane_sphere(&sphere, normal, d).unwrap();
    assert_plane_uvs(normal, d, &curves);
    // NOTE: param1 here is the section-circle angle `(theta, 0.0)`, not a
    // sphere UV — a pre-existing convention this fix deliberately preserves.
}

#[test]
fn direct_plane_cone_carries_plane_uv() {
    let cone = ConicalSurface::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let normal = Vec3::new(0.3, 0.0, 1.0).normalize().unwrap();
    let d = normal.z() * 5.0;
    let curves = intersect_plane_cone(&cone, normal, d).unwrap();
    assert_plane_uvs(normal, d, &curves);
    for p in curves.iter().flat_map(|c| &c.points) {
        let err = (cone.evaluate(p.param1.0, p.param1.1) - p.point).length();
        assert!(err <= TOL, "cone param1 does not round-trip");
    }
}

#[test]
fn direct_plane_torus_carries_plane_uv() {
    // z=0 through an R=5, r=1 torus: the R-r and R+r section rings.
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0, 1.0).unwrap();
    let (normal, d) = (Vec3::new(0.0, 0.0, 1.0), 0.0);
    let curves = intersect_plane_torus(&torus, normal, d).unwrap();
    assert_plane_uvs(normal, d, &curves);
    for p in curves.iter().flat_map(|c| &c.points) {
        let err = (torus.evaluate(p.param1.0, p.param1.1) - p.point).length();
        assert!(err <= TOL, "torus param1 does not round-trip");
    }
}

#[test]
fn plane_analytic_dispatch_carries_plane_uv() {
    // The public `intersect_plane_analytic` quadric arms delegate to the
    // direct routines, so the dispatch inherits their plane UVs.
    let cyl =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let sphere = SphericalSurface::new(Point3::new(1.0, -2.0, 5.0), 4.0).unwrap();
    let cone = ConicalSurface::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0, 1.0).unwrap();
    let tilted = Vec3::new(0.3, 0.0, 1.0).normalize().unwrap();
    let cases: Vec<(AnalyticSurface<'_>, Vec3, f64)> = vec![
        (AnalyticSurface::Cylinder(&cyl), tilted, tilted.z() * 7.0),
        (
            AnalyticSurface::Sphere(&sphere),
            Vec3::new(0.0, 0.0, 1.0),
            6.0,
        ),
        (AnalyticSurface::Cone(&cone), tilted, tilted.z() * 5.0),
        (
            AnalyticSurface::Torus(&torus),
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        ),
    ];
    for (surface, normal, d) in &cases {
        let curves = intersect_plane_analytic(*surface, *normal, *d).unwrap();
        assert_plane_uvs(*normal, *d, &curves);
    }
}
