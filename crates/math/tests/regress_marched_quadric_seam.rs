//! A transverse cone-sphere seam closes once and remains on both carriers between fit samples.
#![allow(clippy::unwrap_used, clippy::panic)]
use remus_math::{
    analytic_intersection::{AnalyticSurface, intersect_analytic_analytic_bounded},
    surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface},
    vec::{Point3, Vec3},
};

#[test]
fn cone_sphere_seam_is_closed_and_supported_in_both_operand_orders() {
    for scale in [0.1, 1.0, 10.0] {
        let apex = Point3::new(0.0, 0.0, 18.0 * scale);
        let center = Point3::new(2.0 * scale, 0.0, 6.0 * scale);
        let angle = 3.0_f64.atan();
        let cone = ConicalSurface::new(apex, Vec3::new(0.0, 0.0, -1.0), angle).unwrap();
        let sphere = SphericalSurface::new(center, 4.0 * scale).unwrap();
        let extent = Some((6.0 * scale / angle.sin(), 18.0 * scale / angle.sin()));
        for reverse in [false, true] {
            let (a, b, va, vb) = if reverse {
                (
                    AnalyticSurface::Sphere(&sphere),
                    AnalyticSurface::Cone(&cone),
                    None,
                    extent,
                )
            } else {
                (
                    AnalyticSurface::Cone(&cone),
                    AnalyticSurface::Sphere(&sphere),
                    extent,
                    None,
                )
            };
            let curves = intersect_analytic_analytic_bounded(a, b, 32, va, vb).unwrap();
            assert_eq!(curves.len(), 1, "scale={scale}, reverse={reverse}");
            let curve = &curves[0].curve;
            let (lo, hi) = curve.domain();
            assert!((curve.evaluate(lo) - curve.evaluate(hi)).length() < 1e-9);
            for i in 0..=2048 {
                let p = curve.evaluate((hi - lo).mul_add(f64::from(i) / 2048.0, lo));
                let sphere_residual = ((p - center).length() - 4.0 * scale).abs();
                let cone_residual =
                    (p.x().hypot(p.y()) - (18.0 * scale - p.z()) / 3.0).abs() * angle.sin();
                assert!(
                    sphere_residual < 2e-9,
                    "sphere residual={sphere_residual}, scale={scale}, reverse={reverse}"
                );
                assert!(
                    cone_residual < 2e-9,
                    "cone residual={cone_residual}, scale={scale}, reverse={reverse}"
                );
            }
        }
    }
}

#[test]
fn offset_sphere_cylinder_has_two_supported_closed_seams() {
    for scale in [0.1, 1.0, 10.0] {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 6.0 * scale).unwrap();
        let cylinder = CylindricalSurface::new(
            Point3::new(2.0 * scale, 0.0, -10.0 * scale),
            Vec3::new(0.0, 0.0, 1.0),
            3.0 * scale,
        )
        .unwrap();
        for reverse in [false, true] {
            let (a, b, va, vb) = if reverse {
                (
                    AnalyticSurface::Cylinder(&cylinder),
                    AnalyticSurface::Sphere(&sphere),
                    Some((0.0, 20.0 * scale)),
                    None,
                )
            } else {
                (
                    AnalyticSurface::Sphere(&sphere),
                    AnalyticSurface::Cylinder(&cylinder),
                    None,
                    Some((0.0, 20.0 * scale)),
                )
            };
            let curves = intersect_analytic_analytic_bounded(a, b, 32, va, vb).unwrap();
            assert_eq!(curves.len(), 2, "scale={scale}, reverse={reverse}");
            for result in curves {
                let curve = result.curve;
                let (lo, hi) = curve.domain();
                assert!((curve.evaluate(lo) - curve.evaluate(hi)).length() < 1e-9);
                for i in 0..=4096 {
                    let p = curve.evaluate((hi - lo).mul_add(f64::from(i) / 4096.0, lo));
                    let sphere_error =
                        ((p - Point3::new(0.0, 0.0, 0.0)).length() - 6.0 * scale).abs();
                    let cylinder_error = ((p.x() - 2.0 * scale).hypot(p.y()) - 3.0 * scale).abs();
                    assert!(
                        sphere_error < 2e-9 && cylinder_error < 2e-9,
                        "scale={scale}, reverse={reverse}, sphere={sphere_error}, cylinder={cylinder_error}"
                    );
                }
            }
        }
    }
}

#[test]
fn offset_torus_sphere_seams_close_before_the_march_budget() {
    use remus_math::surfaces::ToroidalSurface;
    for scale in [0.1, 1.0, 10.0] {
        let torus =
            ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 6.0 * scale, 2.0 * scale).unwrap();
        let center = Point3::new(5.0 * scale, 0.0, scale);
        let sphere = SphericalSurface::new(center, 3.0 * scale).unwrap();
        for reverse in [false, true] {
            let (a, b) = if reverse {
                (
                    AnalyticSurface::Sphere(&sphere),
                    AnalyticSurface::Torus(&torus),
                )
            } else {
                (
                    AnalyticSurface::Torus(&torus),
                    AnalyticSurface::Sphere(&sphere),
                )
            };
            let curves = intersect_analytic_analytic_bounded(a, b, 32, None, None)
                .unwrap_or_else(|error| panic!("scale={scale}, reverse={reverse}: {error}"));
            assert_eq!(curves.len(), 1, "scale={scale}, reverse={reverse}");
            for result in curves {
                let curve = result.curve;
                let (lo, hi) = curve.domain();
                let gap = (curve.evaluate(lo) - curve.evaluate(hi)).length();
                assert!(
                    gap < 1e-9,
                    "open seam gap={gap}, scale={scale}, reverse={reverse}"
                );
                for i in 0..=4096 {
                    let p = curve.evaluate((hi - lo).mul_add(f64::from(i) / 4096.0, lo));
                    let sphere_error = ((p - center).length() - 3.0 * scale).abs();
                    let torus_error =
                        ((p.x().hypot(p.y()) - 6.0 * scale).hypot(p.z()) - 2.0 * scale).abs();
                    assert!(
                        sphere_error < 2e-9 && torus_error < 2e-9,
                        "scale={scale}, reverse={reverse}, sphere={sphere_error}, torus={torus_error}"
                    );
                }
            }
        }
    }
}

#[test]
fn exhausted_torus_march_refuses_instead_of_returning_open_fragments() {
    let torus =
        remus_math::surfaces::ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 6000.0, 2000.0)
            .unwrap();
    let sphere = SphericalSurface::new(Point3::new(5000.0, 0.0, 1000.0), 3000.0).unwrap();
    let result = intersect_analytic_analytic_bounded(
        AnalyticSurface::Torus(&torus),
        AnalyticSurface::Sphere(&sphere),
        32,
        None,
        None,
    );
    assert!(matches!(
        result,
        Err(remus_math::MathError::ConvergenceFailure { iterations: 500 })
    ));
}
