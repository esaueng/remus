//! A transverse cone-sphere seam closes once and remains on both carriers between fit samples.
#![allow(clippy::unwrap_used)]
use remus_math::{
    analytic_intersection::{AnalyticSurface, intersect_analytic_analytic_bounded},
    surfaces::{ConicalSurface, SphericalSurface},
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
