//! Curvature of NURBS surfaces via the first and second fundamental forms.
//!
//! Thin bridge between [`NurbsSurface::derivatives`] and the generic
//! fundamental-form solver in [`crate::curvature`]. The reference normal is
//! the parametric normal `(Xu × Xv)/|Xu × Xv|` — the same normal
//! `NurbsSurface::normal` reports — so the sign convention documented in
//! [`crate::curvature`] (positive for convex-outward) applies unchanged.

use crate::MathError;
use crate::curvature::{SurfaceCurvature, curvature_from_fundamental_forms};
use crate::nurbs::surface::NurbsSurface;

/// Principal curvatures and directions of a NURBS surface at `(u, v)`.
///
/// Evaluates the surface derivatives up to second order and solves the
/// 2×2 generalized eigenproblem of the fundamental forms. Curvatures are
/// sorted `k1 >= k2`; `directions` is `None` at umbilic points.
///
/// # Errors
///
/// Returns [`MathError::SingularMatrix`] when the first fundamental form
/// degenerates at `(u, v)` (zero-area parametrization, e.g. a sphere pole or
/// a collapsed control row).
pub fn surface_curvature(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
) -> Result<SurfaceCurvature, MathError> {
    let d = surface.derivatives(u, v, 2);
    curvature_from_fundamental_forms(d[1][0], d[0][1], d[2][0], d[1][1], d[0][2])
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::surface_curvature;
    use crate::curvature::{
        cone_principal_curvatures, curvature_from_fundamental_forms, cylinder_principal_curvatures,
        sphere_principal_curvatures, torus_principal_curvatures,
    };
    use crate::nurbs::surface::NurbsSurface;
    use crate::surfaces::{ConicalSurface, CylindricalSurface, ToroidalSurface};
    use crate::vec::{Point3, Vec3};

    const ORIGIN: Point3 = Point3::new(0.0, 0.0, 0.0);
    const Z_AXIS: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    /// Exact rational NURBS sphere: a degree-(2, 2) surface of revolution.
    ///
    /// Profile: half circle in the xz-plane from south pole to north pole
    /// (5 control points, 2 quadratic spans). Revolution: full circle in 4
    /// 90° spans (9 control points). Weights multiply per factor
    /// (`√2/2` at every arc-interior control point). Unlike the sampled
    /// `SphericalSurface::to_nurbs` approximation, this representation is
    /// geometrically exact, so its curvature is a differential oracle for
    /// the fundamental-form machinery.
    fn exact_sphere_nurbs(radius: f64) -> NurbsSurface {
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        // Profile control points (x, z) and weights, south → north: two
        // 90° quadratic spans. Each span's interior control point sits at
        // the tangent intersection, radius R/cos(45°) at the span's mid
        // angle, with weight cos(45°) = √2/2.
        let profile: [(f64, f64, f64); 5] = [
            (0.0, -radius, 1.0),
            (radius, -radius, s2),
            (radius, 0.0, 1.0),
            (radius, radius, s2),
            (0.0, radius, 1.0),
        ];
        // Revolution weights for four 90° spans; control points sit at the
        // span boundaries i·45° (odd i carry the √2/2 weight, even i are on
        // the surface).
        let rev_w = [1.0, s2, 1.0, s2, 1.0, s2, 1.0, s2, 1.0];
        let rev_angles: [f64; 9] = std::array::from_fn(|i| i as f64 * std::f64::consts::FRAC_PI_4);

        let mut cps: Vec<Vec<Point3>> = Vec::with_capacity(9);
        let mut weights: Vec<Vec<f64>> = Vec::with_capacity(9);
        for (i, &phi) in rev_angles.iter().enumerate() {
            let (sin_phi, cos_phi) = phi.sin_cos();
            // Odd revolution rows sit at the tangent-intersection radius
            // (x scaled by 1/cos(45°)), matching their cos(45°) weight —
            // the same construction as the exact rational cylinder.
            let rho = if i % 2 == 0 {
                1.0
            } else {
                std::f64::consts::SQRT_2
            };
            let mut row = Vec::with_capacity(5);
            let mut w_row = Vec::with_capacity(5);
            for &(px, pz, w) in &profile {
                row.push(Point3::new(px * rho * cos_phi, px * rho * sin_phi, pz));
                w_row.push(w * rev_w[i]);
            }
            cps.push(row);
            weights.push(w_row);
        }

        let knots_u = vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ];
        let knots_v = vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0];
        NurbsSurface::new(2, 2, knots_u, knots_v, cps, weights).unwrap()
    }

    #[test]
    fn exact_sphere_nurbs_matches_analytic_curvature() {
        let radius = 2.5;
        let nurbs = exact_sphere_nurbs(radius);
        let expected = sphere_principal_curvatures(radius);

        // Sample away from poles (degenerate parametrization) and the seam.
        for &u in &[0.1, 0.35, 0.6, 0.9] {
            for &v in &[0.15, 0.4, 0.5, 0.75, 0.85] {
                let curv = surface_curvature(&nurbs, u, v).unwrap();
                assert!(
                    (curv.k1 - expected.k1).abs() < 1e-9,
                    "k1 at ({u},{v}): {} vs {}",
                    curv.k1,
                    expected.k1
                );
                assert!(
                    (curv.k2 - expected.k2).abs() < 1e-9,
                    "k2 at ({u},{v}): {} vs {}",
                    curv.k2,
                    expected.k2
                );
                // A sphere is umbilic: directions must not be fabricated.
                assert!(
                    curv.directions.is_none(),
                    "sphere must be umbilic at ({u},{v})"
                );
                assert!((curv.gaussian() - 1.0 / (radius * radius)).abs() < 1e-9);
                assert!((curv.mean() - 1.0 / radius).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn exact_sphere_nurbs_degenerate_at_poles() {
        let nurbs = exact_sphere_nurbs(1.0);
        // v = 0 and v = 1 are the poles: Xu vanishes, first form degenerates.
        assert!(surface_curvature(&nurbs, 0.3, 0.0).is_err());
        assert!(surface_curvature(&nurbs, 0.3, 1.0).is_err());
    }

    #[test]
    fn exact_cylinder_nurbs_matches_analytic_curvature() {
        let radius = 3.0;
        let cyl = CylindricalSurface::new(ORIGIN, Z_AXIS, radius).unwrap();
        // The math-layer conversion is the exact rational cylinder.
        let nurbs = cyl.to_nurbs(0.0, 4.0).unwrap();
        let expected = cylinder_principal_curvatures(radius);

        for &u in &[0.05, 0.4, 0.8] {
            for &v in &[0.2, 0.7] {
                let curv = surface_curvature(&nurbs, u, v).unwrap();
                assert!((curv.k1 - expected.k1).abs() < 1e-9);
                assert!((curv.k2 - expected.k2).abs() < 1e-9);
                let (d1, d2) = curv.directions.unwrap();
                // k1 is circumferential: tangent to the circle (no axial part).
                assert!(
                    d1.dot(Z_AXIS).abs() < 1e-9,
                    "k1 dir must be circumferential"
                );
                // k2 is axial: parallel to the axis.
                assert!((d2.dot(Z_AXIS) - 1.0).abs() < 1e-9, "k2 dir must be axial");
            }
        }
    }

    #[test]
    fn nurbs_curvature_matches_fundamental_forms_of_analytic_parametrizations() {
        // Cross-validation: run the analytic surfaces' own parametrizations
        // through the generic fundamental-form path (second derivatives via
        // central finite differences of `evaluate`) and compare with the
        // closed forms. This verifies cone and torus formulas independently
        // of the code that produced them.
        let h = 1e-5;

        let cone = ConicalSurface::new(ORIGIN, Z_AXIS, 0.6).unwrap();
        for &v in &[0.8, 2.0, 5.0] {
            let fds = fd_second_derivatives(|u, vv| cone.evaluate(u, vv), 0.7, v, h);
            let generic =
                curvature_from_fundamental_forms(fds.0, fds.1, fds.2, fds.3, fds.4).unwrap();
            let exact = cone_principal_curvatures(0.6, v).unwrap();
            assert!(
                (generic.k1 - exact.k1).abs() < 1e-5,
                "cone k1: {} vs {}",
                generic.k1,
                exact.k1
            );
            assert!(generic.k2.abs() < 1e-5, "cone k2: {}", generic.k2);
        }

        let torus = ToroidalSurface::new(ORIGIN, 4.0, 1.0).unwrap();
        for &v in &[0.0, 1.1, 2.0, 3.5, std::f64::consts::PI] {
            let fds = fd_second_derivatives(|u, vv| torus.evaluate(u, vv), 0.7, v, h);
            let generic =
                curvature_from_fundamental_forms(fds.0, fds.1, fds.2, fds.3, fds.4).unwrap();
            let exact = torus_principal_curvatures(4.0, 1.0, v).unwrap();
            assert!(
                (generic.k1 - exact.k1).abs() < 1e-5,
                "torus k1 at v={v}: {} vs {}",
                generic.k1,
                exact.k1
            );
            assert!(
                (generic.k2 - exact.k2).abs() < 1e-5,
                "torus k2 at v={v}: {} vs {}",
                generic.k2,
                exact.k2
            );
        }
    }

    /// Central finite-difference first and second derivatives of a
    /// parametrization, returned as `(xu, xv, xuu, xuv, xvv)`.
    fn fd_second_derivatives(
        f: impl Fn(f64, f64) -> Point3,
        u: f64,
        v: f64,
        h: f64,
    ) -> (Vec3, Vec3, Vec3, Vec3, Vec3) {
        let sub = |p: Point3, q: Point3| Vec3::new(q.x() - p.x(), q.y() - p.y(), q.z() - p.z());
        let vsub = |a: Vec3, b: Vec3| Vec3::new(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
        let scale = |d: Vec3, s: f64| Vec3::new(d.x() * s, d.y() * s, d.z() * s);

        let p_up = f(u, v + h);
        let p_dn = f(u, v - h);
        let p_rt = f(u + h, v);
        let p_lf = f(u - h, v);
        let p_c = f(u, v);
        let p_ur = f(u + h, v + h);
        let p_ul = f(u - h, v + h);
        let p_lr = f(u + h, v - h);
        let p_ll = f(u - h, v - h);

        let xu = scale(sub(p_lf, p_rt), 1.0 / (2.0 * h));
        let xv = scale(sub(p_dn, p_up), 1.0 / (2.0 * h));
        let xuu = scale(vsub(sub(p_c, p_lf), sub(p_rt, p_c)), 1.0 / (h * h));
        let xvv = scale(vsub(sub(p_c, p_dn), sub(p_up, p_c)), 1.0 / (h * h));
        let xuv = scale(
            Vec3::new(
                (p_ur.x() - p_ul.x() - p_lr.x() + p_ll.x()) / (4.0 * h * h),
                (p_ur.y() - p_ul.y() - p_lr.y() + p_ll.y()) / (4.0 * h * h),
                (p_ur.z() - p_ul.z() - p_lr.z() + p_ll.z()) / (4.0 * h * h),
            ),
            1.0,
        );
        (xu, xv, xuu, xuv, xvv)
    }
}
