//! Independent implicit-equation oracles for the equal parallel-cone section.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::analytic_intersection::exact_equal_parallel_cone_hyperbola;
use remus_math::surfaces::ConicalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};

#[test]
fn literal_frustum_section_reaches_the_two_bottom_rim_crossings() {
    let first = ConicalSurface::new(
        Point3::new(0.0, 0.0, 12.375),
        Vec3::new(0.0, 0.0, -1.0),
        (11.0_f64 / 4.0).atan(),
    )
    .unwrap();
    let second = first.translated(Vec3::new(7.0, 0.0, 0.0));
    let section = exact_equal_parallel_cone_hyperbola(&first, &second, Tolerance::new())
        .unwrap()
        .unwrap();
    // The radical plane is x=7/2. A bottom radius9/2 leaves y²=8;
    // the nearest common generator point is at height11/4.
    assert!((section.evaluate(0.0) - Point3::new(3.5, 0.0, 2.75)).length() < 1e-13);
    let limit = (9.0_f64 / 7.0).acosh();
    for sign in [-1.0, 1.0] {
        let point = section.evaluate(sign * limit);
        assert!((point.x() - 3.5).abs() < 1e-13);
        assert!((point.y().abs() - 8.0_f64.sqrt()).abs() < 1e-13);
        assert!(point.z().abs() < 1e-13);
    }
}

#[test]
fn shifted_apices_and_rotated_axes_satisfy_both_positive_nappe_equations() {
    for axis in [
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(2.0, -3.0, 9.0).normalize().unwrap(),
    ] {
        let transverse = axis.cross(Vec3::new(1.0, 0.0, 0.0)).normalize().unwrap();
        for scale in [1e-3, 1.0, 1e3] {
            for shift in [-2.0, 0.0, 2.0] {
                let first = ConicalSurface::new(
                    Point3::new(13.0 * scale, -7.0 * scale, 12.375 * scale),
                    axis,
                    (11.0_f64 / 4.0).atan(),
                )
                .unwrap();
                let second = first.translated((transverse * 7.0 + first.axis() * shift) * scale);
                for (a, b) in [(&first, &second), (&second, &first)] {
                    let section = exact_equal_parallel_cone_hyperbola(a, b, Tolerance::new())
                        .unwrap()
                        .unwrap();
                    for t in [-1.2, -0.7, 0.0, 0.7, 1.2] {
                        let point = section.evaluate(t);
                        for cone in [&first, &second] {
                            let offset = point - cone.apex();
                            let axial = offset.dot(cone.axis());
                            let radial = (offset - cone.axis() * axial).length();
                            let residual =
                                radial * cone.half_angle().sin() - axial * cone.half_angle().cos();
                            assert!(axial > 0.0, "scale={scale} shift={shift} t={t}");
                            assert!(
                                residual.abs() < 2e-12 * scale,
                                "scale={scale} shift={shift} t={t}: {residual}"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn structural_nonidentity_and_unresolved_radical_planes_decline() {
    let angle = (11.0_f64 / 4.0).atan();
    let first = ConicalSurface::new(
        Point3::new(0.0, 0.0, 12.375),
        Vec3::new(0.0, 0.0, -1.0),
        angle,
    )
    .unwrap();
    for (transverse, axial) in [
        (0.0, 0.0),
        (7.0, 19.25),
        (7.0, 25.0),
        (7.0, 19.25 - 1e-8),
        (1e-8, 0.0),
    ] {
        let second = first.translated(Vec3::new(transverse, 0.0, -axial));
        assert!(
            exact_equal_parallel_cone_hyperbola(&first, &second, Tolerance::new())
                .unwrap()
                .is_none()
        );
    }
    for second in [
        ConicalSurface::new(Point3::new(7.0, 0.0, 12.375), first.axis(), angle.next_up()).unwrap(),
        ConicalSurface::new(
            Point3::new(7.0, 0.0, 12.375),
            Vec3::new(1e-14, 0.0, -1.0),
            angle,
        )
        .unwrap(),
        ConicalSurface::new(Point3::new(7.0, 0.0, 12.375), -first.axis(), angle).unwrap(),
    ] {
        assert!(
            exact_equal_parallel_cone_hyperbola(&first, &second, Tolerance::new())
                .unwrap()
                .is_none()
        );
    }
    let second = first.translated(Vec3::new(7.0, 0.0, 0.0));
    for linear in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let tolerance = Tolerance {
            linear,
            ..Tolerance::new()
        };
        assert!(
            exact_equal_parallel_cone_hyperbola(&first, &second, tolerance)
                .unwrap()
                .is_none()
        );
    }
}
