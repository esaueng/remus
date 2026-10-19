#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point3, Vec3};

/// The reduction reads a forward face as counter-clockwise in `(u, v)`, which
/// holds only while `∂P/∂u × ∂P/∂v` points along the surface's own outward
/// normal (the one face orientation is defined against), with the magnitude
/// the kernel table integrates.
#[test]
fn natural_normal_is_the_outward_normal_with_the_tabulated_jacobian() {
    let origin = Point3::new(1.0, -2.0, 0.5);
    let axis = Vec3::new(0.3, -0.4, 0.86);
    let cylinder = CylindricalSurface::new(origin, axis, 2.5).unwrap();
    let cone = ConicalSurface::new(origin, axis, 0.6).unwrap();
    let sphere = SphericalSurface::with_axis(origin, 3.0, axis).unwrap();
    let torus = ToroidalSurface::with_axis(origin, 5.0, 1.5, axis).unwrap();
    for &(u, v) in &[(0.3, 0.7), (2.0, 1.1), (4.5, 0.2), (5.9, 2.8)] {
        let check = |s: &dyn ParametricSurface, u: f64, v: f64, jacobian: f64| {
            let (su, sv) = s.partials(u, v);
            let n = su.cross(sv);
            assert!(
                (n.length() - jacobian).abs() <= 1e-12 * jacobian.max(1.0),
                "{} vs {jacobian} at ({u}, {v})",
                n.length()
            );
            assert!(
                n.normalize()
                    .unwrap()
                    .dot(s.normal(u, v).normalize().unwrap())
                    > 1.0 - 1e-12
            );
        };
        check(&cylinder, u, v, 2.5);
        check(&cone, u, v, v * 0.6_f64.cos());
        let phi = 0.5f64.mul_add(v, -0.6);
        check(&sphere, u, phi, 9.0 * phi.cos());
        check(&torus, u, v, 1.5 * 1.5f64.mul_add(v.cos(), 5.0));
    }
}

/// A meridian arc through the chart's pole carries no `du` on either side
/// of the pole, yet `u` jumps by a half turn there: the panel continuity
/// certificate must refuse it rather than integrate a wrong area.
#[test]
fn a_hidden_pole_crossing_is_refused() {
    use remus_math::curves::Circle3D;
    use remus_topology::edge::EdgeCurve;

    let center = Point3::new(0.0, 0.0, 0.0);
    let chart = super::Chart {
        origin: center,
        x: Vec3::new(1.0, 0.0, 0.0),
        y: Vec3::new(0.0, 1.0, 0.0),
        z: Vec3::new(0.0, 0.0, 1.0),
        kernel: super::Kernel::Sphere {
            r: 2.0,
            quadrature_axis: false,
        },
        scale: 2.0,
    };
    // The great circle in the x-z plane passes the pole at t = π/2.
    let meridian = EdgeCurve::Circle(
        Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, -1.0, 0.0),
            2.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let mut sum = super::WireSum {
        area_term: 0.0,
        winding: 0.0,
        tube_angle: None,
        panels: 0,
    };
    let (a, b) = (0.3, 2.9);
    assert!(!super::integrate_panel(
        &chart, &meridian, center, center, a, b, 1e-12, 0, &mut sum
    ));
    // The same arc short of the pole integrates.
    let mut sum = super::WireSum {
        area_term: 0.0,
        winding: 0.0,
        tube_angle: None,
        panels: 0,
    };
    assert!(super::integrate_panel(
        &chart, &meridian, center, center, a, 1.2, 1e-12, 0, &mut sum
    ));
}
