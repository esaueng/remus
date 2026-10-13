//! Regression oracles for analytic witnesses and collapsed curve ranges.
#![allow(clippy::unwrap_used)]
use remus_geometry::extrema::{curve_to_curve, point_to_cone};
use remus_math::{
    nurbs::NurbsCurve,
    surfaces::ConicalSurface,
    vec::{Point3, Vec3},
};

#[test]
fn cone_axis_projection_must_be_on_carrier() {
    let c = ConicalSurface::new(
        Point3::new(0., 0., 0.),
        Vec3::new(0., 0., 1.),
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();
    let p = point_to_cone(Point3::new(0., 0., 2.), &c);
    assert!((p.distance - 2f64.sqrt()).abs() < 1e-10);
    assert!(
        (p.point - c.evaluate(p.u, p.v)).length() < 1e-10,
        "returned {:?}, evaluated {:?}, distance {} (expected sqrt(2))",
        p.point,
        c.evaluate(p.u, p.v),
        p.distance
    );
}

#[test]
fn single_degenerate_curve_range_still_projects_other_curve() {
    let a = NurbsCurve::new(
        1,
        vec![0., 0., 1., 1.],
        vec![Point3::new(0., 0., 0.), Point3::new(1., 0., 0.)],
        vec![1., 1.],
    )
    .unwrap();
    let b = NurbsCurve::new(
        1,
        vec![0., 0., 10., 10.],
        vec![Point3::new(-5., 1., 0.), Point3::new(5., 1., 0.)],
        vec![1., 1.],
    )
    .unwrap();
    let r = curve_to_curve(&a, (0., 0.), &b, (0., 10.));
    assert!(
        (r.distance - 1.).abs() < 1e-10,
        "returned {} at ({},{}) instead of distance 1 at (0,5)",
        r.distance,
        r.param_a,
        r.param_b
    );
}

#[test]
fn cone_axial_witnesses_survive_rotation_translation_and_angle_changes() {
    let apex = Point3::new(17., -29., 41.);
    let axis = Vec3::new(1., 2., 3.).normalize().unwrap();
    for angle in [0.2_f64, 0.7, 1.2] {
        let cone = ConicalSurface::new(apex, axis, angle).unwrap();
        let p = point_to_cone(apex + axis * 5., &cone);
        assert!((p.point - cone.evaluate(p.u, p.v)).length() < 1e-10);
        assert!((p.distance - 5. * angle.cos()).abs() < 1e-10);
        let behind = point_to_cone(apex - axis * 5., &cone);
        assert!((behind.point - apex).length() < 1e-10);
        assert!((behind.distance - 5.).abs() < 1e-10);
    }
}

#[test]
fn collapsed_second_and_both_ranges_keep_witnesses_and_parameters() {
    let a = NurbsCurve::new(
        1,
        vec![0., 0., 10., 10.],
        vec![Point3::new(-5., 1., 0.), Point3::new(5., 1., 0.)],
        vec![1., 1.],
    )
    .unwrap();
    let b = NurbsCurve::new(
        1,
        vec![0., 0., 1., 1.],
        vec![Point3::new(0., 0., 0.), Point3::new(1., 0., 0.)],
        vec![1., 1.],
    )
    .unwrap();
    let r = curve_to_curve(&a, (0., 10.), &b, (0., 0.));
    assert!((r.distance - 1.).abs() < 1e-10);
    assert!((r.param_a - 5.).abs() < 1e-10);
    assert!((r.point_a - a.evaluate(r.param_a)).length() < 1e-10);
    let r = curve_to_curve(&a, (0., 0.), &b, (1., 1.));
    assert!((r.distance - 37f64.sqrt()).abs() < 1e-10);
    assert_eq!((r.param_a, r.param_b), (0., 1.));
}
