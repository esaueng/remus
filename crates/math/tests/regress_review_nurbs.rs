//! Independent oracles for rational derivatives and parameter/weight scaling.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use remus_math::nurbs::projection::project_point_to_curve;
use remus_math::nurbs::surface::DerivativeScratch;
use remus_math::nurbs::{NurbsCurve, NurbsSurface};
use remus_math::vec::{Point3, Vec3};

#[test]
fn rational_curve_derivatives_continue_above_degree() {
    // C(u) = 2u/(1+u); C^(k) = 2(-1)^(k+1)k!/(1+u)^(k+1).
    let curve = NurbsCurve::new(
        1,
        vec![0., 0., 1., 1.],
        vec![Point3::new(0., 0., 0.), Point3::new(1., 0., 0.)],
        vec![1., 2.],
    )
    .unwrap();
    let derivatives = curve.derivatives(0.5, 64);
    let mut expected = 2. / 1.5f64.powi(2);
    for (order, derivative) in derivatives.iter().enumerate().skip(1) {
        assert!(
            (derivative.x() / expected - 1.).abs() < 2e-13,
            "order {order}"
        );
        expected *= -((order + 1) as f64) / 1.5;
    }
}

#[test]
fn rational_surface_derivatives_cover_both_axes_and_mixed_orders() {
    // Degree-elevate the u axis to two: x=2u/(1+u), y=2v/(1+v), z=x*y.
    let xs = [0., 2. / 3., 1.];
    let wu = [1., 1.5, 2.];
    let surface = NurbsSurface::new(
        2,
        1,
        vec![0., 0., 0., 1., 1., 1.],
        vec![0., 0., 1., 1.],
        xs.iter()
            .map(|&x| vec![Point3::new(x, 0., 0.), Point3::new(x, 1., x)])
            .collect(),
        wu.iter().map(|&w| vec![w, 2. * w]).collect(),
    )
    .unwrap();
    let table = surface.derivatives(0.5, 0.5, 4);
    let mut scratch = DerivativeScratch::new();
    let sentinel = Vec3::new(99., 99., 99.);
    let mut out = vec![vec![sentinel; 5]; 5];
    surface.derivatives_into(0.5, 0.5, 4, &mut scratch, &mut out);
    let values = [
        2. / 3.,
        2. / 1.5f64.powi(2),
        -4. / 1.5f64.powi(3),
        12. / 1.5f64.powi(4),
        -48. / 1.5f64.powi(5),
    ];
    for k in 0..=4 {
        for l in 0..=4 - k {
            let expected = Vec3::new(
                if l == 0 { values[k] } else { 0. },
                if k == 0 { values[l] } else { 0. },
                values[k] * values[l],
            );
            assert!((table[k][l] - expected).length() < 2e-12, "({k},{l})");
            assert!((out[k][l] - expected).length() < 2e-12, "into ({k},{l})");
        }
    }
    assert_eq!(out[4][4], sentinel);
}

#[test]
fn polynomial_derivatives_above_degree_are_zero() {
    let curve = NurbsCurve::new(
        1,
        vec![0., 0., 1., 1.],
        vec![Point3::new(0., 0., 0.), Point3::new(1., 0., 0.)],
        vec![1., 1.],
    )
    .unwrap();
    for derivative in curve.derivatives(0.3, 4).iter().skip(2) {
        assert_eq!(*derivative, Vec3::new(0., 0., 0.));
    }
}

#[test]
fn cached_points_and_normals_ignore_common_weight_factors() {
    for factor in [f64::from_bits(1), 1e-310, 1e-300, 1., 1e300] {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0., 0., 1., 1.],
            vec![0., 0., 1., 1.],
            vec![
                vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 0.)],
                vec![Point3::new(1., 0., 0.), Point3::new(1., 1., 0.)],
            ],
            vec![vec![factor, 2. * factor], vec![3. * factor, 4. * factor]],
        )
        .unwrap();
        let mut evaluator = surface.evaluator();
        let expected = Point3::new(5. / 12., 5. / 6., 0.);
        assert!(
            (evaluator.point(0.25, 0.75) - expected).length() < 2e-12,
            "factor {factor:e}"
        );
        assert!((evaluator.normal(0.25, 0.75) - Vec3::new(0., 0., 1.)).length() < 2e-12);
        assert!((evaluator.point(0., 0.) - Point3::new(0., 0., 0.)).length() < 1e-12);
    }
}

#[test]
fn cached_point_uses_local_weight_scale() {
    let surface = NurbsSurface::new(
        1,
        1,
        vec![0., 0., 0.25, 0.5, 1., 1.],
        vec![0., 0., 1., 1.],
        (0..4)
            .map(|i| vec![Point3::new(i as f64, 0., 0.), Point3::new(i as f64, 1., 0.)])
            .collect(),
        vec![
            vec![1e-310; 2],
            vec![1e-310; 2],
            vec![1e-310; 2],
            vec![1e300; 2],
        ],
    )
    .unwrap();
    assert!((surface.evaluator().point(0.125, 0.5) - Point3::new(0.5, 0.5, 0.)).length() < 1e-12);
    assert!((surface.evaluator().normal(0.125, 0.5) - Vec3::new(0., 0., 1.)).length() < 1e-12);
    let corner = NurbsSurface::new(
        1,
        1,
        vec![0., 0., 1., 1.],
        vec![0., 0., 1., 1.],
        vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 0.)],
            vec![Point3::new(1., 0., 0.), Point3::new(1., 1., 0.)],
        ],
        vec![vec![1e-310, 1e300], vec![1e300; 2]],
    )
    .unwrap();
    assert_eq!(corner.evaluator().point(0., 0.), Point3::new(0., 0., 0.));
}

#[test]
fn uniform_span_hint_respects_stored_knot_sides() {
    use remus_math::nurbs::basis::{find_span_uniform, uniform_knot_step};
    // The slightly displaced second knot is accepted as uniform to floating
    // precision, but division by the first spacing rounds its index down.
    let knot = 2.0f64.next_down();
    let knots = [0., 0., 1., knot, 3., 3.];
    let step = uniform_knot_step(&knots, 1).unwrap();
    assert_eq!(find_span_uniform(4, 1, knot.next_down(), &knots, step), 2);
    assert_eq!(find_span_uniform(4, 1, knot, &knots, step), 3);
    assert_eq!(find_span_uniform(4, 1, knot.next_up(), &knots, step), 3);
}

#[test]
fn nonuniform_knot_units_do_not_change_cached_span_selection() {
    for scale in [1e-14, 1., 1e14] {
        let surface = NurbsSurface::new(
            1,
            1,
            [0., 0., 1., 3., 4., 4.].map(|k| k * scale).to_vec(),
            vec![0., 0., 1., 1.],
            [0., 2., -1., 1.]
                .iter()
                .enumerate()
                .map(|(i, &z)| vec![Point3::new(i as f64, 0., z), Point3::new(i as f64, 1., z)])
                .collect(),
            vec![vec![1.; 2]; 4],
        )
        .unwrap();
        let mut evaluator = surface.evaluator();
        assert!(
            (evaluator.point(2.5 * scale, 0.5) - Point3::new(1.75, 0.5, -0.25)).length() < 2e-12
        );
        let expected = Vec3::new(3., 0., 1.).normalize().unwrap();
        assert!((evaluator.normal(2.5 * scale, 0.5) - expected).length() < 2e-12);
    }
}

#[test]
fn projection_obeys_geometry_across_affine_knot_units() {
    // For C=(u,2u(1-u)), stationarity is this strictly increasing cubic.
    let mut lo = 0.;
    let mut hi = 1.;
    for _ in 0..64 {
        let t = (lo + hi) * 0.5;
        if ((8. * t - 12.) * t + 7.8) * t - 2.13 < 0. {
            lo = t;
        } else {
            hi = t;
        }
    }
    let u = (lo + hi) * 0.5;
    let expected = Point3::new(u, 2. * u * (1. - u), 0.);
    for (offset, span) in [(0., 1.), (0., 1e-12), (0., 1e12), (17., 7.)] {
        let curve = NurbsCurve::new(
            2,
            vec![offset; 3]
                .into_iter()
                .chain(vec![offset + span; 3])
                .collect(),
            vec![
                Point3::new(0., 0., 0.),
                Point3::new(0.5, 1., 0.),
                Point3::new(1., 0., 0.),
            ],
            vec![1.; 3],
        )
        .unwrap();
        let projection = project_point_to_curve(&curve, Point3::new(0.73, 0.7, 0.), 1e-9).unwrap();
        assert!(
            (projection.point - expected).length() < 1e-9,
            "offset={offset}, span={span}"
        );
        assert!(((projection.parameter - offset) / span - u).abs() < 1e-9);
    }
}
