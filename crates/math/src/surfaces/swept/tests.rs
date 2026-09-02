#![allow(clippy::expect_used, clippy::unwrap_used)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use super::*;
use crate::context::{CancellationToken, WorkBudgets};
use crate::curves::{Circle3D, Ellipse3D, Hyperbola3D, Line3D, Parabola3D};
use crate::nurbs::NurbsCurve;
use crate::nurbs::curvature::surface_curvature;
use crate::tolerance::Tolerance;
use proptest::prelude::*;

fn context(iterations: usize) -> OperationContext {
    OperationContext::new()
        .with_tolerance(Tolerance::tight())
        .with_budgets(WorkBudgets::new().with_newton_iterations(iterations))
}

fn assert_point_close(actual: Point3, expected: Point3, tolerance: f64, label: &str) {
    let residual = (actual - expected).length();
    assert!(
        residual <= tolerance,
        "{label}: point residual {residual:e} exceeds {tolerance:e}: {actual:?} vs {expected:?}"
    );
}

fn assert_vector_direction(actual: Vec3, expected: Vec3, tolerance: f64, label: &str) {
    let actual = actual.normalize().expect("actual direction is regular");
    let expected = expected.normalize().expect("expected direction is regular");
    let alignment = actual.dot(expected).abs();
    assert!(
        1.0 - alignment <= tolerance,
        "{label}: direction alignment {alignment:e}"
    );
}

fn sample_parameters(domain: (f64, f64)) -> Vec<f64> {
    [0.0, 1.0e-8, 0.071, 0.23, 0.5, 0.79, 0.999_999_99, 1.0]
        .into_iter()
        .map(|fraction| (domain.1 - domain.0).mul_add(fraction, domain.0))
        .collect()
}

fn supported_profiles() -> Vec<(SweptCurve, (f64, f64))> {
    let line =
        Line3D::new(Point3::new(1.0, -2.0, 0.5), Vec3::new(1.0, 2.0, -0.5)).expect("valid line");
    let circle = Circle3D::new_with_ref(
        Point3::new(2.0, 1.0, -3.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.5,
        Vec3::new(1.0, 1.0, 0.0),
    )
    .expect("valid circle");
    let ellipse = Ellipse3D::new_with_ref(
        Point3::new(-1.0, 4.0, 2.0),
        Vec3::new(0.0, 1.0, 1.0),
        3.0,
        1.25,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("valid ellipse");
    let hyperbola = Hyperbola3D::with_axes(
        Point3::new(0.5, -0.75, 1.25),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 1.0, 0.0),
        2.0,
        0.75,
    )
    .expect("valid hyperbola");
    let parabola = Parabola3D::with_axes(
        Point3::new(-2.0, 0.5, 3.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        1.5,
    )
    .expect("valid parabola");
    let nurbs = NurbsCurve::new(
        2,
        vec![-2.0, -2.0, -2.0, 0.0, 2.0, 2.0, 2.0],
        vec![
            Point3::new(-2.0, 0.0, 1.0),
            Point3::new(-1.0, 2.0, 0.0),
            Point3::new(1.0, 2.0, 1.0),
            Point3::new(2.0, 0.0, 2.0),
        ],
        vec![1.0, 0.8, 1.2, 1.0],
    )
    .expect("valid NURBS");
    vec![
        (SweptCurve::Line(line), (-1.75, 2.25)),
        (SweptCurve::Circle(circle), (-0.4, 5.6)),
        (SweptCurve::Ellipse(ellipse), (0.2, 4.8)),
        (SweptCurve::Hyperbola(hyperbola), (-1.1, 1.4)),
        (SweptCurve::Parabola(parabola), (-2.0, 3.5)),
        (SweptCurve::Nurbs(nurbs), (-1.5, 1.25)),
    ]
}

#[test]
fn supported_profiles_lower_to_exact_finite_nurbs_spans() {
    let solve_context = context(64);
    for (profile, bounds) in supported_profiles() {
        for directed_bounds in [bounds, (bounds.1, bounds.0)] {
            let nurbs = profile
                .to_nurbs(directed_bounds.0, directed_bounds.1)
                .expect("supported profile lowers exactly");
            let scale = nurbs
                .control_points()
                .iter()
                .map(|point| point.0.iter().map(|value| value.abs()).fold(0.0, f64::max))
                .fold(1.0, f64::max);
            for parameter in sample_parameters(nurbs.domain()) {
                let point = nurbs.evaluate(parameter);
                let projection = profile
                    .project_point_checked(point, bounds, &solve_context)
                    .expect("exact NURBS point projects to its source profile");
                assert!(
                    projection.residual <= 2.0e-9 * scale,
                    "{} reversed={}: residual {:e}",
                    profile.type_tag(),
                    directed_bounds.0 > directed_bounds.1,
                    projection.residual
                );
            }
            assert_point_close(
                nurbs.evaluate(nurbs.domain().0),
                profile
                    .evaluate_checked(directed_bounds.0)
                    .expect("finite start"),
                2.0e-12 * scale,
                "directed start anchor",
            );
            assert_point_close(
                nurbs.evaluate(nurbs.domain().1),
                profile
                    .evaluate_checked(directed_bounds.1)
                    .expect("finite end"),
                2.0e-12 * scale,
                "directed end anchor",
            );
        }
    }
}

#[test]
fn revolution_nurbs_twin_matches_position_partials_normals_and_curvatures() {
    let profile = Ellipse3D::new_with_ref(
        Point3::new(4.0, 0.0, 0.5),
        Vec3::new(0.0, 1.0, 0.0),
        1.5,
        0.75,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .expect("valid profile");
    let revolution = SurfaceOfRevolution::new(
        SweptCurve::Ellipse(profile),
        Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).expect("valid axis"),
    )
    .expect("regular revolution");
    let u_bounds = (TAU, 0.0);
    let v_bounds = (5.5, 0.3);
    let nurbs = revolution
        .to_nurbs(u_bounds, v_bounds)
        .expect("exact lowering");
    let solve_context = context(64);

    for (u, v) in sample_parameters(nurbs.domain_u())
        .into_iter()
        .zip(sample_parameters(nurbs.domain_v()))
    {
        let point = nurbs.evaluate(u, v);
        let projection = revolution
            .project_point_checked(point, v_bounds, &solve_context)
            .expect("NURBS twin point projects to native carrier");
        assert!(
            projection.residual <= 2.0e-9,
            "position residual {:e}",
            projection.residual
        );

        let nurbs_derivatives = nurbs.derivatives(u, v, 2);
        let (_, native_u, native_v, _, _, _) = revolution
            .derivatives_checked(projection.u, projection.v)
            .expect("native derivatives");
        assert_vector_direction(
            nurbs_derivatives[1][0],
            native_u,
            2.0e-10,
            "revolution u partial",
        );
        assert_vector_direction(
            nurbs_derivatives[0][1],
            native_v,
            2.0e-10,
            "revolution v partial",
        );

        let nurbs_normal = nurbs.normal(u, v).expect("NURBS normal");
        let native_normal = revolution
            .normal_checked(projection.u, projection.v)
            .expect("native normal");
        assert!((1.0 - nurbs_normal.dot(native_normal).abs()) <= 2.0e-10);

        let nurbs_curvature = surface_curvature(&nurbs, u, v).expect("NURBS curvature");
        let native_curvature = revolution
            .curvature(projection.u, projection.v)
            .expect("native curvature");
        let orientation = nurbs_normal.dot(native_normal).signum();
        assert!((nurbs_curvature.k1 - orientation * native_curvature.k1).abs() <= 2.0e-8);
        assert!((nurbs_curvature.k2 - orientation * native_curvature.k2).abs() <= 2.0e-8);
    }
}

#[test]
fn extrusion_nurbs_twin_matches_position_partials_normals_and_curvatures() {
    let parabola = Parabola3D::with_axes(
        Point3::new(1.0, -2.0, 0.5),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        1.25,
    )
    .expect("valid parabola");
    let extrusion =
        SurfaceOfLinearExtrusion::new(SweptCurve::Parabola(parabola), Vec3::new(0.5, -0.25, 3.0))
            .expect("regular extrusion");
    let u_bounds = (2.5, -1.75);
    let v_bounds = (3.0, -2.0);
    let nurbs = extrusion
        .to_nurbs(u_bounds, v_bounds)
        .expect("exact lowering");
    let solve_context = context(64);

    for (u, v) in sample_parameters(nurbs.domain_u())
        .into_iter()
        .zip(sample_parameters(nurbs.domain_v()))
    {
        let point = nurbs.evaluate(u, v);
        let projection = extrusion
            .project_point_checked(point, u_bounds, &solve_context)
            .expect("NURBS twin point projects to native carrier");
        assert!(
            projection.residual <= 2.0e-9,
            "position residual {:e}",
            projection.residual
        );

        let nurbs_derivatives = nurbs.derivatives(u, v, 2);
        let (_, native_u, native_v, _, _, _) = extrusion
            .derivatives_checked(projection.u, projection.v)
            .expect("native derivatives");
        assert_vector_direction(
            nurbs_derivatives[1][0],
            native_u,
            2.0e-10,
            "extrusion u partial",
        );
        assert_vector_direction(
            nurbs_derivatives[0][1],
            native_v,
            2.0e-10,
            "extrusion v partial",
        );

        let nurbs_normal = nurbs.normal(u, v).expect("NURBS normal");
        let native_normal = extrusion
            .normal_checked(projection.u, projection.v)
            .expect("native normal");
        assert!((1.0 - nurbs_normal.dot(native_normal).abs()) <= 2.0e-10);

        let nurbs_curvature = surface_curvature(&nurbs, u, v).expect("NURBS curvature");
        let native_curvature = extrusion
            .curvature(projection.u, projection.v)
            .expect("native curvature");
        let orientation = nurbs_normal.dot(native_normal).signum();
        assert!((nurbs_curvature.k1 - orientation * native_curvature.k1).abs() <= 2.0e-9);
        assert!((nurbs_curvature.k2 - orientation * native_curvature.k2).abs() <= 2.0e-9);
    }
}

#[test]
fn scale_seam_and_pole_adjacent_projection_remain_qualified() {
    let solve_context = context(64);
    for scale in [1.0e-6, 1.0, 1.0e6] {
        let center = Point3::new(7.0 * scale, -5.0 * scale, 3.0 * scale);
        let circle = Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 1.0, 0.0),
            2.0 * scale,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .expect("valid scaled profile");
        let axis = Line3D::new(center, Vec3::new(0.0, 0.0, 1.0)).expect("valid axis");
        let revolution = SurfaceOfRevolution::new(SweptCurve::Circle(circle), axis)
            .expect("regular scaled revolution");
        for (u, v) in [
            (1.0e-10, 0.35),
            (TAU - 1.0e-10, PI + 0.2),
            (1.7, FRAC_PI_2 - 1.0e-7),
        ] {
            let point = revolution.evaluate_checked(u, v).expect("finite point");
            let projection = revolution
                .project_point_checked(point, (0.0, TAU), &solve_context)
                .expect("qualified projection");
            assert!(
                projection.residual <= 5.0e-9 * scale.max(1.0),
                "scale {scale:e}, residual {:e}",
                projection.residual
            );
            let normal = revolution
                .normal_checked(projection.u, projection.v)
                .expect("regular near-pole normal");
            assert!((normal.length() - 1.0).abs() <= 1.0e-12);
        }
    }
}

#[test]
fn typed_refusals_cover_construction_spans_budgets_and_cancellation() {
    let axis =
        Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).expect("valid axis");
    let on_axis = SweptCurve::Line(
        Line3D::new(Point3::new(0.0, 0.0, 2.0), Vec3::new(0.0, 0.0, 1.0)).expect("valid line"),
    );
    assert!(matches!(
        SurfaceOfRevolution::new(on_axis.clone(), axis),
        Err(MathError::ZeroVector)
    ));
    assert!(matches!(
        SurfaceOfRevolution::new(
            SweptCurve::Line(
                Line3D::new(Point3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0))
                    .expect("valid profile line"),
            ),
            Line3D::new(Point3::new(f64::NAN, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0),)
                .expect("direction is valid"),
        ),
        Err(MathError::InvalidControlPointValue { .. })
    ));
    assert!(matches!(
        SurfaceOfLinearExtrusion::new(on_axis.clone(), Vec3::new(0.0, 0.0, 2.0)),
        Err(MathError::ZeroVector)
    ));
    assert!(matches!(
        SurfaceOfLinearExtrusion::new(on_axis.clone(), Vec3::new(f64::NAN, 0.0, 1.0)),
        Err(MathError::InvalidControlPointValue { .. })
    ));
    assert!(matches!(
        on_axis.to_nurbs(1.0, 1.0),
        Err(MathError::ParameterOutOfRange { .. })
    ));

    let extrusion = SurfaceOfLinearExtrusion::new(on_axis, Vec3::new(1.0, 0.0, 0.0))
        .expect("regular extrusion");
    assert!(matches!(
        extrusion.project_point_checked(Point3::new(1.0, 0.0, 0.0), (-2.0, 2.0), &context(0),),
        Err(MathError::ConvergenceFailure { iterations: 0 })
    ));

    let cancellation = CancellationToken::new();
    cancellation.cancel();
    let cancelled_context = context(32).with_cancellation(cancellation);
    assert!(matches!(
        extrusion.project_point_checked(
            Point3::new(1.0, 0.0, 0.0),
            (-2.0, 2.0),
            &cancelled_context,
        ),
        Err(MathError::Cancelled)
    ));
}

#[test]
fn periods_tags_and_direction_scale_are_stable() {
    let circle = SweptCurve::Circle(
        Circle3D::new(Point3::new(3.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0)
            .expect("valid circle"),
    );
    assert_eq!(circle.type_tag(), "circle");
    assert_eq!(circle.period(), Some(TAU));

    let extrusion = SurfaceOfLinearExtrusion::new(circle.clone(), Vec3::new(0.0, 0.0, 7.5))
        .expect("regular extrusion");
    assert_eq!(extrusion.direction(), Vec3::new(0.0, 0.0, 7.5));
    assert_eq!(extrusion.u_period(), Some(TAU));
    assert_eq!(extrusion.v_period(), None);

    let revolution = SurfaceOfRevolution::new(
        circle,
        Line3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).expect("valid axis"),
    )
    .expect("regular revolution");
    assert_eq!(revolution.u_period(), Some(TAU));
    assert_eq!(revolution.v_period(), Some(TAU));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn exact_revolution_twin_property_holds_across_scale_and_parameter_space(
        exponent in -4_i32..5,
        u_fraction in 1.0e-6_f64..0.999_999,
        v_fraction in 1.0e-6_f64..0.999_999,
    ) {
        let scale = 10.0_f64.powi(exponent);
        let axis_origin = Point3::new(7.0 * scale, -11.0 * scale, 5.0 * scale);
        let profile = SweptCurve::Line(
            Line3D::new(
                axis_origin + Vec3::new(3.0 * scale, 0.0, -2.0 * scale),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .expect("regular profile"),
        );
        let revolution = SurfaceOfRevolution::new(
            profile,
            Line3D::new(axis_origin, Vec3::new(0.0, 0.0, 1.0)).expect("regular axis"),
        )
        .expect("regular revolution");
        let v_bounds = (-2.0 * scale, 3.0 * scale);
        let nurbs = revolution
            .to_nurbs((0.0, TAU), v_bounds)
            .expect("exact lowering");
        let (nu0, nu1) = nurbs.domain_u();
        let (nv0, nv1) = nurbs.domain_v();
        let nu = (nu1 - nu0).mul_add(u_fraction, nu0);
        let nv = (nv1 - nv0).mul_add(v_fraction, nv0);
        let point = nurbs.evaluate(nu, nv);
        let projection = revolution
            .project_point_checked(point, v_bounds, &context(64))
            .expect("qualified projection");
        let tolerance = 5.0e-9 * scale.max(1.0);
        prop_assert!(projection.residual <= tolerance);

        let nurbs_derivatives = nurbs.derivatives(nu, nv, 1);
        let (_, native_u, native_v, _, _, _) = revolution
            .derivatives_checked(projection.u, projection.v)
            .expect("native derivatives");
        let u_alignment = nurbs_derivatives[1][0]
            .normalize()
            .expect("regular u partial")
            .dot(native_u.normalize().expect("regular native u"))
            .abs();
        let v_alignment = nurbs_derivatives[0][1]
            .normalize()
            .expect("regular v partial")
            .dot(native_v.normalize().expect("regular native v"))
            .abs();
        prop_assert!(1.0 - u_alignment <= 2.0e-10);
        prop_assert!(1.0 - v_alignment <= 2.0e-10);
    }
}
