#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use remus_math::context::{CancellationToken, WorkBudgets};
use remus_math::mat::Mat4;
use remus_math::nurbs::fitting::interpolate_with_params;
use remus_math::tolerance::Tolerance;

#[derive(Clone, Copy)]
enum Fixture {
    Orthogonal,
    TiltedF3,
}

fn transform_direction(m: Mat4, v: Vec3) -> Vec3 {
    Vec3::new(
        m.0[0][0] * v.x() + m.0[0][1] * v.y() + m.0[0][2] * v.z(),
        m.0[1][0] * v.x() + m.0[1][1] * v.y() + m.0[1][2] * v.z(),
        m.0[2][0] * v.x() + m.0[2][1] * v.y() + m.0[2][2] * v.z(),
    )
}

// Independent Cartesian equations, with no use of the fitter's quadratic.
fn oracle(fixture: Fixture, u: f64, sign: f64) -> Point3 {
    let (sin, cos) = u.sin_cos();
    let (x, y) = (2.0 * cos, 2.0 * sin);
    let z = match fixture {
        Fixture::Orthogonal => sign * (9.0 - x * x).sqrt(),
        Fixture::TiltedF3 => {
            // Tool frame: origin (4,-2,-.5), axis (0,-.5,sqrt(3)/2).
            // Its radial equation is (x-4)^2 + (sqrt(3)/2*(y+2)+.5*(z+.5))^2=5.5^2.
            -0.5 + 2.0
                * (sign * (30.25 - (x - 4.0).powi(2)).sqrt() - 3.0_f64.sqrt() * 0.5 * (y + 2.0))
        }
    };
    Point3::new(x, y, z)
}

fn fixture(
    kind: Fixture,
    scale: f64,
    placed: bool,
    sign: f64,
    reversed: bool,
) -> (
    CylindricalSurface,
    CylindricalSurface,
    NurbsCurve,
    [Point3; 2],
    Mat4,
) {
    let rotation = if placed {
        Mat4::rotation_x(0.37) * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    let shift = if placed {
        Vec3::new(10.0 * scale, -7.0 * scale, 3.0 * scale)
    } else {
        Vec3::new(0.0, 0.0, 0.0)
    };
    let placement = Mat4::translation(shift.x(), shift.y(), shift.z()) * rotation;
    let inverse = rotation.transpose() * Mat4::translation(-shift.x(), -shift.y(), -shift.z());
    let (origin, axis, radius) = match kind {
        Fixture::Orthogonal => (Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 3.0),
        Fixture::TiltedF3 => (
            Point3::new(4.0, -2.0, -0.5),
            Vec3::new(0.0, -0.5, 3.0_f64.sqrt() * 0.5),
            5.5,
        ),
    };
    let scaled = |p: Point3| Point3::new(p.x() * scale, p.y() * scale, p.z() * scale);
    let first = CylindricalSurface::with_ref_dir(
        placement.mul_point(Point3::new(0.0, 0.0, 0.0)),
        transform_direction(rotation, Vec3::new(0.0, 0.0, 1.0)),
        2.0 * scale,
        transform_direction(rotation, Vec3::new(1.0, 0.0, 0.0)),
    )
    .unwrap();
    let second = CylindricalSurface::with_ref_dir(
        placement.mul_point(scaled(origin)),
        transform_direction(rotation, axis),
        radius * scale,
        transform_direction(rotation, Vec3::new(1.0, 0.0, 0.0)),
    )
    .unwrap();
    let (u0, u1) = if reversed { (1.5, 0.8) } else { (0.8, 1.5) };
    let params: Vec<_> = (0..=16).map(|i| f64::from(i) / 16.0).collect();
    let points: Vec<_> = params
        .iter()
        .map(|t| placement.mul_point(scaled(oracle(kind, u0 + (u1 - u0) * t, sign))))
        .collect();
    let anchors = [points[0], points[16]];
    let guide = interpolate_with_params(&points, 3, &params).unwrap();
    (first, second, guide, anchors, inverse)
}

#[test]
fn independent_cylinder_equations_qualify_both_branches_and_directed_anchors() {
    for kind in [Fixture::Orthogonal, Fixture::TiltedF3] {
        for scale in [1e-3, 1.0, 1e3] {
            for placed in [false, true] {
                for sign in [-1.0, 1.0] {
                    for reversed in [false, true] {
                        let (first, second, guide, anchors, inverse) =
                            fixture(kind, scale, placed, sign, reversed);
                        let tolerance = Tolerance {
                            linear: 1e-7 * scale,
                            ..Tolerance::new()
                        };
                        let context = OperationContext::new().with_tolerance(tolerance);
                        let curve = fit_open_cylinder_section(
                            &first,
                            &second,
                            &guide,
                            guide.domain(),
                            anchors,
                            None,
                            &[],
                            &[],
                            &context,
                        )
                        .unwrap()
                        .expect("qualified regular branch");
                        assert_eq!(curve.control_points().first(), Some(&anchors[0]));
                        assert_eq!(curve.control_points().last(), Some(&anchors[1]));
                        assert!(
                            curve
                                .weights()
                                .iter()
                                .all(|w| w.to_bits() == 1.0_f64.to_bits())
                        );
                        let mut previous = None;
                        for i in 0..=1000 {
                            let t = f64::from(i) / 1_000.0;
                            let point = inverse.mul_point(curve.evaluate(t));
                            let point = Point3::new(
                                point.x() / scale,
                                point.y() / scale,
                                point.z() / scale,
                            );
                            assert!((point.x().hypot(point.y()) - 2.0).abs() < 1e-7);
                            let second_radius = match kind {
                                Fixture::Orthogonal => point.x().hypot(point.z()),
                                Fixture::TiltedF3 => (point.x() - 4.0).hypot(
                                    3.0_f64.sqrt() * 0.5 * (point.y() + 2.0)
                                        + 0.5 * (point.z() + 0.5),
                                ),
                            };
                            let expected_radius = match kind {
                                Fixture::Orthogonal => 3.0,
                                Fixture::TiltedF3 => 5.5,
                            };
                            assert!((second_radius - expected_radius).abs() < 1e-7);
                            let u = point.y().atan2(point.x());
                            let expected = oracle(kind, u, sign);
                            assert!((point - expected).length() < 1e-6, "wrong algebraic branch");
                            if let Some(previous) = previous {
                                assert!((u - previous) * if reversed { -1.0 } else { 1.0 } > 0.0);
                            }
                            previous = Some(u);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn unresolved_fit_respects_depth_and_cancellation() {
    let (first, second, guide, anchors, _) = fixture(Fixture::TiltedF3, 1.0, false, 1.0, false);
    let context =
        OperationContext::new().with_budgets(WorkBudgets::new().with_subdivision_depth(0));
    assert!(matches!(
        fit_open_cylinder_section(
            &first,
            &second,
            &guide,
            guide.domain(),
            anchors,
            None,
            &[],
            &[],
            &context
        ),
        Err(AlgoError::ResourceLimitExceeded {
            resource: "cylinder section fit refinement depth",
            limit: 0,
            ..
        })
    ));
    let empty = OperationContext::new().with_budgets(WorkBudgets::new().with_segments(0));
    assert!(matches!(
        fit_open_cylinder_section(
            &first,
            &second,
            &guide,
            guide.domain(),
            anchors,
            None,
            &[],
            &[],
            &empty
        ),
        Err(AlgoError::ResourceLimitExceeded {
            resource: "cylinder section fit components",
            limit: 0,
            actual: 1
        })
    ));
    let token = CancellationToken::new();
    token.cancel();
    let context = OperationContext::new().with_cancellation(token);
    assert!(matches!(
        fit_open_cylinder_section(
            &first,
            &second,
            &guide,
            guide.domain(),
            anchors,
            None,
            &[],
            &[],
            &context
        ),
        Err(AlgoError::Math(remus_math::MathError::Cancelled))
    ));
}

#[test]
fn whole_span_angular_certificate_rejects_backtracking_controls() {
    let cylinder = CylindricalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let points = [
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(1.0, 2.0, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    ];
    let bound = bezier_cylinder_angular_direction(&points, &[1.0; 4], &cylinder).unwrap();
    assert!(
        bound[0] < 0.0 && bound[1] > 0.0,
        "endpoints alone do not prove angular monotonicity"
    );
    let line = [Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)];
    let (bound, _) =
        bezier_cylinder_residual_bound(&line, &[1.0; 2], &cylinder, cylinder.axis()).unwrap();
    assert!(bound >= 2.0, "off-carrier line must be refused");
}

#[test]
fn untrusted_scaled_endpoint_requires_one_independently_certified_junction() {
    let first = CylindricalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2_000.0,
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap();
    let second = CylindricalSurface::with_ref_dir(
        Point3::new(4_000.0, -2_000.0, -500.0),
        Vec3::new(0.0, -0.5, 3.0_f64.sqrt() * 0.5),
        5_500.0,
        Vec3::new(0.0, 3.0_f64.sqrt() * 0.5, 0.5),
    )
    .unwrap();
    let bad = Point3::new(-1_443.913_704_648_756_3, -1_383.881_168_126_760_3, 0.0);
    // Independent Cartesian cap-circle/cylinder solution, z=0.
    let junction = Point3::new(-1_443.898_073_929_748_2, -1_383.892_464_067_191_7, 0.0);
    let end = Point3::new(
        -1_310.587_550_451_098_7,
        -1_510.748_249_246_607,
        1_514.520_437_567_844_3,
    );
    assert!((bad - junction).length() > 0.019);
    let u0 = first.project_point(bad).0;
    let u1 = first.project_point(end).0;
    let points: Vec<_> = (0..=8)
        .map(|i| {
            if i == 0 {
                return bad;
            }
            if i == 8 {
                return end;
            }
            let u = u0 + (u1 - u0) * f64::from(i) / 8.0;
            let x = -2_000.0 * u.sin();
            let y = 2_000.0 * u.cos();
            let z = -500.0
                + 2.0
                    * ((30_250_000.0 - (x - 4_000.0).powi(2)).sqrt()
                        - 3.0_f64.sqrt() * 0.5 * (y + 2_000.0));
            Point3::new(x, y, z)
        })
        .collect();
    let guide = remus_math::nurbs::fitting::interpolate(&points, 3).unwrap();
    let axis_squared = second.axis().dot(second.axis());
    let projected_axis =
        first.axis() - second.axis() * (first.axis().dot(second.axis()) / axis_squared);
    let branch = Branch {
        first: &first,
        second: &second,
        projected_axis,
        axis_squared,
        quadratic: projected_axis.dot(projected_axis),
        index: 0,
    };
    let context = OperationContext::new();
    let limits = Some([(0.0, 3_500.0), (0.0, 1_500.0)]);
    assert!(
        needs_anchor_recovery(
            &first,
            &second,
            &guide,
            guide.domain(),
            [bad, end],
            &context
        )
        .unwrap()
    );
    assert!(
        !needs_anchor_recovery(
            &first,
            &second,
            &guide,
            guide.domain(),
            [junction, end],
            &context
        )
        .unwrap()
    );
    let recovered = recover_anchors(
        &branch,
        &guide,
        guide.domain(),
        [bad, end],
        limits,
        &[],
        &[junction],
        &context,
    )
    .unwrap();
    assert!(same_point_bits(recovered[0], junction));
    assert!(same_point_bits(recovered[1], end));
    let fitted = fit_open_cylinder_section(
        &first,
        &second,
        &guide,
        guide.domain(),
        [bad, end],
        limits,
        &[],
        &[junction],
        &context,
    )
    .unwrap()
    .expect("certified scaled cap recovery must fit at unchanged absolute tolerance");
    assert_eq!(fitted.control_points().first(), Some(&junction));
    assert_eq!(fitted.control_points().last(), Some(&end));
    let displaced = junction + Vec3::new(1e-4, 0.0, 0.0);
    for candidates in [vec![], vec![bad], vec![displaced], vec![end]] {
        assert!(matches!(
            recover_anchors(
                &branch,
                &guide,
                guide.domain(),
                [bad, end],
                limits,
                &[],
                &candidates,
                &context
            ),
            Err(AlgoError::IntersectionFailed(_))
        ));
    }
    let distinct = Point3::new(junction.x().next_up(), junction.y(), junction.z());
    assert!(point_on_carriers(distinct, &first, &second, 1e-7 / 8.0));
    assert!(matches!(
        recover_anchors(
            &branch,
            &guide,
            guide.domain(),
            [bad, end],
            limits,
            &[],
            &[junction, distinct],
            &context
        ),
        Err(AlgoError::IntersectionFailed(_))
    ));
    // The other exact root on the same z=0 cap is outside this guide's
    // adjacent-probe angular window, even when its axial domain is allowed.
    let other_root = Point3::new(-57.179_808_548_654_29, 1_999.182_450_276_697_3, 0.0);
    assert!(point_on_carriers(other_root, &first, &second, 1e-7 / 8.0));
    let wider_limits = Some([(0.0, 3_500.0), (-2_000.0, 1_500.0)]);
    assert!(matches!(
        recover_anchors(
            &branch,
            &guide,
            guide.domain(),
            [bad, end],
            wider_limits,
            &[],
            &[other_root],
            &context
        ),
        Err(AlgoError::IntersectionFailed(_))
    ));
    let priority = recover_anchors(
        &branch,
        &guide,
        guide.domain(),
        [bad, end],
        limits,
        &[junction],
        &[junction, distinct],
        &context,
    )
    .unwrap();
    assert!(same_point_bits(priority[0], junction));
    assert!(matches!(
        recover_anchors(
            &branch,
            &guide,
            guide.domain(),
            [bad, end],
            limits,
            &[junction, distinct],
            &[junction],
            &context
        ),
        Err(AlgoError::IntersectionFailed(_))
    ));
    let outside = Some([(0.0, 3_500.0), (200.0, 1_500.0)]);
    assert!(matches!(
        recover_anchors(
            &branch,
            &guide,
            guide.domain(),
            [bad, end],
            outside,
            &[],
            &[junction],
            &context
        ),
        Err(AlgoError::IntersectionFailed(_))
    ));
}

#[test]
fn finite_faces_refuse_a_branch_excursion_hidden_by_the_guide() {
    let first = CylindricalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let second = CylindricalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        3.0,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let u0 = FRAC_PI_2 - 0.003;
    let u1 = FRAC_PI_2 + 0.003;
    // Independent equations x²+y²=4 and x²+z²=9. Both anchors lie
    // on this cap, but the positive branch rises about 6e-6 beyond it.
    let cap = (9.0 - 4.0 * u0.cos().powi(2)).sqrt();
    let params: Vec<_> = (0..=16).map(|i| f64::from(i) / 16.0).collect();
    let points: Vec<_> = params
        .iter()
        .map(|t| {
            let u = u0 + (u1 - u0) * t;
            Point3::new(2.0 * u.cos(), 2.0 * u.sin(), cap)
        })
        .collect();
    let guide = interpolate_with_params(&points, 3, &params).unwrap();
    let anchors = [points[0], points[16]];
    let context = OperationContext::new();
    assert!(3.0 - cap > 50.0 * context.tolerance.linear);
    assert!(matches!(
        fit_open_cylinder_section(
            &first,
            &second,
            &guide,
            guide.domain(),
            anchors,
            Some([(0.0, cap), (-3.0, 3.0)]),
            &[],
            &[],
            &context,
        ),
        Err(AlgoError::IntersectionFailed(_))
    ));
    let fitted = fit_open_cylinder_section(
        &first,
        &second,
        &guide,
        guide.domain(),
        anchors,
        Some([(0.0, 3.1), (-3.0, 3.0)]),
        &[],
        &[],
        &context,
    )
    .unwrap()
    .expect("the same regular branch lies within wider finite faces");
    assert!((fitted.evaluate(0.5).z() - 3.0).abs() < context.tolerance.linear);
    assert_eq!(fitted.control_points().first(), Some(&anchors[0]));
    assert_eq!(fitted.control_points().last(), Some(&anchors[1]));

    // An unattainable arithmetic band must stop at the global station cap,
    // independently of the larger per-initial-subarc depth allowance.
    let tight = OperationContext::new()
        .with_tolerance(Tolerance {
            linear: 1e-15,
            ..Tolerance::new()
        })
        .with_budgets(WorkBudgets::new().with_subdivision_depth(12));
    assert!(matches!(
        fit_open_cylinder_section(
            &first,
            &second,
            &guide,
            guide.domain(),
            anchors,
            None,
            &[],
            &[],
            &tight,
        ),
        Err(AlgoError::ResourceLimitExceeded {
            resource: "cylinder section fit stations",
            limit: MAX_POINTS,
            actual,
        }) if actual > MAX_POINTS
    ));
}
