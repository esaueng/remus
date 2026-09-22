//! Closed-form plane sections used by exact blend-end reconstruction.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::TAU;

use remus_math::analytic_intersection::{
    AnalyticSurface, ExactIntersectionCurve, exact_plane_analytic,
};
use remus_math::context::OperationContext;
use remus_math::intersect::{
    ContactKind, CurveGeometry, IntersectionElement, PlaneOperand, ResultQuality, SourceMethod,
    SurfaceOperand, intersect_surfaces,
};
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

fn assert_close(actual: f64, expected: f64, tolerance: f64, context: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{context}: expected {expected}, got {actual}"
    );
}

fn plane_distance(normal: Vec3, d: f64, point: Point3) -> f64 {
    (normal.dot(Vec3::new(point.x(), point.y(), point.z())) - d).abs()
}

fn cylinder_distance(cylinder: &CylindricalSurface, point: Point3) -> f64 {
    let offset = point - cylinder.origin();
    let radial = offset - cylinder.axis() * offset.dot(cylinder.axis());
    (radial.length() - cylinder.radius()).abs()
}

fn torus_distance(torus: &ToroidalSurface, point: Point3) -> f64 {
    let offset = point - torus.center();
    let axial = offset.dot(torus.z_axis());
    let radial = (offset - torus.z_axis() * axial).length();
    ((radial - torus.major_radius()).hypot(axial) - torus.minor_radius()).abs()
}

fn cone_residual(cone: &ConicalSurface, point: Point3) -> (f64, f64) {
    let offset = point - cone.apex();
    let axial = offset.dot(cone.axis());
    let radial = (offset - cone.axis() * axial).length();
    let residual = (radial * cone.half_angle().sin() - axial * cone.half_angle().cos()).abs();
    (residual, axial)
}

fn qualified_circles(
    result: &remus_math::intersect::SurfaceIntersection,
) -> Vec<&remus_math::curves::Circle3D> {
    result
        .elements
        .iter()
        .map(|element| match element {
            IntersectionElement::Curve(curve) => match &curve.geometry {
                CurveGeometry::Circle(circle) => circle,
                other => panic!("expected exact circle, got {other:?}"),
            },
            other => panic!("expected circle curve, got {other:?}"),
        })
        .collect()
}

#[test]
fn placed_scaled_parallel_cylinder_sections_are_exact_lines() {
    let context = OperationContext::new();
    let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
    let normal = axis.cross(Vec3::new(0.0, 0.0, 1.0)).normalize().unwrap();

    for scale in [1e-3, 1.0, 1e3] {
        let origin = Point3::new(4.0 * scale, -3.0 * scale, 2.0 * scale);
        let radius = 2.5 * scale;
        let cylinder = CylindricalSurface::new(origin, axis, radius).unwrap();
        let axis_offset = normal.dot(Vec3::new(origin.x(), origin.y(), origin.z()));

        let secant_d = axis_offset + 0.6 * radius;
        let secant = intersect_surfaces(
            SurfaceOperand::Plane(PlaneOperand {
                normal,
                d: secant_d,
            }),
            SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
            &context,
        )
        .unwrap();
        assert!(secant.complete);
        assert_eq!(secant.elements.len(), 2);
        for element in &secant.elements {
            let IntersectionElement::Curve(curve) = element else {
                panic!("expected exact line curve, got {element:?}");
            };
            let CurveGeometry::Line { origin, direction } = &curve.geometry else {
                panic!("expected exact line, got {curve:?}");
            };
            assert_eq!(curve.kind, ContactKind::Transversal);
            assert_eq!(curve.quality, ResultQuality::Exact);
            assert_eq!(curve.method, SourceMethod::ClosedForm);
            assert_close(direction.dot(axis).abs(), 1.0, 1e-12, "line direction");
            assert_close(
                plane_distance(normal, secant_d, *origin),
                0.0,
                1e-10 * scale.max(1.0),
                "line point on plane",
            );
            for parameter in [-7.0 * scale, 0.0, 11.0 * scale] {
                assert_close(
                    cylinder_distance(&cylinder, *origin + *direction * parameter),
                    0.0,
                    1e-10 * scale.max(1.0),
                    "line point on cylinder",
                );
            }
        }

        let tangent_d = axis_offset - radius;
        let tangent = intersect_surfaces(
            SurfaceOperand::Plane(PlaneOperand {
                normal,
                d: tangent_d,
            }),
            SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
            &context,
        )
        .unwrap();
        assert!(tangent.complete);
        assert_eq!(tangent.elements.len(), 1);
        let IntersectionElement::Curve(curve) = &tangent.elements[0] else {
            panic!("expected tangent line curve");
        };
        assert!(matches!(curve.geometry, CurveGeometry::Line { .. }));
        assert_eq!(curve.kind, ContactKind::Tangential);
        assert_eq!(curve.quality, ResultQuality::Exact);
        assert_eq!(curve.method, SourceMethod::ClosedForm);

        let disjoint = intersect_surfaces(
            SurfaceOperand::Plane(PlaneOperand {
                normal,
                d: axis_offset + 1.01 * radius,
            }),
            SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
            &context,
        )
        .unwrap();
        assert!(disjoint.complete);
        assert!(disjoint.elements.is_empty());
    }
}

#[test]
fn axis_normal_ring_torus_sections_are_concentric_and_scale_invariant() {
    let axis = Vec3::new(-2.0, 1.0, 3.0).normalize().unwrap();
    for scale in [1e-3, 1.0, 1e3] {
        let center = Point3::new(7.0 * scale, -5.0 * scale, 3.0 * scale);
        let major = 5.0 * scale;
        let minor = 2.0 * scale;
        let torus = ToroidalSurface::with_axis(center, major, minor, axis).unwrap();
        let height = 0.5 * minor;
        let section_center = center + axis * height;
        let d = axis.dot(Vec3::new(
            section_center.x(),
            section_center.y(),
            section_center.z(),
        ));

        let result = intersect_surfaces(
            SurfaceOperand::Plane(PlaneOperand { normal: axis, d }),
            SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
            &OperationContext::new(),
        )
        .unwrap();
        assert!(result.complete);
        let circles = qualified_circles(&result);
        assert_eq!(circles.len(), 2);
        let radial_offset = (minor * minor - height * height).sqrt();
        assert_close(
            circles[0].radius(),
            major - radial_offset,
            1e-10 * scale.max(1.0),
            "inner section radius",
        );
        assert_close(
            circles[1].radius(),
            major + radial_offset,
            1e-10 * scale.max(1.0),
            "outer section radius",
        );
        for circle in circles {
            assert_close(
                (circle.center() - section_center).length(),
                0.0,
                1e-10 * scale.max(1.0),
                "section center",
            );
            for sample in 0..16 {
                let point = circle.evaluate(TAU * f64::from(sample) / 16.0);
                assert_close(
                    plane_distance(axis, d, point),
                    0.0,
                    1e-10 * scale.max(1.0),
                    "circle on plane",
                );
                assert_close(
                    torus_distance(&torus, point),
                    0.0,
                    1e-10 * scale.max(1.0),
                    "circle on torus",
                );
            }
        }
    }
}

#[test]
fn ring_torus_tangent_and_disjoint_axis_normal_sections_are_closed_form() {
    let center = Point3::new(2.0, -3.0, 5.0);
    let axis = Vec3::new(1.0, -2.0, 4.0).normalize().unwrap();
    let torus = ToroidalSurface::with_axis(center, 6.0, 1.5, axis).unwrap();
    let center_d = axis.dot(Vec3::new(center.x(), center.y(), center.z()));

    let context = OperationContext::new();
    let tangent = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: center_d + torus.minor_radius(),
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(tangent.complete);
    let circles = qualified_circles(&tangent);
    assert_eq!(circles.len(), 1);
    assert_close(
        circles[0].radius(),
        torus.major_radius(),
        1e-12,
        "tangent radius",
    );

    let disjoint = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: center_d + 1.01 * torus.minor_radius(),
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(disjoint.complete);
    assert!(disjoint.elements.is_empty());
}

#[test]
fn placed_ring_torus_meridian_section_is_two_tube_circles() {
    let center = Point3::new(-4.0, 7.0, 2.0);
    let axis = Vec3::new(2.0, 3.0, -1.0).normalize().unwrap();
    let torus = ToroidalSurface::with_axis(center, 8.0, 1.25, axis).unwrap();
    let normal = axis.cross(torus.x_axis()).normalize().unwrap();
    let d = normal.dot(Vec3::new(center.x(), center.y(), center.z()));

    let result = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand { normal, d }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &OperationContext::new(),
    )
    .unwrap();
    assert!(result.complete);
    let circles = qualified_circles(&result);
    assert_eq!(circles.len(), 2);
    for circle in circles {
        assert_close(circle.radius(), torus.minor_radius(), 1e-12, "tube radius");
        assert_close(
            (circle.center() - center).length(),
            torus.major_radius(),
            1e-12,
            "tube center offset",
        );
        for sample in 0..16 {
            let point = circle.evaluate(TAU * f64::from(sample) / 16.0);
            assert_close(
                plane_distance(normal, d, point),
                0.0,
                1e-11,
                "circle on plane",
            );
            assert_close(torus_distance(&torus, point), 0.0, 1e-11, "circle on torus");
        }
    }
}

fn assert_qualified_circles(
    result: &remus_math::intersect::SurfaceIntersection,
    expected_count: usize,
    expected_kind: ContactKind,
) {
    assert!(result.complete);
    assert_eq!(result.elements.len(), expected_count);
    for element in &result.elements {
        let IntersectionElement::Curve(curve) = element else {
            panic!("expected curve element");
        };
        assert!(matches!(curve.geometry, CurveGeometry::Circle(_)));
        assert_eq!(curve.kind, expected_kind);
        assert_eq!(curve.quality, ResultQuality::Exact);
        assert_eq!(curve.method, SourceMethod::ClosedForm);
    }
}

#[test]
fn qualified_torus_special_sections_report_complete_contact_kind() {
    let context = OperationContext::new();
    let center = Point3::new(1.0, 2.0, -4.0);
    let axis = Vec3::new(1.0, 1.0, 2.0).normalize().unwrap();
    let torus = ToroidalSurface::with_axis(center, 5.0, 1.0, axis).unwrap();
    let center_axis_d = axis.dot(Vec3::new(center.x(), center.y(), center.z()));

    let crossing = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: center_axis_d + 0.25,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(crossing.complete, "axis-normal crossing must be complete");
    assert_qualified_circles(&crossing, 2, ContactKind::Transversal);

    let tangent = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: center_axis_d + torus.minor_radius(),
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(tangent.complete, "axis-normal tangent must be complete");
    assert_qualified_circles(&tangent, 1, ContactKind::Tangential);

    let meridian_normal = axis.cross(torus.x_axis()).normalize().unwrap();
    let meridian_d = meridian_normal.dot(Vec3::new(center.x(), center.y(), center.z()));
    let meridian = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: meridian_normal,
            d: meridian_d,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(meridian.complete, "meridian section must be complete");
    assert_qualified_circles(&meridian, 2, ContactKind::Transversal);

    let disjoint = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: center_axis_d + 2.0 * torus.minor_radius(),
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(disjoint.complete);
    assert!(disjoint.elements.is_empty());
}

#[test]
fn legacy_plane_torus_entry_keeps_sampled_chains_for_boolean_trimming() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0, 1.0).unwrap();
    let curves = exact_plane_analytic(
        AnalyticSurface::Torus(&torus),
        Vec3::new(0.0, 0.0, 1.0),
        0.25,
    )
    .unwrap();
    assert!(!curves.is_empty());
    assert!(
        curves
            .iter()
            .all(|curve| matches!(curve, ExactIntersectionCurve::Points(_)))
    );
}

#[test]
fn nearby_oblique_torus_plane_is_not_promoted_to_exact_special_section() {
    let context = OperationContext::new();
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0, 1.0).unwrap();
    let normal = Vec3::new(1e-6, 0.0, 1.0).normalize().unwrap();
    let result = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand { normal, d: 0.25 }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();

    assert!(!result.complete);
    for element in result.elements {
        let IntersectionElement::Curve(curve) = element else {
            panic!("legacy plane-torus fallback should return curves");
        };
        assert_eq!(curve.kind, ContactKind::Unclassified);
        assert_ne!(curve.quality, ResultQuality::Exact);
        assert!(matches!(curve.geometry, CurveGeometry::Sampled(_)));
    }
}

fn assert_qualified_exact_conic(
    result: &remus_math::intersect::SurfaceIntersection,
    expect_ellipse: bool,
) {
    assert!(result.complete);
    assert_eq!(result.elements.len(), 1);
    let IntersectionElement::Curve(curve) = &result.elements[0] else {
        panic!("expected one conic curve");
    };
    if expect_ellipse {
        assert!(matches!(curve.geometry, CurveGeometry::Ellipse(_)));
    } else {
        assert!(matches!(curve.geometry, CurveGeometry::Circle(_)));
    }
    assert_eq!(curve.kind, ContactKind::Transversal);
    assert_eq!(curve.quality, ResultQuality::Exact);
    assert_eq!(curve.method, SourceMethod::ClosedForm);
}

#[test]
fn qualified_cone_circle_and_ellipse_are_exact_in_both_plane_orientations() {
    let context = OperationContext::new();
    let apex = Point3::new(4.0, -7.0, 2.0);
    let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
    let cone = ConicalSurface::new(apex, axis, std::f64::consts::FRAC_PI_4).unwrap();

    let circle_point = apex + axis * 3.0;
    let circle_d = axis.dot(Vec3::new(
        circle_point.x(),
        circle_point.y(),
        circle_point.z(),
    ));
    for (normal, d) in [(axis, circle_d), (-axis, -circle_d)] {
        let result = intersect_surfaces(
            SurfaceOperand::Plane(PlaneOperand { normal, d }),
            SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
            &context,
        )
        .unwrap();
        assert_qualified_exact_conic(&result, false);
        let IntersectionElement::Curve(curve) = &result.elements[0] else {
            unreachable!();
        };
        let CurveGeometry::Circle(circle) = &curve.geometry else {
            unreachable!();
        };
        for sample in 0..16 {
            let point = circle.evaluate(TAU * f64::from(sample) / 16.0);
            let (residual, axial) = cone_residual(&cone, point);
            assert_close(plane_distance(normal, d, point), 0.0, 1e-11, "circle plane");
            assert_close(residual, 0.0, 1e-11, "circle cone");
            assert!(axial > 0.0, "circle must remain on the real nappe");
        }
    }

    let ellipse_normal = (axis + cone.x_axis() * 0.25).normalize().unwrap();
    let ellipse_point = apex + axis * 4.0;
    let ellipse_d = ellipse_normal.dot(Vec3::new(
        ellipse_point.x(),
        ellipse_point.y(),
        ellipse_point.z(),
    ));
    for (normal, d) in [(ellipse_normal, ellipse_d), (-ellipse_normal, -ellipse_d)] {
        let result = intersect_surfaces(
            SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
            SurfaceOperand::Plane(PlaneOperand { normal, d }),
            &context,
        )
        .unwrap();
        assert_qualified_exact_conic(&result, true);
        let IntersectionElement::Curve(curve) = &result.elements[0] else {
            unreachable!();
        };
        let CurveGeometry::Ellipse(ellipse) = &curve.geometry else {
            unreachable!();
        };
        assert!(ellipse.center().x().is_finite());
        assert!(ellipse.center().y().is_finite());
        assert!(ellipse.center().z().is_finite());
        assert!(ellipse.semi_major().is_finite() && ellipse.semi_minor().is_finite());
        for sample in 0..32 {
            let point = remus_math::traits::ParametricCurve::evaluate(
                ellipse,
                TAU * f64::from(sample) / 32.0,
            );
            let (residual, axial) = cone_residual(&cone, point);
            assert_close(
                plane_distance(normal, d, point),
                0.0,
                2e-11,
                "ellipse plane",
            );
            assert_close(residual, 0.0, 2e-11, "ellipse cone");
            assert!(axial > 0.0, "ellipse must remain on the real nappe");
        }
    }
}

#[test]
fn qualified_cone_proves_wrong_nappe_empty_but_leaves_degenerate_cases_unresolved() {
    let context = OperationContext::new();
    let apex = Point3::new(-2.0, 3.0, 5.0);
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let cone = ConicalSurface::new(apex, axis, std::f64::consts::FRAC_PI_4).unwrap();
    let apex_d = axis.dot(Vec3::new(apex.x(), apex.y(), apex.z()));

    let behind = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: apex_d - 2.0,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
        &context,
    )
    .unwrap();
    assert!(behind.complete);
    assert!(behind.elements.is_empty());

    let oblique_normal = (axis + cone.x_axis() * 0.2).normalize().unwrap();
    let wrong_side = apex - axis * 3.0;
    let wrong_d = oblique_normal.dot(Vec3::new(wrong_side.x(), wrong_side.y(), wrong_side.z()));
    let wrong_nappe = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: oblique_normal,
            d: wrong_d,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
        &context,
    )
    .unwrap();
    assert!(wrong_nappe.complete);
    assert!(wrong_nappe.elements.is_empty());

    let apex_only = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: axis,
            d: apex_d,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
        &context,
    )
    .unwrap();
    assert!(!apex_only.complete);

    let generator_normal = (axis + cone.x_axis() * (1.0 - 1e-10)).normalize().unwrap();
    let generator_d = generator_normal.dot(Vec3::new(apex.x(), apex.y(), apex.z())) + 1.0;
    let near_generator = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: generator_normal,
            d: generator_d,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
        &context,
    )
    .unwrap();
    assert!(!near_generator.complete);
    for element in near_generator.elements {
        let IntersectionElement::Curve(curve) = element else {
            panic!("sampled cone section should contain curves");
        };
        assert_eq!(curve.kind, ContactKind::Unclassified);
        assert_ne!(curve.quality, ResultQuality::Exact);
    }
}

#[test]
fn large_translation_resolvable_misses_are_not_promoted_to_tangency() {
    let context = OperationContext::new();
    let translation = 1.0e12;

    let cylinder = CylindricalSurface::new(
        Point3::new(translation, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
    )
    .unwrap();
    let cylinder_miss = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: Vec3::new(1.0, 0.0, 0.0),
            d: translation + 1.01,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
        &context,
    )
    .unwrap();
    assert!(cylinder_miss.complete);
    assert!(cylinder_miss.elements.is_empty());

    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, translation), 4.0, 1.0).unwrap();
    let torus_miss = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: translation + 1.01,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(torus_miss.complete);
    assert!(torus_miss.elements.is_empty());
}

#[test]
fn scaled_plane_equations_have_identical_exact_cylinder_sections() {
    let context = OperationContext::new();
    let cylinder =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();

    for (normal, d) in [
        (Vec3::new(1.0, 0.0, 0.0), 0.5),
        (Vec3::new(2.0, 0.0, 0.0), 1.0),
        (Vec3::new(-8.0, 0.0, 0.0), -4.0),
    ] {
        let result = intersect_surfaces(
            SurfaceOperand::Plane(PlaneOperand { normal, d }),
            SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
            &context,
        )
        .unwrap();
        assert!(result.complete);
        assert_eq!(result.elements.len(), 2);
        for element in result.elements {
            let IntersectionElement::Curve(curve) = element else {
                panic!("expected cylinder section line");
            };
            assert!(matches!(curve.geometry, CurveGeometry::Line { .. }));
            assert_eq!(curve.kind, ContactKind::Transversal);
            assert_eq!(curve.quality, ResultQuality::Exact);
        }

        let nested = intersect_surfaces(
            SurfaceOperand::Analytic(AnalyticSurface::Plane { normal, d }),
            SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
            &context,
        )
        .unwrap();
        assert!(nested.complete);
        assert_eq!(nested.elements.len(), 2);
    }
}

#[test]
fn invalid_public_and_nested_plane_equations_are_rejected() {
    let context = OperationContext::new();
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0, 1.0).unwrap();
    let invalid = [
        (Vec3::new(0.0, 0.0, 0.0), 0.0),
        (Vec3::new(f64::NAN, 0.0, 1.0), 0.0),
        (Vec3::new(0.0, 0.0, 1.0), f64::INFINITY),
    ];

    for (normal, d) in invalid {
        assert!(
            intersect_surfaces(
                SurfaceOperand::Plane(PlaneOperand { normal, d }),
                SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
                &context,
            )
            .is_err()
        );
        assert!(
            intersect_surfaces(
                SurfaceOperand::Analytic(AnalyticSurface::Plane { normal, d }),
                SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
                &context,
            )
            .is_err()
        );
        assert!(exact_plane_analytic(AnalyticSurface::Torus(&torus), normal, d).is_err());
    }
}

#[test]
fn translated_torus_interior_section_keeps_both_circles() {
    let context = OperationContext::new();
    let translation = 1.0e12;
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, translation), 4.0, 1.0).unwrap();
    let result = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: translation + 0.99,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();

    assert_qualified_circles(&result, 2, ContactKind::Transversal);
    let expected_offset = (1.0_f64 - 0.99_f64.powi(2)).sqrt();
    let mut radii = result
        .elements
        .iter()
        .map(|element| match element {
            IntersectionElement::Curve(curve) => match &curve.geometry {
                CurveGeometry::Circle(circle) => circle.radius(),
                other => panic!("expected circle, got {other:?}"),
            },
            _ => panic!("expected circle curve"),
        })
        .collect::<Vec<_>>();
    radii.sort_by(f64::total_cmp);
    assert_close(radii[0], 4.0 - expected_offset, 2e-3, "inner radius");
    assert_close(radii[1], 4.0 + expected_offset, 2e-3, "outer radius");
}

#[test]
fn near_special_orientations_remain_unresolved_without_proven_identity() {
    let context = OperationContext::new();
    let cylinder =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let almost_parallel = Vec3::new(1.0, 0.0, 1e-15).normalize().unwrap();
    let cylinder_result = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: almost_parallel,
            d: 0.25,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&cylinder)),
        &context,
    )
    .unwrap();
    assert!(!cylinder_result.complete);
    assert!(
        cylinder_result
            .elements
            .iter()
            .all(|element| match element {
                IntersectionElement::Curve(curve) => curve.kind == ContactKind::Unclassified,
                IntersectionElement::Point(_) | IntersectionElement::CoincidentSurfaces => false,
            })
    );

    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 4.0, 1.0).unwrap();
    let almost_axis_normal = Vec3::new(1e-15, 0.0, 1.0).normalize().unwrap();
    let torus_result = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal: almost_axis_normal,
            d: 0.25,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Torus(&torus)),
        &context,
    )
    .unwrap();
    assert!(!torus_result.complete);
}

#[test]
fn translated_cone_near_apex_wrong_nappe_is_not_certified_empty() {
    let context = OperationContext::new();
    let apex = Point3::new(1.0e15, 0.0, 0.0);
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let cone = ConicalSurface::new(apex, axis, std::f64::consts::FRAC_PI_4).unwrap();
    let normal = Vec3::new(0.2, 0.0, 1.0).normalize().unwrap();
    let apex_d = normal.dot(Vec3::new(apex.x(), apex.y(), apex.z()));
    let ambiguous_wrong_side = apex_d.next_down();

    let result = intersect_surfaces(
        SurfaceOperand::Plane(PlaneOperand {
            normal,
            d: ambiguous_wrong_side,
        }),
        SurfaceOperand::Analytic(AnalyticSurface::Cone(&cone)),
        &context,
    )
    .unwrap();
    assert!(!result.complete);
}
