//! Independent qualification for Remus's modified Apache-2.0 Truck adaptation.
//! Source algorithms pinned at 88ed005249e5e3a6b07f62425399435905cd3ab6;
//! these tests are newly written for Remus, not copied Truck fixtures.
//! See docs/production-readiness/truck-reuse-provenance.md.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use super::*;
use crate::MathError;
use crate::context::{CancellationToken, FallbackPolicy};
use crate::nurbs::knot_ops::{curve_knot_insert, surface_knot_insert_u, surface_knot_insert_v};

fn rational_patch(scale: f64, shift: Point3) -> NurbsSurface {
    let x = [0.0, 1.0, 3.0];
    let y = [-1.0, 2.0, 4.0];
    let wu = [1.0, 2.0, 0.5];
    let wv = [1.0, 0.75, 1.5];
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        x.iter()
            .map(|&xv| {
                y.iter()
                    .map(|&yv| Point3::new(xv + shift.x(), yv + shift.y(), shift.z()))
                    .collect()
            })
            .collect(),
        wu.iter()
            .map(|&uw| wv.iter().map(|&vw| uw * vw * scale).collect())
            .collect(),
    )
    .unwrap()
}

fn rational_coordinate(parameter: f64, points: [f64; 3], weights: [f64; 3]) -> f64 {
    let basis = [
        (1.0 - parameter).powi(2),
        2.0 * parameter * (1.0 - parameter),
        parameter.powi(2),
    ];
    let numerator: f64 = basis
        .iter()
        .zip(points)
        .zip(weights)
        .map(|((&b, p), w)| b * p * w)
        .sum();
    let denominator: f64 = basis.iter().zip(weights).map(|(&b, w)| b * w).sum();
    numerator / denominator
}

fn assert_closed_form(surface: &NurbsSurface, shift: Point3, tolerance: f64) {
    let (umin, umax) = surface.domain_u();
    let (vmin, vmax) = surface.domain_v();
    // The oracle is the rational quadratic Bernstein formula, independent of
    // both knot reconstruction and Remus's B-spline basis evaluator.
    for u in [0.0, 0.13, 0.37, 0.63, 0.9, 1.0] {
        for v in [0.0, 0.07, 0.31, 0.73, 1.0] {
            let expected = Point3::new(
                rational_coordinate(u, [0.0, 1.0, 3.0], [1.0, 2.0, 0.5]) + shift.x(),
                rational_coordinate(v, [-1.0, 2.0, 4.0], [1.0, 0.75, 1.5]) + shift.y(),
                shift.z(),
            );
            let actual = surface.evaluate(umin + (umax - umin) * u, vmin + (vmax - vmin) * v);
            assert!(
                (actual - expected).length() <= tolerance,
                "closed-form mismatch: {actual:?} vs {expected:?}"
            );
        }
    }
}

fn refined_patch() -> NurbsSurface {
    let base = rational_patch(1.0, Point3::new(0.0, 0.0, 0.0));
    let surface = surface_knot_insert_u(&base, 0.27, 1).unwrap();
    let surface = surface_knot_insert_u(&surface, 0.71, 2).unwrap();
    surface_knot_insert_v(&surface, 0.42, 1).unwrap()
}

#[test]
fn both_directions_recover_rational_patch_with_asymmetric_control_net() {
    let surface = refined_patch();
    let original = surface.clone();
    let options = ReductionOptions::default();
    let context = OperationContext::new();
    let u = surface_knot_remove_u(&surface, 0.27, &options, &context).unwrap();
    let v = surface_knot_remove_v(&surface, 0.42, &options, &context).unwrap();
    assert_eq!(u.removed_knots, 1);
    assert_eq!(v.removed_knots, 1);
    assert_eq!(
        u.geometry.control_points().len() + 1,
        surface.control_points().len()
    );
    assert_eq!(
        v.geometry.control_points()[0].len() + 1,
        surface.control_points()[0].len()
    );
    assert!(u.deviation_bound.is_finite() && u.deviation_bound <= options.tolerance);
    assert!(v.deviation_bound.is_finite() && v.deviation_bound <= options.tolerance);
    assert_closed_form(&u.geometry, Point3::new(0.0, 0.0, 0.0), 1e-12);
    assert_closed_form(&v.geometry, Point3::new(0.0, 0.0, 0.0), 1e-12);
    assert_eq!(surface, original);
}

#[test]
fn simplification_removes_repeated_knots_deterministically() {
    let surface = refined_patch();
    let options = ReductionOptions::default();
    let context = OperationContext::new();
    let first = simplify_surface(&surface, &options, &context).unwrap();
    let second = simplify_surface(&surface, &options, &context).unwrap();
    assert_eq!(first.removed_knots, 4);
    assert_eq!(first.geometry.control_points().len(), 3);
    assert_eq!(first.geometry.control_points()[0].len(), 3);
    assert_eq!(first.geometry, second.geometry);
    assert_eq!(first.work_used, second.work_used);
    assert_eq!(first.deviation_bound, second.deviation_bound);
    assert!(first.deviation_bound <= options.tolerance);
    assert_closed_form(&first.geometry, Point3::new(0.0, 0.0, 0.0), 1e-12);
}

#[test]
fn surface_weight_scale_and_placement_are_not_geometric_tolerances() {
    for scale in [1e-250, 1.0, 1e250] {
        let shift = Point3::new(10_000.0, -20_000.0, 3_000.0);
        let base = rational_patch(scale, shift);
        let surface = surface_knot_insert_u(&base, 0.37, 1).unwrap();
        let result = surface_knot_remove_u(
            &surface,
            0.37,
            &ReductionOptions::default(),
            &OperationContext::new(),
        )
        .unwrap();
        assert_closed_form(&result.geometry, shift, 1e-10);
        assert!(
            result
                .geometry
                .weights()
                .iter()
                .flatten()
                .all(|&weight| weight.is_finite() && weight > 0.0)
        );
    }
}

#[test]
fn tiny_parameter_domains_do_not_merge_distinct_surface_knots() {
    let refined = refined_patch();
    let scaled = NurbsSurface::new(
        refined.degree_u(),
        refined.degree_v(),
        refined.knots_u().iter().map(|knot| knot * 1e-20).collect(),
        refined.knots_v().iter().map(|knot| knot * 3e-18).collect(),
        refined.control_points().to_vec(),
        refined.weights().to_vec(),
    )
    .unwrap();
    let result = simplify_surface(
        &scaled,
        &ReductionOptions::default(),
        &OperationContext::new(),
    )
    .unwrap();
    assert_eq!(result.removed_knots, 4);
    assert_eq!(result.geometry.domain_u(), scaled.domain_u());
    assert_eq!(result.geometry.domain_v(), scaled.domain_v());
    assert_closed_form(&result.geometry, Point3::new(0.0, 0.0, 0.0), 1e-12);
}

#[test]
fn curve_simplification_uses_original_bound_and_preserves_tiny_domain() {
    let base = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
        vec![1.0, 2.0, 0.5],
    )
    .unwrap();
    let inserted = curve_knot_insert(&curve_knot_insert(&base, 0.2, 1).unwrap(), 0.8, 2).unwrap();
    let tiny = NurbsCurve::new(
        inserted.degree(),
        inserted.knots().iter().map(|k| k * 1e-20).collect(),
        inserted.control_points().to_vec(),
        inserted.weights().iter().map(|w| w * 1e-250).collect(),
    )
    .unwrap();
    let result = simplify_curve(
        &tiny,
        &ReductionOptions::default(),
        &OperationContext::new(),
    )
    .unwrap();
    assert_eq!(result.removed_knots, 3);
    assert_eq!(result.geometry.domain(), tiny.domain());
    for u in [0.0, 0.12, 0.4, 0.78, 1.0] {
        let expected = rational_coordinate(u, [0.0, 1.0, 3.0], [1.0, 2.0, 0.5]);
        assert!((result.geometry.evaluate(u * 1e-20).x() - expected).abs() < 1e-12);
    }
}

#[test]
fn degree_one_corner_is_not_misreported_as_a_removable_knot() {
    // The existing curve candidate generator has no overlapping controls for
    // degree one. Whole-domain qualification must still reject this corner.
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.5, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        vec![1.0; 3],
    )
    .unwrap();
    let result = simplify_curve(
        &curve,
        &ReductionOptions::default(),
        &OperationContext::new(),
    )
    .unwrap();
    assert_eq!(result.removed_knots, 0);
    assert_eq!(result.geometry, curve);
    assert_eq!(result.deviation_bound, 0.0);
    let surface = NurbsSurface::new(
        1,
        1,
        curve.knots().to_vec(),
        vec![0.0, 0.0, 1.0, 1.0],
        curve
            .control_points()
            .iter()
            .map(|p| vec![*p, Point3::new(p.x(), p.y(), 1.0)])
            .collect(),
        vec![vec![1.0; 2]; 3],
    )
    .unwrap();
    assert!(matches!(
        surface_knot_remove_u(
            &surface,
            0.5,
            &ReductionOptions::default(),
            &OperationContext::new()
        ),
        Err(ReuseError::ToleranceExceeded { .. })
    ));
}

#[test]
fn absent_end_and_nan_knots_never_claim_a_removal() {
    let surface = refined_patch();
    let original = surface.clone();
    for knot in [0.0, 1.0, 0.1, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            surface_knot_remove_u(
                &surface,
                knot,
                &ReductionOptions::default(),
                &OperationContext::new()
            ),
            Err(ReuseError::KnotNotRemovable)
        ));
    }
    assert_eq!(surface, original);
}

#[test]
fn options_fallback_cancellation_and_aggregate_budget_are_respected() {
    let surface = refined_patch();
    let options = ReductionOptions::default();
    let context = OperationContext::new();
    for tolerance in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            simplify_surface(
                &surface,
                &ReductionOptions {
                    tolerance,
                    ..options
                },
                &context
            ),
            Err(ReuseError::InvalidOptions { .. })
        ));
    }
    assert!(matches!(
        simplify_surface(
            &surface,
            &ReductionOptions {
                max_work: 1,
                ..options
            },
            &context
        ),
        Err(ReuseError::WorkLimit { .. })
    ));
    let complete = simplify_surface(&surface, &options, &context).unwrap();
    assert!(matches!(
        simplify_surface(
            &surface,
            &ReductionOptions {
                max_work: complete.work_used - 1,
                ..options
            },
            &context
        ),
        Err(ReuseError::WorkLimit { .. })
    ));
    let token = CancellationToken::new();
    token.cancel();
    assert!(matches!(
        simplify_surface(
            &surface,
            &options,
            &context.clone().with_cancellation(token)
        ),
        Err(ReuseError::Math(MathError::Cancelled))
    ));
    let exact_context = context.with_fallback(FallbackPolicy::ExactOnly);
    let exact = simplify_surface(&surface, &options, &exact_context).unwrap();
    assert_eq!(exact.removed_knots, 0);
    assert_eq!(exact.geometry, surface);
    assert!(matches!(
        surface_knot_remove_u(&surface, 0.27, &options, &exact_context),
        Err(ReuseError::ToleranceExceeded { tolerance: 0.0, .. })
    ));
    assert!(matches!(
        simplify_surface(
            &surface,
            &options,
            &OperationContext::new()
                .with_fallback(FallbackPolicy::AllowApproximate { budget: f64::NAN })
        ),
        Err(ReuseError::InvalidOptions { .. })
    ));
}

#[test]
fn unclamped_and_discontinuous_domains_refuse() {
    let unclamped = NurbsCurve::new(
        1,
        vec![0.0, 1.0, 2.0, 3.0, 4.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        vec![1.0; 3],
    )
    .unwrap();
    assert!(matches!(
        simplify_curve(
            &unclamped,
            &ReductionOptions::default(),
            &OperationContext::new()
        ),
        Err(ReuseError::Unsupported { .. })
    ));
    let discontinuous = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 0.5, 0.5, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    assert!(matches!(
        simplify_curve(
            &discontinuous,
            &ReductionOptions::default(),
            &OperationContext::new()
        ),
        Err(ReuseError::Unsupported { .. })
    ));
}

#[test]
fn close_curve_knots_refuse_before_the_candidate_generator_can_merge_them() {
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1e-16, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let original = curve.clone();
    assert!(matches!(
        simplify_curve(
            &curve,
            &ReductionOptions::default(),
            &OperationContext::new()
        ),
        Err(ReuseError::Unsupported { .. })
    ));
    assert_eq!(curve, original);
}
