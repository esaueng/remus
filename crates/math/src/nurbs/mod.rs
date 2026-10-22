//! NURBS curve and surface representations.
//!
//! Non-Uniform Rational B-Spline (NURBS) geometry is the standard
//! representation for free-form curves and surfaces in CAD.

pub(crate) mod banded;
pub mod basis;
pub mod bezier_clip;
pub mod cubic_fit;
pub mod curvature;
pub mod curve;
pub mod decompose;
pub mod evaluator;
pub mod fitting;
pub mod intersection;
pub mod knot_ops;
pub mod power_basis;
pub mod projection;
pub mod reduction;
pub mod reuse;
pub(crate) mod reuse_bounds;
pub mod self_intersection;
pub mod surface;
pub mod surface_fitting;

pub use bezier_clip::{
    CurveCurveHit, CurveCurveOverlap, CurveCurveResult, curve_curve_intersect,
    curve_curve_intersect_full,
};
pub use curve::NurbsCurve;
pub use decompose::{
    BezierPatch, curve_degree_elevate, curve_to_bezier_segments, surface_to_bezier_patches,
};
pub use evaluator::SurfaceEvaluator;
pub use fitting::{approximate, interpolate, interpolate_with_params};
pub use knot_ops::{
    curve_knot_insert, curve_knot_refine, curve_knot_remove, curve_split, surface_knot_insert_u,
    surface_knot_insert_v,
};
pub use power_basis::PowerBasis1D;
pub use projection::{
    CurveProjection, SurfaceProjection, SurfaceSeedGrid, curve_coarse_seeds_public,
    curve_newton_refine_public, project_point_to_curve, project_point_to_surface,
    project_point_to_surface_seeded, project_point_to_surface_with_grid,
};
pub use surface::{DerivativeScratch, FirstOrderAxis, NurbsSurface};
pub use surface_fitting::interpolate_surface;

/// Reject control points carrying a non-finite coordinate.
///
/// A NaN or infinite control point produces geometry that evaluates to NaN
/// everywhere downstream, and NaN compares false against every tolerance, so
/// the poison passes silently through distance, containment, and validation
/// checks instead of failing. Rejecting at construction is the only place the
/// value is still attributable to its input.
fn validate_control_point_values(
    points: impl IntoIterator<Item = crate::vec::Point3>,
) -> Result<(), crate::MathError> {
    for (index, point) in points.into_iter().enumerate() {
        if !point.0.iter().all(|c| c.is_finite()) {
            return Err(crate::MathError::InvalidControlPointValue {
                index,
                x: point.x(),
                y: point.y(),
                z: point.z(),
            });
        }
    }
    Ok(())
}

pub(crate) fn validate_knot_values(knots: &[f64]) -> Result<(), crate::MathError> {
    let mut max_value = f64::NEG_INFINITY;
    for (index, &value) in knots.iter().enumerate() {
        if !value.is_finite() {
            return Err(crate::MathError::InvalidKnotValue { index, value });
        }
        // A wobble of a few ulps below the largest preceding knot is representation
        // noise, not a malformed vector: `NurbsCurve::reversed` mirrors knots
        // and clamps the endpoints back, which can leave the last interior
        // knot one ulp above the clamped end. Comparing against the running
        // maximum prevents individually small decreases from accumulating.
        let wobble = 8.0 * f64::EPSILON * max_value.abs().max(value.abs()).max(1.0);
        if index > 0 && value < max_value - wobble {
            return Err(crate::MathError::InvalidKnotValue { index, value });
        }
        max_value = max_value.max(value);
    }
    Ok(())
}

pub(crate) fn validate_knot_domain(
    knots: &[f64],
    degree: usize,
    control_points: usize,
) -> Result<(), crate::MathError> {
    // `>=`, not `>`: an equal pair is a collapsed domain, a curve or surface
    // with no parameter range at all. Evaluation, projection, and splitting
    // are all meaningless on one, and it is exactly what a decomposition bug
    // produces when it reads the wrong knot span — so reject it here rather
    // than let a zero-length entity travel.
    if knots[degree] >= knots[control_points] {
        return Err(crate::MathError::InvalidKnotValue {
            index: control_points,
            value: knots[control_points],
        });
    }
    Ok(())
}

#[cfg(test)]
mod extreme_position_tests {
    #![allow(clippy::unwrap_used)]

    use super::{NurbsCurve, NurbsSurface};
    use crate::vec::Point3;

    fn line(weights: Vec<f64>, points: Vec<Point3>) -> NurbsCurve {
        NurbsCurve::new(1, vec![0., 0., 1., 1.], points, weights).unwrap()
    }

    fn patch(weights: Vec<Vec<f64>>, points: Vec<Vec<Point3>>) -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0., 0., 1., 1.],
            vec![0., 0., 1., 1.],
            points,
            weights,
        )
        .unwrap()
    }

    fn assert_point(actual: Point3, expected: Point3) {
        for axis in 0..3 {
            assert!(actual.0[axis].is_finite(), "{actual:?}");
            let scale = expected.0[axis].abs().max(1.0);
            assert!(
                (actual.0[axis] / scale - expected.0[axis] / scale).abs() < 1e-12,
                "{actual:?} != {expected:?}"
            );
        }
    }

    #[test]
    fn accepted_subnormal_curve_weights_preserve_position_and_derivative_agreement() {
        let tiny = f64::from_bits(1);
        let points = vec![Point3::new(0., 0., 0.), Point3::new(2., 4., -2.)];
        for (weights, ordinary) in [
            (vec![tiny; 2], vec![1.; 2]),
            (vec![2. * tiny, 3. * tiny], vec![2., 3.]),
        ] {
            let curve = line(weights, points.clone());
            let reference = line(ordinary, points.clone());
            for u in [0., 0.25, 0.5, 0.75, 1.] {
                let actual = curve.evaluate(u);
                assert_point(actual, reference.evaluate(u));
                let derivative_point = curve.derivatives(u, 1)[0];
                assert_point(
                    actual,
                    Point3::new(
                        derivative_point.x(),
                        derivative_point.y(),
                        derivative_point.z(),
                    ),
                );
            }
        }
    }

    #[test]
    fn finite_curve_coordinates_do_not_overflow_an_unnormalized_sum() {
        let curve = line(
            vec![1.; 2],
            vec![
                Point3::new(f64::MAX, f64::MAX, 0.),
                Point3::new(f64::MAX, -f64::MAX, 1.),
            ],
        );
        for u in [0., 0.25, 0.5, 0.75, 1.] {
            assert_point(
                curve.evaluate(u),
                Point3::new(f64::MAX, f64::MAX * (1. - 2. * u), u),
            );
        }
    }

    #[test]
    fn tiny_weighted_coordinates_survive_when_the_final_position_is_representable() {
        let tiny = f64::from_bits(1);
        let curve = line(
            vec![tiny, 1.],
            vec![Point3::new(f64::MAX, 0., 0.), Point3::new(0., 0., 0.)],
        );
        let actual = curve.evaluate(0.5).x();
        let expected = f64::MAX * tiny;
        assert!(actual.is_finite() && actual > 0.);
        assert!(
            (actual / expected - 1.).abs() < 1e-12,
            "{actual:e} != {expected:e}"
        );
    }

    #[test]
    fn normal_weights_retain_coordinates_when_the_basis_product_underflows() {
        let u = 1.0_f64.next_down();
        for (weight, denominator_weight) in [
            (f64::MIN_POSITIVE, 1.),
            (f64::MIN_POSITIVE, 1e-300),
            (8e-308, 1.),
            (8e-308, 1e-300),
        ] {
            let expected = ((1. - u) * f64::MAX) * weight / (u * denominator_weight);
            let curve = line(
                vec![weight, denominator_weight],
                vec![Point3::new(f64::MAX, 0., 0.), Point3::new(0., 0., 0.)],
            );
            let surface = patch(
                vec![vec![weight, denominator_weight]; 2],
                vec![vec![Point3::new(f64::MAX, 0., 0.), Point3::new(0., 0., 0.)]; 2],
            );
            for actual in [
                curve.evaluate(u).x(),
                surface.evaluate(0.5, u).x(),
                surface.evaluator().point(0.5, u).x(),
            ] {
                assert!(actual.is_finite() && actual > 0.);
                assert!(
                    (actual / expected - 1.).abs() < 1e-12,
                    "{actual:e} != {expected:e}"
                );
            }
        }
    }

    #[test]
    fn accepted_subnormal_surface_weights_preserve_cached_and_derivative_positions() {
        let tiny = f64::from_bits(1);
        let points = vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 2.)],
            vec![Point3::new(1., 0., 1.), Point3::new(1., 1., 3.)],
        ];
        let surface = patch(vec![vec![tiny; 2]; 2], points.clone());
        let reference = patch(vec![vec![1.; 2]; 2], points);
        let mut evaluator = surface.evaluator();
        for (u, v) in [(0., 0.), (0., 0.5), (0.5, 0.5), (1., 1.), (tiny, 0.5)] {
            let expected = reference.evaluate(u, v);
            assert_point(surface.evaluate(u, v), expected);
            assert_point(evaluator.point(u, v), expected);
            let point = surface.derivatives(u, v, 1)[0][0];
            assert_point(Point3::new(point.x(), point.y(), point.z()), expected);
        }
    }

    #[test]
    fn finite_surface_coordinates_preserve_cached_positions_at_edges_and_interiors() {
        let points = vec![
            vec![Point3::new(f64::MAX, 0., 0.), Point3::new(f64::MAX, 1., 1.)],
            vec![Point3::new(f64::MAX, 1., 0.), Point3::new(f64::MAX, 2., 1.)],
        ];
        let surface = patch(vec![vec![1.; 2]; 2], points);
        let mut evaluator = surface.evaluator();
        for (u, v) in [(0., 0.), (0., 0.5), (0.5, 0.5), (1., 1.), (1e-170, 0.5)] {
            let expected = Point3::new(f64::MAX, u + v, v);
            assert_point(surface.evaluate(u, v), expected);
            assert_point(evaluator.point(u, v), expected);
        }
    }

    #[test]
    fn inactive_and_small_surface_rows_cannot_overflow_before_the_u_contraction() {
        let points = vec![
            vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 2.)],
            vec![Point3::new(1., 0., 1.), Point3::new(1., 1., 3.)],
        ];
        for (weights, u, v, expected) in [
            (
                vec![vec![1e-100; 2], vec![1e300; 2]],
                0.,
                0.5,
                Point3::new(0., 0.5, 1.),
            ),
            (
                vec![vec![1e-100; 2], vec![1e-100, 1e240]],
                1e-170,
                1e-170,
                Point3::new(0.5, 0.5, 1.5),
            ),
            (
                vec![vec![1e-100; 2], vec![1e-100, 1e300]],
                1e-170,
                1e-170,
                Point3::new(1., 1., 3.),
            ),
        ] {
            let surface = patch(weights, points.clone());
            assert_point(surface.evaluate(u, v), expected);
            assert_point(surface.evaluator().point(u, v), expected);
        }
    }
}
