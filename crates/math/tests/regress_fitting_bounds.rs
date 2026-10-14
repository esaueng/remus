//! Curve-fitting entry points are bounded (kernel review 2026-10-14,
//! MATH-8).
//!
//! `interpolate`, `interpolate_with_params`, `approximate` and
//! `approximate_lspia` took any point count and degree. Coincident
//! consecutive points give repeated chord-length parameters, which fail the
//! band check and sent `interpolate` to its dense n×n fallback: 4000 points
//! with five duplicates allocated 128 MB before returning `SingularMatrix`.
//! Every entry point now refuses, with a typed error and before assembling
//! any system: degree 0, a fitted degree above `MAX_FIT_DEGREE`, more than
//! `MAX_FIT_POINTS` points, a dense matrix above `MAX_DENSE_FIT_ENTRIES`,
//! and (for interpolation) a chord-length parameter that does not increase.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss
)]

use remus_math::MathError;
use remus_math::nurbs::fitting::{
    MAX_DENSE_FIT_ENTRIES, MAX_FIT_DEGREE, MAX_FIT_POINTS, approximate, approximate_lspia,
    interpolate, interpolate_with_params,
};
use remus_math::vec::Point3;

fn helix(n: usize) -> Vec<Point3> {
    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64 * 4.0 * std::f64::consts::PI;
            Point3::new(t.cos(), t.sin(), 0.15 * t)
        })
        .collect()
}

fn uniform_params(n: usize) -> Vec<f64> {
    (0..n).map(|i| i as f64 / (n - 1) as f64).collect()
}

fn limit(result: Result<impl std::fmt::Debug, MathError>) -> (&'static str, usize, usize) {
    match result {
        Err(MathError::ResourceLimitExceeded {
            resource,
            limit,
            actual,
        }) => (resource, limit, actual),
        other => panic!("expected a resource-limit refusal, got {other:?}"),
    }
}

#[test]
fn coincident_points_are_refused_before_the_dense_fallback() {
    // The review's repro: 4000 points, five of them repeating their
    // predecessor. Main took the 4000² dense path and only then reported
    // `SingularMatrix`; the refusal now names the first repeated point.
    let mut points = helix(4000);
    for i in [500, 1200, 1201, 2500, 3999] {
        points[i] = points[i - 1];
    }
    match interpolate(&points, 3) {
        Err(MathError::CoincidentFitPoints { index }) => assert_eq!(index, 500),
        other => panic!("expected CoincidentFitPoints, got {other:?}"),
    }
}

#[test]
fn points_closer_than_the_parameters_resolve_are_coincident() {
    // Distinct points two ulps apart in z (2.2e-16) at an accumulated chord
    // of about 6.4, whose half ulp is 4.4e-16: the chord vanishes when
    // accumulated, so the parameters repeat exactly as for coincident
    // points and the collocation rows are identical.
    let mut points = helix(200);
    let p = points[100];
    points.insert(101, Point3::new(p.x(), p.y(), p.z() + 2.5e-16));
    let chord = (points[101] - points[100]).length();
    assert!(chord > 0.0 && chord < 4.4e-16, "premise: chord {chord:e}");
    match interpolate(&points, 3) {
        Err(MathError::CoincidentFitPoints { index }) => assert_eq!(index, 101),
        other => panic!("expected CoincidentFitPoints, got {other:?}"),
    }
}

#[test]
fn coincident_first_pair_is_refused_but_a_fully_coincident_set_is_not() {
    let mut points = helix(30);
    points[1] = points[0];
    assert!(matches!(
        interpolate(&points, 3),
        Err(MathError::CoincidentFitPoints { index: 1 })
    ));
    // All points coinciding takes the uniform-parameter branch: a
    // well-posed system whose solution is the point itself. Kept working.
    let pole = vec![Point3::new(1.0, -2.0, 0.5); 12];
    let curve = interpolate(&pole, 3).expect("a fully coincident set stays fittable");
    for &p in curve.control_points() {
        assert!((p - pole[0]).length() <= 1e-14, "control point {p:?}");
    }
}

#[test]
fn degree_zero_is_refused_by_every_entry_point() {
    let points = helix(20);
    let params = uniform_params(20);
    assert!(matches!(
        interpolate(&points, 0),
        Err(MathError::InvalidDegree { degree: 0, .. })
    ));
    assert!(matches!(
        interpolate_with_params(&points, 0, &params),
        Err(MathError::InvalidDegree { degree: 0, .. })
    ));
    assert!(matches!(
        approximate(&points, 0, 8),
        Err(MathError::InvalidDegree { degree: 0, .. })
    ));
    assert!(matches!(
        approximate_lspia(&points, 0, 8, 1e-6, 10),
        Err(MathError::InvalidDegree { degree: 0, .. })
    ));
}

#[test]
fn fitted_degree_above_the_ceiling_is_refused() {
    let n = MAX_FIT_DEGREE + 5;
    let points = helix(n);
    let params = uniform_params(n);
    let want = ("fit_degree", MAX_FIT_DEGREE, MAX_FIT_DEGREE + 1);
    assert_eq!(limit(interpolate(&points, MAX_FIT_DEGREE + 1)), want);
    assert_eq!(
        limit(interpolate_with_params(
            &points,
            MAX_FIT_DEGREE + 1,
            &params
        )),
        want
    );
    assert_eq!(limit(approximate(&points, MAX_FIT_DEGREE + 1, n - 2)), want);
    assert_eq!(
        limit(approximate_lspia(
            &points,
            MAX_FIT_DEGREE + 1,
            n - 2,
            1e-6,
            10
        )),
        want
    );

    // The bound applies to the degree actually fitted, which every entry
    // point has always capped at the point count: a large request through
    // few points still fits, and the ceiling itself is admitted.
    assert_eq!(interpolate(&helix(6), 100).unwrap().degree(), 5);
    assert_eq!(
        interpolate(&points, MAX_FIT_DEGREE).unwrap().degree(),
        MAX_FIT_DEGREE
    );
}

#[test]
fn point_count_above_the_ceiling_is_refused() {
    // Collinear points keep the build cheap; only the count matters.
    let n = MAX_FIT_POINTS + 1;
    let points: Vec<Point3> = (0..n).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
    let want = ("fit_points", MAX_FIT_POINTS, n);
    assert_eq!(limit(interpolate(&points, 3)), want);
    assert_eq!(limit(approximate(&points, 3, 16)), want);
    assert_eq!(limit(approximate_lspia(&points, 3, 16, 1e-6, 10)), want);
    // Checked before `params` is examined, so no parameter vector is built.
    assert_eq!(limit(interpolate_with_params(&points, 3, &[])), want);
}

#[test]
fn least_squares_dense_budget_is_enforced_before_assembly() {
    // 8192 points and 512 control points: 8192·512 + 2·512² = 4 718 592
    // entries against a budget of 4 194 304. Main allocated them and spent
    // about 2·10⁹ multiply-adds forming the normal equations.
    let (n, m) = (8192, 512);
    let entries = n * m + 2 * m * m;
    assert!(entries > MAX_DENSE_FIT_ENTRIES);
    let points = helix(n);
    assert_eq!(
        limit(approximate(&points, 3, m)),
        ("dense_fit_entries", MAX_DENSE_FIT_ENTRIES, entries)
    );
    // Fewer control points keep the same data inside the budget.
    let m = 64;
    assert!(n * m + 2 * m * m <= MAX_DENSE_FIT_ENTRIES);
    let fitted = approximate(&points, 3, m).expect("inside the dense budget");
    assert_eq!(fitted.control_points().len(), m);
}
