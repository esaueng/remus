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
    MAX_DENSE_FIT_ENTRIES, MAX_FIT_DEGREE, MAX_FIT_POINTS, MAX_FIT_WORK, MAX_SPARSE_FIT_ENTRIES,
    approximate, approximate_lspia, interpolate, interpolate_with_params,
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
    // Review follow-ups: with no control points, or no parameters, either,
    // the degree is still what is reported.
    assert!(matches!(
        interpolate_with_params(&points, 0, &[]),
        Err(MathError::InvalidDegree {
            degree: 0,
            control_points: 20
        })
    ));
    assert!(matches!(
        approximate(&points, 0, 0),
        Err(MathError::InvalidDegree {
            degree: 0,
            control_points: 0
        })
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

#[test]
fn least_squares_work_is_bounded_not_only_its_memory() {
    // Review follow-up: 3345 points and 836 control points fit the dense
    // entry budget (4 194 212 of 4 194 304) but forming NᵀN alone is
    // n·m² ≈ 2.34·10⁹ multiply-adds, more than the 8192 × 512 case.
    let (n, m) = (3345, 836);
    assert!(
        n * m + 2 * m * m <= MAX_DENSE_FIT_ENTRIES,
        "premise: entries fit"
    );
    let work = n * m * m + m * m * m;
    assert!(work > MAX_FIT_WORK);
    assert_eq!(
        limit(approximate(&helix(n), 3, m)),
        ("fit_work", MAX_FIT_WORK, work)
    );
    // 8192 × 64 stays inside both budgets (see the dense-budget test).
    const { assert!(8192 * 64 * 64 + 64 * 64 * 64 <= MAX_FIT_WORK) };
}

#[test]
fn lspia_iteration_work_is_bounded() {
    // Every iterate walks each point's p + 1 basis values three times
    // (evaluate twice, spread the error once), and the step size takes 20
    // power-iteration rounds of two passes: n·(p + 1)·(3·(iterations + 1) + 40).
    // 10 000 points at degree 3 with 10 000 iterations (the WASM cap) is
    // 1.2·10⁹ visits.
    let n = 10_000;
    let work = n * 4 * (3 * 10_001 + 40);
    assert!(work > MAX_FIT_WORK);
    assert_eq!(
        limit(approximate_lspia(&helix(n), 3, 100, 0.0, 10_000)),
        ("fit_work", MAX_FIT_WORK, work)
    );
    // Review follow-up: 10⁶ points at degree 3 and 267 iterations passed
    // the old one-pass count (1.07·10⁹) but visits 3.2·10⁹ basis values.
    let n = 1_000_000;
    let points: Vec<Point3> = (0..n).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
    assert_eq!(
        limit(approximate_lspia(&points, 3, 100, 0.0, 267)),
        ("fit_work", MAX_FIT_WORK, n * 4 * (3 * 268 + 40))
    );
    // The in-tree SSI and self-intersection callers (degree ≤ 3, 100
    // iterations) stay inside the budget up to 780 000 points per chain.
    const { assert!(780_000 * 4 * (3 * 101 + 40) <= MAX_FIT_WORK) };
}

#[test]
fn banded_and_basis_storage_is_bounded() {
    // Review follow-up: 2²⁰ distinct points at degree 25 pass the point and
    // degree bounds, take the compact interpolation path and would allocate
    // a band of n·(3p + 1) = 79 691 776 doubles (608 MiB). Collinear points
    // keep the setup cheap; the refusal comes before the band exists.
    let n = MAX_FIT_POINTS;
    let points: Vec<Point3> = (0..n).map(|i| Point3::new(i as f64, 0.0, 0.0)).collect();
    assert_eq!(
        limit(interpolate(&points, MAX_FIT_DEGREE)),
        (
            "sparse_fit_entries",
            MAX_SPARSE_FIT_ENTRIES,
            n * (3 * 25 + 1)
        )
    );
    // LSPIA's per-point basis table is the same kind of storage: n·(p + 1)
    // values plus six entries of overhead per separately allocated row.
    assert_eq!(
        limit(approximate_lspia(&points, MAX_FIT_DEGREE, 64, 0.0, 0)),
        ("sparse_fit_entries", MAX_SPARSE_FIT_ENTRIES, n * 32)
    );
    // A cubic fit through every admitted point stays inside the budget.
    const { assert!(MAX_FIT_POINTS * (3 * 3 + 1) <= MAX_SPARSE_FIT_ENTRIES) };
    const { assert!(MAX_FIT_POINTS * (3 + 1 + 6) <= MAX_SPARSE_FIT_ENTRIES) };
}

#[test]
fn lspia_rows_are_charged_their_allocation_overhead() {
    // Review follow-up: 10⁶ points at degree 15 hold 16·10⁶ basis values,
    // inside the budget, but each point's row is its own allocation, so the
    // table retained a million allocations beyond those values. Each row now
    // counts six entries of overhead.
    let n = 1_000_000;
    let points: Vec<Point3> = (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64;
            Point3::new(t, (6.0 * t).sin(), 0.0)
        })
        .collect();
    assert_eq!(
        limit(approximate_lspia(&points, 15, 16, 0.0, 0)),
        ("sparse_fit_entries", MAX_SPARSE_FIT_ENTRIES, n * 22)
    );
}

#[test]
fn lspia_scan_fallback_is_budgeted_for_overflowed_parameters() {
    // Review follow-up: finite points alternating between 0 and 1e200
    // overflow the chord lengths, so every chord-length parameter is NaN and
    // the nearest-parameter search falls back to its n·m scan. 2²⁰ points
    // and 2048 control points at zero iterations pass the iteration budget
    // (n·4) but would scan 2·10⁹ pairs.
    let n = MAX_FIT_POINTS;
    let points: Vec<Point3> = (0..n)
        .map(|i| Point3::new(if i % 2 == 0 { 0.0 } else { 1e200 }, 0.0, 0.0))
        .collect();
    let m = 2048;
    assert_eq!(
        limit(approximate_lspia(&points, 3, m, 0.0, 0)),
        ("fit_work", MAX_FIT_WORK, n * 4 * (3 + 40) + n * m)
    );
    // Review follow-up: the scan and the iterations share one budget. Here
    // each fits on its own (scan 832 000 000, iterations 559 000 000) but
    // not together, and the per-point basis table fits its own budget.
    let (n, p, m) = (500_000, 25, 1664);
    const { assert!(500_000 * 1664 <= MAX_FIT_WORK) };
    const { assert!(500_000 * 26 * (3 + 40) <= MAX_FIT_WORK) };
    const { assert!(500_000 * (26 + 6) <= MAX_SPARSE_FIT_ENTRIES) };
    let points: Vec<Point3> = (0..n)
        .map(|i| Point3::new(if i % 2 == 0 { 0.0 } else { 1e200 }, 0.0, 0.0))
        .collect();
    assert_eq!(
        limit(approximate_lspia(&points, p, m, 0.0, 0)),
        ("fit_work", MAX_FIT_WORK, n * (p + 1) * (3 + 40) + n * m)
    );
}

#[test]
fn overflowed_chord_parameters_are_refused_before_assembly() {
    // Review follow-up: finite points alternating between 0 and 1e200
    // overflow the chord lengths, so every chord-length parameter is NaN.
    // NaN passed the "does not increase" test (every comparison is false)
    // and 2048 points reached the 2048² dense fallback before a late
    // `SingularMatrix`. A non-finite parameter is now refused up front.
    for n in [4, 2048] {
        let points: Vec<Point3> = (0..n)
            .map(|i| Point3::new(if i % 2 == 0 { 0.0 } else { 1e200 }, 0.0, 0.0))
            .collect();
        match interpolate(&points, 3) {
            Err(MathError::ParameterOutOfRange { value, min, max }) => {
                assert!(value.is_nan(), "parameter {value}");
                assert_eq!((min, max), (0.0, 1.0));
            }
            other => panic!("n = {n}: expected ParameterOutOfRange, got {other:?}"),
        }
    }
}
