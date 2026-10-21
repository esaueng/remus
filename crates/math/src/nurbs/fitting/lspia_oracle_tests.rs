//! Bit-identity oracle for `approximate_lspia`'s cached-basis loop.
//!
//! `approximate_lspia_reference` is the loop as it stood before it reused
//! `basis_data`: a fresh, fully validated `NurbsCurve` and one
//! `NurbsCurve::evaluate` per parameter on every iteration, kept verbatim.
//! It returns its last iterate, so `reference_fit` runs it once per
//! iteration count to recover every iterate, measures each with `evaluate`,
//! and selects as `approximate_lspia` must (B92): the first iterate under
//! the tolerance, else the first with the smallest maximum deviation.
//! Every case compares that against the cached loop bit for bit, `Err`
//! payload included, and checks LSPIA's copy of the `evaluate` tail,
//! `point_from_basis`, against `evaluate` itself at every fitting parameter.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_lossless
)]

use super::*;

/// `approximate_lspia` before the cached-basis loop, verbatim.
fn approximate_lspia_reference(
    points: &[Point3],
    degree: usize,
    num_control_points: usize,
    tolerance: f64,
    max_iterations: usize,
) -> Result<NurbsCurve, MathError> {
    let n = points.len();
    if n < 2 {
        return Err(MathError::EmptyInput);
    }

    let p = degree.min(n - 1);
    let m = num_control_points.min(n).max(p + 1);

    let params = chord_length_params(points);
    let knots = build_approximation_knots(&params, p, m, n);

    // Initialize control points by sampling closest data points.
    let mut control_points = Vec::with_capacity(m);
    for i in 0..m {
        let t = if m > 1 {
            i as f64 / (m - 1) as f64
        } else {
            0.0
        };
        let mut best_idx = 0;
        let mut best_dist = f64::INFINITY;
        for (j, &param) in params.iter().enumerate() {
            let d = (param - t).abs();
            if d < best_dist {
                best_dist = d;
                best_idx = j;
            }
        }
        control_points.push(points[best_idx]);
    }

    let weights = vec![1.0; m];

    let mut basis_data: Vec<(usize, Vec<f64>)> = Vec::with_capacity(n);
    for &u in &params {
        let span = find_span(u, p, &knots, m);
        let n_vals = basis_funs(span, u, p, &knots);
        basis_data.push((span, n_vals));
    }

    let mu = compute_lspia_step_size(&basis_data, p, m);

    for iter in 0..max_iterations {
        let curve = NurbsCurve::new(p, knots.clone(), control_points.clone(), weights.clone())?;

        let mut max_err = 0.0f64;
        let mut deltas = vec![(0.0f64, 0.0f64, 0.0f64); m];

        for (i, &u) in params.iter().enumerate() {
            let q = curve.evaluate(u);
            let err_x = points[i].x() - q.x();
            let err_y = points[i].y() - q.y();
            let err_z = points[i].z() - q.z();
            let err_mag = (err_x * err_x + err_y * err_y + err_z * err_z).sqrt();
            max_err = max_err.max(err_mag);

            let (span, n_vals) = &basis_data[i];
            for (k, &nv) in n_vals.iter().enumerate() {
                let j = span - p + k;
                if j < m {
                    deltas[j].0 += nv * err_x;
                    deltas[j].1 += nv * err_y;
                    deltas[j].2 += nv * err_z;
                }
            }
        }

        if max_err < tolerance {
            return NurbsCurve::new(p, knots, control_points, weights);
        }

        for j in 0..m {
            control_points[j] = Point3::new(
                mu.mul_add(deltas[j].0, control_points[j].x()),
                mu.mul_add(deltas[j].1, control_points[j].y()),
                mu.mul_add(deltas[j].2, control_points[j].z()),
            );
        }

        if iter == max_iterations - 1 {
            return NurbsCurve::new(p, knots, control_points, weights);
        }
    }

    NurbsCurve::new(p, knots, control_points, weights)
}

fn bits(values: &[f64]) -> Vec<u64> {
    values.iter().map(|v| v.to_bits()).collect()
}

fn point_bits(points: &[Point3]) -> Vec<[u64; 3]> {
    points
        .iter()
        .map(|q| [q.x().to_bits(), q.y().to_bits(), q.z().to_bits()])
        .collect()
}

/// `point_from_basis` over the fit's own `(span, basis)` table must
/// reproduce `curve.evaluate(u)` bit for bit at every fitting parameter.
fn assert_tail_matches_evaluate(
    points: &[Point3],
    degree: usize,
    num_cps: usize,
    curve: &NurbsCurve,
) {
    let n = points.len();
    let p = degree.min(n - 1);
    let m = num_cps.min(n).max(p + 1);
    let params = chord_length_params(points);
    let knots = build_approximation_knots(&params, p, m, n);
    assert_eq!(bits(&knots), bits(curve.knots()));
    for &u in &params {
        let span = find_span(u, p, &knots, m);
        let bf = basis_funs(span, u, p, &knots);
        let tail = point_from_basis(&bf, span, p, curve.control_points(), curve.weights());
        let eval = curve.evaluate(u);
        assert_eq!(point_bits(&[tail]), point_bits(&[eval]), "u = {u:e}");
    }
}

/// Largest data-point deviation of `curve`, measured with `evaluate` in
/// the order and arithmetic the LSPIA loop uses.
fn max_deviation(points: &[Point3], curve: &NurbsCurve) -> f64 {
    let mut max_err = 0.0f64;
    for (q, &u) in points.iter().zip(&chord_length_params(points)) {
        let c = curve.evaluate(u);
        let (err_x, err_y, err_z) = (q.x() - c.x(), q.y() - c.y(), q.z() - c.z());
        max_err = super::fold_residual(
            max_err,
            (err_x * err_x + err_y * err_y + err_z * err_z).sqrt(),
        );
    }
    max_err
}

/// The fit `approximate_lspia` must return, built from the verbatim loop's
/// iterates: `approximate_lspia_reference(.., j)` is iterate `j` (or the
/// first converged one before it).
fn reference_fit(
    points: &[Point3],
    degree: usize,
    num_cps: usize,
    tolerance: f64,
    max_iterations: usize,
) -> Result<LspiaFit, MathError> {
    let mut best: Option<LspiaFit> = None;
    for iteration in 0..=max_iterations {
        let curve = approximate_lspia_reference(points, degree, num_cps, tolerance, iteration)?;
        let max_error = max_deviation(points, &curve);
        let converged = max_error < tolerance;
        let fit = LspiaFit {
            curve,
            max_error,
            iteration,
            converged,
        };
        if converged {
            return Ok(fit);
        }
        if best.as_ref().is_none_or(|b| max_error < b.max_error) {
            best = Some(fit);
        }
    }
    Ok(best.expect("at least one iterate"))
}

/// Run both loops and require bit-identical results; return the new one.
fn assert_same(
    points: &[Point3],
    degree: usize,
    num_cps: usize,
    tolerance: f64,
    max_iterations: usize,
) -> Result<LspiaFit, MathError> {
    let new = approximate_lspia(points, degree, num_cps, tolerance, max_iterations);
    let old = reference_fit(points, degree, num_cps, tolerance, max_iterations);
    match (&new, &old) {
        (Ok(a), Ok(b)) => {
            assert_eq!(a.iteration, b.iteration);
            assert_eq!(a.converged, b.converged);
            assert_eq!(a.max_error.to_bits(), b.max_error.to_bits());
            let (a, b) = (&a.curve, &b.curve);
            assert_eq!(a.degree(), b.degree());
            assert_eq!(bits(a.knots()), bits(b.knots()));
            assert_eq!(
                point_bits(a.control_points()),
                point_bits(b.control_points())
            );
            assert_eq!(bits(a.weights()), bits(b.weights()));
            assert_tail_matches_evaluate(points, degree, num_cps, a);
        }
        // `MathError` has no `PartialEq`, and a NaN payload defeats a float
        // compare; `Debug` prints every variant and field exactly.
        (Err(a), Err(b)) => assert_eq!(format!("{a:?}"), format!("{b:?}")),
        _ => panic!("cached loop returned {new:?}, reference returned {old:?}"),
    }
    new
}

fn helix(n: usize) -> Vec<Point3> {
    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64 * 4.0 * std::f64::consts::PI;
            Point3::new(t.cos(), t.sin(), 0.15 * t)
        })
        .collect()
}

fn circle(n: usize) -> Vec<Point3> {
    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64 * std::f64::consts::TAU;
            Point3::new(2.5 * t.cos(), 2.5 * t.sin(), 0.0)
        })
        .collect()
}

/// A sine with deterministic pseudo-random noise (64-bit LCG).
fn noisy_sine(n: usize) -> Vec<Point3> {
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut noise = || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((state >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 1e-3
    };
    (0..n)
        .map(|i| {
            let x = i as f64 / (n - 1) as f64 * 10.0;
            Point3::new(x + noise(), x.sin() + noise(), noise())
        })
        .collect()
}

#[test]
fn matches_reference_on_smooth_curves() {
    assert_same(&helix(200), 3, 66, 1e-6, 100).unwrap();
    assert_same(&circle(120), 3, 40, 1e-6, 100).unwrap();
    assert_same(&noisy_sine(400), 3, 133, 1e-6, 100).unwrap();
    assert_same(&helix(200), 2, 66, 1e-6, 100).unwrap();
    assert_same(&helix(200), 5, 66, 1e-6, 100).unwrap();
}

#[test]
fn matches_reference_on_bezier_and_minimal_inputs() {
    // m == p + 1: no interior knots, one span.
    assert_same(&helix(50), 3, 4, 1e-6, 100).unwrap();
    // n == 2 clamps p to 1 and m to 2.
    let two = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0)];
    assert_same(&two, 3, 5, 1e-10, 100).unwrap();
    // A straight line converges within the first iterations.
    let line: Vec<Point3> = (0..50)
        .map(|i| {
            let t = i as f64 / 49.0;
            Point3::new(t, 2.0 * t, -t)
        })
        .collect();
    assert_same(&line, 3, 10, 1e-6, 100).unwrap();
}

#[test]
fn matches_reference_on_high_degrees() {
    // p > 8 takes `evaluate`'s heap basis buffer, p > 10 the heap
    // temporaries inside `basis_funs_into`.
    assert_same(&helix(60), 9, 20, 1e-6, 100).unwrap();
    assert_same(&helix(60), 12, 20, 1e-6, 100).unwrap();
}

#[test]
fn matches_reference_on_repeated_and_coincident_points() {
    let mut repeated = helix(80);
    for &i in &[10, 11, 40, 41, 41, 70] {
        let q = repeated[i];
        repeated.insert(i, q);
    }
    assert_same(&repeated, 3, 25, 1e-6, 100).unwrap();

    // Zero total chord length takes the uniform-parameter branch.
    let coincident = vec![Point3::new(1.0, -2.0, 0.5); 30];
    assert_same(&coincident, 3, 10, 1e-6, 100).unwrap();
}

#[test]
fn matches_reference_with_parameters_on_interior_knots() {
    // n / (num_interior + 1) = 60 / 12 = 5 exactly, so every interior knot
    // is params[5 * j] verbatim and those parameters sit on a knot.
    let pts = noisy_sine(60);
    let params = chord_length_params(&pts);
    let knots = build_approximation_knots(&params, 3, 15, 60);
    for j in 1..=11 {
        assert_eq!(knots[3 + j].to_bits(), params[5 * j].to_bits());
    }
    assert_same(&pts, 3, 15, 1e-6, 100).unwrap();
}

#[test]
fn matches_reference_across_iteration_counts() {
    let pts = helix(120);
    let initial = assert_same(&pts, 3, 40, 1e-6, 0).unwrap();
    // A loose tolerance converges at iteration 0 on the initial polygon.
    let converged = assert_same(&pts, 3, 40, 1e3, 100).unwrap();
    assert!(converged.converged && converged.iteration == 0);
    assert_eq!(
        point_bits(converged.curve.control_points()),
        point_bits(initial.curve.control_points())
    );
    let one = assert_same(&pts, 3, 40, 1e-6, 1).unwrap();
    assert_eq!(one.iteration, 1);
    assert_ne!(
        point_bits(one.curve.control_points()),
        point_bits(initial.curve.control_points())
    );
    // A zero tolerance never converges, so every iteration runs.
    assert_same(&pts, 3, 40, 0.0, 100).unwrap();
}

#[test]
fn matches_reference_on_validation_errors() {
    // Degree 0 fails the degree check.
    let err = assert_same(&helix(30), 0, 10, 1e-6, 100).unwrap_err();
    assert!(matches!(err, MathError::InvalidDegree { degree: 0, .. }));

    // A NaN beyond points[0] poisons every parameter and so every interior
    // knot; the knot check fires before any evaluation.
    let mut nan_mid = helix(30);
    nan_mid[7] = Point3::new(f64::NAN, 0.0, 0.0);
    let err = assert_same(&nan_mid, 3, 10, 1e-6, 100).unwrap_err();
    assert!(matches!(err, MathError::InvalidKnotValue { .. }));

    // With no interior knots the knots stay valid; a NaN at points[0] is
    // copied into every control point and fails the control-point check.
    let mut nan_first = helix(30);
    nan_first[0] = Point3::new(f64::NAN, 0.0, 0.0);
    for max_iterations in [0, 1, 100] {
        let err = assert_same(&nan_first, 3, 4, 1e-6, max_iterations).unwrap_err();
        assert!(matches!(
            err,
            MathError::InvalidControlPointValue { index: 0, .. }
        ));
    }
}

#[test]
fn finite_constant_coordinate_survives_the_extreme_lspia_evaluation() {
    // This previously expected InvalidControlPointValue because the finite
    // x=1.5e308 homogeneous sum overflowed. The corrected exceptional position
    // contraction must preserve that constant coordinate in both cached and
    // uncached fits, with a finite measured residual and no weakened guards.
    let pts: Vec<Point3> = (0..120)
        .map(|i| Point3::new(1.5e308, i as f64 * 0.01, (i as f64 * 0.1).sin()))
        .collect();
    for iterations in [0, 1, 100] {
        let fit = assert_same(&pts, 3, 40, 1e-6, iterations).unwrap();
        assert!(
            fit.max_error.is_finite(),
            "iterations={iterations}: {fit:?}"
        );
        assert!(
            fit.curve
                .control_points()
                .iter()
                .all(|point| point.0.iter().all(|value| value.is_finite()))
        );
        for u in [0.0, 0.125, 0.5, 0.875, 1.0] {
            let point = fit.curve.evaluate(u);
            assert_eq!(point.x().to_bits(), 1.5e308_f64.to_bits());
            assert!(point.0.iter().all(|value| value.is_finite()));
        }
    }
}

#[test]
fn matches_reference_on_nan_parameters() {
    // m == p + 1 keeps the knots valid while a NaN beyond points[0] makes
    // every parameter NaN; the initial polygon is all points[0], so both
    // loops reach the NaN evaluation. Its zero scale trips the scale
    // `debug_assert!` in both.
    let mut pts = helix(30);
    pts[7] = Point3::new(f64::NAN, 0.0, 0.0);
    let new = std::panic::catch_unwind(|| approximate_lspia(&pts, 3, 4, 1e-6, 100));
    let old = std::panic::catch_unwind(|| approximate_lspia_reference(&pts, 3, 4, 1e-6, 100));
    if cfg!(debug_assertions) {
        assert!(new.is_err() && old.is_err());
    } else {
        // In release every residual is NaN. The old loop's `f64::max`
        // dropped them, measured 0 and stopped at the initial polygon as
        // if converged. Review follow-up: a NaN residual now counts as
        // infinite, so no iterate converges; the first update writes NaN
        // control points and is refused, and with no update the initial
        // polygon is returned unconverged with an infinite error.
        assert!(old.unwrap().is_ok());
        assert!(matches!(
            new.unwrap(),
            Err(MathError::InvalidControlPointValue { index: 0, .. })
        ));
        let fit = approximate_lspia(&pts, 3, 4, 1e-6, 0).unwrap();
        assert!(!fit.converged);
        assert!(fit.max_error.is_infinite());
        assert_eq!(fit.iteration, 0);
    }
}

/// Thirty collinear points with one spike at (15, 5, 0).
fn spike() -> Vec<Point3> {
    (0..30)
        .map(|i| Point3::new(i as f64, if i == 15 { 5.0 } else { 0.0 }, 0.0))
        .collect()
}

#[test]
fn returns_the_best_iterate_not_the_last() {
    // Kernel review MATH-9. With eight control points the maximum deviation
    // falls from 4.508 (the initial polygon) to 3.560 after one update, then
    // climbs and is still 3.613 after sixty; the old loop returned the
    // sixtieth.
    let pts = spike();
    let last = max_deviation(
        &pts,
        &approximate_lspia_reference(&pts, 3, 8, 0.0, 60).unwrap(),
    );
    let first = max_deviation(
        &pts,
        &approximate_lspia_reference(&pts, 3, 8, 0.0, 1).unwrap(),
    );
    assert!(
        (first - 3.5603).abs() < 1e-4 && (last - 3.6126).abs() < 1e-4,
        "premise: {first} after one update, {last} after sixty"
    );
    let fit = assert_same(&pts, 3, 8, 0.0, 60).unwrap();
    assert_eq!(fit.iteration, 1);
    assert!(!fit.converged);
    assert_eq!(fit.max_error.to_bits(), first.to_bits());

    // With five control points no update beats the initial polygon (2.585
    // against 3.519 after sixty), so the polygon itself comes back.
    let fit = assert_same(&pts, 3, 5, 0.0, 60).unwrap();
    assert_eq!(fit.iteration, 0);
    assert!(!fit.converged);
    assert!((fit.max_error - 2.5848).abs() < 1e-4, "{}", fit.max_error);
}

#[test]
fn reports_convergence_only_below_the_tolerance() {
    let pts = spike();
    // 3.560 after one update is below 3.6: converged there.
    let fit = assert_same(&pts, 3, 8, 3.6, 60).unwrap();
    assert!(fit.converged && fit.iteration == 1);
    // No iterate reaches 3.5: the best one comes back, flagged.
    let fit = assert_same(&pts, 3, 8, 3.5, 60).unwrap();
    assert!(!fit.converged && fit.max_error >= 3.5);

    // The last update is measured too. On a helix the deviation falls
    // monotonically over the first updates; a tolerance between the fourth
    // and fifth iterates converges exactly at iteration five when five
    // updates are allowed, which the old loop returned unmeasured.
    let pts = helix(120);
    let errs: Vec<f64> = (0..=5)
        .map(|k| {
            max_deviation(
                &pts,
                &approximate_lspia_reference(&pts, 3, 40, 0.0, k).unwrap(),
            )
        })
        .collect();
    assert!(errs.windows(2).all(|w| w[1] < w[0]), "premise: {errs:?}");
    let tolerance = f64::midpoint(errs[4], errs[5]);
    let fit = assert_same(&pts, 3, 40, tolerance, 5).unwrap();
    assert!(fit.converged && fit.iteration == 5);
}
