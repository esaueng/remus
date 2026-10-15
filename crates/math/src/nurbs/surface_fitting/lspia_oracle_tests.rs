//! Bit-identity oracle for `approximate_surface_lspia`'s best-iterate result.
//!
//! `approximate_surface_lspia_reference` is the loop as it stood before it
//! measured every iterate, kept verbatim: it returns the first iterate under
//! the tolerance, or else the grid after `max_iterations` updates unmeasured.
//! It shares the step-size helper `col_sum_lambda` with the kernel, so both
//! walk the same iterates under the corrected step.
//! Run once per budget `k`, it recovers iterate `k` of the same sequence, so
//! each case checks the new result against every iterate bit for bit.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::cast_lossless
)]

use super::*;

/// `approximate_surface_lspia` before it measured every iterate, verbatim.
#[allow(
    clippy::too_many_arguments,
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::needless_range_loop
)]
fn approximate_surface_lspia_reference(
    points: &[Vec<Point3>],
    degree_u: usize,
    degree_v: usize,
    num_cps_u: usize,
    num_cps_v: usize,
    tolerance: f64,
    max_iterations: usize,
) -> Result<NurbsSurface, MathError> {
    if points.is_empty() || points[0].is_empty() {
        return Err(MathError::EmptyInput);
    }
    let rows = points.len();
    let cols = points[0].len();
    for row in points {
        if row.len() != cols {
            return Err(MathError::InvalidControlPointGrid {
                expected_rows: rows,
                expected_cols: cols,
            });
        }
    }

    let pu = degree_u.min(rows - 1);
    let pv = degree_v.min(cols - 1);
    let mu = num_cps_u.min(rows).max(pu + 1);
    let mv = num_cps_v.min(cols).max(pv + 1);

    let params_u = compute_grid_params_u(points, rows, cols);
    let params_v = compute_grid_params_v(points, rows, cols);

    let knots_u = fitting::build_approximation_knots(&params_u, pu, mu, rows);
    let knots_v = fitting::build_approximation_knots(&params_v, pv, mv, cols);

    // Initialize control points grid (mu x mv).
    let mut cps: Vec<Vec<Point3>> = Vec::with_capacity(mu);
    for i in 0..mu {
        let mut row = Vec::with_capacity(mv);
        for j in 0..mv {
            let ui = if mu > 1 { i * (rows - 1) / (mu - 1) } else { 0 };
            let vj = if mv > 1 { j * (cols - 1) / (mv - 1) } else { 0 };
            row.push(points[ui.min(rows - 1)][vj.min(cols - 1)]);
        }
        cps.push(row);
    }

    let weights = vec![vec![1.0; mv]; mu];

    let basis_u_data: Vec<(usize, Vec<f64>)> = params_u
        .iter()
        .map(|&u| {
            let span = fitting::find_span(u, pu, &knots_u, mu);
            let n = basis_funs(span, u, pu, &knots_u);
            (span, n)
        })
        .collect();
    let basis_v_data: Vec<(usize, Vec<f64>)> = params_v
        .iter()
        .map(|&v| {
            let span = fitting::find_span(v, pv, &knots_v, mv);
            let n = basis_funs(span, v, pv, &knots_v);
            (span, n)
        })
        .collect();

    // Compute step size for LSPIA convergence.
    // Estimate lambda_max per direction via max column sum of squared basis values.
    let mu_u = col_sum_lambda(&basis_u_data, pu, mu);
    let mu_v = col_sum_lambda(&basis_v_data, pv, mv);
    let lambda_max = mu_u * mu_v;
    let step: f64 = if lambda_max < 1e-30 {
        1.0
    } else {
        1.0 / lambda_max
    };

    for iter in 0..max_iterations {
        let surface = NurbsSurface::new(
            pu,
            pv,
            knots_u.clone(),
            knots_v.clone(),
            cps.clone(),
            weights.clone(),
        )?;

        let mut max_err = 0.0f64;
        let mut deltas = vec![vec![(0.0f64, 0.0f64, 0.0f64); mv]; mu];

        for (i, (su, nu)) in basis_u_data.iter().enumerate() {
            for (j, (sv, nv)) in basis_v_data.iter().enumerate() {
                let q = surface.evaluate(params_u[i], params_v[j]);
                let err_x = points[i][j].x() - q.x();
                let err_y = points[i][j].y() - q.y();
                let err_z = points[i][j].z() - q.z();
                let err_mag = (err_x * err_x + err_y * err_y + err_z * err_z).sqrt();
                max_err = max_err.max(err_mag);

                for (ku, &bu) in nu.iter().enumerate() {
                    for (kv, &bv) in nv.iter().enumerate() {
                        let ci = su - pu + ku;
                        let cj = sv - pv + kv;
                        if ci < mu && cj < mv {
                            let w = bu * bv;
                            deltas[ci][cj].0 += w * err_x;
                            deltas[ci][cj].1 += w * err_y;
                            deltas[ci][cj].2 += w * err_z;
                        }
                    }
                }
            }
        }

        if max_err < tolerance {
            return NurbsSurface::new(pu, pv, knots_u, knots_v, cps, weights);
        }

        // Update control points: CP += step * delta.
        for i in 0..mu {
            for j in 0..mv {
                cps[i][j] = Point3::new(
                    step.mul_add(deltas[i][j].0, cps[i][j].x()),
                    step.mul_add(deltas[i][j].1, cps[i][j].y()),
                    step.mul_add(deltas[i][j].2, cps[i][j].z()),
                );
            }
        }

        if iter == max_iterations - 1 {
            return NurbsSurface::new(pu, pv, knots_u, knots_v, cps, weights);
        }
    }

    NurbsSurface::new(pu, pv, knots_u, knots_v, cps, weights)
}

fn point_bits(grid: &[Vec<Point3>]) -> Vec<Vec<[u64; 3]>> {
    grid.iter()
        .map(|row| {
            row.iter()
                .map(|q| [q.x().to_bits(), q.y().to_bits(), q.z().to_bits()])
                .collect()
        })
        .collect()
}

fn assert_same_surface(a: &NurbsSurface, b: &NurbsSurface, what: &str) {
    let bits = |k: &[f64]| k.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
    assert_eq!(a.degree_u(), b.degree_u(), "{what}: degree_u");
    assert_eq!(a.degree_v(), b.degree_v(), "{what}: degree_v");
    assert_eq!(bits(a.knots_u()), bits(b.knots_u()), "{what}: knots_u");
    assert_eq!(bits(a.knots_v()), bits(b.knots_v()), "{what}: knots_v");
    assert_eq!(
        point_bits(a.control_points()),
        point_bits(b.control_points()),
        "{what}: control points"
    );
    let wa: Vec<_> = a.weights().iter().map(|r| bits(r)).collect();
    let wb: Vec<_> = b.weights().iter().map(|r| bits(r)).collect();
    assert_eq!(wa, wb, "{what}: weights");
}

/// The fit's own measurement, as the kernel loop computes it.
fn kernel_max_error(grid: &[Vec<Point3>], surface: &NurbsSurface) -> f64 {
    let rows = grid.len();
    let cols = grid[0].len();
    let params_u = compute_grid_params_u(grid, rows, cols);
    let params_v = compute_grid_params_v(grid, rows, cols);
    let mut max_err = 0.0f64;
    for i in 0..rows {
        for j in 0..cols {
            let q = surface.evaluate(params_u[i], params_v[j]);
            let e = (
                grid[i][j].x() - q.x(),
                grid[i][j].y() - q.y(),
                grid[i][j].z() - q.z(),
            );
            max_err = max_err.max((e.0 * e.0 + e.1 * e.1 + e.2 * e.2).sqrt());
        }
    }
    max_err
}

/// For every budget up to `max_budget`, check the new result against the
/// reference iterates: the first under `tolerance` if any, else the first
/// with the smallest deviation, bit for bit.
fn assert_best_iterate(
    grid: &[Vec<Point3>],
    degree: usize,
    num_cps: usize,
    tolerance: f64,
    max_budget: usize,
) {
    // A zero tolerance never converges, so the reference returns iterate `k`
    // (the grid after `k` updates) for budget `k`.
    let iterates: Vec<NurbsSurface> = (0..=max_budget)
        .map(|k| {
            approximate_surface_lspia_reference(grid, degree, degree, num_cps, num_cps, 0.0, k)
                .unwrap()
        })
        .collect();
    let errors: Vec<f64> = iterates.iter().map(|s| kernel_max_error(grid, s)).collect();

    for budget in 0..=max_budget {
        let what = format!("degree {degree}, {num_cps} cps, tol {tolerance:e}, budget {budget}");
        let fit =
            approximate_surface_lspia(grid, degree, degree, num_cps, num_cps, tolerance, budget)
                .unwrap();
        let expected = errors[..=budget]
            .iter()
            .position(|&e| e < tolerance)
            .map_or_else(
                || {
                    let mut best = 0;
                    for (k, &e) in errors[..=budget].iter().enumerate() {
                        if e < errors[best] {
                            best = k;
                        }
                    }
                    (best, false)
                },
                |k| (k, true),
            );
        assert_eq!((fit.iteration, fit.converged), expected, "{what}");
        assert_eq!(
            fit.max_error.to_bits(),
            errors[fit.iteration].to_bits(),
            "{what}: max_error"
        );
        assert_same_surface(&fit.surface, &iterates[fit.iteration], &what);

        // A converged result is the grid the old loop returned.
        if fit.converged {
            let old = approximate_surface_lspia_reference(
                grid, degree, degree, num_cps, num_cps, tolerance, budget,
            )
            .unwrap();
            assert_same_surface(&fit.surface, &old, &format!("{what}: converged vs old"));
        }
    }
}

fn raised_flat_grid(n: usize, raised: (usize, usize)) -> Vec<Vec<Point3>> {
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| Point3::new(i as f64, j as f64, if (i, j) == raised { 1.0 } else { 0.0 }))
                .collect()
        })
        .collect()
}

fn paraboloid(n: usize) -> Vec<Vec<Point3>> {
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let u = i as f64 / (n - 1) as f64;
                    let v = j as f64 / (n - 1) as f64;
                    Point3::new(u, v, u * u + v * v)
                })
                .collect()
        })
        .collect()
}

#[test]
fn best_iterate_matches_reference_on_raised_grids() {
    // Non-monotone maximum deviation with a falling residual.
    assert_best_iterate(&raised_flat_grid(7, (3, 3)), 1, 3, 1e-6, 30);
    assert_best_iterate(&raised_flat_grid(5, (2, 2)), 2, 3, 1e-6, 30);
    // Diverged under the old step-size estimate; now converges slowly.
    assert_best_iterate(&raised_flat_grid(7, (1, 1)), 2, 5, 1e-6, 30);
    // Monotone deviation: every budget returns its own last iterate.
    assert_best_iterate(&raised_flat_grid(6, (1, 1)), 1, 6, 1e-6, 30);
}

#[test]
fn best_iterate_matches_reference_when_converging() {
    // 7×7 raised grid: deviations 0.889, 0.593, … converge under 0.7 at
    // iterate 1. 6×6 raised grid, interpolating: 0.943, 0.458, 0.291, …
    assert_best_iterate(&raised_flat_grid(7, (3, 3)), 1, 3, 0.7, 30);
    assert_best_iterate(&raised_flat_grid(6, (1, 1)), 1, 6, 0.2, 30);
    // Loose enough that the initial grid already converges.
    assert_best_iterate(&paraboloid(8), 3, 6, 10.0, 3);

    // The cases above do converge within their largest budget.
    let fit = approximate_surface_lspia(&raised_flat_grid(7, (3, 3)), 1, 1, 3, 3, 0.7, 30).unwrap();
    assert!(fit.converged && fit.iteration == 1, "7x7: {fit:?}");
    let fit = approximate_surface_lspia(&raised_flat_grid(6, (1, 1)), 1, 1, 6, 6, 0.2, 30).unwrap();
    assert!(fit.converged && fit.iteration > 0, "6x6: {fit:?}");
    let fit = approximate_surface_lspia(&paraboloid(8), 3, 3, 6, 6, 10.0, 3).unwrap();
    assert!(fit.converged && fit.iteration == 0, "paraboloid: {fit:?}");
}

/// Pins the premise of the regression: on this grid the residual falls at
/// every step while the maximum deviation rises after iterate 1.
#[test]
fn raised_grid_residual_falls_while_deviation_rises() {
    let grid = raised_flat_grid(7, (3, 3));
    let measure = |k: usize| {
        let s = approximate_surface_lspia_reference(&grid, 1, 1, 3, 3, 0.0, k).unwrap();
        let params_u = compute_grid_params_u(&grid, 7, 7);
        let params_v = compute_grid_params_v(&grid, 7, 7);
        let mut sse = 0.0;
        for i in 0..7 {
            for j in 0..7 {
                sse += (s.evaluate(params_u[i], params_v[j]) - grid[i][j]).length_squared();
            }
        }
        (sse, kernel_max_error(&grid, &s))
    };
    let runs: Vec<(f64, f64)> = (0..=30).map(measure).collect();
    for w in runs.windows(2) {
        assert!(w[1].0 <= w[0].0, "residual rose: {w:?}");
    }
    assert!(
        runs[2].1 > runs[1].1,
        "deviation did not rise: {:?}",
        &runs[..3]
    );
    assert!(
        runs[30].1 > runs[1].1,
        "last iterate not worse: {:?}",
        (runs[1], runs[30])
    );
}

#[test]
fn invalid_grids_fail_like_the_reference() {
    let empty: Vec<Vec<Point3>> = Vec::new();
    let ragged = vec![
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        vec![Point3::new(0.0, 1.0, 0.0)],
    ];
    for grid in [&empty, &ragged] {
        let new = approximate_surface_lspia(grid, 1, 1, 2, 2, 1e-6, 5).unwrap_err();
        let old = approximate_surface_lspia_reference(grid, 1, 1, 2, 2, 1e-6, 5).unwrap_err();
        // `MathError` has no `PartialEq`; `Debug` prints every field.
        assert_eq!(format!("{new:?}"), format!("{old:?}"));
    }
}
