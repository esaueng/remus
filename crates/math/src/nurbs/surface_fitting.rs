//! NURBS surface fitting from a grid of data points via least-squares
//! progressive-iterative approximation (LSPIA).

#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_precision_loss,
    clippy::doc_markdown
)]

use crate::MathError;
use crate::nurbs::basis::basis_funs;
use crate::nurbs::fitting;
use crate::nurbs::fitting::interpolate_with_params;
use crate::nurbs::surface::NurbsSurface;
use crate::vec::Point3;

/// Interpolate a NURBS surface through a grid of data points.
///
/// The input is a row-major grid where `points[i][j]` is the point
/// at row `i`, column `j`. All rows must have the same number of
/// columns. The resulting surface passes through every input point.
///
/// # Parameters
///
/// - `points` — 2D grid of data points (rows × cols)
/// - `degree_u` — polynomial degree in the u (row) direction, clamped to
///   `rows - 1` like [`fitting::interpolate`]
/// - `degree_v` — polynomial degree in the v (column) direction, clamped to
///   `cols - 1`
///
/// # Algorithm
///
/// Global tensor-product interpolation (NURBS Book §9.2.5, A9.4):
/// 1. One u parameter vector: the chord-length parameters of every column,
///    averaged; one v parameter vector likewise over every row. Each
///    direction gets one averaging knot vector from its parameters.
/// 2. Fit every row through its points at the shared v parameters
/// 3. Fit every column of those control points at the shared u parameters
/// 4. The final control point grid and the two shared knot vectors form
///    the surface, which passes through `points[i][j]` at `(u_i, v_j)`
///
/// # Errors
///
/// Returns an error if the grid is too small, rows have inconsistent
/// lengths, a degree is zero, or two adjacent rows or columns coincide
/// everywhere (the shared parameters would not be strictly increasing).
pub fn interpolate_surface(
    points: &[Vec<Point3>],
    degree_u: usize,
    degree_v: usize,
) -> Result<NurbsSurface, MathError> {
    let num_rows = points.len();
    if num_rows < 2 {
        return Err(MathError::EmptyInput);
    }

    let num_cols = points[0].len();
    if num_cols < 2 {
        return Err(MathError::EmptyInput);
    }

    for row in points {
        if row.len() != num_cols {
            return Err(MathError::InvalidControlPointGrid {
                expected_rows: num_rows,
                expected_cols: num_cols,
            });
        }
    }

    let degree_u = degree_u.min(num_rows - 1);
    let degree_v = degree_v.min(num_cols - 1);

    // Every row and column must share one parameter vector (and so one knot
    // vector) per direction: fitting each with its own chord lengths and then
    // assembling with one line's knots misses the data on non-uniform grids.
    let params_u = averaged_chord_params(num_cols, num_rows, |j, i| points[i][j]);
    let params_v = averaged_chord_params(num_rows, num_cols, |i, j| points[i][j]);

    // Step 1: Fit curves through each row (v-direction).
    let mut row_curves = Vec::with_capacity(num_rows);
    for row in points {
        let curve = interpolate_with_params(row, degree_v, &params_v)?;
        row_curves.push(curve);
    }

    // All row curves have num_cols control points (exact interpolation).
    let n_cp_v = row_curves[0].control_points().len();

    // Step 2: Extract column-wise control points from row curves.
    // cp_grid[col][row] = row_curves[row].control_point[col]
    let mut col_points: Vec<Vec<Point3>> = Vec::with_capacity(n_cp_v);
    for j in 0..n_cp_v {
        let col: Vec<Point3> = row_curves.iter().map(|c| c.control_points()[j]).collect();
        col_points.push(col);
    }

    // Step 3: Fit curves through each column (u-direction).
    let mut col_curves = Vec::with_capacity(n_cp_v);
    for col in &col_points {
        let curve = interpolate_with_params(col, degree_u, &params_u)?;
        col_curves.push(curve);
    }

    // Step 4: Extract the final control point grid and the shared knot
    // vectors (identical across rows, and across columns, by construction).
    let n_cp_u = col_curves[0].control_points().len();
    let knots_u = col_curves[0].knots().to_vec();
    let knots_v = row_curves[0].knots().to_vec();

    let mut control_points: Vec<Vec<Point3>> = Vec::with_capacity(n_cp_u);
    for i in 0..n_cp_u {
        let row: Vec<Point3> = col_curves.iter().map(|c| c.control_points()[i]).collect();
        control_points.push(row);
    }

    let weights: Vec<Vec<f64>> = control_points
        .iter()
        .map(|row| vec![1.0; row.len()])
        .collect();

    NurbsSurface::new(
        degree_u,
        degree_v,
        knots_u,
        knots_v,
        control_points,
        weights,
    )
}

/// Result of [`approximate_surface_lspia`]: the returned iterate and how well
/// it fits.
#[derive(Debug, Clone)]
pub struct LspiaSurfaceFit {
    /// The returned surface.
    pub surface: NurbsSurface,
    /// Maximum distance between a data point and the surface at that point's
    /// averaged chord-length parameters.
    pub max_error: f64,
    /// Number of LSPIA updates applied to the initial grid (0 = initial grid).
    pub iteration: usize,
    /// Whether `max_error` is below the requested tolerance.
    pub converged: bool,
}

/// Approximate a NURBS surface through a grid of data points using LSPIA.
///
/// Uses tensor-product basis `N_i(u) * N_j(v)` for iterative refinement.
/// This is more efficient than direct least-squares for large grids, achieving
/// O(rows * cols) per iteration.
///
/// # Parameters
///
/// - `points` -- 2D grid of data points (rows x cols)
/// - `degree_u`, `degree_v` -- polynomial degrees in each direction
/// - `num_cps_u`, `num_cps_v` -- number of control points in each direction
/// - `tolerance` -- convergence threshold for max point deviation
/// - `max_iterations` -- maximum number of PIA updates
///
/// # Result
///
/// Every one of the `max_iterations + 1` iterates (the initial grid and the
/// grid after each update) is measured at the averaged chord-length
/// parameters. The first iterate whose maximum deviation is below `tolerance`
/// comes back with `converged: true`. Otherwise the first iterate with the
/// smallest maximum deviation comes back with `converged: false`: LSPIA's
/// least-squares residual falls monotonically, but its maximum deviation need
/// not, so the last iterate can be worse than an earlier one. A run stops at
/// the first later iterate that overflows (non-finite control points or
/// deviation) and returns the best iterate measured before it.
///
/// # Errors
///
/// Returns [`MathError::EmptyInput`] if the grid is empty.
/// Returns [`MathError::InvalidControlPointGrid`] if rows have inconsistent lengths.
/// Not converging is not an error; check [`LspiaSurfaceFit::converged`].
#[allow(
    clippy::too_many_arguments,
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::needless_range_loop
)]
pub fn approximate_surface_lspia(
    points: &[Vec<Point3>],
    degree_u: usize,
    degree_v: usize,
    num_cps_u: usize,
    num_cps_v: usize,
    tolerance: f64,
    max_iterations: usize,
) -> Result<LspiaSurfaceFit, MathError> {
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
    // Upper bound on lambda_max of the tensor normal matrix, one factor per
    // direction (see `col_sum_lambda`); 1 / bound stays inside LSPIA's
    // convergence interval (0, 2 / lambda_max).
    let mu_u = col_sum_lambda(&basis_u_data, pu, mu);
    let mu_v = col_sum_lambda(&basis_v_data, pv, mv);
    let lambda_max = mu_u * mu_v;
    let step: f64 = if lambda_max < 1e-30 {
        1.0
    } else {
        1.0 / lambda_max
    };

    // Best non-converged iterate so far: (max deviation, iteration, grid).
    let mut best: Option<(f64, usize, Vec<Vec<Point3>>)> = None;

    for iter in 0..=max_iterations {
        let surface = match NurbsSurface::new(
            pu,
            pv,
            knots_u.clone(),
            knots_v.clone(),
            cps.clone(),
            weights.clone(),
        ) {
            Ok(surface) => surface,
            // Defensive: under the bounded step the iteration contracts, but
            // an iterate that overflows must not turn the best fit into an
            // error.
            Err(_) if best.is_some() => break,
            Err(e) => return Err(e),
        };

        let mut max_err = 0.0f64;
        let mut finite = true;
        let mut deltas = vec![vec![(0.0f64, 0.0f64, 0.0f64); mv]; mu];

        for (i, (su, nu)) in basis_u_data.iter().enumerate() {
            for (j, (sv, nv)) in basis_v_data.iter().enumerate() {
                let q = surface.evaluate(params_u[i], params_v[j]);
                let err_x = points[i][j].x() - q.x();
                let err_y = points[i][j].y() - q.y();
                let err_z = points[i][j].z() - q.z();
                let err_mag = (err_x * err_x + err_y * err_y + err_z * err_z).sqrt();
                max_err = max_err.max(err_mag);
                finite &= err_mag.is_finite();

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

        // `f64::max` skips NaN, so an unmeasurable iterate could look good.
        if !finite && best.is_some() {
            break;
        }

        if max_err < tolerance {
            return Ok(LspiaSurfaceFit {
                surface,
                max_error: max_err,
                iteration: iter,
                converged: true,
            });
        }

        // Strict `<`: on a tie the earlier iterate wins.
        if best.as_ref().is_none_or(|(e, _, _)| max_err < *e) {
            best = Some((max_err, iter, cps.clone()));
        }

        if iter == max_iterations {
            break;
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
    }

    // The loop measures iterate 0 at least once, so `best` is set unless
    // iterate 0 converged and returned above.
    let (max_error, iteration, best_cps) = best.ok_or(MathError::EmptyInput)?;
    Ok(LspiaSurfaceFit {
        surface: NurbsSurface::new(pu, pv, knots_u, knots_v, best_cps, weights)?,
        max_error,
        iteration,
        converged: false,
    })
}

/// Chord-length parameters of `num_lines` grid lines of `len` points each,
/// averaged (NURBS Book A9.4); `at(line, k)` is point `k` of `line`.
///
/// A line of zero total length (a pole) has no spacing to contribute and is
/// skipped; if every line is degenerate the parameters are uniform.
#[allow(clippy::cast_precision_loss)]
fn averaged_chord_params(
    num_lines: usize,
    len: usize,
    at: impl Fn(usize, usize) -> Point3,
) -> Vec<f64> {
    let mut params = vec![0.0f64; len];
    let mut chord = vec![0.0f64; len];
    let mut used = 0usize;
    for line in 0..num_lines {
        let mut total = 0.0f64;
        for k in 1..len {
            total += (at(line, k) - at(line, k - 1)).length();
            chord[k] = total;
        }
        // Same coincidence threshold as `fitting::chord_length_params`.
        if total < 1e-15 {
            continue;
        }
        used += 1;
        // Running mean: exact when every line has the same spacing, so a
        // regular grid keeps its per-line parameters bit for bit.
        let used_f = used as f64;
        for k in 1..len {
            params[k] += (chord[k] / total - params[k]) / used_f;
        }
    }
    if used == 0 {
        let last = (len - 1) as f64;
        return (0..len).map(|k| k as f64 / last).collect();
    }
    params
}

/// Compute average chord-length parameters in the u direction (across rows).
#[allow(clippy::cast_precision_loss)]
fn compute_grid_params_u(points: &[Vec<Point3>], rows: usize, cols: usize) -> Vec<f64> {
    let mut params = vec![0.0f64; rows];
    for j in 0..cols {
        let col: Vec<Point3> = (0..rows).map(|i| points[i][j]).collect();
        let col_params = fitting::chord_length_params(&col);
        for (i, &p) in col_params.iter().enumerate() {
            params[i] += p;
        }
    }
    let cols_f = cols as f64;
    for p in &mut params {
        *p /= cols_f;
    }
    params
}

/// Compute average chord-length parameters in the v direction (across columns).
#[allow(clippy::cast_precision_loss)]
fn compute_grid_params_v(points: &[Vec<Point3>], rows: usize, cols: usize) -> Vec<f64> {
    let mut params = vec![0.0f64; cols];
    for i in 0..rows {
        let row_params = fitting::chord_length_params(&points[i]);
        for (j, &p) in row_params.iter().enumerate() {
            params[j] += p;
        }
    }
    let rows_f = rows as f64;
    for p in &mut params {
        *p /= rows_f;
    }
    params
}

/// Upper bound on the spectral radius of `N^T N`: its largest row sum
/// (Gershgorin; every entry is non-negative), which equals the largest column
/// sum of `N` because each row of `N` sums to at most 1 (partition of unity).
///
/// The sum of *squared* basis values is the diagonal of `N^T N`, a lower
/// bound on lambda_max; a step from it can exceed 2 / lambda_max and diverge.
fn col_sum_lambda(basis_data: &[(usize, Vec<f64>)], p: usize, m: usize) -> f64 {
    let mut col_sums = vec![0.0f64; m];
    for (span, n_vals) in basis_data {
        for (k, &nv) in n_vals.iter().enumerate() {
            let j = span - p + k;
            if j < m {
                col_sums[j] += nv;
            }
        }
    }
    col_sums.iter().copied().fold(0.0f64, f64::max).max(1e-30)
}

#[cfg(test)]
mod lspia_oracle_tests;

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::cast_lossless, clippy::suboptimal_flops)]

    use crate::tolerance::Tolerance;
    use crate::vec::Point3;

    use super::*;

    #[test]
    fn interpolate_flat_grid() {
        // A 3×3 flat grid on the XY plane.
        let points = vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            vec![
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
            ],
            vec![
                Point3::new(0.0, 2.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(2.0, 2.0, 0.0),
            ],
        ];

        let surface = interpolate_surface(&points, 2, 2).unwrap();

        let tol = Tolerance::new();

        // Check corners.
        let p00 = surface.evaluate(0.0, 0.0);
        assert!(tol.approx_eq(p00.x(), 0.0), "corner (0,0) x: {}", p00.x());
        assert!(tol.approx_eq(p00.y(), 0.0), "corner (0,0) y: {}", p00.y());
        assert!(tol.approx_eq(p00.z(), 0.0), "corner (0,0) z: {}", p00.z());

        let p11 = surface.evaluate(1.0, 1.0);
        assert!(tol.approx_eq(p11.x(), 2.0), "corner (1,1) x: {}", p11.x());
        assert!(tol.approx_eq(p11.y(), 2.0), "corner (1,1) y: {}", p11.y());

        // All z should be 0 (flat grid).
        let p_mid = surface.evaluate(0.5, 0.5);
        assert!(
            p_mid.z().abs() < 0.01,
            "flat grid mid z should be ~0, got {}",
            p_mid.z()
        );
    }

    #[test]
    fn interpolate_curved_grid() {
        // A 3×3 grid with z-height forming a paraboloid.
        let points = vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 1.0),
                Point3::new(2.0, 0.0, 4.0),
            ],
            vec![
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(1.0, 1.0, 2.0),
                Point3::new(2.0, 1.0, 5.0),
            ],
            vec![
                Point3::new(0.0, 2.0, 4.0),
                Point3::new(1.0, 2.0, 5.0),
                Point3::new(2.0, 2.0, 8.0),
            ],
        ];

        let surface = interpolate_surface(&points, 2, 2).unwrap();

        let tol = Tolerance::new();
        // Check corners pass through.
        let p00 = surface.evaluate(0.0, 0.0);
        assert!(tol.approx_eq(p00.z(), 0.0), "corner z: {}", p00.z());

        let p11 = surface.evaluate(1.0, 1.0);
        assert!(tol.approx_eq(p11.z(), 8.0), "far corner z: {}", p11.z());
    }

    #[test]
    fn interpolate_surface_too_small() {
        let points = vec![vec![Point3::new(0.0, 0.0, 0.0)]];
        assert!(interpolate_surface(&points, 1, 1).is_err());
    }

    #[test]
    fn interpolate_surface_inconsistent_rows() {
        let points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![Point3::new(0.0, 1.0, 0.0)], // different length
        ];
        assert!(interpolate_surface(&points, 1, 1).is_err());
    }

    // ── Interpolation at shared averaged parameters ────────────────────

    /// Independent oracle for the NURBS Book A9.4 parameters: per-line
    /// chord lengths, summed and divided, skipping zero-length lines.
    fn oracle_params(lines: &[Vec<Point3>]) -> Vec<f64> {
        let len = lines[0].len();
        let mut sum = vec![0.0; len];
        let mut used = 0.0;
        for line in lines {
            let mut acc = vec![0.0; len];
            for k in 1..len {
                acc[k] = acc[k - 1] + (line[k] - line[k - 1]).length();
            }
            if acc[len - 1] < 1e-15 {
                continue;
            }
            used += 1.0;
            for k in 0..len {
                sum[k] += acc[k] / acc[len - 1];
            }
        }
        sum.iter().map(|s| s / used).collect()
    }

    fn columns(grid: &[Vec<Point3>]) -> Vec<Vec<Point3>> {
        (0..grid[0].len())
            .map(|j| grid.iter().map(|row| row[j]).collect())
            .collect()
    }

    /// Largest distance between a data point and the surface at its
    /// averaged parameters.
    fn max_data_miss(grid: &[Vec<Point3>], surface: &NurbsSurface) -> f64 {
        let params_u = oracle_params(&columns(grid));
        let params_v = oracle_params(grid);
        let mut worst = 0.0f64;
        for (i, row) in grid.iter().enumerate() {
            for (j, q) in row.iter().enumerate() {
                let p = surface.evaluate(params_u[i], params_v[j]);
                worst = worst.max((p - *q).length());
            }
        }
        worst
    }

    /// Rows at x = `xs`, columns at y = `ys`, sheared and warped so every
    /// row and column has its own chord spacing.
    fn warped_grid(xs: &[f64], ys: &[f64]) -> Vec<Vec<Point3>> {
        xs.iter()
            .map(|&x| {
                ys.iter()
                    .map(|&y| {
                        Point3::new(
                            x + 0.25 * y,
                            y + 0.15 * x * x,
                            0.5 * x.sin() * (0.7 * y).cos() + 0.1 * x * y,
                        )
                    })
                    .collect()
            })
            .collect()
    }

    /// Review finding WASM-7: rows at x = 0, 1, 3, columns at y = 0, 2, 2.5,
    /// centre raised to z = 1. Row-local parameters missed the centre point
    /// by 0.0432 (closest-point distance).
    #[test]
    fn interpolate_non_uniform_3x3_reproduces_data() {
        let xs = [0.0, 1.0, 3.0];
        let ys = [0.0, 2.0, 2.5];
        let grid: Vec<Vec<Point3>> = xs
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                ys.iter()
                    .enumerate()
                    .map(|(j, &y)| Point3::new(x, y, if i == 1 && j == 1 { 1.0 } else { 0.0 }))
                    .collect()
            })
            .collect();
        let surface = interpolate_surface(&grid, 2, 2).unwrap();
        let miss = max_data_miss(&grid, &surface);
        assert!(miss < 1e-9, "3x3 data miss {miss:e}");

        // Parameter-free: the surface passes through the centre point.
        let foot = crate::nurbs::projection::project_point_to_surface(&surface, grid[1][1], 1e-12)
            .unwrap();
        assert!(foot.distance < 1e-9, "centre distance {:e}", foot.distance);
    }

    #[test]
    fn interpolate_non_uniform_4x5_reproduces_data() {
        let grid = warped_grid(&[0.0, 0.4, 1.7, 2.0], &[0.0, 0.3, 1.1, 2.6, 3.0]);
        for (du, dv) in [(3, 3), (2, 3), (3, 2)] {
            let surface = interpolate_surface(&grid, du, dv).unwrap();
            let miss = max_data_miss(&grid, &surface);
            assert!(miss < 1e-9, "4x5 degree ({du},{dv}) data miss {miss:e}");
        }
    }

    #[test]
    fn interpolate_non_uniform_7x4_reproduces_data() {
        let grid = warped_grid(&[0.0, 0.1, 0.5, 1.4, 1.6, 2.9, 3.3], &[0.0, 1.2, 1.5, 4.0]);
        for (du, dv) in [(3, 3), (5, 2), (1, 1)] {
            let surface = interpolate_surface(&grid, du, dv).unwrap();
            let miss = max_data_miss(&grid, &surface);
            assert!(miss < 1e-9, "7x4 degree ({du},{dv}) data miss {miss:e}");
        }
    }

    /// Review finding MATH-10: degrees clamp to `n - 1` per direction like
    /// curve `interpolate` instead of failing in `NurbsSurface::new`.
    #[test]
    fn interpolate_surface_clamps_degree_per_direction() {
        let grid = warped_grid(&[0.0, 0.7, 2.0], &[0.0, 0.5, 2.0, 2.4, 3.1]);
        let surface = interpolate_surface(&grid, 5, 7).unwrap();
        assert_eq!(surface.degree_u(), 2);
        assert_eq!(surface.degree_v(), 4);
        let miss = max_data_miss(&grid, &surface);
        assert!(miss < 1e-9, "clamped data miss {miss:e}");

        let two_rows = warped_grid(&[0.0, 1.3], &[0.0, 0.5, 2.0, 2.4]);
        let surface = interpolate_surface(&two_rows, 3, 3).unwrap();
        assert_eq!(surface.degree_u(), 1);
        assert_eq!(surface.degree_v(), 3);
        let miss = max_data_miss(&two_rows, &surface);
        assert!(miss < 1e-9, "two-row data miss {miss:e}");

        assert!(matches!(
            interpolate_surface(&grid, 0, 2),
            Err(MathError::InvalidDegree { degree: 0, .. })
        ));
    }

    /// A zero-length row (a cone apex) carries no spacing and is left out of
    /// the averaged v parameters; the surface still hits every data point.
    #[test]
    fn interpolate_surface_with_pole_row_reproduces_data() {
        let apex = Point3::new(0.0, 0.0, 2.0);
        let mut grid = vec![vec![apex; 5]];
        for (r, z) in [(0.4, 1.5), (1.1, 0.6), (1.5, 0.0)] {
            grid.push(
                [0.0f64, 0.3, 1.2, 2.0, 2.2]
                    .iter()
                    .map(|&t| Point3::new(r * t.cos(), r * t.sin(), z))
                    .collect(),
            );
        }
        let surface = interpolate_surface(&grid, 3, 3).unwrap();
        let miss = max_data_miss(&grid, &surface);
        assert!(miss < 1e-9, "pole grid data miss {miss:e}");
    }

    /// When every row has the same chord spacing, the shared v parameters
    /// (and so the knots) are that row's own, bit for bit.
    #[test]
    fn interpolate_surface_regular_rows_keep_curve_knots() {
        let xs = [0.0, 0.3, 1.1, 2.6, 3.0];
        let grid: Vec<Vec<Point3>> = [0.0, 0.7, 1.9, 2.0]
            .iter()
            .map(|&y| xs.iter().map(|&x| Point3::new(x, y, x * x)).collect())
            .collect();
        let surface = interpolate_surface(&grid, 3, 3).unwrap();
        let curve = fitting::interpolate(&grid[0], 3).unwrap();
        let bits = |k: &[f64]| k.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(surface.knots_v()), bits(curve.knots()));
    }

    // ── LSPIA surface tests ────────────────────────────────────────────

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn lspia_surface_fits_plane() {
        let grid: Vec<Vec<Point3>> = (0..10)
            .map(|i| {
                (0..10)
                    .map(|j| {
                        let u = i as f64 / 9.0;
                        let v = j as f64 / 9.0;
                        Point3::new(u, v, 0.5)
                    })
                    .collect()
            })
            .collect();
        let surface = approximate_surface_lspia(&grid, 3, 3, 6, 6, 1e-6, 50)
            .unwrap()
            .surface;
        let p = surface.evaluate(0.5, 0.5);
        assert!(
            (p.z() - 0.5).abs() < 0.01,
            "plane z at center: expected ~0.5, got {}",
            p.z()
        );
    }

    /// Averaged chord-length parameters, computed independently of the
    /// kernel: per-line cumulative chord lengths, normalized, averaged.
    fn oracle_lspia_params(lines: &[Vec<Point3>]) -> Vec<f64> {
        let len = lines[0].len();
        let mut sum = vec![0.0; len];
        for line in lines {
            let mut acc = vec![0.0; len];
            for k in 1..len {
                acc[k] = acc[k - 1] + (line[k] - line[k - 1]).length();
            }
            for k in 0..len {
                sum[k] += acc[k] / acc[len - 1];
            }
        }
        sum.iter().map(|s| s / lines.len() as f64).collect()
    }

    /// Largest data-point deviation of `surface` at the averaged parameters.
    fn lspia_max_deviation(grid: &[Vec<Point3>], surface: &NurbsSurface) -> f64 {
        let cols: Vec<Vec<Point3>> = (0..grid[0].len())
            .map(|j| grid.iter().map(|row| row[j]).collect())
            .collect();
        let params_u = oracle_lspia_params(&cols);
        let params_v = oracle_lspia_params(grid);
        let mut worst = 0.0f64;
        for (i, row) in grid.iter().enumerate() {
            for (j, q) in row.iter().enumerate() {
                worst = worst.max((surface.evaluate(params_u[i], params_v[j]) - *q).length());
            }
        }
        worst
    }

    /// Review finding MATH-9 (surface twin): the loop returned the grid after
    /// the last update unmeasured. On a 7×7 flat grid with the centre raised
    /// to z = 1 (degree 1, 3×3 control points) the residual falls at every
    /// step but the maximum deviation goes 0.889, 0.593, 0.669, 0.717, … and
    /// settles at 0.825, so a larger budget returned a worse surface.
    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn lspia_surface_deviation_never_grows_with_budget() {
        let grid: Vec<Vec<Point3>> = (0..7)
            .map(|i| {
                (0..7)
                    .map(|j| {
                        let z = if (i, j) == (3, 3) { 1.0 } else { 0.0 };
                        Point3::new(i as f64, j as f64, z)
                    })
                    .collect()
            })
            .collect();
        let mut previous = f64::INFINITY;
        for budget in 0..=30 {
            let fit = approximate_surface_lspia(&grid, 1, 1, 3, 3, 1e-6, budget).unwrap();
            let deviation = lspia_max_deviation(&grid, &fit.surface);
            assert!(!fit.converged, "budget {budget}: {fit:?}");
            assert!(fit.iteration <= budget, "budget {budget}: {fit:?}");
            assert!(
                (deviation - fit.max_error).abs() < 1e-12,
                "budget {budget}: reported {} vs re-measured {deviation}",
                fit.max_error
            );
            assert!(
                deviation <= previous + 1e-12,
                "budget {budget}: deviation {deviation} grew from {previous}"
            );
            previous = deviation;
        }
        // Iterate 1 is the best of the whole run.
        let fit = approximate_surface_lspia(&grid, 1, 1, 3, 3, 1e-6, 30).unwrap();
        assert_eq!(fit.iteration, 1);
        assert!((fit.max_error - 0.592_642).abs() < 1e-6, "{fit:?}");
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn lspia_surface_reports_convergence() {
        // 6×6 flat grid, (1, 1) raised, interpolating degree-1 fit: the
        // deviation falls 0.943, 0.458, 0.291, 0.213, 0.174, … and crosses
        // 0.2 at iterate 4.
        let grid: Vec<Vec<Point3>> = (0..6)
            .map(|i| {
                (0..6)
                    .map(|j| {
                        let z = if (i, j) == (1, 1) { 1.0 } else { 0.0 };
                        Point3::new(i as f64, j as f64, z)
                    })
                    .collect()
            })
            .collect();
        let fit = approximate_surface_lspia(&grid, 1, 1, 6, 6, 0.2, 50).unwrap();
        assert!(fit.converged && fit.iteration > 0, "{fit:?}");
        assert!(fit.max_error < 0.2, "{fit:?}");
        assert!(lspia_max_deviation(&grid, &fit.surface) < 0.2);

        // Too small a budget: the best iterate, flagged as not converged.
        let fit = approximate_surface_lspia(&grid, 1, 1, 6, 6, 0.2, 1).unwrap();
        assert!(!fit.converged && fit.iteration == 1, "{fit:?}");

        // A zero budget measures the initial grid instead of skipping it.
        let fit = approximate_surface_lspia(&grid, 1, 1, 6, 6, 0.2, 0).unwrap();
        assert!(!fit.converged && fit.iteration == 0, "{fit:?}");
        assert!(
            (lspia_max_deviation(&grid, &fit.surface) - fit.max_error).abs() < 1e-12,
            "{fit:?}"
        );
    }

    /// The step was 1 / (product of the largest diagonal entries of N^T N),
    /// a lower bound on lambda_max, so the iteration diverged on ordinary
    /// grids: this plane's x/y deviation went 0.19 → 120 (10 iterations) →
    /// 3.5e13 (50), overflowing near iterate 1,050. `lspia_surface_fits_plane`
    /// missed it because z never moves on a z = const plane.
    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn lspia_surface_plane_converges_in_every_coordinate() {
        let grid: Vec<Vec<Point3>> = (0..10)
            .map(|i| {
                (0..10)
                    .map(|j| Point3::new(i as f64 / 9.0, j as f64 / 9.0, 0.5))
                    .collect()
            })
            .collect();
        let mut previous = f64::INFINITY;
        for budget in [0, 1, 2, 5, 10, 20, 50, 100] {
            let fit = approximate_surface_lspia(&grid, 3, 3, 6, 6, 0.0, budget).unwrap();
            assert_eq!(
                fit.iteration, budget,
                "a contracting run ends on its last iterate"
            );
            let deviation = lspia_max_deviation(&grid, &fit.surface);
            assert!(
                deviation < previous,
                "budget {budget}: deviation {deviation} did not fall below {previous}"
            );
            previous = deviation;
        }
        assert!(
            previous < 5e-4,
            "deviation after 100 iterations: {previous}"
        );

        let fit = approximate_surface_lspia(&grid, 3, 3, 6, 6, 1e-3, 100).unwrap();
        assert!(fit.converged, "{fit:?}");
        assert!(lspia_max_deviation(&grid, &fit.surface) < 1e-3);
    }

    /// Largest eigenvalue of the dense `N^T N` by power iteration, built
    /// directly from the basis rows (independent of `col_sum_lambda`).
    fn dense_lambda_max(basis: &[(usize, Vec<f64>)], p: usize, m: usize) -> f64 {
        let mut n = vec![vec![0.0; m]; basis.len()];
        for (row, (span, vals)) in n.iter_mut().zip(basis) {
            for (k, &v) in vals.iter().enumerate() {
                row[span - p + k] = v;
            }
        }
        let mut x = vec![1.0; m];
        let mut lambda = 0.0;
        for _ in 0..500 {
            let nx: Vec<f64> = n
                .iter()
                .map(|r| r.iter().zip(&x).map(|(a, b)| a * b).sum())
                .collect();
            let mut y = vec![0.0; m];
            for (r, &v) in n.iter().zip(&nx) {
                for (yj, &a) in y.iter_mut().zip(r) {
                    *yj += a * v;
                }
            }
            let norm = y.iter().map(|v| v * v).sum::<f64>().sqrt();
            lambda = norm / x.iter().map(|v| v * v).sum::<f64>().sqrt();
            x = y.iter().map(|v| v / norm).collect();
        }
        lambda
    }

    /// The step bound must not fall below lambda_max of `N^T N` in either
    /// direction; the old squared-sum estimate did.
    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn lspia_step_bound_dominates_lambda_max() {
        let mut old_underestimated = false;
        for (len, degree, m) in [
            (10, 3, 6),
            (8, 3, 6),
            (7, 1, 3),
            (7, 2, 5),
            (20, 3, 20),
            (6, 1, 6),
        ] {
            let pts: Vec<Point3> = (0..len)
                .map(|i| {
                    let t = i as f64 / (len - 1) as f64;
                    Point3::new(t + 0.3 * t * t, 0.0, 0.0)
                })
                .collect();
            let params = fitting::chord_length_params(&pts);
            let p = degree.min(len - 1);
            let knots = fitting::build_approximation_knots(&params, p, m, len);
            let basis: Vec<(usize, Vec<f64>)> = params
                .iter()
                .map(|&u| {
                    let span = fitting::find_span(u, p, &knots, m);
                    (span, basis_funs(span, u, p, &knots))
                })
                .collect();
            let lambda = dense_lambda_max(&basis, p, m);
            let bound = col_sum_lambda(&basis, p, m);
            assert!(
                bound >= lambda * (1.0 - 1e-12),
                "({len}, {degree}, {m}): bound {bound} < lambda_max {lambda}"
            );
            let mut diag = vec![0.0f64; m];
            for (span, vals) in &basis {
                for (k, &v) in vals.iter().enumerate() {
                    diag[span - p + k] += v * v;
                }
            }
            old_underestimated |= diag.iter().copied().fold(0.0, f64::max) < lambda;
        }
        assert!(
            old_underestimated,
            "no case shows the old estimate below lambda_max"
        );
    }

    #[test]
    fn lspia_surface_empty_returns_error() {
        let grid: Vec<Vec<Point3>> = Vec::new();
        assert!(approximate_surface_lspia(&grid, 3, 3, 4, 4, 1e-6, 50).is_err());
    }

    #[test]
    fn lspia_surface_inconsistent_rows_returns_error() {
        let grid = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![Point3::new(0.0, 1.0, 0.0)],
        ];
        assert!(approximate_surface_lspia(&grid, 1, 1, 2, 2, 1e-6, 50).is_err());
    }

    #[test]
    #[allow(clippy::cast_precision_loss)]
    fn lspia_surface_fits_paraboloid() {
        let grid: Vec<Vec<Point3>> = (0..8)
            .map(|i| {
                (0..8)
                    .map(|j| {
                        let u = i as f64 / 7.0;
                        let v = j as f64 / 7.0;
                        Point3::new(u, v, u * u + v * v)
                    })
                    .collect()
            })
            .collect();
        let surface = approximate_surface_lspia(&grid, 3, 3, 6, 6, 1e-4, 100)
            .unwrap()
            .surface;
        // Parameter (0.5, 0.5) is not position (0.5, 0.5) under chord-length
        // parameters, so check the surface against z = x^2 + y^2 where it is.
        for i in 0..=4 {
            for j in 0..=4 {
                let p = surface.evaluate(i as f64 / 4.0, j as f64 / 4.0);
                let miss = p.z() - (p.x() * p.x() + p.y() * p.y());
                assert!(
                    miss.abs() < 0.02,
                    "({i}, {j}): {p:?} misses z = x² + y² by {miss}"
                );
            }
        }
    }
}
