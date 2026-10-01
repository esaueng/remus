//! DogLeg trust-region solver.
//!
//! Blends Gauss-Newton and steepest-descent steps inside a trust region
//! for globally convergent nonlinear least-squares solving. This is the
//! default solver in FreeCAD's PlaneGCS for the same reason: it converges
//! reliably even when the initial guess is far from the solution.

use super::qr::QrResult;

/// Solve-local reusable scratch for the DogLeg loop (PERF-S03).
///
/// All buffers are sized once per `solve` call (`m` residuals, `n` params)
/// and reused across iterations. Nothing is retained across solves, so a
/// later solve with different dimensions resizes instead of reusing stale
/// extents. Every buffer is fully overwritten before it is read in each
/// iteration; no iteration consumes a previous iteration's leftovers.
#[derive(Debug, Default)]
pub struct DoglegWorkspace {
    /// Saved pre-factorization Jacobian (`m*n`, row-major).
    jac_orig: Vec<f64>,
    /// Working copy factorized in place (`m*n`).
    jac_work: Vec<f64>,
    /// Current residuals (`m`).
    r: Vec<f64>,
    /// Trial residuals (`m`).
    r_trial: Vec<f64>,
    /// Negated residuals (`m`).
    neg_r: Vec<f64>,
    /// Gradient `J^T r` (`n`).
    g: Vec<f64>,
    /// `J*g` (`m`).
    jg: Vec<f64>,
    /// `J*h` (`m`).
    jh: Vec<f64>,
    /// Gauss-Newton step (`n`).
    h_gn: Vec<f64>,
    /// Steepest-descent step (`n`).
    h_sd: Vec<f64>,
    /// Selected DogLeg step (`n`).
    h: Vec<f64>,
    /// Trial params (`n`).
    trial: Vec<f64>,
    /// `h_gn - h_sd` scratch (`n`).
    diff: Vec<f64>,
    /// Householder scales (`k = min(m,n)`).
    tau: Vec<f64>,
    /// Column permutation (`n`).
    perm: Vec<usize>,
    /// Column norms for pivoting (`n`).
    col_norms: Vec<f64>,
    /// `Q^T b` scratch (`m`).
    qtb: Vec<f64>,
    /// Back-substitution scratch (`n`).
    z: Vec<f64>,
}

impl DoglegWorkspace {
    /// Create empty scratch; buffers are sized on first use.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Residual vector at the solver's final iterate.
    ///
    /// Valid only after [`solve_dogleg_fill`] returns: every exit path leaves
    /// the residuals evaluated at the returned `params` in `r` (the
    /// small-step exit copies its trial evaluation across). Callers must not
    /// read this before a solve or after reusing the workspace.
    #[must_use]
    pub fn final_residuals(&self) -> &[f64] {
        &self.r
    }

    /// Size all buffers for `m` residuals and `n` params.
    ///
    /// Existing capacity is retained; contents are not meaningful until
    /// each buffer is overwritten in the loop.
    pub fn ensure(&mut self, m: usize, n: usize) {
        let mn = m.saturating_mul(n);
        let k = m.min(n);
        self.jac_orig.resize(mn, 0.0);
        self.jac_work.resize(mn, 0.0);
        self.r.resize(m, 0.0);
        self.r_trial.resize(m, 0.0);
        self.neg_r.resize(m, 0.0);
        self.g.resize(n, 0.0);
        self.jg.resize(m, 0.0);
        self.jh.resize(m, 0.0);
        self.h_gn.resize(n, 0.0);
        self.h_sd.resize(n, 0.0);
        self.h.resize(n, 0.0);
        self.trial.resize(n, 0.0);
        self.diff.resize(n, 0.0);
        self.tau.resize(k, 0.0);
        self.perm.resize(n, 0);
        self.col_norms.resize(n, 0.0);
        self.qtb.resize(m, 0.0);
        self.z.resize(n, 0.0);
    }
}

/// Largest absolute residual, propagating NaN.
///
/// `f64::max` returns the non-NaN operand, so a max-fold over residuals
/// silently drops a poisoned (NaN) equation and reports the system clean —
/// the solver would exit `converged: true` with `max_residual: 0.0` on input
/// that never evaluated to a number. NaN short-circuits instead, so a
/// poisoned residual can only ever fail the `< tol` convergence test.
///
/// Shared with the PERF-S04 sparse loop, which must use the identical
/// convergence norm; any change here must be mirrored there.
pub fn max_abs_residual(values: &[f64]) -> f64 {
    let mut max = 0.0_f64;
    for &v in values {
        let a = v.abs();
        if a.is_nan() {
            return f64::NAN;
        }
        if a > max {
            max = a;
        }
    }
    max
}

/// Result of a solve attempt.
#[derive(Debug, Clone)]
pub struct SolveResult {
    /// Whether the solver converged within tolerance.
    pub converged: bool,
    /// Number of iterations used.
    pub iterations: usize,
    /// Maximum absolute residual after solving.
    pub max_residual: f64,
}

/// Evaluation counts for one [`solve_dogleg_fill`] call (PERF-S06).
///
/// Every residual, Jacobian and QR-factorization evaluation the solver
/// performs is counted exactly once, at the call site. Diagnostics sharing
/// (PERF-S06) reports these alongside its own follow-up evaluations so a
/// measurement can tell solver-loop work apart from post-solve analysis
/// work. Counts carry no numerical meaning; they only tally calls.
#[derive(Debug, Default, Clone, Copy)]
pub struct SolveStats {
    /// Calls to `residual_fill` (each evaluates all `m` residuals).
    pub residual_evals: usize,
    /// Calls to `jacobian_fill` (each evaluates the full `m × n` Jacobian).
    pub jacobian_evals: usize,
    /// Householder QR factorizations of the working Jacobian.
    pub qr_factorizations: usize,
}

/// DogLeg trust-region solver for the system `r(params) = 0`.
///
/// - `params`: initial parameter values (modified in-place on success)
/// - `residual_fn`: compute residuals given current params
/// - `jacobian_fn`: compute row-major Jacobian given current params
/// - `num_residuals`: number of residual equations
/// - `max_iter`: maximum iterations
/// - `tol`: convergence tolerance on max |residual|
///
/// Returns the solve result. On convergence, `params` holds the solution.
///
/// Solve-local workspaces back the iteration loop (PERF-S03); the
/// `Vec`-returning closures still allocate their temporaries, while all
/// solver-owned gradient, product and step vectors plus the QR copy are
/// reused. Callers that can fill caller-owned buffers should prefer
/// [`solve_dogleg_fill`], which avoids those temporaries as well.
///
/// Retained for the solver unit tests; production `GcsSystem::solve` uses
/// [`solve_dogleg_fill`] with reusable snapshot storage.
#[allow(dead_code)]
pub fn solve_dogleg<F, G>(
    params: &mut [f64],
    residual_fn: &F,
    jacobian_fn: &G,
    num_residuals: usize,
    max_iter: usize,
    tol: f64,
) -> SolveResult
where
    F: Fn(&[f64]) -> Vec<f64>,
    G: Fn(&[f64]) -> Vec<f64>,
{
    let mut ws = DoglegWorkspace::new();
    let mut residual_fill = |p: &[f64], out: &mut Vec<f64>| {
        out.clear();
        out.extend(residual_fn(p));
    };
    let mut jacobian_fill = |p: &[f64], out: &mut [f64]| {
        let j = jacobian_fn(p);
        out.copy_from_slice(&j);
    };
    let mut stats = SolveStats::default();
    solve_dogleg_fill(
        params,
        &mut residual_fill,
        &mut jacobian_fill,
        num_residuals,
        max_iter,
        tol,
        &mut ws,
        &mut stats,
    )
}

/// DogLeg trust-region solver writing into caller-owned buffers (PERF-S03).
///
/// - `residual_fill(p, out)`: overwrites `out` with exactly `num_residuals`
///   residuals at `p` (may reuse `out`'s capacity via `clear` + `push`).
/// - `jacobian_fill(p, out)`: overwrites `out[m*n]` with the row-major
///   Jacobian at `p` (must fully overwrite; zero before accumulating).
/// - `workspace`: solve-local scratch, sized via [`DoglegWorkspace::ensure`].
/// - `stats`: tallies every residual, Jacobian and QR evaluation (PERF-S06).
///
/// Matrix layout, numerical operation order, convergence thresholds,
/// trust-region decisions and iteration accounting match [`solve_dogleg`]
/// exactly; only storage is reused.
///
/// On return, [`DoglegWorkspace::final_residuals`] holds the residual vector
/// evaluated at the returned `params`, on every exit path: the loop-top,
/// stationary-point, budget-exhausted and empty-system paths evaluate there
/// directly, and the small-step path copies its trial evaluation across so
/// no caller reads a stale iterate.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub fn solve_dogleg_fill<F, G>(
    params: &mut [f64],
    residual_fill: &mut F,
    jacobian_fill: &mut G,
    num_residuals: usize,
    max_iter: usize,
    tol: f64,
    workspace: &mut DoglegWorkspace,
    stats: &mut SolveStats,
) -> SolveResult
where
    F: FnMut(&[f64], &mut Vec<f64>),
    G: FnMut(&[f64], &mut [f64]),
{
    let n = params.len();
    let m = num_residuals;
    if n == 0 || m == 0 {
        workspace.ensure(m, n);
        workspace.r.clear();
        residual_fill(params, &mut workspace.r);
        stats.residual_evals += 1;
        let max_r = max_abs_residual(&workspace.r);
        return SolveResult {
            converged: max_r < tol,
            iterations: 0,
            max_residual: max_r,
        };
    }

    workspace.ensure(m, n);
    let ws = workspace;

    // Scale-aware initial trust radius
    let param_norm: f64 = params.iter().map(|x| x * x).sum::<f64>().sqrt();
    let delta_max = 1e4;
    let delta_min = 1e-15;
    let mut delta = (1.0_f64).max(0.1 * param_norm).min(delta_max);

    for iteration in 0..max_iter {
        ws.r.clear();
        residual_fill(params, &mut ws.r);
        stats.residual_evals += 1;
        let max_r = max_abs_residual(&ws.r);

        if max_r < tol {
            return SolveResult {
                converged: true,
                iterations: iteration,
                max_residual: max_r,
            };
        }

        // Build Jacobian. Keep the pre-factorization copy (`jac_orig`) for
        // the gradient and predicted-reduction computations; factorize the
        // working copy in place with reused QR scratch (no owned copy).
        jacobian_fill(params, &mut ws.jac_orig);
        stats.jacobian_evals += 1;
        ws.jac_work.copy_from_slice(&ws.jac_orig);
        QrResult::factorize_reuse(
            &mut ws.jac_work,
            m,
            n,
            &mut ws.tau,
            &mut ws.perm,
            &mut ws.col_norms,
        );
        stats.qr_factorizations += 1;

        // Gauss-Newton step: solve J * h_gn = -r
        for (d, &v) in ws.neg_r.iter_mut().zip(ws.r.iter()) {
            *d = -v;
        }
        solve_gn_step(ws, m, n);

        // Gradient: g = J^T * r  (use the saved pre-factorization Jacobian)
        ws.g.fill(0.0);
        for i in 0..m {
            for j in 0..n {
                ws.g[j] += ws.jac_orig[i * n + j] * ws.r[i];
            }
        }

        // Steepest descent step: h_sd = -alpha * g
        // alpha = ||g||² / ||J*g||²
        let g_norm_sq: f64 = ws.g.iter().map(|&v| v * v).sum();
        if g_norm_sq < 1e-300 {
            // Zero gradient — we're at a stationary point
            return SolveResult {
                converged: max_r < tol,
                iterations: iteration,
                max_residual: max_r,
            };
        }

        ws.jg.fill(0.0);
        for i in 0..m {
            for j in 0..n {
                ws.jg[i] += ws.jac_orig[i * n + j] * ws.g[j];
            }
        }
        let jg_norm_sq: f64 = ws.jg.iter().map(|&v| v * v).sum();
        let alpha = if jg_norm_sq > 1e-300 {
            g_norm_sq / jg_norm_sq
        } else {
            1.0
        };

        for (d, &v) in ws.h_sd.iter_mut().zip(ws.g.iter()) {
            *d = -alpha * v;
        }

        dogleg_step_into(&ws.h_gn, &ws.h_sd, delta, &mut ws.h, &mut ws.diff);
        let h_norm = ws.h.iter().map(|&v| v * v).sum::<f64>().sqrt();

        for (t, (&p, &d)) in ws.trial.iter_mut().zip(params.iter().zip(ws.h.iter())) {
            *t = p + d;
        }
        ws.r_trial.clear();
        residual_fill(&ws.trial, &mut ws.r_trial);
        stats.residual_evals += 1;

        let cost_current: f64 = ws.r.iter().map(|&v| v * v).sum::<f64>() * 0.5;
        let cost_trial: f64 = ws.r_trial.iter().map(|&v| v * v).sum::<f64>() * 0.5;
        let actual_reduction = cost_current - cost_trial;

        // Predicted reduction from linear model
        ws.jh.fill(0.0);
        for i in 0..m {
            for j in 0..n {
                ws.jh[i] += ws.jac_orig[i * n + j] * ws.h[j];
            }
        }
        let predicted: f64 = {
            let mut pred = 0.0;
            for i in 0..m {
                pred += ws.r[i] * ws.jh[i];
                pred += 0.5 * ws.jh[i] * ws.jh[i];
            }
            -pred
        };

        let rho = if predicted.abs() < 1e-300 {
            if actual_reduction > 0.0 { 1.0 } else { 0.0 }
        } else {
            actual_reduction / predicted
        };

        // Update trust region
        if rho > 0.75 {
            delta = (2.0 * delta).min(delta_max);
        } else if rho < 0.25 {
            delta = (delta / 4.0).max(delta_min);
        }

        if rho > 0.0 {
            params.copy_from_slice(&ws.trial);
        }

        if h_norm < 1e-15 * (1.0 + param_norm) {
            ws.r_trial.clear();
            residual_fill(params, &mut ws.r_trial);
            stats.residual_evals += 1;
            let final_max = max_abs_residual(&ws.r_trial);
            // Canonical final state: later readers (PERF-S06) must see the
            // residuals at the returned params, not the loop-top iterate.
            ws.r.clear();
            ws.r.extend_from_slice(&ws.r_trial);
            return SolveResult {
                converged: final_max < tol,
                iterations: iteration + 1,
                max_residual: final_max,
            };
        }
    }

    ws.r.clear();
    residual_fill(params, &mut ws.r);
    stats.residual_evals += 1;
    let max_r = max_abs_residual(&ws.r);
    SolveResult {
        converged: max_r < tol,
        iterations: max_iter,
        max_residual: max_r,
    }
}

/// Gauss-Newton solve `J h_gn = -r` using the factored `jac_work` in `ws`.
///
/// Split out so the borrow checker sees disjoint field accesses without
/// whole-struct borrows or per-iteration allocations.
fn solve_gn_step(ws: &mut DoglegWorkspace, m: usize, n: usize) {
    let DoglegWorkspace {
        jac_work,
        tau,
        perm,
        neg_r,
        h_gn,
        qtb,
        z,
        ..
    } = ws;
    QrResult::solve_least_squares_into(jac_work, tau, perm, m, n, neg_r, h_gn, qtb, z);
}

/// Compute the DogLeg step into `out` (PERF-S03).
///
/// Same layout and operation order as [`dogleg_step`]; `diff_tmp` backs the
/// `h_gn - h_sd` interpolation scratch. `out` and `diff_tmp` are fully
/// overwritten on the paths that read them, so no stale step survives a
/// resize.
///
/// Shared with the PERF-S04 sparse loop, which must use the identical
/// trust-region interpolation; any change here must be mirrored there.
pub fn dogleg_step_into(
    h_gn: &[f64],
    h_sd: &[f64],
    delta: f64,
    out: &mut [f64],
    diff_tmp: &mut [f64],
) {
    let gn_norm = h_gn.iter().map(|&v| v * v).sum::<f64>().sqrt();

    // If GN step is within trust region, use it
    if gn_norm <= delta {
        out.copy_from_slice(h_gn);
        return;
    }

    let sd_norm = h_sd.iter().map(|&v| v * v).sum::<f64>().sqrt();

    // If even SD step exceeds trust region, scale it down
    if sd_norm >= delta {
        let scale = delta / sd_norm;
        for (d, &v) in out.iter_mut().zip(h_sd.iter()) {
            *d = v * scale;
        }
        return;
    }

    // Interpolate between SD and GN: h = h_sd + t * (h_gn - h_sd)
    // Find t such that ||h_sd + t * (h_gn - h_sd)|| = delta
    for (d, (&g, &s)) in diff_tmp.iter_mut().zip(h_gn.iter().zip(h_sd.iter())) {
        *d = g - s;
    }

    let a: f64 = diff_tmp.iter().map(|&v| v * v).sum();
    let b: f64 = h_sd
        .iter()
        .zip(diff_tmp.iter())
        .map(|(&s, &d)| s * d)
        .sum::<f64>()
        * 2.0;
    let c: f64 = sd_norm * sd_norm - delta * delta;

    // Solve a*t² + b*t + c = 0 for the positive root
    let discriminant = (b * b - 4.0 * a * c).max(0.0);
    let t = (-b + discriminant.sqrt()) / (2.0 * a);
    let t = t.clamp(0.0, 1.0);

    for (o, (&s, &d)) in out.iter_mut().zip(h_sd.iter().zip(diff_tmp.iter())) {
        *o = s + t * d;
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn fix_x_converges_in_one_iter() {
        // Single param, single constraint: x = 5.0, starting at x = 3.0
        let mut params = vec![3.0];
        let result = solve_dogleg(
            &mut params,
            &|p: &[f64]| vec![p[0] - 5.0],
            &|_p: &[f64]| vec![1.0],
            1,
            100,
            1e-12,
        );
        assert!(result.converged);
        assert!(result.iterations <= 2, "iters = {}", result.iterations);
        assert!((params[0] - 5.0).abs() < 1e-12);
    }

    #[test]
    fn quadratic_residual() {
        // x² - 4 = 0 → x = 2
        let mut params = vec![3.0];
        let result = solve_dogleg(
            &mut params,
            &|p: &[f64]| vec![p[0] * p[0] - 4.0],
            &|p: &[f64]| vec![2.0 * p[0]],
            1,
            100,
            1e-12,
        );
        assert!(result.converged);
        assert!((params[0] - 2.0).abs() < 1e-10, "x = {}", params[0]);
    }

    #[test]
    fn two_variable_system() {
        // x + y = 3, x - y = 1 → x = 2, y = 1
        let mut params = vec![0.0, 0.0];
        let result = solve_dogleg(
            &mut params,
            &|p: &[f64]| vec![p[0] + p[1] - 3.0, p[0] - p[1] - 1.0],
            &|_p: &[f64]| vec![1.0, 1.0, 1.0, -1.0],
            2,
            100,
            1e-12,
        );
        assert!(result.converged);
        assert!((params[0] - 2.0).abs() < 1e-10);
        assert!((params[1] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn empty_system() {
        let mut params = vec![];
        let result = solve_dogleg(
            &mut params,
            &|_p: &[f64]| vec![],
            &|_p: &[f64]| vec![],
            0,
            100,
            1e-12,
        );
        assert!(result.converged);
    }

    #[test]
    fn trust_region_shrinks_on_bad_step() {
        // A system where the Gauss-Newton step overshoots.
        // f(x) = x^3 - 8, starting far from solution
        let mut params = vec![10.0];
        let result = solve_dogleg(
            &mut params,
            &|p: &[f64]| vec![p[0] * p[0] * p[0] - 8.0],
            &|p: &[f64]| vec![3.0 * p[0] * p[0]],
            1,
            200,
            1e-10,
        );
        assert!(result.converged, "max_r = {}", result.max_residual);
        assert!((params[0] - 2.0).abs() < 1e-8, "x = {}", params[0]);
    }
}
