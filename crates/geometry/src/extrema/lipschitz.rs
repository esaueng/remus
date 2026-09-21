//! Lipschitz global optimizer for 2D parameter-space minimization.
//!
//! Uses interval subdivision with Lipschitz pruning: cells whose lower
//! bound `f(center) - L * radius` exceeds the current best are discarded.
//! This guarantees finding the global minimum, unlike Newton-based methods
//! that can get trapped in local minima.

use remus_math::nurbs::curve::NurbsCurve;
use remus_math::vec::Point3;

// ── LipschitzOptimizer ───────────────────────────────────────────────────────

/// Lipschitz optimizer for 2D parameter-space minimization.
///
/// Finds the global minimum of a scalar function `f(u, v)` over a rectangular
/// domain `[u0, u1] × [v0, v1]`, given a Lipschitz bound `L` such that
/// `|f(a) - f(b)| ≤ L * |a - b|` for all `a`, `b` in the domain.
///
/// The algorithm has three phases:
/// 1. **Grid search** — uniform 16×16 sampling to establish an upper bound.
/// 2. **Coordinate descent** — refine from the best grid point.
/// 3. **Lipschitz subdivision** — depth-first cell subdivision, pruning cells
///    whose Lipschitz lower bound exceeds the current best.
pub(crate) struct LipschitzOptimizer {
    grid_size: usize,
    max_subdivisions: usize,
    max_evals: usize,
}

impl Default for LipschitzOptimizer {
    fn default() -> Self {
        Self::new()
    }
}

impl LipschitzOptimizer {
    /// Create a new optimizer with default parameters.
    ///
    /// Defaults: grid 16×16, max 50 subdivisions per dimension, 500 000 cell budget.
    #[must_use]
    pub fn new() -> Self {
        Self {
            grid_size: 16,
            max_subdivisions: 50,
            max_evals: 500_000,
        }
    }

    /// Find the (approximate) global minimum of `f(u, v)` over the given domain.
    ///
    /// The Lipschitz bound is estimated internally from a finite-difference
    /// grid. `tolerance` controls cell-radius convergence: a cell is terminal
    /// when its radius is below `tolerance`.
    ///
    /// Returns `(u*, v*, f*)` — the minimizer and minimum value found.
    #[must_use]
    #[allow(clippy::too_many_lines, clippy::cast_precision_loss)]
    pub fn minimize_2d<F>(
        &self,
        f: F,
        u_range: (f64, f64),
        v_range: (f64, f64),
        tolerance: f64,
    ) -> (f64, f64, f64)
    where
        F: Fn(f64, f64) -> f64,
    {
        let (u0, u1) = u_range;
        let (v0, v1) = v_range;
        let n = self.grid_size;

        // ── Phase 1: grid search (cached for Lipschitz estimation) ────────────
        let grid_len = (n + 1) * (n + 1);
        let mut grid_vals = vec![0.0_f64; grid_len];
        let mut best = f64::INFINITY;
        let mut best_u = (u0 + u1) * 0.5;
        let mut best_v = (v0 + v1) * 0.5;

        for iu in 0..=n {
            let u = u0 + (u1 - u0) * (iu as f64 / n as f64);
            for iv in 0..=n {
                let v = v0 + (v1 - v0) * (iv as f64 / n as f64);
                let val = f(u, v);
                grid_vals[iu * (n + 1) + iv] = val;
                if val < best {
                    best = val;
                    best_u = u;
                    best_v = v;
                }
            }
        }

        // ── Phase 1b: coordinate descent from best grid point ─────────────────
        {
            let step_u = (u1 - u0) / n as f64;
            let step_v = (v1 - v0) / n as f64;
            let mut lu = (best_u - step_u).max(u0);
            let mut hu = (best_u + step_u).min(u1);
            let mut lv = (best_v - step_v).max(v0);
            let mut hv = (best_v + step_v).min(v1);
            for _ in 0..self.max_subdivisions {
                let m1 = lu + (hu - lu) / 3.0;
                let m2 = hu - (hu - lu) / 3.0;
                if f(m1, best_v) < f(m2, best_v) {
                    hu = m2;
                } else {
                    lu = m1;
                }
                let cu = (lu + hu) * 0.5;
                let fcu = f(cu, best_v);
                if fcu < best {
                    best = fcu;
                    best_u = cu;
                }

                let m1v = lv + (hv - lv) / 3.0;
                let m2v = hv - (hv - lv) / 3.0;
                if f(best_u, m1v) < f(best_u, m2v) {
                    hv = m2v;
                } else {
                    lv = m1v;
                }
                let cv = (lv + hv) * 0.5;
                let fcv = f(best_u, cv);
                if fcv < best {
                    best = fcv;
                    best_v = cv;
                }
            }
        }

        // Estimate Lipschitz bound from cached Phase 1 grid values.
        let du = (u1 - u0) / n as f64;
        let dv = (v1 - v0) / n as f64;
        let mut lip: f64 = 0.0;
        for iu in 0..n {
            for iv in 0..n {
                let f00 = grid_vals[iu * (n + 1) + iv];
                let f10 = grid_vals[(iu + 1) * (n + 1) + iv];
                let f01 = grid_vals[iu * (n + 1) + (iv + 1)];
                let dfu = (f10 - f00).abs() / du;
                let dfv = (f01 - f00).abs() / dv;
                let local_lip = dfu.hypot(dfv);
                if local_lip > lip {
                    lip = local_lip;
                }
            }
        }
        // Add a safety margin.
        lip *= 2.0;
        if lip < 1e-15 {
            // Function appears constant.
            return (best_u, best_v, best);
        }

        // ── Phase 2: Lipschitz subdivision ────────────────────────────────────
        let mut stack: Vec<(f64, f64, f64, f64, usize)> = vec![(u0, u1, v0, v1, 0)];
        let mut cell_count = 0_usize;

        while let Some((cu0, cu1, cv0, cv1, depth)) = stack.pop() {
            cell_count += 1;
            if cell_count > self.max_evals {
                break;
            }

            let um = (cu0 + cu1) * 0.5;
            let vm = (cv0 + cv1) * 0.5;
            let fc = f(um, vm);
            if fc < best {
                best = fc;
                best_u = um;
                best_v = vm;
            }

            let du_cell = cu1 - cu0;
            let dv_cell = cv1 - cv0;
            let radius = (du_cell * du_cell + dv_cell * dv_cell).sqrt() * 0.5;
            let lower = fc - lip * radius;

            // Prune: this cell cannot contain a better minimum.
            if lower > best {
                continue;
            }

            // Terminal: cell converged or at max depth.
            if radius < tolerance || depth >= self.max_subdivisions {
                if fc < best {
                    best = fc;
                    best_u = um;
                    best_v = vm;
                }
                continue;
            }

            // Subdivide along the longer dimension.
            if du_cell >= dv_cell {
                stack.push((cu0, um, cv0, cv1, depth + 1));
                stack.push((um, cu1, cv0, cv1, depth + 1));
            } else {
                stack.push((cu0, cu1, cv0, vm, depth + 1));
                stack.push((cu0, cu1, vm, cv1, depth + 1));
            }
        }

        (best_u, best_v, best)
    }
}

// ── NURBS helpers ────────────────────────────────────────────────────────────

/// Estimate the Lipschitz bound for the squared-distance function between
/// two NURBS curves: `f(u, v) = ‖C₁(u) − C₂(v)‖²`.
///
/// The gradient is:
/// ```text
/// ∂f/∂u =  2 · (C₁(u) − C₂(v)) · C₁'(u)
/// ∂f/∂v = −2 · (C₁(u) − C₂(v)) · C₂'(v)
/// ```
///
/// Upper bound: `L ≤ 2 · max_sep · max(max_deriv₁, max_deriv₂)`.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn estimate_curve_curve_lipschitz(c1: &NurbsCurve, c2: &NurbsCurve) -> f64 {
    let n_samples = 20_usize;
    let (u0, u1) = c1.domain();
    let (v0, v1) = c2.domain();

    let mut max_deriv1: f64 = 0.0;
    let mut max_deriv2: f64 = 0.0;
    let mut max_sep: f64 = 0.0;

    for i in 0..=n_samples {
        let t = i as f64 / n_samples as f64;
        let u = u0 + (u1 - u0) * t;
        let d1 = c1.derivatives(u, 1);
        max_deriv1 = max_deriv1.max(d1[1].length());

        let v = v0 + (v1 - v0) * t;
        let d2 = c2.derivatives(v, 1);
        max_deriv2 = max_deriv2.max(d2[1].length());

        let p1 = Point3::new(d1[0].x(), d1[0].y(), d1[0].z());
        let p2 = Point3::new(d2[0].x(), d2[0].y(), d2[0].z());
        max_sep = max_sep.max((p1 - p2).length());
    }

    2.0 * max_sep * max_deriv1.max(max_deriv2)
}

/// Find the global minimum distance between two NURBS curves.
///
/// Returns `(distance, point on C₁, point on C₂)`.
#[must_use]
pub fn nurbs_curve_curve_distance(
    curve1: &NurbsCurve,
    curve2: &NurbsCurve,
) -> (f64, Point3, Point3) {
    let (u0, u1) = curve1.domain();
    let (v0, v1) = curve2.domain();

    let lip = estimate_curve_curve_lipschitz(curve1, curve2);

    if lip < 1e-15 {
        // Curves appear coincident or degenerate.
        let p1 = curve1.evaluate((u0 + u1) * 0.5);
        let p2 = curve2.evaluate((v0 + v1) * 0.5);
        return ((p1 - p2).length(), p1, p2);
    }

    let f = |u: f64, v: f64| -> f64 {
        let p1 = curve1.evaluate(u);
        let p2 = curve2.evaluate(v);
        (p1 - p2).length_squared()
    };

    let opt = LipschitzOptimizer::new();
    let (best_u, best_v, _) = opt.minimize_2d(f, (u0, u1), (v0, v1), 1e-4);

    let p1 = curve1.evaluate(best_u);
    let p2 = curve2.evaluate(best_v);
    ((p1 - p2).length(), p1, p2)
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn minimize_quadratic_at_origin() {
        // f(u,v) = u² + v², minimum at (0,0,0).
        let f = |u: f64, v: f64| u * u + v * v;
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(f, (-1.0, 1.0), (-1.0, 1.0), 1e-6);
        assert!(val < 1e-8, "val={val}");
        assert!(u.abs() < 1e-4, "u={u}");
        assert!(v.abs() < 1e-4, "v={v}");
    }

    #[test]
    fn minimize_quadratic_offset() {
        // f(u,v) = (u-0.3)² + (v-0.7)², minimum at (0.3, 0.7).
        let f = |u: f64, v: f64| (u - 0.3) * (u - 0.3) + (v - 0.7) * (v - 0.7);
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(f, (0.0, 1.0), (0.0, 1.0), 1e-6);
        assert!(val < 1e-8, "val={val}");
        assert!((u - 0.3).abs() < 1e-3, "u={u}");
        assert!((v - 0.7).abs() < 1e-3, "v={v}");
    }

    #[test]
    fn minimize_returns_global_not_local() {
        // Two basins: one at (0.1, 0.5) depth 0.01, one at (0.9, 0.5) depth 0 (global).
        let f = |u: f64, v: f64| {
            let d1 = (u - 0.1) * (u - 0.1) + (v - 0.5) * (v - 0.5) - 0.01;
            let d2 = (u - 0.9) * (u - 0.9) + (v - 0.5) * (v - 0.5);
            d1.min(d2)
        };
        let opt = LipschitzOptimizer::new();
        let (_u, _v, val) = opt.minimize_2d(f, (0.0, 1.0), (0.0, 1.0), 1e-5);
        // Global minimum is -0.01 at (0.1, 0.5).
        assert!(val < -0.005, "val={val}");
    }

    #[test]
    fn nurbs_parallel_lines_distance_one() {
        // Two parallel NURBS lines: y=0 and y=1 along x.
        let c1 = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        let c2 = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();

        let (dist, _, _) = nurbs_curve_curve_distance(&c1, &c2);
        assert!((dist - 1.0).abs() < 1e-3, "dist={dist}");
    }

    // ── Shared fixture domain ────────────────────────────────────────────────
    //
    // A deliberately asymmetric, off-origin, negative domain: `u0 + u1`,
    // `u1 - u0`, `u1 / u0` and `u0 * u1` are all distinct, and the v-extent is
    // the longer of the two so that the first subdivision splits v.
    const U_RANGE: (f64, f64) = (-5.0, -2.0);
    const V_RANGE: (f64, f64) = (-4.0, -0.5);

    /// Assert the documented structural contract of `minimize_2d`: the
    /// returned point lies inside the requested domain and the returned value
    /// really is `f` evaluated at that point.
    fn assert_result_contract<F>(f: &F, u: f64, v: f64, val: f64)
    where
        F: Fn(f64, f64) -> f64,
    {
        assert!(
            u >= U_RANGE.0 && u <= U_RANGE.1,
            "u={u} escaped {U_RANGE:?}"
        );
        assert!(
            v >= V_RANGE.0 && v <= V_RANGE.1,
            "v={v} escaped {V_RANGE:?}"
        );
        let at_point = f(u, v);
        assert!(
            (val - at_point).abs() < 1e-15 * at_point.abs().max(1.0),
            "reported {val} but f(u,v)={at_point}"
        );
    }

    /// Two-basin objective: a wide shallow bowl whose floor sits at `-0.5`,
    /// and a narrow well of depth `-1.2` that falls entirely between grid
    /// samples, so phase 1 cannot see it. Only a correct Lipschitz bound in
    /// phase 3 keeps the well's cell alive long enough to find it.
    ///
    /// Scaled by `SCALE`: the branch-and-bound is scale-equivariant (both the
    /// incumbent and the estimated bound scale with `f`), so the minimiser is
    /// independent of `SCALE`.
    fn two_basin_raw(u: f64, v: f64) -> f64 {
        let wide = 0.25 * ((u + 4.2).powi(2) + (v + 3.1).powi(2)) - 0.5;
        let well = 90.0 * ((u - WELL_U).powi(2) + (v - WELL_V).powi(2)) - 1.2;
        wide.min(well)
    }

    fn two_basin(u: f64, v: f64) -> f64 {
        const SCALE: f64 = 1e-3;
        SCALE * two_basin_raw(u, v)
    }

    /// The same landscape lifted by a large constant. Adding a constant to `f`
    /// shifts every value equally: it leaves the minimiser, the true Lipschitz
    /// constant and every pruning decision untouched.
    fn two_basin_offset(u: f64, v: f64) -> f64 {
        OFFSET + two_basin_raw(u, v)
    }
    const OFFSET: f64 = 1000.0;
    const WELL_U: f64 = -2.656_25;
    const WELL_V: f64 = -1.046_875;
    const TWO_BASIN_MIN: f64 = -1.2e-3;

    #[test]
    fn minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see() {
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(two_basin, U_RANGE, V_RANGE, 1e-4);
        assert_result_contract(&two_basin, u, v, val);

        // No point of the domain can beat the closed-form floor of the well.
        assert!(
            val >= TWO_BASIN_MIN - 1e-15,
            "val={val} below analytic floor"
        );
        // The wide bowl bottoms out at -0.5 * SCALE; landing there means the
        // subdivision phase never reached the global basin.
        assert!(val < 0.99 * TWO_BASIN_MIN, "val={val}, local basin only");
        assert!((u - WELL_U).abs() < 1e-3, "u={u}");
        assert!((v - WELL_V).abs() < 1e-3, "v={v}");
    }

    #[test]
    fn minimize_2d_is_invariant_under_a_constant_offset() {
        // Same landscape as the test above, lifted by +1000. The optimiser
        // must still land in the deep well: the estimated bound is built from
        // *differences* of sampled values, which a constant cannot change.
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(two_basin_offset, U_RANGE, V_RANGE, 1e-4);
        assert_result_contract(&two_basin_offset, u, v, val);

        assert!(
            val >= OFFSET - 1.2 - 1e-12,
            "val={val} below analytic floor"
        );
        assert!(val < OFFSET - 1.19, "val={val}, local basin only");
        assert!((u - WELL_U).abs() < 1e-3, "u={u}");
        assert!((v - WELL_V).abs() < 1e-3, "v={v}");
    }

    #[test]
    fn minimize_2d_short_circuits_a_constant_objective() {
        use std::cell::Cell;

        // A constant objective has an estimated Lipschitz bound of exactly
        // zero, which the documented "function appears constant" branch must
        // detect before entering the subdivision phase.
        let calls = Cell::new(0_usize);
        let f = |_u: f64, _v: f64| {
            let n = calls.get() + 1;
            calls.set(n);
            assert!(n < 100_000, "constant objective was not short-circuited");
            4.25
        };
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(f, U_RANGE, V_RANGE, 1e-4);

        assert!((val - 4.25).abs() < 1e-15, "val={val}");
        assert!(u >= U_RANGE.0 && u <= U_RANGE.1, "u={u}");
        assert!(v >= V_RANGE.0 && v <= V_RANGE.1, "v={v}");
        // 17x17 grid samples plus 6 evaluations per coordinate-descent step.
        let used = calls.get();
        assert!(
            used < 1_000,
            "used {used} evaluations on a constant function"
        );
    }

    /// Separable double-well objective. Along each axis the function is
    /// `min(narrow deep parabola, wide shallow parabola)`; the deep well is at
    /// `DEEP_U` / `DEEP_V` with floor `-0.5`, so the global minimum is exactly
    /// `-1.0` at `(DEEP_U, DEEP_V)`.
    ///
    /// The nearest grid sample sits ~0.09 (u) / ~0.11 (v) away from the deep
    /// well, and a ternary search over the *whole* axis is deceived by the
    /// shallow well, so only a bracket of the documented width — one grid step
    /// either side of the best grid sample — refines onto the true optimum.
    fn double_well(u: f64, v: f64) -> f64 {
        let gu = (30.0 * (u - DEEP_U).powi(2) - 0.5).min(0.3 * (u + 2.8).powi(2) - 0.1);
        let gv = (30.0 * (v - DEEP_V).powi(2) - 0.5).min(0.3 * (v + 1.2).powi(2) - 0.1);
        gu + gv
    }
    const DEEP_U: f64 = -4.53;
    const DEEP_V: f64 = -3.45;

    #[test]
    fn minimize_2d_refines_far_below_the_cell_tolerance() {
        // Phase 3 only resolves cells down to `tolerance`; the accuracy below
        // comes from phase 2, whose 50 ternary steps shrink a bracket of width
        // 2*(u1-u0)/16 by (2/3)^50 ≈ 1.6e-9.
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(double_well, U_RANGE, V_RANGE, 2e-2);
        assert_result_contract(&double_well, u, v, val);

        assert!(val >= -1.0 - 1e-12, "val={val} below analytic floor");
        assert!((val + 1.0).abs() < 1e-9, "val={val}");
        assert!((u - DEEP_U).abs() < 1e-5, "u={u}");
        assert!((v - DEEP_V).abs() < 1e-5, "v={v}");
    }

    #[test]
    fn minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum() {
        // Bowl centred outside the domain, so the constrained optimum is the
        // corner (u1, v1) = (-2, -0.5) with value 1^2 + 1^2 = 2.
        let f = |u: f64, v: f64| (u + 1.0).powi(2) + (v - 0.5).powi(2);
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(f, U_RANGE, V_RANGE, 1e-4);
        assert_result_contract(&f, u, v, val);

        assert!((u - U_RANGE.1).abs() < 1e-9, "u={u}");
        assert!((v - V_RANGE.1).abs() < 1e-9, "v={v}");
        assert!((val - 2.0).abs() < 1e-9, "val={val}");
    }

    #[test]
    fn minimize_2d_honours_the_documented_evaluation_budget() {
        use std::cell::Cell;

        // A tall narrow spike makes the grid-estimated Lipschitz bound far
        // larger than the surrounding variation, so the pruning test almost
        // never fires and the subdivision runs into its cell budget.
        let calls = Cell::new(0_usize);
        let f = |u: f64, v: f64| {
            let n = calls.get() + 1;
            calls.set(n);
            assert!(n < 2_000_000, "evaluation budget not enforced");
            (u + 3.5).powi(2)
                + (v + 2.25).powi(2)
                + 200.0 * (-100.0 * ((u + 2.2).powi(2) + (v + 0.8).powi(2))).exp()
        };
        let opt = LipschitzOptimizer::new();
        let (u, v, val) = opt.minimize_2d(f, U_RANGE, V_RANGE, 1e-6);

        // 500 000 cells + a 17x17 grid + 6 evaluations per descent step.
        let used = calls.get();
        assert!(used <= 501_000, "used {used} evaluations");
        assert!(u >= U_RANGE.0 && u <= U_RANGE.1, "u={u}");
        assert!(v >= V_RANGE.0 && v <= V_RANGE.1, "v={v}");
        // The spike is strictly positive, so the bowl floor is still global.
        assert!((0.0..1e-9).contains(&val), "val={val}");
        assert!(
            (u + 3.5).abs() < 1e-6 && (v + 2.25).abs() < 1e-6,
            "({u},{v})"
        );
    }

    /// Degree-1 segment from `(0,0,0)` to `(4,0,0)` over the domain `[1, 2]`.
    fn line_segment_c1() -> NurbsCurve {
        NurbsCurve::new(
            1,
            vec![1.0, 1.0, 2.0, 2.0],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap()
    }

    /// Quadratic Bezier over `[1, 2]` with, in local parameter `s = u - 1`,
    /// `C2(s) = (4s, 24s^2 - 24s + 2, 0)`.
    fn bulging_bezier_c2() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![1.0, 1.0, 1.0, 2.0, 2.0, 2.0],
            vec![
                Point3::new(0.0, 2.0, 0.0),
                Point3::new(2.0, -10.0, 0.0),
                Point3::new(4.0, 2.0, 0.0),
            ],
            vec![1.0, 1.0, 1.0],
        )
        .unwrap()
    }

    #[test]
    fn curve_curve_lipschitz_matches_the_documented_bound() {
        // Both curves share the parameterisation `x = 4s`, so the sampled
        // separation is |24s^2 - 24s + 2|, maximal at the sample s = 1/2 with
        // value 4. |C1'| = 4 everywhere; |C2'| = sqrt(16 + (48s - 24)^2) is
        // maximal at the samples s = 0 and s = 1 with value sqrt(592).
        // Hence L = 2 * 4 * sqrt(592).
        let c1 = line_segment_c1();
        let c2 = bulging_bezier_c2();
        let l = estimate_curve_curve_lipschitz(&c1, &c2);
        let expected = 8.0 * 592.0_f64.sqrt();
        assert!((l - expected).abs() < 1e-9, "l={l}, expected {expected}");
    }

    #[test]
    fn nurbs_distance_between_skew_segments_is_not_the_midpoint_distance() {
        // C1 runs along x at y = z = 0; C2 is the vertical segment through
        // (2.9, 1, z). The closest pair is (2.9, 0, 0) / (2.9, 1, 0) at
        // distance 1, while the two domain midpoints are sqrt(0.81 + 1 + 6.25)
        // ≈ 2.84 apart.
        let c1 = line_segment_c1();
        let c2 = NurbsCurve::new(
            1,
            vec![1.0, 1.0, 2.0, 2.0],
            vec![Point3::new(2.9, 1.0, 0.0), Point3::new(2.9, 1.0, 5.0)],
            vec![1.0, 1.0],
        )
        .unwrap();

        let (dist, p1, p2) = nurbs_curve_curve_distance(&c1, &c2);
        assert!((dist - 1.0).abs() < 1e-6, "dist={dist}");
        assert!(
            (p1 - Point3::new(2.9, 0.0, 0.0)).length() < 1e-4,
            "p1={p1:?}"
        );
        assert!(
            (p2 - Point3::new(2.9, 1.0, 0.0)).length() < 1e-4,
            "p2={p2:?}"
        );
    }

    #[test]
    fn nurbs_distance_between_coincident_curves_is_zero() {
        // Identical curves sample to identical points, so the estimated
        // Lipschitz bound is exactly zero and the degenerate branch is taken.
        // Whatever parameter that branch picks, it must pick the *same* one on
        // both curves, or the reported distance is not zero.
        let c1 = line_segment_c1();
        let c2 = line_segment_c1();
        assert!(estimate_curve_curve_lipschitz(&c1, &c2) < 1e-15);

        let (dist, p1, p2) = nurbs_curve_curve_distance(&c1, &c2);
        assert!(dist < 1e-12, "dist={dist}");
        assert!((p1 - p2).length() < 1e-12, "p1={p1:?} p2={p2:?}");
        // Both points must lie on the shared segment.
        assert!(p1.x() >= -1e-12 && p1.x() <= 4.0 + 1e-12, "p1={p1:?}");
        assert!(p1.y().abs() < 1e-12 && p1.z().abs() < 1e-12, "p1={p1:?}");
    }
}
