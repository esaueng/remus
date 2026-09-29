//! Degrees-of-freedom analysis via QR rank detection.

use super::qr::QrResult;

/// Result of DOF analysis on the constraint system.
#[derive(Debug, Clone, Copy)]
pub struct DofAnalysis {
    /// Degrees of freedom remaining (under-constrained dimensions).
    pub dof: usize,
    /// Rank of the Jacobian matrix.
    pub rank: usize,
    /// Total number of solver parameters.
    pub num_params: usize,
    /// Total number of constraint equations.
    pub num_equations: usize,
}
/// Rank threshold shared by [`analyze`] and [`analyze_blocks`].
const RANK_TOL: f64 = 1e-10;

/// Analyze degrees of freedom from a Jacobian matrix.
///
/// DOF = `num_params - rank(J)`. A fully constrained system has DOF = 0.
/// Over-constrained systems have `num_equations > num_params` with DOF = 0
/// (or negative if constraints are contradictory, though we report 0).
pub fn analyze(jacobian: &[f64], m: usize, n: usize) -> DofAnalysis {
    let rank = if m == 0 || n == 0 {
        0
    } else {
        let mut data = jacobian.to_vec();
        let qr = QrResult::factorize(&mut data, m, n);
        qr.rank(RANK_TOL)
    };
    DofAnalysis {
        dof: n.saturating_sub(rank),
        rank,
        num_params: n,
        num_equations: m,
    }
}

/// Analyze degrees of freedom across independent Jacobian blocks (PERF-S02).
///
/// Each `(jacobian, m, n)` triple is one component's row-major Jacobian at the
/// same state. Blocks factorize independently — each cubic cost scales with
/// block size — but rank counts every block's pivots against ONE absolute
/// threshold derived from the global leading magnitude (the max over blocks).
/// That max equals the leading magnitude a global factorization would produce
/// (reflections never couple zero-separated blocks), so the policy is exactly
/// the [`analyze`] policy applied to the block-diagonal assembly: no component
/// is silently re-ranked on its own scale. A fully zero assembly reports rank
/// 0 through the same `1e-300` guard [`QrResult::rank`] uses.
///
/// Degenerate blocks (`m == 0 || n == 0`: the free and pinned groups) carry no
/// factorizable matrix and contribute rank 0, matching [`analyze`].
#[must_use]
pub fn analyze_blocks(
    blocks: &[(Vec<f64>, usize, usize)],
    num_params: usize,
    num_equations: usize,
) -> DofAnalysis {
    let mut factored: Vec<QrResult> = Vec::with_capacity(blocks.len());
    let mut global_leading = 0.0_f64;
    for (jac, m, n) in blocks {
        if *m == 0 || *n == 0 {
            continue;
        }
        let mut data = jac.clone();
        let qr = QrResult::factorize(&mut data, *m, *n);
        global_leading = global_leading.max(qr.leading_magnitude());
        factored.push(qr);
    }
    let rank = if global_leading < 1e-300 {
        0
    } else {
        let threshold = RANK_TOL * global_leading;
        factored.iter().map(|qr| qr.rank_absolute(threshold)).sum()
    };
    DofAnalysis {
        dof: num_params.saturating_sub(rank),
        rank,
        num_params,
        num_equations,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn fully_constrained() {
        // 2 equations, 2 unknowns, full rank
        let j = vec![1.0, 0.0, 0.0, 1.0];
        let result = analyze(&j, 2, 2);
        assert_eq!(result.dof, 0);
        assert_eq!(result.rank, 2);
    }

    #[test]
    fn under_constrained() {
        // 1 equation, 2 unknowns → DOF = 1
        let j = vec![1.0, 1.0];
        let result = analyze(&j, 1, 2);
        assert_eq!(result.dof, 1);
        assert_eq!(result.rank, 1);
    }

    #[test]
    fn over_constrained_redundant() {
        // 3 equations, 2 unknowns, but row 3 = row 1 → rank 2
        let j = vec![1.0, 0.0, 0.0, 1.0, 1.0, 0.0];
        let result = analyze(&j, 3, 2);
        assert_eq!(result.dof, 0);
        assert_eq!(result.rank, 2);
    }

    #[test]
    fn empty_system() {
        let result = analyze(&[], 0, 0);
        assert_eq!(result.dof, 0);
        assert_eq!(result.rank, 0);
    }

    #[test]
    fn free_point_has_two_dof() {
        // No constraints, 2 params → DOF = 2
        let result = analyze(&[], 0, 2);
        assert_eq!(result.dof, 2);
    }

    #[test]
    fn blocks_match_dense_on_single_block() {
        let j = vec![1.0, 0.0, 0.0, 1.0];
        let dense = analyze(&j, 2, 2);
        let split = analyze_blocks(&[(j, 2, 2)], 2, 2);
        assert_eq!((split.dof, split.rank), (dense.dof, dense.rank));
    }

    #[test]
    fn blocks_sum_independent_ranks() {
        let a = vec![1.0, 0.0, 0.0, 1.0];
        let b = vec![2.0];
        let r = analyze_blocks(&[(a, 2, 2), (b, 1, 1)], 3, 3);
        assert_eq!((r.rank, r.dof), (3, 0));
    }

    #[test]
    fn blocks_skip_degenerate_groups() {
        // Free (m == 0) and pinned (n == 0) groups carry no matrix.
        let a = vec![1.0, 1.0];
        let r = analyze_blocks(&[(a, 1, 2), (vec![], 0, 2), (vec![], 3, 0)], 4, 4);
        assert_eq!((r.rank, r.dof), (1, 3));
    }

    #[test]
    fn blocks_keep_the_global_threshold() {
        // A 1e-12-scale pivot beside unit pivots: the global 1e-10·|R00|
        // policy drops it, and the block path must agree with the dense path
        // on the explicit block-diagonal assembly — not with a per-block
        // relative threshold, which would count it.
        let assembled = vec![
            1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, //
            0.0, 0.0, 1e-12,
        ];
        let dense = analyze(&assembled, 3, 3);
        assert_eq!(dense.rank, 2);
        let split = analyze_blocks(
            &[(vec![1.0, 0.0, 0.0, 1.0], 2, 2), (vec![1e-12], 1, 1)],
            3,
            3,
        );
        assert_eq!((split.rank, split.dof), (dense.rank, dense.dof));
    }
}
