//! Solve-local final-iterate evaluation sharing (PERF-S06).
//!
//! `solve_detailed` used to re-evaluate everything after `solve` returned: a
//! fresh snapshot plus Jacobian plus QR inside `dof()`, then another fresh
//! snapshot plus a full residual pass inside `constraint_residuals()`, all at
//! the exact iterate the solver had just evaluated. This module lets the
//! diagnostics reuse the solver's own final evaluation instead — with an
//! explicit identity check, never a bare assumption that "nothing changed".
//!
//! # State semantics (pinned by tests in [`tests`])
//!
//! - `solve` publishes its final iterate: whatever params the solver stopped
//!   on are written back, converged or not.
//! - `solve_detailed` additionally measures at that attempted state, then
//!   restores the pre-solve geometry when the attempt did not converge.
//! - The per-constraint residuals therefore describe the *attempted* state,
//!   which is the informative one: satisfiable constraints read ~0 there,
//!   while the starting geometry would read large everywhere.
//! - `published_max_residual` describes the state actually left published:
//!   it equals `max_residual` unless the attempt was rolled back, in which
//!   case it is re-measured at the restored starting state (a genuinely
//!   different state, so it is always freshly evaluated, never shared).
//!
//! # Sharing contract
//!
//! A [`FinalEvaluation`] captures the solver's flat residual vector together
//! with the identity of the state it was evaluated at: the final parameter
//! values (bitwise), the constraint evaluation order as `(id, row count)`
//! pairs, and the residual/parameter dimensions. Reuse is allowed only when
//! [`FinalEvaluation::matches`] confirms all of those against the live
//! system. In particular:
//!
//! - An accepted final solver step can invalidate the loop's previous
//!   factorization, so no factorization is ever shared: rank always comes
//!   from a fresh factorization of a Jacobian evaluated at the verified
//!   final params. Only the residual vector is shared.
//! - The solve tolerance needs no separate identity field: it selects *which*
//!   iterate is final, and the identity compares the actual final params, so
//!   any tolerance-driven difference surfaces as a parameter mismatch and
//!   falls back to fresh evaluation. Rank uses the shared `dof::analyze`
//!   threshold on both the shared and the fresh paths.
//! - Nothing is retained across solves: the capture is created and consumed
//!   inside one `solve_detailed` call. A later edit (moved point, added or
//!   removed constraint) changes the params or the row layout, the identity
//!   check fails, and measurement falls back to fresh evaluation.
//!
//! [`tests`]: self::tests

use super::constraint::ConstraintId;

/// Identity of the system state a residual evaluation belongs to.
///
/// All fields must agree before a captured evaluation may be reused. The
/// comparison is deliberately strict (bitwise params, exact row layout):
/// a false positive would attribute residuals to the wrong state, while a
/// false negative only costs one fresh evaluation.
#[derive(Debug, Clone)]
pub struct EvalIdentity {
    /// Final parameter values, in `param_map` order.
    params: Vec<f64>,
    /// `(constraint id, residual row count)` in evaluation (arena) order.
    rows: Vec<(ConstraintId, usize)>,
    /// Number of residual equations (`residuals.len()`).
    num_residuals: usize,
    /// Number of free solver parameters.
    num_params: usize,
}

/// The solver's final-iterate evaluation, solve-local (PERF-S06).
///
/// Created from the solver workspace immediately after the solve loop, used
/// for the diagnostics of the same `solve_detailed` call, then dropped.
/// Never stored on the system, never reused across solves.
#[derive(Debug, Clone)]
pub struct FinalEvaluation {
    identity: EvalIdentity,
    /// Flat residual vector at the final iterate, in evaluation order.
    residuals: Vec<f64>,
}

impl FinalEvaluation {
    /// Capture the solver's final evaluation.
    ///
    /// - `params`: the final parameter values the solver stopped on.
    /// - `residuals`: the residual vector evaluated at `params`
    ///   ([`super::solver::DoglegWorkspace::final_residuals`]).
    /// - `rows`: `(constraint id, row count)` in evaluation order.
    pub fn capture(params: &[f64], residuals: &[f64], rows: Vec<(ConstraintId, usize)>) -> Self {
        Self {
            identity: EvalIdentity {
                params: params.to_vec(),
                num_residuals: residuals.len(),
                num_params: params.len(),
                rows,
            },
            residuals: residuals.to_vec(),
        }
    }

    /// Whether this capture still describes the live system state.
    ///
    /// Compares final params bitwise (NaN-safe: `NaN != NaN` would force a
    /// fresh evaluation, which is the safe direction), the exact constraint
    /// row layout in order, and both dimensions. Any edit between capture
    /// and reuse — moved geometry, added/removed constraints, rebuilt
    /// parameter map — fails the check.
    pub fn matches(
        &self,
        params_now: &[f64],
        rows_now: &[(ConstraintId, usize)],
        num_residuals: usize,
        num_params: usize,
    ) -> bool {
        if self.identity.num_residuals != num_residuals
            || self.identity.num_params != num_params
            || self.identity.rows.len() != rows_now.len()
            || self.identity.params.len() != params_now.len()
        {
            return false;
        }
        if self.identity.rows != rows_now {
            return false;
        }
        self.identity
            .params
            .iter()
            .zip(params_now.iter())
            .all(|(a, b)| a.to_bits() == b.to_bits())
    }

    /// The shared flat residual vector, in evaluation order.
    pub fn residuals(&self) -> &[f64] {
        &self.residuals
    }

    /// Row layout the capture was evaluated with, in evaluation order.
    pub fn rows(&self) -> &[(ConstraintId, usize)] {
        &self.identity.rows
    }
}

#[cfg(test)]
mod tests;
