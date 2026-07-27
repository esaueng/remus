//! Solver diagnostics: what the solver actually proved about a system.
//!
//! These types are deliberately conservative. A constraint carrying a large
//! residual is *evidence* about where a system is unsatisfied, never proof
//! that it is the constraint at fault — a single bad constraint pushes error
//! into every constraint that shares its parameters. Nothing here names a
//! culprit; callers get measured magnitudes plus the rank data needed to draw
//! their own conclusions.

use super::constraint::ConstraintId;

/// How the system stands after a solve attempt.
///
/// The variants are ordered by the precedence [`classify`] applies, which is
/// documented on that function. `dof`, `rank`, and `num_equations` on
/// [`SolveDiagnostics`] remain authoritative: a system can be simultaneously
/// under-constrained and redundant, and only one variant can be reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveClassification {
    /// Converged, no remaining freedom, and every equation is independent.
    Solved,
    /// Converged, but the geometry still has unconstrained freedom.
    UnderConstrained,
    /// Converged with no remaining freedom, but some equations are linearly
    /// dependent on others. The dependent equations are consistent — they are
    /// satisfied — they simply carry no new information.
    ///
    /// A system with no free parameters at all (every point pinned via
    /// `PointData::fixed`, no circles) reports this whenever it carries any
    /// constraint. That is the literal truth: with zero columns every
    /// Jacobian row is the zero row, so the rows are linearly dependent and
    /// no constraint is doing work the `fixed` flags had not already done.
    Redundant,
    /// The solver did not reach the requested tolerance. This says nothing
    /// about *which* constraint is unsatisfiable: non-convergence is equally
    /// consistent with contradictory constraints, a poor starting point, or an
    /// iteration budget that was too small.
    Unsatisfied,
}

impl SolveClassification {
    /// Stable lowercase identifier, suitable for a wire format.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Solved => "solved",
            Self::UnderConstrained => "underConstrained",
            Self::Redundant => "redundant",
            Self::Unsatisfied => "unsatisfied",
        }
    }
}

/// Classify a solve outcome from measured facts only.
///
/// Precedence, highest first:
/// 1. Did not converge → [`SolveClassification::Unsatisfied`].
/// 2. `dof > 0` → [`SolveClassification::UnderConstrained`]. Remaining freedom
///    is reported ahead of redundancy because it is the state a sketch UI must
///    act on; [`SolveDiagnostics::redundant`] still reports the redundancy.
/// 3. `rank < num_equations` → [`SolveClassification::Redundant`].
/// 4. Otherwise → [`SolveClassification::Solved`].
#[must_use]
pub const fn classify(
    converged: bool,
    dof: usize,
    rank: usize,
    num_equations: usize,
) -> SolveClassification {
    if !converged {
        SolveClassification::Unsatisfied
    } else if dof > 0 {
        SolveClassification::UnderConstrained
    } else if rank < num_equations {
        SolveClassification::Redundant
    } else {
        SolveClassification::Solved
    }
}

/// Residual magnitude attributed to one constraint.
#[derive(Debug, Clone, Copy)]
pub struct ConstraintResidual {
    /// The constraint this magnitude was measured on.
    pub constraint: ConstraintId,
    /// Largest absolute residual across the constraint's equations, measured
    /// at the solver's *final iterate* — its best attempt, before any
    /// rollback. Constraints the system could satisfy have driven this to
    /// ~0, so a magnitude that survives marks where it could not.
    pub max_abs_residual: f64,
    /// Whether this constraint was created by the kernel rather than the
    /// caller. Internal constraints (an arc's centre–endpoint tie) have no
    /// caller-facing handle and must never be reported as a user constraint.
    pub internal: bool,
}

/// Everything a solve attempt established about the system.
///
/// Two residual figures are reported because they can legitimately differ.
/// [`Self::max_residual`] is what the solver reached at its final iterate — it
/// matches what a plain solve returns, and is the state
/// [`Self::residuals`], [`Self::dof`] and [`Self::rank`] describe.
/// [`Self::published_max_residual`] is measured on the state actually left in
/// the system, which differs whenever [`Self::rolled_back`] is set.
#[derive(Debug, Clone)]
pub struct SolveDiagnostics {
    /// Whether the solver reached the requested tolerance.
    pub converged: bool,
    /// DogLeg iterations consumed.
    pub iterations: usize,
    /// Maximum absolute residual at the solver's final iterate.
    pub max_residual: f64,
    /// Maximum absolute residual at the state now published in the system.
    /// Equal to [`Self::max_residual`] unless the attempt was rolled back.
    pub published_max_residual: f64,
    /// Remaining degrees of freedom (`num_params - rank`).
    pub dof: usize,
    /// Rank of the constraint Jacobian.
    pub rank: usize,
    /// Number of free solver parameters.
    pub num_params: usize,
    /// Number of residual equations, internal constraints included.
    pub num_equations: usize,
    /// Per-constraint residual magnitudes at the solver's final iterate, in
    /// the system's stable constraint order.
    pub residuals: Vec<ConstraintResidual>,
    /// Largest residual over kernel-internal constraints alone, reported
    /// separately so it is never mistaken for a caller's constraint.
    pub internal_max_residual: f64,
    /// Whether the attempt was discarded and the pre-solve geometry restored.
    pub rolled_back: bool,
    /// Whether any equation is linearly dependent on the others
    /// (`rank < num_equations`). True independently of
    /// [`Self::classification`], which can only report one state.
    pub redundant: bool,
    /// Overall classification. See [`classify`] for the precedence rules.
    pub classification: SolveClassification,
}
