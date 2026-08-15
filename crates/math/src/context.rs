//! Operation context: explicit tolerance and resource budgets for modeling
//! operations.
//!
//! [`OperationContext`] is the carrier for policy that algorithms previously
//! hard-coded: comparison tolerances and work budgets. Passing it explicitly
//! makes numeric and resource behavior a caller-visible contract instead of a
//! scattering of module-local constants, and gives every budget one place to
//! be observed and tuned.
//!
//! The context is introduced incrementally (see
//! `docs/design/rfc-0001-operation-context.md`). The default context
//! reproduces the exact constants the integrated algorithms used before it
//! existed, so `*_with_context` entry points called with
//! [`OperationContext::new`] behave identically to their legacy counterparts.
//!
//! Both structs are `#[non_exhaustive]`: construct them with
//! [`OperationContext::new`] and adjust fields through the `with_*` builders,
//! so future policy fields (fallback policy, cancellation, diagnostics) can
//! be added without breaking callers.

use crate::tolerance::Tolerance;

/// Hard work budgets for iterative and exploratory algorithms.
///
/// Every field is an upper bound the algorithm must respect; exhausting a
/// budget terminates the work bounded, it never loops on. Defaults reproduce
/// the constants used by the surface-surface intersection path before
/// budgets were threaded through it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct WorkBudgets {
    /// Maximum marching steps per traced direction of one intersection
    /// curve.
    pub march_steps: usize,
    /// Maximum pending seeds in the branch-aware marcher's work queue.
    pub queue_size: usize,
    /// Maximum traced curve segments before branch exploration stops.
    pub segments: usize,
    /// Maximum branch points detected per march direction.
    pub branches_per_direction: usize,
}

impl WorkBudgets {
    /// Default budgets — identical to the pre-context constants.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            march_steps: 200,
            queue_size: 100,
            segments: 50,
            branches_per_direction: 10,
        }
    }

    /// Returns budgets with the given marching-step cap.
    #[must_use]
    pub const fn with_march_steps(mut self, value: usize) -> Self {
        self.march_steps = value;
        self
    }

    /// Returns budgets with the given work-queue cap.
    #[must_use]
    pub const fn with_queue_size(mut self, value: usize) -> Self {
        self.queue_size = value;
        self
    }

    /// Returns budgets with the given traced-segment cap.
    #[must_use]
    pub const fn with_segments(mut self, value: usize) -> Self {
        self.segments = value;
        self
    }

    /// Returns budgets with the given per-direction branch cap.
    #[must_use]
    pub const fn with_branches_per_direction(mut self, value: usize) -> Self {
        self.branches_per_direction = value;
        self
    }
}

impl Default for WorkBudgets {
    fn default() -> Self {
        Self::new()
    }
}

/// Explicit per-operation policy: tolerances and work budgets.
///
/// The default context ([`OperationContext::new`]) reproduces legacy
/// behavior exactly; `*_with_context` entry points called with it return the
/// same results as their context-free counterparts.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct OperationContext {
    /// Geometric comparison tolerances.
    pub tolerance: Tolerance,
    /// Hard work budgets for iterative algorithms.
    pub budgets: WorkBudgets,
}

impl OperationContext {
    /// The default context: default [`Tolerance`] and default
    /// [`WorkBudgets`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tolerance: Tolerance::new(),
            budgets: WorkBudgets::new(),
        }
    }

    /// Returns the context with the given tolerance.
    #[must_use]
    pub const fn with_tolerance(mut self, tolerance: Tolerance) -> Self {
        self.tolerance = tolerance;
        self
    }

    /// Returns the context with the given work budgets.
    #[must_use]
    pub const fn with_budgets(mut self, budgets: WorkBudgets) -> Self {
        self.budgets = budgets;
        self
    }
}

impl Default for OperationContext {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_budgets_match_legacy_constants() {
        let b = WorkBudgets::new();
        assert_eq!(b.march_steps, 200);
        assert_eq!(b.queue_size, 100);
        assert_eq!(b.segments, 50);
        assert_eq!(b.branches_per_direction, 10);
    }

    #[test]
    fn default_context_uses_default_tolerance() {
        let ctx = OperationContext::new();
        assert_eq!(ctx.tolerance, Tolerance::new());
        assert_eq!(ctx.budgets, WorkBudgets::new());
        assert_eq!(OperationContext::default(), ctx);
    }

    #[test]
    fn builders_replace_only_their_field() {
        let ctx = OperationContext::new()
            .with_tolerance(Tolerance::loose())
            .with_budgets(WorkBudgets::new().with_march_steps(7));
        assert_eq!(ctx.tolerance, Tolerance::loose());
        assert_eq!(ctx.budgets.march_steps, 7);
        assert_eq!(ctx.budgets.queue_size, 100);
    }
}
