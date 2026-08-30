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

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::MathError;
use crate::tolerance::Tolerance;

/// A cooperative cancellation signal shared between an operation and its
/// caller.
///
/// Cancellation is monotonic: once requested, every clone observes it. Long
/// running algorithms poll the token only at documented safe points, so a
/// cancellation never exposes partially-mutated topology.
#[derive(Clone, Debug)]
pub struct CancellationToken {
    cancelled: Arc<AtomicBool>,
}

impl CancellationToken {
    /// Creates an uncancelled token.
    #[must_use]
    pub fn new() -> Self {
        Self {
            cancelled: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Requests cancellation. The request cannot be reset.
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
    }

    /// Whether cancellation has been requested.
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Acquire)
    }
}

impl Default for CancellationToken {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for CancellationToken {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.cancelled, &other.cancelled)
    }
}

impl Eq for CancellationToken {}

/// The historical mesh-fallback deflection, used as the default
/// approximation budget so the default context reproduces legacy behavior.
pub const DEFAULT_APPROXIMATION_BUDGET: f64 = 0.1;

/// What an operation may do when its exact pipeline cannot produce the
/// result (kernel operation contract; RFC 0001 migration queue item 5).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FallbackPolicy {
    /// Any representation-degrading path fails with a typed error instead
    /// of running. Nothing approximate is ever returned.
    ExactOnly,
    /// Approximation is permitted within the given error budget (model
    /// units; for the mesh boolean this is the tessellation deflection).
    /// The result must disclose that a fallback ran.
    AllowApproximate {
        /// Maximum permitted approximation error, in model units.
        budget: f64,
    },
    /// Skip exact attempts and go straight to the approximate path (bulk
    /// and preview use). Results are still validated and still disclose
    /// their quality.
    ApproximateOnly {
        /// Maximum permitted approximation error, in model units.
        budget: f64,
    },
}

impl FallbackPolicy {
    /// The approximation budget, when the policy permits approximation.
    #[must_use]
    pub const fn budget(self) -> Option<f64> {
        match self {
            Self::ExactOnly => None,
            Self::AllowApproximate { budget } | Self::ApproximateOnly { budget } => Some(budget),
        }
    }
}

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
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct OperationContext {
    /// Geometric comparison tolerances.
    pub tolerance: Tolerance,
    /// Hard work budgets for iterative algorithms.
    pub budgets: WorkBudgets,
    /// What the operation may do when its exact pipeline cannot produce
    /// the result. The default reproduces legacy behavior:
    /// [`FallbackPolicy::AllowApproximate`] with
    /// [`DEFAULT_APPROXIMATION_BUDGET`].
    pub fallback: FallbackPolicy,
    /// Optional cooperative cancellation signal. `None` preserves the legacy
    /// non-cancellable behavior without allocating a token per operation.
    pub cancellation: Option<CancellationToken>,
}

impl OperationContext {
    /// The default context: default [`Tolerance`] and default
    /// [`WorkBudgets`].
    #[must_use]
    pub const fn new() -> Self {
        Self {
            tolerance: Tolerance::new(),
            budgets: WorkBudgets::new(),
            fallback: FallbackPolicy::AllowApproximate {
                budget: DEFAULT_APPROXIMATION_BUDGET,
            },
            cancellation: None,
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

    /// Returns the context with the given fallback policy.
    #[must_use]
    pub const fn with_fallback(mut self, fallback: FallbackPolicy) -> Self {
        self.fallback = fallback;
        self
    }

    /// Returns the context with a shared cooperative cancellation token.
    #[must_use]
    pub fn with_cancellation(mut self, cancellation: CancellationToken) -> Self {
        self.cancellation = Some(cancellation);
        self
    }

    /// Fails with the kernel-wide typed cancellation result when requested.
    ///
    /// Algorithms call this only at points where abandoning work is safe; the
    /// public operation transaction then restores all staged topology.
    ///
    /// # Errors
    ///
    /// Returns [`MathError::Cancelled`] after the attached token is cancelled.
    pub fn check_cancelled(&self) -> Result<(), MathError> {
        if self
            .cancellation
            .as_ref()
            .is_some_and(CancellationToken::is_cancelled)
        {
            Err(MathError::Cancelled)
        } else {
            Ok(())
        }
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

    #[test]
    fn cancellation_is_shared_and_monotonic() {
        let token = CancellationToken::new();
        let context = OperationContext::new().with_cancellation(token.clone());
        assert!(context.check_cancelled().is_ok());

        token.cancel();
        assert!(token.is_cancelled());
        assert!(matches!(
            context.check_cancelled(),
            Err(MathError::Cancelled)
        ));
    }
}
