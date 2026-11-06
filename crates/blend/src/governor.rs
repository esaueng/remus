//! Cooperative cancellation and a deterministic work budget for one blend
//! operation.
//!
//! The blend engines are a cascade of attempts, each of which may walk
//! several stripes, and none of them carries an
//! [`OperationContext`](remus_math::context::OperationContext) through its
//! call graph. A caller that must bound a fillet scopes the whole operation
//! with [`governed`]; every engine then answers to that scope at its safe
//! points:
//!
//! - [`charge_step`] once per walker step attempt (a step the walker retries
//!   at half length counts again). Every walking engine (the stripe walker,
//!   rail and ridge chains) goes through
//!   [`Walker::walk`](crate::walker::Walker::walk), so the step count is the
//!   operation's walking work, independent of wall-clock and host speed.
//! - [`checkpoint`] between attempts and before validation, where abandoning
//!   work is safe because the caller's transaction restores the topology.
//!
//! A trip is sticky: once the token is cancelled or the budget is exhausted,
//! every later checkpoint in the scope fails too, so a cascade that turns
//! one attempt's error into "try the next engine" cannot run on. [`governed`]
//! reports the trip, and the caller must refuse the whole operation with it
//! (and roll back) even if a later attempt happened to succeed without
//! reaching a checkpoint.
//!
//! Outside a scope every function here is a no-op that always succeeds, so
//! the ungoverned public entry points keep their behavior exactly.

use std::cell::RefCell;

use remus_math::MathError;
use remus_math::context::CancellationToken;

use crate::BlendError;

/// The stable resource name a station-budget trip reports.
pub const STEP_RESOURCE: &str = "blend_walk_steps";

#[derive(Clone, Copy)]
enum Trip {
    Cancelled,
    Steps { limit: usize, actual: usize },
}

impl Trip {
    const fn error(self) -> MathError {
        match self {
            Self::Cancelled => MathError::Cancelled,
            Self::Steps { limit, actual } => MathError::ResourceLimitExceeded {
                resource: STEP_RESOURCE,
                limit,
                actual,
            },
        }
    }
}

struct Scope {
    cancellation: Option<CancellationToken>,
    step_limit: usize,
    steps: usize,
    trip: Option<Trip>,
}

thread_local! {
    static SCOPE: RefCell<Option<Scope>> = const { RefCell::new(None) };
}

/// What a governed blend did.
#[derive(Debug)]
pub struct GovernedRun<T> {
    /// The operation's own result.
    pub result: T,
    /// Walker step attempts made inside the scope (deterministic work count).
    pub steps: usize,
    /// The cancellation or budget refusal that tripped the scope, if any.
    /// When present the caller must refuse the operation with it.
    pub trip: Option<MathError>,
}

/// Restores the enclosing scope when a governed run ends, even by unwinding.
struct Restore(Option<Scope>);

impl Drop for Restore {
    fn drop(&mut self) {
        let outer = self.0.take();
        SCOPE.with(|scope| *scope.borrow_mut() = outer);
    }
}

/// Runs `run` under a blend scope with an optional cancellation token and a
/// cap on the total walker step attempts it may make.
///
/// Scopes nest: an inner scope governs only its own run and the enclosing
/// one resumes afterwards (an inner scope's steps are not charged to the
/// outer one).
pub fn governed<T>(
    cancellation: Option<CancellationToken>,
    step_limit: usize,
    run: impl FnOnce() -> T,
) -> GovernedRun<T> {
    let outer = SCOPE.with(|scope| {
        scope.borrow_mut().replace(Scope {
            cancellation,
            step_limit,
            steps: 0,
            trip: None,
        })
    });
    let restore = Restore(outer);
    let result = run();
    let (steps, trip) = SCOPE.with(|scope| {
        scope
            .borrow()
            .as_ref()
            .map_or((0, None), |s| (s.steps, s.trip.map(Trip::error)))
    });
    drop(restore);
    GovernedRun {
        result,
        steps,
        trip,
    }
}

fn poll(scope: &mut Scope) -> Result<(), BlendError> {
    if let Some(trip) = scope.trip {
        return Err(BlendError::Math(trip.error()));
    }
    if scope
        .cancellation
        .as_ref()
        .is_some_and(CancellationToken::is_cancelled)
    {
        scope.trip = Some(Trip::Cancelled);
        return Err(BlendError::Math(MathError::Cancelled));
    }
    Ok(())
}

/// Fails with the scope's refusal when its token is cancelled or it already
/// tripped; succeeds outside a scope.
///
/// # Errors
///
/// [`MathError::Cancelled`] or the sticky
/// [`MathError::ResourceLimitExceeded`] trip, wrapped in [`BlendError::Math`].
pub fn checkpoint() -> Result<(), BlendError> {
    SCOPE.with(|scope| match scope.borrow_mut().as_mut() {
        Some(scope) => poll(scope),
        None => Ok(()),
    })
}

/// Charges one walker step attempt, then polls like [`checkpoint`].
///
/// # Errors
///
/// [`MathError::ResourceLimitExceeded`] (resource [`STEP_RESOURCE`]) once
/// the scope's step budget is spent, or a [`checkpoint`] refusal.
pub fn charge_step() -> Result<(), BlendError> {
    SCOPE.with(|scope| {
        let mut scope = scope.borrow_mut();
        let Some(scope) = scope.as_mut() else {
            return Ok(());
        };
        poll(scope)?;
        scope.steps = scope.steps.saturating_add(1);
        if scope.steps > scope.step_limit {
            let trip = Trip::Steps {
                limit: scope.step_limit,
                actual: scope.steps,
            };
            scope.trip = Some(trip);
            return Err(BlendError::Math(trip.error()));
        }
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn outside_a_scope_everything_succeeds() {
        assert!(checkpoint().is_ok());
        for _ in 0..10 {
            assert!(charge_step().is_ok());
        }
    }

    #[test]
    fn station_budget_trips_once_exceeded_and_stays_tripped() {
        let run = governed(None, 3, || {
            let charged: Vec<bool> = (0..5).map(|_| charge_step().is_ok()).collect();
            (charged, checkpoint().is_ok())
        });
        assert_eq!(run.result.0, [true, true, true, false, false]);
        assert!(!run.result.1, "the trip is sticky for later checkpoints");
        assert_eq!(
            run.steps, 4,
            "the refused step is counted, later ones are not"
        );
        assert!(matches!(
            run.trip,
            Some(MathError::ResourceLimitExceeded {
                resource: STEP_RESOURCE,
                limit: 3,
                actual: 4,
            })
        ));
        assert!(checkpoint().is_ok(), "the scope ends with the run");
    }

    #[test]
    fn cancellation_is_observed_at_the_next_safe_point() {
        let token = CancellationToken::new();
        let run = governed(Some(token.clone()), usize::MAX, || {
            let before = charge_step().is_ok();
            token.cancel();
            (before, charge_step().is_ok(), checkpoint().is_ok())
        });
        assert_eq!(run.result, (true, false, false));
        assert_eq!(run.steps, 1);
        assert!(matches!(run.trip, Some(MathError::Cancelled)));
    }

    #[test]
    fn nested_scopes_restore_the_outer_one() {
        let run = governed(None, 2, || {
            charge_step().unwrap();
            let inner = governed(None, 0, || charge_step().is_ok());
            assert!(!inner.result);
            assert!(inner.trip.is_some());
            (charge_step().is_ok(), charge_step().is_ok())
        });
        assert_eq!(run.result, (true, false));
        assert_eq!(run.steps, 3);
    }
}
