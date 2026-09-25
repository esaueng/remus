//! Opt-in counters for synchronous transaction scopes on the current thread.
//!
//! These count instrumented transaction snapshots and checkpoint COW copies,
//! not every `Topology::clone`, allocated bytes, or snapshot duration.

use std::{cell::Cell, marker::PhantomData, rc::Rc};

/// Work since the last reset; setup and validation can be excluded by callers.
#[derive(Clone, Copy, Debug, Default)]
pub struct Counts {
    /// Entered transaction scopes, including read-only batch scopes.
    pub transactions: u64,
    /// Deep rollback snapshots created at transaction entry.
    pub snapshots: u64,
    /// Copies triggered by mutating an Rc-shared topology.
    pub cow_copies: u64,
    /// Largest number of simultaneously active transaction scopes.
    pub max_depth: u64,
    /// Currently active scopes; zero when a synchronous witness is complete.
    pub active_depth: u64,
}

thread_local! {
    static COUNTS: Cell<Counts> = Cell::new(Counts::default());
}

/// Read the current thread's counters.
#[must_use]
pub fn snapshot() -> Counts {
    COUNTS.with(Cell::get)
}

/// Reset counters between witnesses.
///
/// # Errors
/// Refuses to reset within an active transaction, preserving scope bookkeeping.
pub fn reset() -> Result<(), &'static str> {
    COUNTS.with(|counts| {
        if counts.get().active_depth != 0 {
            return Err("cannot reset counters inside a transaction");
        }
        counts.set(Counts::default());
        Ok(())
    })
}

/// A transaction lifetime; thread-bound because the counters are thread-local.
#[must_use]
pub struct Scope(PhantomData<Rc<()>>);

impl Scope {
    /// Enter a scope, recording whether it takes a deep rollback snapshot.
    pub fn enter(deep_snapshot: bool) -> Self {
        COUNTS.with(|counts| {
            let mut value = counts.get();
            value.transactions = value.transactions.saturating_add(1);
            value.snapshots = value.snapshots.saturating_add(u64::from(deep_snapshot));
            value.active_depth = value.active_depth.saturating_add(1);
            value.max_depth = value.max_depth.max(value.active_depth);
            counts.set(value);
        });
        Self(PhantomData)
    }
}

impl Drop for Scope {
    fn drop(&mut self) {
        COUNTS.with(|counts| {
            let mut value = counts.get();
            value.active_depth = value.active_depth.saturating_sub(1);
            counts.set(value);
        });
    }
}

/// Record an actual `Rc::make_mut` topology copy separately from a snapshot.
pub fn record_cow_copy() {
    COUNTS.with(|counts| {
        let mut value = counts.get();
        value.cow_copies = value.cow_copies.saturating_add(1);
        counts.set(value);
    });
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use crate::{
        Topology, TopologyError,
        transaction::{run_transacted, run_validated},
    };

    #[test]
    fn nested_validation_failure_preserves_scope_accounting_and_rollback() {
        reset().unwrap();
        let mut topo = Topology::new();
        let mut retired = None;
        let result = run_transacted(&mut topo, |topo| {
            assert!(reset().is_err());
            run_validated(
                topo,
                |topo| {
                    let solid = topo.add_empty_solid();
                    retired = Some(solid);
                    Ok(solid)
                },
                |_, _| Err(TopologyError::WireNotClosed),
            )
        });
        assert!(matches!(result, Err(TopologyError::WireNotClosed)));
        assert!(topo.solid(retired.unwrap()).is_err());
        assert_eq!(topo.num_solids(), 0);
        let counts = snapshot();
        assert_eq!(
            (
                counts.transactions,
                counts.snapshots,
                counts.max_depth,
                counts.active_depth
            ),
            (2, 2, 2, 0)
        );
        assert_eq!(counts.cow_copies, 0);
        reset().unwrap();
        assert_eq!(snapshot().transactions, 0);
    }
}
