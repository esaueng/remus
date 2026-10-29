//! Opt-in, bounded timings for edit-to-analysis profiling.
//!
//! A host supplies a monotonic millisecond clock (WASM has no `Instant`).
//! Tracing is disabled by default and never changes modeling policy.

use std::cell::RefCell;

/// Observed execution routes of the plain boolean and N-way wrappers.
/// Counts are thread-local, feature-only, and include unsuccessful attempts.
#[cfg(feature = "perf-counters")]
#[derive(Debug, Default, Clone, Copy, serde::Serialize)]
pub struct BooleanRoutes {
    /// Calls into the two-operand general GFA after wrapper shortcuts.
    pub general_gfa_calls: u64,
    /// Enabled store-local attempts, including eligibility declines.
    pub store_local_attempts: u64,
    /// Store-local results that passed the strict result gate.
    pub store_local_successes: u64,
    /// Successful exact through/blind-hole construction shortcuts.
    pub local_cut_successes: u64,
    /// Calls into N-way GFA, including attempts that later fall back.
    pub nary_calls: u64,
}

#[derive(Clone, Copy)]
pub(crate) enum BooleanRoute {
    GeneralGfa,
    StoreLocalAttempt,
    StoreLocalSuccess,
    LocalCutSuccess,
    Nary,
}

#[cfg(feature = "perf-counters")]
thread_local! {
    static BOOLEAN_ROUTES: RefCell<BooleanRoutes> = RefCell::new(BooleanRoutes::default());
}

#[inline]
pub(crate) fn bump_boolean_route(route: BooleanRoute) {
    #[cfg(feature = "perf-counters")]
    BOOLEAN_ROUTES.with(|routes| {
        let mut routes = routes.borrow_mut();
        let count = match route {
            BooleanRoute::GeneralGfa => &mut routes.general_gfa_calls,
            BooleanRoute::StoreLocalAttempt => &mut routes.store_local_attempts,
            BooleanRoute::StoreLocalSuccess => &mut routes.store_local_successes,
            BooleanRoute::LocalCutSuccess => &mut routes.local_cut_successes,
            BooleanRoute::Nary => &mut routes.nary_calls,
        };
        *count = count.saturating_add(1);
    });
    #[cfg(not(feature = "perf-counters"))]
    let _ = route;
}

/// Drain route counts. Call before and after the operation to exclude setup.
#[cfg(feature = "perf-counters")]
#[must_use]
pub fn take_boolean_routes() -> BooleanRoutes {
    BOOLEAN_ROUTES.with(|routes| std::mem::take(&mut *routes.borrow_mut()))
}

/// One completed phase, in the host clock's time domain.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Phase {
    /// Stable phase label; never document content.
    pub name: &'static str,
    /// Host milliseconds at phase entry.
    pub start_ms: f64,
    /// Elapsed milliseconds.
    pub duration_ms: f64,
}

#[derive(Default)]
struct Trace {
    clock: Option<fn() -> f64>,
    phases: Vec<Phase>,
}

thread_local! {
    static TRACE: RefCell<Trace> = RefCell::new(Trace::default());
}

/// Enable a host clock, or disable tracing and release retained events.
pub fn configure(clock: Option<fn() -> f64>) {
    TRACE.with(|trace| {
        *trace.borrow_mut() = Trace {
            clock,
            phases: Vec::new(),
        }
    });
}

/// Drain completed phases. At most 4096 events are retained between drains.
#[must_use]
pub fn drain() -> Vec<Phase> {
    TRACE.with(|trace| std::mem::take(&mut trace.borrow_mut().phases))
}

/// A synchronous phase guard. Drop it before yielding or starting unrelated work.
pub struct Span {
    name: &'static str,
    start: Option<(fn() -> f64, f64)>,
}

/// Start a phase when tracing is enabled.
#[must_use]
// Keep diagnostic bookkeeping out of every modeling call site.
#[inline(never)]
pub fn span(name: &'static str) -> Span {
    let clock = TRACE.with(|trace| trace.borrow().clock);
    Span {
        name,
        start: clock.map(|clock| (clock, clock())),
    }
}

/// Time one synchronous computation, including typed error returns.
pub fn timed<T>(name: &'static str, run: impl FnOnce() -> T) -> T {
    let _span = span(name);
    run()
}

impl Drop for Span {
    #[inline(never)]
    fn drop(&mut self) {
        if let Some((clock, start_ms)) = self.start {
            let duration_ms = (clock() - start_ms).max(0.0);
            TRACE.with(|trace| {
                let mut trace = trace.borrow_mut();
                if trace.clock.is_some() && trace.phases.len() < 4096 {
                    trace.phases.push(Phase {
                        name: self.name,
                        start_ms,
                        duration_ms,
                    });
                }
            });
        }
    }
}

#[cfg(all(test, feature = "perf-counters"))]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::mat::Mat4;
    use remus_topology::Topology;

    #[test]
    fn scaling_phase_counters_distinguish_rejected_and_surviving_pairs() {
        for (offset, overlap) in [(3.0, false), (0.5, true)] {
            let mut topo = Topology::default();
            let a = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            let b = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            crate::transform::transform_solid(&mut topo, b, &Mat4::translation(offset, 0.0, 0.0))
                .unwrap();
            remus_algo::perf::configure_phase_clock(None);
            remus_algo::perf::reset();
            remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, a, b).unwrap();
            let counts = remus_algo::perf::snapshot();
            assert_eq!(counts.ee_pairs_visited, 144);
            assert_eq!(counts.ff_pairs_visited, 36);
            assert_eq!(counts.ee_pairs_surviving > 0, overlap);
            assert_eq!(counts.ff_pairs_surviving > 0, overlap);
            let phases = remus_algo::perf::take_phases();
            for required in [
                "gfa.store_import",
                "pave.vv",
                "pave.ve",
                "pave.ee",
                "pave.vf",
                "pave.ef",
                "pave.ff",
                "pave.coplanar",
                "pave.resolve",
                "gfa.build",
                "gfa.assemble",
                "gfa.store_export",
            ] {
                assert!(phases.iter().any(|p| p.name == required && p.calls == 1));
            }
        }
    }
}
