//! Opt-in, bounded timings for edit-to-analysis profiling.
//!
//! A host supplies a monotonic millisecond clock (WASM has no `Instant`).
//! Tracing is disabled by default and never changes modeling policy.

use std::cell::RefCell;

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
