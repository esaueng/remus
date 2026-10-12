//! Opt-in per-thread memo of revolved-face wire areas.
//!
//! Shell assembly orients every cylinder and cone face's wires by the sign of
//! their area in the surface chart: 33 samples per edge, each projected onto
//! the surface. A plate drilled one hole at a time carries every earlier hole
//! wall into each later cut unchanged, and assembly re-measures all of them
//! (the frozen-untouched-faces stage of `docs/performance/perf-local-boolean.md`).
//!
//! The area is a pure function of the surface and of the wire's edges in
//! traversal order: curve, endpoints, domain and direction. This memo keys on
//! the exact bits of all of them, so a hit is bit-identical to recomputing; a
//! face whose content is unchanged since the last cut costs a key and a
//! lookup. Failures are never stored.
//!
//! It is off unless a thread enables it (`enable_thread_wire_area_memo`),
//! like the other content-keyed memos the WASM kernel turns on module-wide,
//! and bounded: past its capacity the oldest entries go first.

use std::cell::RefCell;

use crate::memo::{BoundedMemo, MemoStats};

/// Default number of wire areas a thread keeps once enabled: a few times the
/// walls of a plate carrying a thousand holes, at well under a kilobyte each.
pub const DEFAULT_WIRE_AREA_MEMO_CAPACITY: usize = 8192;

/// Exact bits of a surface and a wire's edges.
pub(crate) type WireAreaKey = Vec<u64>;

thread_local! {
    static MEMO: RefCell<BoundedMemo<WireAreaKey, Option<f64>>> =
        RefCell::new(BoundedMemo::default());
}

/// Statistics of this thread's memo.
pub type WireAreaMemoStats = MemoStats;

/// Enable this thread's memo at [`DEFAULT_WIRE_AREA_MEMO_CAPACITY`] unless it
/// is already enabled (then its capacity and contents are kept).
pub fn enable_thread_wire_area_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.enable(DEFAULT_WIRE_AREA_MEMO_CAPACITY);
        }
    });
}

/// Set how many areas this thread's memo keeps; zero disables it and drops
/// its contents. Shrinking drops the oldest first.
pub fn set_thread_wire_area_memo_capacity(capacity: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.set_capacity(capacity);
        }
    });
}

/// This thread's memo statistics.
#[must_use]
pub fn thread_wire_area_memo_stats() -> WireAreaMemoStats {
    MEMO.with(|memo| {
        memo.try_borrow()
            .map(|memo| memo.stats())
            .unwrap_or_default()
    })
}

/// Whether the memo is on, so a caller can skip building a key.
pub(crate) fn enabled() -> bool {
    MEMO.with(|memo| {
        memo.try_borrow()
            .is_ok_and(|memo| memo.stats().capacity > 0)
    })
}

/// The area for `key`: the stored one, or `compute`'s result, stored when the
/// memo is on. `compute` runs outside the memo's borrow.
pub(crate) fn memoized<E>(
    key: &WireAreaKey,
    compute: impl FnOnce() -> Result<Option<f64>, E>,
) -> Result<Option<f64>, E> {
    if let Some(area) = MEMO.with(|memo| memo.try_borrow_mut().ok()?.get(key)) {
        return Ok(area);
    }
    let area = compute()?;
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.insert(key, &area);
        }
    });
    Ok(area)
}
