//! Opt-in per-thread memo of the ray classifier's per-face geometry.
//!
//! Classifying a boolean's sub-faces collects ray-cast geometry for every face
//! of the opposing solid. For a torus face that is not a four-arc rectangle
//! that is a polygon of its trim in the torus chart, every trim edge sampled
//! adaptively to about 3e-8 rad: on an imported blend bordered by NURBS curves,
//! thousands of curve evaluations and torus projections per face. Every
//! boolean against the body rebuilds them, though a face the previous edit
//! never touched is a bit-identical copy.
//!
//! The polygon is a pure function of the torus, of the outer wire's edges in
//! traversal order (direction, curve, endpoints, domain, tolerance) and of the
//! classifier tolerance. This memo keys on the exact bits of all of them and
//! returns a clone of what was computed, so every hit is bit-identical to
//! recomputing; a face the trim builder declines is stored as declined. It
//! holds torus trims only.
//!
//! It is off unless a thread enables it (`enable_thread_ray_geom_memo`), like
//! the other content-keyed memos the WASM kernel turns on module-wide. It is
//! bounded by entry count and by bytes (a trim can hold 16,384 points), and
//! past either bound the oldest entries go first.

use std::cell::RefCell;

use crate::classifier::TorusTrim;
use crate::memo::BoundedMemo;

/// Default number of faces a thread keeps once enabled.
pub const DEFAULT_RAY_GEOM_MEMO_CAPACITY: usize = 1024;

/// Default byte budget of a thread's memo once enabled: keys and trim
/// polygons together. The 160-face hammer holder's four sampled trims take
/// about 255 KB.
pub const DEFAULT_RAY_GEOM_MEMO_BYTES: usize = 8 << 20;

/// Exact bits of everything a face's ray geometry depends on.
pub(crate) type RayGeomKey = Vec<u64>;

thread_local! {
    static MEMO: RefCell<BoundedMemo<RayGeomKey, Option<TorusTrim>>> =
        RefCell::new(BoundedMemo::with_budget(DEFAULT_RAY_GEOM_MEMO_BYTES));
}

/// Statistics of this thread's memo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RayGeomMemoStats {
    /// Lookups answered from the memo.
    pub hits: u64,
    /// Lookups the enabled memo could not answer (it computed and stored).
    pub misses: u64,
    /// Faces held.
    pub len: usize,
    /// Most faces held; zero when the memo is off.
    pub capacity: usize,
    /// Bytes held, keys and polygons.
    pub bytes: usize,
}

/// Enable this thread's memo at [`DEFAULT_RAY_GEOM_MEMO_CAPACITY`] faces
/// unless it is already enabled (then its capacity and contents are kept).
/// The byte budget starts at [`DEFAULT_RAY_GEOM_MEMO_BYTES`].
pub fn enable_thread_ray_geom_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.enable(DEFAULT_RAY_GEOM_MEMO_CAPACITY);
        }
    });
}

/// Set how many faces this thread's memo keeps; zero disables it and drops
/// its contents. Shrinking drops the oldest first.
pub fn set_thread_ray_geom_memo_capacity(capacity: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.set_capacity(capacity);
        }
    });
}

/// Set how many bytes this thread's memo keeps. Shrinking drops the oldest
/// first.
pub fn set_thread_ray_geom_memo_bytes(bytes: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.set_budget(bytes);
        }
    });
}

/// This thread's memo statistics.
#[must_use]
pub fn thread_ray_geom_memo_stats() -> RayGeomMemoStats {
    MEMO.with(|memo| {
        memo.try_borrow().map_or(
            RayGeomMemoStats {
                hits: 0,
                misses: 0,
                len: 0,
                capacity: 0,
                bytes: 0,
            },
            |memo| {
                let stats = memo.stats();
                RayGeomMemoStats {
                    hits: stats.hits,
                    misses: stats.misses,
                    len: stats.len,
                    capacity: stats.capacity,
                    bytes: memo.weight(),
                }
            },
        )
    })
}

/// Whether the memo is on, so a caller can skip building a key.
pub(crate) fn enabled() -> bool {
    MEMO.with(|memo| {
        memo.try_borrow()
            .is_ok_and(|memo| memo.stats().capacity > 0)
    })
}

/// The torus trim for `key`: a clone of the stored one, or `compute`'s
/// result, stored when the memo is on. `compute` runs outside the memo's
/// borrow.
pub(crate) fn memoized(
    key: &RayGeomKey,
    compute: impl FnOnce() -> Option<TorusTrim>,
) -> Option<TorusTrim> {
    if let Some(trim) = MEMO.with(|memo| memo.try_borrow_mut().ok()?.get(key)) {
        return trim;
    }
    let trim = compute();
    // Both copies of the key the store holds (see `insert_weighted`).
    let bytes = 2 * std::mem::size_of_val(key.as_slice())
        + trim.as_ref().map_or(0, TorusTrim::heap_bytes)
        + std::mem::size_of::<(RayGeomKey, Option<TorusTrim>)>()
        + std::mem::size_of::<(RayGeomKey, usize)>();
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.insert_weighted(key, &trim, bytes);
        }
    });
    trim
}
