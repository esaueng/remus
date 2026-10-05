//! Per-thread memo of [`super::solid_volume`] readings.
//!
//! A whole-body volume can cost a full tessellation (bodies whose faces the
//! exact routes decline are measured on their closed mesh), and applications
//! read the same body's volume repeatedly. The memo returns a previous
//! reading of the same solid at the same deflection while the topology is
//! provably unchanged since that reading.
//!
//! "Provably unchanged" is the PERF-Q02 cache identity
//! ([`remus_topology::CacheIdentity`]): a lineage that is fresh for every
//! [`Topology`] value and clone, plus a generation that every allocation,
//! exclusive access (`*_mut`), replacement, retirement, registry write and
//! restore/rollback bumps forward and never rewinds. An entry is reused only
//! when lineage, generation, solid index and deflection bits all match, so
//! every in-place edit — `transform_solid`, `face_mut`/`vertex_mut`/
//! `edge_mut`, healing, `RollbackSnapshot::restore`, a checkpoint restore,
//! deleting the solid — makes the old reading unreachable. `solid_volume` is
//! a deterministic function of exactly that state, so a hit is the value a
//! fresh call would return, bit for bit. Errors are never memoized, and a
//! poisoned identity (generation overflow) bypasses the memo.
//!
//! The cost of that soundness is reach: any mutation of the topology,
//! including allocating an unrelated solid, retires every reading taken
//! before it. The memo serves repeated reads between edits, not reads across
//! them.
//!
//! The memo is **disabled by default** (capacity zero), so library callers
//! and benchmarks keep measuring the uncached computation; an application
//! enables it with [`enable_thread_volume_memo`] (the WASM kernel does, on
//! construction).

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_topology::solid::SolidId;
use remus_topology::{CacheIdentity, Topology};

/// Default number of readings retained once the memo is enabled.
pub const DEFAULT_VOLUME_MEMO_CAPACITY: usize = 64;

/// Deterministic memo statistics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VolumeMemoStats {
    /// Readings answered from the memo.
    pub hits: u64,
    /// Readings that ran the computation while the memo was enabled.
    pub misses: u64,
    /// Readings currently retained.
    pub len: usize,
    /// Bound on retained readings; zero disables the memo.
    pub capacity: usize,
}

#[derive(Debug, Clone, Copy)]
struct Entry {
    identity: CacheIdentity,
    solid: usize,
    deflection: u64,
    volume: f64,
}

#[derive(Debug, Default)]
struct VolumeMemo {
    capacity: usize,
    entries: VecDeque<Entry>,
    hits: u64,
    misses: u64,
}

impl VolumeMemo {
    fn lookup(&mut self, identity: CacheIdentity, solid: usize, deflection: u64) -> Option<f64> {
        let found = self
            .entries
            .iter()
            .find(|e| e.identity == identity && e.solid == solid && e.deflection == deflection)
            .map(|e| e.volume);
        if found.is_some() {
            self.hits += 1;
        } else {
            self.misses += 1;
        }
        found
    }

    fn insert(&mut self, entry: Entry) {
        if self.capacity == 0 {
            return;
        }
        // Readings of this lineage taken at an older generation can never
        // match again: the generation only moves forward.
        self.entries.retain(|e| {
            e.identity.lineage != entry.identity.lineage
                || e.identity.generation >= entry.identity.generation
        });
        while self.entries.len() >= self.capacity {
            self.entries.pop_front();
        }
        self.entries.push_back(entry);
    }

    fn set_capacity(&mut self, capacity: usize) {
        self.capacity = capacity;
        while self.entries.len() > capacity {
            self.entries.pop_front();
        }
    }
}

thread_local! {
    static MEMO: RefCell<VolumeMemo> = RefCell::new(VolumeMemo::default());
}

/// Set how many readings this thread's memo keeps; zero (the default)
/// disables it. Shrinking drops the oldest readings first.
pub fn set_thread_volume_memo_capacity(capacity: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.set_capacity(capacity);
        }
    });
}

/// Enable this thread's memo at [`DEFAULT_VOLUME_MEMO_CAPACITY`] unless it is
/// already enabled (then its capacity and contents are kept).
pub fn enable_thread_volume_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut()
            && memo.capacity == 0
        {
            memo.set_capacity(DEFAULT_VOLUME_MEMO_CAPACITY);
        }
    });
}

/// Statistics of this thread's memo.
#[must_use]
pub fn thread_volume_memo_stats() -> VolumeMemoStats {
    MEMO.with(|memo| {
        memo.try_borrow()
            .map(|memo| VolumeMemoStats {
                hits: memo.hits,
                misses: memo.misses,
                len: memo.entries.len(),
                capacity: memo.capacity,
            })
            .unwrap_or_default()
    })
}

/// Drop every reading this thread's memo holds; capacity and counters stay.
pub fn clear_thread_volume_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.entries.clear();
        }
    });
}

/// `compute()` through this thread's memo. The memo is borrowed only around
/// the lookup and the insert, never across the computation, so a nested
/// reading cannot meet a held borrow.
pub(super) fn memoized(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    compute: impl FnOnce() -> Result<f64, crate::OperationsError>,
) -> Result<f64, crate::OperationsError> {
    let enabled = MEMO.with(|memo| memo.try_borrow().is_ok_and(|memo| memo.capacity > 0));
    if !enabled || topo.is_cache_poisoned() {
        return compute();
    }
    let identity = topo.cache_identity();
    let (solid, deflection_bits) = (solid.index(), deflection.to_bits());
    let hit = MEMO.with(|memo| {
        memo.try_borrow_mut()
            .ok()
            .and_then(|mut memo| memo.lookup(identity, solid, deflection_bits))
    });
    if let Some(volume) = hit {
        return Ok(volume);
    }
    let volume = compute()?;
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.insert(Entry {
                identity,
                solid,
                deflection: deflection_bits,
                volume,
            });
        }
    });
    Ok(volume)
}
