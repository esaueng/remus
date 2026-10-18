//! Exact scalar measurement reuse across repeated edits and rebuilt arenas.
//!
//! Every value comes from the original measurement path. Full input words,
//! including tolerances, oriented trims and relative entity order, decide a
//! content hit. There is no translation approximation, algebraically seeded
//! volume, coarser measurement, cached error or skipped geometry validation.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_topology::CacheIdentity;
use remus_topology::Topology;
use remus_topology::face::FaceId;

use super::volume_memo::{ContentKey, face_content_key};

/// Maximum retained face-area readings in the new reusable memo.
pub const DEFAULT_MEASUREMENT_REUSE_ENTRIES: usize = 512;
/// Estimated face-area key, scalar and entry bytes; separate from old memos.
pub const DEFAULT_MEASUREMENT_REUSE_BYTES: usize = 8 * 1024 * 1024;

/// Counts and storage for reusable measurement diagnostics (not process RSS).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct MeasurementReuseStats {
    /// Whether reusable measurement lookup is enabled on this thread.
    pub enabled: bool,
    /// Face-area requests made while enabled.
    pub area_lookups: u64,
    /// Area reads answered by unchanged topology identity and request.
    pub area_identity_hits: u64,
    /// Area reads answered by exact geometry and request content.
    pub area_content_hits: u64,
    /// Area reads calculated through the original measurement path.
    pub area_misses: u64,
    /// Face-area entries evicted for the configured bounds.
    pub area_evictions: u64,
    /// Face-area entries currently retained.
    pub area_entries: usize,
    /// Estimated retained face-area bytes, including keys and bookkeeping.
    pub area_retained_bytes: usize,
    /// Bound on retained area readings.
    pub area_max_entries: usize,
    /// Bound on estimated area storage.
    pub area_max_bytes: usize,
    /// Volume requests made while reusable matching is enabled.
    pub volume_lookups: u64,
    /// Volume reads answered by unchanged identity.
    pub volume_identity_hits: u64,
    /// Volume reads answered by full canonical content.
    pub volume_content_hits: u64,
}

struct AreaEntry {
    key: ContentKey,
    identity: Option<CacheIdentity>,
    face: usize,
    deflection: u64,
    value: f64,
}

impl AreaEntry {
    fn bytes(&self) -> usize {
        self.key.bytes() + size_of::<Self>() + 256
    }
}

struct State {
    // None is the native opt-in default. The browser constructor only sets a
    // default once, preserving an explicitly disabled diagnostic control.
    enabled: Option<bool>,
    entries: VecDeque<AreaEntry>,
    bytes: usize,
    max_entries: usize,
    max_bytes: usize,
    counters: MeasurementReuseStats,
}

impl Default for State {
    fn default() -> Self {
        Self {
            enabled: None,
            entries: VecDeque::new(),
            bytes: 0,
            max_entries: DEFAULT_MEASUREMENT_REUSE_ENTRIES,
            max_bytes: DEFAULT_MEASUREMENT_REUSE_BYTES,
            counters: MeasurementReuseStats::default(),
        }
    }
}

impl State {
    fn evict_front(&mut self) {
        if let Some(entry) = self.entries.pop_front() {
            self.bytes -= entry.bytes();
            self.counters.area_evictions += 1;
        }
    }

    fn trim(&mut self) {
        while !self.entries.is_empty()
            && (self.entries.len() > self.max_entries || self.bytes > self.max_bytes)
        {
            self.evict_front();
        }
    }

    fn hit(&mut self, index: usize, identity: Option<CacheIdentity>, face: usize) -> Option<f64> {
        let mut entry = self.entries.remove(index)?;
        entry.identity = identity;
        entry.face = face;
        let value = entry.value;
        self.entries.push_back(entry);
        Some(value)
    }
}

thread_local! {
    static STATE: RefCell<State> = RefCell::new(State::default());
}

/// Enable exact reusable measurements on this thread without changing old
/// face-integral, NURBS-area or volume memo bounds.
pub fn enable_measurement_reuse() {
    super::enable_thread_volume_memo();
    set_measurement_reuse_enabled(true);
}

/// Set an explicit reusable-measurement preference. Disabling releases the
/// added area entries; existing measurement memo bounds remain unchanged.
pub fn set_measurement_reuse_enabled(enabled: bool) {
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut() {
            state.enabled = Some(enabled);
            if !enabled {
                state.entries.clear();
                state.bytes = 0;
            }
        }
    });
}

/// Initialize the browser's default once, preserving explicit disable/limits.
pub fn enable_default_measurement_reuse() {
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut() {
            state.enabled.get_or_insert(true);
        }
    });
}

/// Change only the new face-area storage limits; zero retains no area entries.
pub fn set_measurement_reuse_limits(max_entries: usize, max_bytes: usize) {
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut() {
            state.max_entries = max_entries;
            state.max_bytes = max_bytes;
            state.trim();
        }
    });
}

/// Release the new area entries, preserving configuration and counters.
pub fn clear_measurement_reuse() {
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut() {
            state.entries.clear();
            state.bytes = 0;
        }
    });
}

/// Snapshot counters and the independent new face-area storage bound.
#[must_use]
pub fn measurement_reuse_stats() -> MeasurementReuseStats {
    STATE.with(|slot| {
        slot.try_borrow().map_or_else(
            |_| MeasurementReuseStats::default(),
            |state| MeasurementReuseStats {
                enabled: state.enabled == Some(true),
                area_entries: state.entries.len(),
                area_retained_bytes: state.bytes,
                area_max_entries: state.max_entries,
                area_max_bytes: state.max_bytes,
                ..state.counters
            },
        )
    })
}

pub(super) fn is_enabled() -> bool {
    STATE.with(|slot| slot.try_borrow().is_ok_and(|s| s.enabled == Some(true)))
}

pub(super) fn record_volume_lookup() {
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut()
            && state.enabled == Some(true)
        {
            state.counters.volume_lookups += 1;
        }
    });
}

pub(super) fn record_volume_hit(content: bool) {
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut()
            && state.enabled == Some(true)
        {
            if content {
                state.counters.volume_content_hits += 1;
            } else {
                state.counters.volume_identity_hits += 1;
            }
        }
    });
}

pub(super) fn face_area(
    topo: &Topology,
    face: FaceId,
    deflection: f64,
    compute: impl FnOnce() -> Result<f64, crate::OperationsError>,
) -> Result<f64, crate::OperationsError> {
    if !is_enabled() {
        return compute();
    }
    let identity = (!topo.is_cache_poisoned()).then(|| topo.cache_identity());
    let request = deflection.to_bits();
    let first = STATE.with(|slot| {
        let mut state = slot.try_borrow_mut().ok()?;
        state.counters.area_lookups += 1;
        let index = state.entries.iter().position(|entry| {
            identity.is_some()
                && entry.identity == identity
                && entry.face == face.index()
                && entry.deflection == request
        })?;
        state.counters.area_identity_hits += 1;
        state.hit(index, identity, face.index())
    });
    if let Some(value) = first {
        return Ok(value);
    }
    let budget = STATE.with(|slot| {
        slot.try_borrow().ok().and_then(|state| {
            (state.max_entries > 0)
                .then_some(state.max_bytes.saturating_sub(size_of::<AreaEntry>() + 256))
        })
    });
    let key = budget
        .filter(|&bytes| bytes > 0)
        .and_then(|bytes| face_content_key(topo, face, deflection, bytes));
    let hit = key.as_ref().and_then(|key| {
        STATE.with(|slot| {
            let mut state = slot.try_borrow_mut().ok()?;
            let index = state.entries.iter().position(|entry| entry.key == *key)?;
            state.counters.area_content_hits += 1;
            state.hit(index, identity, face.index())
        })
    });
    if let Some(value) = hit {
        return Ok(value);
    }
    STATE.with(|slot| {
        if let Ok(mut state) = slot.try_borrow_mut() {
            state.counters.area_misses += 1;
        }
    });
    // No borrow crosses the original algorithm; errors are never retained.
    let value = compute()?;
    if let Some(key) = key {
        let entry = AreaEntry {
            key,
            identity,
            face: face.index(),
            deflection: request,
            value,
        };
        STATE.with(|slot| {
            let Ok(mut state) = slot.try_borrow_mut() else {
                return;
            };
            let bytes = entry.bytes();
            if state.enabled != Some(true) || state.max_entries == 0 || bytes > state.max_bytes {
                return;
            }
            while !state.entries.is_empty()
                && (state.entries.len() >= state.max_entries
                    || bytes > state.max_bytes.saturating_sub(state.bytes))
            {
                state.evict_front();
            }
            state.bytes += bytes;
            state.entries.push_back(entry);
        });
    }
    Ok(value)
}

#[cfg(test)]
mod tests;
