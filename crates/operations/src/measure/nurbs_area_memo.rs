//! Exact scalar memo of standalone NURBS-face area readings.
//!
//! [`super::face_area`] tessellates a NURBS face and sums triangle areas. That
//! numerical path differs from trimmed quadrature and from whole-solid display
//! meshing, so neither can supply an interchangeable area. This memo stores
//! only values measured by that existing path, preserving their bits.
//!
//! # Content and request policy
//!
//! Standalone NURBS tessellation reads the complete support surface rather
//! than outer-wire trim geometry. The key records both degrees, all knots,
//! control points and weights (including row lengths), reversal, deflection,
//! default angular tolerance and the disabled curvature-floor policy. Reals
//! compare by bits, so even signed zeros remain distinct. A hash only filters
//! candidates; the complete key must match. Handles and topology identity are
//! irrelevant: clones and unchanged faces carried into another result may
//! reuse a reading, while changed content misses without invalidation hooks.
//!
//! Faces with inner wires bypass the memo before lookup, preserving their
//! existing standalone-tessellation refusal. A future standalone trim policy
//! must extend the key to every newly-read topology field. There is no reuse
//! under translations or approximate comparisons, and errors are never stored.
//!
//! # Bounds and lifetime
//!
//! The memo is disabled by default. Applications opt in per thread, as they do
//! for face-integral and volume memos. Entries are evicted in deterministic FIFO
//! order under both a count and an estimated retained-byte limit. Keys exceeding
//! the entire byte limit are rejected before allocation; key allocation failure
//! also falls back to the original calculation. Estimated bytes include key
//! words and each entry record, not allocator or container slack and not RSS.
//! No cache borrow is held across the calculation, and an unavailable cache
//! borrow simply takes the original path.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::hash::Hasher;

use remus_math::det_hash::DetHasher;
use remus_math::nurbs::surface::NurbsSurface;

/// Default retained standalone NURBS-area readings per thread once enabled.
pub const DEFAULT_NURBS_AREA_MEMO_CAPACITY: usize = 512;

/// Default estimated retained key and entry bytes per thread once enabled.
pub const DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET: usize = 8 * 1024 * 1024;

/// Bump when the key's content or request encoding changes.
const KEY_VERSION: u64 = 0x5245_4d55_534e_4131;

/// Deterministic counters and current bounds of the thread's NURBS-area memo.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NurbsAreaMemoStats {
    /// Requests answered by an exact-content entry.
    pub hits: u64,
    /// Requests calculated through an available enabled memo, including unretained keys.
    pub misses: u64,
    /// Entries dropped to satisfy the count or byte bound.
    pub evictions: u64,
    /// Readings currently retained.
    pub len: usize,
    /// Maximum retained readings; zero disables lookups and retention.
    pub capacity: usize,
    /// Estimated retained key and entry bytes, excluding allocator slack.
    pub retained_bytes: usize,
    /// Maximum estimated retained bytes; zero disables lookups and retention.
    pub byte_budget: usize,
}

struct Entry {
    hash: u64,
    key: Box<[u64]>,
    value: f64,
}

impl Entry {
    fn bytes(&self) -> usize {
        self.key.len() * std::mem::size_of::<u64>() + std::mem::size_of::<Self>()
    }
}

struct Memo {
    entries: VecDeque<Entry>,
    hits: u64,
    misses: u64,
    evictions: u64,
    retained_bytes: usize,
    max_entries: usize,
    max_bytes: usize,
}

impl Memo {
    fn new(capacity: usize, byte_budget: usize) -> Self {
        Self {
            entries: VecDeque::new(),
            hits: 0,
            misses: 0,
            evictions: 0,
            retained_bytes: 0,
            max_entries: capacity,
            max_bytes: byte_budget,
        }
    }

    const fn enabled(&self) -> bool {
        self.max_entries > 0 && self.max_bytes > 0
    }

    fn evict_front(&mut self) {
        if let Some(entry) = self.entries.pop_front() {
            self.retained_bytes -= entry.bytes();
            self.evictions = self.evictions.saturating_add(1);
        }
    }

    fn enforce_limits(&mut self) {
        while !self.entries.is_empty()
            && (self.entries.len() > self.max_entries || self.retained_bytes > self.max_bytes)
        {
            self.evict_front();
        }
    }

    fn stats(&self) -> NurbsAreaMemoStats {
        NurbsAreaMemoStats {
            hits: self.hits,
            misses: self.misses,
            evictions: self.evictions,
            len: self.entries.len(),
            capacity: self.max_entries,
            retained_bytes: self.retained_bytes,
            byte_budget: self.max_bytes,
        }
    }
}

thread_local! {
    static MEMO: RefCell<Option<Memo>> = const { RefCell::new(None) };
}

/// Initialize this thread's memo with default bounds when no instance exists.
///
/// Existing bounds, entries and counters are preserved, including a zero bound
/// configured to disable retention. Constructing another kernel therefore does
/// not override an application's chosen resource limits.
pub fn enable_thread_nurbs_area_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            slot.get_or_insert_with(|| {
                Memo::new(
                    DEFAULT_NURBS_AREA_MEMO_CAPACITY,
                    DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET,
                )
            });
        }
    });
}

/// Set this thread's memo bounds, evicting oldest entries immediately to fit.
///
/// A zero bound disables lookups and retention without discarding counters.
pub fn set_thread_nurbs_area_memo_limits(capacity: usize, byte_budget: usize) {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            let memo = slot.get_or_insert_with(|| Memo::new(capacity, byte_budget));
            memo.max_entries = capacity;
            memo.max_bytes = byte_budget;
            memo.enforce_limits();
        }
    });
}

/// Disable this thread's memo and release its entries and counters.
pub fn disable_thread_nurbs_area_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            *slot = None;
        }
    });
}

/// Drop this thread's retained readings, keeping its bounds and counters.
pub fn clear_thread_nurbs_area_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut()
            && let Some(memo) = slot.as_mut()
        {
            memo.entries.clear();
            memo.retained_bytes = 0;
        }
    });
}

/// Return counters and bounds, or `None` without an available memo instance.
#[must_use]
pub fn thread_nurbs_area_memo_stats() -> Option<NurbsAreaMemoStats> {
    MEMO.with(|cell| {
        cell.try_borrow()
            .ok()
            .and_then(|slot| slot.as_ref().map(Memo::stats))
    })
}

fn key_words(surface: &NurbsSurface) -> Option<usize> {
    // Eleven header words, plus the complete knot/control-point/weight
    // payload and each row length. Count before allocating or visiting values.
    let words = 11_usize
        .checked_add(surface.knots_u().len())?
        .checked_add(surface.knots_v().len())?;
    let words = surface.control_points().iter().try_fold(words, |n, row| {
        n.checked_add(1)?.checked_add(row.len().checked_mul(3)?)
    })?;
    surface
        .weights()
        .iter()
        .try_fold(words, |n, row| n.checked_add(1)?.checked_add(row.len()))
}

fn content_key(
    surface: &NurbsSurface,
    deflection: f64,
    reversed: bool,
    byte_budget: usize,
) -> Option<Box<[u64]>> {
    let words = key_words(surface)?;
    let bytes = words
        .checked_mul(std::mem::size_of::<u64>())?
        .checked_add(std::mem::size_of::<Entry>())?;
    if bytes > byte_budget {
        return None;
    }
    let mut key = Vec::new();
    key.try_reserve_exact(words).ok()?;
    key.extend([
        KEY_VERSION,
        surface.degree_u() as u64,
        surface.degree_v() as u64,
        u64::from(reversed),
        deflection.to_bits(),
        remus_math::chord::DEFAULT_ANGULAR_TOL.to_bits(),
        0, // Curvature floor disabled in standalone tessellation.
        surface.knots_u().len() as u64,
    ]);
    key.extend(surface.knots_u().iter().map(|x| x.to_bits()));
    key.push(surface.knots_v().len() as u64);
    key.extend(surface.knots_v().iter().map(|x| x.to_bits()));
    key.push(surface.control_points().len() as u64);
    for row in surface.control_points() {
        key.push(row.len() as u64);
        for p in row {
            key.extend([p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]);
        }
    }
    key.push(surface.weights().len() as u64);
    for row in surface.weights() {
        key.push(row.len() as u64);
        key.extend(row.iter().map(|x| x.to_bits()));
    }
    debug_assert_eq!(key.len(), words);
    Some(key.into_boxed_slice())
}

fn hash_key(key: &[u64]) -> u64 {
    let mut hash = DetHasher::new();
    for word in key {
        hash.write(&word.to_le_bytes());
    }
    hash.finish()
}

pub(super) fn memoized(
    surface: &NurbsSurface,
    deflection: f64,
    reversed: bool,
    compute: impl FnOnce() -> Result<f64, crate::OperationsError>,
) -> Result<f64, crate::OperationsError> {
    let budget = MEMO.with(|cell| {
        cell.try_borrow().ok().and_then(|slot| {
            slot.as_ref()
                .filter(|memo| memo.enabled())
                .map(|memo| memo.max_bytes)
        })
    });
    let Some(budget) = budget else {
        return compute();
    };
    let Some(key) = content_key(surface, deflection, reversed, budget) else {
        MEMO.with(|cell| {
            if let Ok(mut slot) = cell.try_borrow_mut()
                && let Some(memo) = slot.as_mut()
            {
                memo.misses = memo.misses.saturating_add(1);
            }
        });
        return compute();
    };
    let hash = hash_key(&key);
    let hit = MEMO.with(|cell| {
        let mut slot = cell.try_borrow_mut().ok()?;
        let memo = slot.as_mut()?;
        let hit = memo
            .entries
            .iter()
            .find(|entry| entry.hash == hash && entry.key == key)
            .map(|entry| entry.value);
        if hit.is_some() {
            memo.hits = memo.hits.saturating_add(1);
        } else {
            memo.misses = memo.misses.saturating_add(1);
        }
        hit
    });
    if let Some(value) = hit {
        return Ok(value);
    }

    // Never hold a cache borrow across the calculation and never retain an
    // error. Every retained value is the established tessellation-area sum.
    let value = compute()?;
    let entry = Entry { hash, key, value };
    let bytes = entry.bytes();
    MEMO.with(|cell| {
        let Ok(mut slot) = cell.try_borrow_mut() else {
            return;
        };
        let Some(memo) = slot.as_mut() else { return };
        if !memo.enabled() || bytes > memo.max_bytes {
            return;
        }
        while !memo.entries.is_empty()
            && (memo.entries.len() >= memo.max_entries
                || bytes > memo.max_bytes.saturating_sub(memo.retained_bytes))
        {
            memo.evict_front();
        }
        memo.entries.push_back(entry);
        memo.retained_bytes += bytes;
    });
    Ok(value)
}

#[cfg(test)]
mod tests;
