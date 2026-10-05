//! Content-keyed memo of fixed-order face integrals (PERF-V02 subset), with
//! readings that survive an edit (PERF-V02 / O07).
//!
//! [`FaceIntegralCache`] remembers what [`face_integrator::integrate_face_fixed_about`]
//! and [`face_integrator::integrate_face_area`] returned for a face, keyed by
//! the face's *content* rather than its handle: everything those integrators
//! read, plus the request (Gauss order and kind). The integrators are pure
//! functions of exactly that input, so a lookup that finds the same content
//! returns the value a fresh integration would compute, bit for bit.
//!
//! # Why content, not identity
//!
//! The expensive consumer is the strict validator's shell-orientation probe,
//! which integrates every face of a body at the default Gauss order to read
//! one sign per shell. Applications validate the same body several times
//! under different handles: a deserialized copy in a short-lived kernel, a
//! sibling result whose faces were copied unchanged, the result an operation
//! has just validated internally. Handle- or generation-keyed caches (see
//! [`crate::classify::ClassificationCache`]) cannot connect those; a content
//! key does, and needs no invalidation at all: a face whose geometry or
//! boundary changes in place simply stops matching its old entry.
//!
//! # Readings across an edit
//!
//! A direct edit moves a body's bounding box, and with it the reference point
//! the probe integrates about (B58: the shell's vertex-box centre), and it
//! translates the faces it moves rigidly. Neither changes what a face
//! encloses, only where it is measured from, and both are recovered from one
//! more quantity the fixed rule now reads off the same Gauss samples (or the
//! same closed form): the face's vector area `N = ∫ n dA = Σ wᵢ nᵢ`.
//!
//! * **Another reference.** The volume term `(1/3) Σ wᵢ (Pᵢ − R) · nᵢ` is
//!   affine in `R`, so `I(R') = I(R) − (1/3)(R' − R) · N`: the same sum
//!   regrouped. The key therefore no longer holds the reference; the entry
//!   records the one it was integrated about.
//! * **A rigid translation.** A face `f + δ` integrates over the same
//!   parameter domain and trim as `f` with every sample moved by `δ`, so
//!   `I(f + δ, R) = I(f, R − δ) = I(f, R) + (1/3) δ · N`, and its area is
//!   `f`'s. The key compares positions relative to an anchor (the first
//!   boundary vertex), so a translated face finds its source's entry, and `δ`
//!   is the difference of the two anchors.
//!
//! Only [`integrate_face_volume_about`](FaceIntegralCache::integrate_face_volume_about)
//! (the probe's area and signed volume) uses these readings. The full
//! contribution — first and second moments included — is served only from an
//! entry of identical content integrated about the very same reference, and
//! an area only from identical content, so both stay bit-identical to a fresh
//! integration.
//!
//! ## How close a reused reading is
//!
//! Re-referencing regroups a sum, so it differs from integrating afresh by
//! rounding alone: a few ulps of `Σ |wᵢ (Pᵢ − R) · nᵢ|`. A translated face is
//! a translation only up to rounding — `p + δ` is rounded, and re-normalising
//! a transformed circle's axes moves them by an ulp — so its content is
//! matched with a tolerance, not bitwise: positions relative to the anchor
//! within `64 ε` of the larger of the two faces' coordinate magnitudes, unit
//! directions within `64 ε`, radii and semi-axes within `64 ε` relative, and
//! every knot, weight, trim, tolerance, count, flag and incidence exactly. A
//! face matched that way reads its source's integral, which differs from
//! integrating the translated copy by that rounding and by however the
//! copy's ulp-moved boundary projects into the surface's parameter domain:
//! far below the trim chording every fixed-order reading already carries.
//! Such a reading is never returned where bit identity is promised.
//!
//! # Key
//!
//! The key holds:
//!
//! * the request family (full contribution and volume share one; area is
//!   another) and the Gauss order;
//! * the face's reversal flag and every field of its surface;
//! * per wire, outer first then inner in face order: the closed flag and the
//!   oriented edges in wire order, each with its direction, every field of
//!   the edge curve, the trim interval, the edge tolerance and both vertices
//!   (position and tolerance).
//!
//! Discrete data (kinds, counts, flags, degrees, incidences) are words
//! compared exactly; every real is kept as its `f64` together with how it
//! may move under a translation (not at all, as a position coordinate, as a
//! unit-direction component, as a length, or as a plane offset). Edge and
//! vertex handles are replaced by their first-occurrence ordinal within the
//! face, so the key records which uses share an entity (a seam walked twice,
//! a closed edge's single vertex) without depending on handle values. The
//! integrators read nothing else — no pcurves, loops, coedges, attributes or
//! neighbouring faces — and order nothing by handle value.
//!
//! Every field of every geometry type is read through its accessor; the
//! `key_reads_every_geometry_field` test pins each type's size so that a new
//! field fails a test until it joins the key.
//!
//! # Bounds and determinism
//!
//! The cache holds at most `capacity` entries and `byte_budget` estimated
//! retained key bytes; an entry larger than the whole budget is computed but
//! not retained. Eviction is deterministic FIFO. The hash quantises the
//! movable reals far more coarsely than the match tolerance, and lookups
//! compare whole keys, so a hash collision costs a comparison and a
//! quantisation boundary a miss, never a wrong value. Errors are never
//! cached.
//!
//! # The per-thread instance
//!
//! The functions ending in `_memoized` consult one cache per thread. It is
//! **disabled by default** (capacity zero), so library callers and benchmarks
//! keep measuring the uncached integrators; an application enables it with
//! [`enable_thread_face_cache`] or [`set_thread_face_cache_limits`] (the WASM
//! kernel does, on construction). While disabled the memoized functions do
//! not even build a key.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use smallvec::SmallVec;

use super::face_integrator::{self, FaceContribution};
use crate::CheckError;

/// Default bound on retained entries (faces) once a cache is enabled.
pub const DEFAULT_FACE_CACHE_CAPACITY: usize = 8192;

/// Default bound on estimated retained key bytes once a cache is enabled.
pub const DEFAULT_FACE_CACHE_BYTE_BUDGET: usize = 16 * 1024 * 1024;

/// Version word leading every key; bump it when the encoding changes.
const KEY_VERSION: u64 = 0x5245_4d55_5346_4332;

/// How far a movable real may differ between two keys that are the same
/// face up to a translation, in units of `f64::EPSILON` (scaled per slot, see
/// [`Key::matches`]). Building `p + δ`, re-anchoring it and re-normalising a
/// transformed axis each round by at most a few ulps.
const MATCH_ULPS: f64 = 64.0;

/// The hash quantises an anchored position to `2^-HASH_GRID_BITS` of the
/// face's own anchored extent — about `1e9` times the match tolerance for a
/// face near the origin — so two matching keys straddle a quantisation step
/// with probability ~1e-9 per coordinate. Straddling costs a miss.
const HASH_GRID_BITS: i32 = 16;

/// Unit directions and log-lengths are hashed on a `2^-20` grid.
const HASH_UNIT_SCALE: f64 = 1_048_576.0;

/// Fixed per-entry bookkeeping charged on top of the key.
const ENTRY_OVERHEAD_BYTES: usize = 96 + std::mem::size_of::<Value>();

/// Deterministic cache statistics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FaceCacheStats {
    /// Lookups answered from a retained entry.
    pub hits: u64,
    /// Of `hits`: volume readings re-expressed about another reference point
    /// from an entry of identical content.
    pub rereferenced: u64,
    /// Of `hits`: volume readings of a face that is a rigid translation of
    /// the entry's (to rounding; see the module docs).
    pub translated: u64,
    /// Lookups that integrated (including ones whose result was not retained).
    pub misses: u64,
    /// Results retained after a miss.
    pub insertions: u64,
    /// Entries dropped to honour `capacity` or `byte_budget`.
    pub evictions: u64,
    /// Entries currently retained.
    pub len: usize,
    /// Bound on retained entries; zero disables the cache.
    pub capacity: usize,
    /// Estimated retained key bytes (not RSS).
    pub retained_bytes: usize,
    /// Bound on `retained_bytes`; zero disables the cache.
    pub byte_budget: usize,
}

/// A face's area and its signed volume term `(1/3) ∫ (P − R) · n dA` about a
/// reference point `R`: what the strict orientation probe sums per shell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceVolumeTerm {
    /// Face area.
    pub area: f64,
    /// Signed volume term about the requested reference.
    pub volume: f64,
}

/// What an entry remembers about one integration.
#[derive(Debug, Clone)]
struct Value {
    contribution: FaceContribution,
    /// The reference the positional terms of `contribution` are taken about.
    reference: Point3,
    /// `∫ n dA` from the same samples, when the integration reported it.
    flux: Option<Vec3>,
}

#[derive(Debug, Clone)]
struct Entry {
    hash: u64,
    seq: u64,
    key: Key,
    value: Value,
    bytes: usize,
}

/// What is being integrated.
#[derive(Debug, Clone, Copy)]
enum Request {
    /// [`face_integrator::integrate_face_fixed_about`]: the full contribution,
    /// served only bit-identically.
    FixedAbout { order: usize, reference: Point3 },
    /// The area and volume term about `reference`, served from any entry of
    /// the same content or of a translation of it.
    Volume { order: usize, reference: Point3 },
    /// [`face_integrator::integrate_face_area`].
    Area { order: usize },
}

impl Request {
    /// Requests of one family share entries; the family and the order are
    /// part of the key, the reference is not.
    const fn family_and_order(self) -> (u64, usize) {
        match self {
            Self::FixedAbout { order, .. } | Self::Volume { order, .. } => (1, order),
            Self::Area { order } => (2, order),
        }
    }

    fn compute(self, topo: &Topology, face: FaceId) -> Result<Value, CheckError> {
        match self {
            Self::FixedAbout { order, reference } | Self::Volume { order, reference } => {
                let (contribution, flux) =
                    face_integrator::integrate_face_fixed_flux_about(topo, face, order, reference)?;
                Ok(Value {
                    contribution,
                    reference,
                    flux,
                })
            }
            Self::Area { order } => {
                let area = face_integrator::integrate_face_area(topo, face, order)?;
                Ok(Value {
                    contribution: area_only(area),
                    reference: Point3::new(0.0, 0.0, 0.0),
                    flux: None,
                })
            }
        }
    }

    /// The answer `value`, just integrated for this request, gives it.
    fn answer_fresh(self, value: &Value) -> Answer {
        match self {
            Self::FixedAbout { .. } => Answer::Contribution(value.contribution.clone()),
            Self::Volume { .. } => Answer::Volume(FaceVolumeTerm {
                area: value.contribution.area,
                volume: value.contribution.volume,
            }),
            Self::Area { .. } => Answer::Area(value.contribution.area),
        }
    }

    /// How (and how well) `entry` answers this request for a face whose key
    /// is `key`; the lower the rank, the better the reading.
    fn answer_from(self, key: &Key, entry: &Entry) -> Option<(Rank, Answer)> {
        let found = key.matches(&entry.key)?;
        let value = &entry.value;
        match self {
            Self::FixedAbout { reference, .. } => {
                (found == Match::Exact && same_point(value.reference, reference)).then(|| {
                    (
                        Rank::Identical,
                        Answer::Contribution(value.contribution.clone()),
                    )
                })
            }
            // `face_area` is a reported measurement: only identical content
            // answers it, so it stays bit-identical to a fresh reading.
            Self::Area { .. } => (found == Match::Exact)
                .then_some((Rank::Identical, Answer::Area(value.contribution.area))),
            Self::Volume { reference, .. } => {
                let area = value.contribution.area;
                if found == Match::Exact && same_point(value.reference, reference) {
                    return Some((
                        Rank::Identical,
                        Answer::Volume(FaceVolumeTerm {
                            area,
                            volume: value.contribution.volume,
                        }),
                    ));
                }
                let flux = value.flux?;
                // The face is the entry's translated by `δ` (zero for an
                // exact match), so about `reference` it reads what the
                // entry's face reads about `reference − δ`.
                let delta = key.anchor - entry.key.anchor;
                let shift = (reference - value.reference) - delta;
                let volume = value.contribution.volume - shift.dot(flux) / 3.0;
                let rank = if found == Match::Exact {
                    Rank::Rereferenced
                } else {
                    Rank::Translated
                };
                Some((rank, Answer::Volume(FaceVolumeTerm { area, volume })))
            }
        }
    }
}

/// How a retained reading answered, best first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Rank {
    Identical,
    Rereferenced,
    Translated,
}

#[derive(Debug, Clone)]
enum Answer {
    Contribution(FaceContribution),
    Volume(FaceVolumeTerm),
    Area(f64),
}

const fn area_only(area: f64) -> FaceContribution {
    FaceContribution {
        area,
        volume: 0.0,
        volume_moment_x: 0.0,
        volume_moment_y: 0.0,
        volume_moment_z: 0.0,
        volume_second_x: 0.0,
        volume_second_y: 0.0,
        volume_second_z: 0.0,
        volume_product_xy: 0.0,
        volume_product_xz: 0.0,
        volume_product_yz: 0.0,
        centroid_x: 0.0,
        centroid_y: 0.0,
        centroid_z: 0.0,
    }
}

fn same_point(a: Point3, b: Point3) -> bool {
    a.x().to_bits() == b.x().to_bits()
        && a.y().to_bits() == b.y().to_bits()
        && a.z().to_bits() == b.z().to_bits()
}

/// Bounded, content-keyed memo of fixed-order face integrals.
///
/// See the [module docs](self) for the key, the bounds, and which readings
/// are bit-identical to a fresh integration.
#[derive(Debug, Clone)]
pub struct FaceIntegralCache {
    capacity: usize,
    byte_budget: usize,
    entries: VecDeque<Entry>,
    /// Key hash to the sequence numbers of the entries carrying it.
    index: DetHashMap<u64, SmallVec<[u64; 1]>>,
    /// Sequence number of `entries.front()` when non-empty.
    front_seq: u64,
    next_seq: u64,
    retained_bytes: usize,
    hits: u64,
    rereferenced: u64,
    translated: u64,
    misses: u64,
    insertions: u64,
    evictions: u64,
}

impl Default for FaceIntegralCache {
    fn default() -> Self {
        Self::new()
    }
}

impl FaceIntegralCache {
    /// An empty cache with the default bounds.
    #[must_use]
    pub fn new() -> Self {
        Self::with_limits(DEFAULT_FACE_CACHE_CAPACITY, DEFAULT_FACE_CACHE_BYTE_BUDGET)
    }

    /// An empty cache that retains nothing: every call integrates.
    #[must_use]
    pub fn disabled() -> Self {
        Self::with_limits(0, 0)
    }

    /// An empty cache bounded by entry count and estimated key bytes.
    ///
    /// Either bound at zero disables retention.
    #[must_use]
    pub fn with_limits(capacity: usize, byte_budget: usize) -> Self {
        Self {
            capacity,
            byte_budget,
            entries: VecDeque::new(),
            index: DetHashMap::default(),
            front_seq: 0,
            next_seq: 0,
            retained_bytes: 0,
            hits: 0,
            rereferenced: 0,
            translated: 0,
            misses: 0,
            insertions: 0,
            evictions: 0,
        }
    }

    /// Whether results are retained at all.
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.capacity > 0 && self.byte_budget > 0
    }

    /// Change both bounds, evicting oldest entries until the cache fits.
    pub fn set_limits(&mut self, capacity: usize, byte_budget: usize) {
        self.capacity = capacity;
        self.byte_budget = byte_budget;
        while !self.entries.is_empty()
            && (self.entries.len() > capacity || self.retained_bytes > byte_budget)
        {
            self.evict_front();
        }
    }

    /// Drop every retained entry; bounds and counters are kept.
    pub fn clear(&mut self) {
        self.entries.clear();
        self.index.clear();
        self.retained_bytes = 0;
        self.front_seq = self.next_seq;
    }

    /// Deterministic statistics snapshot.
    #[must_use]
    pub fn stats(&self) -> FaceCacheStats {
        FaceCacheStats {
            hits: self.hits,
            rereferenced: self.rereferenced,
            translated: self.translated,
            misses: self.misses,
            insertions: self.insertions,
            evictions: self.evictions,
            len: self.entries.len(),
            capacity: self.capacity,
            retained_bytes: self.retained_bytes,
            byte_budget: self.byte_budget,
        }
    }

    /// [`face_integrator::integrate_face_fixed_about`], answered from the
    /// cache when this face content was integrated about the same reference
    /// at the same order before. Bit-identical to the uncached call.
    ///
    /// # Errors
    ///
    /// Exactly the errors of the uncached integrator.
    pub fn integrate_face_fixed_about(
        &mut self,
        topo: &Topology,
        face: FaceId,
        gauss_order: usize,
        reference: Point3,
    ) -> Result<FaceContribution, CheckError> {
        let request = Request::FixedAbout {
            order: gauss_order,
            reference,
        };
        match self.integrate(topo, face, request)? {
            Answer::Contribution(contribution) => Ok(contribution),
            Answer::Volume(_) | Answer::Area(_) => unreachable_answer(),
        }
    }

    /// The area and volume term of [`face_integrator::integrate_face_fixed_about`]
    /// about `reference`, answered from any retained integration of this
    /// face's content — about any reference — or of a rigid translation of it.
    ///
    /// Bit-identical to the uncached reading on a miss and when an entry of
    /// identical content was integrated about this very reference; otherwise
    /// equal to it up to rounding (see the [module docs](self)).
    ///
    /// # Errors
    ///
    /// Exactly the errors of the uncached integrator.
    pub fn integrate_face_volume_about(
        &mut self,
        topo: &Topology,
        face: FaceId,
        gauss_order: usize,
        reference: Point3,
    ) -> Result<FaceVolumeTerm, CheckError> {
        let request = Request::Volume {
            order: gauss_order,
            reference,
        };
        match self.integrate(topo, face, request)? {
            Answer::Volume(term) => Ok(term),
            Answer::Contribution(_) | Answer::Area(_) => unreachable_answer(),
        }
    }

    /// [`face_integrator::integrate_face_area`], answered from the cache when
    /// this face content was measured at the same order before.
    /// Bit-identical to the uncached call.
    ///
    /// # Errors
    ///
    /// Exactly the errors of the uncached integrator.
    pub fn integrate_face_area(
        &mut self,
        topo: &Topology,
        face: FaceId,
        gauss_order: usize,
    ) -> Result<f64, CheckError> {
        match self.integrate(topo, face, Request::Area { order: gauss_order })? {
            Answer::Area(area) => Ok(area),
            Answer::Contribution(_) | Answer::Volume(_) => unreachable_answer(),
        }
    }

    fn integrate(
        &mut self,
        topo: &Topology,
        face: FaceId,
        request: Request,
    ) -> Result<Answer, CheckError> {
        if !self.is_enabled() {
            return Ok(request.answer_fresh(&request.compute(topo, face)?));
        }
        let Ok((hash, key)) = face_key(topo, face, request) else {
            // The integrator meets the same missing entity and reports it.
            return Ok(request.answer_fresh(&request.compute(topo, face)?));
        };
        if let Some(answer) = self.lookup(hash, &key, request) {
            return Ok(answer);
        }
        let value = request.compute(topo, face)?;
        let answer = request.answer_fresh(&value);
        self.insert(hash, key, value);
        Ok(answer)
    }

    fn lookup(&mut self, hash: u64, key: &Key, request: Request) -> Option<Answer> {
        let mut best: Option<(Rank, Answer)> = None;
        if let Some(seqs) = self.index.get(&hash) {
            for &seq in seqs {
                let Some(entry) = seq
                    .checked_sub(self.front_seq)
                    .and_then(|offset| usize::try_from(offset).ok())
                    .and_then(|offset| self.entries.get(offset))
                else {
                    continue;
                };
                let Some((rank, answer)) = request.answer_from(key, entry) else {
                    continue;
                };
                if best.as_ref().is_none_or(|(held, _)| rank < *held) {
                    let identical = rank == Rank::Identical;
                    best = Some((rank, answer));
                    if identical {
                        break;
                    }
                }
            }
        }
        match &best {
            Some((rank, _)) => {
                self.hits += 1;
                match rank {
                    Rank::Identical => {}
                    Rank::Rereferenced => self.rereferenced += 1,
                    Rank::Translated => self.translated += 1,
                }
            }
            None => self.misses += 1,
        }
        best.map(|(_, answer)| answer)
    }

    fn insert(&mut self, hash: u64, key: Key, value: Value) {
        let bytes = key.retained_bytes() + ENTRY_OVERHEAD_BYTES;
        if !self.is_enabled() || bytes > self.byte_budget {
            return;
        }
        while !self.entries.is_empty()
            && (self.entries.len() >= self.capacity
                || self.retained_bytes + bytes > self.byte_budget)
        {
            self.evict_front();
        }
        if self.entries.is_empty() {
            self.front_seq = self.next_seq;
        }
        let seq = self.next_seq;
        self.next_seq += 1;
        self.index.entry(hash).or_default().push(seq);
        self.entries.push_back(Entry {
            hash,
            seq,
            key,
            value,
            bytes,
        });
        self.retained_bytes += bytes;
        self.insertions += 1;
    }

    fn evict_front(&mut self) {
        let Some(entry) = self.entries.pop_front() else {
            return;
        };
        if let Some(seqs) = self.index.get_mut(&entry.hash) {
            seqs.retain(|seq| *seq != entry.seq);
            if seqs.is_empty() {
                self.index.remove(&entry.hash);
            }
        }
        self.retained_bytes -= entry.bytes;
        self.front_seq = entry.seq + 1;
        self.evictions += 1;
    }
}

/// Each request kind is answered in its own kind; reaching this is a bug,
/// reported as an integration failure rather than a panic.
fn unreachable_answer<T>() -> Result<T, CheckError> {
    Err(CheckError::IntegrationFailed(
        "face cache answered a request in another kind".into(),
    ))
}

thread_local! {
    /// The per-thread cache behind the `_memoized` functions; disabled until
    /// an application enables it.
    static THREAD_CACHE: RefCell<FaceIntegralCache> = RefCell::new(FaceIntegralCache::disabled());
}

/// Set this thread's cache bounds. Zero for either disables it (the default);
/// shrinking evicts oldest entries first.
pub fn set_thread_face_cache_limits(capacity: usize, byte_budget: usize) {
    THREAD_CACHE.with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut() {
            cache.set_limits(capacity, byte_budget);
        }
    });
}

/// Enable this thread's cache at the default bounds unless it is already
/// enabled (then its bounds and contents are kept).
pub fn enable_thread_face_cache() {
    THREAD_CACHE.with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut()
            && !cache.is_enabled()
        {
            cache.set_limits(DEFAULT_FACE_CACHE_CAPACITY, DEFAULT_FACE_CACHE_BYTE_BUDGET);
        }
    });
}

/// Statistics of this thread's cache.
#[must_use]
pub fn thread_face_cache_stats() -> FaceCacheStats {
    THREAD_CACHE.with(|cache| cache.try_borrow().map(|c| c.stats()).unwrap_or_default())
}

/// Drop every entry of this thread's cache, keeping its bounds and counters.
pub fn clear_thread_face_cache() {
    THREAD_CACHE.with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut() {
            cache.clear();
        }
    });
}

/// [`face_integrator::integrate_face_fixed_about`] through this thread's
/// cache (a plain call while it is disabled). The value is bit-identical to
/// the uncached call either way.
///
/// # Errors
///
/// Exactly the errors of the uncached integrator.
pub fn integrate_face_fixed_about_memoized(
    topo: &Topology,
    face: FaceId,
    gauss_order: usize,
    reference: Point3,
) -> Result<FaceContribution, CheckError> {
    let request = Request::FixedAbout {
        order: gauss_order,
        reference,
    };
    match memoized(topo, face, request)? {
        Answer::Contribution(contribution) => Ok(contribution),
        Answer::Volume(_) | Answer::Area(_) => unreachable_answer(),
    }
}

/// [`FaceIntegralCache::integrate_face_volume_about`] through this thread's
/// cache (a plain fixed-order integration while it is disabled).
///
/// # Errors
///
/// Exactly the errors of the uncached integrator.
pub fn integrate_face_volume_about_memoized(
    topo: &Topology,
    face: FaceId,
    gauss_order: usize,
    reference: Point3,
) -> Result<FaceVolumeTerm, CheckError> {
    let request = Request::Volume {
        order: gauss_order,
        reference,
    };
    match memoized(topo, face, request)? {
        Answer::Volume(term) => Ok(term),
        Answer::Contribution(_) | Answer::Area(_) => unreachable_answer(),
    }
}

/// [`face_integrator::integrate_face_area`] through this thread's cache (a
/// plain call while it is disabled). The value is bit-identical to the
/// uncached call either way.
///
/// # Errors
///
/// Exactly the errors of the uncached integrator.
pub fn integrate_face_area_memoized(
    topo: &Topology,
    face: FaceId,
    gauss_order: usize,
) -> Result<f64, CheckError> {
    match memoized(topo, face, Request::Area { order: gauss_order })? {
        Answer::Area(area) => Ok(area),
        Answer::Contribution(_) | Answer::Volume(_) => unreachable_answer(),
    }
}

/// The thread cache is borrowed only around the lookup and the insert, never
/// across the integration, so a nested memoized call cannot meet a held
/// borrow; a borrow that is unavailable anyway degrades to a plain call.
fn memoized(topo: &Topology, face: FaceId, request: Request) -> Result<Answer, CheckError> {
    let enabled =
        THREAD_CACHE.with(|cache| cache.try_borrow().is_ok_and(|cache| cache.is_enabled()));
    if !enabled {
        return Ok(request.answer_fresh(&request.compute(topo, face)?));
    }
    let Ok((hash, key)) = face_key(topo, face, request) else {
        return Ok(request.answer_fresh(&request.compute(topo, face)?));
    };
    let hit = THREAD_CACHE.with(|cache| {
        cache
            .try_borrow_mut()
            .ok()
            .and_then(|mut cache| cache.lookup(hash, &key, request))
    });
    if let Some(answer) = hit {
        return Ok(answer);
    }
    let value = request.compute(topo, face)?;
    let answer = request.answer_fresh(&value);
    THREAD_CACHE.with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut() {
            cache.insert(hash, key, value);
        }
    });
    Ok(answer)
}

/// How a real in the key may move when the face is translated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum Slot {
    /// Unchanged by a translation: compared bitwise (knots, weights, trims,
    /// tolerances, the cone's half-angle).
    Exact,
    /// A position's x coordinate: compared relative to the anchor's.
    PosX,
    /// A position's y coordinate.
    PosY,
    /// A position's z coordinate.
    PosZ,
    /// A component of a unit direction.
    Unit,
    /// A length (radius, semi-axis, focal length).
    Length,
    /// A plane's offset `d` (`n · p = d`), written right after its unit
    /// normal: compared as `d − n · anchor`.
    PlaneOffset,
}

/// Whether two keys are the same face, or the same face up to a translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Match {
    Exact,
    Translated,
}

/// A face's content key: see the [module docs](self).
#[derive(Debug, Clone)]
struct Key {
    words: Box<[u64]>,
    reals: Box<[f64]>,
    slots: Box<[Slot]>,
    /// Position of the first boundary vertex (the origin for a face with
    /// none); positions are compared relative to it.
    anchor: Point3,
    /// Largest absolute coordinate (or plane offset) in the key: the scale
    /// rounding moves the positions on.
    magnitude: f64,
}

impl Key {
    const fn retained_bytes(&self) -> usize {
        self.words.len() * std::mem::size_of::<u64>()
            + self.reals.len() * std::mem::size_of::<f64>()
            + self.slots.len()
    }

    /// The translation-invariant value of real `i`.
    fn anchored(&self, i: usize) -> f64 {
        let x = self.reals[i];
        match self.slots[i] {
            Slot::PosX => x - self.anchor.x(),
            Slot::PosY => x - self.anchor.y(),
            Slot::PosZ => x - self.anchor.z(),
            Slot::PlaneOffset if i >= 3 => {
                let n = &self.reals[i - 3..i];
                x - n[0].mul_add(
                    self.anchor.x(),
                    n[1].mul_add(self.anchor.y(), n[2] * self.anchor.z()),
                )
            }
            Slot::Exact | Slot::Unit | Slot::Length | Slot::PlaneOffset => x,
        }
    }

    /// `Exact` when every word and real is identical, `Translated` when the
    /// words are and every real agrees within its slot's tolerance once
    /// positions are taken relative to each key's anchor.
    fn matches(&self, other: &Self) -> Option<Match> {
        if self.words != other.words || self.slots != other.slots {
            return None;
        }
        if self
            .reals
            .iter()
            .zip(other.reals.iter())
            .all(|(a, b)| a.to_bits() == b.to_bits())
        {
            return Some(Match::Exact);
        }
        let unit = MATCH_ULPS * f64::EPSILON;
        let position = unit * self.magnitude.max(other.magnitude);
        for (i, slot) in self.slots.iter().enumerate() {
            let (a, b) = (self.reals[i], other.reals[i]);
            let close = match slot {
                Slot::Exact => a.to_bits() == b.to_bits(),
                Slot::Unit => (a - b).abs() <= unit,
                Slot::Length => (a - b).abs() <= unit * a.abs().max(b.abs()),
                Slot::PosX | Slot::PosY | Slot::PosZ | Slot::PlaneOffset => {
                    (self.anchored(i) - other.anchored(i)).abs() <= position
                }
            };
            if !close {
                return None;
            }
        }
        Some(Match::Translated)
    }
}

/// Word and real sink for a face key.
struct KeyWriter {
    words: Vec<u64>,
    reals: Vec<f64>,
    slots: Vec<Slot>,
    anchor: Option<Point3>,
}

impl KeyWriter {
    const fn new() -> Self {
        Self {
            words: Vec::new(),
            reals: Vec::new(),
            slots: Vec::new(),
            anchor: None,
        }
    }

    fn word(&mut self, word: u64) {
        self.words.push(word);
    }

    fn count(&mut self, n: usize) {
        self.word(n as u64);
    }

    fn flag(&mut self, b: bool) {
        self.word(u64::from(b));
    }

    fn real(&mut self, x: f64, slot: Slot) {
        self.reals.push(x);
        self.slots.push(slot);
    }

    fn exact(&mut self, x: f64) {
        self.real(x, Slot::Exact);
    }

    fn length(&mut self, x: f64) {
        self.real(x, Slot::Length);
    }

    fn point(&mut self, p: Point3) {
        self.real(p.x(), Slot::PosX);
        self.real(p.y(), Slot::PosY);
        self.real(p.z(), Slot::PosZ);
    }

    fn unit(&mut self, v: Vec3) {
        self.real(v.x(), Slot::Unit);
        self.real(v.y(), Slot::Unit);
        self.real(v.z(), Slot::Unit);
    }

    fn exacts(&mut self, xs: &[f64]) {
        self.count(xs.len());
        for &x in xs {
            self.exact(x);
        }
    }

    fn vertex(&mut self, p: Point3, tolerance: f64) {
        self.anchor.get_or_insert(p);
        self.point(p);
        self.exact(tolerance);
    }

    fn surface(&mut self, surface: &FaceSurface) {
        match surface {
            FaceSurface::Plane { normal, d } => {
                self.word(1);
                self.unit(*normal);
                self.real(*d, Slot::PlaneOffset);
            }
            FaceSurface::Nurbs(s) => {
                self.word(2);
                self.count(s.degree_u());
                self.count(s.degree_v());
                self.exacts(s.knots_u());
                self.exacts(s.knots_v());
                self.count(s.control_points().len());
                for (row, weights) in s.control_points().iter().zip(s.weights()) {
                    self.count(row.len());
                    for (&p, &w) in row.iter().zip(weights) {
                        self.point(p);
                        self.exact(w);
                    }
                }
            }
            FaceSurface::Cylinder(s) => {
                self.word(3);
                self.point(s.origin());
                self.unit(s.axis());
                self.length(s.radius());
                self.unit(s.x_axis());
                self.unit(s.y_axis());
            }
            FaceSurface::Cone(s) => {
                self.word(4);
                self.point(s.apex());
                self.unit(s.axis());
                self.exact(s.half_angle());
                self.unit(s.x_axis());
                self.unit(s.y_axis());
            }
            FaceSurface::Sphere(s) => {
                self.word(5);
                self.point(s.center());
                self.length(s.radius());
                self.unit(s.x_axis());
                self.unit(s.y_axis());
                self.unit(s.z_axis());
            }
            FaceSurface::Torus(s) => {
                self.word(6);
                self.point(s.center());
                self.length(s.major_radius());
                self.length(s.minor_radius());
                self.unit(s.x_axis());
                self.unit(s.y_axis());
                self.unit(s.z_axis());
            }
        }
    }

    fn curve(&mut self, curve: &EdgeCurve) {
        match curve {
            EdgeCurve::Line => self.word(1),
            EdgeCurve::NurbsCurve(c) => {
                self.word(2);
                self.count(c.degree());
                self.exacts(c.knots());
                self.count(c.control_points().len());
                for (&p, &w) in c.control_points().iter().zip(c.weights()) {
                    self.point(p);
                    self.exact(w);
                }
            }
            EdgeCurve::Circle(c) => {
                self.word(3);
                self.point(c.center());
                self.unit(c.normal());
                self.length(c.radius());
                self.unit(c.u_axis());
                self.unit(c.v_axis());
            }
            EdgeCurve::Ellipse(c) => {
                self.word(4);
                self.point(c.center());
                self.unit(c.normal());
                self.length(c.semi_major());
                self.length(c.semi_minor());
                self.unit(c.u_axis());
                self.unit(c.v_axis());
            }
            EdgeCurve::Hyperbola(c) => {
                self.word(5);
                self.point(c.center());
                self.unit(c.normal());
                self.length(c.semi_major());
                self.length(c.semi_minor());
                self.unit(c.u_axis());
                self.unit(c.v_axis());
            }
            EdgeCurve::Parabola(c) => {
                self.word(6);
                self.point(c.vertex());
                self.unit(c.axis_dir());
                self.length(c.focal_length());
                self.unit(c.u_axis());
            }
        }
    }

    /// The finished key and its hash. The hash covers the words exactly and
    /// every real through the quantity [`Key::matches`] compares, quantised
    /// far more coarsely than the match tolerance.
    fn finish(self) -> (u64, Key) {
        let mut key = Key {
            words: self.words.into_boxed_slice(),
            reals: self.reals.into_boxed_slice(),
            slots: self.slots.into_boxed_slice(),
            anchor: self.anchor.unwrap_or(Point3::new(0.0, 0.0, 0.0)),
            magnitude: 0.0,
        };
        let mut extent = 0.0_f64;
        for (i, slot) in key.slots.iter().enumerate() {
            if matches!(
                slot,
                Slot::PosX | Slot::PosY | Slot::PosZ | Slot::PlaneOffset
            ) {
                key.magnitude = key.magnitude.max(key.reals[i].abs());
                extent = extent.max(key.anchored(i).abs());
            }
        }
        let grid = if extent.is_finite() && extent > 0.0 {
            (extent.log2().floor() - f64::from(HASH_GRID_BITS)).exp2()
        } else {
            1.0
        };
        let mut hash = 0_u64;
        let mut mix = |word: u64| {
            hash = (hash.rotate_left(5) ^ word).wrapping_mul(0x517c_c1b7_2722_0a95);
        };
        for &word in &key.words {
            mix(word);
        }
        for (i, slot) in key.slots.iter().enumerate() {
            let x = key.reals[i];
            // Saturating float-to-integer casts: only the hash sees these.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let word = match slot {
                Slot::Exact => x.to_bits(),
                Slot::Unit => (x * HASH_UNIT_SCALE).round() as i64 as u64,
                Slot::Length => {
                    ((x.abs().log2() * HASH_UNIT_SCALE).round() as i64 as u64)
                        ^ u64::from(x.is_sign_negative())
                }
                Slot::PosX | Slot::PosY | Slot::PosZ | Slot::PlaneOffset => {
                    (key.anchored(i) / grid).round() as i64 as u64
                }
            };
            mix(word);
        }
        (hash, key)
    }
}

/// First-occurrence ordinal of `index` in `seen`, and whether this is that
/// first occurrence.
fn ordinal(seen: &mut DetHashMap<usize, usize>, index: usize) -> (usize, bool) {
    let next = seen.len();
    match seen.entry(index) {
        std::collections::hash_map::Entry::Occupied(slot) => (*slot.get(), false),
        std::collections::hash_map::Entry::Vacant(slot) => {
            slot.insert(next);
            (next, true)
        }
    }
}

/// The content key of `face` for `request`, with its hash.
fn face_key(topo: &Topology, face_id: FaceId, request: Request) -> Result<(u64, Key), CheckError> {
    let mut w = KeyWriter::new();
    w.word(KEY_VERSION);
    let (family, order) = request.family_and_order();
    w.word(family);
    w.count(order);
    let face = topo.face(face_id)?;
    w.flag(face.is_reversed());
    w.surface(face.surface());
    w.count(1 + face.inner_wires().len());
    let mut edges = DetHashMap::default();
    let mut vertices = DetHashMap::default();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        w.flag(wire.is_closed());
        w.count(wire.edges().len());
        for oriented in wire.edges() {
            w.flag(oriented.is_forward());
            let (edge_ordinal, first) = ordinal(&mut edges, oriented.edge().index());
            w.count(edge_ordinal);
            if !first {
                continue;
            }
            let edge = topo.edge(oriented.edge())?;
            w.curve(edge.curve());
            match edge.trim() {
                Some((t0, t1)) => {
                    w.flag(true);
                    w.exact(t0);
                    w.exact(t1);
                }
                None => w.flag(false),
            }
            match edge.tolerance() {
                Some(tolerance) => {
                    w.flag(true);
                    w.exact(tolerance);
                }
                None => w.flag(false),
            }
            for vertex_id in [edge.start(), edge.end()] {
                let (vertex_ordinal, first) = ordinal(&mut vertices, vertex_id.index());
                w.count(vertex_ordinal);
                if first {
                    let vertex = topo.vertex(vertex_id)?;
                    w.vertex(vertex.point(), vertex.tolerance());
                }
            }
        }
    }
    Ok(w.finish())
}

#[cfg(test)]
mod tests;
