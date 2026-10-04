//! Content-keyed memo of fixed-order face integrals (PERF-V02 subset).
//!
//! [`FaceIntegralCache`] remembers what [`face_integrator::integrate_face_fixed_about`]
//! and [`face_integrator::integrate_face_area`] returned for a face, keyed by
//! the face's *content* rather than its handle: everything those integrators
//! read, encoded bit for bit, plus the request (Gauss order and, for the
//! positional integrals, the reference point). A hit therefore returns the
//! very value a fresh integration would compute — the integrators are pure
//! functions of exactly that input — so a cached reading never differs from an
//! uncached one, not even in the last bit.
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
//! # Key
//!
//! The key holds, as raw `f64`/`u64` bit patterns:
//!
//! * the request kind, Gauss order and (positional integrals) reference point;
//! * the face's reversal flag and every field of its surface;
//! * per wire, outer first then inner in face order: the closed flag and the
//!   oriented edges in wire order, each with its direction, every field of
//!   the edge curve, the trim interval, the edge tolerance and both vertices
//!   (position and tolerance).
//!
//! Edge and vertex handles are replaced by their first-occurrence ordinal
//! within the face, so the key records which uses share an entity (a seam
//! walked twice, a closed edge's single vertex) without depending on handle
//! values. The integrators read nothing else — no pcurves, loops, coedges,
//! attributes or neighbouring faces — and order nothing by handle value.
//!
//! Every field of every geometry type is read through its accessor; the
//! `key_reads_every_geometry_field` test pins each type's size so that a new
//! field fails a test until it joins the key.
//!
//! # Bounds and determinism
//!
//! The cache holds at most `capacity` entries and `byte_budget` estimated
//! retained key bytes; an entry larger than the whole budget is computed but
//! not retained. Eviction is deterministic FIFO. Lookups compare whole keys,
//! so a hash collision costs a miss, never a wrong value. Errors are never
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
const KEY_VERSION: u64 = 0x5245_4d55_5346_4331;

/// Fixed per-entry bookkeeping charged on top of the key words.
const ENTRY_OVERHEAD_BYTES: usize = 64 + std::mem::size_of::<FaceContribution>();

/// Deterministic cache statistics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FaceCacheStats {
    /// Lookups answered from a retained entry.
    pub hits: u64,
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

#[derive(Debug, Clone)]
struct Entry {
    hash: u64,
    seq: u64,
    key: Box<[u64]>,
    value: FaceContribution,
    bytes: usize,
}

/// What is being integrated; part of the key.
#[derive(Debug, Clone, Copy)]
enum Request {
    /// [`face_integrator::integrate_face_fixed_about`].
    FixedAbout { order: usize, reference: Point3 },
    /// [`face_integrator::integrate_face_area`].
    Area { order: usize },
}

impl Request {
    fn integrate(self, topo: &Topology, face: FaceId) -> Result<FaceContribution, CheckError> {
        match self {
            Self::FixedAbout { order, reference } => {
                face_integrator::integrate_face_fixed_about(topo, face, order, reference)
            }
            Self::Area { order } => {
                let area = face_integrator::integrate_face_area(topo, face, order)?;
                Ok(FaceContribution {
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
                })
            }
        }
    }
}

/// Bounded, content-keyed memo of fixed-order face integrals.
///
/// See the [module docs](self) for the key, the bounds and why a hit is
/// bit-identical to a fresh integration.
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
    /// at the same order before.
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
        self.integrate(
            topo,
            face,
            Request::FixedAbout {
                order: gauss_order,
                reference,
            },
        )
    }

    /// [`face_integrator::integrate_face_area`], answered from the cache when
    /// this face content was measured at the same order before.
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
        Ok(self
            .integrate(topo, face, Request::Area { order: gauss_order })?
            .area)
    }

    fn integrate(
        &mut self,
        topo: &Topology,
        face: FaceId,
        request: Request,
    ) -> Result<FaceContribution, CheckError> {
        if !self.is_enabled() {
            return request.integrate(topo, face);
        }
        let Ok((hash, key)) = face_key(topo, face, request) else {
            // The integrator meets the same missing entity and reports it.
            return request.integrate(topo, face);
        };
        if let Some(value) = self.lookup(hash, &key) {
            return Ok(value);
        }
        let value = request.integrate(topo, face)?;
        self.insert(hash, key, value.clone());
        Ok(value)
    }

    fn lookup(&mut self, hash: u64, key: &[u64]) -> Option<FaceContribution> {
        let found = self.index.get(&hash).and_then(|seqs| {
            seqs.iter().find_map(|&seq| {
                let offset = usize::try_from(seq.checked_sub(self.front_seq)?).ok()?;
                let entry = self.entries.get(offset)?;
                (*entry.key == *key).then(|| entry.value.clone())
            })
        });
        if found.is_some() {
            self.hits += 1;
        } else {
            self.misses += 1;
        }
        found
    }

    fn insert(&mut self, hash: u64, key: Vec<u64>, value: FaceContribution) {
        let bytes = key.len() * std::mem::size_of::<u64>() + ENTRY_OVERHEAD_BYTES;
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
            key: key.into_boxed_slice(),
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
    memoized(
        topo,
        face,
        Request::FixedAbout {
            order: gauss_order,
            reference,
        },
    )
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
    Ok(memoized(topo, face, Request::Area { order: gauss_order })?.area)
}

/// The thread cache is borrowed only around the lookup and the insert, never
/// across the integration, so a nested memoized call cannot meet a held
/// borrow; a borrow that is unavailable anyway degrades to a plain call.
fn memoized(
    topo: &Topology,
    face: FaceId,
    request: Request,
) -> Result<FaceContribution, CheckError> {
    let enabled =
        THREAD_CACHE.with(|cache| cache.try_borrow().is_ok_and(|cache| cache.is_enabled()));
    if !enabled {
        return request.integrate(topo, face);
    }
    let Ok((hash, key)) = face_key(topo, face, request) else {
        return request.integrate(topo, face);
    };
    let hit = THREAD_CACHE.with(|cache| {
        cache
            .try_borrow_mut()
            .ok()
            .and_then(|mut cache| cache.lookup(hash, &key))
    });
    if let Some(value) = hit {
        return Ok(value);
    }
    let value = request.integrate(topo, face)?;
    THREAD_CACHE.with(|cache| {
        if let Ok(mut cache) = cache.try_borrow_mut() {
            cache.insert(hash, key, value.clone());
        }
    });
    Ok(value)
}

/// Word sink that hashes as it records (`FxHash` mixing; a collision only
/// costs a miss because lookups compare whole keys).
struct KeyWriter {
    words: Vec<u64>,
    hash: u64,
}

impl KeyWriter {
    const fn new() -> Self {
        Self {
            words: Vec::new(),
            hash: 0,
        }
    }

    fn word(&mut self, word: u64) {
        self.words.push(word);
        self.hash = (self.hash.rotate_left(5) ^ word).wrapping_mul(0x517c_c1b7_2722_0a95);
    }

    fn count(&mut self, n: usize) {
        self.word(n as u64);
    }

    fn flag(&mut self, b: bool) {
        self.word(u64::from(b));
    }

    fn real(&mut self, x: f64) {
        self.word(x.to_bits());
    }

    fn point(&mut self, p: Point3) {
        self.real(p.x());
        self.real(p.y());
        self.real(p.z());
    }

    fn vector(&mut self, v: Vec3) {
        self.real(v.x());
        self.real(v.y());
        self.real(v.z());
    }

    fn reals(&mut self, xs: &[f64]) {
        self.count(xs.len());
        for &x in xs {
            self.real(x);
        }
    }

    fn surface(&mut self, surface: &FaceSurface) {
        match surface {
            FaceSurface::Plane { normal, d } => {
                self.word(1);
                self.vector(*normal);
                self.real(*d);
            }
            FaceSurface::Nurbs(s) => {
                self.word(2);
                self.count(s.degree_u());
                self.count(s.degree_v());
                self.reals(s.knots_u());
                self.reals(s.knots_v());
                self.count(s.control_points().len());
                for (row, weights) in s.control_points().iter().zip(s.weights()) {
                    self.count(row.len());
                    for (&p, &w) in row.iter().zip(weights) {
                        self.point(p);
                        self.real(w);
                    }
                }
            }
            FaceSurface::Cylinder(s) => {
                self.word(3);
                self.point(s.origin());
                self.vector(s.axis());
                self.real(s.radius());
                self.vector(s.x_axis());
                self.vector(s.y_axis());
            }
            FaceSurface::Cone(s) => {
                self.word(4);
                self.point(s.apex());
                self.vector(s.axis());
                self.real(s.half_angle());
                self.vector(s.x_axis());
                self.vector(s.y_axis());
            }
            FaceSurface::Sphere(s) => {
                self.word(5);
                self.point(s.center());
                self.real(s.radius());
                self.vector(s.x_axis());
                self.vector(s.y_axis());
                self.vector(s.z_axis());
            }
            FaceSurface::Torus(s) => {
                self.word(6);
                self.point(s.center());
                self.real(s.major_radius());
                self.real(s.minor_radius());
                self.vector(s.x_axis());
                self.vector(s.y_axis());
                self.vector(s.z_axis());
            }
        }
    }

    fn curve(&mut self, curve: &EdgeCurve) {
        match curve {
            EdgeCurve::Line => self.word(1),
            EdgeCurve::NurbsCurve(c) => {
                self.word(2);
                self.count(c.degree());
                self.reals(c.knots());
                self.count(c.control_points().len());
                for (&p, &w) in c.control_points().iter().zip(c.weights()) {
                    self.point(p);
                    self.real(w);
                }
            }
            EdgeCurve::Circle(c) => {
                self.word(3);
                self.point(c.center());
                self.vector(c.normal());
                self.real(c.radius());
                self.vector(c.u_axis());
                self.vector(c.v_axis());
            }
            EdgeCurve::Ellipse(c) => {
                self.word(4);
                self.point(c.center());
                self.vector(c.normal());
                self.real(c.semi_major());
                self.real(c.semi_minor());
                self.vector(c.u_axis());
                self.vector(c.v_axis());
            }
            EdgeCurve::Hyperbola(c) => {
                self.word(5);
                self.point(c.center());
                self.vector(c.normal());
                self.real(c.semi_major());
                self.real(c.semi_minor());
                self.vector(c.u_axis());
                self.vector(c.v_axis());
            }
            EdgeCurve::Parabola(c) => {
                self.word(6);
                self.point(c.vertex());
                self.vector(c.axis_dir());
                self.real(c.focal_length());
                self.vector(c.u_axis());
            }
        }
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
fn face_key(
    topo: &Topology,
    face_id: FaceId,
    request: Request,
) -> Result<(u64, Vec<u64>), CheckError> {
    let mut w = KeyWriter::new();
    w.word(KEY_VERSION);
    match request {
        Request::FixedAbout { order, reference } => {
            w.word(1);
            w.count(order);
            w.point(reference);
        }
        Request::Area { order } => {
            w.word(2);
            w.count(order);
        }
    }
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
                    w.real(t0);
                    w.real(t1);
                }
                None => w.flag(false),
            }
            match edge.tolerance() {
                Some(tolerance) => {
                    w.flag(true);
                    w.real(tolerance);
                }
                None => w.flag(false),
            }
            for vertex_id in [edge.start(), edge.end()] {
                let (vertex_ordinal, first) = ordinal(&mut vertices, vertex_id.index());
                w.count(vertex_ordinal);
                if first {
                    let vertex = topo.vertex(vertex_id)?;
                    w.point(vertex.point());
                    w.real(vertex.tolerance());
                }
            }
        }
    }
    Ok((w.hash, w.words))
}

#[cfg(test)]
mod tests;
