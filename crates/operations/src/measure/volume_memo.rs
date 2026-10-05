//! Per-thread memo of [`super::solid_volume`] readings.
//!
//! A whole-body volume can cost a full tessellation (bodies whose faces the
//! exact routes decline are measured on their closed mesh), and applications
//! read the same body's volume repeatedly — including after operations that
//! never touched it. The memo returns a previous reading of a solid when
//! nothing `solid_volume` reads about it has changed since.
//!
//! # Keys
//!
//! A reading is found two ways:
//!
//! * **Identity.** The PERF-Q02 cache identity ([`remus_topology::CacheIdentity`]):
//!   a lineage that is fresh for every [`Topology`] value and clone, plus a
//!   generation that every allocation, exclusive access (`*_mut`),
//!   replacement, retirement, registry write and restore/rollback bumps
//!   forward and never rewinds. Same identity and solid means the same
//!   topology state, so this is the cheap first test.
//! * **Content.** Everything `solid_volume` reads about the solid, recorded
//!   bit for bit together with every handle on the way: the solid and its
//!   shells (outer first, body class, face order), each face (handle,
//!   reversal, every surface field, wires in order), each wire (handle,
//!   closed flag, body class, oriented edges), each edge once (handle, every
//!   curve field, trim, tolerance, end vertices), each vertex once (handle,
//!   position, tolerance), and the pcurve of every oriented edge use the
//!   tessellator may consult. The volume routes read nothing else of the
//!   topology — no other solid, no attributes, journal or naming registries,
//!   and no whole-arena index (the face adjacency they build is local to the
//!   solid) — so equal content means an equal reading even after the
//!   identity moved on: another solid allocated, an unrelated edit, a
//!   checkpoint restore that brought the same state back, or a copy-on-write
//!   clone. Handles are part of the key, so any reliance on handle order is
//!   reproduced too. Whole keys are compared; the hash only finds candidates.
//!
//! The deflection is matched as `solid_volume` uses it: the request is
//! clamped to `volume_tessellation_deflection` (the solid's bounding-box
//! diagonal × 5e-5) before any mesh is taken, and every route before the
//! clamp is deflection-independent, so two requests that clamp to the same
//! value are the same reading. An entry records the request it was read at
//! and the clamped value; a lookup tries the request first and computes the
//! clamp (one bounding box) only when that misses.
//!
//! Every measured reading is the value a fresh call would return, bit for
//! bit. Errors are never memoized, and a poisoned identity (generation
//! overflow) bypasses the identity test.
//!
//! # Seeded readings
//!
//! An operation that already knows its result's volume may record it
//! ([`seed_solid_volume`]). Readings it took with `solid_volume` itself are
//! memoized like any other; a value it derived instead — the rigid blend move
//! adds the exact swept prism to its source's reading — is marked as seeded,
//! is never used to seed another value, and is replaced by nothing but a
//! measurement. Each seeding call site states how its value relates to a
//! fresh reading.
//!
//! # Bounds
//!
//! The memo is **disabled by default** (capacity zero), so library callers
//! and benchmarks keep measuring the uncached computation; an application
//! enables it with [`enable_thread_volume_memo`] (the WASM kernel does, on
//! construction). It retains at most `capacity` readings and `byte_budget`
//! bytes of content keys, evicting oldest first; a reading whose key alone
//! exceeds the budget keeps only its identity.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_math::curves2d::Curve2D;
use remus_math::det_hash::DetHashSet;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::{BodyClass, CacheIdentity, Topology};

/// Default number of readings retained once the memo is enabled.
pub const DEFAULT_VOLUME_MEMO_CAPACITY: usize = 64;

/// Default bound on retained content-key bytes once the memo is enabled.
pub const DEFAULT_VOLUME_MEMO_BYTE_BUDGET: usize = 16 * 1024 * 1024;

/// Version word leading every content key; bump it when the encoding changes.
const KEY_VERSION: u64 = 0x5245_4d55_5356_4d31;

/// Identity recorded for a reading taken on a poisoned topology: lineage zero
/// is never allocated, so no state ever matches it and the reading is found
/// by content only.
const NO_IDENTITY: CacheIdentity = CacheIdentity {
    lineage: 0,
    generation: 0,
};

/// Deterministic memo statistics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VolumeMemoStats {
    /// Readings answered from the memo.
    pub hits: u64,
    /// Of `hits`: found by content after the topology identity had moved on.
    pub content_hits: u64,
    /// Of `hits`: answered by a seeded reading.
    pub seeded_hits: u64,
    /// Readings that ran the computation while the memo was enabled.
    pub misses: u64,
    /// Seeded readings recorded.
    pub seeded: u64,
    /// Readings currently retained.
    pub len: usize,
    /// Bound on retained readings; zero disables the memo.
    pub capacity: usize,
    /// Retained content-key bytes.
    pub retained_bytes: usize,
    /// Bound on `retained_bytes`.
    pub byte_budget: usize,
}

/// A solid's content key and its hash.
#[derive(Debug, Clone, PartialEq, Eq)]
struct ContentKey {
    hash: u64,
    words: Box<[u64]>,
}

impl ContentKey {
    const fn bytes(&self) -> usize {
        self.words.len() * std::mem::size_of::<u64>()
    }
}

#[derive(Debug, Clone)]
struct Entry {
    identity: CacheIdentity,
    solid: usize,
    /// Bits of a requested deflection known to read this value.
    requested: u64,
    /// Bits of the deflection the reading is taken at (the clamped request).
    effective: u64,
    content: Option<ContentKey>,
    volume: f64,
    seeded: bool,
}

impl Entry {
    /// Whether a request with these deflection bits reads this entry, given
    /// that the content matches. A request equal to the clamped value clamps
    /// to itself, so it reads the same mesh.
    const fn answers(&self, deflection: u64) -> bool {
        self.requested == deflection || self.effective == deflection
    }
}

#[derive(Debug, Default)]
struct VolumeMemo {
    capacity: usize,
    byte_budget: usize,
    entries: VecDeque<Entry>,
    retained_bytes: usize,
    hits: u64,
    content_hits: u64,
    seeded_hits: u64,
    misses: u64,
    seeded: u64,
}

/// Which entry answered, and how it was found.
enum Found {
    Identity(usize),
    Content(usize),
}

impl VolumeMemo {
    const fn enabled(&self) -> bool {
        self.capacity > 0
    }

    fn find(&self, matches: impl Fn(&Entry) -> bool) -> Option<usize> {
        self.entries.iter().position(matches)
    }

    /// Count a hit on entry `index`; a content hit adopts the current
    /// identity so the next reading of this state takes the cheap test.
    fn hit(&mut self, found: Found, identity: Option<CacheIdentity>, solid: usize) -> Option<f64> {
        let (index, by_content) = match found {
            Found::Identity(index) => (index, false),
            Found::Content(index) => (index, true),
        };
        let entry = self.entries.get_mut(index)?;
        if by_content && let Some(identity) = identity {
            entry.identity = identity;
            entry.solid = solid;
        }
        self.hits += 1;
        self.content_hits += u64::from(by_content);
        self.seeded_hits += u64::from(entry.seeded);
        Some(entry.volume)
    }

    fn insert(&mut self, mut entry: Entry) {
        if !self.enabled() {
            return;
        }
        if entry
            .content
            .as_ref()
            .is_some_and(|key| key.bytes() > self.byte_budget)
        {
            entry.content = None;
        }
        // A reading without a content key is only ever found by identity,
        // and identities of one lineage only move forward.
        self.entries.retain(|e| {
            e.content.is_some()
                || e.identity.lineage != entry.identity.lineage
                || e.identity.generation >= entry.identity.generation
        });
        self.retained_bytes = self.entries.iter().map(entry_bytes).sum();
        let bytes = entry_bytes(&entry);
        while !self.entries.is_empty()
            && (self.entries.len() >= self.capacity
                || self.retained_bytes + bytes > self.byte_budget)
        {
            self.evict_front();
        }
        self.retained_bytes += bytes;
        self.entries.push_back(entry);
    }

    fn evict_front(&mut self) {
        if let Some(entry) = self.entries.pop_front() {
            self.retained_bytes -= entry_bytes(&entry);
        }
    }

    fn set_limits(&mut self, capacity: usize, byte_budget: usize) {
        self.capacity = capacity;
        self.byte_budget = byte_budget;
        while !self.entries.is_empty()
            && (self.entries.len() > capacity || self.retained_bytes > byte_budget)
        {
            self.evict_front();
        }
    }
}

fn entry_bytes(entry: &Entry) -> usize {
    entry.content.as_ref().map_or(0, ContentKey::bytes)
}

thread_local! {
    static MEMO: RefCell<VolumeMemo> = RefCell::new(VolumeMemo::default());
}

/// Set how many readings this thread's memo keeps; zero (the default)
/// disables it.
///
/// Shrinking drops the oldest readings first. The content-key byte budget is
/// set to [`DEFAULT_VOLUME_MEMO_BYTE_BUDGET`] unless one was set already.
pub fn set_thread_volume_memo_capacity(capacity: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            let budget = if memo.byte_budget == 0 {
                DEFAULT_VOLUME_MEMO_BYTE_BUDGET
            } else {
                memo.byte_budget
            };
            memo.set_limits(capacity, budget);
        }
    });
}

/// Set both bounds of this thread's memo. A zero capacity disables it; a
/// zero byte budget keeps readings findable by identity only.
pub fn set_thread_volume_memo_limits(capacity: usize, byte_budget: usize) {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.set_limits(capacity, byte_budget);
        }
    });
}

/// Enable this thread's memo at [`DEFAULT_VOLUME_MEMO_CAPACITY`] and
/// [`DEFAULT_VOLUME_MEMO_BYTE_BUDGET`] unless it is already enabled (then its
/// bounds and contents are kept).
pub fn enable_thread_volume_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut()
            && memo.capacity == 0
        {
            memo.set_limits(
                DEFAULT_VOLUME_MEMO_CAPACITY,
                DEFAULT_VOLUME_MEMO_BYTE_BUDGET,
            );
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
                content_hits: memo.content_hits,
                seeded_hits: memo.seeded_hits,
                misses: memo.misses,
                seeded: memo.seeded,
                len: memo.entries.len(),
                capacity: memo.capacity,
                retained_bytes: memo.retained_bytes,
                byte_budget: memo.byte_budget,
            })
            .unwrap_or_default()
    })
}

/// Drop every reading this thread's memo holds; bounds and counters stay.
pub fn clear_thread_volume_memo() {
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.entries.clear();
            memo.retained_bytes = 0;
        }
    });
}

fn memo_enabled() -> bool {
    MEMO.with(|memo| memo.try_borrow().is_ok_and(|memo| memo.enabled()))
}

/// `compute()` through this thread's memo. The memo is borrowed only around
/// the lookups and the insert, never across the computation, so a nested
/// reading cannot meet a held borrow.
pub(super) fn memoized(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
    compute: impl FnOnce() -> Result<f64, crate::OperationsError>,
) -> Result<f64, crate::OperationsError> {
    if !memo_enabled() {
        return compute();
    }
    let identity = (!topo.is_cache_poisoned()).then(|| topo.cache_identity());
    let requested = deflection.to_bits();
    let index = solid.index();
    let same_state =
        |e: &Entry| identity.is_some_and(|identity| e.identity == identity) && e.solid == index;
    let lookup = |find: &dyn Fn(&VolumeMemo) -> Option<Found>| {
        MEMO.with(|memo| {
            memo.try_borrow_mut().ok().and_then(|mut memo| {
                let found = find(&memo)?;
                memo.hit(found, identity, index)
            })
        })
    };

    // 1. This very state, at this request.
    if let Some(volume) = lookup(&|memo| {
        memo.find(|e| same_state(e) && e.answers(requested))
            .map(Found::Identity)
    }) {
        return Ok(volume);
    }
    // 2. The same content under another identity, at this request.
    let content = solid_content_key(topo, solid);
    let same_content = |e: &Entry| content.is_some() && e.content == content;
    if content.is_some()
        && let Some(volume) = lookup(&|memo| {
            memo.find(|e| same_content(e) && e.answers(requested))
                .map(Found::Content)
        })
    {
        return Ok(volume);
    }
    // 3. Another request that clamps to the same mesh.
    let effective =
        super::volume::volume_tessellation_deflection(topo, solid, deflection).to_bits();
    if effective != requested
        && let Some(volume) = lookup(&|memo| {
            memo.find(|e| same_state(e) && e.effective == effective)
                .map(Found::Identity)
                .or_else(|| {
                    memo.find(|e| same_content(e) && e.effective == effective)
                        .map(Found::Content)
                })
        })
    {
        return Ok(volume);
    }
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            memo.misses += 1;
        }
    });
    let volume = compute()?;
    if identity.is_some() || content.is_some() {
        MEMO.with(|memo| {
            if let Ok(mut memo) = memo.try_borrow_mut() {
                memo.insert(Entry {
                    identity: identity.unwrap_or(NO_IDENTITY),
                    solid: index,
                    requested,
                    effective,
                    content,
                    volume,
                    seeded: false,
                });
            }
        });
    }
    Ok(volume)
}

/// The measured readings this thread's memo holds for `solid`'s current
/// content, as `(requested deflection, volume)` pairs. Seeded readings are
/// left out, so a value derived from one is never derived again. Empty while
/// the memo is disabled.
pub fn measured_readings(topo: &Topology, solid: SolidId) -> Vec<(f64, f64)> {
    if !memo_enabled() {
        return Vec::new();
    }
    let Some(content) = solid_content_key(topo, solid) else {
        return Vec::new();
    };
    MEMO.with(|memo| {
        memo.try_borrow()
            .map(|memo| {
                memo.entries
                    .iter()
                    .filter(|e| !e.seeded && e.content.as_ref() == Some(&content))
                    .map(|e| (f64::from_bits(e.requested), e.volume))
                    .collect()
            })
            .unwrap_or_default()
    })
}

/// Record `volume` as `solid`'s reading at `deflection` without measuring
/// it. A measured reading of the same content at the same clamped
/// deflection is kept instead. Does nothing while the memo is disabled.
///
/// The caller vouches for the value: see the module docs.
pub fn seed_solid_volume(topo: &Topology, solid: SolidId, deflection: f64, volume: f64) {
    if !memo_enabled() || !volume.is_finite() {
        return;
    }
    let Some(content) = solid_content_key(topo, solid) else {
        return;
    };
    let effective =
        super::volume::volume_tessellation_deflection(topo, solid, deflection).to_bits();
    let identity = if topo.is_cache_poisoned() {
        NO_IDENTITY
    } else {
        topo.cache_identity()
    };
    MEMO.with(|memo| {
        if let Ok(mut memo) = memo.try_borrow_mut() {
            if memo
                .entries
                .iter()
                .any(|e| e.content.as_ref() == Some(&content) && e.effective == effective)
            {
                return;
            }
            memo.seeded += 1;
            memo.insert(Entry {
                identity,
                solid: solid.index(),
                requested: deflection.to_bits(),
                effective,
                content: Some(content),
                volume,
                seeded: true,
            });
        }
    });
}

/// Word sink for a solid content key.
struct ContentWriter {
    words: Vec<u64>,
}

impl ContentWriter {
    fn word(&mut self, word: u64) {
        self.words.push(word);
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

    fn reals(&mut self, xs: &[f64]) {
        self.count(xs.len());
        for &x in xs {
            self.real(x);
        }
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

    fn point2(&mut self, p: Point2) {
        self.real(p.x());
        self.real(p.y());
    }

    fn body_class(&mut self, class: BodyClass) {
        self.word(match class {
            BodyClass::Solid => 1,
            BodyClass::Sheet => 2,
            BodyClass::Wire => 3,
            BodyClass::General => 4,
        });
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

    fn curve2d(&mut self, curve: &Curve2D) {
        match curve {
            Curve2D::Line(c) => {
                self.word(1);
                self.point2(c.origin());
                self.real(c.direction().x());
                self.real(c.direction().y());
            }
            Curve2D::Circle(c) => {
                self.word(2);
                self.point2(c.center());
                self.real(c.radius());
            }
            Curve2D::Ellipse(c) => {
                self.word(3);
                self.point2(c.center());
                self.real(c.semi_major());
                self.real(c.semi_minor());
                self.real(c.rotation());
            }
            Curve2D::Nurbs(c) => {
                self.word(4);
                self.count(c.degree());
                self.reals(c.knots());
                self.count(c.control_points().len());
                for (&p, &w) in c.control_points().iter().zip(c.weights()) {
                    self.point2(p);
                    self.real(w);
                }
            }
        }
    }

    fn finish(self) -> ContentKey {
        let mut hash = 0_u64;
        for &word in &self.words {
            hash = (hash.rotate_left(5) ^ word).wrapping_mul(0x517c_c1b7_2722_0a95);
        }
        ContentKey {
            hash,
            words: self.words.into_boxed_slice(),
        }
    }
}

/// Everything `solid_volume` reads about `solid`, with every handle; see the
/// module docs. `None` when an entity is missing (the reading itself then
/// fails and is not memoized).
fn solid_content_key(topo: &Topology, solid: SolidId) -> Option<ContentKey> {
    let mut w = ContentWriter { words: Vec::new() };
    w.word(KEY_VERSION);
    w.count(solid.index());
    let solid_data = topo.solid(solid).ok()?;
    let shells: Vec<_> = std::iter::once(solid_data.outer_shell())
        .chain(solid_data.inner_shells().iter().copied())
        .collect();
    w.count(shells.len());
    let mut edges_seen = DetHashSet::default();
    let mut vertices_seen = DetHashSet::default();
    for shell_id in shells {
        let shell = topo.shell(shell_id).ok()?;
        w.count(shell_id.index());
        w.body_class(shell.body_class());
        w.count(shell.faces().len());
        for &face_id in shell.faces() {
            let face = topo.face(face_id).ok()?;
            w.count(face_id.index());
            w.flag(face.is_reversed());
            w.surface(face.surface());
            w.count(1 + face.inner_wires().len());
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                let wire = topo.wire(wire_id).ok()?;
                w.count(wire_id.index());
                w.flag(wire.is_closed());
                w.body_class(wire.body_class());
                w.count(wire.edges().len());
                for oriented in wire.edges() {
                    let edge_id = oriented.edge();
                    w.count(edge_id.index());
                    w.flag(oriented.is_forward());
                    match topo.pcurve_oriented(edge_id, face_id, oriented.is_forward()) {
                        Some(pcurve) => {
                            w.flag(true);
                            w.curve2d(pcurve.curve());
                            w.real(pcurve.t_start());
                            w.real(pcurve.t_end());
                        }
                        None => w.flag(false),
                    }
                    if !edges_seen.insert(edge_id.index()) {
                        continue;
                    }
                    let edge = topo.edge(edge_id).ok()?;
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
                        w.count(vertex_id.index());
                        if vertices_seen.insert(vertex_id.index()) {
                            let vertex = topo.vertex(vertex_id).ok()?;
                            w.point(vertex.point());
                            w.real(vertex.tolerance());
                        }
                    }
                }
            }
        }
    }
    Some(w.finish())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_math::curves2d::{Circle2D, Ellipse2D, Line2D, NurbsCurve2D};
    use remus_topology::pcurve::PCurve;

    /// Tripwire for the content key's completeness: each type it reads
    /// field by field beyond what `remus_check`'s face cache pins is pinned
    /// at its current size, so adding a field fails here until
    /// `ContentWriter` (and this pin) learn about it.
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn content_key_reads_every_pcurve_field() {
        use std::mem::size_of;
        const F: usize = size_of::<f64>();
        assert_eq!(size_of::<Line2D>(), 4 * F);
        assert_eq!(size_of::<Circle2D>(), 3 * F);
        assert_eq!(size_of::<Ellipse2D>(), 5 * F);
        assert_eq!(
            size_of::<NurbsCurve2D>(),
            size_of::<usize>() + 3 * size_of::<Vec<f64>>()
        );
        assert_eq!(size_of::<PCurve>(), size_of::<Curve2D>() + 2 * F);
    }
}
