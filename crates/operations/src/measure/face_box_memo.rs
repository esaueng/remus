//! Exact-content memo of single-face bounding boxes.
//!
//! The swept-region refusal of a face move boxes every face of the body on
//! every call through [`super::face_set_bounding_box`], and on a NURBS face
//! that box inverts boundary samples onto the surface. Between the frames of
//! a drag almost every face is unchanged, so this memo stores each box under
//! the face's content and answers a later request with the stored bits.
//!
//! # Key
//!
//! The box reads the surface (every field) and the boundary: per wire, outer
//! first then inner in face order, each edge use in wire order with the
//! edge's curve (every field), trim and tolerance, whether its two vertices
//! are one vertex (the closed-edge domain check), and both vertex positions.
//! The key records all of that, plus the use's direction and the wire's
//! closed flag, which the box does not read. Handle values reach the box in
//! one place only: its starting box is taken over the boundary vertices in
//! ascending handle order, and the first of equal coordinates wins. That
//! order shows only when an axis holds both signed zeros or a NaN, and then
//! the key also lists those vertex positions in that order. Reals compare by
//! bits. A hash only filters candidates; the whole key must match. Errors
//! are never stored.
//!
//! # Bounds and lifetime
//!
//! Off by default; applications opt in per thread, as for the face-integral
//! and NURBS-area memos (the WASM kernel does on construction). Entries are
//! evicted in FIFO order under a count and an estimated retained-byte limit.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_math::aabb::Aabb3;
use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

/// Default retained face boxes per thread once enabled.
pub const DEFAULT_FACE_BOX_MEMO_CAPACITY: usize = 4096;

/// Default estimated retained key and entry bytes per thread once enabled.
pub const DEFAULT_FACE_BOX_MEMO_BYTE_BUDGET: usize = 8 * 1024 * 1024;

/// Bump when the key's encoding changes.
const KEY_VERSION: u64 = 0x5245_4d55_4642_4f31;

/// Counters and current bounds of the thread's face-box memo.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FaceBoxMemoStats {
    /// Requests answered by an exact-content entry.
    pub hits: u64,
    /// Requests computed through an enabled memo.
    pub misses: u64,
    /// Entries dropped to satisfy the count or byte bound.
    pub evictions: u64,
    /// Boxes currently retained.
    pub len: usize,
    /// Maximum retained boxes; zero disables lookups and retention.
    pub capacity: usize,
    /// Estimated retained key and entry bytes, excluding allocator slack.
    pub retained_bytes: usize,
    /// Maximum estimated retained bytes; zero disables lookups and retention.
    pub byte_budget: usize,
}

struct Entry {
    hash: u64,
    key: Box<[u64]>,
    value: Aabb3,
}

impl Entry {
    fn bytes(&self) -> usize {
        self.key.len() * std::mem::size_of::<u64>() + std::mem::size_of::<Self>()
    }
}

struct Memo {
    /// Oldest first; entry `k` has sequence number `first_seq + k`.
    entries: VecDeque<Entry>,
    first_seq: u64,
    /// Key hash to the sequence numbers of its entries, ascending.
    buckets: DetHashMap<u64, Vec<u64>>,
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
            first_seq: 0,
            buckets: DetHashMap::default(),
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

    fn find(&self, hash: u64, key: &[u64]) -> Option<Aabb3> {
        let seqs = self.buckets.get(&hash)?;
        seqs.iter().find_map(|&seq| {
            let entry = self
                .entries
                .get(usize::try_from(seq.checked_sub(self.first_seq)?).ok()?)?;
            (*entry.key == *key).then_some(entry.value)
        })
    }

    fn evict_front(&mut self) {
        let Some(entry) = self.entries.pop_front() else {
            return;
        };
        if let Some(seqs) = self.buckets.get_mut(&entry.hash) {
            // Sequence numbers enter each bucket in ascending order, so the
            // oldest entry is the bucket's first.
            if seqs.first() == Some(&self.first_seq) {
                seqs.remove(0);
            }
            if seqs.is_empty() {
                self.buckets.remove(&entry.hash);
            }
        }
        self.first_seq += 1;
        self.retained_bytes -= entry.bytes();
        self.evictions = self.evictions.saturating_add(1);
    }

    fn enforce_limits(&mut self) {
        while !self.entries.is_empty()
            && (self.entries.len() > self.max_entries || self.retained_bytes > self.max_bytes)
        {
            self.evict_front();
        }
    }

    fn insert(&mut self, entry: Entry) {
        let bytes = entry.bytes();
        if !self.enabled() || bytes > self.max_bytes {
            return;
        }
        while !self.entries.is_empty()
            && (self.entries.len() >= self.max_entries
                || bytes > self.max_bytes.saturating_sub(self.retained_bytes))
        {
            self.evict_front();
        }
        let seq = self.first_seq + self.entries.len() as u64;
        self.buckets.entry(entry.hash).or_default().push(seq);
        self.entries.push_back(entry);
        self.retained_bytes += bytes;
    }

    fn clear(&mut self) {
        self.first_seq += self.entries.len() as u64;
        self.entries.clear();
        self.buckets.clear();
        self.retained_bytes = 0;
    }

    fn stats(&self) -> FaceBoxMemoStats {
        FaceBoxMemoStats {
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
/// Existing bounds, entries and counters are preserved, including a zero
/// bound configured to disable retention.
pub fn enable_thread_face_box_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            slot.get_or_insert_with(|| {
                Memo::new(
                    DEFAULT_FACE_BOX_MEMO_CAPACITY,
                    DEFAULT_FACE_BOX_MEMO_BYTE_BUDGET,
                )
            });
        }
    });
}

/// Set this thread's memo bounds, evicting oldest entries immediately to fit.
///
/// A zero bound disables lookups and retention without discarding counters.
pub fn set_thread_face_box_memo_limits(capacity: usize, byte_budget: usize) {
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
pub fn disable_thread_face_box_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut() {
            *slot = None;
        }
    });
}

/// Drop this thread's retained boxes, keeping its bounds and counters.
pub fn clear_thread_face_box_memo() {
    MEMO.with(|cell| {
        if let Ok(mut slot) = cell.try_borrow_mut()
            && let Some(memo) = slot.as_mut()
        {
            memo.clear();
        }
    });
}

/// Return counters and bounds, or `None` without an available memo instance.
#[must_use]
pub fn thread_face_box_memo_stats() -> Option<FaceBoxMemoStats> {
    MEMO.with(|cell| {
        cell.try_borrow()
            .ok()
            .and_then(|slot| slot.as_ref().map(Memo::stats))
    })
}

/// Word-wise key writer with a running hash.
struct KeyWriter {
    key: Vec<u64>,
    hash: u64,
}

impl KeyWriter {
    fn word(&mut self, word: u64) {
        self.key.push(word);
        self.hash = (self.hash ^ word).wrapping_mul(0x0000_0100_0000_01b3);
        self.hash ^= self.hash >> 29;
    }

    fn count(&mut self, count: usize) {
        self.word(count as u64);
    }

    fn real(&mut self, value: f64) {
        self.word(value.to_bits());
    }

    fn reals(&mut self, values: &[f64]) {
        self.count(values.len());
        for &value in values {
            self.real(value);
        }
    }

    fn point(&mut self, p: Point3) {
        self.real(p.x());
        self.real(p.y());
        self.real(p.z());
    }

    fn vec(&mut self, v: Vec3) {
        self.real(v.x());
        self.real(v.y());
        self.real(v.z());
    }

    fn surface(&mut self, surface: &FaceSurface) {
        match surface {
            FaceSurface::Plane { normal, d } => {
                self.word(1);
                self.vec(*normal);
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
                    for &p in row {
                        self.point(p);
                    }
                    self.reals(weights);
                }
                self.count(s.weights().len());
            }
            FaceSurface::Cylinder(s) => {
                self.word(3);
                self.point(s.origin());
                self.vec(s.axis());
                self.real(s.radius());
                self.vec(s.x_axis());
                self.vec(s.y_axis());
            }
            FaceSurface::Cone(s) => {
                self.word(4);
                self.point(s.apex());
                self.vec(s.axis());
                self.real(s.half_angle());
                self.vec(s.x_axis());
                self.vec(s.y_axis());
            }
            FaceSurface::Sphere(s) => {
                self.word(5);
                self.point(s.center());
                self.real(s.radius());
                self.vec(s.x_axis());
                self.vec(s.y_axis());
                self.vec(s.z_axis());
            }
            FaceSurface::Torus(s) => {
                self.word(6);
                self.point(s.center());
                self.real(s.major_radius());
                self.real(s.minor_radius());
                self.vec(s.x_axis());
                self.vec(s.y_axis());
                self.vec(s.z_axis());
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
                for &p in c.control_points() {
                    self.point(p);
                }
                self.reals(c.weights());
            }
            EdgeCurve::Circle(c) => {
                self.word(3);
                self.point(c.center());
                self.vec(c.normal());
                self.real(c.radius());
                self.vec(c.u_axis());
                self.vec(c.v_axis());
            }
            EdgeCurve::Ellipse(c) => {
                self.word(4);
                self.point(c.center());
                self.vec(c.normal());
                self.real(c.semi_major());
                self.real(c.semi_minor());
                self.vec(c.u_axis());
                self.vec(c.v_axis());
            }
            EdgeCurve::Hyperbola(c) => {
                self.word(5);
                self.point(c.center());
                self.vec(c.normal());
                self.real(c.semi_major());
                self.real(c.semi_minor());
                self.vec(c.u_axis());
                self.vec(c.v_axis());
            }
            EdgeCurve::Parabola(c) => {
                self.word(6);
                self.point(c.vertex());
                self.vec(c.axis_dir());
                self.real(c.focal_length());
                self.vec(c.u_axis());
            }
        }
    }

    /// Everything the face box reads; see the module docs. `None` when a
    /// lookup fails, which the box itself then reports.
    fn face(&mut self, topo: &Topology, face_id: FaceId) -> Option<()> {
        let face = topo.face(face_id).ok()?;
        self.word(KEY_VERSION);
        self.surface(face.surface());
        let mut vertex_ids = Vec::new();
        self.count(1 + face.inner_wires().len());
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id).ok()?;
            self.count(wire.edges().len());
            self.word(u64::from(wire.is_closed()));
            for oriented in wire.edges() {
                let edge = topo.edge(oriented.edge()).ok()?;
                self.word(u64::from(oriented.is_forward()));
                self.word(u64::from(edge.start() == edge.end()));
                self.point(topo.vertex(edge.start()).ok()?.point());
                self.point(topo.vertex(edge.end()).ok()?.point());
                self.curve(edge.curve());
                match edge.trim() {
                    Some((t0, t1)) => {
                        self.word(1);
                        self.real(t0);
                        self.real(t1);
                    }
                    None => self.word(0),
                }
                match edge.tolerance() {
                    Some(tolerance) => {
                        self.word(1);
                        self.real(tolerance);
                    }
                    None => self.word(0),
                }
                vertex_ids.push(edge.start());
                vertex_ids.push(edge.end());
            }
        }
        vertex_ids.sort_unstable();
        vertex_ids.dedup();
        let mut points = Vec::with_capacity(vertex_ids.len());
        for vertex in vertex_ids {
            points.push(topo.vertex(vertex).ok()?.point());
        }
        if order_sensitive(&points) {
            self.word(1);
            self.count(points.len());
            for p in points {
                self.point(p);
            }
        } else {
            self.word(0);
        }
        Some(())
    }
}

/// Whether the box over `points` can depend on their order: it keeps the
/// first of equal coordinates, which differ in bits only as `+0.0` and
/// `-0.0`, and a NaN wins only from the front. Every other point list boxes
/// to the same bits in any order, so its positions (already keyed through
/// the edges) settle the box.
fn order_sensitive(points: &[Point3]) -> bool {
    (0..3).any(|axis| {
        let coordinate = |p: &Point3| [p.x(), p.y(), p.z()][axis];
        let has = |zero: f64| {
            points
                .iter()
                .any(|p| coordinate(p).to_bits() == zero.to_bits())
        };
        points.iter().any(|p| coordinate(p).is_nan()) || (has(0.0) && has(-0.0))
    })
}

/// [`super::face_set_bounding_box`] of the single face `face`, through this
/// thread's memo when it is enabled (a plain call otherwise). The box is
/// bit-identical either way.
///
/// # Errors
///
/// Exactly the errors of [`super::face_set_bounding_box`].
pub fn face_bounding_box(topo: &Topology, face: FaceId) -> Result<Aabb3, crate::OperationsError> {
    let budget = MEMO.with(|cell| {
        cell.try_borrow().ok().and_then(|slot| {
            slot.as_ref()
                .filter(|memo| memo.enabled())
                .map(|memo| memo.max_bytes)
        })
    });
    let Some(budget) = budget else {
        return super::face_set_bounding_box(topo, &[face]);
    };
    let mut writer = KeyWriter {
        key: Vec::new(),
        hash: 0xcbf2_9ce4_8422_2325,
    };
    let keyed = writer.face(topo, face).is_some()
        && writer.key.len() * std::mem::size_of::<u64>() + std::mem::size_of::<Entry>() <= budget;
    let (hash, key) = (writer.hash, writer.key);
    let hit = MEMO.with(|cell| {
        let mut slot = cell.try_borrow_mut().ok()?;
        let memo = slot.as_mut()?;
        let hit = keyed.then(|| memo.find(hash, &key)).flatten();
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
    // Never hold a memo borrow across the calculation and never store an
    // error.
    let value = super::face_set_bounding_box(topo, &[face])?;
    if keyed {
        MEMO.with(|cell| {
            if let Ok(mut slot) = cell.try_borrow_mut()
                && let Some(memo) = slot.as_mut()
            {
                memo.insert(Entry {
                    hash,
                    key: key.into_boxed_slice(),
                    value,
                });
            }
        });
    }
    Ok(value)
}

#[cfg(test)]
mod tests;
