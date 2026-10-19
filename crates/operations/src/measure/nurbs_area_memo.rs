//! Exact scalar memo of NURBS-face area readings.
//!
//! [`super::face_area`] tessellates a NURBS face's trimmed region and sums
//! triangle areas. That numerical path differs from trimmed quadrature and
//! from whole-solid display meshing, so neither can supply an
//! interchangeable area. This memo stores only values measured by that
//! existing path, preserving their bits.
//!
//! # Content and request policy
//!
//! The measurement tessellates the face alone through the shared-edge
//! pipeline, which reads the support surface, the reversal flag and the
//! complete boundary: every wire (outer first, then inner in face order),
//! its closed flag and oriented edges, each edge's curve, trim and vertex
//! positions, and the pcurve of each edge use on this face. The key records
//! all of it, plus the deflection, the default angular tolerance and the
//! disabled curvature-floor policy. Edge and vertex handles are replaced by
//! their first-occurrence ordinal within the face, so the key keeps which
//! uses share an entity (a seam walked twice, a closed edge's single vertex)
//! without depending on handle values. Reals compare by bits, so even signed
//! zeros remain distinct. A hash only filters candidates; the complete key
//! must match. Handles and topology identity are irrelevant: clones and
//! unchanged faces carried into another result may reuse a reading, while
//! changed content, including a re-trimmed face on an unchanged carrier,
//! misses without invalidation hooks. A change to what the face mesher reads
//! must extend the key. There is no reuse under translations or approximate
//! comparisons, and errors are never stored.
//!
//! # Bounds and lifetime
//!
//! The memo is disabled by default. Applications opt in per thread, as they do
//! for face-integral and volume memos. Entries are evicted in deterministic FIFO
//! order under both a count and an estimated retained-byte limit. Keys exceeding
//! the entire byte limit are rejected before the key is allocated; key
//! allocation failure also falls back to the original calculation. Estimated
//! bytes include key words and each entry record, not allocator or container
//! slack and not RSS. No cache borrow is held across the calculation, and an
//! unavailable cache borrow simply takes the original path.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::hash::Hasher;

use remus_math::curves2d::Curve2D;
use remus_math::det_hash::{DetHashMap, DetHashSet, DetHasher};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::{BodyClass, Topology};

/// Default retained NURBS-area readings per thread once enabled.
pub const DEFAULT_NURBS_AREA_MEMO_CAPACITY: usize = 512;

/// Default estimated retained key and entry bytes per thread once enabled.
pub const DEFAULT_NURBS_AREA_MEMO_BYTE_BUDGET: usize = 8 * 1024 * 1024;

/// Bump when the key's content or request encoding changes.
const KEY_VERSION: u64 = 0x5245_4d55_534e_4134;
/// Leads a face digest's key, so it never equals an area-memo key.
const FACE_DIGEST_KEY_VERSION: u64 = 0x5245_4d55_5346_4433;

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

/// Counts key words, or writes them once a buffer is attached.
struct KeyWriter {
    words: usize,
    key: Option<Vec<u64>>,
}

impl KeyWriter {
    fn word(&mut self, word: u64) {
        self.words = self.words.saturating_add(1);
        if let Some(key) = self.key.as_mut() {
            key.push(word);
        }
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

    fn surface(&mut self, surface: &NurbsSurface) {
        self.count(surface.knots_u().len());
        for &x in surface.knots_u() {
            self.real(x);
        }
        self.count(surface.knots_v().len());
        for &x in surface.knots_v() {
            self.real(x);
        }
        self.count(surface.control_points().len());
        for row in surface.control_points() {
            self.count(row.len());
            for &p in row {
                self.point(p);
            }
        }
        self.count(surface.weights().len());
        for row in surface.weights() {
            self.reals(row);
        }
    }

    /// Every field of a support surface of any type, for the face digest.
    fn face_surface(&mut self, surface: &FaceSurface) {
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
                self.surface(s);
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

    fn pcurve(&mut self, pcurve: Option<&PCurve>) {
        let Some(pcurve) = pcurve else {
            self.word(0);
            return;
        };
        match pcurve.curve() {
            Curve2D::Line(c) => {
                self.word(1);
                self.reals(&[
                    c.origin().x(),
                    c.origin().y(),
                    c.direction().x(),
                    c.direction().y(),
                ]);
            }
            Curve2D::Circle(c) => {
                self.word(2);
                self.reals(&[c.center().x(), c.center().y(), c.radius()]);
            }
            Curve2D::Ellipse(c) => {
                self.word(3);
                self.reals(&[
                    c.center().x(),
                    c.center().y(),
                    c.semi_major(),
                    c.semi_minor(),
                    c.rotation(),
                ]);
            }
            Curve2D::Nurbs(c) => {
                self.word(4);
                self.count(c.degree());
                self.reals(c.knots());
                self.count(c.control_points().len());
                for p in c.control_points() {
                    self.real(p.x());
                    self.real(p.y());
                }
                self.reals(c.weights());
            }
        }
        self.real(pcurve.t_start());
        self.real(pcurve.t_end());
    }

    /// Every boundary field the face mesher reads; see the module docs.
    fn boundary(&mut self, topo: &Topology, face_id: FaceId) -> Option<()> {
        let face = topo.face(face_id).ok()?;
        let mut edge_ordinals: DetHashMap<usize, u64> = DetHashMap::default();
        let mut vertex_ordinals: DetHashMap<usize, u64> = DetHashMap::default();
        self.count(1 + face.inner_wires().len());
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id).ok()?;
            self.count(wire.edges().len());
            self.word(u64::from(wire.is_closed()));
            for oriented in wire.edges() {
                let edge_id = oriented.edge();
                self.word(u64::from(oriented.is_forward()));
                self.pcurve(topo.pcurve_oriented(edge_id, face_id, oriented.is_forward()));
                let next = edge_ordinals.len() as u64;
                let ordinal = *edge_ordinals.entry(edge_id.index()).or_insert(next);
                self.word(ordinal);
                if ordinal != next {
                    continue;
                }
                let edge = topo.edge(edge_id).ok()?;
                for vertex_id in [edge.start(), edge.end()] {
                    let next = vertex_ordinals.len() as u64;
                    let ordinal = *vertex_ordinals.entry(vertex_id.index()).or_insert(next);
                    self.word(ordinal);
                    if ordinal == next {
                        self.point(topo.vertex(vertex_id).ok()?.point());
                    }
                }
                self.curve(edge.curve());
                match edge.trim() {
                    Some((t0, t1)) => {
                        self.word(1);
                        self.real(t0);
                        self.real(t1);
                    }
                    None => self.word(0),
                }
                // `Edge::strict_domain` validates closed conic trims against it.
                match edge.tolerance() {
                    Some(tolerance) => {
                        self.word(1);
                        self.real(tolerance);
                    }
                    None => self.word(0),
                }
            }
        }
        // The boundary pool inserts edges in arena-index order and keeps the
        // first point landing in each merge cell, so that order is content.
        let mut by_arena: Vec<(usize, u64)> = edge_ordinals.into_iter().collect();
        by_arena.sort_unstable();
        for (_, ordinal) in by_arena {
            self.word(ordinal);
        }
        Some(())
    }

    fn key(
        &mut self,
        topo: &Topology,
        face_id: FaceId,
        surface: &NurbsSurface,
        deflection: f64,
    ) -> Option<()> {
        let reversed = topo.face(face_id).ok()?.is_reversed();
        self.word(KEY_VERSION);
        self.count(surface.degree_u());
        self.count(surface.degree_v());
        self.word(u64::from(reversed));
        self.real(deflection);
        self.real(remus_math::chord::DEFAULT_ANGULAR_TOL);
        self.word(0); // Curvature floor disabled in measurement tessellation.
        self.surface(surface);
        self.boundary(topo, face_id)
    }
}

/// Key words of a face's complete content: its reversal, every field of its
/// support surface, the boundary this memo's key records (wires, oriented
/// edges, curves, trims, tolerances, vertex positions and pcurves), the
/// authoritative loops and coedges with their periodic windings, and the
/// wires' body classes and vertices' tolerances: every field of the face and
/// the entities under it. Handles enter as first-occurrence ordinals. `None`
/// when an entity is missing.
pub(super) fn face_content_words(topo: &Topology, face_id: FaceId) -> Option<Vec<u64>> {
    let face = topo.face(face_id).ok()?;
    let mut writer = KeyWriter {
        words: 0,
        key: Some(Vec::new()),
    };
    writer.word(FACE_DIGEST_KEY_VERSION);
    writer.word(u64::from(face.is_reversed()));
    writer.face_surface(face.surface());
    writer.boundary(topo, face_id)?;
    // The wires' first-occurrence edge ordinals, so a loop names its edges
    // the way the boundary words above do.
    let mut edge_ordinals: DetHashMap<usize, u64> = DetHashMap::default();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id).ok()?.edges() {
            let next = edge_ordinals.len() as u64;
            edge_ordinals.entry(oriented.edge().index()).or_insert(next);
        }
    }
    writer.count(face.boundary_loops().len());
    for &loop_id in face.boundary_loops() {
        let boundary = topo.face_loop(loop_id).ok()?;
        writer.word(u64::from(boundary.is_closed()));
        writer.count(boundary.coedges().len());
        for &coedge_id in boundary.coedges() {
            let coedge = topo.coedge(coedge_id).ok()?;
            // An edge the wires do not use is a different boundary.
            writer.word(
                edge_ordinals
                    .get(&coedge.edge().index())
                    .copied()
                    .unwrap_or(u64::MAX),
            );
            writer.word(u64::from(coedge.is_forward()));
            writer.pcurve(coedge.pcurve());
            let winding = coedge.periodic_winding();
            writer.word(u64::from(winding.u().cast_unsigned()));
            writer.word(u64::from(winding.v().cast_unsigned()));
        }
    }
    // Fields the mesher does not read, so the boundary words above omit
    // them: each wire's body class and each vertex's tolerance, the vertices
    // in the boundary's first-occurrence order.
    let mut vertices_seen: DetHashSet<usize> = DetHashSet::default();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id).ok()?;
        writer.word(match wire.body_class() {
            BodyClass::Solid => 1,
            BodyClass::Sheet => 2,
            BodyClass::Wire => 3,
            BodyClass::General => 4,
        });
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge()).ok()?;
            for vertex_id in [edge.start(), edge.end()] {
                if vertices_seen.insert(vertex_id.index()) {
                    writer.real(topo.vertex(vertex_id).ok()?.tolerance());
                }
            }
        }
    }
    writer.key
}

/// Count the complete key, then allocate and write it only when it fits.
/// `None` sends the request down the original calculation without retention.
fn content_key(
    topo: &Topology,
    face_id: FaceId,
    surface: &NurbsSurface,
    deflection: f64,
    byte_budget: usize,
) -> Option<Box<[u64]>> {
    let mut counter = KeyWriter {
        words: 0,
        key: None,
    };
    counter.key(topo, face_id, surface, deflection)?;
    let words = counter.words;
    let bytes = words
        .checked_mul(std::mem::size_of::<u64>())?
        .checked_add(std::mem::size_of::<Entry>())?;
    if bytes > byte_budget {
        return None;
    }
    let mut key = Vec::new();
    key.try_reserve_exact(words).ok()?;
    let mut writer = KeyWriter {
        words: 0,
        key: Some(key),
    };
    writer.key(topo, face_id, surface, deflection)?;
    let key = writer.key?;
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
    topo: &Topology,
    face_id: FaceId,
    surface: &NurbsSurface,
    deflection: f64,
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
    let Some(key) = content_key(topo, face_id, surface, deflection, budget) else {
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
    // error. Every retained value is the trimmed tessellation-area sum.
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
