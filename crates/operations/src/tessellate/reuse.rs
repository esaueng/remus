//! Exact reuse of shared boundaries, planar CDT jobs and mesh connectivity.
//!
//! Keys contain complete input words, not only hashes or arena handles. A
//! boundary hit remaps edge identities in sorted order; a miss reuses only
//! individually identical edge samples. CDT hits retain local 2D triangles
//! and Steiner points: lifting and shared-boundary reconciliation still run.
//! Incremental connectivity compares every triangle index and updates only
//! changed directed edges. Duplicate-triangle reuse additionally requires
//! freshly recomputed, identical quantized-position equivalence classes;
//! changed triangles must still have unique nondegenerate position keys.
//!
//! This opt-in thread-local cache has separate, explicit bounds. Quantity
//! work cannot evict display work. No tolerance, refinement limit, geometry
//! validation, or face-mesh cache budget changes.

use std::cell::RefCell;
use std::collections::{BTreeMap, VecDeque};

use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;

use super::boundary_plan::BoundaryPlan;
use super::face_cache::{KeyBuilder, RetentionClass};
use super::planar::PlanarCdtOutput;
mod incremental;
use incremental::{DedupeUpdate, WeldUpdate};

/// Default maximum retained stage entries (shared across all stage kinds).
pub const DEFAULT_MESH_REUSE_ENTRIES: usize = 4096;
/// Default maximum estimated retained stage bytes, separate from face meshes.
pub const DEFAULT_MESH_REUSE_BYTES: usize = 16 * 1024 * 1024;

/// Cumulative work and bounded storage of this thread's stage cache.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
pub struct MeshReuseStats {
    /// Boundary plan lookups.
    pub boundary_lookups: u64,
    /// Complete boundary plans reused.
    pub boundary_hits: u64,
    /// Individual edge sample lookups on a boundary-plan miss.
    pub edge_lookups: u64,
    /// Unchanged edges served without resampling.
    pub edge_hits: u64,
    /// Holed-planar CDT lookups.
    pub planar_lookups: u64,
    /// Local CDT results reused (reconciliation still runs).
    pub planar_hits: u64,
    /// Boundary-connectivity lookups for the weld pass.
    pub weld_lookups: u64,
    /// Boundary-connectivity plans updated; spatial welding still runs.
    pub weld_hits: u64,
    /// Triangles updated in retained directed connectivity.
    pub weld_updated_triangles: u64,
    /// Triangles updated in retained coincident-triangle membership.
    pub cleanup_updated_triangles: u64,
    /// Estimated boundary/sample/CDT bytes.
    pub stage_bytes: usize,
    /// Estimated incremental mesh-connectivity bytes.
    pub connectivity_bytes: usize,
    /// Mesh cleanup certificates checked.
    pub cleanup_lookups: u64,
    /// No-removal proofs updated after checking changed triangle membership.
    pub cleanup_hits: u64,
    /// Retained entries.
    pub entries: usize,
    /// Estimated retained bytes, including keys; not RSS.
    pub retained_bytes: usize,
    /// Configured entry limit.
    pub max_entries: usize,
    /// Configured estimated-byte limit.
    pub max_bytes: usize,
    /// Evicted entries.
    pub evictions: u64,
}

thread_local! {
    static CACHE: RefCell<Option<Cache>> = const { RefCell::new(None) };
}

/// Enable stage reuse on this thread with default bounds; idempotent.
pub fn enable_mesh_reuse() {
    enable_mesh_reuse_with_limits(DEFAULT_MESH_REUSE_ENTRIES, DEFAULT_MESH_REUSE_BYTES);
}

/// Enable stage reuse with explicit total entry and estimated-byte bounds.
/// Reducing the bounds evicts quantity entries first, then display entries.
pub fn enable_mesh_reuse_with_limits(max_entries: usize, max_bytes: usize) {
    CACHE.with(|slot| {
        let mut slot = slot.borrow_mut();
        let cache = slot.get_or_insert_with(Cache::default);
        cache.max_entries = max_entries;
        cache.max_bytes = max_bytes;
        cache.trim_classes();
        cache.make_room(0, 0, RetentionClass::Display);
    });
}

/// Disable stage reuse and release all its retained entries.
pub fn disable_mesh_reuse() {
    CACHE.with(|slot| *slot.borrow_mut() = None);
}

/// Clear retained entries without changing configured bounds or counters.
pub fn clear_mesh_reuse() {
    CACHE.with(|slot| {
        if let Some(cache) = slot.borrow_mut().as_mut() {
            cache.entries.clear();
            cache.buckets.clear();
            cache.display.clear();
            cache.quantity.clear();
            cache.bytes = 0;
            cache.class_bytes = [0; 2];
        }
    });
}

/// Statistics, or `None` when reuse is disabled on this thread.
#[must_use]
pub fn mesh_reuse_stats() -> Option<MeshReuseStats> {
    CACHE.with(|slot| slot.borrow().as_ref().map(Cache::stats))
}

#[derive(Clone)]
enum Value {
    Boundary(Box<BoundaryPlan>),
    Edge(Vec<Point3>, Vec<Option<f64>>),
    Planar(PlanarCdtOutput),
    Clean(Box<DedupeUpdate>),
    Weld(Box<WeldUpdate>),
    Seen(bool),
}

impl Value {
    fn class(&self) -> usize {
        match self {
            Self::Clean(_) | Self::Weld(_) | Self::Seen(_) => 1,
            _ => 0,
        }
    }
    fn bytes(&self) -> usize {
        match self {
            Self::Boundary(plan) => plan.retained_bytes(),
            Self::Edge(points, params) => points.capacity() * 24 + params.capacity() * 16,
            Self::Planar((tris, points)) => {
                tris.capacity() * 3 * size_of::<usize>() + points.capacity() * 16
            }
            Self::Clean(state) => state.bytes(),
            Self::Weld(state) => state.bytes(),
            Self::Seen(_) => 0,
        }
    }
}

struct Entry {
    key: Vec<u64>,
    value: Value,
    retention: RetentionClass,
    bytes: usize,
}

#[derive(Default)]
struct Cache {
    entries: BTreeMap<u64, Entry>,
    buckets: DetHashMap<u64, Vec<u64>>,
    display: VecDeque<u64>,
    quantity: VecDeque<u64>,
    next: u64,
    bytes: usize,
    class_bytes: [usize; 2],
    max_entries: usize,
    max_bytes: usize,
    counters: MeshReuseStats,
}

fn hash(words: &[u64]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = remus_math::det_hash::DetLookupHasher::default();
    words.hash(&mut hasher);
    hasher.finish()
}

impl Cache {
    fn stats(&self) -> MeshReuseStats {
        MeshReuseStats {
            entries: self.entries.len(),
            retained_bytes: self.bytes,
            stage_bytes: self.class_bytes[0],
            connectivity_bytes: self.class_bytes[1],
            max_entries: self.max_entries,
            max_bytes: self.max_bytes,
            ..self.counters
        }
    }

    fn find(&self, key: &[u64]) -> Option<u64> {
        self.buckets
            .get(&hash(key))?
            .iter()
            .find(|&&id| self.entries.get(&id).is_some_and(|entry| entry.key == key))
            .copied()
    }

    fn promote(&mut self, id: u64, retention: RetentionClass) {
        let Some(entry) = self.entries.get_mut(&id) else {
            return;
        };
        if retention == RetentionClass::Display {
            entry.retention = RetentionClass::Display;
        }
        // Refresh successful uses so inserting a newly changed whole-body
        // plan retires older plans before the edge/CDT work just reused.
        self.quantity.retain(|&old| old != id);
        self.display.retain(|&old| old != id);
        match entry.retention {
            RetentionClass::Display => self.display.push_back(id),
            RetentionClass::Quantity => self.quantity.push_back(id),
        }
    }

    fn get(&mut self, key: &[u64], retention: RetentionClass) -> Option<Value> {
        let id = self.find(key)?;
        self.promote(id, retention);
        Some(self.entries.get(&id)?.value.clone())
    }

    fn remove(&mut self, id: u64) {
        if let Some(entry) = self.entries.remove(&id) {
            self.bytes -= entry.bytes;
            self.class_bytes[entry.value.class()] -= entry.bytes;
            let bucket_hash = hash(&entry.key);
            if let Some(bucket) = self.buckets.get_mut(&bucket_hash) {
                bucket.retain(|&old| old != id);
                if bucket.is_empty() {
                    self.buckets.remove(&bucket_hash);
                }
            }
            self.display.retain(|&old| old != id);
            self.quantity.retain(|&old| old != id);
            self.counters.evictions += 1;
        }
    }

    fn class_limit(&self, class: usize) -> usize {
        // Keep small reusable sampling/CDT work independent of large mesh
        // connectivity. Both shares remain inside the configured total.
        let stages = self.max_bytes / 4;
        if class == 0 {
            stages
        } else {
            self.max_bytes - stages
        }
    }

    fn connectivity_probe(&mut self, key: &[u64], retention: RetentionClass) -> Option<u64> {
        let Some(id) = self.find(key) else {
            self.insert(key.to_vec(), Value::Seen(false), retention);
            return None;
        };
        if let Some(Entry {
            value: Value::Seen(ready),
            ..
        }) = self.entries.get_mut(&id)
        {
            *ready = true;
            self.promote(id, retention);
            None
        } else {
            Some(id)
        }
    }

    fn capture_ready(&self, key: &[u64]) -> bool {
        self.find(key)
            .and_then(|id| self.entries.get(&id))
            .is_some_and(|entry| matches!(entry.value, Value::Seen(true)))
    }

    fn replace_marker(&mut self, key: Vec<u64>, value: Value, retention: RetentionClass) {
        if let Some(id) = self.find(&key) {
            self.remove(id);
        }
        self.insert(key, value, retention);
    }

    fn trim_classes(&mut self) {
        for class in 0..2 {
            while self.class_bytes[class] > self.class_limit(class) {
                let eligible = |id: &&u64| {
                    self.entries
                        .get(id)
                        .is_some_and(|entry| entry.value.class() == class)
                };
                let id = self
                    .quantity
                    .iter()
                    .find(eligible)
                    .copied()
                    .or_else(|| self.display.iter().find(eligible).copied());
                let Some(id) = id else {
                    break;
                };
                self.remove(id);
            }
        }
    }

    fn finish_update(&mut self, id: u64, success: bool, retention: RetentionClass) {
        if !success {
            self.remove(id);
            return;
        }
        let Some(entry) = self.entries.get(&id) else {
            return;
        };
        let class = entry.value.class();
        let bytes = entry.key.capacity() * 8 + entry.value.bytes() + size_of::<Entry>() + 256;
        let old = entry.bytes;
        if self.bytes - old + bytes > self.max_bytes
            || self.class_bytes[class] - old + bytes > self.class_limit(class)
        {
            self.remove(id);
            return;
        }
        self.bytes = self.bytes - old + bytes;
        self.class_bytes[class] = self.class_bytes[class] - old + bytes;
        if let Some(entry) = self.entries.get_mut(&id) {
            entry.bytes = bytes;
        }
        self.promote(id, retention);
    }

    fn make_room(&mut self, bytes: usize, count: usize, retention: RetentionClass) -> bool {
        if bytes > self.max_bytes || count > self.max_entries {
            return false;
        }
        // Preflight quantity admission: an unfit request must not flush other
        // useful quantity work before discovering that display work is pinned.
        if retention == RetentionClass::Quantity {
            let (display_count, display_bytes) = self
                .entries
                .values()
                .filter(|entry| entry.retention == RetentionClass::Display)
                .fold((0, 0), |(n, b), entry| (n + 1, b + entry.bytes));
            if display_count + count > self.max_entries || display_bytes + bytes > self.max_bytes {
                return false;
            }
        }
        while self.entries.len() + count > self.max_entries || self.bytes + bytes > self.max_bytes {
            let id = self.quantity.pop_front().or_else(|| {
                (retention == RetentionClass::Display)
                    .then(|| self.display.pop_front())
                    .flatten()
            });
            let Some(id) = id else {
                return false;
            };
            self.remove(id);
        }
        true
    }

    fn insert(&mut self, key: Vec<u64>, value: Value, retention: RetentionClass) {
        let bucket_hash = hash(&key);
        if self.buckets.get(&bucket_hash).is_some_and(|ids| {
            ids.iter()
                .any(|id| self.entries.get(id).is_some_and(|entry| entry.key == key))
        }) {
            return;
        }
        // Account for the entry, both indices, allocation slack and key/value
        // buffers. Bounds cover retained storage, not transient working memory.
        let bytes = key.capacity() * 8 + value.bytes() + size_of::<Entry>() + 256;
        let class = value.class();
        let limit = self.class_limit(class);
        if bytes > limit {
            return;
        }
        if retention == RetentionClass::Quantity {
            let (display_count, display_bytes, display_class_bytes) = self
                .entries
                .values()
                .filter(|entry| entry.retention == RetentionClass::Display)
                .fold((0, 0, 0), |(n, b, c), entry| {
                    (
                        n + 1,
                        b + entry.bytes,
                        c + if entry.value.class() == class {
                            entry.bytes
                        } else {
                            0
                        },
                    )
                });
            if display_count + 1 > self.max_entries
                || display_bytes + bytes > self.max_bytes
                || display_class_bytes + bytes > limit
            {
                return;
            }
        }
        while self.class_bytes[class] + bytes > limit {
            let eligible = |id: &&u64| {
                self.entries
                    .get(id)
                    .is_some_and(|entry| entry.value.class() == class)
            };
            let id = self.quantity.iter().find(eligible).copied().or_else(|| {
                (retention == RetentionClass::Display)
                    .then(|| self.display.iter().find(eligible).copied())
                    .flatten()
            });
            let Some(id) = id else {
                return;
            };
            self.remove(id);
        }
        if !self.make_room(bytes, 1, retention) {
            return;
        }
        let Some(next) = self.next.checked_add(1) else {
            return;
        };
        let id = self.next;
        self.next = next;
        self.entries.insert(
            id,
            Entry {
                key,
                value,
                retention,
                bytes,
            },
        );
        self.buckets.entry(bucket_hash).or_default().push(id);
        match retention {
            RetentionClass::Display => self.display.push_back(id),
            RetentionClass::Quantity => self.quantity.push_back(id),
        }
        self.bytes += bytes;
        self.class_bytes[class] += bytes;
    }
}

/// Operation-local ownership prevents nested tessellation from observing a
/// partially captured stage. Drop restores it even after a typed refusal.
pub(super) struct Session {
    cache: Option<Cache>,
    retention: RetentionClass,
}

impl Drop for Session {
    fn drop(&mut self) {
        CACHE.with(|slot| *slot.borrow_mut() = self.cache.take());
    }
}

impl Session {
    pub(super) fn begin(retention: RetentionClass) -> Option<Self> {
        // Measurement already has content-keyed face/volume memos. Its
        // one-shot fine meshes should neither pay for stage capture nor
        // compete with interactive display connectivity.
        if retention == RetentionClass::Quantity {
            return None;
        }
        CACHE.with(|slot| {
            slot.borrow_mut().take().map(|cache| Self {
                cache: Some(cache),
                retention,
            })
        })
    }

    fn get(&mut self, key: &[u64]) -> Option<Value> {
        self.cache.as_mut()?.get(key, self.retention)
    }

    fn insert(&mut self, key: Vec<u64>, value: Value) {
        if let Some(cache) = self.cache.as_mut() {
            cache.insert(key, value, self.retention);
        }
    }

    pub(super) fn boundary(&mut self, key: &[u64], edges: &[usize]) -> Option<BoundaryPlan> {
        self.cache.as_mut()?.counters.boundary_lookups += 1;
        let Value::Boundary(mut plan) = self.get(key)? else {
            return None;
        };
        self.cache.as_mut()?.counters.boundary_hits += 1;
        plan.remap_edges(edges);
        Some(*plan)
    }

    pub(super) fn store_boundary(&mut self, key: Vec<u64>, plan: &BoundaryPlan) {
        if self.cache.as_ref().is_some_and(|cache| {
            key.capacity() * 8 + plan.retained_bytes() + 512 <= cache.class_limit(0)
        }) {
            self.insert(key, Value::Boundary(Box::new(plan.clone())));
        }
    }

    pub(super) fn edge(&mut self, key: &[u64]) -> Option<(Vec<Point3>, Vec<Option<f64>>)> {
        self.cache.as_mut()?.counters.edge_lookups += 1;
        let Value::Edge(points, params) = self.get(key)? else {
            return None;
        };
        self.cache.as_mut()?.counters.edge_hits += 1;
        Some((points, params))
    }

    pub(super) fn store_edge(&mut self, key: Vec<u64>, points: &[Point3], params: &[Option<f64>]) {
        self.insert(key, Value::Edge(points.to_vec(), params.to_vec()));
    }

    pub(super) fn planar(&mut self, key: &[u64]) -> Option<PlanarCdtOutput> {
        self.cache.as_mut()?.counters.planar_lookups += 1;
        let Value::Planar(output) = self.get(key)? else {
            return None;
        };
        self.cache.as_mut()?.counters.planar_hits += 1;
        Some(output)
    }

    pub(super) fn store_planar(&mut self, key: Vec<u64>, output: &PlanarCdtOutput) {
        self.insert(key, Value::Planar(output.clone()));
    }

    pub(super) fn weld(&mut self, indices: &[u32], vertices: usize) -> Option<Vec<u32>> {
        let cache = self.cache.as_mut()?;
        cache.counters.weld_lookups += 1;
        let key = [4, indices.len() as u64, vertices as u64];
        let id = cache.connectivity_probe(&key, self.retention)?;
        let entry = cache.entries.get_mut(&id)?;
        let Value::Weld(state) = &mut entry.value else {
            return None;
        };
        let result = state.update(indices, vertices);
        if result.is_some() {
            cache.counters.weld_hits += 1;
            cache.counters.weld_updated_triangles += state.updated_triangles as u64;
        }
        cache.finish_update(id, result.is_some(), self.retention);
        if result.is_none() {
            cache.insert(key.to_vec(), Value::Seen(false), self.retention);
        }
        result
    }

    pub(super) fn store_weld(
        &mut self,
        indices: &[u32],
        vertices: usize,
        table: super::mesh_ops::HalfEdgeTable,
        missing: &[(u32, u32)],
    ) {
        // The base arrays plus the bounded delta table must fit before any
        // cache-owned copy is allocated. Oversized bodies keep the fresh path.
        let Some(cache) = self.cache.as_ref() else {
            return;
        };
        let key = vec![4, indices.len() as u64, vertices as u64];
        if !cache.capture_ready(&key) {
            return;
        }

        if indices
            .len()
            .saturating_mul(12)
            .saturating_add(vertices.saturating_mul(8))
            .saturating_add(WeldUpdate::delta_limit(indices.len()).saturating_mul(24))
            .saturating_add(512)
            > cache.class_limit(1)
        {
            return;
        }
        if let Some(state) = WeldUpdate::capture(table, indices, vertices, missing)
            && let Some(cache) = self.cache.as_mut()
        {
            cache.replace_marker(key, Value::Weld(Box::new(state)), self.retention);
        }
    }

    pub(super) fn clean(&mut self, indices: &[u32], partition: &[u32]) -> bool {
        let Some(cache) = self.cache.as_mut() else {
            return false;
        };
        cache.counters.cleanup_lookups += 1;
        let key = [5, indices.len() as u64, partition.len() as u64];
        let Some(id) = cache.connectivity_probe(&key, self.retention) else {
            return false;
        };
        let Some(entry) = cache.entries.get_mut(&id) else {
            return false;
        };
        let Value::Clean(state) = &mut entry.value else {
            return false;
        };
        let clean = state.update(indices, partition).is_some();
        if clean {
            cache.counters.cleanup_hits += 1;
            cache.counters.cleanup_updated_triangles += state.updated_triangles as u64;
        }
        cache.finish_update(id, clean, self.retention);
        if !clean {
            cache.insert(key.to_vec(), Value::Seen(false), self.retention);
        }
        clean
    }

    pub(super) fn store_clean(&mut self, indices: &[u32], partition: &[u32]) {
        let Some(cache) = self.cache.as_ref() else {
            return;
        };
        let key = vec![5, indices.len() as u64, partition.len() as u64];
        if !cache.capture_ready(&key) {
            return;
        }

        if indices
            .len()
            .saturating_mul(20)
            .saturating_add(partition.len().saturating_mul(4))
            > cache.class_limit(1)
        {
            return;
        }
        if let Some(state) = DedupeUpdate::capture(indices, partition)
            && let Some(cache) = self.cache.as_mut()
        {
            cache.replace_marker(key, Value::Clean(Box::new(state)), self.retention);
        }
    }
}

/// Full sampling input, including endpoint authority and closed-edge identity.
pub(super) fn edge_key(
    topo: &Topology,
    edge_id: EdgeId,
    deflection: f64,
    angular: f64,
    circle_floor: bool,
) -> Result<Vec<u64>, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let mut key = KeyBuilder::default();
    key.tag(1);
    key.scalar(deflection);
    key.scalar(angular);
    key.flag(circle_floor);
    key.flag(edge.is_closed());
    key.flag(edge.start() == edge.end());
    key.point(topo.vertex(edge.start())?.point());
    key.point(topo.vertex(edge.end())?.point());
    key.curve(edge.curve());
    if let Some((lo, hi)) = edge.trim() {
        key.flag(true);
        key.scalar(lo);
        key.scalar(hi);
    } else {
        key.flag(false);
    }
    let mut words = key.exact();
    words.insert(0, 1);
    Ok(words)
}

/// Lossless boundary input in sorted-edge order. Incidence (including repeated
/// seam uses), directed pcurves, and carrier frames are all part of the key.
/// Handles may differ, but identity and processing order may not.
pub(super) fn boundary_key(
    topo: &Topology,
    faces: &[FaceId],
    edges: &[usize],
    edge_keys: &[Vec<u64>],
    deflection: f64,
    angular: f64,
    circle_floor: bool,
) -> Result<Vec<u64>, crate::OperationsError> {
    let mut key = KeyBuilder::default();
    key.tag(2);
    key.scalar(deflection);
    key.scalar(angular);
    key.flag(circle_floor);
    key.count(edge_keys.len());
    for edge in edge_keys {
        key.count(edge.len());
        for &word in edge {
            key.tag(word);
        }
    }
    key.count(faces.len());
    for &face_id in faces {
        let face = topo.face(face_id)?;
        key.surface(face.surface());
        key.flag(face.is_reversed());
        key.count(face.inner_wires().len() + 1);
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id)?;
            key.flag(wire.is_closed());
            key.count(wire.edges().len());
            for oriented in wire.edges() {
                let ordinal = edges.binary_search(&oriented.edge().index()).map_err(|_| {
                    crate::OperationsError::InvalidInput {
                        reason: "boundary edge is missing from incidence map".into(),
                    }
                })?;
                key.count(ordinal);
                key.flag(oriented.is_forward());
                key.pcurve(topo.pcurve_oriented(oriented.edge(), face_id, oriented.is_forward()));
            }
        }
    }
    let mut words = key.exact();
    words.insert(0, 2);
    Ok(words)
}

pub(super) fn planar_key(points: &[Point2], outer: usize, inner: &[(usize, usize)]) -> Vec<u64> {
    let mut key = vec![3, outer as u64, inner.len() as u64];
    for &(start, end) in inner {
        key.extend([start as u64, end as u64]);
    }
    key.push(points.len() as u64);
    for point in points {
        key.extend([point.x().to_bits(), point.y().to_bits()]);
    }
    key
}

#[cfg(test)]
mod tests;
