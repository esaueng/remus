//! Persistent whole-topology classification cache (PERF-Q02).
//!
//! [`ClassificationCache`] builds on the operation-local [`super::PreparedSolid`]
//! substrate: the same face list, conservative bounds with their shared BVH,
//! and per-face trim polygons, but owned (never borrowed) and keyed by
//! whole-topology [`remus_topology::CacheIdentity`], the queried solid, and
//! all numerical query options.
//!
//! # Identity contract (summary)
//!
//! A cache entry is usable only when its stored
//! [`remus_topology::CacheIdentity`] equals the querying topology's current
//! identity *and* the topology is not poisoned:
//!
//! - Independent [`remus_topology::Topology`] values never share entries:
//!   [`Topology::clone`](remus_topology::Topology::clone) allocates a fresh
//!   lineage, deserialization into an existing value bumps its generation via
//!   ordinary allocation, and a numeric [`remus_topology::solid::SolidId`]
//!   from a different document has a different lineage.
//! - Checkpoint restore, rollback, foreign restore, and `load_journal` never
//!   restore the generation backwards: every restore/rewind path bumps the
//!   generation forward, so two different geometries reaching the same old
//!   journal tick (the ABA case) receive different generations and never
//!   share preparation.
//! - ID retirement never reuses slots, so a retired solid's entry cannot be
//!   mistaken for a later solid with the same index within one lineage; the
//!   generation bump on retirement additionally forces a miss.
//! - Revision overflow poisons instead of wrapping: once the generation would
//!   reach `u64::MAX`, the topology reports [`Topology::is_cache_poisoned`](remus_topology::Topology::is_cache_poisoned)
//!   and every lookup misses without storing.
//!
//! Runtime identity is never written to persistent file formats; arena
//! documents carry entities only.
//!
//! # What is cached and what is not
//!
//! The cache owns derived data only: face handles, conservative bounds, the
//! shared BVH, and sampled trim polygons. Surface carriers (notably NURBS
//! control nets) are never cloned: queries read them live through the passed
//! [`remus_topology::Topology`], which is sound exactly because a hit proves
//! the topology has not changed since preparation. No reference into topology
//! storage is retained across calls.
//!
//! Preparation itself is tolerance-independent (bounds and trims do not take
//! a tolerance), but the key conservatively includes `tolerance` and
//! `max_recovery_attempts` so a future tolerance-dependent preparation cannot
//! alias across options. Different options for the same solid therefore hold
//! duplicate preparations; the bound below keeps that duplication finite.
//!
//! # Bounds
//!
//! The cache holds at most `capacity` preparations (default 16 solids).
//! Eviction is deterministic FIFO over insertion order: the oldest entry
//! leaves first, with no hash-order dependence. Whole-topology invalidation
//! is lazy: entries whose generation no longer matches the querying topology
//! miss, and stale generations for the same lineage are dropped on the next
//! lookup so rolled-back states cannot retain dead preparations.

use remus_math::aabb::Aabb3;
use remus_math::bvh::Bvh;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::{ClassifyOptions, ClassifySource, PointClassification};
use crate::CheckError;

/// Default bound on retained preparations (solids).
pub const DEFAULT_CACHE_CAPACITY: usize = 16;

/// One retained preparation: the same derived data
/// [`super::PreparedSolid`] holds, but owned.
#[derive(Debug, Clone)]
struct CacheEntry {
    lineage: u64,
    generation: u64,
    solid_index: usize,
    tolerance_bits: u64,
    max_recovery: usize,
    faces: Vec<FaceId>,
    bvh: Bvh,
    trims: Vec<Option<super::boundary::FaceTrimData>>,
    retained_bytes: usize,
}

/// Deterministic cache statistics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CacheStats {
    /// Lookups that reused a live preparation.
    pub hits: u64,
    /// Lookups that rebuilt (or errored before rebuilding).
    pub misses: u64,
    /// Successful preparation builds stored.
    pub rebuilds: u64,
    /// Entries evicted (capacity FIFO plus stale-generation drops).
    pub evictions: u64,
    /// Preparations currently retained.
    pub len: usize,
    /// Bound on retained preparations.
    pub capacity: usize,
    /// Estimated retained derived bytes (not RSS).
    pub retained_bytes: usize,
}

/// Persistent whole-topology cache for repeated point classification.
///
/// Owns bounded derived preparation; every query takes `&Topology` and
/// checks identity before reuse. Misses rebuild exactly what
/// [`super::PreparedSolid::prepare`] builds, then run the shared
/// `super::classify_point_with_source` vote loop so verdicts match the
/// one-shot and borrowed-prepared paths bit for bit.
#[derive(Debug, Clone)]
pub struct ClassificationCache {
    capacity: usize,
    entries: Vec<CacheEntry>,
    hits: u64,
    misses: u64,
    rebuilds: u64,
    evictions: u64,
}

impl Default for ClassificationCache {
    fn default() -> Self {
        Self::new()
    }
}

impl ClassificationCache {
    /// An empty cache with [`DEFAULT_CACHE_CAPACITY`] slots.
    #[must_use]
    pub fn new() -> Self {
        Self::with_capacity(DEFAULT_CACHE_CAPACITY)
    }

    /// An empty cache holding at most `capacity` preparations.
    ///
    /// `capacity == 0` disables retention: every lookup misses without
    /// storing. Eviction order is deterministic FIFO regardless of capacity.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            capacity,
            entries: Vec::new(),
            hits: 0,
            misses: 0,
            rebuilds: 0,
            evictions: 0,
        }
    }

    /// Bound on retained preparations.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    /// Preparations currently retained.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether no preparation is retained.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Drops every retained preparation and preserves counters.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Deterministic statistics snapshot.
    #[must_use]
    pub fn stats(&self) -> CacheStats {
        CacheStats {
            hits: self.hits,
            misses: self.misses,
            rebuilds: self.rebuilds,
            evictions: self.evictions,
            len: self.entries.len(),
            capacity: self.capacity,
            retained_bytes: self.retained_bytes(),
        }
    }

    /// Estimated retained derived bytes (owned faces, bounds, BVH nodes,
    /// and trim points; not allocator overhead or RSS).
    #[must_use]
    pub fn retained_bytes(&self) -> usize {
        self.entries.iter().map(|entry| entry.retained_bytes).sum()
    }

    /// Hit rate over all lookups (`hits / (hits + misses)`), or `None`
    /// before the first lookup.
    #[must_use]
    pub fn hit_rate(&self) -> Option<f64> {
        let total = self.hits.saturating_add(self.misses);
        if total == 0 {
            None
        } else {
            #[allow(clippy::cast_precision_loss)]
            Some(self.hits as f64 / total as f64)
        }
    }

    /// Classify one point, reusing or rebuilding preparation as needed.
    ///
    /// # Errors
    ///
    /// Returns the same typed errors as [`super::classify_point`]: an
    /// invalid solid or missing topology entity never returns a stale
    /// verdict. Errors are never cached.
    pub fn classify_point(
        &mut self,
        topo: &Topology,
        solid: SolidId,
        point: Point3,
        options: &ClassifyOptions,
    ) -> Result<PointClassification, CheckError> {
        if topo.is_cache_poisoned() || self.capacity == 0 {
            self.misses = self.misses.saturating_add(1);
            return super::classify_point(topo, solid, point, options);
        }
        let index = self.entry_index_or_rebuild(topo, solid, options)?;
        let entry = &self.entries[index];
        let source = CachedSource {
            topo,
            faces: &entry.faces,
            bvh: &entry.bvh,
            trims: &entry.trims,
        };
        super::classify_point_with_source(&source, point, options)
    }

    /// Classify many points, amortizing one lookup over the batch.
    ///
    /// Output order is deterministic input order. A single preparation
    /// serves the whole batch; per-point verdicts still match a loop over
    /// [`super::classify_point`].
    ///
    /// # Errors
    ///
    /// Returns an error if preparation fails or any single query fails;
    /// earlier results are discarded, matching the all-or-nothing contract
    /// of a loop collected with `collect::<Result<Vec<_>, _>>()`.
    pub fn classify_points(
        &mut self,
        topo: &Topology,
        solid: SolidId,
        points: &[Point3],
        options: &ClassifyOptions,
    ) -> Result<Vec<PointClassification>, CheckError> {
        if topo.is_cache_poisoned() || self.capacity == 0 {
            self.misses = self.misses.saturating_add(1);
            let prepared = super::PreparedSolid::prepare(topo, solid)?;
            return prepared.classify_points(points, options);
        }
        let index = self.entry_index_or_rebuild(topo, solid, options)?;
        let entry = &self.entries[index];
        let source = CachedSource {
            topo,
            faces: &entry.faces,
            bvh: &entry.bvh,
            trims: &entry.trims,
        };
        points
            .iter()
            .map(|&point| super::classify_point_with_source(&source, point, options))
            .collect()
    }

    /// Finds a live entry or rebuilds it, returning its index.
    ///
    /// Drops stale generations for the querying lineage before searching so
    /// rolled-back states cannot accumulate dead preparations. Callers handle
    /// poisoned and zero-capacity cases before reaching here. Invalid solids
    /// error without storing.
    fn entry_index_or_rebuild(
        &mut self,
        topo: &Topology,
        solid: SolidId,
        options: &ClassifyOptions,
    ) -> Result<usize, CheckError> {
        let identity = topo.cache_identity();
        // Validate the handle first so malformed topology preserves the
        // one-shot error instead of a cache verdict.
        let _ = topo.solid(solid)?;

        self.drop_stale_for_lineage(identity.lineage, identity.generation);
        let tolerance_bits = options.tolerance.to_bits();
        if let Some(index) = self.entries.iter().position(|entry| {
            entry.lineage == identity.lineage
                && entry.generation == identity.generation
                && entry.solid_index == solid.index()
                && entry.tolerance_bits == tolerance_bits
                && entry.max_recovery == options.max_recovery_attempts
        }) {
            self.hits = self.hits.saturating_add(1);
            return Ok(index);
        }
        self.misses = self.misses.saturating_add(1);
        // Build exactly what `PreparedSolid::prepare` builds, owning every
        // derived vector. Failures return without storing.
        let faces = remus_topology::explorer::solid_faces(topo, solid)?;
        let mut face_aabbs = Vec::with_capacity(faces.len());
        let mut trims = Vec::with_capacity(faces.len());
        for (i, &fid) in faces.iter().enumerate() {
            crate::perf::bump_classify_face_aabb_eval();
            if let Ok(bounds) = crate::util::face_aabb(topo, fid) {
                face_aabbs.push((i, bounds));
            }
            let trim = super::boundary::FaceTrimData::build(topo, fid).ok();
            trims.push(trim);
        }
        let bvh = Bvh::build(&face_aabbs);
        crate::perf::bump_classify_bvh_build();

        let retained_bytes = estimate_retained_bytes(&faces, &face_aabbs, &bvh, &trims);
        // FIFO: evict the oldest insertion when full. Deterministic; no
        // hash-order dependence.
        if self.entries.len() >= self.capacity {
            self.entries.remove(0);
            self.evictions = self.evictions.saturating_add(1);
        }
        self.entries.push(CacheEntry {
            lineage: identity.lineage,
            generation: identity.generation,
            solid_index: solid.index(),
            tolerance_bits,
            max_recovery: options.max_recovery_attempts,
            faces,
            bvh,
            trims,
            retained_bytes,
        });
        self.rebuilds = self.rebuilds.saturating_add(1);
        Ok(self.entries.len() - 1)
    }

    /// Drops entries for `lineage` whose generation is stale.
    ///
    /// Entries for other lineages (other documents) are kept: they may still
    /// serve their own topology. Stale generations for the querying lineage
    /// can never hit again, so retaining them would only pin dead
    /// preparations.
    fn drop_stale_for_lineage(&mut self, lineage: u64, generation: u64) {
        let mut kept = Vec::with_capacity(self.entries.len());
        for entry in self.entries.drain(..) {
            if entry.lineage == lineage && entry.generation != generation {
                self.evictions = self.evictions.saturating_add(1);
            } else {
                kept.push(entry);
            }
        }
        self.entries = kept;
    }
}

/// Borrowed view over one owned preparation plus live carriers.
struct CachedSource<'a> {
    topo: &'a Topology,
    faces: &'a [FaceId],
    bvh: &'a Bvh,
    trims: &'a [Option<super::boundary::FaceTrimData>],
}

impl ClassifySource for CachedSource<'_> {
    fn check_boundary(&self, point: Point3, tolerance: f64) -> Result<bool, CheckError> {
        for (fid, trim) in self.faces.iter().zip(self.trims.iter()) {
            if super::face_surface_distance(self.topo, *fid, point, tolerance)? < tolerance {
                let owned_trim;
                let trim_ref = if let Some(cached) = trim {
                    cached
                } else {
                    owned_trim = super::boundary::FaceTrimData::build(self.topo, *fid)?;
                    &owned_trim
                };
                if super::trim_contains_point(trim_ref, point) {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    fn count_crossings(&self, point: Point3, direction: Vec3) -> Result<u32, CheckError> {
        let candidates = self.bvh.query_ray(point, direction);
        let mut crossings = 0u32;
        for face_idx in candidates {
            let fid = self.faces[face_idx];
            let owned_trim;
            let trim = if let Some(cached) = &self.trims[face_idx] {
                Some(cached)
            } else {
                owned_trim = super::boundary::FaceTrimData::build(self.topo, fid)?;
                Some(&owned_trim)
            };
            crossings += super::boundary::count_face_ray_crossings_with_trim(
                self.topo, fid, trim, point, direction,
            )?;
        }
        Ok(crossings)
    }
}

/// Estimated retained bytes for one preparation.
///
/// Sums owned face handles, bound pairs, BVH nodes (two per bound, the
/// construction upper bound), and trim points. Deterministic; not RSS.
fn estimate_retained_bytes(
    faces: &[FaceId],
    face_aabbs: &[(usize, Aabb3)],
    bvh: &Bvh,
    trims: &[Option<super::boundary::FaceTrimData>],
) -> usize {
    use std::mem::size_of;
    let _ = bvh;
    let faces_bytes = faces.len().saturating_mul(size_of::<FaceId>());
    let bounds_bytes = face_aabbs
        .len()
        .saturating_mul(size_of::<usize>().saturating_add(size_of::<Aabb3>()));
    // `Bvh` is a flat `Vec<BvhNode>`; its length is private, so bound by the
    // construction cap (at most two nodes per bound) times the node size.
    let bvh_bytes = face_aabbs
        .len()
        .saturating_mul(2)
        .saturating_mul(size_of::<Aabb3>().saturating_add(3 * size_of::<usize>()));
    let mut trim_bytes = 0usize;
    for data in trims.iter().flatten() {
        trim_bytes =
            trim_bytes.saturating_add(data.outer.len().saturating_mul(size_of::<Point3>()));
        for hole in &data.holes {
            trim_bytes = trim_bytes.saturating_add(hole.len().saturating_mul(size_of::<Point3>()));
        }
    }
    faces_bytes
        .saturating_add(bounds_bytes)
        .saturating_add(bvh_bytes)
        .saturating_add(trim_bytes)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use remus_topology::test_utils::make_unit_cube_manifold;

    fn halton(mut i: u32, base: u32) -> f64 {
        let (mut f, mut r) = (1.0_f64, 0.0_f64);
        while i > 0 {
            f /= f64::from(base);
            r += f * f64::from(i % base);
            i /= base;
        }
        r
    }

    fn cube_corpus(count: u32) -> Vec<Point3> {
        let mut points = Vec::with_capacity(count as usize + 8);
        for i in 1..=count {
            points.push(Point3::new(
                2.0f64.mul_add(halton(i, 2), -0.5),
                2.0f64.mul_add(halton(i, 3), -0.5),
                2.0f64.mul_add(halton(i, 5), -0.5),
            ));
        }
        points.extend([
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(0.5, 0.5, 1.0),
            Point3::new(0.0, 0.5, 0.5),
            Point3::new(0.5, 0.5, 1e-9),
            Point3::new(0.5, 0.5, 1.0 + 1e-9),
            Point3::new(0.5, 0.5, -1e-9),
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.5, 0.5),
        ]);
        points
    }

    fn oneshot(
        topo: &Topology,
        solid: SolidId,
        points: &[Point3],
        options: &ClassifyOptions,
    ) -> Vec<PointClassification> {
        points
            .iter()
            .map(|&p| super::super::classify_point(topo, solid, p, options).unwrap())
            .collect()
    }

    fn translate_solid_x(topo: &mut Topology, solid: SolidId, dx: f64) {
        let verts = remus_topology::explorer::solid_vertices(topo, solid).unwrap();
        for vid in verts {
            let p = topo.vertex(vid).unwrap().point();
            topo.vertex_mut(vid)
                .unwrap()
                .set_point(Point3::new(p.x() + dx, p.y(), p.z()));
        }
    }

    #[test]
    fn cache_matches_one_shot_and_prepared_on_corpus() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let corpus = cube_corpus(200);

        let mut cache = ClassificationCache::new();
        // Single-point loop: first miss, rest hits.
        let mut cached = Vec::with_capacity(corpus.len());
        for &p in &corpus {
            cached.push(cache.classify_point(&topo, solid, p, &options).unwrap());
        }
        let expected = oneshot(&topo, solid, &corpus, &options);
        assert_eq!(cached, expected);
        assert!(cached.contains(&PointClassification::Inside));
        assert!(cached.contains(&PointClassification::Outside));
        assert!(cached.contains(&PointClassification::OnBoundary));

        let prepared = super::super::PreparedSolid::prepare(&topo, solid).unwrap();
        assert_eq!(
            prepared.classify_points(&corpus, &options).unwrap(),
            expected
        );

        let stats = cache.stats();
        assert_eq!(stats.rebuilds, 1);
        assert_eq!(stats.hits as usize + 1, corpus.len());
        assert_eq!(stats.misses, 1);
        assert_eq!(stats.len, 1);

        // Batch path shares one preparation and matches.
        let mut cache2 = ClassificationCache::new();
        let batched = cache2
            .classify_points(&topo, solid, &corpus, &options)
            .unwrap();
        assert_eq!(batched, expected);
        assert_eq!(cache2.stats().rebuilds, 1);
        assert_eq!(cache2.stats().misses, 1);
    }

    #[test]
    fn mutate_invalidates_and_tracks_new_geometry() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let inside = Point3::new(0.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        assert_eq!(
            cache
                .classify_point(&topo, solid, inside, &options)
                .unwrap(),
            PointClassification::Inside
        );
        assert_eq!(cache.stats().rebuilds, 1);

        // Translate the whole cube +10 in X: the old interior point is now
        // outside (analytic truth: box is [10,11]^3).
        translate_solid_x(&mut topo, solid, 10.0);
        let verdict = cache
            .classify_point(&topo, solid, inside, &options)
            .unwrap();
        assert_eq!(verdict, PointClassification::Outside);
        assert_eq!(
            verdict,
            super::super::classify_point(&topo, solid, inside, &options).unwrap()
        );
        let stats = cache.stats();
        assert_eq!(stats.rebuilds, 2);
        assert_eq!(stats.misses, 2);
        // Stale generation dropped: only the live preparation remains.
        assert_eq!(stats.len, 1);

        // The moved interior point classifies Inside in both paths.
        let moved_inside = Point3::new(10.5, 0.5, 0.5);
        assert_eq!(
            cache
                .classify_point(&topo, solid, moved_inside, &options)
                .unwrap(),
            PointClassification::Inside
        );
    }

    #[test]
    fn direct_mutable_access_without_write_still_invalidates() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        assert_eq!(
            cache.classify_point(&topo, solid, point, &options).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(cache.stats().rebuilds, 1);

        // Acquire an exclusive reference and drop it without writing.
        // Conservative invalidation fires before the reference escapes.
        let vid = remus_topology::explorer::solid_vertices(&topo, solid).unwrap()[0];
        let _ = topo.vertex_mut(vid).unwrap();
        let verdict = cache.classify_point(&topo, solid, point, &options).unwrap();
        assert_eq!(verdict, PointClassification::Inside);
        assert_eq!(
            verdict,
            super::super::classify_point(&topo, solid, point, &options).unwrap()
        );
        assert_eq!(cache.stats().rebuilds, 2);
        assert_eq!(cache.stats().misses, 2);
    }

    #[test]
    fn failed_edit_rolls_back_and_invalidates() {
        use remus_topology::transaction::run_transacted;
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        assert_eq!(
            cache.classify_point(&topo, solid, point, &options).unwrap(),
            PointClassification::Inside
        );

        let live_before = topo.num_vertices();
        let err = run_transacted(
            &mut topo,
            |topo| -> Result<(), remus_topology::TopologyError> {
                translate_solid_x(topo, solid, 10.0);
                Err(remus_topology::TopologyError::WireNotClosed)
            },
        )
        .unwrap_err();
        assert!(matches!(err, remus_topology::TopologyError::WireNotClosed));
        assert_eq!(topo.num_vertices(), live_before);

        // Rollback rewound the tick but bumped the cache generation:
        // miss, correct original verdict, no stale reuse.
        let verdict = cache.classify_point(&topo, solid, point, &options).unwrap();
        assert_eq!(verdict, PointClassification::Inside);
        assert_eq!(
            verdict,
            super::super::classify_point(&topo, solid, point, &options).unwrap()
        );
        assert_eq!(cache.stats().rebuilds, 2);
    }

    #[test]
    fn nested_rollback_invalidates_each_level() {
        use remus_topology::transaction::RollbackSnapshot;
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let origin_inside = Point3::new(0.5, 0.5, 0.5);
        let moved_inside = Point3::new(10.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        assert_eq!(
            cache
                .classify_point(&topo, solid, origin_inside, &options)
                .unwrap(),
            PointClassification::Inside
        );

        let outer = RollbackSnapshot::capture(&mut topo);
        translate_solid_x(&mut topo, solid, 10.0);
        assert_eq!(
            cache
                .classify_point(&topo, solid, moved_inside, &options)
                .unwrap(),
            PointClassification::Inside
        );

        let inner = RollbackSnapshot::capture(&mut topo);
        translate_solid_x(&mut topo, solid, 10.0);
        inner.restore(&mut topo);
        // Inner rewind invalidates: back to the outer-mutated state.
        assert_eq!(
            cache
                .classify_point(&topo, solid, moved_inside, &options)
                .unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            cache
                .classify_point(&topo, solid, moved_inside, &options)
                .unwrap(),
            super::super::classify_point(&topo, solid, moved_inside, &options).unwrap()
        );

        outer.restore(&mut topo);
        // Outer rewind invalidates back to the original.
        assert_eq!(
            cache
                .classify_point(&topo, solid, origin_inside, &options)
                .unwrap(),
            PointClassification::Inside
        );
        assert!(cache.stats().rebuilds >= 4);
    }

    #[test]
    fn checkpoint_restore_and_rollback_restore_invalidate() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        assert_eq!(
            cache.classify_point(&topo, solid, point, &options).unwrap(),
            PointClassification::Inside
        );

        let checkpoint = topo.clone();
        translate_solid_x(&mut topo, solid, 10.0);
        assert_eq!(
            cache.classify_point(&topo, solid, point, &options).unwrap(),
            PointClassification::Outside
        );

        topo.restore_preserving_handle_slots(&checkpoint);
        let verdict = cache.classify_point(&topo, solid, point, &options).unwrap();
        assert_eq!(verdict, PointClassification::Inside);
        assert_eq!(
            verdict,
            super::super::classify_point(&topo, solid, point, &options).unwrap()
        );

        // Rollback-barrier restore invalidates the same way.
        let snapshot = topo.clone();
        translate_solid_x(&mut topo, solid, 10.0);
        topo.restore_for_rollback(&snapshot);
        let verdict = cache.classify_point(&topo, solid, point, &options).unwrap();
        assert_eq!(verdict, PointClassification::Inside);
    }

    #[test]
    fn changed_clone_restore_never_reuses_stale_preparation() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let origin = Point3::new(0.5, 0.5, 0.5);
        let moved = Point3::new(10.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        assert_eq!(
            cache
                .classify_point(&topo, solid, origin, &options)
                .unwrap(),
            PointClassification::Inside
        );

        let mut changed = topo.clone();
        let changed_solid = changed.solid_id_from_index(solid.index()).unwrap();
        translate_solid_x(&mut changed, changed_solid, 10.0);
        topo.restore_preserving_handle_slots(&changed);

        // Destination keeps its lineage but bumps generation: miss, and the
        // verdict matches the changed geometry, not the stale preparation.
        assert_eq!(
            cache.classify_point(&topo, solid, moved, &options).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            cache.classify_point(&topo, solid, moved, &options).unwrap(),
            super::super::classify_point(&topo, solid, moved, &options).unwrap()
        );
        assert_eq!(
            cache
                .classify_point(&topo, solid, origin, &options)
                .unwrap(),
            PointClassification::Outside
        );
    }

    #[test]
    fn same_id_different_document_never_shares() {
        let mut first = Topology::new();
        let first_solid = make_unit_cube_manifold(&mut first);
        let mut second = Topology::new();
        let second_solid = make_unit_cube_manifold(&mut second);
        translate_solid_x(&mut second, second_solid, 10.0);
        assert_eq!(first_solid.index(), second_solid.index());

        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);
        let mut cache = ClassificationCache::new();

        assert_eq!(
            cache
                .classify_point(&first, first_solid, point, &options)
                .unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            cache
                .classify_point(&second, second_solid, point, &options)
                .unwrap(),
            PointClassification::Outside
        );
        // Both lineages retained; re-querying the first still hits.
        assert_eq!(
            cache
                .classify_point(&first, first_solid, point, &options)
                .unwrap(),
            PointClassification::Inside
        );
        assert_eq!(cache.stats().len, 2);
        assert_eq!(cache.stats().hits, 1);
    }

    #[test]
    fn eviction_is_deterministic_fifo() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold(&mut topo);
        let b = make_unit_cube_manifold(&mut topo);
        let c = make_unit_cube_manifold(&mut topo);
        translate_solid_x(&mut topo, b, 10.0);
        translate_solid_x(&mut topo, c, 20.0);
        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);

        let mut cache = ClassificationCache::with_capacity(2);
        assert_eq!(
            cache.classify_point(&topo, a, point, &options).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            cache.classify_point(&topo, b, point, &options).unwrap(),
            PointClassification::Outside
        );
        assert_eq!(cache.stats().len, 2);
        // Third solid evicts the oldest (a).
        assert_eq!(
            cache.classify_point(&topo, c, point, &options).unwrap(),
            PointClassification::Outside
        );
        assert_eq!(cache.stats().len, 2);
        assert_eq!(cache.stats().evictions, 1);

        // `a` misses again and evicts `b` (FIFO order preserved).
        assert_eq!(
            cache.classify_point(&topo, a, point, &options).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(cache.stats().evictions, 2);
        // `c` is still retained, so it hits.
        let hits_before = cache.stats().hits;
        assert_eq!(
            cache.classify_point(&topo, c, point, &options).unwrap(),
            PointClassification::Outside
        );
        assert_eq!(cache.stats().hits, hits_before + 1);
    }

    #[test]
    fn aba_same_tick_different_geometry_never_shares() {
        // One topology value, one lineage: A -> B -> restore A -> C, where B
        // and C are symmetric opposite translations reaching the identical
        // journal tick but different cache generations.
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let b_inside = Point3::new(10.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        let lineage = topo.cache_lineage();

        // Baseline A: origin interior.
        assert_eq!(
            cache
                .classify_point(&topo, solid, Point3::new(0.5, 0.5, 0.5), &options)
                .unwrap(),
            PointClassification::Inside
        );
        let snapshot = topo.clone();
        let ticks_a = topo.mutation_ticks();

        // Geometry B: +10 in X. `b_inside` is interior.
        translate_solid_x(&mut topo, solid, 10.0);
        assert_eq!(
            cache
                .classify_point(&topo, solid, b_inside, &options)
                .unwrap(),
            PointClassification::Inside
        );
        let ticks_b = topo.mutation_ticks();
        let gen_b = topo.cache_generation();
        assert_eq!(topo.cache_lineage(), lineage);

        // Restore to A: ticks rewind, generation moves forward.
        topo.restore_preserving_handle_slots(&snapshot);
        assert_eq!(topo.mutation_ticks(), ticks_a);
        assert_eq!(topo.cache_lineage(), lineage);
        assert_ne!(topo.cache_generation(), gen_b);

        // Geometry C: -10 in X from restored A. Same overwrite count as B,
        // so the tick matches B exactly; the generation cannot.
        translate_solid_x(&mut topo, solid, -10.0);
        assert_eq!(
            topo.mutation_ticks(),
            ticks_b,
            "ABA setup requires identical journal ticks"
        );
        assert_eq!(topo.cache_lineage(), lineage);
        assert_ne!(
            topo.cache_generation(),
            gen_b,
            "cache generations must differ across the restore"
        );
        // Must not reuse B's preparation: B says Inside, C says Outside.
        let verdict = cache
            .classify_point(&topo, solid, b_inside, &options)
            .unwrap();
        assert_eq!(verdict, PointClassification::Outside);
        assert_eq!(
            verdict,
            super::super::classify_point(&topo, solid, b_inside, &options).unwrap()
        );
    }

    #[test]
    fn malformed_topology_preserves_errors_without_caching() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let empty = Topology::new();
        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);

        let mut cache = ClassificationCache::new();
        let expected_err =
            super::super::classify_point(&empty, solid, point, &options).unwrap_err();
        let cache_err = cache
            .classify_point(&empty, solid, point, &options)
            .unwrap_err();
        assert_eq!(format!("{cache_err:?}"), format!("{expected_err:?}"));
        assert_eq!(cache.stats().len, 0);
        // A second malformed query still errors (miss, never a stale hit).
        let cache_err2 = cache
            .classify_point(&empty, solid, point, &options)
            .unwrap_err();
        assert_eq!(format!("{cache_err2:?}"), format!("{expected_err:?}"));
        assert_eq!(cache.stats().hits, 0);
    }

    #[test]
    fn numerical_options_are_part_of_the_key() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        // 5e-7 above the z=0 cap: boundary at 1e-6, interior at 1e-9.
        let point = Point3::new(0.5, 0.5, 5e-7);

        let mut cache = ClassificationCache::new();
        let tight = ClassifyOptions {
            tolerance: 1e-9,
            ..Default::default()
        };
        let loose = ClassifyOptions {
            tolerance: 1e-6,
            ..Default::default()
        };
        assert_eq!(
            cache.classify_point(&topo, solid, point, &tight).unwrap(),
            PointClassification::Inside
        );
        assert_eq!(
            cache.classify_point(&topo, solid, point, &loose).unwrap(),
            PointClassification::OnBoundary
        );
        assert_eq!(
            cache.classify_point(&topo, solid, point, &tight).unwrap(),
            super::super::classify_point(&topo, solid, point, &tight).unwrap()
        );
        // Conservative keying holds duplicate preparations.
        assert_eq!(cache.stats().len, 2);

        let other_recovery = ClassifyOptions {
            max_recovery_attempts: 3,
            ..Default::default()
        };
        assert_eq!(
            cache
                .classify_point(&topo, solid, point, &other_recovery)
                .unwrap(),
            super::super::classify_point(&topo, solid, point, &other_recovery).unwrap()
        );
        assert_eq!(cache.stats().len, 3);
    }

    #[test]
    fn zero_capacity_never_stores_but_stays_correct() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let corpus = cube_corpus(50);
        let mut cache = ClassificationCache::with_capacity(0);
        let cached = cache
            .classify_points(&topo, solid, &corpus, &options)
            .unwrap();
        assert_eq!(cached, oneshot(&topo, solid, &corpus, &options));
        assert_eq!(cache.stats().len, 0);
        assert_eq!(cache.stats().rebuilds, 0);
    }

    #[test]
    fn retained_bytes_are_bounded_and_deterministic() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold(&mut topo);
        let b = make_unit_cube_manifold(&mut topo);
        let c = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let point = Point3::new(0.5, 0.5, 0.5);

        // All solids exist before any query: no mutation between lookups, so
        // eviction (not whole-topology invalidation) bounds retention.
        let mut cache = ClassificationCache::with_capacity(2);
        cache.classify_point(&topo, a, point, &options).unwrap();
        let bytes_one = cache.retained_bytes();
        assert!(bytes_one > 0);
        cache.classify_point(&topo, b, point, &options).unwrap();
        let bytes_two = cache.retained_bytes();
        assert!(bytes_two > bytes_one);

        // A third solid evicts: retained bytes drop back to two entries'
        // worth, never accumulating three.
        cache.classify_point(&topo, c, point, &options).unwrap();
        assert_eq!(cache.stats().len, 2);
        assert!(cache.retained_bytes() <= bytes_two);

        // Rebuilding the same state yields identical bytes.
        let mut cache2 = ClassificationCache::with_capacity(2);
        cache2.classify_point(&topo, a, point, &options).unwrap();
        cache2.classify_point(&topo, b, point, &options).unwrap();
        assert_eq!(cache2.retained_bytes(), bytes_two);
    }
}
