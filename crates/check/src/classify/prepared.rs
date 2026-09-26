//! Operation-local immutable prepared context for repeated point
//! classification (PERF-Q01 substrate).
//!
//! [`PreparedSolid`] gathers everything `classify_point` rebuilds per ray —
//! the face list, conservative face bounds with their BVH, and the per-face
//! trim polygons backing the boundary and crossing containment tests — once
//! for a borrowed immutable [`Topology`] and one solid, then answers any
//! number of point queries from that frozen preparation.
//!
//! The lifetime does the invalidation: a `PreparedSolid<'a>` borrows the
//! topology it was built from, so it cannot survive a mutation (which needs
//! `&mut Topology`) or a restore (which replaces the arena). There is no
//! global cache, no revision scheme, and no journal hook — the persistent,
//! mutation-invalidated cache of O3.2 remains explicitly pending and must be
//! designed separately.

use remus_math::aabb::Aabb3;
use remus_math::bvh::Bvh;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::{ClassifyOptions, ClassifySource, PointClassification};
use crate::CheckError;

/// One face's share of the frozen preparation.
#[derive(Debug, Clone)]
struct PreparedFace {
    /// The face handle (into the borrowed topology).
    fid: FaceId,
    /// Sampled outer/hole polygons backing containment tests. `None` when the
    /// build fails — queries then rebuild on demand with the original error
    /// semantics instead of silently treating the face as untrimmed.
    trim: Option<super::boundary::FaceTrimData>,
}

/// Operation-local immutable preparation for repeated point classification.
///
/// Built once per borrowed [`Topology`] and solid via [`PreparedSolid::prepare`];
/// every query method takes `&self`. The borrow checker guarantees the context
/// cannot outlive the topology state it was prepared from: any `&mut Topology`
/// mutation or arena restore ends the borrow, so stale reuse cannot compile.
///
/// Preparation is conservative by construction: bounds come from the same
/// `face_aabb` the one-shot path filters with, the BVH is the same `Bvh` over
/// the same `(face-index, bound)` pairs, trim polygons come from the same
/// `face_polygon` / `face_hole_polygons` builders, and queries run the shared
/// `classify_point_with_source` vote loop. A prepared query and a
/// one-shot query over the same topology state therefore admit exactly the
/// same candidates and reach exactly the same verdicts.
#[derive(Debug, Clone)]
pub struct PreparedSolid<'a> {
    topo: &'a Topology,
    solid: SolidId,
    faces: Vec<FaceId>,
    /// `(face-index, bound)` pairs for faces whose bounds computed — the same
    /// filtered set the one-shot path hands to `Bvh::build` per ray.
    face_aabbs: Vec<(usize, Aabb3)>,
    bvh: Bvh,
    /// Per-face preparation, index-aligned with [`PreparedSolid::faces`].
    prepared_faces: Vec<PreparedFace>,
}

impl<'a> PreparedSolid<'a> {
    /// Prepare one solid for repeated classification.
    ///
    /// Gathers the face list (outer plus inner shells), computes every
    /// conservative face bound, builds the shared BVH once, and samples every
    /// face's trim polygons once. Faces whose bounds fail are pruned from the
    /// BVH exactly as the one-shot path prunes them; faces whose trim build
    /// fails keep a live on-demand fallback so queries still surface the
    /// original typed error instead of silently mistrimming.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    pub fn prepare(topo: &'a Topology, solid: SolidId) -> Result<Self, CheckError> {
        let faces = remus_topology::explorer::solid_faces(topo, solid)?;

        let mut face_aabbs = Vec::with_capacity(faces.len());
        let mut prepared_faces = Vec::with_capacity(faces.len());
        for (i, &fid) in faces.iter().enumerate() {
            crate::perf::bump_classify_face_aabb_eval();
            if let Ok(bounds) = crate::util::face_aabb(topo, fid) {
                face_aabbs.push((i, bounds));
            }
            let trim = super::boundary::FaceTrimData::build(topo, fid).ok();
            prepared_faces.push(PreparedFace { fid, trim });
        }
        let bvh = Bvh::build(&face_aabbs);
        crate::perf::bump_classify_bvh_build();

        Ok(Self {
            topo,
            solid,
            faces,
            face_aabbs,
            bvh,
            prepared_faces,
        })
    }

    /// The solid this context was prepared for.
    #[must_use]
    pub const fn solid(&self) -> SolidId {
        self.solid
    }

    /// Number of faces covered (outer plus inner shells).
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Number of faces contributing bounds to the shared BVH.
    ///
    /// Faces whose `face_aabb` fails are pruned, exactly as the one-shot
    /// `.ok()` filter prunes them per ray.
    #[must_use]
    pub fn bounded_face_count(&self) -> usize {
        self.face_aabbs.len()
    }

    /// Number of faces whose trim polygons were cached at preparation time.
    ///
    /// The remainder fall back to on-demand builds with the original error
    /// semantics; on a valid solid this is every face.
    #[must_use]
    pub fn cached_trim_count(&self) -> usize {
        self.prepared_faces
            .iter()
            .filter(|face| face.trim.is_some())
            .count()
    }

    /// Classify one point with the shared vote loop over the frozen preparation.
    ///
    /// # Errors
    ///
    /// Returns an error if a topology lookup fails (same typed errors as the
    /// one-shot path).
    pub fn classify_point(
        &self,
        point: Point3,
        options: &ClassifyOptions,
    ) -> Result<PointClassification, CheckError> {
        super::classify_point_with_source(self, point, options)
    }

    /// Classify many points, amortizing the one preparation over all of them.
    ///
    /// # Errors
    ///
    /// Returns an error if any single query fails; earlier results are
    /// discarded, matching the all-or-nothing contract of a loop over the
    /// one-shot path with `collect::<Result<Vec<_>, _>>()`.
    pub fn classify_points(
        &self,
        points: &[Point3],
        options: &ClassifyOptions,
    ) -> Result<Vec<PointClassification>, CheckError> {
        points
            .iter()
            .map(|&point| self.classify_point(point, options))
            .collect()
    }

    /// The same boundary test [`super::classify_point`] applies, over the
    /// frozen trims.
    ///
    /// # Errors
    ///
    /// Returns an error if a topology lookup fails.
    pub fn is_point_on_boundary(&self, point: Point3, tolerance: f64) -> Result<bool, CheckError> {
        for prepared in &self.prepared_faces {
            if super::face_surface_distance(self.topo, prepared.fid, point, tolerance)? < tolerance
            {
                let owned_trim;
                let trim = if let Some(cached) = &prepared.trim {
                    cached
                } else {
                    owned_trim = super::boundary::FaceTrimData::build(self.topo, prepared.fid)?;
                    &owned_trim
                };
                if super::trim_contains_point(trim, point) {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    }

    /// One ray's crossing count over the frozen BVH and trims.
    ///
    /// Candidates come from the same BVH query the one-shot path runs, over a
    /// BVH built from the same pairs, so faces are visited in the same order
    /// and the parity sum admits exactly the same crossings.
    ///
    /// # Errors
    ///
    /// Returns an error if a topology lookup fails.
    pub fn count_ray_crossings(&self, origin: Point3, direction: Vec3) -> Result<u32, CheckError> {
        let candidates = self.bvh.query_ray(origin, direction);

        let mut crossings = 0u32;
        for face_idx in candidates {
            let prepared = &self.prepared_faces[face_idx];
            let owned_trim;
            let trim = if let Some(cached) = &prepared.trim {
                Some(cached)
            } else {
                owned_trim = super::boundary::FaceTrimData::build(self.topo, prepared.fid)?;
                Some(&owned_trim)
            };
            crossings += super::boundary::count_face_ray_crossings_with_trim(
                self.topo,
                prepared.fid,
                trim,
                origin,
                direction,
            )?;
        }
        Ok(crossings)
    }
}

impl ClassifySource for PreparedSolid<'_> {
    fn check_boundary(&self, point: Point3, tolerance: f64) -> Result<bool, CheckError> {
        self.is_point_on_boundary(point, tolerance)
    }

    fn count_crossings(&self, point: Point3, direction: Vec3) -> Result<u32, CheckError> {
        self.count_ray_crossings(point, direction)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use remus_topology::test_utils::make_unit_cube_manifold;

    /// Deterministic low-discrepancy sample in [0, 1).
    fn halton(mut i: u32, base: u32) -> f64 {
        let (mut f, mut r) = (1.0_f64, 0.0_f64);
        while i > 0 {
            f /= f64::from(base);
            r += f * f64::from(i % base);
            i /= base;
        }
        r
    }

    /// Fixed corpus over the unit cube's neighborhood: interior, exterior,
    /// face centres (OnBoundary), near-face offsets on both sides, and edges.
    fn cube_corpus(count: u32) -> Vec<Point3> {
        let mut points = Vec::with_capacity(count as usize + 8);
        for i in 1..=count {
            points.push(Point3::new(
                2.0f64.mul_add(halton(i, 2), -0.5),
                2.0f64.mul_add(halton(i, 3), -0.5),
                2.0f64.mul_add(halton(i, 5), -0.5),
            ));
        }
        // Exact boundary probes the Halton sweep may never land on.
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

    fn classify_one_shot(
        topo: &Topology,
        solid: SolidId,
        points: &[Point3],
        options: &ClassifyOptions,
    ) -> Vec<PointClassification> {
        points
            .iter()
            .map(|&point| super::super::classify_point(topo, solid, point, options).unwrap())
            .collect()
    }

    #[test]
    fn prepared_matches_one_shot_on_cube_corpus() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let corpus = cube_corpus(200);

        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
        let prepared_results = prepared.classify_points(&corpus, &options).unwrap();
        let oneshot_results = classify_one_shot(&topo, solid, &corpus, &options);

        assert_eq!(prepared_results, oneshot_results);
        // The corpus must exercise every verdict, or the equality is vacuous.
        assert!(prepared_results.contains(&PointClassification::Inside));
        assert!(prepared_results.contains(&PointClassification::Outside));
        assert!(prepared_results.contains(&PointClassification::OnBoundary));
    }

    #[test]
    fn boundary_api_matches_one_shot() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();

        for point in cube_corpus(100) {
            for tolerance in [1e-9, 1e-6, 1e-3] {
                let expected =
                    super::super::is_point_on_boundary(&topo, solid, point, tolerance).unwrap();
                assert_eq!(
                    prepared.is_point_on_boundary(point, tolerance).unwrap(),
                    expected,
                    "point {point:?} tolerance {tolerance}"
                );
            }
        }
    }

    #[test]
    fn tolerance_options_match_across_the_boundary_band() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
        // 5e-7 above the z=0 cap: boundary at 1e-6, interior at 1e-9.
        let point = Point3::new(0.5, 0.5, 5e-7);

        for tolerance in [1e-9, 1e-6] {
            let options = ClassifyOptions {
                tolerance,
                ..ClassifyOptions::default()
            };
            assert_eq!(
                prepared.classify_point(point, &options).unwrap(),
                super::super::classify_point(&topo, solid, point, &options).unwrap(),
                "tolerance {tolerance}"
            );
        }
        assert_eq!(
            prepared
                .classify_point(
                    point,
                    &ClassifyOptions {
                        tolerance: 1e-6,
                        ..Default::default()
                    }
                )
                .unwrap(),
            PointClassification::OnBoundary
        );
        assert_eq!(
            prepared
                .classify_point(
                    point,
                    &ClassifyOptions {
                        tolerance: 1e-9,
                        ..Default::default()
                    }
                )
                .unwrap(),
            PointClassification::Inside
        );
    }

    #[test]
    fn invalid_solid_errors_match_one_shot() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        // The same handle against an empty arena: deterministically invalid.
        let empty = Topology::new();

        let prepared_err = PreparedSolid::prepare(&empty, solid).unwrap_err();
        let oneshot_err = super::super::classify_point(
            &empty,
            solid,
            Point3::new(0.5, 0.5, 0.5),
            &ClassifyOptions::default(),
        )
        .unwrap_err();
        assert_eq!(format!("{prepared_err:?}"), format!("{oneshot_err:?}"));
    }

    #[test]
    fn trim_fallback_surfaces_the_original_error() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let fid = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[0];
        // The same face handle against an empty arena: deterministically invalid.
        let empty = Topology::new();
        let origin = Point3::new(0.5, 0.5, 0.5);
        let direction = Vec3::new(0.0, 0.0, 1.0);

        // The prepared on-demand fallback (None trim) reports exactly what the
        // one-shot path reports for the same broken reference.
        let fallback_err = super::super::boundary::count_face_ray_crossings_with_trim(
            &empty, fid, None, origin, direction,
        )
        .unwrap_err();
        let oneshot_err =
            super::super::boundary::count_face_ray_crossings(&empty, fid, origin, direction)
                .unwrap_err();
        assert_eq!(format!("{fallback_err:?}"), format!("{oneshot_err:?}"));

        assert!(
            super::super::boundary::FaceTrimData::build(&empty, fid).is_err(),
            "trim build over a missing face must fail, not silently untrim"
        );
    }

    #[test]
    fn queries_are_deterministic() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let corpus = cube_corpus(100);

        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
        let first = prepared.classify_points(&corpus, &options).unwrap();
        let second = prepared.classify_points(&corpus, &options).unwrap();
        assert_eq!(first, second);

        let prepared_again = PreparedSolid::prepare(&topo, solid).unwrap();
        assert_eq!(
            prepared_again.classify_points(&corpus, &options).unwrap(),
            first
        );
    }

    #[test]
    fn accessors_report_the_frozen_preparation() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();

        assert_eq!(prepared.solid(), solid);
        assert_eq!(prepared.face_count(), 6);
        assert_eq!(prepared.bounded_face_count(), 6);
        assert_eq!(prepared.cached_trim_count(), 6);
    }

    /// Preparation happens once per solid, not once per ray: the one-shot
    /// path rebuilds bounds/BVH per ray (two minimum per point) and trim
    /// polygons per candidate face per ray, while the prepared path builds
    /// each exactly once however many points follow.
    #[cfg(feature = "perf-counters")]
    #[test]
    fn preparation_is_built_once_per_solid() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let options = ClassifyOptions::default();
        let corpus = cube_corpus(50);
        let n = corpus.len() as u64;

        crate::perf::reset();
        let oneshot_results = classify_one_shot(&topo, solid, &corpus, &options);
        let oneshot = crate::perf::snapshot();

        crate::perf::reset();
        let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
        let prepared_results = prepared.classify_points(&corpus, &options).unwrap();
        let prepared_counts = crate::perf::snapshot();

        assert_eq!(prepared_results, oneshot_results);

        // Boundary verdicts return before any ray, so only non-boundary
        // points build BVHs — at least two rays each (early exit needs two
        // agreeing rays; splits and recovery only add more).
        let queried = oneshot_results
            .iter()
            .filter(|result| **result != PointClassification::OnBoundary)
            .count() as u64;
        assert_eq!(prepared_counts.bvh_builds, 1);
        assert!(
            oneshot.bvh_builds >= 2 * queried,
            "one-shot built {} BVHs for {queried} queried points",
            oneshot.bvh_builds
        );
        // Six face bounds evaluated once vs per ray.
        assert_eq!(prepared_counts.face_aabb_evals, 6);
        assert!(
            oneshot.face_aabb_evals >= 2 * queried * 6,
            "one-shot evaluated {} bounds for {queried} queried points",
            oneshot.face_aabb_evals
        );
        // Six trim builds once vs per near/candidate face per query.
        assert_eq!(prepared_counts.trim_builds, 6);
        assert!(
            oneshot.trim_builds > prepared_counts.trim_builds,
            "one-shot built {} trims for {n} points",
            oneshot.trim_builds
        );
    }
}
