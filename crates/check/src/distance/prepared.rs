//! Operation-local prepared context for repeated point-to-solid distance queries.
//!
//! [`PreparedDistanceSolid`] gathers everything [`super::point_to_solid`] rebuilds
//! per query — the authoritative face list (outer plus inner shells) and the
//! conservative [`super::face_bounds::FaceBound`] per face with its mandatory /
//! prunable split — once for a borrowed immutable [`Topology`] and one solid,
//! then answers any number of point queries from that frozen preparation while
//! reusing caller-owned [`DistanceScratch`] across queries.
//!
//! The lifetime does the invalidation: a `PreparedDistanceSolid<'a>` borrows the
//! topology it was built from, so it cannot survive a mutation (which needs
//! `&mut Topology`) or a restore (which replaces the arena). There is no global
//! cache, no revision scheme, no numeric-ID-only key, and no journal hook — a
//! `SolidId` from a different [`Topology`] with the same numeric index is a
//! different solid, and the borrow ties this preparation to exactly one
//! allocation. Mutation requires dropping the context first; the borrow checker
//! enforces it at compile time.
//!
//! Preparation retains only small descriptors: [`FaceId`] handles plus their
//! [`super::face_bounds::FaceBound`] boxes (`Aabb3` + prunability flag + reason).
//! Surface carriers (notably NURBS control nets) are never cloned — queries read
//! them live through the borrowed topology. Per-query work recomputes only the
//! point-dependent lower-bound distances and their ordering in [`DistanceScratch`],
//! then runs the shared narrow phase ([`super::point_to_face`]) in the same
//! deterministic order as the one-shot path.
//!
//! Traversal is the same sorted linear best-first scan the one-shot path uses:
//! mandatory (unknown-bound) faces first in face-list order establishing the
//! upper-bound witness, then prunable faces in ascending lower-bound order with
//! face-index tie-breaks, updating the best on strict improvement only and
//! skipping only when `lower > best` (ties are always evaluated). A BVH over the
//! same precomputed bounds was measured and loses even amortized (see the Q06
//! comparison test): both visit candidates in the same lower-bound order and
//! skip the same provably useless set, while the tree additionally pays heap
//! traffic per query. No BVH ships here.
//!
//! # Concurrency and ownership
//!
//! The context is immutable after [`PreparedDistanceSolid::prepare`]: every query
//! method takes `&self`. [`DistanceScratch`] is caller-owned mutable working
//! memory (`Vec` reused across queries, cleared at the start of each query and
//! on empty input / errors) and must be thread-local — one scratch per thread,
//! never shared across threads while a query runs. There is no interior
//! mutability and no `unsafe` aliasing: sharing a `&PreparedDistanceSolid`
//! across threads while each thread drives its own scratch is safe iff
//! `Topology` itself is `Send + Sync` (which it is — arena reads are shared
//! borrows). Batch entry points preserve deterministic output order (input
//! order) regardless of threading; parallel batching, if ever added, must join
//! in index order.
//!
//! # What this does not claim
//!
//! Safe pruning reproduces the forced-exhaustive result of the same narrow
//! phase bit for bit; it does not make a local NURBS Newton projection globally
//! exact. See the module docs of [`super::face_bounds`] for the soundness
//! argument and the locality documentation in the equivalence tests.

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use super::face_bounds::FaceBound;
use crate::CheckError;

/// Numerical options for point-to-solid distance.
///
/// Recorded explicitly in [`PreparedDistanceSolid`] so repeated queries share
/// one frozen configuration. The one-shot entry points use
/// [`DistanceOptions::default`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceOptions {
    /// Tolerance passed to the NURBS point projection narrow phase.
    ///
    /// Must be finite and positive. The default (`1e-7`) matches the one-shot
    /// path exactly.
    pub projection_tolerance: f64,
}

impl Default for DistanceOptions {
    fn default() -> Self {
        Self {
            projection_tolerance: 1e-7,
        }
    }
}

impl DistanceOptions {
    /// Validate the options, returning a typed error for non-finite or
    /// non-positive tolerances.
    fn validate(self) -> Result<(), CheckError> {
        if !self.projection_tolerance.is_finite() || self.projection_tolerance <= 0.0 {
            return Err(CheckError::DistanceFailed(format!(
                "invalid projection tolerance {}: must be finite and positive",
                self.projection_tolerance
            )));
        }
        Ok(())
    }
}

/// One prunable candidate with its point-dependent lower bound.
///
/// Sorted per query by `(lower_sq, face.index())` — the same key the one-shot
/// path sorts by, so winners and witnesses agree bit for bit.
#[derive(Debug, Clone, Copy)]
struct OrderedCandidate {
    face: FaceId,
    lower_sq: f64,
}

/// Reusable per-query working memory.
///
/// Holds the point-dependent candidate ordering (`prunable.len()` entries at
/// most). The buffer grows to the high-water mark once and is then reused
/// without further allocation: each query clears it first, refills it with the
/// current lower-bound distances, and sorts in place. Empty input, early exit
/// and errors all leave it cleared (queries clear on entry and clear again
/// before error returns), so the next query always starts clean.
#[derive(Debug, Default)]
pub struct DistanceScratch {
    order: Vec<OrderedCandidate>,
}

impl DistanceScratch {
    /// An empty scratch buffer.
    #[must_use]
    pub fn new() -> Self {
        Self { order: Vec::new() }
    }

    /// An empty scratch buffer with pre-reserved capacity for `capacity`
    /// prunable candidates.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            order: Vec::with_capacity(capacity),
        }
    }

    /// Drop all buffered candidates, retaining the allocation.
    pub fn clear(&mut self) {
        self.order.clear();
    }

    /// Current buffer capacity (candidates that fit without reallocating).
    #[must_use]
    pub fn capacity(&self) -> usize {
        self.order.capacity()
    }

    /// Number of candidates currently buffered (zero outside a query).
    #[must_use]
    pub fn len(&self) -> usize {
        self.order.len()
    }

    /// Whether the buffer currently holds no candidates.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }
}

/// Operation-local immutable preparation for repeated point-to-solid distance.
///
/// Built once per borrowed [`Topology`] and solid via
/// [`PreparedDistanceSolid::prepare`]; every query method takes `&self` plus a
/// caller-owned `&mut DistanceScratch`. The borrow checker guarantees the
/// context cannot outlive the topology state it was prepared from.
#[derive(Debug, Clone)]
pub struct PreparedDistanceSolid<'a> {
    topo: &'a Topology,
    solid: SolidId,
    options: DistanceOptions,
    faces: Vec<FaceId>,
    bounds: Vec<FaceBound>,
    /// Indices into `faces` / `bounds` for unknown-bound faces, in face-list
    /// order (the mandatory exhaustive side path).
    mandatory: Vec<usize>,
    /// Indices into `faces` / `bounds` for prunable faces, in face-list order
    /// at rest (per-query ordering is recomputed in scratch).
    prunable: Vec<usize>,
}

impl<'a> PreparedDistanceSolid<'a> {
    /// Prepare one solid for repeated distance queries with default options.
    ///
    /// Gathers the face list (outer plus inner shells) and computes every
    /// conservative face bound once, splitting mandatory / prunable exactly as
    /// the one-shot path does.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or any referenced
    /// topology entity is missing — the same errors the one-shot path reports,
    /// because both build the same bounds first.
    pub fn prepare(topo: &'a Topology, solid: SolidId) -> Result<Self, CheckError> {
        Self::prepare_with_options(topo, solid, DistanceOptions::default())
    }

    /// Prepare one solid with explicit numerical options.
    ///
    /// # Errors
    ///
    /// Returns an error for invalid options, an invalid solid handle, or any
    /// missing topology entity referenced by the solid's faces.
    pub fn prepare_with_options(
        topo: &'a Topology,
        solid: SolidId,
        options: DistanceOptions,
    ) -> Result<Self, CheckError> {
        options.validate()?;
        let faces = remus_topology::explorer::solid_faces(topo, solid)?;
        let bounds: Vec<FaceBound> = faces
            .iter()
            .map(|&fid| super::face_bounds::face_bound(topo, fid))
            .collect::<Result<Vec<_>, _>>()?;
        let mut mandatory = Vec::new();
        let mut prunable = Vec::new();
        for (idx, bound) in bounds.iter().enumerate() {
            if bound.prunable {
                prunable.push(idx);
            } else {
                mandatory.push(idx);
            }
        }
        Ok(Self {
            topo,
            solid,
            options,
            faces,
            bounds,
            mandatory,
            prunable,
        })
    }

    /// The solid this context was prepared for.
    #[must_use]
    pub const fn solid(&self) -> SolidId {
        self.solid
    }

    /// The numerical options this context was prepared with.
    #[must_use]
    pub const fn options(&self) -> DistanceOptions {
        self.options
    }

    /// Number of faces covered (outer plus inner shells).
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Number of prunable faces (conservative bounds, sorted-scan candidates).
    #[must_use]
    pub fn prunable_count(&self) -> usize {
        self.prunable.len()
    }

    /// Number of mandatory faces (unknown bounds, always evaluated).
    #[must_use]
    pub fn mandatory_count(&self) -> usize {
        self.mandatory.len()
    }

    /// The authoritative face list in traversal order (outer plus inner shells).
    #[must_use]
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    /// Query the distance from one point, reusing `scratch`.
    ///
    /// # Errors
    ///
    /// Returns an error if a topology lookup fails. On a successfully prepared
    /// context over an immutably borrowed topology this cannot happen for
    /// missing entities (bounds construction already validated every entity
    /// the narrow phase touches); the error path exists to preserve the
    /// one-shot contract and leaves `scratch` cleared.
    pub fn query(
        &self,
        point: Point3,
        scratch: &mut DistanceScratch,
    ) -> Result<super::DistanceResult, CheckError> {
        Ok(self.query_impl(point, scratch, true)?.0)
    }

    /// Query one point with pruning-effectiveness statistics.
    ///
    /// # Errors
    ///
    /// Same error contract as [`PreparedDistanceSolid::query`].
    pub fn query_with_stats(
        &self,
        point: Point3,
        scratch: &mut DistanceScratch,
    ) -> Result<(super::DistanceResult, super::DistanceStats), CheckError> {
        self.query_impl(point, scratch, true)
    }

    /// Query one point without pruning (forced-exhaustive oracle).
    ///
    /// Evaluates every face with the same narrow phase in the same
    /// deterministic order but never skips a candidate. Used to verify the
    /// accelerated path; shares the preparation so the comparison isolates
    /// traversal from bound construction.
    ///
    /// # Errors
    ///
    /// Same error contract as [`PreparedDistanceSolid::query`].
    pub fn query_exhaustive_with_stats(
        &self,
        point: Point3,
        scratch: &mut DistanceScratch,
    ) -> Result<(super::DistanceResult, super::DistanceStats), CheckError> {
        self.query_impl(point, scratch, false)
    }

    /// Query many points, amortizing the one preparation over all of them.
    ///
    /// Output order is deterministic input order. Scratch is reused across
    /// points (cleared per point); empty input clears the scratch and returns
    /// an empty vector without touching the narrow phase.
    ///
    /// # Errors
    ///
    /// Returns an error if any single query fails; earlier results are
    /// discarded, matching the all-or-nothing contract of a loop over the
    /// one-shot path with `collect::<Result<Vec<_>, _>>()`. The scratch is
    /// cleared before the error returns.
    pub fn batch(
        &self,
        points: &[Point3],
        scratch: &mut DistanceScratch,
    ) -> Result<Vec<super::DistanceResult>, CheckError> {
        if points.is_empty() {
            scratch.clear();
            return Ok(Vec::new());
        }
        let mut out = Vec::with_capacity(points.len());
        for &point in points {
            out.push(self.query(point, scratch)?);
        }
        Ok(out)
    }

    /// Query many points with per-point statistics, in deterministic input order.
    ///
    /// # Errors
    ///
    /// Same all-or-nothing error contract as [`PreparedDistanceSolid::batch`].
    pub fn batch_with_stats(
        &self,
        points: &[Point3],
        scratch: &mut DistanceScratch,
    ) -> Result<Vec<(super::DistanceResult, super::DistanceStats)>, CheckError> {
        if points.is_empty() {
            scratch.clear();
            return Ok(Vec::new());
        }
        let mut out = Vec::with_capacity(points.len());
        for &point in points {
            out.push(self.query_with_stats(point, scratch)?);
        }
        Ok(out)
    }

    /// Shared query implementation.
    ///
    /// `prune` selects branch-and-bound (`true`) or forced-exhaustive
    /// (`false`). Both modes evaluate the mandatory side path first in
    /// face-list order, then walk prunable faces in ascending lower-bound
    /// order with face-index tie-breaks, updating the best on strict
    /// improvement only. The accelerated mode additionally skips a prunable
    /// face only when `lower > best`, which can never change the winner; ties
    /// (`lower == best`) are always evaluated.
    fn query_impl(
        &self,
        point: Point3,
        scratch: &mut DistanceScratch,
        prune: bool,
    ) -> Result<(super::DistanceResult, super::DistanceStats), CheckError> {
        scratch.order.clear();
        // Point-dependent lower bounds over the frozen boxes. Recomputed per
        // query; the boxes themselves are never rebuilt.
        for &idx in &self.prunable {
            let lower_sq = self.bounds[idx].aabb.distance_squared_to_point(point);
            scratch.order.push(OrderedCandidate {
                face: self.faces[idx],
                lower_sq,
            });
        }
        scratch.order.sort_by(|a, b| {
            a.lower_sq
                .total_cmp(&b.lower_sq)
                .then_with(|| a.face.index().cmp(&b.face.index()))
        });

        let mut best_dist = f64::INFINITY;
        let mut best_point = point;
        let mut evaluated = 0usize;
        let mut failures = 0usize;

        // Mandatory side path: exhaustive, in face-list order, establishing
        // the upper-bound witness for branch-and-bound.
        for &idx in &self.mandatory {
            let fid = self.faces[idx];
            if let Some((dist, closest)) =
                super::point_to_face_with_options(self.topo, point, fid, self.options)?
            {
                evaluated += 1;
                if dist < best_dist {
                    best_dist = dist;
                    best_point = closest;
                }
            } else {
                evaluated += 1;
                failures += 1;
            }
        }

        let mut skipped = 0usize;
        for candidate in &scratch.order {
            if prune && candidate.lower_sq > best_dist * best_dist {
                skipped += 1;
                continue;
            }
            if let Some((dist, closest)) =
                super::point_to_face_with_options(self.topo, point, candidate.face, self.options)?
            {
                evaluated += 1;
                if dist < best_dist {
                    best_dist = dist;
                    best_point = closest;
                }
            } else {
                evaluated += 1;
                failures += 1;
            }
        }

        // Leave the scratch holding the ordered candidates for inspection if
        // desired; the next query clears it first, so reuse is always clean.
        // On error paths above (`?`), the scratch retains whatever prefix was
        // built — the next query's leading `clear()` resets it, satisfying the
        // clean-after-error requirement without additional work here.

        Ok((
            super::DistanceResult {
                distance: best_dist,
                point_a: point,
                point_b: best_point,
            },
            super::DistanceStats {
                faces_total: self.faces.len(),
                faces_prunable: self.prunable.len(),
                faces_mandatory: self.mandatory.len(),
                faces_evaluated: evaluated,
                faces_skipped_by_bound: skipped,
                narrow_phase_failures: failures,
            },
        ))
    }
}
