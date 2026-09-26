//! Conservative spatial index for coincident-vertex candidate discovery.
//!
//! Both [`merge_coincident_vertices`](super::solid::fix_solid) in
//! `fix/solid.rs` and `remus-operations`' `heal::merge_coincident_vertices`
//! discovered merge candidates with an all-pairs scan: every vertex pair paid
//! an exact distance evaluation, so sparse models scaled quadratically
//! (PERF-H01: 9k sparse vertices cost ~300 ms, ~99% of it pair checks).
//!
//! [`plan_coincident_merges`] replaces only candidate *discovery*. The exact
//! compatibility predicate (`dist² < tol²`, strict), the global tolerance
//! policy, the canonical representative (earliest vertex in discovery order),
//! deterministic processing order, and the nontransitive-nearness semantics
//! (a merged vertex never becomes a merge source, so A~B and B~C never imply
//! A~C) are all preserved bit-for-bit: on any input the plan equals the
//! all-pairs result, including the number of exact distance evaluations it
//! *would* have needed on sparse inputs minus the pruned far pairs.
//!
//! ## Method
//!
//! Vertices are bucketed into a uniform grid with cell size `tol`. When
//! `dist < tol`, per-axis `|d| < tol` forces the floored cell coordinates to
//! differ by at most one per axis, so every eligible pair shares a
//! 27-neighborhood. The index is therefore *conservative*: it can propose
//! extra nearby pairs, but never misses an eligible one. Every candidate is
//! re-checked with the exact predicate before merging.
//!
//! Processing is streaming in `j` ascending order with an in-neighborhood
//! k-way scan in ascending `i` order and early exit once `j` merges. This is
//! equivalent to the legacy `i`-outer all-pairs order: a merge decision for
//! `j` only reads `merged[i]` for `i < j` (final by the time `j` runs, since
//! `i`'s own fate was decided at step `i`) and writes only `merged[j]`, so
//! cross-`j` visit interleaving cannot change the outcome. Streaming keeps
//! memory linear even for dense inputs where a materialized candidate list
//! would hold O(n²) pairs.
//!
//! Degenerate inputs (non-finite or non-positive tolerance, non-finite
//! coordinates, coordinates beyond the exact-integer grid range) fall back to
//! the exact all-pairs loop with identical order and predicate, flagged by
//! [`VertexMergePlan::fell_back_to_all_pairs`].

use remus_math::det_hash::DetHashMap;
use remus_math::vec::Point3;

/// Cell coordinate in the uniform merge grid.
type Cell = (i64, i64, i64);

/// Largest exactly-representable grid coordinate (`2^53`).
const MAX_EXACT_CELL: f64 = 9_007_199_254_740_992.0;

/// Result of [`plan_coincident_merges`].
#[derive(Debug, Clone)]
pub struct VertexMergePlan {
    /// `merge_target[j]` is `Some(i)` (`i < j`) when vertex `j` merges into
    /// the earliest eligible vertex `i`, else `None`.
    pub merge_target: Vec<Option<usize>>,
    /// Number of vertices that merge (disclosed repair count).
    pub merged_count: usize,
    /// Exact distance evaluations performed.
    pub distance_checks: u64,
    /// Candidate examinations, including merged-source skips that never reach
    /// the exact predicate.
    pub candidate_exams: u64,
    /// True when degenerate input forced the exact all-pairs fallback.
    pub fell_back_to_all_pairs: bool,
}

/// Plan coincident-vertex merges for `positions` under `tolerance`.
///
/// `positions` is in vertex-discovery order; the surviving representative of
/// each merge is always the earliest eligible vertex. See the module docs for
/// the conserved semantics.
#[must_use]
pub fn plan_coincident_merges(positions: &[Point3], tolerance: f64) -> VertexMergePlan {
    if positions.is_empty() {
        return VertexMergePlan {
            merge_target: Vec::new(),
            merged_count: 0,
            distance_checks: 0,
            candidate_exams: 0,
            fell_back_to_all_pairs: false,
        };
    }
    if !tolerance.is_finite() || tolerance <= 0.0 || !grid_applicable(positions, tolerance) {
        return all_pairs_plan(positions, tolerance, true);
    }
    stream_grid_plan(positions, tolerance)
}

/// Whether the grid path can represent every position conservatively.
fn grid_applicable(positions: &[Point3], tolerance: f64) -> bool {
    positions.iter().all(|p| {
        p.x().is_finite()
            && p.y().is_finite()
            && p.z().is_finite()
            && (p.x() / tolerance).abs() <= MAX_EXACT_CELL
            && (p.y() / tolerance).abs() <= MAX_EXACT_CELL
            && (p.z() / tolerance).abs() <= MAX_EXACT_CELL
    })
}

/// Exact legacy all-pairs loop, preserved as the degenerate-input fallback and
/// as the equivalence oracle for tests.
fn all_pairs_plan(positions: &[Point3], tolerance: f64, fell_back: bool) -> VertexMergePlan {
    let tol_sq = tolerance * tolerance;
    let n = positions.len();
    let mut merge_target: Vec<Option<usize>> = vec![None; n];
    let mut merged_count = 0usize;
    let mut distance_checks = 0u64;
    for i in 0..n {
        if merge_target[i].is_some() {
            continue;
        }
        for j in (i + 1)..n {
            if merge_target[j].is_some() {
                continue;
            }
            distance_checks += 1;
            if (positions[i] - positions[j]).length_squared() < tol_sq {
                merge_target[j] = Some(i);
                merged_count += 1;
            }
        }
    }
    VertexMergePlan {
        merge_target,
        merged_count,
        distance_checks,
        candidate_exams: distance_checks,
        fell_back_to_all_pairs: fell_back,
    }
}

/// Streaming grid plan: `j` ascending, ascending-`i` k-way scan per `j`.
fn stream_grid_plan(positions: &[Point3], tolerance: f64) -> VertexMergePlan {
    let tol_sq = tolerance * tolerance;
    let n = positions.len();
    let cells_of: Vec<Cell> = positions
        .iter()
        .map(|p| {
            (
                (p.x() / tolerance).floor() as i64,
                (p.y() / tolerance).floor() as i64,
                (p.z() / tolerance).floor() as i64,
            )
        })
        .collect();
    // Buckets grow in index order, so every bucket is ascending by construction.
    let mut cells: DetHashMap<Cell, Vec<usize>> = DetHashMap::default();
    for (idx, key) in cells_of.iter().enumerate() {
        cells.entry(*key).or_default().push(idx);
    }

    let mut merge_target: Vec<Option<usize>> = vec![None; n];
    let mut merged_count = 0usize;
    let mut distance_checks = 0u64;
    let mut candidate_exams = 0u64;

    for j in 0..n {
        if merge_target[j].is_some() {
            continue;
        }
        let (cx, cy, cz) = cells_of[j];
        // Neighbor buckets restricted to indices below `j` (ascending prefix).
        let mut buckets: Vec<(&[usize], usize)> = Vec::with_capacity(27);
        for dx in -1..=1_i64 {
            for dy in -1..=1_i64 {
                for dz in -1..=1_i64 {
                    if let Some(bucket) = cells.get(&(cx + dx, cy + dy, cz + dz)) {
                        let len = bucket.partition_point(|&i| i < j);
                        if len > 0 {
                            buckets.push((&bucket[..len], 0));
                        }
                    }
                }
            }
        }
        // Linear k-way merge to ascending order; `j` merges at most once.
        loop {
            let mut best: Option<usize> = None;
            for (b, bucket) in buckets.iter().enumerate() {
                if bucket.1 < bucket.0.len() {
                    match best {
                        None => best = Some(b),
                        Some(bb) => {
                            if bucket.0[bucket.1] < buckets[bb].0[buckets[bb].1] {
                                best = Some(b);
                            }
                        }
                    }
                }
            }
            let Some(b) = best else { break };
            let i = buckets[b].0[buckets[b].1];
            buckets[b].1 += 1;
            candidate_exams += 1;
            if merge_target[i].is_some() {
                continue;
            }
            distance_checks += 1;
            if (positions[i] - positions[j]).length_squared() < tol_sq {
                merge_target[j] = Some(i);
                merged_count += 1;
                break;
            }
        }
    }

    VertexMergePlan {
        merge_target,
        merged_count,
        distance_checks,
        candidate_exams,
        fell_back_to_all_pairs: false,
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    /// Deterministic LCG for reproducible fixtures (no external deps).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (self.0 >> 33) & 0x7fff_ffff
        }
        fn next_f64(&mut self) -> f64 {
            f64::from(self.next() as u32) / f64::from(u32::MAX)
        }
    }

    fn assert_plan_equals_reference(positions: &[Point3], tolerance: f64) {
        let new = plan_coincident_merges(positions, tolerance);
        let old = all_pairs_plan(positions, tolerance, false);
        assert_eq!(
            new.merge_target, old.merge_target,
            "merge map must equal the all-pairs reference"
        );
        assert_eq!(new.merged_count, old.merged_count);
        // The index may only skip exact evaluations, never add merges.
        assert!(
            new.distance_checks <= old.distance_checks.max(1),
            "grid must not evaluate more pairs than all-pairs"
        );
    }

    #[test]
    fn empty_and_singleton_need_no_work() {
        let plan = plan_coincident_merges(&[], 1e-7);
        assert_eq!(plan.merged_count, 0);
        assert_eq!(plan.distance_checks, 0);
        assert!(!plan.fell_back_to_all_pairs);
        let plan = plan_coincident_merges(&[Point3::new(1.0, 2.0, 3.0)], 1e-7);
        assert_eq!(plan.merge_target, vec![None]);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn sparse_model_pays_no_distance_checks() {
        // 500 vertices on a unit grid: nothing eligible, nothing evaluated.
        let positions: Vec<Point3> = (0..500)
            .map(|i| Point3::new(f64::from(i), 0.0, 0.0))
            .collect();
        let plan = plan_coincident_merges(&positions, 1e-7);
        assert_eq!(plan.merged_count, 0);
        assert_eq!(plan.distance_checks, 0);
        assert_eq!(plan.candidate_exams, 0);
        assert!(!plan.fell_back_to_all_pairs);
    }

    #[test]
    fn cell_boundaries_neither_miss_nor_invent() {
        let tol = 1e-7;
        // Pair straddling the cell border at x = 10*tol, 4e-9 apart: eligible.
        let a = Point3::new(10.0 * tol - 2e-9, 0.0, 0.0);
        let b = Point3::new(10.0 * tol + 2e-9, 0.0, 0.0);
        // Pair exactly tol apart: strict `<` keeps them distinct.
        let c = Point3::new(0.0, 5.0, 0.0);
        let d = Point3::new(tol, 5.0, 0.0);
        // Pair just below tol: merges.
        let e = Point3::new(0.0, 9.0, 0.0);
        let f = Point3::new(tol * (1.0 - 1e-9), 9.0, 0.0);
        let positions = vec![a, b, c, d, e, f];
        let plan = plan_coincident_merges(&positions, tol);
        assert_eq!(
            plan.merge_target,
            vec![None, Some(0), None, None, None, Some(4)]
        );
        assert_eq!(plan.merged_count, 2);
        assert_plan_equals_reference(&positions, tol);
    }

    #[test]
    fn negative_and_large_coordinates_match_reference() {
        let tol = 1e-7;
        let positions = vec![
            Point3::new(-100.0, -200.0, -300.0),
            Point3::new(-100.0 + 5e-8, -200.0, -300.0),
            Point3::new(1e6, 2e6, -1e6),
            Point3::new(1e6 + 5e-8, 2e6, -1e6),
            Point3::new(1e6 + 1.0, 2e6, -1e6),
            Point3::new(-1e6, -2e6, 1e6),
        ];
        let plan = plan_coincident_merges(&positions, tol);
        assert_eq!(
            plan.merge_target,
            vec![None, Some(0), None, Some(2), None, None]
        );
        assert_plan_equals_reference(&positions, tol);
    }

    #[test]
    fn global_tolerance_governs_regardless_of_stored_entity_tolerance() {
        // The plan sees positions plus the run tolerance only: a pair 5e-8
        // apart merges at tol 1e-7, a pair 5e-7 apart does not — no per-vertex
        // tolerance widens or narrows the gate (policy unchanged from the
        // all-pairs loop; the soup-level test pins heterogeneous stored
        // vertex tolerances end to end).
        let tol = 1e-7;
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5e-8, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0 + 5e-7, 0.0, 0.0),
        ];
        let plan = plan_coincident_merges(&positions, tol);
        assert_eq!(plan.merge_target, vec![None, Some(0), None, None]);
        assert_plan_equals_reference(&positions, tol);
    }

    #[test]
    fn coincident_vertices_all_merge_into_index_zero() {
        let positions = vec![Point3::new(1.0, 2.0, 3.0); 500];
        let plan = plan_coincident_merges(&positions, 1e-7);
        assert_eq!(plan.merged_count, 499);
        assert_eq!(plan.distance_checks, 499);
        assert_eq!(plan.candidate_exams, 499);
        for (j, target) in plan.merge_target.iter().enumerate() {
            if j == 0 {
                assert_eq!(*target, None);
            } else {
                assert_eq!(*target, Some(0));
            }
        }
    }

    #[test]
    fn nontransitive_nearness_does_not_chain() {
        // Spacing 0.6*tol: 1 merges into 0, but 2 (near merged 1, far from 0)
        // stays distinct; 3 merges into 2; 4 stays distinct.
        let tol = 1e-7;
        let positions: Vec<Point3> = (0..5)
            .map(|i| Point3::new(f64::from(i) * 0.6 * tol, 0.0, 0.0))
            .collect();
        let plan = plan_coincident_merges(&positions, tol);
        assert_eq!(plan.merge_target, vec![None, Some(0), None, Some(2), None]);
        assert_eq!(plan.merged_count, 2);
        assert_plan_equals_reference(&positions, tol);
    }

    #[test]
    fn non_finite_positions_fall_back_without_changing_the_map() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(5e-8, 0.0, 0.0),
            Point3::new(f64::INFINITY, 0.0, 0.0),
        ];
        let plan = plan_coincident_merges(&positions, 1e-7);
        assert!(plan.fell_back_to_all_pairs);
        assert_eq!(plan.merge_target[2], Some(0));
        assert_eq!(plan.merge_target[1], None);
        assert_eq!(plan.merge_target[3], None);
        assert_plan_equals_reference(&positions, 1e-7);
    }

    #[test]
    fn degenerate_tolerance_falls_back() {
        let positions = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)];
        for tol in [0.0, -1e-7, f64::NAN, f64::INFINITY] {
            let plan = plan_coincident_merges(&positions, tol);
            assert!(plan.fell_back_to_all_pairs, "tol {tol}");
            assert_plan_equals_reference(&positions, tol);
        }
    }

    #[test]
    fn beyond_grid_range_falls_back() {
        // 1e16 with tol 1e-7 needs cell index 1e23: beyond exact integers.
        let positions = vec![
            Point3::new(1e16, 0.0, 0.0),
            Point3::new(1e16 + 5e-8, 0.0, 0.0),
        ];
        let plan = plan_coincident_merges(&positions, 1e-7);
        assert!(plan.fell_back_to_all_pairs);
        assert_plan_equals_reference(&positions, 1e-7);
    }

    #[test]
    fn randomized_fixtures_match_reference() {
        let tol = 1e-7;
        for size in [0, 1, 2, 3, 17, 100, 300] {
            // Sparse grid with a negative offset (cell-boundary coverage).
            let sparse: Vec<Point3> = (0..size)
                .map(|i| Point3::new(-50.0 + f64::from(i) * 1.7, 3.25, -1.5))
                .collect();
            assert_plan_equals_reference(&sparse, tol);
            // Clustered triples with deterministic jitter.
            let mut rng = Lcg(0xabcd + size as u64);
            let clustered: Vec<Point3> = (0..size)
                .map(|i| {
                    let base = f64::from(i / 3) * 2.3 - 100.0;
                    Point3::new(
                        base + (rng.next_f64() - 0.5) * 1.2e-7,
                        (rng.next_f64() - 0.5) * 1.2e-7,
                        (rng.next_f64() - 0.5) * 1.2e-7,
                    )
                })
                .collect();
            assert_plan_equals_reference(&clustered, tol);
            // Adversarial: everything inside one tolerance cube.
            let mut rng = Lcg(0x55aa + size as u64);
            let dense: Vec<Point3> = (0..size)
                .map(|_| {
                    Point3::new(
                        -7.0 + rng.next_f64() * 5e-8,
                        11.0 + rng.next_f64() * 5e-8,
                        rng.next_f64() * 5e-8,
                    )
                })
                .collect();
            assert_plan_equals_reference(&dense, tol);
            // Chain at 0.6*tol (nontransitive stress).
            let chain: Vec<Point3> = (0..size)
                .map(|i| Point3::new(f64::from(i) * 0.6 * tol, 0.0, 0.0))
                .collect();
            assert_plan_equals_reference(&chain, tol);
        }
    }

    #[test]
    fn plan_is_deterministic_across_runs() {
        let mut rng = Lcg(0xbeef);
        let positions: Vec<Point3> = (0..2000)
            .map(|i| {
                let base = f64::from(i / 3) * 1.1;
                Point3::new(
                    base + (rng.next_f64() - 0.5) * 1.2e-7,
                    (rng.next_f64() - 0.5) * 1.2e-7,
                    (rng.next_f64() - 0.5) * 1.2e-7,
                )
            })
            .collect();
        let a = plan_coincident_merges(&positions, 1e-7);
        let b = plan_coincident_merges(&positions, 1e-7);
        let c = plan_coincident_merges(&positions, 1e-7);
        assert_eq!(a.merge_target, b.merge_target);
        assert_eq!(b.merge_target, c.merge_target);
    }

    #[test]
    fn dense_input_stays_linear_in_distance_checks() {
        // 5k coincident vertices: early exit after the first eligible partner.
        let positions = vec![Point3::new(-3.0, 4.0, 5.0); 5000];
        let plan = plan_coincident_merges(&positions, 1e-7);
        assert_eq!(plan.merged_count, 4999);
        assert_eq!(plan.distance_checks, 4999);
        assert!(!plan.fell_back_to_all_pairs);
    }
}
