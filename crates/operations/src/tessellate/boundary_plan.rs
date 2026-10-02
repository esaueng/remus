//! Explicit shared-boundary plan for solid tessellation (PERF-D03 substrate).
//!
//! The solid pipeline used to grow its shared edge pool implicitly: edge
//! sampling, circle-density synchronization, torus-rim densification,
//! seam-meridian splits, planar contact refinement, and CDT
//! constraint-recovery Steiner splices each mutated the same maps inline in
//! `solid.rs`. That made every cross-face dependency an ordering accident:
//! two holed-planar CDT jobs splitting the same shared segment spliced their
//! runs one after another, so the second job's insertion point was already
//! gone and its subdivision never reached the neighbours.
//!
//! This module names the stages without changing the meshing policy:
//!
//! * **Stage A (plan).** [`BoundaryPlan`] owns the authoritative per-edge
//!   sample chains — 3D points, authoritative curve parameters, closed-edge
//!   flags, sample identities (global vertex ids), and the display-vs-boolean
//!   sampling policy (`circle_floor`) — plus the merged position pool every
//!   face triangulates against.
//! * **Stage B (local triangulation).** Holed-planar CDT jobs run against a
//!   snapshot of the plan and return [`BoundaryRefinementRequest`]s instead
//!   of mutating shared state. Refinement traffic is data, not a side effect.
//! * **Stage C (reconcile).** [`BoundaryPlan::reconcile`] merges every
//!   request deterministically (sorted keys, sorted parameters, identity
//!   dedup) and splices each shared segment exactly once, so every incident
//!   face observes the same final subdivision. Work is bounded by
//!   [`MAX_BOUNDARY_REFINEMENT_POINTS`]; excess fails closed instead of
//!   returning a partial-success mesh.
//! * **Stage D (assembly).** `solid.rs` consumes the final plan: staged CDT
//!   triangles are repaired onto the reconciled subdivision with the proven
//!   `split_triangles_spanning_boundary_splits` route, and every later face
//!   triangulates directly against the final chains.
//!
//! Curved families keep their established structured/CDT/snap dispatch; only
//! the holed-planar CDT family and its neighbouring faces flow through the
//! request/reconcile path. This substrate is what later mesh caching
//! (PERF-D01) and broader parallelism (PERF-D04) build on; neither is
//! implemented here.

use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;

use super::{MERGE_GRID, TriangleMesh, point_merge_key};

/// Upper bound on the number of boundary-refinement points reconciled into
/// one face set.
///
/// The cap covers CDT constraint-recovery Steiner points landing on shared
/// boundary segments. It is a fail-closed backstop against malformed input
/// (thousands of jobs splitting the same segment), not a tuning knob:
/// production inputs reconcile a handful of points, and exceeding the budget
/// returns `InvalidInput` with no partial mesh rather than degrading
/// quality silently.
const MAX_BOUNDARY_REFINEMENT_POINTS: usize = 1_000_000;

/// Per-edge sample chains in stored start-to-end order, keyed by edge index.
pub(super) type SampleChains = DetHashMap<usize, Vec<Point3>>;

/// Authoritative curve parameter per sample, parallel to [`SampleChains`].
/// `None` marks geometric insertions (seam crossings) carrying no parameter.
pub(super) type SampleParams = DetHashMap<usize, Vec<Option<f64>>>;

/// Merged per-segment split map: shared segment `(lo, hi)` with `lo < hi`
/// to subdivision points with parameters measured from `lo` towards `hi`.
/// Produced by [`BoundaryPlan::reconcile`] for the proven triangle repair
/// route.
pub(super) type BoundarySplits = DetHashMap<(u32, u32), Vec<(f64, u32)>>;

/// One sampled edge: its index, start-to-end points, and parallel curve
/// parameters. Produced per edge by the plan's sampling stage (in parallel
/// on native targets) and assembled into [`BoundaryPlan::from_samples`].
#[cfg(not(target_arch = "wasm32"))]
pub(super) type SampledEdge = (usize, Vec<Point3>, Vec<Option<f64>>);

/// Per-edge sampling policy retained by the plan.
///
/// Display callers pass `circle_floor = false` (constant-curvature circles
/// are exact without the curvature floor); the boolean mesh-fallback passes
/// `true` for co-refinement robustness. Because the shared edge pool drives
/// downstream band density, the one flag governs every circular feature
/// consistently — and the two policies must never share a cache key
/// (see PERF-D01).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct BoundaryPolicy {
    /// Keep the curvature floor on circular edges.
    pub(super) circle_floor: bool,
}

/// Authoritative parameter authority retained per sampled edge.
///
/// Coedge direction is consumed at triangulation time through
/// `OrientedEdge::is_forward()`; the plan exposes
/// [`BoundaryPlan::ordered_chain`] so every consumer walks the same chain in
/// traversal order instead of re-deriving orientation ad hoc.
#[derive(Debug, Clone)]
pub(super) struct EdgeAuthority {
    /// Authoritative curve domain `(t_min, t_max)` for curved edges.
    /// `None` for lines (parameterized by arclength at use sites).
    pub(super) domain: Option<(f64, f64)>,
    /// Whether the edge is topologically closed.
    pub(super) is_closed: bool,
    /// Curve variant tag for diagnostics (`"line"`, `"circle"`, ...).
    pub(super) curve_tag: &'static str,
}

/// Stage-work counters for the boundary plan.
///
/// Counts (not timings) keep the plan `wasm32`-compatible and give later
/// caching/parallelism work a stable accounting surface. Wall-time and
/// allocation totals are measured externally by the paired benchmark over
/// the complete workflow, not by local triangulation alone.
#[derive(Debug, Clone, Default)]
pub(super) struct PlanMetrics {
    /// Number of unique edges sampled.
    pub(super) edges_sampled: usize,
    /// Total shared sample points before refinement.
    pub(super) sample_points: usize,
    /// Edges upsampled by circle-density synchronization.
    pub(super) circle_sync_upsampled: usize,
    /// Rim edges densified for torus two-rim bands.
    pub(super) torus_densified: usize,
    /// Seam-meridian split points inserted.
    pub(super) seam_splits: usize,
    /// Line edges refined by planar contact subdivision.
    pub(super) contact_lines_refined: usize,
    /// Circle refinements merged from the position pool.
    pub(super) contact_circle_insertions: usize,
    /// Steiner points lifted from local CDT triangulation.
    pub(super) steiner_points: usize,
    /// Shared segments reconciled in stage C.
    pub(super) reconciled_segments: usize,
    /// Refinement points spliced into shared chains.
    pub(super) reconciled_points: usize,
}

/// One local triangulation's request to subdivide a shared boundary segment.
///
/// `segment` is the undirected global pair `(lo, hi)` with `lo < hi`;
/// `splits` carries `(t, gid)` with `t` measured from `lo` towards `hi`.
/// Requests are plain data: collecting them never mutates the plan, so
/// local triangulation stays independent and the reconcile step can order
/// them deterministically regardless of job completion order.
#[derive(Debug, Clone)]
pub(super) struct BoundaryRefinementRequest {
    /// Index (into the face set) of the requesting face.
    pub(super) source_face: u32,
    /// Shared segment endpoints, normalized so `lo < hi`.
    pub(super) segment: (u32, u32),
    /// Subdivision points with parameters from `lo` towards `hi`.
    pub(super) splits: Vec<(f64, u32)>,
}

impl BoundaryRefinementRequest {
    /// Build a request, normalizing the segment orientation.
    ///
    /// When `flip` is set, the incoming parameters run from `hi` towards
    /// `lo` and are mirrored (`1 - t`) onto the normalized direction.
    /// Callers whose runs already follow `(first, second)` order pass
    /// `false` and let the segment normalization handle the rest.
    pub(super) fn new(
        source_face: u32,
        first: u32,
        second: u32,
        run: &[(f64, u32)],
        flip: bool,
    ) -> Self {
        let (lo, hi) = if first < second {
            (first, second)
        } else {
            (second, first)
        };
        let oriented_flip = flip != (first > second);
        let splits = run
            .iter()
            .map(|&(t, gid)| (if oriented_flip { 1.0 - t } else { t }, gid))
            .collect();
        Self {
            source_face,
            segment: (lo, hi),
            splits,
        }
    }
}

/// Explicit shared-boundary plan: authoritative sample chains plus the merged
/// vertex pool every face triangulates against.
pub(super) struct BoundaryPlan {
    /// Requested chord deviation.
    pub(super) deflection: f64,
    /// Per-segment tangent-turn cap (0.0 disables the angular criterion).
    pub(super) angular_tol: f64,
    /// Display-vs-boolean sampling policy.
    pub(super) policy: BoundaryPolicy,
    /// Per-edge sample chains in stored start-to-end order.
    pub(super) edge_points: SampleChains,
    /// Authoritative parameter per sample, parallel to `edge_points`.
    pub(super) edge_params: SampleParams,
    /// Per-edge authority (domain, closed flag, curve tag).
    pub(super) authorities: DetHashMap<usize, EdgeAuthority>,
    /// Merged vertex pool (positions/normals grow as Steiner points lift).
    pub(super) merged: TriangleMesh,
    /// Merge-grid key to global vertex id.
    pub(super) point_to_global: DetHashMap<(i64, i64, i64), u32>,
    /// Shared sample chains as global vertex ids.
    pub(super) edge_chains: DetHashMap<usize, Vec<u32>>,
    /// Sorted edge indices for deterministic iteration.
    pub(super) edge_order: Vec<usize>,
    /// Stage-work counters.
    pub(super) metrics: PlanMetrics,
}

impl BoundaryPlan {
    /// Assemble a plan from sampled edge points, retaining authority.
    ///
    /// `edge_points` maps edge index to start-to-end sample chains;
    /// `edge_params` carries the matching curve parameters (parallel
    /// arrays; a missing or short entry reads as unknown parameters).
    /// The merged pool and global chains are built deterministically:
    /// edges iterate in sorted index order and points keep chain order, so
    /// global vertex ids are a pure function of the input — never of thread
    /// scheduling or hash iteration order.
    pub(super) fn from_samples(
        topo: &Topology,
        edge_points: SampleChains,
        edge_params: SampleParams,
        deflection: f64,
        angular_tol: f64,
        circle_floor: bool,
    ) -> Self {
        let mut edge_order: Vec<usize> = edge_points.keys().copied().collect();
        edge_order.sort_unstable();

        let mut authorities: DetHashMap<usize, EdgeAuthority> = DetHashMap::default();
        for &edge_idx in &edge_order {
            let Some(edge_id) = topo.edge_id_from_index(edge_idx) else {
                continue;
            };
            let Ok(edge_data) = topo.edge(edge_id) else {
                continue;
            };
            let (domain, curve_tag) = match edge_data.curve() {
                remus_topology::edge::EdgeCurve::Line => (None, "line"),
                remus_topology::edge::EdgeCurve::Circle(_)
                | remus_topology::edge::EdgeCurve::Ellipse(_)
                | remus_topology::edge::EdgeCurve::Hyperbola(_)
                | remus_topology::edge::EdgeCurve::Parabola(_)
                | remus_topology::edge::EdgeCurve::NurbsCurve(_) => (
                    crate::authoritative_edge_domain(edge_data, "boundary plan authority").ok(),
                    match edge_data.curve() {
                        remus_topology::edge::EdgeCurve::Circle(_) => "circle",
                        remus_topology::edge::EdgeCurve::Ellipse(_) => "ellipse",
                        remus_topology::edge::EdgeCurve::Hyperbola(_) => "hyperbola",
                        remus_topology::edge::EdgeCurve::Parabola(_) => "parabola",
                        remus_topology::edge::EdgeCurve::NurbsCurve(_) => "nurbs",
                        remus_topology::edge::EdgeCurve::Line => "line",
                    },
                ),
            };
            authorities.insert(
                edge_idx,
                EdgeAuthority {
                    domain,
                    is_closed: edge_data.start() == edge_data.end(),
                    curve_tag,
                },
            );
        }

        let sample_points = edge_points.values().map(Vec::len).sum();

        let mut plan = Self {
            deflection,
            angular_tol,
            policy: BoundaryPolicy { circle_floor },
            edge_points,
            edge_params,
            authorities,
            merged: TriangleMesh::default(),
            point_to_global: DetHashMap::default(),
            edge_chains: DetHashMap::default(),
            edge_order,
            metrics: PlanMetrics {
                edges_sampled: 0,
                sample_points,
                ..PlanMetrics::default()
            },
        };
        plan.metrics.edges_sampled = plan.edge_order.len();
        plan.rebuild_pool();
        plan
    }

    /// (Re)build the merged pool and global chains from the sample chains.
    ///
    /// Deterministic by construction: `edge_order` is sorted and each chain
    /// keeps start-to-end order, so the first writer of a merge-grid cell
    /// wins identically on every run, thread count, and platform.
    fn rebuild_pool(&mut self) {
        self.merged = TriangleMesh::default();
        self.point_to_global = DetHashMap::default();
        self.edge_chains = DetHashMap::default();
        for &edge_idx in &self.edge_order {
            let Some(points) = self.edge_points.get(&edge_idx) else {
                continue;
            };
            let mut global_ids = Vec::with_capacity(points.len());
            for &pt in points {
                let key = point_merge_key(pt, MERGE_GRID);
                let idx = self.point_to_global.entry(key).or_insert_with(|| {
                    #[allow(clippy::cast_possible_truncation)]
                    let idx = self.merged.positions.len() as u32;
                    self.merged.positions.push(pt);
                    self.merged.normals.push(Vec3::new(0.0, 0.0, 0.0));
                    idx
                });
                global_ids.push(*idx);
            }
            self.edge_chains.insert(edge_idx, global_ids);
        }
    }

    /// Shared chain for an edge in wire-traversal order.
    ///
    /// `forward` is the coedge direction (`OrientedEdge::is_forward()`):
    /// forward walks the stored chain, reversed walks it back-to-front.
    /// Returns `None` when the edge has no samples in this plan.
    pub(super) fn ordered_chain(&self, edge_idx: usize, forward: bool) -> Option<Vec<u32>> {
        let chain = self.edge_chains.get(&edge_idx)?;
        if forward {
            Some(chain.clone())
        } else {
            let mut reversed = chain.clone();
            reversed.reverse();
            Some(reversed)
        }
    }

    /// Collect a wire's boundary walk from the shared chains.
    ///
    /// Mirrors `planar::collect_wire_global_vertices` exactly (same junction
    /// dedup), but sources each edge through [`BoundaryPlan::ordered_chain`]
    /// so coedge direction is applied in one place. Positions and global ids
    /// run parallel; a trailing vertex duplicating the first is kept here
    /// and removed by the caller's `remove_closing_duplicate_global`.
    pub(super) fn collect_wire_vertices(
        &self,
        wire: &remus_topology::wire::Wire,
        tol: f64,
    ) -> (Vec<Point3>, Vec<Option<u32>>) {
        let mut out_positions: Vec<Point3> = Vec::new();
        let mut out_global_ids: Vec<Option<u32>> = Vec::new();
        for oe in wire.edges() {
            let Some(global_ids) = self.ordered_chain(oe.edge().index(), oe.is_forward()) else {
                continue;
            };
            for (j, gid) in global_ids.iter().copied().enumerate() {
                if j == 0 && !out_global_ids.is_empty() {
                    let last_gid = out_global_ids.last().and_then(|g| *g).unwrap_or(u32::MAX);
                    if last_gid == gid {
                        continue;
                    }
                    if (last_gid as usize) < self.merged.positions.len()
                        && (gid as usize) < self.merged.positions.len()
                        && (self.merged.positions[last_gid as usize]
                            - self.merged.positions[gid as usize])
                            .length()
                            < tol
                    {
                        continue;
                    }
                }
                out_positions.push(self.merged.positions[gid as usize]);
                out_global_ids.push(Some(gid));
            }
        }
        (out_positions, out_global_ids)
    }

    /// Lift CDT constraint-recovery Steiner points to 3D global vertices.
    ///
    /// Returns the new global ids in Steiner order and records the count in
    /// the plan metrics. Lifting is deterministic: identical 3D positions
    /// merge onto the first global id through the shared merge grid.
    pub(super) fn lift_steiner_points(&mut self, points_3d: Vec<Point3>, normal: Vec3) -> Vec<u32> {
        let mut gids = Vec::with_capacity(points_3d.len());
        for p3d in points_3d {
            let key = point_merge_key(p3d, MERGE_GRID);
            let gid = *self.point_to_global.entry(key).or_insert_with(|| {
                #[allow(clippy::cast_possible_truncation)]
                let idx = self.merged.positions.len() as u32;
                self.merged.positions.push(p3d);
                self.merged.normals.push(normal);
                idx
            });
            gids.push(gid);
        }
        self.metrics.steiner_points += gids.len();
        gids
    }

    /// Reconcile local-triangulation refinement requests deterministically.
    ///
    /// Every request group for one shared segment `(lo, hi)` is merged into
    /// a single ordered split list (sorted by parameter, then global id,
    /// deduped by identity), and each shared chain containing that segment
    /// is spliced exactly once, in sorted segment order. A segment split by
    /// several jobs therefore reaches every incident face identically —
    /// independent of job completion or input order — and a chain that never
    /// contained the segment is left untouched.
    ///
    /// Returns the merged per-segment split map for the proven triangle
    /// repair route (`split_triangles_spanning_boundary_splits`). The total
    /// reconciled point count is bounded by
    /// [`MAX_BOUNDARY_REFINEMENT_POINTS`]; excess fails closed with
    /// `InvalidInput` and no partial mesh.
    pub(super) fn reconcile(
        &mut self,
        requests: Vec<BoundaryRefinementRequest>,
    ) -> Result<BoundarySplits, crate::OperationsError> {
        let mut sources: DetHashSet<u32> = DetHashSet::default();
        let mut grouped: BoundarySplits = DetHashMap::default();
        for request in requests {
            sources.insert(request.source_face);
            grouped
                .entry(request.segment)
                .or_default()
                .extend(request.splits);
        }
        let mut keys: Vec<(u32, u32)> = grouped.keys().copied().collect();
        keys.sort_unstable();

        let mut total_points: usize = 0;
        let mut merged: BoundarySplits = DetHashMap::default();
        for key in keys {
            let Some(mut run) = grouped.remove(&key) else {
                continue;
            };
            run.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
            let mut deduped: Vec<(f64, u32)> = Vec::with_capacity(run.len());
            let mut seen: DetHashSet<u32> = DetHashSet::default();
            for (t, gid) in run {
                if seen.insert(gid) {
                    deduped.push((t, gid));
                }
            }
            total_points += deduped.len();
            if total_points > MAX_BOUNDARY_REFINEMENT_POINTS {
                let mut contributing: Vec<u32> = sources.into_iter().collect();
                contributing.sort_unstable();
                return Err(crate::OperationsError::InvalidInput {
                    reason: format!(
                        "boundary reconciliation exceeds its {MAX_BOUNDARY_REFINEMENT_POINTS}-point work budget ({} points from faces {contributing:?}); increase tolerances",
                        total_points + grouped.values().map(Vec::len).sum::<usize>(),
                    ),
                });
            }
            if !deduped.is_empty() {
                merged.insert(key, deduped);
            }
        }

        // Splice each merged run into every chain holding the segment, once.
        // Sorted keys keep the splice order deterministic; splicing the
        // merged run (rather than one job's run at a time) keeps the second
        // job's points from missing their insertion window.
        let mut splice_keys: Vec<(u32, u32)> = merged.keys().copied().collect();
        splice_keys.sort_unstable();
        for key in &splice_keys {
            let Some(run) = merged.get(key) else {
                continue;
            };
            let (lo, hi) = *key;
            let gids: Vec<u32> = run.iter().map(|&(_, gid)| gid).collect();
            for chain in self.edge_chains.values_mut() {
                for p in 0..chain.len().saturating_sub(1) {
                    if chain[p] == lo && chain[p + 1] == hi {
                        let mut insert_at = p + 1;
                        for &gid in &gids {
                            if !chain.contains(&gid) {
                                chain.insert(insert_at, gid);
                                insert_at += 1;
                            }
                        }
                        break;
                    }
                    if chain[p] == hi && chain[p + 1] == lo {
                        let mut insert_at = p + 1;
                        for &gid in gids.iter().rev() {
                            if !chain.contains(&gid) {
                                chain.insert(insert_at, gid);
                                insert_at += 1;
                            }
                        }
                        break;
                    }
                }
            }
        }

        self.metrics.reconciled_segments = merged.len();
        self.metrics.reconciled_points = total_points;
        Ok(merged)
    }

    /// Check the plan's structural invariants against the topology.
    ///
    /// Reads every retained field — tolerances, policy, per-edge points and
    /// parameters, authorities (domain, closed flag, curve tag), chains, and
    /// metrics — so the retention is load-bearing, not decorative. Called
    /// behind `debug_assert!` on the hot path (zero release cost); failures
    /// are typed `InvalidInput`, never panics.
    pub(super) fn validate_structure(&self, topo: &Topology) -> Result<(), crate::OperationsError> {
        let bad = |detail: &str| crate::OperationsError::InvalidInput {
            reason: format!("boundary plan invariant violated: {detail}"),
        };
        if !self.deflection.is_finite() || self.deflection <= 0.0 {
            return Err(bad("non-positive deflection"));
        }
        if !self.angular_tol.is_finite() || self.angular_tol < 0.0 {
            return Err(bad("negative angular tolerance"));
        }
        if self.metrics.edges_sampled != self.edge_order.len() {
            return Err(bad("edge census mismatch"));
        }
        let counted: usize = self.edge_points.values().map(Vec::len).sum();
        if self.metrics.sample_points != counted {
            return Err(bad("sample census mismatch"));
        }
        for &edge_idx in &self.edge_order {
            let Some(points) = self.edge_points.get(&edge_idx) else {
                return Err(bad("sample chain missing for ordered edge"));
            };
            let params_len = self.edge_params.get(&edge_idx).map_or(0, Vec::len);
            if params_len != points.len() {
                return Err(bad("parameter chain out of step with points"));
            }
            let Some(chain) = self.edge_chains.get(&edge_idx) else {
                return Err(bad("global chain missing for ordered edge"));
            };
            if chain.len() < points.len() {
                // Chains only grow past the sample count (contact and Steiner
                // insertions reuse or add global ids, never drop samples).
                return Err(bad("global chain shorter than its sample chain"));
            }
            let Some(edge_id) = topo.edge_id_from_index(edge_idx) else {
                continue;
            };
            let Ok(edge_data) = topo.edge(edge_id) else {
                continue;
            };
            let Some(authority) = self.authorities.get(&edge_idx) else {
                return Err(bad("authority missing for ordered edge"));
            };
            if authority.is_closed != (edge_data.start() == edge_data.end()) {
                return Err(bad("closed flag disagrees with topology"));
            }
            let expected_tag = match edge_data.curve() {
                remus_topology::edge::EdgeCurve::Line => "line",
                remus_topology::edge::EdgeCurve::Circle(_) => "circle",
                remus_topology::edge::EdgeCurve::Ellipse(_) => "ellipse",
                remus_topology::edge::EdgeCurve::Hyperbola(_) => "hyperbola",
                remus_topology::edge::EdgeCurve::Parabola(_) => "parabola",
                remus_topology::edge::EdgeCurve::NurbsCurve(_) => "nurbs",
            };
            if authority.curve_tag != expected_tag {
                return Err(bad("curve tag disagrees with topology"));
            }
            if expected_tag != "line" && authority.domain.is_none() {
                return Err(bad("curved edge without an authoritative domain"));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn plan_with_chain(chain: Vec<u32>) -> BoundaryPlan {
        let mut plan = BoundaryPlan {
            deflection: 0.1,
            angular_tol: 0.5,
            policy: BoundaryPolicy {
                circle_floor: false,
            },
            edge_points: DetHashMap::default(),
            edge_params: DetHashMap::default(),
            authorities: DetHashMap::default(),
            merged: TriangleMesh::default(),
            point_to_global: DetHashMap::default(),
            edge_chains: DetHashMap::default(),
            edge_order: vec![7],
            metrics: PlanMetrics::default(),
        };
        plan.edge_chains.insert(7, chain);
        plan
    }

    #[test]
    fn reconcile_merges_same_segment_requests_independently_of_order() {
        let forward: Vec<BoundaryRefinementRequest> = vec![
            BoundaryRefinementRequest {
                source_face: 0,
                segment: (10, 20),
                splits: vec![(0.25, 101)],
            },
            BoundaryRefinementRequest {
                source_face: 1,
                segment: (10, 20),
                splits: vec![(0.75, 102)],
            },
        ];
        let mut reversed = forward.clone();
        reversed.reverse();

        let mut first = plan_with_chain(vec![10, 20]);
        let merged_first = first
            .reconcile(forward)
            .expect("bounded requests reconcile");
        let mut second = plan_with_chain(vec![10, 20]);
        let merged_second = second
            .reconcile(reversed)
            .expect("bounded requests reconcile");

        assert_eq!(first.edge_chains[&7], vec![10, 101, 102, 20]);
        assert_eq!(second.edge_chains[&7], vec![10, 101, 102, 20]);
        assert_eq!(merged_first, merged_second);
        assert_eq!(first.metrics.reconciled_segments, 1);
        assert_eq!(first.metrics.reconciled_points, 2);
    }

    #[test]
    fn reconcile_dedupes_identity_and_splices_reversed_chains() {
        let requests = vec![
            BoundaryRefinementRequest {
                source_face: 0,
                segment: (10, 20),
                splits: vec![(0.5, 111), (0.5, 111)],
            },
            BoundaryRefinementRequest {
                source_face: 1,
                segment: (10, 20),
                splits: vec![(0.5, 111)],
            },
        ];
        let mut plan = plan_with_chain(vec![20, 10]);
        let merged = plan
            .reconcile(requests)
            .expect("bounded requests reconcile");
        assert_eq!(plan.edge_chains[&7], vec![20, 111, 10]);
        assert_eq!(merged[&(10, 20)], vec![(0.5, 111)]);
    }

    #[test]
    fn reconcile_request_normalizes_flipped_parameters() {
        let request = BoundaryRefinementRequest::new(3, 20, 10, &[(0.25, 55)], true);
        assert_eq!(request.segment, (10, 20));
        assert_eq!(request.source_face, 3);
        assert_eq!(request.splits, vec![(0.25, 55)]);

        let mirrored = BoundaryRefinementRequest::new(3, 20, 10, &[(0.25, 55)], false);
        assert_eq!(mirrored.splits, vec![(0.75, 55)]);
    }

    #[test]
    fn reconcile_rejects_excess_over_its_budget() {
        let splits: Vec<(f64, u32)> = (0..=MAX_BOUNDARY_REFINEMENT_POINTS as u32)
            .map(|gid| (f64::from(gid) / 2_000_000.0, gid + 1_000_000))
            .collect();
        let requests = vec![BoundaryRefinementRequest {
            source_face: 0,
            segment: (1, 2),
            splits,
        }];
        let mut plan = plan_with_chain(vec![1, 2]);
        assert!(plan.reconcile(requests).is_err());
        // The failed reconcile leaves the shared chain untouched: no
        // partial-success subdivision escapes.
        assert_eq!(plan.edge_chains[&7], vec![1, 2]);
    }

    #[test]
    fn ordered_chain_follows_coedge_direction() {
        let plan = plan_with_chain(vec![4, 5, 6]);
        assert_eq!(plan.ordered_chain(7, true), Some(vec![4, 5, 6]));
        assert_eq!(plan.ordered_chain(7, false), Some(vec![6, 5, 4]));
        assert_eq!(plan.ordered_chain(9, true), None);
    }
}
