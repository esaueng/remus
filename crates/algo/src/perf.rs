//! Deterministic work counters for complexity-regression guards.
//!
//! These count the *inner work* of the boolean hot paths that issue #987 found
//! to be O(N²) — so a test can assert the work grows sub-quadratically with
//! input size. Counting work (not wall-clock) makes the guard deterministic: a
//! reintroduced per-item full scan turns a linear count into a quadratic one,
//! which trips the bound with no timing flakiness.
//!
//! Five hot paths were fixed in #990; each has a counter here:
//!
//! | Counter | Hot path | Bounded shape with the fix in |
//! |---|---|---|
//! | `pave_vertex_probes` | PaveFiller endpoint→vertex snap | spatial hash → near-constant per query |
//! | `sd_poly_clips` | `detect_same_domain` polygon clip | bbox gate → ~0 clips |
//! | `ray_geom_builds` | classify sub-faces | ray-cast geometry built once per solid, not per sub-face |
//! | `face_split_probes` | face-splitter section/loop scans | grid index → near-constant candidates per query |
//! | `local_vertex_inserts` | `build_topology_face` vertex pool | layered lookup → only genuinely-new vertices materialized |
//!
//! A second family counts vertex-on-edge (VE) projection work (PERF-B01 /
//! PERF-M04): every cross-solid vertex/edge pair that survives the AABB
//! broad-phase and endpoint rejection runs a closest-point projection. The
//! generic path samples the curve (33 evaluations) and refines with 20 ternary
//! steps (40 more). A future analytic fast path would serve `Line` edges in
//! closed form with no evaluations.
//!
//! | Counter | Hot path | Meaning |
//! |---|---|---|
//! | `ve_line_projections` | VE analytic fast path | `Line` pairs projected in closed form |
//! | `ve_sampled_projections` | VE generic path | non-`Line` (or degenerate) pairs that sampled |
//! | `ve_projection_evals` | VE curve evaluations | `evaluate` calls inside the projection only |
//!
//! A third pair counts edge-face (EF) work on analytic carriers (PERF-B05):
//! a curved edge against a plane, or any edge against a cylinder, cone,
//! sphere or torus, runs a sampled scan of 65 to 130 or more evaluations
//! unless a conservative gate proves the pair has no crossing.
//!
//! | Counter | Hot path | Meaning |
//! |---|---|---|
//! | `ef_analytic_pair_scans` | EF sampled scan | analytic pairs that reached the scan |
//! | `ef_analytic_pairs_gated` | EF analytic gates | pairs a gate proved crossing-free |
//!
//! A fourth family, `RayWorkCounts`, counts the planar work of the ray-cast
//! vote loop (`classifier::ray_cast`, PERF-Q07 subset): votes, plane hits,
//! polygon tests, polygon tests the face's bounding box settles outright,
//! exact point-segment distances and exact winding numbers, plus the planar
//! faces and distinct planes of each geometry build.
//!
//! `winding_cut_projections` counts the winding-loop cuts projected onto a
//! closed section loop in `presplit_closed_winding_loops`. Every face carries
//! every loop's cuts, so without the reach-box gate the count grows with the
//! square of the number of loops; with it, each loop projects only its own.
//!
//! The counters are gated behind the `perf-counters` feature. With the feature
//! off (every normal and release build) the `bump_*` calls are empty `#[inline]`
//! functions that compile to nothing, so the instrumented hot loops pay zero
//! cost. The scaling guard enables the feature only for its own test build.
//! The ray-work counters also count in this crate's own tests: their oracles
//! pin work the bounding-box gate skips without changing any result.

#[cfg(feature = "perf-counters")]
use std::cell::Cell;

// Boolean execution is synchronous within one caller thread. Thread-local
// counters keep the feature deterministic when the Rust test harness runs
// unrelated boolean tests concurrently with the scaling guard.
#[cfg(feature = "perf-counters")]
std::thread_local! {
    static PAVE_VERTEX_PROBES: Cell<u64> = const { Cell::new(0) };
    static SD_POLY_CLIPS: Cell<u64> = const { Cell::new(0) };
    static RAY_GEOM_BUILDS: Cell<u64> = const { Cell::new(0) };
    static FACE_SPLIT_PROBES: Cell<u64> = const { Cell::new(0) };
    static LOCAL_VERTEX_INSERTS: Cell<u64> = const { Cell::new(0) };
    static EF_NURBS_PAIR_PROBES: Cell<u64> = const { Cell::new(0) };
    static DISTANCE_FACE_PROBES: Cell<u64> = const { Cell::new(0) };
    static VE_LINE_PROJECTIONS: Cell<u64> = const { Cell::new(0) };
    static VE_SAMPLED_PROJECTIONS: Cell<u64> = const { Cell::new(0) };
    static VE_PROJECTION_EVALS: Cell<u64> = const { Cell::new(0) };
    static JUNCTION_SEEDS: Cell<u64> = const { Cell::new(0) };
    static EF_ANALYTIC_PAIR_SCANS: Cell<u64> = const { Cell::new(0) };
    static EF_ANALYTIC_PAIRS_GATED: Cell<u64> = const { Cell::new(0) };
    static WINDING_CUT_PROJECTIONS: Cell<u64> = const { Cell::new(0) };
}

#[cfg(any(test, feature = "perf-counters"))]
std::thread_local! {
    static RAY_WORK: std::cell::Cell<[u64; 8]> = const { std::cell::Cell::new([0; 8]) };
}

/// One unit of the ray-cast vote loop's planar work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RayWork {
    Vote,
    PlaneEval,
    PolygonTest,
    FaceSkip,
    SegmentEval,
    Winding,
    PlanarFace,
    PlaneGroup,
}

/// Count one unit of ray-cast planar work. Crate-internal.
#[inline]
pub(crate) fn bump_ray_work(kind: RayWork) {
    #[cfg(any(test, feature = "perf-counters"))]
    RAY_WORK.with(|work| {
        let mut counts = work.get();
        counts[kind as usize] = counts[kind as usize].saturating_add(1);
        work.set(counts);
    });
    #[cfg(not(any(test, feature = "perf-counters")))]
    let _ = kind;
}

/// Ray-cast vote-loop work on this thread (see [`take_ray_work`]).
#[cfg(any(test, feature = "perf-counters"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RayWorkCounts {
    /// Three-ray votes (a cardinal or a generic triple).
    pub votes: u64,
    /// Ray hits on a supporting plane, each shared by every face on it.
    pub plane_evals: u64,
    /// Face polygons tested against a plane hit ahead of the ray origin.
    pub polygon_tests: u64,
    /// Polygon tests the face's bounding box settled with no segment or
    /// winding work.
    pub face_skips: u64,
    /// Exact point-segment distances.
    pub segment_evals: u64,
    /// Exact winding numbers.
    pub windings: u64,
    /// Planar faces collected into ray-cast geometry.
    pub planar_faces: u64,
    /// Distinct supporting planes (exact bits) among those faces.
    pub plane_groups: u64,
}

/// This thread's ray-cast vote-loop work since the previous call, resetting
/// it. Available in this crate's tests and with `perf-counters`.
#[cfg(any(test, feature = "perf-counters"))]
#[must_use]
pub fn take_ray_work() -> RayWorkCounts {
    let [
        votes,
        plane_evals,
        polygon_tests,
        face_skips,
        segment_evals,
        windings,
        planar_faces,
        plane_groups,
    ] = RAY_WORK.with(|work| work.replace([0; 8]));
    RayWorkCounts {
        votes,
        plane_evals,
        polygon_tests,
        face_skips,
        segment_evals,
        windings,
        planar_faces,
        plane_groups,
    }
}

#[cfg(feature = "perf-counters")]
fn increment(counter: &'static std::thread::LocalKey<Cell<u64>>) {
    counter.with(|value| value.set(value.get().saturating_add(1)));
}

/// Count one pave-vertex distance comparison (per candidate examined while
/// snapping an intersection endpoint to a coincident vertex). Crate-internal:
/// only `reset`/`snapshot` cross the crate boundary (for the scaling guard).
#[inline]
pub(crate) fn bump_pave_vertex_probe() {
    #[cfg(feature = "perf-counters")]
    increment(&PAVE_VERTEX_PROBES);
}

/// Count one same-domain polygon-intersection clip (the expensive narrow-phase
/// in `planar_faces_overlap`). Crate-internal, like `bump_pave_vertex_probe`.
/// One edge x NURBS-face pair that survived the conservative box gate in
/// the edge-face interference phase and goes on to project edge samples
/// onto the surface.
#[inline]
pub(crate) fn bump_ef_nurbs_pair_probe() {
    #[cfg(feature = "perf-counters")]
    increment(&EF_NURBS_PAIR_PROBES);
}

/// One boundary-vertex x face distance evaluation in the solid-to-solid
/// distance query that survived its bounding-box pruning. Public because
/// the query lives in `remus-operations`.
#[inline]
pub fn bump_distance_face_probe() {
    #[cfg(feature = "perf-counters")]
    increment(&DISTANCE_FACE_PROBES);
}

#[inline]
#[allow(dead_code)]
pub(crate) fn bump_sd_poly_clip() {
    #[cfg(feature = "perf-counters")]
    increment(&SD_POLY_CLIPS);
}

/// Count one ray-cast geometry collection for a solid (`collect_face_geoms`).
/// This is the O(faces) build the classify loop now does *once* per argument
/// solid; rebuilding it per sub-face was the quadratic. A regression that
/// classifies via the per-call (uncached) path inside the sub-face loop bumps
/// this once per sub-face, so the count grows with the result's face count.
#[inline]
pub(crate) fn bump_ray_geom_build() {
    #[cfg(feature = "perf-counters")]
    increment(&RAY_GEOM_BUILDS);
}

/// Count one unit of face-splitter candidate work — either an endpoint examined
/// by a per-section / per-loop grid query (the "is there a point near this edge"
/// scan), or a chord pair that survives the arrangement's bbox broad-phase and
/// runs the real crossing / T-junction test. Each broad-phase keeps its work
/// near-constant per query/edge; reverting either makes it O(sections²).
/// Crate-internal.
#[inline]
pub(crate) fn bump_face_split_probe() {
    #[cfg(feature = "perf-counters")]
    increment(&FACE_SPLIT_PROBES);
}

/// Count one vertex materialized into a sub-face's local vertex map during
/// `build_topology_face`. The layered lookup resolves existing vertices by
/// reference from the shared seed/rank pools, so only genuinely-new vertices
/// land here — O(new vertices), linear in the result. Re-seeding the per-sub-face
/// map from the shared pools (the former clone) re-materializes pool-sized state
/// per sub-face → O(pool · sub-faces), quadratic. Crate-internal.
#[inline]
pub(crate) fn bump_local_vertex_insert() {
    #[cfg(feature = "perf-counters")]
    increment(&LOCAL_VERTEX_INSERTS);
}

/// Count one closest-point projection of a vertex onto a `Line` edge that took
/// the analytic closed-form path (PERF-B01). Crate-internal: only
/// `reset`/`snapshot` cross the crate boundary (for the scaling guard).
/// Reserved for the future fast path; the current sampler never bumps it.
#[inline]
#[allow(dead_code)]
pub(crate) fn bump_ve_line_projection() {
    #[cfg(feature = "perf-counters")]
    increment(&VE_LINE_PROJECTIONS);
}

/// Count one closest-point projection that took the generic sampled path
/// (33 samples + 20 ternary refinement steps): every non-`Line` pair, plus
/// degenerate `Line` segments that cannot project analytically. Crate-internal.
#[inline]
pub(crate) fn bump_ve_sampled_projection() {
    #[cfg(feature = "perf-counters")]
    increment(&VE_SAMPLED_PROJECTIONS);
}

/// Count one curve evaluation inside the VE closest-point projection.
/// A generic-path projection performs exactly 73 (33 samples + 2 × 20 ternary
/// steps); an analytic `Line` projection performs none. Crate-internal.
#[inline]
pub(crate) fn bump_ve_projection_eval() {
    #[cfg(feature = "perf-counters")]
    increment(&VE_PROJECTION_EVALS);
}

/// Count one pave endpoint seeded into a phase-FF junction registry. The
/// two-solid driver seeds once per boolean; the N-way driver seeds once per
/// run and shares the result across its solid pairs, so the count is linear
/// in the arena's paves rather than pairs × paves. Crate-internal.
#[inline]
pub(crate) fn bump_junction_seed() {
    #[cfg(feature = "perf-counters")]
    increment(&JUNCTION_SEEDS);
}

/// Count one edge-face pair that runs a sampled scan against an analytic
/// carrier: a curved edge against a plane, or any edge against a cylinder,
/// cone, sphere or torus. Crate-internal.
#[inline]
pub(crate) fn bump_ef_analytic_pair_scan() {
    #[cfg(feature = "perf-counters")]
    increment(&EF_ANALYTIC_PAIR_SCANS);
}

/// Count one edge-face pair that an analytic gate proved crossing-free and
/// skipped before its sampled scan. Crate-internal.
#[inline]
pub(crate) fn bump_ef_analytic_pair_gated() {
    #[cfg(feature = "perf-counters")]
    increment(&EF_ANALYTIC_PAIRS_GATED);
}

/// Count one winding-loop cut projected onto a closed section loop (one
/// that survived the loop's reach-box gate). Crate-internal.
#[inline]
pub(crate) fn bump_winding_cut_projection() {
    #[cfg(feature = "perf-counters")]
    increment(&WINDING_CUT_PROJECTIONS);
}

/// A snapshot of every work counter since the last [`reset`]. Only available
/// with `perf-counters`.
#[cfg(feature = "perf-counters")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PerfSnapshot {
    /// Pave-vertex coincidence-lookup candidate comparisons.
    pub pave_vertex_probes: u64,
    /// Same-domain polygon-intersection clips (the expensive narrow-phase).
    pub sd_poly_clips: u64,
    /// Ray-cast geometry collections (`collect_face_geoms` calls).
    pub ray_geom_builds: u64,
    /// Face-splitter candidate work: grid-query endpoints examined plus
    /// arrangement chord pairs that survive the bbox broad-phase.
    pub face_split_probes: u64,
    /// Sub-face-local vertex materializations in `build_topology_face`.
    pub local_vertex_inserts: u64,
    /// Edge x NURBS-face pairs that reached surface projection in the
    /// edge-face interference phase (after the conservative box gate).
    pub ef_nurbs_pair_probes: u64,
    /// Vertex x face distance evaluations in `solid_to_solid_distance`
    /// after bounding-box pruning.
    pub distance_face_probes: u64,
    /// VE `Line` pairs projected by the analytic closed form.
    pub ve_line_projections: u64,
    /// VE pairs that ran the generic sampled projection.
    pub ve_sampled_probes: u64,
    /// Curve evaluations performed inside VE projections.
    pub ve_projection_evals: u64,
    /// Pave endpoints seeded into phase-FF junction registries.
    pub junction_seeds: u64,
    /// EF pairs that ran a sampled scan against an analytic carrier.
    pub ef_analytic_pair_scans: u64,
    /// EF pairs an analytic gate proved crossing-free and skipped.
    pub ef_analytic_pairs_gated: u64,
    /// Winding-loop cuts projected onto closed section loops.
    pub winding_cut_projections: u64,
}

/// Reset all counters to zero. Only available with `perf-counters`.
#[cfg(feature = "perf-counters")]
pub fn reset() {
    PAVE_VERTEX_PROBES.set(0);
    SD_POLY_CLIPS.set(0);
    RAY_GEOM_BUILDS.set(0);
    FACE_SPLIT_PROBES.set(0);
    LOCAL_VERTEX_INSERTS.set(0);
    EF_NURBS_PAIR_PROBES.set(0);
    DISTANCE_FACE_PROBES.set(0);
    VE_LINE_PROJECTIONS.set(0);
    VE_SAMPLED_PROJECTIONS.set(0);
    VE_PROJECTION_EVALS.set(0);
    JUNCTION_SEEDS.set(0);
    EF_ANALYTIC_PAIR_SCANS.set(0);
    EF_ANALYTIC_PAIRS_GATED.set(0);
    WINDING_CUT_PROJECTIONS.set(0);
}

/// Every work counter since the last [`reset`]. Only available with
/// `perf-counters`.
#[cfg(feature = "perf-counters")]
#[must_use]
pub fn snapshot() -> PerfSnapshot {
    PerfSnapshot {
        pave_vertex_probes: PAVE_VERTEX_PROBES.get(),
        sd_poly_clips: SD_POLY_CLIPS.get(),
        ray_geom_builds: RAY_GEOM_BUILDS.get(),
        face_split_probes: FACE_SPLIT_PROBES.get(),
        local_vertex_inserts: LOCAL_VERTEX_INSERTS.get(),
        ef_nurbs_pair_probes: EF_NURBS_PAIR_PROBES.get(),
        distance_face_probes: DISTANCE_FACE_PROBES.get(),
        ve_line_projections: VE_LINE_PROJECTIONS.get(),
        ve_sampled_probes: VE_SAMPLED_PROJECTIONS.get(),
        ve_projection_evals: VE_PROJECTION_EVALS.get(),
        junction_seeds: JUNCTION_SEEDS.get(),
        ef_analytic_pair_scans: EF_ANALYTIC_PAIR_SCANS.get(),
        ef_analytic_pairs_gated: EF_ANALYTIC_PAIRS_GATED.get(),
        winding_cut_projections: WINDING_CUT_PROJECTIONS.get(),
    }
}

#[cfg(all(test, feature = "perf-counters"))]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::expect_used)]
    fn counters_are_isolated_between_test_threads() {
        reset();
        bump_ray_geom_build();

        let worker = std::thread::spawn(|| {
            reset();
            bump_ray_geom_build();
            bump_ray_geom_build();
            snapshot().ray_geom_builds
        });

        assert_eq!(worker.join().expect("worker counter test panicked"), 2);
        assert_eq!(snapshot().ray_geom_builds, 1);
    }
}
