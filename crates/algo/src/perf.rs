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
//! faces and distinct planes of each geometry build, and the torus faces a
//! ray skips because its line stays clear of the carrier.
//!
//! `winding_cut_projections` counts the winding-loop cuts projected onto a
//! closed section loop in `presplit_closed_winding_loops`. Every face carries
//! every loop's cuts, so without the reach-box gate the count grows with the
//! square of the number of loops; with it, each loop projects only its own.
//!
//! Two count the per-loop work of phase FF on a face carrying many loops (a
//! drilled plate's top, stage 3 of `docs/performance/perf-local-boolean.md`):
//! `junction_curve_scans`, the curved boundary edges the junction snap
//! samples (a box gate skips those out of reach), and `face_extent_builds`,
//! the face extents built (once per face per pass, not per pair). A third,
//! `hole_containment_samplings`, counts the hole loops the face splitter's
//! containment test samples (a box gate skips those a query lies outside).
//!
//! `coplanar_section_probes` counts the existing section curves the coplanar
//! phase examines before minting a section. Each check reads only the curves
//! naming its face pair; scanning every curve in the arena made the count
//! grow with the square of an N-way fuse's body count.
//!
//! The counters are gated behind the `perf-counters` feature. With the feature
//! off (every normal and release build) the `bump_*` calls are empty `#[inline]`
//! functions that compile to nothing, so the instrumented hot loops pay zero
//! cost. The scaling guard enables the feature only for its own test build.
//! The ray-work and winding-cut counters also count in this crate's own
//! tests: their oracles pin work a gate skips without changing any result.

#[cfg(feature = "perf-counters")]
use std::cell::Cell;

mod phases {
    //! Feature-only phase attribution with a caller-supplied monotonic clock.
    //!
    //! Counts are thread-local and bounded by the fixed phase inventory. Timings
    //! are diagnostic, inclusive when scopes nest, and never drive kernel policy.
    //! No system clock is read, so this also works on WASM when a host opts in.

    #[derive(Clone, Copy)]
    pub enum PhaseKind {
        StoreImport,
        Vv,
        Ve,
        Ee,
        Vf,
        Ef,
        Ff,
        Coplanar,
        Resolve,
        Build,
        Assemble,
        StoreExport,
        BuildNway,
    }

    #[cfg(any(test, feature = "perf-counters"))]
    const NAMES: [&str; 13] = [
        "gfa.store_import",
        "pave.vv",
        "pave.ve",
        "pave.ee",
        "pave.vf",
        "pave.ef",
        "pave.ff",
        "pave.coplanar",
        "pave.resolve",
        "gfa.build",
        "gfa.assemble",
        "gfa.store_export",
        "gfa.build_nway",
    ];

    /// One phase's work since the last drain or counter reset.
    #[cfg(any(test, feature = "perf-counters"))]
    #[derive(Debug, Clone)]
    pub struct PhaseSnapshot {
        /// Stable label, with no model or document content.
        pub name: &'static str,
        /// Completed scopes, including scopes returning an error.
        pub calls: u64,
        /// Inclusive host milliseconds; zero when no phase clock was enabled.
        pub duration_ms: f64,
    }

    #[cfg(any(test, feature = "perf-counters"))]
    struct State {
        clock: Option<fn() -> f64>,
        epoch: u64,
        calls: [u64; 13],
        duration_ms: [f64; 13],
    }

    #[cfg(any(test, feature = "perf-counters"))]
    thread_local! {
        static STATE: std::cell::RefCell<State> = const {
            std::cell::RefCell::new(State {
                clock: None, epoch: 0, calls: [0; 13], duration_ms: [0.0; 13],
            })
        };
    }

    /// Configure diagnostic timing on this thread and clear old measurements.
    /// Passing `None` keeps phase call counts but stops reading the host clock.
    #[cfg(any(test, feature = "perf-counters"))]
    pub fn configure_phase_clock(clock: Option<fn() -> f64>) {
        STATE.with(|state| {
            let mut state = state.borrow_mut();
            state.clock = clock;
        });
        reset();
    }

    #[cfg(any(test, feature = "perf-counters"))]
    pub(super) fn reset() {
        STATE.with(|state| {
            let mut state = state.borrow_mut();
            state.epoch = state.epoch.wrapping_add(1);
            state.calls.fill(0);
            state.duration_ms.fill(0.0);
        });
    }

    /// Drain the fixed inventory in stable order, including only visited phases.
    /// Draining also invalidates scopes started before the drain.
    #[cfg(any(test, feature = "perf-counters"))]
    #[must_use]
    pub fn take_phases() -> Vec<PhaseSnapshot> {
        let phases = STATE.with(|state| {
            let state = state.borrow();
            NAMES
                .iter()
                .enumerate()
                .filter_map(|(i, &name)| {
                    (state.calls[i] != 0).then_some(PhaseSnapshot {
                        name,
                        calls: state.calls[i],
                        duration_ms: state.duration_ms[i],
                    })
                })
                .collect()
        });
        reset();
        phases
    }

    pub struct PhaseSpan {
        #[cfg(any(test, feature = "perf-counters"))]
        phase: PhaseKind,
        #[cfg(any(test, feature = "perf-counters"))]
        epoch: u64,
        #[cfg(any(test, feature = "perf-counters"))]
        timing: Option<(fn() -> f64, f64)>,
    }

    #[inline]
    pub fn phase(kind: PhaseKind) -> PhaseSpan {
        #[cfg(any(test, feature = "perf-counters"))]
        {
            let (clock, epoch) = STATE.with(|state| {
                let state = state.borrow();
                (state.clock, state.epoch)
            });
            PhaseSpan {
                phase: kind,
                epoch,
                timing: clock.map(|clock| (clock, clock())),
            }
        }
        #[cfg(not(any(test, feature = "perf-counters")))]
        {
            let _ = kind;
            PhaseSpan {}
        }
    }

    impl Drop for PhaseSpan {
        #[inline]
        fn drop(&mut self) {
            #[cfg(any(test, feature = "perf-counters"))]
            {
                let elapsed = self.timing.map_or(0.0, |(clock, start)| {
                    let delta = clock() - start;
                    if delta.is_finite() {
                        delta.max(0.0)
                    } else {
                        0.0
                    }
                });
                STATE.with(|state| {
                    let mut state = state.borrow_mut();
                    if state.epoch == self.epoch {
                        let i = self.phase as usize;
                        state.calls[i] = state.calls[i].saturating_add(1);
                        state.duration_ms[i] = (state.duration_ms[i] + elapsed).min(f64::MAX);
                    }
                });
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        thread_local! { static CLOCK: std::cell::Cell<f64> = const { std::cell::Cell::new(0.0) }; }
        fn clock() -> f64 {
            CLOCK.get()
        }

        #[test]
        fn nested_phases_are_inclusive_and_reset_scopes_cannot_leak() {
            configure_phase_clock(Some(clock));
            CLOCK.set(0.0);
            let outer = phase(PhaseKind::Build);
            CLOCK.set(2.0);
            let inner = phase(PhaseKind::Assemble);
            CLOCK.set(5.0);
            drop(inner);
            drop(outer);
            let measured = take_phases();
            assert_eq!(
                measured
                    .iter()
                    .map(|p| (p.name, p.calls, p.duration_ms))
                    .collect::<Vec<_>>(),
                [("gfa.build", 1, 5.0), ("gfa.assemble", 1, 3.0)]
            );
            let stale = phase(PhaseKind::Build);
            configure_phase_clock(None);
            drop(stale);
            assert!(take_phases().is_empty());
            drop(phase(PhaseKind::Build));
            let untimed = take_phases();
            assert_eq!(untimed[0].calls, 1);
            assert_eq!(untimed[0].duration_ms.to_bits(), 0.0_f64.to_bits());
        }

        #[test]
        fn nonfinite_or_backward_clock_never_produces_invalid_timing() {
            configure_phase_clock(Some(clock));
            CLOCK.set(2.0);
            let backward = phase(PhaseKind::Ve);
            CLOCK.set(1.0);
            drop(backward);
            let nonfinite = phase(PhaseKind::Ee);
            CLOCK.set(f64::NAN);
            drop(nonfinite);
            assert!(take_phases().iter().all(|p| p.duration_ms == 0.0));
            configure_phase_clock(None);
        }
    }
}
pub(crate) use phases::{PhaseKind, phase};
#[cfg(feature = "perf-counters")]
pub use phases::{PhaseSnapshot, configure_phase_clock, take_phases};

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
    static JUNCTION_CURVE_SCANS: Cell<u64> = const { Cell::new(0) };
    static FACE_EXTENT_BUILDS: Cell<u64> = const { Cell::new(0) };
    static HOLE_CONTAINMENT_SAMPLINGS: Cell<u64> = const { Cell::new(0) };
    static FACE_BBOX_BUILDS: Cell<u64> = const { Cell::new(0) };
    static FACE_OUTLINE_SAMPLINGS: Cell<u64> = const { Cell::new(0) };
    static STORE_FACES_COPIED: Cell<u64> = const { Cell::new(0) };
    static JUNCTION_CELLS_COPIED: Cell<u64> = const { Cell::new(0) };
    static COPLANAR_SECTION_PROBES: Cell<u64> = const { Cell::new(0) };
    static VE_PAIRS_VISITED: Cell<u64> = const { Cell::new(0) };
    static EE_PAIRS_VISITED: Cell<u64> = const { Cell::new(0) };
    static EE_PAIRS_SURVIVING: Cell<u64> = const { Cell::new(0) };
    static FF_PAIRS_VISITED: Cell<u64> = const { Cell::new(0) };
    static FF_PAIRS_SURVIVING: Cell<u64> = const { Cell::new(0) };
}

/// Count candidate enumeration separately from expensive narrow-phase work.
#[inline]
pub(crate) fn bump_ve_pair_visited() {
    #[cfg(feature = "perf-counters")]
    increment(&VE_PAIRS_VISITED);
}

#[inline]
pub(crate) fn bump_ee_pair_visited() {
    #[cfg(feature = "perf-counters")]
    increment(&EE_PAIRS_VISITED);
}

#[inline]
pub(crate) fn bump_ee_pair_surviving() {
    #[cfg(feature = "perf-counters")]
    increment(&EE_PAIRS_SURVIVING);
}

#[inline]
pub(crate) fn bump_ff_pair_visited() {
    #[cfg(feature = "perf-counters")]
    increment(&FF_PAIRS_VISITED);
}

#[inline]
pub(crate) fn bump_ff_pair_surviving() {
    #[cfg(feature = "perf-counters")]
    increment(&FF_PAIRS_SURVIVING);
}

#[cfg(any(test, feature = "perf-counters"))]
std::thread_local! {
    static RAY_WORK: std::cell::Cell<[u64; 9]> = const { std::cell::Cell::new([0; 9]) };
    static WINDING_CUT_PROJECTIONS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
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
    TorusCull,
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
    /// Torus faces a ray skipped because its line provably stays clear of
    /// the carrier.
    pub torus_culls: u64,
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
        torus_culls,
    ] = RAY_WORK.with(|work| work.replace([0; 9]));
    RayWorkCounts {
        votes,
        plane_evals,
        polygon_tests,
        face_skips,
        segment_evals,
        windings,
        planar_faces,
        plane_groups,
        torus_culls,
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

/// Count one curved boundary edge sampled by the phase-FF junction snap
/// (one that survived its box gate). Crate-internal.
#[inline]
pub(crate) fn bump_junction_curve_scan() {
    #[cfg(feature = "perf-counters")]
    increment(&JUNCTION_CURVE_SCANS);
}

/// Count one phase-FF face extent built from a face's boundary.
/// Crate-internal.
#[inline]
pub(crate) fn bump_face_extent_build() {
    #[cfg(feature = "perf-counters")]
    increment(&FACE_EXTENT_BUILDS);
}

/// Count one sampled phase-FF face box. Crate-internal.
#[inline]
pub(crate) fn bump_face_bbox_build() {
    #[cfg(feature = "perf-counters")]
    increment(&FACE_BBOX_BUILDS);
}

/// Count one operand face deep-copied into a GFA store. Crate-internal.
#[inline]
pub(crate) fn bump_store_face_copied() {
    #[cfg(feature = "perf-counters")]
    increment(&STORE_FACES_COPIED);
}

/// Count one existing section curve the coplanar phase examined for a face
/// pair. Crate-internal.
#[inline]
pub(crate) fn bump_coplanar_section_probe() {
    #[cfg(feature = "perf-counters")]
    increment(&COPLANAR_SECTION_PROBES);
}

/// Count `cells` junction-registry cells copied when a phase-FF solid pair
/// first writes the shared seeded cells. Crate-internal.
#[inline]
pub(crate) fn bump_junction_cells_copied(cells: usize) {
    #[cfg(feature = "perf-counters")]
    JUNCTION_CELLS_COPIED.with(|value| {
        value.set(value.get().saturating_add(cells as u64));
    });
    #[cfg(not(feature = "perf-counters"))]
    let _ = cells;
}

/// Count one face whose outline phase EF sampled for its containment test.
/// Crate-internal.
#[inline]
pub(crate) fn bump_face_outline_sampling() {
    #[cfg(feature = "perf-counters")]
    increment(&FACE_OUTLINE_SAMPLINGS);
}

/// Count one hole loop the face splitter's containment test sampled (one
/// that survived its box gate). Crate-internal.
#[inline]
pub(crate) fn bump_hole_containment_sampling() {
    #[cfg(feature = "perf-counters")]
    increment(&HOLE_CONTAINMENT_SAMPLINGS);
}

/// Count one winding-loop cut projected onto a closed section loop (one
/// that survived the loop's reach-box gate). Crate-internal.
#[inline]
pub(crate) fn bump_winding_cut_projection() {
    #[cfg(any(test, feature = "perf-counters"))]
    WINDING_CUT_PROJECTIONS.with(|count| count.set(count.get().saturating_add(1)));
}

/// This thread's winding-loop cut projections since the previous call,
/// resetting the count. Available in this crate's tests and with
/// `perf-counters`.
#[cfg(any(test, feature = "perf-counters"))]
#[must_use]
pub fn take_winding_cut_projections() -> u64 {
    WINDING_CUT_PROJECTIONS.with(|count| count.replace(0))
}

/// A snapshot of every work counter since the last [`reset`]. Only available
/// with `perf-counters`.
#[cfg(feature = "perf-counters")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PerfSnapshot {
    /// VE Cartesian pairs visited, before bounds and endpoint rejection.
    pub ve_pairs_visited: u64,
    /// EE Cartesian pairs visited, including shared-edge rejection.
    pub ee_pairs_visited: u64,
    /// EE pairs reaching the crossing solver after bounds rejection.
    pub ee_pairs_surviving: u64,
    /// FF Cartesian pairs visited, before bounds rejection.
    pub ff_pairs_visited: u64,
    /// FF pairs passing the existing AABB gate, before carrier dispatch.
    pub ff_pairs_surviving: u64,
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
    /// Curved boundary edges the phase-FF junction snap sampled.
    pub junction_curve_scans: u64,
    /// Phase-FF face extents built from face boundaries.
    pub face_extent_builds: u64,
    /// Hole loops the face splitter's containment test sampled.
    pub hole_containment_samplings: u64,
    /// Sampled phase-FF face boxes.
    pub face_bbox_builds: u64,
    /// Faces whose outline phase EF sampled for containment.
    pub face_outline_samplings: u64,
    /// Operand faces deep-copied into a GFA store.
    pub store_faces_copied: u64,
    /// Existing section curves the coplanar phase examined for a face pair.
    pub coplanar_section_probes: u64,
    /// Seeded junction cells copied by phase-FF solid pairs that wrote them.
    pub junction_cells_copied: u64,
}

/// Reset all counters to zero. Only available with `perf-counters`.
#[cfg(feature = "perf-counters")]
pub fn reset() {
    phases::reset();
    VE_PAIRS_VISITED.set(0);
    EE_PAIRS_VISITED.set(0);
    EE_PAIRS_SURVIVING.set(0);
    FF_PAIRS_VISITED.set(0);
    FF_PAIRS_SURVIVING.set(0);
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
    JUNCTION_CURVE_SCANS.set(0);
    FACE_EXTENT_BUILDS.set(0);
    HOLE_CONTAINMENT_SAMPLINGS.set(0);
    FACE_BBOX_BUILDS.set(0);
    FACE_OUTLINE_SAMPLINGS.set(0);
    STORE_FACES_COPIED.set(0);
    COPLANAR_SECTION_PROBES.set(0);
    JUNCTION_CELLS_COPIED.set(0);
}

/// Every work counter since the last [`reset`]. Only available with
/// `perf-counters`.
#[cfg(feature = "perf-counters")]
#[must_use]
pub fn snapshot() -> PerfSnapshot {
    PerfSnapshot {
        ve_pairs_visited: VE_PAIRS_VISITED.get(),
        ee_pairs_visited: EE_PAIRS_VISITED.get(),
        ee_pairs_surviving: EE_PAIRS_SURVIVING.get(),
        ff_pairs_visited: FF_PAIRS_VISITED.get(),
        ff_pairs_surviving: FF_PAIRS_SURVIVING.get(),
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
        junction_curve_scans: JUNCTION_CURVE_SCANS.get(),
        face_extent_builds: FACE_EXTENT_BUILDS.get(),
        hole_containment_samplings: HOLE_CONTAINMENT_SAMPLINGS.get(),
        face_bbox_builds: FACE_BBOX_BUILDS.get(),
        face_outline_samplings: FACE_OUTLINE_SAMPLINGS.get(),
        store_faces_copied: STORE_FACES_COPIED.get(),
        coplanar_section_probes: COPLANAR_SECTION_PROBES.get(),
        junction_cells_copied: JUNCTION_CELLS_COPIED.get(),
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
