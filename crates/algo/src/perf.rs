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
//! The counters are gated behind the `perf-counters` feature. With the feature
//! off (every normal and release build) the `bump_*` calls are empty `#[inline]`
//! functions that compile to nothing, so the instrumented hot loops pay zero
//! cost. The scaling guard enables the feature only for its own test build.

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
#[inline]
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
}

/// Reset all counters to zero. Only available with `perf-counters`.
#[cfg(feature = "perf-counters")]
pub fn reset() {
    PAVE_VERTEX_PROBES.set(0);
    SD_POLY_CLIPS.set(0);
    RAY_GEOM_BUILDS.set(0);
    FACE_SPLIT_PROBES.set(0);
    LOCAL_VERTEX_INSERTS.set(0);
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
