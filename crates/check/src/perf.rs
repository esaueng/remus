//! Deterministic work counters for the point-classification hot path (PERF-Q01).
//!
//! These count the *preparation work* of ray-cast point classification — the
//! per-ray face-bound collection, BVH construction, and per-face trim-polygon
//! builds that [`crate::classify::PreparedSolid`] performs once per solid
//! instead of once per ray. A test can therefore assert the preparation
//! reduction deterministically: classifying `n` points through the one-shot
//! path bumps `classify_bvh_builds` once per ray (two to three per point, plus
//! perturbed recovery rays), while the prepared path bumps it exactly once.
//!
//! The counters are gated behind the `perf-counters` feature. With the feature
//! off (every normal and release build) the `bump_*` calls are empty `#[inline]`
//! functions that compile to nothing, so the instrumented paths pay zero cost.
//! The reuse guard enables the feature only for its own test build.
//!
//! Classification runs synchronously within one caller thread. Thread-local
//! counters keep the feature deterministic when the Rust test harness runs
//! unrelated classification tests concurrently with the reuse guard.

#[cfg(feature = "perf-counters")]
use std::cell::Cell;

#[cfg(feature = "perf-counters")]
std::thread_local! {
    static CLASSIFY_BVH_BUILDS: Cell<u64> = const { Cell::new(0) };
    static CLASSIFY_FACE_AABB_EVALS: Cell<u64> = const { Cell::new(0) };
    static CLASSIFY_TRIM_BUILDS: Cell<u64> = const { Cell::new(0) };
}

#[cfg(feature = "perf-counters")]
fn increment(counter: &'static std::thread::LocalKey<Cell<u64>>) {
    counter.with(|value| value.set(value.get().saturating_add(1)));
}

/// Count one BVH construction over the solid's face bounds, whether built
/// per-ray by the one-shot path or once per solid by the prepared path.
/// Crate-internal: only `reset`/`snapshot` cross the crate boundary (for the
/// reuse guard).
#[inline]
pub(crate) fn bump_classify_bvh_build() {
    #[cfg(feature = "perf-counters")]
    increment(&CLASSIFY_BVH_BUILDS);
}

/// Count one face-bound evaluation feeding a classification BVH (one face's
/// `face_aabb`, including faces whose bounds fail and are pruned, exactly as
/// the one-shot filter sees them). Crate-internal, like
/// `bump_classify_bvh_build`.
#[inline]
pub(crate) fn bump_classify_face_aabb_eval() {
    #[cfg(feature = "perf-counters")]
    increment(&CLASSIFY_FACE_AABB_EVALS);
}

/// Count one per-face trim-polygon build (outer wire plus inner wires) backing
/// the boundary and crossing containment tests. The one-shot path builds one
/// per candidate face per ray; the prepared path builds one per face per
/// solid. Crate-internal, like `bump_classify_bvh_build`.
#[inline]
pub(crate) fn bump_classify_trim_build() {
    #[cfg(feature = "perf-counters")]
    increment(&CLASSIFY_TRIM_BUILDS);
}

/// A snapshot of every classification work counter since the last [`reset`].
/// Only available with `perf-counters`.
#[cfg(feature = "perf-counters")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PerfSnapshot {
    /// BVH constructions over the solid's face bounds.
    pub classify_bvh_builds: u64,
    /// Face-bound evaluations feeding those BVHs.
    pub classify_face_aabb_evals: u64,
    /// Per-face trim-polygon builds.
    pub classify_trim_builds: u64,
}

/// Reset all counters to zero. Only available with `perf-counters`.
#[cfg(feature = "perf-counters")]
pub fn reset() {
    CLASSIFY_BVH_BUILDS.set(0);
    CLASSIFY_FACE_AABB_EVALS.set(0);
    CLASSIFY_TRIM_BUILDS.set(0);
}

/// Every classification work counter since the last [`reset`]. Only available
/// with `perf-counters`.
#[cfg(feature = "perf-counters")]
#[must_use]
pub fn snapshot() -> PerfSnapshot {
    PerfSnapshot {
        classify_bvh_builds: CLASSIFY_BVH_BUILDS.get(),
        classify_face_aabb_evals: CLASSIFY_FACE_AABB_EVALS.get(),
        classify_trim_builds: CLASSIFY_TRIM_BUILDS.get(),
    }
}

#[cfg(all(test, feature = "perf-counters"))]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn counters_are_isolated_between_test_threads() {
        reset();
        bump_classify_bvh_build();

        let worker = std::thread::spawn(|| {
            reset();
            bump_classify_bvh_build();
            bump_classify_bvh_build();
            snapshot().classify_bvh_builds
        });

        assert_eq!(worker.join().expect("worker counter test panicked"), 2);
        assert_eq!(snapshot().classify_bvh_builds, 1);
    }
}
