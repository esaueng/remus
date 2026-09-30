//! PERF-Q02 benefit probe: prints hit rate, rebuilds, retained bytes,
//! and preparation counters for repeated / occasional / constant edits.
//!
//! Run: `cargo run -p remus-operations --example perf_q02_probe
//! --features perf-counters --profile profiling`

#![allow(
    clippy::unwrap_used,
    clippy::missing_docs_in_private_items,
    clippy::print_stdout
)]

use remus_check::classify::{ClassificationCache, ClassifyOptions, classify_point};
use remus_math::vec::Point3;
use remus_operations::primitives::make_box;
use remus_topology::Topology;

fn halton(mut i: u32, base: u32) -> f64 {
    let (mut f, mut r) = (1.0_f64, 0.0_f64);
    while i > 0 {
        f /= f64::from(base);
        r += f * f64::from(i % base);
        i /= base;
    }
    r
}

fn points(n: u32) -> Vec<Point3> {
    (1..=n)
        .map(|i| {
            Point3::new(
                11.0f64.mul_add(halton(i, 2), -1.0),
                11.0f64.mul_add(halton(i, 3), -1.0),
                11.0f64.mul_add(halton(i, 5), -1.0),
            )
        })
        .collect()
}

fn main() {
    let options = ClassifyOptions::default();

    // Repeated: 1000 points, no edits.
    {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let pts = points(1000);
        let mut cache = ClassificationCache::new();
        #[cfg(feature = "perf-counters")]
        remus_check::perf::reset();
        let t0 = std::time::Instant::now();
        let cached = cache.classify_points(&topo, solid, &pts, &options).unwrap();
        let dt_cache_first = t0.elapsed();
        let t0 = std::time::Instant::now();
        let cached2 = cache.classify_points(&topo, solid, &pts, &options).unwrap();
        let dt_cache_hit = t0.elapsed();
        assert_eq!(cached, cached2);
        #[cfg(feature = "perf-counters")]
        let cache_counts = remus_check::perf::snapshot();
        let t0 = std::time::Instant::now();
        let mut oneshot = Vec::with_capacity(pts.len());
        for &p in &pts {
            oneshot.push(classify_point(&topo, solid, p, &options).unwrap());
        }
        let dt_oneshot = t0.elapsed();
        assert_eq!(cached, oneshot);
        let stats = cache.stats();
        println!(
            "repeated-1000: oneshot={:?} cache-first={:?} cache-hit={:?} hits={} misses={} rebuilds={} retained_bytes={} hit_rate={:.3}",
            dt_oneshot,
            dt_cache_first,
            dt_cache_hit,
            stats.hits,
            stats.misses,
            stats.rebuilds,
            stats.retained_bytes,
            cache.hit_rate().unwrap_or(-1.0),
        );
        #[cfg(feature = "perf-counters")]
        println!(
            "  perf-counters cache-build: bvh={} aabb={} trims={}",
            cache_counts.bvh_builds, cache_counts.face_aabb_evals, cache_counts.trim_builds
        );
    }

    // Occasional edits: 10 batches of 100 with unrelated allocation between.
    {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let pts = points(100);
        let mut cache = ClassificationCache::new();
        let t0 = std::time::Instant::now();
        for _ in 0..10 {
            let _ = cache.classify_points(&topo, solid, &pts, &options).unwrap();
            let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        }
        let dt = t0.elapsed();
        let stats = cache.stats();
        println!(
            "occasional-10x100: time={:?} hits={} misses={} rebuilds={} evictions={} retained={} hit_rate={:.3}",
            dt,
            stats.hits,
            stats.misses,
            stats.rebuilds,
            stats.evictions,
            stats.retained_bytes,
            cache.hit_rate().unwrap_or(-1.0),
        );
    }

    // Constant edits: allocation before every query.
    {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let pts = points(200);
        let mut cache = ClassificationCache::new();
        let t0 = std::time::Instant::now();
        for &p in &pts {
            let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            let _ = cache.classify_point(&topo, solid, p, &options).unwrap();
        }
        let dt_cache = t0.elapsed();
        let t0 = std::time::Instant::now();
        for &p in &pts {
            let _ = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
            let _ = classify_point(&topo, solid, p, &options).unwrap();
        }
        let dt_oneshot = t0.elapsed();
        let stats = cache.stats();
        println!(
            "constant-200: cache={:?} oneshot={:?} hits={} misses={} rebuilds={} (no speedup claimed when management exceeds saved work)",
            dt_cache, dt_oneshot, stats.hits, stats.misses, stats.rebuilds
        );
    }

    // Thrashing: 32 solids, capacity 16, round-robin.
    {
        let mut topo = Topology::new();
        let mut solids = Vec::new();
        for _ in 0..32 {
            solids.push(make_box(&mut topo, 10.0, 10.0, 10.0).unwrap());
        }
        let mut cache = ClassificationCache::with_capacity(16);
        let pt = Point3::new(5.0, 5.0, 5.0);
        for _ in 0..2 {
            for &s in &solids {
                let _ = cache.classify_point(&topo, s, pt, &options).unwrap();
            }
        }
        let stats = cache.stats();
        println!(
            "thrashing-32x2 cap16: hits={} misses={} rebuilds={} evictions={} len={} retained={} hit_rate={:.3} (correctness holds, no hit-rate win claimed)",
            stats.hits,
            stats.misses,
            stats.rebuilds,
            stats.evictions,
            stats.len,
            stats.retained_bytes,
            cache.hit_rate().unwrap_or(-1.0),
        );
    }
}
