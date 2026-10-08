//! The boolean's log-only validation report is built only when its warning
//! can be emitted, and skipping it changes nothing but the log.
//!
//! The logger and level filter are process-global, so this binary holds a
//! single test.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::atomic::{AtomicUsize, Ordering};

use log::{LevelFilter, Log, Metadata, Record};
use remus_check::properties::face_cache::{
    clear_thread_face_cache, enable_thread_face_cache, set_thread_face_cache_limits,
    thread_face_cache_stats,
};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces, solid_vertices};

/// Accepts every record the global level filter lets through.
struct CountingLogger;

static RECORDS: AtomicUsize = AtomicUsize::new(0);

impl Log for CountingLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            RECORDS.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn flush(&self) {}
}

static LOGGER: CountingLogger = CountingLogger;

/// Turns the thread face cache back off on drop.
struct FaceCache;

impl Drop for FaceCache {
    fn drop(&mut self) {
        set_thread_face_cache_limits(0, 0);
    }
}

/// Box minus a through cylinder; returns a bitwise fingerprint of the result
/// and the face-integral cache misses the boolean incurred.
fn drilled_plate() -> (Vec<u64>, u64) {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    let bore = make_cylinder(&mut topo, 2.0, 20.0).unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(50.0, 50.0, -5.0)).unwrap();

    clear_thread_face_cache();
    let before = thread_face_cache_stats().misses;
    let result = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();
    let misses = thread_face_cache_stats().misses - before;

    let (f, e, v) = solid_entity_counts(&topo, result).unwrap();
    let mut fingerprint = vec![f as u64, e as u64, v as u64];
    fingerprint.push(solid_faces(&topo, result).unwrap().len() as u64);
    for vid in solid_vertices(&topo, result).unwrap() {
        let p = topo.vertex(vid).unwrap().point();
        fingerprint.extend([p.x().to_bits(), p.y().to_bits(), p.z().to_bits()]);
    }
    fingerprint.push(solid_volume(&topo, result, 0.01).unwrap().to_bits());
    (fingerprint, misses)
}

#[test]
fn boolean_soft_validation_runs_only_for_a_live_warn_listener() {
    log::set_logger(&LOGGER).unwrap();
    let _cache = FaceCache;
    enable_thread_face_cache();

    log::set_max_level(LevelFilter::Off);
    let (quiet, quiet_misses) = drilled_plate();

    log::set_max_level(LevelFilter::Warn);
    let (loud, loud_misses) = drilled_plate();
    log::set_max_level(LevelFilter::Off);

    assert_eq!(
        quiet, loud,
        "skipping the log-only report must not change the boolean result"
    );
    assert!(
        quiet_misses < loud_misses,
        "the report's orientation probe must run only with Warn enabled \
         (face-integral misses: off {quiet_misses}, warn {loud_misses})"
    );
}
