//! Edit-aware measurement on the Shapr3D hammer holder (PERF-V02 / Q02 /
//! O07): with both measurement memos on — as the WASM kernel runs them — the
//! −6 mm move of the 1045.93 mm² +X face reads every face it did not
//! re-limit from the face-integral cache; the application's first reading of
//! the result's volume, after a copy-on-write clone and an unrelated
//! allocation, is a measurement bit-identical to a cold kernel's, and every
//! reading after it is a hit.

#![allow(clippy::expect_used, clippy::unwrap_used, clippy::cast_precision_loss)]

use remus_check::properties::face_cache::{
    enable_thread_face_cache, set_thread_face_cache_limits, thread_face_cache_stats,
};
use remus_io::step::reader::read_step;
use remus_operations::measure::{
    enable_thread_volume_memo, face_area, set_thread_volume_memo_capacity, solid_volume,
    thread_volume_memo_stats,
};
use remus_operations::primitives::make_box;
use remus_operations::push_pull::move_faces;
use remus_operations::validate::{ValidationOptions, validate_solid_with_budget_probes};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("data/shapr3d_hammer_holder.step");

/// Turns both memos off on drop.
struct MemosOff;

impl MemosOff {
    fn new() -> Self {
        set_thread_face_cache_limits(0, 0);
        set_thread_volume_memo_capacity(0);
        Self
    }
}

impl Drop for MemosOff {
    fn drop(&mut self) {
        set_thread_face_cache_limits(0, 0);
        set_thread_volume_memo_capacity(0);
    }
}

/// The +X planar face whose area is closest to 1045.93 mm².
fn plus_x_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { normal, .. } if normal.x() > 0.99
            )
        })
        .min_by(|&a, &b| {
            let gap = |face| (face_area(topo, face, 0.05).unwrap() - 1045.93).abs();
            gap(a).total_cmp(&gap(b))
        })
        .unwrap()
}

/// The strict probe's signed volume and verdict.
fn probe(topo: &Topology, solid: SolidId) -> (bool, f64) {
    let (report, probes) =
        validate_solid_with_budget_probes(topo, solid, &ValidationOptions::default()).unwrap();
    (report.is_valid(), probes.shells[0].signed_volume)
}

#[test]
fn hammer_move_reads_its_unchanged_faces_and_measures_its_volume_once() {
    let _off = MemosOff::new();
    let mut topo = Topology::new();
    let solid = read_step(HAMMER_HOLDER, &mut topo).expect("import")[0];
    let face = plus_x_face(&topo, solid);
    let faces = solid_faces(&topo, solid).unwrap().len() as u64;

    // Import: the application measures and strictly validates the source.
    enable_thread_face_cache();
    enable_thread_volume_memo();
    let source = solid_volume(&topo, solid, 0.08).unwrap();
    assert!(probe(&topo, solid).0);

    // The edit runs on the application's copy-on-write clone.
    let mut edit = topo.clone();
    let before = (thread_face_cache_stats(), thread_volume_memo_stats());
    let moved = move_faces(&mut edit, solid, &[face], -6.0).expect("move");
    let during = (thread_face_cache_stats(), thread_volume_memo_stats());
    let integrated = during.0.misses - before.0.misses;
    // Only the faces the move re-limits (the carriers whose boundary slid)
    // are integrated; every unchanged face is re-expressed about the
    // result's reference and every moved one read as a translation. 14 on
    // this body when recorded.
    assert!(
        integrated <= 20,
        "the move integrated {integrated} faces afresh"
    );
    assert!(
        during.0.translated - before.0.translated >= 20,
        "moved faces are read as translations"
    );
    assert!(
        during.0.rereferenced - before.0.rereferenced >= 100,
        "unchanged faces are re-expressed about the result's reference"
    );
    assert_eq!(during.1.misses, before.1.misses, "no whole-body volume");

    // What the application does next, on a checkpoint clone and after an
    // unrelated allocation.
    let mut after = edit.clone();
    make_box(&mut after, 1.0, 1.0, 1.0).unwrap();
    let start = (thread_face_cache_stats(), thread_volume_memo_stats());
    let (valid, signed) = probe(&after, moved);
    let read = solid_volume(&after, moved, 0.08).unwrap();
    let end = (thread_face_cache_stats(), thread_volume_memo_stats());
    assert!(valid);
    assert_eq!(end.0.misses, start.0.misses, "the result's probe only hits");
    assert_eq!(end.0.hits - start.0.hits, faces);
    assert_eq!(
        (end.1.misses - start.1.misses, end.1.hits - start.1.hits),
        (1, 0),
        "the result's first volume is a measurement, not a derived value"
    );
    let again = solid_volume(&after, moved, 0.08).unwrap();
    let repeat = thread_volume_memo_stats();
    assert_eq!(again.to_bits(), read.to_bits());
    assert_eq!(repeat.hits - end.1.hits, 1, "the second reading is a hit");

    // Against fresh readings with the memos off.
    set_thread_face_cache_limits(0, 0);
    set_thread_volume_memo_capacity(0);
    let (fresh_valid, fresh_signed) = probe(&after, moved);
    assert!(fresh_valid);
    assert!(
        (signed - fresh_signed).abs() <= fresh_signed.abs() * 1e-12,
        "probe {signed} vs fresh {fresh_signed}"
    );
    let fresh = solid_volume(&after, moved, 0.08).unwrap();
    assert_eq!(
        read.to_bits(),
        fresh.to_bits(),
        "a warm memo must read exactly what a cold kernel reads: {read} vs {fresh}"
    );
    // The swept prism of a 1045.93 mm² face pushed in by 6 mm (its blends
    // and carriers included).
    assert!(read < source - 6.0 * 1045.93 * 0.9 && read > source - 6.0 * 1045.93 * 1.7);
}
