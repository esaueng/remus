//! The opt-in measurement memos — the content-keyed face-integral cache
//! behind strict validation's orientation probe and `face_area`, and the
//! identity- and content-keyed `solid_volume` memo — must be invisible:
//! every reading of unchanged content through them is bit-identical to the
//! uncached computation, a re-expressed or translated face integral is
//! within rounding of it, and no in-place mutation can make a memoized
//! reading stale.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::properties::face_cache::{
    enable_thread_face_cache, set_thread_face_cache_limits, thread_face_cache_stats,
};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{
    enable_thread_volume_memo, face_area, set_thread_volume_memo_capacity,
    set_thread_volume_memo_limits, solid_volume, thread_volume_memo_stats,
};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_operations::validate::{ValidationOptions, validate_solid_with_budget_probes};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Disables both memos on drop, so a failing test cannot leave them on for
/// a later test that shares the thread.
struct Memos;

impl Memos {
    fn off() -> Self {
        set_thread_face_cache_limits(0, 0);
        set_thread_volume_memo_capacity(0);
        Self
    }
}

impl Drop for Memos {
    fn drop(&mut self) {
        set_thread_face_cache_limits(0, 0);
        set_thread_volume_memo_capacity(0);
    }
}

fn bodies(topo: &mut Topology) -> Vec<(&'static str, SolidId)> {
    let block = make_box(topo, 4.0, 3.0, 2.0).unwrap();
    let bore = make_cylinder(topo, 0.6, 4.0).unwrap();
    transform_solid(topo, bore, &Mat4::translation(2.0, 1.5, -1.0)).unwrap();
    let drilled = boolean(topo, BooleanOp::Cut, block, bore).unwrap();
    let stock = make_box(topo, 4.0, 4.0, 4.0).unwrap();
    let void = make_box(topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(topo, void, &Mat4::translation(1.0, 1.0, 1.0)).unwrap();
    let hollow = boolean(topo, BooleanOp::Cut, stock, void).unwrap();
    let sphere = make_sphere(topo, 1.5, 16).unwrap();
    transform_solid(topo, sphere, &Mat4::translation(7.0, -2.0, 3.0)).unwrap();
    vec![
        ("box", make_box(topo, 1.0, 2.0, 3.0).unwrap()),
        ("cylinder", make_cylinder(topo, 1.25, 3.0).unwrap()),
        ("cone", make_cone(topo, 2.0, 0.5, 3.0).unwrap()),
        ("sphere", sphere),
        ("torus", make_torus(topo, 3.0, 1.0, 16).unwrap()),
        ("drilled block", drilled),
        ("hollow cube", hollow),
    ]
}

fn probe_bits(topo: &Topology, solid: SolidId) -> (String, Vec<(usize, usize, usize, u64)>) {
    let (report, probes) =
        validate_solid_with_budget_probes(topo, solid, &ValidationOptions::default()).unwrap();
    (
        format!("{report:?}"),
        probes
            .shells
            .iter()
            .map(|p| (p.shell, p.order, p.faces, p.signed_volume.to_bits()))
            .collect(),
    )
}

fn area_bits(topo: &Topology, solid: SolidId) -> Vec<u64> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| face_area(topo, face, 0.05).unwrap().to_bits())
        .collect()
}

/// Strict reports, budget probes and face areas: uncached, cold cache, warm
/// cache and a cloned topology (other lineage, same content) all agree bit
/// for bit, and the warm and cloned passes are answered from the cache.
#[test]
fn face_cache_readings_are_bit_identical() {
    let _memos = Memos::off();
    let mut topo = Topology::new();
    let bodies = bodies(&mut topo);
    let uncached: Vec<_> = bodies
        .iter()
        .map(|&(_, s)| (probe_bits(&topo, s), area_bits(&topo, s)))
        .collect();
    assert_eq!(
        thread_face_cache_stats().misses,
        0,
        "disabled cache is never consulted"
    );

    enable_thread_face_cache();
    let clone = topo.clone();
    for pass in ["cold", "warm", "clone"] {
        let before = thread_face_cache_stats();
        let t = if pass == "clone" { &clone } else { &topo };
        for (&(label, solid), expected) in bodies.iter().zip(&uncached) {
            let got = (probe_bits(t, solid), area_bits(t, solid));
            assert_eq!(&got, expected, "{label}: {pass} pass");
        }
        let after = thread_face_cache_stats();
        if pass == "cold" {
            assert!(after.misses > before.misses);
        } else {
            assert_eq!(after.misses, before.misses, "{pass} pass must only hit");
            assert!(after.hits > before.hits);
        }
    }
}

/// The strict probe's signed volume with the face cache off.
fn uncached_probe(topo: &Topology, solid: SolidId) -> f64 {
    let stats = thread_face_cache_stats();
    set_thread_face_cache_limits(0, 0);
    let (_, probes) =
        validate_solid_with_budget_probes(topo, solid, &ValidationOptions::default()).unwrap();
    set_thread_face_cache_limits(stats.capacity, stats.byte_budget);
    probes.shells[0].signed_volume
}

/// A rigidly translated body is the same faces moved: its probe reads every
/// face from the source's entries (re-expressed by the vector area, so
/// within rounding of integrating afresh) and reaches the same verdict. A
/// rotated body is new content and integrates afresh.
#[test]
fn translated_bodies_read_their_source_integrals() {
    let _memos = Memos::off();
    enable_thread_face_cache();
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let faces = solid_faces(&topo, solid).unwrap().len() as u64;
    let (report, _) = probe_bits(&topo, solid);
    transform_solid(&mut topo, solid, &Mat4::translation(0.5, -3.25, 7.0)).unwrap();
    let before = thread_face_cache_stats();
    let (moved, probes) = probe_bits(&topo, solid);
    let after = thread_face_cache_stats();
    assert_eq!(moved, report);
    assert_eq!(probes.len(), 1);
    assert_eq!(after.misses, before.misses, "every face is a translation");
    assert_eq!(after.translated - before.translated, faces);
    let read = f64::from_bits(probes[0].3);
    let fresh = uncached_probe(&topo, solid);
    assert!(
        (read - fresh).abs() <= fresh.abs() * 1e-12,
        "translated probe {read} vs fresh {fresh}"
    );

    transform_solid(
        &mut topo,
        solid,
        &Mat4::rotation_x(std::f64::consts::FRAC_PI_3),
    )
    .unwrap();
    let before = thread_face_cache_stats();
    let (rotated, _) = probe_bits(&topo, solid);
    assert_eq!(rotated, report);
    assert!(thread_face_cache_stats().misses > before.misses);
}

fn uncached_volume(topo: &Topology, solid: SolidId) -> u64 {
    let capacity = thread_volume_memo_stats().capacity;
    set_thread_volume_memo_capacity(0);
    let volume = solid_volume(topo, solid, 0.05).unwrap().to_bits();
    set_thread_volume_memo_capacity(capacity);
    volume
}

/// What the memo may do with a reading after an edit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum After {
    /// The edit changed what `solid_volume` reads about the solid: retaken.
    Retaken,
    /// The edit left (or put back) the solid's content: served by content.
    Reused,
}

/// Read the volume twice (the second must hit), apply `edit`, and read it
/// again. The reading must be exactly the uncached reading of the edited
/// topology either way; an edit that changes the solid's content must make
/// the memo retake it, and one that does not may be answered from it.
fn assert_edit(label: &str, after: After, edit: impl FnOnce(&mut Topology, SolidId)) {
    let _memos = Memos::off();
    enable_thread_volume_memo();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let first = solid_volume(&topo, solid, 0.05).unwrap();
    let stats = thread_volume_memo_stats();
    let again = solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(again.to_bits(), first.to_bits(), "{label}");
    assert_eq!(
        thread_volume_memo_stats().hits,
        stats.hits + 1,
        "{label}: repeat hits"
    );

    edit(&mut topo, solid);
    let stats = thread_volume_memo_stats();
    let read = solid_volume(&topo, solid, 0.05).unwrap();
    let now = thread_volume_memo_stats();
    match after {
        After::Retaken => assert_eq!(
            now.misses,
            stats.misses + 1,
            "{label}: the edit must retire the reading"
        ),
        After::Reused => assert_eq!(
            (now.misses, now.content_hits),
            (stats.misses, stats.content_hits + 1),
            "{label}: unchanged content must be found again"
        ),
    }
    assert_eq!(read.to_bits(), uncached_volume(&topo, solid), "{label}");
}

/// Two requests that `solid_volume` clamps to the same deflection read one
/// mesh, so they share a reading; requests finer than the clamp do not.
#[test]
fn volume_memo_matches_requests_by_clamped_deflection() {
    let _memos = Memos::off();
    enable_thread_volume_memo();
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    // The clamp is the bounding-box diagonal (4√3) × 5e-5 ≈ 3.5e-4.
    let a = solid_volume(&topo, solid, 0.05).unwrap();
    let b = solid_volume(&topo, solid, 0.02).unwrap();
    assert_eq!(a.to_bits(), b.to_bits());
    let stats = thread_volume_memo_stats();
    assert_eq!((stats.misses, stats.hits), (1, 1));
    let fine = solid_volume(&topo, solid, 1e-4).unwrap();
    let finer = solid_volume(&topo, solid, 5e-5).unwrap();
    let again = solid_volume(&topo, solid, 1e-4).unwrap();
    assert_eq!(fine.to_bits(), again.to_bits());
    let _ = finer;
    let stats = thread_volume_memo_stats();
    assert_eq!((stats.misses, stats.hits), (3, 2));
    // Off: never consulted.
    set_thread_volume_memo_capacity(0);
    let _ = solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(thread_volume_memo_stats().misses, 3);
}

#[test]
fn volume_memo_invalidated_by_in_place_transform() {
    assert_edit("transform_solid", After::Retaken, |topo, solid| {
        transform_solid(topo, solid, &Mat4::scale(1.5, 1.0, 1.0)).unwrap();
    });
}

#[test]
fn volume_memo_invalidated_by_vertex_mut() {
    assert_edit("vertex_mut", After::Retaken, |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[0];
        let wire = topo.face(face).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[0].edge();
        let vertex = topo.edge(edge).unwrap().start();
        let p = topo.vertex(vertex).unwrap().point();
        topo.vertex_mut(vertex)
            .unwrap()
            .set_point(Point3::new(p.x() + 0.25, p.y(), p.z()));
    });
}

#[test]
fn volume_memo_invalidated_by_face_mut() {
    assert_edit("face_mut", After::Retaken, |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[0];
        let f = topo.face_mut(face).unwrap();
        f.set_reversed(!f.is_reversed());
    });
}

#[test]
fn volume_memo_invalidated_by_edge_mut() {
    assert_edit("edge_mut", After::Retaken, |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[0];
        let wire = topo.face(face).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[0].edge();
        topo.edge_mut(edge)
            .unwrap()
            .set_tolerance(Some(1e-6))
            .unwrap();
    });
}

/// The tessellator consults pcurves, so they are part of the content.
#[test]
fn volume_memo_invalidated_by_a_pcurve_write() {
    assert_edit("set_pcurve_oriented", After::Retaken, |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[0];
        let wire = topo.face(face).unwrap().outer_wire();
        let used = topo.wire(wire).unwrap().edges()[0];
        let line = remus_math::curves2d::Line2D::new(
            remus_math::vec::Point2::new(0.0, 0.0),
            remus_math::vec::Vec2::new(1.0, 0.0),
        )
        .unwrap();
        topo.set_pcurve_oriented(
            used.edge(),
            face,
            used.is_forward(),
            remus_topology::pcurve::PCurve::new(
                remus_math::curves2d::Curve2D::Line(line),
                0.0,
                1.0,
            ),
        )
        .unwrap();
    });
}

#[test]
fn volume_memo_reused_after_rollback() {
    assert_edit("RollbackSnapshot::restore", After::Reused, |topo, solid| {
        let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
        transform_solid(topo, solid, &Mat4::scale(2.0, 2.0, 2.0)).unwrap();
        snapshot.restore(topo);
    });
}

#[test]
fn volume_memo_reused_after_snapshot_restore() {
    assert_edit(
        "restore_preserving_handle_slots",
        After::Reused,
        |topo, solid| {
            let snapshot = topo.clone();
            transform_solid(topo, solid, &Mat4::scale(2.0, 1.0, 1.0)).unwrap();
            topo.restore_preserving_handle_slots(&snapshot);
        },
    );
}

#[test]
fn volume_memo_invalidated_by_in_place_heal() {
    assert_edit(
        "heal::fix_face_orientations",
        After::Retaken,
        |topo, solid| {
            let face = solid_faces(topo, solid).unwrap()[2];
            let f = topo.face_mut(face).unwrap();
            f.set_reversed(!f.is_reversed());
            remus_operations::heal::fix_face_orientations(topo, solid).unwrap();
        },
    );
}

/// An allocation elsewhere moves the topology identity but not the solid's
/// content, so the reading is found again by content (and is still exact).
#[test]
fn volume_memo_survives_an_unrelated_allocation() {
    assert_edit("unrelated make_box", After::Reused, |topo, _| {
        make_box(topo, 1.0, 1.0, 1.0).unwrap();
    });
}

#[test]
fn volume_memo_never_answers_for_a_deleted_solid() {
    let _memos = Memos::off();
    enable_thread_volume_memo();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    solid_volume(&topo, solid, 0.05).unwrap();
    topo.delete_solid(solid).unwrap();
    assert!(solid_volume(&topo, solid, 0.05).is_err());
}

/// A clone has its own lineage but the same content and handles: it is
/// answered from the source's reading, and an edit to either retires it for
/// that one only.
#[test]
fn volume_memo_reads_a_clone_by_content() {
    let _memos = Memos::off();
    enable_thread_volume_memo();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let a = solid_volume(&topo, solid, 0.05).unwrap();
    let mut clone = topo.clone();
    let stats = thread_volume_memo_stats();
    let b = solid_volume(&clone, solid, 0.05).unwrap();
    assert_eq!(a.to_bits(), b.to_bits());
    assert_eq!(
        thread_volume_memo_stats().content_hits,
        stats.content_hits + 1
    );
    transform_solid(&mut clone, solid, &Mat4::scale(1.0, 2.0, 1.0)).unwrap();
    let stats = thread_volume_memo_stats();
    let c = solid_volume(&clone, solid, 0.05).unwrap();
    assert_eq!(thread_volume_memo_stats().misses, stats.misses + 1);
    assert_eq!(c.to_bits(), uncached_volume(&clone, solid));
    let d = solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(d.to_bits(), a.to_bits());
}

/// A content key larger than the byte budget is not retained; the reading
/// stays findable by identity alone.
#[test]
fn volume_memo_keeps_oversized_readings_by_identity_only() {
    let _memos = Memos::off();
    set_thread_volume_memo_limits(8, 16);
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    solid_volume(&topo, solid, 0.05).unwrap();
    solid_volume(&topo, solid, 0.05).unwrap();
    let stats = thread_volume_memo_stats();
    assert_eq!((stats.hits, stats.retained_bytes), (1, 0));
    make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(thread_volume_memo_stats().misses, stats.misses + 1);
}
