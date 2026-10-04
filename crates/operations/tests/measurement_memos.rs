//! The opt-in measurement memos — the content-keyed face-integral cache
//! behind strict validation's orientation probe and `face_area`, and the
//! identity-keyed `solid_volume` memo — must be invisible: every reading
//! through them is bit-identical to the uncached computation, and no
//! in-place mutation can make a memoized reading stale.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::properties::face_cache::{
    enable_thread_face_cache, set_thread_face_cache_limits, thread_face_cache_stats,
};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{
    enable_thread_volume_memo, face_area, set_thread_volume_memo_capacity, solid_volume,
    thread_volume_memo_stats,
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

/// A rigidly moved copy is new content: it misses, and still reads the same
/// verdict.
#[test]
fn moved_faces_are_integrated_afresh() {
    let _memos = Memos::off();
    enable_thread_face_cache();
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let (report, _) = probe_bits(&topo, solid);
    transform_solid(&mut topo, solid, &Mat4::translation(0.5, 0.0, 0.0)).unwrap();
    let before = thread_face_cache_stats();
    let (moved, probes) = probe_bits(&topo, solid);
    assert_eq!(moved, report);
    assert_eq!(probes.len(), 1);
    assert!(thread_face_cache_stats().misses > before.misses);
}

fn uncached_volume(topo: &Topology, solid: SolidId) -> u64 {
    let capacity = thread_volume_memo_stats().capacity;
    set_thread_volume_memo_capacity(0);
    let volume = solid_volume(topo, solid, 0.05).unwrap().to_bits();
    set_thread_volume_memo_capacity(capacity);
    volume
}

/// Read the volume twice (the second must hit), apply `edit`, and read it
/// again: the memo must miss and return exactly the uncached reading of the
/// edited topology.
fn assert_edit_invalidates(label: &str, edit: impl FnOnce(&mut Topology, SolidId)) {
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
    let after = solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(
        thread_volume_memo_stats().misses,
        stats.misses + 1,
        "{label}: the edit must retire the reading"
    );
    assert_eq!(after.to_bits(), uncached_volume(&topo, solid), "{label}");
}

#[test]
fn volume_memo_hits_only_at_the_same_deflection() {
    let _memos = Memos::off();
    enable_thread_volume_memo();
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 2.0, 16).unwrap();
    let a = solid_volume(&topo, solid, 0.05).unwrap();
    let b = solid_volume(&topo, solid, 0.02).unwrap();
    let c = solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(a.to_bits(), c.to_bits());
    let _ = b;
    let stats = thread_volume_memo_stats();
    assert_eq!((stats.misses, stats.hits), (2, 1));
    // Off: never consulted.
    set_thread_volume_memo_capacity(0);
    let _ = solid_volume(&topo, solid, 0.05).unwrap();
    assert_eq!(thread_volume_memo_stats().misses, 2);
}

#[test]
fn volume_memo_invalidated_by_in_place_transform() {
    assert_edit_invalidates("transform_solid", |topo, solid| {
        transform_solid(topo, solid, &Mat4::scale(1.5, 1.0, 1.0)).unwrap();
    });
}

#[test]
fn volume_memo_invalidated_by_vertex_mut() {
    assert_edit_invalidates("vertex_mut", |topo, solid| {
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
    assert_edit_invalidates("face_mut", |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[0];
        let f = topo.face_mut(face).unwrap();
        f.set_reversed(!f.is_reversed());
    });
}

#[test]
fn volume_memo_invalidated_by_edge_mut() {
    assert_edit_invalidates("edge_mut", |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[0];
        let wire = topo.face(face).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[0].edge();
        topo.edge_mut(edge)
            .unwrap()
            .set_tolerance(Some(1e-6))
            .unwrap();
    });
}

#[test]
fn volume_memo_invalidated_by_rollback() {
    assert_edit_invalidates("RollbackSnapshot::restore", |topo, solid| {
        let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
        transform_solid(topo, solid, &Mat4::scale(2.0, 2.0, 2.0)).unwrap();
        snapshot.restore(topo);
    });
}

#[test]
fn volume_memo_invalidated_by_snapshot_restore() {
    assert_edit_invalidates("restore_preserving_handle_slots", |topo, solid| {
        let snapshot = topo.clone();
        transform_solid(topo, solid, &Mat4::scale(2.0, 1.0, 1.0)).unwrap();
        topo.restore_preserving_handle_slots(&snapshot);
    });
}

#[test]
fn volume_memo_invalidated_by_in_place_heal() {
    assert_edit_invalidates("heal::fix_face_orientations", |topo, solid| {
        let face = solid_faces(topo, solid).unwrap()[2];
        let f = topo.face_mut(face).unwrap();
        f.set_reversed(!f.is_reversed());
        remus_operations::heal::fix_face_orientations(topo, solid).unwrap();
    });
}

#[test]
fn volume_memo_misses_after_an_unrelated_allocation() {
    // Conservative: the identity cannot tell an allocation elsewhere from an
    // edit, so the reading is retaken (and still exact).
    assert_edit_invalidates("unrelated make_box", |topo, _| {
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

/// Clones have their own lineage: a reading of the source is not reused for
/// the clone even though the content is equal.
#[test]
fn volume_memo_is_per_topology_value() {
    let _memos = Memos::off();
    enable_thread_volume_memo();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let a = solid_volume(&topo, solid, 0.05).unwrap();
    let clone = topo.clone();
    let stats = thread_volume_memo_stats();
    let b = solid_volume(&clone, solid, 0.05).unwrap();
    assert_eq!(a.to_bits(), b.to_bits());
    assert_eq!(thread_volume_memo_stats().misses, stats.misses + 1);
}
