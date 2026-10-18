//! Fresh-result and mutation oracles for reusable scalar measurements.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use crate::measure::{self, solid_volume};
use crate::primitives::{make_box, make_cylinder};
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::{Point2, Point3, Vec2};
use remus_topology::explorer::solid_faces;
use remus_topology::pcurve::PCurve;

struct Guard;
impl Guard {
    fn new() -> Self {
        STATE.with(|slot| *slot.borrow_mut() = State::default());
        measure::set_thread_volume_memo_capacity(0);
        measure::disable_thread_nurbs_area_memo();
        Self
    }
}
impl Drop for Guard {
    fn drop(&mut self) {
        set_measurement_reuse_enabled(false);
        measure::set_thread_volume_memo_capacity(0);
    }
}

#[test]
fn rebuilt_arenas_reuse_area_and_volume_without_changing_bits() {
    let _guard = Guard::new();
    let mut source = Topology::new();
    let a = make_cylinder(&mut source, 2.0, 7.0).unwrap();
    let mut rebuilt = Topology::new();
    make_box(&mut rebuilt, 1.0, 2.0, 3.0).unwrap();
    let b = make_cylinder(&mut rebuilt, 2.0, 7.0).unwrap();
    let af = solid_faces(&source, a).unwrap();
    let bf = solid_faces(&rebuilt, b).unwrap();
    let fresh: Vec<_> = bf
        .iter()
        .map(|&f| measure::face_area(&rebuilt, f, 0.05).unwrap().to_bits())
        .collect();
    let volume = solid_volume(&rebuilt, b, 0.05).unwrap().to_bits();
    enable_measurement_reuse();
    for &face in &af {
        measure::face_area(&source, face, 0.05).unwrap();
    }
    solid_volume(&source, a, 0.05).unwrap();
    let before = measurement_reuse_stats();
    for (&face, bits) in bf.iter().zip(fresh) {
        assert_eq!(
            measure::face_area(&rebuilt, face, 0.05).unwrap().to_bits(),
            bits
        );
    }
    assert_eq!(solid_volume(&rebuilt, b, 0.05).unwrap().to_bits(), volume);
    let after = measurement_reuse_stats();
    assert_eq!(
        after.area_content_hits - before.area_content_hits,
        bf.len() as u64
    );
    assert_eq!(after.volume_content_hits - before.volume_content_hits, 1);
    assert_eq!(solid_volume(&rebuilt, b, 0.05).unwrap().to_bits(), volume);
    assert_eq!(
        measurement_reuse_stats().volume_identity_hits,
        after.volume_identity_hits + 1
    );
}

#[test]
fn vertex_edits_restore_and_quality_requests_match_fresh_area_bits() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    let wire = topo.face(face).unwrap().outer_wire();
    let edge = topo.wire(wire).unwrap().edges()[0].edge();
    let vertex = topo.edge(edge).unwrap().start();
    let snapshot = topo.clone();
    enable_measurement_reuse();
    let original = measure::face_area(&topo, face, 0.05).unwrap();
    let p = topo.vertex(vertex).unwrap().point();
    topo.vertex_mut(vertex)
        .unwrap()
        .set_point(Point3::new(p.x() + 0.25, p.y(), p.z()));
    let changed = measure::face_area(&topo, face, 0.05).unwrap();
    assert_ne!(original.to_bits(), changed.to_bits());
    set_measurement_reuse_enabled(false);
    assert_eq!(
        changed.to_bits(),
        measure::face_area(&topo, face, 0.05).unwrap().to_bits()
    );
    set_measurement_reuse_enabled(true);
    measure::face_area(&snapshot, face, 0.05).unwrap();
    topo.restore_preserving_handle_slots(&snapshot);
    let before = measurement_reuse_stats();
    assert_eq!(
        measure::face_area(&topo, face, 0.05).unwrap().to_bits(),
        original.to_bits()
    );
    assert_eq!(
        measurement_reuse_stats().area_content_hits,
        before.area_content_hits + 1
    );
    let before = measurement_reuse_stats();
    measure::face_area(&topo, face, 0.01).unwrap();
    assert_eq!(
        measurement_reuse_stats().area_misses,
        before.area_misses + 1
    );
}

#[test]
fn tolerances_trims_pcurves_and_orientations_force_fresh_keys() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    let wire = topo.face(face).unwrap().outer_wire();
    let oriented = topo.wire(wire).unwrap().edges()[0];
    let edge = oriented.edge();
    let key =
        |t: &Topology| face_content_key(t, face, 0.05, DEFAULT_MEASUREMENT_REUSE_BYTES).unwrap();
    let original = key(&topo);
    topo.edge_mut(edge)
        .unwrap()
        .set_tolerance(Some(0.0))
        .unwrap();
    assert_ne!(original, key(&topo), "tolerance presence");
    let presence = key(&topo);
    topo.edge_mut(edge)
        .unwrap()
        .set_tolerance(Some(1e-5))
        .unwrap();
    assert_ne!(presence, key(&topo), "tolerance value");
    topo.edge_mut(edge).unwrap().set_tolerance(None).unwrap();
    assert_eq!(original, key(&topo));
    topo.edge_mut(edge).unwrap().set_trim(Some((0.0, 0.5)));
    assert_ne!(original, key(&topo), "edge trim");
    topo.edge_mut(edge).unwrap().set_trim(None);
    let pc = PCurve::new(
        Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
        0.0,
        1.0,
    );
    topo.set_pcurve_oriented(edge, face, oriented.is_forward(), pc)
        .unwrap();
    let traced = key(&topo);
    assert_ne!(original, traced, "directed pcurve");
    topo.remove_pcurve_oriented(edge, face, oriented.is_forward())
        .unwrap();
    assert_eq!(original, key(&topo));
    topo.face_mut(face).unwrap().set_reversed(true);
    assert_ne!(original, key(&topo), "face orientation");
}

#[test]
fn bounded_storage_preserves_hot_entries_and_explicit_preferences() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let faces = solid_faces(&topo, solid).unwrap();
    enable_measurement_reuse();
    set_measurement_reuse_limits(2, DEFAULT_MEASUREMENT_REUSE_BYTES);
    for &face in &faces {
        measure::face_area(&topo, face, 0.05).unwrap();
    }
    let stats = measurement_reuse_stats();
    assert_eq!(stats.area_entries, 2);
    assert!(stats.area_retained_bytes <= stats.area_max_bytes);
    assert!(stats.area_evictions >= 4);
    set_measurement_reuse_limits(2, 1);
    assert_eq!(measurement_reuse_stats().area_entries, 0);
    measure::face_area(&topo, faces[0], 0.05).unwrap();
    assert_eq!(measurement_reuse_stats().area_entries, 0);
    set_measurement_reuse_limits(0, DEFAULT_MEASUREMENT_REUSE_BYTES);
    measure::face_area(&topo, faces[0], 0.05).unwrap();
    assert_eq!(measurement_reuse_stats().area_entries, 0);
    set_measurement_reuse_enabled(false);
    enable_default_measurement_reuse();
    assert!(
        !measurement_reuse_stats().enabled,
        "new kernels preserve ablation"
    );
    assert_eq!(measurement_reuse_stats().area_max_entries, 0);
    let old = measure::thread_volume_memo_stats();
    assert_eq!(old.capacity, measure::DEFAULT_VOLUME_MEMO_CAPACITY);
    assert_eq!(old.byte_budget, measure::DEFAULT_VOLUME_MEMO_BYTE_BUDGET);
}

#[test]
fn content_hash_collision_does_not_answer_a_different_face() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let faces = solid_faces(&topo, solid).unwrap();
    enable_measurement_reuse();
    measure::face_area(&topo, faces[0], 0.05).unwrap();
    let other = faces
        .into_iter()
        .find(|&face| {
            measure::face_area(&topo, face, 0.05).unwrap()
                != measure::face_area(&topo, solid_faces(&topo, solid).unwrap()[0], 0.05).unwrap()
        })
        .unwrap();
    let expected = measure::face_area(&topo, other, 0.05).unwrap();
    clear_measurement_reuse();
    measure::face_area(&topo, solid_faces(&topo, solid).unwrap()[0], 0.05).unwrap();
    let target = face_content_key(&topo, other, 0.05, DEFAULT_MEASUREMENT_REUSE_BYTES).unwrap();
    STATE.with(|slot| {
        let mut state = slot.borrow_mut();
        let entry = &mut state.entries[0];
        entry.identity = None;
        entry.key.hash = target.hash;
    });
    let before = measurement_reuse_stats();
    assert_eq!(
        measure::face_area(&topo, other, 0.05).unwrap().to_bits(),
        expected.to_bits()
    );
    assert_eq!(
        measurement_reuse_stats().area_misses,
        before.area_misses + 1
    );
}

#[test]
fn failed_measurements_are_retried_and_never_retained() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    enable_measurement_reuse();
    for _ in 0..2 {
        let result = super::face_area(&topo, face, 0.05, || {
            Err(crate::OperationsError::InvalidInput {
                reason: "measurement refused".into(),
            })
        });
        assert!(result.is_err());
    }
    assert_eq!(measurement_reuse_stats().area_misses, 2);
    assert_eq!(measurement_reuse_stats().area_entries, 0);
}
