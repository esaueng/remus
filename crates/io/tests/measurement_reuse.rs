//! Exact measurement reuse on the public complex filleted editing fixture.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::{arena_io::serialize_solids, step::reader::read_step};
use remus_operations::{measure::*, push_pull::move_faces};
use remus_topology::{
    Topology,
    explorer::solid_faces,
    face::{FaceId, FaceSurface},
    solid::SolidId,
};

const FIXTURE: &str = include_str!("data/shapr3d_hammer_holder.step");

struct Guard;
impl Guard {
    fn new() -> Self {
        set_measurement_reuse_enabled(false);
        clear_measurement_reuse();
        set_measurement_reuse_limits(
            DEFAULT_MEASUREMENT_REUSE_ENTRIES,
            DEFAULT_MEASUREMENT_REUSE_BYTES,
        );
        set_thread_volume_memo_capacity(0);
        disable_thread_nurbs_area_memo();
        remus_check::properties::face_cache::set_thread_face_cache_limits(0, 0);
        remus_operations::tessellate::disable_face_mesh_cache();
        remus_operations::tessellate::disable_mesh_reuse();
        Self
    }
}
impl Drop for Guard {
    fn drop(&mut self) {
        set_measurement_reuse_enabled(false);
        set_thread_volume_memo_capacity(0);
    }
}
fn areas(topo: &Topology, solid: SolidId, deflection: f64) -> Vec<u64> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| face_area(topo, face, deflection).unwrap().to_bits())
        .collect()
}
fn edited_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid).unwrap().into_iter().find(|&face| {
        matches!(topo.face(face).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.x() > 0.99)
            && (face_area(topo, face, 0.05).unwrap() - 1045.93).abs() < 1.0
    }).expect("outer arm plane")
}

#[test]
fn filleted_edits_and_reversals_preserve_every_area_and_volume_bit() {
    let _guard = Guard::new();
    let mut topo = Topology::new();
    let source = read_step(FIXTURE, &mut topo).unwrap()[0];
    let source_bytes = serialize_solids(&topo, &[source]).unwrap();
    let selected = edited_face(&topo, source);
    let solids: Vec<_> = [0.5, 1.0, 5.0, -5.0, -1.0, 0.5]
        .into_iter()
        .map(|distance| move_faces(&mut topo, source, &[selected], distance).unwrap())
        .collect();
    let expected: Vec<_> = std::iter::once(source)
        .chain(solids.iter().copied())
        .map(|solid| {
            (
                areas(&topo, solid, 0.05),
                solid_volume(&topo, solid, 0.05).unwrap().to_bits(),
            )
        })
        .collect();
    enable_measurement_reuse();
    let before = measurement_reuse_stats();
    for (solid, (area, volume)) in std::iter::once(source)
        .chain(solids.iter().copied())
        .zip(expected)
    {
        assert_eq!(solid_faces(&topo, solid).unwrap().len(), 160);
        assert_eq!(
            areas(&topo, solid, 0.05),
            area,
            "every face area matches fresh bits"
        );
        assert_eq!(
            solid_volume(&topo, solid, 0.05).unwrap().to_bits(),
            volume,
            "volume matches fresh bits"
        );
    }
    let after = measurement_reuse_stats();
    assert!(
        after.area_content_hits - before.area_content_hits >= 600,
        "unchanged faces reused on new values: {after:?}"
    );
    assert!(
        after.volume_content_hits > before.volume_content_hits,
        "repeat edit measured once: {after:?}"
    );
    assert!(after.area_entries <= after.area_max_entries);
    assert!(after.area_retained_bytes <= after.area_max_bytes);
    let old = thread_volume_memo_stats();
    assert_eq!(old.capacity, DEFAULT_VOLUME_MEMO_CAPACITY);
    assert_eq!(old.byte_budget, DEFAULT_VOLUME_MEMO_BYTE_BUDGET);
    assert!(old.retained_bytes <= old.byte_budget);
    assert_eq!(
        serialize_solids(&topo, &[source]).unwrap(),
        source_bytes,
        "source geometry is immutable"
    );
}

#[test]
fn rebuilt_filleted_arena_and_tighter_quality_match_cold_measurements() {
    let _guard = Guard::new();
    let mut first = Topology::new();
    let source = read_step(FIXTURE, &mut first).unwrap()[0];
    let mut second = Topology::new();
    remus_operations::primitives::make_box(&mut second, 1.0, 2.0, 3.0).unwrap();
    let rebuilt = read_step(FIXTURE, &mut second).unwrap()[0];
    let requests = [0.05, 0.0148, 0.005];
    let expected: Vec<_> = requests
        .into_iter()
        .map(|quality| {
            (
                areas(&second, rebuilt, quality),
                solid_volume(&second, rebuilt, quality).unwrap().to_bits(),
            )
        })
        .collect();
    enable_measurement_reuse();
    areas(&first, source, requests[0]);
    solid_volume(&first, source, requests[0]).unwrap();
    let before = measurement_reuse_stats();
    for (quality, (area, volume)) in requests.into_iter().zip(expected) {
        assert_eq!(areas(&second, rebuilt, quality), area);
        assert_eq!(
            solid_volume(&second, rebuilt, quality).unwrap().to_bits(),
            volume
        );
    }
    let after = measurement_reuse_stats();
    assert_eq!(
        after.area_content_hits - before.area_content_hits,
        160,
        "only matching area quality may hit"
    );
    assert!(
        after.volume_content_hits > before.volume_content_hits,
        "rebuilt volume matched across handles"
    );
}
