//! Multi-solid STEP export keeps bodies distinct through a round trip.
//!
//! `write_step` has always accepted a slice of solids, but nothing pinned that
//! the emitted file actually reads back as *separate* bodies rather than one
//! fused blob — and nothing pinned that the representation's item list is
//! well-formed. ISO-10303-21 lists have no trailing comma; the writer used to
//! emit `(#10, #20,)`, which strict readers reject.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::step::reader::read_step;
use remus_io::step::write_step;
use remus_math::mat::Mat4;
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_topology::Topology;

#[test]
fn two_solids_survive_a_step_round_trip_as_two_solids() {
    let mut topo = Topology::new();

    // 2x3x4 at the origin, plus a copy pushed clear along +x so the two bodies
    // do not touch. Distinct volumes make a swap or a fuse obvious.
    let first = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let second = make_box(&mut topo, 5.0, 5.0, 5.0).unwrap();
    let second =
        copy_and_transform_solid(&mut topo, second, &Mat4::translation(20.0, 0.0, 0.0)).unwrap();

    let step = write_step(&topo, &[first, second]).unwrap();

    let mut reread = Topology::new();
    let solids = read_step(&step, &mut reread).unwrap();
    assert_eq!(solids.len(), 2, "both bodies must come back separately");

    let mut volumes: Vec<f64> = solids
        .iter()
        .map(|&id| solid_volume(&reread, id, 0.01).unwrap())
        .collect();
    volumes.sort_by(f64::total_cmp);
    assert!(
        (volumes[0] - 24.0).abs() < 1e-6,
        "first body volume {} != 24",
        volumes[0]
    );
    assert!(
        (volumes[1] - 125.0).abs() < 1e-6,
        "second body volume {} != 125",
        volumes[1]
    );
}

#[test]
fn representation_item_list_has_no_trailing_comma() {
    let mut topo = Topology::new();
    let one = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let two = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

    for solids in [&[one][..], &[one, two][..]] {
        let step = write_step(&topo, solids).unwrap();
        let line = step
            .lines()
            .find(|line| line.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"))
            .expect("export must carry a shape representation");
        assert!(
            !line.contains(",)"),
            "STEP lists must not end with a comma: {line}"
        );
        assert_eq!(
            line.matches('#').count(),
            solids.len() + 2,
            "the entity's own id, one reference per solid, \
             and the representation context: {line}"
        );
    }
}

#[test]
fn exporting_no_solids_is_a_typed_error() {
    let topo = Topology::new();
    // A silent empty file would be the worst outcome: the caller would ship a
    // valid-looking STEP with nothing in it.
    assert!(write_step(&topo, &[]).is_err());
}
