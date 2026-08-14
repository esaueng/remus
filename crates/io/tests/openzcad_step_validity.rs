//! OpenZCAD parity-corpus STEP solids must import as valid analytic B-Reps.

#![allow(clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use brepkit_io::step::reader::read_step;
use brepkit_io::step::writer::write_step;
use brepkit_operations::measure::solid_volume;
use brepkit_operations::validate::validate_solid;
use brepkit_topology::Topology;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::solid::SolidId;
use brepkit_topology::validation::validate_shell_closed;

const BORED_PLATE: &str = include_str!("data/openzcad_a_export_bored_plate.step");
const FILLETED_PLATE: &str = include_str!("data/openzcad_e_analytic_fillet_plate.step");

fn import_one(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).expect("import STEP fixture");
    assert_eq!(solids.len(), 1, "fixture must contain exactly one solid");
    (topo, solids[0])
}

fn surface_census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in solid_faces(topo, solid).expect("solid faces") {
        *census
            .entry(topo.face(face).expect("face").surface().type_tag())
            .or_default() += 1;
    }
    census
}

fn assert_valid_round_trip(
    step: &str,
    expected_volume: f64,
    expected_census: &[(&'static str, usize)],
) {
    let (topo, solid) = import_one(step);
    let shell = topo
        .shell(topo.solid(solid).expect("solid").outer_shell())
        .expect("outer shell");
    validate_shell_closed(shell, &topo).expect("closed imported shell");

    let report = validate_solid(&topo, solid).expect("validate imported solid");
    assert!(
        report.is_valid(),
        "valid STEP fixture was rejected: {:?}",
        report.issues
    );
    let volume = solid_volume(&topo, solid, 0.05).expect("imported volume");
    assert!(
        (volume - expected_volume).abs() <= expected_volume * 1e-10,
        "imported volume {volume:.12} != {expected_volume:.12}"
    );
    let expected: BTreeMap<_, _> = expected_census.iter().copied().collect();
    assert_eq!(surface_census(&topo, solid), expected);

    let exported = write_step(&topo, &[solid]).expect("write round-trip STEP");
    assert!(exported.contains("MANIFOLD_SOLID_BREP"));
    let (round_topo, round_solid) = import_one(&exported);
    let round_report = validate_solid(&round_topo, round_solid).expect("validate round-trip solid");
    assert!(
        round_report.is_valid(),
        "round-trip STEP was rejected: {:?}",
        round_report.issues
    );
    let round_volume = solid_volume(&round_topo, round_solid, 0.05).expect("round-trip volume");
    assert!(
        (round_volume - volume).abs() <= volume.abs().max(1.0) * 1e-10,
        "round-trip volume {round_volume:.12} != source {volume:.12}"
    );
    assert_eq!(surface_census(&round_topo, round_solid), expected);
}

#[test]
fn openzcad_bored_plate_imports_as_one_valid_analytic_solid() {
    assert_valid_round_trip(
        BORED_PLATE,
        8_814.601_836_602_553,
        &[("cylinder", 1), ("plane", 6)],
    );
}

#[test]
fn openzcad_analytic_fillet_plate_imports_as_one_valid_analytic_solid() {
    assert_valid_round_trip(
        FILLETED_PLATE,
        9_522.606_928_409_188,
        &[("cylinder", 4), ("plane", 6)],
    );
}
