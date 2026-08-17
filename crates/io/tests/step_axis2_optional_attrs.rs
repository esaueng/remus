//! Import of a file whose `AXIS2_PLACEMENT_3D`s omit OPTIONAL attributes.
//!
//! ISO 10303-42 declares `axis` and `ref_direction` OPTIONAL, so a placement
//! may write `$` in either slot. A real customer export did exactly that —
//! `AXIS2_PLACEMENT_3D('Circle Axis2P3D',#65,#66,$)`, 445 times — and the
//! reader, which located sub-entities by scanning for `#NNN` tokens, counted
//! two references where it demanded three and rejected the whole file.
//!
//! The nastier form is an omission *before* a reference: the scan then binds
//! the ref_direction as the axis and turns the frame with no error at all.
//! The fixture puts one of those (`#18`) on the cylinder's bottom circle,
//! where a mis-bind swings the circle into the xz plane.
//!
//! The writer always emits all three references and never `$`, so a
//! round trip can never exercise any of this; the fixture is the coverage,
//! and the round trip here only confirms the imported solid is the ordinary
//! one.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

use remus_operations::measure;
use remus_topology::Topology;
use remus_topology::explorer;

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

const FIXTURE: &str = "axis2_optional_attrs_cylinder.step";
const DEFLECTION: f64 = 0.01;

/// Radius 4, height 10.
const EXPECTED_VOLUME: f64 = std::f64::consts::PI * 16.0 * 10.0;

/// The fuzz corpus seed for the same code, relative to this crate.
const FUZZ_SEED: &str = "../../fuzz/corpus/step_reader/axis2-optional-attrs.step";

#[test]
fn omitted_optional_placements_import_as_one_solid() {
    let text = std::fs::read_to_string(fixture(FIXTURE)).unwrap();
    assert!(
        text.contains("AXIS2_PLACEMENT_3D('Circle Axis2P3D', #15, $, #17)"),
        "the fixture must keep the omission-before-a-reference form"
    );

    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(&text, &mut topo).unwrap();
    assert_eq!(solids.len(), 1, "expected exactly one solid");

    let faces = explorer::solid_faces(&topo, solids[0]).unwrap();
    assert_eq!(faces.len(), 3, "cylinder: two caps and a wall");

    // A mis-bound axis would tilt the bottom circle out of the xy plane and
    // this volume would not come out anywhere near right.
    let volume = measure::solid_volume(&topo, solids[0], DEFLECTION).unwrap();
    let rel_error = (volume - EXPECTED_VOLUME).abs() / EXPECTED_VOLUME;
    assert!(
        rel_error < 1e-3,
        "volume {volume} differs from {EXPECTED_VOLUME} (rel {rel_error})"
    );
}

/// The fuzz seed has to reach the code it seeds. A STEP file with no
/// `MANIFOLD_SOLID_BREP` in it never resolves a placement at all — `read_step`
/// returns `Ok(0 solids)` without calling `build_axis2_placement` once — so a
/// seed without a solid exercises nothing no matter how many placements it
/// contains. This pins the seed to a real solid, and to the frames its
/// placements are supposed to produce: it covers the forms the fixture does
/// not, a Part 21 complex instance and a statement only the reference-scan
/// fallback can read.
#[test]
fn the_fuzz_seed_reaches_the_placement_code() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(FUZZ_SEED);
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(
        text.contains("MANIFOLD_SOLID_BREP"),
        "the seed needs a solid or the reader stops before any placement"
    );

    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(&text, &mut topo).unwrap();
    assert_eq!(solids.len(), 1, "expected exactly one solid");
    assert_eq!(
        explorer::solid_faces(&topo, solids[0]).unwrap().len(),
        3,
        "cylinder: two caps and a wall"
    );

    let volume = measure::solid_volume(&topo, solids[0], DEFLECTION).unwrap();
    let rel_error = (volume - EXPECTED_VOLUME).abs() / EXPECTED_VOLUME;
    assert!(
        rel_error < 1e-3,
        "volume {volume} differs from {EXPECTED_VOLUME} (rel {rel_error})"
    );
}

/// The placements' frames survive as ordinary geometry: writing the imported
/// solid back out and reading it again reproduces the same solid.
#[test]
fn omitted_optional_placements_round_trip() {
    let text = std::fs::read_to_string(fixture(FIXTURE)).unwrap();

    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(&text, &mut topo).unwrap()[0];
    let volume = measure::solid_volume(&topo, solid, DEFLECTION).unwrap();
    let center = measure::solid_center_of_mass(&topo, solid, DEFLECTION).unwrap();

    let written = remus_io::step::write_step(&topo, &[solid]).unwrap();
    let mut round_topo = Topology::new();
    let round_solids = remus_io::step::reader::read_step(&written, &mut round_topo).unwrap();
    assert_eq!(round_solids.len(), 1);

    let round_faces = explorer::solid_faces(&round_topo, round_solids[0]).unwrap();
    assert_eq!(
        round_faces.len(),
        explorer::solid_faces(&topo, solid).unwrap().len(),
        "face count changed over the round trip"
    );

    let round_volume = measure::solid_volume(&round_topo, round_solids[0], DEFLECTION).unwrap();
    assert!(
        (round_volume - volume).abs() / volume < 1e-9,
        "volume changed over the round trip: {volume} then {round_volume}"
    );

    let round_center =
        measure::solid_center_of_mass(&round_topo, round_solids[0], DEFLECTION).unwrap();
    assert!(
        (round_center.x() - center.x()).abs() < 1e-9
            && (round_center.y() - center.y()).abs() < 1e-9
            && (round_center.z() - center.z()).abs() < 1e-9,
        "centre of mass moved over the round trip"
    );
}
