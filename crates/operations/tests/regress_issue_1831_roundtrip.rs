//! Remus #1831: round-trip the curved cut through the arena and STEP.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_io::arena_io::{deserialize_solid, serialize_solid};
use remus_io::step::{reader::read_step, writer::write_step};
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const TARGET: &str = include_str!("data/issue_1242_swept_tube_target.step");
const TOOL: &str = include_str!("data/issue_1242_annular_tool.step");

fn read_one(topo: &mut Topology, step: &str) -> SolidId {
    let solids = read_step(step, topo).expect("read one STEP solid");
    assert_eq!(solids.len(), 1);
    solids[0]
}

#[test]
fn curved_cut_serializes_and_round_trips_through_step() {
    let mut operands = Topology::new();
    let target = read_one(&mut operands, TARGET);
    let tool = read_one(&mut operands, TOOL);
    let cut = boolean(&mut operands, BooleanOp::Cut, target, tool).expect("curved cut");

    let arena = serialize_solid(&operands, cut).expect("serialize curved cut");
    let mut decoded = Topology::new();
    let cut = deserialize_solid(&arena, &mut decoded).expect("decode curved cut");

    let step = write_step(&decoded, &[cut]).expect("export STEP");
    let mut imported = Topology::new();
    let cut = read_one(&mut imported, &step);
    let volume = solid_volume(&imported, cut, 0.005).expect("read cut volume");
    assert!((volume - 175.430_430).abs() < 0.005, "cut volume: {volume}");

    let removed = Point3::new(6.175_785_17, 1.031_013_2, 4.870_290_36);
    let kept = Point3::new(2.0, 1.0, 4.14);
    assert_eq!(
        classify_point(&imported, cut, removed, 0.01, 1e-7).unwrap(),
        PointClassification::Outside
    );
    assert_eq!(
        classify_point(&imported, cut, kept, 0.01, 1e-7).unwrap(),
        PointClassification::Inside
    );
}
