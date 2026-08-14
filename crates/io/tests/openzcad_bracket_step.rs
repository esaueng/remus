//! STEP round-trip regression for OpenZCAD's walkthrough bracket.

#![allow(clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use brepkit_io::step::reader::read_step;
use brepkit_io::step::writer::write_step;
use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_operations::transform::transform_solid;
use brepkit_operations::validate::validate_solid;
use brepkit_topology::Topology;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::face::FaceSurface;
use brepkit_topology::solid::SolidId;
use brepkit_topology::validation::validate_shell_closed;

fn assert_one_closed_valid_solid(topo: &Topology, solid: SolidId) {
    let shell = topo
        .shell(topo.solid(solid).expect("solid").outer_shell())
        .expect("outer shell");
    validate_shell_closed(shell, topo).expect("closed shell");

    let mut uses = HashMap::new();
    for face_id in solid_faces(topo, solid).expect("solid faces") {
        let face = topo.face(face_id).expect("face");
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for edge in topo.wire(wire_id).expect("wire").edges() {
                *uses.entry(edge.edge().index()).or_insert(0usize) += 1;
            }
        }
    }
    assert!(
        uses.values().all(|&count| count == 2),
        "all edges must have exactly two uses: {uses:?}"
    );

    let report = validate_solid(topo, solid).expect("validate solid");
    assert!(report.is_valid(), "invalid solid: {:?}", report.issues);
}

fn build_walkthrough_bracket() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 60.0, 8.0, 30.0).expect("plate");

    let boss = make_cylinder(&mut topo, 12.0, 24.0).expect("boss");
    transform_solid(&mut topo, boss, &Mat4::translation(0.0, 8.0, 0.0)).expect("place boss");
    let bossed = boolean(&mut topo, BooleanOp::Fuse, plate, boss).expect("fuse boss");

    let bore = make_cylinder(&mut topo, 6.0, 48.0).expect("bore");
    let bracket = boolean(&mut topo, BooleanOp::Cut, bossed, bore).expect("drill bracket");
    (topo, bracket)
}

#[test]
fn openzcad_walkthrough_bracket_round_trips_as_one_valid_solid() {
    let (topo, bracket) = build_walkthrough_bracket();
    assert_one_closed_valid_solid(&topo, bracket);
    let source_volume = solid_volume(&topo, bracket, 0.05).expect("source volume");

    let source_cylinders = solid_faces(&topo, bracket)
        .expect("source faces")
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).expect("face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .count();
    assert!(
        source_cylinders >= 1,
        "boss or bore must remain an analytic cylinder"
    );

    let step = write_step(&topo, &[bracket]).expect("write STEP");
    assert!(step.contains("MANIFOLD_SOLID_BREP"));
    assert!(step.contains("CYLINDRICAL_SURFACE"));

    let mut imported = Topology::new();
    let solids = read_step(&step, &mut imported).expect("re-import STEP");
    assert_eq!(solids.len(), 1, "STEP must contain exactly one solid");
    let round = solids[0];
    assert_one_closed_valid_solid(&imported, round);
    let round_volume = solid_volume(&imported, round, 0.05).expect("round-trip volume");
    assert!(
        (round_volume - source_volume).abs() <= source_volume.abs().max(1.0) * 1e-9,
        "round-trip volume {round_volume:.12} != source {source_volume:.12}"
    );

    let round_cylinders = solid_faces(&imported, round)
        .expect("round faces")
        .into_iter()
        .filter(|&face| {
            matches!(
                imported.face(face).expect("face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .count();
    assert_eq!(round_cylinders, source_cylinders);
}
