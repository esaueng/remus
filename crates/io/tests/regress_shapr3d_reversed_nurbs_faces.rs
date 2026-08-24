//! Regression for ADVANCED_FACE.same_sense on B-spline surfaces.
//!
//! ISO 10303-42 stores each EDGE_LOOP in the face's topological sense
//! (surface normal composed with `same_sense`) for every surface type.
//! The reader used to exempt NURBS faces from that composition, so any
//! conforming external file with a reversed B-spline face — this
//! Shapr3D 26.143 / HOOPS Exchange AP242 export has eight — imported
//! with 24 misoriented shared edges and failed strict `validate_solid`,
//! tripping OpenZCAD's B-rep validity warning on every such import.
//!
//! The orientation assertion here mirrors `check_shell_orientation`
//! (crates/check/src/validate/shell.rs) inline, because remus-io cannot
//! depend on remus-check; the full strict `validate_solid` gate for a
//! reversed-NURBS import is covered on a small fixture by
//! `openzcad_step_validity.rs`.

#![allow(clippy::expect_used)]

use std::collections::HashMap;

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("data/shapr3d_hammer_holder.step");
const EXPECTED_VOLUME: f64 = 50_240.482_8;

fn reversed_nurbs_face_count(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .expect("enumerate faces")
        .iter()
        .filter(|&&face_id| {
            let face = topo.face(face_id).expect("face");
            face.is_reversed() && matches!(face.surface(), FaceSurface::Nurbs(_))
        })
        .count()
}

/// Count shared edges whose two effective uses do not oppose — the exact
/// property strict `validate_solid` reports as "shared edges have
/// inconsistent face orientations".
fn misoriented_shared_edge_count(topo: &Topology, solid: SolidId) -> usize {
    let mut edge_uses: HashMap<EdgeId, Vec<bool>> = HashMap::new();
    for face_id in solid_faces(topo, solid).expect("enumerate faces") {
        let face = topo.face(face_id).expect("face");
        let reversed = face.is_reversed();
        let mut wires = vec![face.outer_wire()];
        wires.extend(face.inner_wires().iter().copied());
        for wire_id in wires {
            let wire = topo.wire(wire_id).expect("wire");
            for oe in wire.edges() {
                edge_uses
                    .entry(oe.edge())
                    .or_default()
                    .push(oe.is_forward() != reversed);
            }
        }
    }
    edge_uses
        .values()
        .filter(|uses| uses.len() == 2 && uses[0] == uses[1])
        .count()
}

fn assert_consistent(topo: &Topology, solid: SolidId, label: &str) {
    assert_eq!(
        reversed_nurbs_face_count(topo, solid),
        8,
        "{label}: reversed-NURBS face census changed"
    );
    assert_eq!(
        misoriented_shared_edge_count(topo, solid),
        0,
        "{label}: shared edges must have opposing face orientations \
         (was: 24 misoriented edges, one per reversed-NURBS face boundary)"
    );
    let volume = solid_volume(topo, solid, 0.5).expect("measure solid");
    assert!(
        (volume - EXPECTED_VOLUME).abs() <= EXPECTED_VOLUME * 0.001,
        "{label}: volume {volume} differs from measured reference {EXPECTED_VOLUME}"
    );
    let mesh = tessellate_solid(topo, solid, 0.1).expect("tessellate solid");
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "{label}: mesh must be watertight"
    );
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{label}: mesh must be manifold"
    );
}

#[test]
fn shapr3d_reversed_nurbs_faces_import_and_round_trip_consistently() {
    let mut topo = Topology::new();
    let solids = read_step(HAMMER_HOLDER, &mut topo).expect("import Shapr3D STEP");
    assert_eq!(solids.len(), 1, "fixture must contain one solid");
    let solid = solids[0];
    assert_consistent(&topo, solid, "import");

    // Round trip: the writer must emit conforming (face-sense) loops for
    // the reversed NURBS faces so the file re-reads consistently.
    let step = write_step(&topo, &[solid]).expect("write imported solid");
    let mut topo2 = Topology::new();
    let reread = read_step(&step, &mut topo2).expect("re-read written STEP");
    assert_eq!(reread.len(), 1, "round trip must keep one solid");
    assert_consistent(&topo2, reread[0], "round trip");
}
