//! STEP name round trip for the attribute store (Issue 14; deferred-e3b).

#![allow(clippy::unwrap_used, clippy::expect_used)]

use brepkit_io::step::{reader::read_step, writer::write_step};
use brepkit_operations::primitives::make_box;
use brepkit_topology::Topology;
use brepkit_topology::attributes::EntityAttributes;
use brepkit_topology::explorer::solid_faces;

fn with_name(name: &str) -> EntityAttributes {
    EntityAttributes {
        name: Some(name.to_string()),
        ..Default::default()
    }
}

#[test]
fn solid_and_face_names_survive_a_round_trip() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let face = solid_faces(&topo, solid).unwrap()[0];
    topo.attributes_mut()
        .set_solid(solid, with_name("bracket body"));
    // STEP escapes apostrophes by doubling; exercise it.
    topo.attributes_mut()
        .set_face(face, with_name("mounting 'datum' face"));

    let step = write_step(&topo, &[solid]).unwrap();
    assert!(step.contains("bracket body"));

    let mut restored = Topology::new();
    let solids = read_step(&step, &mut restored).unwrap();
    assert_eq!(solids.len(), 1);

    assert_eq!(
        restored
            .attributes()
            .solid(solids[0])
            .and_then(|a| a.name.as_deref()),
        Some("bracket body")
    );
    let face_names: Vec<_> = solid_faces(&restored, solids[0])
        .unwrap()
        .into_iter()
        .filter_map(|f| restored.attributes().face(f))
        .filter_map(|a| a.name.as_deref())
        .collect();
    assert_eq!(face_names, vec!["mounting 'datum' face"]);

    // Second trip is stable.
    let step2 = write_step(&restored, &solids).unwrap();
    let mut again = Topology::new();
    let solids2 = read_step(&step2, &mut again).unwrap();
    assert_eq!(
        again
            .attributes()
            .solid(solids2[0])
            .and_then(|a| a.name.as_deref()),
        Some("bracket body")
    );
}

#[test]
fn unnamed_output_is_byte_stable_with_the_previous_format() {
    // No attributes set: the writer must keep emitting the historical
    // empty-name form.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let step = write_step(&topo, &[solid]).unwrap();
    assert!(step.contains("MANIFOLD_SOLID_BREP('', "));
    assert!(step.contains("ADVANCED_FACE('', "));
}
