//! Qualification tests for the exact journaled imprint operation.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeMap;

use remus_math::mat::Mat4;
use remus_operations::imprint::imprint;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::journal::{
    EntityEvent, EntityKey, EntityKind, EntryPayload, EventDraft, EvolutionDraft,
};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

fn fixture() -> (Topology, remus_topology::SolidId, remus_topology::SolidId) {
    let mut topo = Topology::new();
    let target = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 6.0, 6.0, 6.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(2.0, -3.0, 2.0)).unwrap();
    (topo, target, tool)
}

fn split_target_face(topo: &Topology, target: remus_topology::SolidId) -> remus_topology::FaceId {
    solid_faces(topo, target)
        .unwrap()
        .into_iter()
        .find(|&face_id| {
            let face = topo.face(face_id).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            wire.edges().iter().all(|oriented| {
                let edge = topo.edge(oriented.edge()).unwrap();
                let point = topo.vertex(edge.start()).unwrap().point();
                point.y().abs() < 1.0e-12
            })
        })
        .expect("the target box must have a y=0 face")
}

fn anchor_face(topo: &mut Topology, face: remus_topology::FaceId) -> PersistentRef {
    let pending = topo.journal_begin("fixture_anchor");
    let key = EntityKey::face(face.index());
    let mut draft = EvolutionDraft::construction();
    draft.push(key, EventDraft::Preserved { from: key });
    let op = topo.journal_record_evolution(pending, draft).unwrap();
    PersistentRef::operation_output(op, EntityKind::Face, 0)
}

#[test]
fn planar_box_imprint_preserves_volume_and_resolves_split_face_bound_many() {
    let (mut topo, target, tool) = fixture();
    let target_face = split_target_face(&topo, target);
    let reference = anchor_face(&mut topo, target_face);
    let tool_face = solid_faces(&topo, tool).unwrap()[0];
    let tool_reference = anchor_face(&mut topo, tool_face);
    assert!(matches!(
        resolve(&topo, &reference),
        Resolution::Bound {
            provenance: Provenance::Construction,
            ..
        }
    ));

    let before_volume = solid_volume(&topo, target, 0.01).unwrap();
    let imprinted = imprint(&mut topo, target, tool).unwrap();
    let after_volume = solid_volume(&topo, imprinted.solid, 0.01).unwrap();
    assert!((after_volume - before_volume).abs() < 1.0e-9);

    let output_faces = solid_faces(&topo, imprinted.solid).unwrap();
    assert!(output_faces.len() > 6, "a target face must be partitioned");
    let split_pieces = imprinted
        .evolution
        .faces
        .iter()
        .filter(|(_, source)| *source == Some(target_face.index()))
        .count();
    assert!(split_pieces > 1, "the anchored target face must split");
    assert!(
        imprinted
            .evolution
            .faces
            .iter()
            .all(|(_, source)| source.is_some())
    );
    assert!(
        imprinted
            .evolution
            .edges
            .iter()
            .all(|(_, event)| { !matches!(event, remus_algo::gfa::EdgeEvent::Unresolved) })
    );
    assert!(imprinted.evolution.edges.iter().any(|(_, event)| {
        matches!(
            event,
            remus_algo::gfa::EdgeEvent::Generated {
                face_a: Some(_),
                face_b: Some(_)
            }
        )
    }));

    match resolve(&topo, &reference) {
        Resolution::BoundMany {
            entities,
            provenance: Provenance::Construction,
        } => assert_eq!(entities.len(), split_pieces),
        other => panic!("split reference did not resolve exactly: {other:?}"),
    }
    assert_eq!(
        resolve(&topo, &tool_reference),
        Resolution::Bound {
            entity: EntityKey::face(tool_face.index()),
            provenance: Provenance::Construction,
        },
        "the unchanged tool must carry through the imprint entry"
    );

    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == imprinted.op)
        .expect("imprint journal entry");
    assert_eq!(entry.kind(), "imprint");
    let EntryPayload::Evolution { origin, events, .. } = entry.payload() else {
        panic!("imprint must journal evolution");
    };
    assert_eq!(
        *origin,
        remus_topology::journal::RecordedOrigin::Construction
    );
    assert!(events.iter().all(|(_, event)| {
        !matches!(event, EntityEvent::Deleted | EntityEvent::Unresolved { .. })
    }));
    let mut modified_counts = BTreeMap::new();
    for (_, event) in events {
        if let EntityEvent::Modified { from } = event {
            *modified_counts.entry(from).or_insert(0usize) += 1;
        }
    }
    assert!(modified_counts.values().any(|&count| count > 1));
    assert!(events.iter().any(|(_, event)| {
        matches!(event, EntityEvent::Generated { sources } if sources.len() == 2)
    }));
}

#[test]
fn identical_handle_imprint_refuses_without_mutation() {
    let (mut topo, target, _) = fixture();
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.allocated_slot_count(),
        topo.journal().entries().len(),
    );
    let error = imprint(&mut topo, target, target).unwrap_err();
    assert!(error.to_string().contains("unsupported imprint"));
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.allocated_slot_count(),
            topo.journal().entries().len(),
        )
    );
}

#[test]
fn imprint_construction_evolution_is_deterministic() {
    let mut runs = Vec::new();
    for _ in 0..2 {
        let (mut topo, target, tool) = fixture();
        let result = imprint(&mut topo, target, tool).unwrap();
        let volume = solid_volume(&topo, result.solid, 0.01).unwrap();
        runs.push((result.evolution, (volume * 1.0e9).round() as i64));
    }
    assert_eq!(runs[0], runs[1]);
}

#[test]
fn disjoint_planar_imprint_refuses_without_mutation() {
    let mut topo = Topology::new();
    let target = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(20.0, 0.0, 0.0)).unwrap();
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.allocated_slot_count(),
        topo.journal().entries().len(),
    );
    let error = imprint(&mut topo, target, tool).unwrap_err();
    assert!(
        error
            .to_string()
            .contains("tool did not divide any target face")
    );
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.allocated_slot_count(),
            topo.journal().entries().len(),
        )
    );
}
