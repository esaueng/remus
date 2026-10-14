//! An arena document carries the history of its own entities, never the
//! whole session journal (review finding IO-3): serializing and
//! deserializing a body back into the same session must not duplicate
//! unrelated history, and persistent references must still resolve.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::bop::BooleanOp;
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_math::vec::{Point3, Vec3};
use remus_operations::journal_ops::{
    boolean_journaled, record_barrier_over_solid, split_journaled,
};
use remus_operations::primitives::make_box;
use remus_topology::attributes::EntityAttributes;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityKey, EntityKind, OpId};
use remus_topology::naming::{PersistentRef, Resolution, resolve};
use remus_topology::{SolidId, Topology};

fn journaled_box(topo: &mut Topology) -> (SolidId, OpId) {
    let outer = make_box(topo, 7.0, 3.0, 4.0).unwrap();
    let inner = make_box(topo, 1.0, 1.0, 1.0).unwrap();
    let result = boolean_journaled(topo, BooleanOp::Fuse, outer, inner).unwrap();
    (result.solid, result.op)
}

fn reference(op: OpId) -> PersistentRef {
    PersistentRef::operation_output(op, EntityKind::Face, 0)
}

fn bound(topo: &Topology, reference: &PersistentRef) -> EntityKey {
    match resolve(topo, reference) {
        Resolution::Bound { entity, .. } => entity,
        other => panic!("expected a bound selection, got {other:?}"),
    }
}

fn assert_face_of(topo: &Topology, solid: SolidId, key: EntityKey) {
    assert_eq!(key.kind, EntityKind::Face);
    assert!(
        solid_faces(topo, solid)
            .unwrap()
            .iter()
            .any(|face| face.index() == key.index),
        "the reference must bind a face of the expected body"
    );
}

fn journal_of(document: &[u8]) -> serde_json::Value {
    let value: serde_json::Value = serde_json::from_slice(document).unwrap();
    value["journal"].clone()
}

/// The document with its journal's two high-water counters removed.
fn without_counters(document: &[u8]) -> serde_json::Value {
    let mut value: serde_json::Value = serde_json::from_slice(document).unwrap();
    let journal = value["journal"].as_object_mut().unwrap();
    journal.remove("next_op").unwrap();
    journal.remove("next_ordinal").unwrap();
    value
}

#[test]
fn same_session_round_trips_append_the_bodys_lineage_once_per_import() {
    let mut topo = Topology::new();
    let (original, op) = journaled_box(&mut topo);
    let original_reference = reference(op);
    let original_face = bound(&topo, &original_reference);
    let lineage = topo.journal().len();
    let first = serialize_solids(&topo, &[original]).unwrap();

    for _ in 0..6 {
        let entries_before = topo.journal().len();
        let offset = topo.journal().snapshot().next_op;
        let document = serialize_solids(&topo, &[original]).unwrap();
        // Earlier copies' history is not the original's: the document is
        // unchanged except for the high-water counters, at most 20 digits
        // each.
        assert_eq!(without_counters(&document), without_counters(&first));
        assert!(document.len() <= first.len() + 2 * 20);

        let copy = deserialize_solids(&document, &mut topo).unwrap()[0];
        assert_eq!(topo.journal().len(), entries_before + lineage);
        assert_eq!(bound(&topo, &original_reference), original_face);
        assert_face_of(
            &topo,
            copy,
            bound(&topo, &reference(OpId::from_value(offset + op.value()))),
        );
    }
    assert_eq!(topo.journal().len(), 7 * lineage);
}

#[test]
fn chained_same_session_round_trips_carry_only_the_latest_copys_lineage() {
    let mut topo = Topology::new();
    let (mut latest, mut anchor) = journaled_box(&mut topo);
    let first_index = journal_of(&serialize_solids(&topo, &[latest]).unwrap())["index"]
        .as_array()
        .unwrap()
        .len();

    for _ in 0..6 {
        let entries_before = topo.journal().len();
        let offset = topo.journal().snapshot().next_op;
        let document = serialize_solids(&topo, &[latest]).unwrap();
        let journal = journal_of(&document);
        assert_eq!(journal["entries"].as_array().unwrap().len(), 1);
        assert_eq!(journal["entries"][0]["op"], anchor.value());
        assert_eq!(journal["index"].as_array().unwrap().len(), first_index);

        latest = deserialize_solids(&document, &mut topo).unwrap()[0];
        assert_eq!(topo.journal().len(), entries_before + 1);
        anchor = OpId::from_value(offset + anchor.value());
        assert_face_of(&topo, latest, bound(&topo, &reference(anchor)));
    }
}

#[test]
fn history_free_bodies_round_trip_without_touching_the_session_journal() {
    let mut topo = Topology::new();
    // Built before history starts, so no unjournaled gap precedes the import.
    let plain = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let (_, op) = journaled_box(&mut topo);
    let session_reference = reference(op);
    let session_face = bound(&topo, &session_reference);
    let history = topo.journal().snapshot();
    let first = serialize_solids(&topo, &[plain]).unwrap();
    assert!(
        journal_of(&first).is_null(),
        "a body no journal entry mentions carries no history"
    );

    for _ in 0..6 {
        let document = serialize_solids(&topo, &[plain]).unwrap();
        assert_eq!(document, first);
        deserialize_solids(&document, &mut topo).unwrap();
        assert_eq!(topo.journal().snapshot(), history);
        assert_eq!(bound(&topo, &session_reference), session_face);
    }
}

/// Session-independent description of a resolution, restricted to the
/// entities of `solid`: what a full-journal load of a document holding only
/// `solid` would report (entities outside it are not present). No-match
/// outcomes compare by class.
fn describe(topo: &Topology, solid: SolidId, reference: &PersistentRef) -> String {
    let in_solid = |key: &EntityKey| match key.kind {
        EntityKind::Face => solid_faces(topo, solid)
            .unwrap()
            .iter()
            .any(|id| id.index() == key.index),
        EntityKind::Edge => solid_edges(topo, solid)
            .unwrap()
            .iter()
            .any(|id| id.index() == key.index),
        EntityKind::Vertex => solid_vertices(topo, solid)
            .unwrap()
            .iter()
            .any(|id| id.index() == key.index),
    };
    let label = |key: &EntityKey| match key.kind {
        EntityKind::Face => {
            let face = topo.face_id_from_index(key.index).unwrap();
            topo.attributes().face(face).unwrap().name.clone().unwrap()
        }
        EntityKind::Edge => {
            let edge = topo
                .edge(topo.edge_id_from_index(key.index).unwrap())
                .unwrap();
            let mut ends = [edge.start(), edge.end()]
                .map(|vertex| format!("{:?}", topo.vertex(vertex).unwrap().point()));
            ends.sort();
            format!("{}:{ends:?}", edge.curve().type_tag())
        }
        EntityKind::Vertex => {
            let vertex = topo.vertex_id_from_index(key.index).unwrap();
            format!("{:?}", topo.vertex(vertex).unwrap().point())
        }
    };
    let (entities, provenance) = match resolve(topo, reference) {
        Resolution::Bound { entity, provenance } => (vec![entity], provenance),
        Resolution::BoundMany {
            entities,
            provenance,
        } => (entities, provenance),
        Resolution::NoMatch { .. } => return "NoMatch".to_owned(),
        other => return format!("{other:?}"),
    };
    let mut labels: Vec<String> = entities
        .iter()
        .filter(|key| in_solid(key))
        .map(label)
        .collect();
    labels.sort();
    if labels.is_empty() {
        "NoMatch".to_owned()
    } else {
        format!("{labels:?}:{provenance:?}")
    }
}

#[test]
fn fresh_sessions_resolve_the_documents_history_like_a_full_journal_load() {
    let mut source = Topology::new();
    // Every operand exists before history starts, so no unjournaled gap
    // severs the lineages under test.
    let outer = make_box(&mut source, 7.0, 3.0, 4.0).unwrap();
    let inner = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let other_outer = make_box(&mut source, 2.0, 2.0, 2.0).unwrap();
    let other_inner = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let fused = boolean_journaled(&mut source, BooleanOp::Fuse, outer, inner).unwrap();
    let unrelated =
        boolean_journaled(&mut source, BooleanOp::Fuse, other_outer, other_inner).unwrap();
    let split = split_journaled(
        &mut source,
        fused.solid,
        Point3::new(3.5, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let (kept, sibling) = (split.result.positive, split.result.negative);
    let pending = source.journal_begin("edit_sibling");
    let sever = record_barrier_over_solid(&mut source, pending, sibling).unwrap();
    for (index, face) in solid_faces(&source, kept).unwrap().into_iter().enumerate() {
        source
            .set_face_attributes(
                face,
                EntityAttributes {
                    name: Some(format!("kept-{index}")),
                    ..Default::default()
                },
            )
            .unwrap();
    }

    let document = serialize_solids(&source, &[kept]).unwrap();
    let carried: Vec<u64> = journal_of(&document)["entries"]
        .as_array()
        .unwrap()
        .iter()
        .map(|entry| entry["op"].as_u64().unwrap())
        .collect();
    // The sibling's barrier rides along (it severs the shared lineage); the
    // unrelated body's history does not.
    assert_eq!(carried, [fused.op, split.op, sever].map(OpId::value));

    let mut restored = Topology::new();
    let restored_solid = deserialize_solids(&document, &mut restored).unwrap()[0];
    let mut severed_by_sibling = 0;
    for op in [fused.op, unrelated.op, split.op, sever] {
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..32 {
                let reference = PersistentRef::operation_output(op, kind, index);
                let after = describe(&restored, restored_solid, &reference);
                if op == unrelated.op {
                    assert!(
                        matches!(
                            resolve(&restored, &reference),
                            Resolution::UnknownOperation { .. }
                        ),
                        "{after}"
                    );
                    continue;
                }
                let before = describe(&source, kept, &reference);
                assert_eq!(after, before, "{op:?} {kind:?} #{index}");
                if op == fused.op
                    && matches!(
                        resolve(&source, &reference),
                        Resolution::UnresolvedAcrossOperation { op, .. } if op == sever
                    )
                {
                    severed_by_sibling += 1;
                }
            }
        }
    }
    assert!(
        severed_by_sibling > 0,
        "the fixture must exercise a lineage that only the sibling's barrier severs"
    );
}
