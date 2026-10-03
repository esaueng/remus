//! Appending exact arena documents must not redirect an existing selection or
//! turn an imported history barrier into a claim about an unrelated model.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::bop::BooleanOp;
use remus_io::IoError;
use remus_io::arena_io::{
    deserialize_document, deserialize_document_with_limits, serialize_document,
};
use remus_io::limits::ImportLimits;
use remus_math::mat::Mat4;
use remus_operations::journal_ops::boolean_journaled;
use remus_operations::primitives::make_box;
use remus_topology::explorer::solid_faces;
use remus_topology::journal::{
    EntityKey, EntityKind, OpId, PayloadSnapshot, UNJOURNALED_MUTATIONS,
};
use remus_topology::naming::{PersistentRef, Resolution, resolve};
use remus_topology::{SolidId, Topology};

fn journaled_box(topo: &mut Topology, width: f64) -> (SolidId, OpId) {
    let outer = make_box(topo, width, 3.0, 4.0).unwrap();
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

fn assert_imported_face(topo: &Topology, solid: SolidId, key: EntityKey) {
    assert_eq!(key.kind, EntityKind::Face);
    assert!(
        solid_faces(topo, solid)
            .unwrap()
            .iter()
            .any(|face| face.index() == key.index),
        "the rebased source selection must bind an imported face"
    );
}

#[test]
fn independent_operation_zero_journals_preserve_destination_selections() {
    let mut destination = Topology::new();
    let (_, destination_op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(destination_op);
    let old_face = bound(&destination, &old_reference);
    let original_history = destination.journal().snapshot();

    let mut source = Topology::new();
    let (source_solid, source_op) = journaled_box(&mut source, 7.0);
    assert_eq!(destination_op.value(), 0);
    assert_eq!(source_op.value(), 0);
    let document = serialize_document(&source, &[source_solid], &[]).unwrap();
    let roots = deserialize_document(&document, &mut destination).unwrap();

    assert_eq!(bound(&destination, &old_reference), old_face);
    let imported_op = OpId::from_value(original_history.next_op + source_op.value());
    assert_imported_face(
        &destination,
        roots.solids[0],
        bound(&destination, &reference(imported_op)),
    );
    let combined = destination.journal().snapshot();
    assert_eq!(
        combined.entries[..original_history.entries.len()],
        original_history.entries
    );
    assert_eq!(
        combined.index[..original_history.index.len()],
        original_history.index
    );
    assert_eq!(combined.next_op, 2);
}

#[test]
fn imported_parent_operations_chase_rebased_output_ordinals() {
    let mut source = Topology::new();
    // Construct both tools before history starts, so there is no unrelated
    // unjournaled gap between the two recorded operations.
    let outer = make_box(&mut source, 7.0, 3.0, 4.0).unwrap();
    let tool_a = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let tool_b = make_box(&mut source, 0.5, 0.5, 0.5).unwrap();
    let parent = boolean_journaled(&mut source, BooleanOp::Fuse, outer, tool_a).unwrap();
    let child = boolean_journaled(&mut source, BooleanOp::Fuse, parent.solid, tool_b).unwrap();
    let parent_reference = reference(parent.op);
    assert_imported_face(&source, child.solid, bound(&source, &parent_reference));
    let document = serialize_document(&source, &[child.solid], &[]).unwrap();

    let mut destination = Topology::new();
    let (_, old_op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(old_op);
    let old_face = bound(&destination, &old_reference);
    let offset = destination.journal().snapshot().next_op;
    let roots = deserialize_document(&document, &mut destination).unwrap();
    let rebased_parent = reference(OpId::from_value(offset + parent.op.value()));
    let rebased_child = reference(OpId::from_value(offset + child.op.value()));

    assert_imported_face(
        &destination,
        roots.solids[0],
        bound(&destination, &rebased_parent),
    );
    assert_imported_face(
        &destination,
        roots.solids[0],
        bound(&destination, &rebased_child),
    );
    assert_eq!(bound(&destination, &old_reference), old_face);
}

#[test]
fn imported_global_and_explicit_barriers_are_scoped_to_imported_entities() {
    let mut source = Topology::new();
    let (source_solid, source_op) = journaled_box(&mut source, 7.0);
    let source_face = bound(&source, &reference(source_op));
    let face_id = source.face_id_from_index(source_face.index).unwrap();
    // Even taking a mutable borrow marks a real, conservatively unknown edit.
    let _ = source.face_mut(face_id).unwrap();
    let pending = source.journal_begin("source_explicit_barrier");
    source.journal_record_barrier(pending, vec![source_face]);
    assert!(matches!(
        resolve(&source, &reference(source_op)),
        Resolution::UnresolvedAcrossOperation { .. }
    ));
    let document = serialize_document(&source, &[source_solid], &[]).unwrap();

    let mut destination = Topology::new();
    let (_, old_op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(old_op);
    let old_face = bound(&destination, &old_reference);
    let offset = destination.journal().snapshot().next_op;
    deserialize_document(&document, &mut destination).unwrap();

    assert_eq!(bound(&destination, &old_reference), old_face);
    assert!(matches!(
        resolve(&destination, &reference(OpId::from_value(offset + source_op.value()))),
        Resolution::UnresolvedAcrossOperation { kind, .. } if kind == UNJOURNALED_MUTATIONS
    ));
    // The explicit barrier also retains a disjoint, imported-only scope.
    assert!(
        destination
            .journal()
            .barriers_crossing(destination.journal().ordinal_of(old_face).unwrap())
            .is_empty()
    );
}

#[test]
fn fresh_destination_keeps_source_reference_ids_and_clean_history() {
    let mut source = Topology::new();
    let (solid, op) = journaled_box(&mut source, 7.0);
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    let mut destination = Topology::new();
    let roots = deserialize_document(&document, &mut destination).unwrap();

    assert_imported_face(
        &destination,
        roots.solids[0],
        bound(&destination, &reference(op)),
    );
    assert_eq!(
        destination.journal().snapshot().entries,
        source.journal().snapshot().entries
    );
    let pending = destination.journal_begin("after_clean_load");
    destination.journal_record_barrier(pending, Vec::new());
    assert!(
        destination
            .journal()
            .entries()
            .iter()
            .all(|entry| entry.kind() != UNJOURNALED_MUTATIONS)
    );
}

#[test]
fn fresh_destination_preserves_global_barrier_payload_and_roundtrip_bytes() {
    let mut source = Topology::new();
    let (solid, op) = journaled_box(&mut source, 7.0);
    let face = bound(&source, &reference(op));
    let face_id = source.face_id_from_index(face.index).unwrap();
    let _ = source.face_mut(face_id).unwrap();
    let pending = source.journal_begin("after_gap");
    source.journal_record_barrier(pending, Vec::new());
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    assert!(
        source
            .journal()
            .snapshot()
            .entries
            .iter()
            .any(|entry| { matches!(entry.payload, PayloadSnapshot::GlobalBarrier) })
    );

    let mut destination = Topology::new();
    let roots = deserialize_document(&document, &mut destination).unwrap();
    assert_eq!(
        destination.journal().snapshot().entries,
        source.journal().snapshot().entries
    );
    assert_eq!(
        serialize_document(&destination, &roots.solids, &[]).unwrap(),
        document
    );
}

#[test]
fn history_free_translator_documents_preserve_destination_continuity() {
    let mut destination = Topology::new();
    let (_, op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(op);
    let old_face = bound(&destination, &old_reference);
    let original_history = destination.journal().snapshot();

    let mut source = Topology::new();
    let solid = make_box(&mut source, 7.0, 3.0, 4.0).unwrap();
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    let roots = deserialize_document(&document, &mut destination).unwrap();
    assert_eq!(bound(&destination, &old_reference), old_face);
    assert_eq!(destination.journal().snapshot(), original_history);
    let pending = destination.journal_begin("after_import");
    destination.journal_record_barrier(pending, Vec::new());
    assert_eq!(bound(&destination, &old_reference), old_face);

    // Synchronizing the append must not disable subsequent gap detection.
    remus_operations::transform::transform_solid(
        &mut destination,
        roots.solids[0],
        &Mat4::translation(1.0, 0.0, 0.0),
    )
    .unwrap();
    let pending = destination.journal_begin("after_real_mutation");
    destination.journal_record_barrier(pending, Vec::new());
    assert!(matches!(
        resolve(&destination, &old_reference),
        Resolution::UnresolvedAcrossOperation { kind, .. } if kind == UNJOURNALED_MUTATIONS
    ));
}

#[test]
fn preexisting_unjournaled_destination_gap_is_not_hidden_by_import() {
    let mut destination = Topology::new();
    let (_, op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(op);
    let old_face = bound(&destination, &old_reference);
    let face = destination.face_id_from_index(old_face.index).unwrap();
    let _ = destination.face_mut(face).unwrap();

    let mut source = Topology::new();
    let (solid, _) = journaled_box(&mut source, 7.0);
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    deserialize_document(&document, &mut destination).unwrap();
    assert!(matches!(
        resolve(&destination, &old_reference),
        Resolution::UnresolvedAcrossOperation { kind, .. } if kind == UNJOURNALED_MUTATIONS
    ));
}

#[test]
fn checkpoint_restore_never_reissues_imported_operation_ids_and_invalidates_cache() {
    let mut destination = Topology::new();
    let (_, op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(op);
    let old_face = bound(&destination, &old_reference);
    let checkpoint = destination.clone();

    let mut source = Topology::new();
    let (solid, source_op) = journaled_box(&mut source, 7.0);
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    let first_offset = destination.journal().snapshot().next_op;
    let generation_before = destination.cache_generation();
    let first = deserialize_document(&document, &mut destination).unwrap();
    let stale_op = OpId::from_value(first_offset + source_op.value());
    let stale_reference = reference(stale_op);
    assert_imported_face(
        &destination,
        first.solids[0],
        bound(&destination, &stale_reference),
    );
    assert!(destination.cache_generation() > generation_before);
    let generation_imported = destination.cache_generation();

    destination.restore_preserving_handle_slots(&checkpoint);
    assert!(destination.cache_generation() > generation_imported);
    assert_eq!(bound(&destination, &old_reference), old_face);
    assert!(matches!(
        resolve(&destination, &stale_reference),
        Resolution::UnknownOperation { .. }
    ));
    let next_offset = destination.journal().snapshot().next_op;
    assert!(next_offset > stale_op.value());
    let second = deserialize_document(&document, &mut destination).unwrap();
    assert_imported_face(
        &destination,
        second.solids[0],
        bound(
            &destination,
            &reference(OpId::from_value(next_offset + source_op.value())),
        ),
    );
    assert!(matches!(
        resolve(&destination, &stale_reference),
        Resolution::UnknownOperation { .. }
    ));
}

#[test]
fn empty_restored_journal_keeps_retired_operation_namespace_reserved() {
    let mut destination = Topology::new();
    let checkpoint = destination.clone();
    let (_, retired_op) = journaled_box(&mut destination, 2.0);
    destination.restore_preserving_handle_slots(&checkpoint);
    assert!(destination.journal().is_empty());
    let reserved = destination.journal().snapshot();
    assert!(reserved.next_op > retired_op.value());

    let mut source = Topology::new();
    let (solid, source_op) = journaled_box(&mut source, 7.0);
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    let roots = deserialize_document(&document, &mut destination).unwrap();
    assert!(matches!(
        resolve(&destination, &reference(retired_op)),
        Resolution::UnknownOperation { .. }
    ));
    assert_imported_face(
        &destination,
        roots.solids[0],
        bound(
            &destination,
            &reference(OpId::from_value(reserved.next_op + source_op.value())),
        ),
    );
    assert!(
        destination
            .journal()
            .snapshot()
            .index
            .iter()
            .all(|(ordinal, _)| *ordinal >= reserved.next_ordinal)
    );
}

#[test]
fn history_free_import_keeps_an_empty_journals_reserved_counters() {
    let mut destination = Topology::new();
    let checkpoint = destination.clone();
    let (_, retired_op) = journaled_box(&mut destination, 2.0);
    destination.restore_preserving_handle_slots(&checkpoint);
    let reserved = destination.journal().snapshot();
    assert!(reserved.entries.is_empty());
    assert!(reserved.next_op > retired_op.value());

    let mut source = Topology::new();
    let solid = make_box(&mut source, 7.0, 3.0, 4.0).unwrap();
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    deserialize_document(&document, &mut destination).unwrap();
    assert_eq!(destination.journal().snapshot(), reserved);
    assert!(matches!(
        resolve(&destination, &reference(retired_op)),
        Resolution::UnknownOperation { .. }
    ));
    let pending = destination.journal_begin("after_history_free_import");
    let fresh_op = destination.journal_record_barrier(pending, Vec::new());
    assert!(fresh_op.value() > retired_op.value());
}

#[test]
fn malformed_imported_history_and_counter_overflow_leave_destination_unchanged() {
    let mut destination = Topology::new();
    let (original, op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(op);
    let old_face = bound(&destination, &old_reference);
    let before = serialize_document(&destination, &[original], &[]).unwrap();
    let journal_before = destination.journal().snapshot();
    let cache_before = destination.cache_identity();
    let ticks_before = destination.mutation_ticks();
    let slots_before = destination.allocated_slot_count();

    let mut source = Topology::new();
    let (solid, _) = journaled_box(&mut source, 7.0);
    let document = serialize_document(&source, &[solid], &[]).unwrap();
    for invalid_field in ["local", "next_op", "next_ordinal"] {
        let mut malformed: serde_json::Value = serde_json::from_slice(&document).unwrap();
        if invalid_field == "local" {
            malformed["journal"]["index"][0]["local"] = serde_json::json!(u64::MAX);
        } else {
            malformed["journal"][invalid_field] = serde_json::json!(u64::MAX);
        }
        let error =
            deserialize_document(&serde_json::to_vec(&malformed).unwrap(), &mut destination)
                .unwrap_err();
        if invalid_field != "local" {
            assert!(
                error.to_string().contains("overflows during append"),
                "{error}"
            );
        }
        assert_eq!(
            serialize_document(&destination, &[original], &[]).unwrap(),
            before
        );
        assert_eq!(destination.journal().snapshot(), journal_before);
        assert_eq!(destination.cache_identity(), cache_before);
        assert_eq!(destination.mutation_ticks(), ticks_before);
        assert_eq!(destination.allocated_slot_count(), slots_before);
        assert_eq!(bound(&destination, &old_reference), old_face);
    }
}

fn global_barrier_document(ordinals: u64, barriers: u64) -> Vec<u8> {
    let empty = serialize_document(&Topology::new(), &[], &[]).unwrap();
    let mut document: serde_json::Value = serde_json::from_slice(&empty).unwrap();
    document["journal"] = serde_json::json!({
        "next_op": barriers,
        "next_ordinal": ordinals,
        "index": (0..ordinals).map(|ordinal| serde_json::json!({"ordinal": ordinal, "kind": "face", "local": null})).collect::<Vec<_>>(),
        "entries": (0..barriers).map(|op| serde_json::json!({"op": op, "kind": "unjournaled_mutations", "payload": "GlobalBarrier"})).collect::<Vec<_>>(),
    });
    serde_json::to_vec(&document).unwrap()
}

#[test]
fn global_barrier_append_expansion_is_bounded_before_allocation_and_atomic() {
    let mut destination = Topology::new();
    let (solid, op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(op);
    let old_face = bound(&destination, &old_reference);
    let document_before = serialize_document(&destination, &[solid], &[]).unwrap();
    let journal_before = destination.journal().snapshot();
    let identity_before = destination.cache_identity();
    let ticks_before = destination.mutation_ticks();
    let slots_before = destination.allocated_slot_count();
    let input = global_barrier_document(200, 200);
    let limits = ImportLimits {
        max_model_entities: 1000,
        ..ImportLimits::default()
    };
    assert!(matches!(
        deserialize_document_with_limits(&input, &mut destination, limits),
        Err(IoError::LimitExceeded {
            resource: "arena journal append items",
            limit: 1000,
            actual: 40_400,
        })
    ));
    assert_eq!(
        serialize_document(&destination, &[solid], &[]).unwrap(),
        document_before
    );
    assert_eq!(destination.journal().snapshot(), journal_before);
    assert_eq!(destination.cache_identity(), identity_before);
    assert_eq!(destination.mutation_ticks(), ticks_before);
    assert_eq!(destination.allocated_slot_count(), slots_before);
    assert_eq!(bound(&destination, &old_reference), old_face);

    // A fresh load keeps compact global payloads and the source namespace;
    // it does not allocate the append-only scoped expansion.
    let mut fresh = Topology::new();
    deserialize_document_with_limits(&input, &mut fresh, limits).unwrap();
    assert!(
        fresh
            .journal()
            .snapshot()
            .entries
            .iter()
            .all(|entry| matches!(entry.payload, PayloadSnapshot::GlobalBarrier))
    );
}

#[test]
fn global_barrier_append_accepts_exact_aggregate_expansion_boundary() {
    let input = global_barrier_document(20, 20);
    // Twenty index entries, twenty entries, and twenty scoped vectors of
    // twenty ordinals consume a single 440-item budget.
    for (limit, admitted) in [(439, false), (440, true)] {
        let mut destination = Topology::new();
        let (_, op) = journaled_box(&mut destination, 2.0);
        let old_reference = reference(op);
        let old_face = bound(&destination, &old_reference);
        let options = ImportLimits {
            max_model_entities: limit,
            ..ImportLimits::default()
        };
        let result = deserialize_document_with_limits(&input, &mut destination, options);
        if admitted {
            result.unwrap();
            let snapshot = destination.journal().snapshot();
            let affected: usize = snapshot
                .entries
                .iter()
                .map(|entry| match &entry.payload {
                    PayloadSnapshot::Barrier { affected } => affected.len(),
                    PayloadSnapshot::GlobalBarrier | PayloadSnapshot::Evolution { .. } => 0,
                })
                .sum();
            assert_eq!(affected, 400);
            assert_eq!(bound(&destination, &old_reference), old_face);
        } else {
            assert!(matches!(
                result,
                Err(IoError::LimitExceeded {
                    resource: "arena journal append items",
                    limit: 439,
                    actual: 440
                })
            ));
        }
    }
}

#[test]
fn append_budget_includes_explicit_payloads_and_reconstructed_evolution_scope() {
    let mut document: serde_json::Value =
        serde_json::from_slice(&global_barrier_document(20, 20)).unwrap();
    let entries = document["journal"]["entries"].as_array_mut().unwrap();
    entries.push(serde_json::json!({
        "op": 20,
        "kind": "source_evolution",
        "payload": "Evolution",
        "construction": false,
        "scope": [0, 1],
        "events": [
            [0, {"event": "Generated", "sources": [1, 2]}],
            [1, {"event": "Modified", "from": 2}],
            [2, {"event": "Deleted"}],
        ],
    }));
    entries.push(serde_json::json!({
        "op": 21,
        "kind": "source_barrier",
        "payload": "Barrier",
        "affected": [3, 4, 5],
    }));
    document["journal"]["next_op"] = serde_json::json!(22);
    let input = serde_json::to_vec(&document).unwrap();
    // Index + entries + expanded barriers = 442. Evolution uses two scope
    // slots, three events, three subjects added to scope, and three references
    // each retained in the event and added to scope (14). Explicit barrier = 3.
    for (limit, admitted) in [(458, false), (459, true)] {
        let mut destination = Topology::new();
        let (_, op) = journaled_box(&mut destination, 2.0);
        let old_reference = reference(op);
        let old_face = bound(&destination, &old_reference);
        let history_before = destination.journal().snapshot();
        let options = ImportLimits {
            max_model_entities: limit,
            ..ImportLimits::default()
        };
        let result = deserialize_document_with_limits(&input, &mut destination, options);
        if admitted {
            result.unwrap();
            let snapshot = destination.journal().snapshot();
            let evolution = &snapshot.entries[history_before.entries.len() + 20];
            assert!(matches!(
                &evolution.payload,
                PayloadSnapshot::Evolution { scope, events, .. }
                    if scope.len() == 3 && events.len() == 3
            ));
        } else {
            assert!(matches!(
                result,
                Err(IoError::LimitExceeded {
                    resource: "arena journal append items",
                    limit: 458,
                    actual: 459,
                })
            ));
            assert_eq!(destination.journal().snapshot(), history_before);
        }
        assert_eq!(bound(&destination, &old_reference), old_face);
    }
}
