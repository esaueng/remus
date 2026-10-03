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

fn empty_geometry_journal(index_count: usize, payloads: Vec<serde_json::Value>) -> Vec<u8> {
    let index: Vec<_> = (0..index_count)
        .map(|ordinal| serde_json::json!({"ordinal":ordinal,"kind":"face","local":null}))
        .collect();
    let entries: Vec<_> = payloads
        .into_iter()
        .enumerate()
        .map(|(op, mut payload)| {
            payload["op"] = serde_json::json!(op);
            payload["kind"] = serde_json::json!("imported");
            payload
        })
        .collect();
    serde_json::to_vec(&serde_json::json!({
        "version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[],
        "solids":[],"solid_roots":[],"compounds":[],"pcurves":[],
        "journal":{"next_op":entries.len(),"next_ordinal":index_count,"index":index,"entries":entries}
    }))
    .unwrap()
}

#[test]
fn expanded_global_barriers_obey_append_budget_and_preserve_destination_on_refusal() {
    // There are only 32 encoded journal array items. Appending expands the
    // sixteen compact global barriers to 16*16 ordinal references, totaling
    // 288 append items. The encoded input is therefore below a cap of 100.
    let bytes =
        empty_geometry_journal(16, vec![serde_json::json!({"payload":"GlobalBarrier"}); 16]);
    let limits = ImportLimits {
        max_model_entities: 100,
        ..ImportLimits::default()
    };
    let mut fresh = Topology::new();
    deserialize_document_with_limits(&bytes, &mut fresh, limits).unwrap();
    assert!(
        fresh
            .journal()
            .snapshot()
            .entries
            .iter()
            .all(|entry| { matches!(entry.payload, PayloadSnapshot::GlobalBarrier) })
    );

    let mut destination = Topology::new();
    let (solid, op) = journaled_box(&mut destination, 2.0);
    let old_reference = reference(op);
    let old_face = bound(&destination, &old_reference);
    let before = serialize_document(&destination, &[solid], &[]).unwrap();
    let journal_before = destination.journal().snapshot();
    let slots_before = destination.allocated_slot_count();
    let cache_before = destination.cache_identity();
    let ticks_before = destination.mutation_ticks();
    for cap in [100, 287] {
        let error = deserialize_document_with_limits(
            &bytes,
            &mut destination,
            ImportLimits {
                max_model_entities: cap,
                ..limits
            },
        )
        .unwrap_err();
        assert!(matches!(error, IoError::LimitExceeded {
            resource: "arena journal append items", limit, actual:288
        } if limit == cap));
        assert_eq!(
            serialize_document(&destination, &[solid], &[]).unwrap(),
            before
        );
        assert_eq!(destination.journal().snapshot(), journal_before);
        assert_eq!(destination.allocated_slot_count(), slots_before);
        assert_eq!(destination.cache_identity(), cache_before);
        assert_eq!(destination.mutation_ticks(), ticks_before);
        assert_eq!(bound(&destination, &old_reference), old_face);
    }
    deserialize_document_with_limits(
        &bytes,
        &mut destination,
        ImportLimits {
            max_model_entities: 288,
            ..limits
        },
    )
    .unwrap();
    let appended = destination.journal().snapshot();
    let expected: Vec<_> = (0..16)
        .map(|ordinal| ordinal + journal_before.next_ordinal)
        .collect();
    for (position, entry) in appended.entries[journal_before.entries.len()..]
        .iter()
        .enumerate()
    {
        assert_eq!(
            entry.op,
            journal_before.next_op + u64::try_from(position).unwrap()
        );
        assert_eq!(
            entry.payload,
            PayloadSnapshot::Barrier {
                affected: expected.clone()
            }
        );
    }
    assert_eq!(bound(&destination, &old_reference), old_face);
}

#[test]
fn restored_evolution_scopes_share_the_checked_append_budget() {
    for (event, reference_count) in [
        (serde_json::json!({"event":"Preserved","from":1}), 1),
        (serde_json::json!({"event":"Modified","from":1}), 1),
        (
            serde_json::json!({"event":"Generated","sources":[1,2,3]}),
            3,
        ),
        (serde_json::json!({"event":"Merged","from":[1,2,3]}), 3),
        (
            serde_json::json!({"event":"Unresolved","candidates":[1,2,3]}),
            3,
        ),
    ] {
        for scope in [vec![], vec![0, 1, 2, 3]] {
            // Four index items, one entry, one event, its reconstructed
            // subject, and references stored both in the event and scope.
            let append_items = 4 + 1 + scope.len() + 2 + 2 * reference_count;
            let bytes = empty_geometry_journal(
                4,
                vec![serde_json::json!({
                    "payload":"Evolution","construction":false,"scope":scope,"events":[[0,event]]
                })],
            );
            let mut destination = Topology::new();
            let pending = destination.journal_begin("sentinel");
            destination.journal_record_barrier(pending, vec![]);
            let before = destination.journal().snapshot();
            let error = deserialize_document_with_limits(
                &bytes,
                &mut destination,
                ImportLimits {
                    max_model_entities: append_items - 1,
                    ..ImportLimits::default()
                },
            )
            .unwrap_err();
            assert!(matches!(error, IoError::LimitExceeded {
                resource:"arena journal append items", actual, ..
            } if actual == append_items));
            assert_eq!(destination.journal().snapshot(), before);
            deserialize_document_with_limits(
                &bytes,
                &mut destination,
                ImportLimits {
                    max_model_entities: append_items,
                    ..ImportLimits::default()
                },
            )
            .unwrap();
            assert_eq!(destination.journal().snapshot().entries.len(), 2);
        }
    }
}

#[test]
fn compact_empty_and_explicit_scopes_keep_zero_and_exact_limits() {
    for affected in [vec![], vec![0, 1, 2, 3]] {
        let bytes = empty_geometry_journal(
            4,
            vec![serde_json::json!({
                "payload":"Barrier","affected":affected
            })],
        );
        let cap = 4 + 1 + affected.len();
        let mut destination = Topology::new();
        let pending = destination.journal_begin("sentinel");
        destination.journal_record_barrier(pending, vec![]);
        deserialize_document_with_limits(
            &bytes,
            &mut destination,
            ImportLimits {
                max_model_entities: cap,
                ..ImportLimits::default()
            },
        )
        .unwrap();
        assert_eq!(destination.journal().snapshot().entries.len(), 2);
    }
    let mut destination = Topology::new();
    let pending = destination.journal_begin("sentinel");
    destination.journal_record_barrier(pending, vec![]);
    let before = destination.journal().snapshot();
    deserialize_document_with_limits(
        &empty_geometry_journal(0, vec![]),
        &mut destination,
        ImportLimits {
            max_model_entities: 0,
            ..ImportLimits::default()
        },
    )
    .unwrap();
    assert_eq!(destination.journal().snapshot(), before);
}

#[test]
fn retired_empty_destination_namespace_still_budgets_expanded_barriers() {
    let mut destination = Topology::new();
    let checkpoint = destination.clone();
    let _ = journaled_box(&mut destination, 2.0);
    destination.restore_preserving_handle_slots(&checkpoint);
    let before = destination.journal().snapshot();
    assert!(before.entries.is_empty());
    assert!(before.next_op > 0);
    let slots_before = destination.allocated_slot_count();
    let mut encoded: serde_json::Value = serde_json::from_slice(&empty_geometry_journal(
        16,
        vec![serde_json::json!({"payload":"GlobalBarrier"}); 16],
    ))
    .unwrap();
    // Reserved source counter holes do not create additional vector slots.
    encoded["journal"]["next_op"] = serde_json::json!(1_000_000_000_000_u64);
    encoded["journal"]["next_ordinal"] = serde_json::json!(1_000_000_000_000_u64);
    let bytes = serde_json::to_vec(&encoded).unwrap();
    let error = deserialize_document_with_limits(
        &bytes,
        &mut destination,
        ImportLimits {
            max_model_entities: 100,
            ..ImportLimits::default()
        },
    )
    .unwrap_err();
    assert!(matches!(
        error,
        IoError::LimitExceeded {
            resource: "arena journal append items",
            actual: 288,
            ..
        }
    ));
    assert_eq!(destination.journal().snapshot(), before);
    assert_eq!(destination.allocated_slot_count(), slots_before);
    deserialize_document_with_limits(
        &bytes,
        &mut destination,
        ImportLimits {
            max_model_entities: 288,
            ..ImportLimits::default()
        },
    )
    .unwrap();
    let imported = destination.journal().snapshot();
    assert_eq!(imported.next_op, before.next_op + 1_000_000_000_000);
    assert_eq!(
        imported.next_ordinal,
        before.next_ordinal + 1_000_000_000_000
    );
    assert!(imported.entries.iter().all(|entry|matches!(entry.payload, PayloadSnapshot::Barrier { ref affected } if affected.len()==16)));
}
