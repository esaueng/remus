//! Shared topology-evolution contracts for direct editing.
//!
//! These tests pin the operation-local source-to-result mapping through
//! real journaled operations: which entity types are tracked (faces,
//! edges, vertices), how unchanged / modified / generated / deleted
//! entities are represented, that one source can map to many results
//! (split) and many sources to one result (merge), that mappings compose
//! across two edits, and that a failed operation rolls back topology and
//! history together.
//!
//! What these tests deliberately do NOT assert:
//! - arena handles (`usize` indices) are session-local; nothing here treats
//!   them as persistent across serialization — see
//!   `step_round_trip_severs_persistent_naming_but_keeps_geometry`;
//! - STEP entity numbers are parse-local and carry no history;
//! - no test performs nearest-face guessing: every rebinding goes through
//!   construction-recorded journal claims resolved by
//!   [`remus_topology::naming::resolve`], failing closed otherwise.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::Vec3;
use remus_operations::boolean::{BooleanOp, boolean_with_evolution};
use remus_operations::imprint::imprint;
use remus_operations::journal_ops::{
    boolean_journaled_with_operation, move_faces_journaled, replace_surface_journaled,
    shell_journaled, solid_entity_keys,
};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::journal::{
    EntityEvent, EntityKey, EntityKind, EntryPayload, EventDraft, EvolutionDraft, JournalOrdinal,
    OpId,
};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
use remus_topology::{FaceId, SolidId, Topology};

/// Construction-recorded anchor over every entity of `solid`, mirroring the
/// pattern in `tests/journal.rs`: each live entity becomes a `Generated`
/// subject with no sources, so `operation_output(anchor, kind, index)`
/// addresses the `index`-th entity of that kind in deterministic order.
struct Anchor {
    op: OpId,
    faces: Vec<EntityKey>,
    edges: Vec<EntityKey>,
    vertices: Vec<EntityKey>,
}

fn anchor_all_entities(topo: &mut Topology, solid: SolidId) -> Anchor {
    let keys = solid_entity_keys(topo, solid).unwrap();
    let pending = topo.journal_begin("anchor");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for &key in &keys {
        draft.push(
            key,
            EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let op = topo.journal_record_evolution(pending, draft).unwrap();
    let mut anchor = Anchor {
        op,
        faces: Vec::new(),
        edges: Vec::new(),
        vertices: Vec::new(),
    };
    for key in keys {
        match key.kind {
            EntityKind::Face => anchor.faces.push(key),
            EntityKind::Edge => anchor.edges.push(key),
            EntityKind::Vertex => anchor.vertices.push(key),
        }
    }
    anchor
}

fn face_ref(op: OpId, index: usize) -> PersistentRef {
    PersistentRef::operation_output(op, EntityKind::Face, index)
}

fn face_with_normal(topo: &Topology, solid: SolidId, normal: Vec3) -> FaceId {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(normal) > 0.9)
        })
        .expect("solid must have a face with the requested normal")
}

/// The evolution entry for `op`: (subject ordinal, event) pairs in
/// deterministic subject order.
fn evolution_events(topo: &Topology, op: OpId) -> Vec<(JournalOrdinal, EntityEvent)> {
    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == op)
        .unwrap_or_else(|| panic!("journal has no entry for op {}", op.value()));
    let EntryPayload::Evolution { events, .. } = entry.payload() else {
        panic!("op {} recorded a barrier, expected evolution", op.value());
    };
    events.clone()
}

fn modified_from(
    events: &[(JournalOrdinal, EntityEvent)],
    from: JournalOrdinal,
) -> Vec<JournalOrdinal> {
    events
        .iter()
        .filter_map(|(subject, event)| match event {
            EntityEvent::Modified { from: source } if *source == from => Some(*subject),
            _ => None,
        })
        .collect()
}

#[test]
fn imprint_preserves_tool_and_splits_target_face() {
    let mut topo = Topology::new();
    let target = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 2.0, 2.0, 4.0).unwrap();
    // The tool straddles the target's top plane transversally: its side
    // faces pierce z = 10 inside the target footprint, its bottom sits
    // interior (no same-domain overlap), its top stays outside.
    transform_solid(&mut topo, tool, &Mat4::translation(-1.0, -1.0, 8.0)).unwrap();

    let target_anchor = anchor_all_entities(&mut topo, target);
    let tool_anchor = anchor_all_entities(&mut topo, tool);
    let top = face_with_normal(&topo, target, Vec3::new(0.0, 0.0, 1.0));
    let top_index = target_anchor
        .faces
        .iter()
        .position(|key| key.index == top.index())
        .expect("anchor covers the top face");

    let result = imprint(&mut topo, target, tool).unwrap();

    // Unchanged entities: every tool face is Preserved, so each tool
    // reference binds the same live entity with construction provenance.
    for (index, key) in tool_anchor.faces.iter().enumerate() {
        match resolve(&topo, &face_ref(tool_anchor.op, index)) {
            Resolution::Bound { entity, provenance } => {
                assert_eq!(entity, *key, "tool face {index} must bind itself");
                assert_eq!(provenance, Provenance::Construction);
            }
            other => panic!("tool face {index} must stay bound, got {other:?}"),
        }
    }
    let events = evolution_events(&topo, result.op);
    for key in &tool_anchor.faces {
        let subject = topo.journal().ordinal_of(*key).unwrap();
        assert!(
            matches!(
                events
                    .iter()
                    .find(|(candidate, _)| *candidate == subject)
                    .map(|(_, event)| event),
                Some(EntityEvent::Preserved { .. })
            ),
            "tool face {key:?} must be a Preserved subject"
        );
    }

    // Split: the pierced top face fans out to several Modified patches.
    let top_ordinal = topo
        .journal()
        .ordinal_of(EntityKey::face(top.index()))
        .unwrap();
    let patches = modified_from(&events, top_ordinal);
    assert!(
        patches.len() >= 2,
        "the pierced top face must split into patches, got {}",
        patches.len()
    );
    match resolve(&topo, &face_ref(target_anchor.op, top_index)) {
        Resolution::BoundMany {
            entities,
            provenance,
        } => {
            assert_eq!(provenance, Provenance::Construction);
            assert!(
                entities.len() >= 2,
                "split top must resolve to all patches, got {entities:?}"
            );
            assert!(
                !entities.contains(&EntityKey::face(top.index())),
                "the consumed input must not resolve to itself"
            );
        }
        other => panic!("split top face must resolve BoundMany, got {other:?}"),
    }

    // The imprint keeps every target patch: total volume is unchanged.
    let volume = solid_volume(&topo, result.solid, 0.1).unwrap();
    assert!(
        (volume - 1000.0).abs() < 1e-6,
        "imprint must preserve volume, got {volume}"
    );
}

#[test]
fn adjacent_fuse_keeps_coplanar_patches_separate() {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let b = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    // Adjacent along x: A spans 0..4, B spans 4..8, sharing the plane x = 4.
    transform_solid(&mut topo, b, &Mat4::translation(4.0, 0.0, 0.0)).unwrap();

    let anchor_a = anchor_all_entities(&mut topo, a);
    let anchor_b = anchor_all_entities(&mut topo, b);
    let top_a = face_with_normal(&topo, a, Vec3::new(0.0, 0.0, 1.0));
    let top_b = face_with_normal(&topo, b, Vec3::new(0.0, 0.0, 1.0));
    let top_a_index = anchor_a
        .faces
        .iter()
        .position(|key| key.index == top_a.index())
        .unwrap();
    let top_b_index = anchor_b
        .faces
        .iter()
        .position(|key| key.index == top_b.index())
        .unwrap();

    let result = boolean_journaled_with_operation(&mut topo, BooleanOp::Fuse, a, b).unwrap();

    // No merge is invented: the GFA keeps coplanar patches as separate
    // Modified faces (10 faces, not 6), so each top binds its own live
    // face. Consumers can rely on this cardinality — a future same-domain
    // merge would resolve BoundMany instead of Bound, and must be a
    // producer claim (as in heal unification), never resolver inference.
    // Geometric merges are covered by
    // `verified_unification_journals_merged_faces_and_consumed_center` in
    // tests/journal.rs; this test pins the fuse side of that boundary.
    let entity_a = match resolve(&topo, &face_ref(anchor_a.op, top_a_index)) {
        Resolution::Bound { entity, provenance } => {
            assert_eq!(provenance, Provenance::Construction);
            entity
        }
        other => panic!("fused top A must stay bound, got {other:?}"),
    };
    let entity_b = match resolve(&topo, &face_ref(anchor_b.op, top_b_index)) {
        Resolution::Bound { entity, provenance } => {
            assert_eq!(provenance, Provenance::Construction);
            entity
        }
        other => panic!("fused top B must stay bound, got {other:?}"),
    };
    assert_ne!(
        entity_a, entity_b,
        "fuse must not merge coplanar patches without a producer claim"
    );
    let events = evolution_events(&topo, result.op);
    for (ordinal_key, expected) in [
        (EntityKey::face(top_a.index()), "top A"),
        (EntityKey::face(top_b.index()), "top B"),
    ] {
        let ordinal = topo.journal().ordinal_of(ordinal_key).unwrap();
        assert!(
            modified_from(&events, ordinal).len() == 1,
            "{expected} must be exactly one Modified subject"
        );
    }
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        10,
        "adjacent fuse keeps all outer patches"
    );

    let volume = solid_volume(&topo, result.solid, 0.1).unwrap();
    assert!(
        (volume - 128.0).abs() < 1e-6,
        "fused 8x4x4 box must have volume 128, got {volume}"
    );
}

#[test]
fn shell_opening_deletes_exactly_the_opened_face() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, solid);
    let top = face_with_normal(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let top_index = anchor
        .faces
        .iter()
        .position(|key| key.index == top.index())
        .unwrap();

    let result = shell_journaled(&mut topo, solid, 1.0, &[top]).unwrap();
    let top_ordinal = topo
        .journal()
        .ordinal_of(EntityKey::face(top.index()))
        .unwrap();
    let events = evolution_events(&topo, result.op);

    // The producer records an explicit Deleted subject for the opened
    // input — that claim is honest and must stay in the entry.
    assert!(
        events.contains(&(top_ordinal, EntityEvent::Deleted)),
        "the opened input must be an explicit Deleted subject"
    );
    // But the new rim face names the opened face as a plausible source,
    // contesting the deletion: an Unresolved candidacy makes every other
    // claim about the input unsafe to follow, so the resolver severs
    // fail-closed naming the operation instead of reporting Dangling.
    // Either spelling refuses to rebind; downstream selections must
    // handle both Dangling and UnresolvedAcrossOperation as terminal.
    assert!(
        events.iter().any(|(_, event)| matches!(
            event,
            EntityEvent::Unresolved { candidates } if candidates.contains(&top_ordinal)
        )),
        "the rim must name the opened face as a candidate, got {events:?}"
    );
    match resolve(&topo, &face_ref(anchor.op, top_index)) {
        Resolution::UnresolvedAcrossOperation { op, .. } => assert_eq!(op, result.op),
        other => panic!("contested deletion must sever fail-closed, got {other:?}"),
    }
    // Every other outer face survives with construction provenance.
    for (index, key) in anchor.faces.iter().enumerate() {
        if index == top_index {
            continue;
        }
        match resolve(&topo, &face_ref(anchor.op, index)) {
            Resolution::Bound { entity, provenance } => {
                assert_eq!(provenance, Provenance::Construction);
                assert!(
                    entity != *key || topo.face_id_from_index(key.index).is_some(),
                    "kept face {index} must bind a live entity"
                );
            }
            other => panic!("kept face {index} must stay bound, got {other:?}"),
        }
    }
    // The inner skin is new geometry, not any input modified.
    assert!(
        events
            .iter()
            .any(|(_, event)| matches!(event, EntityEvent::Generated { .. })),
        "the inner skin must be Generated, got {events:?}"
    );
}

#[test]
fn lineage_composes_across_move_then_replace() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, solid);
    let top = face_with_normal(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let top_index = anchor
        .faces
        .iter()
        .position(|key| key.index == top.index())
        .unwrap();

    // First edit: planar pull moves the top from z = 10 to z = 12.
    let moved = move_faces_journaled(&mut topo, solid, &[top], 2.0).unwrap();
    // Second, different edit: exact support replacement lifts it to z = 14.
    let moved_top = face_with_normal(&topo, moved.solid, Vec3::new(0.0, 0.0, 1.0));
    let replaced = replace_surface_journaled(
        &mut topo,
        moved.solid,
        moved_top,
        remus_topology::face::FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 14.0,
        },
    )
    .unwrap();

    // One reference anchored before both edits chases the two-entry chain
    // to a single live face with construction provenance throughout.
    match resolve(&topo, &face_ref(anchor.op, top_index)) {
        Resolution::Bound { entity, provenance } => {
            assert_eq!(provenance, Provenance::Construction);
            let live_top = face_with_normal(&topo, replaced.solid, Vec3::new(0.0, 0.0, 1.0));
            assert_eq!(entity, EntityKey::face(live_top.index()));
        }
        other => panic!("moved-then-replaced top must compose to one face, got {other:?}"),
    }
    // Both entries claimed this exact face by identity (Modified), not
    // adjacency: walk the ordinal chase step by step and require a
    // single claimant per entry, ending at the live top face.
    let mut ordinal = topo
        .journal()
        .ordinal_of(EntityKey::face(top.index()))
        .unwrap();
    for op in [moved.op, replaced.op] {
        let events = evolution_events(&topo, op);
        let mut subjects = modified_from(&events, ordinal);
        subjects.extend(events.iter().filter_map(|(subject, event)| match event {
            EntityEvent::Merged { from } if from.contains(&ordinal) => Some(*subject),
            _ => None,
        }));
        assert_eq!(
            subjects.len(),
            1,
            "op {} must carry exactly one identity claim for the face",
            op.value()
        );
        ordinal = subjects[0];
    }
    let live_top = face_with_normal(&topo, replaced.solid, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(
        topo.journal().key_of(ordinal),
        Some(EntityKey::face(live_top.index())),
        "the ordinal chase must end at the live top face"
    );
    let volume = solid_volume(&topo, replaced.solid, 0.1).unwrap();
    assert!(
        (volume - 1400.0).abs() < 1e-6,
        "10x10x14 box must have volume 1400, got {volume}"
    );
}

#[test]
fn failed_imprint_rolls_back_topology_and_journal() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let volume_before = solid_volume(&topo, solid, 0.1).unwrap();
    let entries_before = topo.journal().entries().len();
    let witness = face_with_normal(&topo, solid, Vec3::new(0.0, 0.0, 1.0));

    // Target and tool sharing entities is refused before any recording.
    assert!(imprint(&mut topo, solid, solid).is_err());

    assert_eq!(
        topo.journal().entries().len(),
        entries_before,
        "a failed operation must not publish history"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len(),
        faces_before,
        "a failed operation must not change topology"
    );
    let volume_after = solid_volume(&topo, solid, 0.1).unwrap();
    assert!(
        (volume_after - volume_before).abs() < 1e-9,
        "volume must be unchanged by a failed operation"
    );
    assert!(
        topo.face(witness).is_ok(),
        "pre-existing handles must stay live across rollback"
    );
    // Recording after the rollback sees no unjournaled-mutation gap: the
    // failed operation left neither partial topology nor partial history.
    let pending = topo.journal_begin("post_rollback_probe");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(solid_entity_keys(&topo, solid).unwrap());
    topo.journal_record_evolution(pending, draft).unwrap();
    assert!(
        topo.journal()
            .entries()
            .iter()
            .all(|entry| !matches!(entry.payload(), EntryPayload::GlobalBarrier)),
        "a clean rollback must not read as an unjournaled gap"
    );
}

#[test]
fn step_round_trip_severs_persistent_naming_but_keeps_geometry() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, solid);
    let top = face_with_normal(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let top_index = anchor
        .faces
        .iter()
        .position(|key| key.index == top.index())
        .unwrap();
    let moved = move_faces_journaled(&mut topo, solid, &[top], 2.0).unwrap();
    // Sanity: the reference binds in the session that recorded it.
    assert!(matches!(
        resolve(&topo, &face_ref(anchor.op, top_index)),
        Resolution::Bound { .. }
    ));

    // STEP carries geometry, not history: the writer takes only
    // `(topo, solids)` — there is no journal channel in the export API.
    let step = remus_io::step::writer::write_step(&topo, &[moved.solid]).unwrap();
    let mut fresh = Topology::new();
    let imported = remus_io::step::reader::read_step(&step, &mut fresh).unwrap();
    assert_eq!(imported.len(), 1);
    let volume = solid_volume(&fresh, imported[0], 0.1).unwrap();
    assert!(
        (volume - 1200.0).abs() < 1e-3,
        "STEP must preserve the edited geometry, got {volume}"
    );

    // Operation-local evolution does not survive the round trip: the same
    // reference value resolves UnknownOperation in the fresh session
    // instead of rebinding to a lookalike face.
    match resolve(&fresh, &face_ref(moved.op, 0)) {
        Resolution::UnknownOperation { op } => assert_eq!(op, moved.op),
        other => panic!("STEP import must not resurrect history, got {other:?}"),
    }
    assert_eq!(
        fresh.journal().entries().len(),
        0,
        "STEP import must arrive with an empty journal"
    );
}

#[test]
fn cut_consumed_faces_are_deleted_in_map_and_severed_in_journal() {
    // Map path: the faithful boolean reports consumed inputs as deleted.
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(5.0, 0.0, 0.0)).unwrap();
    let plus_x = face_with_normal(&topo, a, Vec3::new(1.0, 0.0, 0.0));
    let (_solid, map) = boolean_with_evolution(&mut topo, BooleanOp::Cut, a, b).unwrap();
    assert!(
        map.deleted.contains(&plus_x.index()),
        "the +X face of A lies inside B and must be reported deleted, got {:?}",
        map.deleted
    );

    // Journal path over an identical setup: the GFA records no face
    // deletions, so the consumed input is severed by its in-scope silence
    // and fails closed naming the operation — never Dangling, never a
    // guess at a surviving face.
    let mut topo = Topology::new();
    let a = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    transform_solid(&mut topo, b, &Mat4::translation(5.0, 0.0, 0.0)).unwrap();
    let anchor = anchor_all_entities(&mut topo, a);
    let plus_x = face_with_normal(&topo, a, Vec3::new(1.0, 0.0, 0.0));
    let plus_x_index = anchor
        .faces
        .iter()
        .position(|key| key.index == plus_x.index())
        .unwrap();
    let result = boolean_journaled_with_operation(&mut topo, BooleanOp::Cut, a, b).unwrap();
    match resolve(&topo, &face_ref(anchor.op, plus_x_index)) {
        Resolution::UnresolvedAcrossOperation { op, .. } => assert_eq!(op, result.op),
        other => panic!("consumed cut face must sever fail-closed, got {other:?}"),
    }
    // The two provenance paths disagree on the failure spelling
    // (map `Deleted` vs journal scope-severing); both refuse to rebind.
    // See docs/design/evolution-contracts.md for the unification proposal.
}
