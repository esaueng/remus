//! Integration tests for the evolution journal (RFC 0003, Stage 1).
//!
//! The stage-1 exit gate: every operation either journals real evolution
//! or an explicit barrier — and anything that bypasses the journal
//! entirely surfaces as a synthetic global barrier, so no operation is
//! silently absent from history.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::bop::BooleanOp;
use remus_operations::journal_ops::{
    begin_scoped, boolean_journaled, record_barrier_over_solid, record_face_evolution,
};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{
    EntityEvent, EntityKey, EntryPayload, RecordedOrigin, UNJOURNALED_MUTATIONS,
};

fn two_overlapping_boxes(
    topo: &mut Topology,
) -> (remus_topology::SolidId, remus_topology::SolidId) {
    let a = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let b = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let shift = remus_math::mat::Mat4::translation(5.0, 5.0, 5.0);
    remus_operations::transform::transform_solid(topo, b, &shift).unwrap();
    (a, b)
}

#[test]
fn journaled_boolean_records_total_construction_history() {
    let mut topo = Topology::new();
    let (a, b) = two_overlapping_boxes(&mut topo);

    let result = boolean_journaled(&mut topo, BooleanOp::Fuse, a, b).unwrap();

    let journal = topo.journal();
    let entry = journal.entries().last().unwrap();
    assert_eq!(entry.op(), result.op);
    assert_eq!(entry.kind(), "boolean_fuse");
    let EntryPayload::Evolution { origin, events, .. } = entry.payload() else {
        panic!("a journaled boolean records evolution, not a barrier");
    };
    assert_eq!(
        *origin,
        RecordedOrigin::Construction,
        "GFA evolution is construction-derived, and the journal must say so"
    );

    // Totality: every face, edge, and vertex of the result is a subject of
    // exactly one event (the entry refuses duplicates, so counting the
    // matches proves both directions).
    let faces = solid_faces(&topo, result.solid).unwrap();
    let edges = solid_edges(&topo, result.solid).unwrap();
    let vertices = solid_vertices(&topo, result.solid).unwrap();
    for key in faces
        .iter()
        .map(|id| EntityKey::face(id.index()))
        .chain(edges.iter().map(|id| EntityKey::edge(id.index())))
        .chain(vertices.iter().map(|id| EntityKey::vertex(id.index())))
    {
        let ordinal = journal
            .ordinal_of(key)
            .unwrap_or_else(|| panic!("{key:?} missing from the journal index"));
        assert!(
            events
                .binary_search_by_key(&ordinal, |(subject, _)| *subject)
                .is_ok(),
            "{key:?} has no event: history must be total over the result"
        );
        // The live index round-trips: ordinal → current arena key.
        assert_eq!(journal.key_of(ordinal), Some(key));
    }

    // The fuse of two overlapping boxes preserves far entities, modifies
    // crossing ones, and generates section geometry — all three claim
    // strengths must be present, and every claim strength that binds
    // (Preserved/Modified) must reference an ordinal the index knows.
    let mut preserved = 0;
    let mut modified = 0;
    let mut generated = 0;
    for (_, event) in events {
        match event {
            EntityEvent::Preserved { from } | EntityEvent::Modified { from } => {
                assert!(journal.key_of(*from).is_some());
                if matches!(event, EntityEvent::Preserved { .. }) {
                    preserved += 1;
                } else {
                    modified += 1;
                }
            }
            EntityEvent::Generated { .. } => generated += 1,
            EntityEvent::Merged { .. } | EntityEvent::Deleted | EntityEvent::Unresolved { .. } => {}
        }
    }
    assert!(preserved > 0, "far entities are preserved");
    assert!(modified > 0, "crossing entities are modified");
    assert!(generated > 0, "section geometry is generated");
}

#[test]
fn journaled_boolean_is_deterministic() {
    let run = || {
        let mut topo = Topology::new();
        let (a, b) = two_overlapping_boxes(&mut topo);
        let result = boolean_journaled(&mut topo, BooleanOp::Fuse, a, b).unwrap();
        let entry = topo.journal().entries().last().unwrap().clone();
        (result.op, entry)
    };
    let (op_1, entry_1) = run();
    let (op_2, entry_2) = run();
    assert_eq!(op_1, op_2);
    assert_eq!(
        entry_1, entry_2,
        "identical history must journal identically"
    );
}

#[test]
fn unjournaled_operation_surfaces_as_a_global_barrier() {
    let mut topo = Topology::new();
    let (a, b) = two_overlapping_boxes(&mut topo);
    let fused = boolean_journaled(&mut topo, BooleanOp::Fuse, a, b).unwrap();

    // An operation that bypasses the journal entirely.
    let shift = remus_math::mat::Mat4::translation(1.0, 0.0, 0.0);
    remus_operations::transform::transform_solid(&mut topo, fused.solid, &shift).unwrap();

    // The next journaled operation must not read as continuous history.
    let c = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let pending = topo.journal_begin("test_barrier");
    record_barrier_over_solid(&mut topo, pending, c).unwrap();

    let barrier_kinds: Vec<&str> = topo
        .journal()
        .entries()
        .iter()
        .filter(|entry| entry.is_barrier())
        .map(remus_topology::journal::JournalEntry::kind)
        .collect();
    assert!(
        barrier_kinds.contains(&UNJOURNALED_MUTATIONS),
        "the unjournaled transform must sever continuity: {barrier_kinds:?}"
    );

    // The global barrier severs even entities the journal knew before it.
    let known = topo
        .journal()
        .ordinal_of(EntityKey::face(
            solid_faces(&topo, fused.solid).unwrap()[0].index(),
        ))
        .unwrap();
    assert!(
        !topo.journal().barriers_crossing(known).is_empty(),
        "no reference may resolve across an unjournaled gap"
    );
}

#[test]
fn explicit_barrier_covers_every_entity_of_the_solid() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let pending = topo.journal_begin("offset_solid");
    let op = record_barrier_over_solid(&mut topo, pending, solid).unwrap();

    let entry = topo.journal().entries().last().unwrap();
    assert_eq!(entry.op(), op);
    assert_eq!(entry.kind(), "offset_solid");
    let EntryPayload::Barrier { affected } = entry.payload() else {
        panic!("expected an explicit barrier");
    };
    // A 10-cube: 6 faces + 12 edges + 8 vertices.
    assert_eq!(affected.len(), 26);

    for id in solid_faces(&topo, solid).unwrap() {
        let ordinal = topo
            .journal()
            .ordinal_of(EntityKey::face(id.index()))
            .unwrap();
        assert_eq!(topo.journal().barriers_crossing(ordinal), vec![op]);
    }
}

#[test]
fn blend_face_evolution_journals_with_unresolved_claims_intact() {
    use remus_operations::blend_ops::fillet_with_evolution;

    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, cube).unwrap();

    let pending = begin_scoped(&mut topo, "fillet", &[cube]).unwrap();
    let (result, map) = fillet_with_evolution(&mut topo, cube, &[edges[0]], 1.0).unwrap();
    let op = record_face_evolution(&mut topo, pending, &map, &[result.solid]).unwrap();

    let entry = topo.journal().entries().last().unwrap();
    assert_eq!(entry.op(), op);
    let EntryPayload::Evolution { origin, events, .. } = entry.payload() else {
        panic!("face evolution must journal as an evolution entry");
    };
    assert_eq!(
        *origin,
        if map.origin.is_exact() {
            RecordedOrigin::Construction
        } else {
            RecordedOrigin::Geometry
        },
        "the entry's origin must mirror the map's provenance claim"
    );

    // Every map claim appears, one event per subject: outputs claimed by
    // several inputs (a band generated from both base faces, a merge)
    // group into one event naming all sources.
    let distinct =
        |m: &std::collections::HashMap<usize, Vec<usize>>| -> std::collections::BTreeSet<usize> {
            m.values().flatten().copied().collect()
        };
    let claimed = distinct(&map.modified).len()
        + distinct(&map.generated).len()
        + map.deleted.len()
        + map.unresolved.len();
    assert!(claimed > 0, "a fillet must claim something");
    assert_eq!(
        events.len(),
        claimed,
        "faces-only entry: exactly the map's claims, nothing invented"
    );
    assert!(
        solid_faces(&topo, result.solid).unwrap().len() > 6,
        "the fillet added its band"
    );

    // Edges are deliberately absent: a faces-only entry makes no edge
    // claims, so an edge reference does not resolve across this operation.
    let some_edge = EntityKey::edge(solid_edges(&topo, result.solid).unwrap()[0].index());
    let edge_is_subject = topo
        .journal()
        .ordinal_of(some_edge)
        .is_some_and(|ordinal| !topo.journal().events_for(ordinal).is_empty());
    assert!(
        !edge_is_subject,
        "absent claims are gaps, not implicit preservation"
    );
}

#[test]
fn merged_outputs_journal_as_one_merged_event() {
    use remus_operations::evolution::EvolutionMap;

    let mut topo = Topology::new();
    let mut map = EvolutionMap::exact();
    // Two coplanar input halves flowing into one output: the map records
    // one output under both inputs' modified lists.
    map.add_modified(3, 100);
    map.add_modified(5, 100);
    map.add_modified(7, 101);

    let pending = topo.journal_begin("unify_same_domain");
    record_face_evolution(&mut topo, pending, &map, &[]).unwrap();

    let journal = topo.journal();
    let merged_subject = journal.ordinal_of(EntityKey::face(100)).unwrap();
    let events = journal.events_for(merged_subject);
    let EntityEvent::Merged { from } = events[0].1 else {
        panic!("two inputs flowing into one output is a merge: {events:?}");
    };
    assert_eq!(
        from,
        &vec![
            journal.ordinal_of(EntityKey::face(3)).unwrap(),
            journal.ordinal_of(EntityKey::face(5)).unwrap(),
        ]
    );

    let plain_subject = journal.ordinal_of(EntityKey::face(101)).unwrap();
    assert!(matches!(
        journal.events_for(plain_subject)[0].1,
        EntityEvent::Modified { .. }
    ));
}

#[test]
fn transacted_rollback_truncates_journal_without_reusing_op_ids() {
    let mut topo = Topology::new();
    let (a, b) = two_overlapping_boxes(&mut topo);
    let first = boolean_journaled(&mut topo, BooleanOp::Fuse, a, b).unwrap();

    let snapshot = topo.clone();

    // A journaled operation whose transaction is rolled back.
    let c = make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
    let rolled_back = boolean_journaled(&mut topo, BooleanOp::Fuse, first.solid, c).unwrap();
    topo.restore_preserving_handle_slots(&snapshot);

    assert_eq!(
        topo.journal().entries().last().unwrap().op(),
        first.op,
        "entries after the checkpoint truncate with the restore"
    );

    // History recorded after the rollback continues cleanly — the model
    // and journal rolled back together, so nothing reads as an unjournaled
    // gap — and never reuses the rolled-back operation's id.
    let pending = topo.journal_begin("post_rollback_barrier");
    let next_op = record_barrier_over_solid(&mut topo, pending, first.solid).unwrap();
    assert!(
        next_op > rolled_back.op,
        "an OpId issued by a rolled-back operation must never be reissued"
    );
    assert!(
        topo.journal()
            .entries()
            .iter()
            .all(|entry| entry.kind() != UNJOURNALED_MUTATIONS),
        "a clean rollback must not read as an unjournaled gap"
    );
}
