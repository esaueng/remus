//! Integration tests for the evolution journal (RFC 0003, Stage 1).
//!
//! The stage-1 exit gate: every operation either journals real evolution
//! or an explicit barrier — and anything that bypasses the journal
//! entirely surfaces as a synthetic global barrier, so no operation is
//! silently absent from history.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::bop::BooleanOp;
use remus_operations::journal_ops::{
    begin_scoped, boolean_journaled, offset_journaled, record_barrier_over_solid,
    record_face_evolution,
};
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{
    EntityEvent, EntityKey, EntityKind, EntryPayload, RecordedOrigin, UNJOURNALED_MUTATIONS,
};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

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
fn journaled_offsets_carry_face_references_through_exact_evolution() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();

    let first = offset_journaled(&mut topo, source, 0.5).unwrap();
    assert!(first.map.origin.is_exact());
    assert_eq!(first.map.modified.len(), 6);
    let first_entry = topo.journal().entries().last().unwrap();
    let EntryPayload::Evolution { origin, events, .. } = first_entry.payload() else {
        panic!("an offset must journal evolution, not a barrier");
    };
    assert_eq!(*origin, RecordedOrigin::Construction);
    assert_eq!(events.len(), 6, "one exact event per source face");

    let reference = PersistentRef::operation_output(first.op, EntityKind::Face, 0);
    let Resolution::Bound {
        entity: first_face,
        provenance: Provenance::Construction,
    } = resolve(&topo, &reference)
    else {
        panic!("the first offset output must resolve exactly");
    };
    assert!(
        solid_faces(&topo, first.solid)
            .unwrap()
            .iter()
            .any(|face| face.index() == first_face.index)
    );

    let second = offset_journaled(&mut topo, first.solid, 0.5).unwrap();
    let Resolution::Bound {
        entity: second_face,
        provenance: Provenance::Construction,
    } = resolve(&topo, &reference)
    else {
        panic!("the face reference must follow the second offset exactly");
    };
    assert!(
        solid_faces(&topo, second.solid)
            .unwrap()
            .iter()
            .any(|face| face.index() == second_face.index)
    );
    assert_ne!(first_face, second_face);
}

#[test]
fn failed_journaled_offset_rolls_back_topology_and_history() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_loops(),
        topo.num_coedges(),
    );
    let journal_len = topo.journal().entries().len();

    let error = offset_journaled(&mut topo, source, -1.5).unwrap_err();
    assert!(error.to_string().contains("collapsed"), "{error}");
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_loops(),
            topo.num_coedges(),
        ),
        counts,
        "a failed offset must leave no live topology"
    );
    assert_eq!(topo.journal().entries().len(), journal_len);
    assert!(topo.solid(source).is_ok(), "the input handle remains live");
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

#[test]
fn successive_draft_preserves_all_original_entity_references() {
    use remus_math::mat::Mat4;
    use remus_math::vec::Point3;
    use remus_topology::journal::{EventDraft, EvolutionDraft};
    for scale in [1e-3_f64, 1.0, 1e3] {
        for rotation in [0.0, 0.7] {
            for bore in [false, true] {
                let mut topo = Topology::new();
                let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
                let source = if bore {
                    let tool =
                        remus_operations::primitives::make_cylinder(&mut topo, 1.0, 10.0).unwrap();
                    remus_operations::transform::transform_solid(
                        &mut topo,
                        tool,
                        &remus_math::mat::Mat4::translation(5.0, 5.0, 0.0),
                    )
                    .unwrap();
                    remus_operations::boolean::boolean(
                        &mut topo,
                        remus_operations::boolean::BooleanOp::Cut,
                        source,
                        tool,
                    )
                    .unwrap()
                } else {
                    source
                };

                let neutral = Point3::new(123.0 * scale, -57.0 * scale, 31.0 * scale);
                let frame = Mat4::rotation_x(rotation);
                let transform = Mat4::translation(neutral.x(), neutral.y(), neutral.z())
                    * frame
                    * Mat4::scale(scale, scale, scale);
                remus_operations::transform::transform_solid(&mut topo, source, &transform)
                    .unwrap();
                let pull = frame.mul_point(Point3::new(0.0, 0.0, 1.0)) - Point3::new(0.0, 0.0, 0.0);
                let source_volume =
                    remus_operations::measure::solid_volume(&topo, source, 0.01 * scale).unwrap();
                let keys = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
                let pending = topo.journal_begin("draft_fixture");
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
                let anchor = topo.journal_record_evolution(pending, draft).unwrap();
                let mut solid = source;
                for (step, angle) in [0.05, -0.02].into_iter().enumerate() {
                    let face = solid_faces(&topo, solid)
                        .unwrap()
                        .into_iter()
                        .find(|&face| {
                            topo.face(face)
                                .unwrap()
                                .effective_plane_normal()
                                .is_some_and(|normal| normal.x() > 0.9)
                        })
                        .unwrap();
                    let result = remus_operations::journal_ops::draft_journaled(
                        &mut topo,
                        solid,
                        &[face],
                        pull,
                        neutral,
                        angle,
                    )
                    .unwrap();
                    solid = result.solid;
                    if step == 0 {
                        let expected = source_volume + 500.0 * scale.powi(3) * angle.tan();
                        let actual =
                            remus_operations::measure::solid_volume(&topo, solid, 0.01 * scale)
                                .unwrap();
                        assert!(
                            (actual - expected).abs() <= expected.abs() * 1e-5,
                            "scale {scale}, rotation {rotation}, bore {bore}: {actual} != {expected}"
                        );
                    }
                    let unchanged =
                        remus_operations::measure::solid_volume(&topo, source, 0.01 * scale)
                            .unwrap();
                    assert!((unchanged - source_volume).abs() <= source_volume.abs() * 1e-12);

                    let live: std::collections::BTreeSet<_> =
                        remus_operations::journal_ops::solid_entity_keys(&topo, solid)
                            .unwrap()
                            .into_iter()
                            .collect();
                    let mut resolved = std::collections::BTreeSet::new();
                    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                            let reference = PersistentRef::operation_output(anchor, kind, index);
                            let outcome = resolve(&topo, &reference);
                            let Resolution::Bound {
                                entity,
                                provenance: Provenance::Construction,
                            } = outcome
                            else {
                                panic!("draft angle {angle}, {kind:?}/{index}: {outcome:?}");
                            };
                            assert!(resolved.insert(entity));
                        }
                    }
                    assert_eq!(resolved, live);
                    assert!(
                        remus_operations::validate::validate_solid(&topo, solid)
                            .unwrap()
                            .is_valid()
                    );
                }
            }
        }
    }
}

#[test]
fn journaled_draft_refusals_restore_topology_and_history() {
    use remus_math::vec::{Point3, Vec3};
    for angle in [0.0, -1.4, f64::NAN, f64::INFINITY] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let face = solid_faces(&topo, source)
            .unwrap()
            .into_iter()
            .find(|&face| {
                topo.face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.x() > 0.9)
            })
            .unwrap();
        let counts = |topo: &Topology| {
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
                topo.num_pcurves(),
            )
        };
        let before_counts = counts(&topo);
        let before_history = topo.journal().snapshot();
        let error = remus_operations::journal_ops::draft_journaled(
            &mut topo,
            source,
            &[face],
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            angle,
        )
        .expect_err("invalid or folding draft must refuse");
        assert!(!error.to_string().is_empty());
        assert_eq!(counts(&topo), before_counts, "angle {angle}");
        assert_eq!(topo.journal().snapshot(), before_history, "angle {angle}");
        assert!(
            (remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap() - 1000.0).abs()
                < 1e-6
        );
    }
}

#[test]
fn failed_draft_does_not_publish_a_preexisting_mutation_gap() {
    use remus_math::vec::{Point3, Vec3};
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let pending = topo.journal_begin("source_fixture");
    record_barrier_over_solid(&mut topo, pending, source).unwrap();
    let unrelated = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let face = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.9)
        })
        .unwrap();
    let before = topo.journal().snapshot();
    assert!(
        remus_operations::journal_ops::draft_journaled(
            &mut topo,
            source,
            &[face],
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            0.0
        )
        .is_err()
    );
    let after = topo.journal().snapshot();
    assert_eq!(after.entries, before.entries);
    assert_eq!(after.index, before.index);
    assert_eq!(after.next_ordinal, before.next_ordinal);
    assert!(after.next_op >= before.next_op);
    let result = remus_operations::journal_ops::draft_journaled(
        &mut topo,
        source,
        &[face],
        Vec3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        0.05,
    )
    .unwrap();
    let entries = topo.journal().entries();
    assert_eq!(entries.len(), before.entries.len() + 2);
    assert_eq!(entries[entries.len() - 2].kind(), UNJOURNALED_MUTATIONS);
    assert_eq!(entries.last().unwrap().op(), result.op);
    assert!(
        (remus_operations::measure::solid_volume(&topo, unrelated, 0.01).unwrap() - 24.0).abs()
            < 1e-6
    );
}

#[test]
fn capped_defeature_records_retained_and_consumed_boundaries() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        for rotation in [0.0, 0.7] {
            let mut topo = Topology::new();
            let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
            let tool = make_box(&mut topo, 4.0, 4.0, 20.0).unwrap();
            remus_operations::transform::transform_solid(
                &mut topo,
                tool,
                &remus_math::mat::Mat4::translation(3.0, 3.0, -5.0),
            )
            .unwrap();
            let source = remus_operations::boolean::boolean(
                &mut topo,
                remus_operations::boolean::BooleanOp::Cut,
                cube,
                tool,
            )
            .unwrap();
            let walls: Vec<_> = solid_faces(&topo, source)
                .unwrap()
                .into_iter()
                .filter(|&face| {
                    topo.wire(topo.face(face).unwrap().outer_wire())
                        .unwrap()
                        .edges()
                        .iter()
                        .all(|oe| {
                            let edge = topo.edge(oe.edge()).unwrap();
                            [edge.start(), edge.end()].iter().all(|&vertex| {
                                let p = topo.vertex(vertex).unwrap().point();
                                p.x() > 2.9 && p.x() < 7.1 && p.y() > 2.9 && p.y() < 7.1
                            })
                        })
                })
                .collect();
            assert_eq!(walls.len(), 4);
            let transform =
                remus_math::mat::Mat4::translation(123.0 * scale, -57.0 * scale, 31.0 * scale)
                    * remus_math::mat::Mat4::rotation_x(rotation)
                    * remus_math::mat::Mat4::scale(scale, scale, scale);
            remus_operations::transform::transform_solid(&mut topo, source, &transform).unwrap();

            let source_keys =
                remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
            let pending = begin_scoped(&mut topo, "hole_fixture", &[source]).unwrap();
            let mut anchor_draft = remus_topology::journal::EvolutionDraft::construction();
            for &key in &source_keys {
                anchor_draft.push(
                    key,
                    remus_topology::journal::EventDraft::Generated {
                        sources: Vec::new(),
                    },
                );
            }
            let anchor = topo
                .journal_record_evolution(pending, anchor_draft)
                .unwrap();
            let result =
                remus_operations::journal_ops::defeature_journaled(&mut topo, source, &walls)
                    .unwrap();
            let journal = topo.journal();
            let EntryPayload::Evolution { events, .. } =
                journal.entries().last().unwrap().payload()
            else {
                panic!("expected evolution")
            };
            for key in
                remus_operations::journal_ops::solid_entity_keys(&topo, result.solid).unwrap()
            {
                let ordinal = journal.ordinal_of(key).unwrap();
                assert!(
                    events.iter().any(|(subject, event)| *subject == ordinal
                        && matches!(event, EntityEvent::Modified { .. })),
                    "missing retained boundary {key:?}"
                );
            }
            let deleted_boundaries = source_keys
                .iter()
                .filter(|key| key.kind != EntityKind::Face)
                .filter(|key| {
                    let ordinal = journal.ordinal_of(**key).unwrap();
                    events.iter().any(|(subject, event)| {
                        *subject == ordinal && matches!(event, EntityEvent::Deleted)
                    })
                })
                .count();
            assert_eq!(deleted_boundaries, 20);
            let mut bound = std::collections::BTreeSet::new();
            let mut dangling = 0;
            for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                for index in 0..source_keys.iter().filter(|key| key.kind == kind).count() {
                    match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                        Resolution::Bound {
                            entity,
                            provenance: Provenance::Construction,
                        } => {
                            bound.insert(entity);
                        }
                        Resolution::Dangling { deleted_at } => {
                            assert_eq!(deleted_at, result.op);
                            dangling += 1;
                        }
                        other => panic!("unqualified capped history: {other:?}"),
                    }
                }
            }
            assert_eq!(dangling, 24);
            assert_eq!(
                bound,
                remus_operations::journal_ops::solid_entity_keys(&topo, result.solid)
                    .unwrap()
                    .into_iter()
                    .collect()
            );

            assert!(
                (remus_operations::measure::solid_volume(&topo, result.solid, 0.01 * scale)
                    .unwrap()
                    - 1000.0 * scale.powi(3))
                .abs()
                    < 1e-6 * scale.powi(3)
            );
            assert!(
                (remus_operations::measure::solid_volume(&topo, source, 0.01 * scale).unwrap()
                    - 840.0 * scale.powi(3))
                .abs()
                    < 1e-6 * scale.powi(3)
            );
        }
    }
}

#[test]
fn failed_defeature_restores_unpublished_gap_and_topology() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let pending = topo.journal_begin("source_fixture");
    record_barrier_over_solid(&mut topo, pending, source).unwrap();
    let unrelated = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let before = topo.journal().snapshot();
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let foreign = solid_faces(&topo, unrelated).unwrap();
    let all = solid_faces(&topo, source).unwrap();
    for selection in [Vec::new(), foreign, all] {
        assert!(
            remus_operations::journal_ops::defeature_journaled(&mut topo, source, &selection)
                .is_err()
        );
        let after = topo.journal().snapshot();
        assert_eq!(after.entries, before.entries);
        assert_eq!(after.index, before.index);
        assert_eq!(after.next_ordinal, before.next_ordinal);
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids()
            ),
            counts
        );
        assert!(
            (remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap() - 1000.0).abs()
                < 1e-6
        );
    }
}

#[test]
fn extended_defeature_records_reconstructed_boundaries() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        for rotation in [0.0, 0.7] {
            for fillet in [false, true] {
                let mut topo = Topology::new();
                let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
                let edge = solid_edges(&topo, cube)
                    .unwrap()
                    .into_iter()
                    .find(|&edge| {
                        let edge = topo.edge(edge).unwrap();
                        [edge.start(), edge.end()]
                            .iter()
                            .all(|&v| (topo.vertex(v).unwrap().point().z() - 10.0).abs() < 1e-9)
                    })
                    .unwrap();
                let source = if fillet {
                    remus_operations::blend_ops::fillet_v2(&mut topo, cube, &[edge], 2.0)
                        .unwrap()
                        .solid
                } else {
                    remus_operations::chamfer::chamfer(&mut topo, cube, &[edge], 2.0).unwrap()
                };
                let bevel: Vec<_> = solid_faces(&topo, source)
                    .unwrap()
                    .into_iter()
                    .filter(|&face| {
                        let Some(n) = topo.face(face).unwrap().effective_plane_normal() else {
                            return true;
                        };
                        [n.x(), n.y(), n.z()]
                            .iter()
                            .filter(|x| x.abs() > 1e-6)
                            .count()
                            > 1
                    })
                    .collect();
                assert_eq!(bevel.len(), 1);
                let transform =
                    remus_math::mat::Mat4::translation(123.0 * scale, -57.0 * scale, 31.0 * scale)
                        * remus_math::mat::Mat4::rotation_x(rotation)
                        * remus_math::mat::Mat4::scale(scale, scale, scale);
                remus_operations::transform::transform_solid(&mut topo, source, &transform)
                    .unwrap();

                let keys = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
                let pending = begin_scoped(&mut topo, "chamfer_fixture", &[source]).unwrap();
                let mut draft = remus_topology::journal::EvolutionDraft::construction();
                for &key in &keys {
                    draft.push(
                        key,
                        remus_topology::journal::EventDraft::Generated {
                            sources: Vec::new(),
                        },
                    );
                }
                let anchor = topo.journal_record_evolution(pending, draft).unwrap();
                let result =
                    remus_operations::journal_ops::defeature_journaled(&mut topo, source, &bevel)
                        .unwrap();
                let journal = topo.journal();
                let EntryPayload::Evolution { events, .. } =
                    journal.entries().last().unwrap().payload()
                else {
                    panic!("expected evolution")
                };
                for key in
                    remus_operations::journal_ops::solid_entity_keys(&topo, result.solid).unwrap()
                {
                    let ordinal = journal.ordinal_of(key).unwrap();
                    assert!(
                        events.iter().any(|(subject, event)| *subject == ordinal
                            && !matches!(event, EntityEvent::Unresolved { .. })),
                        "missing reconstructed boundary {key:?}"
                    );
                }
                let mut resolved = std::collections::BTreeSet::new();
                let mut deleted = 0;
                for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
                    for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                        match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index))
                        {
                            Resolution::Bound {
                                entity,
                                provenance: Provenance::Construction,
                            } => {
                                resolved.insert(entity);
                            }
                            Resolution::Dangling { deleted_at } => {
                                assert_eq!(deleted_at, result.op);
                                deleted += 1;
                            }
                            other => panic!("unqualified extension history {other:?}"),
                        }
                    }
                }
                assert_eq!(deleted, 3);
                assert_eq!(
                    resolved,
                    remus_operations::journal_ops::solid_entity_keys(&topo, result.solid)
                        .unwrap()
                        .into_iter()
                        .collect()
                );
                assert_eq!(
                    events
                        .iter()
                        .filter(|(_, event)| matches!(event, EntityEvent::Merged { .. }))
                        .count(),
                    3
                );
                let expected_source = if fillet {
                    1000.0 - 10.0 * (4.0 - std::f64::consts::PI)
                } else {
                    980.0
                };
                assert!(
                    (remus_operations::measure::solid_volume(&topo, result.solid, 0.01 * scale)
                        .unwrap()
                        - 1000.0 * scale.powi(3))
                    .abs()
                        < 1e-6 * scale.powi(3)
                );
                assert!(
                    (remus_operations::measure::solid_volume(&topo, source, 0.01 * scale).unwrap()
                        - expected_source * scale.powi(3))
                    .abs()
                        < 1e-5 * scale.powi(3)
                );
            }
        }
    }
}

#[test]
fn closed_rim_defeature_preserves_construction_references() {
    let mut topo = Topology::new();
    let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let rim = solid_edges(&topo, cylinder)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let edge = topo.edge(edge).unwrap();
            edge.is_closed() && (topo.vertex(edge.start()).unwrap().point().z() - 20.0).abs() < 1e-9
        })
        .unwrap();
    let source = remus_operations::blend_ops::fillet_v2(&mut topo, cylinder, &[rim], 2.0)
        .unwrap()
        .solid;
    let band: Vec<_> = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                remus_topology::face::FaceSurface::Torus(_)
            )
        })
        .collect();
    assert_eq!(band.len(), 1);
    let keys = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
    let pending = begin_scoped(&mut topo, "rim_fixture", &[source]).unwrap();
    let mut draft = remus_topology::journal::EvolutionDraft::construction();
    for &key in &keys {
        draft.push(
            key,
            remus_topology::journal::EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let anchor = topo.journal_record_evolution(pending, draft).unwrap();
    let result =
        remus_operations::journal_ops::defeature_journaled(&mut topo, source, &band).unwrap();
    let mut resolved = std::collections::BTreeSet::new();
    let mut deleted = 0;
    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
            match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                Resolution::Bound {
                    entity,
                    provenance: Provenance::Construction,
                } => {
                    resolved.insert(entity);
                }
                Resolution::Dangling { deleted_at } => {
                    assert_eq!(deleted_at, result.op);
                    deleted += 1;
                }
                other => panic!("unqualified rim history {other:?}"),
            }
        }
    }
    assert!(deleted >= 2);
    assert_eq!(
        resolved,
        remus_operations::journal_ops::solid_entity_keys(&topo, result.solid)
            .unwrap()
            .into_iter()
            .collect()
    );
    let expected = std::f64::consts::PI * 100.0 * 20.0;
    assert!(
        (remus_operations::measure::solid_volume(&topo, result.solid, 0.01).unwrap() - expected)
            .abs()
            < 1e-6
    );
}

#[test]
fn cylinder_cone_defeature_preserves_construction_references() {
    let mut topo = Topology::new();
    let cylinder = remus_operations::primitives::make_cylinder(&mut topo, 3.0, 5.0).unwrap();
    let cone = remus_operations::primitives::make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        cone,
        &remus_math::mat::Mat4::translation(0.0, 0.0, 5.0),
    )
    .unwrap();
    let sharp = remus_operations::boolean::boolean(
        &mut topo,
        remus_operations::boolean::BooleanOp::Fuse,
        cylinder,
        cone,
    )
    .unwrap();
    let adjacency = topo.build_adjacency(sharp).unwrap();
    let shoulder = solid_edges(&topo, sharp)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let faces = adjacency.faces_for_edge(edge);
            faces.len() == 2
                && matches!(
                    (
                        topo.face(faces[0]).unwrap().surface(),
                        topo.face(faces[1]).unwrap().surface()
                    ),
                    (
                        remus_topology::face::FaceSurface::Cylinder(_),
                        remus_topology::face::FaceSurface::Cone(_)
                    ) | (
                        remus_topology::face::FaceSurface::Cone(_),
                        remus_topology::face::FaceSurface::Cylinder(_)
                    )
                )
        })
        .unwrap();
    let source = remus_operations::blend_ops::fillet_v2(&mut topo, sharp, &[shoulder], 0.25)
        .unwrap()
        .solid;
    let band: Vec<_> = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                remus_topology::face::FaceSurface::Torus(_)
            )
        })
        .collect();
    assert_eq!(band.len(), 1);
    let keys = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
    let pending = begin_scoped(&mut topo, "rim_fixture", &[source]).unwrap();
    let mut draft = remus_topology::journal::EvolutionDraft::construction();
    for &key in &keys {
        draft.push(
            key,
            remus_topology::journal::EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let anchor = topo.journal_record_evolution(pending, draft).unwrap();
    let result =
        remus_operations::journal_ops::defeature_journaled(&mut topo, source, &band).unwrap();
    let mut resolved = std::collections::BTreeSet::new();
    let mut deleted = 0;
    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
            match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                Resolution::Bound {
                    entity,
                    provenance: Provenance::Construction,
                } => {
                    resolved.insert(entity);
                }
                Resolution::Dangling { deleted_at } => {
                    assert_eq!(deleted_at, result.op);
                    deleted += 1;
                }
                other => panic!("unqualified rim history {other:?}"),
            }
        }
    }
    assert!(deleted >= 2);
    assert_eq!(
        resolved,
        remus_operations::journal_ops::solid_entity_keys(&topo, result.solid)
            .unwrap()
            .into_iter()
            .collect()
    );
    let expected = std::f64::consts::PI * 187.0 / 3.0;
    assert!(
        (remus_operations::measure::solid_volume(&topo, result.solid, 0.01).unwrap() - expected)
            .abs()
            < 1e-6
    );
}

#[test]
fn verified_healing_preserves_in_place_references_through_vertex_merge() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, source).unwrap()[0];
    let original = topo.edge(edge).unwrap().start();
    let point = topo.vertex(original).unwrap().point();
    let duplicate = topo.add_vertex(remus_topology::vertex::Vertex::new(point, 1e-7));
    topo.edge_mut(edge).unwrap().set_start(duplicate);
    let keys = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
    let pending = begin_scoped(&mut topo, "damaged_fixture", &[source]).unwrap();
    let mut draft = remus_topology::journal::EvolutionDraft::construction();
    for &key in &keys {
        draft.push(
            key,
            remus_topology::journal::EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let anchor = topo.journal_record_evolution(pending, draft).unwrap();
    let first = remus_operations::journal_ops::fix_shape_journaled(
        &mut topo,
        source,
        &remus_heal::fix::FixConfig::default(),
        None,
    )
    .unwrap();
    assert_eq!(first.result.solid, source);
    assert!(first.result.fixing.actions_taken > 0);
    let second = remus_operations::journal_ops::fix_shape_journaled(
        &mut topo,
        source,
        &remus_heal::fix::FixConfig::default(),
        None,
    )
    .unwrap();
    assert_ne!(first.op, second.op);
    let mut resolved = std::collections::BTreeSet::new();
    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
            match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                Resolution::Bound {
                    entity,
                    provenance: Provenance::Construction,
                } => {
                    resolved.insert(entity);
                }
                other => panic!("healing lost {kind:?}/{index}: {other:?}"),
            }
        }
    }
    assert_eq!(
        resolved,
        remus_operations::journal_ops::solid_entity_keys(&topo, source)
            .unwrap()
            .into_iter()
            .collect()
    );
    assert_eq!(solid_vertices(&topo, source).unwrap().len(), 8);
    assert!(
        (remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap() - 1000.0).abs()
            < 1e-6
    );
}

#[test]
fn verified_pipeline_preserves_references_across_private_fix_contexts() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, source).unwrap()[0];
    let original = topo.edge(edge).unwrap().start();
    let point = topo.vertex(original).unwrap().point();
    let duplicate = topo.add_vertex(remus_topology::vertex::Vertex::new(point, 1e-7));
    topo.edge_mut(edge).unwrap().set_start(duplicate);
    let keys = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
    let pending = begin_scoped(&mut topo, "damaged_fixture", &[source]).unwrap();
    let mut draft = remus_topology::journal::EvolutionDraft::construction();
    for &key in &keys {
        draft.push(
            key,
            remus_topology::journal::EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let anchor = topo.journal_record_evolution(pending, draft).unwrap();
    let mut process = remus_heal::pipeline::process::HealProcess::new();
    process.add_step("fix_shape");
    process.add_step("merge_vertices");
    process.add_step("fix_shape");
    let first = remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, source, &process)
        .unwrap();
    assert_eq!(first.result.solid, source);
    let second =
        remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, source, &process)
            .unwrap();
    assert_ne!(first.op, second.op);
    let mut resolved = std::collections::BTreeSet::new();
    for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
        for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
            match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                Resolution::Bound {
                    entity,
                    provenance: Provenance::Construction,
                } => {
                    resolved.insert(entity);
                }
                other => panic!("healing lost {kind:?}/{index}: {other:?}"),
            }
        }
    }
    assert_eq!(
        resolved,
        remus_operations::journal_ops::solid_entity_keys(&topo, source)
            .unwrap()
            .into_iter()
            .collect()
    );
    assert_eq!(solid_vertices(&topo, source).unwrap().len(), 8);
    assert!(
        (remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap() - 1000.0).abs()
            < 1e-6
    );
}

#[test]
fn verified_pipeline_rolls_back_a_repair_before_a_later_step_failure() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(&topo, source).unwrap()[0];
    let original = topo.edge(edge).unwrap().start();
    let point = topo.vertex(original).unwrap().point();
    let duplicate = topo.add_vertex(remus_topology::vertex::Vertex::new(point, 1e-7));
    topo.edge_mut(edge).unwrap().set_start(duplicate);
    let before = remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap();
    let history = topo.journal().snapshot();
    let mut process = remus_heal::pipeline::process::HealProcess::new();
    process.add_step("fix_shape");
    process.add_step("missing_operator");
    assert!(
        remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, source, &process)
            .is_err()
    );
    assert_eq!(topo.edge(edge).unwrap().start(), duplicate);
    assert_eq!(
        remus_operations::journal_ops::solid_entity_keys(&topo, source).unwrap(),
        before
    );
    assert_eq!(topo.journal().snapshot(), history);
}

#[test]
fn ordinary_verified_pipeline_does_not_invoke_custom_history_override() {
    #[derive(Debug)]
    struct HistoryRefusal;
    impl remus_heal::pipeline::operator::HealOperator for HistoryRefusal {
        fn name(&self) -> &'static str {
            "history_refusal"
        }
        fn execute(
            &self,
            _topo: &mut Topology,
            solid: remus_topology::SolidId,
            _ctx: &mut remus_heal::context::HealContext,
        ) -> Result<(remus_topology::SolidId, remus_heal::fix::FixResult), remus_heal::HealError>
        {
            Ok((solid, remus_heal::fix::FixResult::ok()))
        }
        fn execute_with_history(
            &self,
            _topo: &mut Topology,
            _solid: remus_topology::SolidId,
            _ctx: &mut remus_heal::context::HealContext,
        ) -> Result<
            (
                remus_topology::SolidId,
                remus_heal::fix::FixResult,
                remus_heal::reshape::ReShape,
            ),
            remus_heal::HealError,
        > {
            Err(remus_heal::HealError::FixFailed(
                "custom history refused".into(),
            ))
        }
    }
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let mut process = remus_heal::pipeline::process::HealProcess::new();
    process
        .registry_mut()
        .register("history_refusal", Box::new(HistoryRefusal));
    process.add_step("history_refusal");
    let report =
        remus_operations::heal::run_heal_pipeline_verified(&mut topo, solid, &process).unwrap();
    assert!(report.is_valid_after());
    let before = topo.journal().snapshot();
    let error = remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &process)
        .err()
        .unwrap();
    assert!(error.to_string().contains("custom history refused"));
    assert_eq!(topo.journal().snapshot(), before);
}

#[test]
#[allow(clippy::too_many_lines)]
fn verified_unification_journals_merged_faces_and_consumed_center() {
    use remus_topology::{
        edge::{Edge, EdgeCurve},
        face::Face,
        shell::Shell,
        vertex::Vertex,
        wire::{OrientedEdge, Wire},
    };
    fn line(
        topo: &mut Topology,
        a: remus_topology::VertexId,
        b: remus_topology::VertexId,
    ) -> remus_topology::EdgeId {
        topo.add_edge(Edge::new(a, b, EdgeCurve::Line))
    }
    for scale in [0.001, 1.0, 10.0] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
        let shell = topo.solid(solid).unwrap().outer_shell();
        let faces = topo.shell(shell).unwrap().faces().to_vec();
        let original = topo.face(faces[0]).unwrap().clone();
        let boundary = topo.wire(original.outer_wire()).unwrap().edges().to_vec();
        let vertices: Vec<_> = boundary
            .iter()
            .map(|oe| oe.oriented_start(topo.edge(oe.edge()).unwrap()))
            .collect();
        let a = topo.vertex(vertices[0]).unwrap().point();
        let b = topo.vertex(vertices[2]).unwrap().point();
        let center = topo.add_vertex(Vertex::new(a + (b - a) * 0.5, 1e-7));
        let spokes: Vec<_> = vertices
            .iter()
            .map(|&vertex| line(&mut topo, center, vertex))
            .collect();
        let mut triangles = Vec::new();
        for i in 0..4 {
            let wire = topo.add_wire(
                Wire::new(
                    vec![
                        boundary[i],
                        OrientedEdge::new(spokes[(i + 1) % 4], false),
                        OrientedEdge::new(spokes[i], true),
                    ],
                    true,
                )
                .unwrap(),
            );
            let mut face = Face::new(wire, Vec::new(), original.surface().clone());
            face.set_reversed(original.is_reversed());
            triangles.push(topo.add_face(face));
        }
        let mut replacement = faces[1..].to_vec();
        replacement.extend(triangles.iter().copied());
        *topo.shell_mut(shell).unwrap() = Shell::new(replacement).unwrap();

        let keys = remus_operations::journal_ops::solid_entity_keys(&topo, solid).unwrap();
        let pending = begin_scoped(&mut topo, "triangulated_fixture", &[solid]).unwrap();
        let mut draft = remus_topology::journal::EvolutionDraft::construction();
        for &key in &keys {
            draft.push(
                key,
                remus_topology::journal::EventDraft::Generated {
                    sources: Vec::new(),
                },
            );
        }
        let anchor = topo.journal_record_evolution(pending, draft).unwrap();
        let mut process = remus_heal::pipeline::process::HealProcess::new();
        process.add_step("unify_same_domain");
        let result =
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &process)
                .unwrap();
        assert!(result.result.is_valid_after());
        assert_eq!(solid_faces(&topo, result.result.solid).unwrap().len(), 6);
        let mut bound = std::collections::BTreeSet::new();
        let mut deleted = 0;
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                    Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } => {
                        bound.insert(entity);
                    }
                    Resolution::Dangling { deleted_at } => {
                        assert_eq!(deleted_at, result.op);
                        deleted += 1;
                    }
                    other => panic!("unification lost {kind:?}/{index}: {other:?}"),
                }
            }
        }
        assert_eq!(deleted, 5);
        assert_eq!(
            bound,
            remus_operations::journal_ops::solid_entity_keys(&topo, result.result.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        let volume =
            remus_operations::measure::solid_volume(&topo, result.result.solid, 0.01 * scale)
                .unwrap();
        assert!((volume - 1000.0 * scale.powi(3)).abs() < 1e-6 * scale.powi(3));
        let bytes = remus_io::arena_io::serialize_solids(&topo, &[result.result.solid]).unwrap();
        let mut restored = Topology::new();
        make_box(&mut restored, 1.0, 1.0, 1.0).unwrap();
        let restored_solid =
            remus_io::arena_io::deserialize_solids(&bytes, &mut restored).unwrap()[0];
        let repeat = remus_operations::journal_ops::heal_pipeline_journaled(
            &mut restored,
            restored_solid,
            &process,
        )
        .unwrap();
        let wall = solid_faces(&restored, repeat.result.solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                restored
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.x() > 0.9)
            })
            .unwrap();
        let drafted = remus_operations::journal_ops::draft_journaled(
            &mut restored,
            repeat.result.solid,
            &[wall],
            remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
            remus_math::vec::Point3::new(0.0, 0.0, 0.0),
            0.05,
        )
        .unwrap();
        let mut surviving = std::collections::BTreeSet::new();
        let mut still_deleted = 0;
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                match resolve(
                    &restored,
                    &PersistentRef::operation_output(anchor, kind, index),
                ) {
                    Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } => {
                        surviving.insert(entity);
                    }
                    Resolution::Dangling { deleted_at } => {
                        assert_eq!(deleted_at, result.op);
                        still_deleted += 1;
                    }
                    other => panic!("unify/arena/draft lost {kind:?}/{index}: {other:?}"),
                }
            }
        }
        assert_eq!(still_deleted, 5);
        assert_eq!(
            surviving,
            remus_operations::journal_ops::solid_entity_keys(&restored, drafted.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        let expected = (1000.0 + 500.0 * 0.05_f64.tan()) * scale.powi(3);
        let actual =
            remus_operations::measure::solid_volume(&restored, drafted.solid, 0.01 * scale)
                .unwrap();
        assert!((actual - expected).abs() < expected * 1e-6);
    }
}

#[test]
#[allow(clippy::too_many_lines)]
fn verified_sewing_preserves_all_references_through_arena_and_later_draft() {
    use remus_topology::wire::{OrientedEdge, Wire};
    #[derive(Debug)]
    struct RefuseAfterSewing;
    impl remus_heal::pipeline::operator::HealOperator for RefuseAfterSewing {
        fn name(&self) -> &'static str {
            "refuse_after_sewing"
        }
        fn execute(
            &self,
            topo: &mut Topology,
            solid: remus_topology::SolidId,
            _ctx: &mut remus_heal::context::HealContext,
        ) -> Result<(remus_topology::SolidId, remus_heal::fix::FixResult), remus_heal::HealError>
        {
            assert_eq!(
                remus_topology::explorer::solid_edges(topo, solid)
                    .unwrap()
                    .len(),
                12
            );
            Err(remus_heal::HealError::FixFailed(
                "refused after sewing".into(),
            ))
        }
    }

    for scale in [0.001, 1.0, 10.0] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
        for face in solid_faces(&topo, solid).unwrap() {
            let wire = topo.face(face).unwrap().outer_wire();
            let boundary = topo.wire(wire).unwrap().edges().to_vec();
            let mut vertices = std::collections::HashMap::new();
            let mut uses = Vec::new();
            for oe in boundary {
                let mut edge = topo.edge(oe.edge()).unwrap().clone();
                for source in [edge.start(), edge.end()] {
                    vertices.entry(source).or_insert_with(|| {
                        let vertex = topo.vertex(source).unwrap().clone();
                        topo.add_vertex(vertex)
                    });
                }
                edge.set_start(vertices[&edge.start()]);
                edge.set_end(vertices[&edge.end()]);
                uses.push(OrientedEdge::new(topo.add_edge(edge), oe.is_forward()));
            }
            let wire = topo.add_wire(Wire::new(uses, true).unwrap());
            topo.set_face_boundary_wires(face, wire, Vec::new())
                .unwrap();
        }
        let keys = remus_operations::journal_ops::solid_entity_keys(&topo, solid).unwrap();
        let pending = begin_scoped(&mut topo, "disjoint_face_fixture", &[solid]).unwrap();
        let mut draft = remus_topology::journal::EvolutionDraft::construction();
        for &key in &keys {
            draft.push(
                key,
                remus_topology::journal::EventDraft::Generated {
                    sources: Vec::new(),
                },
            );
        }
        let anchor = topo.journal_record_evolution(pending, draft).unwrap();
        let mut failing = remus_heal::pipeline::process::HealProcess::new();
        failing
            .registry_mut()
            .register("refuse_after_sewing", Box::new(RefuseAfterSewing));
        failing.add_step("sew_shells");
        failing.add_step("refuse_after_sewing");
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let journal_before = topo.journal().snapshot();
        let error =
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &failing)
                .err()
                .unwrap();
        assert!(error.to_string().contains("refused after sewing"));
        assert_eq!(topo.journal().snapshot(), journal_before);
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
        let mut process = remus_heal::pipeline::process::HealProcess::new();
        process.add_step("sew_shells");
        let result =
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &process)
                .unwrap();
        assert!(result.result.is_valid_after());
        assert_eq!(solid_faces(&topo, result.result.solid).unwrap().len(), 6);
        let mut bound = std::collections::BTreeSet::new();
        let mut deleted = 0;
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                    Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } => {
                        bound.insert(entity);
                    }
                    Resolution::Dangling { deleted_at } => {
                        assert_eq!(deleted_at, result.op);
                        deleted += 1;
                    }
                    other => panic!("sewing lost {kind:?}/{index}: {other:?}"),
                }
            }
        }
        assert_eq!(deleted, 0);
        assert_eq!(
            bound,
            remus_operations::journal_ops::solid_entity_keys(&topo, result.result.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        let volume =
            remus_operations::measure::solid_volume(&topo, result.result.solid, 0.01 * scale)
                .unwrap();
        assert!((volume - 1000.0 * scale.powi(3)).abs() < 1e-6 * scale.powi(3));
        let bytes = remus_io::arena_io::serialize_solids(&topo, &[result.result.solid]).unwrap();
        let mut restored = Topology::new();
        make_box(&mut restored, 1.0, 1.0, 1.0).unwrap();
        let restored_solid =
            remus_io::arena_io::deserialize_solids(&bytes, &mut restored).unwrap()[0];
        let repeat = remus_operations::journal_ops::heal_pipeline_journaled(
            &mut restored,
            restored_solid,
            &process,
        )
        .unwrap();
        let wall = solid_faces(&restored, repeat.result.solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                restored
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.x() > 0.9)
            })
            .unwrap();
        let drafted = remus_operations::journal_ops::draft_journaled(
            &mut restored,
            repeat.result.solid,
            &[wall],
            remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
            remus_math::vec::Point3::new(0.0, 0.0, 0.0),
            0.05,
        )
        .unwrap();
        let mut surviving = std::collections::BTreeSet::new();
        let mut still_deleted = 0;
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                match resolve(
                    &restored,
                    &PersistentRef::operation_output(anchor, kind, index),
                ) {
                    Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } => {
                        surviving.insert(entity);
                    }
                    Resolution::Dangling { deleted_at } => {
                        assert_eq!(deleted_at, result.op);
                        still_deleted += 1;
                    }
                    other => panic!("sew/arena/draft lost {kind:?}/{index}: {other:?}"),
                }
            }
        }
        assert_eq!(still_deleted, 0);
        assert_eq!(
            surviving,
            remus_operations::journal_ops::solid_entity_keys(&restored, drafted.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        let expected = (1000.0 + 500.0 * 0.05_f64.tan()) * scale.powi(3);
        let actual =
            remus_operations::measure::solid_volume(&restored, drafted.solid, 0.01 * scale)
                .unwrap();
        assert!((actual - expected).abs() < expected * 1e-6);
    }
}

#[test]
#[allow(clippy::too_many_lines)]
fn verified_inner_wire_removal_deletes_consumed_references_and_preserves_survivors() {
    use remus_topology::{
        edge::{Edge, EdgeCurve},
        vertex::Vertex,
        wire::{OrientedEdge, Wire},
    };
    for scale in [0.001, 1.0, 10.0] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
        let face = solid_faces(&topo, solid).unwrap()[0];
        let outer = topo.face(face).unwrap().outer_wire();
        let points: Vec<_> = topo
            .wire(outer)
            .unwrap()
            .edges()
            .iter()
            .map(|oe| {
                topo.vertex(oe.oriented_start(topo.edge(oe.edge()).unwrap()))
                    .unwrap()
                    .point()
            })
            .collect();
        let vertices = [(0.25, 0.25), (0.25, 0.75), (0.75, 0.75), (0.75, 0.25)].map(|(u, v)| {
            let point = points[0] + (points[1] - points[0]) * u + (points[3] - points[0]) * v;
            topo.add_vertex(Vertex::new(point, 1e-7))
        });
        let edges: Vec<_> = (0..4)
            .map(|i| {
                let edge = topo.add_edge(Edge::new(
                    vertices[i],
                    vertices[(i + 1) % 4],
                    EdgeCurve::Line,
                ));
                OrientedEdge::new(edge, true)
            })
            .collect();
        let hole = topo.add_wire(Wire::new(edges, true).unwrap());
        topo.set_face_boundary_wires(face, outer, vec![hole])
            .unwrap();
        let keys = remus_operations::journal_ops::solid_entity_keys(&topo, solid).unwrap();
        let pending = begin_scoped(&mut topo, "open_hole_fixture", &[solid]).unwrap();
        let mut draft = remus_topology::journal::EvolutionDraft::construction();
        for &key in &keys {
            draft.push(
                key,
                remus_topology::journal::EventDraft::Generated {
                    sources: Vec::new(),
                },
            );
        }
        let anchor = topo.journal_record_evolution(pending, draft).unwrap();
        let mut failing = remus_heal::pipeline::process::HealProcess::new();
        failing.add_step("remove_internal_wires");
        failing.add_step("missing_after_removal");
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let journal_before = topo.journal().snapshot();
        assert!(
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &failing)
                .is_err()
        );
        assert_eq!(topo.journal().snapshot(), journal_before);
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
        let mut process = remus_heal::pipeline::process::HealProcess::new();
        process.add_step("remove_internal_wires");
        let result =
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &process)
                .unwrap();
        assert!(result.result.is_valid_after());
        assert_eq!(solid_faces(&topo, result.result.solid).unwrap().len(), 6);
        let mut bound = std::collections::BTreeSet::new();
        let mut deleted = 0;
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                match resolve(&topo, &PersistentRef::operation_output(anchor, kind, index)) {
                    Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } => {
                        bound.insert(entity);
                    }
                    Resolution::Dangling { deleted_at } => {
                        assert_eq!(deleted_at, result.op);
                        deleted += 1;
                    }
                    other => panic!("inner-wire removal lost {kind:?}/{index}: {other:?}"),
                }
            }
        }
        assert_eq!(deleted, 8);
        assert_eq!(
            bound,
            remus_operations::journal_ops::solid_entity_keys(&topo, result.result.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        let volume =
            remus_operations::measure::solid_volume(&topo, result.result.solid, 0.01 * scale)
                .unwrap();
        assert!((volume - 1000.0 * scale.powi(3)).abs() < 1e-6 * scale.powi(3));
        let bytes = remus_io::arena_io::serialize_solids(&topo, &[result.result.solid]).unwrap();
        let mut restored = Topology::new();
        make_box(&mut restored, 1.0, 1.0, 1.0).unwrap();
        let restored_solid =
            remus_io::arena_io::deserialize_solids(&bytes, &mut restored).unwrap()[0];
        let repeat = remus_operations::journal_ops::heal_pipeline_journaled(
            &mut restored,
            restored_solid,
            &process,
        )
        .unwrap();
        let wall = solid_faces(&restored, repeat.result.solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                restored
                    .face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|normal| normal.x() > 0.9)
            })
            .unwrap();
        let drafted = remus_operations::journal_ops::draft_journaled(
            &mut restored,
            repeat.result.solid,
            &[wall],
            remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
            remus_math::vec::Point3::new(0.0, 0.0, 0.0),
            0.05,
        )
        .unwrap();
        let mut surviving = std::collections::BTreeSet::new();
        let mut still_deleted = 0;
        for kind in [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex] {
            for index in 0..keys.iter().filter(|key| key.kind == kind).count() {
                match resolve(
                    &restored,
                    &PersistentRef::operation_output(anchor, kind, index),
                ) {
                    Resolution::Bound {
                        entity,
                        provenance: Provenance::Construction,
                    } => {
                        surviving.insert(entity);
                    }
                    Resolution::Dangling { deleted_at } => {
                        assert_eq!(deleted_at, result.op);
                        still_deleted += 1;
                    }
                    other => panic!("inner-wire/arena/draft lost {kind:?}/{index}: {other:?}"),
                }
            }
        }
        assert_eq!(still_deleted, 8);
        assert_eq!(
            surviving,
            remus_operations::journal_ops::solid_entity_keys(&restored, drafted.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        let expected = (1000.0 + 500.0 * 0.05_f64.tan()) * scale.powi(3);
        let actual =
            remus_operations::measure::solid_volume(&restored, drafted.solid, 0.01 * scale)
                .unwrap();
        assert!((actual - expected).abs() < expected * 1e-6);
    }
}

#[test]
fn common_vertex_split_refuses_open_fans_and_rolls_back_completed_splits() {
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::{
        edge::{Edge, EdgeCurve},
        face::{Face, FaceSurface},
        shell::Shell,
        solid::Solid,
        vertex::Vertex,
        wire::{OrientedEdge, Wire},
    };
    #[derive(Debug)]
    struct RefuseAfterSplit;
    impl remus_heal::pipeline::operator::HealOperator for RefuseAfterSplit {
        fn name(&self) -> &'static str {
            "refuse_after_split"
        }
        fn execute(
            &self,
            topo: &mut Topology,
            solid: remus_topology::SolidId,
            _ctx: &mut remus_heal::context::HealContext,
        ) -> Result<(remus_topology::SolidId, remus_heal::fix::FixResult), remus_heal::HealError>
        {
            assert_eq!(solid_vertices(topo, solid).unwrap().len(), 33);
            Err(remus_heal::HealError::FixFailed(
                "refused after completed vertex split".into(),
            ))
        }
    }
    for scale in [0.001, 1.0, 10.0] {
        let mut topo = Topology::new();
        let source = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let mut faces = Vec::new();
        for i in 0..11 {
            let x = f64::from(i + 1) * scale;
            let a = topo.add_vertex(Vertex::new(Point3::new(x, 0.0, 0.0), 1e-7));
            let b = topo.add_vertex(Vertex::new(Point3::new(x, scale, 0.0), 1e-7));
            let mut uses = Vec::new();
            for (start, end) in [(source, a), (a, b), (b, source)] {
                let edge = topo.add_edge(Edge::new(start, end, EdgeCurve::Line));
                uses.push(OrientedEdge::new(edge, true));
            }
            let wire = topo.add_wire(Wire::new(uses, true).unwrap());
            faces.push(topo.add_face(Face::new(
                wire,
                vec![],
                FaceSurface::Plane {
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    d: 0.0,
                },
            )));
        }
        let shell = topo.add_shell(Shell::new(faces).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let keys = remus_operations::journal_ops::solid_entity_keys(&topo, solid).unwrap();
        let pending = begin_scoped(&mut topo, "open_vertex_fans_fixture", &[solid]).unwrap();
        let mut draft = remus_topology::journal::EvolutionDraft::construction();
        for &key in &keys {
            draft.push(
                key,
                remus_topology::journal::EventDraft::Generated {
                    sources: Vec::new(),
                },
            );
        }
        topo.journal_record_evolution(pending, draft).unwrap();
        let before = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
        let journal_before = topo.journal().snapshot();
        let mut process = remus_heal::pipeline::process::HealProcess::new();
        process.add_step("split_common_vertex");
        let mut probe = topo.clone();
        let (_, reports, history) = process.execute_with_history(&mut probe, solid).unwrap();
        assert_eq!(reports[0].actions_taken, 10);
        assert_eq!(
            history[0].replacements.entity_history().unwrap()[&EntityKey::vertex(source.index())]
                .len(),
            11
        );
        let error =
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &process)
                .err()
                .unwrap();
        assert!(
            error
                .to_string()
                .contains("configured healing result refused"),
            "{error}"
        );
        assert_eq!(topo.journal().snapshot(), journal_before);
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
        process
            .registry_mut()
            .register("refuse_after_split", Box::new(RefuseAfterSplit));
        process.add_step("refuse_after_split");
        let error =
            remus_operations::journal_ops::heal_pipeline_journaled(&mut topo, solid, &process)
                .err()
                .unwrap();
        assert!(
            error
                .to_string()
                .contains("refused after completed vertex split"),
            "{error}"
        );
        assert_eq!(topo.journal().snapshot(), journal_before);
        assert_eq!(
            remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
            before
        );
    }
}
