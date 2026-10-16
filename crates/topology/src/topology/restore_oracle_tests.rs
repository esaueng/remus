//! Differential oracle for the handle-slot-preserving restore: the
//! in-place arena restores against the implementation they replaced, kept
//! verbatim (tests).

#![allow(clippy::unwrap_used)]

use super::*;
use crate::transaction::RollbackSnapshot;
use crate::transaction::savepoint_tests::{fixture, mutate};

/// `restore_preserving_handle_slots` with the arena restores it had before
/// they were rewritten in place, otherwise verbatim (the undo coordination
/// is shared).
fn legacy_restore_preserving_handle_slots(this: &mut Topology, snapshot: &Topology) {
    // Persistent preparation invalidates on every restore, even when the
    // restored state is bit-identical: the journal tick rolls back with
    // the state and could otherwise alias an earlier generation (ABA).
    // Lineage stays with the destination; only the generation moves
    // forward.
    this.bump_cache_generation();
    // A kept retirement (and the faces it orphans, re-promoted below),
    // or a snapshot whose index or face boundaries the rebuild below
    // re-derives, means the restore does not land on `snapshot`, so live
    // scopes cannot treat it as a rewind to the clone point. Checked
    // only once the rest of the truncation certificate holds.
    this.undo_truncate_for_foreign_restore(snapshot, |destination| {
        !destination.retires_live_entity_of(snapshot) && snapshot.is_restore_fixed_point()
    });
    this.vertices
        .legacy_restore_preserving_slots(&snapshot.vertices);
    this.edges.legacy_restore_preserving_slots(&snapshot.edges);
    this.wires.legacy_restore_preserving_slots(&snapshot.wires);
    this.faces.legacy_restore_preserving_slots(&snapshot.faces);
    this.shells
        .legacy_restore_preserving_slots(&snapshot.shells);
    this.solids
        .legacy_restore_preserving_slots(&snapshot.solids);
    this.compounds
        .legacy_restore_preserving_slots(&snapshot.compounds);
    this.compsolids
        .legacy_restore_preserving_slots(&snapshot.compsolids);
    this.loops.legacy_restore_preserving_slots(&snapshot.loops);
    this.coedges
        .legacy_restore_preserving_slots(&snapshot.coedges);
    this.attributes.clone_from(&snapshot.attributes);
    // Journal entries recorded after the snapshot are truncated with the
    // restore — the journal and the model roll back together, so the
    // history matches the state again — while `OpId`s and ordinals
    // issued by rolled-back operations are high-water preserved and
    // never reissued (the journal analogue of arena slot preservation).
    // The tick count rolls back with the state: the restored model is
    // exactly the one the restored journal describes.
    this.journal.restore_preserving_ids(&snapshot.journal);
    this.mutation_ticks = snapshot.mutation_ticks;
    let retired_faces = snapshot
        .faces
        .iter()
        .filter_map(|(id, _)| this.faces.get(id).is_none().then_some(id))
        .collect();
    let retired_solids = snapshot
        .solids
        .iter()
        .filter_map(|(id, _)| this.solids.get(id).is_none().then_some(id))
        .collect();
    this.attributes
        .remove_for_retired_entities(&retired_solids, &retired_faces);
    // Retirement is sticky here, so restored faces can reference
    // Loop/Coedge handles that stayed retired. Rebuild the compatibility
    // index from surviving authority (so the snapshot's index is never
    // copied), then promote affected live faces onto fresh handles while
    // carrying their snapshot pcurves.
    this.pcurves = PCurveRegistry::new();
    let mut indexed_uses = Vec::new();
    let mut missing_faces = Vec::new();
    for (face_id, face) in this.faces.iter() {
        let authority_is_live = !face.boundary_loops().is_empty()
            && face.boundary_loops().iter().all(|loop_id| {
                this.loops.get(*loop_id).is_some_and(|boundary| {
                    boundary
                        .coedges()
                        .iter()
                        .all(|coedge_id| this.coedges.get(*coedge_id).is_some())
                })
            });
        if !authority_is_live {
            missing_faces.push(face_id);
            continue;
        }
        for &loop_id in face.boundary_loops() {
            if let Some(boundary) = this.loops.get(loop_id) {
                for &coedge_id in boundary.coedges() {
                    if let Some(coedge) = this.coedges.get(coedge_id) {
                        indexed_uses.push((coedge.edge(), face_id, coedge.is_forward(), coedge_id));
                    }
                }
            }
        }
    }
    for (edge, face, forward, coedge_id) in indexed_uses {
        this.pcurves.index_use(edge, face, forward, coedge_id);
    }
    for face_id in missing_faces {
        let Ok(face) = this.face(face_id) else {
            continue;
        };
        let mut wire_ids = vec![face.outer_wire()];
        wire_ids.extend(face.inner_wires().iter().copied());
        let Ok(specs) = this.boundary_loop_specs(&wire_ids, None) else {
            continue;
        };
        let carried = snapshot.carried_face_authority(face_id);
        let _ = this.install_face_loop_specs_carrying(face_id, specs, carried, false);
    }
}

/// Restores `snapshot` into identical copies of `topo` through both paths
/// and requires identical results, tombstones included (`Debug` renders
/// them), then the same again after one more allocation. Returns whether
/// the restore re-derived a face boundary onto fresh handles, carrying the
/// snapshot's face authority.
fn assert_paths_agree(topo: &Topology, snapshot: &Topology, context: &str) -> bool {
    let mut current = topo.clone();
    let mut legacy = topo.clone();
    current.restore_preserving_handle_slots(snapshot);
    legacy_restore_preserving_handle_slots(&mut legacy, snapshot);
    assert_eq!(format!("{current:?}"), format!("{legacy:?}"), "{context}");
    assert_eq!(
        current.allocated_slot_count(),
        legacy.allocated_slot_count(),
        "{context}"
    );
    let promoted = current.loops.slot_len() > topo.loops.slot_len().max(snapshot.loops.slot_len());
    for t in [&mut current, &mut legacy] {
        t.add_empty_solid();
    }
    assert_eq!(format!("{current:?}"), format!("{legacy:?}"), "{context}");
    promoted
}

#[test]
fn in_place_restore_matches_the_verbatim_restore_at_every_fault_prefix() {
    // Each prefix of the savepoint corpus: allocations, in-place edits,
    // attribute and pcurve writes, loop re-derivation (retiring loops and
    // coedges), compounds, journal records and a solid deletion.
    let mut promoted = 0;
    for stop in 0..15 {
        let (original, face, solid, _) = fixture();
        let mut topo = original.clone();
        let snapshot = topo.clone();
        let _ = mutate(&mut topo, face, solid, stop);
        promoted += usize::from(assert_paths_agree(
            &topo,
            &snapshot,
            &format!("stop {stop}"),
        ));
        // Foreign snapshots: unrelated, smaller and larger than the target.
        let (unrelated, ..) = fixture();
        assert_paths_agree(&topo, &unrelated, &format!("foreign stop {stop}"));
        assert_paths_agree(&original, &topo, &format!("larger stop {stop}"));
    }
    // Every prefix from the first loop retirement (stage 8) on re-derives
    // the face, reading its carried authority from the snapshot.
    assert_eq!(promoted, 7);
}

#[test]
fn in_place_restore_matches_inside_a_scope_and_across_repeated_restores() {
    for stop in 0..15 {
        let (original, face, solid, _) = fixture();
        let mut current = original.clone();
        let mut legacy = original.clone();
        let current_scope = RollbackSnapshot::capture(&mut current);
        let legacy_scope = RollbackSnapshot::capture(&mut legacy);
        let (current_snapshot, legacy_snapshot) = (current.clone(), legacy.clone());
        // A drag: edit, restore, edit again from the restored state.
        for round in 0..3 {
            let _ = mutate(&mut current, face, solid, stop);
            let _ = mutate(&mut legacy, face, solid, stop);
            current.restore_preserving_handle_slots(&current_snapshot);
            legacy_restore_preserving_handle_slots(&mut legacy, &legacy_snapshot);
            assert_eq!(
                format!("{current:?}"),
                format!("{legacy:?}"),
                "stop {stop} round {round}"
            );
        }
        current_scope.restore(&mut current);
        legacy_scope.restore(&mut legacy);
        assert_eq!(format!("{current:?}"), format!("{legacy:?}"), "stop {stop}");
    }
}

#[test]
fn restore_fixed_points_are_exactly_the_values_a_self_restore_leaves_unchanged() {
    let self_restore_is_identity = |t: &Topology| {
        let mut restored = t.clone();
        restored.restore_preserving_handle_slots(t);
        format!("{restored:?}") == format!("{t:?}")
    };
    let (consistent, face, solid, _) = fixture();
    assert!(consistent.is_restore_fixed_point());
    assert!(self_restore_is_identity(&consistent));
    // The check agrees with the definition at every corpus prefix.
    for stop in 0..15 {
        let mut t = consistent.clone();
        let _ = mutate(&mut t, face, solid, stop);
        assert_eq!(
            t.is_restore_fixed_point(),
            self_restore_is_identity(&t),
            "stop {stop}"
        );
    }

    // A face whose loop is retired is re-derived by a restore.
    let mut dead_loop = consistent.clone();
    let loop_id = dead_loop.loops_of_face(face).unwrap()[0];
    assert!(dead_loop.loops.retire(loop_id));
    assert!(!dead_loop.is_restore_fixed_point());
    assert!(!self_restore_is_identity(&dead_loop));

    // An index entry no live boundary use backs is dropped by a restore.
    let mut stale_use = consistent;
    let coedge = stale_use.face_loop(loop_id).unwrap().coedges()[0];
    let (edge, forward) = {
        let used = stale_use.coedge(coedge).unwrap();
        (used.edge(), used.is_forward())
    };
    stale_use.pcurves.index_use(edge, face, !forward, coedge);
    assert!(!stale_use.is_restore_fixed_point());
    assert!(!self_restore_is_identity(&stale_use));
}
