//! Nested savepoint state, failure injection, and stale-handle contract tests.

#![allow(clippy::unwrap_used)]
use super::tests::{seed, triangle_face};
use super::*;
use crate::TopologyError;
use crate::attributes::EntityAttributes;
use crate::journal::{EntityKey, EventDraft, EvolutionDraft};
use crate::naming::{PersistentRef, resolve};
use crate::pcurve::PCurve;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::Point3;
use remus_math::vec::{Point2, Vec2};

fn run<T>(
    legacy: bool,
    topo: &mut Topology,
    op: impl FnOnce(&mut Topology) -> Result<T, TopologyError>,
) -> Result<T, TopologyError> {
    if legacy {
        let snapshot = topo.clone();
        let result = op(topo);
        if result.is_err() {
            topo.restore_for_rollback(&snapshot);
        }
        result
    } else {
        run_transacted(topo, op)
    }
}

fn state(t: &Topology) -> String {
    let mut rows = Vec::new();
    macro_rules! arena {
        ($method:ident) => {
            rows.push(format!("{:?}", t.$method().iter().collect::<Vec<_>>()));
        };
    }
    arena!(vertices);
    arena!(edges);
    arena!(wires);
    arena!(faces);
    arena!(shells);
    arena!(solids);
    arena!(compounds);
    arena!(compsolids);
    for (id, _) in t.solids().iter() {
        rows.push(format!("{:?}", t.attributes().solid(id)));
    }
    for (id, _) in t.faces().iter() {
        rows.push(format!("{:?}", t.attributes().face(id)));
        for &lid in t.loops_of_face(id).unwrap() {
            let l = t.face_loop(lid).unwrap();
            rows.push(format!("{lid:?} {l:?}"));
            for &cid in l.coedges() {
                let c = t.coedge(cid).unwrap();
                rows.push(format!(
                    "{cid:?} {c:?} {:?}",
                    t.pcurve_oriented(c.edge(), id, c.is_forward())
                ));
            }
        }
    }
    for id in t.live_loop_ids() {
        rows.push(format!("loop {id:?} {:?}", t.face_loop(id).unwrap()));
    }
    for id in t.live_coedge_ids() {
        rows.push(format!("coedge {id:?} {:?}", t.coedge(id).unwrap()));
    }
    let mut journal = t.journal().snapshot();
    journal.next_op = 0;
    journal.next_ordinal = 0;
    rows.push(format!("{journal:?}"));
    rows.join("\n")
}

fn attributes(name: &str) -> EntityAttributes {
    EntityAttributes {
        name: Some(name.into()),
        ..EntityAttributes::default()
    }
}

pub fn fixture() -> (Topology, crate::FaceId, crate::SolidId, PersistentRef) {
    let mut t = Topology::new();
    let face = triangle_face(&mut t);
    t.set_face_attributes(face, attributes("before")).unwrap();
    let c = t
        .face_loop(t.loops_of_face(face).unwrap()[0])
        .unwrap()
        .coedges()[0];
    t.set_coedge_pcurve(
        c,
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
            0.0,
            1.0,
        ),
    )
    .unwrap();
    let solid = t.add_empty_solid();
    t.set_solid_attributes(solid, attributes("solid")).unwrap();
    let pending = t.journal_begin("seed");
    let mut draft = EvolutionDraft::construction();
    draft.push(
        EntityKey::face(face.index()),
        EventDraft::Generated { sources: vec![] },
    );
    let op = t.journal_record_evolution(pending, draft).unwrap();
    let reference = PersistentRef::operation_output(op, crate::journal::EntityKind::Face, 0);
    (t, face, solid, reference)
}

// Each stage is a real public mutation. Every prefix can fail, including after
// retirement and publication into the history index.
pub fn mutate(
    t: &mut Topology,
    face: crate::FaceId,
    solid: crate::SolidId,
    stop: usize,
) -> Result<(), TopologyError> {
    for stage in 0..14 {
        match stage {
            0 => {
                seed(t);
            }
            1 => {
                let v = t.vertices().iter().next().unwrap().0;
                t.vertex_mut(v)?.set_point(Point3::new(7.0, 8.0, 9.0));
            }
            2 => {
                t.set_face_attributes(face, attributes("changed"))?;
                t.set_solid_attributes(solid, attributes("changed"))?;
            }
            3 => {
                let cid = t.face_loop(t.loops_of_face(face).unwrap()[0])?.coedges()[0];
                t.set_coedge_pcurve(
                    cid,
                    PCurve::new(
                        Curve2D::Line(
                            Line2D::new(Point2::new(3.0, 4.0), Vec2::new(0.0, 1.0)).unwrap(),
                        ),
                        2.0,
                        7.0,
                    ),
                )?;
                t.build_face_loops(face)?;
            }
            4 => {
                let e = t.edges().iter().next().unwrap().0;
                t.edge_mut(e)?.set_trim(Some((0.25, 0.75)));
            }
            5 => {
                t.face_mut(face)?
                    .set_surface(crate::face::FaceSurface::Plane {
                        normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                        d: 2.0,
                    });
            }
            6 => {
                let wire = t.face(face)?.outer_wire();
                t.wire_mut(wire)?.edges_mut().reverse();
            }
            7 => {
                let cid = t.face_loop(t.loops_of_face(face).unwrap()[0])?.coedges()[0];
                t.set_coedge_periodic_winding(cid, crate::coedge::PeriodicWinding::new(2, -1))?;
                t.remove_coedge_pcurve(cid)?;
            }
            8 => {
                let wire = t.face(face)?.outer_wire();
                t.set_face_boundary_wires(face, wire, vec![])?;
            }
            9 => {
                let host = t.add_empty_solid();
                t.add_compound(crate::compound::Compound::new(vec![host]));
                t.add_compsolid(crate::compsolid::CompSolid::new(vec![host], vec![]));
            }
            10 => {
                t.set_face_attributes(face, EntityAttributes::default())?;
            }
            11 => {
                let fresh = t.vertices().iter().last().unwrap().0;
                let pending = t.journal_begin("injected-evolution");
                let mut draft = EvolutionDraft::construction();
                draft.push(
                    EntityKey::face(face.index()),
                    EventDraft::Generated {
                        sources: vec![EntityKey::vertex(fresh.index())],
                    },
                );
                t.journal_record_evolution(pending, draft)?;
            }
            12 => {
                t.delete_solid(solid).unwrap();
            }
            _ => {
                let p = t.journal_begin("injected");
                t.journal_record_barrier(p, vec![EntityKey::face(face.index())]);
            }
        }
        if stage == stop {
            return Err(TopologyError::WireNotClosed);
        }
    }
    Ok(())
}

#[test]
fn fault_prefixes_match_full_snapshot_oracle_and_preserve_outer_work() {
    for stop in 0..14 {
        let (original, face, solid, reference) = fixture();
        let before = state(&original);
        let resolved = format!("{:?}", resolve(&original, &reference));
        for legacy in [true, false] {
            let mut t = original.clone();
            let checkpoint = t.clone();
            let mut failed = Vec::new();
            run(legacy, &mut t, |t| {
                t.set_face_attributes(face, attributes("outer"))?;
                let outer = state(t);
                for _ in 0..3 {
                    let result = run(legacy, t, |t| {
                        failed.push(seed(t));
                        mutate(t, face, solid, stop)
                    });
                    assert!(result.is_err());
                    assert_eq!(state(t), outer, "caught failure at {stop}");
                    let entries = t.journal().len();
                    let _ = t.journal_begin("observe-restored-ticks");
                    assert_eq!(
                        t.journal().len(),
                        entries,
                        "rollback introduced a history gap"
                    );
                }
                Ok(())
            })
            .unwrap();
            assert_eq!(
                t.attributes().face(face).unwrap().name.as_deref(),
                Some("outer")
            );
            t.restore_preserving_handle_slots(&checkpoint);
            assert_eq!(state(&t), before);
            assert_eq!(format!("{:?}", resolve(&t, &reference)), resolved);
            for _ in 0..4 {
                seed(&mut t);
            }
            for id in failed {
                assert!(t.vertex(id).is_err());
            }
        }
    }
}

#[test]
fn propagated_failure_and_outer_failure_after_success_match_oracle() {
    for inner_fails in [false, true] {
        let (original, face, solid, _) = fixture();
        for legacy in [true, false] {
            let mut t = original.clone();
            let before = state(&t);
            let result = run(legacy, &mut t, |t| {
                seed(t);
                run(legacy, t, |t| {
                    mutate(t, face, solid, if inner_fails { 13 } else { 14 })
                })?;
                Err::<(), _>(TopologyError::WireNotClosed)
            });
            assert!(result.is_err());
            assert_eq!(state(&t), before);
        }
    }
}

#[test]
fn nested_validation_veto_restores_its_entry_state() {
    let (mut t, face, solid, _) = fixture();
    run_transacted(&mut t, |t| {
        t.set_face_attributes(face, attributes("outer"))?;
        let before = state(t);
        let result = run_validated(
            t,
            |t| mutate(t, face, solid, 14),
            |_, ()| Err(TopologyError::WireNotClosed),
        );
        assert!(result.is_err());
        assert_eq!(state(t), before);
        Ok::<_, TopologyError>(())
    })
    .unwrap();
}

#[test]
fn captures_cost_no_copies_and_restore_by_marks() {
    // Capture is O(1): no records are written and no document data moves,
    // however many scopes nest. Marks order by capture position, so inner
    // and outer restores each return to their own entry state.
    let (mut t, face, _, _) = fixture();
    assert_eq!(t.undo_record_count(), 0);
    let outer = RollbackSnapshot::capture(&mut t);
    let same = RollbackSnapshot::capture(&mut t);
    assert_eq!(t.undo_record_count(), 0, "capture must not copy state");
    drop(same);
    t.set_face_attributes(face, attributes("outer-only"))
        .unwrap();
    assert!(t.undo_record_count() > 0, "writes are recorded, not copied");
    let changed = RollbackSnapshot::capture(&mut t);
    let before = state(&t);
    t.set_face_attributes(face, attributes("inner-only"))
        .unwrap();
    changed.restore(&mut t);
    assert_eq!(state(&t), before);
    outer.restore(&mut t);
    assert_eq!(
        t.attributes().face(face).unwrap().name.as_deref(),
        Some("before")
    );
    assert_eq!(t.undo_record_count(), 0);
}

#[test]
fn clones_start_with_independent_empty_logs() {
    // A clone never shares rollback storage with its source: it records
    // where the source's log stood (for exact restores into the source) and
    // starts empty.
    // Journal appends inside a scope are covered by journal-length marks, so
    // journal-only traffic needs no snapshot of its own.
    let (mut t, face, _, _) = fixture();
    let _outer = RollbackSnapshot::capture(&mut t);
    let mut copy = t.clone();
    assert_eq!(copy.undo_record_count(), 0);
    let before = t.undo_record_count();
    let _copied = RollbackSnapshot::capture(&mut copy);
    copy.set_face_attributes(face, attributes("copy-only"))
        .unwrap();
    assert!(copy.undo_record_count() > 0);
    assert_eq!(
        t.undo_record_count(),
        before,
        "the copy logs into its own log"
    );
    drop(_copied);
    drop(_outer);
}

#[test]
fn clone_chain_restore_keeps_outer_undo_records() {
    let mut topo = Topology::new();
    let mut staged = None;
    let result = run_transacted(&mut topo, |topo| {
        let vertex = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 2.0, 3.0), 1e-7));
        staged = Some(vertex);
        let intermediate = topo.clone();
        let snapshot = intermediate.clone();
        topo.restore_for_rollback(&snapshot);
        assert_eq!(intermediate.num_vertices(), 1);
        Err::<(), _>(TopologyError::WireNotClosed)
    });
    assert!(matches!(result, Err(TopologyError::WireNotClosed)));
    assert_eq!(topo.num_vertices(), 0);
    assert!(topo.vertex(staged.unwrap()).is_err());
}

#[test]
fn mutated_clone_chain_restore_preserves_complete_outer_rollback() {
    let mut topo = Topology::new();
    let baseline = topo.add_vertex(crate::vertex::Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    let mut staged = None;
    let mut overwritten = None;
    let result = run_transacted(&mut topo, |topo| {
        staged =
            Some(topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7)));
        let mut intermediate = topo.clone();
        intermediate.add_vertex(crate::vertex::Vertex::new(Point3::new(2.0, 0.0, 0.0), 1e-7));
        let snapshot = intermediate.clone();
        overwritten =
            Some(topo.add_vertex(crate::vertex::Vertex::new(Point3::new(3.0, 0.0, 0.0), 1e-7)));
        topo.restore_for_rollback(&snapshot);
        assert_eq!(topo.num_vertices(), 3);
        Err::<(), _>(TopologyError::WireNotClosed)
    });
    assert!(matches!(result, Err(TopologyError::WireNotClosed)));
    assert_eq!(topo.num_vertices(), 1);
    assert!(topo.vertex(baseline).is_ok());
    assert!(topo.vertex(staged.unwrap()).is_err());
    assert!(topo.vertex(overwritten.unwrap()).is_err());
}

#[test]
fn unrelated_restore_with_matching_counters_preserves_outer_rollback() {
    let mut topo = Topology::new();
    let mut other = Topology::new();
    for x in [4.0, 5.0] {
        other.add_vertex(crate::vertex::Vertex::new(Point3::new(x, 0.0, 0.0), 1e-7));
    }
    let snapshot = other.clone();
    let result = run_transacted(&mut topo, |topo| {
        for x in [1.0, 2.0] {
            topo.add_vertex(crate::vertex::Vertex::new(Point3::new(x, 0.0, 0.0), 1e-7));
        }
        topo.restore_for_rollback(&snapshot);
        assert_eq!(topo.num_vertices(), 2);
        Err::<(), _>(TopologyError::WireNotClosed)
    });
    assert!(matches!(result, Err(TopologyError::WireNotClosed)));
    assert_eq!(topo.num_vertices(), 0);
}

#[test]
fn unrelated_restore_without_scope_rebases_inherited_clone_lineage() {
    for preserving_slots in [false, true] {
        let mut topo = Topology::new();
        let original =
            topo.add_vertex(crate::vertex::Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let before = state(&topo);
        let mut intermediate = topo.clone();
        let mut unrelated = Topology::new();
        unrelated.add_vertex(crate::vertex::Vertex::new(Point3::new(9.0, 0.0, 0.0), 1e-7));
        if preserving_slots {
            intermediate.restore_preserving_handle_slots(&unrelated.clone());
        } else {
            intermediate.restore_for_rollback(&unrelated.clone());
        }
        let snapshot = intermediate.clone();
        let result = run_transacted(&mut topo, |topo| {
            topo.restore_for_rollback(&snapshot);
            assert_eq!(topo.vertex(original)?.point(), Point3::new(9.0, 0.0, 0.0));
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        assert!(matches!(result, Err(TopologyError::WireNotClosed)));
        assert_eq!(state(&topo), before, "preserving slots: {preserving_slots}");
        assert_eq!(
            topo.vertex(original).unwrap().point(),
            Point3::new(1.0, 0.0, 0.0)
        );
    }
}

#[test]
fn barrier_restore_inside_a_scope_keeps_the_outer_rollback_exact() {
    // A same-lineage barrier restore keeps the retirements made after its
    // snapshot (and re-derives the loops of faces that lost theirs), so it
    // does not land on the snapshot. An enclosing scope must still rewind to
    // its own entry state, whichever stage the retirements came from, both
    // for a value never cloned and for one that is itself a clone (as a
    // kernel topology is after a checkpoint and its first write).
    for stop in 0..15 {
        for inherited in [false, true] {
            let (original, face, solid, _) = fixture();
            let before = state(&original);
            let mut t = if inherited {
                original.clone()
            } else {
                original
            };
            let outer = RollbackSnapshot::capture(&mut t);
            let barrier = t.clone();
            let _ = mutate(&mut t, face, solid, stop);
            t.restore_preserving_handle_slots(&barrier);
            crate::validation::validate_boundary_authority(&t).unwrap();
            // Stage 8 is the first retirement. Before it the restore lands on
            // the snapshot and still truncates instead of copying the
            // document. A copy of a clone carries the lineage its source
            // inherited, opened before the scope, so it never truncates.
            assert_eq!(
                t.undo_record_count() == 0,
                stop < 8 && !inherited,
                "stop {stop}, inherited {inherited}"
            );
            outer.restore(&mut t);
            crate::validation::validate_boundary_authority(&t).unwrap();
            assert_eq!(state(&t), before, "stop {stop}, inherited {inherited}");
            assert!(t.solid(solid).is_ok(), "stop {stop}, inherited {inherited}");
        }
    }
}

/// Restores `snapshot` with transaction (`rollback`) or checkpoint-barrier
/// semantics.
fn restore_from(t: &mut Topology, snapshot: &Topology, rollback: bool) {
    if rollback {
        t.restore_for_rollback(snapshot);
    } else {
        t.restore_preserving_handle_slots(snapshot);
    }
}

#[test]
fn restoring_a_clone_of_a_cloned_value_keeps_its_scope_records() {
    // A value that is itself a clone (as a kernel topology is after a
    // checkpoint and its first write) must cut its own log at its own
    // clone-time position, never at the position its source stood at. The
    // tick-free attribute write recorded before the inner copy has to
    // survive the restore so the scope can still undo it.
    for rollback in [false, true] {
        let (original, face, _, _) = fixture();
        let mut t = original.clone();
        let outer = RollbackSnapshot::capture(&mut t);
        let entry = state(&t);
        t.set_face_attributes(face, attributes("staged")).unwrap();
        let snapshot = t.clone();
        t.set_face_attributes(face, attributes("later")).unwrap();
        restore_from(&mut t, &snapshot, rollback);
        assert_eq!(
            t.attributes().face(face).unwrap().name.as_deref(),
            Some("staged")
        );
        outer.restore(&mut t);
        assert_eq!(state(&t), entry, "rollback restore: {rollback}");
    }
}

#[test]
fn restoring_a_snapshot_older_than_the_scope_rewinds_to_the_scope_entry() {
    // Edits made before a scope opened are not in its log, so a snapshot
    // taken before them cannot be reached by cutting the log: the scope
    // must still rewind to its own entry state, not to the snapshot.
    for stop in 0..15 {
        for rollback in [false, true] {
            let (mut t, face, solid, _) = fixture();
            let snapshot = t.clone();
            let _ = mutate(&mut t, face, solid, stop);
            let entry = state(&t);
            let entry_check = crate::validation::validate_boundary_authority(&t).is_ok();
            let scope = RollbackSnapshot::capture(&mut t);
            restore_from(&mut t, &snapshot, rollback);
            scope.restore(&mut t);
            assert_eq!(state(&t), entry, "stop {stop}, rollback {rollback}");
            assert_eq!(
                crate::validation::validate_boundary_authority(&t).is_ok(),
                entry_check
            );
        }
    }
}

#[test]
fn restoring_a_clone_changed_without_ticks_keeps_the_outer_rollback_exact() {
    // Attribute writes and journal appends move no mutation tick, yet they
    // change the clone: restoring it is not a return to its copy point.
    type Change = fn(&mut Topology, crate::FaceId);
    let changes: [(&str, Change); 2] = [
        ("attribute", |t, face| {
            t.set_face_attributes(face, attributes("private")).unwrap();
        }),
        ("journal", |t, _| {
            let pending = t.journal_begin("private");
            t.journal_record_barrier(pending, vec![EntityKey::vertex(2)]);
        }),
    ];
    for (name, change) in changes {
        for rollback in [false, true] {
            let (mut t, face, _, _) = fixture();
            let outer = RollbackSnapshot::capture(&mut t);
            let entry = state(&t);
            let mut snapshot = t.clone();
            change(&mut snapshot, face);
            restore_from(&mut t, &snapshot, rollback);
            outer.restore(&mut t);
            assert_eq!(state(&t), entry, "{name}, rollback {rollback}");
        }
    }
}

#[test]
fn restoring_after_the_log_was_cut_below_the_copy_point_keeps_the_rollback_exact() {
    // A rewind cut the record the copy was taken after, and a new record
    // took its place: the log no longer leads from the copy back to the
    // scope entry, however long it is.
    for rollback in [false, true] {
        let (mut t, face, solid, _) = fixture();
        let outer = RollbackSnapshot::capture(&mut t);
        let entry = state(&t);
        let inner = RollbackSnapshot::capture(&mut t);
        t.set_face_attributes(face, attributes("inner")).unwrap();
        let snapshot = t.clone();
        inner.restore(&mut t);
        t.set_solid_attributes(solid, attributes("other")).unwrap();
        restore_from(&mut t, &snapshot, rollback);
        outer.restore(&mut t);
        assert_eq!(state(&t), entry, "rollback restore: {rollback}");
    }
}

#[test]
fn restoring_an_unchanged_copy_of_this_value_still_cuts_the_log() {
    // The exact path survives: a copy taken inside the scope, restored
    // twice (as a retrying caller does), keeps the log at the copy point
    // instead of recording a document-sized inverse.
    for rollback in [false, true] {
        let (mut t, face, solid, _) = fixture();
        let outer = RollbackSnapshot::capture(&mut t);
        let entry = state(&t);
        t.set_face_attributes(face, attributes("kept")).unwrap();
        let snapshot = t.clone();
        let at_copy = t.undo_record_count();
        for _ in 0..2 {
            let nested = RollbackSnapshot::capture(&mut t);
            let _ = mutate(&mut t, face, solid, 7);
            nested.restore(&mut t);
            let _ = mutate(&mut t, face, solid, 7);
            restore_from(&mut t, &snapshot, rollback);
            assert_eq!(t.undo_record_count(), at_copy, "rollback {rollback}");
        }
        outer.restore(&mut t);
        assert_eq!(state(&t), entry, "rollback restore: {rollback}");
    }
}

#[test]
fn restoring_a_copy_of_a_changed_clone_keeps_recording_a_full_inverse() {
    // Unchanged from the lineage certificate alone: a value that is itself
    // a clone, changed since, still carries its source's position, so a
    // restore of its own copy inside a scope records the full inverse as it
    // always did (and a later rewind leaves the same tombstones), although
    // the copy's own position would now certify a cut.
    for rollback in [false, true] {
        let (original, face, solid, _) = fixture();
        let mut t = original.clone();
        let _ = mutate(&mut t, face, solid, 1);
        let outer = RollbackSnapshot::capture(&mut t);
        let entry = state(&t);
        t.set_face_attributes(face, attributes("kept")).unwrap();
        let snapshot = t.clone();
        t.set_face_attributes(face, attributes("later")).unwrap();
        let before = t.undo_record_count();
        restore_from(&mut t, &snapshot, rollback);
        assert_eq!(t.undo_record_count(), before + 1, "rollback {rollback}");
        outer.restore(&mut t);
        assert_eq!(state(&t), entry, "rollback restore: {rollback}");
    }
}

#[test]
fn nested_success_commits_and_dropped_scopes_release_storage() {
    fn send_sync<T: Send + Sync>() {}
    let mut t = Topology::new();
    let v = run_transacted(&mut t, |t| {
        run_transacted(t, |t| Ok::<_, TopologyError>(seed(t)))
    })
    .unwrap();
    assert!(t.vertex(v).is_ok());
    assert_eq!(t.undo_record_count(), 0, "committed scopes keep no records");
    // A dropped snapshot commits: the allocation stays live, and the next
    // transaction access reclaims the dead scope entry and its records.
    let committed = {
        let snapshot = RollbackSnapshot::capture(&mut t);
        let w = seed(&mut t);
        assert_eq!(
            t.undo_record_count(),
            0,
            "allocations are covered by the scope's slot marks"
        );
        t.vertex_mut(v)
            .unwrap()
            .set_point(Point3::new(1.0, 2.0, 3.0));
        assert!(t.undo_record_count() > 0);
        (snapshot, w)
    };
    assert!(
        t.vertex(committed.1).is_ok(),
        "dropped scope committed its work"
    );
    assert_eq!(t.vertex(v).unwrap().point(), Point3::new(1.0, 2.0, 3.0));
    drop(committed.0);
    let _ = RollbackSnapshot::capture(&mut t);
    assert_eq!(
        t.undo_record_count(),
        0,
        "dead scope entries release storage"
    );
    assert!(t.vertex(committed.1).is_ok());
    send_sync::<Topology>();
}

#[test]
fn mutable_entity_and_pcurve_access_create_genuine_savepoints() {
    for kind in 0..9 {
        let (mut t, face, solid, _) = fixture();
        // An enclosing live scope: the inner capture must still isolate and
        // restore exactly, with outer work preserved.
        let _outer = RollbackSnapshot::capture(&mut t);
        let vertex = t.vertices().iter().next().unwrap().0;
        let edge = t.edges().iter().next().unwrap().0;
        let wire = t.face(face).unwrap().outer_wire();
        let shell = t.solid(solid).unwrap().outer_shell();
        let coedge = t
            .face_loop(t.loops_of_face(face).unwrap()[0])
            .unwrap()
            .coedges()[0];
        match kind {
            0 => {
                t.vertex_mut(vertex)
                    .unwrap()
                    .set_point(Point3::new(9.0, 8.0, 7.0));
            }
            1 => {
                t.edge_mut(edge).unwrap();
            }
            2 => {
                let replacement = t.wire(wire).unwrap().clone();
                t.replace_boundary_wire(wire, replacement).unwrap();
            }
            3 => {
                t.face_mut(face).unwrap();
            }
            4 => {
                t.shell_mut(shell).unwrap();
            }
            5 => {
                t.solid_mut(solid).unwrap();
            }
            6 => {
                t.remove_coedge_pcurve(coedge).unwrap();
            }
            7 => {
                t.set_face_boundary_wires(face, wire, vec![]).unwrap();
            }
            _ => {
                t.delete_solid(solid).unwrap();
            }
        }
        let before_inner = state(&t);
        let records_before_inner = t.undo_record_count();
        let inner = RollbackSnapshot::capture(&mut t);
        t.set_face_attributes(face, attributes("inner")).unwrap();
        assert!(
            t.undo_record_count() > records_before_inner,
            "mutable access {kind}"
        );
        inner.restore(&mut t);
        assert_eq!(state(&t), before_inner, "mutable access {kind}");
    }
}

#[test]
fn rolled_back_journal_ids_and_new_boundary_handles_are_never_reissued() {
    let (mut t, face, _, reference) = fixture();
    let checkpoint = t.clone();
    let resolved = format!("{:?}", resolve(&t, &reference));
    let mut failed_ops = Vec::new();
    let mut failed_loops = Vec::new();
    let mut failed_coedges = Vec::new();
    for _ in 0..3 {
        let result = run_transacted(&mut t, |t| {
            let wire = t.face(face)?.outer_wire();
            t.set_face_boundary_wires(face, wire, vec![])?;
            let loops = t.loops_of_face(face).unwrap().to_vec();
            for &id in &loops {
                failed_coedges.extend_from_slice(t.face_loop(id)?.coedges());
            }
            failed_loops.extend(loops);
            let pending = t.journal_begin("failed");
            failed_ops.push(t.journal_record_barrier(pending, vec![EntityKey::face(face.index())]));
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        assert!(result.is_err());
    }
    t.restore_preserving_handle_slots(&checkpoint);
    t.build_face_loops(face).unwrap();
    let pending = t.journal_begin("later");
    let later = t.journal_record_barrier(pending, vec![]);
    for id in failed_ops {
        assert!(later.value() > id.value());
    }
    for id in failed_loops {
        assert!(t.face_loop(id).is_err());
    }
    for id in failed_coedges {
        assert!(t.coedge(id).is_err());
    }
    t.restore_preserving_handle_slots(&checkpoint);
    assert_eq!(format!("{:?}", resolve(&t, &reference)), resolved);
}

/// TOPO-3: a handle-preserving restore keeps retirements made after its
/// snapshot and re-promotes the faces they orphaned, so it does not land on
/// the clone-time state. Inside a live scope it must stay undoable: here the
/// boundary loops go [0] → [1] → [2], and the rollback must revive [0].
#[test]
fn handle_preserving_restore_inside_a_scope_rolls_back_exactly() {
    let mut t = Topology::new();
    let face = triangle_face(&mut t);
    let before = state(&t);
    let original = t.loops_of_face(face).unwrap().to_vec();
    let mut staged = Vec::new();
    let result = run_transacted(&mut t, |t| {
        let checkpoint = t.clone();
        let wire = t.face(face)?.outer_wire();
        t.set_face_boundary_wires(face, wire, vec![])?;
        staged.extend_from_slice(t.loops_of_face(face).unwrap());
        t.restore_preserving_handle_slots(&checkpoint);
        assert!(t.face_loop(original[0]).is_err(), "retirement is sticky");
        staged.extend_from_slice(t.loops_of_face(face).unwrap());
        Err::<(), _>(TopologyError::WireNotClosed)
    });
    assert!(result.is_err());
    assert_eq!(t.loops_of_face(face).unwrap(), original.as_slice());
    assert!(
        t.face_loop(original[0]).is_ok(),
        "rollback revives the loop"
    );
    crate::validation::validate_face_loops(&t, face).unwrap();
    assert_eq!(state(&t), before);
    assert_eq!(staged.len(), 2);
    for id in staged {
        assert!(t.face_loop(id).is_err());
    }
}

/// TOPO-3 without a live scope: the same restore on a clone that inherited
/// its source's lineage rolls the tick back to the clone point, so a later
/// clone of it would pass for an unchanged prefix of the source's log and a
/// scoped restore of that clone would drop the source's rollback records.
#[test]
fn handle_preserving_restore_rebases_an_inheriting_clone() {
    let mut t = Topology::new();
    let face = triangle_face(&mut t);
    let before = state(&t);
    let mut intermediate = t.clone();
    let checkpoint = intermediate.clone();
    let wire = intermediate.face(face).unwrap().outer_wire();
    intermediate
        .set_face_boundary_wires(face, wire, vec![])
        .unwrap();
    intermediate.restore_preserving_handle_slots(&checkpoint);
    let snapshot = intermediate.clone();
    let result = run_transacted(&mut t, |t| {
        t.restore_for_rollback(&snapshot);
        Err::<(), _>(TopologyError::WireNotClosed)
    });
    assert!(result.is_err());
    crate::validation::validate_face_loops(&t, face).unwrap();
    assert_eq!(state(&t), before);
}

/// Append-only construction commits at O(new content): no full copy, no
/// retained log, and every allocation stays live above the entry marks.
#[test]
fn append_only_construction_commits_without_copying() {
    let (mut t, face, _, _) = fixture();
    let live_before = (t.num_vertices(), t.num_edges(), t.num_faces());

    // A pre-existing write trips the guard: the scope falls back with an
    // identical result instead of committing a silent partial scope.
    let (staged, path) = run_append_only(&mut t, |t| {
        let v = seed(t);
        t.set_face_attributes(face, attributes("fallback-attr"))?;
        Ok::<_, TopologyError>(v)
    })
    .unwrap();
    assert_eq!(path, AppendPath::FullFallback);
    assert!(t.vertex(staged).is_ok());
    assert_eq!(
        t.attributes().face(face).unwrap().name.as_deref(),
        Some("fallback-attr"),
        "the fallback retry executes the refused write"
    );

    // Pure allocation commits on the fast path with no retained storage.
    let (only_new, path) = run_append_only(&mut t, |t| Ok::<_, TopologyError>(seed(t))).unwrap();
    assert_eq!(path, AppendPath::AppendOnly);
    assert!(t.vertex(only_new).is_ok());
    assert_eq!(t.undo_record_count(), 0, "committed scopes keep no records");
    assert!(t.num_vertices() > live_before.0);
    assert_eq!(
        (t.num_edges(), t.num_faces()),
        (live_before.1, live_before.2)
    );
}

/// Structural equality modulo allocation numbering: a fallback retry
/// tombstones the tripped attempt's allocations, so numeric handles for new
/// entities legitimately differ from a single-pass full-path run. Everything
/// observable — outcome aside — must match: live counts, every
/// pre-existing slot's value, boundary structure, pcurves, attributes, and
/// journal kinds.
fn assert_structurally_equal(
    kind: &str,
    actual: &Topology,
    expected: &Topology,
    original: &Topology,
) {
    assert_eq!(
        actual.num_vertices(),
        expected.num_vertices(),
        "kind {kind}"
    );
    assert_eq!(actual.num_edges(), expected.num_edges(), "kind {kind}");
    assert_eq!(actual.num_wires(), expected.num_wires(), "kind {kind}");
    assert_eq!(actual.num_faces(), expected.num_faces(), "kind {kind}");
    assert_eq!(actual.num_shells(), expected.num_shells(), "kind {kind}");
    assert_eq!(actual.num_solids(), expected.num_solids(), "kind {kind}");
    assert_eq!(
        actual.num_compounds(),
        expected.num_compounds(),
        "kind {kind}"
    );
    assert_eq!(
        actual.num_compsolids(),
        expected.num_compsolids(),
        "kind {kind}"
    );
    assert_eq!(actual.num_loops(), expected.num_loops(), "kind {kind}");
    assert_eq!(actual.num_coedges(), expected.num_coedges(), "kind {kind}");
    assert_eq!(actual.num_pcurves(), expected.num_pcurves(), "kind {kind}");
    assert_eq!(
        actual.journal().len(),
        expected.journal().len(),
        "kind {kind}"
    );
    let kinds = |t: &Topology| {
        t.journal()
            .entries()
            .iter()
            .map(|entry| entry.kind().to_owned())
            .collect::<Vec<_>>()
    };
    assert_eq!(kinds(actual), kinds(expected), "kind {kind}");
    // Pre-existing slots keep their indices across the retry (only new
    // allocations tombstone), so their values compare directly.
    for (id, _) in original.vertices().iter() {
        assert_eq!(
            format!("{:?}", actual.vertex(id)),
            format!("{:?}", expected.vertex(id)),
            "kind {kind}"
        );
    }
    for (id, _) in original.edges().iter() {
        assert_eq!(
            format!("{:?}", actual.edge(id)),
            format!("{:?}", expected.edge(id)),
            "kind {kind}"
        );
    }
    for (id, _) in original.wires().iter() {
        assert_eq!(
            format!("{:?}", actual.wire(id)),
            format!("{:?}", expected.wire(id)),
            "kind {kind}"
        );
    }
    for (id, _) in original.shells().iter() {
        assert_eq!(
            format!("{:?}", actual.shell(id)),
            format!("{:?}", expected.shell(id)),
            "kind {kind}"
        );
    }
    for (id, _) in original.solids().iter() {
        assert_eq!(
            format!("{:?}", actual.solid(id)),
            format!("{:?}", expected.solid(id)),
            "kind {kind}"
        );
        assert_eq!(
            actual.attributes().solid(id),
            expected.attributes().solid(id),
            "kind {kind}"
        );
    }
    for (id, _) in original.compounds().iter() {
        assert_eq!(
            format!("{:?}", actual.compound(id)),
            format!("{:?}", expected.compound(id)),
            "kind {kind}"
        );
    }
    for (id, _) in original.compsolids().iter() {
        assert_eq!(
            format!("{:?}", actual.compsolid(id)),
            format!("{:?}", expected.compsolid(id)),
            "kind {kind}"
        );
    }
    // Faces carry derived loop handles (renumbered by the retry), so they
    // compare by boundary wires, surface, and orientation instead.
    for (id, _) in original.faces().iter() {
        let (a, b) = (actual.face(id).unwrap(), expected.face(id).unwrap());
        assert_eq!(a.outer_wire(), b.outer_wire(), "kind {kind}");
        assert_eq!(a.inner_wires(), b.inner_wires(), "kind {kind}");
        assert_eq!(
            format!("{:?}", a.surface()),
            format!("{:?}", b.surface()),
            "kind {kind}"
        );
        assert_eq!(a.is_reversed(), b.is_reversed(), "kind {kind}");
        assert_eq!(
            actual.attributes().face(id),
            expected.attributes().face(id),
            "kind {kind}"
        );
        // Boundary structure: loop counts, then (edge, orientation, pcurve)
        // uses per loop in install order.
        let (a_loops, b_loops) = (actual.loops_of_face(id), expected.loops_of_face(id));
        assert_eq!(
            a_loops.map(<[_]>::len),
            b_loops.map(<[_]>::len),
            "kind {kind}"
        );
        if let (Some(a_loops), Some(b_loops)) = (a_loops, b_loops) {
            for (a_loop, b_loop) in a_loops.iter().zip(b_loops.iter()) {
                let uses = |t: &Topology, loop_id: &crate::LoopId| {
                    t.face_loop(*loop_id)
                        .unwrap()
                        .coedges()
                        .iter()
                        .map(|cid| {
                            let c = t.coedge(*cid).unwrap();
                            format!("{:?}:{}{:?}", c.edge(), c.is_forward(), c.pcurve())
                        })
                        .collect::<Vec<_>>()
                };
                assert_eq!(uses(actual, a_loop), uses(expected, b_loop), "kind {kind}");
            }
        }
        let pcurves = |t: &Topology| {
            t.pcurves_for_face(id)
                .iter()
                .map(|(e, fwd, p)| format!("{e:?}{fwd}{p:?}"))
                .collect::<Vec<_>>()
        };
        assert_eq!(pcurves(actual), pcurves(expected), "kind {kind}");
    }
    // New allocations: same values in encounter order, whatever their slots.
    let fresh = |t: &Topology| {
        t.vertices()
            .iter()
            .filter(|(id, _)| original.vertex(*id).is_err())
            .map(|(_, v)| format!("{v:?}"))
            .collect::<Vec<_>>()
    };
    assert_eq!(fresh(actual), fresh(expected), "kind {kind}");
}

/// Every pre-existing write kind trips the guard before landing; journal
/// appends stay on the fast path. Each run matches the full-path result:
/// same outcome on the expected path, same structure modulo the retry's
/// tombstoned allocation numbers.
#[test]
fn append_only_guard_matrix_matches_the_full_path() {
    type Write = fn(&mut Topology, crate::FaceId, crate::SolidId) -> Result<(), TopologyError>;
    let fallback_kinds: &[(&str, Write)] = &[
        ("vertex", |t: &mut Topology, _, _| {
            let v = t.vertices().iter().next().unwrap().0;
            t.vertex_mut(v)
                .map(|v| v.set_point(Point3::new(1.0, 2.0, 3.0)))
        }),
        ("edge", |t: &mut Topology, _, _| {
            let e = t.edges().iter().next().unwrap().0;
            t.edge_mut(e).map(|_| ())
        }),
        ("wire", |t: &mut Topology, face, _| {
            let w = t.face(face).unwrap().outer_wire();
            t.wire_mut(w).map(|_| ())
        }),
        ("face", |t: &mut Topology, face, _| {
            t.face_mut(face).map(|_| ())
        }),
        ("shell", |t: &mut Topology, _, solid| {
            let s = t.solid(solid).unwrap().outer_shell();
            t.shell_mut(s).map(|_| ())
        }),
        ("solid", |t: &mut Topology, _, solid| {
            t.solid_mut(solid).map(|_| ())
        }),
        ("delete", |t: &mut Topology, _, solid| {
            t.delete_solid(solid)
                .map_err(|_| TopologyError::WireNotClosed)
        }),
        ("face-attr", |t: &mut Topology, face, _| {
            t.set_face_attributes(face, attributes("trip"))
        }),
        ("solid-attr", |t: &mut Topology, _, solid| {
            t.set_solid_attributes(solid, attributes("trip"))
        }),
        ("pcurve", |t: &mut Topology, face, _| {
            let cid = t.face_loop(t.loops_of_face(face).unwrap()[0])?.coedges()[0];
            t.set_coedge_pcurve(
                cid,
                PCurve::new(
                    Curve2D::Line(Line2D::new(Point2::new(9.0, 9.0), Vec2::new(1.0, 0.0)).unwrap()),
                    0.0,
                    1.0,
                ),
            )
            .map(|_| ())
        }),
        ("winding", |t: &mut Topology, face, _| {
            let cid = t.face_loop(t.loops_of_face(face).unwrap()[0])?.coedges()[0];
            t.set_coedge_periodic_winding(cid, crate::coedge::PeriodicWinding::new(1, 1))
                .map(|_| ())
        }),
        ("boundary", |t: &mut Topology, face, _| {
            let wire = t.face(face)?.outer_wire();
            let replacement = t.wire(wire)?.clone();
            t.replace_boundary_wire(wire, replacement)
        }),
        ("load-journal", |t: &mut Topology, _, _| {
            let loaded = t.journal().clone();
            t.load_journal(loaded);
            Ok(())
        }),
        ("foreign-restore", |t: &mut Topology, _, _| {
            let snapshot = t.clone();
            t.restore_for_rollback(&snapshot);
            Ok(())
        }),
    ];
    // Journal record calls only append, so they stay on the fast path while
    // recording exactly what the full path records.
    let append_kinds: &[(&str, Write)] = &[("journal-evolution", |t: &mut Topology, face, _| {
        let pending = t.journal_begin("append-evolution");
        let mut draft = EvolutionDraft::construction();
        draft.push(
            EntityKey::face(face.index()),
            EventDraft::Generated { sources: vec![] },
        );
        t.journal_record_evolution(pending, draft).map(|_| ())
    })];

    for (name, write, expected_path) in fallback_kinds
        .iter()
        .map(|(name, write)| (*name, *write, AppendPath::FullFallback))
        .chain(
            append_kinds
                .iter()
                .map(|(name, write)| (*name, *write, AppendPath::AppendOnly)),
        )
    {
        let (original, face, solid, _) = fixture();
        let before = state(&original);
        // Full-path reference result for the same closure.
        let mut expected = original.clone();
        let expected_result = run_transacted(&mut expected, |t| {
            seed(t);
            write(t, face, solid)
        });
        // Append-only run must match it exactly on the expected path.
        let mut t = original.clone();
        let result = run_append_only(&mut t, |t| {
            seed(t);
            write(t, face, solid)
        });
        match (&expected_result, &result) {
            (Ok(()), Ok(((), path))) => {
                assert_eq!(*path, expected_path, "kind {name}");
                assert_structurally_equal(name, &t, &expected, &original);
            }
            (Err(_), Err(_)) => {
                assert_eq!(state(&t), before, "kind {name}");
            }
            (other_expected, other_actual) => {
                // Unreachable for deterministic closures: the retry executes
                // the same closure on the rewound entry state. Fails with
                // both outcomes rendered when that assumption breaks.
                assert_eq!(
                    format!("{other_expected:?}"),
                    format!("{other_actual:?}"),
                    "kind {name}: append-only and full-path outcomes must agree"
                );
            }
        }
    }
}

/// A directly armed guard reports the typed trip error before any write;
///
/// genuine failures keep their own errors and also land nothing.
#[test]
fn append_guard_reports_a_typed_trip_before_writing() {
    let (mut t, _, _, _) = fixture();
    let before = state(&t);
    let mut leaked = None;
    let _ = run_transacted(&mut t, |t| {
        leaked = Some(seed(t));
        Err::<(), _>(TopologyError::WireNotClosed)
    });
    let stale = leaked.unwrap();

    t.arm_append();
    let vertex = t.vertices().iter().next().unwrap().0;
    let err = t.vertex_mut(vertex).unwrap_err();
    assert!(matches!(err, TopologyError::AppendOnlyGuardTrip { .. }));
    let not_found = t.vertex_mut(stale).unwrap_err();
    assert!(
        matches!(not_found, TopologyError::VertexNotFound(_)),
        "liveness is checked before the guard: stale handles keep typed errors"
    );
    assert!(t.disarm_append(), "the refused write tripped the guard");
    assert_eq!(state(&t), before, "refused writes land nothing");
}

/// Failed append-only scopes retire every staged allocation; handles stay
/// stale and pre-existing state is untouched.
#[test]
fn append_only_failure_retires_staged_allocations() {
    let (mut t, _, _, _) = fixture();
    let live_before = t.num_vertices();
    let slots_before = t.allocated_slot_count();
    let mut leaked = None;
    let err = run_append_only(&mut t, |t| {
        leaked = Some(seed(t));
        Err::<(), _>(TopologyError::WireNotClosed)
    })
    .unwrap_err();
    assert!(matches!(err, TopologyError::WireNotClosed));
    assert_eq!(t.num_vertices(), live_before);
    assert!(t.allocated_slot_count() >= slots_before);
    let staged = leaked.unwrap();
    assert!(t.vertex(staged).is_err());
    let fresh = seed(&mut t);
    assert_ne!(fresh, staged, "a rolled-back slot must never be reissued");
}

/// Append-only scopes nest inside plain scopes (and each other): inner
/// commits merge records outward, and an outer failure still rewinds all.
#[test]
fn append_only_scopes_nest_and_rewind_with_the_outer_scope() {
    let (mut t, _, _, _) = fixture();
    let before = state(&t);
    let mut inner_staged = None;
    let mut outer_staged = None;
    let err = run_transacted(&mut t, |t| {
        let (v, path) = run_append_only(t, |t| {
            let (w, inner_path) = run_append_only(t, |t| Ok::<_, TopologyError>(seed(t)))?;
            assert_eq!(inner_path, AppendPath::AppendOnly);
            inner_staged = Some(w);
            Ok::<_, TopologyError>(seed(t))
        })?;
        assert_eq!(path, AppendPath::AppendOnly);
        outer_staged = Some(v);
        Err::<(), _>(TopologyError::WireNotClosed)
    })
    .unwrap_err();
    assert!(matches!(err, TopologyError::WireNotClosed));
    assert_eq!(state(&t), before);
    assert!(t.vertex(inner_staged.unwrap()).is_err());
    assert!(t.vertex(outer_staged.unwrap()).is_err());
}

/// Handles a scope allocated while rewriting its own allocations: the undo
/// path records none of those writes (the slot marks retire them), so these
/// stages pin it to the full-snapshot oracle.
struct Staged {
    face: crate::FaceId,
    solid: crate::SolidId,
    vertex: crate::VertexId,
    edge: crate::EdgeId,
}

/// `wire` traversed the other way: reversed use order, flipped orientations.
fn reversed_wire(t: &Topology, wire: crate::WireId) -> Result<crate::wire::Wire, TopologyError> {
    let wire = t.wire(wire)?;
    let uses = wire
        .edges()
        .iter()
        .rev()
        .map(|used| crate::wire::OrientedEdge::new(used.edge(), !used.is_forward()))
        .collect();
    crate::wire::Wire::new(uses, wire.is_closed())
}

// Every stage rewrites entities the same scope allocated, including from
// nested scopes (where they count as pre-existing) and after re-deriving or
// retiring them. Each prefix can fail.
fn rewrite_own_allocations(
    legacy: bool,
    t: &mut Topology,
    stop: usize,
    staged: &mut Vec<Staged>,
) -> Result<(), TopologyError> {
    let face = triangle_face(t);
    let solid = t.add_empty_solid();
    let vertex = t.vertices().iter().last().unwrap().0;
    let edge = t.edges().iter().last().unwrap().0;
    staged.push(Staged {
        face,
        solid,
        vertex,
        edge,
    });
    let first_coedge = |t: &Topology| -> Result<crate::CoedgeId, TopologyError> {
        Ok(t.face_loop(t.loops_of_face(face).unwrap()[0])?.coedges()[0])
    };
    for stage in 0..11 {
        match stage {
            0 => {
                t.face_mut(face)?.set_reversed(true);
                t.vertex_mut(vertex)?.set_point(Point3::new(4.0, 5.0, 6.0));
            }
            1 => {
                t.set_face_attributes(face, attributes("own"))?;
                t.set_solid_attributes(solid, attributes("own"))?;
            }
            2 => {
                let cid = first_coedge(t)?;
                t.set_coedge_pcurve(
                    cid,
                    PCurve::new(
                        Curve2D::Line(
                            Line2D::new(Point2::new(0.5, 0.5), Vec2::new(1.0, 0.0)).unwrap(),
                        ),
                        0.0,
                        1.0,
                    ),
                )?;
                t.set_coedge_periodic_winding(cid, crate::coedge::PeriodicWinding::new(1, 0))?;
            }
            3 => {
                // Retires the fresh loops and coedges, allocates new ones.
                let wire = t.face(face)?.outer_wire();
                t.set_face_boundary_wires(face, wire, vec![])?;
            }
            4 => {
                // A caught nested failure: for the inner scope the outer
                // allocations are pre-existing and must come back exactly.
                let inner_before = state(t);
                let result = run(legacy, t, |t| {
                    t.face_mut(face)?.set_reversed(false);
                    t.edge_mut(edge)?.set_trim(Some((0.25, 0.5)));
                    let wire = t.face(face)?.outer_wire();
                    let replacement = reversed_wire(t, wire)?;
                    t.replace_boundary_wire(wire, replacement)?;
                    t.set_face_attributes(face, attributes("inner"))?;
                    t.delete_solid(solid)
                        .map_err(|_| TopologyError::WireNotClosed)?;
                    seed(t);
                    // Distinct from every stage error: the whole body ran.
                    Err::<(), _>(TopologyError::Empty { entity: "injected" })
                });
                assert!(
                    matches!(result, Err(TopologyError::Empty { entity: "injected" })),
                    "{result:?}"
                );
                assert_eq!(state(t), inner_before, "caught inner failure");
            }
            5 => {
                // A committed nested write to an outer allocation.
                run(legacy, t, |t| {
                    t.edge_mut(edge)?.set_trim(Some((0.0, 0.75)));
                    t.remove_coedge_pcurve(first_coedge(t)?)?;
                    Ok(())
                })?;
            }
            6 => {
                // Rewrites the wire slot and re-derives the face's loops in
                // the allocating scope (the second pass also rewrites the
                // first pass's wire).
                let wire = t.face(face)?.outer_wire();
                let replacement = reversed_wire(t, wire)?;
                t.replace_boundary_wire(wire, replacement)?;
                t.build_face_loops(face)?;
            }
            7 => {
                let pending = t.journal_begin("own-allocations");
                let mut draft = EvolutionDraft::construction();
                draft.push(
                    EntityKey::face(face.index()),
                    EventDraft::Generated {
                        sources: vec![EntityKey::vertex(vertex.index())],
                    },
                );
                t.journal_record_evolution(pending, draft)?;
            }
            8 => {
                t.delete_solid(solid)
                    .map_err(|_| TopologyError::WireNotClosed)?;
            }
            9 => {
                let pre_existing = t.vertices().iter().next().unwrap().0;
                t.vertex_mut(pre_existing)?
                    .set_point(Point3::new(-1.0, -1.0, -1.0));
            }
            _ => {
                t.edge_mut(edge)?.set_trim(None);
            }
        }
        if stage == stop {
            return Err(TopologyError::WireNotClosed);
        }
    }
    Ok(())
}

#[test]
fn rewriting_own_allocations_matches_full_snapshot_oracle() {
    // `stop` past the last stage commits: both paths must then agree too.
    for stop in 0..12 {
        let mut outcomes = Vec::new();
        for legacy in [true, false] {
            let (mut t, _, _, _) = fixture();
            let before = state(&t);
            let mut staged = Vec::new();
            let result = run(legacy, &mut t, |t| {
                // Twice: the second pass runs over the first pass's
                // allocations as well as its own.
                rewrite_own_allocations(legacy, t, 99, &mut staged)?;
                rewrite_own_allocations(legacy, t, stop, &mut staged)
            });
            if stop < 11 {
                assert!(result.is_err(), "stop {stop}");
                assert_eq!(state(&t), before, "stop {stop}, legacy {legacy}");
                for entity in &staged {
                    assert!(t.face(entity.face).is_err());
                    assert!(t.solid(entity.solid).is_err());
                    assert!(t.vertex(entity.vertex).is_err());
                    assert!(t.edge(entity.edge).is_err());
                }
            } else {
                result.unwrap();
            }
            outcomes.push(state(&t));
        }
        assert_eq!(outcomes[0], outcomes[1], "stop {stop}");
    }
}

/// A snapshot cloned before the scope opened is not a prefix of the scope's
/// log even when the log lengths coincide: restoring it inside the scope
/// must still let the rollback revive what was allocated in between.
#[test]
fn restoring_a_snapshot_older_than_the_scope_rolls_back_exactly() {
    for preserving_slots in [false, true] {
        let mut t = Topology::new();
        let checkpoint = t.clone();
        let between = seed(&mut t);
        let before = state(&t);
        let result = run_transacted(&mut t, |t| {
            if preserving_slots {
                t.restore_preserving_handle_slots(&checkpoint);
            } else {
                t.restore_for_rollback(&checkpoint);
            }
            assert!(t.vertex(between).is_err());
            Err::<(), _>(TopologyError::WireNotClosed)
        });
        assert!(result.is_err());
        assert!(
            t.vertex(between).is_ok(),
            "preserving slots: {preserving_slots}"
        );
        assert_eq!(state(&t), before, "preserving slots: {preserving_slots}");
    }
}
