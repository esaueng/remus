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

fn fixture() -> (Topology, crate::FaceId, crate::SolidId, PersistentRef) {
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
fn mutate(
    t: &mut Topology,
    face: crate::FaceId,
    solid: crate::SolidId,
    stop: usize,
) -> Result<(), TopologyError> {
    for stage in 0..6 {
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
    for stop in 0..6 {
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
                    mutate(t, face, solid, if inner_fails { 5 } else { 6 })
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
            |t| mutate(t, face, solid, 6),
            |_, ()| Err(TopologyError::WireNotClosed),
        );
        assert!(result.is_err());
        assert_eq!(state(t), before);
        Ok::<_, TopologyError>(())
    })
    .unwrap();
}

#[test]
fn only_unchanged_entry_states_share_storage() {
    let (mut t, face, _, _) = fixture();
    let outer = RollbackSnapshot::capture(&mut t);
    let same = RollbackSnapshot::capture(&mut t);
    assert!(Arc::ptr_eq(&outer.0, &same.0));
    t.set_face_attributes(face, attributes("outer-only"))
        .unwrap();
    let changed = RollbackSnapshot::capture(&mut t);
    assert!(!Arc::ptr_eq(&outer.0, &changed.0));
    let before = state(&t);
    t.set_face_attributes(face, attributes("inner-only"))
        .unwrap();
    changed.restore(&mut t);
    assert_eq!(state(&t), before);
    let after_restore = RollbackSnapshot::capture(&mut t);
    assert!(!Arc::ptr_eq(&outer.0, &after_restore.0));
    outer.restore(&mut t);
    assert_eq!(
        t.attributes().face(face).unwrap().name.as_deref(),
        Some("before")
    );
}

#[test]
fn independent_clone_and_journal_only_changes_do_not_share_snapshots() {
    let (mut t, _, _, _) = fixture();
    let outer = RollbackSnapshot::capture(&mut t);
    let mut copy = t.clone();
    let copied = RollbackSnapshot::capture(&mut copy);
    assert!(!Arc::ptr_eq(&outer.0, &copied.0));
    let pending = t.journal_begin("barrier");
    t.journal_record_barrier(pending, vec![]);
    let changed = RollbackSnapshot::capture(&mut t);
    assert!(!Arc::ptr_eq(&outer.0, &changed.0));
    let mut loaded = t.journal().snapshot();
    loaded.entries.clear();
    t.load_journal(crate::journal::Journal::from_snapshot(loaded).unwrap());
    let reloaded = RollbackSnapshot::capture(&mut t);
    assert!(!Arc::ptr_eq(&changed.0, &reloaded.0));
}

#[test]
fn nested_success_commits_and_expired_snapshots_do_not_retain_documents() {
    fn send_sync<T: Send + Sync>() {}
    let mut t = Topology::new();
    let v = run_transacted(&mut t, |t| {
        run_transacted(t, |t| Ok::<_, TopologyError>(seed(t)))
    })
    .unwrap();
    assert!(t.vertex(v).is_ok());
    let weak = {
        let snapshot = RollbackSnapshot::capture(&mut t);
        Arc::downgrade(&snapshot.0)
    };
    assert!(weak.upgrade().is_none());
    send_sync::<Topology>();
}

#[test]
fn mutable_entity_and_pcurve_access_create_genuine_savepoints() {
    for kind in 0..9 {
        let (mut t, face, solid, _) = fixture();
        let outer = RollbackSnapshot::capture(&mut t);
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
                t.wire_mut(wire).unwrap();
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
        let inner = RollbackSnapshot::capture(&mut t);
        assert!(!Arc::ptr_eq(&outer.0, &inner.0), "mutable access {kind}");
        t.set_face_attributes(face, attributes("inner")).unwrap();
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
