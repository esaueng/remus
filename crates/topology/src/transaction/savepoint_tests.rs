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
    // A clone never shares rollback storage with its source: it records the
    // source's lineage (for same-lineage foreign restores) and starts empty.
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
        assert!(t.undo_record_count() > 0);
        (snapshot, w)
    };
    assert!(
        t.vertex(committed.1).is_ok(),
        "dropped scope committed its work"
    );
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
