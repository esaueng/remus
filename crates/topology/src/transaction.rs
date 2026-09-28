//! Transactional topology mutation (RFC 0002; kernel operation contract).
//!
//! The kernel's transaction model is *stage → validate → commit / roll
//! back*: an operation builds new topology in place, and if it fails — or
//! its result fails validation — the pre-operation state is restored
//! exactly. A failed operation never exposes partial topology.
//!
//! Guarantees, inherited from the undo log (see
//! [`Topology::undo_rewind_scope`](crate::Topology::undo_rewind_scope)):
//!
//! - **Atomicity**: on failure, every entity allocated by the operation is
//!   retired, every retirement it staged is undone, and every other
//!   mutation is undone; live entity counts and contents match the
//!   pre-operation state.
//! - **Handle safety**: handles issued before the transaction remain valid
//!   after a rollback; handles allocated inside a rolled-back transaction
//!   fail typed lookups permanently and can never alias a later entity
//!   (arena slots are high-water preserved, never reused).
//!
//! Rollback storage is mutation-local (PERF-T02): capturing a scope is O(1)
//! and records scale with state the scope actually wrote, never with
//! unrelated document size. No scope clones the document for its entry
//! state. Dropping a snapshot commits its scope (records merge into the
//! enclosing scope, or are released when the last scope closes); the merge
//! of a dropped scope is applied lazily on the next transaction access.
//!
//! These free functions are the standard implementation; ad-hoc
//! snapshot/restore pairs in operation code should migrate onto them so
//! the contract has one implementation to audit.

use crate::Topology;
use crate::topology::undo_log::UndoMark;

/// A transaction scope over a [`Topology`].
///
/// Coordinates a host's rollback boundary with nested native transactions.
/// Dropping a snapshot commits that scope (lazily merged on next access);
/// [`Self::restore`] rolls it back; [`Self::commit`] commits it eagerly.
/// Unlike a user checkpoint, rollback undoes retirements too. Host-owned
/// state outside `Topology` is not captured.
///
/// A snapshot must be restored into the same [`Topology`] value it was
/// captured from: the rollback records live in that value, not in the
/// snapshot. Cloning the topology starts independent coordination, so a
/// snapshot never follows its value across a clone.
#[derive(Debug)]
#[must_use]
pub struct RollbackSnapshot {
    mark: UndoMark,
    generation: u64,
    /// Liveness token for the scope entry in the topology's scope stack;
    /// dropping the snapshot lets the entry be reclaimed lazily (commit).
    /// Never read directly: its strong count is the liveness signal.
    #[allow(dead_code)]
    alive: std::sync::Arc<()>,
}

impl RollbackSnapshot {
    /// Capture this scope's entry state. Capture is O(1): no document data
    /// is copied, whatever the nesting depth or document size.
    pub fn capture(topo: &mut Topology) -> Self {
        let ticket = topo.undo_begin_scope();
        Self {
            mark: ticket.mark,
            generation: topo.undo_generation(),
            alive: ticket.alive,
        }
    }

    /// Restore live state while preserving allocation and journal high-water
    /// marks. Pre-existing handles survive; failed allocations stay stale.
    pub fn restore(self, topo: &mut Topology) {
        if topo.undo_generation() != self.generation {
            // A foreign-lineage restore swapped state underneath this scope
            // and invalidated its mark; there is no entry state to return
            // to. No such interleaving exists in the audited call graph
            // (every foreign restore uses a same-lineage snapshot), so this
            // is unreachable through the transaction API.
            return;
        }
        topo.undo_rewind_scope(self.mark);
    }

    /// Commit this scope eagerly: its records merge into the enclosing
    /// scope, or are released when the last scope closes. Dropping the
    /// snapshot without restoring commits lazily with the same effect.
    pub fn commit(self, topo: &mut Topology) {
        topo.undo_commit_scope(self.mark);
    }
}

/// Which storage path an append-only scope took (PERF-T03).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AppendPath {
    /// The scope wrote only new content and committed without copying
    /// unrelated state.
    AppendOnly,
    /// The scope attempted a pre-existing write, was rewound at O(new
    /// content), and the operation was re-executed under the full
    /// transaction path. The result is identical; only the cost differs.
    FullFallback,
}

/// Runs `operation` transactionally: on `Err`, the topology is restored to
/// its pre-operation state (including handle-slot high-water marks) before
/// the error is returned.
///
/// # Errors
///
/// Returns `operation`'s error unchanged after rolling back.
pub fn run_transacted<T, E>(
    topo: &mut Topology,
    operation: impl FnOnce(&mut Topology) -> Result<T, E>,
) -> Result<T, E> {
    let snapshot = RollbackSnapshot::capture(topo);
    match operation(topo) {
        Ok(value) => {
            snapshot.commit(topo);
            Ok(value)
        }
        Err(error) => {
            snapshot.restore(topo);
            Err(error)
        }
    }
}

/// Runs `operation` transactionally and validates its result before
/// committing: if either the operation or `validate` fails, the topology is
/// restored to its pre-operation state.
///
/// `validate` sees the post-operation topology and the operation's value;
/// returning `Err` vetoes the commit. This is the *stage → validate →
/// commit / roll back* contract in one call.
///
/// # Errors
///
/// Returns the operation's or the validator's error unchanged after
/// rolling back.
pub fn run_validated<T, E>(
    topo: &mut Topology,
    operation: impl FnOnce(&mut Topology) -> Result<T, E>,
    validate: impl FnOnce(&Topology, &T) -> Result<(), E>,
) -> Result<T, E> {
    let snapshot = RollbackSnapshot::capture(topo);
    let result = operation(topo).and_then(|value| validate(topo, &value).map(|()| value));
    match result {
        Ok(value) => {
            snapshot.commit(topo);
            Ok(value)
        }
        Err(error) => {
            snapshot.restore(topo);
            Err(error)
        }
    }
}

/// Runs a constructor that is expected to only allocate, with a guard
/// against writes to pre-existing state.
///
/// While the guard is armed, any write to a pre-existing slot, any
/// destruction of a pre-existing index or attribute entry, and any wholesale
/// history replacement trips the guard *before* the write lands. A tripped
/// (or failed) scope is rewound — retiring every allocation it made without
/// reissuing IDs — and, when tripped, the operation is re-executed once
/// under the full [`run_transacted`] path, returning
/// [`AppendPath::FullFallback`]. A clean scope commits at O(new content)
/// and returns [`AppendPath::AppendOnly`].
///
/// The operation must be re-runnable (pure construction): a tripped scope
/// runs it twice. Its observable result is identical on either path; only
/// the transaction cost differs.
///
/// # Errors
///
/// Returns `operation`'s error unchanged after rolling back. Guard trips
/// never surface: they resolve into the fallback retry internally.
pub fn run_append_only<T, E>(
    topo: &mut Topology,
    mut operation: impl FnMut(&mut Topology) -> Result<T, E>,
) -> Result<(T, AppendPath), E> {
    let snapshot = RollbackSnapshot::capture(topo);
    topo.arm_append();
    match operation(topo) {
        Ok(value) => {
            let tripped = topo.disarm_append();
            if tripped {
                snapshot.restore(topo);
                retry_full(topo, &mut operation)
            } else {
                snapshot.commit(topo);
                Ok((value, AppendPath::AppendOnly))
            }
        }
        Err(error) => {
            let tripped = topo.disarm_append();
            snapshot.restore(topo);
            if tripped {
                retry_full(topo, &mut operation)
            } else {
                Err(error)
            }
        }
    }
}

/// Re-executes a tripped append-only operation under the full path, with the
/// guard disarmed so pre-existing writes proceed with mutation-local undo.
fn retry_full<T, E>(
    topo: &mut Topology,
    operation: &mut impl FnMut(&mut Topology) -> Result<T, E>,
) -> Result<(T, AppendPath), E> {
    let snapshot = RollbackSnapshot::capture(topo);
    match operation(topo) {
        Ok(value) => {
            snapshot.commit(topo);
            Ok((value, AppendPath::FullFallback))
        }
        Err(error) => {
            snapshot.restore(topo);
            Err(error)
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_math::vec::Point3;

    use crate::TopologyError;
    use crate::edge::{Edge, EdgeCurve};
    use crate::vertex::Vertex;

    use super::*;

    pub(super) fn seed(topo: &mut Topology) -> crate::VertexId {
        topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7))
    }

    #[test]
    fn success_commits_the_mutation() {
        let mut topo = Topology::new();
        let v = run_transacted(&mut topo, |topo| Ok::<_, TopologyError>(seed(topo))).unwrap();
        assert!(topo.vertex(v).is_ok());
        assert_eq!(topo.num_vertices(), 1);
    }

    #[test]
    fn failure_rolls_back_and_retires_new_handles() {
        let mut topo = Topology::new();
        let pre_existing = seed(&mut topo);
        let slots_before = topo.allocated_slot_count();

        let mut leaked = None;
        let err = run_transacted(&mut topo, |topo| {
            let v = seed(topo);
            let e = topo.add_edge(Edge::new(pre_existing, v, EdgeCurve::Line));
            leaked = Some((v, e));
            Err::<(), _>(TopologyError::WireNotClosed)
        })
        .unwrap_err();
        assert!(matches!(err, TopologyError::WireNotClosed));

        // Pre-existing handles survive; live counts match the pre-state.
        assert!(topo.vertex(pre_existing).is_ok());
        assert_eq!(topo.num_vertices(), 1);
        assert_eq!(topo.num_edges(), 0);

        // Handles allocated inside the rolled-back transaction fail typed
        // lookups and can never alias a later entity: new allocations land
        // above the preserved high-water mark.
        let (v, e) = leaked.unwrap();
        assert!(topo.vertex(v).is_err());
        assert!(topo.edge(e).is_err());
        assert!(topo.allocated_slot_count() >= slots_before);
        let fresh = seed(&mut topo);
        assert_ne!(fresh, v, "a rolled-back slot must never be reissued");
    }

    #[test]
    fn validation_veto_rolls_back() {
        let mut topo = Topology::new();
        let err = run_validated(
            &mut topo,
            |topo| Ok::<_, TopologyError>(seed(topo)),
            |topo, v| {
                assert!(topo.vertex(*v).is_ok(), "validator sees the staged state");
                Err(TopologyError::WireNotClosed)
            },
        )
        .unwrap_err();
        assert!(matches!(err, TopologyError::WireNotClosed));
        assert_eq!(topo.num_vertices(), 0, "vetoed commit leaves no topology");
    }

    #[test]
    fn validation_pass_commits() {
        let mut topo = Topology::new();
        let v = run_validated(
            &mut topo,
            |topo| Ok::<_, TopologyError>(seed(topo)),
            |_, _| Ok(()),
        )
        .unwrap();
        assert!(topo.vertex(v).is_ok());
    }

    pub(super) fn triangle_face(topo: &mut Topology) -> crate::FaceId {
        use crate::edge::EdgeCurve;
        use crate::face::{Face, FaceSurface};
        use crate::wire::{OrientedEdge, Wire};
        use remus_math::vec::Vec3;

        let v0 = seed(topo);
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        let wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e2, true),
                ],
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    #[test]
    fn failure_undoes_an_in_window_rederivation() {
        // A re-derivation retires the face's previous loops. Rolled back,
        // the retirement is undone: the original handles resolve again and
        // the derivation map matches the pre-transaction state exactly.
        let mut topo = Topology::new();
        let face = triangle_face(&mut topo);
        let original = topo.build_face_loops(face).unwrap();
        let original_coedges = topo.face_loop(original[0]).unwrap().coedges().to_vec();

        let err = run_transacted(&mut topo, |topo| {
            topo.build_face_loops(face)?;
            Err::<(), _>(TopologyError::WireNotClosed)
        })
        .unwrap_err();
        assert!(matches!(err, TopologyError::WireNotClosed));

        assert_eq!(topo.num_loops(), 1);
        assert_eq!(topo.num_coedges(), 3);
        assert_eq!(
            topo.loops_of_face(face),
            Some(original.as_slice()),
            "rollback must restore the original derivation"
        );
        assert!(topo.face_loop(original[0]).is_ok());
        for coedge_id in &original_coedges {
            assert!(topo.coedge(*coedge_id).is_ok());
        }
        crate::validation::validate_face_loops(&topo, face).unwrap();
    }

    #[test]
    fn compound_and_compsolid_rollback_keeps_allocations_stale() {
        use crate::compound::Compound;
        use crate::compsolid::CompSolid;

        // Compounds have no mutation or retirement API (immutable after
        // construction), so their undo coverage is allocation rollback plus
        // the overwrite-record round trip through the `*_mut` accessors.
        let mut topo = Topology::new();
        let solid = topo.add_empty_solid();
        let compound = topo.add_compound(Compound::new(vec![solid]));
        let compsolid = topo.add_compsolid(CompSolid::new(vec![solid], vec![]));
        let slots_before = topo.allocated_slot_count();

        let mut leaked = None;
        let err = run_transacted(&mut topo, |topo| {
            let staged_compound = topo.add_compound(Compound::new(vec![solid]));
            let staged_compsolid = topo.add_compsolid(CompSolid::new(vec![solid], vec![]));
            // Overwrite records execute (nothing observable to write back —
            // the entities are immutable — but the guard and log paths run).
            topo.compound_mut(compound)?;
            topo.compsolid_mut(compsolid)?;
            leaked = Some((staged_compound, staged_compsolid));
            Err::<(), _>(TopologyError::WireNotClosed)
        })
        .unwrap_err();
        assert!(matches!(err, TopologyError::WireNotClosed));

        // Pre-existing handles survive; staged allocations stay stale and
        // later allocations append above the preserved high-water mark.
        assert!(topo.compound(compound).is_ok());
        assert!(topo.compsolid(compsolid).is_ok());
        let (staged_compound, staged_compsolid) = leaked.unwrap();
        assert!(topo.compound(staged_compound).is_err());
        assert!(topo.compsolid(staged_compsolid).is_err());
        assert!(topo.allocated_slot_count() >= slots_before);
        let fresh = topo.add_compound(Compound::new(vec![solid]));
        assert_ne!(
            fresh, staged_compound,
            "a rolled-back slot must never be reissued"
        );
        assert!(topo.compound(staged_compound).is_err());
    }
    #[test]
    fn failure_undoes_an_in_window_deletion() {
        // delete_solid retires a pre-existing tree. Rolled back, the solid
        // and its shell resolve again — the failure was never observed.
        let mut topo = Topology::new();
        let solid = topo.add_empty_solid();

        let err = run_transacted(&mut topo, |topo| {
            topo.delete_solid(solid).map_err(|_| TopologyError::Empty {
                entity: "delete in test",
            })?;
            Err::<(), _>(TopologyError::WireNotClosed)
        })
        .unwrap_err();
        assert!(matches!(err, TopologyError::WireNotClosed));

        assert!(topo.solid(solid).is_ok());
        assert_eq!(topo.num_solids(), 1);
        assert_eq!(topo.num_shells(), 1);
    }
}

#[cfg(test)]
#[path = "transaction/savepoint_tests.rs"]
mod savepoint_tests;
