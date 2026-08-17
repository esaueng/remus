//! Transactional topology mutation (RFC 0002; kernel operation contract).
//!
//! The kernel's transaction model is *stage → validate → commit / roll
//! back*: an operation builds new topology in place, and if it fails — or
//! its result fails validation — the pre-operation state is restored
//! exactly. A failed operation never exposes partial topology.
//!
//! Guarantees, inherited from
//! [`Topology::restore_preserving_handle_slots`](crate::Topology::restore_preserving_handle_slots):
//!
//! - **Atomicity**: on failure, every entity allocated by the operation is
//!   retired and every mutation is undone; live entity counts and contents
//!   match the pre-operation state.
//! - **Handle safety**: handles issued before the transaction remain valid
//!   after a rollback; handles allocated inside a rolled-back transaction
//!   fail typed lookups permanently and can never alias a later entity
//!   (arena slots are high-water preserved, never reused).
//!
//! Cost: one deep snapshot of the topology per transaction — the same
//! price the WASM batch dispatcher already pays per mutating operation
//! (its `dispatch_with_rollback` is this pattern, plus an `Rc`-sharing
//! fast path for read-only operations). Operations that already isolate
//! their work (e.g. the GFA boolean's shape store) still benefit: the
//! snapshot covers the caller-visible export window too.
//!
//! These free functions are the standard implementation; ad-hoc
//! snapshot/restore pairs in operation code should migrate onto them so
//! the contract has one implementation to audit.

use crate::Topology;

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
    let snapshot = topo.clone();
    match operation(topo) {
        Ok(value) => Ok(value),
        Err(error) => {
            topo.restore_preserving_handle_slots(&snapshot);
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
    let snapshot = topo.clone();
    let result = operation(topo).and_then(|value| validate(topo, &value).map(|()| value));
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use remus_math::vec::Point3;

    use crate::TopologyError;
    use crate::edge::{Edge, EdgeCurve};
    use crate::vertex::Vertex;

    use super::*;

    fn seed(topo: &mut Topology) -> crate::VertexId {
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
}
