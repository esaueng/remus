//! A cylinder resize can reach the same grouped fillet from two different
//! restoration orders. Both calls start on a newly copied, reindexed solid.
//! Remember one measured refusal for the duration of those plan attempts.

use std::cell::RefCell;

use remus_topology::{EdgeId, SolidId, Topology};

use super::{BlendClusterError, OperationsError};
use crate::measure::volume_memo::{ContentKey, solid_move_key_with_budget};

#[derive(PartialEq, Eq)]
struct Input {
    solid: ContentKey,
    edges: Vec<EdgeId>,
    radius: u64,
}

#[derive(Default)]
struct Attempts {
    last: Option<(Input, BlendClusterError)>,
}

thread_local! {
    static ATTEMPTS: RefCell<Option<Attempts>> = const { RefCell::new(None) };
}

struct Restore(Option<Attempts>);

impl Drop for Restore {
    fn drop(&mut self) {
        ATTEMPTS.with(|slot| slot.replace(self.0.take()));
    }
}

pub(super) fn during<T>(enabled: bool, run: impl FnOnce() -> T) -> T {
    let _restore = Restore(ATTEMPTS.with(|slot| slot.replace(enabled.then(Attempts::default))));
    run()
}

/// Called only on the single solid freshly allocated by `copy_in`. Its
/// canonical allocation order means the identity-sensitive key describes
/// both the geometry and the IDs used in the private refusal.
pub(super) fn run<T>(
    topo: &mut Topology,
    solid: SolidId,
    edges: &[EdgeId],
    radius: f64,
    build: impl FnOnce(&mut Topology, SolidId, &[EdgeId], f64) -> Result<T, OperationsError>,
) -> Result<T, OperationsError> {
    if !ATTEMPTS.with(|slot| slot.borrow().is_some())
        || remus_blend::governor::is_governed()
        || crate::blend_ops::display_only_blends()
        || crate::blend_ops::preview_checks::capturing()
        || !topo.attributes().is_empty()
    {
        return build(topo, solid, edges, radius);
    }
    let Some(content) = solid_move_key_with_budget(topo, solid, Some(512 * 1024)) else {
        return build(topo, solid, edges, radius);
    };
    let input = Input {
        solid: content,
        edges: edges.to_vec(),
        radius: radius.to_bits(),
    };
    if let Some(error) = ATTEMPTS.with(|slot| {
        let attempts = slot.borrow();
        let (previous, error) = attempts.as_ref()?.last.as_ref()?;
        (previous == &input).then(|| error.clone())
    }) {
        let _hit = crate::performance::span("blend_group.reused_refusal");
        return Err(error.into());
    }
    let result = build(topo, solid, edges, radius);
    if let Err(OperationsError::BlendCluster(error @ BlendClusterError::ReblendFailed { .. })) =
        &result
    {
        ATTEMPTS.with(|slot| {
            if let Some(attempts) = &mut *slot.borrow_mut() {
                attempts.last = Some((input, error.clone()));
            }
        });
    }
    result
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use std::cell::Cell;

    fn box_copy(size: f64) -> (Topology, SolidId, Vec<EdgeId>) {
        let mut source = Topology::new();
        let solid = crate::primitives::make_box(&mut source, size, 3.0, 4.0).unwrap();
        let (copy, map, _) = super::super::copy_in(&source, solid).unwrap();
        let edges = remus_topology::explorer::solid_edges(&copy, map.solid).unwrap();
        (copy, map.solid, edges)
    }

    fn attempt(
        work: &mut Topology,
        solid: SolidId,
        edges: &[EdgeId],
        radius: f64,
        calls: &Cell<usize>,
    ) -> String {
        let result: Result<(), OperationsError> = run(work, solid, edges, radius, |_, _, _, _| {
            calls.set(calls.get() + 1);
            Err(BlendClusterError::ReblendFailed {
                reason: format!("private attempt {}", calls.get()),
            }
            .into())
        });
        match result.unwrap_err() {
            OperationsError::BlendCluster(BlendClusterError::ReblendFailed { reason }) => {
                Some(reason)
            }
            _ => None,
        }
        .expect("lost typed group refusal")
    }

    #[test]
    fn shares_only_the_same_private_input_within_one_resize() {
        let calls = Cell::new(0);
        let (mut first, solid, edges) = box_copy(2.0);
        let (mut identical, second, other_edges) = box_copy(2.0);
        let (mut different, third, changed_edges) = box_copy(2.25);
        during(true, || {
            assert_eq!(
                attempt(&mut first, solid, &edges[..2], 1.0, &calls),
                "private attempt 1"
            );
            assert_eq!(
                attempt(&mut identical, second, &other_edges[..2], 1.0, &calls),
                "private attempt 1"
            );
            assert_eq!(
                attempt(&mut different, third, &changed_edges[..2], 1.0, &calls),
                "private attempt 2"
            );
            let reversed = [changed_edges[1], changed_edges[0]];
            assert_eq!(
                attempt(&mut different, third, &reversed, 1.0, &calls),
                "private attempt 3"
            );
            assert_eq!(
                attempt(&mut different, third, &reversed, 1.5, &calls),
                "private attempt 4"
            );
        });
        assert_eq!(
            attempt(
                &mut different,
                third,
                &[changed_edges[1], changed_edges[0]],
                1.5,
                &calls
            ),
            "private attempt 5"
        );
    }

    #[test]
    fn replaying_a_refusal_cannot_skip_observed_blend_work() {
        let calls = Cell::new(0);
        let (mut work, solid, edges) = box_copy(2.0);
        let mut attempt = || attempt(&mut work, solid, &edges[..1], 1.0, &calls);
        during(true, || {
            assert_eq!(attempt(), "private attempt 1");
            let governed = remus_blend::governor::governed(None, usize::MAX, &mut attempt);
            assert_eq!(governed.result, "private attempt 2");
            assert_eq!(
                crate::blend_ops::preview_checks::capture(&mut attempt).0,
                "private attempt 3"
            );
            assert_eq!(
                crate::blend_ops::display_only_scope(&mut attempt),
                "private attempt 4"
            );
            assert_eq!(during(false, &mut attempt), "private attempt 5");
            assert_eq!(attempt(), "private attempt 1");
        });
    }
}
