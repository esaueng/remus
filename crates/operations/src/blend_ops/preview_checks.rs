//! Private, immutable obligations for adopting a display-only group build.
//!
//! Capture the exact inputs of the measuring guards, rather than substituting
//! a weaker whole-group check for the individual engine checks. If a failed
//! attempt ran a deferred guard, full validation might choose another engine;
//! that preview cannot be adopted and commit uses its ordinary route.

use std::cell::{Cell, RefCell};

use remus_topology::{SolidId, Topology};

use super::{BlendBaseline, BlendSize, ConvexMaterialGuard, validate_blend_volume};
use crate::OperationsError;

thread_local! {
    static CAPTURE: RefCell<Option<Checks>> = const { RefCell::new(None) };
}

#[derive(Default)]
pub struct Checks {
    obligations: Vec<Check>,
    branched: bool,
    validated: Cell<bool>,
}

enum Check {
    Volume {
        topo: Box<Topology>,
        input: SolidId,
        result: SolidId,
        operation: &'static str,
        edges: Vec<remus_topology::EdgeId>,
        size: BlendSize,
    },
    Material {
        pristine: Box<Topology>,
        live: Box<Topology>,
        input: SolidId,
        result: SolidId,
        edges: Vec<remus_topology::EdgeId>,
        radius: f64,
    },
    Group {
        topo: Box<Topology>,
        result: SolidId,
    },
}

/// Capture one private build, restoring the caller's capture on unwind too.
pub fn capture<T>(run: impl FnOnce() -> T) -> (T, Option<Checks>) {
    struct Restore(Option<Checks>);
    impl Drop for Restore {
        fn drop(&mut self) {
            CAPTURE.with(|slot| *slot.borrow_mut() = self.0.take());
        }
    }
    let _restore = Restore(CAPTURE.with(|slot| slot.replace(Some(Checks::default()))));
    let result = run();
    let checks = CAPTURE
        .with(RefCell::take)
        .filter(|checks| !checks.branched);
    (result, checks)
}

fn record(check: impl FnOnce() -> Check) {
    CAPTURE.with(|slot| {
        if let Some(checks) = slot.borrow_mut().as_mut().filter(|checks| !checks.branched) {
            checks.obligations.push(check());
        }
    });
}

pub(super) fn volume(
    baseline: &BlendBaseline<'_>,
    operation: &'static str,
    result: SolidId,
    edges: &[remus_topology::EdgeId],
    size: BlendSize,
) {
    record(|| Check::Volume {
        topo: Box::new(baseline.topo.clone()),
        input: baseline.input,
        result,
        operation,
        edges: edges.to_vec(),
        size,
    });
}

pub(super) fn active() -> bool {
    CAPTURE.with(|slot| {
        slot.borrow()
            .as_ref()
            .is_some_and(|checks| !checks.branched)
    })
}

fn mark() -> Option<usize> {
    CAPTURE.with(|slot| {
        slot.borrow()
            .as_ref()
            .map(|checks| checks.obligations.len())
    })
}

fn declined(mark: Option<usize>) {
    CAPTURE.with(|slot| {
        if let (Some(mark), Some(checks)) = (mark, slot.borrow_mut().as_mut()) {
            checks.branched |= checks.obligations.len() != mark;
            checks.obligations.truncate(mark);
        }
    });
}

/// Observe an optional fallback without changing its transaction semantics.
/// A declined route that deferred a guard cannot certify a later candidate.
pub fn optional<T, E>(run: impl FnOnce() -> Result<Option<T>, E>) -> Result<Option<T>, E> {
    let mark = mark();
    let result = run();
    if !matches!(&result, Ok(Some(_))) {
        declined(mark);
    }
    result
}

pub(super) fn material(
    pristine: Topology,
    live: &Topology,
    input: SolidId,
    result: SolidId,
    edges: &[remus_topology::EdgeId],
    radius: f64,
) {
    record(|| Check::Material {
        pristine: Box::new(pristine),
        live: Box::new(live.clone()),
        input,
        result,
        edges: edges.to_vec(),
        radius,
    });
}

pub fn group(topo: &Topology, result: SolidId) {
    record(|| Check::Group {
        topo: Box::new(topo.clone()),
        result,
    });
}

/// Observe failed speculative attempts. A failure before any measuring guard
/// has the same outcome with full guards; a failure after one is ambiguous.
pub fn transaction<T, E>(
    topo: &mut Topology,
    run: impl FnOnce(&mut Topology) -> Result<T, E>,
) -> Result<T, E> {
    let mark = mark();
    let result = remus_topology::transaction::run_transacted(topo, run);
    if result.is_err() {
        declined(mark);
    }
    result
}

impl Checks {
    /// Run every guard omitted by the frame. No live arena or journal writes.
    pub(crate) fn validate(&self) -> Result<(), OperationsError> {
        if self.validated.get() {
            return Ok(());
        }
        let _span = crate::performance::span("blend_group.preview_checks");
        crate::classify::mesh_memo::scoped(|| {
            crate::measure::volume_memo::scoped(|| self.validate_uncached())
        })?;
        self.validated.set(true);
        Ok(())
    }

    fn validate_uncached(&self) -> Result<(), OperationsError> {
        for check in &self.obligations {
            match check {
                Check::Volume {
                    topo,
                    input,
                    result,
                    operation,
                    edges,
                    size,
                } => {
                    let mut baseline = BlendBaseline::new(topo, *input);
                    validate_blend_volume(&mut baseline, operation, *result, edges, *size)?;
                }
                Check::Material {
                    pristine,
                    live,
                    input,
                    result,
                    edges,
                    radius,
                } => {
                    let before: Vec<_> = remus_topology::explorer::solid_faces(pristine, *input)?
                        .into_iter()
                        .filter(|&face| {
                            pristine.face(face).is_ok_and(|face| {
                                matches!(
                                    face.surface(),
                                    remus_topology::face::FaceSurface::Nurbs(_)
                                )
                            })
                        })
                        .collect();
                    let has_new_nurbs = remus_topology::explorer::solid_faces(live, *result)?
                        .into_iter()
                        .any(|face| {
                            !before.contains(&face)
                                && live.face(face).is_ok_and(|face| {
                                    matches!(
                                        face.surface(),
                                        remus_topology::face::FaceSurface::Nurbs(_)
                                    )
                                })
                        });
                    if has_new_nurbs
                        && let Some(guard) =
                            ConvexMaterialGuard::capture(pristine, *input, edges, *radius)?
                    {
                        guard.check(live, "fillet", *result)?;
                    }
                }
                Check::Group { topo, result } => {
                    crate::blend_cluster::reblend::measuring_postconditions(topo, *result)?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn failed_attempt_after_a_deferred_guard_disables_adoption() {
        let ((), checks) = capture(|| {
            let mut topo = Topology::new();
            let solid = crate::primitives::make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
            let _: Result<(), ()> = transaction(&mut topo, |topo| {
                group(topo, solid);
                Err(())
            });
        });
        assert!(checks.is_none());
        assert!(!active());
    }

    #[test]
    fn early_structural_refusal_and_nested_capture_restore_the_outer_scope() {
        let ((), checks) = capture(|| {
            let mut topo = Topology::new();
            let _: Result<(), ()> = transaction(&mut topo, |_| Err(()));
            let ((), inner) = capture(|| {});
            assert!(inner.is_some());
            assert!(active());
        });
        assert!(checks.is_some());
        assert!(!active());
    }

    #[test]
    fn optional_decline_after_a_guard_disables_adoption() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
        let (result, checks) = capture(|| {
            optional::<(), ()>(|| {
                group(&topo, solid);
                Ok(None)
            })
        });
        assert!(result.unwrap().is_none());
        assert!(checks.is_none());
        let (result, checks) = capture(|| optional::<(), ()>(|| Ok(None)));
        assert!(result.unwrap().is_none());
        assert!(checks.is_some());
    }
}
