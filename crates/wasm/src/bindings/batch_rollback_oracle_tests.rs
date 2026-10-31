//! Differential oracle for the batch rollback scope: every item runs in
//! lockstep through the current dispatcher and the full-copy dispatcher it
//! replaced, kept verbatim below (tests).

#![allow(clippy::unwrap_used)]

use remus_topology::Topology;

use super::{BatchItemResult, BatchOpKind, batch_op_kind};
use crate::bindings::transaction_tests::{
    batch_cases, break_fixed_point, brep_json, fixture, high_water, live_state, observe,
    session_state,
};
use crate::error::WasmError;
use crate::kernel::RemusKernel;

/// `dispatch_with_rollback` before the mutating arm moved onto a rollback
/// scope, verbatim.
fn legacy_dispatch_with_rollback(
    kernel: &mut RemusKernel,
    kind: BatchOpKind,
    op: &str,
    args: &serde_json::Value,
) -> BatchItemResult {
    if matches!(op, "fuse" | "cut" | "intersect") {
        let snapshot = remus_topology::transaction::RollbackSnapshot::capture(kernel.topo_mut());
        let result = kernel.dispatch_op(op, args);
        if result.is_err() {
            snapshot.restore(kernel.topo_mut());
        }
        result
    } else if kind == BatchOpKind::ReadOnly {
        kernel.run_read_only(|kernel| kernel.dispatch_op(op, args))
    } else {
        // Copy aside so `self.topo` stays unshared and the operation
        // mutates it in place, keeping its arena capacity.
        let snapshot = kernel.topo().clone();
        let result = kernel.dispatch_op(op, args);
        if result.is_err() {
            kernel.topo_mut().restore_preserving_handle_slots(&snapshot);
        }
        result
    }
}

/// `with_topology_transaction` before it moved onto a rollback scope,
/// verbatim.
fn legacy_with_topology_transaction<T, E>(
    kernel: &mut RemusKernel,
    operation: impl FnOnce(&mut Topology) -> Result<T, E>,
) -> Result<T, E> {
    let snapshot = kernel.topo().clone();
    let result = operation(kernel.topo_mut());
    if result.is_err() {
        kernel.topo_mut().restore_preserving_handle_slots(&snapshot);
    }
    result
}

/// `from_brep_impl` before it moved onto a rollback scope, verbatim.
fn legacy_from_brep_impl(kernel: &mut RemusKernel, json: &str) -> Result<u32, WasmError> {
    let snapshot = kernel.topo().clone();
    let result = kernel.from_brep_impl_unchecked(json);
    if result.is_err() {
        kernel.topo_mut().restore_preserving_handle_slots(&snapshot);
    }
    result
}

fn outcome(result: &BatchItemResult) -> String {
    match result {
        Ok(value) => format!("ok {value}"),
        Err(error) => format!("error {}", error.message()),
    }
}

#[test]
fn rollback_scope_matches_the_full_copy_dispatcher() {
    // Two kernels with one history; with and without a retained checkpoint,
    // so the scope also opens on a copy-on-write first write.
    for retain_checkpoint in [false, true] {
        let mut current = RemusKernel::new();
        let mut legacy = RemusKernel::new();
        let f = fixture(&mut current);
        fixture(&mut legacy);
        if retain_checkpoint {
            current.checkpoint().unwrap();
            legacy.checkpoint().unwrap();
        }
        let mut refusals = 0;
        for round in 0..2 {
            for (family, ok, bad) in batch_cases(&f) {
                for item in [&bad, &ok, &bad] {
                    let op = item["op"].as_str().unwrap();
                    let kind = batch_op_kind(op).unwrap();
                    let before = observe(&current);
                    let new = current.dispatch_with_rollback(kind, op, &item["args"]);
                    let old = legacy_dispatch_with_rollback(&mut legacy, kind, op, &item["args"]);
                    let context = format!("{family} round {round}: {item}");
                    assert_eq!(outcome(&new), outcome(&old), "{context}");
                    assert_eq!(
                        live_state(current.topo()),
                        live_state(legacy.topo()),
                        "{context}"
                    );
                    assert_eq!(observe(&current), observe(&legacy), "{context}");
                    assert_same_document(&current, &legacy, &context);
                    if new.is_err() {
                        refusals += 1;
                        assert_eq!(observe(&current), before, "{context}");
                    }
                }
            }
        }
        // Every refusal in the corpus that surfaces at the batch level.
        assert!(refusals >= 2 * 2 * 10, "{refusals}");
    }
}

/// Pins the premise of the oracle: refusals in the corpus that mutate the
/// document before failing exist, so the boundary rollback is not vacuous
/// (most operations also roll back natively).
#[test]
fn corpus_refusals_really_dirty_the_document_without_rollback() {
    let mut raw = RemusKernel::new();
    let f = fixture(&mut raw);
    let mut dirty = Vec::new();
    for (family, _, bad) in batch_cases(&f) {
        let op = bad["op"].as_str().unwrap();
        if batch_op_kind(op) != Some(BatchOpKind::Mutating) {
            continue;
        }
        let before = live_state(raw.topo());
        let failed = raw.dispatch_op(op, &bad["args"]).is_err();
        if failed && live_state(raw.topo()) != before {
            dirty.push(family);
        }
    }
    assert_eq!(dirty, ["pushPullFace", "resizeCylindricalFace"]);
}

/// A direct-binding body run through both transaction helpers.
type Body = Box<dyn Fn(&mut Topology) -> Result<(), remus_operations::OperationsError>>;

/// Asserts the two kernels hold the same complete topology (`Debug`,
/// tombstones and the pcurve index included), high-water marks and session
/// state.
fn assert_same_document(current: &RemusKernel, legacy: &RemusKernel, context: &str) {
    assert_eq!(
        format!("{:?}", current.topo()),
        format!("{:?}", legacy.topo()),
        "{context}"
    );
    assert_eq!(
        high_water(current.topo()),
        high_water(legacy.topo()),
        "{context}"
    );
    assert_eq!(session_state(current), session_state(legacy), "{context}");
}

#[test]
fn refusals_off_the_restore_fixed_point_match_the_full_copy_boundary() {
    // Every refusal starts from a document that is not a restore fixed
    // point, where the full-copy boundary's restore rebuilt the pcurve
    // index: the rollback scope must land on that same rebuilt state
    // (direct, batch and fromBREP paths), while the boolean arm, which
    // always rolled back exactly, must keep the entry state as it was.
    for retain_checkpoint in [false, true] {
        let mut current = RemusKernel::new();
        let mut legacy = RemusKernel::new();
        let f = fixture(&mut current);
        fixture(&mut legacy);
        if retain_checkpoint {
            current.checkpoint().unwrap();
            legacy.checkpoint().unwrap();
        }
        let mut rebuilt = 0;
        let read_only =
            serde_json::json!({"op":"volume","args":{"solid":99_999,"deflection":0.05}});
        let mut items: Vec<(&str, serde_json::Value)> = vec![("volume", read_only)];
        for (family, ok, bad) in batch_cases(&f) {
            items.extend([(family, bad.clone()), (family, ok), (family, bad)]);
        }
        for (family, item) in &items {
            break_fixed_point(&mut current);
            break_fixed_point(&mut legacy);
            let op = item["op"].as_str().unwrap();
            let kind = batch_op_kind(op).unwrap();
            let new = current.dispatch_with_rollback(kind, op, &item["args"]);
            let old = legacy_dispatch_with_rollback(&mut legacy, kind, op, &item["args"]);
            let context = format!("{family} (checkpoint {retain_checkpoint}): {item}");
            assert_eq!(outcome(&new), outcome(&old), "{context}");
            assert_same_document(&current, &legacy, &context);
            if new.is_err() {
                // The rebuild ran, except for the exact boolean arm and a
                // read-only item, which left the document untouched.
                let rebuilds =
                    kind == BatchOpKind::Mutating && !matches!(op, "fuse" | "cut" | "intersect");
                assert_eq!(
                    current.topo().is_restore_fixed_point(),
                    rebuilds,
                    "{context}"
                );
                rebuilt += usize::from(rebuilds);
            }
        }
        assert!(rebuilt >= 10, "{rebuilt}");

        // Direct bindings: the same refusing bodies through the verbatim
        // full-copy helper and the rollback scope.
        let small = current.resolve_solid(f.small).unwrap();
        let block = current.resolve_solid(f.block).unwrap();
        let singular = remus_math::mat::Mat4([[0.0; 4]; 4]);
        let bodies: Vec<(&str, Body)> = vec![
            (
                "singular transform",
                Box::new(move |topo| {
                    remus_operations::transform::transform_solid(topo, small, &singular)
                }),
            ),
            (
                "injected fault",
                Box::new(move |topo| {
                    remus_operations::primitives::make_box(topo, 1.0, 2.0, 3.0)?;
                    remus_operations::transform::transform_solid(
                        topo,
                        small,
                        &remus_math::mat::Mat4::translation(1.0, 0.0, 0.0),
                    )?;
                    topo.set_solid_attributes(
                        block,
                        remus_topology::attributes::EntityAttributes {
                            name: Some("staged".into()),
                            ..Default::default()
                        },
                    )?;
                    let pending = topo.journal_begin("injected");
                    let _ = topo.journal_record_barrier(pending, Vec::new());
                    Err(remus_operations::OperationsError::InvalidInput {
                        reason: "injected".into(),
                    })
                }),
            ),
        ];
        for (name, body) in &bodies {
            break_fixed_point(&mut current);
            break_fixed_point(&mut legacy);
            let new = current.with_topology_transaction(body);
            let old = legacy_with_topology_transaction(&mut legacy, body);
            let context = format!("direct {name} (checkpoint {retain_checkpoint})");
            assert_eq!(format!("{new:?}"), format!("{old:?}"), "{context}");
            assert!(new.is_err(), "{context}");
            assert_same_document(&current, &legacy, &context);
            assert!(current.topo().is_restore_fixed_point(), "{context}");
        }

        // fromBREP.
        for json in [brep_json(true), brep_json(false), brep_json(true)] {
            break_fixed_point(&mut current);
            break_fixed_point(&mut legacy);
            let new = current.from_brep_impl(&json);
            let old = legacy_from_brep_impl(&mut legacy, &json);
            let context = format!("fromBREP (checkpoint {retain_checkpoint})");
            assert_eq!(format!("{new:?}"), format!("{old:?}"), "{context}");
            assert_same_document(&current, &legacy, &context);
        }
    }
}
