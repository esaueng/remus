//! Differential oracle for the modifier twins' rollback envelope: every twin
//! runs through the rollback scope and through the full-copy envelope it
//! replaced, kept verbatim below, on identical kernels (tests).

#![allow(clippy::unwrap_used)]

use std::cell::Cell;

use serde_json::{Map, Value};

use crate::bindings::transaction_tests::{break_fixed_point, high_water};
use crate::error::StructuredWasmError;
use crate::handles::{edge_id_to_u32, face_id_to_u32};
use crate::helpers::panic_message;
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

thread_local! {
    /// Routes `run_modifier_detailed` through
    /// [`BrepKernel::legacy_run_modifier_detailed`].
    pub(super) static LEGACY_ENVELOPE: Cell<bool> = const { Cell::new(false) };
}

impl BrepKernel {
    /// `run_modifier_detailed` before it moved onto a rollback scope,
    /// verbatim.
    pub(super) fn legacy_run_modifier_detailed(
        &mut self,
        operation: &'static str,
        panic_label: &str,
        body: impl FnOnce(&mut Self) -> Result<(u32, Map<String, Value>), StructuredWasmError>,
    ) -> SolidOperationDetailedResult {
        if self.poisoned {
            return SolidOperationDetailedResult::error(
                StructuredWasmError::operation_failed(
                    "Kernel poisoned after panic. Create a new BrepKernel instance.",
                )
                .with_direct_operation(operation),
            );
        }
        let snapshot = self.topo().clone();
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| body(self)));
        match outcome {
            Ok(Ok((value, details))) => {
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Ok(Err(error)) => {
                self.topo_mut().restore_for_rollback(&snapshot);
                SolidOperationDetailedResult::error(error.with_direct_operation(operation))
            }
            Err(panic_info) => {
                self.poisoned = true;
                self.topo_mut().restore_for_rollback(&snapshot);
                SolidOperationDetailedResult::error(
                    StructuredWasmError::operation_failed(panic_message(&panic_info, panic_label))
                        .with_direct_operation(operation),
                )
            }
        }
    }
}

type Call = fn(&mut BrepKernel, u32, u32, u32) -> SolidOperationDetailedResult;

/// `(name, twin call)` over a solid, one of its edges and one of its faces:
/// refusals that run the engine (and roll back) before and after a success.
fn calls() -> Vec<(&'static str, Call)> {
    vec![
        ("fillet refused", |k, s, e, _| {
            k.fillet_detailed_impl(s, &[e], 50.0, false)
        }),
        ("chamfer refused", |k, s, e, _| {
            k.chamfer_detailed_impl(s, &[e], 50.0, false)
        }),
        ("shell refused", |k, s, _, f| {
            k.shell_detailed_impl(s, 20.0, &[f], None)
        }),
        ("offset refused", |k, s, _, _| {
            k.offset_detailed_impl(s, -20.0, false)
        }),
        ("filletV2 refused", |k, s, e, _| {
            k.fillet_v2_detailed_impl(s, &[e], 50.0, false)
        }),
        ("chamferV2 refused", |k, s, e, _| {
            k.chamfer_v2_detailed_impl(s, &[e], 50.0, 40.0, false)
        }),
        ("chamferDistanceAngle refused", |k, s, e, _| {
            k.chamfer_distance_angle_detailed_impl(s, &[e], 50.0, 0.5, false)
        }),
        ("fillet", |k, s, e, _| {
            k.fillet_detailed_impl(s, &[e], 1.0, false)
        }),
        ("fillet refused again", |k, s, e, _| {
            k.fillet_detailed_impl(s, &[e], 50.0, false)
        }),
        ("unknown solid", |k, _, e, _| {
            k.fillet_detailed_impl(99_999, &[e], 1.0, false)
        }),
    ]
}

/// A kernel holding a 10³ box and a smaller one, with the box's first edge
/// and face.
fn kernel() -> (BrepKernel, u32, u32, u32) {
    let mut k = BrepKernel::new();
    let solid = k.make_box_solid(10.0, 10.0, 10.0).unwrap();
    k.make_box_solid(4.0, 4.0, 4.0).unwrap();
    let id = k.resolve_solid(solid).unwrap();
    let edge = edge_id_to_u32(remus_topology::explorer::solid_edges(k.topo(), id).unwrap()[0]);
    let face = face_id_to_u32(remus_topology::explorer::solid_faces(k.topo(), id).unwrap()[0]);
    (k, solid, edge, face)
}

#[test]
fn twin_rollback_matches_the_full_copy_envelope() {
    // On and off the restore fixed point (where the full-copy envelope's
    // exact restore copied the index as it was), with and without a
    // retained checkpoint (so the scope opens on a copy-on-write first
    // write). The complete topology `Debug` (tombstones and the pcurve
    // index included), the high-water marks and the response must match.
    let mut refusals = 0;
    for off_fixed_point in [false, true] {
        for retain_checkpoint in [false, true] {
            let (mut current, solid, edge, face) = kernel();
            let (mut legacy, ..) = kernel();
            if retain_checkpoint {
                current.checkpoint().unwrap();
                legacy.checkpoint().unwrap();
            }
            for (name, call) in calls() {
                if off_fixed_point {
                    break_fixed_point(&mut current);
                    break_fixed_point(&mut legacy);
                }
                let new = serde_json::to_value(call(&mut current, solid, edge, face)).unwrap();
                LEGACY_ENVELOPE.set(true);
                let old = serde_json::to_value(call(&mut legacy, solid, edge, face)).unwrap();
                LEGACY_ENVELOPE.set(false);
                let context = format!(
                    "{name} (off the fixed point {off_fixed_point}, checkpoint {retain_checkpoint})"
                );
                assert_eq!(new, old, "{context}");
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
                assert_eq!(current.topo().undo_record_count(), 0, "{context}");
                if new["status"] == "error" {
                    refusals += 1;
                    assert_eq!(
                        current.topo().is_restore_fixed_point(),
                        !off_fixed_point,
                        "{context}: the twins never rebuilt the index"
                    );
                }
            }
        }
    }
    assert!(refusals >= 4 * 8, "{refusals}");
}
