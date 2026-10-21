//! Typed F7 refusals share the direct/batch contract and preserve live state.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::transaction_tests::{assert_stale, observe, slot_marks};
use crate::error::StructuredWasmError;
use crate::kernel::BrepKernel;
use serde_json::{Value, json};

fn setup() -> (BrepKernel, u32) {
    let mut k = BrepKernel::new();
    let result: Vec<Value> = serde_json::from_str(&k.execute_batch(r#"[
        {"op":"makeCylinder","args":{"radius":1,"height":1}},
        {"op":"makeBox","args":{"width":1,"height":1,"depth":1}},
        {"op":"booleanWithQuality","args":{"solidA":1,"solidB":0,"operation":"fuse","exactOnly":true}}
    ]"#)).unwrap();
    assert_eq!(result[2]["ok"]["quality"], "exact");
    let solid = u32::try_from(result[2]["ok"]["solid"].as_u64().unwrap()).unwrap();
    let control = k.make_box_solid(3.0, 3.0, 3.0).unwrap();
    k.offset_journaled_js(control, 0.1).unwrap();
    k.assembly_new("retained assembly").unwrap();
    let sketch = k.sketch_new().unwrap();
    k.sketch_add_point(sketch, 1.0, 2.0, false).unwrap();
    k.gcs_new().unwrap();
    k.checkpoint().unwrap();
    (k, solid)
}

#[test]
fn offset_assembly_refusal_has_direct_and_both_batch_parity() {
    for surface in ["direct", "executeBatch", "executeBatchV2"] {
        for operation in [
            "offsetSolid",
            "offsetSolidV2",
            "offsetDetailed",
            "offsetJournaled",
        ] {
            let (mut k, solid) = setup();
            let before = observe(&k);
            let marks = slot_marks(k.topo());
            if surface == "direct" && operation == "offsetDetailed" {
                let result =
                    serde_json::to_value(k.offset_detailed_impl(solid, -0.25, false)).unwrap();
                assert_eq!(result["status"], "error");
                assert_eq!(result["category"], "internal");
                assert!(
                    result["details"]["message"]
                        .as_str()
                        .unwrap()
                        .contains("wire loop walk")
                );
            } else if surface == "direct" && operation != "offsetJournaled" {
                // Both exports delegate to this fallible body. Actual FFI
                // error objects are exercised by the packaged JS witness.
                let error =
                    StructuredWasmError::from(k.offset_solid_impl(solid, -0.25).unwrap_err());
                let (code, category, details) = error.into_direct_parts();
                assert_eq!(code, "operation_failed");
                assert_eq!(category, "internal");
                assert!(
                    details["message"]
                        .as_str()
                        .unwrap()
                        .contains("wire loop walk")
                );
            } else if surface != "direct" {
                let request =
                    json!([{ "op": operation, "args": { "solid":solid, "distance":-0.25 } }]);
                let response = if surface == "executeBatch" {
                    k.execute_batch(&request.to_string())
                } else {
                    k.execute_batch_v2(&request.to_string())
                };
                let rows: Vec<Value> = serde_json::from_str(&response).unwrap();
                if operation == "offsetDetailed" {
                    assert_eq!(rows[0]["ok"]["status"], "error");
                    assert_eq!(rows[0]["ok"]["category"], "internal");
                } else if surface == "executeBatchV2" {
                    assert_eq!(rows[0]["error"]["code"], "operation_failed");
                    assert_eq!(rows[0]["error"]["category"], "internal");
                    assert!(
                        rows[0]["error"]["message"]
                            .as_str()
                            .unwrap()
                            .contains("wire loop walk")
                    );
                } else {
                    assert!(
                        rows[0]["error"]
                            .as_str()
                            .unwrap()
                            .contains("assembly failed")
                    );
                    assert!(
                        !rows[0]["error"]
                            .as_str()
                            .unwrap()
                            .starts_with("invalid input")
                    );
                }
            } else {
                // A direct JsError cannot be constructed on native targets.
                continue;
            }
            assert_eq!(observe(&k), before, "{surface}/{operation}");
            let after = slot_marks(k.topo());
            assert!(after.iter().zip(marks).any(|(a, b)| *a > b));
            assert_stale(k.topo(), marks, after);
            assert_eq!(k.checkpoint_count(), 1);
        }
    }
}

#[test]
fn invalid_offset_distance_keeps_caller_error_classification() {
    let (mut k, solid) = setup();
    let before = observe(&k);
    for distance in [0.0, f64::NAN, f64::INFINITY] {
        let error = StructuredWasmError::from(k.offset_solid_impl(solid, distance).unwrap_err());
        let (code, category, _) = error.into_direct_parts();
        assert_eq!(code, "invalid_argument");
        assert_eq!(category, "invalid_input");
        assert_eq!(observe(&k), before);
    }
    for v2 in [false, true] {
        let request = json!([{ "op":"offsetSolidV2", "args": { "solid":solid, "distance":0 } }]);
        let response = if v2 {
            k.execute_batch_v2(&request.to_string())
        } else {
            k.execute_batch(&request.to_string())
        };
        let rows: Vec<Value> = serde_json::from_str(&response).unwrap();
        if v2 {
            assert_eq!(rows[0]["error"]["code"], "invalid_argument");
        } else {
            assert!(rows[0]["error"].as_str().unwrap().contains("invalid input"));
        }
        assert_eq!(observe(&k), before);
    }
}
