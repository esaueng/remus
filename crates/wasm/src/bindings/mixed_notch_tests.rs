//! WASM contract tests for the mixed-notch L-bracket boundary.
//!
//! The concave L-bracket (XY polygon
//! `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]` extruded 20 mm) exercises the
//! kernel's mixed convex/concave corner boundary through the same entry
//! points a JS consumer uses:
//!
//! - whole-edge constant-radius fillet refuses typed with
//!   `unsupported-vertex-blend` on both the legacy string contract and the
//!   `executeBatchV2` structured `kernelCode`, leaving the input (volume
//!   and edge inventory) unchanged;
//! - whole-edge chamfer at distance 1 succeeds with the native closed-form
//!   volume (`12905.3333 mm^3`), proving the refusal is fillet-specific.
//!
//! Fixture construction uses the direct polygon binding (infallible here)
//! plus batch extrude; every fallible step goes through `executeBatch`.
//! The direct `#[wasm_bindgen]` methods are not callable from native tests
//! on failure paths; these batch tests cover the dispatch plus the shared
//! helpers, while `crates/operations/tests/regress_mixed_notch_whole_edge.rs`
//! and `regress_chamfer_mixed_notch_whole_edge.rs` pin the engines natively.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;

fn run(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch(&json)).unwrap()
}

fn run_v2(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch_v2(&json)).unwrap()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

/// Extruded L-bracket fixture (exact task dimensions). Polygon construction
/// uses the direct binding (infallible here, so no off-wasm `JsError`
/// arises); every fallible step below goes through batch.
fn make_l_bracket(kernel: &mut BrepKernel) -> u32 {
    let face = kernel
        .make_polygon(vec![
            0.0, 0.0, 0.0, 40.0, 0.0, 0.0, 40.0, 8.0, 0.0, 8.0, 8.0, 0.0, 8.0, 50.0, 0.0, 0.0,
            50.0, 0.0,
        ])
        .unwrap();
    let out = run(
        kernel,
        &[op(
            "extrude",
            serde_json::json!({"face": face, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": 20.0}),
        )],
    );
    u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap()
}

fn edge_handles(kernel: &mut BrepKernel, solid: u32) -> Vec<u32> {
    let out = run(
        kernel,
        &[op("solidEdges", serde_json::json!({"solid": solid}))],
    );
    out[0]["ok"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| u32::try_from(v.as_u64().unwrap()).unwrap())
        .collect()
}

fn volume(kernel: &mut BrepKernel, solid: u32) -> f64 {
    let out = run(
        kernel,
        &[op(
            "volume",
            serde_json::json!({"solid": solid, "deflection": 0.01}),
        )],
    );
    out[0]["ok"].as_f64().unwrap()
}

/// Whole-edge fillet refuses typed on both contracts; input is unchanged.
#[test]
fn mixed_notch_whole_edge_fillet_refuses_typed_on_both_contracts() {
    let mut kernel = BrepKernel::new();
    let solid = make_l_bracket(&mut kernel);
    let edges = edge_handles(&mut kernel, solid);
    assert_eq!(
        edges.len(),
        18,
        "extruded L must expose 18 edges, got {edges:?}"
    );
    let before_volume = volume(&mut kernel, solid);
    assert!(
        (before_volume - 13120.0).abs() < 1.0,
        "extruded L volume must be 13120, got {before_volume}"
    );
    let before_edges = edges.len();

    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "edges": edges, "radius": 1.0}),
        )],
    );
    let err = out[0]["error"].as_str().unwrap().to_string();
    assert!(
        err.contains("unsupported vertex blend") && err.contains("stripes meet"),
        "legacy refusal must name the vertex-blend cause, got: {err}"
    );

    let out = run_v2(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "edges": edges, "radius": 1.0}),
        )],
    );
    let code = out[0]["error"]["details"]["kernelCode"]
        .as_str()
        .unwrap()
        .to_string();
    assert_eq!(
        code, "unsupported-vertex-blend",
        "V2 refusal must carry the structured code, got: {}",
        out[0]["error"]
    );

    // Atomicity across the WASM boundary: same volume, same edge inventory.
    assert!(
        (volume(&mut kernel, solid) - before_volume).abs() < 1e-9,
        "refused fillet must not move volume"
    );
    assert_eq!(
        edge_handles(&mut kernel, solid).len(),
        before_edges,
        "refused fillet must not change the edge inventory"
    );
}

/// Whole-edge chamfer succeeds through batch with the native closed form.
#[test]
fn mixed_notch_whole_edge_chamfer_matches_native_closed_form() {
    let mut kernel = BrepKernel::new();
    let solid = make_l_bracket(&mut kernel);
    let edges = edge_handles(&mut kernel, solid);
    let out = run(
        &mut kernel,
        &[op(
            "chamfer",
            serde_json::json!({"solid": solid, "edges": edges, "distance": 1.0}),
        )],
    );
    let chamfered = u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap();
    let vol = volume(&mut kernel, chamfered);
    assert!(
        (vol - 12905.3333).abs() < 0.5,
        "batch chamfer must meet the native closed form, got {vol}"
    );
}
