//! WASM contract tests for the mixed-notch L-bracket boundary.
//!
//! The concave L-bracket (XY polygon
//! `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]` extruded 20 mm) exercises the
//! kernel's mixed convex/concave corner boundary through the same entry
//! points a JS consumer uses:
//!
//! - whole-edge constant-radius fillet succeeds with torus corners on both
//!   the legacy and `executeBatchV2` contracts, meeting the closed-form
//!   volume and preserving the input;
//! - whole-edge chamfer at distance 1 succeeds with the native closed-form
//!   volume.
//!
//! Fixture construction uses the direct polygon binding (infallible here)
//! plus batch extrude; every fallible step goes through `executeBatch`.
//! The direct `#[wasm_bindgen]` methods are not callable from native tests
//! on failure paths; these batch tests cover the dispatch plus the shared
//! helpers, while `crates/operations/tests/regress_notch_torus_whole_edge.rs`
//! pins the engine natively.

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

/// Whole-edge fillet succeeds on both contracts with torus corners;
/// the input is preserved and the result meets the closed form.
///
/// Measurement: batch `volume` sums the boundary-trimmed Gauss integrals
/// on the torus-notch family (not the closed mesh), so the 0.05 window
/// distinguishes the exact route from the mesh route; geometry is proven
/// identical to the independent reference by STEP exchange (1.2e-5).
#[test]
fn mixed_notch_whole_edge_fillet_succeeds_on_both_contracts() {
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

    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "edges": edges, "radius": 1.0}),
        )],
    );
    let result = u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap();
    assert_ne!(result, solid, "fillet must return a new handle");
    let vol = volume(&mut kernel, result);
    assert!(
        (vol - 13027.2829).abs() < 0.05,
        "batch fillet must meet the closed form 13027.2829, got {vol}"
    );

    let out = run_v2(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "edges": edges, "radius": 1.0}),
        )],
    );
    let v2 = u32::try_from(out[0]["ok"].as_u64().expect("V2 fillet must succeed")).unwrap();
    assert_ne!(v2, solid, "V2 fillet must return a new handle");
    let v2_vol = volume(&mut kernel, v2);
    assert!(
        (v2_vol - 13027.2829).abs() < 0.05,
        "V2 fillet must meet the closed form 13027.2829 on the exact route, got {v2_vol}"
    );

    // Success preserves the input across the WASM boundary.
    assert!(
        (volume(&mut kernel, solid) - before_volume).abs() < 1e-9,
        "successful fillet must not move input volume"
    );
    assert_eq!(
        edge_handles(&mut kernel, solid).len(),
        18,
        "successful fillet must not change the input edge inventory"
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

/// Rib-on-plate fixture through batch: 60x50x6 plate plus one 6x20x18 rib
/// standing at x=[12,18], y=[10,30], z=[6,24] (translation matrix, row-major
/// 16 elements). The four rib-base vertices are convex-singleton mirrors.
fn make_rib_plate(kernel: &mut BrepKernel) -> u32 {
    let out = run(
        kernel,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 60.0, "height": 50.0, "depth": 6.0}),
            ),
            op(
                "makeBox",
                serde_json::json!({"width": 6.0, "height": 20.0, "depth": 18.0}),
            ),
        ],
    );
    let plate = u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap();
    let rib = u32::try_from(out[1]["ok"].as_u64().unwrap()).unwrap();
    let out = run(
        kernel,
        &[op(
            "transform",
            serde_json::json!({"solid": rib, "matrix": [1.0, 0.0, 0.0, 12.0, 0.0, 1.0, 0.0, 10.0, 0.0, 0.0, 1.0, 6.0, 0.0, 0.0, 0.0, 1.0]}),
        )],
    );
    let moved = u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap();
    let out = run(
        kernel,
        &[op(
            "fuse",
            serde_json::json!({"solidA": plate, "solidB": moved}),
        )],
    );
    u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap()
}

/// Whole-edge rib fillet succeeds on both contracts: the four rib-base
/// mirrors close with torus patches, material is removed, and both
/// contracts agree on the exact route (native pins 20046.9186 at r=1).
#[test]
fn rib_mirror_whole_edge_fillet_succeeds_on_both_contracts() {
    let mut kernel = BrepKernel::new();
    let solid = make_rib_plate(&mut kernel);
    let before = volume(&mut kernel, solid);
    assert!(
        (before - 20160.0).abs() < 1.0,
        "rib-plate fixture must be 20160, got {before}"
    );
    let edges = edge_handles(&mut kernel, solid);
    assert_eq!(edges.len(), 24, "rib plate must expose 24 edges");

    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "edges": edges, "radius": 1.0}),
        )],
    );
    let result = u32::try_from(out[0]["ok"].as_u64().unwrap()).unwrap();
    let vol = volume(&mut kernel, result);
    assert!(
        before - vol > 50.0 && before - vol < 500.0,
        "rib fillet must remove groove-excess material, got {before} -> {vol}"
    );

    let out = run_v2(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "edges": edges, "radius": 1.0}),
        )],
    );
    let v2 = u32::try_from(out[0]["ok"].as_u64().expect("V2 rib fillet must succeed")).unwrap();
    let v2_vol = volume(&mut kernel, v2);
    assert!(
        (v2_vol - vol).abs() < 0.05,
        "V2 rib fillet must agree with legacy on the exact route, got {v2_vol} vs {vol}"
    );
}
