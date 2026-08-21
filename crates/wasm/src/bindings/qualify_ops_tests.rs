//! WASM contract tests for the stabilization-plan Phase A operations:
//! draft and defeature through `executeBatch`, with the same closed-form
//! volume oracles as the native qualification suites
//! (`crates/operations/tests/qualify_draft.rs`, `qualify_defeature.rs`),
//! plus native/WASM determinism pins.
//!
//! Every kernel call goes through `execute_batch`: `JsError` cannot be
//! constructed on non-wasm targets, so the `#[wasm_bindgen]` methods are
//! not directly testable on their error paths.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;

fn run(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&k.execute_batch(&json)).unwrap()
}

fn run_all_ok(k: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let results = run(k, ops);
    for (i, r) in results.iter().enumerate() {
        assert!(
            r.get("ok").is_some(),
            "op {i} ({}) failed: {r}",
            ops[i]["op"]
        );
    }
    results
        .into_iter()
        .map(|r| r.get("ok").cloned().unwrap())
        .collect()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn as_u32(v: &serde_json::Value) -> u32 {
    u32::try_from(v.as_u64().unwrap()).unwrap()
}

/// The wall of `solid` whose stored plane normal is closest to `target`.
fn wall_by_normal(k: &mut BrepKernel, solid: u32, target: [f64; 3]) -> u32 {
    let faces = run_all_ok(
        k,
        &[op("getSolidFaces", serde_json::json!({"solid": solid}))],
    );
    let handles: Vec<u32> = faces[0].as_array().unwrap().iter().map(as_u32).collect();
    let mut best = (f64::MIN, 0u32);
    for h in handles {
        let n = run_all_ok(k, &[op("getFaceNormal", serde_json::json!({"face": h}))]);
        let n = n[0].as_array().unwrap();
        let dot = n[0].as_f64().unwrap() * target[0]
            + n[1].as_f64().unwrap() * target[1]
            + n[2].as_f64().unwrap() * target[2];
        if dot > best.0 {
            best = (dot, h);
        }
    }
    assert!(best.0 > 0.99, "no wall matching normal {target:?}");
    best.1
}

/// Draft through the batch API matches the native closed form
/// (1 + tan(a)/2 for a unit cube's +X wall about the base plane).
#[test]
fn batch_draft_volume_matches_closed_form() {
    let mut k = BrepKernel::new();
    let out = run_all_ok(
        &mut k,
        &[op(
            "makeBox",
            serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
        )],
    );
    let cube = as_u32(&out[0]);
    let wall = wall_by_normal(&mut k, cube, [1.0, 0.0, 0.0]);

    let angle = 5.0_f64.to_radians();
    let out = run_all_ok(
        &mut k,
        &[op(
            "draft",
            serde_json::json!({
                "solid": cube, "faces": [wall], "angle": angle,
                "dirX": 0.0, "dirY": 0.0, "dirZ": 1.0,
                "neutralX": 0.0, "neutralY": 0.0, "neutralZ": 0.0,
            }),
        )],
    );
    let drafted = as_u32(&out[0]);

    let out = run_all_ok(
        &mut k,
        &[op(
            "volume",
            serde_json::json!({"solid": drafted, "deflection": 0.01}),
        )],
    );
    let vol = out[0].as_f64().unwrap();
    let expected = 1.0 + angle.tan() / 2.0;
    assert!(
        ((vol - expected) / expected).abs() < 1e-9,
        "expected {expected}, got {vol}"
    );
}

/// Defeature through the batch API: removing a through-hole's walls
/// restores the plain box volume exactly.
#[test]
fn batch_defeature_restores_plain_box() {
    let mut k = BrepKernel::new();
    let out = run_all_ok(
        &mut k,
        &[
            op(
                "makeBox",
                serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
            ),
            op(
                "makeBox",
                serde_json::json!({"width": 0.4, "height": 0.4, "depth": 2.0}),
            ),
        ],
    );
    let cube = as_u32(&out[0]);
    let cutter = as_u32(&out[1]);
    run_all_ok(
        &mut k,
        &[op(
            "transform",
            serde_json::json!({"solid": cutter, "matrix": [
                1.0, 0.0, 0.0, 0.3,
                0.0, 1.0, 0.0, 0.3,
                0.0, 0.0, 1.0, -0.5,
                0.0, 0.0, 0.0, 1.0,
            ]}),
        )],
    );
    let out = run_all_ok(
        &mut k,
        &[op(
            "cut",
            serde_json::json!({"solidA": cube, "solidB": cutter}),
        )],
    );
    let holed = as_u32(&out[0]);

    // The four hole walls: faces whose plane offset |d| is strictly inside
    // the unit box (0.3 or 0.7 on x/y).
    let faces = run_all_ok(
        &mut k,
        &[op("getSolidFaces", serde_json::json!({"solid": holed}))],
    );
    let handles: Vec<u32> = faces[0].as_array().unwrap().iter().map(as_u32).collect();
    let mut walls = Vec::new();
    for h in handles {
        let vs = run_all_ok(
            &mut k,
            &[op("getFaceVertexPositions", serde_json::json!({"face": h}))],
        );
        // Fallback: use normals + a vertex probe is not available as a batch
        // op; classify by every vertex lying strictly inside the hole prism.
        let coords = vs[0].as_array().unwrap();
        let mut inside = !coords.is_empty();
        for chunk in coords.chunks(3) {
            let (x, y) = (chunk[0].as_f64().unwrap(), chunk[1].as_f64().unwrap());
            inside &= (0.29..=0.71).contains(&x) && (0.29..=0.71).contains(&y);
        }
        if inside {
            walls.push(h);
        }
    }
    assert_eq!(walls.len(), 4, "expected the 4 hole walls");

    let out = run_all_ok(
        &mut k,
        &[op(
            "defeature",
            serde_json::json!({"solid": holed, "faces": walls}),
        )],
    );
    let healed = as_u32(&out[0]);
    let out = run_all_ok(
        &mut k,
        &[op(
            "volume",
            serde_json::json!({"solid": healed, "deflection": 0.01}),
        )],
    );
    let vol = out[0].as_f64().unwrap();
    assert!(
        (vol - 1.0).abs() < 1e-9,
        "defeatured box should be exactly 1.0, got {vol}"
    );
}

/// The batch draft is deterministic across fresh kernels: identical result
/// volumes bit-for-bit.
#[test]
fn batch_draft_is_deterministic() {
    let run_once = || {
        let mut k = BrepKernel::new();
        let out = run_all_ok(
            &mut k,
            &[op(
                "makeBox",
                serde_json::json!({"width": 1.0, "height": 1.0, "depth": 1.0}),
            )],
        );
        let cube = as_u32(&out[0]);
        let wall = wall_by_normal(&mut k, cube, [1.0, 0.0, 0.0]);
        let out = run_all_ok(
            &mut k,
            &[op(
                "draft",
                serde_json::json!({
                    "solid": cube, "faces": [wall], "angle": 0.1,
                    "dirZ": 1.0,
                }),
            )],
        );
        let drafted = as_u32(&out[0]);
        let out = run_all_ok(
            &mut k,
            &[op(
                "volume",
                serde_json::json!({"solid": drafted, "deflection": 0.01}),
            )],
        );
        out[0].as_f64().unwrap().to_bits()
    };
    assert_eq!(run_once(), run_once());
}
