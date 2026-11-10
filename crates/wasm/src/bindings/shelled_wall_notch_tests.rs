//! Direct (`cutDetailed` / `fuseDetailed`) and both batch WASM qualification
//! of a round cutter standing on the wall of a shelled box. Tier: core — the
//! booleans write the live session.
//!
//! A 100 × 100 × 50 box shelled 2 thick with its top open, and a cylinder of
//! radius 2 or 6 through the middle of one wall. The rim is a planar face
//! whose hole is the cavity opening; its section with the cutter ran on
//! across the opening and every such cut and fuse was refused. The native
//! root causes and oracles are in
//! `crates/operations/tests/regress_shelled_wall_rim_notch.rs`.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use serde_json::{Value, json};

use crate::kernel::RemusKernel;
use remus_operations::boolean::BooleanOp;

const SCALES: [f64; 3] = [0.1, 1.0, 10.0];
const RADII: [f64; 2] = [2.0, 6.0];
const TOL: f64 = 1e-10;
const SHELL_VOLUME: f64 = 57_632.0;

/// Row-major translation.
fn translation(x: f64, y: f64, z: f64) -> Vec<f64> {
    vec![
        1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0,
    ]
}

/// Area of the disc of radius `r` within `|x| <= 1` of its centre: the part
/// of a cutter centred on a 2 mm wall that lies in the wall.
fn disc_in_wall(r: f64) -> f64 {
    if r <= 1.0 {
        return PI * r * r;
    }
    2.0 * (r * r - 1.0).sqrt() + 2.0 * r * r * (1.0 / r).asin()
}

/// Area of the disc of radius `r` within `x <= 1` of its centre: the part
/// over the floor, which runs out to the wall's outer side.
fn disc_over_floor(r: f64) -> f64 {
    if r <= 1.0 {
        return PI * r * r;
    }
    0.5 * PI * r * r + (r * r - 1.0).sqrt() + r * r * (1.0 / r).asin()
}

/// The closed-form volume after a through cutter of radius `r` (unit scale):
/// two units of floor and forty-eight of wall lie inside the cutter.
fn expected(operation: &str, r: f64) -> f64 {
    let common = 2.0 * disc_over_floor(r) + 48.0 * disc_in_wall(r);
    match operation {
        "cut" => SHELL_VOLUME - common,
        _ => SHELL_VOLUME + PI * r * r * 100.0 - common,
    }
}

fn run(kernel: &mut RemusKernel, v2: bool, request: &Value) -> Vec<Value> {
    let request = request.to_string();
    let response = if v2 {
        kernel.execute_batch_v2(&request)
    } else {
        kernel.execute_batch(&request)
    };
    let cells: Vec<Value> = serde_json::from_str(&response).unwrap();
    assert!(
        cells.iter().all(|cell| cell.get("ok").is_some()),
        "{request}: {cells:?}"
    );
    cells
}

fn handle(cell: &Value) -> u32 {
    u32::try_from(cell["ok"].as_u64().unwrap()).unwrap()
}

/// The shelled box and the cutter, built through the batch `v2` selects.
fn operands(kernel: &mut RemusKernel, v2: bool, s: f64, r: f64) -> (u32, u32) {
    let made = run(
        kernel,
        v2,
        &json!([{"op": "makeBox", "args": {"width": 100.0 * s, "height": 100.0 * s, "depth": 50.0 * s}}]),
    );
    let solid = handle(&made[0]);
    let faces = run(
        kernel,
        v2,
        &json!([{"op": "getSolidFaces", "args": {"solid": solid}}]),
    );
    let faces: Vec<u64> = faces[0]["ok"]
        .as_array()
        .unwrap()
        .iter()
        .map(|face| face.as_u64().unwrap())
        .collect();
    let normals = run(
        kernel,
        v2,
        &Value::Array(
            faces
                .iter()
                .map(|face| json!({"op": "getFaceNormal", "args": {"face": face}}))
                .collect(),
        ),
    );
    let top = faces
        .iter()
        .zip(&normals)
        .find(|(_, normal)| normal["ok"][2].as_f64().unwrap() > 0.9)
        .map(|(face, _)| *face)
        .unwrap();
    let built = run(
        kernel,
        v2,
        &json!([
            {"op": "shell", "args": {"solid": solid, "thickness": 2.0 * s, "faces": [top]}},
            {"op": "makeCylinder", "args": {"radius": r * s, "height": 100.0 * s}},
        ]),
    );
    let (shelled, cutter) = (handle(&built[0]), handle(&built[1]));
    run(
        kernel,
        v2,
        &json!([{"op": "transform", "args": {
            "solid": cutter,
            "matrix": translation(99.0 * s, 50.0 * s, -20.0 * s),
        }}]),
    );
    (shelled, cutter)
}

/// Validator-clean, watertight at the consumer's deflection, analytic with
/// the cutter's cylinder present, and the closed-form volume.
fn assert_exact(kernel: &mut RemusKernel, solid: u32, operation: &str, r: f64, s: f64, what: &str) {
    assert_eq!(kernel.validate_solid(solid).unwrap(), 0, "{what}");
    let volume = kernel.volume(solid, 0.01 * s).unwrap();
    let target = expected(operation, r) * s.powi(3);
    assert!(
        (volume - target).abs() <= TOL * target,
        "{what}: volume {volume} vs closed form {target}"
    );
    let quality = run(
        kernel,
        false,
        &json!([{"op": "meshQuality", "args": {"solid": solid, "deflection": 0.01 * s}}]),
    );
    assert_eq!(
        quality[0]["ok"]["isWatertight"], true,
        "{what}: {quality:?}"
    );
    let types: Vec<String> = kernel
        .get_solid_faces(solid)
        .unwrap()
        .into_iter()
        .map(|face| kernel.get_surface_type(face).unwrap())
        .collect();
    assert!(types.len() <= 32, "{what}: {} faces", types.len());
    assert!(types.iter().any(|t| t == "cylinder"), "{what}: {types:?}");
    assert!(
        types.iter().all(|t| t == "plane" || t == "cylinder"),
        "{what}: {types:?}"
    );
}

#[test]
fn detailed_booleans_notch_a_shelled_wall_exactly() {
    for s in SCALES {
        for r in RADII {
            for (op, operation) in [(BooleanOp::Cut, "cut"), (BooleanOp::Fuse, "fuse")] {
                let what = format!("{operation} r={r} scale {s}");
                let mut kernel = RemusKernel::new();
                let (shelled, cutter) = operands(&mut kernel, false, s, r);
                let result = serde_json::to_value(
                    kernel.binary_boolean_detailed_impl(op, operation, shelled, cutter),
                )
                .unwrap();
                assert_eq!(result["status"], "ok", "{what}: {result}");
                let solid = u32::try_from(result["value"].as_u64().unwrap()).unwrap();
                assert_exact(&mut kernel, solid, operation, r, s, &what);
            }
        }
    }
}

#[test]
fn both_batches_notch_a_shelled_wall_exactly() {
    for v2 in [false, true] {
        for s in SCALES {
            for r in RADII {
                for operation in ["cut", "fuse"] {
                    let what = format!("v2={v2} {operation} r={r} scale {s}");
                    let mut kernel = RemusKernel::new();
                    let (shelled, cutter) = operands(&mut kernel, v2, s, r);
                    let cells = run(
                        &mut kernel,
                        v2,
                        &json!([{"op": operation, "args": {"solidA": shelled, "solidB": cutter}}]),
                    );
                    let solid = handle(&cells[0]);
                    let probes = run(
                        &mut kernel,
                        v2,
                        &json!([
                            {"op": "validateSolid", "args": {"solid": solid}},
                            {"op": "volume", "args": {"solid": solid, "deflection": 0.01 * s}},
                            {"op": "meshQuality", "args": {"solid": solid, "deflection": 0.01 * s}},
                        ]),
                    );
                    assert_eq!(probes[0]["ok"], 0, "{what}: {probes:?}");
                    let volume = probes[1]["ok"].as_f64().unwrap();
                    let target = expected(operation, r) * s.powi(3);
                    assert!(
                        (volume - target).abs() <= TOL * target,
                        "{what}: volume {volume} vs closed form {target}"
                    );
                    assert_eq!(probes[2]["ok"]["isWatertight"], true, "{what}: {probes:?}");
                    assert_exact(&mut kernel, solid, operation, r, s, &what);
                }
            }
        }
    }
}
