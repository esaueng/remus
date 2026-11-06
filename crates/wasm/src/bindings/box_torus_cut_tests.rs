//! Direct (`cutDetailed` / `fuseDetailed`) and both batch qualifications of
//! box × complete-torus booleans whose box face cuts the torus normal to its
//! axis. Tier: core — the boolean writes the live session.
//!
//! A 60 × 60 × 20 box cut by a torus R = 15, r = 4 centred on its top face
//! returned the box unchanged with status ok. The native root cause and
//! oracle are in `crates/operations/tests/regress_box_torus_cut.rs`: the
//! torus below tube height a has volume 2πR (πr² − segment(a)) by Pappus.
//!
//! The sliver cell at 1e-3 scale (the box top 1e-6 into the tube) used to
//! return the box unchanged with status ok, and the bottom face through the
//! tube centre with the torus seam turned used to refuse; both are exact.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use serde_json::{Value, json};

use crate::kernel::RemusKernel;
use remus_operations::boolean::BooleanOp;

const MAJOR: f64 = 15.0;
const MINOR: f64 = 4.0;
const BOX_VOLUME: f64 = 60.0 * 60.0 * 20.0;
const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

fn torus_volume() -> f64 {
    2.0 * PI * PI * MAJOR * MINOR * MINOR
}

/// Unit-scale torus volume below tube height `a`.
fn torus_below(a: f64) -> f64 {
    let segment = MINOR * MINOR * (a / MINOR).acos() - a * (MINOR * MINOR - a * a).sqrt();
    2.0 * PI * MAJOR * (PI * MINOR * MINOR - segment)
}

/// Unit-scale torus volume inside the box (0 ≤ z ≤ 20) for a torus centred
/// at height `z`.
fn inside(z: f64) -> f64 {
    torus_below((20.0 - z).clamp(-MINOR, MINOR)) - torus_below((-z).clamp(-MINOR, MINOR))
}

/// `(name, torus centre z, seam turn)`: the matrix's aligned, raised and
/// sliver cells, a kept band wider than half the tube, and the bottom face
/// through the tube centre with the seam turned.
const CELLS: [(&str, f64, f64); 5] = [
    ("aligned", 20.0, 0.0),
    ("raised", 22.0, 0.0),
    ("sunk", 17.0, 0.0),
    ("sliver", 24.0 - 1e-3, 0.0),
    ("bottom turned", 0.0, 0.7),
];

/// Row-major translation by `(x, y, z)` after a turn about +Z.
fn placement(x: f64, y: f64, z: f64, turn: f64) -> Vec<f64> {
    let (s, c) = turn.sin_cos();
    vec![
        c, -s, 0.0, x, s, c, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0,
    ]
}

/// The 10° tilt about X through `(30, 30, 20) · scale`, row-major.
fn tilted(scale: f64) -> Vec<f64> {
    let (s, c) = 10f64.to_radians().sin_cos();
    vec![
        1.0,
        0.0,
        0.0,
        30.0 * scale,
        0.0,
        c,
        -s,
        30.0 * scale,
        0.0,
        s,
        c,
        20.0 * scale,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn place(kernel: &mut RemusKernel, scale: f64, matrix: Vec<f64>) -> (u32, u32) {
    let block = kernel
        .make_box_solid(60.0 * scale, 60.0 * scale, 20.0 * scale)
        .unwrap();
    let torus = kernel
        .make_torus_solid(MAJOR * scale, MINOR * scale, 32)
        .unwrap();
    kernel.transform_solid_binding(torus, matrix).unwrap();
    (block, torus)
}

fn ok_volume(kernel: &RemusKernel, result: &Value, expected: f64, what: &str) {
    assert_eq!(result["status"], "ok", "{what}: {result}");
    let handle = u32::try_from(result["value"].as_u64().unwrap()).unwrap();
    assert_eq!(kernel.validate_solid(handle).unwrap(), 0, "{what}");
    let volume = kernel.volume(handle, 0.01).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "{what}: volume {volume} vs closed form {expected}"
    );
}

#[test]
fn detailed_cut_and_fuse_match_the_closed_form() {
    for (name, z, turn) in CELLS {
        for scale in SCALES {
            let s3 = scale * scale * scale;
            let inside = inside(z) * s3;
            let what = format!("{name} scale {scale}");

            let mut kernel = RemusKernel::new();
            let (block, torus) = place(
                &mut kernel,
                scale,
                placement(30.0 * scale, 30.0 * scale, z * scale, turn),
            );
            let cut = serde_json::to_value(kernel.binary_boolean_detailed_impl(
                BooleanOp::Cut,
                "cut",
                block,
                torus,
            ))
            .unwrap();
            ok_volume(
                &kernel,
                &cut,
                BOX_VOLUME.mul_add(s3, -inside),
                &format!("{what} cut"),
            );

            for (a, b) in [(block, torus), (torus, block)] {
                let fuse = serde_json::to_value(kernel.binary_boolean_detailed_impl(
                    BooleanOp::Fuse,
                    "fuse",
                    a,
                    b,
                ))
                .unwrap();
                ok_volume(
                    &kernel,
                    &fuse,
                    (BOX_VOLUME + torus_volume()).mul_add(s3, -inside),
                    &format!("{what} fuse {a}/{b}"),
                );
            }
        }
    }
}

/// The tilted cell has no exact band split: the typed cut refuses and leaves
/// the session unchanged instead of returning the box with status ok.
#[test]
fn detailed_tilted_cut_refuses_instead_of_returning_the_box() {
    for scale in SCALES {
        let mut kernel = RemusKernel::new();
        let (block, torus) = place(&mut kernel, scale, tilted(scale));
        let counts = |kernel: &RemusKernel| {
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_faces(),
                kernel.topo().num_solids(),
            )
        };
        let before = counts(&kernel);
        let result = serde_json::to_value(kernel.binary_boolean_detailed_impl(
            BooleanOp::Cut,
            "cut",
            block,
            torus,
        ))
        .unwrap();
        if result["status"] == "ok" {
            let s3 = scale * scale * scale;
            ok_volume(
                &kernel,
                &result,
                (BOX_VOLUME - torus_volume() / 2.0) * s3,
                &format!("tilted scale {scale}"),
            );
        } else {
            assert_eq!(result["status"], "error", "{result}");
            assert!(result["code"].is_string(), "{result}");
            assert!(result["value"].is_null(), "{result}");
            assert_eq!(result["details"]["operation"], "cut", "{result}");
            assert_eq!(
                counts(&kernel),
                before,
                "a typed refusal must not mutate topology"
            );
        }
    }
}

#[test]
fn both_batches_cut_and_fuse_match_the_closed_form() {
    for v2 in [false, true] {
        for (name, z, turn) in CELLS {
            for scale in SCALES {
                let s3 = scale * scale * scale;
                let inside = inside(z) * s3;
                for (op, a, b, expected) in [
                    ("cut", 0, 1, BOX_VOLUME.mul_add(s3, -inside)),
                    (
                        "fuse",
                        0,
                        1,
                        (BOX_VOLUME + torus_volume()).mul_add(s3, -inside),
                    ),
                    (
                        "fuse",
                        1,
                        0,
                        (BOX_VOLUME + torus_volume()).mul_add(s3, -inside),
                    ),
                ] {
                    let what = format!("v2={v2} {name} scale {scale} {op} {a}/{b}");
                    let request = json!([
                        {"op": "makeBox", "args": {"width": 60.0 * scale, "height": 60.0 * scale, "depth": 20.0 * scale}},
                        {"op": "makeTorus", "args": {"majorRadius": MAJOR * scale, "minorRadius": MINOR * scale, "segments": 32}},
                        {"op": "transform", "args": {"solid": 1, "matrix": placement(30.0 * scale, 30.0 * scale, z * scale, turn)}},
                        {"op": op, "args": {"solidA": a, "solidB": b}},
                    ])
                    .to_string();
                    let mut kernel = RemusKernel::new();
                    let mut run = |request: &str| -> Vec<Value> {
                        let response = if v2 {
                            kernel.execute_batch_v2(request)
                        } else {
                            kernel.execute_batch(request)
                        };
                        serde_json::from_str(&response).unwrap()
                    };
                    let cells = run(&request);
                    assert!(
                        cells.iter().all(|cell| cell.get("ok").is_some()),
                        "{what}: {cells:?}"
                    );
                    let solid = cells[3]["ok"].as_u64().unwrap();
                    let probes = json!([
                        {"op": "validateSolid", "args": {"solid": solid}},
                        {"op": "volume", "args": {"solid": solid, "deflection": 0.01}},
                    ])
                    .to_string();
                    let cells = run(&probes);
                    assert_eq!(cells[0]["ok"], 0, "{what}: validation {cells:?}");
                    let volume = cells[1]["ok"].as_f64().unwrap();
                    assert!(
                        (volume - expected).abs() <= 1e-9 * expected,
                        "{what}: volume {volume} vs closed form {expected}"
                    );
                }
            }
        }
    }
}

/// Through either batch the tilted cut is an error cell, never the box.
#[test]
fn both_batches_refuse_the_tilted_cut() {
    for v2 in [false, true] {
        let request = json!([
            {"op": "makeBox", "args": {"width": 60.0, "height": 60.0, "depth": 20.0}},
            {"op": "makeTorus", "args": {"majorRadius": MAJOR, "minorRadius": MINOR, "segments": 32}},
            {"op": "transform", "args": {"solid": 1, "matrix": tilted(1.0)}},
            {"op": "cut", "args": {"solidA": 0, "solidB": 1}},
        ])
        .to_string();
        let mut kernel = RemusKernel::new();
        let response = if v2 {
            kernel.execute_batch_v2(&request)
        } else {
            kernel.execute_batch(&request)
        };
        let cells: Vec<Value> = serde_json::from_str(&response).unwrap();
        if let Some(handle) = cells[3]["ok"].as_u64() {
            let volume = kernel.volume(u32::try_from(handle).unwrap(), 0.01).unwrap();
            let expected = BOX_VOLUME - torus_volume() / 2.0;
            assert!(
                (volume - expected).abs() <= 1e-9 * expected,
                "v2={v2}: volume {volume} vs closed form {expected}"
            );
        } else {
            assert!(cells[3].get("error").is_some(), "v2={v2}: {response}");
        }
    }
}
