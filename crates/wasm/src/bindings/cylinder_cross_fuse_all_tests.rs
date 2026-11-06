//! Direct and both batch WASM qualification of `fuseAll` on crossing
//! cylinders. Tier: core — the union writes the live session.
//!
//! `fuseAll` of a bar crossed by a drill used to shell-merge the two
//! cylinders unsplit (six faces, one validation error); the consumer's union
//! gate refused every such union. The native root cause and oracle are in
//! `crates/operations/tests/regress_cylinder_cross_fuse_all.rs`. Here the
//! union must be valid, split, and equal to the pairwise `fuse` volume.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

/// `(name, bar radius, drill radius, drill offset along Y)`.
const CROSSES: [(&str, f64, f64, f64); 4] = [
    ("r6 x r6", 6.0, 6.0, 0.0),
    ("aligned", 10.0, 6.0, 0.0),
    ("off-centre", 10.0, 6.0, 3.0),
    ("equal-radius", 10.0, 10.0, 0.0),
];

/// Row-major placements: the bar along X centred on the origin, the drill
/// along Z through it.
fn bar_matrix(scale: f64) -> Vec<f64> {
    // rotation_y(90°) then a shift of half the length back along X.
    vec![
        0.0,
        0.0,
        1.0,
        -30.0 * scale,
        0.0,
        1.0,
        0.0,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn drill_matrix(scale: f64, offset: f64) -> Vec<f64> {
    vec![
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        offset * scale,
        0.0,
        0.0,
        1.0,
        -30.0 * scale,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn place(kernel: &mut RemusKernel, bar: f64, drill: f64, offset: f64, scale: f64) -> (u32, u32) {
    let a = kernel
        .make_cylinder_solid(bar * scale, 60.0 * scale)
        .unwrap();
    kernel
        .transform_solid_binding(a, bar_matrix(scale))
        .unwrap();
    let b = kernel
        .make_cylinder_solid(drill * scale, 60.0 * scale)
        .unwrap();
    kernel
        .transform_solid_binding(b, drill_matrix(scale, offset))
        .unwrap();
    (a, b)
}

fn faces(kernel: &RemusKernel, solid: u32) -> usize {
    let id = kernel.resolve_solid(solid).unwrap();
    remus_topology::explorer::solid_faces(kernel.topo(), id)
        .unwrap()
        .len()
}

#[test]
fn direct_fuse_all_unions_crossing_cylinders() {
    for (name, bar, drill, offset) in CROSSES {
        for scale in SCALES {
            for swap in [false, true] {
                let what = format!("{name} scale {scale} swap {swap}");
                let mut kernel = RemusKernel::new();
                let (a, b) = place(&mut kernel, bar, drill, offset, scale);
                let (x, y) = if swap { (b, a) } else { (a, b) };
                let pairwise = kernel.fuse(x, y).unwrap();
                let union = kernel.fuse_all(vec![x, y]).unwrap();
                assert!(faces(&kernel, union) > 6, "{what}: faces left unsplit");
                assert_eq!(kernel.validate_solid(union).unwrap(), 0, "{what}");
                let deflection = 0.01 * scale;
                let volume = kernel.volume(union, deflection).unwrap();
                let expected = kernel.volume(pairwise, deflection).unwrap();
                assert!(
                    (volume - expected).abs() <= 1e-9 * expected,
                    "{what}: fuseAll {volume} vs fuse {expected}"
                );
            }
        }
    }
}

#[test]
fn both_batches_fuse_all_crossing_cylinders() {
    for v2 in [false, true] {
        for (name, bar, drill, offset) in CROSSES {
            for scale in SCALES {
                let what = format!("v2={v2} {name} scale {scale}");
                let request = json!([
                    {"op": "makeCylinder", "args": {"radius": bar * scale, "height": 60.0 * scale}},
                    {"op": "transform", "args": {"solid": 0, "matrix": bar_matrix(scale)}},
                    {"op": "makeCylinder", "args": {"radius": drill * scale, "height": 60.0 * scale}},
                    {"op": "transform", "args": {"solid": 1, "matrix": drill_matrix(scale, offset)}},
                    {"op": "fuseAll", "args": {"solids": [0, 1]}},
                    {"op": "fuse", "args": {"solidA": 0, "solidB": 1}},
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
                let (union, pairwise) = (
                    cells[4]["ok"].as_u64().unwrap(),
                    cells[5]["ok"].as_u64().unwrap(),
                );
                let deflection = 0.01 * scale;
                let probes = json!([
                    {"op": "validateSolid", "args": {"solid": union}},
                    {"op": "volume", "args": {"solid": union, "deflection": deflection}},
                    {"op": "volume", "args": {"solid": pairwise, "deflection": deflection}},
                    {"op": "meshQuality", "args": {"solid": union, "deflection": 0.1 * scale}},
                ])
                .to_string();
                let cells = run(&probes);
                assert_eq!(cells[0]["ok"], 0, "{what}: {cells:?}");
                let (volume, expected) = (
                    cells[1]["ok"].as_f64().unwrap(),
                    cells[2]["ok"].as_f64().unwrap(),
                );
                assert!(
                    (volume - expected).abs() <= 1e-9 * expected,
                    "{what}: fuseAll {volume} vs fuse {expected}"
                );
                assert_eq!(cells[3]["ok"]["isWatertight"], true, "{what}: {cells:?}");
            }
        }
    }
}
