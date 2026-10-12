//! WASM contract tests for the store-local boolean switch, through
//! `executeBatch` (the JS surface).
//!
//! The switch is off by default and toggles the thread's setting, and a slot
//! cut, a rib fuse and a window intersect on a drilled plate each give the
//! same solid either way: the same volume and the same face and edge counts. Which path ran, and
//! that its result equals the general boolean's face by face, is pinned by
//! the operations crate's store-local oracle tests.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::kernel::BrepKernel;

/// Run one batch operation and return its `ok` payload.
fn call(kernel: &mut BrepKernel, op: &str, args: Value) -> Value {
    let json = json!([{"op": op, "args": args}]).to_string();
    let out: Vec<Value> = serde_json::from_str(&kernel.execute_batch(&json)).unwrap();
    out[0]
        .get("ok")
        .cloned()
        .unwrap_or_else(|| panic!("{op} failed: {}", out[0]))
}

fn handle(v: &Value) -> u32 {
    u32::try_from(v.as_u64().unwrap_or_else(|| panic!("not a handle: {v}"))).unwrap()
}

fn translated(kernel: &mut BrepKernel, solid: u32, x: f64, y: f64, z: f64) -> u32 {
    let matrix = json!([
        1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0
    ]);
    call(
        kernel,
        "transform",
        json!({"solid": solid, "matrix": matrix}),
    );
    solid
}

/// A 64 × 64 × 10 plate drilled with a 3 × 3 grid of r = 2 holes.
fn drilled_plate(kernel: &mut BrepKernel) -> u32 {
    let mut plate = handle(&call(
        kernel,
        "makeBox",
        json!({"width": 64.0, "height": 64.0, "depth": 10.0}),
    ));
    for row in 0..3 {
        for col in 0..3 {
            let tool = handle(&call(
                kernel,
                "makeCylinder",
                json!({"radius": 2.0, "height": 20.0}),
            ));
            let tool = translated(
                kernel,
                tool,
                8.0 + f64::from(col) * 12.0,
                8.0 + f64::from(row) * 12.0,
                -5.0,
            );
            plate = handle(&call(
                kernel,
                "cut",
                json!({"solidA": plate, "solidB": tool}),
            ));
        }
    }
    plate
}

fn counts(kernel: &mut BrepKernel, solid: u32) -> (usize, usize) {
    let count = |kernel: &mut BrepKernel, op: &str| {
        call(kernel, op, json!({"solid": solid}))
            .as_array()
            .unwrap_or_else(|| panic!("{op} returned no list"))
            .len()
    };
    (count(kernel, "getSolidFaces"), count(kernel, "solidEdges"))
}

#[test]
fn store_local_switch_is_off_by_default_and_toggles() {
    let mut kernel = BrepKernel::new();
    assert_eq!(
        call(&mut kernel, "storeLocalCutsEnabled", json!({})),
        json!(false)
    );
    assert!(!kernel.store_local_cuts_enabled());
    assert_eq!(
        call(&mut kernel, "setStoreLocalCuts", json!({"enabled": true})),
        json!(true)
    );
    assert!(kernel.store_local_cuts_enabled());
    kernel.set_store_local_cuts(false);
    assert_eq!(
        call(&mut kernel, "storeLocalCutsEnabled", json!({})),
        json!(false)
    );
}

/// Run `op` on the drilled plate and a box of `size` at `at`, with the
/// switch off and on; both results must have the same counts and volume,
/// which is returned.
fn same_solid_both_ways(op: &str, size: (f64, f64, f64), at: (f64, f64, f64)) -> f64 {
    let mut kernel = BrepKernel::new();
    let plate = drilled_plate(&mut kernel);
    let tool = handle(&call(
        &mut kernel,
        "makeBox",
        json!({"width": size.0, "height": size.1, "depth": size.2}),
    ));
    let tool = translated(&mut kernel, tool, at.0, at.1, at.2);

    let general = handle(&call(
        &mut kernel,
        op,
        json!({"solidA": plate, "solidB": tool}),
    ));
    call(&mut kernel, "setStoreLocalCuts", json!({"enabled": true}));
    let local = handle(&call(
        &mut kernel,
        op,
        json!({"solidA": plate, "solidB": tool}),
    ));
    call(&mut kernel, "setStoreLocalCuts", json!({"enabled": false}));

    assert_eq!(
        counts(&mut kernel, local),
        counts(&mut kernel, general),
        "{op}: counts"
    );
    let volume = |kernel: &mut BrepKernel, solid: u32| {
        call(
            kernel,
            "volume",
            json!({"solid": solid, "deflection": 0.01}),
        )
        .as_f64()
        .unwrap()
    };
    let (v_local, v_general) = (volume(&mut kernel, local), volume(&mut kernel, general));
    assert!(
        (v_local - v_general).abs() <= 1e-9 * v_general,
        "{op}: volume {v_local} with the switch on vs {v_general} off"
    );
    v_local
}

#[test]
fn slot_cut_gives_the_same_solid_with_the_switch_on() {
    let measured = same_solid_both_ways("cut", (2.0, 6.0, 20.0), (13.0, 11.0, -5.0));
    let expected = 64.0 * 64.0 * 10.0 - 9.0 * std::f64::consts::PI * 4.0 * 10.0 - 2.0 * 6.0 * 10.0;
    assert!(
        (measured - expected).abs() < 1e-6 * expected,
        "volume {measured}"
    );
}

#[test]
fn rib_fuse_gives_the_same_solid_with_the_switch_on() {
    // Sunk 2 into the top face, standing 2 proud.
    let measured = same_solid_both_ways("fuse", (6.0, 3.0, 4.0), (11.0, 12.5, 8.0));
    let expected = 64.0 * 64.0 * 10.0 - 9.0 * std::f64::consts::PI * 4.0 * 10.0 + 6.0 * 3.0 * 2.0;
    assert!(
        (measured - expected).abs() < 1e-6 * expected,
        "volume {measured}"
    );
}

#[test]
fn window_intersect_gives_the_same_solid_with_the_switch_on() {
    // A 20 × 20 window holding four of the holes.
    let measured = same_solid_both_ways("intersect", (20.0, 20.0, 20.0), (4.0, 4.0, -5.0));
    let expected = 20.0 * 20.0 * 10.0 - 4.0 * std::f64::consts::PI * 4.0 * 10.0;
    assert!(
        (measured - expected).abs() < 1e-6 * expected,
        "volume {measured}"
    );
}
