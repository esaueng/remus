//! Contract tests for `split` across a curved face's interior through the
//! handle-only `split` binding and `executeBatch`.
//!
//! A plane through the bulge of a sphere or torus, clear of every edge, used
//! to be refused as "entirely on one side": the side was read from edges
//! only, and a sphere's edges are its equator arcs while a primitive torus's
//! are two degenerate seams. Both halves now build to their closed forms,
//! direct and batched bit for bit, while tangent and missing planes keep
//! their refusal and leave the session as it was.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_operations::measure::solid_volume;

use crate::kernel::RemusKernel;

fn assert_relative(label: &str, actual: f64, expected: f64) {
    let relative = (actual - expected).abs() / expected.abs().max(1e-300);
    assert!(
        relative <= 1e-9,
        "{label}: expected {expected:.12e}, got {actual:.12e}, relative error {relative:.3e}"
    );
}

fn parse(response: &str) -> serde_json::Value {
    serde_json::from_str(response).expect("batch response must be valid JSON")
}

/// Sphere cap of height `h` on a sphere of radius `r`.
fn sphere_cap(r: f64, h: f64) -> f64 {
    PI * h * h * (3.0 * r - h) / 3.0
}

/// Torus R10 r3 above `z = 1`: the tube's disc segment swept at radius R.
fn torus_above_one() -> f64 {
    let (r, c) = (3.0_f64, 1.0_f64);
    let segment = r * r * (c / r).acos() - c * (r * r - c * c).sqrt();
    2.0 * PI * 10.0 * segment
}

/// Split `solid` at `z = 1` directly and in a batch, and require both halves
/// at the closed forms, valid, and equal bit for bit across the two routes.
fn direct_and_batch_split(
    make: &serde_json::Value,
    build: fn(&mut RemusKernel) -> u32,
    pos: f64,
    neg: f64,
    what: &str,
) {
    let mut direct = RemusKernel::new();
    let solid = build(&mut direct);
    let halves = direct
        .split_solid(solid, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0)
        .unwrap_or_else(|_| panic!("{what}: direct split refused"));
    assert_eq!(halves.len(), 2, "{what}: split must return two halves");
    let mut direct_volumes = [0.0; 2];
    for (slot, &handle) in halves.iter().enumerate() {
        let half = direct.resolve_solid(handle).unwrap();
        direct_volumes[slot] = solid_volume(direct.topo(), half, 0.01).unwrap();
        assert_eq!(
            direct.validate_solid(handle).unwrap(),
            0,
            "{what}: half {slot} validates"
        );
    }
    assert_relative(&format!("{what}: direct positive"), direct_volumes[0], pos);
    assert_relative(&format!("{what}: direct negative"), direct_volumes[1], neg);

    let mut batch = RemusKernel::new();
    let program = serde_json::json!([
        make,
        {"op": "split", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": 1.0, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
    ])
    .to_string();
    let response = parse(&batch.execute_batch_v2(&program));
    let split = &response[1]["ok"];
    let handles = [&split["positive"], &split["negative"]].map(|h| {
        h.as_u64()
            .unwrap_or_else(|| panic!("{what}: batch split refused: {response:?}"))
    });
    for (slot, handle) in handles.into_iter().enumerate() {
        let program = serde_json::json!([
            {"op": "volume", "args": {"solid": handle, "deflection": 0.01}},
            {"op": "validateSolid", "args": {"solid": handle}},
        ])
        .to_string();
        let response = parse(&batch.execute_batch_v2(&program));
        let got = response[0]["ok"].as_f64().expect("batch volume");
        assert!(
            got.to_bits() == direct_volumes[slot].to_bits(),
            "{what}: batch half {slot} {got} must equal direct {} bit for bit",
            direct_volumes[slot]
        );
        assert_eq!(
            response[1]["ok"],
            serde_json::json!(0),
            "{what}: batch half {slot} validates"
        );
    }
}

#[test]
fn sphere_split_off_its_equator_builds_direct_and_batched() {
    direct_and_batch_split(
        &serde_json::json!({"op": "makeSphere", "args": {"radius": 5.0, "segments": 16}}),
        |k| k.make_sphere_solid(5.0, 16).unwrap(),
        sphere_cap(5.0, 4.0),
        sphere_cap(5.0, 6.0),
        "R5 sphere at z = 1",
    );
}

#[test]
fn torus_split_through_its_tube_builds_direct_and_batched() {
    let total = 2.0 * PI * PI * 10.0 * 9.0;
    direct_and_batch_split(
        &serde_json::json!({"op": "makeTorus", "args": {"majorRadius": 10.0, "minorRadius": 3.0, "segments": 16}}),
        |k| k.make_torus_solid(10.0, 3.0, 16).unwrap(),
        torus_above_one(),
        total - torus_above_one(),
        "R10 r3 torus at z = 1",
    );
}

/// Tangent (`z = 5`) and missing (`z = 6`) planes stay refused as one-sided,
/// and the sphere measures the same afterwards.
#[test]
fn batch_sphere_tangent_and_miss_stay_refused() {
    for z in [5.0, 6.0] {
        let mut kernel = RemusKernel::new();
        let program = serde_json::json!([
            {"op": "makeSphere", "args": {"radius": 5.0, "segments": 16}},
            {"op": "split", "args": {"solid": 0, "px": 0.0, "py": 0.0, "pz": z, "nx": 0.0, "ny": 0.0, "nz": 1.0}},
            {"op": "volume", "args": {"solid": 0, "deflection": 0.01}},
        ])
        .to_string();
        let response = parse(&kernel.execute_batch_v2(&program));
        let message = response[1]["error"].to_string();
        assert!(
            message.contains("entirely on one side"),
            "z = {z}: expected the one-sided refusal, got {response:?}"
        );
        assert_relative(
            &format!("z = {z}: sphere after refusal"),
            response[2]["ok"].as_f64().expect("volume after refusal"),
            4.0 * PI * 125.0 / 3.0,
        );
    }
}
