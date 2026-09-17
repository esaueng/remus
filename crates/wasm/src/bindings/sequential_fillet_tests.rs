//! WASM contract test for filleting an edge that ends on an earlier fillet.
//!
//! Mirrors the interactive sequence a modeller performs: fillet one box
//! edge, then select the top edge that now runs into that fillet's band and
//! fillet it too, at the same radius. The `fillet` batch op must answer with
//! a new handle, a watertight body, and the closed-form volume of the
//! propagated blend (the second fillet follows its tangent ridgeline across
//! the band's arc onto the far top edge, and the corner becomes a sphere
//! octant).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;

fn run(kernel: &mut BrepKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
    let json = serde_json::Value::Array(ops.to_vec()).to_string();
    serde_json::from_str(&kernel.execute_batch(&json)).unwrap()
}

fn op(name: &str, args: serde_json::Value) -> serde_json::Value {
    serde_json::json!({"op": name, "args": args})
}

fn ok_handle(out: &serde_json::Value) -> u32 {
    let handle = out["ok"]
        .as_u64()
        .unwrap_or_else(|| panic!("expected an ok handle, got {out}"));
    u32::try_from(handle).unwrap()
}

/// The edge of `solid` whose endpoints both satisfy `at`, as a batch handle.
fn edge_where(kernel: &mut BrepKernel, solid: u32, at: impl Fn(f64, f64, f64) -> bool) -> u32 {
    let out = run(
        kernel,
        &[op("solidEdges", serde_json::json!({"solid": solid}))],
    );
    let matching: Vec<u32> = out[0]["ok"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| u32::try_from(v.as_u64().unwrap()).unwrap())
        .filter(|&handle| {
            let edge_id = kernel.resolve_edge(handle).unwrap();
            let edge = kernel.topo().edge(edge_id).unwrap();
            let a = kernel.topo().vertex(edge.start()).unwrap().point();
            let b = kernel.topo().vertex(edge.end()).unwrap().point();
            at(a.x(), a.y(), a.z()) && at(b.x(), b.y(), b.z())
        })
        .collect();
    assert_eq!(
        matching.len(),
        1,
        "exactly one edge should match: {matching:?}"
    );
    matching[0]
}

#[test]
fn fillet_top_edge_after_vertical_edge_fillet() {
    let (dx, dy, dz) = (9.5, 12.5, 8.5);
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    let mut kernel = BrepKernel::new();
    let out = run(
        &mut kernel,
        &[op(
            "makeBox",
            serde_json::json!({"width": dx, "height": dy, "depth": dz}),
        )],
    );
    let solid = ok_handle(&out[0]);

    let vertical = edge_where(&mut kernel, solid, |x, y, _| near(x, dx) && near(y, 0.0));
    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "radius": 1.0, "edges": [vertical]}),
        )],
    );
    let first = ok_handle(&out[0]);

    let top = edge_where(&mut kernel, first, |x, _, z| near(x, dx) && near(z, dz));
    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": first, "radius": 1.0, "edges": [top]}),
        )],
    );
    let second = ok_handle(&out[0]);
    assert_ne!(
        second, first,
        "a fillet that changed the body returns a new handle"
    );

    let counts = kernel.get_entity_counts(second).unwrap();
    assert_eq!(counts, vec![10, 21, 13], "faces, edges, vertices");
    let euler = i64::from(counts[2]) - i64::from(counts[1]) + i64::from(counts[0]);
    assert_eq!(euler, 2, "genus 0");
    assert_eq!(
        kernel.validate_solid(second).unwrap(),
        0,
        "no validation errors"
    );

    let out = run(
        &mut kernel,
        &[op(
            "volume",
            serde_json::json!({"solid": second, "deflection": 0.01}),
        )],
    );
    let vol = out[0]["ok"].as_f64().unwrap();
    let straight = (1.0 - std::f64::consts::FRAC_PI_4) * ((dz - 1.0) + (dy - 1.0) + (dx - 1.0));
    let corner = 1.0 - std::f64::consts::FRAC_PI_6;
    let expected = dx * dy * dz - straight - corner;
    assert!(
        (vol - expected).abs() < 0.05,
        "volume {vol} should match the closed form {expected}"
    );
}
