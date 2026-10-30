//! Batch `sewFaces` without a `tolerance` sews at the shared batch default
//! `TOL` (1e-7), like every other batch tolerance (RFC 0010 drift D10). It
//! used to default to 1e-6, so edges 2e-7 apart were silently joined.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::RemusKernel;

fn ok_u32(kernel: &mut RemusKernel, op: &str, args: &serde_json::Value) -> u32 {
    let out = kernel.execute_batch_v2(&serde_json::json!([{"op": op, "args": args}]).to_string());
    let results: Vec<serde_json::Value> = serde_json::from_str(&out).unwrap();
    let value = results[0]["ok"]
        .as_u64()
        .unwrap_or_else(|| panic!("{op} failed: {}", results[0]));
    u32::try_from(value).unwrap()
}

/// A planar quad in z = 0 through `corners`, built from loose line edges.
fn quad(kernel: &mut RemusKernel, corners: [(f64, f64); 4]) -> u32 {
    let edges: Vec<u32> = (0..4)
        .map(|i| {
            let (x1, y1) = corners[i];
            let (x2, y2) = corners[(i + 1) % 4];
            ok_u32(
                kernel,
                "makeLineEdge",
                &serde_json::json!({"x1": x1, "y1": y1, "z1": 0.0, "x2": x2, "y2": y2, "z2": 0.0}),
            )
        })
        .collect();
    let wire = ok_u32(kernel, "makeWire", &serde_json::json!({"edges": edges}));
    ok_u32(
        kernel,
        "makePlanarFaceFromWire",
        &serde_json::json!({"wire": wire}),
    )
}

/// Distinct edges in the sewn result of two unit quads whose facing edges
/// are `gap` apart, sewn with `tolerance` (`None`: argument absent).
fn sewn_edge_count(gap: f64, tolerance: Option<f64>) -> usize {
    let mut kernel = RemusKernel::new();
    let left = quad(
        &mut kernel,
        [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
    );
    let x = 1.0 + gap;
    let right = quad(
        &mut kernel,
        [(x, 0.0), (x + 1.0, 0.0), (x + 1.0, 1.0), (x, 1.0)],
    );
    let mut args = serde_json::json!({"faces": [left, right]});
    if let Some(tolerance) = tolerance {
        args["tolerance"] = serde_json::json!(tolerance);
    }
    let solid = ok_u32(&mut kernel, "sewFaces", &args);
    let solid = kernel.resolve_solid(solid).unwrap();
    let topo = kernel.topo();
    let mut edges = std::collections::BTreeSet::new();
    for face in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
        edges.extend(wire.edges().iter().map(|oe| oe.edge().index()));
    }
    edges.len()
}

#[test]
fn absent_tolerance_sews_at_the_batch_default() {
    // Sewing snaps vertices to a grid of cell `tolerance`; at x = 1 a 2e-7
    // gap shares a 1e-6 cell but spans two 1e-7 cells.
    let gap = 2e-7;
    assert_eq!(sewn_edge_count(gap, Some(1e-6)), 7, "1e-6 joins the gap");
    assert_eq!(sewn_edge_count(gap, Some(crate::helpers::TOL)), 8);
    assert_eq!(
        sewn_edge_count(gap, None),
        8,
        "absent tolerance must be TOL"
    );
    // Coincident edges still share one edge at the default.
    assert_eq!(sewn_edge_count(0.0, None), 7);
}
