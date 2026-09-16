//! Direct/batch WASM qualification for the B6 convex-hull/Minkowski matrix.
//!
//! Same closed-form volume oracles as the native qualification suite
//! (`crates/operations/tests/qualify_convex_hull.rs`), exercised through
//! `execute_batch` where the entry point is batch-dispatched and through
//! direct kernel calls otherwise: `JsError` cannot be constructed on
//! non-wasm targets, so direct `#[wasm_bindgen]` methods are only callable
//! on their success paths. `convexHull` has no batch dispatch today, so its
//! parity pin goes through the direct binding; `minkowskiSum` is covered
//! both direct and batch.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use remus_math::vec::Point3;
use remus_operations::measure::solid_volume;
use remus_topology::explorer::solid_entity_counts;

use crate::kernel::BrepKernel;

fn flat(points: &[Point3]) -> Vec<f64> {
    let mut out = Vec::with_capacity(points.len() * 3);
    for p in points {
        out.extend([p.x(), p.y(), p.z()]);
    }
    out
}

fn tet_points(scale: f64) -> Vec<Point3> {
    [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| Point3::new(x * scale, y * scale, z * scale))
    .collect()
}

fn octa_points(scale: f64) -> Vec<Point3> {
    [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
    ]
    .iter()
    .map(|&(x, y, z)| Point3::new(x * scale, y * scale, z * scale))
    .collect()
}

fn assert_relative(label: &str, actual: f64, expected: f64, limit: f64) {
    let relative = (actual - expected).abs() / expected.abs();
    assert!(
        relative <= limit,
        "{label}: expected {expected:.12e}, got {actual:.12e}, relative error {relative:.3e} > {limit:.3e}"
    );
}

fn parse(response: &str) -> serde_json::Value {
    serde_json::from_str(response).expect("batch response must be valid JSON")
}

/// `convexHull` through the direct binding matches the native closed forms
/// across the scale ladder: tetrahedron `s^3/6`, octahedron `4s^3/3`.
#[test]
fn direct_convex_hull_matches_closed_forms_across_scales() {
    for scale in [1e-3, 1.0, 1e3] {
        for (label, points, expected) in [
            ("tet", tet_points(scale), scale.powi(3) / 6.0),
            ("octa", octa_points(scale), 4.0 / 3.0 * scale.powi(3)),
        ] {
            let mut kernel = BrepKernel::new();
            let handle = kernel.convex_hull(flat(&points)).unwrap();
            let solid = kernel.resolve_solid(handle).unwrap();
            let volume = solid_volume(kernel.topo(), solid, 0.01 * scale).expect("hull volume");
            assert_relative(
                &format!("direct convexHull {label} at scale {scale}"),
                volume,
                expected,
                1e-9,
            );
            // Structural postconditions travel with the volume oracle.
            assert_eq!(kernel.validate_solid(handle).unwrap(), 0, "{label}");
            let entities = solid_entity_counts(kernel.topo(), solid).expect("entity counts");
            let expected_entities = if label == "tet" {
                (4, 6, 4)
            } else {
                (8, 12, 6)
            };
            assert_eq!(entities, expected_entities, "{label} census");
        }
    }
}

/// Degenerate hull input refuses typed without mutating live topology. The
/// direct binding is only callable on success paths under native test
/// (calling a `#[wasm_bindgen]` method that returns `Err` traps outside
/// wasm), so the coplanar refusal goes through the native constructor while
/// topology preservation is asserted on the kernel's arena.
#[test]
fn batch_convex_hull_degenerate_refuses_typed() {
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 1.0, "height": 2.0, "depth": 3.0}},
        {"op": "volume", "args": {"solid": 0, "deflection": 0.01}},
    ])
    .to_string();
    let response = parse(&BrepKernel::new().execute_batch_v2(&program));
    assert_relative(
        "box before hull refusals",
        response[1]["ok"].as_f64().expect("box volume"),
        6.0,
        1e-12,
    );
    // Direct success path still works natively: a valid hull builds.
    let mut kernel = BrepKernel::new();
    let handle = kernel.convex_hull(flat(&tet_points(1.0))).unwrap();
    assert_eq!(kernel.validate_solid(handle).unwrap(), 0);
}

/// `minkowskiSum` direct/batch parity across scales: `box+box = 8s^3`,
/// `tet+tet = 4s^3/3` (homothety `T+T = 2T`), `octa+octa = 32s^3/3`.
#[test]
fn minkowski_sum_has_direct_batch_parity_across_scales() {
    for scale in [1e-3, 1.0, 1e3] {
        // Direct: box + box.
        let mut direct = BrepKernel::new();
        let a = direct.make_box_solid(scale, scale, scale).unwrap();
        let b = direct.make_box_solid(scale, scale, scale).unwrap();
        let sum = direct.minkowski_sum(a, b).unwrap();
        let solid = direct.resolve_solid(sum).unwrap();
        let direct_volume = solid_volume(direct.topo(), solid, 0.01 * scale).expect("sum volume");
        assert_relative(
            &format!("direct minkowskiSum box+box at scale {scale}"),
            direct_volume,
            8.0 * scale.powi(3),
            1e-9,
        );
        assert_eq!(direct.validate_solid(sum).unwrap(), 0, "box+box");

        // Batch: the same pair through `execute_batch_v2`, pinned to the
        // direct volume bits.
        let program = serde_json::json!([
            {"op": "makeBox", "args": {"width": scale, "height": scale, "depth": scale}},
            {"op": "makeBox", "args": {"width": scale, "height": scale, "depth": scale}},
            {"op": "minkowskiSum", "args": {"solidA": 0, "solidB": 1}},
            {"op": "volume", "args": {"solid": 2, "deflection": 0.01 * scale}},
            {"op": "validateSolid", "args": {"solid": 2}},
        ])
        .to_string();
        let first = BrepKernel::new().execute_batch_v2(&program);
        let second = BrepKernel::new().execute_batch_v2(&program);
        assert_eq!(
            first, second,
            "batch minkowskiSum replay must be deterministic at scale {scale}"
        );
        let batch = parse(&first);
        assert_eq!(batch[2]["ok"], 2, "minkowskiSum handle at scale {scale}");
        let batch_volume = batch[3]["ok"].as_f64().expect("batch volume");
        assert_eq!(
            batch_volume.to_bits(),
            direct_volume.to_bits(),
            "direct/batch volume parity at scale {scale}"
        );
        assert_eq!(batch[4]["ok"], 0, "batch validation at scale {scale}");

        // Direct: tet + tet and octa + octa via hull operands.
        // (`convexHull` has no batch dispatch, so the hull-operand path is
        // pinned direct-only here.)
        let mut kernel = BrepKernel::new();
        let t1 = kernel.convex_hull(flat(&tet_points(scale))).unwrap();
        let t2 = kernel.convex_hull(flat(&tet_points(scale))).unwrap();
        let tet_sum = kernel.minkowski_sum(t1, t2).unwrap();
        let tet_solid = kernel.resolve_solid(tet_sum).unwrap();
        assert_relative(
            &format!("direct minkowskiSum tet+tet at scale {scale}"),
            solid_volume(kernel.topo(), tet_solid, 0.01 * scale).expect("tet sum volume"),
            4.0 / 3.0 * scale.powi(3),
            1e-9,
        );

        let o1 = kernel.convex_hull(flat(&octa_points(scale))).unwrap();
        let o2 = kernel.convex_hull(flat(&octa_points(scale))).unwrap();
        let octa_sum = kernel.minkowski_sum(o1, o2).unwrap();
        let octa_solid = kernel.resolve_solid(octa_sum).unwrap();
        assert_relative(
            &format!("direct minkowskiSum octa+octa at scale {scale}"),
            solid_volume(kernel.topo(), octa_solid, 0.01 * scale).expect("octa sum volume"),
            32.0 / 3.0 * scale.powi(3),
            1e-9,
        );
    }
}

/// Batch `minkowskiSum` on a foreign handle refuses with the stable-coded
/// error and leaves the existing solid measurable.
#[test]
fn batch_minkowski_sum_foreign_handle_refuses_typed() {
    let program = serde_json::json!([
        {"op": "makeBox", "args": {"width": 1.0, "height": 1.0, "depth": 1.0}},
        {"op": "minkowskiSum", "args": {"solidA": 0, "solidB": 9999}},
        {"op": "volume", "args": {"solid": 0, "deflection": 0.01}},
    ])
    .to_string();
    let response = parse(&BrepKernel::new().execute_batch_v2(&program));
    assert_eq!(response[0]["ok"], 0);
    assert!(
        response[1].get("error").is_some(),
        "foreign handle must refuse"
    );
    assert_relative(
        "box after refused minkowskiSum",
        response[2]["ok"].as_f64().expect("box volume"),
        1.0,
        1e-12,
    );
}
