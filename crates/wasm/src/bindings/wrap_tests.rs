//! WASM contract tests for `wrapProfileOnCylinder` and `wrapProfileOnCone`,
//! with the profile built the way OpenZCAD builds a sketch region: line
//! edges, `makeWire`, `makeFaceFromWire`.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use crate::kernel::RemusKernel;

fn counts(kernel: &RemusKernel) -> [usize; 6] {
    let t = kernel.topo();
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
    ]
}

/// A 4 × 2 rectangle on the plane x = 20, `y ∈ [−2, 2]`, `z ∈ [11, 13]`.
fn label(kernel: &mut RemusKernel) -> u32 {
    let corners = [(-2.0, 11.0), (2.0, 11.0), (2.0, 13.0), (-2.0, 13.0)];
    let edges: Vec<u32> = (0..4)
        .map(|i| {
            let (a, b) = (corners[i], corners[(i + 1) % 4]);
            kernel
                .make_line_edge(20.0, a.0, a.1, 20.0, b.0, b.1)
                .unwrap()
        })
        .collect();
    let wire = kernel.make_wire_impl(&edges, true).unwrap();
    kernel.make_face_from_wire(wire).unwrap()
}

fn volume(kernel: &RemusKernel, solid: u32) -> f64 {
    let id = kernel.resolve_solid(solid).unwrap();
    remus_operations::measure::mass_properties(kernel.topo(), id)
        .unwrap()
        .mass
}

fn batch_solid(kernel: &mut RemusKernel, op: serde_json::Value) -> u32 {
    let out: Vec<serde_json::Value> =
        serde_json::from_str(&kernel.execute_batch(&serde_json::json!([op]).to_string())).unwrap();
    u32::try_from(out[0]["ok"].as_u64().expect("solid handle")).unwrap()
}

#[test]
fn a_label_wraps_onto_a_cylinder_directly_and_in_a_batch() {
    // Radius 20 about Z: 8 of area between radii 20 and 21 is 8·41/40.
    let exact = 8.0 * (21.0 * 21.0 - 400.0) / 40.0;
    let mut kernel = RemusKernel::new();
    let face = label(&mut kernel);
    let solid = kernel
        .wrap_profile_on_cylinder_impl(
            face,
            [0.0; 3],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [20.0, 20.0, 21.0],
        )
        .unwrap();
    assert!((volume(&kernel, solid) - exact).abs() / exact < 1e-9);

    let mut kernel = RemusKernel::new();
    let face = label(&mut kernel);
    let solid = batch_solid(
        &mut kernel,
        serde_json::json!({"op": "wrapProfileOnCylinder", "args": {
            "profile": face, "originX": 0.0, "originY": 0.0, "originZ": 0.0,
            "axisX": 0.0, "axisY": 0.0, "axisZ": 1.0, "refX": 1.0, "refY": 0.0, "refZ": 0.0,
            "radius": 20.0, "inner": 20.0, "outer": 21.0}}),
    );
    assert!((volume(&kernel, solid) - exact).abs() / exact < 1e-9);
}

#[test]
fn a_label_wraps_onto_a_cone_directly_and_in_a_batch() {
    let beta = 60f64.to_radians();
    let mut kernel = RemusKernel::new();
    let face = label(&mut kernel);
    let direct = kernel
        .wrap_profile_on_cone_impl(
            face,
            [0.0; 3],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 10.0],
            [beta, 0.0, 1.0],
        )
        .unwrap();
    let a = volume(&kernel, direct);
    // About Δd·A (8) plus the offset's widening.
    assert!(a > 8.0 && a < 9.0, "{a}");

    let mut kernel = RemusKernel::new();
    let face = label(&mut kernel);
    let solid = batch_solid(
        &mut kernel,
        serde_json::json!({"op": "wrapProfileOnCone", "args": {
            "profile": face, "apexX": 0.0, "apexY": 0.0, "apexZ": 0.0,
            "axisX": 0.0, "axisY": 0.0, "axisZ": 1.0, "refX": 1.0, "refY": 0.0, "refZ": 0.0,
            "originX": 0.0, "originY": 0.0, "originZ": 10.0,
            "halfAngle": beta, "inner": 0.0, "outer": 1.0}}),
    );
    assert!((volume(&kernel, solid) - a).abs() < 1e-12 * a);
}

#[test]
fn refused_wraps_leave_the_session_unchanged() {
    let mut kernel = RemusKernel::new();
    let face = label(&mut kernel);
    let before = counts(&kernel);
    // An inner offset that passes the apex.
    assert!(
        kernel
            .wrap_profile_on_cone_impl(
                face,
                [0.0; 3],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 10.0],
                [60f64.to_radians(), -100.0, 1.0],
            )
            .is_err()
    );
    // Radii the wrong way round.
    assert!(
        kernel
            .wrap_profile_on_cylinder_impl(
                face,
                [0.0; 3],
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [20.0, 21.0, 20.0],
            )
            .is_err()
    );
    assert_eq!(counts(&kernel), before);
}
