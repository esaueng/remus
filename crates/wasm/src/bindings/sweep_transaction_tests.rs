//! Sweep bindings refuse atomically (kernel review WASM-6).
//!
//! `multiSectionSweep`, `helicalSweep` and `guidedSweep` called their
//! operation on the live topology, so a refusal after the operation had
//! placed profile copies left them in the session. The direct methods return
//! `JsError`, so the refusal runs through their natively callable `_impl`
//! bodies; the `executeBatch` arm (already rolled back per op) is checked for
//! the same outcome.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::Point3;

use crate::handles::face_id_to_u32;
use crate::kernel::BrepKernel;

fn counts(kernel: &BrepKernel) -> [usize; 6] {
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

fn square(kernel: &mut BrepKernel) -> u32 {
    let points = [
        Point3::new(-1.0, -1.0, 0.0),
        Point3::new(1.0, -1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(-1.0, 1.0, 0.0),
    ];
    face_id_to_u32(kernel.make_planar_face(&points).unwrap())
}

const SPINE_KNOTS: [f64; 4] = [0.0, 0.0, 1.0, 1.0];
const SPINE_POINTS: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 10.0];

/// Two sections at the same spine parameter: the loft between the placed
/// copies fails after both copies exist (8 vertices and 2 faces leaked).
#[test]
fn refused_multi_section_sweep_leaves_the_session_unchanged() {
    let mut kernel = BrepKernel::new();
    let (a, b) = (square(&mut kernel), square(&mut kernel));
    let before = counts(&kernel);
    let refused = kernel.multi_section_sweep_impl(
        &[a, b],
        &[0.5, 0.5],
        1,
        SPINE_KNOTS.to_vec(),
        &SPINE_POINTS,
        vec![1.0, 1.0],
        true,
    );
    assert!(refused.is_err());
    assert_eq!(counts(&kernel), before);

    // The same sections at distinct parameters still sweep.
    kernel
        .multi_section_sweep_impl(
            &[a, b],
            &[0.0, 1.0],
            1,
            SPINE_KNOTS.to_vec(),
            &SPINE_POINTS,
            vec![1.0, 1.0],
            true,
        )
        .unwrap();
    assert_eq!(counts(&kernel)[5], before[5] + 1);
}

#[test]
fn batch_multi_section_sweep_refusal_matches_the_direct_path() {
    let mut kernel = BrepKernel::new();
    let (a, b) = (square(&mut kernel), square(&mut kernel));
    let spine: Vec<serde_json::Value> = serde_json::from_str(&kernel.execute_batch(
        &serde_json::json!([{"op": "makeNurbsEdge", "args": {
            "startX": 0.0, "startY": 0.0, "startZ": 0.0,
            "endX": 0.0, "endY": 0.0, "endZ": 10.0,
            "degree": 1, "knots": SPINE_KNOTS, "controlPoints": SPINE_POINTS, "weights": [1.0, 1.0]
        }}])
        .to_string(),
    ))
    .unwrap();
    let spine = spine[0]["ok"].as_u64().expect("spine edge");
    let before = counts(&kernel);
    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{"op": "multiSectionSweep", "args": {
                "faces": [a, b], "params": [0.5, 0.5], "spineEdge": spine, "ruled": true
            }}])
            .to_string(),
        ),
    )
    .unwrap();
    assert!(out[0].get("error").is_some(), "{out:?}");
    assert_eq!(counts(&kernel), before);
}
