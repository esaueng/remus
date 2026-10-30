//! Sketch input validation contracts (kernel review WASM-1, WASM-2, WASM-8).
//!
//! The legacy `sketch*` bindings are not in `executeBatch`, so their refusals
//! go through `sketch_add_constraint_impl` (the direct method returns
//! `JsError`, which cannot be built natively); the typed GCS solve runs
//! through `executeBatch`.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_operations::sketch::Constraint;

use crate::error::WasmError;
use crate::kernel::RemusKernel;

/// A legacy sketch holding `n` free points along the x axis.
fn legacy_sketch(n: u32) -> (RemusKernel, u32) {
    let mut kernel = RemusKernel::new();
    let sketch = kernel.sketch_new().unwrap();
    for i in 0..n {
        kernel
            .sketch_add_point(sketch, f64::from(i), f64::from(i % 2), false)
            .unwrap();
    }
    (kernel, sketch)
}

fn stored(kernel: &RemusKernel, sketch: u32) -> (usize, usize) {
    let sk = kernel.sketches.get(sketch as usize).unwrap();
    (sk.constraints.len(), sk.deferred_constraints.len())
}

fn refused(kernel: &mut RemusKernel, sketch: u32, json: &str) -> WasmError {
    let before = stored(kernel, sketch);
    let error = kernel
        .sketch_add_constraint_impl(sketch, json)
        .expect_err(json);
    assert_eq!(stored(kernel, sketch), before, "{json} must not be stored");
    error
}

/// WASM-1: `p3`/`p4` went through `as_u64` with `p1`/`p2` defaults, so `-1`
/// and `"2"` silently fell back to the first line and a lone `p3` became
/// `Angle(0, 1, 2, 1)`.
#[test]
fn legacy_angle_refuses_malformed_or_half_specified_second_line() {
    let (mut kernel, sketch) = legacy_sketch(4);
    for json in [
        r#"{"type":"angle","p1":0,"p2":1,"p3":-1,"p4":3,"value":0.5}"#,
        r#"{"type":"angle","p1":0,"p2":1,"p3":"2","p4":3,"value":0.5}"#,
        r#"{"type":"angle","p1":0,"p2":1,"p3":2,"p4":3.5,"value":0.5}"#,
        r#"{"type":"angle","p1":0,"p2":1,"p3":2,"value":0.5}"#,
        r#"{"type":"angle","p1":0,"p2":1,"p4":3,"value":0.5}"#,
    ] {
        let error = refused(&mut kernel, sketch, json);
        assert!(
            matches!(error, WasmError::InvalidInput { .. }),
            "{json}: {error}"
        );
    }
    // An out-of-range second line is refused by the point-reference check.
    let error = refused(
        &mut kernel,
        sketch,
        r#"{"type":"angle","p1":0,"p2":1,"p3":2,"p4":4,"value":0.5}"#,
    );
    assert!(matches!(error, WasmError::Sketch(_)), "{error}");
}

/// Issue #1049: overflowing optional references must never attach to a
/// low-numbered point; on native targets the reference validator rejects them.
#[test]
fn legacy_angle_refuses_overflowing_second_line_without_mutation() {
    let (mut kernel, sketch) = legacy_sketch(4);
    for field in ["p3", "p4"] {
        for index in [1_u64 << 32, (1_u64 << 32) + 1, u64::MAX] {
            let mut spec = serde_json::json!({
                "type":"angle", "p1":0, "p2":1, "p3":2, "p4":3, "value":0.5
            });
            spec[field] = index.into();
            refused(&mut kernel, sketch, &spec.to_string());
        }
    }
}

#[test]
fn legacy_angle_keeps_its_two_accepted_forms() {
    let (mut kernel, sketch) = legacy_sketch(4);
    kernel
        .sketch_add_constraint_impl(
            sketch,
            r#"{"type":"angle","p1":0,"p2":1,"p3":2,"p4":3,"value":0.5}"#,
        )
        .unwrap();
    // The three-field form keeps its documented `Angle(p1, p2, p1, p2)`; a
    // `null` second line reads as absent.
    kernel
        .sketch_add_constraint_impl(sketch, r#"{"type":"angle","p1":0,"p2":1,"value":0.0}"#)
        .unwrap();
    kernel
        .sketch_add_constraint_impl(
            sketch,
            r#"{"type":"angle","p1":2,"p2":3,"p3":null,"p4":null,"value":0.0}"#,
        )
        .unwrap();
    let sk = kernel.sketches.get(sketch as usize).unwrap();
    let angles: Vec<_> = sk
        .constraints
        .iter()
        .map(|c| match c {
            Constraint::Angle(a, b, c, d, _) => (*a, *b, *c, *d),
            other => unreachable!("{other:?}"),
        })
        .collect();
    assert_eq!(angles, [(0, 1, 2, 3), (0, 1, 0, 1), (2, 3, 2, 3)]);
}

/// WASM-8: arc- and circle-referencing constraints were stored as raw JSON
/// without checking them, so one that could never be built was accepted and
/// then failed every later `sketchSolve` and `sketchDof` of that sketch.
#[test]
fn unbuildable_deferred_constraints_are_refused_when_added() {
    let (mut kernel, sketch) = legacy_sketch(3);
    kernel.sketch_add_arc(sketch, 0, 1, 2).unwrap();
    kernel.sketch_add_circle(sketch, 0, 1.0).unwrap();

    for (json, entity) in [
        (r#"{"type":"pointOnArc","point":0,"arc":5}"#, "sketch arc"),
        (r#"{"type":"pointOnArc","point":9,"arc":0}"#, "sketch point"),
        (
            r#"{"type":"pointOnCircle","point":1,"circle":1}"#,
            "sketch circle",
        ),
        (
            r#"{"type":"tangentLineArc","line":[0,7],"arc":0,"point":1}"#,
            "sketch point",
        ),
        (
            r#"{"type":"equalRadiusArcCircle","arc":0,"circle":3}"#,
            "sketch circle",
        ),
    ] {
        let error = refused(&mut kernel, sketch, json);
        assert!(
            matches!(error, WasmError::InvalidHandle { entity: e, .. } if e == entity),
            "{json}: {error}"
        );
    }
    for json in [
        r#"{"type":"arcLength","arc":0}"#,
        r#"{"type":"tangentArcArc","arc1":0,"arc2":0}"#,
        r#"{"type":"tangentLineArc","line":[0],"arc":0,"point":1}"#,
        r#"{"type":"concentricArcCircle","arc":"0","circle":0}"#,
    ] {
        let error = refused(&mut kernel, sketch, json);
        assert!(
            matches!(error, WasmError::InvalidInput { .. }),
            "{json}: {error}"
        );
    }

    // The sketch stays solvable, and a well-formed deferred constraint is
    // still accepted and applied.
    kernel
        .sketch_add_constraint_impl(
            sketch,
            r#"{"type":"concentricArcCircle","arc":0,"circle":0}"#,
        )
        .unwrap();
    assert_eq!(stored(&kernel, sketch), (0, 1));
    let solved: serde_json::Value =
        serde_json::from_str(&kernel.sketch_solve(sketch, 50, 1e-10).unwrap()).unwrap();
    assert_eq!(solved["converged"], true, "{solved}");
    serde_json::from_str::<serde_json::Value>(&kernel.sketch_dof(sketch).unwrap()).unwrap();
}

fn batch(kernel: &mut RemusKernel, ops: &serde_json::Value) -> Vec<serde_json::Value> {
    serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap()
}

/// Ellipse at the origin with `b = 1`, `phi = 0` and a fixed point at
/// `(1, 0)` on it, so `(1/a)² = 1`, then a solve and a parameter read.
fn ellipse_sign_ops(a0: f64) -> serde_json::Value {
    let constraint = |c: serde_json::Value| serde_json::json!({"op": "gcsAddConstraint", "args": {"sketch": 0, "constraint": c}});
    serde_json::json!([
        {"op": "gcsNew", "args": {}},
        {"op": "gcsAddPoint", "args": {"sketch": 0, "x": 0.0, "y": 0.0, "fixed": true}},
        {"op": "gcsAddEllipse", "args": {"sketch": 0, "center": 0, "a": a0, "b": 1.0, "angle": 0.0}},
        constraint(serde_json::json!({"type": "ellipseAxisB", "ellipse": 0, "value": 1.0})),
        constraint(serde_json::json!({"type": "ellipseAngle", "ellipse": 0, "value": 0.0})),
        {"op": "gcsAddPoint", "args": {"sketch": 0, "x": 1.0, "y": 0.0, "fixed": true}},
        constraint(serde_json::json!({"type": "pointOnEllipse", "point": 1, "ellipse": 0})),
        {"op": "gcsSolve", "args": {"sketch": 0, "maxIterations": 200, "tolerance": 1e-10}},
        {"op": "gcsEllipseParams", "args": {"sketch": 0, "ellipse": 0}},
    ])
}

/// WASM-2: from `a = 5` the solve converged on the mirror solution `a = −1`;
/// from `a = 3` it reported convergence on the collapsed `a = 0`.
#[test]
fn gcs_solve_never_publishes_a_non_positive_ellipse_semiaxis() {
    let mut kernel = RemusKernel::new();
    let results = batch(&mut kernel, &ellipse_sign_ops(5.0));
    assert_eq!(results[7]["ok"]["converged"], true, "{:?}", results[7]);
    let a = results[8]["ok"][2].as_f64().unwrap();
    assert!((a - 1.0).abs() < 1e-9, "a = {a}");

    let mut kernel = RemusKernel::new();
    let results = batch(&mut kernel, &ellipse_sign_ops(3.0));
    let message = results[7]["error"]["message"].as_str().unwrap_or_default();
    assert!(
        message.contains("collapsed an ellipse semiaxis"),
        "{:?}",
        results[7]
    );
    assert_eq!(results[8]["ok"][2], 3.0, "pre-solve semiaxis restored");
}
