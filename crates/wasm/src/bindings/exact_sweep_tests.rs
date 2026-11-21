//! WASM contract tests for `sweepExact`, built the way OpenZCAD builds a
//! sweep: a circle profile from `makeCircleEdge` / `makeWire` /
//! `makeFaceFromWire`, a path of `makeLineEdge` and `makeCircleArc3d` edges.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::f64::consts::{FRAC_PI_2, PI};

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

/// A radius-2 circle at the origin, square to +X.
fn profile(kernel: &mut RemusKernel) -> u32 {
    let edge = kernel
        .make_circle_edge(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0)
        .unwrap();
    let wire = kernel.make_wire_impl(&[edge], true).unwrap();
    kernel.make_face_from_wire(wire).unwrap()
}

/// Line 20 along +X, a quarter arc of radius 10 turning to +Y, line 20.
fn bend(kernel: &mut RemusKernel) -> Vec<u32> {
    vec![
        kernel
            .make_line_edge(0.0, 0.0, 0.0, 20.0, 0.0, 0.0)
            .unwrap(),
        kernel
            .make_circle_arc_3d(
                20.0, 0.0, 0.0, 30.0, 10.0, 0.0, 20.0, 10.0, 0.0, 0.0, 0.0, 1.0,
            )
            .unwrap(),
        kernel
            .make_line_edge(30.0, 10.0, 0.0, 30.0, 30.0, 0.0)
            .unwrap(),
    ]
}

fn volume(kernel: &RemusKernel, solid: u32) -> f64 {
    let id = kernel.resolve_solid(solid).unwrap();
    remus_operations::measure::mass_properties(kernel.topo(), id)
        .unwrap()
        .mass
}

#[test]
fn sweep_exact_builds_the_exact_pipe_bend_directly_and_in_a_batch() {
    let exact = PI * 4.0 * (40.0 + 10.0 * FRAC_PI_2);

    let mut kernel = RemusKernel::new();
    let face = profile(&mut kernel);
    let path = bend(&mut kernel);
    let solid = kernel.sweep_exact_impl(face, &path).unwrap();
    let measured = volume(&kernel, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );

    let mut kernel = RemusKernel::new();
    let face = profile(&mut kernel);
    let path = bend(&mut kernel);
    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{"op": "sweepExact", "args": {"profile": face, "edges": path}}])
                .to_string(),
        ),
    )
    .unwrap();
    let solid = u32::try_from(out[0]["ok"].as_u64().expect("solid handle")).unwrap();
    let measured = volume(&kernel, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );
}

/// A line meeting an arc at a square corner: refused (a corner is mitred
/// only between straight runs), directly and in a batch, with the session
/// untouched.
#[test]
fn refused_sweep_exact_leaves_the_session_unchanged() {
    let mut kernel = RemusKernel::new();
    let face = profile(&mut kernel);
    let path = vec![
        kernel
            .make_line_edge(0.0, 0.0, 0.0, 10.0, 0.0, 0.0)
            .unwrap(),
        // From (10, 0) clockwise about +Z to (15, 5): it leaves along +Y.
        kernel
            .make_circle_arc_3d(
                10.0, 0.0, 0.0, 15.0, 5.0, 0.0, 15.0, 0.0, 0.0, 0.0, 0.0, -1.0,
            )
            .unwrap(),
    ];
    let before = counts(&kernel);
    let refused = kernel.sweep_exact_impl(face, &path);
    assert!(
        refused
            .as_ref()
            .is_err_and(|e| e.to_string().contains("next to an arc")),
        "{refused:?}"
    );
    assert_eq!(counts(&kernel), before);

    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{"op": "sweepExact", "args": {"profile": face, "edges": path}}])
                .to_string(),
        ),
    )
    .unwrap();
    assert!(out[0].get("error").is_some(), "{out:?}");
    assert_eq!(counts(&kernel), before);
}

/// A radius-0.1 tube along 10 units of +X, then back 10 units at a 178°
/// hairpin (past the former 175° limit): mitred exactly, directly and in a
/// batch.
#[test]
fn sweep_exact_mitres_a_hairpin_past_the_old_limit_directly_and_in_a_batch() {
    let exact = PI * 0.01 * 20.0;
    let turn = 178.0_f64.to_radians();
    let build = |kernel: &mut RemusKernel| -> (u32, Vec<u32>) {
        let edge = kernel
            .make_circle_edge(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1)
            .unwrap();
        let wire = kernel.make_wire_impl(&[edge], true).unwrap();
        let face = kernel.make_face_from_wire(wire).unwrap();
        let path = vec![
            kernel
                .make_line_edge(0.0, 0.0, 0.0, 10.0, 0.0, 0.0)
                .unwrap(),
            kernel
                .make_line_edge(
                    10.0,
                    0.0,
                    0.0,
                    10.0f64.mul_add(turn.cos(), 10.0),
                    10.0 * turn.sin(),
                    0.0,
                )
                .unwrap(),
        ];
        (face, path)
    };

    let mut kernel = RemusKernel::new();
    let (face, path) = build(&mut kernel);
    let solid = kernel.sweep_exact_impl(face, &path).unwrap();
    let measured = volume(&kernel, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );

    let mut kernel = RemusKernel::new();
    let (face, path) = build(&mut kernel);
    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{"op": "sweepExact", "args": {"profile": face, "edges": path}}])
                .to_string(),
        ),
    )
    .unwrap();
    let solid = u32::try_from(out[0]["ok"].as_u64().expect("solid handle")).unwrap();
    let measured = volume(&kernel, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );
}

/// `sweepExactWithOptions`: a radius-2 circle along 10 units of +X, scaled
/// to 1.5 and turned a quarter turn, is the exact cone frustum, directly and
/// in a batch; with no options it is `sweepExact`.
#[test]
fn sweep_exact_with_options_scales_and_turns_directly_and_in_a_batch() {
    let frustum = PI * 4.0 * 10.0 * 1.5f64.mul_add(1.5, 2.5) / 3.0;
    let build = |kernel: &mut RemusKernel| -> (u32, Vec<u32>) {
        let face = profile(kernel);
        let path = vec![
            kernel
                .make_line_edge(0.0, 0.0, 0.0, 10.0, 0.0, 0.0)
                .unwrap(),
        ];
        (face, path)
    };
    let options = super::operations::parse_exact_sweep_options(&serde_json::json!({
        "twist": FRAC_PI_2,
        "endScale": 1.5,
    }))
    .unwrap();

    let mut kernel = RemusKernel::new();
    let (face, path) = build(&mut kernel);
    let solid = kernel
        .sweep_exact_with_options_impl(face, &path, &options)
        .unwrap();
    let measured = volume(&kernel, solid);
    assert!(
        (measured - frustum).abs() / frustum < 1e-9,
        "{measured} vs {frustum}"
    );

    let mut kernel = RemusKernel::new();
    let (face, path) = build(&mut kernel);
    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{
                "op": "sweepExactWithOptions",
                "args": {"profile": face, "edges": path, "options": {"endScale": 1.5, "twist": FRAC_PI_2}}
            }, {
                "op": "sweepExactWithOptions",
                "args": {"profile": face, "edges": path}
            }])
            .to_string(),
        ),
    )
    .unwrap();
    let scaled = u32::try_from(out[0]["ok"].as_u64().expect("solid handle")).unwrap();
    let measured = volume(&kernel, scaled);
    assert!(
        (measured - frustum).abs() / frustum < 1e-9,
        "{measured} vs {frustum}"
    );
    let plain = u32::try_from(out[1]["ok"].as_u64().expect("solid handle")).unwrap();
    let cylinder = PI * 4.0 * 10.0;
    let measured = volume(&kernel, plain);
    assert!(
        (measured - cylinder).abs() / cylinder < 1e-9,
        "{measured} vs {cylinder}"
    );
}

/// An unknown option or a non-number is refused before anything is built,
/// so a kernel never silently ignores an option it cannot build; a twist
/// along a bend is refused with the session unchanged.
#[test]
fn sweep_exact_with_options_refuses_unknown_options_and_bends() {
    for (options, phrase) in [
        (
            serde_json::json!({"twist": 1.0, "posture": "parallel"}),
            "unknown option",
        ),
        (
            serde_json::json!({"orientation": "sideways"}),
            "must be \"normal\" or \"parallel\"",
        ),
        (serde_json::json!({"twist": "a lot"}), "must be a number"),
        (serde_json::json!([1.0]), "must be an object"),
    ] {
        let refused = super::operations::parse_exact_sweep_options(&options);
        assert!(
            refused.as_ref().is_err_and(|e| e.contains(phrase)),
            "{options}: {refused:?}"
        );
    }

    let mut kernel = RemusKernel::new();
    let face = profile(&mut kernel);
    let path = bend(&mut kernel);
    let before = counts(&kernel);
    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{
                "op": "sweepExactWithOptions",
                "args": {"profile": face, "edges": path, "options": {"twist": 1.0}}
            }, {
                "op": "sweepExactWithOptions",
                "args": {"profile": face, "edges": path, "options": {"bogus": 1.0}}
            }])
            .to_string(),
        ),
    )
    .unwrap();
    let error = out[0]["error"].to_string();
    assert!(error.contains("only along a straight path"), "{out:?}");
    assert!(
        out[1]["error"].to_string().contains("unknown option"),
        "{out:?}"
    );
    assert_eq!(counts(&kernel), before);
}

/// `"orientation": "parallel"`: the radius-2 disc square to +X, translated
/// unturned through a 45° corner (a mitred sweep would turn it), is the
/// disc's area times the 20-unit rise along +X.
#[test]
fn sweep_exact_with_options_keeps_the_profile_parallel_through_a_corner() {
    let mut kernel = RemusKernel::new();
    let face = profile(&mut kernel);
    let path = vec![
        kernel
            .make_line_edge(0.0, 0.0, 0.0, 10.0, 0.0, 0.0)
            .unwrap(),
        kernel
            .make_line_edge(10.0, 0.0, 0.0, 20.0, 10.0, 0.0)
            .unwrap(),
    ];
    let out: Vec<serde_json::Value> = serde_json::from_str(
        &kernel.execute_batch(
            &serde_json::json!([{
                "op": "sweepExactWithOptions",
                "args": {"profile": face, "edges": path, "options": {"orientation": "parallel"}}
            }])
            .to_string(),
        ),
    )
    .unwrap();
    let solid = u32::try_from(out[0]["ok"].as_u64().expect("solid handle")).unwrap();
    let exact = PI * 4.0 * 20.0;
    let measured = volume(&kernel, solid);
    assert!(
        (measured - exact).abs() / exact < 1e-9,
        "{measured} vs {exact}"
    );
}
