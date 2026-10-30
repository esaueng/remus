//! WASM contract tests for the store-local boolean switch, through
//! `executeBatch` (the JS surface).
//!
//! The switch is off by default and toggles the thread's setting, and a slot
//! cut, a rib fuse and a window intersect on a drilled plate each give the
//! same solid either way: volume, counts, material probes and a closed manifold mesh. Which path ran, and
//! that its result equals the general boolean's face by face, is pinned by
//! the operations crate's store-local oracle tests.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use serde_json::{Value, json};

use crate::kernel::RemusKernel;

/// Run one batch operation and return its `ok` payload.
fn call(kernel: &mut RemusKernel, op: &str, args: Value) -> Value {
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

fn translated(kernel: &mut RemusKernel, solid: u32, x: f64, y: f64, z: f64) -> u32 {
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
fn drilled_plate(kernel: &mut RemusKernel) -> u32 {
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

fn counts(kernel: &mut RemusKernel, solid: u32) -> (usize, usize) {
    let count = |kernel: &mut RemusKernel, op: &str| {
        call(kernel, op, json!({"solid": solid}))
            .as_array()
            .unwrap_or_else(|| panic!("{op} returned no list"))
            .len()
    };
    (count(kernel, "getSolidFaces"), count(kernel, "solidEdges"))
}

#[test]
fn store_local_switch_is_off_by_default_and_toggles() {
    let mut kernel = RemusKernel::new();
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
    let _restore = RestoreSwitch(remus_operations::boolean::thread_store_local_cuts_enabled());
    let mut kernel = RemusKernel::new();
    kernel.set_store_local_cuts(false);
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
    let inputs = kernel.serialize_solids(&[plate, tool]).unwrap();
    call(&mut kernel, "setStoreLocalCuts", json!({"enabled": true}));
    let local = handle(&call(
        &mut kernel,
        op,
        json!({"solidA": plate, "solidB": tool}),
    ));
    // Successful direct bindings run the same native work as the batch arm.
    // Error paths below stay in executeBatch because JsError is not native.
    let direct = match op {
        "cut" => kernel.cut(plate, tool).unwrap(),
        "fuse" => kernel.fuse(plate, tool).unwrap(),
        "intersect" => kernel.intersect_solids(plate, tool).unwrap(),
        _ => panic!("unexpected operation {op}"),
    };
    call(&mut kernel, "setStoreLocalCuts", json!({"enabled": false}));
    assert_eq!(kernel.serialize_solids(&[plate, tool]).unwrap(), inputs);

    assert_eq!(
        counts(&mut kernel, local),
        counts(&mut kernel, general),
        "{op}: counts"
    );
    assert_eq!(counts(&mut kernel, direct), counts(&mut kernel, general));
    for solid in [general, local, direct] {
        let id = kernel.resolve_solid(solid).unwrap();
        let report = remus_operations::validate::validate_solid(kernel.topo(), id).unwrap();
        assert!(report.is_valid(), "{op}: {report:?}");
        let mesh = remus_operations::tessellate::tessellate_solid(kernel.topo(), id, 0.05).unwrap();
        assert_eq!(remus_operations::tessellate::boundary_edge_count(&mesh), 0);
        assert_eq!(
            remus_operations::tessellate::non_manifold_edge_count(&mesh),
            0
        );
        let probes = match op {
            "cut" => [
                ((14.0, 14.0, 5.0), "outside"),
                ((30.0, 12.0, 5.0), "inside"),
                ((13.0, 14.0, 5.0), "boundary"),
            ],
            "fuse" => [
                ((14.0, 14.0, 11.0), "inside"),
                ((14.0, 14.0, 13.0), "outside"),
                ((17.0, 14.0, 11.0), "boundary"),
            ],
            "intersect" => [
                ((14.0, 14.0, 5.0), "inside"),
                ((30.0, 12.0, 5.0), "outside"),
                ((4.0, 14.0, 5.0), "boundary"),
            ],
            _ => unreachable!(),
        };
        for ((x, y, z), expected) in probes.into_iter().chain([((8.0, 8.0, 5.0), "outside")]) {
            assert_eq!(
                call(
                    &mut kernel,
                    "classifyPoint",
                    json!({"solid":solid,"x":x,"y":y,"z":z})
                ),
                json!(expected),
                "{op}: ({x}, {y}, {z})"
            );
        }
    }
    let volume = |kernel: &mut RemusKernel, solid: u32| {
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

struct RestoreSwitch(bool);

impl Drop for RestoreSwitch {
    fn drop(&mut self) {
        remus_operations::boolean::set_thread_store_local_cuts(self.0);
    }
}

#[test]
fn store_local_setting_is_shared_across_kernels_and_survives_item_refusal() {
    let _restore = RestoreSwitch(remus_operations::boolean::thread_store_local_cuts_enabled());
    let mut first = RemusKernel::new();
    first.set_store_local_cuts(false);
    call(&mut first, "setStoreLocalCuts", json!({"enabled":true}));
    // Constructing another kernel does not silently reset this module setting.
    let mut second = RemusKernel::new();
    assert!(second.store_local_cuts_enabled());
    let blank = handle(&call(
        &mut second,
        "makeBox",
        json!({"width":10,"height":10,"depth":10}),
    ));
    let inputs = second.serialize_solids(&[blank]).unwrap();
    let refused: Value = serde_json::from_str(
        &second.execute_batch_v2(
            &json!([
                {"op":"setStoreLocalCuts","args":{"enabled":"invalid"}},
                {"op":"cut","args":{"solidA":blank,"solidB":u32::MAX}},
                {"op":"storeLocalCutsEnabled","args":{}},
            ])
            .to_string(),
        ),
    )
    .unwrap();
    assert!(refused[0]["error"].is_object(), "{refused}");
    assert!(refused[1]["error"].is_object(), "{refused}");
    assert_eq!(refused[2]["ok"], true);
    assert_eq!(second.serialize_solids(&[blank]).unwrap(), inputs);
    assert!(first.store_local_cuts_enabled());
    second.set_store_local_cuts(false);
    assert_eq!(call(&mut first, "storeLocalCutsEnabled", json!({})), false);
}

#[test]
fn store_local_switch_preserves_evolution_journal_and_reference_contracts() {
    let _restore = RestoreSwitch(remus_operations::boolean::thread_store_local_cuts_enabled());
    for op in ["cutWithEntityEvolution", "cutJournaled"] {
        let run = |enabled: bool| {
            let mut kernel = RemusKernel::new();
            kernel.set_store_local_cuts(false);
            let plate = drilled_plate(&mut kernel);
            let tool = handle(&call(
                &mut kernel,
                "makeBox",
                json!({"width":2,"height":6,"depth":20}),
            ));
            translated(&mut kernel, tool, 13.0, 11.0, -5.0);
            let source_faces: Vec<u32> = [plate, tool]
                .into_iter()
                .flat_map(|solid| {
                    call(&mut kernel, "getSolidFaces", json!({"solid":solid}))
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(handle)
                        .collect::<Vec<_>>()
                })
                .collect();
            let inputs = kernel.face_content_digests_impl(&source_faces).unwrap();
            call(&mut kernel, "setStoreLocalCuts", json!({"enabled":enabled}));
            let output = call(&mut kernel, op, json!({"solidA":plate,"solidB":tool}));
            // Journaled operations legitimately extend the operands' history.
            // Their exact geometric content must remain unchanged.
            assert_eq!(
                kernel.face_content_digests_impl(&source_faces).unwrap(),
                inputs
            );
            let solid = handle(&output["solid"]);
            let material = call(
                &mut kernel,
                "classifyPoint",
                json!({"solid":solid,"x":14,"y":14,"z":5}),
            );
            assert_eq!(material, "outside");
            let resolved = if op == "cutJournaled" {
                let reference = call(
                    &mut kernel,
                    "makeOperationOutputRef",
                    json!({"op":output["op"],"kind":"face","index":0}),
                );
                let resolved = call(&mut kernel, "resolveRef", json!({"ref":reference["ref"]}));
                assert_eq!(resolved["status"], "bound", "{resolved}");
                assert_eq!(resolved["provenance"], "construction", "{resolved}");
                resolved
            } else {
                Value::Null
            };
            let history = call(&mut kernel, "journalSummary", json!({}));
            let geometry = kernel.serialize_solids(&[solid]).unwrap();
            kernel.set_store_local_cuts(false);
            (output, history, resolved, geometry)
        };
        // These entry points intentionally retain the general path and its
        // entity ordering. Turning on plain-boolean locality cannot silently
        // alter construction-derived history or persistent-reference targets.
        assert_eq!(run(false), run(true), "{op}: switch changed its contract");
    }
}
