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

use crate::kernel::RemusKernel;

fn run(kernel: &mut RemusKernel, ops: &[serde_json::Value]) -> Vec<serde_json::Value> {
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
fn edge_where(kernel: &mut RemusKernel, solid: u32, at: impl Fn(f64, f64, f64) -> bool) -> u32 {
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
    let mut kernel = RemusKernel::new();
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

/// The opposite order with a larger second radius: the vertical edge runs
/// into the top fillet's band, which no 2 mm ball can follow. The batch op
/// rebuilds the 1 mm band after the 2 mm fillet, as filleting the vertical
/// edge first would, instead of refusing.
#[test]
fn fillet_vertical_edge_larger_than_earlier_top_fillet() {
    let (dx, dy, dz) = (9.5, 12.5, 8.5);
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    let mut kernel = RemusKernel::new();
    let out = run(
        &mut kernel,
        &[op(
            "makeBox",
            serde_json::json!({"width": dx, "height": dy, "depth": dz}),
        )],
    );
    let solid = ok_handle(&out[0]);

    let top = edge_where(&mut kernel, solid, |x, _, z| near(x, dx) && near(z, dz));
    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": solid, "radius": 1.0, "edges": [top]}),
        )],
    );
    let first = ok_handle(&out[0]);

    let vertical = edge_where(&mut kernel, first, |x, y, _| near(x, dx) && near(y, 0.0));
    let out = run(
        &mut kernel,
        &[op(
            "fillet",
            serde_json::json!({"solid": first, "radius": 2.0, "edges": [vertical]}),
        )],
    );
    let second = ok_handle(&out[0]);
    assert_ne!(second, first);
    assert_eq!(
        kernel.validate_solid(second).unwrap(),
        0,
        "no validation errors"
    );

    let volume = |kernel: &mut RemusKernel, solid: u32| {
        let out = run(
            kernel,
            &[op(
                "volume",
                serde_json::json!({"solid": solid, "deflection": 0.01}),
            )],
        );
        out[0]["ok"].as_f64().unwrap()
    };
    let before = volume(&mut kernel, first);
    let after = volume(&mut kernel, second);
    assert!(after < before, "a convex fillet removes material");
}

fn bounded_tab_source(kernel: &mut RemusKernel) -> (u32, Vec<u32>) {
    let solid = kernel
        .deserialize_solid(include_bytes!(
            "../../../operations/tests/data/rollover_tabbed_source.arena.json"
        ))
        .unwrap();
    let selected: Vec<_> = kernel
        .get_solid_edges(solid)
        .unwrap()
        .into_iter()
        .filter(|&handle| {
            let id = kernel.resolve_edge(handle).unwrap();
            let edge = kernel.topo().edge(id).unwrap();
            let a = kernel.topo().vertex(edge.start()).unwrap().point();
            let b = kernel.topo().vertex(edge.end()).unwrap().point();
            (a.y() - 10.0).abs() < 1e-8
                && (b.y() - 10.0).abs() < 1e-8
                && (a.z() - 4.0).abs() < 1e-8
                && (b.z() - 4.0).abs() < 1e-8
        })
        .collect();
    assert_eq!(selected.len(), 2);
    (solid, selected)
}

#[test]
fn bounded_tab_rollover_native_direct_detailed_both_batches_and_history() {
    for surface in ["direct", "detailed", "batch", "batchV2", "history"] {
        let mut kernel = RemusKernel::new();
        let (source, seeds) = bounded_tab_source(&mut kernel);
        let other = kernel.make_box_solid(2.0, 3.0, 5.0).unwrap();
        let before = kernel.serialize_solid(source).unwrap();
        let other_before = kernel.serialize_solid(other).unwrap();
        let result = match surface {
            "direct" => kernel.fillet_solid(source, seeds.clone(), 3.0).unwrap(),
            "detailed" => {
                let payload =
                    serde_json::to_value(kernel.fillet_detailed_impl(source, &seeds, 3.0, true))
                        .unwrap();
                assert_eq!(payload["status"], "ok");
                assert_eq!(payload["details"]["quality"], "exact");
                assert_eq!(payload["details"]["engine"], "rollingBall");
                u32::try_from(payload["value"].as_u64().unwrap()).unwrap()
            }
            "history" => {
                let payload = serde_json::to_value(
                    kernel
                        .fillet_with_evolution_impl(source, seeds.clone(), 3.0)
                        .unwrap(),
                )
                .unwrap();
                assert_eq!(payload["source"]["faces"].as_array().unwrap().len(), 32);
                assert_eq!(payload["result"]["faces"].as_array().unwrap().len(), 28);
                assert_eq!(payload["evolution"]["provenance"], "construction");
                assert_eq!(payload["evolution"]["deleted"].as_array().unwrap().len(), 2);
                assert!(
                    payload["evolution"]["unresolvedSources"]
                        .as_array()
                        .unwrap()
                        .is_empty()
                );
                assert!(
                    payload["evolution"]["unresolvedResults"]
                        .as_array()
                        .unwrap()
                        .is_empty()
                );
                u32::try_from(payload["result"]["solid"].as_u64().unwrap()).unwrap()
            }
            _ => {
                let program=serde_json::json!([{"op":"fillet","args":{"solid":source,"edges":seeds,"radius":3.0}}]).to_string();
                let output = if surface == "batch" {
                    kernel.execute_batch(&program)
                } else {
                    kernel.execute_batch_v2(&program)
                };
                let payload: serde_json::Value = serde_json::from_str(&output).unwrap();
                ok_handle(&payload[0])
            }
        };
        assert_ne!(result, source);
        assert_eq!(kernel.get_solid_faces(result).unwrap().len(), 28);
        let solid = kernel.resolve_solid(result).unwrap();
        let topo = kernel.topo();
        let mut uses = 0;
        for face in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
            for oe in topo
                .wire(topo.face(face).unwrap().outer_wire())
                .unwrap()
                .edges()
            {
                assert!(
                    topo.pcurve_oriented(oe.edge(), face, oe.is_forward())
                        .is_some()
                );
                remus_topology::validation::validate_same_parameter_strict(
                    topo,
                    oe.edge(),
                    face,
                    oe.is_forward(),
                    1e-7,
                    2,
                )
                .unwrap();
                remus_topology::validation::validate_same_range_strict(
                    topo,
                    oe.edge(),
                    face,
                    oe.is_forward(),
                    1e-7,
                )
                .unwrap();
                uses += 1;
            }
        }
        assert_eq!(uses, 156);
        assert_eq!(kernel.serialize_solid(source).unwrap(), before);
        assert_eq!(kernel.serialize_solid(other).unwrap(), other_before);
        let counts = (
            kernel.topo().num_vertices(),
            kernel.topo().num_edges(),
            kernel.topo().num_faces(),
            kernel.topo().num_coedges(),
        );
        let refused =
            serde_json::to_value(kernel.fillet_detailed_impl(source, &seeds, 4.1, true)).unwrap();
        assert_eq!(refused["status"], "error");
        assert_eq!(refused["code"], "cliff-encountered");
        assert_eq!(
            (
                kernel.topo().num_vertices(),
                kernel.topo().num_edges(),
                kernel.topo().num_faces(),
                kernel.topo().num_coedges()
            ),
            counts
        );
        assert_eq!(kernel.serialize_solid(source).unwrap(), before);
        assert_eq!(kernel.serialize_solid(other).unwrap(), other_before);
    }
}
