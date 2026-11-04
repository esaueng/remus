//! Browser-facing spline chamfer contracts through both batch envelopes.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::{handles::edge_id_to_u32, kernel::RemusKernel};
use remus_topology::{edge::EdgeCurve, explorer::solid_edges};
use serde_json::{Value, json};

fn call(kernel: &mut RemusKernel, v2: bool, op: &str, args: Value) -> Value {
    let request = json!([{"op":op,"args":args}]).to_string();
    let response = if v2 {
        kernel.execute_batch_v2(&request)
    } else {
        kernel.execute_batch(&request)
    };
    let response: Value = serde_json::from_str(&response).unwrap();
    assert!(response[0].get("error").is_none(), "{op}: {response}");
    response[0]["ok"].clone()
}

fn handle(value: Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

fn fixture(kernel: &mut RemusKernel, v2: bool) -> (u32, u32) {
    fixture_variant(kernel, v2, false)
}

fn fixture_variant(kernel: &mut RemusKernel, v2: bool, rim: bool) -> (u32, u32) {
    let mut edges = Vec::new();
    for (a, b) in [([0, 0, 0], [10, 0, 0]), ([10, 0, 0], [10, 10, 0])] {
        edges.push(handle(call(
            kernel,
            v2,
            "makeLineEdge",
            json!({"x1":a[0],"y1":a[1],"z1":a[2],"x2":b[0],"y2":b[1],"z2":b[2]}),
        )));
    }
    let spline = if rim {
        json!({"startX":10,"startY":10,"startZ":0,"endX":0,"endY":10,"endZ":0,"degree":3,"knots":[0,0,0,0,0.4,0.7,1,1,1,1],"controlPoints":[10,10,0,8,10,0,7,14,0,3,14,0,2,10,0,0,10,0],"weights":[1,1,1,1,1,1]})
    } else {
        json!({"startX":10,"startY":10,"startZ":0,"endX":0,"endY":10,"endZ":0,"degree":2,"knots":[0,0,0,1,1,1],"controlPoints":[10,10,0,5,13,0,0,10,0],"weights":[1,1,1]})
    };
    edges.push(handle(call(kernel, v2, "makeNurbsEdge", spline)));
    edges.push(handle(call(
        kernel,
        v2,
        "makeLineEdge",
        json!({"x1":0,"y1":10,"z1":0,"x2":0,"y2":0,"z2":0}),
    )));
    let wire = handle(call(
        kernel,
        v2,
        "makeWire",
        json!({"edges":edges,"closed":true}),
    ));
    let face = handle(call(
        kernel,
        v2,
        "makePlanarFaceFromWire",
        json!({"wire":wire}),
    ));
    let solid = handle(call(
        kernel,
        v2,
        "extrude",
        json!({"face":face,"distance":20}),
    ));
    let id = kernel.resolve_solid(solid).unwrap();
    let edge = solid_edges(kernel.topo(), id)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let e = kernel.topo().edge(e).unwrap();
            if rim {
                matches!(e.curve(), EdgeCurve::NurbsCurve(_))
                    && [e.start(), e.end()]
                        .into_iter()
                        .all(|v| (kernel.topo().vertex(v).unwrap().point().z() - 20.0).abs() < 1e-8)
            } else {
                matches!(e.curve(), EdgeCurve::Line)
                    && [e.start(), e.end()].into_iter().all(|v| {
                        let p = kernel.topo().vertex(v).unwrap().point();
                        p.x().abs() < 1e-8 && (p.y() - 10.0).abs() < 1e-8
                    })
            }
        })
        .unwrap();
    (solid, edge_id_to_u32(edge))
}

#[test]
fn spline_chamfers_succeed_through_every_public_batch_mode() {
    for v2 in [false, true] {
        for mode in ["chamfer", "chamferV2", "chamferDistanceAngle"] {
            let mut kernel = RemusKernel::new();
            let (solid, edge) = fixture(&mut kernel, v2);
            let before = kernel.serialize_solids(&[solid]).unwrap();
            let mut args = json!({"solid":solid,"edges":[edge],"distance":1.0,"d1":1.0,"d2":1.0,"angle":std::f64::consts::FRAC_PI_4});
            if mode == "chamferV2" {
                args.as_object_mut().unwrap().remove("distance");
            }
            let result = handle(call(&mut kernel, v2, mode, args));
            assert_ne!(result, solid);
            assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
            let volume = call(
                &mut kernel,
                v2,
                "volume",
                json!({"solid":result,"deflection":0.001}),
            )
            .as_f64()
            .unwrap();
            assert!(
                (volume - 2_191.093_119_880_501).abs() < 0.01,
                "{mode}, v2={v2}: {volume}"
            );
            let id = kernel.resolve_solid(result).unwrap();
            assert!(
                remus_operations::validate::validate_solid(kernel.topo(), id)
                    .unwrap()
                    .is_valid()
            );
            let pcurves = remus_topology::validation::validate_solid_pcurve_contracts(
                kernel.topo(),
                id,
                1e-7,
                0,
            )
            .unwrap();
            assert!(pcurves.validated_uses >= 18);
        }
    }
}

#[test]
fn curved_spline_rims_succeed_in_both_batch_envelopes() {
    for v2 in [false, true] {
        let mut kernel = RemusKernel::new();
        let (solid, edge) = fixture_variant(&mut kernel, v2, true);
        let before = kernel.serialize_solids(&[solid]).unwrap();
        let result = handle(call(
            &mut kernel,
            v2,
            "chamferV2",
            json!({"solid":solid,"edges":[edge],"d1":0.3,"d2":0.5}),
        ));
        assert_ne!(result, solid);
        assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
        let id = kernel.resolve_solid(result).unwrap();
        assert_eq!(
            remus_topology::explorer::solid_faces(kernel.topo(), id)
                .unwrap()
                .len(),
            7
        );
        assert!(
            remus_operations::validate::validate_solid(kernel.topo(), id)
                .unwrap()
                .is_valid()
        );
        let pcurves =
            remus_topology::validation::validate_solid_pcurve_contracts(kernel.topo(), id, 1e-7, 0)
                .unwrap();
        assert!(pcurves.validated_uses >= 18);
    }
}

#[test]
fn oversized_spline_chamfer_refuses_without_poisoning_the_batch_session() {
    for v2 in [false, true] {
        let mut kernel = RemusKernel::new();
        let (solid, edge) = fixture(&mut kernel, v2);
        let before = kernel.serialize_solids(&[solid]).unwrap();
        let slots = kernel.topo().allocated_slot_count();
        let request =
            json!([{"op":"chamferV2","args":{"solid":solid,"edges":[edge],"d1":20,"d2":20}}])
                .to_string();
        let response = if v2 {
            kernel.execute_batch_v2(&request)
        } else {
            kernel.execute_batch(&request)
        };
        let response: Value = serde_json::from_str(&response).unwrap();
        assert!(response[0].get("error").is_some());
        assert_eq!(kernel.serialize_solids(&[solid]).unwrap(), before);
        assert_eq!(kernel.topo().allocated_slot_count(), slots);
        assert!(
            call(
                &mut kernel,
                v2,
                "volume",
                json!({"solid":solid,"deflection":0.001})
            )
            .as_f64()
            .unwrap()
                > 2199.9
        );
    }
}
