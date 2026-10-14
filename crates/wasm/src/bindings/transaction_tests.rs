//! Coordinated boolean batch commits and retained session-state tests.

#![allow(clippy::unwrap_used)]
use crate::kernel::BrepKernel;

#[test]
fn coordinated_boolean_items_commit_independently_and_keep_session_state() {
    let mut k = BrepKernel::new();
    let a = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    let b = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    k.transform_solid_binding(
        b,
        vec![
            1.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
    )
    .unwrap();
    k.assembly_new("retained").unwrap();
    let sketch = k.sketch_new().unwrap();
    k.sketch_add_point(sketch, 1.0, 2.0, false).unwrap();
    k.gcs_new().unwrap();
    let checkpoint = k.checkpoint().unwrap();
    let input = k.serialize_solids(&[a, b]).unwrap();
    let session = format!(
        "{:?}{:?}{:?}{:?}{}",
        k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
    );
    let mut retired = Vec::new();
    for _ in 0..3 {
        let rows: serde_json::Value = serde_json::from_str(&k.execute_batch(&format!(
            r#"[
            {{"op":"fuse","args":{{"solidA":{a},"solidB":{b}}}}},
            {{"op":"cut","args":{{"solidA":{a},"solidB":{a}}}}},
            {{"op":"fuse","args":{{"solidA":{a},"solidB":{b}}}}}
        ]"#
        )))
        .unwrap();
        assert!(rows[1]["error"].is_string());
        for row in [&rows[0], &rows[2]] {
            let id = u32::try_from(row["ok"].as_u64().unwrap()).unwrap();
            assert!((k.volume(id, 0.01).unwrap() - 1.5).abs() < 1e-8);
            retired.push(id);
        }
        assert_eq!(k.serialize_solids(&[a, b]).unwrap(), input);
        assert_eq!(
            format!(
                "{:?}{:?}{:?}{:?}{}",
                k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
            ),
            session
        );
        k.restore(checkpoint).unwrap();
        k.make_box_solid(2.0, 2.0, 2.0).unwrap();
        for &id in &retired {
            assert!(k.resolve_solid(id).is_err());
        }
        k.restore(checkpoint).unwrap();
    }
    assert_eq!(k.serialize_solids(&[a, b]).unwrap(), input);
}

mod exact_boolean_gate_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use crate::kernel::BrepKernel;

    fn batch(
        kernel: &mut BrepKernel,
        v2: bool,
        operations: serde_json::Value,
    ) -> serde_json::Value {
        let response = if v2 {
            kernel.execute_batch_v2(&operations.to_string())
        } else {
            kernel.execute_batch(&operations.to_string())
        };
        serde_json::from_str(&response).unwrap()
    }

    fn cavity_fixture(kernel: &mut BrepKernel, tool_side: f64, second_piece: bool) -> (u32, u32) {
        let wall = (10.0 - tool_side) * 0.5;
        let x = wall + if second_piece { 20.0 } else { 0.0 };
        let results = batch(
            kernel,
            false,
            serde_json::json!([
                {"op":"makeBox","args":{"width":10.0,"height":10.0,"depth":10.0}},
                {"op":"makeBox","args":{"width":10.0,"height":10.0,"depth":10.0}},
                {"op":"makeBox","args":{"width":tool_side,"height":tool_side,"depth":tool_side}},
                {"op":"transform","args":{"solid":1,"matrix":[1,0,0,20,0,1,0,0,0,0,1,0,0,0,0,1]}},
                {"op":"transform","args":{"solid":2,"matrix":[1,0,0,x,0,1,0,wall,0,0,1,wall,0,0,0,1]}},
                {"op":"fuse","args":{"solidA":0,"solidB":1}},
            ]),
        );
        assert!(
            results
                .as_array()
                .unwrap()
                .iter()
                .all(|r| r.get("error").is_none()),
            "{results}"
        );
        (
            u32::try_from(results[5]["ok"].as_u64().unwrap()).unwrap(),
            2,
        )
    }

    #[test]
    fn exact_boolean_multi_piece_cavity_survives_both_batch_envelopes() {
        check_cavity_batch(2.0, false);
    }

    #[test]
    fn exact_boolean_thin_multi_piece_cavity_survives_both_batch_envelopes() {
        for second_piece in [false, true] {
            check_cavity_batch(9.98, second_piece);
        }
    }

    fn check_cavity_batch(tool_side: f64, second_piece: bool) {
        for operation in ["cutWithEntityEvolution", "cutJournaled"] {
            for v2 in [false, true] {
                let mut kernel = BrepKernel::new();
                let (blank, tool) = cavity_fixture(&mut kernel, tool_side, second_piece);
                let response = batch(
                    &mut kernel,
                    v2,
                    serde_json::json!([
                        {"op":operation,"args":{"solidA":blank,"solidB":tool}},
                    ]),
                );
                assert!(
                    response[0].get("error").is_none(),
                    "{operation} v2={v2}: {response}"
                );
                let payload = &response[0]["ok"];
                let solid = u32::try_from(payload["solid"].as_u64().unwrap()).unwrap();
                let expected = 2000.0 - tool_side.powi(3);
                assert!((kernel.volume(solid, 1e-4).unwrap() - expected).abs() < 1e-6);
                let id = kernel.resolve_solid(solid).unwrap();
                assert_eq!(kernel.topo().solid(id).unwrap().inner_shells().len(), 1);
                assert!(
                    remus_operations::validate::validate_solid(kernel.topo(), id)
                        .unwrap()
                        .is_valid()
                );
                let piece_x = if second_piece { 20.0 } else { 0.0 };
                let wall = (10.0 - tool_side) * 0.5;
                assert_eq!(
                    remus_operations::classify::classify_points(
                        kernel.topo(),
                        id,
                        &[
                            remus_math::vec::Point3::new(piece_x + 5.0, 5.0, 5.0),
                            remus_math::vec::Point3::new(piece_x + wall * 0.5, 5.0, 5.0),
                            remus_math::vec::Point3::new(25.0 - piece_x, 5.0, 5.0),
                        ],
                        0.01,
                        1e-7,
                    )
                    .unwrap(),
                    [
                        remus_operations::classify::PointClassification::Outside,
                        remus_operations::classify::PointClassification::Inside,
                        remus_operations::classify::PointClassification::Inside,
                    ]
                );
                if operation == "cutJournaled" {
                    let summary: serde_json::Value =
                        serde_json::from_str(&kernel.journal_summary()).unwrap();
                    assert_eq!(
                        summary.as_array().unwrap().last().unwrap()["detail"]["origin"],
                        "construction"
                    );
                    let resolved = batch(
                        &mut kernel,
                        v2,
                        serde_json::json!([
                            {"op":"resolveOperationOutput","args":{"op":payload["op"],"kind":"face","index":17}},
                        ]),
                    );
                    assert_eq!(resolved[0]["ok"]["status"], "bound");
                    assert_eq!(resolved[0]["ok"]["provenance"], "construction");
                } else {
                    assert_eq!(payload["evolution"]["faces"].as_array().unwrap().len(), 18);
                    assert_eq!(
                        payload["evolution"]["edges"].as_array().unwrap().len(),
                        remus_topology::explorer::solid_edges(kernel.topo(), id)
                            .unwrap()
                            .len()
                    );
                    assert_eq!(
                        payload["evolution"]["vertices"].as_array().unwrap().len(),
                        remus_topology::explorer::solid_vertices(kernel.topo(), id)
                            .unwrap()
                            .len()
                    );
                }
            }
        }
    }

    #[test]
    fn exact_boolean_gate_refusals_preserve_operands_and_history_in_batch() {
        for operation in ["fuseWithEntityEvolution", "fuseJournaled"] {
            for v2 in [false, true] {
                let mut kernel = BrepKernel::new();
                let a = kernel.make_sphere_solid(3.0, 16).unwrap();
                let b = kernel.copy_solid(a).unwrap();
                let before = kernel.serialize_solids(&[a, b]).unwrap();
                let history = kernel.journal_summary();
                let counts = (
                    kernel.topo().num_vertices(),
                    kernel.topo().num_edges(),
                    kernel.topo().num_faces(),
                    kernel.topo().num_solids(),
                );
                let response = batch(
                    &mut kernel,
                    v2,
                    serde_json::json!([
                        {"op":operation,"args":{"solidA":a,"solidB":b}},
                    ]),
                );
                assert!(
                    response[0].get("error").is_some(),
                    "{operation} v2={v2}: {response}"
                );
                if v2 {
                    assert_eq!(response[0]["error"]["code"], "operation_failed");
                    assert_eq!(
                        response[0]["error"]["details"]["kernelCode"],
                        "exact_only_unattainable"
                    );
                }
                assert_eq!(kernel.serialize_solids(&[a, b]).unwrap(), before);
                assert_eq!(kernel.journal_summary(), history);
                assert_eq!(
                    (
                        kernel.topo().num_vertices(),
                        kernel.topo().num_edges(),
                        kernel.topo().num_faces(),
                        kernel.topo().num_solids()
                    ),
                    counts
                );
            }
        }
    }
}
