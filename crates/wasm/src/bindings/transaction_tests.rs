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

// ── Checkpoint restore of the value the kernel still holds ─────────

/// Per-arena slot high-water marks.
pub(super) fn slot_marks(t: &remus_topology::Topology) -> [usize; 8] {
    [
        t.vertices().slot_len(),
        t.edges().slot_len(),
        t.wires().slot_len(),
        t.faces().slot_len(),
        t.shells().slot_len(),
        t.solids().slot_len(),
        t.compounds().slot_len(),
        t.compsolids().slot_len(),
    ]
}

/// Slot and journal-id high-water marks: never rewound, never reissued.
pub(super) fn high_water(t: &remus_topology::Topology) -> String {
    let journal = t.journal().snapshot();
    format!(
        "{:?} {} {} {}",
        slot_marks(t),
        t.allocated_slot_count(),
        journal.next_op,
        journal.next_ordinal
    )
}

/// Takes the document off the restore fixed point the way a sketch workflow
/// does: a profile face extruded (the solid's bottom cap shares the
/// profile's edges), then the solid deleted. The shared edges retire with
/// it, and so do the standalone profile face's index entries, while its
/// loops and coedges stay live. A sheet body stands beside it.
pub(super) fn break_fixed_point(k: &mut BrepKernel) {
    let square = |x: f64| {
        vec![
            x,
            30.0,
            0.0,
            x + 4.0,
            30.0,
            0.0,
            x + 4.0,
            33.0,
            0.0,
            x,
            33.0,
            0.0,
        ]
    };
    let profile = k.make_polygon(square(30.0)).unwrap();
    let body = k.extrude_face(profile, 0.0, 0.0, 1.0, 2.0).unwrap();
    let sheet = k.make_polygon(square(40.0)).unwrap();
    k.make_sheet_body(vec![sheet]).unwrap();
    k.delete_solid(body).unwrap();
    assert!(!k.topo().is_restore_fixed_point());
}

/// The same short modeling history in each kernel.
fn shared_history(kernels: [&mut BrepKernel; 2]) -> u32 {
    let mut small = 0;
    for k in kernels {
        k.make_box_solid(10.0, 10.0, 10.0).unwrap();
        small = k.make_box_solid(4.0, 4.0, 4.0).unwrap();
        k.transform_solid_binding(small, SHIFT.to_vec()).unwrap();
    }
    small
}

const SINGULAR: [f64; 16] = [0.0; 16];

const SHIFT: [f64; 16] = [
    1.0, 0.0, 0.0, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

fn assert_same_topology(a: &BrepKernel, b: &BrepKernel, context: &str) {
    assert_eq!(
        format!("{:?}", a.topo()),
        format!("{:?}", b.topo()),
        "{context}"
    );
    assert_eq!(high_water(a.topo()), high_water(b.topo()), "{context}");
}

fn shares_checkpoint(k: &BrepKernel, saved: u32) -> bool {
    std::rc::Rc::ptr_eq(&k.topo, &k.checkpoints.get(saved as usize).unwrap().topo)
}

#[test]
fn restoring_the_still_shared_value_copies_nothing_and_matches_the_copying_restore() {
    let mut skipped = BrepKernel::new();
    let mut copied = BrepKernel::new();
    let small = shared_history([&mut skipped, &mut copied]);
    let saved = skipped.checkpoint().unwrap();
    assert_eq!(copied.checkpoint().unwrap(), saved);
    // Nothing mutated since the checkpoint: the restore copies nothing and
    // keeps sharing the checkpoint's value.
    let identity = skipped.topo().cache_identity();
    skipped.restore(saved).unwrap();
    assert!(shares_checkpoint(&skipped, saved));
    assert_eq!(skipped.topo().cache_identity(), identity);
    // The same restore through the copy the restore used to make.
    copied.topo_mut();
    assert!(!shares_checkpoint(&copied, saved));
    copied.restore(saved).unwrap();
    assert_same_topology(&skipped, &copied, "restore");
    // Later edits, refusals and restores behave identically.
    let refused = serde_json::json!([
        {"op":"transform","args":{"solid":small,"matrix":SHIFT}},
        {"op":"transform","args":{"solid":small,"matrix":SINGULAR}},
    ]);
    for round in 0..2 {
        for k in [&mut skipped, &mut copied] {
            let rows = batch(k, &refused);
            assert!(rows[0].get("error").is_none(), "{rows}");
            assert!(rows[1]["error"].is_string(), "{rows}");
        }
        assert_same_topology(&skipped, &copied, &format!("edit {round}"));
        skipped.restore(saved).unwrap();
        copied.restore(saved).unwrap();
        assert_same_topology(&skipped, &copied, &format!("restore {round}"));
    }
}

#[test]
fn restoring_a_still_shared_value_off_the_fixed_point_still_restores() {
    // The skip holds only on a restore fixed point: off it, the restore
    // rebuilds the pcurve index, so the still-shared value must be restored
    // (copied) like any other.
    let mut skipped = BrepKernel::new();
    let mut copied = BrepKernel::new();
    shared_history([&mut skipped, &mut copied]);
    for k in [&mut skipped, &mut copied] {
        break_fixed_point(k);
    }
    let saved = skipped.checkpoint().unwrap();
    assert_eq!(copied.checkpoint().unwrap(), saved);
    skipped.restore(saved).unwrap();
    assert!(!shares_checkpoint(&skipped, saved));
    assert!(skipped.topo().is_restore_fixed_point());
    let snapshot = std::rc::Rc::clone(&copied.checkpoints.get(saved as usize).unwrap().topo);
    copied.topo_mut().restore_preserving_handle_slots(&snapshot);
    assert_same_topology(&skipped, &copied, "off the fixed point");
}

fn batch(k: &mut BrepKernel, operations: &serde_json::Value) -> serde_json::Value {
    serde_json::from_str(&k.execute_batch(&operations.to_string())).unwrap()
}
