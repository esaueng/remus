//! Coordinated boolean batch commits and retained session-state tests.

#![allow(clippy::unwrap_used)]
use crate::kernel::RemusKernel;

#[test]
fn coordinated_boolean_items_commit_independently_and_keep_session_state() {
    let mut k = RemusKernel::new();
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

    use crate::kernel::RemusKernel;

    fn batch(
        kernel: &mut RemusKernel,
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

    fn cavity_fixture(kernel: &mut RemusKernel, tool_side: f64, second_piece: bool) -> (u32, u32) {
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
                let mut kernel = RemusKernel::new();
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
                let mut kernel = RemusKernel::new();
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

// ── Boundary rollback fault matrix (every migrated family) ─────────

/// Every live entity, its boundary authority, pcurves, attributes and the
/// journal. Slot and journal-id high-water marks only grow, so they are
/// compared separately ([`high_water`]); tombstone contents are
/// deliberately absent: no API can read them.
pub(super) fn live_state(t: &remus_topology::Topology) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    macro_rules! arena {
        ($method:ident) => {
            let arena = t.$method();
            let _ = writeln!(
                out,
                "{} {:?}",
                stringify!($method),
                arena.iter().collect::<Vec<_>>()
            );
        };
    }
    arena!(vertices);
    arena!(edges);
    arena!(wires);
    arena!(faces);
    arena!(shells);
    arena!(solids);
    arena!(compounds);
    arena!(compsolids);
    let _ = writeln!(
        out,
        "loops={} coedges={} uses={} ticks={}",
        t.num_loops(),
        t.num_coedges(),
        t.num_pcurves(),
        t.mutation_ticks()
    );
    for (face, _) in t.faces().iter() {
        let _ = writeln!(
            out,
            "{face:?} {:?} {:?}",
            t.attributes().face(face),
            t.pcurves_for_face(face)
        );
        for &loop_id in t.loops_of_face(face).unwrap_or_default() {
            let boundary = t.face_loop(loop_id);
            let _ = writeln!(out, "  {loop_id:?} {boundary:?}");
            if let Ok(boundary) = boundary {
                for &coedge in boundary.coedges() {
                    let _ = writeln!(out, "    {coedge:?} {:?}", t.coedge(coedge));
                }
            }
        }
    }
    for (edge, _) in t.edges().iter() {
        let _ = writeln!(out, "{edge:?} {:?}", t.coedges_of_edge(edge));
    }
    for (solid, _) in t.solids().iter() {
        let _ = writeln!(out, "{solid:?} {:?}", t.attributes().solid(solid));
    }
    let mut journal = t.journal().snapshot();
    journal.next_op = 0;
    journal.next_ordinal = 0;
    let _ = writeln!(out, "{journal:?}");
    out
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

/// Session state a topology rollback must never touch.
pub(super) fn session_state(k: &RemusKernel) -> String {
    format!(
        "{:?}{:?}{:?}{:?}{}",
        k.assemblies, k.sketches, k.gcs_sketches, k.checkpoints, k.poisoned
    )
}

/// Whether the next journaled operation would first record a barrier for
/// unjournaled mutations: a rollback must leave this exactly as it was.
pub(super) fn journal_gap(t: &remus_topology::Topology) -> bool {
    let mut probe = t.clone();
    let entries = probe.journal().len();
    let _ = probe.journal_begin("gap-probe");
    probe.journal().len() != entries
}

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

/// Every handle issued in `[from, to)` must stay stale.
pub(super) fn assert_stale(t: &remus_topology::Topology, from: [usize; 8], to: [usize; 8]) {
    for (arena, (&start, &end)) in from.iter().zip(&to).enumerate() {
        for index in start..end {
            let live = match arena {
                0 => t.vertex_id_from_index(index).is_some(),
                1 => t.edge_id_from_index(index).is_some(),
                2 => t.wire_id_from_index(index).is_some(),
                3 => t.face_id_from_index(index).is_some(),
                4 => t.shell_id_from_index(index).is_some(),
                5 => t.solid_id_from_index(index).is_some(),
                6 => t.compound_id_from_index(index).is_some(),
                _ => t.compsolid_id_from_index(index).is_some(),
            };
            assert!(!live, "arena {arena} slot {index} came back to life");
        }
    }
}

/// What a refusal must leave untouched.
#[derive(Debug, PartialEq, Eq)]
pub(super) struct Observed {
    live: String,
    session: String,
    gap: bool,
}

pub(super) fn observe(k: &RemusKernel) -> Observed {
    remus_topology::validation::validate_boundary_authority(k.topo()).unwrap();
    Observed {
        live: live_state(k.topo()),
        session: session_state(k),
        gap: journal_gap(k.topo()),
    }
}

/// Seeds assembly, legacy sketch and GCS sketch state so a rollback that
/// touched session state would show.
fn seed_session(k: &mut RemusKernel) {
    k.assembly_new("retained").unwrap();
    let sketch = k.sketch_new().unwrap();
    k.sketch_add_point(sketch, 1.0, 2.0, false).unwrap();
    k.gcs_new().unwrap();
}

fn batch(k: &mut RemusKernel, operations: &serde_json::Value) -> serde_json::Value {
    serde_json::from_str(&k.execute_batch(&operations.to_string())).unwrap()
}

/// A refusal through `refuse` must restore every observed field exactly,
/// leave every handle it allocated stale (also after later allocations and
/// a checkpoint restore), and let the next success commit normally.
/// `refuse` returns whether the call refused; `succeed` whether it committed.
fn assert_refusal_rolls_back(
    k: &mut RemusKernel,
    family: &str,
    refuse: &mut dyn FnMut(&mut RemusKernel) -> bool,
    succeed: &mut dyn FnMut(&mut RemusKernel) -> bool,
) {
    assert!(succeed(k), "{family}: first success");
    let checkpoint = k.checkpoint().unwrap();
    let mut failed = Vec::new();
    for round in 0..3 {
        let before = observe(k);
        let marks = slot_marks(k.topo());
        assert!(refuse(k), "{family}: refusal {round} must fail");
        assert_eq!(observe(k), before, "{family}: refusal {round}");
        failed.push((marks, slot_marks(k.topo())));
        assert!(succeed(k), "{family}: success after refusal {round}");
        k.make_box_solid(1.0, 1.0, 1.0).unwrap();
        for &(from, to) in &failed {
            assert_stale(k.topo(), from, to);
        }
    }
    k.restore(checkpoint).unwrap();
    k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    for &(from, to) in &failed {
        assert_stale(k.topo(), from, to);
    }
}

/// Handles of the shared fixture: two boxes, a drilled plate and its bore.
pub(super) struct Fixture {
    pub(super) block: u32,
    pub(super) small: u32,
    plate: u32,
    bore: u32,
    edge: u32,
    face: u32,
}

fn handle(index: usize) -> u32 {
    u32::try_from(index).unwrap()
}

pub(super) fn fixture(k: &mut RemusKernel) -> Fixture {
    use remus_operations::{boolean, primitives, transform};
    seed_session(k);
    let block = k.make_box_solid(10.0, 10.0, 10.0).unwrap();
    let small = k.make_box_solid(4.0, 4.0, 4.0).unwrap();
    let mut source = remus_topology::Topology::new();
    let blank = primitives::make_box(&mut source, 60.0, 40.0, 10.0).unwrap();
    let tool = primitives::make_cylinder(&mut source, 4.0, 30.0).unwrap();
    transform::transform_solid(
        &mut source,
        tool,
        &remus_math::mat::Mat4::translation(10.0, 10.0, -10.0),
    )
    .unwrap();
    let drilled = boolean::boolean(&mut source, boolean::BooleanOp::Cut, blank, tool).unwrap();
    let bytes = remus_io::arena_io::serialize_solids(&source, &[drilled]).unwrap();
    let plate = k.deserialize_solids(&bytes).unwrap()[0];
    let plate_id = k.resolve_solid(plate).unwrap();
    let bore = remus_topology::explorer::solid_faces(k.topo(), plate_id)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(
                k.topo().face(f).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .unwrap();
    let block_id = k.resolve_solid(block).unwrap();
    let edge = remus_topology::explorer::solid_edges(k.topo(), block_id).unwrap()[0];
    let face = remus_topology::explorer::solid_faces(k.topo(), block_id).unwrap()[0];
    Fixture {
        block,
        small,
        plate,
        bore: handle(bore.index()),
        edge: handle(edge.index()),
        face: handle(face.index()),
    }
}

const SINGULAR: [f64; 16] = [0.0; 16];

const SHIFT: [f64; 16] = [
    1.0, 0.0, 0.0, 0.25, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// `(family, committing op, refused op)` for the non-boolean mutating batch
/// arm, the boolean arm, and the typed modifier twins (which refuse inside
/// an `ok` envelope).
pub(super) fn batch_cases(
    f: &Fixture,
) -> Vec<(&'static str, serde_json::Value, serde_json::Value)> {
    use serde_json::json;
    let (block, small, plate, bore, edge, face) =
        (f.block, f.small, f.plate, f.bore, f.edge, f.face);
    vec![
        (
            "transform",
            json!({"op":"transform","args":{"solid":small,"matrix":SHIFT}}),
            json!({"op":"transform","args":{"solid":small,"matrix":SINGULAR}}),
        ),
        (
            "fillet",
            json!({"op":"fillet","args":{"solid":block,"edges":[edge],"radius":1.0}}),
            json!({"op":"fillet","args":{"solid":block,"edges":[edge],"radius":50.0}}),
        ),
        (
            "chamfer",
            json!({"op":"chamfer","args":{"solid":block,"edges":[edge],"distance":1.0}}),
            json!({"op":"chamfer","args":{"solid":block,"edges":[edge],"distance":50.0}}),
        ),
        (
            "shell",
            json!({"op":"shell","args":{"solid":block,"faces":[face],"thickness":1.0}}),
            json!({"op":"shell","args":{"solid":block,"faces":[face],"thickness":20.0}}),
        ),
        (
            "moveFaces bore",
            json!({"op":"moveFaces","args":{"solid":plate,"faces":[bore],"distance":0.5}}),
            json!({"op":"moveFaces","args":{"solid":plate,"faces":[bore],"distance":-7.0}}),
        ),
        (
            "moveFaces planar",
            json!({"op":"moveFaces","args":{"solid":block,"faces":[face],"distance":1.0}}),
            json!({"op":"moveFaces","args":{"solid":block,"faces":[face],"distance":-20.0}}),
        ),
        (
            "pushPullFace",
            json!({"op":"pushPullFace","args":{"solid":block,"face":face,"distance":2.0}}),
            json!({"op":"pushPullFace","args":{"solid":block,"face":face,"distance":-20.0}}),
        ),
        (
            "resizeCylindricalFace",
            json!({"op":"resizeCylindricalFace","args":{"solid":plate,"face":bore,"radius":3.0}}),
            json!({"op":"resizeCylindricalFace","args":{"solid":plate,"face":bore,"radius":30.0}}),
        ),
        (
            "offsetSolid",
            json!({"op":"offsetSolid","args":{"solid":block,"distance":1.0}}),
            json!({"op":"offsetSolid","args":{"solid":block,"distance":-20.0}}),
        ),
        (
            "copySolid",
            json!({"op":"copySolid","args":{"solid":block}}),
            json!({"op":"copySolid","args":{"solid":99_999}}),
        ),
        (
            "fuse",
            json!({"op":"fuse","args":{"solidA":block,"solidB":small}}),
            json!({"op":"cut","args":{"solidA":block,"solidB":block}}),
        ),
        (
            "filletDetailed",
            json!({"op":"filletDetailed","args":{"solid":block,"edges":[edge],"radius":1.0}}),
            json!({"op":"filletDetailed","args":{"solid":block,"edges":[edge],"radius":50.0}}),
        ),
        (
            "chamferDetailed",
            json!({"op":"chamferDetailed","args":{"solid":block,"edges":[edge],"distance":1.0}}),
            json!({"op":"chamferDetailed","args":{"solid":block,"edges":[edge],"distance":50.0}}),
        ),
        (
            "shellDetailed",
            json!({"op":"shellDetailed","args":{"solid":block,"faces":[face],"thickness":1.0}}),
            json!({"op":"shellDetailed","args":{"solid":block,"faces":[face],"thickness":20.0}}),
        ),
        (
            "offsetDetailed",
            json!({"op":"offsetDetailed","args":{"solid":block,"distance":1.0}}),
            json!({"op":"offsetDetailed","args":{"solid":block,"distance":-20.0}}),
        ),
    ]
}

/// Whether a batch row committed (typed twins report refusals in `ok`).
pub(super) fn committed(row: &serde_json::Value) -> bool {
    row.get("error").is_none() && row["ok"]["status"] != "error"
}

#[test]
fn batch_refusals_roll_back_exactly_and_items_commit_independently() {
    for v2 in [false, true] {
        let mut k = RemusKernel::new();
        let f = fixture(&mut k);
        for (family, ok, bad) in batch_cases(&f) {
            let run = |k: &mut RemusKernel, op: &serde_json::Value| {
                let ops = serde_json::Value::Array(vec![op.clone()]);
                let response = if v2 {
                    k.execute_batch_v2(&ops.to_string())
                } else {
                    k.execute_batch(&ops.to_string())
                };
                let rows: serde_json::Value = serde_json::from_str(&response).unwrap();
                committed(&rows[0])
            };
            assert_refusal_rolls_back(&mut k, family, &mut |k| !run(k, &bad), &mut |k| run(k, &ok));
            // One batch: the refused middle item rolls back alone.
            let before = slot_marks(k.topo());
            let rows = batch(&mut k, &serde_json::json!([ok, bad, ok]));
            assert!(
                committed(&rows[0]) && committed(&rows[2]),
                "{family}: {rows}"
            );
            assert!(!committed(&rows[1]), "{family}: {rows}");
            assert!(slot_marks(k.topo()) >= before);
        }
    }
}

#[test]
fn direct_topology_transaction_refusals_roll_back_exactly() {
    let mut k = RemusKernel::new();
    let f = fixture(&mut k);
    assert_refusal_rolls_back(
        &mut k,
        "offsetSolid direct",
        &mut |k| k.offset_solid_impl(f.block, -20.0).is_err(),
        &mut |k| k.transform_solid_binding(f.small, SHIFT.to_vec()).is_ok(),
    );
    // An injected fault after allocations, in-place edits, a retirement and
    // a journal record: the failed call's retirement is revived.
    let block = k.resolve_solid(f.block).unwrap();
    let small = k.resolve_solid(f.small).unwrap();
    let input = k.serialize_solids(&[f.block, f.small]).unwrap();
    let mut leaked = Vec::new();
    assert_refusal_rolls_back(
        &mut k,
        "injected",
        &mut |k| {
            k.with_topology_transaction(|topo| {
                leaked.push(remus_operations::primitives::make_box(topo, 1.0, 2.0, 3.0)?);
                remus_operations::transform::transform_solid(
                    topo,
                    small,
                    &remus_math::mat::Mat4::translation(1.0, 0.0, 0.0),
                )?;
                topo.set_solid_attributes(
                    block,
                    remus_topology::attributes::EntityAttributes {
                        name: Some("staged".into()),
                        ..Default::default()
                    },
                )?;
                topo.delete_solid(block).unwrap();
                let pending = topo.journal_begin("injected");
                let _ = topo.journal_record_barrier(pending, Vec::new());
                Err::<(), remus_operations::OperationsError>(
                    remus_operations::OperationsError::InvalidInput {
                        reason: "injected".into(),
                    },
                )
            })
            .is_err()
        },
        &mut |k| k.make_box_solid(1.0, 1.0, 1.0).is_ok(),
    );
    assert_eq!(k.serialize_solids(&[f.block, f.small]).unwrap(), input);
    for id in leaked {
        assert!(k.topo().solid(id).is_err());
    }
}

#[test]
fn failed_operation_retirements_are_revived_not_sticky() {
    // The documented boundary semantic change: the full-copy boundary
    // restored with checkpoint-barrier semantics, so a failed operation's
    // retirement stayed sticky. The rollback scope undoes it exactly.
    let mut k = RemusKernel::new();
    let solid = k.make_box_solid(1.0, 1.0, 1.0).unwrap();
    let id = k.resolve_solid(solid).unwrap();
    let mut legacy = k.topo().clone();
    let snapshot = legacy.clone();
    legacy.delete_solid(id).unwrap();
    legacy.restore_preserving_handle_slots(&snapshot);
    assert!(
        legacy.solid(id).is_err(),
        "the legacy boundary kept it retired"
    );

    let before = observe(&k);
    let result = k.with_topology_transaction(|topo| {
        topo.delete_solid(id).unwrap();
        Err::<(), _>(remus_topology::TopologyError::WireNotClosed)
    });
    assert!(result.is_err());
    assert!(k.topo().solid(id).is_ok());
    assert_eq!(observe(&k), before);
}

/// Takes the document off the restore fixed point the way a sketch workflow
/// does: a profile face extruded (the solid's bottom cap shares the
/// profile's edges), then the solid deleted. The shared edges retire with
/// it, and so do the standalone profile face's index entries, while its
/// loops and coedges stay live. A sheet body stands beside it.
pub(super) fn break_fixed_point(k: &mut RemusKernel) {
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

#[test]
fn successful_items_leave_no_undo_records_for_a_checkpoint_to_keep() {
    // A committed scope releases its records at once: a checkpoint taken
    // right after shares the topology value and would otherwise keep one
    // item's records for its whole life.
    for v2 in [false, true] {
        let mut k = RemusKernel::new();
        let f = fixture(&mut k);
        for (family, ok, _) in batch_cases(&f) {
            let ops = serde_json::Value::Array(vec![ok]);
            let response = if v2 {
                k.execute_batch_v2(&ops.to_string())
            } else {
                k.execute_batch(&ops.to_string())
            };
            let rows: serde_json::Value = serde_json::from_str(&response).unwrap();
            assert!(committed(&rows[0]), "{family}: {rows}");
            let saved = k.checkpoint().unwrap();
            let stored = &k.checkpoints.get(saved as usize).unwrap().topo;
            assert_eq!(stored.undo_record_count(), 0, "{family}");
        }
    }
    let mut k = RemusKernel::new();
    let f = fixture(&mut k);
    k.transform_solid_binding(f.small, SHIFT.to_vec()).unwrap();
    assert_eq!(k.topo().undo_record_count(), 0, "direct transaction");
    k.from_brep_impl(&brep_json(false)).unwrap();
    assert_eq!(k.topo().undo_record_count(), 0, "fromBREP");
}

#[cfg(debug_assertions)]
#[test]
#[should_panic(expected = "shared inside a boundary rollback scope")]
fn a_boundary_scope_refuses_to_rewind_a_shared_topology() {
    let mut k = RemusKernel::new();
    let _ = k.with_kernel_transaction(crate::kernel::Refusal::Exact, |k| {
        k.checkpoint().unwrap();
        Err::<(), _>(())
    });
}

#[cfg(debug_assertions)]
#[test]
#[should_panic(expected = "replaced inside a boundary rollback scope")]
fn a_boundary_scope_refuses_to_rewind_a_replaced_topology() {
    let mut k = RemusKernel::new();
    let _ = k.with_kernel_transaction(crate::kernel::Refusal::Exact, |k| {
        k.topo = std::rc::Rc::new(remus_topology::Topology::new());
        Err::<(), _>(())
    });
}

/// A one-triangle `fromBREP` document; `missing_vertex` makes it refuse.
pub(super) fn brep_json(missing_vertex: bool) -> String {
    serde_json::json!({
        "vertices": [
            {"id": 0, "position": [0.0, 0.0, 0.0]},
            {"id": 1, "position": [1.0, 0.0, 0.0]},
            {"id": 2, "position": [0.0, 1.0, 0.0]}
        ],
        "edges": [
            {"id": 0, "curveType": "line", "startVertex": 0, "endVertex": 1},
            {"id": 1, "curveType": "line", "startVertex": 1, "endVertex": 2},
            {"id": 2, "curveType": "line", "startVertex": 2,
             "endVertex": if missing_vertex { 7 } else { 0 }}
        ],
        "faces": [{
            "outerWireEdges": [0, 1, 2],
            "outerWireOrientations": [true, true, true],
            "surfaceType": "plane",
            "surfaceParams": {"normal": [0.0, 0.0, 1.0], "d": 0.0},
            "reversed": false,
            "innerWires": []
        }]
    })
    .to_string()
}

#[test]
fn from_brep_refusals_roll_back_exactly() {
    let mut k = RemusKernel::new();
    fixture(&mut k);
    let (good, bad) = (brep_json(false), brep_json(true));
    assert_refusal_rolls_back(
        &mut k,
        "fromBREP",
        &mut |k| k.from_brep_impl(&bad).is_err(),
        &mut |k| k.from_brep_impl(&good).is_ok(),
    );
}

// ── Checkpoint restore of the value the kernel still holds ─────────

/// The same short modeling history in each kernel.
fn shared_history(kernels: [&mut RemusKernel; 2]) -> u32 {
    let mut small = 0;
    for k in kernels {
        k.make_box_solid(10.0, 10.0, 10.0).unwrap();
        small = k.make_box_solid(4.0, 4.0, 4.0).unwrap();
        k.transform_solid_binding(small, SHIFT.to_vec()).unwrap();
    }
    small
}

fn assert_same_topology(a: &RemusKernel, b: &RemusKernel, context: &str) {
    assert_eq!(
        format!("{:?}", a.topo()),
        format!("{:?}", b.topo()),
        "{context}"
    );
    assert_eq!(high_water(a.topo()), high_water(b.topo()), "{context}");
}

fn shares_checkpoint(k: &RemusKernel, saved: u32) -> bool {
    std::rc::Rc::ptr_eq(&k.topo, &k.checkpoints.get(saved as usize).unwrap().topo)
}

#[test]
fn restoring_the_still_shared_value_copies_nothing_and_matches_the_copying_restore() {
    let mut skipped = RemusKernel::new();
    let mut copied = RemusKernel::new();
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
    let mut skipped = RemusKernel::new();
    let mut copied = RemusKernel::new();
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
