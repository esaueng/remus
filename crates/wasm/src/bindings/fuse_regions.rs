//! N-ary General Fuse with regions, lineage and operand dispositions
//! (P-Class 4.8, core tier: it returns live-session handles and evolution).

use remus_operations::boolean::{NaryFuseResult, OperandDisposition, fuse_body_regions};
use remus_topology::BodyId;
use wasm_bindgen::prelude::*;

use crate::error::StructuredWasmError;
use crate::handles::{compound_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;

fn index_u32(index: usize) -> u32 {
    u32::try_from(index).unwrap_or(u32::MAX)
}

fn indices(values: &[usize]) -> Vec<u32> {
    values.iter().copied().map(index_u32).collect()
}

/// The stable JSON encoding of an N-ary fuse result. `solids` are the
/// caller's operand handles, echoed beside each disposition.
fn result_json(result: &NaryFuseResult, solids: &[u32]) -> serde_json::Value {
    let regions: Vec<serde_json::Value> = result
        .regions
        .iter()
        .map(|region| {
            serde_json::json!({
                "solid": solid_id_to_u32(region.solid),
                "operands": indices(&region.operands),
                "faceOperands": indices(&region.face_operands),
                "evolution": super::evolution::entity_evolution_json(&region.evolution),
            })
        })
        .collect();
    let operands: Vec<serde_json::Value> = result
        .operands
        .iter()
        .zip(solids)
        .map(|(disposition, &solid)| match disposition {
            OperandDisposition::Preserved { region } => serde_json::json!({
                "solid": solid,
                "disposition": "preserved",
                "regions": [index_u32(*region)],
            }),
            OperandDisposition::Contributes { regions } => serde_json::json!({
                "solid": solid,
                "disposition": "contributes",
                "regions": indices(regions),
            }),
            OperandDisposition::Absorbed { regions } => serde_json::json!({
                "solid": solid,
                "disposition": "absorbed",
                "regions": indices(regions),
            }),
        })
        .collect();
    serde_json::json!({
        "compound": compound_id_to_u32(result.compound),
        "regions": regions,
        "operands": operands,
    })
}

impl RemusKernel {
    /// Shared body of the direct binding and the batch arm, so the two
    /// cannot drift apart.
    pub(crate) fn fuse_regions_json(
        &mut self,
        solids: &[u32],
        sheets: &[u32],
        wires: &[u32],
    ) -> Result<serde_json::Value, StructuredWasmError> {
        let mut bodies = Vec::with_capacity(solids.len() + sheets.len() + wires.len());
        for &handle in solids {
            bodies.push(BodyId::Solid(
                self.resolve_solid(handle)
                    .map_err(StructuredWasmError::from)?,
            ));
        }
        for &handle in sheets {
            bodies.push(BodyId::Shell(
                self.resolve_shell(handle)
                    .map_err(StructuredWasmError::from)?,
            ));
        }
        for &handle in wires {
            bodies.push(BodyId::Wire(
                self.resolve_wire(handle)
                    .map_err(StructuredWasmError::from)?,
            ));
        }
        let result =
            fuse_body_regions(self.topo_mut(), &bodies).map_err(StructuredWasmError::from)?;
        Ok(result_json(&result, solids))
    }
}

fn structured_to_js(error: StructuredWasmError) -> JsError {
    JsError::new(error.message())
}

#[wasm_bindgen]
impl RemusKernel {
    /// Fuse a list of solids into the regions of their union, with total
    /// face lineage and a disposition for every operand.
    ///
    /// Returns JSON:
    ///
    /// - `compound`: a Compound of the regions, expandable with
    ///   `getCompoundSolids`;
    /// - `regions`: one entry per region, ordered by the lowest operand
    ///   index contributing a face — `solid`, `operands` (operand indices
    ///   with a face in it), `evolution` (`faces`/`edges`/`vertices` as in
    ///   `fuseWithEntityEvolution`; no face is unsourced and no edge
    ///   unresolved) and `faceOperands`, the operand index of each
    ///   `evolution.faces` entry in the same order;
    /// - `operands`: one entry per input solid, in input order — `solid`,
    ///   `regions` and `disposition`: `preserved` (it can touch no other
    ///   operand; its region is the same handle), `contributes` (a result
    ///   face descends from it) or `absorbed` (none does, because the others
    ///   cover every one of its faces).
    ///
    /// Exact-only and additive: `fuseAll` is unchanged. Operands that may
    /// touch must be all-planar with straight edges. `sheets` and `wires`
    /// exist so a mixed-dimensional list has a typed answer; any entry in
    /// either refuses with `body_class_operand_unsupported`.
    ///
    /// # Errors
    ///
    /// Invalid handles; `body_class_operand_unsupported` for a sheet or
    /// wire; and `unsupported_nary_fuse` with a `reason` detail of
    /// `too_few_operands`, `duplicate_operand`, `curved_cluster`,
    /// `pair_budget_exceeded`, `classification_unresolved`,
    /// `lineage_unresolved`, `non_manifold_contact` or `result_unqualified`.
    /// Every refusal leaves the model unchanged.
    #[wasm_bindgen(js_name = "fuseRegions")]
    pub fn fuse_regions(
        &mut self,
        solids: Vec<u32>,
        sheets: Option<Vec<u32>>,
        wires: Option<Vec<u32>>,
    ) -> Result<String, JsError> {
        self.fuse_regions_json(
            &solids,
            sheets.as_deref().unwrap_or_default(),
            wires.as_deref().unwrap_or_default(),
        )
        .map(|value| value.to_string())
        .map_err(structured_to_js)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use remus_math::mat::Mat4;
    use serde_json::{Value, json};

    use crate::kernel::RemusKernel;

    /// The three-box witness (`A`, `A + (1, 0.5, 0.25)`, `A + (0.5, 1, 0.75)`
    /// with `A = [0, 2]³`), a far box and a small box inside `A`.
    const SHIFTS: [[f64; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.5, 0.25], [0.5, 1.0, 0.75]];

    fn direct_box(kernel: &mut RemusKernel, size: [f64; 3], at: [f64; 3]) -> u32 {
        let handle = kernel.make_box_solid(size[0], size[1], size[2]).unwrap();
        let solid = kernel.resolve_solid(handle).unwrap();
        remus_operations::transform::transform_solid(
            kernel.topo_mut(),
            solid,
            &Mat4::translation(at[0], at[1], at[2]),
        )
        .unwrap();
        handle
    }

    fn batch_box(size: [f64; 3], at: [f64; 3], handle: usize) -> [Value; 2] {
        [
            json!({"op": "makeBox", "args": {"width": size[0], "height": size[1], "depth": size[2]}}),
            json!({"op": "transform", "args": {"solid": handle, "matrix": [
                1.0, 0.0, 0.0, at[0],
                0.0, 1.0, 0.0, at[1],
                0.0, 0.0, 1.0, at[2],
                0.0, 0.0, 0.0, 1.0,
            ]}}),
        ]
    }

    /// The five witness boxes as `(size, position)`.
    fn witness() -> Vec<([f64; 3], [f64; 3])> {
        let mut boxes: Vec<_> = SHIFTS.iter().map(|&at| ([2.0; 3], at)).collect();
        boxes.push(([1.0, 2.0, 3.0], [40.0, 30.0, 20.0]));
        boxes.push(([0.5; 3], [0.25; 3]));
        boxes
    }

    fn run_batch(kernel: &mut RemusKernel, ops: &[Value]) -> Vec<Value> {
        let response = kernel.execute_batch_v2(&Value::Array(ops.to_vec()).to_string());
        serde_json::from_str::<Value>(&response)
            .unwrap()
            .as_array()
            .unwrap()
            .clone()
    }

    fn region_volumes(kernel: &RemusKernel, result: &Value) -> Vec<f64> {
        result["regions"]
            .as_array()
            .unwrap()
            .iter()
            .map(|region| {
                let solid = u32::try_from(region["solid"].as_u64().unwrap()).unwrap();
                kernel.volume(solid, 0.01).unwrap()
            })
            .collect()
    }

    /// Everything but the handles, which differ between two kernels only
    /// when their histories differ: here they do not, so the payloads are
    /// compared whole.
    #[test]
    fn direct_and_batch_agree_on_regions_lineage_and_dispositions() {
        let boxes = witness();

        let mut direct = RemusKernel::new();
        let handles: Vec<u32> = boxes
            .iter()
            .map(|&(size, at)| direct_box(&mut direct, size, at))
            .collect();
        let direct_result = direct.fuse_regions_json(&handles, &[], &[]).unwrap();

        let mut batch = RemusKernel::new();
        let mut ops: Vec<Value> = boxes
            .iter()
            .enumerate()
            .flat_map(|(handle, &(size, at))| batch_box(size, at, handle))
            .collect();
        ops.push(json!({"op": "fuseRegions", "args": {"solids": handles}}));
        let rows = run_batch(&mut batch, &ops);
        let batch_result = rows.last().unwrap()["ok"].clone();
        assert!(
            batch_result.is_object(),
            "batch fuseRegions failed: {rows:?}"
        );

        assert_eq!(batch_result, direct_result, "direct and batch payloads");

        // Two regions in input order: the fused three boxes (holding the
        // small one), then the far box, which is its own operand handle.
        let regions = direct_result["regions"].as_array().unwrap();
        assert_eq!(regions.len(), 2);
        assert_eq!(regions[0]["operands"], json!([0, 1, 2]));
        assert_eq!(regions[1]["operands"], json!([3]));
        assert_eq!(regions[1]["solid"], json!(handles[3]));
        let volumes = region_volumes(&direct, &direct_result);
        assert!((volumes[0] - 17.375).abs() <= 1e-9, "{volumes:?}");
        assert!((volumes[1] - 6.0).abs() <= 1e-9, "{volumes:?}");
        assert_eq!(region_volumes(&batch, &batch_result), volumes);

        let compound = u32::try_from(direct_result["compound"].as_u64().unwrap()).unwrap();
        let members = direct.get_compound_solids(compound).unwrap();
        assert_eq!(
            members,
            regions
                .iter()
                .map(|region| u32::try_from(region["solid"].as_u64().unwrap()).unwrap())
                .collect::<Vec<_>>()
        );

        assert_eq!(
            direct_result["operands"],
            json!([
                {"solid": handles[0], "disposition": "contributes", "regions": [0]},
                {"solid": handles[1], "disposition": "contributes", "regions": [0]},
                {"solid": handles[2], "disposition": "contributes", "regions": [0]},
                {"solid": handles[3], "disposition": "preserved", "regions": [1]},
                {"solid": handles[4], "disposition": "absorbed", "regions": [0]},
            ])
        );

        // Lineage is total: one operand per face claim, every face sourced,
        // no edge unresolved.
        for region in regions {
            let faces = region["evolution"]["faces"].as_array().unwrap();
            assert_eq!(
                faces.len(),
                region["faceOperands"].as_array().unwrap().len()
            );
            assert!(faces.iter().all(|face| face["source"].is_u64()));
            assert!(
                region["evolution"]["edges"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|edge| edge["event"] != "unresolved")
            );
        }
    }

    /// One refusal through both routes: the same code and reason, and no
    /// slot allocated by either.
    fn assert_refused_both_ways(
        setup: &[([f64; 3], [f64; 3])],
        extra: &[Value],
        args: &Value,
        kernel_code: &str,
        reason: Option<&str>,
    ) {
        let list = |key: &str| -> Vec<u32> {
            args.get(key)
                .and_then(Value::as_array)
                .map(|values| {
                    values
                        .iter()
                        .map(|value| u32::try_from(value.as_u64().unwrap()).unwrap())
                        .collect()
                })
                .unwrap_or_default()
        };
        let mut ops: Vec<Value> = setup
            .iter()
            .enumerate()
            .flat_map(|(handle, &(size, at))| batch_box(size, at, handle))
            .collect();
        ops.extend_from_slice(extra);

        // Direct, on a kernel prepared by the same ops.
        let mut direct = RemusKernel::new();
        let prepared = run_batch(&mut direct, &ops);
        assert!(
            prepared.iter().all(|row| row.get("ok").is_some()),
            "{prepared:?}"
        );
        let before = direct.topo().allocated_slot_count();
        let error = direct
            .fuse_regions_json(&list("solids"), &list("sheets"), &list("wires"))
            .unwrap_err();
        assert_eq!(
            direct.topo().allocated_slot_count(),
            before,
            "direct rollback"
        );
        let (_, category, details) = error.into_direct_parts();
        assert_eq!(category, "unsupported");
        assert_eq!(details["kernelCode"], kernel_code);
        if let Some(reason) = reason {
            assert_eq!(details["reason"], reason);
        }

        // Batch.
        let mut batch = RemusKernel::new();
        run_batch(&mut batch, &ops);
        let before = batch.topo().allocated_slot_count();
        let rows = run_batch(&mut batch, &[json!({"op": "fuseRegions", "args": args})]);
        assert_eq!(
            batch.topo().allocated_slot_count(),
            before,
            "batch rollback"
        );
        let error = &rows[0]["error"];
        assert_eq!(error["category"], "unsupported", "{rows:?}");
        assert_eq!(error["details"]["kernelCode"], kernel_code);
        if let Some(reason) = reason {
            assert_eq!(error["details"]["reason"], reason);
        }
    }

    #[test]
    fn refusals_are_typed_and_transactional_through_both_routes() {
        let pair = [([2.0; 3], SHIFTS[0]), ([2.0; 3], SHIFTS[1])];

        // An overlapping cylinder (handle 2, at the origin).
        assert_refused_both_ways(
            &pair,
            &[json!({"op": "makeCylinder", "args": {"radius": 1.0, "height": 3.0}})],
            &json!({"solids": [0, 2]}),
            "unsupported_nary_fuse",
            Some("curved_cluster"),
        );
        // The same handle twice.
        assert_refused_both_ways(
            &pair,
            &[],
            &json!({"solids": [0, 1, 0]}),
            "unsupported_nary_fuse",
            Some("duplicate_operand"),
        );
        // One operand.
        assert_refused_both_ways(
            &pair,
            &[],
            &json!({"solids": [1]}),
            "unsupported_nary_fuse",
            Some("too_few_operands"),
        );
        // A box and a sheet: the sheet is the shell `makeSheetBody` returns.
        let mut probe = RemusKernel::new();
        let mut ops: Vec<Value> = pair
            .iter()
            .enumerate()
            .flat_map(|(handle, &(size, at))| batch_box(size, at, handle))
            .collect();
        ops.push(json!({"op": "makeSheetBody", "args": {"faces": [0]}}));
        let sheet = run_batch(&mut probe, &ops).last().unwrap()["ok"]
            .as_u64()
            .unwrap();
        assert_refused_both_ways(
            &pair,
            &[json!({"op": "makeSheetBody", "args": {"faces": [0]}})],
            &json!({"solids": [0, 1], "sheets": [sheet]}),
            "body_class_operand_unsupported",
            None,
        );
        // A box and a wire: wire 0 is a boundary wire of the first box.
        assert_refused_both_ways(
            &pair,
            &[],
            &json!({"solids": [0, 1], "wires": [0]}),
            "body_class_operand_unsupported",
            None,
        );
    }

    #[test]
    fn a_class_refusal_names_the_class() {
        let mut kernel = RemusKernel::new();
        let ops: Vec<Value> = batch_box([2.0; 3], SHIFTS[0], 0)
            .into_iter()
            .chain([json!({"op": "makeSheetBody", "args": {"faces": [0]}})])
            .collect();
        let sheet = run_batch(&mut kernel, &ops).last().unwrap()["ok"]
            .as_u64()
            .unwrap();
        let rows = run_batch(
            &mut kernel,
            &[
                json!({"op": "fuseRegions", "args": {"solids": [0], "sheets": [sheet]}}),
                json!({"op": "fuseRegions", "args": {"solids": [0], "wires": [0]}}),
                json!({"op": "fuseRegions", "args": {"solids": [0, 99]}}),
                json!({"op": "fuseRegions", "args": {"sheets": [sheet]}}),
            ],
        );
        assert_eq!(rows[0]["error"]["details"]["actual"], "sheet", "{rows:?}");
        assert_eq!(rows[1]["error"]["details"]["actual"], "wire", "{rows:?}");
        for row in &rows[..2] {
            assert_eq!(
                row["error"]["details"]["kernelCode"],
                "body_class_operand_unsupported"
            );
        }
        // A dead handle and a missing `solids` array are argument errors,
        // not refusals.
        for row in &rows[2..] {
            assert_ne!(row["error"]["category"], "unsupported", "{rows:?}");
            assert!(
                row["error"]["details"].get("kernelCode").is_none(),
                "{rows:?}"
            );
        }
        assert_eq!(rows[3]["error"]["code"], "invalid_argument", "{rows:?}");
    }

    /// `fuseAll` on the same operands still returns one solid handle.
    #[test]
    fn fuse_all_is_unchanged() {
        let mut kernel = RemusKernel::new();
        let mut ops: Vec<Value> = SHIFTS
            .iter()
            .enumerate()
            .flat_map(|(handle, &at)| batch_box([2.0; 3], at, handle))
            .collect();
        ops.push(json!({"op": "fuseAll", "args": {"solids": [0, 1, 2]}}));
        let rows = run_batch(&mut kernel, &ops);
        let fused = u32::try_from(rows.last().unwrap()["ok"].as_u64().unwrap()).unwrap();
        assert!((kernel.volume(fused, 0.01).unwrap() - 17.375).abs() <= 1e-9);
    }
}
