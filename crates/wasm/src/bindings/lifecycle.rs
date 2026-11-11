//! Topological entity lifecycle bindings.

use wasm_bindgen::prelude::*;

use crate::error::WasmError;
use crate::kernel::RemusKernel;

#[derive(Debug, thiserror::Error)]
enum LifecycleError {
    #[error(transparent)]
    Wasm(#[from] WasmError),
    #[error(transparent)]
    DeleteSolid(#[from] remus_topology::DeleteSolidError),
}

impl RemusKernel {
    fn delete_solid_impl(&mut self, solid: u32) -> Result<(), LifecycleError> {
        let solid_id = self.resolve_solid(solid)?;
        if let Some((assembly_index, _)) = self.assemblies.iter().find(|(_, assembly)| {
            assembly
                .bill_of_materials()
                .iter()
                .any(|entry| entry.solid_index == solid_id.index())
        }) {
            return Err(remus_topology::DeleteSolidError::Referenced {
                solid: solid_id,
                dependent: "assembly",
                dependent_index: assembly_index,
            }
            .into());
        }
        self.topo_mut().delete_solid(solid_id)?;
        Ok(())
    }
}

#[wasm_bindgen]
impl RemusKernel {
    /// Retire a solid handle and its unshared topology subtree.
    ///
    /// The handle becomes permanently invalid. This does not compact the
    /// kernel or reclaim arena memory; future entities receive new handles so
    /// a stale handle can never alias a different solid.
    ///
    /// # Errors
    ///
    /// Returns an error if `solid` is not a live solid handle, if a live
    /// compound, comp-solid, or assembly still references it, or if its
    /// topology tree contains an invalid reference.
    #[wasm_bindgen(js_name = "deleteSolid")]
    pub fn delete_solid(&mut self, solid: u32) -> Result<(), JsError> {
        self.delete_solid_impl(solid)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use crate::error::WasmError;
    use crate::kernel::RemusKernel;

    fn identity_matrix() -> Vec<f64> {
        vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]
    }

    #[test]
    fn delete_solid_invalidates_handle_without_reusing_its_slot() {
        let mut kernel = RemusKernel::new();
        let stale = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();

        kernel.delete_solid(stale).unwrap();
        assert!(matches!(
            kernel.resolve_solid(stale),
            Err(WasmError::InvalidHandle {
                entity: "solid",
                ..
            })
        ));

        let fresh = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        assert!(fresh > stale);
        assert!(matches!(
            kernel.resolve_solid(stale),
            Err(WasmError::InvalidHandle {
                entity: "solid",
                ..
            })
        ));
        assert!((kernel.volume(fresh, 0.01).unwrap() - 8.0).abs() < 0.05);
    }

    #[test]
    fn delete_solid_rejects_live_compound_reference_atomically() {
        let mut kernel = RemusKernel::new();
        let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let compound = kernel.make_compound(vec![solid]).unwrap();

        assert!(kernel.delete_solid_impl(solid).is_err());
        assert_eq!(kernel.get_compound_solids(compound).unwrap(), vec![solid]);
        assert!(kernel.resolve_solid(solid).is_ok());
    }

    #[test]
    fn delete_solid_rejects_live_assembly_reference_atomically() {
        let mut kernel = RemusKernel::new();
        let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let assembly = kernel.assembly_new("assembly").unwrap();
        kernel
            .assembly_add_root(assembly, "box", solid, identity_matrix())
            .unwrap();

        assert!(kernel.delete_solid_impl(solid).is_err());
        assert!(kernel.resolve_solid(solid).is_ok());
        let flattened: serde_json::Value =
            serde_json::from_str(&kernel.assembly_flatten(assembly).unwrap()).unwrap();
        assert_eq!(flattened[0]["solid"].as_u64(), Some(u64::from(solid)));
    }

    #[test]
    fn restored_assembly_tombstones_do_not_pin_retained_solids() {
        let mut kernel = RemusKernel::new();
        let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let checkpoint = kernel.checkpoint().unwrap();
        let retired = kernel.assembly_new("retired").unwrap();
        kernel
            .assembly_add_root(retired, "box", solid, identity_matrix())
            .unwrap();
        assert!(kernel.delete_solid_impl(solid).is_err());
        kernel.restore(checkpoint).unwrap();
        assert!(kernel.assemblies.get(retired as usize).is_none());
        kernel.delete_solid_impl(solid).unwrap();
        assert!(kernel.resolve_solid(solid).is_err());
    }

    #[test]
    fn restore_does_not_revive_retired_solid() {
        let mut kernel = RemusKernel::new();
        let retired = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let checkpoint = kernel.checkpoint().unwrap();

        kernel.delete_solid(retired).unwrap();
        kernel.restore(checkpoint).unwrap();

        assert!(matches!(
            kernel.resolve_solid(retired),
            Err(WasmError::InvalidHandle {
                entity: "solid",
                ..
            })
        ));
        let fresh = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        assert!(fresh > retired);
    }
    /// Runs one `executeBatch` operation and returns its `ok` value.
    fn batch_ok(kernel: &mut RemusKernel, op: &str, args: &serde_json::Value) -> serde_json::Value {
        let ops = serde_json::json!([{"op": op, "args": args}]);
        let output: serde_json::Value =
            serde_json::from_str(&kernel.execute_batch(&ops.to_string())).unwrap();
        assert!(output[0].get("error").is_none(), "{op}: {output}");
        output[0]["ok"].clone()
    }

    /// Extrudes `face` 3 along +Z through `executeBatch` and returns the
    /// solid handle and its batch-measured volume.
    fn batch_extrude(kernel: &mut RemusKernel, face: u32) -> (u32, f64) {
        let args = serde_json::json!({"face": face, "dz": 1.0, "distance": 3.0});
        let solid = batch_ok(kernel, "extrude", &args);
        let solid = u32::try_from(solid.as_u64().unwrap()).unwrap();
        let args = serde_json::json!({"solid": solid, "deflection": 0.001});
        let volume = batch_ok(kernel, "volume", &args).as_f64().unwrap();
        (solid, volume)
    }

    #[test]
    fn deleting_an_extrusion_keeps_its_profile_for_the_next_extrude() {
        // The extrusion's bottom cap reuses the profile's edges. Deleting it
        // once retired them under the still-live profile face, and the next
        // extrude failed with "edge not found".
        let mut kernel = RemusKernel::new();
        let face = kernel.make_rectangle(10.0, 5.0).unwrap();
        let (first, first_volume) = batch_extrude(&mut kernel, face);
        assert!((first_volume - 150.0).abs() < 1e-6, "{first_volume}");

        kernel.delete_solid_impl(first).unwrap();
        assert!(kernel.resolve_solid(first).is_err());
        let face_id = kernel.resolve_face(face).unwrap();
        remus_topology::validation::validate_face_loops(&kernel.topo, face_id).unwrap();
        assert!(kernel.topo.is_restore_fixed_point());

        let (second, second_volume) = batch_extrude(&mut kernel, face);
        assert!(second > first);
        assert!((second_volume - 150.0).abs() < 1e-6, "{second_volume}");

        // A checkpoint restore around a second delete keeps the profile too.
        let checkpoint = kernel.checkpoint().unwrap();
        kernel.delete_solid_impl(second).unwrap();
        kernel.restore(checkpoint).unwrap();
        assert!(kernel.resolve_solid(second).is_err());
        let (_, third_volume) = batch_extrude(&mut kernel, face);
        assert!((third_volume - 150.0).abs() < 1e-6, "{third_volume}");
    }

    #[test]
    fn fuse_all_does_not_pin_inputs_in_a_hidden_compound() {
        for batch in [false, true] {
            let mut kernel = RemusKernel::new();
            let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
            let result = if batch {
                let output: serde_json::Value = serde_json::from_str(&kernel.execute_batch(
                    &format!(r#"[{{"op":"fuseAll","args":{{"solids":[{solid}]}}}}]"#),
                ))
                .unwrap();
                assert!(output[0]["ok"].is_number(), "{output}");
                u32::try_from(output[0]["ok"].as_u64().unwrap()).unwrap()
            } else {
                kernel.fuse_all(vec![solid]).unwrap()
            };
            assert_eq!(kernel.topo.num_compounds(), 0);
            kernel.delete_solid_impl(solid).unwrap();
            assert!((kernel.volume(result, 0.01).unwrap() - 1.0).abs() < 1e-7);
        }
    }

    #[test]
    fn failed_batch_fuse_all_does_not_allocate_a_compound() {
        let mut kernel = RemusKernel::new();
        let output: serde_json::Value = serde_json::from_str(
            &kernel.execute_batch(r#"[{"op":"fuseAll","args":{"solids":[]}}]"#),
        )
        .unwrap();
        assert!(output[0]["error"].is_string(), "{output}");
        assert_eq!(kernel.topo.num_compounds(), 0);
    }
}
