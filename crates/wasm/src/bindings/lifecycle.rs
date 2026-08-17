//! Topological entity lifecycle bindings.

use wasm_bindgen::prelude::*;

use crate::error::WasmError;
use crate::kernel::BrepKernel;

#[derive(Debug, thiserror::Error)]
enum LifecycleError {
    #[error(transparent)]
    Wasm(#[from] WasmError),
    #[error(transparent)]
    DeleteSolid(#[from] remus_topology::DeleteSolidError),
}

impl BrepKernel {
    fn delete_solid_impl(&mut self, solid: u32) -> Result<(), LifecycleError> {
        let solid_id = self.resolve_solid(solid)?;
        if let Some((assembly_index, _)) =
            self.assemblies.iter().enumerate().find(|(_, assembly)| {
                assembly
                    .bill_of_materials()
                    .iter()
                    .any(|entry| entry.solid_index == solid_id.index())
            })
        {
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
impl BrepKernel {
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
    use crate::kernel::BrepKernel;

    fn identity_matrix() -> Vec<f64> {
        vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]
    }

    #[test]
    fn delete_solid_invalidates_handle_without_reusing_its_slot() {
        let mut kernel = BrepKernel::new();
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
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let compound = kernel.make_compound(vec![solid]).unwrap();

        assert!(kernel.delete_solid_impl(solid).is_err());
        assert_eq!(kernel.get_compound_solids(compound).unwrap(), vec![solid]);
        assert!(kernel.resolve_solid(solid).is_ok());
    }

    #[test]
    fn delete_solid_rejects_live_assembly_reference_atomically() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let assembly = kernel.assembly_new("assembly");
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
    fn restore_does_not_revive_retired_solid() {
        let mut kernel = BrepKernel::new();
        let retired = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let checkpoint = kernel.checkpoint();

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
}
