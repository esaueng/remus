//! Topological entity lifecycle bindings.

use wasm_bindgen::prelude::*;

use crate::kernel::BrepKernel;

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
    /// Returns an error if `solid` is not a live solid handle or its topology
    /// tree contains an invalid reference.
    #[wasm_bindgen(js_name = "deleteSolid")]
    pub fn delete_solid(&mut self, solid: u32) -> Result<(), JsError> {
        let solid_id = self.resolve_solid(solid)?;
        self.topo_mut().delete_solid(solid_id)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use crate::error::WasmError;
    use crate::kernel::BrepKernel;

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
}
