//! Diagnostic-build state oracle for workflow qualification, not persistence.

use wasm_bindgen::prelude::*;

use crate::kernel::BrepKernel;

#[wasm_bindgen]
impl BrepKernel {
    /// Snapshot every logical session field, including vacant handle slots,
    /// sketches, assemblies, checkpoints, and the poison flag.
    ///
    /// Compare only within one live process and one build. Debug formatting
    /// is deliberately not a portable serialization format or a public API.
    #[wasm_bindgen(js_name = "workflowStateSnapshot")]
    #[must_use]
    pub fn workflow_state_snapshot(&self) -> String {
        format!("{self:?}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::{GcsSketchState, SketchState};

    #[test]
    fn snapshot_detects_non_geometry_state_changes() {
        let mut kernel = BrepKernel::new();
        let mut before = kernel.workflow_state_snapshot();
        kernel.sketches.push(SketchState::default());
        let mut after = kernel.workflow_state_snapshot();
        assert_ne!(before, after);
        before = after;
        kernel.gcs_sketches.push(GcsSketchState::default());
        after = kernel.workflow_state_snapshot();
        assert_ne!(before, after);
        before = after;
        kernel.assembly_new("sentinel");
        after = kernel.workflow_state_snapshot();
        assert_ne!(before, after);
        before = after;
        kernel.checkpoint();
        after = kernel.workflow_state_snapshot();
        assert_ne!(before, after);
        before = after;
        kernel.poisoned = true;
        assert_ne!(before, kernel.workflow_state_snapshot());
    }
}
