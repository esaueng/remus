//! Opt-in kernel substage timings. Events contain no model contents.

use wasm_bindgen::prelude::*;

use crate::kernel::BrepKernel;

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = performance, js_name = now)]
    fn host_now_ms() -> f64;
}

fn now_ms() -> f64 {
    #[cfg(target_arch = "wasm32")]
    {
        host_now_ms()
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
        START
            .get_or_init(std::time::Instant::now)
            .elapsed()
            .as_secs_f64()
            * 1000.0
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Accounted live/retained topology bytes and unique NURBS payload bytes.
    /// Excludes allocator overhead, session sketches, pcurves, journals, caches
    /// and GPU data. Includes retired slots. Values are estimates, not RSS.
    ///
    /// # Errors
    /// Returns an error if the diagnostic cannot be serialized.
    #[wasm_bindgen(js_name = "checkpointMemoryStats")]
    pub fn checkpoint_memory_stats(&self) -> Result<String, JsError> {
        let mut topologies = std::collections::HashSet::new();
        let mut payloads = std::collections::HashSet::new();
        let mut bytes = 0usize;
        let mut nurbs_bytes = 0usize;
        let mut allocated_slots = 0usize;
        let mut retired_slots = 0usize;
        for topo in std::iter::once(&self.topo).chain(self.checkpoints.topologies()) {
            if !topologies.insert(std::rc::Rc::as_ptr(topo) as usize) {
                continue;
            }
            let usage = topo.memory_estimate(&mut payloads);
            bytes = bytes.saturating_add(usage.bytes());
            nurbs_bytes = nurbs_bytes.saturating_add(usage.nurbs_bytes);
            allocated_slots += usage.allocated_slots;
            retired_slots += usage.retired_slots;
        }
        let current = self
            .topo
            .memory_estimate(&mut std::collections::HashSet::new());
        #[cfg(target_arch = "wasm32")]
        let linear_bytes = Some(core::arch::wasm32::memory_size(0) * 65536);
        #[cfg(not(target_arch = "wasm32"))]
        let linear_bytes: Option<usize> = None;
        serde_json::to_string(&serde_json::json!({
            "linearMemoryBytes": linear_bytes,
            "estimatedBytes": bytes,
            "currentEstimatedBytes": current.bytes(),
            "nextMutationEstimatedBytes": bytes.saturating_add(current.arena_bytes).saturating_add(current.entity_list_bytes),
            "uniqueNurbsBytes": nurbs_bytes,
            "uniqueTopologies": topologies.len(),
            "allocatedSlots": allocated_slots,
            "retiredSlots": retired_slots,
            "checkpoints": self.checkpoints.active_len()
        })).map_err(|error| JsError::new(&error.to_string()))
    }

    /// Enable bounded module-local timings; disabled by default. This does
    /// not change tolerances, cache admission or numerical policy.
    #[wasm_bindgen(js_name = "setPerformanceTracing")]
    #[allow(clippy::unused_self)] // Instance method matches the host kernel API.
    pub fn set_performance_tracing(&self, enabled: bool) {
        remus_operations::performance::configure(enabled.then_some(now_ms));
    }

    /// Drain up to 4096 completed phase events as JSON.
    ///
    /// # Errors
    /// Returns an error if events cannot be serialized.
    #[wasm_bindgen(js_name = "drainPerformanceTrace")]
    #[allow(clippy::unused_self)] // Tracing is module-local across probe kernels.
    pub fn drain_performance_trace(&self) -> Result<String, JsError> {
        serde_json::to_string(&remus_operations::performance::drain())
            .map_err(|error| JsError::new(&error.to_string()))
    }
}
