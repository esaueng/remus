//! Opt-in kernel substage timings. Events contain no model contents.

use wasm_bindgen::prelude::*;

use crate::kernel::RemusKernel;

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

#[cfg(any(target_arch = "wasm32", test))]
fn linear_memory_bytes(pages: usize) -> u64 {
    u64::try_from(pages)
        .unwrap_or(u64::MAX)
        .saturating_mul(65536)
}

#[wasm_bindgen]
impl RemusKernel {
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
        let linear_bytes = Some(linear_memory_bytes(core::arch::wasm32::memory_size(0)));
        #[cfg(not(target_arch = "wasm32"))]
        let linear_bytes: Option<u64> = None;
        let copy_bytes = if std::rc::Rc::strong_count(&self.topo) > 1 {
            current
                .arena_bytes
                .saturating_add(current.entity_list_bytes)
        } else {
            0
        };
        serde_json::to_string(&serde_json::json!({
            "linearMemoryBytes": linear_bytes,
            "estimatedBytes": bytes,
            "currentEstimatedBytes": current.bytes(),
            "nextMutationEstimatedBytes": bytes.saturating_add(copy_bytes),
            "uniqueNurbsBytes": nurbs_bytes,
            "uniqueTopologies": topologies.len(),
            "allocatedSlots": allocated_slots,
            "retiredSlots": retired_slots,
            "checkpoints": self.checkpoints.active_len()
        }))
        .map_err(|error| JsError::new(&error.to_string()))
    }

    /// Enable bounded module-local timings; disabled by default. This does
    /// not change tolerances, cache admission or numerical policy.
    #[wasm_bindgen(js_name = "setPerformanceTracing")]
    #[allow(clippy::unused_self)] // Instance method matches the host kernel API.
    pub fn set_performance_tracing(&self, enabled: bool) {
        remus_operations::performance::configure(enabled.then_some(now_ms));
    }

    /// Enable or disable this module's exact mesh-stage reuse. Disabling
    /// releases its retained entries. New kernels enable the default cache.
    #[wasm_bindgen(js_name = "setMeshReuseEnabled")]
    #[allow(clippy::unused_self)]
    pub fn set_mesh_reuse_enabled(&self, enabled: bool) {
        if enabled {
            remus_operations::tessellate::enable_mesh_reuse();
        } else {
            remus_operations::tessellate::disable_mesh_reuse();
        }
    }

    /// Bounded, module-local mesh reuse counters and storage estimate as
    /// JSON, or `null` when disabled. Contains no geometry or arena handles.
    /// The estimate is separate from checkpoint bytes and is not RSS.
    ///
    /// # Errors
    /// Returns an error if the diagnostic cannot be serialized.
    #[wasm_bindgen(js_name = "meshReuseStats")]
    #[allow(clippy::unused_self)]
    pub fn mesh_reuse_stats(&self) -> Result<String, JsError> {
        serde_json::to_string(&remus_operations::tessellate::mesh_reuse_stats())
            .map_err(|error| JsError::new(&error.to_string()))
    }

    /// Enable or disable exact reusable scalar measurements for this module.
    /// Explicit preferences are preserved when another kernel is constructed.
    #[wasm_bindgen(js_name = "setMeasurementReuseEnabled")]
    #[allow(clippy::unused_self)]
    pub fn set_measurement_reuse_enabled(&self, enabled: bool) {
        remus_operations::measure::set_measurement_reuse_enabled(enabled);
    }

    /// Exact measurement reuse counters and independent area storage bounds,
    /// as camelCase JSON. Existing volume memo bounds are included separately.
    ///
    /// # Errors
    /// Returns an error when the diagnostic cannot be serialized.
    #[wasm_bindgen(js_name = "measurementReuseStats")]
    #[allow(clippy::unused_self)]
    pub fn measurement_reuse_stats(&self) -> Result<String, JsError> {
        let reuse = remus_operations::measure::measurement_reuse_stats();
        let volume = remus_operations::measure::thread_volume_memo_stats();
        serde_json::to_string(&serde_json::json!({
            "reuse": reuse,
            "volumeMemo": {
                "hits": volume.hits, "contentHits": volume.content_hits,
                "misses": volume.misses, "entries": volume.len,
                "maxEntries": volume.capacity, "retainedBytes": volume.retained_bytes,
                "maxBytes": volume.byte_budget,
            }
        }))
        .map_err(|error| JsError::new(&error.to_string()))
    }

    /// Counters, entries and estimated retained bytes of every cache the
    /// kernel reads, as camelCase JSON: `{ caches: { <name>: { scope, ... } },
    /// estimatedBytes }`.
    ///
    /// `scope` is `"session"` for caches this kernel's cache scope holds (RFC
    /// 0009) and `"thread"` for caches still shared by the whole module. A
    /// cache that is off reads `null`; a byte figure the cache does not track
    /// reads `null` and is left out of `estimatedBytes`. Byte figures are
    /// estimates, not RSS. Contains no geometry or arena handles.
    ///
    /// # Errors
    /// Returns an error when the diagnostic cannot be serialized.
    #[wasm_bindgen(js_name = "cacheMemoryStats")]
    pub fn cache_memory_stats(&self) -> Result<String, JsError> {
        serde_json::to_string(&self.cache_memory_stats_json())
            .map_err(|error| JsError::new(&error.to_string()))
    }

    /// Drain up to 4096 phase and boolean-route events as JSON. Route
    /// observations have zero duration and names beginning `boolean.route.`.
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

impl RemusKernel {
    pub(crate) fn cache_memory_stats_json(&self) -> serde_json::Value {
        use remus_operations::{measure, query, tessellate};
        use serde_json::{Value, json};

        let pcurve = remus_algo::pcurve_memo::thread_pcurve_memo_stats();
        let wire = remus_algo::wire_area_memo::thread_wire_area_memo_stats();
        let ray = remus_algo::ray_geom_memo::thread_ray_geom_memo_stats();
        let face = remus_check::properties::face_cache::thread_face_cache_stats();
        let volume = measure::thread_volume_memo_stats();
        let reuse = measure::measurement_reuse_stats();
        let classification = self
            .classify_cache
            .try_borrow()
            .ok()
            .map(|cache| cache.stats());
        let caches = json!({
            "pcurve": {"scope": "thread", "hits": pcurve.hits, "misses": pcurve.misses,
                "entries": pcurve.len, "maxEntries": pcurve.capacity,
                "retainedBytes": null, "maxBytes": null},
            "wireArea": {"scope": "thread", "hits": wire.hits, "misses": wire.misses,
                "entries": wire.len, "maxEntries": wire.capacity,
                "retainedBytes": null, "maxBytes": null},
            "rayGeom": {"scope": "thread", "hits": ray.hits, "misses": ray.misses,
                "entries": ray.len, "maxEntries": ray.capacity,
                "retainedBytes": ray.bytes, "maxBytes": null},
            "faceIntegrals": {"scope": "thread", "hits": face.hits,
                "rereferenced": face.rereferenced, "translated": face.translated,
                "misses": face.misses, "evictions": face.evictions, "entries": face.len,
                "maxEntries": face.capacity, "retainedBytes": face.retained_bytes,
                "maxBytes": face.byte_budget},
            "volume": {"scope": "thread", "hits": volume.hits,
                "contentHits": volume.content_hits, "misses": volume.misses,
                "entries": volume.len, "maxEntries": volume.capacity,
                "retainedBytes": volume.retained_bytes, "maxBytes": volume.byte_budget},
            "measurementReuse": {"scope": "thread", "enabled": reuse.enabled,
                "areaIdentityHits": reuse.area_identity_hits,
                "areaContentHits": reuse.area_content_hits, "areaMisses": reuse.area_misses,
                "areaEvictions": reuse.area_evictions,
                "volumeIdentityHits": reuse.volume_identity_hits,
                "volumeContentHits": reuse.volume_content_hits,
                "entries": reuse.area_entries, "maxEntries": reuse.area_max_entries,
                "retainedBytes": reuse.area_retained_bytes, "maxBytes": reuse.area_max_bytes},
            "nurbsArea": measure::thread_nurbs_area_memo_stats().map(|s| json!({
                "scope": "thread", "hits": s.hits, "misses": s.misses,
                "evictions": s.evictions, "entries": s.len, "maxEntries": s.capacity,
                "retainedBytes": s.retained_bytes, "maxBytes": s.byte_budget})),
            "faceBox": measure::thread_face_box_memo_stats().map(|s| json!({
                "scope": "thread", "hits": s.hits, "misses": s.misses,
                "evictions": s.evictions, "entries": s.len, "maxEntries": s.capacity,
                "retainedBytes": s.retained_bytes, "maxBytes": s.byte_budget})),
            "faceMesh": tessellate::face_mesh_cache_stats().map(|s| json!({
                "scope": "thread", "exactHits": s.exact_hits,
                "translatedHits": s.translated_hits, "misses": s.misses,
                "translationRefused": s.translation_refused,
                "replayConflicts": s.replay_conflicts, "uncacheable": s.uncacheable,
                "evictions": s.evictions, "entries": s.entries, "maxEntries": s.max_entries,
                "retainedBytes": s.retained_bytes, "maxBytes": s.max_bytes})),
            "meshStage": tessellate::mesh_reuse_stats().map(|s| json!({
                "scope": "thread", "boundaryHits": s.boundary_hits, "edgeHits": s.edge_hits,
                "planarHits": s.planar_hits, "weldHits": s.weld_hits,
                "cleanupHits": s.cleanup_hits, "evictions": s.evictions,
                "entries": s.entries, "maxEntries": s.max_entries,
                "retainedBytes": s.retained_bytes, "maxBytes": s.max_bytes})),
            "crossing": query::crossing_memo_stats(&self.caches).map(|s| json!({
                "scope": "session", "hits": s.hits, "misses": s.misses,
                "entries": s.len, "maxEntries": s.capacity, "faces": s.faces,
                "retainedBytes": query::crossing_memo_content_bytes(&self.caches),
                "maxBytes": null})),
            "classification": classification.map(|s| json!({
                "scope": "session", "hits": s.hits, "misses": s.misses,
                "rebuilds": s.rebuilds, "evictions": s.evictions, "entries": s.len,
                "maxEntries": s.capacity, "retainedBytes": s.retained_bytes,
                "maxBytes": null})),
        });
        let estimated: u64 = caches
            .as_object()
            .into_iter()
            .flat_map(|caches| caches.values())
            .filter_map(|cache| cache.get("retainedBytes").and_then(Value::as_u64))
            .sum();
        json!({"caches": caches, "estimatedBytes": estimated})
    }
}

#[cfg(test)]
mod tests {
    #[test]
    #[allow(clippy::unwrap_used)]
    fn measurement_diagnostics_and_explicit_disable_survive_new_kernel() {
        let kernel = super::RemusKernel::new();
        kernel.set_measurement_reuse_enabled(false);
        let next = super::RemusKernel::new();
        let json: serde_json::Value =
            serde_json::from_str(&next.measurement_reuse_stats().unwrap()).unwrap();
        assert_eq!(json["reuse"]["enabled"], false);
        assert_eq!(json["reuse"]["areaMaxEntries"], 512);
        assert_eq!(json["reuse"]["areaMaxBytes"], 8 * 1024 * 1024);
        assert_eq!(json["volumeMemo"]["maxEntries"], 64);
        assert_eq!(json["volumeMemo"]["maxBytes"], 16 * 1024 * 1024);
        next.set_measurement_reuse_enabled(true);
    }

    /// A drilled block: its bore is the non-planar face the crossing memo
    /// interns.
    #[allow(clippy::unwrap_used)]
    fn drilled(kernel: &mut super::RemusKernel) -> u64 {
        let mut run = |op: serde_json::Value| -> serde_json::Value {
            let rows: serde_json::Value =
                serde_json::from_str(&kernel.execute_batch(&serde_json::json!([op]).to_string()))
                    .unwrap();
            assert!(rows[0].get("error").is_none(), "{rows}");
            rows[0]["ok"].clone()
        };
        let block =
            run(serde_json::json!({"op":"makeBox","args":{"width":20,"height":12,"depth":6}}));
        let tool = run(serde_json::json!({"op":"makeCylinder","args":{"radius":2,"height":10}}));
        run(serde_json::json!({"op":"transform","args":{"solid":tool,
            "matrix":[1,0,0,10,0,1,0,6,0,0,1,-2,0,0,0,1]}}));
        run(serde_json::json!({"op":"cut","args":{"solidA":block,"solidB":tool}}))
            .as_u64()
            .unwrap()
    }

    #[allow(clippy::unwrap_used)]
    fn relations(kernel: &mut super::RemusKernel, solid: u64) {
        let rows: serde_json::Value = serde_json::from_str(&kernel.execute_batch(
            &serde_json::json!([{"op":"solidEdgeRelations","args":{"solid":solid}}]).to_string(),
        ))
        .unwrap();
        assert!(rows[0]["ok"].is_array(), "{rows}");
    }

    fn crossing(kernel: &super::RemusKernel) -> (u64, u64) {
        let stats = kernel.cache_memory_stats_json();
        let row = &stats["caches"]["crossing"];
        (
            row["hits"].as_u64().unwrap_or_default(),
            row["misses"].as_u64().unwrap_or_default(),
        )
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn cache_memory_stats_reports_every_cache_with_its_scope() {
        let mut kernel = super::RemusKernel::new();
        let solid = drilled(&mut kernel);
        relations(&mut kernel, solid);
        let stats = kernel.cache_memory_stats_json();
        let caches = stats["caches"].as_object().unwrap();
        let names: Vec<&str> = caches.keys().map(String::as_str).collect();
        assert_eq!(
            names.len(),
            12,
            "eleven content caches and the classification cache: {names:?}"
        );
        for (name, row) in caches {
            assert!(row.is_object(), "{name} is on in a new kernel: {row}");
            let scope = row["scope"].as_str().unwrap();
            let session = matches!(name.as_str(), "crossing" | "classification");
            assert_eq!(scope, if session { "session" } else { "thread" }, "{name}");
        }
        assert!(caches["crossing"]["misses"].as_u64().unwrap() > 0);
        assert!(caches["crossing"]["retainedBytes"].as_u64().unwrap() > 0);
        let known: u64 = caches
            .values()
            .filter_map(|row| row["retainedBytes"].as_u64())
            .sum();
        assert_eq!(stats["estimatedBytes"].as_u64(), Some(known));
    }

    #[test]
    fn kernels_on_one_thread_share_the_session_crossing_memo() {
        let mut first = super::RemusKernel::new();
        let solid = drilled(&mut first);
        relations(&mut first, solid);
        let (hits, misses) = crossing(&first);
        assert!(misses > 0);
        // A second session on the thread builds the same body in its own
        // document: every count is a content-keyed hit in the shared memo.
        let mut second = super::RemusKernel::new();
        assert_eq!(second.caches, first.caches);
        let copy = drilled(&mut second);
        relations(&mut second, copy);
        let (shared_hits, shared_misses) = crossing(&second);
        assert_eq!(shared_misses, misses, "no count is recomputed");
        assert!(shared_hits > hits);
    }

    #[test]
    fn maximum_wasm_memory_is_four_gibibytes() {
        assert_eq!(super::linear_memory_bytes(65536), 4_294_967_296);
    }
}
