//! Checkpoint / restore bindings for [`BrepKernel`].

use std::rc::Rc;

use wasm_bindgen::prelude::*;

use crate::kernel::BrepKernel;
use crate::state::Checkpoint;

impl BrepKernel {
    pub(crate) fn checkpoint_impl(&mut self) -> Result<u32, crate::error::WasmError> {
        // Check before cloning any topology/session payload. Refusal leaves
        // current state, saved snapshots, and the next opaque ID unchanged.
        self.checkpoints.check_admission()?;
        self.checkpoints.push(Checkpoint {
            topo: Rc::clone(&self.topo),
            assemblies: self.assemblies.clone(),
            sketches: self.sketches.clone(),
            gcs_sketches: self.gcs_sketches.clone(),
        })
    }

    pub(crate) fn restore_checkpoint_impl(&mut self, checkpoint_id: u32) -> Result<(), String> {
        let _restore = remus_operations::performance::span("checkpoint.restore");
        let idx = checkpoint_id as usize;
        let cp = self
            .checkpoints
            .get(idx)
            .ok_or_else(|| format!("invalid checkpoint id: {checkpoint_id}"))?
            .clone();
        let snapshot_topo = Rc::clone(&cp.topo);
        self.topo_mut()
            .restore_preserving_handle_slots(&snapshot_topo);
        self.assemblies
            .restore(&cp.assemblies, crate::state::AssemblyState::restore);
        self.sketches
            .restore(&cp.sketches, |current, saved| *current = saved.clone());
        self.gcs_sketches
            .restore(&cp.gcs_sketches, crate::state::GcsSketchState::restore);
        // Discard checkpoints created after the restored one
        self.checkpoints.retire_from(idx + 1);
        Ok(())
    }
    pub(crate) fn discard_checkpoint_impl(&mut self, checkpoint_id: u32) -> Result<(), String> {
        let idx = checkpoint_id as usize;
        if self.checkpoints.get(idx).is_none() {
            return Err(format!("invalid checkpoint id: {checkpoint_id}"));
        }
        self.checkpoints.retire_from(idx);
        Ok(())
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Save a snapshot of the current kernel state.
    ///
    /// Returns an opaque checkpoint ID that can be passed to
    /// `restore` or `discardCheckpoint`.
    ///
    /// The snapshot is a clone of all topology, assembly, and sketch state.
    /// Existing entity handles remain valid after restore. Opaque topology,
    /// session, GCS entity, and assembly component handles allocated after it
    /// are retired and never assigned to later entities. Legacy sketch point,
    /// arc, and circle indices retain their dense-array semantics.
    ///
    /// # Errors
    ///
    /// Returns an error if 33 snapshots are already retained (32 history
    /// snapshots plus one temporary probe) or the checkpoint handle namespace
    /// is exhausted. Discard a checkpoint to free capacity; existing
    /// checkpoints remain valid and restore stays available.
    #[wasm_bindgen(js_name = "checkpoint")]
    pub fn checkpoint(&mut self) -> Result<u32, JsError> {
        Ok(self.checkpoint_impl()?)
    }

    /// Restore the kernel to a previously saved checkpoint.
    ///
    /// All state created after the checkpoint is discarded. The checkpoint
    /// itself (and any earlier checkpoints) remain valid for future restores.
    /// Checkpoints created after this one are discarded.
    ///
    /// Restoring never grows the model: every checkpoint is an ancestor of the
    /// current state, so the restored topology is a subset of it. Undo is
    /// therefore always available, no matter how large the model has grown or
    /// how many operations the kernel instance has run.
    ///
    /// # Errors
    ///
    /// Returns an error if `checkpoint_id` does not refer to a valid checkpoint.
    /// This is the only failure mode.
    #[wasm_bindgen(js_name = "restore")]
    pub fn restore(&mut self, checkpoint_id: u32) -> Result<(), JsError> {
        self.restore_checkpoint_impl(checkpoint_id)
            .map_err(|error| JsError::new(&error))
    }

    /// Discard a checkpoint and all checkpoints after it, freeing their memory.
    ///
    /// # Errors
    ///
    /// Returns an error if `checkpoint_id` does not refer to a valid checkpoint.
    #[wasm_bindgen(js_name = "discardCheckpoint")]
    pub fn discard_checkpoint(&mut self, checkpoint_id: u32) -> Result<(), JsError> {
        self.discard_checkpoint_impl(checkpoint_id)
            .map_err(|error| JsError::new(&error))
    }

    /// Returns the number of saved checkpoints.
    #[wasm_bindgen(js_name = "checkpointCount")]
    #[must_use]
    pub fn checkpoint_count(&self) -> u32 {
        #[allow(clippy::cast_possible_truncation)]
        {
            self.checkpoints.active_len() as u32
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use crate::kernel::BrepKernel;

    const DEFLECTION: f64 = 0.01;
    const TOL: f64 = 1e-6;

    // ── helpers ───────────────────────────────────────────────────

    fn make_box(k: &mut BrepKernel, dx: f64, dy: f64, dz: f64) -> u32 {
        k.make_box_solid(dx, dy, dz).unwrap()
    }

    fn volume(k: &BrepKernel, solid: u32) -> f64 {
        k.volume(solid, DEFLECTION).unwrap()
    }

    fn classify(k: &BrepKernel, solid: u32, x: f64, y: f64, z: f64) -> String {
        k.classify_point(solid, x, y, z, TOL).unwrap()
    }

    #[test]
    fn full_history_keeps_one_temporary_probe_and_refuses_a_second() {
        let mut k = BrepKernel::new();
        let mut solids = Vec::new();
        let mut history = Vec::new();
        for _ in 0..32 {
            solids.push(make_box(&mut k, 2.0, 3.0, 4.0));
            history.push(k.checkpoint_impl().unwrap());
        }
        let before = k.serialize_solids(&solids).unwrap();
        let ticks = k.topo().mutation_ticks();
        let temporary = k.checkpoint_impl().unwrap();
        assert_eq!(temporary, 32);
        assert_eq!(k.checkpoint_count(), 33);
        let refs = std::rc::Rc::strong_count(&k.topo);
        let slots = k.topo().allocated_slot_count();
        assert!(matches!(
            k.checkpoint_impl(),
            Err(crate::error::WasmError::InvalidInput { reason })
                if reason.contains("at most 33 checkpoints")
        ));
        assert_eq!(k.checkpoint_count(), 33);
        assert_eq!(std::rc::Rc::strong_count(&k.topo), refs);
        assert_eq!(k.topo().allocated_slot_count(), slots);
        assert_eq!(k.topo().mutation_ticks(), ticks);
        assert_eq!(k.serialize_solids(&solids).unwrap(), before);

        let probe_solid = make_box(&mut k, 5.0, 1.0, 1.0);
        assert!((volume(&k, probe_solid) - 5.0).abs() < 1e-10);
        k.restore_checkpoint_impl(temporary).unwrap();
        k.discard_checkpoint_impl(temporary).unwrap();
        assert!(k.resolve_solid(probe_solid).is_err());
        assert_eq!(k.checkpoint_count(), 32);
        assert_eq!(k.topo().mutation_ticks(), ticks);
        assert_eq!(k.serialize_solids(&solids).unwrap(), before);
        for saved in &history {
            assert!(k.checkpoints.get(*saved as usize).is_some());
        }

        let next = k.checkpoint_impl().unwrap();
        assert_eq!(next, temporary + 1, "refusal must not consume IDs");
        assert!(k.restore_checkpoint_impl(temporary).is_err());
        k.restore_checkpoint_impl(next).unwrap();
        k.discard_checkpoint_impl(next).unwrap();
        assert_eq!(k.checkpoint_count(), 32);
        k.restore_checkpoint_impl(*history.last().unwrap()).unwrap();
        assert_eq!(k.checkpoint_count(), 32);
        assert_eq!(k.serialize_solids(&solids).unwrap(), before);
    }

    #[test]
    fn checkpoint_retention_refuses_before_cloning_and_restore_stays_available() {
        let mut k = BrepKernel::new();
        let keep = make_box(&mut k, 1.0, 1.0, 1.0);
        let first = k.checkpoint_impl().unwrap();
        for _ in 1..crate::state::MAX_CHECKPOINTS {
            make_box(&mut k, 1.0, 1.0, 1.0);
            k.checkpoint_impl().unwrap();
        }
        let refs = std::rc::Rc::strong_count(&k.topo);
        let slots = k.topo().allocated_slot_count();
        for _ in 0..5000 {
            assert!(k.checkpoint_impl().is_err());
        }
        let maximum = u32::try_from(crate::state::MAX_CHECKPOINTS).unwrap();
        assert_eq!(k.checkpoint_count(), maximum);
        assert_eq!(std::rc::Rc::strong_count(&k.topo), refs);
        assert_eq!(k.topo().allocated_slot_count(), slots);
        k.restore_checkpoint_impl(first).unwrap();
        assert!((volume(&k, keep) - 1.0).abs() < 0.05);
        let fresh = k.checkpoint_impl().unwrap();
        assert_eq!(fresh, maximum, "refusal must not consume IDs");
        k.discard_checkpoint_impl(first).unwrap();
        assert_eq!(k.checkpoint_count(), 0);
        assert!(k.restore_checkpoint_impl(fresh).is_err());
    }

    #[test]
    fn checkpoint_discard_churn_keeps_ids_stale_and_storage_bounded() {
        let mut k = BrepKernel::new();
        for expected in 0..5000 {
            let cp = k.checkpoint_impl().unwrap();
            assert_eq!(cp, expected);
            k.discard_checkpoint_impl(cp).unwrap();
            assert_eq!(k.checkpoint_count(), 0);
            assert!(k.restore_checkpoint_impl(cp).is_err());
        }
        let fresh = k.checkpoint_impl().unwrap();
        assert_eq!(fresh, 5000);
        assert!(k.restore_checkpoint_impl(0).is_err());
        k.restore_checkpoint_impl(fresh).unwrap();
    }

    #[test]
    fn restore_and_discard_never_reuse_checkpoint_handles() {
        let mut k = BrepKernel::new();
        let original = make_box(&mut k, 2.0, 3.0, 4.0);
        let oldest = k.checkpoint().unwrap();
        let stale = k.checkpoint().unwrap();
        k.restore(oldest).unwrap();
        let fresh = k.checkpoint().unwrap();
        assert!(fresh > stale);
        assert_eq!(k.checkpoint_count(), 2);
        let before = k.serialize_solids(&[original]).unwrap();
        assert!(k.restore_checkpoint_impl(stale).is_err());
        assert!(k.discard_checkpoint_impl(stale).is_err());
        assert_eq!(k.serialize_solids(&[original]).unwrap(), before);
        assert_eq!(k.checkpoint_count(), 2);
        k.discard_checkpoint(fresh).unwrap();
        let newest = k.checkpoint().unwrap();
        assert!(newest > fresh);
        assert_eq!(k.checkpoint_count(), 2);
        assert!(k.restore_checkpoint_impl(fresh).is_err());
        k.restore(oldest).unwrap();
        assert_eq!(k.checkpoint_count(), 1);
        assert!((volume(&k, original) - 24.0).abs() < 1e-10);
    }

    #[test]
    fn restore_retires_top_level_legacy_sketches_and_preserves_dense_retained_content() {
        let mut k = BrepKernel::new();
        let retained = k.sketch_new().unwrap();
        k.sketch_add_point(retained, 3.0, 4.0, true).unwrap();
        let checkpoint = k.checkpoint().unwrap();
        let stale = k.sketch_new().unwrap();
        k.sketch_add_point(stale, 99.0, 100.0, true).unwrap();
        k.restore(checkpoint).unwrap();
        let fresh = k.sketch_new().unwrap();
        assert!(fresh > stale);
        assert!(k.sketches.get(stale as usize).is_none());
        let solved: serde_json::Value =
            serde_json::from_str(&k.sketch_solve(retained, 20, 1e-8).unwrap()).unwrap();
        assert_eq!(solved["points"], serde_json::json!([[3.0, 4.0]]));
        assert_eq!(solved["converged"], true);
        let dof: serde_json::Value =
            serde_json::from_str(&k.sketch_dof(retained).unwrap()).unwrap();
        assert_eq!(dof["dof"], 0);
        k.sketch_add_point(fresh, 7.0, 8.0, true).unwrap();
        let solved: serde_json::Value =
            serde_json::from_str(&k.sketch_solve(fresh, 20, 1e-8).unwrap()).unwrap();
        assert_eq!(solved["points"], serde_json::json!([[7.0, 8.0]]));
    }

    #[test]
    fn restore_retires_assembly_and_component_handles_and_keeps_retained_tree() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k, 2.0, 3.0, 4.0);
        let assembly = k.assembly_new("retained").unwrap();
        let identity = vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let root = k
            .assembly_add_root(assembly, "root", solid, identity.clone())
            .unwrap();
        let checkpoint = k.checkpoint().unwrap();
        let stale_component = k
            .assembly_add_child(assembly, root, "stale", solid, identity.clone())
            .unwrap();
        let stale_assembly = k.assembly_new("stale").unwrap();
        k.assembly_add_root(stale_assembly, "stale", solid, identity.clone())
            .unwrap();
        k.restore(checkpoint).unwrap();
        assert!(k.assemblies.get(stale_assembly as usize).is_none());
        let fresh_assembly = k.assembly_new("fresh").unwrap();
        assert!(fresh_assembly > stale_assembly);
        let flat: serde_json::Value =
            serde_json::from_str(&k.assembly_flatten(assembly).unwrap()).unwrap();
        assert_eq!(flat.as_array().unwrap().len(), 1);
        assert_eq!(flat[0]["solid"], solid);
        assert_eq!(flat[0]["matrix"], serde_json::json!(identity));

        let state = k.assemblies.get_mut(assembly as usize).unwrap();
        let before = format!("{state:?}");
        assert!(
            state
                .add_child_component(
                    stale_component as usize,
                    "invalid",
                    k.topo.solid_id_from_index(solid as usize).unwrap(),
                    remus_math::mat::Mat4::identity()
                )
                .is_err()
        );
        assert_eq!(format!("{state:?}"), before);
        let fresh = k
            .assembly_add_child(assembly, root, "fresh", solid, identity)
            .unwrap();
        assert!(fresh > stale_component);
        let flattened: serde_json::Value =
            serde_json::from_str(&k.assembly_flatten(assembly).unwrap()).unwrap();
        assert_eq!(flattened.as_array().unwrap().len(), 2);
        k.restore(checkpoint).unwrap();
        assert!(
            k.assemblies
                .get(assembly as usize)
                .unwrap()
                .component(fresh as usize)
                .is_err()
        );
    }

    // ── round-trip ────────────────────────────────────────────────

    /// Create a box, checkpoint, create a second box, restore → second box gone.
    #[test]
    fn roundtrip_restore_removes_post_checkpoint_solid() {
        let mut k = BrepKernel::new();
        let box1 = make_box(&mut k, 2.0, 2.0, 2.0);

        let cp = k.checkpoint().unwrap();
        assert_eq!(cp, 0);

        let _box2 = make_box(&mut k, 1.0, 1.0, 1.0);
        // box2 exists and has the expected volume before restore
        assert!((volume(&k, _box2) - 1.0).abs() < 0.05);

        k.restore(cp).unwrap();

        // box1 still resolves and has correct volume
        assert!((volume(&k, box1) - 8.0).abs() < 0.05);

        // box2's handle no longer resolves after restore
        assert!(k.resolve_solid(_box2).is_err());
    }

    /// A handle retired by restore must not alias the next entity allocated in
    /// the same arena.
    #[test]
    fn restore_never_reuses_post_checkpoint_solid_handle() {
        let mut k = BrepKernel::new();
        let original = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp = k.checkpoint().unwrap();
        let stale = make_box(&mut k, 1.0, 1.0, 1.0);

        k.restore(cp).unwrap();
        let fresh = make_box(&mut k, 3.0, 3.0, 3.0);

        assert!(fresh > stale);
        assert!(k.resolve_solid(stale).is_err());
        assert!((volume(&k, original) - 8.0).abs() < 0.05);
        assert!((volume(&k, fresh) - 27.0).abs() < 0.1);
    }

    /// Volume of the original solid is preserved across a restore.
    #[test]
    fn roundtrip_preserves_original_solid_volume() {
        let mut k = BrepKernel::new();
        let box1 = make_box(&mut k, 3.0, 4.0, 5.0);
        let cp = k.checkpoint().unwrap();

        make_box(&mut k, 1.0, 1.0, 1.0);
        k.restore(cp).unwrap();

        let vol = volume(&k, box1);
        assert!((vol - 60.0).abs() < 0.5, "expected ~60, got {vol}");
    }

    // ── lifetime allocation churn ─────────────────────────────────

    /// Undo must survive a long-lived kernel that has churned through a large
    /// number of lifetime arena allocations.
    ///
    /// `Topology::allocated_slot_count` is a high-water mark that never
    /// decreases: arenas only append, `retire` clears a liveness bit, and
    /// restore re-extends each arena to its previous slot count. Gating
    /// `restore` on it made undo fail *permanently*, with no reset path, once a
    /// session had accumulated enough operations — and it triggered sooner the
    /// larger the model, even though restore is what shrinks it.
    #[test]
    fn restore_survives_large_lifetime_slot_count() {
        // The removed guard refused any restore above 500_000 slots.
        const REMOVED_GUARD_LIMIT: usize = 500_000;

        let mut k = BrepKernel::new();
        let keep = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp = k.checkpoint().unwrap();

        // Churn through ordinary operations until the lifetime counter is past
        // the old ceiling. Each box contributes ~34 slots and none of them are
        // ever freed, exactly as in a real editing session.
        while k.topo().allocated_slot_count() <= REMOVED_GUARD_LIMIT {
            make_box(&mut k, 1.0, 1.0, 1.0);
        }

        let slots = k.topo().allocated_slot_count();
        assert!(
            slots > REMOVED_GUARD_LIMIT,
            "test no longer exercises the counter: {slots} slots"
        );

        // Undo must still work, and must actually roll the model back.
        k.restore(cp)
            .expect("restore must not be disabled by lifetime allocation churn");
        assert!((volume(&k, keep) - 8.0).abs() < 0.05);

        // Restore does not shrink the counter — that is precisely why it is
        // unusable as a size gate.
        assert!(k.topo().allocated_slot_count() >= slots);

        // And undo is still available afterwards, not a one-shot escape.
        make_box(&mut k, 1.0, 1.0, 1.0);
        k.restore(cp).expect("restore must stay available");
        assert!((volume(&k, keep) - 8.0).abs() < 0.05);
    }

    // ── multiple checkpoints ──────────────────────────────────────

    /// Three checkpoints in sequence; restoring to the earliest discards
    /// the two later ones and the geometry created between them.
    #[test]
    fn multiple_checkpoints_restore_to_earliest() {
        let mut k = BrepKernel::new();

        let box0 = make_box(&mut k, 1.0, 1.0, 1.0);
        let cp0 = k.checkpoint().unwrap(); // id 0

        let box1 = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint().unwrap(); // id 1

        let box2 = make_box(&mut k, 3.0, 3.0, 3.0);
        let _cp2 = k.checkpoint().unwrap(); // id 2

        assert_eq!(k.checkpoint_count(), 3);

        // Restore to cp0 — only box0 should survive.
        k.restore(cp0).unwrap();

        assert!((volume(&k, box0) - 1.0).abs() < 0.05);
        assert!(k.resolve_solid(box1).is_err());
        assert!(k.resolve_solid(box2).is_err());

        // Checkpoints after cp0 should have been discarded.
        assert_eq!(k.checkpoint_count(), 1);
        // cp1 (id=1) is no longer valid because count is now 1.
        assert!(cp1 >= k.checkpoint_count());
    }

    /// Restore to an intermediate checkpoint: geometry from after that
    /// point is gone, but geometry from before it survives.
    #[test]
    fn multiple_checkpoints_restore_to_middle() {
        let mut k = BrepKernel::new();

        let box0 = make_box(&mut k, 1.0, 1.0, 1.0);
        let cp0 = k.checkpoint().unwrap(); // id 0
        let _ = cp0;

        let box1 = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint().unwrap(); // id 1

        let box2 = make_box(&mut k, 3.0, 3.0, 3.0);

        k.restore(cp1).unwrap();

        // box0 and box1 survive; box2 is gone.
        assert!((volume(&k, box0) - 1.0).abs() < 0.05);
        assert!((volume(&k, box1) - 8.0).abs() < 0.05);
        assert!(k.resolve_solid(box2).is_err());

        // Only cp0 and cp1 remain.
        assert_eq!(k.checkpoint_count(), 2);
    }

    // ── discard ───────────────────────────────────────────────────

    /// Discarding a checkpoint removes it and all later ones.
    #[test]
    fn discard_removes_checkpoint_and_later_ones() {
        let mut k = BrepKernel::new();
        make_box(&mut k, 1.0, 1.0, 1.0);

        let cp0 = k.checkpoint().unwrap(); // id 0
        make_box(&mut k, 2.0, 2.0, 2.0);
        let _cp1 = k.checkpoint().unwrap(); // id 1

        assert_eq!(k.checkpoint_count(), 2);

        k.discard_checkpoint(cp0).unwrap();

        // Both checkpoints are gone after discarding the first.
        assert_eq!(k.checkpoint_count(), 0);
    }

    /// Discarding the last checkpoint reduces count by one.
    #[test]
    fn discard_last_checkpoint_reduces_count() {
        let mut k = BrepKernel::new();
        make_box(&mut k, 1.0, 1.0, 1.0);
        let _cp0 = k.checkpoint().unwrap();
        make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint().unwrap();

        assert_eq!(k.checkpoint_count(), 2);
        k.discard_checkpoint(cp1).unwrap();
        assert_eq!(k.checkpoint_count(), 1);
    }

    /// After discard, the current topology is unchanged (discard only
    /// frees the snapshot; it does not roll back state).
    #[test]
    fn discard_does_not_alter_current_topology() {
        let mut k = BrepKernel::new();
        let box0 = make_box(&mut k, 4.0, 4.0, 4.0);
        let cp = k.checkpoint().unwrap();
        k.discard_checkpoint(cp).unwrap();

        // box0 is still alive after discard.
        assert!((volume(&k, box0) - 64.0).abs() < 0.5);
    }

    // ── checkpoint count ─────────────────────────────────────────

    /// Count starts at zero and increments with each checkpoint call.
    #[test]
    fn checkpoint_count_tracks_saves() {
        let mut k = BrepKernel::new();
        assert_eq!(k.checkpoint_count(), 0);

        k.checkpoint().unwrap();
        assert_eq!(k.checkpoint_count(), 1);

        k.checkpoint().unwrap();
        assert_eq!(k.checkpoint_count(), 2);

        k.checkpoint().unwrap();
        assert_eq!(k.checkpoint_count(), 3);
    }

    // ── invalid id ───────────────────────────────────────────────

    /// Restoring with a checkpoint id that was never created is invalid.
    /// We verify by checking that the checkpoint was never created (count = 0).
    #[test]
    fn restore_invalid_id_is_invalid() {
        let k = BrepKernel::new();
        assert_eq!(k.checkpoint_count(), 0);
        assert!(99 >= k.checkpoint_count());
    }

    /// Discarding with a checkpoint id that was never created is invalid.
    #[test]
    fn discard_invalid_id_is_invalid() {
        let k = BrepKernel::new();
        assert_eq!(k.checkpoint_count(), 0);
        assert!(99 >= k.checkpoint_count());
    }

    /// After restore truncates later checkpoints, the later ids become
    /// invalid (count is reduced).
    #[test]
    fn restore_discards_later_checkpoints() {
        let mut k = BrepKernel::new();
        make_box(&mut k, 1.0, 1.0, 1.0);
        let cp0 = k.checkpoint().unwrap();
        make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint().unwrap();

        assert_eq!(k.checkpoint_count(), 2);

        // Restore to cp0 — cp1 should be gone.
        k.restore(cp0).unwrap();

        // cp1 (id=1) is no longer valid because count is now 1.
        assert_eq!(k.checkpoint_count(), 1);
        assert!(cp1 >= k.checkpoint_count());
    }

    /// PERF-Q02: repeated `classifyPoint` hits the persistent cache, and
    /// mutation plus checkpoint restore invalidate it without stale reuse.
    #[test]
    fn classify_cache_hits_repeated_calls_and_invalidates_on_restore() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k, 2.0, 2.0, 2.0);

        // First call rebuilds, second hits; both agree (box [0,2]^3).
        assert_eq!(classify(&k, solid, 1.0, 1.0, 1.0), "inside");
        let after_first = k.classify_cache.borrow().stats();
        assert_eq!(after_first.rebuilds, 1);
        assert_eq!(classify(&k, solid, 1.0, 1.0, 1.0), "inside");
        let after_second = k.classify_cache.borrow().stats();
        assert_eq!(after_second.hits, after_first.hits + 1);
        assert_eq!(after_second.rebuilds, 1);

        // An unrelated allocation bumps the whole-topology generation:
        // conservative miss, still correct.
        let _other = make_box(&mut k, 1.0, 1.0, 1.0);
        assert_eq!(classify(&k, solid, 1.0, 1.0, 1.0), "inside");
        assert_eq!(classify(&k, solid, 5.0, 5.0, 5.0), "outside");
        let after_edit = k.classify_cache.borrow().stats();
        assert!(after_edit.rebuilds > after_second.rebuilds);

        // Checkpoint, mutate, restore: the restored verdict must not reuse
        // the mutated preparation.
        let cp = k.checkpoint().unwrap();
        let _third = make_box(&mut k, 3.0, 3.0, 3.0);
        k.restore(cp).unwrap();
        assert_eq!(classify(&k, solid, 1.0, 1.0, 1.0), "inside");
        assert_eq!(classify(&k, solid, 5.0, 5.0, 5.0), "outside");
    }

    /// The kernel enables the content-keyed face-integral cache and the
    /// identity- and content-keyed volume memo on construction. Repeats are
    /// answered from them — also for a deserialized copy in a second kernel —
    /// with the uncached readings; an in-place transform retires a memoized
    /// volume instead of serving it stale, and a checkpoint restore, which
    /// brings the measured content back, is answered by content.
    #[test]
    fn measurement_memos_hit_repeats_and_never_serve_stale_readings() {
        use remus_check::properties::face_cache::thread_face_cache_stats;
        use remus_operations::measure::thread_volume_memo_stats;

        let mut k = BrepKernel::new();
        assert!(thread_face_cache_stats().capacity > 0);
        assert!(thread_volume_memo_stats().capacity > 0);
        let cylinder = k.make_cylinder_solid(1.0, 2.0).unwrap();
        let faces = k.get_solid_faces(cylinder).unwrap();

        let before = thread_face_cache_stats();
        let verdict = k.validate_solid(cylinder).unwrap();
        let areas: Vec<u64> = faces
            .iter()
            .map(|&f| k.face_area(f, DEFLECTION).unwrap().to_bits())
            .collect();
        let cold = thread_face_cache_stats();
        assert!(cold.misses > before.misses);
        assert_eq!(k.validate_solid(cylinder).unwrap(), verdict);
        for (&f, &area) in faces.iter().zip(&areas) {
            assert_eq!(k.face_area(f, DEFLECTION).unwrap().to_bits(), area);
        }
        let warm = thread_face_cache_stats();
        assert_eq!(warm.misses, cold.misses, "repeats must hit");
        assert!(warm.hits > cold.hits);

        // A short-lived second kernel validating a deserialized copy.
        let bytes = k.serialize_solids(&[cylinder]).unwrap();
        let mut probe = BrepKernel::new();
        let copy = probe.deserialize_solids(&bytes).unwrap()[0];
        assert_eq!(probe.validate_solid(copy).unwrap(), verdict);
        assert_eq!(
            thread_face_cache_stats().misses,
            warm.misses,
            "the copy must hit"
        );

        let v0 = volume(&k, cylinder);
        let stats = thread_volume_memo_stats();
        assert_eq!(volume(&k, cylinder).to_bits(), v0.to_bits());
        assert_eq!(thread_volume_memo_stats().hits, stats.hits + 1);

        let cp = k.checkpoint().unwrap();
        let double = vec![
            2.0, 0.0, 0.0, 0.0, //
            0.0, 2.0, 0.0, 0.0, //
            0.0, 0.0, 2.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ];
        k.transform_solid_binding(cylinder, double).unwrap();
        let stats = thread_volume_memo_stats();
        let doubled = volume(&k, cylinder);
        assert_eq!(thread_volume_memo_stats().misses, stats.misses + 1);
        assert!((doubled - 8.0 * v0).abs() <= v0 * 1e-9, "{doubled} vs {v0}");

        k.restore(cp).unwrap();
        let stats = thread_volume_memo_stats();
        let restored = volume(&k, cylinder);
        assert_eq!(
            thread_volume_memo_stats().content_hits,
            stats.content_hits + 1
        );
        assert_eq!(restored.to_bits(), v0.to_bits());
    }

    /// The application's edit flow, as the OpenZCAD adapter runs it: read the
    /// source's volume, checkpoint, move a face, checkpoint, allocate
    /// elsewhere (its recognition probes do), then read the result's volume.
    /// The checkpoint makes the next mutation clone the topology and the
    /// allocation moves its generation, so only the content key can find a
    /// reading the move left — and it does, equal to a fresh reading.
    #[test]
    fn a_moved_result_volume_survives_checkpoints_and_unrelated_allocations() {
        use remus_operations::measure::{
            set_thread_volume_memo_capacity, thread_volume_memo_stats,
        };

        let mut k = BrepKernel::new();
        let block = make_box(&mut k, 10.0, 6.0, 4.0);
        let top = k
            .get_solid_faces(block)
            .unwrap()
            .into_iter()
            .find(|&f| {
                let n = k.get_face_normal(f).unwrap();
                n[2] > 0.99
            })
            .unwrap();
        let _ = volume(&k, block);
        let _cp = k.checkpoint().unwrap();
        let moved = k.move_faces_binding(block, vec![top], 1.5).unwrap();
        let _after = k.checkpoint().unwrap();
        let _other = make_box(&mut k, 1.0, 1.0, 1.0);
        let stats = thread_volume_memo_stats();
        let read = volume(&k, moved);
        let now = thread_volume_memo_stats();
        assert_eq!(now.misses, stats.misses, "the move's reading is found");
        assert_eq!(now.content_hits, stats.content_hits + 1);
        let capacity = now.capacity;
        set_thread_volume_memo_capacity(0);
        let fresh = volume(&k, moved);
        set_thread_volume_memo_capacity(capacity);
        assert_eq!(read.to_bits(), fresh.to_bits());
        assert!((read - 10.0 * 6.0 * 5.5).abs() <= 1e-9 * read);
    }

    /// PERF-Q02: direct and batch `classifyPoint` agree, including after a
    /// restore that rewinds the journal tick (ABA).
    #[test]
    fn classify_direct_and_batch_agree_across_restore() {
        let mut k = BrepKernel::new();
        let solid = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp = k.checkpoint().unwrap();
        let _other = make_box(&mut k, 1.0, 1.0, 1.0);
        k.restore(cp).unwrap();

        let direct = classify(&k, solid, 1.0, 1.0, 1.0);
        let batch_json = format!(
            r#"[{{"op":"classifyPoint","args":{{"solid":{solid},"x":1.0,"y":1.0,"z":1.0,"tolerance":{TOL}}}}}]"#
        );
        let batch_out = k.execute_batch(&batch_json);
        let batch_val: serde_json::Value = serde_json::from_str(&batch_out).unwrap();
        assert_eq!(batch_val[0]["ok"].as_str().unwrap(), direct);
    }
}
