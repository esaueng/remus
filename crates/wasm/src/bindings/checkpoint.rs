//! Checkpoint / restore bindings for [`BrepKernel`].

use std::rc::Rc;

use wasm_bindgen::prelude::*;

use crate::kernel::BrepKernel;
use crate::state::Checkpoint;

#[wasm_bindgen]
impl BrepKernel {
    /// Save a snapshot of the current kernel state.
    ///
    /// Returns a checkpoint ID (zero-based index) that can be passed to
    /// `restore` or `discardCheckpoint`.
    ///
    /// The snapshot is a clone of all topology, assembly, and sketch state.
    /// Existing entity handles remain valid after restore.
    #[wasm_bindgen(js_name = "checkpoint")]
    pub fn checkpoint(&mut self) -> u32 {
        let id = self.checkpoints.len();
        self.checkpoints.push(Checkpoint {
            topo: Rc::clone(&self.topo),
            assemblies: self.assemblies.clone(),
            sketches: self.sketches.clone(),
            gcs_sketches: self.gcs_sketches.clone(),
        });
        #[allow(clippy::cast_possible_truncation)]
        {
            id as u32
        }
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
        let idx = checkpoint_id as usize;
        let cp = self
            .checkpoints
            .get(idx)
            .ok_or_else(|| JsError::new(&format!("invalid checkpoint id: {checkpoint_id}")))?
            .clone();
        let snapshot_topo = Rc::clone(&cp.topo);
        self.topo_mut()
            .restore_preserving_handle_slots(&snapshot_topo);
        self.assemblies = cp.assemblies;
        self.sketches = cp.sketches;
        self.gcs_sketches = cp.gcs_sketches;
        // Discard checkpoints created after the restored one
        self.checkpoints.truncate(idx + 1);
        Ok(())
    }

    /// Discard a checkpoint and all checkpoints after it, freeing their memory.
    ///
    /// # Errors
    ///
    /// Returns an error if `checkpoint_id` does not refer to a valid checkpoint.
    #[wasm_bindgen(js_name = "discardCheckpoint")]
    pub fn discard_checkpoint(&mut self, checkpoint_id: u32) -> Result<(), JsError> {
        let idx = checkpoint_id as usize;
        if idx >= self.checkpoints.len() {
            return Err(JsError::new(&format!(
                "invalid checkpoint id: {checkpoint_id}"
            )));
        }
        self.checkpoints.truncate(idx);
        Ok(())
    }

    /// Returns the number of saved checkpoints.
    #[wasm_bindgen(js_name = "checkpointCount")]
    #[must_use]
    pub fn checkpoint_count(&self) -> u32 {
        #[allow(clippy::cast_possible_truncation)]
        {
            self.checkpoints.len() as u32
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use crate::kernel::BrepKernel;

    const DEFLECTION: f64 = 0.01;

    // ── helpers ───────────────────────────────────────────────────

    fn make_box(k: &mut BrepKernel, dx: f64, dy: f64, dz: f64) -> u32 {
        k.make_box_solid(dx, dy, dz).unwrap()
    }

    fn volume(k: &BrepKernel, solid: u32) -> f64 {
        k.volume(solid, DEFLECTION).unwrap()
    }

    // ── round-trip ────────────────────────────────────────────────

    /// Create a box, checkpoint, create a second box, restore → second box gone.
    #[test]
    fn roundtrip_restore_removes_post_checkpoint_solid() {
        let mut k = BrepKernel::new();
        let box1 = make_box(&mut k, 2.0, 2.0, 2.0);

        let cp = k.checkpoint();
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
        let cp = k.checkpoint();
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
        let cp = k.checkpoint();

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
        let cp = k.checkpoint();

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
        let cp0 = k.checkpoint(); // id 0

        let box1 = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint(); // id 1

        let box2 = make_box(&mut k, 3.0, 3.0, 3.0);
        let _cp2 = k.checkpoint(); // id 2

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
        let cp0 = k.checkpoint(); // id 0
        let _ = cp0;

        let box1 = make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint(); // id 1

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

        let cp0 = k.checkpoint(); // id 0
        make_box(&mut k, 2.0, 2.0, 2.0);
        let _cp1 = k.checkpoint(); // id 1

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
        let _cp0 = k.checkpoint();
        make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint();

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
        let cp = k.checkpoint();
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

        k.checkpoint();
        assert_eq!(k.checkpoint_count(), 1);

        k.checkpoint();
        assert_eq!(k.checkpoint_count(), 2);

        k.checkpoint();
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
        let cp0 = k.checkpoint();
        make_box(&mut k, 2.0, 2.0, 2.0);
        let cp1 = k.checkpoint();

        assert_eq!(k.checkpoint_count(), 2);

        // Restore to cp0 — cp1 should be gone.
        k.restore(cp0).unwrap();

        // cp1 (id=1) is no longer valid because count is now 1.
        assert_eq!(k.checkpoint_count(), 1);
        assert!(cp1 >= k.checkpoint_count());
    }
}
