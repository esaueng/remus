//! Checkpoint and sketch state types used by [`super::kernel::BrepKernel`].

use std::collections::BTreeMap;
use std::rc::Rc;

use remus_topology::Topology;

/// A saved snapshot of the kernel state that can be restored.
#[derive(Debug, Clone)]
pub struct Checkpoint {
    pub topo: Rc<Topology>,
    pub assemblies: HandleStore<AssemblyState>,
    pub sketches: HandleStore<SketchState>,
    pub gcs_sketches: HandleStore<GcsSketchState>,
}

/// Maximum retained complete snapshots in one kernel session.
pub const MAX_CHECKPOINTS: usize = 32;

/// Sparse, monotonic checkpoint handles. Retired snapshots leave no tombstone
/// allocation, while their IDs remain permanently stale.
#[derive(Debug, Default)]
pub struct CheckpointStore {
    live: BTreeMap<u32, Checkpoint>,
    next: u32,
}

impl CheckpointStore {
    pub(crate) fn check_admission(&self) -> Result<(), crate::error::WasmError> {
        if self.live.len() >= MAX_CHECKPOINTS {
            return Err(crate::error::WasmError::InvalidInput {
                reason: format!(
                    "at most {MAX_CHECKPOINTS} checkpoints may be retained; discard a checkpoint before saving another"
                ),
            });
        }
        next_handle(self.next as usize)?;
        Ok(())
    }

    pub(crate) fn push(&mut self, checkpoint: Checkpoint) -> Result<u32, crate::error::WasmError> {
        self.check_admission()?;
        let id = self.next;
        self.live.insert(id, checkpoint);
        self.next += 1;
        Ok(id)
    }

    pub(crate) fn get(&self, index: usize) -> Option<&Checkpoint> {
        self.live.get(&u32::try_from(index).ok()?)
    }

    pub(crate) fn active_len(&self) -> usize {
        self.live.len()
    }

    pub(crate) fn retire_from(&mut self, index: usize) {
        if let Ok(id) = u32::try_from(index) {
            // split_off removes all later IDs in one bounded pass.
            drop(self.live.split_off(&id));
        }
    }
}

/// State for one sketch in the typed GCS API (`gcs*` bindings).
///
/// Holds a persistent [`remus_sketch::GcsSystem`] plus the handle
/// tables that map the opaque `u32` values held by JS onto the system's
/// generational handles. Removed entities leave a stale entry in their
/// table; the generational arena rejects stale handles, so reuse after
/// removal surfaces as a typed error instead of aliasing.
#[derive(Debug, Default, Clone)]
pub struct GcsSketchState {
    /// The persistent constraint system.
    pub sys: remus_sketch::GcsSystem,
    /// JS handle → point id.
    pub points: Vec<Option<remus_sketch::PointId>>,
    /// JS handle → line id.
    pub lines: Vec<Option<remus_sketch::LineId>>,
    /// JS handle → circle id.
    pub circles: Vec<Option<remus_sketch::CircleId>>,
    /// JS handle → arc id.
    pub arcs: Vec<Option<remus_sketch::ArcId>>,
    /// JS handle → ellipse id.
    pub ellipses: Vec<Option<remus_sketch::EllipseId>>,
    /// JS handle → constraint id.
    pub constraints: Vec<Option<remus_sketch::ConstraintId>>,
}

/// Internal state for an in-progress sketch.
///
/// Stores points and constraints for the legacy index-based JS API.
/// A `GcsSystem` is created on-the-fly during `sketch_solve`.
#[derive(Debug, Default, Clone)]
pub struct SketchState {
    /// Legacy point/constraint storage for backward-compat API.
    pub points: Vec<remus_operations::sketch::SketchPoint>,
    pub constraints: Vec<remus_operations::sketch::Constraint>,
    /// Arc definitions: `(center_idx, start_idx, end_idx)` into points.
    pub arcs: Vec<(usize, usize, usize)>,
    /// Circle definitions: `(center_idx, radius)`, where `center_idx` indexes into `points`.
    pub circles: Vec<(usize, f64)>,
    /// Deferred arc-referencing constraints stored as raw JSON.
    /// These are resolved into real `GcsConstraint` values at solve time
    /// when entity IDs are available.
    pub deferred_constraints: Vec<serde_json::Value>,
}

/// Append-only opaque session handles. Restore retires slots instead of
/// rewinding the index used by the next allocation.
#[derive(Debug, Clone)]
pub struct HandleStore<T> {
    slots: Vec<Option<T>>,
}

impl<T> Default for HandleStore<T> {
    fn default() -> Self {
        Self { slots: Vec::new() }
    }
}

impl<T> HandleStore<T> {
    pub(crate) fn get(&self, index: usize) -> Option<&T> {
        self.slots.get(index)?.as_ref()
    }
    pub(crate) fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.slots.get_mut(index)?.as_mut()
    }
    pub(crate) fn iter(&self) -> impl Iterator<Item = (usize, &T)> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(index, value)| value.as_ref().map(|v| (index, v)))
    }
    pub(crate) fn push(&mut self, value: T) -> Result<u32, crate::error::WasmError> {
        let handle = next_handle(self.slots.len())?;
        self.slots.push(Some(value));
        Ok(handle)
    }
}

impl<T: Clone> HandleStore<T> {
    pub(crate) fn restore(&mut self, snapshot: &Self, restore_inner: impl Fn(&mut T, &T)) {
        for (index, slot) in self.slots.iter_mut().enumerate() {
            match (slot.as_mut(), snapshot.get(index)) {
                (Some(current), Some(saved)) => restore_inner(current, saved),
                (_, saved) => *slot = saved.cloned(),
            }
        }
        // Checkpoints may be restored repeatedly; their live values always fit
        // below the preserved allocation high-water mark.
        for slot in snapshot.slots.iter().skip(self.slots.len()) {
            self.slots.push(slot.clone());
        }
    }
}

/// Check the opaque session namespace before any native allocation.
///
/// # Errors
///
/// Returns an error if another handle cannot fit within the namespace.
pub fn next_handle(len: usize) -> Result<u32, crate::error::WasmError> {
    u32::try_from(len)
        .ok()
        .filter(|handle| *handle < u32::MAX)
        .ok_or_else(|| crate::error::WasmError::InvalidInput {
            reason: "session handle namespace exhausted".into(),
        })
}

impl GcsSketchState {
    pub(crate) fn restore(&mut self, snapshot: &Self) {
        fn restore_table<T: Copy>(current: &mut Vec<Option<T>>, saved: &[Option<T>]) {
            current.resize(current.len().max(saved.len()), None);
            for (index, slot) in current.iter_mut().enumerate() {
                *slot = saved.get(index).copied().flatten();
            }
        }
        self.sys = snapshot.sys.clone();
        restore_table(&mut self.points, &snapshot.points);
        restore_table(&mut self.lines, &snapshot.lines);
        restore_table(&mut self.circles, &snapshot.circles);
        restore_table(&mut self.arcs, &snapshot.arcs);
        restore_table(&mut self.ellipses, &snapshot.ellipses);
        restore_table(&mut self.constraints, &snapshot.constraints);
    }
}

/// Assembly component handles are independent of the native dense allocator.
#[derive(Debug, Default, Clone)]
pub struct AssemblyState {
    pub assembly: remus_operations::assembly::Assembly,
    components: Vec<Option<usize>>,
}

impl AssemblyState {
    pub(crate) fn new(name: &str) -> Self {
        Self {
            assembly: remus_operations::assembly::Assembly::new(name),
            components: Vec::new(),
        }
    }
    pub(crate) fn restore(&mut self, snapshot: &Self) {
        self.assembly = snapshot.assembly.clone();
        self.components
            .resize(self.components.len().max(snapshot.components.len()), None);
        for (index, slot) in self.components.iter_mut().enumerate() {
            *slot = snapshot.components.get(index).copied().flatten();
        }
    }
    pub(crate) fn component(&self, handle: usize) -> Result<usize, crate::error::WasmError> {
        self.components.get(handle).copied().flatten().ok_or(
            crate::error::WasmError::InvalidHandle {
                entity: "assembly component",
                index: handle,
            },
        )
    }
    pub(crate) fn add_root_component(
        &mut self,
        name: &str,
        solid: remus_topology::SolidId,
        matrix: remus_math::mat::Mat4,
    ) -> Result<u32, crate::error::WasmError> {
        let handle = next_handle(self.components.len())?;
        let native = self.assembly.add_root_component(name, solid, matrix);
        self.components.push(Some(native));
        Ok(handle)
    }
    pub(crate) fn add_child_component(
        &mut self,
        parent: usize,
        name: &str,
        solid: remus_topology::SolidId,
        matrix: remus_math::mat::Mat4,
    ) -> Result<u32, crate::error::WasmError> {
        let parent = self.component(parent)?;
        let handle = next_handle(self.components.len())?;
        let native = self
            .assembly
            .add_child_component(parent, name, solid, matrix)?;
        self.components.push(Some(native));
        Ok(handle)
    }
}

impl std::ops::Deref for AssemblyState {
    type Target = remus_operations::assembly::Assembly;
    fn deref(&self) -> &Self::Target {
        &self.assembly
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]
    #[test]
    fn handle_namespace_guard_is_checked_before_allocation() {
        assert_eq!(super::next_handle(0).unwrap(), 0);
        assert_eq!(
            super::next_handle(u32::MAX as usize - 1).unwrap(),
            u32::MAX - 1
        );
        assert!(super::next_handle(u32::MAX as usize).is_err());
        #[cfg(target_pointer_width = "64")]
        assert!(super::next_handle(u32::MAX as usize + 1).is_err());
    }
}
