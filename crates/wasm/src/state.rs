//! Checkpoint and sketch state types used by [`super::kernel::BrepKernel`].

use std::rc::Rc;

use remus_topology::Topology;

/// A saved snapshot of the kernel state that can be restored.
#[derive(Clone)]
pub struct Checkpoint {
    pub topo: Rc<Topology>,
    pub assemblies: Vec<remus_operations::assembly::Assembly>,
    pub sketches: Vec<SketchState>,
    pub gcs_sketches: Vec<GcsSketchState>,
}

/// State for one sketch in the typed GCS API (`gcs*` bindings).
///
/// Holds a persistent [`remus_sketch::GcsSystem`] plus the handle
/// tables that map the opaque `u32` values held by JS onto the system's
/// generational handles. Removed entities leave a stale entry in their
/// table; the generational arena rejects stale handles, so reuse after
/// removal surfaces as a typed error instead of aliasing.
#[derive(Default, Clone)]
pub struct GcsSketchState {
    /// The persistent constraint system.
    pub sys: remus_sketch::GcsSystem,
    /// JS handle → point id.
    pub points: Vec<remus_sketch::PointId>,
    /// JS handle → line id.
    pub lines: Vec<remus_sketch::LineId>,
    /// JS handle → circle id.
    pub circles: Vec<remus_sketch::CircleId>,
    /// JS handle → arc id.
    pub arcs: Vec<remus_sketch::ArcId>,
    /// JS handle → constraint id.
    pub constraints: Vec<remus_sketch::ConstraintId>,
}

/// Internal state for an in-progress sketch.
///
/// Stores points and constraints for the legacy index-based JS API.
/// A `GcsSystem` is created on-the-fly during `sketch_solve`.
#[derive(Default, Clone)]
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
