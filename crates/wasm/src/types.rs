//! Typed result structs for structured WASM returns.
//!
//! Types annotated with `Tsify` automatically generate TypeScript definitions
//! and can be serialized via `serde-wasm-bindgen` for zero-copy JS interop.

use tsify::Tsify;

/// Typed result for `tessellateSolidGrouped`.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GroupedMeshResult {
    pub positions: Vec<f64>,
    pub normals: Vec<f64>,
    pub indices: Vec<u32>,
    pub face_offsets: Vec<u32>,
}

/// Typed result for `tessellateSolidUV`.
#[derive(serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct UvMeshResult {
    pub positions: Vec<f64>,
    pub normals: Vec<f64>,
    pub indices: Vec<u32>,
    pub uvs: Vec<f64>,
}

/// Typed result for `boundingBox`.
#[derive(serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct BoundingBoxResult {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

/// Typed result for boolean operations with evolution tracking.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct EvolutionResult {
    pub solid: u32,
    pub generated: Vec<u32>,
    pub modified: Vec<u32>,
}

/// Typed result for `massProperties`.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct MassPropertiesResult {
    /// Solid volume (mass at unit density).
    pub volume: f64,
    /// Center of mass `[x, y, z]`.
    pub center_of_mass: Vec<f64>,
    /// Inertia tensor about the center of mass, global axes:
    /// `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` (unit density).
    pub inertia: Vec<f64>,
    /// Principal moments of inertia, ascending.
    pub principal_moments: Vec<f64>,
    /// Principal axes as three unit vectors, row-major
    /// `[x0, y0, z0, x1, y1, z1, x2, y2, z2]`, matching `principalMoments`.
    pub principal_axes: Vec<f64>,
}

/// Typed result for `meshQuality`.
#[derive(serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct MeshQualityResult {
    /// Edges used by exactly one triangle after position welding (0 for a
    /// watertight mesh).
    pub boundary_edges: u32,
    /// Edges used by more than two triangles after position welding.
    pub non_manifold_edges: u32,
    /// Euler characteristic `V - E + F` of the welded mesh (2 for a single
    /// closed genus-0 shell).
    pub euler_characteristic: i32,
    /// True when the welded mesh has no boundary and no non-manifold edges.
    pub is_watertight: bool,
}

/// Typed result for `sketchSolve`.
#[derive(serde::Serialize, Tsify)]
#[tsify(into_wasm_abi)]
pub struct SketchSolveResult {
    pub converged: bool,
    pub points: Vec<f64>,
    pub residual: f64,
}

/// Per-step entry in a `HealPipelineResult`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct HealStepResult {
    /// Operator name that ran.
    pub step: String,
    /// Number of individual repair actions taken.
    pub actions_taken: u32,
    /// At least one fix was applied.
    pub done: bool,
    /// At least one fix could not be applied.
    pub failed: bool,
}

/// Typed result for `fixShapeWithConfig`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct HealFixResult {
    /// Handle of the healed solid (may differ from the input).
    pub solid: u32,
    /// Number of individual repair actions taken.
    pub actions_taken: u32,
    /// At least one fix was applied.
    pub done: bool,
    /// At least one fix could not be applied.
    pub failed: bool,
}

/// Typed result for `runHealPipeline`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct HealPipelineResult {
    /// Handle of the healed solid (may differ from the input).
    pub solid: u32,
    /// One entry per executed step, in order.
    pub steps: Vec<HealStepResult>,
}

/// Typed result for `gcsSolve`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GcsSolveResult {
    /// Whether the solver converged within tolerance.
    pub converged: bool,
    /// Number of DogLeg iterations used.
    pub iterations: u32,
    /// Maximum absolute residual after solving.
    pub max_residual: f64,
}

/// Typed result for `gcsDof`.
#[derive(Debug, serde::Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
#[tsify(into_wasm_abi)]
pub struct GcsDofResult {
    /// Degrees of freedom remaining (under-constrained dimensions).
    pub dof: u32,
    /// Rank of the constraint Jacobian.
    pub rank: u32,
    /// Total solver parameters.
    pub num_params: u32,
    /// Total constraint equations.
    pub num_equations: u32,
}
