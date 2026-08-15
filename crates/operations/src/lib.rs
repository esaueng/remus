//! # brepkit-operations
//!
//! CAD modeling operations for B-Rep solids. Layer L3, depending on
//! `brepkit-math`, `brepkit-topology`, `brepkit-geometry`, `brepkit-algo`,
//! `brepkit-blend`, `brepkit-heal`, `brepkit-check`, and `brepkit-offset`.
//!
//! # Module families
//!
//! | Family | Modules | Purpose |
//! |--------|---------|---------|
//! | **Core** | [`primitives`], [`extrude`], [`revolve`], [`sweep`], [`loft`], [`pipe`], [`helix`] | Shape creation |
//! | **Transform** | [`transform`], [`copy`], [`mirror`], [`pattern`] | Spatial operations |
//! | **Boolean** | [`boolean`], [`mesh_boolean`] | Set operations |
//! | **Blend** | [`fillet`], [`chamfer`], [`blend_ops`], [`resize_blend`] | Edge smoothing and exact band editing |
//! | **Offset** | [`offset_face`], [`offset_trim`], [`offset_v2`], [`offset_wire`] | Wall thickness |
//! | **Direct edit** | [`push_pull`] | Move a face of an existing solid |
//! | **Surface** | [`fill_face`], [`thicken`], [`shell_op`], [`draft`], [`section`], [`split`] | Surface/solid modification |
//! | **Repair** | [`heal`], [`defeature`], [`sew`], [`untrim`] | Shape fixing |
//! | **Analysis** | [`measure`], [`distance`], [`classify`], [`validate`], [`query`], [`feature_recognition`] | Interrogation |
//! | **Tessellation** | [`tessellate`] | Mesh generation |
//! | **Infrastructure** | [`assembly`], [`compound_ops`], [`evolution`], [`sketch`] | Utilities |

use brepkit_math::vec::{Point3, Vec3};

pub mod extrude;
pub mod helix;
pub mod loft;
pub mod pipe;
pub mod primitives;
pub mod projection;
pub mod revolve;
pub mod sweep;

pub mod copy;
pub mod mirror;
pub mod pattern;
pub mod transform;

pub mod boolean;
pub mod mesh_boolean;

pub mod blend_ops;
pub mod chamfer;
pub mod fillet;
pub mod resize_blend;

pub mod offset_face;
pub mod offset_trim;
pub mod offset_v2;
pub mod offset_wire;
pub mod push_pull;

pub mod draft;
pub mod fill_face;
pub mod section;
pub mod shell_op;
pub mod split;
pub mod thicken;

pub mod defeature;
pub mod heal;
pub mod sew;
pub mod untrim;

pub mod classify;
pub mod distance;
pub mod feature_recognition;
pub mod measure;
pub mod query;
pub mod validate;

pub mod tessellate;

pub mod assembly;
pub(crate) mod cap;
pub mod compound_ops;
pub mod evolution;
pub mod sketch;
pub(crate) mod winding;

#[cfg(test)]
pub(crate) mod test_helpers;

/// Compute `n · p` treating a `Point3` as a direction vector.
///
/// Equivalent to the dot product `n.x*p.x + n.y*p.y + n.z*p.z`, used
/// for the plane equation `n · point = d`.
fn dot_normal_point(n: Vec3, p: Point3) -> f64 {
    n.dot(Vec3::new(p.x(), p.y(), p.z()))
}

/// Errors from modeling operations.
#[derive(Debug, thiserror::Error)]
pub enum OperationsError {
    /// The input shape is invalid for this operation.
    #[error("invalid input: {reason}")]
    InvalidInput {
        /// Description of what is wrong.
        reason: String,
    },

    /// The operation produced a non-manifold result.
    #[error("non-manifold result")]
    NonManifoldResult,

    /// The operation produced an empty result (no geometry).
    ///
    /// Boolean operations return this when the algebraic outcome is the
    /// empty set: `Cut(A, B)` when `A ⊆ B`, or any operation on
    /// pre-collapsed inputs. Distinguishable from [`InvalidInput`] so
    /// callers can apply empty-operand identity rules without
    /// string-matching the error message.
    ///
    /// [`InvalidInput`]: Self::InvalidInput
    #[error("empty result: {reason}")]
    EmptyResult {
        /// Description of the empty-result scenario.
        reason: String,
    },

    /// The input is well-formed but the operation has no exact construction
    /// for this configuration.
    ///
    /// Returned in place of an approximate, degraded or invalid result, so
    /// callers can distinguish "this cannot be built exactly" from "these
    /// arguments are wrong" ([`InvalidInput`]) without matching on message
    /// text. Retrying with the same inputs will not help; the caller must
    /// either change the model or accept an explicitly approximate route.
    ///
    /// [`InvalidInput`]: Self::InvalidInput
    #[error("{operation}: unsupported configuration: {reason}")]
    Unsupported {
        /// Name of the operation that declined.
        operation: &'static str,
        /// Why this configuration has no exact construction.
        reason: String,
    },

    /// A referenced topology entity was not found.
    #[error(transparent)]
    Topology(#[from] brepkit_topology::TopologyError),

    /// A math error occurred during the operation.
    #[error(transparent)]
    Math(#[from] brepkit_math::MathError),

    /// A GFA algorithm error occurred.
    #[error("algo: {0}")]
    Algo(#[from] brepkit_algo::error::AlgoError),

    /// A blend (fillet/chamfer v2) error occurred.
    #[error("blend: {0}")]
    Blend(#[from] brepkit_blend::BlendError),

    /// An exact blend-band resize was refused.
    #[error("resize blend: {0}")]
    ResizeBlend(#[from] resize_blend::ResizeBlendError),

    /// A check (classification/validation/distance) error occurred.
    #[error("check: {0}")]
    Check(#[from] brepkit_check::CheckError),

    /// A geometry conversion error occurred.
    #[error("geometry: {0}")]
    Geometry(#[from] brepkit_geometry::error::GeomError),

    /// A shape-healing operation failed.
    #[error("heal: {0}")]
    Heal(#[from] brepkit_heal::HealError),

    /// An operation completed only a subset of the requested items.
    #[error("{operation} produced a partial result: {succeeded} succeeded, {failed} failed")]
    PartialResult {
        /// Name of the operation.
        operation: &'static str,
        /// Number of requested items that succeeded.
        succeeded: usize,
        /// Number of requested items that failed.
        failed: usize,
    },
}
