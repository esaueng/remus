//! # brepkit-io
//!
//! Data exchange for brepkit: STEP, IGES, 3MF, STL, OBJ, PLY, and glTF import/export.
//!
//! This is layer L3, depending on `brepkit-math`, `brepkit-topology`,
//! and `brepkit-operations`.

pub mod arena_io;
pub mod gltf;
pub mod iges;
pub mod limits;
pub mod obj;
pub mod ply;
pub mod step;
pub mod stl;
pub mod threemf;

pub use limits::ImportLimits;

/// Errors from data exchange operations.
#[derive(Debug, thiserror::Error)]
pub enum IoError {
    /// A configured import resource limit was exceeded.
    #[error("import limit exceeded for {resource}: {actual} > {limit}")]
    LimitExceeded {
        /// The bounded resource (for example, `input bytes` or `mesh entities`).
        resource: &'static str,
        /// Configured maximum value.
        limit: usize,
        /// Observed or declared value.
        actual: usize,
    },

    /// The input file format is invalid or malformed.
    #[error("parse error: {reason}")]
    ParseError {
        /// Description of the parse failure.
        reason: String,
    },

    /// An unsupported STEP entity was encountered.
    #[error("unsupported STEP entity: {entity}")]
    UnsupportedEntity {
        /// The entity type name.
        entity: String,
    },

    /// The topology is incomplete or inconsistent for export.
    #[error("invalid topology for export: {reason}")]
    InvalidTopology {
        /// Description of the topology issue.
        reason: String,
    },

    /// A topology lookup failed.
    #[error(transparent)]
    Topology(#[from] brepkit_topology::TopologyError),

    /// An I/O error occurred.
    #[error(transparent)]
    Io(#[from] std::io::Error),

    /// An error from a modeling operation (e.g. tessellation).
    #[error(transparent)]
    Operations(#[from] brepkit_operations::OperationsError),

    /// An error writing the ZIP archive.
    #[error(transparent)]
    Zip(#[from] zip::result::ZipError),
}
