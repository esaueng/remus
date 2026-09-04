//! # remus-io
//!
//! Data exchange for remus: STEP, IGES, 3MF, STL, OBJ, PLY, and glTF import/export.
//!
//! This is layer L3, depending on `remus-math`, `remus-topology`,
//! and `remus-operations`.
//!
//! The format translators sit behind the default `formats` feature. Without
//! it the crate is only the exact arena document codec ([`arena_io`]) and
//! the persistent-reference codec ([`naming_io`]), the two pieces the browser
//! kernel module needs to exchange bodies with the separate translator
//! module.

pub mod arena_io;
#[cfg(feature = "formats")]
pub mod gltf;
#[cfg(feature = "formats")]
pub mod iges;
pub mod limits;
pub mod naming_io;
#[cfg(feature = "formats")]
pub mod obj;
#[cfg(feature = "formats")]
pub mod ply;
#[cfg(feature = "formats")]
pub mod step;
#[cfg(feature = "formats")]
pub mod stl;
#[cfg(feature = "formats")]
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

    /// A requested STEP validation-property check found a malformed contract.
    #[error("invalid STEP validation properties ({code}): {reason}")]
    InvalidValidationProperties {
        /// Stable diagnostic code for the refusal.
        code: &'static str,
        /// Human-readable context; not a stable wire contract.
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
    Topology(#[from] remus_topology::TopologyError),

    /// An I/O error occurred.
    #[error(transparent)]
    Io(#[from] std::io::Error),

    /// An error from a modeling operation (e.g. tessellation).
    #[error(transparent)]
    Operations(#[from] remus_operations::OperationsError),

    /// An error writing the ZIP archive.
    #[cfg(feature = "formats")]
    #[error(transparent)]
    Zip(#[from] zip::result::ZipError),
}
