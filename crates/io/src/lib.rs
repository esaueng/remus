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

#[cfg(feature = "formats")]
fn retain_nondegenerate_triangles(mesh: &mut remus_operations::tessellate::TriangleMesh) {
    let indices = std::mem::take(&mut mesh.indices);
    mesh.indices.reserve(indices.len());
    for triangle in indices.chunks_exact(3) {
        let a = mesh.positions[triangle[0] as usize];
        let b = mesh.positions[triangle[1] as usize];
        let c = mesh.positions[triangle[2] as usize];
        let area_squared = (b - a).cross(c - a).length_squared();
        if area_squared.is_finite() && area_squared > 0.0 {
            mesh.indices.extend_from_slice(triangle);
        }
    }
}

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

#[cfg(all(test, feature = "formats"))]
mod tests {
    use remus_math::vec::Point3;
    use remus_operations::tessellate::TriangleMesh;

    use super::retain_nondegenerate_triangles;

    #[test]
    fn export_mesh_filter_removes_only_exact_zero_area_triangles() {
        let mut mesh = TriangleMesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0e-15, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            normals: Vec::new(),
            indices: vec![0, 1, 2, 0, 0, 2, 0, 1, 4, 0, 1, 3],
        };

        retain_nondegenerate_triangles(&mut mesh);

        assert_eq!(mesh.indices, vec![0, 1, 2, 0, 1, 3]);
    }
}
