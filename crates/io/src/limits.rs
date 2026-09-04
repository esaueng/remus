//! Resource limits for untrusted model imports.

use crate::IoError;

/// Production defaults used by all importer entry points.
///
/// Limits are measured before large allocations whenever the format exposes a
/// declared count. `max_archive_entry_bytes` is separate from compressed input
/// size so ZIP-based 3MF files cannot expand without bound.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_field_names)]
pub struct ImportLimits {
    /// Maximum encoded file size accepted by an importer (256 MiB by default).
    pub max_input_bytes: usize,
    /// Maximum uncompressed 3MF model XML entry (256 MiB by default).
    pub max_archive_entry_bytes: usize,
    /// Maximum parsed model records, vertices, faces, or triangles.
    ///
    /// Importers apply this limit to the format-specific entity counts that
    /// drive allocation and work. Default: 3,000,000.
    pub max_model_entities: usize,
}

impl Default for ImportLimits {
    fn default() -> Self {
        Self {
            max_input_bytes: 256 * 1024 * 1024,
            max_archive_entry_bytes: 256 * 1024 * 1024,
            max_model_entities: 3_000_000,
        }
    }
}

pub(crate) fn ensure_limit(
    resource: &'static str,
    actual: usize,
    limit: usize,
) -> Result<(), IoError> {
    if actual > limit {
        return Err(IoError::LimitExceeded {
            resource,
            limit,
            actual,
        });
    }
    Ok(())
}

pub(crate) fn ensure_input_size(data_len: usize, limits: ImportLimits) -> Result<(), IoError> {
    ensure_limit("input bytes", data_len, limits.max_input_bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_accept_writer_scale_faceted_step_files() {
        let limits = ImportLimits::default();
        assert_eq!(limits.max_input_bytes, 256 * 1024 * 1024);
        assert_eq!(limits.max_model_entities, 3_000_000);
    }

    #[test]
    fn input_limit_reports_resource_and_values() {
        let limits = ImportLimits {
            max_input_bytes: 3,
            ..ImportLimits::default()
        };
        assert!(matches!(
            ensure_input_size(4, limits),
            Err(IoError::LimitExceeded {
                resource: "input bytes",
                limit: 3,
                actual: 4
            })
        ));
    }
}
