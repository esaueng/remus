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
    /// Maximum encoded file size accepted by an importer (128 MiB by default).
    pub max_input_bytes: usize,
    /// Maximum uncompressed 3MF model XML entry (256 MiB by default).
    pub max_archive_entry_bytes: usize,
    /// Maximum parsed model records, vertices, faces, or triangles.
    ///
    /// Importers apply this limit to the format-specific entity counts that
    /// drive allocation and work. Default: 2,000,000.
    pub max_model_entities: usize,
}

impl Default for ImportLimits {
    fn default() -> Self {
        Self {
            max_input_bytes: 128 * 1024 * 1024,
            max_archive_entry_bytes: 256 * 1024 * 1024,
            max_model_entities: 2_000_000,
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
    fn input_limit_reports_resource_and_values() {
        let limits = ImportLimits {
            max_input_bytes: 3,
            ..ImportLimits::default()
        };
        let err = ensure_input_size(4, limits).unwrap_err();
        assert!(matches!(
            err,
            IoError::LimitExceeded {
                resource: "input bytes",
                limit: 3,
                actual: 4
            }
        ));
    }
}
