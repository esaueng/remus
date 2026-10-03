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

impl ImportLimits {
    /// Restrict the production defaults using optional numeric API budgets.
    ///
    /// # Errors
    ///
    /// Rejects non-finite, fractional, non-positive, or above-default counts
    /// before converting them to platform-sized integers. Native callers can
    /// still construct explicit limits for workloads outside these defaults.
    pub fn with_restricted_overrides(
        max_input_bytes: Option<f64>,
        max_model_entities: Option<f64>,
    ) -> Result<Self, IoError> {
        fn count(value: Option<f64>, name: &str, maximum: usize) -> Result<usize, IoError> {
            let Some(value) = value else {
                return Ok(maximum);
            };
            if !value.is_finite() || value < 1.0 || value.fract() != 0.0 || value > maximum as f64 {
                return Err(IoError::ParseError {
                    reason: format!(
                        "{name} must be an integer between 1 and {maximum}, got {value}"
                    ),
                });
            }
            Ok(value as usize)
        }

        let defaults = Self::default();
        Ok(Self {
            max_input_bytes: count(max_input_bytes, "maxInputBytes", defaults.max_input_bytes)?,
            max_model_entities: count(
                max_model_entities,
                "maxEntities",
                defaults.max_model_entities,
            )?,
            ..defaults
        })
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
    #![allow(clippy::expect_used)]

    use super::*;

    #[test]
    fn numeric_overrides_only_tighten_defaults_without_truncation_or_saturation() {
        let defaults = ImportLimits::default();
        assert_eq!(
            ImportLimits::with_restricted_overrides(None, None).ok(),
            Some(defaults)
        );
        for bad in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            -1.0,
            0.0,
            0.5,
            1.5,
            1e30,
            f64::MAX,
        ] {
            assert!(ImportLimits::with_restricted_overrides(Some(bad), None).is_err());
            assert!(ImportLimits::with_restricted_overrides(None, Some(bad)).is_err());
        }
        assert!(
            ImportLimits::with_restricted_overrides(
                Some(defaults.max_input_bytes as f64 + 1.0),
                None
            )
            .is_err()
        );
        assert!(
            ImportLimits::with_restricted_overrides(
                None,
                Some(defaults.max_model_entities as f64 + 1.0)
            )
            .is_err()
        );
        let restricted =
            ImportLimits::with_restricted_overrides(Some(1.0), Some(2.0)).expect("valid counts");
        assert_eq!(restricted.max_input_bytes, 1);
        assert_eq!(restricted.max_model_entities, 2);
        assert_eq!(
            restricted.max_archive_entry_bytes,
            defaults.max_archive_entry_bytes
        );
        assert_eq!(
            ImportLimits::with_restricted_overrides(
                Some(defaults.max_input_bytes as f64),
                Some(defaults.max_model_entities as f64)
            )
            .ok(),
            Some(defaults)
        );
    }

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
