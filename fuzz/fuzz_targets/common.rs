use brepkit_io::ImportLimits;

pub fn limits() -> ImportLimits {
    ImportLimits {
        max_input_bytes: 1024 * 1024,
        max_archive_entry_bytes: 2 * 1024 * 1024,
        max_model_entities: 10_000,
    }
}
