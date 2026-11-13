//! 3MF archive parsing, then the `import_mesh` step every 3MF import ends in.
//!
//! The input is the whole ZIP, so it mostly exercises the archive layer: CRC
//! and deflate checks reject almost every mutation before the model XML is
//! read. `threemf_xml` fuzzes the XML parser behind a valid archive.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    if let Ok(meshes) = remus_io::threemf::read_threemf_with_limits(data, common::limits()) {
        common::import_meshes(&meshes);
    }
});
