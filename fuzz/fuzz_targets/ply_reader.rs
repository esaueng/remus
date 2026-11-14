//! PLY parsing, then the `import_mesh` step every PLY import ends in.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    if let Ok(mesh) = remus_io::ply::read_ply_with_limits(data, common::limits()) {
        common::import_meshes(&[mesh]);
    }
});
