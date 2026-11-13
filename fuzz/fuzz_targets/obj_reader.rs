//! OBJ parsing, then the `import_mesh` step every OBJ import ends in.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    if let Ok(input) = std::str::from_utf8(data)
        && let Ok(mesh) = remus_io::obj::read_obj_with_limits(input, common::limits())
    {
        common::import_meshes(&[mesh]);
    }
});
