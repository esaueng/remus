//! GLB parsing, then the `import_mesh` step every GLB import ends in.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    if let Ok(mesh) = remus_io::gltf::read_glb_with_limits(data, common::limits()) {
        common::import_meshes(&[mesh]);
    }
});
