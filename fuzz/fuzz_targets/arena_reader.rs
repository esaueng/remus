#![no_main]

use remus_topology::Topology;
use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    let mut topo = Topology::new();
    let _ = remus_io::arena_io::deserialize_document_with_limits(
        data,
        &mut topo,
        common::limits(),
    );
});
