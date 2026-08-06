#![no_main]

use brepkit_topology::Topology;
use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    let mut topo = Topology::new();
    let _ = brepkit_io::arena_io::deserialize_document_with_limits(
        data,
        &mut topo,
        common::limits(),
    );
});
