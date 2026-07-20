#![no_main]

use brepkit_topology::Topology;
use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    if let Ok(input) = std::str::from_utf8(data) {
        let mut topo = Topology::new();
        let _ = brepkit_io::step::read_step_with_limits(input, &mut topo, common::limits());
    }
});
