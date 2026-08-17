#![no_main]

use remus_topology::Topology;
use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    if let Ok(input) = std::str::from_utf8(data) {
        let mut topo = Topology::new();
        let _ = remus_io::iges::read_iges_with_limits(input, &mut topo, common::limits());
    }
});
