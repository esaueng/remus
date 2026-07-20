#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;

fuzz_target!(|data: &[u8]| {
    let _ = brepkit_io::threemf::read_threemf_with_limits(data, common::limits());
});
