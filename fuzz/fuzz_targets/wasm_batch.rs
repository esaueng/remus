#![no_main]

use libfuzzer_sys::fuzz_target;
use remus_wasm::kernel::BrepKernel;

fuzz_target!(|data: &[u8]| {
    if let Ok(input) = std::str::from_utf8(data) {
        let _legacy = BrepKernel::new().execute_batch(input);
        let _v2 = BrepKernel::new().execute_batch_v2(input);
    }
});
