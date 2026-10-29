//! Instruction-count workload for `executeBatch` read-only failures.
//!
//! Drills a 64-hole plate through batches, then runs one measured batch of
//! read-only queries on it, half of them against a stale solid handle. Run
//! `setup` and `full` under cachegrind; their difference is the measured
//! batch. The printed digest covers every query result.
//!
//! ```text
//! cargo rustc --profile profiling -p remus-wasm --example readonly_batch_ir \
//!     -- -C link-arg=-fuse-ld=bfd
//! valgrind --tool=cachegrind --cache-sim=no \
//!     target/profiling/examples/readonly_batch_ir full
//! ```
#![allow(clippy::print_stdout, clippy::expect_used, clippy::unwrap_used)]

use remus_wasm::kernel::RemusKernel;

fn ok_u32(result: &str) -> u32 {
    let value: serde_json::Value = serde_json::from_str(result).unwrap();
    let ok = value[0]["ok"].as_u64().expect("batch op succeeds");
    u32::try_from(ok).unwrap()
}

fn drilled_plate(kernel: &mut RemusKernel) -> u32 {
    let mut plate = ok_u32(
        &kernel.execute_batch(r#"[{"op":"makeBox","args":{"width":100,"height":100,"depth":10}}]"#),
    );
    for i in 0..64_u32 {
        let x = 100.0 / 9.0 * f64::from(i % 8 + 1);
        let y = 100.0 / 9.0 * f64::from(i / 8 + 1);
        let tool = ok_u32(
            &kernel.execute_batch(r#"[{"op":"makeCylinder","args":{"radius":2,"height":20}}]"#),
        );
        kernel.execute_batch(&format!(
            r#"[{{"op":"transform","args":{{"solid":{tool},"matrix":[1,0,0,{x},0,1,0,{y},0,0,1,-5,0,0,0,1]}}}}]"#
        ));
        plate = ok_u32(&kernel.execute_batch(&format!(
            r#"[{{"op":"cut","args":{{"solidA":{plate},"solidB":{tool}}}}}]"#
        )));
    }
    plate
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_default();
    let mut kernel = RemusKernel::new();
    let plate = drilled_plate(&mut kernel);
    if mode != "full" {
        println!("setup plate={plate}");
        return;
    }
    let stale = plate + 100_000;
    let ops: Vec<String> = (0..16)
        .flat_map(|_| {
            [
                format!(r#"{{"op":"getSolidFaces","args":{{"solid":{plate}}}}}"#),
                format!(r#"{{"op":"getSolidFaces","args":{{"solid":{stale}}}}}"#),
            ]
        })
        .collect();
    let results = kernel.execute_batch(&format!("[{}]", ops.join(",")));
    let mut digest: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in results.bytes() {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x0100_0000_01b3);
    }
    let after = kernel.execute_batch(&format!(
        r#"[{{"op":"getSolidFaces","args":{{"solid":{plate}}}}}]"#
    ));
    for byte in after.bytes() {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x0100_0000_01b3);
    }
    println!(
        "full plate={plate} results={} digest={digest:016x}",
        results.len()
    );
}
