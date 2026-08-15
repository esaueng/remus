//! Wall-clock profile of `executeBatch` and the topology-transaction path.
//!
//! Isolates the per-operation rollback snapshot cost: every scenario keeps the
//! geometry work per operation small and constant, so the growth term is the
//! snapshot, not the modelling.
//!
//! ```text
//! cargo run --release -p brepkit-wasm --example batch_profile
//! cargo run --release -p brepkit-wasm --example batch_profile -- 15
//! ```
#![allow(clippy::print_stdout, clippy::expect_used, clippy::unwrap_used)]

use std::time::Instant;

use brepkit_wasm::kernel::BrepKernel;

/// Seeds a kernel with `boxes` solids so the arenas are non-trivial before the
/// measured batch runs.
fn seeded_kernel(boxes: usize) -> BrepKernel {
    let mut kernel = BrepKernel::new();
    let ops: Vec<String> = (0..boxes)
        .map(|i| {
            let s = 1.0 + (i % 7) as f64 * 0.1;
            format!(r#"{{"op":"makeBox","args":{{"width":{s},"height":{s},"depth":{s}}}}}"#)
        })
        .collect();
    kernel.execute_batch(&format!("[{}]", ops.join(",")));
    kernel
}

fn translation_matrix(dx: f64) -> String {
    format!("[1,0,0,{dx},0,1,0,0,0,0,1,0,0,0,0,1]")
}

/// Array / assembly workflow: copy a solid, move it, measure it.
fn assembly_batch(base: u32, triples: usize) -> String {
    let mut ops = Vec::with_capacity(triples * 3);
    for i in 0..triples {
        ops.push(format!(r#"{{"op":"copySolid","args":{{"solid":{base}}}}}"#));
        ops.push(format!(
            r#"{{"op":"transform","args":{{"solid":{base},"matrix":{}}}}}"#,
            translation_matrix(i as f64 * 0.01)
        ));
        ops.push(format!(
            r#"{{"op":"boundingBox","args":{{"solid":{base}}}}}"#
        ));
    }
    format!("[{}]", ops.join(","))
}

/// Pure read-only batch: no operation mutates topology at all.
fn query_batch(base: u32, count: usize) -> String {
    let ops: Vec<String> = (0..count)
        .map(|i| {
            if i % 2 == 0 {
                format!(r#"{{"op":"boundingBox","args":{{"solid":{base}}}}}"#)
            } else {
                format!(r#"{{"op":"volume","args":{{"solid":{base},"deflection":0.5}}}}"#)
            }
        })
        .collect();
    format!("[{}]", ops.join(","))
}

/// Pure additive batch: every operation mutates.
fn build_batch(count: usize) -> String {
    let ops: Vec<String> = (0..count)
        .map(|i| {
            let s = 1.0 + (i % 5) as f64 * 0.1;
            format!(r#"{{"op":"makeBox","args":{{"width":{s},"height":{s},"depth":{s}}}}}"#)
        })
        .collect();
    format!("[{}]", ops.join(","))
}

/// Reports the *minimum* over repetitions.
///
/// Contention and allocator noise only ever add time, so the minimum is the
/// robust estimator here; means on a loaded machine vary by more than 10x.
fn report(label: &str, seed: usize, ops: usize, samples: &[std::time::Duration]) {
    let best = samples.iter().min().copied().unwrap_or_default();
    let per_op = best.as_secs_f64() * 1e6 / ops as f64;
    println!(
        "{label:<28} seed={seed:<5} ops={ops:<5} best={best:>10.3?}  per-op={per_op:>9.1}us  (min of {})",
        samples.len()
    );
}

/// Runs `body` `reps` times on a freshly seeded kernel, returning each timing.
fn sample<F>(seed: usize, reps: usize, mut body: F) -> Vec<std::time::Duration>
where
    F: FnMut(&mut BrepKernel) -> std::time::Duration,
{
    (0..reps)
        .map(|_| {
            let mut kernel = seeded_kernel(seed);
            body(&mut kernel)
        })
        .collect()
}

/// Times one `executeBatch` call, asserting every operation succeeded.
fn time_batch(kernel: &mut BrepKernel, batch: &str, label: &str) -> std::time::Duration {
    let start = Instant::now();
    let out = kernel.execute_batch(batch);
    let elapsed = start.elapsed();
    assert!(!out.contains("\"error\""), "{label} batch failed: {out}");
    elapsed
}

/// `transformSolid` goes through `with_topology_transaction`; called per
/// instance in array workflows.
fn transaction_loop(kernel: &mut BrepKernel, base: u32, calls: usize) -> std::time::Duration {
    let matrix: Vec<f64> = vec![
        1.0, 0.0, 0.0, 0.001, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    let start = Instant::now();
    for _ in 0..calls {
        let _ = kernel.transform_solid_binding(base, matrix.clone());
    }
    start.elapsed()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let reps: usize = args.first().and_then(|s| s.parse().ok()).unwrap_or(9);
    let count = 150;

    for &seed in &[50usize, 200, 400] {
        let batch = assembly_batch(0, count / 3);
        let s = sample(seed, reps, |k| time_batch(k, &batch, "assembly"));
        report("assembly(copy+xform+bbox)", seed, (count / 3) * 3, &s);

        let batch = query_batch(0, count);
        let s = sample(seed, reps, |k| time_batch(k, &batch, "query"));
        report("query(bbox+volume)", seed, count, &s);

        let batch = build_batch(count);
        let s = sample(seed, reps, |k| time_batch(k, &batch, "build"));
        report("build(makeBox)", seed, count, &s);

        let s = sample(seed, reps, |k| transaction_loop(k, 0, count));
        report("transaction(transformSolid)", seed, count, &s);
    }
}
