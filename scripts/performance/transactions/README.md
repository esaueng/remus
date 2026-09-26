# PERF-T01 reproduction

Run `instrument.py ROOT` from this revision against two disposable source
archives: baseline `a3d85e799196315522ac2f08138ae6fe6daa27c2` and the candidate source
revision named in the evidence report. Do not run it in a working checkout you
intend to commit. It deliberately inserts diagnostic-only allocation counters,
full-state readers and one-shot failure hooks; none is in the shipped kernel.
It assumes the two exact boolean mutation boundary statements still exist and
refuses if their occurrence count changes. Instrumentation is single-threaded.

For each archive, build the two native examples and the actual wasm32 module:

```sh
cargo build --release --manifest-path "$ROOT/Cargo.toml" -p remus-wasm \
  --example perf-t01 --example perf-t01-fault
RUSTFLAGS='-C target-feature=+simd128' cargo build --release \
  --manifest-path "$ROOT/Cargo.toml" -p remus-wasm \
  --target wasm32-unknown-unknown --no-default-features
wasm-bindgen "$ROOT/target/wasm32-unknown-unknown/release/remus_wasm.wasm" \
  --target nodejs --out-dir "$ROOT/pkg-probe"
wasm-opt -Oz --enable-simd "$ROOT/pkg-probe/remus_wasm_bg.wasm" \
  -o "$ROOT/pkg-probe/remus_wasm_bg.wasm"
python3 scripts/performance/transactions/run.py "$BASELINE_ROOT" "$CANDIDATE_ROOT" "$OUTPUT"
```

Use pinned Rust 1.96.0 and wasm-bindgen 0.2.126. The wasm-opt version, source SHA,
source/patch/harness hashes and module hashes belong in the run manifest. These
commands use the same release profile, SIMD and wasm-opt flags as the package
build. The normal candidate packages must additionally be rebuilt with
`cargo xtask wasm-build`, which runs installed-tarball consumer tests.

`run.py` alternates baseline/candidate order over seven fresh processes per cell.
Setup, validation, serialization and teardown are outside the operation timer.
These are cold-operation samples with a populated allocator/arena; they are not
browser frame timings or a steady-state JIT benchmark. Small-sample percentiles
are not presented as reliable tail latency. The count/depth/byte invariants are
more robust than small latency differences on a shared machine.

Metrics are `[topology_clone_count, clone_allocation_bytes, live_bytes, peak_live_bytes]`.
The byte counter records requested heap allocation during `Topology::clone`,
including nested payloads and map storage. It is **not** serialized size, exact
CPU memcpy traffic, allocator metadata or resident memory. Peak live heap covers
the operation including its input document. WASM also reports process peak RSS
(setup and V8 included) and reserved linear memory (never claimed as live heap).
The small Arc control allocation is included in total live/peak, not clone bytes.

Faults are injected after geometry/export and after winding normalization in
the real public operation. Tests compare all live arenas, loops/coedges, pcurve
lookups, attributes and journal entries/ordinal index, plus WASM session fields.
Only intentionally monotonic handle/journal high-water marks are excluded from
state equality; separate checks require failed handles to remain stale. The
committed native unit matrix additionally compares the original full-snapshot
implementation for caught/propagated failures, outer rollback and validation veto.

Probe-only `unsafe` is confined to `allocator.rs`, a forwarding `System` allocator
inserted into disposable measurement sources. Production code remains safe Rust;
there is no new runtime feature, dependency, fault switch or JS export.

For uninstrumented timing, start from fresh source archives and run
`instrument.py ROOT --production`: this writes only the native workload example
(no allocator, fault injection or library changes). Build that one example with
the same release profile. Use `production.py` with the two native executable paths,
the two installed Node package entry paths, and an output JSONL filename. The
baseline may be a private Node-only tarball of the rebuilt wasm-bindgen output;
the candidate is the actual package produced by `cargo xtask wasm-build`.
`npm pack` and local tarball installation are sufficient; do not publish.
The runner observes linear memory from the constructed WebAssembly instance
without editing package bindings. RSS and reserved linear memory include setup
and validation; allocator peak is read before validation.

Use `summarize.py samples.jsonl` for the grouped medians/ranges and clone/memory
counters. To verify runtime source provenance, independently archive the claimed
source commit, apply this instrumentation recipe to that reference archive, then
run `provenance.py DIAGNOSTIC_ROOT REFERENCE_ROOT OUTPUT.json`. It compares every
local source dependency named by Cargo for both the native binary and WASM module.
