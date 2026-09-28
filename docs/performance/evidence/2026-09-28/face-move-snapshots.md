# Shared entry snapshot for face moves: native evidence

The ordinary planar `operations::move_faces_with_entity_evolution` path captures
an entry snapshot before calling `offset::move_faces_with_entity_map`. No topology
mutation occurs between those captures. The candidate uses the existing
unchanged-entry savepoint coordinator for both calls, so the nested capture
shares one allocation. Each scope retains its own failure restore boundary and
the existing checkpoint-style retirement policy. Changed-state nested scopes
still deep-copy. The outer snapshot remains proportional to the entire arena.

Base source: `f69d05da37185b4fdfe4661fc10e4a975e0b452c` (`main`). Candidate
source diff across `crates/topology/src/transaction.rs`, its savepoint tests,
`crates/operations/src/push_pull.rs`, and `crates/offset/src/move_faces.rs` has
SHA-256 `c480a2265531d0ebda626db4d2701105e75bc43f8b9bcb359f32ef35b1942d9e`.
All native runs used Rust 1.96.0, Linux x86_64, AMD Ryzen 9 5900XT (32 logical
CPUs), `CARGO_BUILD_JOBS=2`, and the release profile. The unchanged tracked
`crates/operations/examples/body_edit_probe.rs` produced
[all raw output](face-move-native-runs.csv) with 5 warmups and 30 samples per
operation and arena size. Run command:

```sh
CARGO_BUILD_JOBS=2 cargo run --release -p remus-operations --example body_edit_probe
```

This fixture measures exact edit and subsequent tessellation separately after
creating 0, 1,000, or 10,000 unrelated boxes. It checks result and source
volumes, strict result validation, and nonempty mesh output on every sample.
It does not retain WASM checkpoints or measure OpenZCAD replay, worker transfer,
or viewport presentation.

The first candidate run reduced `moveFaces` median at 1,000 unrelated boxes
from 5.963 to 1.164 ms, but at 10,000 it was 40.605 to 40.358 ms. A second
candidate run reported 3.523 and 48.298 ms. In that repeat, unchanged `extrude`
at 1,000 boxes rose from 0.964 to 3.308 ms and unchanged `pushPullFace` at
10,000 rose from 68.701 to 154.762 ms. Host contention or memory pressure
therefore confounds an end-to-end speedup estimate. The source change and
savepoint test establish one fewer full clone in the unchanged nested entry,
but these operation timings do not establish a stable latency gain for a
large document.

`cargo test --release -p remus-topology --lib transaction::` passed 14 existing
tests. The new shared-snapshot checkpoint-policy oracle passed. The five tests
in `cargo test --release -p remus-operations --test qualify_move_faces_generalized`
passed, including journal refusal, persistent-reference binding, blend moves,
scale/translation, and bore replacement. `cargo fmt --all --check`, the crate
boundary script, and `git diff --check` passed. This is native-only evidence;
paired generated WASM packages and an installed OpenZCAD consumer must be
rebuilt and checked from the final source before claiming browser performance.
