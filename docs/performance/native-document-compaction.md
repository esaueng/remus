# Native document copy compaction

Tier: native. Owner: PERF-T06 / P-Class 8.6. This is the first additive
document-copy slice of the [E6b design](../design/deferred-e6b-arena-compaction-and-slot-reuse.md).
It does not implement whole-WASM-session reclamation or slot reuse.

`remus_io::arena_io::compact_document` copies explicitly selected solid, sheet,
wire and compound roots into a fresh dense topology and returns their new
roots. Source handles and snapshots stay with their original owner. Selected
geometry, per-use boundary authority, attributes and lineage use checked arena
replay without intermediate JSON. The source must be dropped, together with
any snapshots that retain it, before its memory can be reclaimed.

## Session diagnostic

`crates/io/examples/compact_session.rs` repeatedly allocates a disposable box
and restores one retained box. It compares the selected box's exact serialized
document after every cycle. With `--compact`, it replaces the topology with a
fresh document copy after each 100-tick cycle. Without it, retirement tombstones
remain in the original topology. This isolates lifecycle scaling; it does not
model actual face edits, NURBS editing, rendering or browser presentation.

Build and run separate processes:

```sh
cargo build --profile ci-test -p remus-io --example compact_session
target/ci-test/examples/compact_session --cycles 20 --ticks 100
target/ci-test/examples/compact_session --cycles 20 --ticks 100 --compact
target/ci-test/examples/compact_session --cycles 40 --ticks 100 --compact
target/ci-test/examples/compact_session --cycles 80 --ticks 100 --compact
```

The maintained raw observations are in
[`evidence/native-compaction/session.jsonl`](evidence/native-compaction/session.jsonl).
They were collected on Linux x86_64, AMD Ryzen 9 5900XT, Rust 1.96.0,
`ci-test` profile (opt-level 1, assertions enabled), during concurrent agent
builds. Three alternating process pairs cover 20 cycles; 40/80-cycle rows are
single-process scaling observations. This is not a quiet latency qualification
and supports no user-visible speedup or tail-latency claim.

All samples preserve the same 4,275-byte selected document. Every no-copy
20-cycle process retains 128,064 slots; every copy process retains 64 slots,
with a peak of 6,464 before each explicit copy. The 40/80-cycle copy runs have
the same retained and peak slot counts. Linux final resident-set observations
are reported in the raw rows; they are process/allocator observations, not
portable memory guarantees or WASM linear-memory shrinkage claims.

## Qualification and remaining scope

The integration suite compares full exact documents on the 160-face Hammer
Holder and a multi-cavity model, checks mixed roots and shared ownership,
resolves a selected journal reference in both owners, and rejects stale roots,
invalid carriers/body classes and excessive reference lists without changing
the source. The session test keeps the old snapshot usable and proves later
source allocations do not reissue retired source handles.

The API returns only root remaps. A caller that retains arbitrary subentity
handles needs a separate migration strategy. Assemblies, comp-solids, sketches,
checkpoints, prepared moves, per-session caches and browser worker ownership are
outside this slice. Neither process RSS nor total live memory is capped by the
entity/reference budget. Old and new topologies coexist during the copy.
