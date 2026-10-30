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

## Explicit WASM ownership boundary

Tier: core. The additive `RemusKernel.copyDocument(solids, sheets, wires,
maxEntities?)` binding applies the same selected-document copy inside the
module. `maxEntities` may lower the default entity/reference limit. It returns
a `KernelDocumentCopy` with ordered `solids`, `sheets` and `wires` getters and
a single-use `takeKernel()` method. No compound roots or arbitrary subentity
remaps are exposed. The operation is direct-only; JSON batches cannot return
ownership of a new kernel object.

```js
const transfer = source.copyDocument([body], [], []);
const roots = Array.from(transfer.solids);
const next = transfer.takeKernel();
transfer.free(); // next remains independently owned
// Use roots only with next; source handles and checkpoints stay with source.
try {
  const volume = next.volume(roots[0], 0.02);
  // Rebuild any application selections and preparation in the new owner.
} finally {
  next.free();
}
```

The old kernel retains its complete session until explicitly freed. The new
one contains only the selected document and lineage, with no checkpoint,
assembly, sketch, GCS or prepared-move state. Old numeric handles must stay
with the old owner; equal numbers across owners do not imply identity, and
the existing u32 API cannot detect foreign handles. A poisoned native kernel
refuses copying. Dropping an untaken transfer drops its copied topology.
Module-wide bounded content caches retain their previous configuration and
lifetime; copying does not reset or re-enable them.

The packaged contract in `scripts/document-copy-packaged.mjs` exercises
three physical scales, mixed roots and aliases, exact Hammer Holder document
bytes, refusal/state preservation, once-only transfer, old checkpoint
restoration and repeated owner replacement. Its allocate/restore loop is a
lifecycle diagnostic, not a face-edit or presentation benchmark. Freeing an
old kernel releases its owned payload to the module allocator; linear memory
need not shrink, and both owners coexist during copying. Browser worker
adoption, undo migration and whole-session reclamation remain open.

The [WASM session observations](evidence/wasm-document-copy/session.json)
record 20 copy cycles after 25 allocation/restore ticks each: 1,728 accounted
slots before a copy (including the retained checkpoint), 64 slots and 7,824
estimated current-topology bytes afterwards on every cycle. Module linear
memory stayed at 4,259,840 bytes during this diagnostic. This is an observed
allocator high-water plateau for the box lifecycle; it is not a portable
memory guarantee, a shrink result, or an imported face-edit workload. The
record includes exact candidate package, driver and runtime source hashes.
