# PERF-T01: shared unchanged-state savepoints

## Contract (written before implementation)

Every transaction is a savepoint at its own entry state. Nested success commits
into the enclosing transaction. A propagated failure restores each failed scope
in turn. A caught inner failure restores the inner entry state, preserving prior
outer work; the outer scope may then succeed. Outer failure after inner success
restores the outer entry state. A validation veto is a failure at that scope.
Batch items commit independently, including successful items after a refused item.

Rollback covers live topology contents and liveness, authoritative loops/coedges,
pcurves and their index, attributes, journals, naming ordinals and mutation ticks.
Allocation and journal identity high-water marks are deliberately NOT rewound.
Pre-existing handles survive failed retirement; failed allocations never resolve
again, including after later allocations or checkpoint restores. Checkpoint
restore remains a distinct externally observable retirement policy.

WASM topology-only operations do not mutate assemblies, sketches, GCS sketches,
checkpoints or poison state. The selected boolean integration must preserve those
session fields. This slice does not broaden the set of batch operations.

## Bounded design

A topology-local coordinator holds only a weak reference to the current immutable
savepoint. Each active scope owns a strong reference. An inner scope can share
that snapshot only if no mutable access has occurred since capture. Every mutable
topology entry point invalidates the weak reference BEFORE exposing or changing
state, including attribute-only and journal-only changes, restoration and mutable
entity access. Conservative invalidation is allowed; missed invalidation is not.
Independent topology clones start with an empty coordinator. There is no global
cache, persistent snapshot retention or transaction-depth shortcut. Genuine
savepoints after mutations still deep-copy the entire topology.

The existing result-based contract does not promise rollback on panic/unwind.
Snapshot ownership must nevertheless release normally on unwind. Failed operations
use `restore_for_rollback`; checkpoint restore retains its separate policy.

## Qualification plan

Trace native `boolean` and direct/batch WASM `fuse` to GFA and export. Measure
unchanged nesting depths and increasing unrelated box counts, including retained
checkpoints. Compare baseline and candidate with identical fixture/harness sources.
Record snapshot count, clone allocation bytes, operation latency, live allocation
peak/RSS and WASM linear-memory high-water separately. Instrumented measurements
are not production timings; serialize size is not a clone-byte measurement.

Inject failures after allocation, in-place mutation, retirement, pcurve/attribute
changes and journal recording. Compare complete logical state to the original
full-snapshot semantics, excluding only intentionally monotonic identity state.
Exercise nested success, propagated/caught failure, outer failure, validation veto,
repeated failure and per-item partial commits through packaged WASM too.

## Remaining work and prerequisites

The outer snapshot and genuine changed-state savepoints remain O(document size).
PERF-T02 needs an audit of all write accessors plus undo versus chunk-COW evidence.
PERF-T03 needs a guarded append-only write set and fallback. PERF-T04 needs immutable
carrier ownership and detachment contracts. PERF-W04 needs checkpoint retention
and first-write measurements. Arena compaction/ID reuse remain P-Class 8.6 and
require versioned handle indirection, stale-handle and serialization migration
contracts. Global caches require O3.2 mutation/invalidation contracts. None of
those mechanisms is introduced here.

## Public path and mutation audit

The fixed fixtures fuse two unit boxes: the fast-path tool is translated by
0.5 along X (union volume 1.5); the GFA tool is rotated 45 degrees about its
center around Z (union volume `4 - 2*sqrt(2)`). Unrelated unit boxes are stored
in the same document but are not boolean operands. Their geometry is not scanned
by the measured operation; their arena data is copied by document snapshots.

Native `operations::boolean` opens the operations transaction, then
`boolean_with_operation_context` calls `boolean_with_context_impl`. The axis-aligned
fixture returns from the exact box fast path. The rotated fixture reaches
`algo::gfa::boolean_with_context`, which opens another transaction before building
an operand-local `GfaShapeStore`. Pave filler and builder mutate that isolated
store; export allocates in the caller topology, invalidating snapshot sharing.
The operations wrapper normalizes winding before committing. Native facade
`Model::fuse` delegates through `Model::boolean` to `boolean_with_context`, whose
transaction has the same boundary; it adds no facade snapshot.

Direct WASM `BrepKernel::fuse` resolves handles, calls `topo_mut`, then the same
native `boolean`. Batch `executeBatch` processes each item independently through
`dispatch_with_rollback`; its `fuse` arm calls the native operation directly.
Thus uncheckpointed baseline clone counts are 1/2 for native/direct WASM
fast/GFA, and 2/3 for batch fast/GFA. A retained checkpoint adds an independent
`Rc::make_mut` document copy at the first write. It remains necessary here.
Synthetic native enclosing transactions measure unchanged entry nesting at
0/1/4/8 additional scopes; direct versus batch measures real WASM wrapper depth.

Sharing is invalidated at both arena API macros (allocation and mutable lookup),
face allocation, body-class setters, both restore policies, reservation,
attribute setters/propagation, journal begin/record/load, boundary installation,
wire/face boundary replacement, loop derivation, deletion, empty-solid creation,
and every pcurve/periodic-winding setter/remover. Private boundary installers also
invalidate conservatively. `mutation_ticks` retains its journal meaning and is
not used as a snapshot revision. Future mutable accessors must invalidate sharing
before writing or returning an exclusive reference.

The host integration is deliberately only batch `fuse`, `cut`, and `intersect`.
Other batch arms and the unrelated `with_topology_transaction` helper keep their
existing policy; this slice makes no new failure-atomicity claim for them.
