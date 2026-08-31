# Operation contract

The common result, quality, fallback, and postcondition contract that every
public modeling operation converges on. Existing APIs keep their current
signatures; the detailed contract arrives through **additive** APIs
(`*_detailed` natively, versioned methods at the WASM boundary), never by
breaking existing callers.

## OperationResult

The common detailed result concept, `OperationResult<T>`:

| Field | Meaning |
| --- | --- |
| `value: T` | The operation's product (ids, measures, meshes). |
| `quality` | `Exact` \| `Approximate` \| `Repaired` — see below. |
| `diagnostics` | Structured warnings/notes with stable codes ([failure-taxonomy.md](failure-taxonomy.md)). |
| `evolution` | Topology evolution (vertex, edge, face events), or an explicit statement that it is unresolved/unavailable. |
| `tolerance_report` | Every tolerance consulted, every tolerance increased, maximum introduced deviation. |
| `fallback` | Whether a fallback path ran, which one, and under what budget. |
| `stats` | Operation statistics: iterations used, entities generated, budgets consumed. |

`quality` semantics:

- **Exact** — the result's geometry is the exact analytic/NURBS solution
  within the operation's declared tolerance model. No representation was
  degraded.
- **Approximate** — the result is correct topology whose geometry carries a
  reported approximation error within a caller-supplied budget (e.g. a
  marched intersection curve, a mesh-fallback boolean).
- **Repaired** — the result required healing to satisfy postconditions; the
  tolerance report and diagnostics fully disclose what was changed.

A result may not claim `Exact` if any participating geometry was silently
converted, refit, or tessellated.

## Fallback policy

Callers choose, per operation (through the future operation context; the
current default is the existing behavior of each API):

- **ExactOnly** — any path that would degrade quality fails with a typed
  error instead.
- **AllowApproximate(budget)** — approximation permitted within an explicit
  caller-supplied error budget; the result reports method, actual error
  bound, whether analytic surfaces were lost, and validation state.
- **ApproximateOnly** — skip exact attempts (for previews and bulk paths);
  results are still validated and still report their error.

Exact-to-approximate fallback is **never silent** regardless of policy: it is
visible in `quality` and `fallback` at minimum.

## Universal postconditions

Every public operation, on every input, in every build (native and WASM):

1. No panic, trap, NaN propagation, or unbounded loop.
2. Same input and same context produce deterministic output — across repeated
   runs, and across native/WASM.
3. A successful result's topology passes the appropriate validator for its
   body type.
4. Unsupported cases fail with stable typed errors, not generic strings.
5. Exact-to-mesh (or any representation-degrading) fallback is never silent.
6. Healing and tolerance increases are reported, never implied.
7. Result history (evolution) is complete or explicitly marked unresolved —
   an entity in no bucket is a contract violation, not an omission.
8. All operation budgets (iterations, subdivisions, generated topology,
   memory, time where applicable) are bounded and observable in `stats`.

These postconditions are what a **Qualified** capability cell asserts
([capability-matrix.md](capability-matrix.md)); tests that qualify a cell must
check them, not only the happy-path value.

## Transactional mutation

Every public mutating operation eventually runs through one transaction
contract (generalizing the existing checkpoint/restore machinery):

1. Begin staged mutation.
2. Build new topology.
3. Validate.
4. Commit atomically.
5. On any failure, roll back without exposing partial topology; previously
   valid topology and all existing handles remain unchanged.

The existing arena no-reuse invariant (stale handles never alias new
entities — see `docs/design/deferred-e6b-arena-compaction-and-slot-reuse.md`)
is part of this contract and is preserved by all rollback paths.

Implementation: `remus_topology::transaction` (`run_transacted`,
`run_validated`) is the standard implementation, promoted from the three
ad-hoc snapshot/restore copies that preceded it. Running through it today:
the v2 blend wrappers (fillet/chamfer), `resize_blend`, and the additive
`boolean_transacted` entry (transacted + L3-validated commit). The WASM
batch dispatcher implements the same contract with an `Rc`-sharing
read-only fast path. Remaining public mutating operations migrate
incrementally.

## Context

Operations receive an explicit operation context (introduced incrementally,
starting with one high-risk intersection path and one boolean path) carrying:

- absolute linear, relative, and angular tolerance;
- approximation error budget and fallback policy;
- parameter-space tolerance policy;
- iteration / subdivision / generated-topology / memory budgets;
- cancellation state;
- determinism options;
- diagnostics sink.

Current implementation: public booleans consume tolerance and fallback policy;
NURBS SSI consumes marching/queue/segment/branch, coupled-Newton, and recursive
seed-subdivision budgets; both consume the optional monotonic
`CancellationToken`. GFA polls between phases and face pairs, and SSI polls at
phase boundaries plus every marcher/adaptive-step iteration. Cancellation
returns typed `operation_cancelled` and the boolean transaction restores the
pre-operation topology. The other fields above remain the target contract, not
a claim of complete implementation.

The kernel's millimetre/radian convention is retained. Algorithm-local magic
epsilons in high-risk paths are replaced by named, scale-aware policy values
derived from the context — classified as physical-space tolerance,
parameter-space tolerance, angular tolerance, numerical floor, convergence
threshold, topological resolution threshold, or resource budget. Constants are
migrated by classification, not mechanically.

## Compatibility rules

- Existing functions keep returning their current values indefinitely.
- Detailed results are additive: new functions or versioned WASM methods
  (following the `executeBatchV2` precedent in
  `docs/design/deferred-e5b-stable-error-codes.md`).
- The detailed and legacy paths share one execution path internally so
  diagnostics cannot drift between them.
- Public API includes REST-like WASM method names, batch operation names,
  handle formats, and serialized shapes; all follow additive versioning.
