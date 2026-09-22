# Native/WASM operation parity

Run the O1.5 slices from the repository root:

```bash
bash scripts/test-o15-parity.sh
```

The script builds the native facade runner and a release Node-target WASM
package, packs `remus-wasm`, installs that tarball into a disposable npm
consumer, and then executes the same generated `executeBatchV2` bundle on
both targets: once through the native facade (`crates/remus`, `Model`) and
once through the real WASM batch build. The installed entry is resolved and
checked before any observation is accepted; the committed package directory
is never overlaid or rewritten. The first-slice JSON report is written to
`$CARGO_TARGET_DIR/o15-parity.json`, the extended-slice report to
`$CARGO_TARGET_DIR/o15-parity-extended.json`, and the contract-slice report
to `$CARGO_TARGET_DIR/o15-contract.json` (normally `target/`).

## Evidence modes and provenance

Every report carries a `provenance` block distinguishing three evidence
modes where supported:

- `native`: the source-built facade runner (`remus-parity-native`,
  `remus::Model`), with the source revision, dirty flag, and toolchain.
- `fresh` / `fresh-tarball`: the freshly built Node WASM package packed and
  installed from that source, with the package version, tarball SHA-256,
  WASM SHA-256, build options (target, profile, features), and the resolved
  installed entry path.
- `committed`: the existing committed/distributed package directory
  (`crates/wasm/pkg`), installed into its own disposable consumer for the
  contract slice only. Its provenance records the harness baseline revision
  for comparison but labels the relationship `unverified` — committed bytes
  are never labeled with the current source SHA unless a rebuild
  establishes it. `provenance.mjs` owns this vocabulary
  (`collectFreshProvenance`, `collectCommittedProvenance`,
  `describeStaleness`, `validateProvenance`, `validateInstalledEntry`,
  `requireObservations`); the contract report also records a
  version/hash staleness verdict between the fresh and committed bytes.

## First matrix slice

`quadric-boolean-matrix.json` owns the fixture inputs and independent analytic
volume oracles for the newly qualified cone/sphere, sphere/cylinder, and
torus/sphere boolean families. `quadric-boolean.mjs` generates all 54 cells:

- fuse, cut, and intersect;
- scale 0.1, 1, and 10;
- origin and rigidly transformed placements.

Each target must independently report exact result quality, no structured
diagnostics, a valid B-Rep, only the declared analytic carrier types, both
required operand carriers, a watertight manifold mesh, and volume within the
fixture's analytic bound. The cross-target comparison then requires equal
outcome and diagnostic codes, exact census and carrier histograms, equal mesh
quality invariants, and volume agreement at a `5e-8` relative cross-target
bound (20,000 times tighter than the independent geometry-oracle bound).
Serialized arena length and SHA-256 are also compared, but byte identity is a
visible non-gating gap in this first semantic-parity slice: all 54 raw
documents currently differ across targets despite semantic agreement.
Timings are recorded as evidence only; they are not a competitive benchmark.

## Extended matrix slice

`parity-matrix.json` extends the harness to the remaining primitive pairs,
the modifier ops (fillet, chamfer, shell, offset, draft), and the sweep
family (extrude, revolve, sweep, loft, pipe, helical) at scales 1e-3, 1,
and 1e3. `parity-matrix.mjs` generates all 366 cells:

- 16 boolean families (every box/cylinder/cone/sphere/torus pair, plus the
  three first-slice families) x fuse/cut/intersect x three scales x origin
  and rigidly transformed placements;
- 13 modifier/sweep families x three scales x two placements.

Boolean cells reuse the first slice's `booleanWithQuality` bundle shape
(without `exactOnly`, so disclosure is whatever each surface returns).
Modifier and sweep cells drive the same batch ops the WASM build exposes
(`solidEdges` + `fillet`, `chamfer`, `getSolidFaces` + `shell`/`draft`,
`offsetSolid`, profile `makeLineEdge`/`makeWire`/`makePlanarFaceFromWire`
feeding `extrude`/`revolve`/`sweep`/`pipe`/`loft`/`helicalSweep`); the
native side executes each bundle through the `remus::Model` facade rather
than a natively compiled kernel. Handle references (`{fromOp}`) resolve
against earlier responses on both targets, mirroring chunked batch
dispatch.

Each cell asserts bit-identical or bounded-identical volume (closed-form
oracle per family where one exists, else the 5e-8 relative cross-target
bound), exact face census, and disclosure outcome (`exact` vs disclosed
`approximate`/typed refusal, compared in the WASM wire vocabulary) between
the native facade and the real WASM batch build. Any divergence is a
finding: seed plus new §B row. Serialized arena SHA-256 is compared as a
visible non-gating gap, as in the first slice. Timings are recorded as
evidence only; they are not a competitive benchmark.

This extends but does not complete O1.5. Byte identity and the
Linux/macOS/Windows x86-64/arm64 nightly matrix remain open; the contract
slice below carries the failure/evolution fixtures.

## Contract slice

`contract-matrix.mjs` adds the small contract-focused matrix the geometric
slices do not cover, executed on all three evidence modes (native,
fresh-tarball, committed-package) plus direct installed JS calls alongside
batch where the API exists:

- exact result (overlapping boxes fuse, quality `exact`, volume 15);
- exact-only refusal (tangent-boss fuse with `exactOnly`, typed
  `operation_failed` instead of a silent mesh);
- opted-in disclosed approximation (same boss without `exactOnly`,
  quality `approximate` with its deflection);
- invalid handle (stale solid refuses `invalid_handle`);
- transactional rollback (a mid-batch failure leaves earlier solids
  measurable with identical volume and face count on every surface);
- supported cancellation via a pre-cancelled cooperative token
  (`OperationCancellationToken` + `booleanWithCancellation` direct on WASM,
  `booleanWithCancelledContext` on the native runner);
- evolution reports (overlapping boxes through `fuseWithEvolution` and
  `cutWithEvolution`): the exact-only boolean's `{solid, evolution}` response
  is gated per surface on the bucket counts (`modified` inputs/outputs,
  `generated` inputs/outputs, `deleted`, `unresolved`) plus `origin`
  (`construction`), on an empty `unresolved` bucket, and on oracle volume
  (15 / 7) and face count (12 / 9); across surfaces the summary, the
  unresolved bucket, and the face count must agree, while raw index-level
  agreement (which arena handles each bucket names) rides along as
  non-gating evidence like byte identity. The native runner dispatches the
  same `remus_operations::boolean::boolean_with_evolution` entry point the
  WASM batch arm calls and passes `EvolutionMap::to_json` through verbatim;
- invalid primitive input (`makeBox` with a negative width refuses
  `invalid_argument`; the earlier box still measures volume 8 with 6 faces);
- empty-intersect sentinel (two far-apart boxes `intersect` succeed with the
  kernel's typed empty-result solid: quality `exact`, volume 0, zero faces,
  and the operand still measures volume 8 through an `operandVolumeIndex`
  probe the native runner answers via `volumesByIndex`);
- contained-cut refusal (cutting a box fully inside its tool refuses typed:
  the kernel `EmptyResult` reaches the wire as `operation_failed`, and the
  tool still measures volume 64 with 6 faces).

The failure-class cells assert the same coarse wire `code` on every surface
(the native runner mirrors `StructuredWasmError`'s `From<OperationsError>`
mapping); the empty-result refusal maps through that mapping's catch-all
arm, so its `code` is `operation_failed` with category `internal` on all
three surfaces — a vocabulary observation, not a parity gap.

Cancellation scope is labeled honestly: a synchronous WASM call cannot
process a later JS cancellation message on the same thread, so only the
pre-cancelled token is asserted; concurrent mid-call cancellation needs a
worker/shared-memory transport and remains unqualified. Batch has no
cancellation op, so that cell is direct-only by construction. Direct-vs-batch
consistency per installed surface is recorded as non-gating evidence; the
cross-target gates are outcome, diagnostic-code, quality, oracle-volume
(where applicable), and rollback preservation. The geometric matrices and
their known approximate partition differences are unchanged: byte identity
stays visible and non-gating, and no tolerance was relaxed to make a cell
pass.
