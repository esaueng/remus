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
  tool still measures volume 64 with 6 faces);
- hammer-holder shifted intersect (`contract/hammer-shifted-intersect`, the
  WASM smoke's "hammer opening replay" up to its `shiftedCommon` step): the
  real Shapr3D hammer holder (`crates/io/tests/data/shapr3d_hammer_holder.step`)
  is cut and intersected with the smoke's translated 29×53×70 mask, then the
  101-face common is intersected with a copy of the source shifted by −2 in
  x, every boolean exact-only. Per surface the cell gates 104 faces, quality
  `exact`, the operations validator at zero errors, watertightness at
  (0.05, 0.1), and a volume strictly between 0 and the common's; across
  surfaces it gates face count, validation, mesh quality, quality, and the
  volume at the 5e-8 relative bound. The check-crate validator has no WASM
  binding, so it gates natively only (`validateSolidChecked`) and is labeled
  native-only evidence rather than passing vacuously. The STEP fixture enters
  every surface as ONE exact arena document (`remus-parity-native
  --step-to-arena`, decoded by `deserializeSolids` on each kernel), because
  the kernel package ships without the STEP translator; the operands are
  therefore bit-identical by construction. Batch has no arena-document op and
  the harness must not extend the WASM API, so the cell is direct-only on the
  installed surfaces (`directOnly`), like the cancellation cell.

The failure-class cells assert the same coarse wire `code` on every surface
(the native runner mirrors `StructuredWasmError`'s `From<OperationsError>`
mapping); the empty-result refusal maps through that mapping's catch-all
arm, so its `code` is `operation_failed` with category `internal` on all
three surfaces — a vocabulary observation, not a parity gap.

Every failed required stage carries a failure class, summarized at the top
of the report as `failure_classes`. A `platform_divergence` is a stage the
native facade meets while an installed WASM surface does not, or a
cross-surface disagreement over a native-green cell; everything else is a
`contract_violation` (the native facade itself misses the pin, or every
surface misses it together). The hammer cell is the model case: PR #618
(B39) made its shifted intersect refuse on wasm32 only — the marched
torus-fillet × cylinder section ends agreed across platforms to ~1e-12, yet
the WASM result carried 4 free edges while the native run closed — and
nothing native could catch that class (`io/tests/regress_hammer_opening_wasm_replay.rs`
passes on both sides). On the pre-#627 landing the cell reports the refusal
and the free edges as `platform_divergence` on the fresh tarball; a bad
result on every surface would be a `contract_violation`.

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

## Split import/export slice

`split-io-matrix.mjs` qualifies the actual distributed split packages
together — kernel plus translator — over the exact arena-document boundary,
executed on three evidence modes (native facade, freshly packed/installed
pair, committed installed pair). Run it from the repository root:

```bash
bash scripts/test-o15-split-io.sh
```

The script builds the native split runner and fresh Node-target WASM builds
of **both** packages (kernel `--no-default-features`, translator default
features) into temp dirs, packs each tarball, installs the pair into a
disposable npm consumer, and drives six cells through direct calls only
(the shipped kernel has no legacy I/O batch ops by construction):

- valid STEP import (`RemusIo.importStep` → arena bytes →
  `kernel.deserializeSolids` → volume/validate/census/probes) against the
  shared `split-box-2x3x4.step` fixture (volume 24, 6 planes, 1 shell);
- kernel-created box plus qualified hollow cut (outer 10 minus inner 8 at
  1,1,1) through `serializeSolids` → `exportStep` → re-import into a fresh
  session (volumes 24/488, faces 6/12, shells 1/2, wall-inside /
  cavity-outside / outside-outside probes);
- the existing `openzcad_e_analytic_fillet_plate.step` periodic fixture
  (volume 9522.743…, 10 faces, 48 per-use pcurves on native and 48 `PCURVE(`
  in re-exported STEP on every surface);
- malformed/limit-refused STEP (`maxInputBytes: 4`) preserving the
  pre-existing box (volume, faces, serialized bytes identical);
- truncated/empty arena transfer preserving earlier solids and handle
  validity;
- two independent sessions with repeated success/refusal (no partial
  solids, no leaked cross-session handles).

Fresh mode records one pinned source/configuration for both tarballs
(same revision, dirty flag, toolchain, features, both package versions,
both tarball/WASM hashes, both installed consumer entries). Committed mode
installs `crates/wasm/pkg` + `crates/wasm-io/pkg` into its own consumer and
labels the relationship `unverified` — a version match alone is never
source equivalence. The geometric 366-cell matrix and its disclosed
approximate-partition differences are untouched.
