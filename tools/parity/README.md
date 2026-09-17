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
`$CARGO_TARGET_DIR/o15-parity.json` and the extended-slice report to
`$CARGO_TARGET_DIR/o15-parity-extended.json` (normally `target/`).

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

This extends but does not complete O1.5. Byte identity, failure/evolution
fixtures, and the Linux/macOS/Windows x86-64/arm64 nightly matrix remain
open.
