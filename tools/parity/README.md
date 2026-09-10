# Native/WASM operation parity

Run the first O1.5 slice from the repository root:

```bash
bash scripts/test-o15-parity.sh
```

The script builds the native runner and a release Node-target WASM package,
packs `remus-wasm`, installs that tarball into a disposable npm consumer, and
then executes the same generated `executeBatchV2` bundle on both targets. The
installed entry is resolved and checked before any observation is accepted;
the committed package directory is never overlaid or rewritten. The JSON
report is written to `$CARGO_TARGET_DIR/o15-parity.json` (normally
`target/o15-parity.json`).

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

This is a bounded first slice, not O1.5 completion. Expanding the manifest to
the full batch-operation inventory, closing the serialized-byte gap, adding
failure/evolution fixtures, and wiring the Linux/macOS/Windows x86-64/arm64
nightly matrix remain open.
