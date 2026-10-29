# Store-local Boolean correctness qualification

`qualify.cjs` exercises an actual Node WASM package. It records no timings and
makes no speed claim. Run it on the npm-installed directory after the canonical
package build and consumer checks; the package version, wrapper/WASM/package
hashes, Node version and harness hashes are recorded. A supplied source SHA is
explicitly external provenance, not something the driver verifies.

```bash
node scripts/performance/store-local/qualify.cjs \
  --kernel-pkg /absolute/path/to/node_modules/remus-wasm \
  --package-source-sha <40-character-verified-source-sha> \
  --out /tmp/store-local-qualification.json
```

The comparator's adversarial normalization/authority tests run with
`node --test scripts/performance/store-local/qualify.test.cjs`.

The output must not already exist. The default matrix contains six synthetic
fixtures with the switch off/on and direct/batch calls: slot cut, rib fuse,
window intersect, cross-drilled shaft, a cavity blank and a converted NURBS
box. It also replays the existing 31-feature holder recipe at widths 47 and
49 with the switch off/on. Pass `--recipe off` for only the synthetic matrix.
`--cases` accepts a comma-separated subset of the fixture names below.

The driver checks strict validation, independent material probes, a closed-form
volume where available, intact source operands, index and position-welded mesh
closure, and exact boundary content after removing only entity numbering,
face/inner-loop order and cyclic loop start. Numeric bits, curves, trims,
vertex/edge tolerances, winding and authoritative per-use pcurves remain in the
comparison. This is exact boundary-use content plus entity counts, not an
alias/incidence-preserving graph isomorphism for coincident distinct entities.
Strict validation, material and mesh checks support these bounded fixtures;
they do not establish arbitrary topology or naming equivalence. Raw arena
hashes can differ when numbering/order differ.

It separately checks the module-wide setting across kernel objects and
constructors, unchanged settings/inputs after refused batch items, and exact
agreement for entity-evolution and journal/reference contracts. Journaled calls
legitimately extend history; their source geometry is compared independently.

## Adoption boundary

The switch remains off by default. Enabling it applies to every kernel in the
same WASM module instance. The JS API does not expose the internal route taken:
a request alone does not prove locality. Native `perf-counters` tests assert
attempts/successes separately, including successful Cut/Fuse/Intersect and
cavity/NURBS declines. Evolution/journaled entry points retain their existing
general path.

The initial committed `2026.1.83` package passed exact boundary checks for
`slot_cut`, `rib_fuse`, `window_intersect`, `cavity_fallback` and
`nurbs_patch_fallback`. Both holder widths agreed exactly after the permitted
renumbering/order normalization. This is bounded qualification, not evidence
for arbitrary imported bodies, cavities, NURBS edits or naming adoption.

The default matrix also preserves a failed adoption gate:
`cross_drilled_shaft` with the switch on changes a seam-line edge's stored
tolerance from explicit `1e-7` to `None` (inherit from vertices), through both
direct and batch calls. For this fixture the current effective tolerance is
still `1e-7`, and material/mesh checks pass. Stored authority is different and
future vertex-tolerance changes can distinguish the policies. The exact
comparator does not normalize this discrepancy. It writes `passed: false`,
records the first difference, and exits with status 1. The native ignored
`cross_drilled_store_local_preserves_stored_line_tolerances` witness is its
independent source repro. A qualified positive subset can be run explicitly:

```bash
node scripts/performance/store-local/qualify.cjs \
  --kernel-pkg /absolute/path/to/node_modules/remus-wasm \
  --cases slot_cut,rib_fuse,window_intersect,cavity_fallback,nurbs_patch_fallback \
  --out /tmp/store-local-qualified-subset.json
```

A blanket default enablement or wider adoption is blocked until the strict
witness and affected identity, tolerance, pcurve and lineage contracts are
resolved and requalified on matched current-source packages. These checks
change no runtime behavior or numerical/mesh tolerance.
