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
  --require-route-proof on \
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
comparison. A second, boundary-ordered graph encoding preserves sharing and
incidence across every vertex, edge, wire, coedge, loop, face, shell and solid,
including orphan table entries. Equal encodings establish a concrete bijection;
coincident distinct entities cannot silently collapse or change their sharing.
This is not a complete general graph-isomorphism solver: indistinguishable
ordering ties can reject an otherwise equivalent graph. Supported documents
have solid roots. Attributes and journal/naming contracts are checked separately.
Raw arena hashes can differ when numbering/order differ.

It separately checks the module-wide setting across kernel objects and
constructors, unchanged settings/inputs after refused batch items, and exact
agreement for entity-evolution and journal/reference contracts. Journaled calls
legitimately extend history; their source geometry is compared independently.

## Adoption boundary

The switch remains off by default. Enabling it applies to every kernel in the
same WASM module instance. A request alone does not prove locality. The existing
opt-in performance trace now includes `boolean.route.*` observations. The
qualifier traces only the synthetic operation, separately from setup and
quality checks, and records holder preparation and rebuild routes separately.
`--require-route-proof on` requires real local successes for the enabled
slot/rib/window/cross-drilled cells and the expected general/decline observations. Missing
capabilities or truncated traces are unavailable evidence, never zero successes.
The default is `off` so historical packages can still be compared; use `on` for
current adoption evidence. Native `perf-counters` tests independently assert
actual routes. Evolution/journaled entry points retain their existing general
path. A converted planar NURBS operand can decline initially, then take the
existing planar-normalization retry; the observation distinguishes these steps.

The initial committed `2026.1.83` package passed exact boundary checks for
`slot_cut`, `rib_fuse`, `window_intersect`, `cavity_fallback` and
`nurbs_patch_fallback`. Both holder widths agreed exactly after the permitted
renumbering/order normalization. This is bounded qualification, not evidence
for arbitrary imported bodies, cavities, NURBS edits or naming adoption.

The historical default matrix preserves a failed adoption gate:
`cross_drilled_shaft` with the switch on changes a seam-line edge's stored
tolerance from explicit `1e-7` to `None` (inherit from vertices), through both
direct and batch calls. For this fixture the current effective tolerance is
still `1e-7`, and material/mesh checks pass. Stored authority is different and
future vertex-tolerance changes can distinguish the policies. The exact
comparator does not normalize this discrepancy. On an affected package it
writes `passed: false`, records the first difference, and exits with status 1.
The normal, non-ignored native
`cross_drilled_store_local_preserves_stored_line_tolerances` regression now
requires both exact authority and actual local success. The repair reuses source
entities only on the boundary shared with retained faces; new patch interiors
keep the GFA builder's stored authority. Boundary reuse follows the full builder's
original-edge precedence. Ambiguous matches, incompatible vertex authority and
unsupported rebuilt per-use data decline without mutation. Tests cover explicit
and inherited edge tolerances, sharing, periodic kept-face authority and exact
public fallback. A historical positive subset can still be run explicitly:

```bash
node scripts/performance/store-local/qualify.cjs \
  --kernel-pkg /absolute/path/to/node_modules/remus-wasm \
  --cases slot_cut,rib_fuse,window_intersect,cavity_fallback,nurbs_patch_fallback \
  --out /tmp/store-local-qualified-subset.json
```

The repaired witness does not authorize blanket default enablement or arbitrary
imported-body, cavity, NURBS-edit or naming adoption. Qualification uses matched
current-source packages and retains numerical/mesh tolerances and strict gates.
