# Testing strategy

What evidence qualifies a capability cell, how defects become permanent
regressions, and what CI gates grow toward. Complements the repo's existing
test layout (unit, proptest, golden, integration, wasm contract, bench) — it
does not replace it.

## Evidence classes

A capability cell moves to **Qualified** ([capability-matrix.md](capability-matrix.md))
only with:

1. **Matrix-generated tests** — the cell's configuration is produced by the
   family's matrix harness (axes × axes), not a hand-picked fixture, so
   coverage claims are enumerable.
2. **Postcondition checks** — tests assert the universal postconditions of
   [operation-contract.md](operation-contract.md) (validity, determinism,
   bounded budgets, typed failure, disclosed fallback), not only the value.
3. **Negative/boundary tests** — the cell's declared boundary is tested from
   both sides; refusals pin their stable code.
4. **Native + WASM agreement** — for user-visible behavior, a contract test
   through the WASM path (the `execute_batch` contract-test pattern).

Characterization-first: a slice that changes behavior lands its failing
characterization or regression test before the implementation.

The B6 primitive family is the current reference matrix: seven
primitive kinds × 1e-3/1/1e3 scale, with closed-form volume/bounds, exact
topology and surface censuses, dual validation, oriented closed B-Rep,
watertight/manifold mesh, independent mesh-volume integration, determinism,
typed non-mutating refusal, and direct/batch WASM parity. The ellipsoid cell
also demonstrates the characterization-first rule: its initially ignored
acceptance target exposed distinct geometry, tessellation, and bounding-box
defects before becoming an always-on matrix row and deterministic replay.

## Reproduction bundles (Issue 2)

A versioned format + runner containing: model input, operation sequence,
operation context, build revision, and expected invariant results. Every new
regression uses it; every confirmed bug becomes a minimized permanent bundle.
Bundles must replay deterministically on native and WASM builds and against
future builds (versioned schema).

## Property and metamorphic testing

Standing properties, applied per family as they qualify:

- native/WASM per-operation invariant agreement over the whole batch
  operation list, generated rather than hand-written (O1.5);
- perturbation stability: a qualification fixture keeps its classification
  or refuses typed under coordinate nudges 1e-13..1e-6 (O2.4);

- transform invariance; uniform-scale invariance with correspondingly scaled
  tolerance;
- parameter reversal; operand symmetry where applicable;
- boolean idempotence; split-then-rejoin;
- tessellation refinement convergence;
- STEP read/write/read stability;
- native/WASM equivalence; determinism across repeated runs.

## Differential testing

Compare against an established open-source kernel as an oracle, and against
commercial kernels only where appropriately licensed. Mechanics:

- The oracle runs **out-of-process** in a separate harness outside this
  workspace — no copyleft code links into the Apache-2.0 workspace, and no
  oracle source is read while implementing kernel code (clean-room rule in
  [target.md](target.md)). Repository policy also bans naming the reference
  kernel in committed text; harness scripts are referenced by path.
- Compare invariant outcomes, not raw face ordering: validity, volume/area,
  containment, region count/type, surface types, intersection curves,
  bounding boxes, distances, expected-failure classification.
- A disagreement is a triage input, not automatic proof the oracle is right.
- The head-to-head harness (O1.2) is the only sanctioned instrument for a
  *competitive* claim. Its per-scenario metric schema (O1.2d), workflow
  scenarios W1–W9 (O1.2e), and baseline-pin milestone (O1.2f) are specified
  in [industrial-parity.md](industrial-parity.md) §3–§4. Two rules bind every
  comparison: equivalence before speed (a timing is admissible only when both
  kernels succeeded at the same declared output quality), and no numeric
  band anywhere before the baseline is pinned. The absolute gates in §3.4
  there (zero silent-wrong, zero crash, zero undisclosed approximation or
  repair, typed non-success, determinism, native/WASM agreement, complete
  evolution, permanent reproduction, reproducible claims) need no baseline
  and gate every horizon from H5 on.

## Fuzzing

Persistent fuzz targets (extending `fuzz/`): STEP/IGES parsing, NURBS
construction/evaluation, curve and surface intersections, topology mutations,
boolean operation sequences, blend and offset inputs, WASM batch decoding,
native serialization. Minimized corpus inputs are stored under version control
when licensing permits.

Current scheduled coverage includes the public import readers, structured
boolean trees, modifiers, `nurbs_surface`, and `topology_mutation`. The
NURBS target generates bounded rational surfaces and an independent
horizontal NURBS plane, then requires finite evaluation/derivatives, typed
constructor refusal for corrupted data, and SSI points, parameters, and
fitted curves that satisfy both surfaces and the plane equation. The
topology-mutation target drives derivation, transactional-rollback,
checkpoint-restore, and solid-deletion sequences over a bounded box against
the mutation contracts: exact-state rollback, tombstoned checkpoint
retirements with a never-dangling derivation map, permanent stale-handle
invalidation without slot reissue, atomic typed deletion refusal, and
complete unshared-tree retirement, with validation, closed-manifold census,
and closed-form volume oracles checked after every mutation. Its committed
corpus carries the checkpoint re-derivation seed that exposed a dangling
loop-derivation map after restore. The `arena_roundtrip` target round-trips
bounded box/cylinder documents — duplicate roots, shared-shell aliases,
repeated/aliased compound members, hostile tolerances, attributes — through
the native arena format, requiring per-position validation, census, and
closed-form volumes, bit-exact tolerance/trim/attribute survival,
byte-identical re-serialization, and typed, non-mutating refusal of
corrupted references. Its byte-identity oracle pinned the serde_json
`float_roundtrip` feature as load-bearing for exact f64 replay.
`arena_reader` and `wasm_batch` currently compile in PR CI but are not
scheduled; curve-intersection, offset, GCS, and tessellation fuzzing remain
outstanding and are owned by bridge row B19.

B19's mutation-scope slice moves the config to `.cargo/mutants.toml`, the
location cargo-mutants 27.0.0 actually discovers. The previous root-level
file was ignored by the weekly command; loading it explicitly also exposed
its stale `cdt.rs` glob. The corrected `cdt/**` glob selects 846 mutants
across all five production CDT modules (at source `f4d03b76`), preserves the
existing NURBS/predicate/hull scope, and excludes unrelated math and test
files. `python3 scripts/test-mutants-scope.py` exercises real default
selection and rejects both missing-config and stale-glob negative controls.
The Mutation Scope workflow runs that contract with the weekly job's pinned
tool version whenever its config, math sources, or workflow changes.

A bounded execution check used cargo-mutants 27.0.0 and Rust 1.96.0:

```bash
cargo mutants --package remus-math --re sorted_pair --jobs 1 \
  --output /tmp/remus-cdt-mutants -- --lib cdt::tests
```

The unmutated baseline passed; all five selected CDT mutants were examined,
four were caught and one survived: `replace <= with > in sorted_pair`.
The command correctly exited 2 and retained that survivor in `missed.txt`
and `outcomes.json`; no mutant was suppressed. Reversing the pair ordering
still provides a consistent undirected key, but this bounded run does not
claim mutation completeness or close the remaining B19 fuzz work. The
weekly changed-lines filter and mutation-result failure policy are unchanged.

## Corpus

Licensed or generated corpora: primitive adversarial cases, imported STEP,
periodic seams, tangency/coincidence, slivers and short edges, large and tiny
scales, cavities and nested shells, high-degree NURBS, realistic mechanical
parts, long operation sequences. Corpus replay is deterministic and becomes a
CI stage.

## Approximation census

CI runs `scripts/check-approx-census.py` against the committed
`crates/operations/examples/approx_census.snapshot`. The snapshot is a
semantic ratchet: operation and case identity, result face count,
exact/fallback/error provenance, and revolve surface counts are authoritative.
Wall-clock timings and rebuild-local arena IDs are normalized away. Any other
movement fails with a reviewable diff; an intentional change updates the
snapshot in the same PR after the affected geometry has its own oracle-backed
regression.

## Performance and resource qualification

Track runtime, peak memory, entity growth, iteration counts, generated
intersection segments, tessellation size, and WASM package/memory behavior
(the criterion suite and the existing 64-cut determinism/perf gate are the
seeds). Correctness gates pass before performance comparisons are accepted.

## CI growth path

Gradually added, in roughly this order: persistent corpus replay; fuzz smoke
tests; risk-based coverage gates and changed-line coverage; native/WASM parity
tests; platform matrix; performance regression budgets; memory/entity-growth
budgets; sanitizer-compatible tests where applicable; determinism reruns.

Do not pursue a single global coverage percentage at the expense of testing
critical geometric states — a matrix cell with no test is a coverage gap even
when line coverage is high.

## Anti-goals

- No test weakened, tolerance widened, or fallback added merely to pass.
- No promotion on one fixture.
- No silent cap: when a harness bounds coverage (top-N, sampling), it says so
  in its output.
