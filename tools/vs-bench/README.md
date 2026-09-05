# Comparison scorecard protocol

O1.2d supplies the versioned observation format and JSON report generator for
`tools/vs-bench`. Kernel runners, real comparison scenarios, public tables and
baseline pins remain O1.2a–c/e/f. The committed input is **synthetic protocol
coverage**, not a measured kernel comparison or a performance claim.

From the repository root:

```sh
cargo run -p remus-vs-bench -- < tools/vs-bench/fixtures/exact-pair.json
cargo test -p remus-vs-bench
```

The CLI reads one JSON job from stdin and writes one JSON report to stdout.
Exit 0 means all gates passed, 1 means a report was emitted with failed gates,
and 2 means invalid input or an I/O error (diagnostic on stderr). A single
kernel is sufficient to evaluate gates; it never produces timing columns.
The workspace CI test job automatically runs the crate and CLI contracts.

## Version 1 input

`fixtures/exact-pair.json` is a complete executable example. Every struct
rejects unknown fields. Versions other than 1 are rejected. Reports are a
separate output shape and cannot be replayed as observations.

A job declares `schema_version`, `run_id`, `repetitions` (at least two), the
full `harness_sha`, an existing gauntlet `manifest_sha256`, unique `kernels`,
unique `scenarios`, and exactly one observation for every scenario/kernel
pair. Each kernel string identifies its build/binding. A scenario names its
`oracle` explicitly and declares its quality, applicable metrics, whether
it produces topology and whether native/WASM agreement is required.

The runner is the evidence authority: it executes the named oracle and
validator, compares repeated invariants, counts disclosures and resolves
manifest/reproduction references. This tool checks the evidence contract;
it does not execute geometry or authenticate claims against external files.
Use gauntlet manifest hashes, never a parallel corpus. The fixture's repeated
`a`/`b` hashes are placeholders, not published provenance.

Quality contains `representation` (`exact` or `approximate`), positive
`deflection`, an explicit canonical `tolerance_model` description including
units/bands, and nonnegative `error_budget`. Input and observed quality must
match exactly for a successful comparison. Units are declared by the job;
resource times are seconds, sizes/memory are bytes, and ratios are unitless.
No numeric competitor bands are introduced by this protocol.

Observations declare a `reported` status (`correct_success`, `exact_success`,
`approximate_success`, `repaired_success`, `refusal`, `error`, `crash`, `hang`),
nullable stable `diagnostic`, nullable `oracle_agrees`/`validator_accepts`,
`repeat_agrees`, observed `quality`, nullable `approximation`, counted typed
`repairs`, `repair_occurred`, `evolution_explicit`, nullable `defect_repro`,
and `metrics`. Success requires explicit oracle and validator booleans.
Approximation requires a named method and finite nonnegative `error_bound`
within the scenario budget. Every repair (including tolerance growth)
requires a code, positive count and independent verification.

`evolution_explicit` attests that every result entity has a construction event
or an explicit typed unresolved record. `evolution_completeness` records the
fraction having resolved events, so explicit unresolved records do not
artificially inflate completeness. Native/WASM-required and topology-producing
scenarios must declare the corresponding metrics applicable.

## Metrics and outcomes

All six measurement groups (plus the generated outcome group) and every column in `GROUPS` in `src/lib.rs` must be
present in input. Applicable metrics must be measured; inapplicable ones must
be null. Unknown columns, missing columns, wrong types, negative values and
out-of-range fractions are rejected. Both runtime statistics must be declared
together, with p95 at least the median. The runner supplies the statistics
from the declared repetition count; baseline statistical policy belongs to
O1.2f.

- History: evolution completeness and persistent-reference survival fractions.
- Interchange: import validity and post-import success booleans;
  round-trip geometry errors by volume, area, centroid and bounds;
  assembly fidelity fractions separately for tree, transforms, names, colors
  and materials. Relative-error normalization is part of the named oracle.
- Geometry: watertightness boolean and relative volume/area/centroid/inertia errors.
- Resources: runtime median/p95, peak memory, entity growth and cancellation latency.
- Browser: cold initialization time, raw/gzip/Brotli sizes, native/WASM agreement.
- Concurrency: thread scaling efficiency, requiring deterministic observations.

The output has all eleven outcome columns, exactly one set to 1. They are
categories, not overlapping counters: `correct_success` does not also count
`exact_success`. Reported success with oracle disagreement always becomes
`silent_wrong`, even if the validator or repeat check also fails. Otherwise
validator rejection becomes `invalid_success`, then repeat disagreement
becomes `nondeterminism`. Missing approximation/repair disclosure becomes
`untyped_error`, never a disclosed/verified success. Every failed gate remains
visible even when another defect takes outcome precedence.

Runtime median/p95 columns are emitted only if at least two kernels all pass
the gates and yield `correct_success`, `exact_success`, or
`disclosed_approximate_success` at the same declared quality. Refusals,
repairs and failed observations remain in the outcome table with timing
columns absent. Other metrics remain visible. Rows are sorted by
scenario/kernel for deterministic JSON output.

## Absolute gates

Every row reports pass/fail for oracle correctness, validity, crashes/hangs,
approximation disclosure, verified repair disclosure, typed non-success,
untyped errors, repeat determinism, required native/WASM agreement, complete
or explicitly unresolved evolution, declared quality and label consistency.
Any failure also requires a permanent minimized `defect_repro` reference;
providing one never waives the original failure. Reproducible run/manifest
identity is required before evaluation. These gates run identically for every
kernel and need no competitor baseline. A typed refusal is admissible for
this infrastructure gate; it is not proof of a workflow's capability gate.
