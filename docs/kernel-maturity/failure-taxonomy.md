# Failure taxonomy

Stable failure categories for the whole kernel. Goal: any failure can be
described with a stable category plus a reproducible input, native and WASM,
without parsing prose.

## Relationship to the existing registry

The WASM batch API already has a stable, additive error-code registry
(`executeBatchV2`, `docs/design/deferred-e5b-stable-error-codes.md`). This
taxonomy generalizes that design kernel-wide rather than inventing a second
one:

- The batch registry's codes remain unchanged and become the WASM projection
  of the native categories below.
- Native diagnostics get typed category + code values with the same rules:
  lowercase snake case, additive registry, meanings never broadened or
  reassigned, and **codes never derived from `Display` strings, `Debug`
  output, or Rust enum/type names**.
- Direct (non-batch) WASM methods gain structured errors only through
  additive detailed APIs, per the resolved decisions in E5b.

Implementation: the categories and the native code registry live in
`remus_math::diagnostic` (`FailureCategory`, `Diagnostic`, `ToDiagnostic`),
currently implemented for `MathError`, `TopologyError`, and `AlgoError` with
pinned registry tests. `executeBatchV2` errors carry `category` and, when the
failure originated in a typed native error, `details.kernelCode` — see the
book's WebAssembly chapter for the wire contract. The batch
`booleanWithQuality` path pins `ExactOnlyUnattainable` as
`quality_refused` / `exact_only_unattainable`, including rollback.

## Categories

Every kernel failure belongs to exactly one category. Categories are the
stable coarse level; codes within them are the stable fine level.

| Category | Meaning | Examples of existing/expected codes |
| --- | --- | --- |
| `invalid_input` | The request is malformed independent of geometry difficulty: non-finite values, out-of-range arguments, bad handles, empty topology. | `invalid_argument`, `invalid_handle`, `invalid_json`, `missing_operation`, `unknown_operation` |
| `invalid_topology` | Referenced topology exists but is inconsistent or fails validation preconditions. | `topology_error` |
| `unsupported` | The operation is well-formed but the capability cell is declared unsupported. Always cites the cell (family + configuration). | `unsupported_configuration`, existing typed refusals such as `RadiusTooLarge`, unsupported-support-pair |
| `nonconvergence` | An iterative algorithm exhausted its declared budget without certifying an answer. Never presented as an empty success. | `iteration_budget_exhausted`, `no_convergence` |
| `resource_limit` | A byte/entity/work/memory budget was exceeded (import limits, batch caps, topology growth caps). | `resource_limit_exceeded`, `batch_limit_exceeded` |
| `tolerance_violation` | A result was produced but failed its own tolerance contract (SameParameter/SameRange deviation, validation deviation beyond limit) and the policy forbids repair. | `same_parameter_exceeded`, `validation_deviation` |
| `quality_refused` | The only achievable result would degrade quality beyond the caller's fallback policy (ExactOnly meets a case needing approximation). | `exact_only_unattainable` |
| `cancelled` | The operation observed a cancellation request and stopped at a safe point (rollback complete). | `cancelled` |
| `internal` | A failure that cannot be safely classified. Reaching this category is itself a defect to burn down. | `internal_error`, `operation_failed` (legacy broad code) |

Rules:

- A category is part of the public contract; moving a failure between
  categories is a breaking change and requires a new code, not a
  reinterpretation.
- `unsupported` failures must name the capability-matrix cell so the failure
  is auditable against the declared domain.
- `nonconvergence` and `resource_limit` failures must report the budget and
  the amount consumed.
- Every failure must leave topology in its pre-operation state (see the
  transactional contract in
  [operation-contract.md](operation-contract.md)).

## Diagnostics (non-fatal)

Warnings share the code registry and appear in `OperationResult.diagnostics`:
tolerance increases, healing actions, approximate-quality notes, ambiguous
evolution, near-degenerate classifications resolved by policy. A warning must
carry enough structured context to be machine-actionable (entity ids, measured
deviation, limit).

## Validation results

Validators report: stable check code, offending entities, severity, measured
deviation, expected limit, and suggested repair when available (the existing
`CheckId`/`ValidationReport` structure in `crates/check` is the seed; codes
join this registry).

## Reproducibility requirement

A failure report is complete only when paired with a reproducible input. The
deterministic reproduction bundle (backlog Issue 2; see
[testing-strategy.md](testing-strategy.md)) is the canonical carrier: model
input, operation sequence, operation context, build revision, expected
invariant results. Until it exists, the fallback is the existing practice of
minimized fixtures (arena serialization or STEP) per
`book/src/tolerances.md`.
