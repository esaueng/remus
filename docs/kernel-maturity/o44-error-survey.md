# O4.4 error survey (read-only, no code changes)

Survey for roadmap row O4.4 (Stable error-code registry, e5b): a complete
catalogue of every error variant the kernel can surface to JS, and where the
existing structured-diagnostics machinery (`crates/math/src/diagnostic.rs`:
`FailureCategory`, `Diagnostic`, `ToDiagnostic`) already covers it.

- Raw data: [o44-error-survey.csv](o44-error-survey.csv) — one row per variant
  (`crate,enum,variant,message,has_code,reachable_from_wasm,embeds_values`).
- Design context: [deferred-e5b-stable-error-codes](../design/deferred-e5b-stable-error-codes.md),
  [failure-taxonomy](failure-taxonomy.md),
  [O4.4 contract-surface section](open-kernel-implementation.md#o44-contract-surface-completion-s).
- Method: one read-only pass per crate over every `thiserror` enum
  (`#[error(...)]` templates quoted verbatim; multi-line templates joined);
  `ToDiagnostic` coverage checked by reading each crate's `diagnostic`
  impls; JS reachability traced through the `From` chain into
  `WasmError`/`JsError` (`crates/wasm/src/error.rs`) and `RemusIo`
  (`crates/wasm-io/src/`).

## Totals

184 variants across 17 error enums in 13 crates.

| Crate | Enum(s) | Variants | With native `ToDiagnostic` code |
| --- | --- | --- | --- |
| math | `MathError` (`crates/math/src/lib.rs`) | 13 | 13 |
| geometry | `GeomError` (`crates/geometry/src/error.rs`) | 4 | 1 (delegates to `MathError`) |
| topology | `TopologyError`, `DeleteSolidError` (`crates/topology/src/lib.rs`), `EdgeDomainError` (`crates/topology/src/edge.rs`), `BoundaryAuthorityError`, `CurveUseValidationError` (`crates/topology/src/validation.rs`) | 49 | 47 (44 direct + 3 delegating wrappers) |
| algo | `AlgoError` (`crates/algo/src/error.rs`) | 12 | 12 (10 direct + 2 delegating) |
| blend | `BlendError` (`crates/blend/src/lib.rs`) | 17 | 0 |
| heal | `HealError` (`crates/heal/src/error.rs`) | 8 | 0 |
| check | `CheckError` (`crates/check/src/error.rs`) | 7 | 0 |
| offset | `OffsetError` (`crates/offset/src/error.rs`) | 12 | 0 |
| sketch | `SketchError` (`crates/sketch/src/lib.rs`) | 3 | 0 |
| operations | `OperationsError` (`crates/operations/src/lib.rs`), `ResizeBlendError` (`crates/operations/src/resize_blend.rs`) | 31 | 0 native (`ResizeBlendError` has 7 ad-hoc `code()` strings, not `ToDiagnostic`) |
| io | `IoError` (`crates/io/src/lib.rs`) | 9 | 0 native (`InvalidValidationProperties.code` is an ad-hoc string forwarded as `kernelCode`) |
| wasm | `WasmError` (`crates/wasm/src/error.rs`), `LifecycleError` (`crates/wasm/src/bindings/lifecycle.rs`), `ReproError` (`crates/wasm/src/repro.rs`) | 16 | 6 wire codes on `WasmError` (batch-V2 registry); 0 native |
| wasm-io | `IoWasmError` (`crates/wasm-io/src/error.rs`) | 3 | 0 |

Native registry coverage: 73 of 184 variants (67 direct codes + 6
delegating wrappers). 137 of 184 messages embed runtime values
(floats, ids, indices, counts, or free-form reason strings); only static
messages are safe to match on, which is the point of the registry.

JS reachability: everything except 9 variants reaches JS today —
`ReproError` (8, native-only bundle harness) and
`SketchError::EntityInUse` (raised only by `remove_*` paths no binding
calls). All other sketch variants arrive stringified inside
`WasmError::InvalidInput`; `EdgeDomainError`,
`BoundaryAuthorityError`, and `CurveUseValidationError` arrive
stringified inside heal/check failures and validation-issue prose.

## Variants with no diagnostic code (97 + 14 ad-hoc)

No `ToDiagnostic` impl anywhere in their crate (see CSV `has_code=no`):

- blend 17/17, heal 8/8, check 7/7, offset 12/12, sketch 3/3,
  `OperationsError` 24/24, `IoError` 8/9 (the ninth is ad-hoc, below),
  `IoWasmError` 3/3, `LifecycleError` 2/2, `ReproError` 8/8 (native-only),
  `GeomError` 3/4 (except the `Math` delegate),
  `DeleteSolidError` 2/2.
- Ad-hoc codes that are NOT `ToDiagnostic` (do not satisfy the O4.4 exit
  gate as-is): `ResizeBlendError::code()` (7 kebab-case strings),
  `blend_ops::blend_failure_code` (wired as `kernelCode` on the fillet/
  chamfer throw path), the 6 `WasmError` batch-V2 wire codes plus the
  per-arm `kernelCode` projections in `StructuredWasmError`
  (`crates/wasm/src/error.rs`), and `InvalidValidationProperties.code`.

This confirms the inherited-queue claim: `OperationsError` is the one
large enum still outside the pinned registries, and blend/heal/check/
offset/sketch/io have no native registry at all.

## Duplicates: same meaning, different types

Candidates for one code (or explicitly aliased codes) when the registry
is built:

- `invalid input: {reason}` — identical template in `OperationsError`,
  `OffsetError`, `ResizeBlendError`, `WasmError`, `IoWasmError`.
- Empty input — `MathError::EmptyInput` vs `GeomError::EmptyInput`.
- Nonconvergence — `MathError::ConvergenceFailure` vs
  `GeomError::ConvergenceFailure` vs `CheckError::IntegrationFailed`.
- `topology error: {0}` / `math error: {0}` wrappers in `AlgoError` and
  `OffsetError` vs `#[error(transparent)]` for the same inner errors in
  blend/heal/check/operations/io (one failure already has one code via
  delegation; the prefix variants should delegate too).
- `assembly failed` — `AlgoError::AssemblyFailed` vs
  `OffsetError::AssemblyFailed`.
- `intersection failed` — `AlgoError::IntersectionFailed` vs
  `OffsetError::IntersectionFailed`.
- `analysis failed` — `HealError::AnalysisFailed` vs
  `OffsetError::AnalysisFailed` (different shapes: free string vs
  edge + reason).
- Classification — `AlgoError::ClassificationFailed` vs
  `CheckError::ClassificationFailed`.
- Radius too large — `BlendError::RadiusTooLarge` (edge + max) vs
  `ResizeBlendError::RadiusTooLarge` (scalar radius, mm).
- Unsupported pair — `AlgoError::UnsupportedSurfacePair` vs
  `ResizeBlendError::UnsupportedSupportPair`.
- Invalid handle — `SketchError::InvalidHandle` (no detail) vs
  `WasmError::InvalidHandle` (entity + index).
- `operation_failed` / `internal_error` wire codes collect all of the
  above at the batch boundary today; several distinct native failures
  share one wire code with no `kernelCode`.

## Proposed code namespace scheme (proposal only — not implemented)

1. Keep `FailureCategory` (9 categories) unchanged; category moves stay
   breaking.
2. New codes take the shape `<domain>_<specific>`, all lowercase
   snake case, where `<domain>` is a short crate tag:
   `math`, `geom`, `topo`, `gfa`, `blend`, `heal`, `chk`, `offs`,
   `gcs`, `ops`, `io`, `wasm`. Examples: `ops_invalid_input`,
   `blend_radius_too_large`, `offs_collapsed_solid`,
   `gcs_invalid_handle`, `topo_entity_not_found`.
3. Grandfather clause: all 67 pinned native codes and the 11 batch-V2
   wire codes keep their exact strings (additive rule in
   `crates/math/src/diagnostic.rs`); duplicates above get explicit
   alias notes, not renames.
4. Wrapper variants (`#[from]`/`transparent`) must delegate to the inner
   `diagnostic()` (one failure, one code) — extend the algo/topology
   pattern to blend/heal/check/offset/operations/io.
5. Registry-completeness test per crate (every variant maps; codes
   unique within the crate) plus one central uniqueness test over the
   concatenated registry; wire through `StructuredWasmError` as
   `details.kernelCode` with the existing broad wire code unchanged.
6. `embeds_values=yes` payloads move to typed `Diagnostic` details
   (ids, deviations, limits); prose messages stay non-contract.

## What O4.4 implementation still needs (out of scope for this survey)

- `ToDiagnostic` impls for blend, heal, check, offset, sketch,
  operations, io (and `GeomError`'s three native variants).
- A home for `DeleteSolidError`, `LifecycleError`, and the ad-hoc
  `ResizeBlendError::code()` / `blend_failure_code` /
  `InvalidValidationProperties.code` strings (adopt, alias, or retire).
- The registry-completeness test and the rendered code table for O6.1.
- No Rust changes were made in this survey.
