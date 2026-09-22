# Boolean contract audit — exactness and disclosure

**Date:** 2026-09-21
**Baseline:** `origin/main` at `75cce2c9` (`fix(heal): make pinch wire splitting linear (#557)`).
**Comparison snapshot checked:** `76dbffd7` (the PR #568 comparison SHA) — the stale
`booleanWithQuality` comment is present there and is corrected here by tracing
implementation, not the comment.
**Scope owner:** Boolean contract audit only. No GFA geometry, properties,
edge-validation, evolution-implementation, parity-runner, or benchmark-runner
changes. O4.7's unrelated families stay open.

B21 remains **Done**. Plain handle-returning Boolean entry points were already
exact-only at both SHAs; this audit preserves that status, corrects the one
contradictory doc comment, inventories every public entry point, and closes the
single remaining silent-fallback dispatch gap (`compound_cut`).

## Method and evidence classes

- **Source observation:** traced dispatch from the public entry point to
  `boolean_with_operation_context` / `run_mesh_fallback` / GFA, recording the
  effective `FallbackPolicy`, return type, and transaction boundary.
- **Reproduced failure:** `compound_cut` on the B21 tangent-boss fixture returned
  a bare handle via the mesh fallback where every sibling bare-handle entry
  point refuses. Witness below (native example run, since removed).
- **Expected refusal:** tangent-boss exact-only paths return typed
  `ExactOnlyUnattainable` (`quality_refused` / `exact_only_unattainable` over
  WASM) with full rollback.
- **Qualified result:** overlapping-box exact fuse/cut/intersect verified by
  independent closed-form volume, dual validation, and watertight mesh —
  preview appearance and two wrappers over one integrator are not used as proof.

Units are millimetres/radians throughout. Tolerances are the kernel defaults
(`Tolerance::new()`; linear 1e-7) unless a context/option overrides them.
Cavity shells, exact carriers/trims, rollback, public wire formats, defaults,
and Apache-2.0 lineage are preserved — no tolerance widening, silent healing,
faceting, or weakened assertions were used to make any case pass.

## Finite contract inventory

`ExactOnly` = mesh fallback refused with `OperationsError::ExactOnlyUnattainable`.
`Permissive-disclosed` = fallback may run at the context budget but the outcome
reports `BooleanQuality::{Exact, Approximate{deflection}}`. Rollback is via
`remus_topology::transaction::run_transacted` unless noted.

### Native (`crates/operations/src/boolean/`)

| Entry point | Effective fallback | Quality / diagnostic return | Refusal | Rollback |
|---|---|---|---|---|
| `boolean(topo, op, a, b) -> SolidId` | `ExactOnly` via `exact_only_context()` | None (bare handle = exact or no result) | `ExactOnlyUnattainable`; invalid/empty/non-manifold typed | `run_transacted` — live counts + slots unchanged, operands valid |
| `boolean_with_context(topo, op, a, b, ctx) -> BooleanOutcome` | `ctx.fallback` (`OperationContext::new()` = `AllowApproximate{0.1}`) | `BooleanOutcome{ solid, quality }`; approximate carries deflection = policy budget | `ExactOnlyUnattainable` under `ExactOnly`; invalid-context typed | `run_transacted` |
| `boolean_with_options(topo, op, a, b, opts) -> SolidId` | `ExactOnly` (options context validated then pinned) | None (bare handle) | `ExactOnlyUnattainable`; `deflection` validated but never consent | `run_transacted` + `apply_boolean_options` validates; unify rollback keeps valid unsimplified result |
| `boolean_outcome_with_options(topo, op, a, b, opts, ctx) -> BooleanOutcome` | `ctx.fallback`; `opts.deflection/tolerance` ignored for context budget/tolerance | `BooleanOutcome` with disclosed quality | `ExactOnlyUnattainable` under `ExactOnly` | `run_transacted` |
| `boolean_with_evolution(topo, op, a, b) -> (SolidId, EvolutionMap)` | `ExactOnly` (faithful GFA path + `boolean` fallback, both exact-only) | `EvolutionMap` with `Construction` vs `Geometry` origin; `unresolved` bucket explicit | `ExactOnlyUnattainable` | One `run_transacted` over both attempts |
| `boolean_with_entity_evolution(topo, op, a, b) -> (SolidId, EntityEvolution)` | No fallback (direct GFA) | Raw construction records; no quality (exact-only by construction) | GFA typed errors | Caller transaction (WASM/natively transacted by callers) |
| `boolean_regions(topo, op, a, b) -> BooleanRegionsResult` | No fallback (direct `boolean_regions_with_entity_evolution`) | Per-region `BooleanRegion` lineage; incomplete edge lineage refused | `EmptyResult` / lineage / validation typed | `run_transacted` |
| `boolean_compound_regions(topo, op, a/c, b/c) -> BooleanRegionsResult` | No fallback | Same per-region lineage | Overlapping members / multi-tool cut `Unsupported`; empty typed | `run_transacted` |
| `boolean_transacted(topo, op, a, b) -> SolidId` | `ExactOnly` | None + pre-commit L3 `validate_solid` gate | Validation failure rolls back with count | `run_validated` |
| `compound_cut(topo, target, tools, opts) -> SolidId` | `ExactOnly` **after this audit** (was `AllowApproximate` from `opts.deflection` — the reproduced defect) | None (bare handle) | `ExactOnlyUnattainable`; `>256` tools `InvalidInput` | One `run_transacted` over batch + sequential paths |
| `compound_ops::fuse_solids(topo, &[SolidId]) -> SolidId` (`fuseAll`) | `ExactOnly` via `fuse_cluster` + disjoint merge (no mesh path) | None (bare handle) | `ExactOnlyUnattainable`; empty `InvalidInput` | One `run_transacted` over whole reduction |
| `compound_ops::fuse_all(topo, CompoundId)` | Same as `fuse_solids` | None | Same | Same |

### Facade (`crates/remus/src/model.rs`)

| Entry point | Effective fallback | Quality / diagnostic return | Refusal | Rollback |
|---|---|---|---|---|
| `Model::boolean / fuse / cut / intersect -> BooleanOutcome` | `self.context.fallback` (`Model::new()` = `AllowApproximate{0.1}` — permissive-**disclosed**, not silent) | `BooleanOutcome` quality | `ExactOnlyUnattainable` when policy is `ExactOnly` | Via `boolean_with_context` transaction |
| `Model::boolean_journaled -> JournaledBoolean` | No fallback (direct `boolean_journaled_with_operation` → GFA) | Journal `op` + construction draft; mesh has no history so never routed there | GFA typed | Scope + geometry + history in one transaction |

`OperationContext::new()` still defaults to `AllowApproximate{0.1}`
(`DEFAULT_APPROXIMATION_BUDGET`), preserving legacy `*_with_context` behavior
and the `quality_context(false, None…​) == OperationContext::new()` pin. The
default was **not** changed; exactness is enforced per bare-handle entry point
by pinning to `ExactOnly`, which is why `booleanWithQuality` without
`exactOnly` remains the permissive-disclosed opt-in.

### Direct WASM (`crates/wasm/src/bindings/booleans.rs`, `evolution.rs`)

| Binding | Native dispatch | Fallback | Return | Refusal code |
|---|---|---|---|---|
| `fuse` / `cut` / `intersect` | `boolean()` | `ExactOnly` | bare `u32` | `quality_refused` / `exact_only_unattainable` |
| `fuseAll` | `compound_ops::fuse_solids` | `ExactOnly` | bare `u32` | same |
| `fuseWithOptions` / `cutWithOptions` / `intersectWithOptions` | `boolean_with_options` | `ExactOnly` | bare `u32` | same |
| `fuseWithEvolution` / `cutWithEvolution` / `intersectWithEvolution` | `boolean_with_evolution` | `ExactOnly` | `{"solid","evolution"}` JSON | same |
| `fuseWithEntityEvolution` / `cutWithEntityEvolution` / `intersectWithEntityEvolution` | `boolean_with_entity_evolution` | No fallback | `{"solid","evolution"}` JSON | GFA typed |
| `fuseDetailed` / `cutDetailed` / `intersectDetailed` | `boolean()` via `binary_boolean_detailed_impl` | `ExactOnly` | `SolidOperationDetailedResult` typed data (never throws on refusal) | `invalid_handle` / `operation_failed` with `operation` detail; direct == `executeBatchV2` code/category |
| `booleanRegions` / `booleanCompoundRegions` | `boolean_regions` / `boolean_compound_regions` | No fallback | compound `u32` | invalid/unsupported/empty typed |
| `booleanWithQuality` | `boolean_with_context` via `quality_context` | Permissive-disclosed unless `exactOnly:true` | `{solid, quality, deflection?}` | `exact_only_unattainable` under `exactOnly`; invalid budgets name the argument |
| `booleanWithCancellation` | `boolean_with_context` + token | Same as above | `CancellableBooleanResult` (`completed` vs typed `cancelled`) | `operation_cancelled`; transactional, no partial topology |
| `compoundCut` | `boolean::compound_cut` with `BooleanOptions::default()` | `ExactOnly` **after this audit** | bare `u32` | `exact_only_unattainable` (was silent approximate — fixed) |
| `meshBoolean` | `mesh_boolean::mesh_boolean` (raw triangles) | Explicit mesh path (not a B-Rep fallback) | `JsMesh` | validation typed |
| `detectCoincidentFaces` | `algo::diagnostic::detect_coincident_faces` (read-only) | N/A | JSON array `[{faceA,faceB,sameOrientation,aabbOverlap}]` | invalid handles typed |

Wire formats and defaults are unchanged. `quality_context` is shared by direct
and batch `booleanWithQuality` so budgets cannot strand one path.

### Batch (`executeBatch` / `executeBatchV2`)

`fuse`, `cut`, `intersect`, `booleanRegions`, `booleanCompoundRegions`,
`booleanWithQuality` (with `exactOnly` + six SSI budgets),
`fuseWithOptions`/`cutWithOptions`/`intersectWithOptions`,
`fuseWithEvolution`/`cutWithEvolution`/`intersectWithEvolution`,
`compoundCut`, `fuseAll`, `detectCoincidentFaces` dispatch to the same native
functions as their direct twins (see `batch.rs` arms). `fuseDetailed` family
has no batch arm by design — `executeBatchV2` already returns typed
`{ok,error}` envelopes, and `detailed_binary_boolean_refusals_match_batch_v2_codes`
pins direct-detailed == batch-V2 code/category. `meshBoolean` has no batch arm.

## Healing-boundary disclosure (no algorithm change)

- **Exact GFA path** (`boolean_with_context_impl`): best-effort
  `remove_degenerate_edges`, `remove_wire_spurs`, conditional
  `unify_coincident_boundary_edges`, conditional `unify_faces` (≤3 passes),
  then Euler / closed-manifold / operand-representation / bounds /
  `validate_boolean_result` gates. Repairs are internal validity gates, not
  caller-visible counts; the disclosed contract is exact success vs typed
  refusal with rollback.
- **Mesh fallback** (`run_mesh_fallback`): `remove_degenerate_edges`,
  optional `unify_faces` (≤3), `enforce_manifold_shell`, `is_closed_manifold`
  gate. Reachable only through a permissive context; disclosure is
  `Approximate{deflection}` plus the `remus_approx` warn log — per-repair
  ledgers are **not** returned in `BooleanOutcome` (recorded here as a known
  boundary, owned by B1/healing work, not expanded by this audit).
- **`apply_boolean_options`** (`unify_faces`, `heal_after_boolean=false` by
  default): `unify_same_domain` is validated and rolled back to the valid
  unsimplified result on failure; `heal_after_boolean` runs the full
  `heal_solid` pass and fails/rolls back the whole operation on healing or
  post-heal validation failure. Verified-healing reports (`RepairReport`,
  `ConfiguredRepairReport` with before/healing/after + independent L2
  `check_after`) are the healing-side disclosure surface; Boolean entry points
  do not re-emit them.

## Defect fixed in this audit (local, contract-scope)

**`compound_cut` silently accepted the mesh fallback.** Native
`compound_cut` built its context from `BooleanOptions` (`AllowApproximate`
budget `0.1`) without the `ExactOnly` pin every sibling bare-handle entry
point applies, and discarded `used_fallback` — a Cut requiring the fallback
returned a bare handle with no quality.

- Reproduced natively on two cylinders r5 h10 near-tangent (centers 9.999
  apart, radii 5+5): pre-fix `compound_cut` succeeded where `boolean(Cut)`
  refuses `ExactOnlyUnattainable` and `boolean_with_context` reports
  `Approximate{0.1}`. (The B21 tangent-boss block-cylinder pair was checked
  first: its Fuse needs the fallback but its Cut succeeds exactly, so it
  cannot witness a Cut-policy gap — the cylinder-cylinder tangent is the
  Cut witness.)
- Fix (`crates/operations/src/boolean/mod.rs`): validate the option-derived
  context, then pin to `FallbackPolicy::ExactOnly` — the same two lines as
  `boolean_with_options`. Docs on native `compound_cut` and WASM `compoundCut`
  now state exact-only. No wire-format, default-budget, or tolerance change.
- Stale comments corrected without behavior change:
  `booleanWithQuality` docs no longer claim plain `fuse`/`cut`/`intersect`
  silently accept fallback (true before B21, false since B21 at both `76dbffd`
  and the baseline); `run_mesh_fallback` comment no longer names plain
  `boolean()` as a silent-fallback caller.

## Coverage (reuse first, gaps only)

Reused: `fallback_policy.rs` (exact success, fixture-needs-fallback guard,
plain/options/evolution refusal + rollback, permissive disclosed outcome,
`fuse_solids` refusal), WASM `plain_batch_booleans_refuse…`,
`tangent_boss_batch_contract…`, detailed direct-vs-batch-V2 pins, invalid-handle
and SSI-budget pins.

Added (genuine gaps only):

- Native `compound_cut_refuses_like_the_handle_entry_point`
  (`crates/operations/tests/fallback_policy.rs`): cylinder-cylinder
  near-tangent single-tool cut refuses `ExactOnlyUnattainable` with unchanged
  live counts and intact operands (plus a fixture guard proving exact Cut
  still needs the fallback); exact-box cut still succeeds with closed-form
  volume 7.0.
- Batch `plain_batch_booleans_refuse…` extended with `fuseAll` on the
  tangent-boss Fuse fixture, plus new `compound_cut_batch_refuses…` on the
  cylinder-cylinder tangent Cut fixture (refusal + disclosed
  `booleanWithQuality` approximation on the same pair).

Mutation check: flipping the new `compound_cut` pin back to permissive (and,
separately, restoring the stale `booleanWithQuality` sentence) fails the new
native refusal test and the extended batch pin respectively; the injected
changes were removed after the demonstration run.

Real WASM execution: `booleanWithQuality` / batch shape assertions run through
`execute_batch` / `execute_batch_v2` natively (the documented contract-test
pattern — `JsError` cannot be constructed off-wasm), with the same dispatch
functions the JS bindings call; no new JS-only shape was introduced, so no
separate browser run was required beyond the existing harness.

## Remaining qualification (not claimed)

- General curved/tangent Boolean coverage stays with its geometry owners
  (B7/B8/M5/5.7b, B26/B32–B44/B47/B49); one tangent-boss fixture does not
  qualify the family.
- Installed-package (O1.5/O4.2) and independent-oracle (B26/B19/B6/B10/O1.2)
  gates are untouched; this audit adds no ranking or schedule claim.
- O4.7 typed-result migration for the remaining 13 JSON-string returns and
  non-Boolean families stays open.

## Files changed

- `crates/operations/src/boolean/mod.rs` — `compound_cut` exact-only pin +
  docs; `run_mesh_fallback` comment correction.
- `crates/wasm/src/bindings/booleans.rs` — stale `booleanWithQuality` comment
  correction; `compoundCut` exact-only docs.
- `crates/operations/tests/fallback_policy.rs` — native `compound_cut`
  refusal/rollback gap test.
- `crates/wasm/src/bindings/batch.rs` — batch refusal pin extended with
  `fuseAll` + new `compoundCut` gap test (tests only).
- `docs/kernel-maturity/boolean-contract-audit-2026-09-21.md` — this inventory
  (new).
