# Kernel maturity target

This document defines what "professional-grade modeling kernel" means for this
project, and how progress toward it is measured. It is the root of the
kernel-maturity contract:

- [capability-matrix.md](capability-matrix.md) — what each operation supports,
  cell by cell, and the promotion authority for feature labels.
- [operation-contract.md](operation-contract.md) — the result, quality,
  fallback, and postcondition contract every operation converges on.
- [failure-taxonomy.md](failure-taxonomy.md) — stable failure categories and
  their relationship to the existing stable error-code registry.
- [testing-strategy.md](testing-strategy.md) — the evidence rules: what kind of
  test qualifies a capability cell, and what CI must eventually gate.

## Naming

The kernel is Remus: the repository is `remus` and the crates carry the
`remus-*` prefix. These documents say "the kernel" wherever
possible. Nothing in this contract depends on the name.

## Objective

Expand the kernel from a broad but still maturing exact B-rep kernel into a
professional-grade modeling kernel that approaches the reliability, modeling
coverage, predictable behavior, and integration quality associated with
commercial benchmark kernels.

Commercial kernels are a category benchmark, not a source implementation. This
program does not clone any external kernel's API, internal architecture,
proprietary formats, or implementation.

Progress is not measured by the number of exported functions. It is measured
by:

- qualified capability-matrix coverage,
- deterministic behavior,
- valid topology,
- controlled numerical error,
- complete diagnostics,
- persistent topology history,
- repeatable industrial-corpus results.

## Target kernel characteristics

The long-term kernel provides:

1. Exact analytic and NURBS B-rep modeling.
2. Explicit tolerant-modeling semantics.
3. Deterministic, bounded modeling operations.
4. First-class solid, sheet, wire, and eventually general-body topology.
5. Correct support for periodic surfaces, seams, poles, singularities, and
   multiple edge uses.
6. Robust curve-curve, curve-surface, and surface-surface intersections.
7. Exact-first booleans with an explicit approximation policy.
8. Persistent vertex, edge, and face history.
9. Direct face editing and reliable feature modification.
10. General filleting, chamfering, offsets, shelling, sweeps, and lofts.
11. Detailed validation and controlled healing.
12. Versioned native and WASM operation contracts.
13. Reliable STEP exchange with topology attributes.
14. Reproducible failure bundles and an industrial regression corpus.
15. Native and WASM behavioral parity.
16. Resource budgets, cancellation, and deterministic parallelism where added.
17. Optional mixed exact/faceted and non-manifold modeling after the exact
    B-rep core is qualified.

## Program priorities

**P0 — Kernel foundations and correctness:** capability and failure contracts;
reproduction/corpus infrastructure; first-class coedges; explicit curve and
p-curve trimming; unified tolerance and operation context; intersection
robustness; General Fuse and boolean robustness; transactional topology
mutation; kernel-wide structured diagnostics.

**P1 — Professional modeling behavior:** complete vertex/edge/face evolution;
persistent topological naming; general blends, offsets, shelling, sweeps, and
lofts; direct face editing; attribute propagation; broad STEP round-trip
behavior; memory compaction and session lifecycle.

**P2 — Extended kernel scope:** general/non-manifold bodies; mixed B-rep and
facet modeling; cellular topology; lattice representation; concurrent
operations; large-model scaling.

## Constraints

These are program invariants, not aspirations:

- Preserve the existing crate-layer dependency DAG
  (`scripts/check-boundaries.sh` is authoritative).
- Keep compatibility with Rust 1.88 (the declared MSRV).
- No unsafe code, panics, unwraps, expects, or unchecked numeric conversions
  in production kernel code (already enforced by workspace lints).
- All implementations are clean-room and Apache-2.0 compatible. Do not copy
  code from proprietary kernels, copyleft kernels, or the prohibited
  post-license predecessor lineage. Upstream behavior from the fork's origin
  arrives only under an explicit Apache-2.0 grant or is independently
  implemented (see `docs/production-readiness/fork-maintenance.md`).
- Do not weaken tests, increase tolerances, silently heal a result, or
  introduce a mesh fallback merely to make a failing case pass.
- No repository-wide rewrite. Work proceeds through versioned RFCs and
  incremental vertical slices with compatibility facades.
- No breaking change to the current Rust or WASM APIs without an additive
  versioned migration path.
- Raw arena indices are never persistent user-facing topology names.
- No feature is promoted from experimental or beta on one successful fixture.
  Promotion requires completion of its declared capability matrix
  (see [capability-matrix.md](capability-matrix.md), "Promotion authority").
- Planning is by dependencies and acceptance gates, not calendar estimates.

## Relationship to existing production-readiness documents

`docs/production-readiness/stability-matrix.md` records the current audited
disposition of each README feature label and remains the ledger of record for
*today's* labels. The capability matrix defined by this contract is the
*forward-looking* qualification structure. To avoid two diverging sources of
truth:

- The stability matrix continues to describe the shipped state and is updated
  when a row's evidence changes.
- The capability matrix defines what evidence a row needs; each stability
  matrix row maps to one or more capability families.
- A README label changes only when the capability matrix says its gate set is
  complete, and the stability matrix row is updated in the same change.

## Definition of done (program-wide)

The kernel is materially closer to a professional kernel when:

- Supported domains are explicitly documented and test-generated.
- Difficult configurations return correct topology or a precise stable error.
- No public operation silently changes representation quality.
- Numeric and resource behavior is bounded.
- Periodic and seam topology is represented correctly.
- Exact edge and p-curve trims are stored and validated.
- Booleans handle tangency, coincidence, slivers, cavities, and mixed analytic
  surface pairs without relying on shape-specific shortcuts.
- Every topology-producing operation reports complete or explicitly unresolved
  vertex, edge, and face evolution.
- Persistent selections survive normal model edits.
- Healing and tolerance increases are explicit.
- STEP round trips preserve the declared model contract.
- Every discovered defect becomes a permanent replayable regression.
- Native and WASM behavior remains consistent.
- Public API and serialized-data compatibility are managed through versioning.
- Feature status is based on capability-matrix evidence rather than individual
  demonstrations.
