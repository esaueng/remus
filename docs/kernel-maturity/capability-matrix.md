# Capability matrix

This document defines the qualification structure for every public modeling
operation: the axes a family is classified across, the vocabulary for what a
cell may claim, and the current mapped state of each operation family. It is
the promotion authority for feature labels.

## Promotion authority

`docs/production-readiness/stability-matrix.md` is the ledger of the *current*
audited label dispositions. This matrix defines the *evidence* a label change
requires. The rule:

1. A README feature label changes only when the operation family's declared
   capability cells are all Qualified, Partial-with-declared-bounds, or
   Unsupported-typed (no Unqualified and no Unsupported-untyped cells remain).
2. The stability-matrix row is updated in the same change as the label.
3. This audit-stage document changes no README label itself.

## Cell states

Every cell of every family matrix carries exactly one state:

| State | Meaning |
| --- | --- |
| **Qualified** | Active tests cover the cell; postconditions of the [operation contract](operation-contract.md) hold; behavior is deterministic native and WASM. |
| **Partial** | The cell works on a declared sub-domain with a documented, typed refusal outside it. The boundary itself is tested from both sides. |
| **Unqualified** | The code path exists but the cell has no qualifying evidence. It may work; nothing proves it. Default state for all cells until mapped. |
| **Unsupported-typed** | The kernel refuses the cell with a stable typed error ([failure-taxonomy.md](failure-taxonomy.md)) and a test pins the refusal. |
| **Unsupported-untyped** | The kernel fails the cell in an undeclared way (wrong result, generic error, silent fallback). These are defects of contract, tracked as gaps. |

A cell may additionally be annotated **Approximate** when success is delivered
under an explicit approximation policy rather than exactly
(see the fallback policy in [operation-contract.md](operation-contract.md)).

## Classification axes

### Geometric families (booleans, intersections, blends, offsets, sweeps, sectioning)

These families are classified across the full grid:

- **Geometry type** — plane, cylinder, cone, sphere, torus, NURBS, and mixed
  pairs. Curve-level cells additionally include line, circle, ellipse,
  hyperbola, parabola, and NURBS curves (the full `EdgeCurve` set — conics are
  in scope, not just circle/ellipse).
- **Topological relationship** — disjoint, crossing, nested, tangent,
  coincident, near-coincident, seam-crossing, singular (pole/apex), sliver,
  degenerate.
- **Body type** — solid, sheet, wire, compound, cavity-bearing solid, and
  later general body.
- **Scale** — at least three model scales relative to the configured
  tolerance (e.g. 1e-3, 1, 1e3 in the kernel's millimetre convention), with
  the tolerance scaled correspondingly.
- **Expected result** — exact success, approximate success, explicit
  unsupported error, invalid-input error, resource-limit error, or
  nonconvergence error.

### Non-geometric families

The full grid does not fit every public operation. These families use
family-specific axes, declared here so "covers every public operation" is
achievable rather than aspirational:

- **Measurement / classification / distance** — geometry type × body type
  (including cavity-bearing) × scale; expected result is a value with error
  bound, not topology.
- **Tessellation** — surface type × deflection regime × body type; expected
  result is a watertight mesh within deflection, or typed failure.
- **I/O formats** — entity coverage × malformed-input class × resource-limit
  class × round-trip fidelity.
- **Validation / healing** — defect class × severity × repair policy;
  expected result is a report or an explicit, fully-disclosed repair.
- **Sketch (GCS)** — constraint type × system state (well-, over-,
  under-constrained, redundant, inconsistent) × scale.
- **Evolution / naming** — operation family × event type (preserved,
  modified, generated, split, merged, deleted, unresolved).
- **Assemblies, feature recognition, defeaturing, projection** —
  family-specific axes to be declared when each family is first worked on;
  until then every cell is Unqualified.

## Operation family inventory and current mapped state

Every public operation belongs to exactly one family below. "Ledger row"
references `docs/production-readiness/stability-matrix.md`. The current-state
column is a summary of that ledger plus the known code-level limitations; it
does not itself promote or demote anything.

### Booleans (union, cut, intersect, batch fuse-all)

- Ledger rows: "Plane/cylinder/cone/sphere/NURBS union, cut, intersect"
  (Guarded), "Batch fuse-all" (Blocked), "Torus booleans" (Beta).
- Implementation note: the kernel has **three** boolean paths — the GFA
  pave/block pipeline in `crates/algo` (authoritative), the older path in
  `crates/operations/src/boolean/`, and the bounded mesh fallback in
  `crates/operations/src/mesh_boolean.rs`. Qualification targets the GFA
  pipeline; the others are measured against it and either proven equivalent
  (as fast paths under the contract) or retired.
- Known qualified evidence: cavity semantics regressions, fail-closed bounded
  mesh fallback, 64-cut determinism gate, analytic cylinder-crossing-plane
  overlap sweep with operand-loss acceptance gate, and exact coaxial blind-bore
  cuts across wall/radius 0.01–0.10 and 1e-3–1e3 scale
  (`regress_thin_wall_coaxial_bore.rs`; six canonical edges, closed-form
  volume, closed B-Rep, and watertight indexed meshes). The tangent-boss
  witness is also pinned at d/r +0.01, 0, and -0.01 over 1e-3/1/1e3 scale: the
  1e-3 and unit tangent cells are exact, while the 1e3 tangent cell refuses
  under `ExactOnly` and succeeds only with disclosed approximation quality.
  Native, direct WASM, batch-v2, and versioned repro coverage all retain the
  operand. The OpenZCAD cross-drilled shaft sequence is qualified at
  bore/shaft radius ratios 1, 2/3, and 1/3 and scales 0.1, 1, and 10:
  independent orthogonal-cylinder volume oracles match, and coarse/fine
  display meshes are non-empty, closed, and manifold through the deterministic
  WASM batch path and a versioned replay bundle. The CI-ratcheted
  `approx_census` additionally exposes exact/fallback/error path and result
  face-count drift across its representative operation matrix; it is a drift
  detector, not by itself qualification evidence for a cell.
- Known Unsupported-untyped / Partial cells: exact plane/cylinder tangency is
  not generally qualified beyond those witnesses; sliver crossings (~1e-5 to
  0.05 mm on r = 10) fall over to approximate; general torus pairs limited;
  seam-crossing, nested-shell, sheet-solid, and multi-body General Fuse cells
  Unqualified.

### Intersections (curve-curve, curve-surface, surface-surface)

- Ledger rows: "Analytic intersections", "Surface-surface intersection",
  "Curve-curve intersection" (all Stable, evidence pending).
- Qualified result model: `remus_math::intersect` (contact kind ×
  quality × source method; `complete` marks certified coverage). The
  matrix harness (`crates/math/tests/intersection_matrix.rs`) generates
  configuration × scale cells with on-surface invariants and
  scale-invariant classification.
- **Qualified cells** (closed-form classification incl. tangency and
  coincidence, scale-invariant): plane–plane, plane–sphere,
  plane–cylinder, sphere–sphere, parallel-axis cylinder–cylinder, coaxial
  sphere–cylinder.
- Known gaps: remaining surface pairs delegate to the legacy path and are
  wrapped as Unclassified/incomplete (declared, not silent); curve-curve
  and curve-surface qualification pending. NURBS SSI consumes caller-owned
  march/queue/segment/branch, coupled-Newton, and recursive seed-subdivision
  budgets, and is cooperatively cancellable through seed discovery, Newton
  refinement, and marching via `OperationContext`; its scheduled
  `nurbs_surface` fuzzer validates bounded rational patch
  construction/evaluation and plane-section output against an independent
  plane oracle. Parameter-space budgets remain incomplete. Conic curve cells
  (hyperbola, parabola) Unqualified;
  periodic seam parameter reporting and pole cells Unqualified.

### Blends (fillet, chamfer, blend resize/removal)

- Ledger rows: "Fillet, chamfer" (Stable/Experimental, guarded), "Resize/
  remove analytic blend band" (Experimental, guarded).
- Known Qualified/Partial evidence: planar line-edge manifold builders;
  exact toroidal cylinder-cap rim assembler across `0 < f < r_c` with
  closed-form volume/area verification; typed `RadiusTooLarge` refusals;
  blind-hole floor rim deliberately capped at `r_c/2`.
- Fail-closed contract (Qualified, every public entry point): direct WASM
  bindings, `executeBatch`/`executeBatchV2`, journaled wrappers, and the
  legacy v1 Rust APIs (`fillet`, `fillet_rolling_ball`, `fillet_variable`,
  flat-bevel `chamfer`) are all transactional (a failed call leaves the
  arena, journal, attributes, and input solid byte-identical), never answer
  with the input handle or a silently reduced selection, validate the result
  against the input baseline, and bound the volume change to what the
  requested blend can physically produce. Refusals carry the stable
  `blend_failure_code` vocabulary (prefixed message on direct bindings,
  `kernelCode` detail on the structured batch contract). Repro bundle
  `fillet-variable-fail-closed` plus the `regress_fillet_fail_closed` and
  `fillet_fail_closed_tests` suites pin the contract from both sides across
  a 1e-3/1/1e3 scale sweep.
- Known gaps: closed-rim chamfers and curved assembly experimental and
  fail-closed; variable radius on curved domains, setbacks, multi-edge
  corners, G2 profiles, overflow handling Unqualified or absent.

### Offset, shell, thicken

- Ledger rows: "Shell" (guarded), "Offset, thicken, mirror, pattern"
  (guarded).
- Known Qualified/Partial evidence: closed-topology + L3 validation on shell;
  cavity-preserving offsets with shell-separation precondition; rolling-ball
  arc joints on convex polyhedra verified against the Minkowski/Steiner
  closed form across 1e3–1e-3 scale; typed refusals for each unsupported arc
  configuration.
- Known Unsupported-typed cells (kept typed, target of future work): global
  self-intersection removal; NURBS-NURBS 3D intersection in the offset path;
  excluded faces on cavity solids.

### Direct edits (push/pull and move face)

- Ledger row: "Push/pull face" (Stable for the declared domain, guarded).
- **Qualified cell:** moving either untrimmed cap of a three-face analytic
  cylinder is exact for positive and negative distances, both cap sides,
  rotated and translated frames, and 1e-3/1/1e3 model scales. The height-collapse
  boundary is a typed invalid-input error. Native closed-form volume and
  topology oracles plus the versioned WASM `push-pull-cylinder-top-cap` repro
  bundle pin the public contract.
- Known Partial/Unqualified cells: decorated cylindrical solids and general
  planar faces retain the validated boolean/re-limitation paths; generalized
  curved-face re-limitation and direct-edit evolution remain roadmap work.

### Sweeps (extrude, revolve, sweep, loft, pipe, helix)

- Ledger rows: "Extrude", "Revolve, sweep, loft, pipe", "Helical sweep"
  (Stable, blocked), "Non-planar profiles" (Beta).
- Known gaps: degenerate/cavity matrices, topology and nonconvergence
  budgets, termination/performance evidence incomplete; guide rails, laws,
  periodic lofts, continuity options largely absent.

### Sectioning and splitting

- Ledger row: "Cross-section, split by plane" (Stable, blocked). Cavity and
  degeneracy matrices incomplete.

### Construction (fill, sew, untrim), primitives, transforms, patterns

- Ledger rows: "Coons fill, sew, untrim" (blocked); "Primitives" (blocked);
  mirror/pattern under the offset row (guarded).
- Pattern qualification: linear, circular, and grid patterns now preflight
  pairwise material overlap using exact-only intersections and a
  scale-relative volume floor. Material overlap is a typed, transactional
  `pattern_instances_overlap` refusal at native and WASM batch boundaries;
  touching and disjoint instances preserve copy-derived face provenance.
  Native tests pin closed-form box intersection volume, rollback, contact, and
  1e-3/1/1e3 scale behavior; the versioned
  `pattern-overlap-typed-refusal` bundle pins deterministic WASM behavior.
- Known gaps: native/WASM invalid-input, scale, and postcondition matrices
  incomplete outside the qualified pattern cells; exact fusing of overlapping
  pattern instances and provenance through that fuse remain unimplemented;
  convex hull / Minkowski degenerate coverage incomplete.

### Measurement, classification, distance

- Ledger rows: "Bounding box, area, center of mass", "Distance and
  classification" (evidence pending). Inner-shell regressions pass;
  cross-drilled volume now has a WASM ratio/scale matrix against independent
  orthogonal-cylinder oracles. Planar face area is exact and independent of
  caller deflection when every boundary is made from lines, circles, or
  parabolas; closed-form scale and circular-hole oracles cover native, direct
  WASM, and batch WASM paths. Ellipse, hyperbola, and NURBS planar boundaries,
  general curved-face area, exact curved-body volume, and the remaining
  curved-cavity and scale cells remain incomplete.

### Tessellation

- Ledger row: "Adaptive/CDT/analytic optimization". Face-failure abort is
  qualified. Cross-drilled display tessellation is qualified at two relative
  deflections, three bore ratios, and scales 0.1 through 10; `meshQuality`
  accepts the render angular tolerance and cannot label an empty mesh
  watertight. Broader scale/performance cells remain Unqualified.

### Validation and healing

- Ledger row: "Healing, sewing, validation" (blocked: permissive healing can
  mask invalid result semantics — the family's central Unsupported-untyped
  cell, addressed by the healing-disclosure rules in
  [operation-contract.md](operation-contract.md)).

### I/O (STEP, IGES, mesh formats)

- Ledger rows: "STEP" (guarded), "STL, 3MF, OBJ, PLY, glTF" (guarded),
  "IGES" (Experimental).
- Qualified evidence: shared byte/entity import limits with regressions;
  malformed-input panics fixed (see `docs/production-readiness/audit.md`);
  CAx-IF v4.6 product/per-solid volume, surface-area, and centroid declarations
  round-trip against independent analytic oracles, with opt-in recomputation,
  stable deviation diagnostics, and transactional malformed-property refusal
  ([STEP conformance](../production-readiness/step-conformance.md)).
- Known gaps: inner-shell export, attribute round trips
  (`docs/design/deferred-e3b-step-names-and-colors.md`), periodic seam and
  p-curve round trips, deterministic entity ordering, AP242 schema output,
  and validation properties for independent non-solid geometry.

### Sketch (GCS)

- Ledger row: "DogLeg solver" (evidence pending): nonconvergence budget and
  degeneracy matrix incomplete.

### Evolution and naming

- Ledger row: "Face provenance" (Beta). Construction-derived face provenance
  covers booleans, walking/planar blend builders, and patterns; offset,
  shell, draft, split, defeature, and direct edits produce none.
  **Edge and vertex history** (Issue 12): `gfa::boolean_with_entity_evolution`
  returns construction-derived edge events (Preserved / Modified via the
  splitter's source-edge chain and pave blocks / Generated via FF
  section-origin records / honest Unresolved for the assembly rebuild paths,
  pinned as a minority) and vertex events (Preserved / Created via the copy
  maps). Operations/WASM surfacing, the assembly rebuild records, and the
  lineage graph remain queued. Persistent naming does not exist yet; arena
  indices are the only handles (and are explicitly not persistent names).
  Design: `docs/design/rfc-0003-persistent-naming.md` (journal + resolver
  over the evolution events, staged; Issue 13).
  **Evolution journal** (RFC 0003 Stage 1): `remus_topology::journal` is
  the append-only per-topology history — journal-local ordinals with a
  live index (entries never hold arena indices), `OpId`s and ordinals
  high-water preserved across restores (never reused), entries truncated
  with checkpoint rollback. Ingestion: `operations::journal_ops`
  (`boolean_journaled` journals full Issue-12 construction history;
  `record_face_evolution` journals any `EvolutionMap` producer, e.g. v2
  blends; `record_barrier_over_solid` journals the explicit barrier for
  operations without evolution records). Gap coverage is structural:
  `Topology` counts mutations, and any mutation no entry accounts for
  triggers a synthetic global barrier at the next `journal_begin` — no
  operation can be silently absent from history.
  **Persistent references** (RFC 0003 Stage 2): `remus_topology::naming`
  — `PersistentRef` (OperationOutput / LineageOf anchors + SurfaceType /
  CurveType discriminators) resolves against the journal by chasing
  identity claims to the present model. Typed outcomes (Bound / BoundMany
  for splits / Dangling / UnresolvedAcrossOperation / UnknownOperation /
  NoMatch, each with a pinned `ref_*` diagnostic code), disclosed
  Construction-vs-Inferred provenance, entry scopes so unrelated solids'
  references survive (in-scope-unclaimed severs, out-of-scope carries).
  **Signature tier** (RFC 0003 Stage 3): `EntitySignature` — quantized
  analytic parameters (tolerance-derived quantum, never raw float
  equality), structural adjacency counts, endpoint vertex signatures for
  edges; `Anchor::Signature` resolves against the current model only,
  always `Inferred`, `Ambiguous` on several matches (never first-match) —
  the recovery path for imported or severed references.
  **Attribute integration** (RFC 0003 Stage 4):
  `Topology::propagate_attributes_for_op` — per-event, journal-driven
  face-attribute propagation (splits keep names unchanged, merges carry
  only agreement, generated/unresolved stay bare, inference is an
  explicit opt-in); `naming::resolve_face_attributes` reads attributes
  through a `PersistentRef` with every non-binding resolution a typed
  `ref_*` error.
  **Serialization** (RFC 0003 Stage 5): the v2 arena document carries the
  journal and attributes additively (absent when empty; byte-identical
  legacy output), with validated snapshots
  (`journal_snapshot_invalid`), local-index remapping, `UNMAPPED`
  placeholders for out-of-document entities, and tick re-derivation so a
  clean load is not a gap; `io::naming_io` serializes references as
  versioned context-free JSON. References resolve identically across
  save/load (pinned) — a naming regression is a replayable fixture.
  **WASM reference API**: `bindings/naming.rs` — `fuseJournaled` /
  `cutJournaled` / `intersectJournaled`, `journalBarrier`,
  `propagateAttributesForOp`, `resolveOperationOutput`,
  `setFaceName`/`getFaceName`, and (io feature) `makeOperationOutputRef`
  / `captureSignatureRef` / `addRefDiscriminator` / `resolveRef` /
  `resolveRefFaceAttributes`; all with `executeBatch` companions and
  contract tests. Resolution outcomes are data (`status` JSON), never
  exceptions. RFC 0003 is fully surfaced.
  **Evolution surfacing**: `operations::boolean::boolean_with_entity_evolution`
  (L3 surface of the Issue-12 entry point, re-exported event types);
  one-call journaled wrappers `journal_ops::{fillet_journaled,
  chamfer_journaled, linear_pattern_journaled}` — the journal is now
  populated by every construction-evolution producer (booleans, v2
  blends, patterns) per the RFC 0003 Stage 1 goal; WASM
  `fuseWithEntityEvolution` (+cut/intersect) exposing the full
  vertex/edge/face event payload as stable JSON, `filletJournaled` /
  `chamferJournaled` / `linearPatternJournaled`, and the read-only
  `journalSummary`, all with executeBatch companions and contract
  tests.
  **Assembly-rebuild lineage records**: every GFA result-assembly path
  that rebuilds edges records construction lineage — the perform-phase
  vertex-merge wire rebuild, welds, and the collinear line/arc splits —
  and `build_result_with_origins` returns the complete log. A cube
  fuse's edge history is total construction fact (pinned: zero
  unresolved). Remaining evolution queue: real evolution for the
  declared-gap operations (offset, shell, draft, split, defeature —
  journaled as barriers until each grows records) and cap-synthesis
  edges (absent from planar boolean fixtures).

### Draft

- Ledger row: "Draft" (Stable, planar faces; promoted 2026-08-21).
- Declared axes: face selection (single wall, multi-wall, holed neighbours) ×
  pull/neutral placement × angle sign and near-zero boundary × scale
  (1e-3/1/1e3) × body type (plain, holed, cavity-bearing).
- Qualified cells: planar targets across the axes above, with closed-form
  volume oracles and native + WASM-batch determinism
  (`crates/operations/tests/qualify_draft.rs`, wasm `qualify_ops_tests`).
- Unsupported-typed cells (both-sides tested): zero angle, non-planar target,
  drafted face carrying a hole rim, cavity-bearing solid, foreign face,
  parting-plane target, moved inner wires, curved re-trim neighbours.

### Defeaturing

- Ledger row: "Planar face removal" (Stable, declared domain; promoted
  2026-08-21).
- Declared axes: feature class (through-hole, blind pocket, boss, cylindrical
  bore wall, chamfer) × heal strategy (cap, extend) × scale.
- Qualified cells: the classes above with exact restored volumes and full
  validation (`qualify_defeature.rs`); construction-derived evolution
  (`defeature_with_evolution`).
- Unsupported-typed cells: cavity solids, wounds crossing curved kept faces,
  removals leaving fewer than four faces, empty/foreign selections.

### Assemblies

- Ledger row: "Hierarchy, transforms, BOM" (Stable; promoted 2026-08-21).
- Declared axes: hierarchy depth × transform composition × instance sharing ×
  BOM/flatten determinism.
- Qualified cells: deep chains verified against direct matrix products,
  rotation+translation bounding boxes against transformed corners,
  sub-assembly nodes' own solids counted consistently by flatten and BOM,
  deterministic ordering across rebuilds (`qualify_assembly.rs`).
- Unsupported-typed cells: empty-assembly bounding box, invalid parents.
  Cycles cannot be constructed (no re-parenting API).

### Feature recognition

- Ledger row: "Holes, pockets, chamfers, fillets" (Stable, declared set;
  promoted 2026-08-21).
- Declared axes: feature type × geometry (planar/cylindrical walls,
  through/blind) × post-boolean noise.
- Qualified cells: precision (plain bodies yield no features; `FilletLike`
  never claims planar faces), recall on constructed ground truth (holes with
  diameter and through-ness, all-planar rectangular pockets, chamfers),
  deterministic output (`qualify_feature_recognition.rs`).
- Outside the declared set, absence of a claim is the declared contract.

### Projection, drafting (2D)

- Ledger rows: evidence-pending. Axes to be declared when first worked on;
  cells Unqualified.

## Known representation-level limitations (cross-family)

These are not cells of any one family; they bound what many families can
claim, and they are the first implementation targets of the program:

1. **No coedge/edge-use entity.** Face boundaries are ordered oriented-edge
   lists (`crates/topology/src/wire.rs`); p-curves are keyed by
   `(EdgeId, FaceId)` (`crates/topology/src/pcurve.rs`), so a seam edge used
   twice on one periodic face cannot carry two p-curves — the registry's
   second `set` silently overwrites the first (pinned by the
   `seam_characterization` tests in that file). Every seam-crossing cell is
   at best Partial until this lands. Design:
   `docs/design/rfc-0002-coedge-architecture.md`.
2. **No stored trim domains.** Edge domains are reconstructed from endpoint
   projections at evaluation time (`crates/topology/src/edge.rs`,
   `domain_with_endpoints`) with module-local match bands. SameParameter /
   SameRange validation cannot be stated, let alone enforced.
3. **Face-only, one-level evolution.** No vertex/edge events, no lineage
   graph, no persistent references.
4. **Partial operation-context coverage.** Public booleans carry tolerance,
   fallback policy, NURBS marching, coupled-Newton, and recursive
   seed-subdivision budgets, and cooperative cancellation explicitly.
   Generated-topology/memory budgets,
   parameter-space tolerance, determinism policy, and non-boolean operation
   families remain local or unmigrated.

## Maintenance rules

- New public operations must add their family (or extend an existing one)
  in the same PR, with all cells initially Unqualified or Unsupported-typed.
- Moving a cell to Qualified requires the evidence classes in
  [testing-strategy.md](testing-strategy.md).
- Discovering an Unsupported-untyped cell requires filing it here (or in the
  family's detailed matrix once split out) with a reproduction, in the same
  change that discovers it.
- When a family's matrix outgrows this file it moves to
  `docs/kernel-maturity/matrix/<family>.md`; this file keeps the inventory
  row and a link.
