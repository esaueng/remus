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
  later general body. RFC 0005's class/tagging/validation substrate landed in
  PR #209; sheet construction, area, typed volume refusal, and open
  tessellation entry points were implemented in PR #210, arena roots and
  spatial properties in PRs #211–#212, and STEP surface-model exchange in PR #213.
  Together they qualify the bounded unit-scale trimmed-NURBS sheet workflow.
  PR #214 additionally qualifies a crossing solid × single cylindrical-sheet
  cell: two deterministic valid regions in a Compound with a closed-form
  volume oracle and native/direct/batch WASM parity. PR #215 qualifies the
  planar solid × sheet keep-side cell: both exact inside and outside sheet
  remainders validate and preserve deterministic WASM parity, while coincident
  patches refuse typed. PR #216 qualifies transversal single-planar-face
  sheet×sheet side trims, including a six-sheet trim-and-sew solid with exact
  primitive-volume parity. PR #217 qualifies a transversal planar solid×solid
  imprint cell: every target patch survives in a validated solid with exact
  volume, construction-derived split history, persistent-reference rebinding,
  and native/direct/batch parity. PR #218 qualifies the two-solid planar
  cellular-output cells: a severing cut returns two independently valid
  400-volume regions and a disjoint fuse returns two exact regions, both as a
  deterministic Compound with total per-member construction lineage and
  direct/batch WASM parity. PR #219 additionally qualifies pairwise-disjoint
  Compound operands for member-preserving fuse, pairwise exact intersect, and
  distributed single-tool cut, with total member lineage and direct/batch WASM
  parity. PR #222 qualifies the bounded first-class Wire cell: body-level
  length, existing copy/transform, exact arena-v5 root replay, and validated
  closed-planar profile sweep with native/direct/batch WASM parity; open and
  non-planar sweep profiles are Unsupported-typed. Curved and same-domain
  imprint, intersecting Compound-member fuse, multi-tool Compound cut,
  multi-face and other surface sheet cells, broader wire geometry/scale
  matrices, and general-body cells remain Unqualified.
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
  WASM batch path and a versioned replay bundle. General-position
  equal-radius sphere×sphere fuse, cut, and intersect are exact: the result
  retains four spherical faces, matches the independent spherical-lens and
  inclusion–exclusion volumes, classifies material probes, and tessellates
  closed and manifold across three deflections, including an oblique-center
  witness. A sphere centred in and protruding through all six faces of a box
  now fuses through the exact-only path as 6 planar patches plus 10 spherical
  caps; the result is closed/manifold and matches a separately constructed
  exact intersection through inclusion–exclusion.
  Equal-radius perpendicular cylinder×cylinder is also exact for all
  three operators: intersection retains six cylinder patches bounded by eight
  authoritative ellipse arcs, matches the independent `16r³/3` Steinmetz
  oracle across radii and rigid motion, and tessellates closed and manifold at
  three deflections. The additive exact cellular boolean path also preserves a severing
  planar cut as two valid 400-volume Compound members and disjoint planar fuse
  operands as two exact members, with deterministic per-region construction
  lineage and native/direct/batch WASM parity. Pairwise-disjoint Compound
  operands are also qualified for member-preserving fuse, distributed exact
  intersect, and distributed single-tool cut; overlapping-member fuse and
  multi-tool cut refuse typed. The CI-ratcheted
  `approx_census` additionally exposes exact/fallback/error path and result
  face-count drift across its representative operation matrix; it is a drift
  detector, not by itself qualification evidence for a cell.
- Known Unsupported-untyped / Partial cells: exact plane/cylinder tangency is
  not generally qualified beyond those witnesses; sliver crossings (~1e-5 to
  0.05 mm on r = 10) fall over to approximate. The torus/box notch family is
  exact for Cut and Intersect: the complement result retains one torus and
  four planes, validates without errors, and matches a mesh-volume oracle
  within 1% through native and WASM exact-only paths. General torus pairs,
  seam-crossing, nested-shell, sheet-solid, intersecting Compound-member fuse,
  multi-tool Compound cut, and broader multi-body General Fuse cells remain
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
  sphere–cylinder, and intersecting perpendicular equal-radius
  cylinder–cylinder (two exact planar ellipse branches).
- Known gaps: remaining surface pairs delegate to the legacy path and are
  wrapped as Unclassified/incomplete (declared, not silent); curve-curve
  and curve-surface qualification pending. NURBS SSI consumes caller-owned
  march/queue/segment/branch, coupled-Newton, and recursive seed-subdivision
  budgets; all six are exposed by direct quality/cancellation and batch
  quality WASM booleans. SSI is cooperatively cancellable through seed
  discovery, Newton refinement, and marching via `OperationContext`; its
  scheduled `nurbs_surface` fuzzer validates bounded rational patch
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
  blind-hole floor rim deliberately capped at `r_c/2`; in-review PR #226
  qualifies straight-edge perpendicular-plane variable-radius walking bands:
  exact standard-law extrema, typed tolerance-collapse and caller-supplied
  local-limit boundaries, an analytic ruled-surface plus closed-form linear
  volume oracle, and sampled S-curve radius/incidence/tangency invariants.
  In-review PR #228 qualifies constant-radius closed curved-support assembly
  for coaxial cylinder/cone, cylinder/sphere, cone/cone, and the segmented
  orthogonal cylinder/cylinder rim of a cross-drilled shaft. The analytic
  cylinder/cone cell is recovered as an exact torus; other closed walks use a
  periodic degree-1 NURBS band tessellated from shared contact-edge vertices.
  The native matrix pins solid validation, zero free/non-manifold edges,
  watertight welded meshes, and B-Rep/mesh volume agreement within 2%; direct
  and batch WASM agree. The pre-existing closed-rim chamfer regression matrix
  also remains green. Cylinder/cone `resize_blend` preserves either the
  material-side branch or an exact torus carrier's proven external branch; the
  imported Shapr3D radius-4 band now rebuilds exactly at radius 3. In-review
  PR #231 qualifies same-radius planar N-way vertex blends with one common
  tangent ball and one connected material-side orientation. Three-contact
  corners emit one analytic sphere cap; higher valence uses an analytic sphere
  fan with shared internal edges and vertices. The all-edge box and four-edge
  pyramid witnesses pin closed/manifold B-Reps, watertight welded meshes, and
  every sphere/cylinder seam within angular tolerance; the four- and
  five-stripe torture cases also compare B-Rep volume with an independent mesh
  integral. Transverse planar runouts retain exact trimmed ellipse edges, and
  direct/batch WASM agree on the all-edge box. PR #232 adds a bounded
  variable-radius setback cell: straight spines at a planar 3+-way corner may
  carry different S-curve laws when their declared active endpoints meet one
  common stationary radius and tangent ball. A three-edge box witness pins the
  exact sphere, all physical setback stations, angular-tolerance G1 seams,
  closed/manifold B-Rep, watertight mesh, independent volume agreement, the
  exact census row, and direct/batch WASM parity.
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
- Known gaps: the qualified variable-radius cell is the walking band, not its
  trimmed-solid assembly; opaque custom callbacks are preserved and checked at
  every consumed station but cannot prove arbitrary between-sample behavior.
  Curved-support qualification is limited to the closed analytic cells above;
  open and non-coaxial curved assembly, curved-support chamfers, variable
  radius on curved domains, alternating-material-side vertex fans, nonplanar
  corners, setbacks outside the stationary common-ball planar cell, general
  mixed-radius junction surfaces, G2 profiles, and overflow handling remain
  Unqualified or absent and fail closed.

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
- In-review PR #222 adds a Partial Wire-profile cell: a validated closed
  planar polygonal Wire produces a validation-gated solid without aliasing the
  input, with exact prism-volume native/direct/batch WASM oracles. Open and
  non-planar Wire profiles are Unsupported-typed and rollback exactly.
- Known gaps: degenerate/cavity matrices, topology and nonconvergence
  budgets, termination/performance evidence incomplete; guide rails, laws,
  periodic lofts, continuity options, and broader wire curve/scale matrices
  largely absent.

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
  curved-cavity and scale cells remain incomplete. In-review PR #210 adds a
  unit-scale trimmed NURBS sheet-area witness through native/direct entry
  points, a planar batch contract, and pinned `body_class_measure_mismatch`
  failures for sheet volume and non-sheet area. PR #212 adds body-class-checked
  bounding box and center-of-area, including an offset-hole centroid oracle,
  through direct and batch WASM. These bounded sheet cells are qualified in
  review. PR #222 routes Wire length through the body-level measurement
  contract and pins exact native/direct/batch perimeter agreement; the broader
  geometry/scale matrix remains Unqualified.

### Tessellation

- Ledger row: "Adaptive/CDT/analytic optimization". Face-failure abort is
  qualified. Cross-drilled display tessellation is qualified at two relative
  deflections, three bore ratios, and scales 0.1 through 10; `meshQuality`
  accepts the render angular tolerance and cannot label an empty mesh
  watertight. In-review PR #210 adds deterministic open sheet tessellation for
  a trimmed NURBS patch and pins that solid-only proximity welding cannot erase
  an intentional triangular hole smaller than the requested deflection.
  PR #213 carries that patch through native and direct/batch WASM STEP exchange.
  The unit-scale patch workflow is qualified in review; broader sheet
  geometry/scale/parity and performance cells remain Unqualified.

### Validation and healing

- Ledger row: "Healing, sewing, validation" (blocked: permissive healing can
  mask invalid result semantics — the family's central Unsupported-untyped
  cell, addressed by the healing-disclosure rules in
  [operation-contract.md](operation-contract.md)). PR #209's in-review sheet
  profile reports free boundaries as warnings while retaining manifold and
  orientation errors; PR #210 validates transactionally before a constructed
  sheet is committed and tests rollback for disconnected faces. This evidence
  does not remove the family-wide healing blocker.

### I/O (STEP, IGES, mesh formats)

- Ledger rows: "STEP" (guarded), "STL, 3MF, OBJ, PLY, glTF" (guarded),
  "IGES" (Experimental).
- Qualified evidence: shared byte/entity import limits with regressions;
  malformed-input panics fixed (see `docs/production-readiness/audit.md`);
  CAx-IF v4.6 product/per-solid volume, surface-area, and centroid declarations
  round-trip against independent analytic oracles, with opt-in recomputation,
  stable deviation diagnostics, and transactional malformed-property refusal
  ([STEP conformance](../production-readiness/step-conformance.md)). Physical
  Loop/Coedge authority round-trips both positioned periodic-seam branches and
  winding counts. The external analytic fillet fixture retains all 48 pcurves
  through byte-identical write/read/write, and malformed count/endpoint
  authority rolls the import back. PR #213 adds deterministic first-class
  sheet exchange as `SHELL_BASED_SURFACE_MODEL` over open or closed shells,
  including a trimmed bilinear NURBS patch through native, direct WASM, and
  batch WASM paths; wrong-class and malformed imports fail transactionally.
  PR #222 adds standalone Wire roots in arena v5 with exact reserialization,
  ordered duplicate-root preservation, resource limits, transactional corrupt
  input refusal, native/direct WASM parity, v1–v4 readers, and frozen v3/v4
  writer bytes.
- Known gaps: attribute round trips
  (`docs/design/deferred-e3b-step-names-and-colors.md`), AP242 schema output,
  general SameParameter proofs beyond the certified curve/surface matrix, and
  validation properties for independent non-solid geometry (currently a typed
  writer refusal).

### Sketch (GCS)

- Ledger row: "DogLeg solver" (evidence pending): nonconvergence budget and
  degeneracy matrix incomplete.

### Evolution and naming

- Ledger row: "Face provenance" (Beta). Construction-derived face provenance
  covers booleans, walking/planar blend builders, patterns, draft, split,
  defeature, shell, and default intersection-joint V2 offsets. Direct edits
  produce none. Arc-joint offsets and offsets followed by self-intersection
  removal do not expose a face map: those variants may synthesize or replace
  faces after the one-to-one offset construction and fail closed rather than
  publishing stale provenance.
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
  `operations::boolean::boolean_regions` returns the same construction record
  partitioned per independently valid Compound member and refuses any
  `Unresolved` edge;
  one-call journaled wrappers `journal_ops::{fillet_journaled,
  chamfer_journaled, linear_pattern_journaled, offset_journaled}` — the
  journal is populated by the construction-evolution producers (booleans,
  v2 blends, patterns, default V2 offsets) per the RFC 0003 Stage 1 goal; WASM
  `fuseWithEntityEvolution` (+cut/intersect) exposing the full
  vertex/edge/face event payload as stable JSON, `filletJournaled` /
  `chamferJournaled` / `linearPatternJournaled` / `offsetJournaled`, and the
  read-only `journalSummary`, all with executeBatch companions and contract
  tests.
  **Assembly-rebuild lineage records**: every GFA result-assembly path
  that rebuilds edges records construction lineage — the perform-phase
  vertex-merge wire rebuild, welds, and the collinear line/arc splits —
  and `build_result_with_origins` returns the complete log. A cube
  fuse's edge history is total construction fact (pinned: zero
  unresolved). Remaining evolution queue: direct-edit face evolution, richer
  provenance for arc-joint/self-intersection-removal offsets, and
  cap-synthesis edges (absent from planar boolean fixtures).

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

1. **Per-use authority is landed, with a compatibility facade.** Physical
   Loop/Coedge entities own boundary order, pcurves, and periodic winding;
   whole-topology validation refuses dangling/reused ownership and partial
   seams. Read-only Face/Wire access remains for compatibility until its
   measured no-consumer and one-release deletion gate is met.
2. **Stored trim domains are landed.** The 132-site production reader ratchet
   is at zero and every measured result writer preserves explicit non-Line
   ranges. Strict SameParameter/SameRange validation is exhaustive only for
   its certified curve/surface combinations; unsupported combinations refuse
   with stable capability diagnostics rather than reconstructing authority.
3. **Evolution is construction-derived where declared, not yet per-use.**
   Boolean vertex/edge/face events, a lineage journal, and persistent
   references exist. Coedge-use evolution and complete records for every
   modifier remain queued; those gaps stay disclosed rather than inferred.
4. **Partial operation-context coverage.** Public booleans expose and carry
   fallback policy, all six NURBS SSI work budgets, and cooperative
   cancellation explicitly, alongside the context's tolerance.
   Generated-topology/memory budgets,
   parameter-space tolerance, determinism policy, and non-boolean operation
   families remain local or unmigrated.
5. **Per-entity tolerance is predicate-partial, not yet result-qualified.**
   Vertex balls and edge tubes are validated, serialized, journal-recordable,
   and consumed by VV/VE/EE pave predicates plus SameParameter/SameRange
   validation. FF/EF acceptance, result-tolerance growth, builder assembly,
   import/sew integration, and downstream statistics remain open in P3.4–3.6;
   imperfect-body booleans therefore remain unqualified.

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
