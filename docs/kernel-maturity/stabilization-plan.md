# Stabilization plan: promoting every non-Stable feature

This document is the working plan for driving each README [Status](../../README.md#status)
row currently labeled **Beta** or **Experimental** to **Stable**. It is a plan,
not a promotion: labels change only under the rules of the
[capability matrix](capability-matrix.md), and every promotion PR updates the
[stability matrix](../production-readiness/stability-matrix.md) in the same
change.

## Ground rules

These constraints shape every workstream below; they are restated once here so
the per-feature sections can stay short.

1. **Promotion bar.** A README label changes only when the family's declared
   capability cells are all Qualified, Partial-with-declared-bounds, or
   Unsupported-typed — no Unqualified and no Unsupported-untyped cells remain
   ([capability-matrix.md](capability-matrix.md)). "Stable" therefore does
   **not** mean "handles everything": a documented sub-domain with tested,
   typed refusals on both sides of the boundary is a legitimate Stable row.
   That is the cheapest honest path for several rows below, and the plan uses
   it deliberately.
2. **Chase filters.** Prefer work that re-creates existing analytic surfaces
   over work that invents new approximation surfaces; solve narrow
   configurations, not general problems; prefer work with a stable repro over
   work that needs tooling first. Cases the roadmap marks TERMINAL (the
   pinch-vertex cylinder union, the general UV-arrangement splitter, the scoop
   coordinated cone split) are **not** dependencies of any promotion here and
   must not be re-attempted as part of this plan.
3. **Evidence classes.** Qualifying a cell requires the test classes in
   [testing-strategy.md](testing-strategy.md): positive fixtures with
   independent oracles (closed-form volume/area where available), both-sides
   boundary tests for every declared limit, typed-refusal pins, determinism
   native + WASM, and a reproduction bundle for every defect found on the way.
4. **Per-PR gates.** Full workspace suite including
   `cargo test -p remus-wasm --lib gridfinity`, `approx_census` clean or
   improved, criterion benches non-regressing on touched paths,
   `./scripts/check-boundaries.sh`, and no weakened tests or widened
   tolerances.
5. **Out of scope.** The Stable-but-Blocked/Guarded rows of the stability
   matrix (primitives, core booleans, sweeps, etc.) have their own evidence
   backlog and are not covered here; this plan covers only the eleven
   Beta/Experimental items.

## Phase A — evidence-led promotions (code exists, evidence doesn't)

These four rows are Beta mostly because no qualification matrix has ever been
declared for them. The work is dominated by tests and typed boundaries, not new
geometry. They come first because they are cheap, independent, and exercise the
promotion machinery end to end.

### A1. Draft (Beta → Stable, planar domain)

Owner code: `crates/operations/src/draft.rs`.

- Declare the family axes (face type × pull direction × angle sign/magnitude ×
  body type including cavity solids × scale).
- Qualify the planar domain: volume/angle oracles for drafted prisms at three
  scales, cavity-bearing input, neutral-plane edge cases (neutral plane
  through/outside the body), zero-angle no-op semantics.
- Pin typed refusals for non-planar faces, angles that would invert a face,
  and drafts that would create self-intersection — tested from both sides of
  each boundary.
- README row becomes "Draft (planar faces) — Stable"; non-planar draft stays a
  roadmap P1 item, refused typed. Effort: **S**.

### A2. Defeaturing (Beta → Stable, planar domain)

Owner code: `crates/operations/src/defeature.rs`.

- Same shape as draft: declare axes (feature class × surrounding-face count ×
  cavity interaction × scale), qualify planar-face removal with
  watertight/manifold/volume oracles, pin typed refusals for curved-face
  removal and removals that would open the shell.
- Add negative fixtures where the healed cap is impossible (feature face
  bounded by non-extendable neighbors) and assert the typed error, not a
  partial solid. Effort: **S**.

### A3. Assemblies (Beta → Stable)

Owner code: `crates/operations/src/assembly.rs`, WASM
`crates/wasm/src/bindings/assembly.rs`.

- Declare axes: hierarchy depth × transform composition × instance sharing ×
  BOM determinism × serialization round-trip.
- Qualify: deep nesting with composed transforms verified against directly
  transformed geometry; BOM counts stable under re-ordering; deterministic
  iteration order pinned native and WASM; checkpoint/restore interaction.
- Pin typed errors for cycles, dangling references, and invalid transforms.
  Effort: **S–M**.

### A4. Feature recognition (Beta → Stable, declared feature set)

Owner code: `crates/operations/src/feature_recognition.rs`.

- Declare axes: feature type (hole, pocket, chamfer, fillet) × geometry
  (through/blind, planar/cylindrical walls) × scale × noise (post-boolean
  topology, split faces).
- Build a recognition corpus from existing primitives + booleans with known
  ground truth; assert precision (no false positives pinned) and recall on the
  declared set; "not recognized" is a first-class, tested outcome — the row
  goes Stable for the *declared* feature set with everything else explicitly
  out of domain, not silently missed.
- Recognition on post-boolean split faces is the risky cell; if it cannot be
  qualified, declare it Partial with the boundary tested. Effort: **M**.

## Phase B — targeted geometry gaps (narrow-case engineering)

### B1. Torus booleans (Beta → Stable, declared configurations)

Owner code: `crates/algo` (GFA), torus intersection in
`crates/math/src/nurbs/intersection/` and `crates/math/src/analytic_intersection.rs`.

Current qualified ground: box ± torus and coaxial torus. Plan, per the
narrow-case doctrine — one configuration per PR, each with closed-form or
convergence volume oracles:

1. Torus × plane at arbitrary tilt (Villarceau/general section curves are
   already sampled; qualify the sampled path as **Approximate** with declared
   fit bands, exact where the plane contains or is perpendicular to the axis).
2. Torus × cylinder, coaxial (exact circles) then axis-parallel offset.
3. Torus × sphere, concentric (exact circles).
4. General torus × torus stays **Unsupported-typed → bounded mesh fallback**,
   documented as the row's declared boundary; the fallback is already
   fail-closed, so the cell is typed-approximate, which the promotion rule
   accepts.

Promotion requires flipping the remaining torus rows in `approx_census` to
analytic for the declared cells and both-sides tests at each configuration
boundary. Effort: **M–L**.

### B2. Non-planar profiles for loft / sweep / pipe / revolve (Beta → Stable)

Owner code: `crates/operations/src/{loft,sweep,pipe}/`, `revolve.rs`,
shared caps in `cap.rs`.

The README enumerates exactly four gaps; each is one workstream:

1. **Non-planar section boundaries with more than four edges.** Replace the
   bilinear-only cap with a Coons/loft cap over an n-sided ring (the
   `fill_face.rs` Coons machinery is the starting point). Oracle: watertight
   mesh + volume convergence across deflections.
2. **Holes on non-planar sections.** Cap ring-with-hole sections (annular
   Coons or cap-then-subtract); qualify against extruded-annulus ground truth.
3. **Partial revolution with non-planar boundary.** Build the two cap faces
   from the swept boundary curves instead of requiring a plane.
4. **Miter-corner sweep with non-planar profiles.** The bisector-plane joint
   face becomes a genuinely non-planar joint patch — this *invents* a surface
   (chase-filter tension), so scope it last and accept a declared typed
   refusal here if the patch cannot be validated; the row can still promote
   with this single cell Partial-with-declared-bounds.

Each lands with degenerate-profile and self-intersecting-path typed refusals
for the new paths. Effort: **L** (workstreams 1–3 are the bulk of the value).

### B3. Evolution / face provenance (Beta → Stable)

Owner code: `crates/operations/src/evolution.rs`, `journal_ops.rs`,
`crates/topology/src/` journal + naming (RFC 0003 is fully surfaced already).

The gap is coverage, not architecture: booleans, walking/planar blends, and
patterns carry construction-derived provenance; **offset, shell, draft, split,
defeature, and direct edits produce none** (they journal as explicit
barriers). Plan:

1. Add construction-derived face records to the operations that structurally
   already know their mapping: **split** (each output face has exactly one
   source), **draft** (face-preserving modification), **pattern-adjacent
   copies** — cheap wins first.
2. **Shell and offset**: each offset face is derived 1:1 from a source face by
   construction (`crates/offset/src/offset.rs` walks source faces); thread
   that mapping out instead of discarding it. Rim/arc-joint faces are
   Generated.
3. **Defeature**: removed faces are Deleted, healed caps Generated with the
   neighbor set as sources.
4. Decide the **edge/vertex provenance** question explicitly: the capability
   matrix shows Issue-12 edge/vertex events already exist for booleans. Either
   scope the Stable claim to *face* provenance (README row renamed to say so)
   or extend edge events to the ops above. Recommended: promote as face
   provenance, keep edge/vertex as the P1 roadmap item it already is.
5. Promotion evidence: for every covered operation, a fixture asserting the
   full source→result domain is enumerated (no phantom, duplicate, or
   contradictory claims — the WASM payload validator already enforces this
   shape), plus journal replay across save/load. Effort: **M**.

## Phase C — experimental rows (each is its own program)

### C1. Curved blend geometry (Experimental half of the fillet/chamfer row → Stable, declared domains)

Owner code: `crates/blend/` (walking engine), `crates/operations/src/blend_ops.rs`,
`fillet/`.

This is the row where "Stable = general" is unreachable by chase-filter 1
(blend walls are invented surfaces), so the plan is to grow the **validated
domain list** until the experimental remainder is empty *as a label*, i.e.
everything outside the list fails closed with typed errors — which it already
does. Domain growth order, each with closed-form oracles where they exist:

1. **Blind-hole floor rim past `r_c/2`** — the one *known-wrong-direction*
   defect named in the stability matrix (volume moves the wrong way). This is
   a correctness fix inside an already-shipped assembler and comes first.
2. **Closed-rim chamfers** (cone frustum band mirroring the validated toroidal
   fillet assembler; exact surfaces, closed-form volume — squarely inside
   chase-filter 1).
3. **Walking-engine trim completion** — the four open items pinned in
   `crates/blend/src/trimmer.rs` / the regression tests: keep-side selection
   hint, shared contact edges between trim and blend face, end-cap notch trim
   at stripe termination, chamfer external-tangent branch selection. This is
   what lets the v2 walker reach parity on plane/cylinder edge chains.
4. **Plane–cylinder and cylinder–cylinder convex edge fillets** as validated
   analytic domains (torus/cylinder bands, exact), then vertex corners via
   `corner.rs` spherical patches.
5. Variable radius on the validated domains last; everything else stays
   typed-refused.

Promotion: README row drops the "/ Experimental" once every remaining
non-validated request is Unsupported-typed with both-sides tests — the general
walking blend does not need to succeed everywhere for the label to be honest.
Effort: **XL** (the largest single item in this plan; items 1–3 are the
critical path).

### C2. `resize_blend` (Experimental → Stable)

Owner code: `crates/operations/src/resize_blend.rs`.

Already guarded well (snapshot/rollback, strict validation, typed refusals).
Remaining domain work:

1. Positive-radius **cylinder/cone band reconstruction** (currently a typed
   `unsupported-support-pair` refusal) — the one named gap.
2. Broader support-pair matrix: plane/cone, cone/cone; each pair either
   qualifies or stays typed-refused.
3. Keep variable-radius bands explicitly out of scope (typed).

Because the failure envelope is already fail-closed with stable codes, this row
promotes as soon as the declared pair matrix has both-sides evidence. Effort:
**M**, and it should ride behind C1's assembler work since they share band
geometry.

### C3. IGES (Experimental → Stable) — decision gate first

Owner code: `crates/io/src/iges/{reader,writer}.rs`.

Current state is far from the label: export skips analytic surfaces and
polylines conic edges; import builds planar placeholders only. Two honest
paths — **pick one at the decision gate, do not drift**:

- **Option 1 (recommended): scoped Stable.** Make IGES a *correct, declared
  subset* rather than a full exchange format: export planes + NURBS surfaces
  (analytic surfaces converted to exact rational NURBS via
  `crates/geometry/src/convert/`, which is lossless-in-geometry), conic edges
  as real arcs (entity 100/104) instead of polylines; import trimmed-surface
  B-Rep (110/100/102/120/122/124/126/128/142/144) into real faces with
  heal-after-import sewing. Round-trip fidelity is declared per entity in the
  capability matrix; everything unhandled is a typed `UnsupportedEntity`, and
  STEP remains the documented primary exchange path.
- **Option 2: retire the ambition.** Keep IGES Experimental permanently,
  document it as legacy-read-only, and spend the effort elsewhere. The README
  already steers users to STEP.

If Option 1: reader entity coverage first (import is the user-facing half),
writer second, then the corpus tests mirroring the STEP fixture pattern in
`crates/io/tests/`, plus `ImportLimits` parity. Effort: **L–XL**; this is the
lowest-leverage large item, which is why the decision gate exists.

### C4. Rendering / `remus-render` (Experimental → Stable)

Owner code: `crates/render/src/`.

Blockers are contract and CI, not geometry:

1. **API contract**: freeze `RenderOpts`/`RenderOutput` semantics (camera
   model, face-id encoding, background/AA behavior), document them, and add a
   typed error taxonomy for device-loss/unsupported-adapter paths.
2. **Headless CI**: run on a software Vulkan adapter (lavapipe/SwiftShader) so
   render tests gate PRs; without CI coverage the row cannot promote.
3. **Evidence**: golden-image tests with perceptual thresholds for the
   analytic quadric compute mesher vs. CPU tessellation; face-id buffer
   determinism (same scene → identical id buffer) pinned; readback size/format
   invariants; `viewer` feature explicitly excluded from the Stable claim.
4. Promote to **Beta first** once CI gates exist, Stable after a release cycle
   without contract changes — rendering is the one row where an intermediate
   label is worth the patience, since wgpu adapter variance is outside the
   kernel's control.

Effort: **M**.

## Sequencing

Dependency-driven order, not calendar-driven. Phases overlap freely except
where arrows are noted.

| Order | Item | Effort | Depends on |
| --- | --- | --- | --- |
| 1 | A1 Draft | S | — |
| 2 | A2 Defeaturing | S | — |
| 3 | A3 Assemblies | S–M | — |
| 4 | A4 Feature recognition | M | — |
| 5 | B3 Evolution coverage | M | — (unlocks A1/A2 provenance cells too) |
| 6 | B2 Non-planar profiles (caps 1–3) | L | — |
| 7 | C1 Curved blends (items 1–3) | XL | — |
| 8 | C2 resize_blend | M | C1 band assemblers |
| 9 | B1 Torus booleans | M–L | — |
| 10 | C4 Rendering | M | CI adapter work |
| 11 | C3 IGES | L–XL | decision gate |

Rationale for the order: Phase A items are cheap, independent, and each one
exercises the full promotion pipeline (axes → evidence → matrix update → label
change), which de-risks the process before the expensive rows. B3 comes early
because draft/defeature provenance cells land naturally while A1/A2 fixtures
are being written. C1 is started early despite its size because its first item
is a known correctness defect, not a feature. C3 is last because it has the
weakest consumer pull and an explicit alternative (STEP).

## Reporting

Each promotion lands as one PR (or a short series) that: updates the family
section of [capability-matrix.md](capability-matrix.md), flips the
[stability-matrix](../production-readiness/stability-matrix.md) row and the
README label together, and ships the evidence. This plan file is updated in
the same PR — each item's section gains a one-line disposition (done / partial
/ re-scoped) so the plan cannot rot silently, per the same maintenance rule the
roadmap skill applies to itself.
