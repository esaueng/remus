---
name: roadmap
description: Use at the start of an autonomous or unsupervised session to pick what to work on, when deciding whether a geometry case is worth chasing, when a task looks like something a past session already tried, or before claiming a case is closed. The sanctioned work-selection doctrine: what is open and ready, what is terminal, the chase filters, and the acceptance bar.
---

# Roadmap: choosing what to work on

This is the sanctioned work-selection doctrine for autonomous sessions: which
work to chase and which to skip, what is TERMINAL (do not re-attempt without
new tooling), the bar a case must clear to be called closed, and the lessons
past campaigns paid for. It is deliberately short. The narrative behind every
closed row — the digs, the refuted theories, the tool-era scores — lives in
`docs/kernel-maturity/campaign-history.md`, which nothing loads by default.

## This is a LIVING document: maintenance is mandatory

When a session **closes, defers, or discovers** a work item, it MUST update the
queue (`docs/kernel-maturity/roadmap.md` or the item's program ledger) in the
same PR, and this file whenever a filter, a TERMINAL entry, a trap, or a
lesson changes. A stale doctrine is worse than none: past sessions burned large
budgets rediscovering dead ends this file was supposed to name. Keep every
entry here to ONE line with a pointer (a test path, a PR number, a source
file) that carries the detail; put the story in `campaign-history.md`, never
here.

The `#[ignore]` inventory is the load-bearing artifact behind any "deferred"
claim. Regenerate and reconcile it before quoting one:

```bash
rg -n -A2 '#\[ignore' crates/    # filter the doc-comment false hits by hand
```

## When to use

- Starting a session with no assigned task and needing to pick high-value work.
- A task resembles something that may already be tried, closed, or proven impossible.
- Deciding whether an analytic-recovery or parity case is worth the budget.
- Before writing "this case is closed" anywhere.

## The north star

**Serve OpenZCAD.** Remus's consumer is the OpenZCAD web/desktop CAD application
(`esaueng/OpenZCAD`, `packages/kernel-adapter` pins `remus-wasm` from this repo's
committed package). The bar is the one OpenZCAD's users feel: exact analytic results,
watertight manifold output, correct volumes, and interactive-speed booleans on real
modeling chains. Verification is IN-REPO: the full workspace suites (including the
wasm contract tests), the `crates/io/tests` fixture corpus, `approx_census`, the
corpus gauntlet scoreboard (`tools/gauntlet`, results branch), and the criterion
benches.

The gridfinity layout tool, its scenario matrices, and the brepjs head-to-head
bench were the upstream project's harness and were retired on 2026-08-20; any
"tool-side re-probe pending" note in the history is closed-as-not-applicable,
and the engine-side fixture named beside it is the bar. The gridfinity-derived
`*_inmem` fixtures and the wasm `gridfinity_tests` module stay: they are generic
hard-geometry regression coverage with no external dependency.

## The queue lives in the program docs

Work selection starts from `docs/kernel-maturity/roadmap.md` — the unified queue
merging the P-Class program (ledger `p-class-status.md`), the Open Kernel program
(ledger `open-kernel-status.md`), and the bridge backlog (§B). Pick by the
session-playbook section there, then apply THIS file's filters, TERMINAL list, and
acceptance bar to whatever you picked: the roadmap doc governs *what* is open, this
file governs *how* to chase. Before claiming anything: `gh pr list --state open`,
and read the last scheduled runs (Corpus Gauntlet, Fuzz Smoke, Mutation Testing)
— a red proof job nobody looked at is not proof.

`docs/kernel-maturity/industrial-parity.md` is the non-owning competitive
overlay: it says where a row stands against the reference kernel and which
program row owns the gap, never what is open — never claim work from it
directly; claim the owner row it points at.

## The priority filters (rules with reasons)

1. **Chase operations that RE-CREATE an existing analytic surface type. Do NOT chase
   ops that INVENT a blend or approximation surface.** A boolean or revolve result face
   is a trimmed patch of an *input* surface, so it is always closable with the right
   split. Fillet and chamfer walls, general sweep and loft side faces, and offsets of
   NURBS input introduce a NEW surface with no closed form; they are fundamentally
   approximate. See `analytic-preservation`.
2. **Solve the NARROW case (coaxial, perpendicular, equal-radius), not the general
   problem.** Every primitive-boolean win was gated to one specific configuration and
   defers to the generic marcher otherwise. Sessions that reached for a general solver
   burned budget and shipped nothing.
3. **Prefer work with a stable primitive repro over work that needs tooling first.**
   The four primitive-boolean cases (stable repros in
   `crates/operations/examples/approx_census.rs`) were picked over the tooling-blocked
   scoop case for exactly this reason.
4. **After ANY GFA or boolean change, run the FULL workspace suite (including the
   wasm gridfinity contract tests) plus `approx_census` before claiming anything.**
   Scorecards rot silently; a stale one once hid a regression through a whole
   release. (Historically this filter demanded a tool-side scenario re-probe; the
   tool is retired — the in-repo fixture corpus is the scoreboard now.)

## TERMINAL cases: do not re-attempt without the named missing primitive

Several past sessions burned large budgets rediscovering these. Each needs a component
that does not exist yet; without it, stop.

- **Equal-radius perpendicular cylinder-union RENDER.** The exact seam is a
  self-touching figure-eight (a genuine non-manifold singularity, odd Euler). The
  shipped artifact (#1008: analytic B-Rep whose marched-NURBS seam dodges the touch,
  plus exact closed-form volume) STANDS. Needs a face-split-at-pinch primitive on a
  periodic wall, or a periodic-aware crossing-holes mesher. There is no
  `exact_cylinder_cylinder` symbol; do not go looking for one.
- **Plane-by-sphere splitting across the chord-discretized equator.** The general
  capability behind box-sphere; a section circle's crossings miss a polygon-approximated
  equator by the sagitta. Box-sphere was closed (#1006) with a case-specific seam-plane
  fit (`rg -n 'seam_plane' crates/`). The general fix is a UV-space arrangement
  splitter, a dedicated multi-day component not yet built. The boundary-plane
  crossing technique is proven and reusable.
- **Gridfinity scoop fuse (3x3 scoop+label+lip).** Root: a lip-foot cone must be split
  with a coordinated staircase cone-split plus bracket-cap re-trim sharing the new edge;
  every one-sided attempt regresses. Many sequential autonomous passes exhausted.
  Parity is already MET via a correct-but-slow mesh fallback (this is perf-only). Note:
  STEP-faithful in-memory repros now EXIST (`crates/io/tests/scoop*_inmem.rs`); the old
  "needs serialization tooling first" framing is stale. The real blocker is the
  coordinated split.
- **Snap-clip deepened-notch case — formerly terminal, CLOSED on both faces**
  (fixture `crates/io/tests/deepened_wall_opening_inmem.rs`; the plane-face union
  handles all-Line openings only — arc-bounded openings still bail, extend only
  when a repro demands it). Kept here so the name is not re-chased.
- **A universal smarter merge-key for duplicate edges. PROVEN UNBUILDABLE.** The
  gridfinity lip corner (chord + arc, same endpoints) MUST merge; the torus-box in-tube
  lens (line + co-endpoint arc) MUST stay distinct. No merge-key discriminant separates
  them; the distinction is global. Sanctioned pattern: splitter-side midpoint splits,
  per case, so no two edges share both endpoints, and leave
  `merge_duplicate_edges` (in `crates/algo/src/builder/builder_solid.rs`) alone. Control
  the geometry you emit; do not make the shared merge smarter.

## Open items with a repro

The `#[ignore]` inventory (regenerated 2026-09-04) holds no open engine defects.
The remaining ignores are
two fork-policy pins blocked on the trim-contract reconciliation
(`crates/operations/tests/regress_chamfer_obtuse_ridge.rs`,
`regress_fillet_concave_notch.rs`, see PR #126), one ~2 min perf run
(`boolean/tests.rs::staircase_fuse_with_cylinders`), and print-only
diagnostics (`profile_intersect.rs` ×3, the two #696 dovetail probes, the four
`diag_*tangency*` landscape probes — re-run those with `--ignored --nocapture`
before re-opening the tangency row). Everything else that was once "deferred"
is either a §B row in `roadmap.md`, a program-ledger issue, or a closed entry in
`campaign-history.md`.

The `modifier_ops` fuzz red that stood from 2026-08-16 to 2026-09-02 was the
harness's own option-honoured floor misreading a correct 0.05 fillet on an
800 u³ body, not a kernel defect (fixed in #223; seed committed at
`fuzz/corpus/modifier_ops/fillet-small-radius-on-large-disjoint-body`).

## Durable lessons (one line each; the story is in `campaign-history.md`)

- **Deterministic STEP emission sorts unordered face, void-shell, and hole-loop aggregates by arena ID but never sorts coedges;** coedge sequence carries boundary traversal semantics (`crates/io/src/step/writer.rs`).
- **Public profile construction must use the strict wire-to-face path;** the low-level plane-from-points builder is not a collinearity validity gate (`crates/remus/src/model.rs`, PR #225).
- **Performance baselines start from measured stack families, not a guessed loop list;** O3.1's 3% census and native-only Criterion map live in `docs/kernel-maturity/o31-inner-loop-baseline.md`.
- **Exact rational conic twins do not preserve angle-linear parameter speed;** compare positions after projection plus tangent direction and curvature, and use a deterministic one-sided radial derivative at revolution poles (`crates/math/src/surfaces/swept/tests.rs`, PR #189).

- **Replay a fuzz artifact natively and print BOTH measurements before believing its message;** an assertion that formats one reading twice reads exactly like a no-op that never happened (`modifier_ops`, 2026-09-02).
- **Not every scenario failure is a boolean fallback.** Tessellation density,
  shared-rim meshing, and face orientation produced whole failure families with
  zero mesh fallbacks; capture the actual boolean traffic and replay operands
  natively before assuming GFA.
- **Print operand free/over-edge counts before diagnosing any replayed
  capture.** Ray-cast parity against a non-closed operand is undefined; one
  whole goma iteration was spent "debugging" the classifier on malformed input.
- **The by-edge-id manifold gate and `validate_solid` are blind to
  position-duplicate free edges and nested same-orientation faces.** Any
  watertightness claim needs the position-quantized check; `solid_volume` being
  translation-variant is the cheap detector for a doubled boundary.
- **Marched/fitted section geometry is good to ~1e-6; every exact-tolerance
  (1e-7) gate it meets needs a weld-scale (100·tol) band.** Four separate gaps
  in one family (weld anchors, T-splits, on-plane checks, junction discs).
- **The face splitter is a web of mutual calibrations.** On any
  `face_splitter` or section/clip change run ALL foils: d4, honeycomb pcut3,
  divider-lip, the nub fixtures, cylinder-slot, groove-mouth, junction-disc.
  Each has caught a discriminant that the target case alone blessed.
- **Splitter interior points of notched or symmetric pieces land on
  feature-plane intersections by construction;** classification must survive
  on-plane samples (`classifier/ray_cast.rs` per-ray degeneracy re-cast).
- **Any coverage or containment test that decides a section's fate by interior
  sampling is blind to overhang at the ends;** use exact interval math where
  the salvageable fraction can be arbitrarily small. The same aliasing class
  killed straight sections in the phase-FF AABB pre-filter (goma, #1224).
- **Stored `start_uv` and hole-wire pcurves can be fitted in a foreign frame,
  and a wire can mix pcurve orientation conventions;** any new polygon consumer
  samples 3D through the `PlaneFrame` instead. Do not widen fixes into
  `sample_wire_loop_uv`.
- **A centroid is not an interior point for concentric or non-convex wires;**
  hole removal in tessellation needs per-wire interior seeds and even-odd
  depth by geometric containment (stored winding cannot classify them).
- **A wire with no angular gap is a wrapped face;** any consumer that
  polygon-approximates it inherits the parity flip.
- **Full mesh area + zero boundary/non-manifold edges + volume deficit + zero
  inverted normals = sparse-interior deep chords, not winding.**
- **When a range and a mask disagree, instrument both before blaming either**
  (the torus band split: `face_uv_bounds` was right, the second consumer
  unwrapped only `u`).
- **Fix at the layer that owns the artifact.** Do NOT "restore" out-of-domain
  NURBS extrapolation to fix a caller, and do NOT silence a predicate with a
  kernel-wide numerical change; bisect the fork-local commits to locate, then
  fix the caller (`sweep_miter::compute_frames`, the bezier-clip degenerate
  AABB exit).
- **The greedy wire walker is chaotic in junction-level geometry;** fix
  junction identity everywhere first, then partition health follows. A
  "graze" heuristic keyed to face extent is blind to corner-window exits.
- **Test a boolean result's VERTICES, never its bounding box:**
  `solid_bounding_box` bounds a trimmed curved patch by its untrimmed surface.
- **A geometrically closed splitter loop can still carry the wrong stored winding;**
  normalize selected cylinder outer/inner loops against the carrier normal before
  duplicate-edge merge (`builder_solid.rs`, B15).
- **A volume band wide enough to hide an operand is not an oracle;** pin
  closed forms, inclusion–exclusion sums, and ray-cast classification.
- **`log::debug!` inside `fill_images_faces.rs` does not emit** (cause
  undiagnosed): log-based probes there read as a false zero; use an env-gated
  `eprintln!` and do not commit it.
- **Check capture-directory mtimes before replaying mixed capture dirs;** a
  stale pre-fix operand cost one full iteration.
- **The reference kernel's snapshot pins are kernel-specific.** Triangle
  counts below a pin are benign density difference; 10× above is a defect;
  a volume that disagrees with a pin can be Remus being MORE exact (the
  snapClip clip volume, the K0.1 parabolic fillet file).

## Subsystem trap notes (crates without their own skill)

- **heal `fix_duplicate_faces` IS implemented** (solid-scoped, `crates/heal/src/fix/solid.rs`,
  returns `Status::DONE2`), not a no-op stub. It compares only centroid, normal, and
  edge count, so it can miss true-but-differently-wound duplicates; do not rely on it
  for subtle cases. Verify current state before quoting either way.
- **heal, offset, and sketch have no distilled campaign knowledge.** They follow the
  same `debugging-doctrine`, but no skill covers their internals. Treat any diagnosis
  there as first-of-kind and write findings down (a test comment or a new note).
- **The v1 fillet default was flipped to v2-first (2026-07, product decision):**
  `try_fillet` now tries `blend_ops::fillet_v2` first, rolling-ball second,
  bevel last. The v1 engines remain as fallbacks and behind `filletVariable`.
- **The v1 fillet deprecations are entangled with the public wasm API.**
  `operations/src/fillet/mod.rs::fillet` and `fillet/rolling_ball.rs::fillet_rolling_ball`
  are `#[deprecated]` yet still reached through the wasm `fillet` binding, via
  `wasm/src/helpers.rs::try_fillet` (it tries `fillet_rolling_ball` and `fillet` in its
  engine-preference chain). Migrating them changes public behavior; that is a product
  decision, not safe cleanup. The offset v1 path was already dropped in #850.
  `offsetSolid` now routes through the non-deprecated `offset_v2::offset_solid_v2`. See
  `fillet-blend` and `wasm-bindings`.

## Acceptance bar for a geometry campaign case

Every box before "closed":

- [ ] **Exact analytic result** where the inputs are analytic (typed faces, single to
      low-tens face count, not hundreds).
- [ ] **Watertight** tessellation (zero boundary edges).
- [ ] **Manifold** B-Rep (every edge used by exactly two faces, Euler balanced).
- [ ] **Full workspace suites green, INCLUDING** `cargo test -p remus-wasm --lib gridfinity`
      (running only algo/io/operations has shipped a gridfinity regression before).
- [ ] **Regression fixture shipped** with the fix (STEP or arena `.bin`; see `testing`).
- [ ] **Census clean or improved:** the row flips FALLBACK to analytic
      (`cargo run --release --example approx_census -p remus-operations`).
- [ ] **No perf regression** on the criterion benches touching the changed path
      (`cargo bench -p remus-operations` — compare against the prior run's saved
      baselines; the retired tool-side head-to-head is no longer part of the bar).
- [ ] **Release published** when user-facing (see `release-flow`).

## Anti-patterns

- Do NOT re-attempt a TERMINAL case hoping this time is different; it needs the named
  missing primitive, not another pass.
- Do NOT reach for the general solver when the narrow case is what parity needs.
- Do NOT call a case closed on an "exact analytic" census row alone; the census does not
  check correctness (see `analytic-preservation`).
- Do NOT quote a "deferred" or face-count claim without regenerating the inventory;
  it rots silently.
- Do NOT close, defer, or discover an item and leave the queue unchanged.
- Do NOT write a dig narrative into this file; one line and a pointer here, the
  story in `campaign-history.md`.

## Related skills

`analytic-preservation` (the chase filters in depth), `debugging-doctrine`
(before any multi-pass dig), `solid-verification` (the acceptance oracles),
`testing` (fixtures and ready-repros), `fillet-blend` (the blend traps),
`release-flow` (shipping a user-facing close), `parity-benchmarking`
(RETIRED-HISTORICAL — capture recipes only). History:
`docs/kernel-maturity/campaign-history.md`.
