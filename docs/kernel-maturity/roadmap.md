# Unified forward roadmap

The one page a session — human or agent — reads to know what to work on
next, and where every open workstream lives. It merges the three sources of
record and the bridge backlog neither program owns:

| Source | Covers | Ledger |
|---|---|---|
| [P-Class program](p-class-program.md) | Correctness & capability (M2–M8) | [p-class-status.md](p-class-status.md) |
| [Open Kernel program](open-kernel-program.md) · [implementation plan](open-kernel-implementation.md) | Proof, adoption, interchange, ecosystem (O1–O7) | [open-kernel-status.md](open-kernel-status.md) |
| [Stabilization plan](stabilization-plan.md) | Historical label promotions; residue absorbed below | its Dispositions section |
| **Bridge backlog (§B below)** | Ready items covered by neither program | §B table, updated in-place |
| [Campaign history](campaign-history.md) | Closed roots, refuted theories, and the digs behind them (not loaded by default) | read-only, append when a dig closes |

The work-selection *doctrine* (chase filters, TERMINAL list, acceptance bar,
durable lessons) remains `.claude/skills/roadmap/SKILL.md`; this page is
the *queue*; the narrative behind closed rows is `campaign-history.md`. All
are living documents: update the relevant row in the same PR that changes
its state. Before claiming anything: `gh pr list --state open` (R6).

- **Drafted:** 2026-08-29, baseline `main` @ `3c232e8`.
- **External K-S1 disposition — tangent-boss operand drop: done (PR #143,
  2026-08-30).** The historical pre-fix sequence returned the unchanged plate
  (19,200 instead of 21,713.274 cubic millimetres); current native and WASM
  contracts retain the operand across the ratio/scale matrix, with exact-only
  refusal and explicit approximation provenance where the exact path is unavailable.
- **External K-S1 disposition — cross-drilled render/measure:** done in PR
  #144. The OpenZCAD operation sequence now has a deterministic replay bundle,
  independent volume oracles, ratio/scale display-mesh qualification, and a
  non-vacuous WASM `meshQuality` contract. Follow-ups remain for the separate
  face-orientation inconsistency and the sub-millimeter fine-mesh boundary
  residue; neither is hidden by this disposition.
- **External K-S1 disposition — fillet fail-closed migration: done (PR #181,
  2026-09-01).** Every public fillet/chamfer mutation path — the WASM `fillet`
  cascade, `filletVariable`, `filletV2`/`chamferV2`/`chamferDistanceAngle`,
  the journaled wrappers, `executeBatch`/`executeBatchV2`, and the legacy v1
  Rust engines — is transactional and postcondition-validated: no path returns
  the input handle or a clone of it as success, exposes partially mutated
  topology, or ships a geometrically invalid result; refusals carry the stable
  `blend_failure_code` vocabulary on every surface. The versioned repro bundle
  `fillet-variable-fail-closed` fails on the pre-fix kernel and passes after.
  Two damaged-success cases the old closed-shell gate could not see (the
  blend-adjacent second-pass fillet and the gridfinity lip peak-rim fillet)
  are honest typed refusals now; un-refusing them is the B4 trimmer work.
- **Remus K-S4 disposition — NURBS fuzz slice:** done in PR #163. Bounded
  rational-surface construction/evaluation and NURBS SSI now run in the
  scheduled fuzz campaign against an independent plane oracle; topology
  mutation, native serialization, curve-intersection, and offset-specific
  campaigns remain S4 follow-ups.
- **Remus K-S4 disposition — topology-mutation fuzz slice:** done in PR #170.
  Derivation, validated/transactional rollback, checkpoint restore, and
  solid-deletion sequences now run in the scheduled campaign over a bounded
  box against exact-state, stale-handle, atomic-refusal, closed-manifold
  census, and closed-form volume oracles. The campaign's first run found the
  rollback/restore contract split fixed in the same PR: transactional
  rollback now undoes in-window retirements (`restore_for_rollback`), and the
  checkpoint barrier no longer leaves a dangling face-loop derivation map.
  Native serialization, curve-intersection, and offset-specific campaigns
  remain S4 follow-ups; migrating ad-hoc snapshot/restore call sites off the
  checkpoint-barrier primitive is flagged for the W5 contract work.
- **Remus K-S4 disposition — native-serialization fuzz slice:** done in PR
  #173. Bounded box/cylinder documents with duplicate roots, shared-shell
  aliases, repeated/aliased compound members, hostile tolerances, and
  attributes now round-trip in the scheduled campaign against per-position
  validation/census/closed-form volume oracles, bit-exact state survival,
  byte-identical re-serialization, and typed non-mutating refusal of
  corrupted references. The byte-identity oracle pinned serde_json's
  `float_roundtrip` feature as load-bearing for exact f64 replay (fixed in
  the same PR with a bit-exact tolerance regression). Curve-intersection and
  offset-specific campaigns remain S4 follow-ups.

## §H Horizons

### H0 — in flight (verify before duplicating)

P-Class 2.0 partially landed (#125 FF section ranges, #130 edge domain
authority), RFC 0004 merged (#126); open: 2.1 honest-failure hygiene
(#129), RFC 0005 draft (#127), the program docs themselves (#133).

OpenZCAD consumer-roadmap K-S4 (`approx_census` CI enforcement): **done (PR
#140)**. Its authoritative disposition line remains in planning PR
esaueng/OpenZCAD#140 so the two repositories retain separate commit streams.

### H1 — now: three non-colliding lanes

1. **Geometry lane (P-Class M2 track — one session at a time in
   algo/pave-filler):** finish 2.0 (reader migration, boundary-authority
   flip), then 2.2 sphere-in-general-position, 2.3 Steinmetz, toward 2.4.
   Bridge items that ride this lane's files: B2, B7, B8 below.
2. **Infrastructure lane (Open Kernel Wave A — new dirs and io):** O1.1
   gauntlet, O1.3a fillet torture corpus, O1.4a validation properties,
   O4.1 facade, O4.4 error registry, O5.1 STEP assemblies, O3.1 benches,
   O6.1/O6.4 docs + contributing, O2.1a–b RFC 0006 + math substrate.
3. **Qualification lane (bridge backlog — bounded, evidence-heavy,
   disjoint):** B1 healing disclosure, B3 closed-rim chamfers, B4 v2
   trimmer items, B5 offset provenance, B6 evidence matrices, B10/B11
   small hygiene items.

### H2 — after P-Class 2.4 (the parallelization point)

P-Class M3 integration ∥ M4 ∥ 7.4+8.1 (per its §4), plus Open Kernel
Wave B (O2.1c–e variant ripple, O2.3 arrangement splitter, O3.2 spatial
cache, O4.2 publish dry-run, O1.2 head-to-head, O4.3 Python, O5.2 e3b,
O5.3a AP242, O6.2 playground). Bridge: B2 scale residuals close inside
2.6; B9 tangent-torus rides 2.7's tangency machinery.

### H3 — after M4 / M5

M5 blend depth ∥ M6 direct modeling ∥ M7 surfacing; O1.3b torture-suite
publication, O5.3b PMI read, O7 hybrid RFC. Bridge: B12 non-planar cap
holes (with M7's cap work).

### H4 — v1.0

**Definition of v1.0** (the first stable publish, O4.2c): P-Class exit
benchmarks **B1–B5** green as permanent tests + Open Kernel scoreboard
claims **S1–S7** live + zero Unsupported-untyped cells in the capability
matrix + the bridge backlog empty or explicitly re-triaged. Anything
short of all four publishes as 0.x.

## §B Bridge backlog — owned by neither program

Ready items from the stabilization-plan residue, the capability-matrix
sweep, and the deferred-work inventory (2026-08-29). Each row is claimable
by a bounded session; update state in-place. Items that map onto an
existing program issue are listed there instead — notably: Steinmetz = 
P-Class 2.3 · conic boolean cells = O2.2 · offset self-intersection = 5.7
· e3b = O5.2 · error registry = O4.4 · seam/p-curve round-trips = 2.0.

| ID | Item | Where | Size | Why it matters | State |
|---|---|---|---|---|---|
| B1 | **Healing disclosure typing** — the matrix's only named Unsupported-untyped cell: permissive healing can mask an invalid result as valid. Type every repair (report what changed, refuse to claim validity it didn't verify); both-sides tests. | `heal/src/fix/`, `check/src/validate/` | M | The last untyped silent-failure path in the kernel; highest correctness value per line. Do first in the qualification lane. | **Done (2026-09-03, this PR):** fixer results enumerate counted repair kinds and typed declined repairs; L2 `OK` explicitly means only “no fixer action,” never validity. Operations, facade verified mode, configurable direct WASM, named pipelines, and additive detailed direct/batch WASM surfaces commit only after independent operations/check validation. Invalid and unverifiable results return stable typed refusals with attempted repairs and roll back. Native and WASM both-sides regressions pin verified success and refusal. |
| B2 | **Boolean scale residuals** — 1e-5 fails closed (100·tol weld bands); raw-GFA 1e6 silently 0.9467 vs 0.8400 (ExactOnly refuses; measure + pin). | `algo` bands | M | Feeds P-Class 2.6 directly; the 1e6 cell is a possible silent-wrong class. Geometry lane. | Open |
| B3 | **Closed-rim chamfers** — cone-frustum band mirroring the validated toroidal fillet assembler; closed-form volume oracle. Stabilization C1.2. | `blend`, `operations/src/chamfer.rs` | M | Exact surfaces, cheap, passes chase filter 1; unblocks resize_blend cylinder/cone (C2). | Open |
| B4 | **v2 walking-trimmer completion** — the four named gaps: keep-side hint, shared contact edges, end-cap notch trim, chamfer external-tangent branch. Stabilization C1.3. | `blend/src/trimmer.rs` | M | Critical path for v2 walker parity → legacy engine retirement (M5 precondition). | Open |
| B5 | **Offset face provenance** — offset derives faces 1:1 and discards the mapping; journal real evolution instead of a barrier. | `offset`, `operations/src/offset_v2.rs` | S | The last declared-barrier operation nobody owns; closes the B3-residual from stabilization. | **Done (2026-09-02, PR #224):** default intersection-joint V2 offsets retain and validate the total 1:1 construction map; native and direct/batch WASM journal wrappers record it transactionally. Closed-form plane/volume, persistent-reference, rollback, and WASM parity oracles pin the claim. Arc-joint and self-intersection-removal variants explicitly refuse this map because later face synthesis/replacement needs richer provenance. |
| B6 | **Evidence matrices, batched** — the "Stable-but-blocked" ledger rows that are pure test work: primitives invalid-input/scale/postconditions; plane-section cavity+degeneracy; measurement curved-cavity+scale; sweeps degenerate/cavity + nonconvergence budgets; convex hull/Minkowski degenerates. One qualify_*.rs per family, stabilization-plan pattern. | `operations/tests/` | M (S per family) | Flips ~8 Blocked ledger rows with zero new geometry; ideal bounded-session work. | Open |
| B7 | **Pave-block attachment for marched FF curves on curved faces** — the named canonical fix for the cross-face boundary-desync family; three cheaper altitudes already failed. | `algo/pave_filler/make_blocks.rs`, `phase_ff.rs` | L | Deepest structural payoff in algo; root-causes a whole non-manifold family. Geometry lane, coordinate with M2; repro `replay_scplate.rs`. | Open |
| B8 | **Reversed NURBS sub-span convention** — forward spans shipped; reversed validated sub-spans blocked on the same arrangement defect as B7. | `topology/src/edge.rs` | M | Completes the endpoint-trimmed contract 2.0 builds on. | Open (after/with B7) |
| B9 | **Torus ∖ coaxial cylinder tangent cut** — the single cell keeping torus booleans Beta; needs a tangent-contact primitive (explicitly NOT the band splitter). | `math/analytic_intersection.rs`, `algo` splitter | M | B1-ledger promotion Beta→Stable; closed-form oracle exists. Rides 2.7 tangency machinery. | Open |
| B10 | **Curve-curve / curve-surface classification qualification** + conic distance/classification cells | `math`, `geometry/extrema`, matrix harness | M | Unqualified since the matrix was written; sits under many families' claims; pure evidence. | Open |
| B11 | **Small hygiene set** — `log::debug!` false-zero in `fill_images_faces.rs` (diagnostic-infra bug); deterministic STEP entity ordering; heal `fix_duplicate_faces` winding-blind comparison; plane×plane sampled in-both exact upgrade; `n_fine` clamp hazard note→guard. | various | S each | Cheap, each has already cost or will cost a debugging session. | Open |
| B12 | **Holes on non-planar section caps** — annular Coons or cap-then-subtract vs extruded-annulus ground truth (stabilization B2.2). | `operations/src/cap.rs`, `fill_face.rs` | M | Largest remaining non-planar-cap value with clean ground truth. H3, with M7 cap work. | Open |
| B13 | **STEP inner-shell (voids) export** — `BREP_WITH_VOIDS` reads; export of cavity solids incomplete. | `io/src/step/writer.rs` | S–M | Round-trip honesty for hollow parts; gauntlet round-trip stage will hit it. | Open |
| B15 | **Cut/intersect pocket faces on a cylinder wall come back inconsistently oriented** — pinned but ignored since #198; the fix is engine-side, the repro is already in-tree. | `crates/operations/tests/regress_parallel_boss_band_sections.rs`, `algo` assembly | M | The only `#[ignore]` in the inventory that pins a live engine defect rather than a fork-policy or diagnostic case. Geometry lane. | Open |
| B14 | **Render promotion track** — Experimental→Beta after a contract-stable release cycle (stabilization C4 residue); outside both programs. | `render` | S (time-gated) | Cleans the last stabilization row. | Open |

**Explicitly not queued** (decided or terminal — do not re-open without
the named primitive): IGES growth (C3, decided), box∪sphere and torus∩box
census rows (TERMINAL → O2.3 re-opens them properly), universal
duplicate-edge merge key (proven unbuildable), mesh co-refinement
watertightness (below the chase filter until a live case routes there),
kumiko lattice family (probe only per the roadmap skill's engine-side
question), v1-fillet API migration (product decision, owner's).

## §D External roadmap dispositions

- **K-W3 distributed WASM budget — partial
  ([PR #174](https://github.com/esaueng/remus/pull/174), 2026-08-31):** every
  consumer package workflow now deterministically optimizes the distributed
  bundler binary, validation fails above the 8 MiB OpenZCAD ceiling, and the
  PR size report compares committed distribution artifacts. Current `main`
  falls from 8,773,687 to 7,724,098 bytes, leaving 664,510 bytes of headroom.
  OpenZCAD cold-load timing on target hardware remains the product-side W3
  follow-up.
- **K-S2 exact measurement — partial (PR #151):** production `faceArea` and
  `surfaceArea` now reuse the exact planar boundary-moment integrator for
  line/circle/parabola wires, including circular holes, with scale,
  deflection-independence, direct-WASM, and batch-WASM oracles. Exact ellipse,
  hyperbola, and NURBS planar boundaries, general curved-face area, and
  deflection-independent curved-body volume remain.
- **K-S3 SSI Newton budget — done
  ([PR #147](https://github.com/esaueng/remus/pull/147), 2026-08-30):**
  `WorkBudgets::newton_iterations` is authoritative across NURBS×NURBS seed,
  branch, and march refinement; cancellation is polled inside the coupled
  Newton loop and propagates through the existing typed, transactional WASM
  boolean contract. Per R8 the cap is JS-callable: an additive optional
  `newton_iterations` argument on `booleanWithQuality` /
  `booleanWithCancellation` and a `newtonIterations` field on the
  `executeBatch` `booleanWithQuality` op, validated (non-negative integer
  within the public work budget) with contract tests on the default,
  bounded, and rejection paths. Default behavior remains the historical 20
  iterations. Its then-remaining subdivision slice is closed immediately
  below; parameter-space budgeting remains queued under P-Class 2.8.
- **K-S3 SSI subdivision budget — done
  ([PR #160](https://github.com/esaueng/remus/pull/160), 2026-08-30):**
  `WorkBudgets::subdivision_depth` replaces the seed finder's hard-coded
  recursion depth and is authoritative before every recursive Bezier-patch
  split. The default depth 6 reproduces prior behavior; depth 0 performs no
  recursive split. Direct `booleanWithQuality` / `booleanWithCancellation`
  expose additive `subdivision_depth`, and batch `booleanWithQuality` exposes
  `subdivisionDepth`, with shared validation and default/boundary/rejection
  contract tests. Parameter-space budgeting and wider operation-family
  adoption remain under P-Class 2.8.
- **K-S3 SSI marcher-budget WASM surface — done ([PR #202](https://github.com/esaueng/remus/pull/202),
  2026-09-02):** the existing `march_steps`, `queue_size`, `segments`, and
  `branches_per_direction` caps are additive optional arguments on direct
  `booleanWithQuality` / `booleanWithCancellation` and matching camelCase
  fields on batch `booleanWithQuality`. Shared bounded-integer validation,
  legacy-default equivalence, generated-WASM smoke coverage, context-authority
  tests, and a batch rejection/rollback volume oracle pin the contract.
  Parameter-space tolerance and wider operation-family adoption remain under
  P-Class 2.8, so that parent item stays partial.
- **K-S1 pattern overlap — done (PR #142, 2026-08-30):** linear,
  circular, and grid patterns now refuse measured material overlap with the
  typed `pattern_instances_overlap` contract and full rollback across native,
  direct WASM, and `executeBatchV2` repro coverage. Touching and disjoint
  instances remain supported across a 1e-3/1/1e3 scale sweep. Exact instance
  fusing is intentionally deferred until the separately queued
  pattern-through-fuse provenance work can make its evolution claims truthful.

## §S Session playbook

Match session type to lane; check both ledgers and `gh pr list` first.

- **Geometry-hard session** (budget for multi-pass debugging): H1 lane 1
  in P-Class order, or B7 if M2 files are contended. Never two sessions
  in `algo/pave_filler` at once.
- **Bounded/short session:** one B-row (B5, B11, B13, or one B6 family),
  or an inherited-queue item from P-Class §6.
- **Infrastructure session:** next unclaimed Wave A row in
  [open-kernel-status.md](open-kernel-status.md).
- **Evidence session** (test-writing capacity): B6 families, B10, O1.3a.
- **Docs/ecosystem session:** O6 rows.
- **Owner-only:** O4.2c/O4.3c publishes, O6.2 hosting, O6.3 outreach,
  v1-fillet migration decision.

Maintenance rule: any PR that changes an item's state updates its row
here (or its program ledger) in the same PR — same discipline as the
skill's living-document mandate.
