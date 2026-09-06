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
| [Industrial parity overlay](industrial-parity.md) | Non-owning competitive crosswalk against the reference kernel, scope contract, scorecard, workflow scenarios, H5–H7 gates | no ledger — points at the rows above; update pointers when an owner row flips |

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

As of 2026-09-04: P-Class 2.0–2.3, 3.1–3.3, 4.1–4.7, 5.1–5.7, and 6.1 are
merged (see `p-class-status.md`); 2.4, 2.8, and 6.2 are partial — 6.2
generalized face moves (PR #257) qualifies a holed planar boss cap moving
through an incident constant-radius fillet with exact evolution and
direct/batch WASM parity, reusing 6.1 for coaxial bores; rotation, lateral
relocation, outward cylinders, and surface-type changes remain open. The
Open Kernel Wave A rows still unclaimed are O4.4, O5.1a–c, O6.1, and O6.4.
Bridge rows closed since the draft: B1, B5, the B6 primitive family
(including ellipsoid), the B11 STEP-ordering item, B13 void export, and B15
pocket-face orientation; B12 is partial (rectangular holes on sweep/pipe
caps, PR #252). No `#[ignore]` pins a live engine defect.

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
publication, O5.3b PMI read, O7 hybrid RFC. Bridge: the B12 residue
(loft-hole correspondence, holed partial revolutions) with M7's cap work.

### H4 — v1.0

**Definition of v1.0** (the first stable publish, O4.2c): P-Class exit
benchmarks **B1–B5** green as permanent tests + Open Kernel scoreboard
claims **S1–S7** live + zero Unsupported-untyped cells in the capability
matrix + the bridge backlog empty or explicitly re-triaged. Anything
short of all four publishes as 0.x.

### H5 — Core modeling parity (post-v1.0)

Prove parity with the reference kernel across the in-scope
authored-modeling domain. Gates (full text in
[industrial-parity.md](industrial-parity.md) §6): complete crosswalk with
no `Unknown` or unowned in-scope rows; zero Unsupported-untyped cells
re-verified; W1, W2, W5, W7 complete correctly (not by refusal) on both
surfaces at three scales; 2.4c/d, 2.5, 2.6, 2.7, 4.8 closed; blend
(M5 + 5.8), chamfer (B3), shell/offset (5.7b), draft (6.4), sweep (7.1),
loft (7.2), direct-edit (6.2, 6.3) matrices qualified with typed
both-sides boundaries; evolution complete for every covered family (B18,
6.5); stable Rust (O4.1c, O4.2a/b), JS/WASM (O4.4, O4.7), and Python
(O4.3a/b) surfaces; parity numbers published for the from-scratch
workflows with the O1.2f baseline pinned. Numeric bands are locked by
O1.2f, not before.

### H6 — Industrial interchange and corpus parity

Comparable outcomes on real supplier data, assemblies, and large models.
Gates: W3 passes with zero heal invocations on the tolerant path (3.4,
3.5, B1, B2 exit benchmark); AP242 structure, attributes, and the declared
PMI read profile qualified on CAx-IF models (O5.1, O5.2, O5.3a/b, O1.4b);
occurrence identity stable (O5.4); gauntlet stage pass rates within the
locked bands with the taxonomy public (O1.1c/d); zero silent-wrong,
crash, and unbounded-hang outcomes on the gated corpus; imported-model
boolean/blend/offset/direct-edit/tessellation/measurement stages within
bands; large-model memory and tail-latency budgets enforced (8.2, 8.6,
O3.2, O3.4; W8); deterministic concurrent sessions and supported parallel
operations (8.3, 8.4; 8.7 decided); a second external consumer with its
corpus in CI (O6.3, S6).

### H7 — Demonstrated technical leadership

At least five of the overlay's leadership claims LC1–LC13 meeting the
leadership discipline (stable corpus, equivalent quality, pinned
baseline, repeatable results, published losses, always-on gate), at
least two from the correctness family and one from the browser family.

Direct-edit follow-up under [P-Class 6.5](p-class-status.md):
[boundary-aware resizing of partial cylindrical faces](../roadmap/partial-cylinder-resize.md)
is planned; the Jolly Fox reproduction, scope, and acceptance criteria are recorded,
with implementation still pending.

### Correctness follow-up: curved hole winding (#278)

Cut/intersect assembly now compares multi-opening cylinder wires in
seam-unwrapped UV and preserves reversed coedge p-curves. Periodic
same-wound holes are validation errors. The cross-drilled shaft regression
covers raw GFA and public booleans across scales and bore angles, STEP
round trips, and the WASM render/measure matrix.

This exposed a false success in the cross-drilled rim fillet: its convex
edge received added material, and the malformed input had suppressed the
volume-sign gate. The corrected input now receives a transactional refusal.
Correct-side curved rim assembly remains B4/M5 work; this case is not a
qualified blend success.

### Correctness follow-up: wide spherical caps (#285)

Exact circular rims now enable the shared latitude-cap tessellator without
requiring a second trimmed face on the same sphere. Rim traversal selects
the retained pole, including caps larger than a hemisphere. The primitive
polygon-equator path stays unchanged. `regress_wide_sphere_cap.rs` checks
small and large caps, the radius-9/cut-7.5 ball-stud case, scales, rigid
transforms, two deflections, manifold meshes, closed-form volume, standalone
face area, and STEP. The packaged WASM consumer replays the generated
wide-cap STEP fixtures through the translator and kernel, checking
volume and direct/batch mesh quality, including the explicit doubled pole seam. Equal-axis ellipse representations of
circular rims use the same verified path.
This qualification covers circular rims with or without one doubled pole
seam; arbitrary non-circular trims are not included.

## §B Bridge backlog — owned by neither program

Ready items from the stabilization-plan residue, the capability-matrix
sweep, and the deferred-work inventory (2026-08-29). Each row is claimable
by a bounded session; update state in-place. Items that map onto an
existing program issue are listed there instead — notably: Steinmetz = 
P-Class 2.3 · conic boolean cells = O2.2 · offset self-intersection = 5.7
· e3b = O5.2 · error registry = O4.4 · seam/p-curve round-trips = 2.0.

| ID | Item | Where | Size | Why it matters | State |
|---|---|---|---|---|---|
| B1 | **Healing disclosure typing** — the matrix's only named Unsupported-untyped cell: permissive healing can mask an invalid result as valid. Type every repair (report what changed, refuse to claim validity it didn't verify); both-sides tests. | `heal/src/fix/`, `check/src/validate/` | M | The last untyped silent-failure path in the kernel; highest correctness value per line. Do first in the qualification lane. | **Done (2026-09-03, PR #243):** fixer results enumerate counted repair kinds and typed declined repairs; L2 `OK` explicitly means only “no fixer action,” never validity. Operations, facade verified mode, configurable direct WASM, named pipelines, and additive detailed direct/batch WASM surfaces commit only after independent operations/check validation. Invalid and unverifiable results return stable typed refusals with attempted repairs and roll back. Native and WASM both-sides regressions pin verified success and refusal. |
| B2 | **Boolean scale residuals** — 1e-5 fails closed (100·tol weld bands); raw-GFA 1e6 silently 0.9467 vs 0.8400 (ExactOnly refuses; measure + pin). | `algo` bands | M | Feeds P-Class 2.6 directly; the 1e6 cell is a possible silent-wrong class. Geometry lane. | Open |
| B3 | **Closed-rim chamfers** — cone-frustum band mirroring the validated toroidal fillet assembler; closed-form volume oracle. Stabilization C1.2. | `blend`, `operations/src/chamfer.rs` | M | Exact surfaces, cheap, passes chase filter 1; unblocks resize_blend cylinder/cone (C2). | Open |
| B4 | **v2 walking-trimmer completion** — the four named gaps: keep-side hint, shared contact edges, end-cap notch trim, chamfer external-tangent branch. Stabilization C1.3. | `blend/src/trimmer.rs` | M | Critical path for v2 walker parity → legacy engine retirement (M5 precondition). | Open |
| B5 | **Offset face provenance** — offset derives faces 1:1 and discards the mapping; journal real evolution instead of a barrier. | `offset`, `operations/src/offset_v2.rs` | S | The last declared-barrier operation nobody owns; closes the B3-residual from stabilization. | **Done (2026-09-02, PR #224 (landed via #233)):** default intersection-joint V2 offsets retain and validate the total 1:1 construction map; native and direct/batch WASM journal wrappers record it transactionally. Closed-form plane/volume, persistent-reference, rollback, and WASM parity oracles pin the claim. Arc-joint and self-intersection-removal variants explicitly refuse this map because later face synthesis/replacement needs richer provenance. |
| B6 | **Evidence matrices, batched** — the "Stable-but-blocked" ledger rows that are pure test work: primitives invalid-input/scale/postconditions; plane-section cavity+degeneracy; measurement curved-cavity+scale; sweeps degenerate/cavity + nonconvergence budgets; convex hull/Minkowski degenerates. One qualify_*.rs per family, stabilization-plan pattern. | `operations/tests/` | M (S per family) | Flips ~8 Blocked ledger rows with zero new geometry; ideal bounded-session work. | **Partial — primitive family done (2026-09-03):** box, cylinder, pointed cone, frustum, sphere, torus, and ellipsoid are qualified across 1e-3/1/1e3 scale by closed-form volume/bounds, exact entity/surface censuses, dual validators, oriented closed B-Rep, watertight/manifold mesh, independent mesh-volume, determinism, and direct/batch WASM parity (`qualify_primitives.rs`, WASM `qualify_primitives_tests.rs`). The invalid matrix also closed non-finite box/sphere/torus acceptance. The ellipsoid follow-up repaired hemisphere selection, exact rational preservation, shared-equator pole-cap tessellation, and polar bounds; `ellipsoid-tessellation-scale.json` pins the permanent replay. Plane-section, measurement, sweeps, and convex hull/Minkowski families remain open. |
| B7 | **Pave-block attachment for marched FF curves on curved faces** — the named canonical fix for the cross-face boundary-desync family; three cheaper altitudes already failed. | `algo/pave_filler/make_blocks.rs`, `phase_ff.rs` | L | Deepest structural payoff in algo; root-causes a whole non-manifold family. Geometry lane, coordinate with M2; repro `replay_scplate.rs`. | Open |
| B8 | **Reversed NURBS sub-span convention** — forward spans shipped; reversed validated sub-spans blocked on the same arrangement defect as B7. | `topology/src/edge.rs` | M | Completes the endpoint-trimmed contract 2.0 builds on. | Open (after/with B7) |
| B9 | **Torus ∖ coaxial cylinder tangent cut** — the single cell keeping torus booleans Beta; needs a tangent-contact primitive (explicitly NOT the band splitter). | `math/analytic_intersection.rs`, `algo` splitter | M | B1-ledger promotion Beta→Stable; closed-form oracle exists. Rides 2.7 tangency machinery. | Open |
| B10 | **Curve-curve / curve-surface classification qualification** + conic distance/classification cells | `math`, `geometry/extrema`, matrix harness | M | Unqualified since the matrix was written; sits under many families' claims; pure evidence. | Open |
| B11 | **Small hygiene set** — `log::debug!` false-zero in `fill_images_faces.rs` (diagnostic-infra bug); deterministic STEP entity ordering; heal `fix_duplicate_faces` winding-blind comparison; plane×plane sampled in-both exact upgrade; `n_fine` clamp hazard note→guard. | various | S each | Cheap, each has already cost or will cost a debugging session. | **Partial (2026-09-03, PR #239):** STEP export now canonicalizes unordered face, void-shell, and hole-loop aggregates while preserving semantic coedge traversal order; byte-equality regressions cover reordered faces and void shells. Remaining: false-zero diagnostic, winding-aware duplicate-face healing, plane×plane exact upgrade, and `n_fine` guard. |
| B12 | **Holes on non-planar section caps** — annular Coons or cap-then-subtract vs extruded-annulus ground truth (stabilization B2.2). | `operations/src/cap.rs`, `fill_face.rs` | M | Largest remaining non-planar-cap value with clean ground truth. H3, with M7 cap work. | **Partial (2026-09-04, PR #252):** sweep and pipe caps preserve disjoint rectangular iso-parametric holes on four-sided bilinear caps, matched against an independently extruded annulus by converged volume, manifold B-Rep, watertight mesh, classification, and direct/batch WASM. Off-surface, curved, touching, and n-sided holed trims refuse typed; loft-hole correspondence and holed partial revolutions remain Unsupported-typed and ride M7's cap work. |
| B13 | **STEP inner-shell (voids) export** — emit and read `BREP_WITH_VOIDS`, preserving cavity shell count and volume. | `io/src/step/{writer,reader}.rs` | S–M | Round-trip honesty for hollow parts; gauntlet round-trip stage will hit it. | Complete (2026-09-04): one- and two-void regressions verify single-solid round trips, shell counts, and volume. |
| B15 | **Cut/intersect pocket-face orientation on cylinder walls** | `crates/operations/tests/regress_parallel_boss_band_sections.rs`, `algo` assembly | M | **Done (2026-09-04, PR #255):** non-fuse assembly normalizes selected cylinder outer/inner wire winding before edge merge. Box and cylinder tools on both wall sides pass exact cut/intersect, dual validation, closed-form volume, material classification, and welded-mesh orientation oracles. The formerly ignored regression is permanent coverage. | Done |
| B14 | **Render promotion track** — Experimental→Beta after a contract-stable release cycle (stabilization C4 residue); outside both programs. | `render` | S (time-gated) | Cleans the last stabilization row. | Open |
| B16 | **Consumer topology-query API set** — one binding per OpenZCAD heuristic it currently reimplements (its roadmap C2): trimmed edge parameter domain, face material sense, ordered wire traversal, per-edge convexity, sphere-patch identity, seam-edge parity, `maxFilletRadius(solid, edges)`, batched `classifyPoint`, per-edge ids in `meshEdgesAll`; plus the GCS qualification matrix (constraint type × system state × scale, nonconvergence budget) from the P-Class §6 inherited queue. Each: exact, typed refusal on foreign handles, direct + batch WASM, contract test. Added 2026-09-04 by the [industrial-parity overlay](industrial-parity.md) (rows IP-15.9, IP-9.3, IP-10.3, IP-13.1/13.4). | `wasm/bindings/query.rs`, `batch.rs`, `operations/src/query.rs`, `sketch/`, `operations/tests/qualify_gcs.rs` (new) | S each | Every row retires an adapter-side heuristic; highest OpenZCAD impact per line. Exit: the named heuristic deleted from the consumer's adapter (recorded in the PR), matrix green. | Open |
| B17 | **Healing defect-class qualification matrix** — a generated defect class × severity × repair policy × scale matrix per fixer (wire order/closure/gaps/small edges, face orientation/small faces, seams, shell orientation/sewing/free bounds, duplicates, continuity splits, representation conversion), plus an operand self-interference report for booleans and the faceted-import sew/unify contract (issue #244). Every cell: verified repair with counted disclosure, or typed refusal; both sides. Added 2026-09-04 by the overlay (rows IP-8.3, IP-3.8, IP-8.6). | `heal/`, `operations/src/heal.rs`, `operations/tests/qualify_heal.rs` (new), `stl/import.rs` | M (S per fixer) | The family is Qualified only at the B1 boundary; the reference kernel's healing breadth is its strongest documented area. Exit: every fixer has a matrix; #244 fixture green; self-interference report typed on a self-touching corpus. | Open |
| B18 | **Evolution completeness audit** — every topology-producing family reports total attribution or a typed unresolved record: unify same-domain (`unify_with_evolution`, OpenZCAD C1's top ask), sew, sweep/loft/revolve/extrude caps, arc-joint and self-intersection-removal offsets, section/split edges, direct edits (with 6.5), edge/vertex events beyond booleans. Added 2026-09-04 by the overlay (rows IP-3.6, IP-5.7, IP-12.1; leadership claim LC3). | `journal_ops.rs`, `evolution.rs`, `qualify_evolution_coverage.rs`, per-op modules | M (S per family) | Absolute gate §3.4 item 8; unblocks OpenZCAD's adoption order boolean → pattern → chamfer → shell/offset → direct edits. Exit: the coverage fixture claims every result face of every family exactly once or pins its typed unresolved; no `record_barrier_over_solid` call remains for a family that can construct its map. | Open |
| B19 | **Remaining fuzz slices and mutation scope** — curve-intersection, offset, GCS, and tessellation fuzz targets with independent oracles on the weekly schedule. The mutation-scope slice moves the previously undiscovered root config to `.cargo/mutants.toml` and selects the current CDT directory. | `fuzz/fuzz_targets/`, `.cargo/mutants.toml`, `.github/workflows/fuzz.yml` | S each | Exit: four targets scheduled with committed seeds; mutants report shows CDT mutants examined. Scope regression checks reject ignored config and the stale CDT file glob; the bounded five-mutant sample caught four and retained one survivor for review. | Partial — mutation scope verified; four fuzz slices remain open. Evidence: `scripts/test-mutants-scope.py`, `docs/kernel-maturity/testing-strategy.md` |
| B20 | **Exact measurement completion (K-S2 remainder)** — ellipse, hyperbola, and NURBS planar boundaries; general curved-face area; deflection-independent curved-body volume, centroid, and inertia by Gauss quadrature over exact geometry with a stated bound; direct + batch WASM; scale matrix. Added 2026-09-04 by the overlay (row IP-10.1). | `check/src/properties/`, `operations/src/measure/` | M | OpenZCAD S2 measures 0.2–3.5 % volume error on filleted parts at its display deflection; the reference kernel integrates surfaces directly. Exit: relative error ≤ 1e-6 against closed forms on filleted and cavity primitives at 1e-3/1/1e3, independent of caller deflection; ledger row loses its "incomplete" caveat. | Open |

**Explicitly not queued** (decided or terminal — do not re-open without
the named primitive): IGES growth (C3, decided), box∪sphere and torus∩box
census rows (TERMINAL → O2.3 re-opens them properly), universal
duplicate-edge merge key (proven unbuildable), mesh co-refinement
watertightness (below the chase filter until a live case routes there),
kumiko lattice family (probe only per the roadmap skill's engine-side
question), v1-fillet API migration (product decision, owner's),
monolithic mechanical-feature operators and tessellated-STEP read
(overlay §1: composable from imprint/boolean, and no corpus pull),
non-manifold shared-face topology (RFC 0005 later RFC; owner decision).

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
- **Consumer-impact session:** B16 (one query binding), O4.7, B20, or
  B18's unify item — each retires a named OpenZCAD adapter heuristic.
- **Parity-evidence session:** O1.2d–e (Remus-only rows first), O1.5,
  B19; see the overlay's §8 ranking.
- **Owner-only:** O4.2c/O4.3c publishes, O6.2 hosting, O6.3 outreach,
  v1-fillet migration decision.

Maintenance rule: any PR that changes an item's state updates its row
here (or its program ledger) in the same PR — same discipline as the
skill's living-document mandate.
