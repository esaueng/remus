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
| [Kernel performance audit and roadmap](../performance/kernel-optimization-roadmap.md) | 111 measured or evidence-gated optimization work packages, nine execution phases, dependencies and reproduction evidence | Non-owning audit; implementation state stays in this roadmap and the program ledgers |

The work-selection *doctrine* (chase filters, TERMINAL list, acceptance bar,
durable lessons) remains `.claude/skills/roadmap/SKILL.md`; this page is
the *queue*; the narrative behind closed rows is `campaign-history.md`. All
are living documents: update the relevant row in the same PR that changes
its state. Before claiming anything: `gh pr list --state open` (R6).

- **Drafted:** 2026-08-29. **Last reconciled:** 2026-09-11 against `main` @
  `95de160` (regenerate with `git rev-parse --short origin/main` when touching
  §H0; do not hand-type a baseline older than the section it heads).
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

Reconciled 2026-09-11 UTC at `95de160` (merges through #396). Open at
inspection: none (no open PRs, no open issues). Since the previous
reconciliation at `63e388c`: #382 (translation endpoint certificates) and
the package refresh #393 merged; the hammer-holder perf campaign landed as
a stacked train (#389 NURBS weight cache, #394 edge-face box gating, #395
distance-query NURBS pruning; see B28) with #391 (`unifyFacesChecked`, a
B16 row) on top; and #396 fixed converted-B-spline planes measuring 29 %
high and losing a fused peg, and disclosed the first named P-Class 2.5
slices. The previous reconciliation's notes: the contract lane closed
(B21 #356, B22 #350, B23 #359), typed binary boolean results (#354) and the
native/wasm quadric parity matrix (#355) landed, and the hammer-holder
opening stack merged as a linear train (#363, #375, #366, #367, #368, #370,
#371, #373, #374; see B27). #363's opposing-face clip regressed the
deepened-notch fixture to eight unpaired edges and was fixed in-branch
before the train (two arrangement gaps, recorded in the skill's lessons).
The 2026-09-06 mutation survivors are triaged (#381). The bounded quadric,
scale, tangency (#307), quarter-wall (#308), seam (#312), persistent
edit/healing integration (#338), cylindrical blend resize (#346), and
zero-radius blend removal (#348) work remains merged. The detailed
[6.5/B18 audit](evolution-audit.md) records source/test evidence and
remaining history gaps. Recheck live PRs and runs before claiming new work.

**Proof gates, as of the reconciliation (verified, not inherited):**

- Workspace suites: `cargo nextest run --workspace` 5,024 passed, 0 failed
  at #396 (its PR body). Corpus Gauntlet: green on every daily run through
  2026-09-11. Fuzz Smoke: green on its 2026-09-06 scheduled run. OSV: green.
  Both weekly jobs run again on 2026-09-13; the mutation verdict there
  covers the perf train's diff and is a new finding, not the one below.
- **Mutation Testing: red on 2026-09-06, triaged 2026-09-10.** The weekly
  job mutates only that week's diff (`--in-diff`), so the nine survivors
  were the week's new code, not the whole scope. Disposition: two were the
  `bench-internals`-only `BenchPlane` surface, unreachable from any test
  build and now excluded by `exclude_re` in `.cargo/mutants.toml`; six
  were real coverage gaps, each pinned by a unit test that was verified to
  fail under the hand-applied mutant (`HealingReport::total`,
  `sphere_loop_projected_area`'s circle `v`-term, the coaxial loft radius
  guard, `wire_surface_alignment`'s sample-count guard, the rolling-ball
  rational-arc control point, the variable-fillet corner-radius band); one
  (`tessellate_nurbs_pole_cap_shared`'s `> idx_save` return) is equivalent
  under the `ring.len() >= 3` guard above it and left as is. The next
  scheduled run judges the following week's diff; a red there is a new
  finding, not this one.
- **`main` CI is cancelled on every fast merge.** `ci.yml` uses
  `cancel-in-progress: true` keyed on `github.ref`, so merge commits #338,
  #339, #341, #346, and #349 never received a completed CI verdict on their
  own head; only the newest push in a burst is proven. Fix queued as B22.
  Until it lands, treat "CI green on main" as "green on the last push in
  the burst", and re-run the workflow for a specific merge before quoting it.

**Open correctness/API findings from the 2026-09-09 review** (queued as
§B rows B21–B26): the plain `boolean()` / WASM `fuse`/`cut`/`intersect`
entry points return a mesh-fallback result with only a `log::warn!` as
disclosure (B21, closed 2026-09-10); three fillet engines ran under two
different cascade orders and the Rust facade used a third policy (B23,
closed 2026-09-10); ~180 wildcard match
arms over `EdgeCurve`/`FaceSurface` have no lint or CI gate (B24); three
public offset paths (B25); generative coverage is 15 proptest blocks and 8
golden files against ~445k lines (B26).

2.4 remains partial pending its integration qualification; 2.6 retains the
remaining band and anisotropic precision audit. 2.7 has a merged bounded
162-cell exact-or-typed contract, including 42 explicit refusals; this does
not qualify general exact tangency. 6.5 has qualified direct-edit and
healing history, including journaled single-cylinder blend resize and
zero-radius removal on planar supports. Broader blend resize, ambiguous
boundaries, and later direct-edit geometry still prevent full closure.
B18 retains faces-only and unjournaled families. The previous detailed
quadric/tangency checkpoint is preserved in
[campaign history](campaign-history.md#pre-integration-quadric-and-tangency-checkpoint-archived-2026-09-09).

OpenZCAD consumer-roadmap K-S4 (`approx_census` CI enforcement): **done (PR
#140)**. Its authoritative disposition line remains in planning PR
esaueng/OpenZCAD#140 so the two repositories retain separate commit streams.

### H1 — current work queue

Ordered by consumer impact per line (reprioritized 2026-09-09; the previous
ordering front-loaded B18 journaling slices, of which nine landed in a row
while B4 and B16 stayed untouched). The contract lane that headed this list
(B22, B21, B23) closed on 2026-09-09/10 and was removed on 2026-09-11.

1. **Consumer lane:** B16, one query binding per PR, each deleting a named
   OpenZCAD adapter heuristic (one row, `unifyFacesChecked`, landed in #391
   and awaits its adapter-deletion link); then B20. Prefer this over any
   further B18 slice until B16 has at least three rows closed.
2. **Geometry lane (one session at a time in algo/pave-filler):** B4 (v2
   trimmer, the M5 precondition and the un-refusal path for the two known
   damaged-success fillet cases), then verify 2.4d against the merged
   bounded quadric matrices, then the remaining 2.6/2.7 witnesses. Expand
   2.5 through the two slices #396 named (curved-NURBS trimmed FF extent and
   transversal EF crossings; the plane×cylinder LSPIA fit error); build
   missing arrangement primitives only when a pinned case requires them.
3. **Qualification lane:** B24 (wildcard-arm conversion in the four densest files); B26
   (boolean invariant proptests); complete B17/B19 matrices and remaining
   B6 evidence.
4. **Perf lane (bounded, bench-pinned):** B28 — every PR carries a before/after
   Criterion or scaling-guard number and changes no tolerance, sample count, or
   threshold; result-identical by construction or it is not a perf PR.
5. **Baseline/history lane (after lanes 1–2 have moved):** remaining
   6.5/B18 families one per PR, per the
   [audit queue](evolution-audit.md#next-bounded-slices).

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

**Definition of v1.0** (the first stable publish, O4.2c), as four
countable gates. Each is a number a session can regenerate; quote the
number, not the adjective.

| Gate | Measure | 2026-09-09 |
|---|---|---|
| Exit benchmarks | P-Class B1–B5 green as permanent tests | 0 / 5 |
| Scoreboard | Open Kernel S1–S7 claims live | 0 / 7 |
| Capability matrix | cells listed under "Known Unsupported-untyped / Partial cells" in [capability-matrix.md](capability-matrix.md) | 2 named families (plane/cylinder tangency, sliver crossings) |
| Bridge backlog | §B rows not Done or explicitly re-triaged | 21 of 28 (2026-09-11) |

Anything short of all four at zero/full publishes as 0.x. Gate 3 was
previously worded "zero Unsupported-untyped cells" while B1 claimed to be
the only such cell; the matrix disagrees, so the matrix section is the
authority from now on.

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

### Merged slice reports (moved to campaign history)

The direct-edit (#308/#338), healing (#338), curved-hole-winding (#278), and
wide-spherical-cap (#285) slice reports that used to sit here are archived in
[campaign-history.md](campaign-history.md#direct-edit-healing-and-correctness-slice-reports-archived-2026-09-09).
Current obligations are in the [6.5/B18 audit](evolution-audit.md) and the
§B rows below; nothing in the archive is an open claim.

## §B Bridge backlog — owned by neither program

Ready items from the stabilization-plan residue, the capability-matrix
sweep, and the deferred-work inventory (2026-08-29). Each row is claimable
by a bounded session; update state in-place. Items that map onto an
existing program issue are listed there instead — notably: Steinmetz = 
P-Class 2.3 · conic boolean cells = O2.2 · offset self-intersection = 5.7
· e3b = O5.2 · error registry = O4.4 · seam/p-curve round-trips = 2.0.

| ID | Item | Where | Size | Why it matters | State |
|---|---|---|---|---|---|
| B1 | **Healing disclosure typing** — the matrix's only named Unsupported-untyped *healing* cell (the matrix still lists tangency and sliver families; see H4): permissive healing can mask an invalid result as valid. Type every repair (report what changed, refuse to claim validity it didn't verify); both-sides tests. | `heal/src/fix/`, `check/src/validate/` | M | The last untyped silent-failure path in the kernel; highest correctness value per line. Do first in the qualification lane. | **Done (2026-09-03, PR #243):** fixer results enumerate counted repair kinds and typed declined repairs; L2 `OK` explicitly means only “no fixer action,” never validity. Operations, facade verified mode, configurable direct WASM, named pipelines, and additive detailed direct/batch WASM surfaces commit only after independent operations/check validation. Invalid and unverifiable results return stable typed refusals with attempted repairs and roll back. Native and WASM both-sides regressions pin verified success and refusal. |
| B2 | **Boolean scale residuals** — the through-tool family now returns exact material at 1e-5 and 1e6; straight-edge refinement and local planar bands are qualified by 72 operator/scale/placement cells. | `algo` bands | M | Feeds P-Class 2.6; remaining dimensional and curved-band work is tracked in [the audit](scale-band-audit.md). | Partial — named matrix passes; broader audit remains |
| B3 | **Closed-rim chamfers** — cone-frustum band mirroring the validated toroidal fillet assembler; closed-form volume oracle. Stabilization C1.2. | `blend`, `operations/src/chamfer.rs` | M | Exact surfaces, cheap, passes chase filter 1; unblocks resize_blend cylinder/cone (C2). | Open |
| B4 | **v2 walking-trimmer completion** — the four named gaps: keep-side hint, shared contact edges, end-cap notch trim, chamfer external-tangent branch. Stabilization C1.3. | `blend/src/trimmer.rs` | M | Critical path for v2 walker parity → legacy engine retirement (M5 precondition). | **Partial — planar trimmer done (this PR):** keep-side `AwayFrom` + spine-edge authority, shared contact-edge adoption, two/three-edge end-cap notch, scale-aware parallel gate, vertex sharing across trims, solid-scoped live-wire stitch gate, multi-edge stitch runs, and reversed-loop rewinding are implemented with unit + concave-notch fillet/chamfer pins enabled (`regress_fillet_concave_notch`, `regress_chamfer_obtuse_ridge`, traversal-side analytic tests). The two e2e fixtures remain honest typed refusals: the blend-adjacent second pass (contact endpoints 0.5 off every boundary carrier) and the gridfinity peak rim (endpoints 0.6 = one radius off) both bottom out in curved-face trimming, which is M5 remainder work, not one of the four planar gaps. |
| B5 | **Offset face provenance** — offset derives faces 1:1 and discards the mapping; journal real evolution instead of a barrier. | `offset`, `operations/src/offset_v2.rs` | S | The last declared-barrier operation nobody owns; closes the B3-residual from stabilization. | **Done (2026-09-02, PR #224 (landed via #233)):** default intersection-joint V2 offsets retain and validate the total 1:1 construction map; native and direct/batch WASM journal wrappers record it transactionally. Closed-form plane/volume, persistent-reference, rollback, and WASM parity oracles pin the claim. Arc-joint and self-intersection-removal variants explicitly refuse this map because later face synthesis/replacement needs richer provenance. |
| B6 | **Evidence matrices, batched** — the "Stable-but-blocked" ledger rows that are pure test work: primitives invalid-input/scale/postconditions; plane-section cavity+degeneracy; measurement curved-cavity+scale; sweeps degenerate/cavity + nonconvergence budgets; convex hull/Minkowski degenerates. One qualify_*.rs per family, stabilization-plan pattern. | `operations/tests/` | M (S per family) | Flips ~8 Blocked ledger rows with zero new geometry; ideal bounded-session work. | **Partial — primitive family done (2026-09-03):** box, cylinder, pointed cone, frustum, sphere, torus, and ellipsoid are qualified across 1e-3/1/1e3 scale by closed-form volume/bounds, exact entity/surface censuses, dual validators, oriented closed B-Rep, watertight/manifold mesh, independent mesh-volume, determinism, and direct/batch WASM parity (`qualify_primitives.rs`, WASM `qualify_primitives_tests.rs`). The invalid matrix also closed non-finite box/sphere/torus acceptance. The ellipsoid follow-up repaired hemisphere selection, exact rational preservation, shared-equator pole-cap tessellation, and polar bounds; `ellipsoid-tessellation-scale.json` pins the permanent replay. Plane-section, measurement, sweeps, and convex hull/Minkowski families remain open. |
| B7 | **Pave-block attachment for marched FF curves on curved faces** — the named canonical fix for the cross-face boundary-desync family; three cheaper altitudes already failed. | `algo/pave_filler/make_blocks.rs`, `phase_ff.rs` | L | Deepest structural payoff in algo; root-causes a whole non-manifold family. Geometry lane, coordinate with M2; repro `replay_scplate.rs`. | Open |
| B8 | **Reversed NURBS sub-span convention** — forward spans shipped; reversed validated sub-spans blocked on the same arrangement defect as B7. | `topology/src/edge.rs` | M | Completes the endpoint-trimmed contract 2.0 builds on. | Open (after/with B7) |
| B9 | **Torus ∖ coaxial cylinder tangent cut** — the single cell keeping torus booleans Beta; needs a tangent-contact primitive (explicitly NOT the band splitter). | `math/analytic_intersection.rs`, `algo` splitter | M | B1-ledger promotion Beta→Stable; closed-form oracle exists. Rides 2.7 tangency machinery. | Open |
| B10 | **Curve-curve / curve-surface classification qualification** + conic distance/classification cells | `math`, `geometry/extrema`, matrix harness | M | Unqualified since the matrix was written; sits under many families' claims; pure evidence. | Open |
| B11 | **Small hygiene set** — `log::debug!` false-zero in `fill_images_faces.rs` (diagnostic-infra bug); deterministic STEP entity ordering; heal `fix_duplicate_faces` winding-blind comparison; plane×plane sampled in-both exact upgrade; `n_fine` clamp hazard note→guard. | various | S each | Cheap, each has already cost or will cost a debugging session. | **Partial (2026-09-03, PR #239):** STEP export now canonicalizes unordered face, void-shell, and hole-loop aggregates while preserving semantic coedge traversal order; byte-equality regressions cover reordered faces and void shells. Remaining: false-zero diagnostic, winding-aware duplicate-face healing, plane×plane exact upgrade, and `n_fine` guard. |
| B12 | **Holes on non-planar section caps** — annular Coons or cap-then-subtract vs extruded-annulus ground truth (stabilization B2.2). | `operations/src/cap.rs`, `fill_face.rs` | M | Largest remaining non-planar-cap value with clean ground truth. H3, with M7 cap work. | **Partial (2026-09-04, PR #252):** sweep and pipe caps preserve disjoint rectangular iso-parametric holes on four-sided bilinear caps, matched against an independently extruded annulus by converged volume, manifold B-Rep, watertight mesh, classification, and direct/batch WASM. Off-surface, curved, touching, and n-sided holed trims refuse typed; loft-hole correspondence and holed partial revolutions remain Unsupported-typed and ride M7's cap work. |
| B13 | **STEP inner-shell (voids) export** — emit and read `BREP_WITH_VOIDS`, preserving cavity shell count and volume. | `io/src/step/{writer,reader}.rs` | S–M | Round-trip honesty for hollow parts; gauntlet round-trip stage will hit it. | Complete (2026-09-04): one- and two-void regressions verify single-solid round trips, shell counts, and volume. |
| B14 | **Render promotion track** — Experimental→Beta after a contract-stable release cycle (stabilization C4 residue); outside both programs. | `render` | S (time-gated) | Cleans the last stabilization row. | Open |
| B15 | **Cut/intersect pocket-face orientation on cylinder walls** | `crates/operations/tests/regress_parallel_boss_band_sections.rs`, `algo` assembly | M | **Done (2026-09-04, PR #255):** non-fuse assembly normalizes selected cylinder outer/inner wire winding before edge merge. Box and cylinder tools on both wall sides pass exact cut/intersect, dual validation, closed-form volume, material classification, and welded-mesh orientation oracles. The formerly ignored regression is permanent coverage. | Done |
| B16 | **Consumer topology-query API set** — one binding per OpenZCAD heuristic it currently reimplements (its roadmap C2): trimmed edge parameter domain, face material sense, ordered wire traversal, per-edge convexity, sphere-patch identity, seam-edge parity, `maxFilletRadius(solid, edges)`, batched `classifyPoint`, per-edge ids in `meshEdgesAll`; `unifyFacesChecked` (the strict input/result verdicts `unify_faces` already computes, so the adapter's union gate stops re-validating the same body up to five times per rebuild; added 2026-09-11); plus the GCS qualification matrix (constraint type × system state × scale, nonconvergence budget) from the P-Class §6 inherited queue. Each: exact, typed refusal on foreign handles, direct + batch WASM, contract test. Added 2026-09-04 by the [industrial-parity overlay](industrial-parity.md) (rows IP-15.9, IP-9.3, IP-10.3, IP-13.1/13.4). | `wasm/bindings/query.rs`, `batch.rs`, `operations/src/query.rs`, `sketch/`, `operations/tests/qualify_gcs.rs` (new) | S each | Every row retires an adapter-side heuristic; highest OpenZCAD impact per line. Exit: the named heuristic deleted from the consumer's adapter (recorded in the PR), matrix green. | **Partial (2026-09-11):** `unifyFacesChecked` shipped in PR #391 (native `heal::unify_faces_checked` + `UnifyFacesReport`, WASM binding, batch parity, packages 2.130.13); the row closes when the OpenZCAD adapter PR that deletes the repeated `validateSolid` calls is linked here. Every other row is open. |
| B17 | **Healing defect-class qualification matrix** — a generated defect class × severity × repair policy × scale matrix per fixer (wire order/closure/gaps/small edges, face orientation/small faces, seams, shell orientation/sewing/free bounds, duplicates, continuity splits, representation conversion), plus an operand self-interference report for booleans and the faceted-import sew/unify contract (issue #244). Every cell: verified repair with counted disclosure, or typed refusal; both sides. Added 2026-09-04 by the overlay (rows IP-8.3, IP-3.8, IP-8.6). | `heal/`, `operations/src/heal.rs`, `operations/tests/qualify_heal.rs` (new), `stl/import.rs` | M (S per fixer) | The family is Qualified only at the B1 boundary; the reference kernel's healing breadth is its strongest documented area. Exit: every fixer has a matrix; #244 fixture green; self-interference report typed on a self-touching corpus. | Open |
| B18 | **Evolution completeness audit** — every topology-producing family reports total attribution or a typed unresolved record: unify same-domain (`unify_with_evolution`, OpenZCAD C1's top ask), sew, sweep/loft/revolve/extrude caps, arc-joint and self-intersection-removal offsets, section/split edges, direct edits (with 6.5), edge/vertex events beyond booleans. Added 2026-09-04 by the overlay (rows IP-3.6, IP-5.7, IP-12.1; leadership claim LC3). | `journal_ops.rs`, `evolution.rs`, `qualify_evolution_coverage.rs`, per-op modules | M (S per family) | Absolute gate §3.4 item 8; unblocks OpenZCAD's adoption order boolean → pattern → chamfer → shell/offset → direct edits. Exit: the coverage fixture claims every result face of every family exactly once or pins its typed unresolved; no `record_barrier_over_solid` call remains for a family that can construct its map. | Partial — direct-edit and healing slices merged in #338; face-only and unjournaled families remain. See [audit](evolution-audit.md). |
| B19 | **Remaining fuzz slices and mutation scope** — curve-intersection, offset, GCS, and tessellation fuzz targets with independent oracles on the weekly schedule. The mutation-scope slice moves the previously undiscovered root config to `.cargo/mutants.toml` and selects the current CDT directory. | `fuzz/fuzz_targets/`, `.cargo/mutants.toml`, `.github/workflows/fuzz.yml` | S each | Exit: four targets scheduled with committed seeds; mutants report shows CDT mutants examined. Scope regression checks reject ignored config and the stale CDT file glob; the bounded five-mutant sample caught four and retained one survivor for review. | Partial — mutation scope verified; four fuzz slices remain open. Evidence: `scripts/test-mutants-scope.py`, `docs/kernel-maturity/testing-strategy.md` |
| B20 | **Exact measurement completion (K-S2 remainder)** — ellipse, hyperbola, and NURBS planar boundaries; general curved-face area; deflection-independent curved-body volume, centroid, and inertia by Gauss quadrature over exact geometry with a stated bound; direct + batch WASM; scale matrix. Added 2026-09-04 by the overlay (row IP-10.1). | `check/src/properties/`, `operations/src/measure/` | M | OpenZCAD S2 measures 0.2–3.5 % volume error on filleted parts at its display deflection; the reference kernel integrates surfaces directly. Exit: relative error ≤ 1e-6 against closed forms on filleted and cavity primitives at 1e-3/1/1e3, independent of caller deflection; ledger row loses its "incomplete" caveat. | Open |
| B21 | **Boolean fallback disclosure by default** — `boolean()`, `fuse`/`cut`/`intersect`, `fuseAll`, and the `*WithOptions`/`*WithEvolution` bindings return a bare handle when the GFA path falls over to the mesh (co-refinement) boolean; the only disclosure is `log::warn!(target: "remus_approx")` in `boolean/mod.rs::run_mesh_fallback`, and `OperationContext::new()` defaults to `AllowApproximate { budget: 0.1 }`. The Rust facade already returns `BooleanOutcome`. Decide one of: default to `ExactOnly` with `booleanWithQuality` as the opt-in, or return quality from every entry point. Found 2026-09-09. | `operations/src/boolean/mod.rs`, `math/src/context.rs`, `wasm/src/bindings/booleans.rs`, `remus/src/model.rs` | S–M | A log line is not a return value; an "exact" kernel whose consumer renders the handle silently ships NURBS-degraded faces. Same contract on Rust and JS. Breaking for JS callers that relied on the silent fallback: changelog + adapter notice. | **Done (2026-09-10):** plain Rust and WASM boolean entry points (handle-returning, options, evolution, `fuseAll`, batch) run `ExactOnly` and return the typed refusal; approximation is reachable only through `boolean_with_context` / `booleanWithQuality` and the new `boolean_outcome_with_options`, all of which return the disclosed quality. Tangent-boss regression pins refusal, rollback, and disclosure natively and in batch; changelogs carry the breaking notice. |
| B22 | **`main` CI cancellation** — `ci.yml` `concurrency.group: ci-${{ github.ref }}` with `cancel-in-progress: true` cancels the CI run of every merge commit that is followed by another merge within ~30 min (#338, #339, #341, #346, #349 on 2026-09-09 have `conclusion: cancelled`). Scope cancellation to `pull_request` refs (e.g. `cancel-in-progress: ${{ github.event_name == 'pull_request' }}`) so every `main` head gets a verdict. | `.github/workflows/ci.yml` | S | Every CI-green claim on `main` is currently "green on the last push in the burst". Also affects the committed-package refresh evidence. | **Done (2026-09-09):** `cancel-in-progress` is now `github.event_name == 'pull_request'`; the same change tiers the suite (`docs/owner-pr-ci.md` § Tiers) so kernel PRs run ~12 min of jobs and the publisher's refresh PR runs none. |
| B23 | **One fillet cascade policy** — WASM `fillet` runs v2 → rolling-ball → bevel (`wasm/src/helpers.rs::try_fillet`), `blend_ops.rs::planar_fillet_result` orders a legacy attempt first for concave edges, and the facade `Model::fillet` is v2-only; the deprecated v1 engines are reached in production through `#[allow(deprecated)]`. Move the cascade into `operations` behind one function both surfaces call, with the engine that produced the result disclosed in the outcome. Does NOT decide v1 retirement (owner's product decision, still "not queued"). | `wasm/src/helpers.rs`, `operations/src/blend_ops.rs`, `operations/src/fillet/`, `remus/src/model.rs` | M | Rust and JS callers get different geometry for the same request today; a rejected attempt can leave the input partly filleted unless rollback is explicit (documented trap). Precondition for honest M5 parity numbers. | **Done (2026-09-10):** `blend_ops::fillet_cascade` is the one policy — walking engine (`fillet_v2`, which already tries the rolling-ball rebuild first for planar-line selections), then the guarded rolling-ball rebuild, each transactional; the flat bevel is no longer a fillet fallback. WASM `fillet` / `filletWithEvolution` / batch `fillet`, the facade `Model::fillet`, and `fillet_with_evolution` all call it, and `BlendResult::engine` discloses which engine produced the result. v1 retirement is still not decided. |
| B24 | **Wildcard-arm audit and gate** — ~180 `_ =>` arms over `EdgeCurve`/`FaceSurface` (densest, regenerated 2026-09-11 at `95de160` with the skill's `rg --multiline` query, 180 total: `measure/volume.rs` 16, `pave_filler/phase_ff.rs` 16, `tessellate/nonplanar.rs` 12, `resize_blend.rs` 11). A new variant compiles clean and silently takes the approximate branch in volume, meshing, and GFA splitting. Convert the four densest files to exhaustive matches; add a `scripts/check-wildcard-arms.sh` ratchet (count per file, no growth) to the `repo-policy` job, same pattern as `check-det-hash.sh`. Companion to RFC 0006 (face-surface wildcard audit). | `operations/src/measure/volume.rs`, `algo/src/pave_filler/phase_ff.rs`, `operations/src/tessellate/nonplanar.rs`, `operations/src/resize_blend.rs`, `scripts/` | M | The only current defence is `approx_census`, which detects drift after the fact. RFC 0006 swept-analytic surfaces will add a variant and hit every one of these. | Open |
| B25 | **Offset path consolidation** — `offset_v2` (wrapper over `remus-offset`), sampled `offset_face` (public `samples: u32` knob), and `offset_trim` are all exposed independently through WASM (`bindings/operations.rs`, `batch.rs`), and `shell_op.rs` still calls the sampled `offset_face` internally. Route `shell_op` through the exact offset where the face family allows it and mark the sampled binding as approximate in its result/type, or fold it. | `operations/src/offset_face.rs`, `offset_trim.rs`, `shell_op.rs`, `wasm/src/bindings/operations.rs` | M | A discretization knob on the public API contradicts the exact-kernel contract; three ways to offset is a support burden for the adapter. | Open |
| B27 | **Hammer-holder opening edit (OpenZCAD consumer case)** — imported Shapr3D holder, 46 → 50 mm opening by a mask cut, shifted-source intersection, and fuse. Not a §B chase by filter 2 (a general NURBS-import boolean chain), queued here because ten merged PRs of geometry-lane work had no owner row. Status doc: [hammer-holder-opening-status.md](../hammer-holder-opening-status.md); fixtures `crates/io/tests/hammer_opening_partition.rs`, diagnostic `crates/io/examples/hammer_opening.rs`. | `algo/builder/fill_images_faces.rs`, `face_splitter/`, `pave_filler/phase_ff.rs` | L | The first consumer-driven imported-model edit to complete exact-only end to end; every repair on the way (branch retention, tolerance carry-through, curved sections, torus-patch classification, concave subdivision) is generic. Its residual risk is the calibration web: every PR in the stack touched the section/clip/arrangement path, and the deepened-notch foil caught one regression. | **Partial (2026-09-10, #357, #361, #363–#375):** the specific replay completes all eight exact-only booleans natively (36-face shifted intersection, 194-face fuse, dimensions/bores/lettering preserved, strict validation, watertight mesh, STEP round trip) and through the WASM smoke path. Not qualified: general parameter ranges, OpenZCAD parameter/history references, AI editing path, and any second imported model. |
| B26 | **Boolean and mesh invariant proptests** — the workspace has 15 proptest blocks (13 in `math`) and 8 golden files against ~445k lines, while roughly half of `scripts/` tests the CI policy itself. Add property tests over random primitive pairs and rigid transforms: inclusion–exclusion volume identity (A ∪ B + A ∩ B = A + B), fuse/cut complement, translation invariance of `solid_volume` (the doubled-boundary detector), watertight/manifold mesh, exact-only path stability under 1e-13 nudges. | `operations/tests/prop_boolean_invariants.rs` (new), `operations/src/boolean/tests.rs` | S–M | Cheapest broad oracle the kernel lacks; every lesson in the roadmap skill about doubled boundaries and translation-variant volume is a property nobody generates. Rides B19's schedule for the slow variants. | Open |
| B28 | **NURBS-heavy imported-model interactive perf (OpenZCAD rebuild chain)** — the growing hammer-holder rebuild (esaueng/OpenZCAD#275: 42 NURBS faces, 38×58 bicubic patches) spent 86 % of every heavy stage in `NurbsSurface::derivatives`, projected 730k edge×NURBS-face pairs per fuse, and re-validated one body five times. Owner row for perf work on this chain; every PR is result-identical by construction (no tolerance, sample, or threshold change) and pins a before/after number. Remaining named stages: strict `validate_solid` ≈ 1 s native for one 42-face body (the inside-out signed-volume integral is still the floor), `mass_properties` ≈ 2.3 s, and the 170 s shifted-holder exact intersection (B27's replay) — none has a budget yet; when one is set it becomes P-Class 8.2's first gate. Queued 2026-09-11 after three merged PRs had no owner row. | `math/src/nurbs/{curve,surface}.rs`, `algo/src/pave_filler/phase_ef.rs`, `operations/src/distance.rs`, `check/src/properties/face_integrator.rs`, bench `crates/io/benches/nurbs_properties.rs`, `perf-counters` scaling guards | M (S per stage) | OpenZCAD's rebuild is interactive or it is not used; before this train the union gate took 14 s per strict validation in WASM. Exit: a per-stage budget on the holder operands enforced by a scaling guard or Criterion floor in CI, and the OpenZCAD rebuild timing recorded against the pinned package. | **Partial (2026-09-11, #389, #394, #395):** weight-scale cache + shared partials (validate 3.16 s → 0.97 s, mass 7.88 s → 2.25 s native; WASM strict validate 14.2 s → 1.89 s), conservative edge/face box gating (3-way fuse at 16.1 mm 3.42 s → 0.17 s native, `fuseAll` 9.61 s → 0.87 s WASM), NURBS-face pruning in the solid distance query (arm×arm 757 ms → 54 ms). Bench and two `scaling_` guards are permanent; no budget is enforced yet. |

**Explicitly not queued** (decided or terminal — do not re-open without
the named primitive): IGES growth (C3, decided), box∪sphere and torus∩box
census rows (TERMINAL → O2.3 re-opens them properly), universal
duplicate-edge merge key (proven unbuildable), mesh co-refinement
watertightness (below the chase filter until a live case routes there),
kumiko lattice family (harness retired 2026-08-20; re-open only from an
in-repo fixture), v1-fillet API migration (product decision, owner's),
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
- **Bounded/short session:** one B-row (B11 remainder, B26, one B6
  family, or one B28 stage with its number), or an inherited-queue item
  from P-Class §6.
- **Infrastructure session:** next unclaimed Wave A row in
  [open-kernel-status.md](open-kernel-status.md).
- **Evidence session** (test-writing capacity): B26, B6 families, B10,
  B24 (one file per PR), O1.3b.
- **Docs/ecosystem session:** O6 rows.
- **Consumer-impact session:** B16 (one query binding; link the adapter
  deletion for `unifyFacesChecked` first), O4.7, B20, or B18's unify item —
  each retires a named OpenZCAD adapter heuristic or a silent-degradation
  path. The contract lane (B21, B22, B23) is closed.
- **Parity-evidence session:** O1.2d–e (Remus-only rows first), O1.5,
  B19; see the overlay's §8 ranking.
- **Owner-only:** O4.2c/O4.3c publishes, O6.2 hosting, O6.3 outreach,
  v1-fillet migration decision.

Maintenance rule: any PR that changes an item's state updates its row
here (or its program ledger) in the same PR — same discipline as the
skill's living-document mandate.
