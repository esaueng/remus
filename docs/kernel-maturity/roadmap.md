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

The work-selection *doctrine* (chase filters, TERMINAL list, acceptance bar)
remains `.claude/skills/roadmap/SKILL.md`; this page is the *queue*. Both
are living documents: update the relevant row in the same PR that changes
its state. Before claiming anything: `gh pr list --state open` (R6).

- **Drafted:** 2026-08-29, baseline `main` @ `3c232e8`.
- **External K-S1 disposition — cross-drilled render/measure:** done in PR
  #144. The OpenZCAD operation sequence now has a deterministic replay bundle,
  independent volume oracles, ratio/scale display-mesh qualification, and a
  non-vacuous WASM `meshQuality` contract. Follow-ups remain for the separate
  face-orientation inconsistency and the sub-millimeter fine-mesh boundary
  residue; neither is hidden by this disposition.

## §H Horizons

### H0 — in flight (verify before duplicating)

P-Class 2.0 partially landed (#125 FF section ranges, #130 edge domain
authority), RFC 0004 merged (#126); open: 2.1 honest-failure hygiene
(#129), RFC 0005 draft (#127), the program docs themselves (#133).

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
| B1 | **Healing disclosure typing** — the matrix's only named Unsupported-untyped cell: permissive healing can mask an invalid result as valid. Type every repair (report what changed, refuse to claim validity it didn't verify); both-sides tests. | `heal/src/fix/`, `check/src/validate/` | M | The last untyped silent-failure path in the kernel; highest correctness value per line. Do first in the qualification lane. | Open |
| B2 | **Boolean scale residuals** — 1e-5 fails closed (100·tol weld bands); raw-GFA 1e6 silently 0.9467 vs 0.8400 (ExactOnly refuses; measure + pin). | `algo` bands | M | Feeds P-Class 2.6 directly; the 1e6 cell is a possible silent-wrong class. Geometry lane. | Open |
| B3 | **Closed-rim chamfers** — cone-frustum band mirroring the validated toroidal fillet assembler; closed-form volume oracle. Stabilization C1.2. | `blend`, `operations/src/chamfer.rs` | M | Exact surfaces, cheap, passes chase filter 1; unblocks resize_blend cylinder/cone (C2). | Open |
| B4 | **v2 walking-trimmer completion** — the four named gaps: keep-side hint, shared contact edges, end-cap notch trim, chamfer external-tangent branch. Stabilization C1.3. | `blend/src/trimmer.rs` | M | Critical path for v2 walker parity → legacy engine retirement (M5 precondition). | Open |
| B5 | **Offset face provenance** — offset derives faces 1:1 and discards the mapping; journal real evolution instead of a barrier. | `offset`, `operations/src/offset_v2.rs` | S | The last declared-barrier operation nobody owns; closes the B3-residual from stabilization. | Open |
| B6 | **Evidence matrices, batched** — the "Stable-but-blocked" ledger rows that are pure test work: primitives invalid-input/scale/postconditions; plane-section cavity+degeneracy; measurement curved-cavity+scale; sweeps degenerate/cavity + nonconvergence budgets; convex hull/Minkowski degenerates. One qualify_*.rs per family, stabilization-plan pattern. | `operations/tests/` | M (S per family) | Flips ~8 Blocked ledger rows with zero new geometry; ideal bounded-session work. | Open |
| B7 | **Pave-block attachment for marched FF curves on curved faces** — the named canonical fix for the cross-face boundary-desync family; three cheaper altitudes already failed. | `algo/pave_filler/make_blocks.rs`, `phase_ff.rs` | L | Deepest structural payoff in algo; root-causes a whole non-manifold family. Geometry lane, coordinate with M2; repro `replay_scplate.rs`. | Open |
| B8 | **Reversed NURBS sub-span convention** — forward spans shipped; reversed validated sub-spans blocked on the same arrangement defect as B7. | `topology/src/edge.rs` | M | Completes the endpoint-trimmed contract 2.0 builds on. | Open (after/with B7) |
| B9 | **Torus ∖ coaxial cylinder tangent cut** — the single cell keeping torus booleans Beta; needs a tangent-contact primitive (explicitly NOT the band splitter). | `math/analytic_intersection.rs`, `algo` splitter | M | B1-ledger promotion Beta→Stable; closed-form oracle exists. Rides 2.7 tangency machinery. | Open |
| B10 | **Curve-curve / curve-surface classification qualification** + conic distance/classification cells | `math`, `geometry/extrema`, matrix harness | M | Unqualified since the matrix was written; sits under many families' claims; pure evidence. | Open |
| B11 | **Small hygiene set** — `log::debug!` false-zero in `fill_images_faces.rs` (diagnostic-infra bug); deterministic STEP entity ordering; heal `fix_duplicate_faces` winding-blind comparison; plane×plane sampled in-both exact upgrade; `n_fine` clamp hazard note→guard. | various | S each | Cheap, each has already cost or will cost a debugging session. | Open |
| B12 | **Holes on non-planar section caps** — annular Coons or cap-then-subtract vs extruded-annulus ground truth (stabilization B2.2). | `operations/src/cap.rs`, `fill_face.rs` | M | Largest remaining non-planar-cap value with clean ground truth. H3, with M7 cap work. | Open |
| B13 | **STEP inner-shell (voids) export** — `BREP_WITH_VOIDS` reads; export of cavity solids incomplete. | `io/src/step/writer.rs` | S–M | Round-trip honesty for hollow parts; gauntlet round-trip stage will hit it. | Open |
| B14 | **Render promotion track** — Experimental→Beta after a contract-stable release cycle (stabilization C4 residue); outside both programs. | `render` | S (time-gated) | Cleans the last stabilization row. | Open |

**Explicitly not queued** (decided or terminal — do not re-open without
the named primitive): IGES growth (C3, decided), box∪sphere and torus∩box
census rows (TERMINAL → O2.3 re-opens them properly), universal
duplicate-edge merge key (proven unbuildable), mesh co-refinement
watertightness (below the chase filter until a live case routes there),
kumiko lattice family (probe only per the roadmap skill's engine-side
question), v1-fillet API migration (product decision, owner's).

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
