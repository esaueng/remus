# STEP MAMBO B1 carrier miss: measurement and recommendation

Date: 2026-09-19. Question: the import of
`crates/io/tests/data/mambo_b1_untrimmed_nurbs.step` refuses because an
`EDGE_CURVE` carrier misses its surface by 6.16e-5 mm against the reader's
1e-6 cap. Is the cap wrong, is the file sloppy, or should the documented
heal-after-import path handle it?

Short answer: **the file is sloppy and the refusal is correct — keep
refusing (option (c))**. The miss is a true 3D gap 61.6x beyond the file's
own declared uncertainty, the corpus distribution supports the current caps,
and the healer provably cannot help (it is a no-op on the admitted solid).
No reader or writer changes are proposed or made in this task; this document
is measurement plus recommendation only.

Source note: the task brief named three inputs that do not exist in this
tree — `docs/kernel-maturity/step-roundtrip-audit-2026-09-18.md`,
`crates/io/tests/step_roundtrip_mambo_b1_repro.rs`, and
`.claude/skills/heal/SKILL.md` (there is no heal skill; the skills present
are listed under `.claude/skills/`). The real equivalents used here are:
the pinned mambo behaviour in
`crates/io/tests/step_untrimmed_nurbs_domain.rs:1-9,187-227`, the heal route
documented in `.claude/skills/io-formats/SKILL.md:105-115` (`heal_solid` /
`repair_solid`, `convert_to_elementary`, `unify_same_domain`), and the heal
matrix note `docs/kernel-maturity/b17-heal-matrix-note.md`. All `file:line`
citations below are against the branch base (`main` @ `fac12dd1`).

The three evidence sections were produced by parallel throwaway probes (no
committed code; every temporary test and every local-only source tweak was
reverted — see Method, §5).

## 1. Corpus distribution: the 6.16e-5 miss is an outlier, the 1e-6 cap has 20x headroom

A throwaway test imported all 31 `*.step` fixtures under
`crates/io/tests/data/` and recomputed each edge's endpoint-to-carrier
residual exactly mirroring the reader's final gate
(`crates/io/src/step/reader.rs:4863`). Refusals were captured from the
`IoError` text. Caps were cross-checked against each fixture's declared
`UNCERTAINTY_MEASURE_WITH_UNIT`.

Corpus totals: **1637 edges** — 450 Circle, 1067 Line, 120 NurbsCurve.
**Zero Ellipse, Hyperbola, or Parabola edges** occur in the corpus, so the
per-type histogram covers only Line / Circle / NurbsCurve.

| Miss band (mm) | All edges (1637) | Circle (450) | Line (1067) | NurbsCurve (120) |
|---|---|---|---|---|
| <1e-9 | 1578 (96.4%) | 427 | 1067 (all) | 84 |
| 1e-9..1e-7 | 25 (1.5%) | 21 | 0 | 4 |
| 1e-7..1e-6 | 8 (0.5%) | 0 | 0 | 8 |
| 1e-6..1e-5 | 9 (0.5%) | 0 | 0 | 9 |
| 1e-5..1e-4 | 17 (1.0%) | 2 | 0 | 15 |
| >1e-4 | 0 | 0 | 0 | 0 |

Per-file extremes (residuals in mm; full table in the probe record, §5):

- 29 of 30 imported files peak at or below 4.2e-8
  (`lipfuse_3x3_body.step`, Circle) — **more than 20x below the 1e-6
  projected-recovery cap** (`crates/io/src/step/reader.rs:63`).
- The 1e-7..1e-5 bands are tails of `shapr_untrimmed_nurbs_domain.step`
  (max 7.06e-7, imported with two `UntrimmedNurbsDomainRecovered`
  diagnostics) and `shapr3d_hammer_holder.step` (max 4.46e-5 on 15 Nurbs +
  2 Circle edges, imported **under its file-declared 1e-3 model
  uncertainty**, which flows through the vertex/model tolerance cap at
  `crates/io/src/step/reader.rs:4632-4635`).
- `mambo_b1_untrimmed_nurbs.step` is the **only refusal** in the corpus:
  `EDGE_CURVE #253`, start endpoint, 6.164947e-5 mm vs the 1e-6 local
  recovery cap — matching the pinned test at
  `crates/io/tests/step_untrimmed_nurbs_domain.rs:204-209`.

Placement of the two mambo misses:

- **#253 (6.16e-5)** lands in the 1e-5..1e-4 band shared with only 17
  accepted hammer-holder edges (~1% of the corpus), and those are accepted
  solely via a 1e-3 declared model cap. It is ~8.7x above the largest miss
  accepted under a 1e-7-scale cap (4.16e-8) and ~87x above the largest
  accepted projected-recovery miss (7.06e-7). Outlier.
- **#258 (~1.106e-4)** exceeds even the absolute 1e-4 second-stage ceiling
  (`MAX_UNTRIMMED_NURBS_RECOVERY_TOLERANCE_MM`,
  `crates/io/src/step/reader.rs:57`). The corpus contains zero accepted
  edges above 1e-4. It is the corpus maximum by a wide margin.

Cap justification from the distribution: edges exceeding 1e-6 are 26/1637
(1.6%, all hammer-holder or the mambo refusal); exceeding 1e-5, ~1.0%;
exceeding 1e-4, zero accepted. **0% of importable-corpus files require more
than the current 1e-6 (projected) / 1e-4 (absolute) caps.** The data do not
demand a higher cap — the single file needing >1e-6 that still imports
(hammer-holder) already carries a 1e-3 declared model cap covering it
through the vertex/model path.

## 2. Heal path: the healer never gets a chance, and is a no-op when it does

Probe method: with the projected-recovery cap temporarily raised in a
local-only build, import the mambo file, then run the documented route —
`repair_solid` (`crates/operations/src/heal.rs:99`), `heal_solid`
(`crates/operations/src/heal.rs:360`), `convert_to_elementary`
(`crates/operations/src/heal.rs:2636`) — plus the L3 validator
(`remus_operations::validate::validate_solid`), the check-crate validator,
the closed-shell check, and the tessellation watertightness oracle
(`tessellate_solid` + `is_watertight` / `boundary_edge_count`). All
temporary changes reverted; pinned tests re-pass.

Critical gate detail first: **raising the 1e-6 constant alone does not admit
the file.** The call site passes
`tolerance_cap.min(MAX_PROJECTED_NURBS_RECOVERY_TOLERANCE_MM)`
(`crates/io/src/step/reader.rs:4820`), and `tolerance_cap`
(`crates/io/src/step/reader.rs:4632-4635`) is ~1e-6 from the file's declared
uncertainty — so `min(1e-6, raised)` stays 1e-6. The probe had to widen the
effective cap past the declared tolerance as well (all reverted).

Minimum effective cap admitting the whole file: **in (1.105e-4, 1.106e-4]
mm — exactly the #258 residual 1.105842e-4 mm.** At cap 1e-4, #253 is
admitted but #258 still refuses
(`end endpoint misses its carrier by 1.105842e-4 mm (local recovery cap
1.000000e-4 mm)`); at 1.106e-4 the file imports (1 solid, 8
`UntrimmedNurbsDomainRecovered` diagnostics on #237/#239/#241/#243/#244/#245
at ~1e-15–1e-14 plus #253 at 6.164957e-5 and #258 at 1.105842e-4). **The
binding constraint is #258 (~110x declared uncertainty), not the #253 named
in the question.**

Heal results at raised cap (verbatim):

- `repair_solid(topo, solid, 1e-6)` → `Ok`, `valid_after=true`,
  `before_err=0 after_err=0 check_err=0`,
  `healing=HealingReport { vertices_merged: 0, degenerate_edges_removed: 0,
  orientations_fixed: 0, wire_gaps_closed: 0, small_faces_removed: 0,
  duplicate_faces_removed: 0 }`. Identical at tolerance 1e-4.
- `heal_solid(topo, solid, 1e-4)` → `Ok(HealingReport { all fields 0 })`;
  post-heal L3 `valid=true errors=0 issues=[]`.
- `convert_to_elementary(topo, solid, 1e-4)` → `Ok(0)` — nothing to convert;
  the census is already analytic (12 faces: 8 cylinder, 4 plane).
- Check-crate validator after healing: 0 errors (one `SolidEulerCharacteristic`
  warning plus `VertexOnCurve` warnings present identically before and after).

Without heal (raw import at raised cap): closed shell OK, L3 valid with zero
warnings, check-crate 0 errors, tessellation at deflection 0.01 gives
**2770 triangles, watertight, 0 boundary edges, 0 non-manifold** — and the
post-heal tessellation is identical. Volume 468.720464129761, bbox
min=(-10.0,-5.000035,-7.0) max=(13.0,5.0,7.5).

Judgment: **the documented heal-after-import path does not handle the miss —
it never gets the chance.** The refusal fires inside the reader's
fail-closed domain-recovery gate (`crates/io/src/step/reader.rs:4902-4951`)
before any solid exists to heal. Once the gate is widened, the solid is
already closed, valid, and watertight with **zero healing actions**, so
option (b) ("route sub-tolerance misses to the healer") buys nothing here:
there is nothing for the healer to do, and the refusal happens upstream of
it by design.

## 3. Provenance: what the file actually encodes

File identity: `crates/io/tests/data/mambo_b1_untrimmed_nurbs.step`, 651
lines / 635 `#`-entities; per `crates/io/tests/step_untrimmed_nurbs_domain.rs:27-31`
the untrimmed cylinder-intersection seam carriers keep their file bytes
verbatim (only administrative header entities were dropped).

Header (`.step` file line numbers via `grep -n`):

- File line 17: `#43=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-06),#45,'','');`
  — declared global uncertainty **1.0e-6 mm**.
- File lines 18/31: millimetre units
  (`CONVERSION_BASED_UNIT('MILLIMETRE',...)`, `SI_UNIT(.MILLI.,.METRE.)`).

Offending entities (file lines 209/#253, 214/#258; contrast 197/#241):

- `#253=EDGE_CURVE('',#297,#269,#299,.T.);` — bare carrier, no
  `TRIMMED_CURVE` anywhere in the file (`grep -c TRIMMED_CURVE` = 0); the
  trailing `.T.` is the same-sense flag. Start vertex
  `#560=CARTESIAN_POINT('',(-0.0250628144668997,-2.99989530739508,-2.5))`;
  end vertex `#321=(~-0.0,-3.0,-2.0)` exact to roundoff. Carrier #299 is a
  **polynomial (non-rational) clamped cubic**: degree 3, 26 control points,
  13 distinct knots over [7.30875582806202, 15.6470023670403]
  (`grep -c RATIONAL` = 0 for the whole file).
- `#258=EDGE_CURVE('',#268,#305,#307,.T.);` — same bare-carrier form. Start
  vertex `#320=(0.0,-5.0,-2.0)` exact; end vertex
  `#595=(-0.0250628144669001,-4.99993718513853,-2.5)`. Carrier #307:
  polynomial clamped cubic, degree 3, 30 control points, 15 distinct knots
  over [0.47263283695117, 12.1796556804249].
- Contrast #241 (exact-foot seam): both endpoints exact to roundoff
  (6.5e-17 / 0.0); carrier #280, polynomial clamped cubic, 39 control points.

Independent recomputation (throwaway script over the file's own numbers only
— regex-parse points/curves, de Boor evaluation, dense scan plus Newton
refinement, independent of the reader's Rust projector) reproduces the
reader exactly: **#253-start 6.164947e-05** (all six significant figures),
**#258-end 1.105842e-04**.

Decisive findings for #253 (same pattern at #258):

- **True 3D gap, not a parameterization mismatch.** The failing foot sits at
  u=7.5747, fraction 0.032 inside the domain — unambiguously interior; the
  distance to the nearer domain endpoint's curve point is 0.279 mm (4500x
  larger). No re-parameterization resolves this; accepting the edge would be
  *healing*, not re-naming a parameter.
- **No bitangent/loop ambiguity.** The only other stationary points are the
  domain endpoints at 0.279 mm and 6.82 mm (~4500x the near foot). The
  reader's second-foot uniqueness sweep (separation fraction 1e-9,
  `crates/io/src/step/reader.rs:64-67`) would pass trivially; the refusal
  comes purely from the 1e-6 distance cap
  (`crates/io/src/step/reader.rs:4943-4951`).
- **Systematic exporter signature, not random corruption.** Both failing
  vertices share x≈-0.0250628144669 (identical to 13 significant figures)
  and z=-2.5 exactly — a snap onto the z=-2.5 construction plane — while the
  sibling endpoint on each edge is exact to roundoff and every exact foot
  lands bit-for-bit on an interior knot value. The carriers are intact; one
  vertex per edge drifted at lower precision than the spline.

Uncertainty arithmetic: 6.164947e-5 / 1.0e-6 = **61.6x** the file's declared
uncertainty (#253); 1.105842e-4 / 1.0e-6 = **110.6x** (#258). A ~1e-4-scale
vertex drift against a 1e-6 declaration is the textbook signature of a
loose-tolerance export. Verdict: **(i) sloppy-but-common exporter tolerance
artifact** — and the fail-closed refusal is the correct behavior, since
healing onto the foot would silently move topology 61–110x beyond what the
file's own uncertainty certifies.

## 4. Recommendation: (c) keep refusing and mark the fixture unsupported

| Option | Verdict | Reason |
|---|---|---|
| (a) Raise the cap to X | **Reject** | The corpus (§1) gives no justification: 0% of importable files need more than 1e-6/1e-4, and the current caps hold 20x headroom. Admitting mambo would require an effective cap ≥1.106e-4 — fitted to a single sloppy file, ~110x its declared uncertainty, with zero headroom to the next failure. A global raise trades a principled fail-closed gate for one file. |
| (b) Route sub-tolerance misses to the healer | **Reject** | Architecturally unavailable and empirically vacuous: the refusal fires before any solid exists (§2), and once admitted the solid needs zero healing actions (all-zero `HealingReport`, identical watertight tessellation). There is nothing to route and nothing to fix. |
| (c) Keep refusing, mark fixture unsupported | **Recommend** | The file exceeds its own declared uncertainty by 61–110x (§3); refusal is the designed fail-closed behavior, pinned by `mambo_b1_exact_foot_seams_reach_projected_recovery` (`crates/io/tests/step_untrimmed_nurbs_domain.rs:188-210`). The #241/#243/#244/#245 exact-foot seams already import through projected recovery — only the genuinely off-carrier #253/#258 refuse. |

What would change this answer: a product decision that MAMBO B1-class
loose-tolerance files must import. The honest mechanism would be an
**explicit caller-supplied tolerance override at the import call site**
(opt-in loosening with provenance recorded, e.g. alongside the existing
`UntrimmedNurbsDomainRecovered` diagnostic at
`crates/io/src/step/reader.rs:74-91`), never a global cap raise — the
current `min(declared, absolute)` structure
(`crates/io/src/step/reader.rs:4820-4823`) exists precisely so a coarse
declaration cannot silently heal visibly off-carrier endpoints
(`crates/io/src/step/reader.rs:58-63`). That is new API surface and out of
scope for this measurement task; it is noted here only so a future
proposal starts from the measured numbers (effective cap ≥1.106e-4 for this
file, §2).

## 5. Method and reproducibility notes

- Three parallel probes, all reverted: (1) a throwaway histogram test
  (test-only residuals mirroring `crates/io/src/step/reader.rs:4863`, no
  gate-semantics change) plus refusal capture — deleted afterwards; (2) a
  temporary local-only cap widening plus a throwaway heal/watertightness
  test — both reverted, pinned tests re-passed; (3) a strictly read-only
  file parse with scratch work in `/tmp` only.
- The shared checkout carried concurrent in-flight modifications during the
  probes (algo builder/pave-filler files and `reader.rs` `_ =>`-arm edits
  from another worker; both heal and corpus probes observed the tree
  shifting mid-run). Both measurement probes re-ran to settlement and
  reproduced the pinned #253/6.164947e-5/1e-6 refusal exactly; histogram
  bands were identical across runs. This document cites `main`-tree line
  numbers, verified in a clean worktree of the branch base.
- This branch contains only this document; no reader, writer, or test
  sources are touched.
