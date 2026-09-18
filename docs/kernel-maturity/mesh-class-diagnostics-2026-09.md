# B26 mesh-class diagnostics — per-face boundary tables (2026-09)

Date: 2026-09-18. Branch: `docs/mesh-class-diagnostics`. Base: `origin/main`
at `3785eebc`.
Task: READ-ONLY diagnostic. No kernel code was modified; no fix was attempted.
Hypotheses below are hypotheses, not fixes.

## Method (as requested, with one deviation disclosed)

- Wrote ONE throwaway integration test
  `crates/operations/tests/zz_mesh_probe.rs` (never committed, deleted before
  the PR). For each shape builder it runs the boolean with
  `FallbackPolicy::ExactOnly` via `boolean_with_context`, then calls
  `remus_operations::tessellate::tessellate_solid_grouped_with_tolerance` at
  deflections `0.1`, `1e-2`, `1e-3`, and `bbox-diagonal*1e-5` (floor `1e-7`,
  same formula as `prop_boolean_invariants.rs::mesh_deflection`), with
  `DEFAULT_ANGULAR_TOL`. For each deflection it reports total boundary edges,
  non-manifold edges (via `boundary_edge_count` /
  `non_manifold_edge_count`), and per face: face index (in
  `explorer::solid_faces` order), `surface().type_tag()`, inner-wire count,
  outer-wire edge `type_tag()` list, boundary-edge count owned by that face
  (global boundary directed-edges attributed through the grouped
  `face_offsets` to the owning triangle's face), and the first six
  boundary-edge endpoints (sorted, `%.6f`).
- Spawned one subagent per row (B32/B37/B38/B39/B40). Each copied the shape
  construction verbatim from the ignored repro, ran or read the probe, and
  returned the table plus a two-sentence reading using the tessellation
  skill's "Symptom to cause" table.
- Deviation: mesher branches were inferred from
  `crates/operations/src/tessellate/solid.rs::tessellate_face_with_shared_edges`
  dispatch logic, NOT from a temporary `eprintln`. No `eprintln` was added;
  `git diff` for `crates/` is empty. Rationale: preserve the READ-ONLY
  constraint and avoid five parallel edits to the same dispatch file. Branch
  statements below are therefore marked "inferred".
- Logs: fast rows (B32/B37/B40) were probed on `docs/mesh-class-diagnostics`
  at `3785eebc` (`/tmp/probe_b32_new.log`, `/tmp/probe_b37_new.log`,
  `/tmp/probe_b40.log`). Slow rows (B38/B39) were probed by subagents while
  the checkout had been externally switched to `test/step-roundtrip-audit`
  (same `3785eebc` base for kernel paths; constructions verified verbatim
  identical). Their tables are retained with that caveat
  (`/tmp/probe_b38_new.log` 403 s, `/tmp/probe_b39_new.log` 73 s).
- What the probe does NOT do: no validators (`validate_solid`,
  check-crate supplement, position-quantized recount), no
  `classify_point` probes, no volume/translation readings. Where the row's
  failing oracle is validity or drift rather than open mesh, the report says
  "not diagnosed" explicitly.

Scope: exactly the rows/repros listed in the task (B32, B37, B38, B39
subset, B40). Other mesh-class rows (B34, B41, B43, B44, B45/B46) are out of
scope and are not covered here.

Skills read: `CLAUDE.md` (== `AGENTS.md`), `.claude/skills/tessellation/SKILL.md`,
`.claude/skills/solid-verification/SKILL.md`, `.claude/skills/testing/SKILL.md`.

---

## B32 — Translation-variant exact cut volumes (findings 1/17)

Repros: `b26_finding_cylinder_cut_translation_variant`,
`b26_finding17_boxcone_cut_drift`, `b26_finding17_boxcone_sibling`
(`prop_boolean_invariants.rs` ~2835, ~3059, ~3097). Constructions verified
verbatim vs `test_b32_probe` — no mismatch.

| probe leg (op) | RESULT | faces | diag / scaled | boundary @ [0.1, 0.01, 0.001, scaled] / nonmanifold |
|---|---|---|---|---|
| `b32_cylcut_cut` (cyl 1.5×1 cut by cyl 3×3, `T(4,3.5,-2)*Rx(π/2)`) | ok Exact | 4 | 4.358898944 / 0.000043589 | 0, 0, 0, 0 / 0 throughout (tris 244/440/1354/6438) |
| `b32_boxcone_cut` (box 2.5×1×1 cut by cone 1.0/2.5/2.5, `T(0.5,-1.5,-0.5)*Rz(3π/2)`) | ok Exact | 7 | 5.964059020 / 0.000059641 | 0, 0, 0, 0 / 0 (tris 340/352/410/656) |
| `b32_boxcone_fuse` (same input, fuse) | ok Exact | 9 | 7.500000000 / 0.000075000 | 0, 0, 0, 0 / 0 (tris 510/930/5260/60748) |
| `b32_sibling_fuse` (box 2.5×1×1 + cone 1.5/2.5/1.0, same placement) | REFUSED `ExactOnlyUnattainable` | — | — | no mesh to probe |
| `b32_sibling_cut` (same sibling, cut) | ok Exact | 9 | 7.141428429 / 0.000071414 | 0, 0, 0, 0 / 0 (tris 250/284/416/946) |

Per-face (all legs, all deflections): every face `owned_boundary=0`,
`first6=[]`. Representative face lists: `b32_cylcut_cut` faces are
plane/cylinder/cylinder/plane with multi-circle+line outers;
`b32_boxcone_cut` faces are 6 planes + 1 cone (`[circle,nurbs,nurbs]`);
`b32_boxcone_fuse` faces are 6 planes + cone inner=1
(`[circle,line,circle,line]`) + 2 circle planes; `b32_sibling_cut` faces are
8 planes + 1 cone (`[circle,circle,nurbs,circle,nurbs]`). No offending face
exists, so no endpoint table is reproduced here (full `FACE` lines in
`/tmp/probe_b32_new.log`, all `owned_boundary=0`).

Mesher branch (inferred, no instrumentation): planes with inner=0 take the
planar shared path (`cdt_triangulate_simple`); cone/cylinder faces attempt
`revolution_band_shared` (plus apex-fan for cones) then CDT-then-snap, the
inner=1 fuse cone taking the hole-aware snap path. All succeed watertight.

Reading: per the tessellation skill every entry keys off `boundary>0`, and
B32 has `boundary=0` at all four deflections, so no mesher-crack symptom
fires; per the solid-verification ladder, mesh-watertight (rung 4) passing
does not prove classification/volume (rungs 5–6). B32's failing oracle is
translation-variant volume (the finding-1 doubled-boundary class), not an
open mesh. Hypothesis (not a fix): the defect lies upstream in boolean
face selection / volume measurement, not in the mesher; note the sibling
fuse now refuses under `ExactOnly` where the repro comment recorded a
322-boundary fuse mesh, so the refusal itself deserves a validity check the
probe did not run. **Not diagnosed**: the volume drift — a mesh probe
cannot explain translation-variant volume on watertight meshes.

---

## B37 — Pointed-cone–cylinder pair defects (finding 12)

Repros: `b26_finding12_cone_fuse_mesh_open`, `b26_finding12_cone_cut_broken`
(~3353, ~3383). Verbatim: cone `r0=2.0` apex `h=1.0` + cylinder `r=1.0 h=1.0`
at `(0,1.5,0.5)`; fuse + cut. Both `quality=Exact` on this head.

Total mesh (`/tmp/probe_b37_new.log`, diag fuse 6.204836823/scaled
6.2048e-05; cut diag 6.067330550/scaled 6.0673e-05):

| leg | faces | 0.1 | 0.01 | 0.001 | scaled | nonmanifold |
|---|---|---|---|---|---|---|
| `b37_conefuse_fuse` | 5 | 18 | 40 | 372 | 719 | 0 throughout (tris 298/572/2040/7401) |
| `b37_conecut_cut` | 4 | 18 | 40 | 372 | 724 | 0 throughout (tris 22/58/458/1088) |

Matches the roadmap (18/40/372 at 0.1/0.01/0.001 — coarser reads cleaner,
not a resolution artifact).

Per-face owned boundary (fuse; cut is structurally identical with one fewer
plane face):

| defl | f0 plane inner0 `[circle]` | f1 cone inner1 `[circle]` | f2 cylinder inner0 13-edge `[line,circle×10,nurbs×2]` | f3 plane inner0 7×`[circle]` | f4 plane inner0 4×`[circle]` |
|---|---|---|---|---|---|
| 0.1 | 18 | 0 | 0 | 0 | 0 |
| 0.01 | 32 | 0 | 0 | 8 | 0 |
| 0.001 | 100 | 0 | 254 | 18 | 0 |
| scaled | 399 | 0 | 254 | 66 | 0 |

Cut leg (`b37_conecut_cut`, faces plane/cone/cylinder(`[circle,nurbs,nurbs]`)/plane-3-circles):
f0 owns 18/32/100/404; f1 owns 0; f2 owns 0/0/254/254; f3 owns 0/8/18/66.
The f2 owned count freezes at 254 at both 0.001 and scaled (independent
resample signature); f0 grows with refinement (32→100→404).

First endpoints (worst deflection, fuse):
- f0 (cone-base rim, z=0): `(-2.000000,0.000000,0.000000)->(-1.879385,0.684040,0.000000)`,
  `(-1.879385,-0.684040,0.000000)->(-2.000000,0.000000,0.000000)`,
  `(-1.879385,0.684040,0.000000)->(-1.532089,1.285575,0.000000)`, … (full six
  in log; at scaled the fan densifies to 399 edges).
- f2/f3 shared corner (z=0.5 cap): f2
  `(-0.661438,0.750000,0.500000)->(-0.657348,0.746413,0.502698)->(-0.653237,0.742847,0.505394)->…`
  (identical at 0.001 and scaled — frozen); f3
  `(-0.599143,0.800642,0.500000)->(-0.661438,0.750000,0.500000)`,
  `(-0.532987,0.846124,0.500000)->(-0.599143,0.800642,0.500000)`, … sharing
  f2's start vertex `(-0.661438,0.750000,0.500000)`.

Mesher branch (inferred): f0 (plane, no inner) takes the plane arm
(`cdt_triangulate_simple`); f1 (cone with inner) tries
`revolution_band_shared` (declines, one rim) then `cone_apex_fan_shared` —
clean (owns 0); f2 is `band_eligible` (Line/Circle/Nurbs mix allowed) but
NOT `is_standard_rect` (>4 edges, not all Line/Circle), so it falls to the
generic CDT-then-snap path, never `revolution_band_shared`; f3/f4 (plane,
multi-circle outer but inner=0) take the same `cdt_triangulate_simple` path
as f0, not the holed-plane parallel-CDT job (which requires inner wires).

Reading: the f2/f3 crack sits on the shared arc at the cone–cylinder
intersection corner `(-0.661438,0.75,0.5)` on the z=0.5 cap plane — a
"CDT wall vs separately meshed cap / snap-path proximity miss (#696 class)"
failure of the generic CDT-then-snap cylinder wall against the planar cap's
independent triangulation; the f0 crack on the cone-base rim is the same
class, where the inner-wire skip leaves the two sides on different rim
samplings. Hypothesis (not a fix): route the >4-edge mixed-rim cylinder
wall (f2) to a structured shared-vertex band instead of CDT-then-snap, and
re-examine the circle-densify sync skip for inner-wire cone neighbors (f1);
do not tune CDT density or snap proximity.

---

## B38 — Sphere–torus boolean misbuilds (finding 13)

Repros: `b26_finding13_spheretorus_smallscale_invalid`,
`b26_finding13_spheretorus_sibling_fuse`,
`b26_finding13_spheretorus_unit_invalid` (~3260, ~3289, ~3319). Constructions
verified verbatim identical to `test_b38_probe`. Probed on the
`test/step-roundtrip-audit` checkout (same `3785eebc` kernel base); all five
legs returned `ok Exact` (no refusal on this head).

| leg | faces | diag / scaled | 0.1 | 0.01 | 0.001 | scaled | nonmanifold |
|---|---|---|---|---|---|---|---|
| `b38_small_inter` (sph 0.002 + tor 0.002/0.0005, `T(0,0.001,0.001)*Rx(3π/2)`, inter) | 3 | 0.003279098 / 1e-07 | 0 | 0 | 0 | 472 | 0 (tris 5244×3 then 162122) |
| `b38_small_cut` (same, cut) | 3 | 0.006449588 / 1e-07 | 0 | 0 | 0 | 1214 | 6 @scaled only (2.4M tris) |
| `b38_sibling_fuse` (same shapes, `T(0,0.002,0.001)*Rx(3π/2)`, fuse) | 3 | 0.008689074 / 1e-07 | 0 | 0 | 0 | 453 | 0 (1.3M tris @scaled) |
| `b38_unit_fuse` (sph 3 + tor 4/0.5, `T(1.5,2,1)*Ry(3π/2)`, fuse) | 3 | 14.396180049 / 0.000143962 | 0 | 427 | 427 | 848 | 0 |
| `b38_unit_cut` (same unit, cut) | 3 | 14.396180049 / 0.000143962 | 0 | 427 | 427 | 848 | 0 |

Worst-deflection owners:
- `small_inter`@1e-07: f0 sphere/0/`[nurbs×2,circle]`/0; f1
  torus/0/`[nurbs×6]`/470; f2 sphere/0/`[nurbs×4,circle]`/468. f1+f2 mirror
  each other, e.g.
  `(0.000872,0.001367,-0.001171)->(0.000872,0.001372,-0.001165)`.
- `small_cut`@1e-07: f0 sphere/0/374 (rim near `(-0.002,±0.0001,0)`); f1
  torus/0/470; f2 sphere/0/836.
- `sibling_fuse`@1e-07: f0 sphere/0/`[line×8]`/8; f1 sphere/1/`[line×8]`/445
  (rim x≈-0.002 slabs); f2 torus/1/`[circle×16]`/0 — hole-rim crack with a
  clean counterpart.
- `unit_fuse`+`unit_cut`@0.01 and @scaled: f0 sphere/1/0; f1 sphere/1/0; f2
  torus/1/`[nurbs]`/all (427 @0.01/0.001, 848 @scaled), e.g.
  `(1.000024,-1.821990,2.163401)->(1.000083,-1.841649,2.146663)`,
  `(1.000027,0.638188,-2.755479)->(1.000380,0.663402,-2.749389)`.

Mesher branch (inferred): none of these faces qualify for
`torus_notch_band`, `torus_two_rim_band`, `latitude_band_shared`, or
`sphere_cap_shared` (multi-NURBS outers, holed spheres, single-NURBS torus
hole — no two-rim latitude/notch/cap shape), so all five legs fall through
to `tessellate_nonplanar_cdt` / `tessellate_nonplanar_snap`.

Reading: the 1e-3 legs are coarse-clean and explode only at the 1e-7 floor
with mirrored torus+sphere rim endpoints — a snap-path rim-sampling
divergence at extreme refinement, while the sibling's 453 edges owned almost
entirely by the holed sphere against a clean torus hole is the textbook
snap-path proximity miss on the sphere CDT face, and the unit legs'
stable 427→848 single-face torus-hole ownership is a structured
shared-vertex violation (CDT wall vs rim pool), not a lucky-deflection
crack. Hypothesis (not a fix): the torus-hole faces need a structured
shared-vertex path (or a hole-aware band acceptance that actually fires on
single-NURBS holes); do not densify CDT or widen snap. **Not diagnosed**:
validity (the roadmap's inconsistent-orientation edges — the probe never
runs validators), translation/volume drift (no volume oracle), the
upstream figure-eight contribution to `small_cut`'s 6 non-manifold edges,
and any refusal dimension (none reproduced — all `ok Exact`).

---

## B39 — Torus–cone pair-cell misbuilds (finding 14)

Requested: `b26_finding14_toruscone_unit`,
`b26_finding14_toruscone_oblique_drift` (~3463, ~3500); also probed
`b26_finding14_toruscone_cut_invalid`, `b26_finding14_toruscone_sibling`
(~3409, ~3432) as same-cell context. All constructions verified verbatim vs
`test_b39_probe`. Probed on `test/step-roundtrip-audit` (same kernel base);
`EXIT=0`, 73 s.

| leg | RESULT | faces | 0.1 | 0.01 | 0.001 | scaled | nonmanifold |
|---|---|---|---|---|---|---|---|
| `b39_cut_invalid` (tor 3e-3/5e-4 + cone 1.5e-3/1e-3/1e-3, `Tx(2.5e-3)*Rx(3π/2)`, cut) | ok Exact | 3 | 0 | 0 | 0 | 0 @1e-07 (906k tris) | 0 |
| `b39_sibling_fuse` (same 1e-3 shapes, `Tx(2.5e-3,-0.5e-3,0)*Rx(3π/2)`, fuse) | ok Exact | 4 | 0 | 0 | 0 | 0 @1.07e-07 (850k tris) | 0 |
| `b39_sibling_cut` (same sibling, cut) | ok Exact | 3 | 0 | 0 | 0 | 0 @1e-07 (907k tris) | 0 |
| `b39_unit_fuse` (tor 3/0.5 + cone 1.5/1/1, `T(1.5,2,1)*Ry(3π/2)`, fuse) | REFUSED `ExactOnlyUnattainable` | — | — | — | — | — |
| `b39_unit_cut` (same unit, cut) | REFUSED `ExactOnlyUnattainable` | — | — | — | — | — |
| `b39_oblique_fuse` (same unit shapes, `T(2.5,0,0)*Rx(π/4)`, fuse) | ok Exact | 4 | 0 | 0 | 16 | 2276 @1.055e-04 (921k tris) | 1 @0.001, 0 @scaled |

Worst-deflection faces:
- `cut_invalid`/`sibling_cut`@scaled: all faces `owned=0` (plane/NURBS,
  torus inner1/NURBS, plane/NURBS) — no endpoints.
- `sibling_fuse`@scaled: f0 plane/1/`[circle]`/0; f1 cone/0/`[circle,line,circle,line]`/0;
  f2 torus/1/`[nurbs]`/0; f3 plane/1/`[circle]`/0.
- `oblique_fuse`@0.001: all 16 owned by f0 plane/1/`[circle]`, e.g.
  `(3.484847,0.114830,0.114830)->(3.491546,0.085714,0.085714)`,
  `(3.485225,0.113310,0.113310)->(3.484847,0.114830,0.114830)`, …; f1 cone 0,
  f2 torus/16×circle 0, f3 plane 0.
- `oblique_fuse`@scaled: same owner f0=2276, e.g.
  `(2.500000,0.000247,0.000247)->(2.500001,-0.000939,-0.000939)`; others 0.

Mesher branch (inferred): cone/cylinder faces take
`revolution_band_shared` when Line/Circle/NURBS rims qualify, else CDT/snap;
torus faces try `notch_band`/`two_rim_band`/`latitude_band_shared`, else
CDT/snap. The 1e-3 NURBS-rim torus faces decline the structured bands yet
mesh clean. The oblique torus (16×circle) + cone (4-edge circle/line) are
band-eligible, yet the crack sits on holed plane f0 — a planar hole-snap
density crack, not a torus/cone band failure.

Reading: the requested unit pair no longer yields a mesh to diagnose — both
legs refuse under `ExactOnly`, so the roadmap's fine-only fuse crack (254
boundary, distinct from finding-8's always-open caps) is unreproduced on
this head; the oblique fuse is conversely mesh-open fine-only (16@1e-3,
2276@scaled, all on the holed plane) against the roadmap's "watertight"
claim, pointing to a planar snap-density crack while its ~16% drift remains
untouched by this probe. Hypothesis (not a fix): check what changed the
unit pair from valid-open to refused (recent torus-band/sphere-frame work?),
and treat the oblique f0 opening as a planar-hole shared-vertex issue, not
a torus-band issue. **Not diagnosed**: `unit` (refused — no face data),
`oblique` drift (no volume/translation oracle), `cut_invalid`/`sibling`
validity (validators not run; mesher clean at all deflections).

---

## B40 — Cone–sphere small-scale wire self-intersections + open mesh (finding 15)

Repro: `b26_finding15_conesphere_wire_mesh` (~3530). Verbatim: frustum cone
`r0=0.001/r1=0.0005/h=0.001`, sphere `r=0.001` seg 8, `T(0,0,0.001)*Rx(3π/2)`;
fuse + cut legs probed (`/tmp/probe_b40.log`, head `3785eebc`).

| leg | faces | diag / scaled | 0.1 | 0.01 | 0.001 | scaled (1e-07) | nonmanifold |
|---|---|---|---|---|---|---|---|
| `b40_fuse` Exact | 4 | 0.003464102 / 1e-07 | 0 (46t/27v) | 0 | 0 | 714 (58982t/29649v) | 0 throughout (was 1432+54) |
| `b40_cut` Exact | 4 | 0.002899931 / 1e-07 | 0 (1530t/767v) | 0 | 0 | 0 (59264t/29634v) | 0 |

Per-face at scaled deflection (types identical at all deflections, inner=0
all; owned counts double-count shared edges — reported as observed):
- f0 plane `[circle]` — 0 (both legs).
- f1 cone 9-edge `[nurbs×6,line,circle,line]` — fuse 714, cut 0.
- f2 sphere `[nurbs×4,circle]` — fuse 476, cut 0.
- f3 sphere `[nurbs×2,circle]` — fuse 238, cut 0.
Fuse first endpoints cluster at `(-0.0008,~0,0.0004)` (5–10 µm segments),
e.g. cone
`(-0.000800,-0.000000,0.000400)->(-0.000800,0.000011,0.000400)`,
`(-0.000800,0.000011,0.000400)->(-0.000800,0.000021,0.000400)`, …; sphere
`(-0.000800,-0.000005,0.000400)->(-0.000800,-0.000000,0.000400)`, … (full six
per face in `/tmp/probe_b40.log`).

Mesher branch (inferred): the 9-edge mixed cone face is not
`is_standard_rect`, so `band_eligible` may hold yet
`revolution_band_shared` likely declines (wavy marched-NURBS rim, not a
two-full-rim band) into the generic CDT-then-snap path; the NURBS-section
sphere faces likewise decline `latitude_band`/`sphere_cap_shared` into
CDT/snap.

Reading: the fuse opens only at the 1e-7 floor while the cut is clean at
every deflection including 1e-7, so this is a deflection-dependent
snap-reconciliation failure on the marched-NURBS intersection rim, not a
missing-face structural defect. Hypothesis (not a fix): the 1e-7-only snap
miss on this rim class wants a shared-vertex treatment for NURBS-section
rims rather than proximity reconciliation; verify against the B41
sphere-frame fix which already removed the non-manifold component (54→0).
**Not diagnosed**: the two supplement `WireSelfIntersection` errors — a mesh
probe cannot see wire self-intersection, and no claim is made about
figure-eight wires.

---

## Cross-row notes for the fix session

- Scale pattern: B38-small, B39-1e-3, and B40 open only at the 1e-7 floor
  (or are clean there), while B38-unit opens already at 0.01 (427, stable)
  and B37 opens at every deflection including 0.1 (18). Do not treat a
  1e-7-only opening as the same defect as an every-deflection opening; the
  former smells of snap sampling density, the latter of a structural
  shared-vertex violation (tessellation skill: "stable small nonzero
  boundary count … means a shared-vertex violation, not 'almost done
  tuning'").
- Refusal changes on this head vs the roadmap: `b32_sibling_fuse` and both
  `b39_unit` legs now refuse under `ExactOnly` (previously valid-open).
  Re-run their witnesses with validators + closed-form volumes before
  chasing them as mesh bugs — the defect may have moved from the mesher to
  the boolean's refusal classification (or been partially fixed).
- Non-manifold is gone in B40 (54→0) and absent in B37/B39-oblique-scaled,
  but persists as 6 edges in `b38_small_cut`@scaled alongside 1214 boundary
  edges and 2.4M triangles. That leg alone needs the upstream
  figure-eight-wire check (`split_self_intersecting_wires`, boolean-debugging
  skill) in addition to mesher work.
- The probe never runs `validate_solid`, the check-crate supplement,
  `classify_point`, or volume/translation oracles. Every "not diagnosed"
  above is a pointer to the solid-verification ladder rung the fix session
  must still climb; in particular B32-oblique/B39-oblique drift and all
  B38/B39 validity claims are untouched by this report.
- No fixes were attempted even where one looked obvious. The closest calls
  were B37-f2 (route the mixed-rim wall to a structured band) and B40-fuse
  (shared-vertex treatment for NURBS-section rims) — recorded as hypotheses
  only.

## Reproducibility

- Throwaway probe (deleted before the PR; full `FACE` lines lived in
  `/tmp/probe_*.log` during the session): boolean via
  `boolean_with_context(..., OperationContext::new().with_fallback(ExactOnly))`;
  tessellation via `tessellate_solid_grouped_with_tolerance` at
  0.1/0.01/0.001/`diag*1e-5`; attribution via `face_offsets`; per-face
  `type_tag()`, inner count, outer `type_tag()`s, owned boundary, first six
  endpoints.
- Rerun (after restoring the probe file from this report's description):
  `cargo test -p remus-operations --test zz_mesh_probe test_bXX_probe -- --nocapture`
  with `test_b32_probe`, `test_b37_probe`, `test_b38_probe` (≈400 s),
  `test_b39_probe` (≈73 s), `test_b40_probe`.
- `git diff` for `crates/` is empty; the only committed change is this file.
  The probe file was deleted and no `eprintln` was ever added to `solid.rs`.
