# Gauntlet approximate-boolean census (2026-10-02)

Date: 2026-10-02. Branch: `muse/m6-gauntlet-census`. Base: `origin/main`
at `b98cf32f`.
Task: READ-ONLY census. No kernel code was modified; no fix was attempted.
The throwaway replay harness
(`crates/operations/examples/m6_probe_replay.rs`, never committed, deleted
before the PR; copy retained at `/tmp/m6-gauntlet/m6_probe_replay.rs.txt`
on the build server only) replayed each probe cut natively. Hypotheses
below are hypotheses, not fixes.

Skills read: `roadmap`, `analytic-preservation`, `io-formats`,
`boolean-debugging`, `testing`.

## Method

1. Fetched the pinned smoke manifest (`tools/gauntlet/manifests/smoke.json`,
   sha256 `779fcc7f…c3186`, 50 MAMBO models) into a content-addressed cache
   and ran the gauntlet locally:
   `cargo run --release -p remus-gauntlet -- run --output <out>
   --timeout-ms 60000 $(cat <list>)`,
   start 2026-10-02T17:52:34Z (≈10 s wall for 50 models).
   Remus SHA `b98cf32f57ddaa28429753ba718c54186c46a362`.
2. Replayed every boolean-stage probe cut natively (import → operand face
   census by surface type, operand free/over-used edge counts, raw
   `remus_algo::gfa::boolean` Cut for the first refusal, then
   `boolean_with_context` Cut for the disclosed quality and result census),
   one model per process, `RUST_LOG=warn` to capture the gate line. The
   probe is the gauntlet's centred box cut: box side = bbox-diagonal/2,
   centred on the solid, `BooleanOp::Cut`.
3. Re-ran the same gauntlet command against an `origin/main` export at
   `c50ec1fa` (the 2026-09-29 run head) in `/tmp/m6-old`, and A/B-tested
   the two moved cells with cold builds at candidate commits in fresh
   `/tmp/m6-t0`, `/tmp/m6-t1` trees.
   Process trap found during (3): re-extracting an older commit over a
   tree whose `target/` holds newer artifacts makes cargo skip rebuilding
   dependencies (commit-time mtimes read older than the artifacts), so the
   "bisection" runs silently re-ran the newer code. All overwrite runs
   were discarded; only cold builds in fresh dirs were trusted. One-line
   lesson recorded in the `roadmap` skill.

## Scoreboard (local run at `b98cf32f`)

Models 40/50. Stages: read 44, validate 42, boolean 41, tessellate 40,
round-trip 40. Boolean quality: exact 15, disclosed approximate 26
(deflection 0.1, the context default budget, in every approximate cell).
Failure taxonomy: `invalid_input` 6, `invalid_topology` 3,
`tolerance_violation` 1.

Baseline delta vs 2026-09-29 (run 36551362279, head `c50ec1fa`:
40/50, read 44, validate 42, boolean 41, tessellate 40, round-trip 40,
13 exact / 28 approximate, same 6/3/1 taxonomy): exact 13 → 15.
Reproduced the old head locally first (`/tmp/m6-old-out`: exact 13,
approximate 28, identical stage counts), then diffed per-model rows.

### The two moved cells (approximate → exact)

| model | operand | old raw-GFA refusal (`c50ec1fa`) | new result (`b98cf32f`) |
|---|---|---|---|
| mambo-basic-b67 | 8 faces (5 cyl + 3 plane), diag 14.044572 | `AssemblyFailed("closed hole shell is not contained by any growth region")` → approx 95 all-plane faces | exact, 12 faces (5 cyl + 7 plane), 0 free / 0 over |
| mambo-simple-s33 | 16 faces (2 cyl + 14 plane), diag 86.168440 | same `AssemblyFailed` → approx 87 all-plane faces | exact, 21 faces (3 cyl + 18 plane), 0 free / 0 over |

Operands are byte-identical between heads (same census, same diag);
the flip is engine-side. Cold-build A/B isolation: parent `73c49ceb^`
still refuses both (same hole-shell error, approx 95/87 faces);
commit `73c49ceb` (PR #884, B64/B65 lidded-box collar lumps) renders
both exact with the census above, deterministic over repeats (3/3 exact
at HEAD). Attributing commit: #884. Its message names two candidate
mechanisms — `remove_doubled_faces` keeping mixed-carrier lens groups,
and multi-shell negatives consulting outward flux for the growth/hole
sign. The pre-fix error is a hole-vs-growth containment refusal on
cylinder+plane operands (no spheres involved), which points at the
outward-flux sign rule rather than the sphere-cap lens keep, but the
exact hunk was not isolated (that is F2's one-revert check, not this
census). Earlier suspects were refuted by the same cold-build method:
B64's parent already postdates them, and the stale-overwrite runs that
briefly implicated `fd6c300c` (check trim containment) and `658fedcf`
(phase-VV vertex-pair pruning) were discarded as build artifacts (see
Method §3).

## The 15 exact cells (operand → result, all 0 free / 0 over)

b0 (2cyl+7pl → 12pl), b10 (1cyl+6pl → 2cyl+12pl), b15 (4nurbs →
2nurbs+2pl; tessellation still fails, see Failures), b20 (5pl → 9pl),
b23 (1cyl+4pl → 1cyl+8pl), b29 (2cyl+6pl → 10pl), b43 (10cyl+6pl →
8cyl+6pl), b65 (2cyl+6pl → 3cyl+11pl), b67, b72 (4cyl+3pl → 4cyl+4pl),
b74 (4cyl+3pl → 4cyl+4pl), b76 (16pl → 23pl), b8 (6pl+1sph →
12pl+1sph), m2 (11pl+15cyl+2cone → 10pl+8cyl), s33. All-analytic
results stay analytic; all-planar operands stay planar. (b0/b29 lose
their cylinders because the cut removes them entirely — vacuous, not a
degradation.)

## The 26 approximate cells (per-model, grouped by cause)

All operands import closed (free 0 / over 0 on every operand, printed
before any diagnosis). Disclosed quality is `Approximate @ 0.1` in all
26 cells; every approximate result is all-planar (analytic types lost
to the mesh co-refinement fallback), from 22 to 2234 faces.

### Cluster A — open shell dropped at assembly (11 models)

Raw GFA errors (fail-closed `AssemblyFailed`, no result) and the gate
falls back. Owner lane: GFA `BuilderSolid` assembly
(`crates/algo/src/builder/builder_solid.rs`); the missing piece is
upstream face splitting/classification leaving an open shell.

| model | operand (plane/cyl/sph/nurbs) | diag / probe side | raw-GFA refusal | fallback faces |
|---|---|---|---|---|
| b34 | 5/2/0/0 (7) | 17.306 / 8.653 | open growth shell, 7 faces | 1850 |
| b36 | 28/0/0/0 (28) | 36.715 / 18.358 | open growth shell, 26 faces | 44 |
| b44 | 9/7/0/0 (16) | 14.177 / 7.089 | open growth shell, 6 faces | 165 |
| b52 | 6/2/0/0 (8) | 14.629 / 7.314 | open growth shell, 4 faces | 77 |
| b53 | 6/4/0/0 (10) | 14.629 / 7.314 | open growth shell, 6 faces | 92 |
| b55 | 5/3/0/0 (8) | 14.457 / 7.228 | open growth shell, 6 faces | 42 |
| b63 | 5/3/0/0 (8) | 18.466 / 9.233 | open growth shell, 5 faces | 41 |
| s27 | 34/18/0/0 (52) | 112.418 / 56.209 | open growth shell, 12 faces | 682 |
| s36 | 18/3/0/0 (21) | 263.721 / 131.861 | open HOLE shell, 13 faces | 185 |
| s41 | 23/10/3n (36) | 174.926 / 87.463 | open growth shell, 7 faces | 1194 |
| s43 | 16/8/3n (27) | 174.926 / 87.463 | open growth shell, 7 faces | 1155 |

Sub-clusters: A-plane (b36: all-planar, plane×plane only — no curved
arm is even involved); A-curved (the other 10); A-nurbs (s41/s43: 3
genuinely curved NURBS faces each, control-net deviation rel 4e-3–2e-2,
NOT planes in disguise — measured natively, see below).

### Cluster B — GFA result rejected at the gate (11 models)

Raw GFA returns a result, but with free boundary edges (± non-manifold
edges, ± one unclosed wire); `validate_boolean_result` rejects it and
the gate falls back.

| model | operand | diag | raw-GFA result (free / over / unclosed wires / non-manifold) | fallback faces |
|---|---|---|---|---|
| b32 | 10/5/0/0 (15) | 7.154 | 20 f (5cyl+15pl), 14 / 4 / 1 / 4 | 54 |
| b33 | 9/6/0/0 (15) | 7.017 | 23 f (6cyl+17pl), 15 / 5 / 1 / 5 | 63 |
| b35 | 19/0/0/0 (19) | 35.953 | 26 pl, 8 / 4 / 0 / 4 | 22 |
| b37 | 32/0/0/0 (32) | 40.294 | 70 pl, 16 / 8 / 0 / 8 | 62 |
| b57 | 11/10/0/0 (21) | 13.748 | 24 f (8cyl+16pl), 8 / 0 / 0 / 0 | 89 |
| b59 | 10/4/4s (18) | 15.564 | 28 f (4cyl+20pl+4sph), 36 / 0 / 0 / 0 | 588 |
| b60 | 3/1/1s (5) | 34.641 | 7 f (6pl+1sph), 10 / 0 / 0 / 0 | 1626 |
| b69 | 3/4/0/0 (7) | 14.671 | 10 f (5cyl+5pl), 10 / 0 / 0 / 0 | 105 |
| b9 | 2/0/1s (3) | 30.000 | 6 f (5pl+1sph), 8 / 0 / 0 / 0 | 2234 |
| s0 | 12/7/0/0 (19) | 209.669 | 17 f (2cyl+15pl), 12 / 0 / 0 / 0 | 122 |
| s31 | 22/20/0/0 (42) | 61.449 | 22 f (12cyl+10pl), 32 / 0 / 0 / 0 | 130 |

Sub-clusters: B-plane (b35, b37: all-planar); B-open (b57, b59, b60,
b69, b9, s0, s31: free edges only, manifold otherwise); B-tangled
(b32, b33: free + non-manifold + unclosed wire).

### Cluster C — degenerate remnant refused by the min-face gate (2 models)

| model | operand | diag | raw-GFA result | gate reason | fallback faces |
|---|---|---|---|---|---|
| b12 | 2pl+1torus (3) | 5.339 | 1 plane face, 3 free | only 1 face (min 3) | 174 |
| b75 | 4pl+6cyl (10) | 12.247 | 3 f (2cyl+1pl), 4 free | only 2 faces post-heal (min 3) | 116 |

The centred box nearly swallows (b12) or nearly misses the keep region;
the analytic remainder is not a closed solid. Whether the true cut is
this degenerate was not independently verified (no ray-cast probes);
recorded as gate behavior, not as a correctness claim.

### Cluster D — carried-circle trim mismatch in face images (2 models)

| model | operand | diag | raw-GFA refusal | fallback faces |
|---|---|---|---|---|
| b61 | 3pl+1cyl+1sph (5) | 38.419 | `FaceSplitFailed("carried circle range endpoints do not match the wire edge": trim=(3.9162, 4.3009) … guard=1e-5)` | 377 |
| s45 | 9pl+4cyl+3nurbs (16) | 174.926 | same site, trim=(0.12828, 0.20136) … guard=1e-5 | 1097 |

Site: `crates/algo/src/builder/fill_images_faces.rs:4580` (face-image
population, downstream of splitting). The carried circle trim and the
wire edge disagree beyond the 1e-5 guard (100× linear tolerance — the
weld-scale band class). s45's 3 NURBS faces are genuinely curved
(same control-net measurement as s41/s43).

### NURBS-planarity measurements (ruling the disguise hypothesis in/out)

Native control-net check per NURBS face (plane through first three
non-collinear control points, max deviation / span):

- s41: 6 pts span 66.34 dev 0.28 (rel 4.2e-3); 4 pts span 61.60 dev
  1.43/1.31 (rel ~2.2e-2). Genuinely curved.
- s43/s45: same three nets (6 pts dev 0.75; 4 pts dev 1.31/1.43).
  Genuinely curved. There is NO planes-in-disguise cluster in this
  census (the `planar_nurbs_as_plane` path is not implicated).
- b15 (exact boolean, failing elsewhere): 2 faces dev 0.0 (planes in
  disguise, harmless here) + 2 faces dev 100 over span 111.8
  (strongly curved).

## The 10 failures

### `invalid_input` read failures (6) — STEP entity classification

Five are the same class: a `B_SPLINE_CURVE_WITH_KNOTS` edge carrier
whose endpoint misses the carrier beyond the 1e-6 local recovery cap —
accurate refusal, no widening (same doctrine as the O1.1d endpoint
authority; caller-supplied tolerance override P-3.5 is the named path,
cf. mambo-b25). One is the period-winding `FACE_BOUND` class:

| model | entity | miss vs cap 1e-6 |
|---|---|---|
| b1 | EDGE_CURVE #253 on B-spline (start) | 6.16e-5 |
| m4 | EDGE_CURVE #589 on B-spline (start) | 1.22e-5 |
| m7 | EDGE_CURVE #2817 on B-spline (end) | 4.71e-5 |
| m8 | EDGE_CURVE #11714 on B-spline (start) | 4.04e-4 |
| m9 | EDGE_CURVE #614 on B-spline (end) | 2.16e-4 |
| b6 | ADVANCED_FACE #46 bound #54: does not close in unwrapped cylinder UV; period-winding bounds cannot be assigned an outer role | n/a (topological refusal) |

Carrier entities resolved against the cached files (all five edge
carriers are `B_SPLINE_CURVE_WITH_KNOTS`; b1/m9 files also contain
`CIRCLE`/`LINE` edges elsewhere — the failures are on the B-spline
ones). b6 is the O1.1d period-winding class that #177 narrowed but did
not close.

### `invalid_topology` validate failures (3) — importer vs model

- b39 (20 faces 2cyl+18pl): check-crate supplement fails with 2
  `WireSelfIntersection` errors at deviation 0.0 (edges 3&11) and
  8.95e-16 (edges 1&9); ops gate is clean. Native probe-cut replay on
  the imported solid gives a cluster-B outcome (23 faces, 4 free →
  approx 180 faces), so the cut itself is also unqualified.
- b41 (28 faces 2cyl+26pl): same signature, deviations 8.95e-16
  (edges 15&19) and 1.83e-15 (edges 1&6); native cut → 37 faces,
  16 free → approx 181 faces.
- Attribution: the zero/sub-ulp deviations prove the touches exist in
  the imported 3D geometry (the checker measures true distance), which
  rules out reader loop mis-ordering (that class shows large
  deviations). This is the B33/B53 tangent-touch signature: loops that
  touch at a point. Whether the touch is the file's own degenerate loop
  or created by reader-side vertex handling was not resolved to file
  bytes (edge ids are runtime ids; mapping them to STEP entities needs
  a debug print that was out of budget) — recorded as the open half of
  this classification. The reader performs no healing (per `io-formats`),
  so a file-side touch imports faithfully.
- s1 (14 faces 7cyl+7pl): boolean-stage failure, different class —
  raw GFA returns 22 faces with 29 free edges and the mesh fallback
  also fails (`NonManifoldResult`). Supplement on the operand is valid
  (2 orientation warnings only). Belongs with cluster B mechanically
  (open GFA result) but the fallback cannot save it, so it surfaces as
  a boolean failure rather than a disclosed approximation.

### `tolerance_violation` tessellation failure (1)

- b15: all-NURBS 4-face solid; boolean EXACT (4 faces). Supplement
  carries 12 `VertexOnSurface` warnings at deviation ≈1e4 — the model
  scale (carrier control points reach z=10000 while trims sit near
  z≈0; file is mm-unit, Spatial InterOp export, uncertainty 1e-6).
  Tessellation fails open (760 indexed/welded boundary edges).
  Whether the carrier/trim mismatch is file-side sloppiness or
  reader-side trim handling needs a viewer check — open. This is the
  scale/tolerance-band class, not a boolean-fallback class (per the
  doctrine: not every scenario failure is a boolean fallback).

## Cluster summary and owner mapping

| cluster | models | FF content | owner row / TERMINAL |
|---|---|---|---|
| A-plane (open shell, all-planar) | b36 (1) | plane×plane only | O2.3 splitter arrangement (or same-domain handling); F2 to confirm. Not TERMINAL |
| B-plane (gate-reject, all-planar) | b35, b37 (2) | plane×plane only | same as A-plane. Not TERMINAL |
| A-curved (open shell) | b34, b44, b52, b53, b55, b63, s27, s36, (s41, s43 minus NURBS) (8–10) | plane×cyl/sph (+plane×NURBS for s41/s43) | P-Class 2.4 (quadric pair quality; exact arms exist for plane×cyl/sph/cone — failure is splitter/classification/assembly, not a missing arm); s41/s43 NURBS faces → P-Class 2.5 genuinely-curved slices (no trimmed FF extent, no transversal EF detection). Not TERMINAL |
| B-curved/B-tangled (gate-reject) | b32, b33, b57, b59, b60, b69, b9, s0, s31 (9) | plane×cyl/sph | P-Class 2.4, same note. b32/b33 add non-manifold + unclosed wire (junction-identity class). Not TERMINAL |
| C (degenerate remnant) | b12, b75 (2) | plane×torus (b12; plane×torus arm is grid-march+fit), plane×cyl (b75) | P-Class 2.4 torus scope (b12); min-face gate behavior (b75). Not TERMINAL |
| D (carried-circle trim mismatch) | b61, s45 (2) | circle-trim vs wire edge, guard 1e-5 | O2.3 splitter / face-image lane (`fill_images_faces.rs:4580`); P-Class 2.4 alternate. Not TERMINAL |
| NURBS-carrier-plane in disguise | — (0 models; ruled out by measurement) | — | — |
| Read-refusal NURBS edges | b1, m4, m7, m8, m9 (5) | STEP import, endpoint authority | O1.1d triage class (P-3.5 override path). Not TERMINAL |
| Read-refusal period winding | b6 (1) | STEP import | O1.1d triage class (#177 narrowed). Not TERMINAL |
| Validate tangent-touch wires | b39, b41 (2) | import/supplement | B33/B53 tangent-touch class; importer-vs-model half open. Not TERMINAL |
| Tessellation scale band | b15 (1) | NURBS carrier extent | O1.1d triage / P-Class 2.5 NURBS domain. Not TERMINAL |

No cluster matches a TERMINAL entry in the roadmap skill
(figure-eight cylinder-union render, plane-through-sphere seam,
scoop fuse, snap-clip, merge-key): all remain chasable. Euler-characteristic
warnings co-occur on approximate AND exact models (b43/b72/b74/m2 are
exact with V-E+F 6/3/3/3), so Euler warnings are noted per model but
not used as a cluster signal.

## Candidates for F2 (census ranking)

**Decision (F2, 2026-10-02, PR #935):** candidate 1 (all-planar: b35, b36, b37)
→ [B79](roadmap.md#b79), a closed plane-face section loop crossing an existing
hole, reproduced from boxes. Runners-up D (b61/s45), C (b12/b75), curved A/B
and NURBS remain recorded on the B79 row. The census below preserves the
evidence available before that selection.

Ranked by: narrowest failing configuration first (exact arms already
exist ⇒ smallest scope), then model count, then fixture cost.
Selection is F2's job; no ready-repro is written here and no B-row is
added.

| rank | family | models | reduces to primitives? | owner row | TERMINAL names it? |
|---|---|---|---|---|---|
| 1 | All-planar probe-cut failures (A-plane + B-plane) | 3 (b35, b36, b37) | Unknown — MAMBO slotted/pocketed blocks seen only by census (19/28/32 planes); F2 to attempt box-vs-notched-block reduction, else STEP-faithful fixture from cache (no bytes committed) | O2.3 | No |
| 2 | Carried-circle trim mismatch (D) | 2 (b61, s45) | plausibly — box × cylinder/sphere + centred box cut; exact error site pinned (`fill_images_faces.rs:4580`, guard 1e-5) | O2.3 (P-Class 2.4 alternate) | No |
| 3 | Degenerate remnants (C) | 2 (b12, b75) | plausibly — box × torus segment (b12), box × small cylinder set (b75) + centred cut | P-Class 2.4 | No |
| 4 | Curved open-shell / gate-reject probe cuts (A-curved + B-curved incl. B-tangled, excluding the NURBS trio) | 17 (b32–b34, b44, b52, b53, b55, b57, b59, b60, b63, b69, b9, s0, s27, s31, s36) | plausibly per-model (box × cylinders/spheres + centred cut), but 17 distinct geometries need F2 triage before any single repro | P-Class 2.4 | No |
| 5 | Genuinely-curved NURBS probe cuts | 3 (s41, s43, s45; overlap with candidate 2 at s45) | No — needs STEP-faithful `*_inmem` fixture (curved carriers, rel dev 4e-3–2e-2, not primitive-reducible) | P-Class 2.5 | No (open slices) |

Notes for F2: (a) all 26 probe cuts use the same centred-box recipe,
so every family shares one replay harness shape; (b) the s41/s43/s45
trio shares NURBS nets and the 174.926 diag — likely one model family
with variants; (c) b39/b41 (validate-stage, not boolean-stage) already
have a native cluster-B-equivalent outcome if the wire-touch question
resolves model-side.

## Session log

- Proven: local scoreboard at `b98cf32f` (40/50; 15 exact / 26 approx;
  6/3/1 taxonomy); old-head reproduction at `c50ec1fa` (13/28, same
  counts); movers b67+s33 and the moving commit #884 via cold-build A/B
  at `73c49ceb^` vs `73c49ceb`; all operands closed (free/over 0);
  per-model first-refusal reasons; NURBS nets genuinely curved
  (s41/s43/s45) — disguise hypothesis refuted by measurement; read
  failures all B-spline-carrier or period-winding; b39/b41 touches at
  0/sub-ulp deviation.
- Refuted: B64 Fix attribution to any narrower hunk (not isolated);
  `fd6c300c`/`658fedcf` as movers (early runs were stale-build
  artifacts — discarded, see Method §3); Euler warnings as a
  discriminator (present on exact cells too); NURBS-disguise cluster
  (empty).
- Next (F2): pick a family from the candidates table, reduce to a
  primitive or STEP-faithful repro, write the ready-repro. Family
  selection completed by F2 (PR #935): candidate 1 → [B79](roadmap.md#b79).
