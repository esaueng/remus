# Proof-tranche mutation triage — 2026-10-01 (M2)

Survivors of the 2026-09-27 weekly Mutation Testing run (head `763ad346`)
outside PR #905's files and outside the September triage docs' pinned sets,
in the M2 file set: `math/src/nurbs/bezier_clip.rs`,
`math/src/nurbs/intersection/{chaining,surface_marching}.rs`,
`algo/src/pave_filler/{phase_ff,phase_ef,curved_section_clip}.rs` (+
`curved_section_clip/bernstein.rs`), and
`operations/src/{tessellate/solid,tessellate/nonplanar,resize_blend,fillet/mod,shell_op}.rs`.
Of the 1,596 in the M2 file set, the CI lists split **513 MISSED, 2 TIMEOUT,
1,081 unexamined** (see Source and scope): only the MISSED and TIMEOUT mutants
ran against code in CI; the 1,081 unexamined never ran and are untested, not
survivors. Every MISSED or TIMEOUT mutant in the `math` and `algo` files was
re-run under the committed config on current code, plus every unexamined
mutant in the functions those runs reached; the `operations` files and the
remaining unexamined-only functions were not reached in the time box and are
listed under Not yet examined. A mutant enters the "Needs review (F3)" table
only if it ran and was MISSED (or TIMEOUT) on current code. No survivor is
called by any sameness word; no assertion was weakened; no file under
`builder/face_splitter/` or `arrangement/` is touched (open PR #905 owns
those survivors).

## Source and scope

- Source of truth: GitHub Actions run 36302856971 (Mutation Testing,
  2026-09-27, head `763ad346`). All 24 shards failed closed on missed or
  unexamined. Shard reports are artifacts `mutants-report-shard-N` containing
  `mutants.out/{missed,timeout,unexamined,caught}.txt`, `outcomes.json`, and
  per-mutant `diff/`.
- The survivor list was read from every shard's `missed.txt`, `timeout.txt`,
  and `unexamined.txt` (downloaded with `gh run download 36302856971 -n
  mutants-report-shard-$i`), concatenated and deduped by full mutant name:
  2,883 unique survivors workspace-wide; 2,367 after dropping everything under
  `builder/face_splitter/` and `arrangement/` (#905's files).
- Source split of the 1,596 (CI lists, deduped by full mutant name):
  **513 MISSED, 2 TIMEOUT, 1,081 unexamined**. Only MISSED and TIMEOUT ran
  against code in CI; the unexamined remainder never ran — it is untested,
  not surviving. Per file (missed / timeout / unexamined):
  `bezier_clip.rs` 106/0/126, `chaining.rs` 68/2/120,
  `surface_marching.rs` 110/0/200, `phase_ff.rs` 22/0/89,
  `phase_ef.rs` 10/0/7, `curved_section_clip.rs` 38/0/105,
  `bernstein.rs` 5/0/35, `tessellate/solid.rs` 36/0/38,
  `tessellate/nonplanar.rs` 59/0/125, `resize_blend.rs` 53/0/214,
  `fillet/mod.rs` 6/0/21, `shell_op.rs` 0/0/1.
- In scope for this doc (the M2 file set above): **1,596** survivors.
  Out of scope but counted: `local_wound.rs` 162, `sphere_pole_patch.rs` 128,
  `measure/volume.rs` 109, `boolean/mod.rs` 58, `cdt/constraints.rs` 52,
  `affine_blend_caps.rs` 37, `blend/src/analytic.rs` 32, `planar.rs` 32,
  `remove_blends.rs` 23, `boundary_evolution.rs` 22, and smaller files (771
  total). Those files were either pinned by the September docs
  (`mutants-{algo-ff,tessellate,geometry,topology,check,blend-reconstruction}-2026-09-*.md`,
  PRs #664/#667/#681) or lie outside the M2 file set; they are not retriaged
  here and remain owned by their rows.
- Line numbers below are the CI run's (head `763ad346`). Line drift to
  current code is normal (production files identical to `origin/main` except
  noted below); every survivor was confirmed by function name at `main`
  before triage, and re-run matching is by file, function, and mutation, not
  by line number.

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly workflow's pin), the committed
  `.cargo/mutants.toml` (profile `ci-test`, nextest, `--tests`,
  first-failure stop, per-package oracle). Scope verified with
  `scripts/test-mutants-scope.py` before any count is quoted.
- Before: the CI run's missed/timeout/unexamined lists. After: new tests in
  the owning crates (the oracle is per package: a `math` test kills a `math`
  mutant; an `operations` test does not kill a `math` mutant).
- Kill attribution: a survivor is marked caught only when it runs and is
  caught on current code under the committed config. The new tests below were
  verified the same way the grid and fraction classes were first checked:
  single-mutant manual runs (mutant patch applied, narrowed test run, revert)
  for `cell_key` (representative of the grid class) and by construction for
  the other grid/fraction assertions (each asserts a hand-computed value the
  constant mutant cannot produce) — and then confirmed by the full re-runs
  recorded here.
- Re-run (2026-10-02, branch head; M2 production files identical to
  `origin/main` except `chaining.rs` test-only additions and line drift in
  `fillet/mod.rs`, `shell_op.rs`, `nonplanar.rs` since the CI run):
  `cargo mutants -p remus-math -F "<chaining functions>" -j32` (list 189:
  49 missed, 128 caught, 10 unviable, 2 timeout),
  `cargo mutants -p remus-algo -F "<curved_section_clip + bernstein functions>" -j32`
  (list 298: 73 missed, 216 caught, 9 unviable),
  `cargo mutants -p remus-math -F "<bezier_clip functions>" -j32` (list 393:
  199 missed, 136 caught, 58 unviable),
  `cargo mutants -p remus-math -F "<surface_marching functions>" -j32`
  (list 557: 191 missed, 308 caught, 58 unviable),
  `cargo mutants -p remus-algo -F "<phase_ef functions>" -j32` (list 111:
  81 missed, 5 caught, 25 unviable),
  `cargo mutants -p remus-algo -F "<phase_ff functions>" -j32` (list 549:
  184 missed, 281 caught, 84 unviable).
  Each `-F` is a regex alternation over the file's functions that held a CI
  MISSED or TIMEOUT mutant (`--file` is ignored under the committed
  `.cargo/mutants.toml`); scope confirmed with
  `scripts/test-mutants-scope.py` before quoting any count (CDT scope,
  exclusions, and verdict/planner tests pass). The `operations` scopes
  (2,019 listed across `fillet/mod`, `resize_blend`, `tessellate/solid`,
  and `tessellate/nonplanar`; `shell_op` unscoped) were not completed in the
  6-hour box (a `fillet/mod` run was launched but yielded no outcomes before
  the box ended), nor were the unexamined-only functions; their mutants sit
  under Not yet examined.
- Verdicts: caught on re-run (ran and caught on current code); (F3) needs
  review — the mutant ran and survived (MISSED or TIMEOUT) on current code.
  `bezier_clip.rs` and `surface_marching.rs` ran-and-survived mutants stay in
  the table without kill attempts, as assigned.
  A test that accepts `None` or "any direction" kills nothing and was not
  written. If a mutant had exposed what looked like a real defect, the failing
  test would be filed with `#[ignore = "open: …"]` and listed first; no such
  defect was found.
- `bezier_clip.rs` (168 ran and survived) and `surface_marching.rs` (168 ran
  and survived) stay in F3 by assignment, without kill attempts.

## New tests and their oracles

| Test module | What it pins | Oracle (one line) |
| --- | --- | --- |
| `math/src/nurbs/intersection/chaining.rs::grid_oracle_tests` (6 tests) | `cell_key` hand quotients; `ring_cells` 0/1/2 membership and counts; `neighborhood` 27 distinct cells; `StampSet` epoch advance and retirement | Hand-computed `(p/width).floor()` triples on dyadic widths, Chebyshev-distance shells (1, 26, 98 cells), and epoch `(1, 2)` — shares no code with the grid |
| `algo/src/pave_filler/curved_section_clip/tests.rs::fraction_is_the_hand_computed_affine_coordinate`, `::same_is_exact_equality_and_finite_rejects_non_finite` (2 tests) | `fraction` affine coordinate and domain refusals; `same` exact equality; `finite` non-finite refusal | `(0.75-0.5)/(1.0-0.5)=0.5`, endpoint 0/1, out-of-domain `Err`, `1.0==1.0` but `1.0!=1.0+1e-12`, `NaN`/`Inf` refused |

No new files were created (both additions are inline `#[cfg(test)]` modules),
so the AGENTS.md Module Map needs no new rows.

## Before / after per file (CI counts; re-run measured where run)

CI survivors split missed / timeout / unexamined; re-run outcomes split
caught on re-run / still missed-or-timeout (F3) / unviable on re-run /
not yet examined. A mutant is F3 only if it ran and survived on current
code; `bezier_clip.rs` and `surface_marching.rs` survivors stay in F3
without kill attempts, as assigned.

| File | CI (m / t / u) | Caught on re-run | Still missed/timeout (F3) | Not yet examined |
| --- | ---: | ---: | ---: | ---: |
| `chaining.rs` | 68 / 2 / 120 | 80 | 51 | 56 |
| `bezier_clip.rs` | 106 / 0 / 126 | 34 | 168 | 14 |
| `surface_marching.rs` | 110 / 0 / 200 | 111 | 168 | 12 |
| `curved_section_clip.rs` | 38 / 0 / 105 | 72 | 65 | 5 |
| `curved_section_clip/bernstein.rs` | 5 / 0 / 35 | 8 | 8 | 24 |
| `phase_ff.rs` | 22 / 0 / 89 | 67 | 30 | 6 |
| `phase_ef.rs` | 10 / 0 / 7 | 0 | 15 | 0 |
| `tessellate/solid.rs` | 36 / 0 / 38 | 0 | 0 | 74 |
| `tessellate/nonplanar.rs` | 59 / 0 / 125 | 0 | 0 | 184 |
| `resize_blend.rs` | 53 / 0 / 214 | 0 | 0 | 267 |
| `fillet/mod.rs` | 6 / 0 / 21 | 0 | 0 | 27 |
| `shell_op.rs` | 0 / 0 / 1 | 0 | 0 | 1 |
| **Area total** | **513 / 2 / 1,081** | **372** | **505** | **670** |

Re-run-confirmed catches in the new-test classes: every current `cell_key`
(incl. whole-body), `ChainGrid::ring_cells`, `StampSet::next_epoch`, and
`fraction` mutant caught (0 still missed in those functions); the 2 CI
TIMEOUT mutants (`nearest_unused_ring` `+=` with `*=`/`-=`) reproduce as
TIMEOUT on current code and stay in F3. `ChainGrid::neighborhood`, `same`,
and `finite` were unexamined in CI and sit in unexamined-only functions the
re-runs did not reach, so their kills rest on the new tests plus the
single-mutant manual runs and construction arguments only. All other
re-run functions keep their ran-and-survived mutants in F3 below; unreached
files and functions are listed under Not yet examined, owned by the B19
budget proposal.

## Needs review (F3)

505 mutants that ran and survived (MISSED or TIMEOUT) on current code, with
file:line (current code), the mutation, what was tried, and the test that
would be written if the closed form were known. Mutants caught on re-run are
not here (see Caught on re-run); mutants never run are not here (see Not yet
examined). No survivor is called by any sameness word. If a mutant had
exposed a real defect it would be listed first with an `#[ignore]`
ready-repro; none did.

### `chaining.rs` — chaining and ring search (51 in F3)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `chaining.rs:245:12` etc. (16 in `chain_intersection_points`) | `chain_intersection_points` | comparison and arithmetic flips | Re-ran on current code: still missed; needs a multi-branch SSI fixture with a known chain count | Chain the two-line crossing fixture (as in `bezier_clip` line tests) through the grid and assert two chains of known length |
| `chaining.rs:580:14` (2, TIMEOUT on re-run, as in CI) | `nearest_unused_ring` | `+=` with `*=` / `-=` | Re-ran on current code: still times out under the 120 s auto timeout | Bound the ring walk with a step budget so the mutant fails fast, then assert the walk result on the 3×3×3 lattice |
| `chaining.rs:572:43` etc. (31 in `nearest_unused_ring`) | `nearest_unused_ring` | comparison, arithmetic, assignment flips | Re-ran on current code: still missed; ring expansion needs a grid with a known nearest unused at a known ring | Build a 3×3×3 lattice with one unused point at ring 2 and assert the walk returns it |
| `chaining.rs:425:28` etc. (2 in `chain_from_adjacency`) | `chain_from_adjacency` | arithmetic flips | Re-ran on current code: still missed | Same lattice fixture asserting the full chain output |

### `bezier_clip.rs` (168 ran and survived, no kill attempts per assignment)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `bezier_clip.rs:158:19` | `param_tolerance` | `/` with `*` | Nothing (assignment); re-ran on current code: still missed | A unit-speed line where `tolerance/speed` is hand-computed; assert the parameter slack equals `tol` |
| `bezier_clip.rs:407:33` | `fat_line_normal` | `>` with `>=` | Nothing (assignment); re-ran on current code: still missed | Two parallel segments at exactly the flatness bound; assert the normal is `None` vs `Some` per the bound |
| `bezier_clip.rs:785:21` | `tolerance_contact` | `>` with `==` | Nothing (assignment); re-ran on current code: still missed | Two clips touching at exactly `tol`; assert contact is reported |
| `bezier_clip.rs` (165 others across `bezier_clip_recurse`, `fat_line_normal`, `newton_refine`, `project_onto_window`, `tangent_and_curvature`, `clip_to_fat_line`, `coincident_to_second_order`, `check_overlap*`, `merge_*`, `shared_window`, `SubSegment::flatness`, `ClipSide::*`) | various | various | Nothing (assignment); re-ran on current code: still missed | Per-function hand-solved line/circle crossings on 3-4-5 geometry, as the file's existing line tests do |

### `surface_marching.rs` (168 ran and survived, no kill attempts per assignment)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `surface_marching.rs:223:18` | `orthonormal_difference` | `<=` with `>` | Nothing (assignment); re-ran on current code: still missed | Orthonormal frames of two axis-aligned planes; assert the difference matrix entries by hand |
| `surface_marching.rs:537:33` | `find_branch_directions` | `+` with `-` | Nothing (assignment); re-ran on current code: still missed | The `z=c·x·y` saddle at known `c`; assert the reported branch count and tangent |
| `surface_marching.rs:542:30` | `find_branch_directions` | `\|\|` with `&&` | Nothing (assignment); re-ran on current code: still missed | Same saddle fixture with the branch-gate inputs at the boundary |
| `surface_marching.rs` (165 others across `orthonormal_difference`, `null_directions`, `find_branch_directions`, `march_direction_with_branches`, `singular_tangent_direction`, `ssi_tangent_params`, whole-function) | various | various | Nothing (assignment); re-ran on current code: still missed | Steep-saddle crossing fixtures with hand-derived branch/tangent, as `tests.rs::marching_oracles` does |

### `phase_ff.rs` (30 ran and survived)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `phase_ff.rs:3197:49` | `trim_torus_oval_to_box_face` | `<` with `<=` | Re-ran on current code: still missed. Compared against the September `mutants-algo-ff` oracles (disc distance, torus oval on caps); the 09-27 line sits in the oval-domain preamble added after that run, so the old fixtures do not reach it | A disk-cap oval whose domain starts below zero (as in the September doc's 3197:61 kill) with the rim seam at the re-run line's branch |
| `phase_ff.rs:5479:38` | `is_marched_trace_of_tangent_rim` | `*` with `+` | Re-ran on current code: still missed. Read the function: it scales a tangent-rim tolerance by the trace length; no existing helper-oracle fixture covers the tangent-rim path | A hand-placed tangent rim trace (circle tangent to the wall) asserting traced vs not-traced at the scaled bound |
| `phase_ff.rs:7762:21`, `7771:37` | `clip_line_to_face` | `-` with `+`, `*` with `+` | Re-ran on current code: still missed. Compared against September `disc_clip_*` oracles; the 09-27 lines are in the shifted preamble (same function, new lines), so coverage was assumed but not re-measured | Re-run the September disc-clip generated-segments oracle with the line remap through the current preamble |
| `phase_ff.rs` (26 others: `clip_line_to_face`, `trim_torus_oval_to_box_face`, `perform_with_context`, `single_hit_circle_is_graze`, `conic_arc_axis_extrema`) | various | various | Re-ran on current code: still missed. Compared function names against the September `mutants-algo-ff-2026-09-25.md` tables; the 09-27 mutants sit at shifted lines in the same functions | Same-class oracles as that doc's `helper_oracle_tests.rs`, remapped to current lines |

### `phase_ef.rs` (15 ran and survived)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `phase_ef.rs:873:60` | `edge_extra_pave_within` | `>` with `<` | Re-ran on current code: still missed. Read the function: the line is the ambiguity flag `ambiguous \|= (pos-best_pos).length() > distinct_tol`; building the GFA arena fixture (edge pave blocks with two extra paves at a known separation) was not attempted in the time box | Two extra paves on one edge at distance `d` apart with `distinct_tol` just below/above `d`; assert the within-radius query returns `None` (ambiguous) vs the nearer vertex |
| `phase_ef.rs:855:5`, `868:25` etc. (13 in `edge_extra_pave_within`) | `edge_extra_pave_within` | whole-body, arithmetic, comparison, logic flips | Re-ran on current code: still missed; no arena fixture was built | Same two-pave fixture with radius/accept gates at the boundary |
| `phase_ef.rs:763:67` | `check_edge_face_pairs` | `<=` with `>` | Re-ran on current code: still missed | A hand-placed edge-face pair at exactly the gate distance |

### `curved_section_clip.rs` + `bernstein.rs` (73 ran and survived)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `curved_section_clip.rs:589:17` | `clip_section` | `&&` with `\|\|` | Re-ran on current code: still missed. Read the site: it conjoins the chart-coordinate and source-end sameness gates for overlapping events; no overlapping-event fixture was built | Two events on the same chart with different source ends (and vice versa); assert `AmbiguousBoundary` vs success per the gate |
| `curved_section_clip.rs:340:17` | `rectangles` | `\|\|` with `&&` | Re-ran on current code: still missed. Read the site: it disjoins rectangle-bound checks; no bound fixture was built | A loop straddling one bound but inside the other; assert inside vs outside |
| `bernstein.rs:197:33` (`residual`) | `residual` | `\|\|` with `&&` | Re-ran on current code: still missed. Read the site: it conjoins the finiteness and positivity gates on the denominator; the existing `bernstein_residual_bounds_*` test was inspected but not extended to the gate boundary | A homogeneous pair whose denominator lower bound is exactly non-finite vs zero; assert `INFINITY` vs the bound |
| `curved_section_clip.rs` + `bernstein.rs` (70 others: `clip_section`, `rectangles`, `patch`, `inside`, `bezier`, `curve_h`, `exact_sum`, `choose`, `derivative`, `restrict`, `I::div`) | various | various | Re-ran on current code: still missed. The existing `curved_section_clip/tests.rs` fixtures were inspected; extending them to each gate boundary was not attempted | Per-gate one-clause fixtures in the style of `phase_ff/helper_oracle_tests.rs`: hand-placed events, bounds, and intervals at each comparison boundary |

The `operations` files (`tessellate/solid.rs`, `tessellate/nonplanar.rs`,
`resize_blend.rs`, `fillet/mod.rs`, `shell_op.rs`) were not reached by any
re-run in the time box, so none of their mutants enters F3 here; all 553 sit
under Not yet examined.

## Caught on re-run

372 CI-listed mutants ran and were caught on current code (no table needed;
counts per file). In the new-test classes: every current `cell_key` (incl.
whole-body replacements), `ChainGrid::ring_cells`, `StampSet::next_epoch`,
and `fraction` mutant is caught — 0 still missed in those functions. The 2 CI
TIMEOUT mutants are not caught; they reproduce as TIMEOUT and stay in F3.

| File | Caught on re-run |
| --- | ---: |
| `surface_marching.rs` | 111 (`orthonormal_difference` 48, `find_branch_directions` 31, `null_directions` 23, `march_direction_with_branches` 7, `ssi_tangent_params` 1, whole-body 1) |
| `chaining.rs` | 80 (`cell_key` 7 + whole-body 27, `ChainGrid::ring_cells` whole-body 26, `chain_intersection_points` 8 + whole-body 2, `nearest_unused_ring` 5 + whole-body 1, `StampSet::next_epoch` 2 + whole-body 1, `chain_from_adjacency` 2) |
| `curved_section_clip.rs` | 72 (`clip_section` 25, `rectangles` 12 + whole-body 1, `patch` 6, `exact_sum` 5, `inside` 5 + whole-body 1, `bezier` 4, `check_residual` 2, `curve_h` 1) |
| `phase_ff.rs` | 67 (`clip_line_to_face` 28, `trim_torus_oval_to_box_face` 19 + whole-body 1, `conic_arc_axis_extrema` 5, `is_marched_trace_of_tangent_rim` 5 + whole-body 2, `single_hit_circle_is_graze` 5 + whole-body 2) |
| `bezier_clip.rs` | 34 (`newton_refine` 10, `coincident_to_second_order` 3 + whole-body 1, `clip_to_fat_line` 3, `tangent_and_curvature` 3 + whole-body 1, `tolerance_contact` whole-body 3, `SubSegment::flatness` 1, `fat_line_normal` 1, `bezier_clip_recurse` 3, `newton_refine` whole-body 1, `tangent_and_curvature` whole-body 1, `ClipSide::at_param_floor` 2, misc whole-body 2) |
| `curved_section_clip/bernstein.rs` | 8 (`I::div` 4 + `choose` 2 + `residual` 1 + `restrict` 1, incl. `choose` whole-body 2) |
| `phase_ef.rs` | 0 |
| **Total** | **372** |

49 further CI-listed mutants are unviable on current code (`bezier_clip.rs`
16, `surface_marching.rs` 19, `phase_ff.rs` 8, `chaining.rs` 3,
`phase_ef.rs` 2, `curved_section_clip.rs` 1); they build no test target and
carry no verdict either way.

## Not yet examined

670 CI-listed mutants were not run against current code in the time box. They
are untested, not survivors, and NOT for F3; they belong to the B19 budget
proposal (carry the unexamined tail into the next in-diff set or a catch-up
shard matrix).

| File | Not yet examined |
| --- | ---: |
| `resize_blend.rs` | 267 (all: `heal_cylinder_plane_band_surgical` 168, `prove_spring_chain` 29, `face_contains_contiguous_chain` 13, `sharp_triple_corner` 9, `orient_corners` 7, `point_line_distance` 6, `add_certified_closed_circle_edge` 7, `contact_direction` 3, rest) |
| `tessellate/nonplanar.rs` | 184 (all: `table_u_fold_vs` 48, `classify_dense_candidate` 39, `tessellate_nonplanar_cdt` 34, `tessellate_torus_notch_band` 32, `stepped_rim_interior_points` 23, rest) |
| `tessellate/solid.rs` | 74 (all: `tessellate_faces_core` 48, `split_triangles_spanning_boundary_splits` 23, rest) |
| `chaining.rs` | 56 (`ChainGrid::neighborhood` 25 + `ChainGrid::cell_of_index` 12 + `ChainGrid::cell_of_point` 9 + `chain_with_clique` 5 + `nearest_unused_scan` 3 + `nearest_unused_ring` whole-body 1 + `StampSet::value` 1) |
| `fillet/mod.rs` | 27 (all: `reject_variable_support_cliffs` 14, `support_ray_exit` 13) |
| `curved_section_clip/bernstein.rs` | 24 (interval ops and whole-body replacements outside the re-run functions) |
| `bezier_clip.rs` | 14 (`ClipSide::span/mid/at/narrowed`, `SubSegment::extent`, `same_contact` — unexamined-only functions) |
| `surface_marching.rs` | 12 (whole-body replacements and `march_with_branches`, `rkf45_step`, `second_order_tangent`, `surface_newton_step` outside the re-run functions) |
| `phase_ff.rs` | 6 (`section_notches_one_rim` 6) |
| `curved_section_clip.rs` | 5 (`boundary_h` 2, `same` 2, `finite` 1) |
| `shell_op.rs` | 1 (`finish_shell` whole-body) |
| `phase_ef.rs` | 0 |
| **Total** | **670** |

## Budget

84 unexamined in one shard is a plan problem. Measured on this run:
2,883 unique survivors across 24 shards (missed + timeout + unexamined
deduped by full mutant name); per-shard lists run ~120–150 survivors (shard 3:
96 caught, 36 missed, 16 unviable, 1 timeout, 84 unexamined — the unexamined
are the in-diff tail the shard's 300-minute budget never reached). The
in-diff plan prices per-package test cost but not the unexamined tail: a
shard that times out reports its remainder as unexamined rather than
rescheduling it. Proposed, not applied: carry each shard's unexamined list
into the next week's in-diff set (or a dedicated catch-up shard matrix sized
by `scripts/plan-mutation-shards.py`'s per-package costs with the same 300-minute
budget), so the weekly verdict measures the week's whole list instead of its
reachable prefix. No config or workflow change is made here.

## Run provenance (disclosed)

- Fuzz confirmation for this tranche: Fuzz Smoke run 36818334059 on `main`
  (`3b24fd4c`, post-#881 plus the shell-correctness campaign) is green
  (10m4s, all targets success, no crash artifacts). The committed seed
  `fuzz/corpus/modifier_ops/shell-box-fused-with-oblique-cylinder-boss-thin-wall-collapse`
  replays natively through the target's decode path to the typed refusal the
  campaign declares (inner cylinder misses its carrier by 0.7155 over the
  tolerance bound at thickness 0.6, rollback intact).
- Mutation source: `gh run download 36302856971 -n mutants-report-shard-$i`
  for `i` in 0..23; concatenation and dedupe by script (see Budget).
- Killed classes first checked by single-mutant manual runs (mutant patch
  applied, narrowed test run, revert) for `cell_key` (representative of the
  grid class) and by construction for the other grid/fraction assertions
  (each asserts a hand-computed value the constant mutant cannot produce) —
  then confirmed by the re-runs above for every current `cell_key`,
  `ChainGrid::ring_cells`, `StampSet::next_epoch`, and `fraction` mutant (0
  still missed). `ChainGrid::neighborhood`, `same`, and `finite` were not
  reached by any re-run; their kills rest on the new tests plus the manual
  runs and construction arguments only.
- Re-run scope was confirmed with `scripts/test-mutants-scope.py` before any
  count was quoted (CDT scope, long-tail exclusions, and verdict/planner
  tests pass). Production files are identical to `origin/main` except the
  `chaining.rs` test-only additions on this branch and line drift since the
  CI run in `fillet/mod.rs` (+1), `shell_op.rs` (+350), and
  `tessellate/nonplanar.rs` (±54); mutant matching across the drift is by
  file, function, and mutation, not by line number.
- The `operations` scopes (1,580 listed across the four files) and the
  unexamined-only functions were not reached in the 6-hour box; no
  after-run over them is claimed here. The next weekly Mutation Testing run,
  with the B19 budget proposal applied, is their verdict.

## Verification

- `cargo test -p remus-math --lib grid_oracle` — 6 passed.
- `cargo test -p remus-algo --lib curved_section_clip::tests::fraction`
  and `::same_is_exact` — 2 passed.
- `cargo test -p remus-operations --test regress_shell_fuse_cylinder_boss_collapse`
  — 14 passed (fuzz-seed regression held while this tranche was written).
- Re-run outcome counts above come from the six completed
  `cargo mutants -p remus-{math,algo} -F "<per-function regexes>" -j32`
  invocations (lists 189/298/393/557/111/549; per-mutant outcomes in each
  run's `mutants.out/{missed,caught,timeout,unviable}.txt`).
- `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings` on the
  touched crates, `scripts/check-boundaries.sh`,
  `scripts/check-doc-paths.sh`: clean (reported below).
- This is local validation; the weekly Mutation Testing run on `main`
  remains the whole-list proof.
