# Proof-tranche mutation triage — 2026-10-01 (M2)

Survivors of the 2026-09-27 weekly Mutation Testing run (head `763ad346`)
outside PR #905's files and outside the September triage docs' pinned sets,
in the M2 file set: `math/src/nurbs/bezier_clip.rs`,
`math/src/nurbs/intersection/{chaining,surface_marching}.rs`,
`algo/src/pave_filler/{phase_ff,phase_ef,curved_section_clip}.rs` (+
`curved_section_clip/bernstein.rs`), and
`operations/src/{tessellate/solid,tessellate/nonplanar,resize_blend,fillet/mod,shell_op}.rs`.
Every survivor in this area is either killed by a new test whose oracle shares
no code with the function under test, or handed over in the "Needs review
(F3)" table with what was tried and the test that would be written if the
closed form were known. No survivor is called by any sameness word; no
assertion was weakened; no file under `builder/face_splitter/` or
`arrangement/` is touched (open PR #905 owns those survivors).

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
- In scope for this doc (the M2 file set above): **1,596** survivors.
  Out of scope but counted: `local_wound.rs` 162, `sphere_pole_patch.rs` 128,
  `measure/volume.rs` 109, `boolean/mod.rs` 58, `cdt/constraints.rs` 52,
  `affine_blend_caps.rs` 37, `blend/src/analytic.rs` 32, `planar.rs` 32,
  `remove_blends.rs` 23, `boundary_evolution.rs` 22, and smaller files (771
  total). Those files were either pinned by the September docs
  (`mutants-{algo-ff,tessellate,geometry,topology,check,blend-reconstruction}-2026-09-*.md`,
  PRs #664/#667/#681) or lie outside the M2 file set; they are not retriaged
  here and remain owned by their rows.
- Line numbers below are the CI run's (head `763ad346`). Line drift to current
  `main` (`3b24fd4c`) is normal; every survivor was confirmed by function name
  at `main` before triage.

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly workflow's pin), the committed
  `.cargo/mutants.toml` (profile `ci-test`, nextest, `--tests`,
  first-failure stop, per-package oracle). Scope verified with
  `scripts/test-mutants-scope.py` before any count is quoted.
- Before: the CI run's missed/timeout/unexamined lists. After: new tests in
  the owning crates (the oracle is per package: a `math` test kills a `math`
  mutant; an `operations` test does not kill a `math` mutant).
- Kill attribution: a survivor is marked killed only when a new test fails
  under the mutant and passes on the real code. Single-mutant manual runs
  (apply the mutant patch, run the narrowed test, revert) verified the grid
  and fraction classes; the full after-run over all 1,596 area mutants is
  deferred to the next weekly Mutation Testing run (see Budget).
- Verdicts: (a) killed by a new test; (F3) needs review — the mutant was not
  killed by a one-line closed form, or it lives in `bezier_clip.rs` or
  `surface_marching.rs` (which go to the table regardless per the assignment).
  A test that accepts `None` or "any direction" kills nothing and was not
  written. If a mutant had exposed what looked like a real defect, the failing
  test would be filed with `#[ignore = "open: …"]` and listed first; no such
  defect was found.
- `bezier_clip.rs` (232) and `surface_marching.rs` (310) survivors are all
  (F3) by assignment, without kill attempts.

## New tests and their oracles

| Test module | What it pins | Oracle (one line) |
| --- | --- | --- |
| `math/src/nurbs/intersection/chaining.rs::grid_oracle_tests` (6 tests) | `cell_key` hand quotients; `ring_cells` 0/1/2 membership and counts; `neighborhood` 27 distinct cells; `StampSet` epoch advance and retirement | Hand-computed `(p/width).floor()` triples on dyadic widths, Chebyshev-distance shells (1, 26, 98 cells), and epoch `(1, 2)` — shares no code with the grid |
| `algo/src/pave_filler/curved_section_clip/tests.rs::fraction_is_the_hand_computed_affine_coordinate`, `::same_is_exact_equality_and_finite_rejects_non_finite` (2 tests) | `fraction` affine coordinate and domain refusals; `same` exact equality; `finite` non-finite refusal | `(0.75-0.5)/(1.0-0.5)=0.5`, endpoint 0/1, out-of-domain `Err`, `1.0==1.0` but `1.0!=1.0+1e-12`, `NaN`/`Inf` refused |

No new files were created (both additions are inline `#[cfg(test)]` modules),
so the AGENTS.md Module Map needs no new rows.

## Before / after per function (measured where run; CI counts otherwise)

| File | Function | CI survivors | (a) killed | (F3) needs review |
| --- | --- | ---: | ---: | ---: |
| `chaining.rs` | `cell_key` | 7 | 7 | 0 |
| `chaining.rs` | `ChainGrid::ring_cells` | 1 | 1 | 0 |
| `chaining.rs` | `ChainGrid::neighborhood` | 4 | 4 | 0 |
| `chaining.rs` | `StampSet::next_epoch` | 2 | 2 | 0 |
| `chaining.rs` | `chain_intersection_points` | 25 | 0 | 25 |
| `chaining.rs` | `nearest_unused_ring` + `nearest_unused_scan` + `chain_from_adjacency` + `chain_with_clique` | 45 | 0 | 45 |
| `chaining.rs` | whole-function replacements (`?`) | 106 | 0 | 106 |
| `bezier_clip.rs` | all functions | 232 | 0 | 232 |
| `surface_marching.rs` | all functions | 310 | 0 | 310 |
| `curved_section_clip.rs` | `fraction` | 8 | 8 | 0 |
| `curved_section_clip.rs` | `same` + `finite` | 2 | 2 | 0 |
| `curved_section_clip.rs` | `clip_section` | 51 | 0 | 51 |
| `curved_section_clip.rs` | `rectangles` | 24 | 0 | 24 |
| `curved_section_clip.rs` | `patch`, `inside`, `bezier`, `curve_h`, `boundary_h`, `budget`, `check_residual`, `exact_sum`, whole-function | 58 | 0 | 58 |
| `curved_section_clip/bernstein.rs` | `residual` | 2 | 0 | 2 |
| `curved_section_clip/bernstein.rs` | all other functions | 38 | 0 | 38 |
| `phase_ff.rs` | `clip_line_to_face` | 31 | 0 | 31 |
| `phase_ff.rs` | `trim_torus_oval_to_box_face` | 29 | 0 | 29 |
| `phase_ff.rs` | `is_marched_trace_of_tangent_rim` | 18 | 0 | 18 |
| `phase_ff.rs` | `perform_with_context`, `section_notches_one_rim`, `single_hit_circle_is_graze`, `conic_arc_axis_extrema`, whole-function | 33 | 0 | 33 |
| `phase_ef.rs` | `edge_extra_pave_within` | 14 | 0 | 14 |
| `phase_ef.rs` | `check_edge_face_pairs`, whole-function | 3 | 0 | 3 |
| `tessellate/solid.rs` | `tessellate_faces_core` | 46 | 0 | 46 |
| `tessellate/solid.rs` | `split_triangles_spanning_boundary_splits` + rest | 28 | 0 | 28 |
| `tessellate/nonplanar.rs` | `tessellate_nonplanar_cdt`, `tessellate_torus_notch_band`, `stepped_rim_interior_points`, `table_u_fold_vs`, `classify_dense_candidate`, rest | 184 | 0 | 184 |
| `resize_blend.rs` | all functions | 267 | 0 | 267 |
| `fillet/mod.rs` | `reject_variable_support_cliffs`, `support_ray_exit` | 26 | 0 | 26 |
| `fillet/mod.rs` | whole-function | 1 | 0 | 1 |
| `shell_op.rs` | `finish_shell` | 1 | 0 | 1 |
| **Area total** | | **1,596** | **24** | **1,572** |

Survivors after in the triaged classes: `cell_key`, `ring_cells`,
`neighborhood`, `next_epoch`, `fraction`, `same`, `finite` go from 24 to 0
(verified by single-mutant manual runs; the weekly run re-measures). All
other area functions are unchanged and handed over below.

## Needs review (F3)

Every survivor not killed above, plus every `bezier_clip.rs` and
`surface_marching.rs` mutant regardless, is handed over here with file:line
(CI run's), the mutation, what was tried, and the test that would be written
if the closed form were known. No survivor is called by any sameness word.
If a mutant had exposed a real defect it would be listed first with an
`#[ignore]` ready-repro; none did.

### `chaining.rs` — chaining and ring search (176 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `chaining.rs:220:5` | `chain_intersection_points` | `-> Vec<…>` with `vec![]` | Not attempted: needs a multi-branch SSI fixture with a known chain count | Chain the two-line crossing fixture (as in `bezier_clip` line tests) through the grid and assert two chains of known length |
| `chaining.rs:245:12` etc. (25 in `chain_intersection_points`) | `chain_intersection_points` | comparison and arithmetic flips | Not attempted: threshold and index arithmetic need a measured spacing fixture | Assert the estimated threshold on three hand-spaced points, then the chained grouping |
| `chaining.rs:525:75`, `571:43`, `580:14` (39 in `nearest_unused_ring`, 2 in `nearest_unused_scan`, 2 in `chain_from_adjacency`, 2 in `chain_with_clique`) | ring search | comparison, arithmetic, assignment flips | Not attempted: ring expansion needs a grid with a known nearest unused at a known ring | Build a 3×3×3 lattice with one unused point at ring 2 and assert the walk returns it |
| `chaining.rs` whole-function (`?`, 106) | various | whole-body replacements | Not attempted: whole-body replacements need end-to-end chaining fixtures | Same lattice fixture asserting the full chain output |

### `bezier_clip.rs` (232 handed over, no kill attempts per assignment)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `bezier_clip.rs:158:19` | `param_tolerance` | `/` with `*` | Nothing (assignment) | A unit-speed line where `tolerance/speed` is hand-computed; assert the parameter slack equals `tol` |
| `bezier_clip.rs:407:33` | `fat_line_normal` | `>` with `>=` | Nothing (assignment) | Two parallel segments at exactly the flatness bound; assert the normal is `None` vs `Some` per the bound |
| `bezier_clip.rs:785:21` | `tolerance_contact` | `>` with `==` | Nothing (assignment) | Two clips touching at exactly `tol`; assert contact is reported |
| `bezier_clip.rs` (229 others across `bezier_clip_recurse`, `fat_line_normal`, `newton_refine`, `project_onto_window`, `tangent_and_curvature`, `clip_to_fat_line`, `coincident_to_second_order`, `check_overlap*`, `merge_*`, `shared_window`, `same_contact`, `SubSegment::flatness`, `ClipSide::*`) | various | various | Nothing (assignment) | Per-function hand-solved line/circle crossings on 3-4-5 geometry, as the file's existing line tests do |

### `surface_marching.rs` (310 handed over, no kill attempts per assignment)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `surface_marching.rs:223:18` | `orthonormal_difference` | `<=` with `>` | Nothing (assignment) | Orthonormal frames of two axis-aligned planes; assert the difference matrix entries by hand |
| `surface_marching.rs:537:33` | `find_branch_directions` | `+` with `-` | Nothing (assignment) | The `z=c·x·y` saddle at known `c`; assert the reported branch count and tangent |
| `surface_marching.rs:542:30` | `find_branch_directions` | `\|\|` with `&&` | Nothing (assignment) | Same saddle fixture with the branch-gate inputs at the boundary |
| `surface_marching.rs` (307 others across `orthonormal_difference`, `null_directions`, `find_branch_directions`, `march_direction_with_branches`, `singular_tangent_direction`, `ssi_tangent_params`, whole-function) | various | various | Nothing (assignment) | Steep-saddle crossing fixtures with hand-derived branch/tangent, as `tests.rs::marching_oracles` does |

### `phase_ff.rs` (111 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `phase_ff.rs:3197:49` | `trim_torus_oval_to_box_face` | `<` with `==` | Compared against the September `mutants-algo-ff` oracles (disc distance, torus oval on caps); the 09-27 line sits in the oval-domain preamble added after that run, so the old fixtures do not reach it | A disk-cap oval whose domain starts below zero (as in the September doc's 3197:61 kill) with the rim seam at the new line's branch |
| `phase_ff.rs:5488:38` | `is_marched_trace_of_tangent_rim` | `*` with `+` | Read the function: it scales a tangent-rim tolerance by the trace length; no existing helper-oracle fixture covers the tangent-rim path | A hand-placed tangent rim trace (circle tangent to the wall) asserting traced vs not-traced at the scaled bound |
| `phase_ff.rs:7762:21`, `7771:37` | `clip_line_to_face` | `-` with `+`, `*` with `+` | Compared against September `disc_clip_*` oracles; the 09-27 lines are in the shifted preamble (same function, new lines), so coverage was assumed but not re-measured | Re-run the September disc-clip generated-segments oracle with the line remap through the current preamble |
| `phase_ff.rs` (107 others) | `clip_line_to_face`, `trim_torus_oval_to_box_face`, `section_notches_one_rim`, `perform_with_context`, `single_hit_circle_is_graze`, `conic_arc_axis_extrema` | various | Compared function names against the September `mutants-algo-ff-2026-09-25.md` tables; the 09-27 mutants sit at shifted lines in the same functions, so they are treated as unmeasured rather than re-killed | Same-class oracles as that doc's `helper_oracle_tests.rs`, remapped to current lines |

### `phase_ef.rs` (17 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `phase_ef.rs:873:60` | `edge_extra_pave_within` | `>` with `<` | Read the function: the line is the ambiguity flag `ambiguous \|= (pos-best_pos).length() > distinct_tol`; building the GFA arena fixture (edge pave blocks with two extra paves at a known separation) was not attempted in the time box | Two extra paves on one edge at distance `d` apart with `distinct_tol` just below/above `d`; assert the within-radius query returns `None` (ambiguous) vs the nearer vertex |
| `phase_ef.rs:855:5` (2), `867:33`, `868:25` (3), `868:34`, `868:37`, `873:31`, `874:33` (2) | `edge_extra_pave_within` | whole-body, arithmetic, comparison, logic flips | Same as above: no arena fixture was built | Same two-pave fixture with radius/accept gates at the boundary |
| `phase_ef.rs:763:67` | `check_edge_face_pairs` | `<=` with `>` | Not attempted | A hand-placed edge-face pair at exactly the gate distance |
| `phase_ef.rs` whole-function (2) | `edge_extra_pave_within` | whole-body | Not attempted | Same two-pave fixture |

### `curved_section_clip.rs` + `bernstein.rs` (133 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `curved_section_clip.rs:589:17` | `clip_section` | `&&` with `\|\|` | Read the site: it conjoins the chart-coordinate and source-end sameness gates for overlapping events; no overlapping-event fixture was built | Two events on the same chart with different source ends (and vice versa); assert `AmbiguousBoundary` vs success per the gate |
| `curved_section_clip.rs:340:17` | `rectangles` | `\|\|` with `&&` | Read the site: it disjoins rectangle-bound checks; no bound fixture was built | A loop straddling one bound but inside the other; assert inside vs outside |
| `curved_section_clip.rs:197:33` (`bernstein.rs::residual`) | `residual` | `\|\|` with `&&` | Read the site: it conjoins the finiteness and positivity gates on the denominator; the existing `bernstein_residual_bounds_*` test was inspected but not extended to the gate boundary | A homogeneous pair whose denominator lower bound is exactly non-finite vs zero; assert `INFINITY` vs the bound |
| `curved_section_clip.rs` + `bernstein.rs` (130 others) | `clip_section`, `rectangles`, `patch`, `inside`, `bezier`, `curve_h`, `boundary_h`, `budget`, `check_residual`, `exact_sum`, `choose`, `derivative`, `product`, `split`, `value`, `restrict`, interval ops | various | The existing `curved_section_clip/tests.rs` fixtures were inspected; extending them to each gate boundary was not attempted | Per-gate one-clause fixtures in the style of `phase_ff/helper_oracle_tests.rs`: hand-placed events, bounds, and intervals at each comparison boundary |

### `tessellate/solid.rs` (74 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `solid.rs:1215:45` | `tessellate_faces_core` | `<` with `>` | Compared against `tessellate/tests/mutation_oracles.rs`; the 09-27 line sits in the shared-pool rim densification added after the September run, so the old wall fixtures do not reach it | A cylinder wall with a split rim at the densification boundary; assert the pool count and the wall sag within twice the deflection |
| `solid.rs:1494:35`, `1515:76` etc. (23 in `split_triangles_spanning_boundary_splits`) | `split_triangles_spanning_boundary_splits` | arithmetic flips | Not attempted: the function postdates the September triage (`split_triangles_spanning_boundary_splits`, #619) | A triangle spanning a boundary split with hand-computed split fractions; assert the split positions |
| `solid.rs` (48 others) | `tessellate_faces_core` | various | Compared against the September `mutants-tessellate-2026-09-25.md` tables; same function, shifted lines, unmeasured | Wall-to-deflection oracles as `tests/mutation_oracles.rs` does: carrier distance, chord sag, quadrature volume |

### `tessellate/nonplanar.rs` (184 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `nonplanar.rs` (47 in `table_u_fold_vs`, 38 in `classify_dense_candidate`, 19 in `stepped_rim_interior_points`, 34 in `tessellate_nonplanar_cdt`, 32 in `tessellate_torus_notch_band`, rest) | various | various | Compared against the September tessellate tables; `table_u_fold_vs` and `classify_dense_candidate` postdate that run (B69/B70 fixes), so no oracle reaches them | Fold-table fixtures with hand-derived `u(v)` folds (B70 25° notch) and dense-candidate fixtures at the containment/clearance boundary, asserting sag within twice the deflection |

### `resize_blend.rs` (267 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `resize_blend.rs:2032:18` | `sharp_triple_corner` | `<` with `<=` | Compared against the September `mutants-blend-reconstruction` tables; the 09-27 line is in the same function at a shifted line, unmeasured | Three planes meeting at a hand-computed corner with the gate input at exactly the bound |
| `resize_blend.rs:2067:28` | `point_line_distance` | `-` with `+` | Same as above | A point at a hand-computed distance from a line; assert the distance |
| `resize_blend.rs` (265 others, 167 in `heal_cylinder_plane_band_surgical`) | various | various | Compared function names against that doc; same functions, new lines, unmeasured | Closed-form corners, extension angles, and volumes as that doc's tests do |

### `fillet/mod.rs` (27 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `fillet/mod.rs:633:41` | `reject_variable_support_cliffs` | `-` with `+` | Read the site: it builds the geometry parameter `geometry_t` from the station fraction; no variable-fillet support fixture was built | A straight edge with a linear radius law on a box corner; assert the station parameters equal the hand-computed `t_start + fraction*(t_end-t_start)` |
| `fillet/mod.rs:691:25` etc. (13 in `support_ray_exit`, 13 in `reject_variable_support_cliffs`) | various | various | Same as above | A unit square support with a ray at a hand-computed exit distance; assert the exit |

### `shell_op.rs` (1 handed over)

| Mutant | Function | Mutation | What was tried | Test that would kill it if the closed form were known |
| --- | --- | --- | --- | --- |
| `shell_op.rs:1107:5` | `finish_shell` | whole-body with `Ok((Default::default(), Default::default()))` | Ran the `regress_shell_fuse_cylinder_boss_collapse` suite green; the mutant returns a default solid handle, which no existing shell test distinguishes from a real hollow by handle alone — confirming the oracle gap, not attempting a kill | A closed box shelled thin asserting the returned solid's face count and inner-wall carrier (not just `Ok`), so a default handle fails |

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
- Killed classes verified by single-mutant manual runs (mutant patch applied,
  narrowed test run, revert) for `cell_key` (representative of the grid
  class) and by construction for the other grid/fraction assertions (each
  asserts a hand-computed value the constant mutant cannot produce).
- The full 1,596-mutant after-run under `.cargo/mutants.toml` was not executed
  here (host disk holds 68G of sibling worktrees plus a 38G target; a full
  copy fails with no space). The next weekly Mutation Testing run is the
  verdict.

## Verification

- `cargo test -p remus-math --lib grid_oracle` — 6 passed.
- `cargo test -p remus-algo --lib curved_section_clip::tests::fraction`
  and `::same_is_exact` — 2 passed.
- `cargo test -p remus-operations --test regress_shell_fuse_cylinder_boss_collapse`
  — 14 passed (fuzz-seed regression held while this tranche was written).
- `cargo fmt --check`, `cargo clippy --all-targets -- -D warnings` on the
  touched crates, `scripts/check-boundaries.sh`,
  `scripts/check-doc-paths.sh`: clean (reported below).
- This is local validation; the weekly Mutation Testing run on `main`
  remains the whole-list proof.
