# remus-operations tessellation mutation triage — 2026-09-25

B19 survivor tranche for `crates/operations/src/tessellate/{nonplanar,planar,solid}.rs`. Every
survivor of the source run is triaged into exactly one of: **killed** by a new test whose oracle is
independent of the code under test; **equivalent / unkillable**, with a one-line proof; or **real
defect**, fixed narrowly with a regression test or filed as a roadmap row.

## Source of truth

- GitHub Actions run 36075171651 (Mutation Testing, sharded, 2026-09-25, source `1fa7d150`; the three
  files are byte-identical at `9a4d4a7`, this branch's base, except `solid.rs`, whose later additions
  sit below every survivor's line). 18 of 19 shards printed a verdict: 2,247 listed, 2,207 examined,
  825 missed. Shard 9 was cancelled by a runner shutdown after 4 misses and ~121 unexamined mutants;
  its 4 misses are included here, the unexamined ones are not (so 829 missed in all).
- Artifact downloads were blocked by this environment's egress policy, so the missed lists were
  harvested from each shard's `mutants-verdict.py` log summary (shard 9: its `MISSED` log lines) and
  cross-checked against every shard's reported `missed` count.
- Tessellation scope: **142** survivors — `nonplanar.rs` 76 (75 + 1 from shard 9), `planar.rs` 55,
  `solid.rs` 11. `split_triangles_spanning_boundary_splits` (#619) postdates the run's source and has
  no survivors in it.

## Method

- `cargo-mutants 27.1.0`, Rust 1.96.0, `--no-config` (the committed config's extra `delete field`
  mutants escape `--file`/`--re` filtering), `--profile ci-test`, `--test-tool nextest`,
  `--baseline skip` (suite verified green first), `--timeout 300`, `-C --lib`, and exactly the
  survivor set selected by an anchored `--re` of their names, line numbers remapped through a line
  diff after the two fixes moved `nonplanar.rs`.
- **Narrowed oracle, conservative by construction:** each run's test filter is the new tests only
  (`-E 'test(/…oracle…/)'`). A mutant caught by a subset of the suite is caught by the suite; a
  mutant missed by the subset was already missed by the full suite in CI. The per-mutant weekly
  oracle (`.cargo/mutants.toml`) is unchanged.
- Coverage probe: env-gated markers at each survivor region, one full `remus-operations` run
  (2,188 tests), to tell unreached code from unguarded code before designing fixtures.
- Oracles (all in `tessellate/tests/mesh_oracles.rs` and `mutation_oracles.rs`, plus inline test
  modules): closed-form distance and normal for every analytic carrier (vertex on the surface to the
  1e-5 weld scale, chord sag at centroids and edge midpoints within twice the deflection — the bound
  `test_max_sag_within_deflection` uses — or within 1.1 × for a NURBS-railed wall, the bound
  `nurbs_trimmed_cylinder_keeps_chords_near_surface` uses), outward orientation per face, closed and
  manifold mesh, mesh volume within the chord slab (`2 d × area`) of a closed form computed by
  quadrature, face area within the developable chord bound, and metamorphic invariance (translation
  and scale). No test pins a triangle list; the one count assertion is scale invariance of a count.

## Real defects found

Three oracles the survivors pointed at were missing, and each exposed a wrong mesh that every existing
check passed (closed, manifold, no inverted triangles, volume inside a loose band).

| Defect | Evidence | Disposition |
| --- | --- | --- |
| Single-rim cone charted at a unit-scale radius | The B37 cone–cylinder fuse at 10× sagged 2.4× the (scaled) deflection, 9× at the finest; at 1000× it meshed 274,571 cone triangles at the coarsest deflection and read 1 % low in volume at the finest. `compute_v_param_range`'s `(-1, 1)` fallback fed `radius_at(1)`. | Fixed: `cone_chart_radius`. The body now meshes identically at 1, 10 and 1000×; regression `pointed_cone_with_a_side_hole_tiles_its_exact_area` fails on the old code. |
| Curved-trim densification dropped whole past its budget | A cylinder cut by a slab tilted 30° meshed chords 1.6 off the wall at 0.005–0.002 deflection; the closed mesh read 643.5 against the exact 676.6 (−4.9 %). | Fixed: `dense_trim_rows_within_budget` keeps the rows the budget admits. Volume within 1e-4 from 0.05 to 0.001; regression `ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound` fails on the old code. Residual 2.2–4.8× sag beside a line trim at 0.002 / 0.0015: **B69**, ignored ready-repro. |
| Cross-drilled bore wall meshed without its densification | The shaft of `cross_drilled_display_mesh_is_closed_and_matches_brep_volume`, bore r = 2: 16× the deflection at 0.05, 397× at 0.002, volume up to 0.87 % high (inside that test's 2 % band); r = 1: 6.5×. The four gate-opening mutants cure it and are caught only because `pclass_curved_blend::cross_drilled_hole_rim_fillet_refuses_wrong_side_material` leans on the defective mesh. | Filed: **B71**, ignored ready-repro; the blend refusal must move to an exact criterion before the gate can open. |

Found while building vehicles, outside the tranche's survivors: the torus notch with its box turned
25° or 30° about x is exact and valid but meshes open at fine deflection (**B70**, ignored
ready-repro); the same body at 15° and 20° is refused exact.

## What the unkilled survivors have in common

- **Torus notch band (15):** the decline for touching loops is unreachable (0 of 2,188 operations
  tests reach it), and the ring-angle interpolation mutants move row vertices by less than the row
  spacing on every notch the exact boolean accepts.
- **Densify-only and tolerance mutants (most of the rest):** they add rows or rim samples, move a tie
  or a noise tolerance, or act where a caller-fixed value (`v_apex = 0`, `n_u ≥ 2`) makes the two
  forms agree. Rows on a straight-ruled wall bound triangle aspect only, never chord sag; every
  emitted row spans all base columns.

## Before / after per function (measured)

| File | Function | Survivors before | Killed (new tests) | Killed (existing test) | Equivalent | Unkillable | Survivors after |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `nonplanar.rs` | `tessellate_torus_notch_band` | 15 | 0 | 0 | 2 | 13 | 15 |
| `nonplanar.rs` | `tessellate_nonplanar_cdt` | 28 | 5 | 4 | 19 | 0 | 19 |
| `nonplanar.rs` | `validate_interior_polygon_work` | 1 | 1 | 0 | 0 | 0 | 0 |
| `nonplanar.rs` | `validate_stepped_rim_level_count` | 2 | 2 | 0 | 0 | 0 | 0 |
| `nonplanar.rs` | `stepped_rim_interior_points` | 13 | 0 | 0 | 12 | 1 | 13 |
| `nonplanar.rs` | `interior_rows_for_boundary` | 17 | 13 | 0 | 4 | 0 | 4 |
| `planar.rs` | `ConstraintIndex::contains` | 15 | 12 | 0 | 3 | 0 | 3 |
| `planar.rs` | `tessellate_revolved_with_holes` | 26 | 3 | 0 | 21 | 2 | 23 |
| `planar.rs` | `cdt_triangulate_simple` | 1 | 0 | 0 | 1 | 0 | 1 |
| `planar.rs` | `ear_clip_triangulate` | 10 | 8 | 0 | 2 | 0 | 2 |
| `planar.rs` | `triangulation_covers_polygon` | 3 | 3 | 0 | 0 | 0 | 0 |
| `solid.rs` | `tessellate_faces_core` | 11 | 0 | 0 | 11 | 0 | 11 |
| **Total** | | **142** | **47** | **4** | **75** | **16** | **91** |

## Survivors by function

Lines are the source run's (`9a4d4a7`); the kill checks ran the same mutants at their shifted
lines after the fixes. "Killed" names the first failing test in the cargo-mutants log.

### `nonplanar.rs` — `tessellate_torus_notch_band`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `1262:53` replace - with + | equivalent | `wrap_pi` then returns its value + 2π for every sample alike: the ring angle enters only through `rem_euclid(TAU)` before `torus.evaluate` and through `delta`, a difference taken mod 2π. |
| `1278:28` replace + with - | unkillable | Degrades linear interpolation of the loop's ring angle between two adjacent samples to piecewise-constant or midpoint. The error is bounded by the loop's u change across one sample gap (≤ 1e-3 rad measured on the 10°-turned notch, against ~8e-3 rad of interior row spacing), and every row vertex stays on the torus inside the band: chord bound, orientation and volume are unchanged. A kill needs a loop whose ring angle moves faster than the row spacing between samples; the exact boolean refuses the 15° and 20° turns and the 25°+ turns mesh open (B70). |
| `1283:23` replace - with + | unkillable | Degrades linear interpolation of the loop's ring angle between two adjacent samples to piecewise-constant or midpoint. The error is bounded by the loop's u change across one sample gap (≤ 1e-3 rad measured on the 10°-turned notch, against ~8e-3 rad of interior row spacing), and every row vertex stays on the torus inside the band: chord bound, orientation and volume are unchanged. A kill needs a loop whose ring angle moves faster than the row spacing between samples; the exact boolean refuses the 15° and 20° turns and the 25°+ turns mesh open (B70). |
| `1283:23` replace - with / | unkillable | Degrades linear interpolation of the loop's ring angle between two adjacent samples to piecewise-constant or midpoint. The error is bounded by the loop's u change across one sample gap (≤ 1e-3 rad measured on the 10°-turned notch, against ~8e-3 rad of interior row spacing), and every row vertex stays on the torus inside the band: chord bound, orientation and volume are unchanged. A kill needs a loop whose ring angle moves faster than the row spacing between samples; the exact boolean refuses the 15° and 20° turns and the 25°+ turns mesh open (B70). |
| `1284:15` replace <= with > | unkillable | Degrades linear interpolation of the loop's ring angle between two adjacent samples to piecewise-constant or midpoint. The error is bounded by the loop's u change across one sample gap (≤ 1e-3 rad measured on the 10°-turned notch, against ~8e-3 rad of interior row spacing), and every row vertex stays on the torus inside the band: chord bound, orientation and volume are unchanged. A kill needs a loop whose ring angle moves faster than the row spacing between samples; the exact boolean refuses the 15° and 20° turns and the 25°+ turns mesh open (B70). |
| `1287:45` replace / with % | unkillable | Degrades linear interpolation of the loop's ring angle between two adjacent samples to piecewise-constant or midpoint. The error is bounded by the loop's u change across one sample gap (≤ 1e-3 rad measured on the 10°-turned notch, against ~8e-3 rad of interior row spacing), and every row vertex stays on the torus inside the band: chord bound, orientation and volume are unchanged. A kill needs a loop whose ring angle moves faster than the row spacing between samples; the exact boolean refuses the 15° and 20° turns and the 25°+ turns mesh open (B70). |
| `1287:45` replace / with * | unkillable | Degrades linear interpolation of the loop's ring angle between two adjacent samples to piecewise-constant or midpoint. The error is bounded by the loop's u change across one sample gap (≤ 1e-3 rad measured on the 10°-turned notch, against ~8e-3 rad of interior row spacing), and every row vertex stays on the torus inside the band: chord bound, orientation and volume are unchanged. A kill needs a loop whose ring angle moves faster than the row spacing between samples; the exact boolean refuses the 15° and 20° turns and the 25°+ turns mesh open (B70). |
| `1296:36` replace < with <= | equivalent | `collect_torus_phi_ring` only returns single-period windings (±2π), so `outer_winding` is never 0. |
| `1316:26` replace < with <= | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |
| `1316:26` replace < with == | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |
| `1316:33` replace || with && | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |
| `1316:44` replace > with == | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |
| `1316:44` replace > with >= | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |
| `1316:50` replace - with + | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |
| `1316:50` replace - with / | unkillable | Weakens the decline for loops within 1e-6 rad of touching in u. No test or construction reaches it (coverage probe: 0 of 2,188 operations tests); a notch band that thin or that complete is a sliver or the near-full complement, where the ruled rows are still well formed, so only the fallback mesher's own output would differ. |

### `nonplanar.rs` — `tessellate_nonplanar_cdt`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `3071:34` replace * with + | killed (existing test) | `pclass_curved_blend::cross_drilled_hole_rim_fillet_refuses_wrong_side_material` at this base (the CI source missed it; see B71: this mutant opens the NURBS-rail gate on the cross-drilled bore) |
| `3071:34` replace * with / | equivalent | Shrinks the rim band to 1e-9 / dv; rim samples come from the shared pool on the rim circle itself and project to its v to rounding, so the count is unchanged. |
| `3074:34` replace <= with > | killed (existing test) | `pclass_curved_blend::cross_drilled_hole_rim_fillet_refuses_wrong_side_material` at this base (the CI source missed it; see B71: this mutant opens the NURBS-rail gate on the cross-drilled bore) |
| `3074:43` replace + with * | equivalent | Tests the bottom rim against `v_min · tol`: identical for `v_min = 0` (walls starting at their carrier origin), and otherwise it can only lower the count, i.e. close the gate, which the full rims' own samples (≥ 8 per rim) keep open on every railed wall. |
| `3074:64` replace >= with < | killed (existing test) | `pclass_curved_blend::cross_drilled_hole_rim_fillet_refuses_wrong_side_material` at this base (the CI source missed it; see B71: this mutant opens the NURBS-rail gate on the cross-drilled bore) |
| `3080:13` replace && with || | killed (existing test) | `pclass_curved_blend::cross_drilled_hole_rim_fillet_refuses_wrong_side_material` at this base (the CI source missed it; see B71: this mutant opens the NURBS-rail gate on the cross-drilled bore) |
| `3106:29` replace || with && | equivalent | Lets non-NURBS edges into the bending test when the gate is open; on a cylinder or cone wall those are rims (constant v) and generators (constant u), which the both-directions span test rejects anyway. |
| `3127:32` replace - with + | equivalent | Weakens the u-extent half of the bending test, so NURBS edges that run straight along v (seams stored as NURBS) also count as curved: the densified band only widens, on walls already past the gate. |
| `3127:32` replace - with / | equivalent | Weakens the u-extent half of the bending test, so NURBS edges that run straight along v (seams stored as NURBS) also count as curved: the densified band only widens, on walls already past the gate. |
| `3127:41` replace > with >= | equivalent | Tie at exactly `1e-9` of the span. |
| `3127:48` replace * with / | equivalent | Shrinks one extent threshold, so more NURBS edges count as curved: the densified band only widens. |
| `3127:53` replace && with || | equivalent | Counts NURBS edges that extend in either direction (rims or seams stored as NURBS) as curved: the densified band only widens, on walls already past the gate. |
| `3127:63` replace - with / | equivalent | Weakens the v-extent half of the bending test the same way: the densified band only widens. |
| `3127:72` replace > with >= | equivalent | Tie at exactly `1e-9` of the span. |
| `3127:79` replace * with / | equivalent | Shrinks one extent threshold, so more NURBS edges count as curved: the densified band only widens. |
| `3137:29` replace > with >= | equivalent | Only differs when every curved sample shares one v; an ellipse is never constant-v and the both-directions span test excludes constant-v NURBS, so the band is never a single level. |
| `3137:44` replace && with || | equivalent | With no curved samples the band is `[inf, -inf]`: one wanted row whose v is NaN, and `point_in_polygon_2d` admits no NaN point, so nothing is added. |
| `3137:56` replace > with >= | equivalent | `dense_dv = dv / n_u` is never exactly 1e-15. |
| `3138:40` replace - with + | equivalent | Drops only the margin row below the trim's lowest sample; the rim below it is at most one dense spacing further down, so the chords there stay shorter than a column (measured: every oracle unchanged on the slab, cone and stepped vehicles). |
| `3141:34` replace - with + | equivalent | Asks for more rows (`(hi + lo) / dense_dv`), which the budget cap then bounds: over-densification only. |
| `3141:40` replace / with % | killed | `ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound` |
| `3141:40` replace / with * | killed | `ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound` |
| `3170:78` replace + with - | equivalent | Moves the foot used by the clearance filter, so candidates within ~0.4 column of the trim are kept or dropped differently; kept ones never lie on a constraint (that would crack the shared rim, and every vehicle asserts a closed mesh) and the chord, orientation and volume oracles are unchanged (measured). |
| `3171:53` replace > with >= | equivalent | Tie at exactly the clearance distance. |
| `3175:42` replace - with + | equivalent | Stretches the band by 2·lo / (hi − lo) (7 % on the slab), so a few top rows land past `hi` and are clipped; the remaining rows stay denser than the base columns (measured: every oracle unchanged). |
| `3175:48` replace * with + | killed | `ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound` |
| `3175:48` replace * with / | killed | `ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound` |
| `3175:62` replace / with % | killed | `ellipse_trimmed_cylinder_wall_stays_within_the_chord_bound` |

### `nonplanar.rs` — `validate_interior_polygon_work`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `3446:38` replace > with >= | killed | `polygon_work_at_exactly_the_limit_is_accepted` |

### `nonplanar.rs` — `validate_stepped_rim_level_count`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `3474:45` replace / with * | killed | `stepped_rim_levels_are_budgeted_against_the_column_count` |
| `3477:33` replace > with >= | killed | `stepped_rim_levels_are_budgeted_against_the_column_count` |

### `nonplanar.rs` — `stepped_rim_interior_points`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `3651:32` replace - with + | equivalent | Keeps duplicate levels, so neighbouring gaps read zero and flank rows are skipped: 20 % fewer triangles on the stepped wall with sag (0.12 × deflection), orientation and volume identical at six deflections (measured). On a straight-ruled wall rows only bound triangle aspect; every emitted row spans all base columns, which bound the chord across the arc. |
| `3651:32` replace - with / | equivalent | Keeps duplicate levels, so neighbouring gaps read zero and flank rows are skipped: 20 % fewer triangles on the stepped wall with sag (0.12 × deflection), orientation and volume identical at six deflections (measured). On a straight-ruled wall rows only bound triangle aspect; every emitted row spans all base columns, which bound the chord across the arc. |
| `3655:41` replace - with + | unkillable | `span` is only compared against tolerance. Levels are measured from the carrier origin or apex, which every native constructor and transform keeps at or below the wall (levels ≥ 0), and then `last + first > tol` exactly when `last − first > tol` for three distinct levels. Only a carrier placed above its wall's bottom — reachable by import alone — could tell them apart. |
| `3656:26` replace || with && | equivalent | Three distinct levels after the tolerance dedup cannot span ≤ tolerance, and NaN levels need NaN vertices; for every finite input both forms agree. |
| `3716:28` replace / with * | equivalent | Collapses each inter-level band to one subdivision row; level rows and flanks stay, and on a straight-ruled wall rows only bound triangle aspect, never sag (measured: every oracle unchanged). |
| `3717:36` replace - with + | equivalent | Changes which of at most six subdivision rows the stride keeps; same argument as the row-count mutant above. |
| `3717:36` replace - with / | equivalent | Changes which of at most six subdivision rows the stride keeps; same argument as the row-count mutant above. |
| `3728:30` replace - with + | equivalent | Keeps rows within tolerance of each other; their candidates coincide and the CDT's point insertion welds them onto one vertex. |
| `3728:30` replace - with / | equivalent | Keeps rows within tolerance of each other; their candidates coincide and the CDT's point insertion welds them onto one vertex. |
| `3767:34` replace > with < | equivalent | Emits only the floor/ceil columns of every boundary sample; the wall's full rims are sampled at least at column density across the chart, so those are every base column and the point set is the same. |
| `3767:34` replace > with >= | equivalent | `n_u` comes from `interior_grid_resolution`, which floors it at 2, so `n_u ≥ 1` and `n_u > 1` agree. |
| `3769:45` replace > with == | equivalent | Redundant with `validate_stepped_rim_level_count`: nine rows per level within `MAX_INTERIOR_GRID_POINTS / columns` already bounds `rows × columns`; equality needs exactly the budget in rows, ~1e6 / columns of them. |
| `3769:45` replace > with >= | equivalent | Redundant with `validate_stepped_rim_level_count`: nine rows per level within `MAX_INTERIOR_GRID_POINTS / columns` already bounds `rows × columns`; equality needs exactly the budget in rows, ~1e6 / columns of them. |

### `nonplanar.rs` — `interior_rows_for_boundary`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `3869:24` replace - with + | killed | `rows_are_translation_and_scale_invariant` |
| `3870:20` replace * with / | killed | `documented_sample_count_bounds_decide_growth` |
| `3873:30` replace > with >= | equivalent | Differs only for a sample exactly at `v_range ± 1e-9·dv`, the edge of a noise allowance with no correct side; rim samples sit at the rim itself. |
| `3873:42` replace + with * | killed | `rows_are_translation_and_scale_invariant` |
| `3873:53` replace < with <= | equivalent | Differs only for a sample exactly at `v_range ± 1e-9·dv`, the edge of a noise allowance with no correct side; rim samples sit at the rim itself. |
| `3887:23` replace > with >= | killed | `documented_sample_count_bounds_decide_growth` |
| `3887:46` replace < with <= | killed | `documented_sample_count_bounds_decide_growth` |
| `3892:30` replace > with >= | equivalent | Differs only for a sample exactly at `v_range ± 1e-9·dv`, the edge of a noise allowance with no correct side; rim samples sit at the rim itself. |
| `3892:42` replace + with * | killed | `a_constant_v_run_keeps_two_rows_anywhere_on_the_axis` |
| `3892:53` replace < with <= | equivalent | Differs only for a sample exactly at `v_range ± 1e-9·dv`, the edge of a noise allowance with no correct side; rim samples sit at the rim itself. |
| `3909:26` replace > with < | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |
| `3909:26` replace > with == | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |
| `3909:26` replace > with >= | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |
| `3910:11` replace + with * | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |
| `3910:38` replace / with * | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |
| `3910:45` replace - with + | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |
| `3910:45` replace - with / | killed | `grown_rows_fill_but_never_exceed_the_grid_budget` |

### `planar.rs` — `ConstraintIndex::contains`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `280:26` replace - with + | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `281:70` replace - with + | killed | `constraint_index_answer_is_translation_invariant_in_v` |
| `281:70` replace - with / | killed | `constraint_index_answer_is_translation_invariant_in_v` |
| `291:23` replace < with <= | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:23` replace < with == | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:30` replace || with && | equivalent | Weakens the tolerance-inflated AABB prefilter only; every point it lets through is decided by the exact segment-distance test, and the box contains every point within `tol` of the segment. |
| `291:40` replace > with == | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:40` replace > with >= | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:47` replace || with && | equivalent | Weakens the tolerance-inflated AABB prefilter only; every point it lets through is decided by the exact segment-distance test, and the box contains every point within `tol` of the segment. |
| `291:57` replace < with <= | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:57` replace < with == | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:64` replace || with && | equivalent | Weakens the tolerance-inflated AABB prefilter only; every point it lets through is decided by the exact segment-distance test, and the box contains every point within `tol` of the segment. |
| `291:74` replace > with == | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `291:74` replace > with >= | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |
| `297:29` replace > with >= | killed | `constraint_index_counts_points_at_exactly_the_tolerance` |

### `planar.rs` — `tessellate_revolved_with_holes`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `563:48` replace * with / | equivalent | Shrinks the single-rim tolerance; a rim's samples share its v to rounding (≤ 1e-13 at 1000×, measured through the 1, 10, 1000 sweep), inside both tolerances. |
| `563:65` replace + with - | equivalent | Same tolerance, scaled by (|hi| − |lo| + 1) instead of (|hi| + |lo| + 1); a rim's v spread is rounding-level, far inside either. |
| `575:19` replace - with + | equivalent | The rim-on-apex guard. Callers pass `v_apex = 0`, so `|v_rim ± 0|` and `|v_rim / 0| = ∞` compare the same way for every rim above the apex; a rim at the apex is a zero-area cone. |
| `575:19` replace - with / | equivalent | The rim-on-apex guard. Callers pass `v_apex = 0`, so `|v_rim ± 0|` and `|v_rim / 0| = ∞` compare the same way for every rim above the apex; a rim at the apex is a zero-area cone. |
| `575:43` replace * with / | equivalent | Tolerance scale of the same guard; it fires only for a zero-height cone. |
| `575:58` replace + with * | equivalent | Tolerance scale of the same guard; it fires only for a zero-height cone. |
| `575:58` replace + with - | equivalent | Tolerance scale of the same guard; it fires only for a zero-height cone. |
| `594:62` replace / with % | killed | `pointed_cone_with_a_side_hole_tiles_its_exact_area` |
| `594:62` replace / with * | killed | `pointed_cone_with_a_side_hole_tiles_its_exact_area` |
| `611:28` replace - with + | equivalent | Callers pass `v_apex = 0`, so `v_rim − v_apex = v_rim + v_apex`. |
| `616:30` replace - with + | equivalent | Shifts the u of apex-row chart vertices only; every one of them folds onto the apex and the triangles spanning the row are dropped as zero-area, so the 3D fan from the apex is unchanged (measured: area, volume and scale-invariant count unchanged). |
| `616:57` replace / with % | equivalent | Same apex-row u only (`j % nu = j`); folded onto the apex as above. |
| `622:28` replace - with + | equivalent | Callers pass `v_apex = 0`, so `v_rim − v_apex = v_rim + v_apex`. |
| `741:32` replace - with + | equivalent | The grid spans `outer_v.0 + (outer_v.1 ± outer_v.0)·t`; `outer_v.0 = 0` for every cone chart (the apex) and every wall starting at its carrier origin. Otherwise the rows stretch by `2·v0 / span` and rows past the top are clipped; on a straight-ruled wall rows only bound triangle aspect. |
| `769:69` replace + with * | unkillable | Breaks the constraint index for wrapping inner loops (`band_ranges`) only, so a grid seed lying exactly on a band edge would survive and crack the rim (the B47 class). No test or construction found gives a cylinder wall a wrapping inner loop with an edge on a grid row; the B47 fixture is a pocket. |
| `769:69` replace + with - | unkillable | Breaks the constraint index for wrapping inner loops (`band_ranges`) only, so a grid seed lying exactly on a band edge would survive and crack the rim (the B47 class). No test or construction found gives a cylinder wall a wrapping inner loop with an edge on a grid row; the B47 fixture is a pocket. |
| `774:23` replace * with / | equivalent | Tolerance scale of the on-constraint seed drop; seeds on a constraint sit on it to rounding (B47: ~1e-16), inside either tolerance, and seeds off it are a grid step away. |
| `774:73` replace - with + | equivalent | Same tolerance with `span + ...` for `span − ...`; the same argument. |
| `876:26` replace * with / | equivalent | Tolerance of the apex-row fold. Apex-row chart vertices are inserted with `v = v_apex` exactly and no other vertex is within a row of it, so any non-negative tolerance selects exactly them. |
| `876:42` replace + with * | equivalent | Tolerance of the apex-row fold. Apex-row chart vertices are inserted with `v = v_apex` exactly and no other vertex is within a row of it, so any non-negative tolerance selects exactly them. |
| `876:53` replace - with + | equivalent | Tolerance of the apex-row fold. Apex-row chart vertices are inserted with `v = v_apex` exactly and no other vertex is within a row of it, so any non-negative tolerance selects exactly them. |
| `876:69` replace + with * | equivalent | Tolerance of the apex-row fold. Apex-row chart vertices are inserted with `v = v_apex` exactly and no other vertex is within a row of it, so any non-negative tolerance selects exactly them. |
| `876:69` replace + with - | equivalent | Tolerance of the apex-row fold. Apex-row chart vertices are inserted with `v = v_apex` exactly and no other vertex is within a row of it, so any non-negative tolerance selects exactly them. |
| `879:36` replace - with + | equivalent | Callers pass `v_apex = 0`. |
| `879:52` replace <= with > | killed | `pointed_cone_with_a_side_hole_tiles_its_exact_area` |
| `883:28` replace - with + | equivalent | Callers pass `v_apex = 0`. |

### `planar.rs` — `cdt_triangulate_simple`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `1752:15` replace < with > | equivalent | `mapped <= triangles` always, so the branch never fires; a dropped Steiner triangle (positive area) then fails `triangulation_covers_polygon` below, which takes the same `ear_clip_or_fan` fallback. |

### `planar.rs` — `ear_clip_triangulate`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `1793:39` replace || with && | equivalent | A zero or non-finite signed area can only reach the clip loop under the mutant; there every candidate turn is 0 or NaN, and the final coverage check rejects any triangle list against a zero/NaN polygon area, so the result is `None` either way. |
| `1796:33` replace > with == | killed | `ear_clipping_declines_past_its_work_budget` |
| `1796:33` replace > with >= | equivalent | `signed_area_twice` is already known non-zero here, so `> 0` and `>= 0` agree. |
| `1808:29` replace <= with > | killed | `ear_clipping_tiles_concave_polygons_in_both_windings` |
| `1808:37` replace || with && | killed | `ear_clipping_tiles_concave_polygons_in_both_windings` |
| `1808:41` delete ! | killed | `ear_clipping_tiles_concave_polygons_in_both_windings` |
| `1820:29` replace += with *= | killed | `ear_clipping_declines_past_its_work_budget` |
| `1826:23` replace >= with < | killed | `ear_clipping_tiles_concave_polygons_in_both_windings` |
| `1826:35` replace >= with < | killed | `ear_clipping_tiles_concave_polygons_in_both_windings` |
| `1826:47` replace >= with < | killed | `ear_clipping_tiles_concave_polygons_in_both_windings` |

### `planar.rs` — `triangulation_covers_polygon`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `1855:5` replace triangulation_covers_polygon -> bool with true | killed | `coverage_check_rejects_malformed_and_partial_triangulations` |
| `1858:9` replace || with && | killed | `coverage_check_rejects_malformed_and_partial_triangulations` |
| `1883:49` replace * with / | killed | `coverage_check_rejects_malformed_and_partial_triangulations` |

### `solid.rs` — `tessellate_faces_core`

| Mutant | Verdict | Killing test / proof |
| --- | --- | --- |
| `563:51` replace || with && | equivalent | Admits a torus face with a closed or second doubled edge to the rim pass. The pass only densifies (never coarsens) once-used circles in the shared pool, so every neighbour still meets the same vertices. |
| `580:25` replace || with && | equivalent | Admits non-band torus faces to the same densify-only pass: more rim samples in the shared pool, never fewer, so no face can crack or coarsen. |
| `580:40` replace || with && | equivalent | Admits non-band torus faces to the same densify-only pass: more rim samples in the shared pool, never fewer, so no face can crack or coarsen. |
| `616:40` replace * with + | equivalent | A full rim's target becomes `full_cols + 2` instead of `full_cols + 1`; a split rim's arc gets about a full turn's samples. Densify-only; the band's sag stays ≤ 0.25 × deflection. |
| `616:87` replace + with * | equivalent | Target one or two samples short of the band's wrap density: `stitch_rings` pairs the unequal counts and the band stays closed with sag ≤ 0.25 × deflection on the ρ = 1.5 and ρ = 0.05 vehicles (measured); the B46 crack needs a pool far sparser than the rows, which the densification still removes. |
| `616:87` replace + with - | equivalent | Target one or two samples short of the band's wrap density: `stitch_rings` pairs the unequal counts and the band stays closed with sag ≤ 0.25 × deflection on the ρ = 1.5 and ρ = 0.05 vehicles (measured); the B46 crack needs a pool far sparser than the rows, which the densification still removes. |
| `619:50` replace < with <= | equivalent | Also resamples a rim that already has the target count; `sample_uniform` over the same arc with the same count and pinned ends reproduces the pool's uniform circle samples. |
| `842:34` replace && with || | equivalent | Widens the tangent-contact candidate filter only: the consumer re-applies the identical `0 < t < 1` and distance test to every candidate before splitting the line, and an empty group adds zero work. |
| `868:26` replace > with >= | equivalent | Widens the tangent-contact candidate filter only: the consumer re-applies the identical `0 < t < 1` and distance test to every candidate before splitting the line, and an empty group adds zero work. |
| `869:25` replace && with || | equivalent | Widens the tangent-contact candidate filter only: the consumer re-applies the identical `0 < t < 1` and distance test to every candidate before splitting the line, and an empty group adds zero work. |
| `869:30` replace < with <= | equivalent | Widens the tangent-contact candidate filter only: the consumer re-applies the identical `0 < t < 1` and distance test to every candidate before splitting the line, and an empty group adds zero work. |


## Not covered

- The ~121 mutants shard 9 listed but never examined, and the 40 the time-limited shards 3, 4, 5, 10,
  12 and 14 did not reach, are outside this triage. None of their names is recoverable from the logs.
- "After" counts use a narrowed oracle (the new tests, plus `pclass_curved_blend` for the CDT gates),
  so they are an upper bound on today's full-suite survivors: four CDT gate mutants the CI source
  missed are caught by an existing test at this base, and others may be.

## Verification

After merging `origin/main` at `8972565` (with #624, #646, #664, #674 and the other B19 tranches):

- `cargo nextest run --workspace --cargo-profile ci-test`: 6,012 passed, 0 failed.
- `cargo test -p remus-wasm`: 584 + 1 passed.
- `cargo clippy --all-targets --all-features -- -D warnings`, `cargo fmt --all -- --check`,
  `taplo fmt --check`, `cargo machete` (pre-commit hook on every commit but the first, which predates
  installing the hooks in this clone; each hook run checks the whole tree).
- `scripts/check-boundaries.sh`, `check-det-hash.sh`, `check-doc-paths.sh`,
  `check-wildcard-arms.sh`, `check-apache-lineage.sh`, `check-remus-rename.sh`,
  `check-doc-module-map.py`, `check-edge-domain-authority.py`,
  `check-apache-replay-provenance.py`, `check-wasm-version.py --base origin/main`,
  `sync-roadmap-inventory.py --check`: all pass.
- `python3 scripts/check-approx-census.py`: matches all 52 committed rows.
- `cargo xtask wasm-build` (with `wasm-opt` 117 on `PATH`: `wasm-pack`'s own download does not trust
  this environment's TLS proxy) and `node scripts/test-wasm-smoke.mjs`: all pass. The rebuilt
  packages were not committed.
- The two fixes' regression tests fail with the old computation restored and pass with the fix; the
  B69, B70 and B71 ready-repros fail on this base.
- This is local validation; the weekly Mutation Testing run on `main` remains the whole-list proof.
