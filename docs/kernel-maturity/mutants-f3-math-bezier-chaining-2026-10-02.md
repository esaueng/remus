# F3a mutation triage: `bezier_clip.rs` and `chaining.rs` (2026-10-02)

B19 follow-up to the M2 proof tranche
([mutants-proof-2026-10-01](mutants-proof-2026-10-01.md), "Needs review (F3)").
This slice owns the F3 survivors in `crates/math/src/nurbs/bezier_clip.rs`
and `crates/math/src/nurbs/intersection/chaining.rs`; the
`surface_marching.rs` and `algo` survivors belong to sibling slices and are
not touched here.

Outcome: 206 survivors: 180 killed, 5 equivalent, 5 unkillable (rounding-only), 8 superseded by a defect fix, 8 not reached in `bezier_clip.rs`; 51 survivors: 18 killed, 33 equivalent, 0 unkillable (rounding-only), 0 superseded by a defect fix, 0 not reached
in `chaining.rs`. Together these cover all 219 F3 survivors M2 recorded
(168 + 51) and the further `bezier_clip.rs` survivors a whole-file run
measures (see Method). Every survivor carries a verdict; the 8 "not
reached" are distinguishable in principle but have no oracle yet, and are
listed with what a kill needs (they are not called equivalent). Two real defects were found by the triage and fixed with
regressions (below). No assertion was weakened and no tolerance widened.

## Method

- Tool: `cargo-mutants 27.0.0` under the committed `.cargo/mutants.toml`
  (profile `ci-test`, nextest, `--tests`, first-failure stop, per-package
  oracle), `-j 2`, `CARGO_BUILD_JOBS=4`. `-F` regex scoping (the config
  ignores `--file`).
- Before (survivor set): re-run on `origin/main` at `b98cf32f` (the
  `bezier_clip.rs` run from the unmodified branch before any edit — the
  tool copies the tree at start; the `chaining.rs` run from a
  `git archive` copy of `origin/main`).
  - `bezier_clip.rs`, whole file (`-F bezier_clip`): 521 tested in 39m: 206 missed, 247 caught, 68 unviable.
    M2 recorded 168 ran-and-survived plus 14 unexamined in this file; the
    whole-file run measures 206 missed, 189 of them in the functions M2
    re-ran and 17 in functions M2 did not run (`curve_curve_intersect_full`
    9, `convex_hull_clip` 5, `cross_2d` 2, `SubSegment::magnitude` 1). All
    206 are triaged here.
  - `chaining.rs`, the three F3 functions
    (`-F 'chaining\.rs:.*(chain_intersection_points|nearest_unused_ring|chain_from_adjacency)'`):
    115 tested in 18m: 49 missed, 54 caught, 10 unviable, 2 timeouts. The 49 missed + 2 timeouts are exactly M2's 51.
- After: the same selections on the branch head.
  - `bezier_clip.rs` whole file: 487 tested in 44m: 33 missed, 390 caught, 64 unviable; the 33 survivors were
    then re-run alone after the last tests landed: 33 tested: 20 missed, 13 caught; the two of those 20 with new boundary tests (`407:26`, `932:17` on the branch) were then re-run alone: 2 tested, 2 caught. Whole-file survivors on the branch head: 18 (5 equivalent, 5 unkillable, 8 not reached).
  - `chaining.rs` F3 functions: 115 tested in 9m: 33 missed, 72 caught, 10 unviable, 0 timeouts (the 33 are exactly the 33 equivalents below).
  Every run's tested count equals its selection (each completed with its
  summary line).
- Mutants are matched before/after by function, column and mutation, with
  line numbers mapped through the diff; verdicts below quote the
  `origin/main` line:col.
- A kill is credited only when the mutant ran and was caught on the branch
  head; the killing test is the first failure nextest reported.
  "Equivalent" carries a one-line proof that no input distinguishes it.
  "Unkillable (rounding-only)" means the mutant changes only which side of
  a rounding a computed value lands on; a test would have to pin that
  rounding. "Not reached" means distinguishable in principle but no
  closed-form oracle was built in the time box (each with what it needs).

## Defects found and fixed

1. **Curved coincident overlaps fragmented** (`check_overlap`). The overlap
   test measured each sample's distance to the other window's *samples*.
   Two arcs of one circle sharing 40..80 degrees run at different parameter
   speeds over the shared stretch, so the test passed only on windows below
   ~100 tolerances of arc: the stretch came back as 49 overlap fragments and
   55 point hits. Each sample is now projected onto the other window's
   curve. Regression:
   `bezier_clip/mutation_oracle_tests.rs::coincident_arcs_report_one_overlap_with_closed_form_ends`
   (three scales; failed on `origin/main` with 55 hits). Found while
   building the oracle for the depth-gated overlap route.
2. **Overflowing chaining thresholds dropped points**
   (`chain_with_clique`). An infinite or square-overflowing threshold joined
   every pair of finite points, but the all-pairs comparison `d² < t²` does
   not join a pair whose squared distance overflows; the walk's scan cannot
   select such a point either, so a point 1e200 away vanished from the
   output. The clique path now applies `d² < inf`. Regression:
   `chaining/mutation_oracle_tests.rs::overflowing_threshold_keeps_overflowing_pairs_apart`
   (failed on `origin/main`: one chain of 2 for 3 points). Found from
   survivors `chaining.rs:245:12` (`>` with `>=` / `==`), which became
   equivalent once the clique matched the comparison.

Both fixes change behavior only on these inputs: `curve_curve_intersect*`
has no production caller outside tests and the bench, and no finite input
reaches the clique path through `estimate_chain_threshold`.

## New tests and their oracles

| Test module | What it pins | Oracle |
| --- | --- | --- |
| `crates/math/src/nurbs/bezier_clip/mutation_oracle_tests.rs` | Every bezier-clip helper (param slack, `ClipSide` maps and floor, `SubSegment` extent/magnitude/flatness, fat-line normals incl. collinear axis choice and overflow, fat-line and hull clips, projection, curvature, Newton, contact merge, overlap merge, overlap guards, Hausdorff, tolerance contact) and the recursion's strategy | 3-4-5 and dyadic geometry, circle feet and curvature (1/r toward the centre), exact parabola/segment crossings, threshold pairs bracketing each documented bound (just inside / just outside, or exactly on a strict bound), and hand-derived recursion call counts |
| `crates/math/src/nurbs/intersection/chaining/mutation_oracle_tests.rs` | `nearest_unused_ring` stop rule (scale 1, 1/8 and 2^-500), tier-one and ring tie-breaks, member/used filters, z-distance; adjacency on the scan and grid paths; the power-of-two width doubling; subnormal thresholds; termination | Hand-placed points whose nearest unused member is obvious (a Chebyshev-ring decoy vs a closer point one ring out), asserted together with the full scan (the specification) |

A test-only frame guard (`RecurseFrame`, `#[cfg(test)]`) in
`bezier_clip_recurse` counts calls and asserts that `depth` is the
recursion level. The clip-or-subdivide strategy never changes *which*
contacts are found, only the work, so its oracles are hand-derived call
counts: perpendicular lines converge in 3 calls, a parabola against a
segment inside its fat line in 3 (through the second clip), identical
segments overlap in 1, identical arcs need at least 9 (the depth-8 gate),
a 60%-exact clip is not a good clip (3 calls, not 4). The depth assertion
kills every `depth + 1` mutant on every exercised path.

## Per-function before / after

### `bezier_clip.rs`

| Function | Survivors before | Killed | Equivalent | Unkillable (rounding-only) | Superseded by fix | Not reached |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `curve_curve_intersect_full` | 9 | 9 | 0 | 0 | 0 | 0 |
| `param_tolerance` | 9 | 9 | 0 | 0 | 0 | 0 |
| `ClipSide<'s>::at_param_floor` | 2 | 2 | 0 | 0 | 0 | 0 |
| `SubSegment::magnitude` | 1 | 1 | 0 | 0 | 0 | 0 |
| `SubSegment::flatness` | 7 | 7 | 0 | 0 | 0 | 0 |
| `fat_line_normal` | 29 | 26 | 3 | 0 | 0 | 0 |
| `clip_to_fat_line` | 3 | 3 | 0 | 0 | 0 | 0 |
| `convex_hull_clip` | 5 | 4 | 0 | 1 | 0 | 0 |
| `cross_2d` | 2 | 2 | 0 | 0 | 0 | 0 |
| `bezier_clip_recurse` | 40 | 32 | 2 | 0 | 0 | 6 |
| `check_overlap_aligned` | 5 | 5 | 0 | 0 | 0 | 0 |
| `tolerance_contact` | 10 | 10 | 0 | 0 | 0 | 0 |
| `shared_window` | 3 | 3 | 0 | 0 | 0 | 0 |
| `project_onto_window` | 23 | 20 | 0 | 1 | 0 | 2 |
| `check_overlap` | 33 | 25 | 0 | 0 | 8 | 0 |
| `tangent_and_curvature` | 7 | 7 | 0 | 0 | 0 | 0 |
| `coincident_to_second_order` | 7 | 7 | 0 | 0 | 0 | 0 |
| `newton_refine` | 4 | 1 | 0 | 3 | 0 | 0 |
| `merge_duplicate_hits` | 3 | 3 | 0 | 0 | 0 | 0 |
| `merge_overlaps` | 4 | 4 | 0 | 0 | 0 | 0 |
| **Total** | **206** | **180** | **5** | **5** | **8** | **8** |

### `chaining.rs` (F3 functions)

| Function | Survivors before | Killed | Equivalent | Unkillable (rounding-only) | Superseded by fix | Not reached |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `chain_intersection_points` | 16 | 7 | 9 | 0 | 0 | 0 |
| `chain_from_adjacency` | 2 | 0 | 2 | 0 | 0 | 0 |
| `nearest_unused_ring` | 33 | 11 | 22 | 0 | 0 | 0 |
| **Total** | **51** | **18** | **33** | **0** | **0** | **0** |

## Every mutant

### `bezier_clip.rs`

| Mutant (origin/main line:col) | Before | Verdict |
| --- | --- | --- |
| `142:8: delete ! in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `145:13: delete ! in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:31: replace >= with < in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:45: replace - with + in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:45: replace - with / in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:53: replace && with \|\| in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:61: replace <= with > in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:73: replace + with * in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `147:73: replace + with - in curve_curve_intersect_full` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `156:5: replace param_tolerance -> f64 with -1.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `156:5: replace param_tolerance -> f64 with 0.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `156:5: replace param_tolerance -> f64 with 1.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |
| `157:26: replace && with \|\| in param_tolerance` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::param_tolerance_is_tolerance_over_speed` |
| `157:35: replace > with < in param_tolerance` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_arcs_overlap_only_past_the_depth_gate` |
| `157:35: replace > with == in param_tolerance` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `157:35: replace > with >= in param_tolerance` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::param_tolerance_is_tolerance_over_speed` |
| `158:19: replace / with % in param_tolerance` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |
| `158:19: replace / with * in param_tolerance` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |
| `241:9: replace ClipSide<'s>::at_param_floor -> bool with false` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_side_param_floor_scales_with_magnitude` |
| `242:44: replace * with / in ClipSide<'s>::at_param_floor` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_side_param_floor_scales_with_magnitude` |
| `325:9: replace SubSegment::magnitude -> f64 with 1.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::sub_segment_extent_magnitude_flatness` |
| `334:9: replace SubSegment::flatness -> f64 with -1.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_arcs_overlap_only_past_the_depth_gate` |
| `334:9: replace SubSegment::flatness -> f64 with 0.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_arcs_overlap_only_past_the_depth_gate` |
| `334:9: replace SubSegment::flatness -> f64 with 1.0` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::sub_segment_extent_magnitude_flatness` |
| `337:34: replace != with == in SubSegment::flatness` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::sub_segment_extent_magnitude_flatness` |
| `340:32: replace / with % in SubSegment::flatness` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::sub_segment_extent_magnitude_flatness` |
| `340:32: replace / with * in SubSegment::flatness` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::sub_segment_extent_magnitude_flatness` |
| `345:20: replace - with + in SubSegment::flatness` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::sub_segment_extent_magnitude_flatness` |
| `366:5: replace fat_line_normal -> Option<Vec3> with None` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `369:23: replace <= with > in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_curved_window` |
| `369:35: replace * with + in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_curved_window` |
| `369:35: replace * with / in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_curved_window` |
| `379:8: delete ! in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `379:32: replace && with \|\| in fat_line_normal` | MISSED | Equivalent: chord_len in {0, NaN} makes dir NaN, and chord_len = inf makes dir zero while every later `<= chord_len·k` test is true, so the axis fallback crosses a zero dir; both reach the final finite-length check and return None, as line 379 does. |
| `379:45: replace > with < in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `379:45: replace > with == in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `379:45: replace > with >= in fat_line_normal` | MISSED | Equivalent: chord_len = 0 gives dir = 0·inf = NaN, which reaches the final finite-length check and returns None, as line 379 does. |
| `382:28: replace / with % in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_matches_the_hand_interval` |
| `382:28: replace / with * in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_straight_window_borrows_from_b` |
| `387:19: replace - with + in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_matches_the_hand_interval` |
| `392:24: replace <= with > in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_straight_window_borrows_from_b` |
| `392:37: replace * with + in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_curved_window` |
| `392:37: replace * with / in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_curved_window` |
| `395:24: replace <= with > in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `395:37: replace * with + in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `395:37: replace * with / in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_curved_window` |
| `397:37: replace <= with > in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_two_collinear_windows_uses_the_least_axis` |
| `397:54: replace && with \|\| in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_two_collinear_windows_uses_the_least_axis` |
| `397:71: replace <= with > in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_two_collinear_windows_uses_the_least_axis` |
| `399:33: replace <= with > in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_two_collinear_windows_uses_the_least_axis` |
| `407:8: delete ! in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_two_collinear_windows_uses_the_least_axis` |
| `407:26: replace && with \|\| in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_refuses_an_overflowing_offset` |
| `407:33: replace > with < in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_matches_the_hand_interval` |
| `407:33: replace > with == in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_matches_the_hand_interval` |
| `407:33: replace > with >= in fat_line_normal` | MISSED | Equivalent: len == 0 is unreachable here: the normal either passed `length > chord_len·1e-15 >= 0` or is dir × (axis of dir's smallest component), which is non-zero for a non-zero dir; a NaN len fails `is_finite` either way. |
| `410:24: replace / with % in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_matches_the_hand_interval` |
| `410:24: replace / with * in fat_line_normal` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::fat_line_normal_of_a_closed_window` |
| `436:11: replace -= with /= in clip_to_fat_line` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_matches_the_hand_interval` |
| `461:11: replace > with == in clip_to_fat_line` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `461:11: replace > with >= in clip_to_fat_line` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `491:29: replace * with / in convex_hull_clip` | MISSED | Unkillable (rounding-only): product and quotient have the same sign except when d1 == d; then the vertex (t1, d1) lies on the band and the vertex test adds t1 exactly, so the only difference is the interpolated copy t0 + (d - d0)(t1 - t0)/dd, which equals t1 up to one rounding. A test would have to pin that rounding. |
| `493:33: replace < with <= in convex_hull_clip` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::convex_hull_clip_hand_cases` |
| `493:33: replace < with == in convex_hull_clip` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::convex_hull_clip_hand_cases` |
| `513:13: replace > with == in convex_hull_clip` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_to_fat_line_touching_and_disjoint` |
| `513:13: replace > with >= in convex_hull_clip` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::convex_hull_clip_hand_cases` |
| `546:10: replace - with + in cross_2d` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hulls_and_cross_product_hand_cases` |
| `546:43: replace - with / in cross_2d` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::convex_hull_clip_hand_cases` |
| `589:33: replace <= with > in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `589:46: replace \|\| with && in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `590:33: replace <= with > in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `590:46: replace \|\| with && in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `591:15: replace && with \|\| in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `619:14: replace <= with > in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_lines_overlap_in_one_call` |
| `620:29: replace < with <= in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::straight_overlap_shortcut_needs_flatness_strictly_below_the_bound` |
| `620:29: replace < with == in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_lines_overlap_in_one_call` |
| `620:29: replace < with > in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_lines_overlap_in_one_call` |
| `621:29: replace < with <= in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::straight_overlap_shortcut_needs_flatness_strictly_below_the_bound` |
| `621:29: replace < with == in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_lines_overlap_in_one_call` |
| `621:29: replace < with > in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_lines_overlap_in_one_call` |
| `627:30: replace * with + in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `627:30: replace * with / in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_keeping_exactly_sixty_percent_is_not_a_good_clip` |
| `634:12: replace - with + in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `634:12: replace - with / in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `634:18: replace < with <= in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::clip_keeping_exactly_sixty_percent_is_not_a_good_clip` |
| `634:18: replace < with == in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `636:59: replace + with * in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `641:32: replace > with < in bezier_clip_recurse` | MISSED | Not reached: reuses the unclipped B's fat line after a one-sided clip. That slab contains the clipped B's curve, so the clip stays sound and only the work changes; pinning it needs a curved B and a hand-derived call count, not built in the box. |
| `641:32: replace > with == in bezier_clip_recurse` | MISSED | Not reached: as 641:32 `<` (sound superset slab; work-only). |
| `641:32: replace > with >= in bezier_clip_recurse` | MISSED | Equivalent: tb0 >= 0 always (clamped by max(0)), so the mutant always rebuilds the window; an unclipped window rebuilds the identical SubSegment (b.narrowed(0, 1) is b's own window). |
| `641:38: replace \|\| with && in bezier_clip_recurse` | MISSED | Not reached: as 641:32 `<` (sound superset slab after a one-sided clip; work-only). |
| `641:45: replace < with <= in bezier_clip_recurse` | MISSED | Equivalent: tb1 <= 1 always (clamped by min(1)), so the mutant always rebuilds the window; an unclipped window rebuilds the identical SubSegment. |
| `641:45: replace < with == in bezier_clip_recurse` | MISSED | Not reached: as 641:32 `<` (sound superset slab; work-only). |
| `641:45: replace < with > in bezier_clip_recurse` | MISSED | Not reached: as 641:32 `<` (sound superset slab; work-only). |
| `653:12: replace - with + in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `653:12: replace - with / in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `653:18: replace < with <= in bezier_clip_recurse` | MISSED | Not reached: the A-clip twin of 634:18 (killed by `clip_keeping_exactly_sixty_percent_is_not_a_good_clip`); a 60%-exact A clip needs B ineffective first and a curved B, whose call count was not derived in the box. |
| `653:18: replace < with == in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `654:66: replace + with * in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `654:66: replace + with - in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::curve_against_segment_takes_the_second_clip_in_three_calls` |
| `660:14: replace >= with < in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_arcs_overlap_only_past_the_depth_gate` |
| `684:24: replace >= with < in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `692:19: replace + with * in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `699:19: replace + with * in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `708:19: replace + with * in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `708:19: replace + with - in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::identical_arcs_overlap_only_past_the_depth_gate` |
| `715:19: replace + with * in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `715:19: replace + with - in bezier_clip_recurse` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_arcs_report_one_overlap_with_closed_form_ends` |
| `745:5: replace check_overlap_aligned -> bool with false` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `746:25: replace * with + in check_overlap_aligned` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `746:25: replace * with / in check_overlap_aligned` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `751:17: replace <= with > in check_overlap_aligned` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `758:17: replace <= with > in check_overlap_aligned` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `779:5: replace tolerance_contact -> Option<(f64, f64)> with None` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `779:5: replace tolerance_contact -> Option<(f64, f64)> with Some((1.0, 1.0))` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `783:37: replace / with % in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `783:37: replace / with * in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `785:21: replace > with < in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `785:21: replace > with == in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `785:21: replace > with >= in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_accepts_a_gap_of_exactly_the_tolerance` |
| `788:49: replace < with <= in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `788:49: replace < with == in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `788:49: replace < with > in tolerance_contact` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tolerance_contact_starts_at_the_closest_sample` |
| `802:5: replace shared_window -> Option<ClipSide<'s>> with None` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `804:9: replace < with <= in shared_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::shared_window_spans_the_given_parameters` |
| `804:9: replace < with == in shared_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `811:5: replace project_onto_window -> (f64, f64) with (-1.0, 1.0)` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `811:5: replace project_onto_window -> (f64, f64) with (0.0, 1.0)` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `811:5: replace project_onto_window -> (f64, f64) with (1.0, -1.0)` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `811:5: replace project_onto_window -> (f64, f64) with (1.0, 0.0)` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `811:5: replace project_onto_window -> (f64, f64) with (1.0, 1.0)` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `816:34: replace / with % in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `816:34: replace / with * in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `818:14: replace < with <= in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::project_onto_window_keeps_the_first_of_equidistant_samples` |
| `818:14: replace < with == in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `818:14: replace < with > in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `831:29: replace + with * in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `831:29: replace + with - in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::project_onto_window_matches_the_radial_foot` |
| `832:12: delete ! in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `832:29: replace && with \|\| in project_onto_window` | MISSED | Not reached: lets Newton step with a non-finite or negative curvature term; every step still has to lower the distance to be kept, so no input built here distinguishes it; not proven. |
| `832:35: replace > with < in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_brackets_the_hausdorff_bound` |
| `832:35: replace > with == in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `832:35: replace > with >= in project_onto_window` | MISSED | Not reached: differs only when c1·c1 + r·c2 is exactly 0 (an infinite step clamped to a window end that must then lower the distance); no such input was constructed. |
| `835:23: replace - with + in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_brackets_the_hausdorff_bound` |
| `835:27: replace / with % in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_brackets_the_hausdorff_bound` |
| `835:27: replace / with * in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `837:14: replace < with <= in project_onto_window` | MISSED | Unkillable (rounding-only): accepts a Newton iterate whose distance EQUALS the best so far; distinct parameters at an exactly equal distance near a foot arise only from rounding of the foot, so a test would pin that rounding. |
| `837:14: replace < with == in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_brackets_the_hausdorff_bound` |
| `837:14: replace < with > in project_onto_window` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `856:25: replace - with + in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `856:25: replace - with / in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `857:25: replace - with + in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::hits_inside_an_overlap_are_removed_and_others_kept` |
| `857:25: replace - with / in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:14: replace < with <= in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_bounds_are_strict` |
| `866:14: replace < with == in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:14: replace < with > in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:26: replace * with + in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:26: replace * with / in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:33: replace && with \|\| in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:43: replace < with <= in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_bounds_are_strict` |
| `866:43: replace < with == in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:43: replace < with > in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:55: replace * with + in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:55: replace * with / in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:63: replace && with \|\| in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `866:73: replace < with <= in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_bounds_are_strict` |
| `866:73: replace < with == in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:73: replace < with > in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:85: replace * with + in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `866:85: replace * with / in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_short_stretch_guard` |
| `874:36: replace - with + in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `874:46: replace * with + in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `874:59: replace / with % in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `874:59: replace / with * in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `880:40: replace - with + in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `889:26: replace + with * in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `889:26: replace + with - in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `889:36: replace - with + in check_overlap` | MISSED | Superseded by the defect fix: this sample-to-sample loop of `check_overlap` was replaced by sample-to-curve projection (the replacement's own mutants are tested in the after-run). |
| `902:17: replace < with <= in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_bounds_are_strict` |
| `902:17: replace < with > in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `902:29: replace * with + in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_brackets_the_hausdorff_bound` |
| `902:29: replace * with / in check_overlap` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::check_overlap_aligned_trims_to_the_shared_stretch` |
| `915:31: replace && with \|\| in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_direction_gap` |
| `915:43: replace > with >= in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_direction_gap` |
| `918:29: replace / with % in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_direction_gap` |
| `918:29: replace / with * in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_direction_gap` |
| `919:26: replace - with + in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::tangent_and_curvature_of_a_circle` |
| `920:39: replace / with % in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `920:39: replace / with * in tangent_and_curvature` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `952:47: replace * with / in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_direction_gap` |
| `953:29: replace - with + in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `953:44: replace * with / in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `953:51: replace * with / in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `953:58: replace / with % in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `953:58: replace / with * in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_curvature_gap` |
| `954:32: replace && with \|\| in coincident_to_second_order` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::coincident_to_second_order_brackets_the_direction_gap` |
| `1005:37: replace > with >= in newton_refine` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::newton_refine_keeps_the_start_on_parallel_tangents` |
| `1005:67: replace * with / in newton_refine` | MISSED | Unkillable (rounding-only): the singularity threshold eps²·j11·j22 lies far below the rounding noise of det itself (~eps·j11·j22); dividing by j11 instead moves it inside that noise, so which side a computed det lands on is rounding, not geometry. |
| `1005:73: replace * with / in newton_refine` | MISSED | Unkillable (rounding-only): as 1005:67 (threshold moved within det's own rounding noise). |
| `1017:16: replace < with <= in newton_refine` | MISSED | Unkillable (rounding-only): accepts an iterate whose gap EQUALS the best so far; an exactly equal gap at a different parameter pair arises only from rounding at the polish floor. |
| `1070:20: replace < with <= in merge_duplicate_hits` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_duplicate_hits_keeps_the_first_of_equal_gaps` |
| `1070:20: replace < with == in merge_duplicate_hits` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_duplicate_hits_keeps_the_tightest_representative` |
| `1070:20: replace < with > in merge_duplicate_hits` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_duplicate_hits_keeps_the_tightest_representative` |
| `1083:5: replace merge_overlaps with ()` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |
| `1094:28: replace <= with > in merge_overlaps` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |
| `1094:43: replace + with * in merge_overlaps` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |
| `1094:43: replace + with - in merge_overlaps` | MISSED | Killed: `nurbs::bezier_clip::mutation_oracle_tests::merge_overlaps_joins_within_the_parameter_slack` |

### `chaining.rs`

| Mutant (origin/main line:col) | Before | Verdict |
| --- | --- | --- |
| `245:12: replace > with == in chain_intersection_points` | MISSED | Equivalent after the clique fix: exp >= 1023 means threshold > 2^1022, whose square is inf on every path, and the clique path now joins exactly the pairs with finite d² (as the scan/grid comparison does); exp = 1024 under `==` reaches the inf-width check and the same clique. (Exposed the clique defect: before the fix it dropped a point.) |
| `245:12: replace > with >= in chain_intersection_points` | MISSED | Equivalent after the clique fix: exp >= 1023 means threshold > 2^1022, whose square is inf on every path, and the clique path now joins exactly the pairs with finite d² (as the scan/grid comparison does); exp = 1024 under `==` reaches the inf-width check and the same clique. (Exposed the clique defect: before the fix it dropped a point.) |
| `254:29: replace << with >> in chain_intersection_points` | MISSED | Equivalent: the subnormal branch serves cells < 2^-1022, whose square underflows to 0, so `d² < 0` joins nothing on any path and every chain is a singleton whatever the width (cell_key saturates, no panic). |
| `254:37: replace + with * in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::subnormal_threshold_connects_nothing` |
| `254:37: replace + with - in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::subnormal_threshold_connects_nothing` |
| `256:14: replace < with <= in chain_intersection_points` | MISSED | Equivalent: only a cell that is already a power of two doubles; 2·cell is still a power of two >= the threshold, so division stays exact and every within-threshold pair stays in the 27-cell neighborhood. |
| `256:14: replace < with == in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::grid_width_covers_a_threshold_just_above_a_power_of_two` |
| `256:14: replace < with > in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::grid_width_covers_a_threshold_just_above_a_power_of_two` |
| `257:15: replace *= with += in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::grid_width_covers_a_threshold_just_above_a_power_of_two` |
| `257:15: replace *= with /= in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::grid_width_covers_a_threshold_just_above_a_power_of_two` |
| `268:10: replace < with <= in chain_intersection_points` | MISSED | Equivalent: the grid path is the scan's exact acceleration (power-of-two cells hold every within-threshold pair in the 27-cell neighborhood; the ring walk returns the scan's argmin), so switching paths at another size changes no output. |
| `268:10: replace < with == in chain_intersection_points` | MISSED | Equivalent: the grid path is the scan's exact acceleration (power-of-two cells hold every within-threshold pair in the 27-cell neighborhood; the ring walk returns the scan's argmin), so switching paths at another size changes no output. |
| `268:10: replace < with > in chain_intersection_points` | MISSED | Equivalent: the grid path is the scan's exact acceleration (power-of-two cells hold every within-threshold pair in the 27-cell neighborhood; the ring walk returns the scan's argmin), so switching paths at another size changes no output. |
| `272:25: replace + with * in chain_intersection_points` | MISSED | Equivalent: the mutant only adds each point twice to its own list; self entries are already visited/used and raise every degree by 2, so components, start point and walk are unchanged. |
| `274:78: replace < with <= in chain_intersection_points` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::overflowing_threshold_keeps_overflowing_pairs_apart` |
| `303:22: replace <= with > in chain_intersection_points` | MISSED | Equivalent: the mutant only adds each point twice to its own list; self entries are already visited/used and raise every degree by 2, so components, start point and walk are unchanged. |
| `425:28: replace * with + in chain_from_adjacency` | MISSED | Equivalent: cmax_sq only enters tol_sq; cmax+cmax and cmax/cmax (1, or NaN at cmax = 0) keep tol_sq non-negative or NaN, which only delays the stop (see tol_sq rows). |
| `425:28: replace * with / in chain_from_adjacency` | MISSED | Equivalent: cmax_sq only enters tol_sq; cmax+cmax and cmax/cmax (1, or NaN at cmax = 0) keep tol_sq non-negative or NaN, which only delays the stop (see tol_sq rows). |
| `525:20: replace < with == in nearest_unused_ring` | MISSED | Equivalent: the incumbent can never be set (no distance is == / > f64::MAX, or the conjunction needs an incumbent first), so this tier finds nothing and the exact later stage (ring search, or the scan at the cap) returns the scan's argmin. |
| `525:20: replace < with > in nearest_unused_ring` | MISSED | Equivalent: the incumbent can never be set (no distance is == / > f64::MAX, or the conjunction needs an incumbent first), so this tier finds nothing and the exact later stage (ring search, or the scan at the cap) returns the scan's argmin. |
| `525:30: replace \|\| with && in nearest_unused_ring` | MISSED | Equivalent: the incumbent can never be set (no distance is == / > f64::MAX, or the conjunction needs an incumbent first), so this tier finds nothing and the exact later stage (ring search, or the scan at the cap) returns the scan's argmin. |
| `525:83: replace < with <= in nearest_unused_ring` | MISSED | Equivalent: component ranks are distinct and each member is examined once per search, so rank == best_rank only re-selects the incumbent itself. |
| `558:28: replace < with <= in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_search_breaks_ties_by_component_rank` |
| `558:28: replace < with == in nearest_unused_ring` | MISSED | Equivalent: the incumbent can never be set (no distance is == / > f64::MAX, or the conjunction needs an incumbent first), so this tier finds nothing and the exact later stage (ring search, or the scan at the cap) returns the scan's argmin. |
| `558:28: replace < with > in nearest_unused_ring` | MISSED | Equivalent: the incumbent can never be set (no distance is == / > f64::MAX, or the conjunction needs an incumbent first), so this tier finds nothing and the exact later stage (ring search, or the scan at the cap) returns the scan's argmin. |
| `559:21: replace \|\| with && in nearest_unused_ring` | MISSED | Equivalent: the incumbent can never be set (no distance is == / > f64::MAX, or the conjunction needs an incumbent first), so this tier finds nothing and the exact later stage (ring search, or the scan at the cap) returns the scan's argmin. |
| `559:74: replace < with <= in nearest_unused_ring` | MISSED | Equivalent: component ranks are distinct and each member is examined once per search, so rank == best_rank only re-selects the incumbent itself. |
| `559:74: replace < with == in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_search_breaks_ties_by_component_rank` |
| `559:74: replace < with > in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_search_breaks_ties_by_component_rank` |
| `571:38: delete - in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:43: replace * with + in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:43: replace * with / in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:54: replace + with * in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:54: replace + with - in nearest_unused_ring` | MISSED | Equivalent: tol_sq turns negative only when best_sq > cmax_sq + w² + 1e-280, and then \|tol_sq\| <= 2^-40·best_sq, far below the one-ring margin (2r-1)·w² because the incumbent lies within ring r <= 32 (sqrt(best_sq) <= (r+1)·sqrt(3)·w). |
| `571:64: replace + with * in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:64: replace + with - in nearest_unused_ring` | MISSED | Equivalent: tol_sq turns negative only when w² > cmax_sq + best_sq + 1e-280, and then \|tol_sq\| <= 2^-40·w², below the one-ring margin (2r-1)·w². |
| `571:68: replace * with + in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:68: replace * with / in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:72: replace + with * in nearest_unused_ring` | MISSED | Equivalent: tol_sq stays >= 0 (or NaN, which never stops and falls back to the scan at the cap); a non-negative tol only delays the stop, and unsearched rings lie beyond r·w > (r-1)·w. |
| `571:72: replace + with - in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_stop_floor_holds_at_tiny_scale` |
| `572:31: replace - with / in nearest_unused_ring` | MISSED | Equivalent: reach = ring·w is still an exact stop, since a ring-(r+1) cell lies more than r·w from any point of the home cell; (r-1) only adds a ring of margin. |
| `572:43: replace * with + in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_stop_compares_squared_reach_with_squared_distance` |
| `572:43: replace * with / in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_stop_floor_holds_at_tiny_scale` |
| `573:22: replace * with + in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_stop_floor_holds_at_tiny_scale` |
| `573:22: replace * with / in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_stop_compares_squared_reach_with_squared_distance` |
| `573:30: replace > with == in nearest_unused_ring` | MISSED | Equivalent: stopping only on exact equality (or at >=) never stops early; the search runs on to the cap, whose scan fallback returns the same argmin. |
| `573:30: replace > with >= in nearest_unused_ring` | MISSED | Equivalent: stopping only on exact equality (or at >=) never stops early; the search runs on to the cap, whose scan fallback returns the same argmin. |
| `573:40: replace + with - in nearest_unused_ring` | MISSED | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_stop_floor_holds_at_tiny_scale` |
| `577:17: replace >= with < in nearest_unused_ring` | MISSED | Equivalent: the full scan is the specification; falling back to it at ring 0 returns the same argmin with the same first-in-component tie-break. |
| `580:14: replace += with *= in nearest_unused_ring` | TIMEOUT | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_walk_terminates_within_a_deadline` |
| `580:14: replace += with -= in nearest_unused_ring` | TIMEOUT | Killed: `nurbs::intersection::chaining::mutation_oracle_tests::ring_walk_terminates_within_a_deadline` |

## Not reached

8 `bezier_clip.rs` mutants are distinguishable in principle but have no
closed-form oracle yet. None can hide a wrong contact: each either swaps a
fat line for a superset slab (sound) or changes a Newton safeguard whose
steps must still lower the distance to be kept.

| Mutant (origin/main) | What a kill needs |
| --- | --- |
| `641:32` `>` with `<` / `==`, `641:38` `\|\|` with `&&`, `641:45` `<` with `==` / `>` (5) | After a one-sided B clip, the mutant clips A against the unclipped B's fat line (a superset slab: sound, more work). Needs a curved B, so the two slabs differ, and a hand-derived call count. |
| `653:18` `<` with `<=` | The A-clip twin of `634:18` (killed by `clip_keeping_exactly_sixty_percent_is_not_a_good_clip`). A 60%-exact A clip needs the B clip ineffective first, so B is curved, and the subdivision that follows has no hand-derivable call count yet. |
| `832:29` `&&` with `\|\|` (project_onto_window) | Lets Newton step on a non-positive curvature term. The step is kept only if it lowers the distance; an input where such a step lands below every coarse sample was not constructed. |
| `832:35` `>` with `>=` (project_onto_window) | Differs only when `c1·c1 + r·c2` is exactly 0; no such input was constructed. |

Owner: B19. These are the remaining F3a survivors for the next mutation
tranche; they are not called equivalent.

## Verification

- `cargo test -p remus-math`: 773 passed, 0 failed (all targets).
- `cargo test -p remus-geometry`: 301 passed, 0 failed (includes
  `b10_curve_curve`, the consumer-level curve-curve matrix).
- `cargo fmt --all --check`, `cargo clippy --all-targets -- -D warnings`
  (workspace): clean.
- `./scripts/check-boundaries.sh`, `./scripts/check-det-hash.sh`,
  `./scripts/check-doc-paths.sh`, `python3 scripts/test-mutants-scope.py`:
  pass.
- Both regressions fail on `origin/main` (55 hits for the coincident arcs;
  one chain of 2 for the 3-point overflow case) and pass on the branch.
- Mutation counts above are local runs under the committed config; the
  weekly Mutation Testing run on `main` remains the whole-list proof.
