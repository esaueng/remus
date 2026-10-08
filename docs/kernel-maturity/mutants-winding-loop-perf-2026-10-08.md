# Winding-loop perf mutation pre-triage — 2026-10-08

The three `perf(algo)` commits that speed up the winding-loop pass of
`crates/algo/src/builder/fill_images_faces.rs` (fmod-free order check,
sample reuse in the meridian bracket scan, reach-box gate on the presplit
projections) put 81 `remus-algo` mutants into the weekly in-diff set
(`cargo mutants --list --in-diff`, cargo-mutants 27.0.0, diff against
`f23349a`). This note says which test is expected to kill each, and proves
the two that cannot be killed under the weekly run's default features.

A fast path that falls back to an exact general case normally leaves
equivalent mutants: narrowing the fast range only sends more inputs to the
fallback. `rem_tau_fast` avoids that by returning `None` instead of calling
the fallback itself, so its tests assert the `Some`/`None` shape at every
range boundary and any narrowing or widening fails them.

## Verification status

A local in-place run of the 81 under the committed profile was started and
aborted after 16 mutants: the shared build disk filled, 15 of those 16
were reported unviable, and the build log inspected failed with "No space
left on device". Those 15 are not evidence either way. The one mutant that ran to completion,
`replace bump_winding_cut_projection with ()`, was MISSED while the counter
compiled out under default features; it now counts in this crate's own
tests (see below). The kills in the table are read from the tests, not
measured. Three were confirmed by hand mutation:

- `offset.abs() <= SEAM_DEGENERATE_TOL` changed to `>` fails
  `winding_loops_are_cut_on_the_seam_and_two_more_meridians`, so that fixture
  reaches the on-sample fallback.
- A reach box grown by `0.5·weld` with no slack fails
  `the_reach_box_gate_never_changes_a_presplit`.
- Removing the gate takes `scaling_winding_cut_projections_skip_the_other_loop`
  from 12 projections to 24, and it fails.

## By function

Tests are in `builder/fill_images_faces/helper_oracle_tests.rs` unless noted.

| Function | Mutants | Expected | Killing test |
| --- | ---: | --- | --- |
| `rem_tau_fast` | 18 | caught | `rem_tau_fast_takes_exactly_its_three_ranges_and_matches_rem_euclid`: `Some`/`None` and bits at `-TAU`, `next_up(-TAU)`, `-0.0`, `0`, `next_down(2·TAU)`, `2·TAU` and the specials |
| `wrap_pi_exact` | 7 | caught | `wrap_pi_exact_matches_the_rem_euclid_wrap_at_every_branch_point` |
| `nearest_v` | 11 | caught | `nearest_v_keeps_the_first_of_equally_near_samples` (exact ties kill `>` → `>=`) |
| `loops_strictly_ordered` | 12 | caught | `loops_strictly_ordered_needs_more_than_gap_in_both_directions` (a pair at exactly `gap` in one scan direction at a time) |
| `compute_winding_loop_cuts` | 8 | caught | `winding_loops_are_cut_on_the_seam_and_two_more_meridians` (the seam is not at `u = 0`, so `u - target` → `u + target` moves every cut) |
| `presplit_closed_winding_loops` | 3 | caught or unviable | `far_cuts_are_skipped_and_the_split_is_unchanged` (deleting the gate's `!` drops the loops' own cuts) |
| `weld_reach_box` | 19 | caught or unviable | `weld_reach_box_is_the_control_box_grown_by_two_welds_and_rounding_slack` (box compared bit for bit), `weld_reach_box_refuses_weight_ratios_past_1e100` |
| `bump_winding_cut_projection` | 1 | caught | `scaling_winding_cut_projections_stay_linear_in_the_loop_count` (the count stays 0) |
| `take_winding_cut_projections` | 2 | caught | same test (a constant `0` or `1` misses `3·L`); added after the list above was taken |
| `perf.rs` `reset`, `snapshot` | 2 | (b) equivalent | none under default features |

## (b) Equivalent under default features

The weekly run builds without `perf-counters`, so these items compile out.
The same holds for the feature-only counters elsewhere in
`crates/algo/src/perf.rs`. The winding-cut counter is not one of them: like
`RAY_WORK`, it also counts under `cfg(test)`, so the algo scaling test runs
in the default workspace job and kills `bump_winding_cut_projection`.

| Mutant | Proof |
| --- | --- |
| `perf.rs`: replace `reset` with `()` | `reset` is `#[cfg(feature = "perf-counters")]`; the mutated item is not compiled. |
| `perf.rs`: replace `snapshot -> PerfSnapshot` with `Default::default()` | Same: `snapshot` and `PerfSnapshot` are compiled out, so nothing type-checks the replacement. |

With `perf-counters` on, the mutated `snapshot` does not compile
(`PerfSnapshot` has no `Default`). `scaling_winding_cut_projections_skip_the_other_loop`
(`crates/operations/src/boolean/tests.rs`, run by CI's `scaling_` step)
also catches `bump_winding_cut_projection` → `()` there.
