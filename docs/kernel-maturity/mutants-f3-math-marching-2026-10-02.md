# F3b: `surface_marching.rs` mutation survivors — 2026-10-02

Owner row: [B19](roadmap.md#b19). Input: the 168 `surface_marching.rs`
mutants that M2 re-ran on current code and that survived (MISSED), listed
under "Needs review (F3)" in
[mutants-proof-2026-10-01](mutants-proof-2026-10-01.md) (M2's re-run list
of 557, function filter over the CI run 36302856971 at `763ad346`). M2's
doc is not edited here; this doc carries the outcomes for this file only.
The `bezier_clip.rs`, `chaining.rs` and pave-filler survivors belong to the
sibling F3 slices.

**Outcome:** of the 168, 97 are killed by closed-form oracles on
the current code, 11 more were killed on the old code inside the
`null_directions` arm that the defect fix replaced, 49 are
equivalent with a one-line proof each, 1 was a real defect (fixed), 6 sat in
code the fix deleted (each with its old-code verdict), and 4 remain
open (not killed, not called equivalent). A second defect, an exact-crossing
L-turn in the marcher, was found while building the fixtures and is fixed.

## Defects found and fixed

| Defect | Evidence | Fix | Regression |
| --- | --- | --- | --- |
| `null_directions` divided by the *smaller* diagonal, so for `Q = [[1, 1], [1, 1e-20]]` the root `-b + √disc` cancelled to 0 and it returned `e1`, where `Q(e1, e1) = 1`; for `[[1, 1], [1, 1e-16]]` it returned a direction with `Q(t, t) = -0.55`. Masked downstream by Newton refinement of the branch seeds, but the function's contract (every returned direction is a null) was broken | Surviving mutant `321:20` (`>=` → `<`, i.e. divide by the larger diagonal) is the stable choice; direct evaluation printed above values | Both nulls are now `(q, a)` and `(d, q)` with `q = -(b + sign(b)·√disc)`: no cancellation, no division, no zero-diagonal special case (net −51 lines) | `oracle_tests::ill_conditioned_indefinite_forms_keep_true_nulls` (fails on the old code, passes now); orientation and order pinned by `indefinite_nulls_are_sorted_and_oriented_deterministically` |
| A march step landing exactly on a crossing found `n1 × n2 = 0` there and overwrote its previous tangent with `None`, so the next singular step took the first sorted null and turned onto the other branch (an L-turn) | `z = 0.2·x·y` against `z = 0`, seeded at `(0, -0.05)`, step `0.05`: the backward trace went `(0, 0) → (0.1, 0) → … → (0.998, 0)` | A point with no first-order tangent keeps the incoming one (`march_direction`) | `oracle_tests::march_landing_exactly_on_a_crossing_keeps_its_branch` (fails on the old code: trace left `x = 0`; passes now, and its branch seeds lie on `y = 0` on both sides) |

## Method

- Tool: `cargo-mutants 27.0.0`, the committed `.cargo/mutants.toml`
  (profile `ci-test`, nextest, `--tests`, first-failure stop, per-package
  oracle). Scope checked with `scripts/test-mutants-scope.py` (CDT scope,
  long-tail exclusions, verdict/planner tests pass). `CARGO_BUILD_JOBS=4`,
  `-j 2`.
- **Stage A (verdicts for the 168 on the code M2 measured):** `origin/main`
  production code (`b98cf32f`; `surface_marching.rs` identical to M2's
  re-run, same line numbers) plus the new oracle module, with the two defect
  regressions `#[ignore]`d (they fail on that code, which would fail the
  baseline). `cargo mutants -p remus-math -j 2 -F <each of the 168 exact
  names>`: **168 tested in 22 min: 102 caught, 66 missed.** Kill
  attribution is the first `FAIL` line in each mutant's nextest log.
- **Stage B (fixed code):** every mutant of `null_directions` (rewritten),
  `march_direction` (changed) and `march_direction_with_branches`, with the
  full oracle module: **88 tested in 13 min: 57 caught, 18 missed, 13 unviable.** Stage-A survivors in
  `march_direction_with_branches` and `null_directions` that later tests
  target are matched to their shifted Stage B line (`line − 54` after the
  `null_directions` rewrite) and counted killed only when Stage B caught them.
- Two tests were tightened after Stage A and their kills were first checked
  by single-mutant manual runs (mutation applied, filtered test run,
  `git checkout`), then confirmed in Stage B: `312:26` (the overflowing
  definite form needed a representable eigenvector) and the three `716 *→+`
  fallback-tangent mutants (`one_point_trace_scans_with_the_normal_cross_fallback`).
- Equivalence is claimed only with a one-line proof (dead branch, identical
  arms, bitwise-equal arithmetic, or an exact later gate that refuses every
  input the mutant admits). No assertion was weakened; no existing test was
  changed.

## New tests

All in `crates/math/src/nurbs/intersection/surface_marching/oracle_tests.rs`
(new file, a `#[cfg(test)]` child of `surface_marching.rs` so it reaches the
private functions).

| Test | What it pins | Oracle |
| --- | --- | --- |
| `saddle_difference_is_the_closed_form_form` | `orthonormal_difference` on `z = c·x·y`, operand swap, flipped normal | `Q = [[0, c], [c, 0]]`, `det = -c²`, maps `½·I`; swap and the `s = sign(n1·n2) = -1` case both give `b = -c` |
| `generic_difference_is_the_hessian_difference_on_skewed_charts` | every term of `C = I⁻¹B`, `Q = CᵀIIC`, `Q1 - s·Q2` | Both surfaces curved on skewed charts: `Q = H1 - H2 = [[2, 5/2], [5/2, -3]]`, `C_k = A_k⁻¹` |
| `frame_projects_the_smallest_normal_component` | the deterministic frame | `n = z → (x, y)`, `n = x → (y, z)`, `n ∝ (2,3,1) → e1 = (-2,-3,13)/√182, e2 = (3,-2,0)/√13` |
| `shared_plane_gate_is_strict_at_cos_0_99` | `|n1·n2| < 0.99` gate | perpendicular planes refused; a tilt with `n·z == 0.99` exactly kept |
| `anisotropic_chart_is_refused_by_the_metric_guard` | the per-surface `det(I) <= 1e-24·(E+G)²` guard and its scale | partials `2^10`, `2^-10`, sine `2^-22`: passes the sine gate, fails only the metric guard, both orders |
| `near_collinear_chart_is_refused_by_the_sine_gate` | the `sin² <= 1e-24` gate on either side | `sin² ≈ 1e-26` whose rounded `det(I)` is positive above the metric guard |
| `tiny_partials_from_a_wide_knot_domain_keep_the_form` | scale-invariant sine | knot domain `2^43` (partials `2^-42`): `Q` unchanged, maps `2^42·I` |
| `indefinite_forms_return_their_two_null_lines`, `indefinite_classification_is_scale_invariant`, `definite_zero_and_non_finite_forms_have_no_null`, `parabolic_forms_return_their_single_ruling`, `classification_gates_are_strict`, `indefinite_nulls_are_sorted_and_oriented_deterministically`, `ill_conditioned_indefinite_forms_keep_true_nulls` | `null_directions` | every returned direction is unit, in the frame plane, and `|Q(t,t)| <= 1e-12·‖Q‖`; exact counts (2 / 1 / 0); strict gates at `det = ±1e-12·‖Q‖²`; `λ·Q` for `λ = 1e-6…1e6`; the sorted, oriented output |
| `transverse_seeds_sit_at_the_normalised_offset_along_the_null` | `find_branch_directions` seed placement | seeds at `±(0.5·step/peak)·B` lifted onto `s1`, `B = (3,-1)/√10`, `peak` from `A_k⁻¹B`; four chart variants make each parameter component dominant |
| `bisector_arrival_seeds_both_nulls_on_both_sides` | the tie arm and the transverse choice | `(1,1)/√2` arrival seeds `(±step, 0)` and `(0, ±step)` |
| `seed_distance_and_merge_boundaries_are_strict` | `dist < tol` and `< 100·tol` | `tol` set to the measured offset and to separation/100 exactly |
| `nearly_flat_crossing_still_seeds_its_transverse_branch` | relative `det` gate | curvature `1e-7` (`det = -1e-14`) still branches |
| `no_cosine_lands_exactly_on_the_branch_angle` | equivalence proof for `565:41`, executable | no f64 in ±4000 ulps of `cos 30°` has `acos == 30°.to_radians()` |
| `singular_tangent_takes_the_most_aligned_oriented_null`, `singular_parameter_tangent_follows_the_incoming_null` | the oriented-null choice at `n1 × n2 = 0` | tie keeps the first maximum; `-A` arrival returns `-A`; parameter tangents `(0,∓1,0,∓1)` / `(1,0,1,0)`; quarter-turned copies make the `z` components live |
| `march_through_a_nearly_flat_crossing_reports_its_branch`, `march_seeded_at_the_crossing_scans_its_first_point`, `third_order_contact_line_is_marched_by_the_perturbation_search`, `one_point_trace_scans_with_the_normal_cross_fallback`, `march_landing_exactly_on_a_crossing_keeps_its_branch` | march-level scan and fallbacks | branch seeds on both surfaces and on the transverse line; `z = x³/27` contact (`Q = 0`) marched only by the perturbation search; a one-point trace's `±(n1 × n2)` fallback; the exact-crossing continuation |

## Before / after per function

Before: all 168 ran and survived in M2's re-run. After (columns: killed on
current code; killed on old code in the replaced arm; equivalent; defect;
superseded by the fix; open):

| Function | F3 in | Killed | Killed (old code, replaced) | Equivalent | Defect | Superseded | Open | Unresolved |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `orthonormal_difference` | 51 | 36 | 0 | 15 | 0 | 0 | 0 | 0 |
| `null_directions` | 41 | 10 | 11 | 13 | 1 | 6 | 0 | 0 |
| `find_branch_directions` | 37 | 29 | 0 | 8 | 0 | 0 | 0 | 0 |
| `march_direction_with_branches` | 25 | 12 | 0 | 9 | 0 | 0 | 4 | 0 |
| `ssi_tangent_params` | 6 | 3 | 0 | 3 | 0 | 0 | 0 | 0 |
| `singular_tangent_direction` | 6 | 6 | 0 | 0 | 0 | 0 | 0 | 0 |
| `SsiScratch::solve2` | 1 | 0 | 0 | 1 | 0 | 0 | 0 | 0 |
| `perturbation_tangent` | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |
| **Total** | **168** | **97** | **11** | **49** | **1** | **6** | **4** | **0** |

## Per-mutant verdicts

Line:col are the M2 re-run's (= `origin/main` before this PR). "killed" names
the first failing test.

| Mutant | Function | Mutation | Verdict | Evidence / proof |
| --- | --- | --- | --- | --- |
| `173:17` | `orthonormal_difference` | replace < with > | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `173:21` | `orthonormal_difference` | replace \|\| with && | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `173:36` | `orthonormal_difference` | replace < with > | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `173:40` | `orthonormal_difference` | replace \|\| with && | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `173:52` | `orthonormal_difference` | replace < with > | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `173:56` | `orthonormal_difference` | replace \|\| with && | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `173:71` | `orthonormal_difference` | replace < with > | equivalent | `derivative_table_from` resizes the table to exactly 3×3 before the slice-only fill, so every `len() < 3` term is false and so is any `>`/`&&` recombination |
| `190:32` | `orthonormal_difference` | replace > with >= | equivalent | differs only when a squared partial length is 0; the quotient is then x/0 or x/NaN, non-finite, and `!is_finite()` refuses exactly as the original `sin² = 0 <= 1e-24` does |
| `190:38` | `orthonormal_difference` | replace && with \|\| | equivalent | differs only when a squared partial length is 0; the quotient is then x/0 or x/NaN, non-finite, and `!is_finite()` refuses exactly as the original `sin² = 0 <= 1e-24` does |
| `190:51` | `orthonormal_difference` | replace > with >= | equivalent | differs only when a squared partial length is 0; the quotient is then x/0 or x/NaN, non-finite, and `!is_finite()` refuses exactly as the original `sin² = 0 <= 1e-24` does |
| `191:18` | `orthonormal_difference` | replace / with * | killed | `tiny_partials_from_a_wide_knot_domain_keep_the_form` |
| `191:31` | `orthonormal_difference` | replace * with + | killed | `tiny_partials_from_a_wide_knot_domain_keep_the_form` |
| `191:31` | `orthonormal_difference` | replace * with / | killed | `tiny_partials_from_a_wide_knot_domain_keep_the_form` |
| `195:32` | `orthonormal_difference` | replace > with >= | equivalent | differs only when a squared partial length is 0; the quotient is then x/0 or x/NaN, non-finite, and `!is_finite()` refuses exactly as the original `sin² = 0 <= 1e-24` does |
| `195:38` | `orthonormal_difference` | replace && with \|\| | equivalent | differs only when a squared partial length is 0; the quotient is then x/0 or x/NaN, non-finite, and `!is_finite()` refuses exactly as the original `sin² = 0 <= 1e-24` does |
| `195:51` | `orthonormal_difference` | replace > with >= | equivalent | differs only when a squared partial length is 0; the quotient is then x/0 or x/NaN, non-finite, and `!is_finite()` refuses exactly as the original `sin² = 0 <= 1e-24` does |
| `196:18` | `orthonormal_difference` | replace / with * | killed | `tiny_partials_from_a_wide_knot_domain_keep_the_form` |
| `196:31` | `orthonormal_difference` | replace * with + | killed | `tiny_partials_from_a_wide_knot_domain_keep_the_form` |
| `196:31` | `orthonormal_difference` | replace * with / | killed | `tiny_partials_from_a_wide_knot_domain_keep_the_form` |
| `202:29` | `orthonormal_difference` | replace \|\| with && | equivalent | differs only when exactly one sin² is non-finite, which needs fl(E·G) ∈ {0, ∞, NaN}; that surface's `det_i = E·G − F²` is then ≤ 0 or non-finite, so its `quadric_form` returns None and the function still refuses |
| `202:53` | `orthonormal_difference` | replace \|\| with && | killed | `near_collinear_chart_is_refused_by_the_sine_gate` |
| `202:73` | `orthonormal_difference` | replace \|\| with && | killed | `near_collinear_chart_is_refused_by_the_sine_gate` |
| `209:18` | `orthonormal_difference` | replace < with <= | killed | `shared_plane_gate_is_strict_at_cos_0_99` |
| `209:18` | `orthonormal_difference` | replace < with == | killed | `shared_plane_gate_is_strict_at_cos_0_99` |
| `213:20` | `orthonormal_difference` | replace >= with < | killed | `saddle_difference_is_the_closed_form_form` |
| `213:42` | `orthonormal_difference` | delete - | killed | `saddle_difference_is_the_closed_form_form` |
| `221:27` | `orthonormal_difference` | replace <= with > | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `221:33` | `orthonormal_difference` | replace && with \|\| | killed | `frame_projects_the_smallest_normal_component` |
| `221:39` | `orthonormal_difference` | replace <= with > | killed | `frame_projects_the_smallest_normal_component` |
| `223:18` | `orthonormal_difference` | replace <= with > | killed | `frame_projects_the_smallest_normal_component` |
| `228:25` | `orthonormal_difference` | replace - with + | killed | `frame_projects_the_smallest_normal_component` |
| `242:27` | `orthonormal_difference` | replace - with + | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `242:31` | `orthonormal_difference` | replace * with + | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `245:24` | `orthonormal_difference` | replace + with * | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `245:24` | `orthonormal_difference` | replace + with - | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `245:29` | `orthonormal_difference` | replace * with + | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `245:29` | `orthonormal_difference` | replace * with / | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `245:34` | `orthonormal_difference` | replace + with * | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `245:34` | `orthonormal_difference` | replace + with - | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `246:31` | `orthonormal_difference` | replace \|\| with && | equivalent | a non-finite `det_i` forces a non-finite `scale` ((E+G)² ≥ 4EG), and an infinite `scale` makes the guard 1e-24·∞ = ∞, which every finite `det_i` meets, so the chart is refused either way |
| `246:53` | `orthonormal_difference` | replace \|\| with && | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `246:71` | `orthonormal_difference` | replace * with / | killed | `anisotropic_chart_is_refused_by_the_metric_guard` |
| `259:21` | `orthonormal_difference` | delete - | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `261:31` | `orthonormal_difference` | replace + with - | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `262:31` | `orthonormal_difference` | replace + with - | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `269:27` | `orthonormal_difference` | replace + with - | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `282:19` | `orthonormal_difference` | replace - with + | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `283:19` | `orthonormal_difference` | replace - with + | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `284:19` | `orthonormal_difference` | replace - with + | killed | `singular_tangent_takes_the_most_aligned_oriented_null` |
| `285:29` | `orthonormal_difference` | replace * with + | killed | `frame_projects_the_smallest_normal_component` |
| `285:37` | `orthonormal_difference` | replace + with - | killed | `generic_difference_is_the_hessian_difference_on_skewed_charts` |
| `312:26` | `null_directions` | replace \|\| with && | killed (by a test added after Stage A; confirmed in Stage B) | `definite_zero_and_non_finite_forms_have_no_null` (Stage B `312:26`) |
| `312:46` | `null_directions` | replace \|\| with && | equivalent | every caller passes ‖Q‖ = √(a²+2b²+d²) and det = ad − b²: a non-finite det forces a non-finite norm (|ad|, b² ≤ ‖Q‖²), and norm = 0 forces Q = 0, which the parabolic arm's own `norm <= 0` re-check refuses |
| `315:28` | `null_directions` | replace * with / | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `315:35` | `null_directions` | replace * with / | killed | `indefinite_classification_is_scale_invariant` |
| `316:12` | `null_directions` | replace < with <= | killed (old code); line replaced by the fix | `classification_gates_are_strict` |
| `321:20` | `null_directions` | replace >= with < | defect → fixed | this branch flip is the stable choice (divide by the larger diagonal); the old solve lost `-b + √disc` to cancellation, e.g. returned e1 with Q(e1,e1) = 1 for [[1,1],[1,1e-20]]; fixed by the cancellation-free pair, regression `ill_conditioned_indefinite_forms_keep_true_nulls`; old code killed by `singular_tangent_takes_the_most_aligned_oriented_null` (old code: killed by `singular_tangent_takes_the_most_aligned_oriented_null`) |
| `330:50` | `null_directions` | replace > with >= | superseded (code removed by the fix) | on the old code equivalent: differs only at |d| = 1e-300 exactly, where the d≈0 sub-case rebuilds `dirs` when |b| > 1e-300, and with |b| ≤ 1e-300 det cannot pass the indefinite gate (|ad| < 1e-12·a² needs |a| < 1e-288, where ad underflows) |
| `348:45` | `null_directions` | replace > with >= | superseded (code removed by the fix) | on the old code equivalent: |b| = 1e-300 exactly makes b² underflow to 0, and |ad| ≤ 1e-300·|a| cannot pass the indefinite gate (as 330:50) |
| `350:43` | `null_directions` | replace + with - | superseded (code removed by the fix) | orientation-only (−e2 for e2); the replacement's orientation and order are pinned by `indefinite_nulls_are_sorted_and_oriented_deterministically` |
| `356:40` | `null_directions` | delete - | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `356:45` | `null_directions` | replace * with + | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `356:45` | `null_directions` | replace * with / | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `356:50` | `null_directions` | replace + with - | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `367:26` | `null_directions` | delete - | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `367:29` | `null_directions` | replace + with * | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `369:50` | `null_directions` | replace > with >= | superseded (code removed by the fix) | mirror of 330:50 with a and d exchanged |
| `381:45` | `null_directions` | replace > with >= | superseded (code removed by the fix) | mirror of 348:45 |
| `383:43` | `null_directions` | replace + with - | superseded (code removed by the fix) | on the old code equivalent: e1's zero components are always +0 and +0 ± (±0) = +0, so e1·1 ± e2·0 are bitwise equal |
| `386:41` | `null_directions` | replace + with - | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `386:49` | `null_directions` | delete - | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `386:54` | `null_directions` | replace * with + | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `386:54` | `null_directions` | replace * with / | killed (old code); line replaced by the fix | `indefinite_forms_return_their_two_null_lines` |
| `400:34` | `null_directions` | replace - with + | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `400:49` | `null_directions` | replace < with <= | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `400:49` | `null_directions` | replace < with == | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `400:57` | `null_directions` | replace \|\| with && | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `400:64` | `null_directions` | replace + with - | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `400:79` | `null_directions` | replace < with <= | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `400:79` | `null_directions` | replace < with == | equivalent | the dedup is a no-op: det < −1e-12·‖Q‖² puts the two null lines at least ~2e-6 rad apart, so neither |p − q| nor |p + q| of the unit nulls can reach 1e-12, under any of these predicate variants |
| `402:19` | `null_directions` | replace > with >= | killed | `classification_gates_are_strict` |
| `410:23` | `null_directions` | replace + with * | equivalent | `trace` is bound only to be discarded (`let _ = trace`) |
| `410:23` | `null_directions` | replace + with - | equivalent | `trace` is bound only to be discarded (`let _ = trace`) |
| `412:40` | `null_directions` | replace >= with < | equivalent | both arms of the |b|-dominance test evaluate the identical `if |a| >= |d| {(−b, a)} else {(d, −b)}`, so the condition is unobservable |
| `412:51` | `null_directions` | replace && with \|\| | equivalent | both arms of the |b|-dominance test evaluate the identical `if |a| >= |d| {(−b, a)} else {(d, −b)}`, so the condition is unobservable |
| `412:62` | `null_directions` | replace >= with < | equivalent | both arms of the |b|-dominance test evaluate the identical `if |a| >= |d| {(−b, a)} else {(d, −b)}`, so the condition is unobservable |
| `415:24` | `null_directions` | replace >= with < | killed | `parabolic_forms_return_their_single_ruling` |
| `415:38` | `null_directions` | delete - | killed | `parabolic_forms_return_their_single_ruling` |
| `415:58` | `null_directions` | delete - | killed | `parabolic_forms_return_their_single_ruling` |
| `417:14` | `null_directions` | delete - | killed | `parabolic_forms_return_their_single_ruling` |
| `419:17` | `null_directions` | delete - | killed | `parabolic_forms_return_their_single_ruling` |
| `421:18` | `null_directions` | replace == with != | killed | `parabolic_forms_return_their_single_ruling` |
| `500:9` | `find_branch_directions` | replace \|\| with && | equivalent | `orthonormal_difference` returns None before any non-finite norm, and det = ad − b² is finite whenever the norm is, so `(A && B) || C` is `C` |
| `501:24` | `find_branch_directions` | delete - | equivalent | the mutant returns early only for det ≥ +1e-12·‖Q‖², a subset of the original condition; on the rest `null_directions` applies the original gate and the `nulls.len() < 2` exit returns the same empty Vec |
| `501:36` | `find_branch_directions` | replace * with + | killed | `nearly_flat_crossing_still_seeds_its_transverse_branch` |
| `501:36` | `find_branch_directions` | replace * with / | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `501:48` | `find_branch_directions` | replace * with + | equivalent | threshold becomes ‖Q‖(1 − 1e-12) ≥ 0 > −1e-12·‖Q‖², a stricter early return; the rest is refused by `null_directions` and the `< 2` exit as for 501:24 |
| `501:48` | `find_branch_directions` | replace * with / | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `501:9` | `find_branch_directions` | replace \|\| with && | equivalent | the norm/det finiteness terms are always false (see 500:9), and wherever the mutant stops returning early det ≥ −1e-12·‖Q‖² still holds, so `null_directions` (same gate) returns fewer than two nulls and the `nulls.len() < 2` exit returns the same empty Vec |
| `508:20` | `find_branch_directions` | replace < with > | equivalent | after the indefinite gate `null_directions` always returns exactly two nulls (disc = −det > 0, both candidates finite and non-zero, dedup a no-op), so neither `len() < 2` nor `len() > 2` holds |
| `515:56` | `find_branch_directions` | replace - with + | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `515:56` | `find_branch_directions` | replace - with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `515:70` | `find_branch_directions` | replace <= with > | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `522:20` | `find_branch_directions` | replace < with <= | equivalent | `dot0 == dot1` gives |dot0 − dot1| = 0 ≤ 1e-12, which the tie arm takes first |
| `537:33` | `find_branch_directions` | replace + with * | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `537:33` | `find_branch_directions` | replace + with - | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `537:41` | `find_branch_directions` | replace * with / | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `538:33` | `find_branch_directions` | replace + with * | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `538:33` | `find_branch_directions` | replace + with - | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `538:41` | `find_branch_directions` | replace * with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `539:25` | `find_branch_directions` | replace * with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `539:33` | `find_branch_directions` | replace + with * | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `539:33` | `find_branch_directions` | replace + with - | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `539:41` | `find_branch_directions` | replace * with / | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `540:33` | `find_branch_directions` | replace + with * | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `540:33` | `find_branch_directions` | replace + with - | killed | `transverse_seeds_sit_at_the_normalised_offset_along_the_null` |
| `540:41` | `find_branch_directions` | replace * with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `542:30` | `find_branch_directions` | replace \|\| with && | equivalent | `C_k = I_k⁻¹B_k` is finite (det_i passed its guard) and non-singular (B_k is invertible since |n1·n2| ≥ 0.99), and (alpha, beta) is a unit vector, so `peak` is finite and positive and neither form of the guard ever fires |
| `545:31` | `find_branch_directions` | replace * with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `549:26` | `find_branch_directions` | replace + with - | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `549:33` | `find_branch_directions` | replace * with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `551:26` | `find_branch_directions` | replace + with - | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `551:33` | `find_branch_directions` | replace * with / | killed | `bisector_arrival_seeds_both_nulls_on_both_sides` |
| `560:25` | `find_branch_directions` | replace < with <= | killed | `seed_distance_and_merge_boundaries_are_strict` |
| `560:25` | `find_branch_directions` | replace < with == | killed | `seed_distance_and_merge_boundaries_are_strict` |
| `565:41` | `find_branch_directions` | replace > with >= | equivalent | no f64 has `acos` exactly equal to `30f64.to_radians()` (checked over the ±4000-ulp neighbourhood of cos 30°, outside which monotone acos cannot land on it: `no_cosine_lands_exactly_on_the_branch_angle`), so `>` and `>=` agree |
| `568:64` | `find_branch_directions` | replace < with <= | killed | `seed_distance_and_merge_boundaries_are_strict` |
| `568:64` | `find_branch_directions` | replace < with == | killed | `seed_distance_and_merge_boundaries_are_strict` |
| `568:76` | `find_branch_directions` | replace * with / | killed | `seed_distance_and_merge_boundaries_are_strict` |
| `653:88` | `march_direction_with_branches` | replace < with <= | open (not killed, not equivalent) | needs two adjacent traced points with bitwise-equal cross magnitudes below the band (a tied valley); the original then seeds from both, the mutant from neither. No march fixture producing an exact tie was found in the box |
| `655:87` | `march_direction_with_branches` | replace < with <= | open (not killed, not equivalent) | as 653:88, for the following neighbour |
| `683:17` | `march_direction_with_branches` | replace && with \|\| | equivalent | `has_branch` only pre-filters: `find_branch_directions` re-applies the original finite/indefinite gate to the same `OrthoDiff` and returns no seeds otherwise, and this mutant only admits more candidates (or none fewer) |
| `684:17` | `march_direction_with_branches` | replace && with \|\| | equivalent | `has_branch` only pre-filters: `find_branch_directions` re-applies the original finite/indefinite gate to the same `OrthoDiff` and returns no seeds otherwise, and this mutant only admits more candidates (or none fewer) |
| `684:29` | `march_direction_with_branches` | replace < with <= | equivalent | `has_branch` only pre-filters: `find_branch_directions` re-applies the original finite/indefinite gate to the same `OrthoDiff` and returns no seeds otherwise, and this mutant only admits more candidates (or none fewer) |
| `684:31` | `march_direction_with_branches` | delete - | equivalent | `has_branch` only pre-filters: `find_branch_directions` re-applies the original finite/indefinite gate to the same `OrthoDiff` and returns no seeds otherwise, and this mutant only admits more candidates (or none fewer) |
| `684:43` | `march_direction_with_branches` | replace * with + | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `684:43` | `march_direction_with_branches` | replace * with / | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `684:55` | `march_direction_with_branches` | replace * with + | equivalent | `has_branch` only pre-filters: `find_branch_directions` re-applies the original finite/indefinite gate to the same `OrthoDiff` and returns no seeds otherwise, and this mutant only admits more candidates (or none fewer) |
| `684:55` | `march_direction_with_branches` | replace * with / | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `695:46` | `march_direction_with_branches` | delete - | equivalent | `sign` reaches `find_branch_directions` only through `current_tangent`, which enters only `|dot|` products, so its orientation is unobservable |
| `696:44` | `march_direction_with_branches` | replace >= with < | killed (by a test added after Stage A; confirmed in Stage B) | `march_landing_exactly_on_a_crossing_keeps_its_branch` (Stage B `642:44`) |
| `697:31` | `march_direction_with_branches` | replace > with < | open (not killed, not equivalent) | with `idx < 0` the scan uses the forward difference `next - pt` instead of the central `next - prev` for `idx > 0`; on straight branches the two are identical and on curved ones they differ by O(h·κ), observable only at the 30° acceptance gate; a curved-branch fixture at that gate was not built in the box |
| `697:31` | `march_direction_with_branches` | replace > with == | killed | `march_seeded_at_the_crossing_scans_its_first_point` |
| `697:31` | `march_direction_with_branches` | replace > with >= | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `698:28` | `march_direction_with_branches` | replace - with / | open (not killed, not equivalent) | as 697:31 (`idx / 1` makes `prev = pt`) |
| `702:35` | `march_direction_with_branches` | replace < with > | killed (by a test added after Stage A; confirmed in Stage B) | `march_landing_exactly_on_a_crossing_keeps_its_branch` (Stage B `648:35`) |
| `703:28` | `march_direction_with_branches` | replace + with * | killed (by a test added after Stage A; confirmed in Stage B) | `march_landing_exactly_on_a_crossing_keeps_its_branch` (Stage B `649:28`) |
| `703:28` | `march_direction_with_branches` | replace + with - | killed | `march_through_a_nearly_flat_crossing_reports_its_branch` |
| `716:46` | `march_direction_with_branches` | replace * with + | killed (by a test added after Stage A; confirmed in Stage B) | `one_point_trace_scans_with_the_normal_cross_fallback` (Stage B `662:46`) |
| `716:46` | `march_direction_with_branches` | replace * with / | equivalent | `x / ±1` and `x * ±1` are bitwise equal in IEEE arithmetic |
| `716:60` | `march_direction_with_branches` | replace * with + | killed (by a test added after Stage A; confirmed in Stage B) | `one_point_trace_scans_with_the_normal_cross_fallback` (Stage B `662:60`) |
| `716:60` | `march_direction_with_branches` | replace * with / | equivalent | `x / ±1` and `x * ±1` are bitwise equal in IEEE arithmetic |
| `716:74` | `march_direction_with_branches` | replace * with + | killed (by a test added after Stage A; confirmed in Stage B) | `one_point_trace_scans_with_the_normal_cross_fallback` (Stage B `662:74`) |
| `716:74` | `march_direction_with_branches` | replace * with / | equivalent | `x / ±1` and `x * ±1` are bitwise equal in IEEE arithmetic |
| `837:53` | `ssi_tangent_params` | replace * with + | killed | `singular_parameter_tangent_follows_the_incoming_null` |
| `837:53` | `ssi_tangent_params` | replace * with / | equivalent | `x / ±1` and `x * ±1` are bitwise equal in IEEE arithmetic |
| `837:67` | `ssi_tangent_params` | replace * with + | killed | `singular_parameter_tangent_follows_the_incoming_null` |
| `837:67` | `ssi_tangent_params` | replace * with / | equivalent | `x / ±1` and `x * ±1` are bitwise equal in IEEE arithmetic |
| `837:81` | `ssi_tangent_params` | replace * with + | killed | `singular_parameter_tangent_follows_the_incoming_null` |
| `837:81` | `ssi_tangent_params` | replace * with / | equivalent | `x / ±1` and `x * ±1` are bitwise equal in IEEE arithmetic |
| `86:9` | `SsiScratch::solve2` | replace SsiScratch::solve2 -> &mut DerivativeScratch with Box::leak(Box::new(Default::default())) | equivalent | a fresh `DerivativeScratch` gives bit-identical derivatives (`NurbsSurface::derivatives` itself uses one; buffers are written before read and span hints are verified before use), so `surface_newton_step` takes the same step |
| `902:48` | `singular_tangent_direction` | delete - | killed | `singular_tangent_takes_the_most_aligned_oriented_null` |
| `902:56` | `singular_tangent_direction` | delete - | killed | `singular_tangent_takes_the_most_aligned_oriented_null` |
| `902:64` | `singular_tangent_direction` | delete - | killed | `singular_tangent_takes_the_most_aligned_oriented_null` |
| `904:30` | `singular_tangent_direction` | replace > with < | killed | `singular_parameter_tangent_follows_the_incoming_null` |
| `904:30` | `singular_tangent_direction` | replace > with == | killed | `singular_parameter_tangent_follows_the_incoming_null` |
| `904:30` | `singular_tangent_direction` | replace > with >= | killed | `singular_tangent_takes_the_most_aligned_oriented_null` |
| `969:5` | `perturbation_tangent` | replace perturbation_tangent -> Result<Option<Vec3>, MathError> with Ok(None) | killed | `third_order_contact_line_is_marched_by_the_perturbation_search` |

## Open

Four mutants are neither killed nor called equivalent. All four sit in the
trace scan of `march_direction_with_branches` and change only which
neighbouring trace points feed the branch scan; none changes an exact
answer on the fixtures above.

| Mutant | Why it is not equivalent | What would kill it |
| --- | --- | --- |
| `653:88` `<` → `<=` (`dominated_by_prev`) | With two adjacent traced points whose cross magnitudes below the band are bitwise equal, the original seeds from both, the mutant from neither, and the crossing loses its transverse branch | A march whose valley holds a bitwise tie: e.g. symmetric samples `(0, ±δ)` across a crossing. No march fixture producing an exact tie was found; the RKF45 steps and turning-point halving never placed symmetric samples |
| `655:87` `<` → `<=` (`dominated_by_next`) | As 653:88 for the following neighbour | As 653:88 |
| `697:31` `>` → `<` (`prev = pt` for every `idx`) | For `idx > 0` the scan's incoming direction becomes the forward difference `next - pt` instead of the central `next - prev`. On a straight branch the two are identical; on a curved branch they differ by `O(h·κ)`, which matters only at the 30° seed-acceptance gate or the tie arm | A curved first branch whose nulls are about 30° apart, so the forward difference rotates a seed below the gate while the central one keeps it. Not built in the box |
| `698:28` `-` → `/` (`traced[idx / 1]`) | Same effect as 697:31 (`prev = pt`) | As 697:31 |

These are recorded against B19; none is a suspected defect (the original
behaviour is the intended one, and the mutants only weaken the scan).

## Stage B (fixed code)

Scope: every mutant of the rewritten `null_directions` (67, including its two
whole-body replacements) and the 19 mutants on the six
`march_direction_with_branches` lines whose Stage-A survivors later tests
target (`642, 643, 644, 648, 649, 662`, i.e. old `696–716`). The
`march_direction` change (`if cur_tangent.is_some()`) generates no mutant;
its pin is the regression test failing on the old code. The rest of
`march_direction` and `march_direction_with_branches` was not re-run (a full
241-mutant Stage B was started and cut for time after 15 results, all in
line with Stage A).

`cargo mutants -p remus-math -j 2 -F <3 regexes>`: **88 tested in 13 min:
57 caught, 18 missed, 13 unviable.**

| Scope | Caught | Missed | Unviable |
| --- | ---: | ---: | ---: |
| `null_directions` (fixed) | 43 | 13 | 13 |
| `march_direction_with_branches` lines 642–662 | 14 | 5 | 0 |

Every Stage B survivor has a verdict above: in `null_directions`, `312:46`,
the seven dedup mutants (`346:*`, same proof as old `400:*`), the two
`trace` mutants (`356:23`, old `410:23`) and the three redundant-condition
mutants (`358:*`, old `412:*`) are the equivalences already proven; in the
scan, `662:{46,60,74} * → /` are the bitwise `±1` equivalences and
`643:31 > → <`, `644:28 - → /` are the two open entries (old `697:31`,
`698:28`). Everything the fix added (the cancellation-free root, both
candidate vectors, the gate) is caught: by
`indefinite_forms_return_their_two_null_lines`,
`indefinite_nulls_are_sorted_and_oriented_deterministically` and
`ill_conditioned_indefinite_forms_keep_true_nulls`.

## Verification

- `cargo test -p remus-math --lib intersection`: 163 passed (the new
  module's 26 tests plus the existing intersection suite).
- Both defect regressions fail on `origin/main` code
  (`ill_conditioned_indefinite_forms_keep_true_nulls`: "Q=(1,1,1e-20):
  [1, 0, 0] is not a null, Q(t,t)=1"; `march_landing_exactly_on_a_crossing_keeps_its_branch`:
  "trace left x = 0: (0.998, 0, 0)") and pass with the fixes.
- `cargo nextest run --workspace --cargo-profile ci-test --no-fail-fast`:
  7080 tests run: 7080 passed (18 slow), 92 skipped (622.6 s, branch head with both fixes).
- `cargo fmt --all`, `cargo clippy -p remus-math --all-targets -- -D warnings`
  (and the pre-commit workspace clippy on every commit),
  `./scripts/check-boundaries.sh`, `./scripts/check-det-hash.sh`,
  `./scripts/check-doc-paths.sh`, `scripts/test-mutants-scope.py`: clean.
- Stage A / Stage B outcome counts come from each run's
  `mutants.out/{caught,missed,unviable}.txt`; kill attribution from the
  per-mutant logs. This is local validation; the weekly Mutation Testing run
  on `main` remains the whole-list verdict.
