# Pave-filler F3 mutation outcomes — 2026-10-02 (B19 F3c)

One row per "Needs review (F3)" survivor that the M2 proof tranche
([mutants-proof-2026-10-01](mutants-proof-2026-10-01.md)) left in
`crates/algo/src/pave_filler/`: `curved_section_clip.rs` (65),
`curved_section_clip/bernstein.rs` (8), `phase_ff.rs` (30) and `phase_ef.rs` (15),
118 in all. Each is killed by a new `remus-algo` test whose oracle is a closed form
or a one-clause input, proven equivalent or unkillable in one line, or listed under
Not reached. No survivor exposed a defect. There is no production change and no
assertion was weakened. The M2 doc is unchanged; this doc carries the outcomes for
these four files.

## Source and scope

- The 118 are exactly CI run 36302856971's `missed`/`timeout`/`unexamined` lists for
  the four files (311 names, deduped across the 24 shards) intersected with M2's
  re-run `missed` lists for the same files (`curved_section_clip` + `bernstein` 73,
  `phase_ff` 184, `phase_ef` 81). Per file: 65 + 8 + 30 + 15, matching M2's table.
- The four production files are byte-identical between the CI head `763ad346` and
  this branch's base `b98cf32f`, so every line number below is both the CI's and the
  current one.

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly pin), the committed `.cargo/mutants.toml`
  (profile `ci-test`, nextest, `--tests`, first-failure stop, per-package oracle):
  the oracle is the whole `remus-algo` suite, as in CI. Scope: `-p remus-algo -F`
  with an anchored alternation of the 118 exact mutant names (`--list` returns
  exactly 118), `-j 2`, `CARGO_BUILD_JOBS=4`.
- Before: CI run 36302856971 and M2's re-run (all 118 ran and were MISSED on current
  code). After: the classifications below report the local triage; a completed
  final 118-mutant run artifact was not independently verified during integration.
  A first run of the 73 clip/Bernstein mutants on an earlier test tree
  (58 caught, 15 missed) found three survivors whose fixture still refused for an
  unrelated reason; the fixture was fixed and each was re-checked by a manual
  mutant run (patch applied, narrowed test run, revert).
- Verdicts: (a) killed by the named test; (b) equivalent: no input distinguishes it;
  (u) unkillable: distinguishable only below a stated resolution (a rounding ulp, an
  enclosure width) or behind a stronger certificate the code checks first; (n) not
  reached in the 5-hour box. No (c) defect rows.
- Oracle style: `phase_ff/helper_oracle_tests.rs` — hand-placed inputs at each
  comparison boundary. Distances are 3-4-5 exact; chart fixtures put rectangle
  corners where the axis-chart exactness gate passes; the clip's interval gates were
  first modelled in a 120-line Python emulation of the event and material checks
  (scratch, not committed) to find inputs where exactly one gate decides.

## New tests and their oracles

| Test (all in `remus-algo`) | What it pins | Oracle |
| --- | --- | --- |
| `curved_section_clip/tests.rs::exact_sum_is_true_exactly_when_the_float_sum_rounds_nothing` | TwoSum exactness test | `1 + 2^-60` and `0.1 + 0.2` round; `1 + 0.5`, `-2 + 3` do not |
| `::bezier_accepts_exactly_one_clamped_span_of_degree_one_to_three` | Bezier-span gate, one clause per input | Degree 4, count off by one, decreasing span, `±1e308` width overflow, an interior knot in each clamp |
| `::check_residual_accepts_a_bound_equal_to_the_tolerance_and_refuses_above` | `bound > tol` refusal boundary | 3-4-5 points: bound encloses 5; `tol = bound` passes, `bound.next_down()` refuses |
| `::work_budget_admits_exactly_its_segment_count` | `used > segments` boundary | Two one-loop faces peak at 1 + 4 + 4 = 8 segments: 8 passes, 7 refuses |
| `::nonpositive_or_nonfinite_tolerance_is_invalid_input` | Tolerance gate | `0`, `-1e-7`, NaN, ∞ each `InvalidInput` |
| `::trims_wholly_above_or_below_the_trace_contribute_no_events` | Trace-outside-rectangle skip | Only the partner's 2 crossings for a trim above/below `v = 0`; 4 for a hole off the trace |
| `::nurbs_side_edges_map_the_trace_through_their_own_trim` | Edge-parameter map `r0 + g(r1 − r0)` | Side edges as degree-1 NURBS trimmed `(7, 11)`: `v = 0` halves each side, so every event sits at 9 |
| `::coincident_interior_events_on_one_chart_are_one_proven_cut` | Same-chart overlap acceptance | Both faces trimmed at `u = 1/8, 7/8` on one chart: one interval, two uses per end |
| `::full_source_ends_on_foreign_charts_keep_both_uses` | Exact-source-end acceptance | FULL × FULL on charts `(0,1)`, `(-1,1)`: `[0, 1]` kept, two uses per end |
| `::overlapping_events_need_a_shared_chart_coordinate_or_source_end` | Chart-coordinate conjunction | Events 8 ulps apart (`0.5`, `0.5 + 2^-50`): intervals overlap, every later gate stays decided, so only the order proof refuses; equal fractions on charts sharing only their upper end refuse |
| `::an_inexact_source_end_coincidence_is_not_a_shared_end` | Each `same(…, 0/1)` conjunct of the source-end test, both pair orders | `2^-1074/4` underflows to fraction 0 and `(896 + 2^60)/(1024 + 2^60)` rounds to 1 while their enclosures are not exact; the section re-knotted over `(0, 1)` keeps the subnormal window nonempty |
| `::adjacent_cuts_without_a_representable_midpoint_refuse` | Midpoint gate | Chart `(0, 1024)` edge at `2^-1064`: fraction `2^-1074`, so `(0, 2^-1074)` has no midpoint while the material samples stay clear |
| `::rectangle_membership_is_ambiguous_exactly_when_the_interval_touches_a_side` | `inside` comparisons | FULL bounds `[0,1]×[-1.5,1.5]`; intervals touching or straddling either side refuse, clear ones decide |
| `::a_loop_that_is_not_four_closed_uses_is_an_invalid_boundary` | Loop shape gate | A closed triangle and an open four-use loop |
| `::boundary_charts_must_be_exact_axis_lines_from_parameter_zero` | Axis-chart exactness gate, one clause each | Range `[1, 1+len]`; directions `(2^-1074, ±1)` and `(±1, 2^-1074)` on quarter-unit sides (the off-axis step rounds to 0); an end at `0.1 + 0.2` |
| `::a_chart_corner_one_ulp_off_its_neighbour_is_an_invalid_boundary` | Exact corner connectivity | A side shifted 1 ulp still certifies against its 3D edge; only the corner test refuses |
| `::a_hole_touching_any_outer_side_is_an_invalid_boundary` | Hole-inside-outer test, one side each | Holes on each of the four outer sides refuse; a clear hole keeps `[0, 1/4] ∪ [3/4, 1]` |
| `::a_patch_reported_periodic_in_u_refuses_although_regular` | Periodic gate | Section scaled `1e-8`: first/last rows `1.4e-8` apart (< the absolute `1e-7` of `is_periodic_u`), rows a unit apart in v |
| `::unvalidated_section_weights_are_invalid_input`, `::unvalidated_patch_grids_are_invalid_input` | Weight/grid gates the constructors pre-empt | `serde_json`-edited NURBS: zero and negative weights, a short weight vector, a zero-weight row, weight rows of three, control rows of three |
| `::interval_division_follows_the_divisor_sign` | `I::div` sign test | `1/[-2,-1] = [-1,-0.5]`, `1/[1,2] = [0.5,1]`, divisors touching 0 give the whole line |
| `::bernstein_product_uses_binomial_degree_elevation` | `choose` weights | `1·1 = 1` at degrees 2 and 3, `(1−t)t = [0, ½, 0]`, `t² = [0, 0, 1]` |
| `::ruled_normals_are_the_hodograph_cross_the_ruling` | Derivative factor `n` | `x' = 2` on a segment gives normal `(0,0,6)`; `x = 2t²` gives `[0, 4, 8, 12]` |
| `::residual_has_no_certificate_when_a_weight_enclosure_reaches_zero` | Denominator gate | Weight `0` or `-1` gives `+∞`; identical points a sub-`1e-150` bound |
| `phase_ef.rs::tests::extra_pave_within_keeps_the_radius_and_the_acceptance_test` | Radius and accept gates | Pave at distance 5: radius 6 and exactly 5 find it, 4 does not, a refusing `accept` does not |
| `::extra_pave_within_returns_the_nearest_unless_candidates_are_distinct` | Nearest choice and ambiguity | Candidates 3 and 5 away, 4 apart: tolerances 5 and 4 return the nearer in both orders, 1 returns `None`; equidistant candidates keep the first |
| `pave_filler/tests.rs::ef_grazing_contact_reuses_the_edges_own_on_wall_extra_pave` | B53 snap through `check_edge_face_pairs` | Box edge `y = 0, z = 1` grazes the cylinder `(x−1)² + (y+1)² = 1` at exactly `(1,0,1)`, where an extra pave already sits: the EF interference must reuse it |
| `phase_ff/helper_oracle_tests.rs::a_trace_reaching_the_position_band_exactly_is_still_a_rim_duplicate` | Position band boundary and the in-plane radial | Rim `r = 1e-6 < δ/2`: band is `δ`; a run with ends `(r, 0, ±δ)` gives `hypot(δ, 0) = δ` exactly |
| `::a_trace_beside_the_rim_is_kept_at_any_normal_angle` | Position band width | A run `2δ` outside the rim with parallel normals is kept; on the rim it is dropped |
| `::the_angle_band_is_the_tangency_conditioning_sqrt_two_delta_over_r` | Angle band `√(2δ/r)` | `r = 4`: band `2.236e-3`; carriers at `2e-3` drop, at `3e-3` keep |
| `::every_sample_of_the_trace_domain_is_checked` | Sample parameters over a domain not starting at 0 | One `5δ` excursion centred on sample 2 of 32 over `(1, 2)` |
| `::conic_extrema_need_a_finite_window` | Finite-window gate | `[0, ∞)`, `(-∞, 1]`, `[0, NaN]` give no extrema |

New dev-dependencies of `remus-algo` (flagged): `serde_json` (already a workspace
dependency) and `remus-math` with its `serde` feature. They are the only route to the
clip adapter's weight and grid gates, which `NurbsCurve::new` and `NurbsSurface::new`
pre-empt but deserialized NURBS bypass. No new test file was added, so the AGENTS.md
Module Map is unchanged.

## Before and after, by function

Before: CI MISSED/unexamined, M2 re-run MISSED (all 118). After: reported local
classifications, with the verification limits stated below.

| File | Function | F3 | (a) killed | (b) equivalent | (u) unkillable | (n) not reached |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `curved_section_clip.rs` | `exact_sum` | 4 | 3 | 1 | 0 | 0 |
| `curved_section_clip.rs` | `bezier` | 9 | 8 | 1 | 0 | 0 |
| `curved_section_clip.rs` | `curve_h` | 3 | 3 | 0 | 0 | 0 |
| `curved_section_clip.rs` | `check_residual` | 1 | 1 | 0 | 0 | 0 |
| `curved_section_clip.rs` | `patch` | 7 | 7 | 0 | 0 | 0 |
| `curved_section_clip.rs` | `budget` | 1 | 1 | 0 | 0 | 0 |
| `curved_section_clip.rs` | `rectangles` | 12 | 12 | 0 | 0 | 0 |
| `curved_section_clip.rs` | `inside` | 3 | 3 | 0 | 0 | 0 |
| `curved_section_clip.rs` | `clip_section` | 25 | 16 | 3 | 6 | 0 |
| `bernstein.rs` | `I::div`, `choose`, `derivative`, `residual` | 7 | 7 | 0 | 0 | 0 |
| `bernstein.rs` | `restrict` | 1 | 0 | 0 | 1 | 0 |
| `phase_ef.rs` | `edge_extra_pave_within` | 14 | 14 | 0 | 0 | 0 |
| `phase_ef.rs` | `check_edge_face_pairs` | 1 | 1 | 0 | 0 | 0 |
| `phase_ff.rs` | `is_marched_trace_of_tangent_rim` | 13 | 13 | 0 | 0 | 0 |
| `phase_ff.rs` | `conic_arc_axis_extrema` | 2 | 1 | 0 | 1 | 0 |
| `phase_ff.rs` | `clip_line_to_face` | 2 | 0 | 2 | 0 | 0 |
| `phase_ff.rs` | `single_hit_circle_is_graze` | 1 | 0 | 0 | 1 | 0 |
| `phase_ff.rs` | `perform_with_context` | 5 | 0 | 2 | 1 | 2 |
| `phase_ff.rs` | `trim_torus_oval_to_box_face` | 7 | 0 | 0 | 0 | 7 |
| **Total** | | **118** | **90** | **9** | **10** | **9** |

## Every mutant

Line:column as in the CI run (identical on this branch). "Killed by" names the test
whose oracle distinguishes the mutant; modules as in the table above.

### `curved_section_clip.rs` (65)

| Mutant | Verdict | Killed by / proof |
| --- | --- | --- |
| 136:5 `exact_sum` → `true` | (a) | `exact_sum_is_true_exactly_when_the_float_sum_rounds_nothing` |
| 137:25 `-` → `+` in `exact_sum` | (a) | same (`1 + 2^-60` reads exact) |
| 138:34 `+` → `*` in `exact_sum` | (a) | same |
| 138:34 `+` → `-` in `exact_sum` | (b) | TwoSum's partial errors are never equal and nonzero: `ae = 0` when `|a| ≥ |b|`, and `ae = be = d ≠ 0` would need `ulp(b) ≤ |d| ≤ ulp(s)/4` while `|s| < 2|b|` gives `ulp(s) ≤ 2 ulp(b)`; so `ae − be = 0 ⇔ ae + be = 0` (exhaustive 5-bit check: 0 of 295,936 pairs differ) |
| 141:5 `bezier` → `Ok(())` | (a) | `bezier_accepts_exactly_one_clamped_span_of_degree_one_to_three` |
| 141:35 `\|\|` → `&&` in `bezier` | (a) | same (degree 4) |
| 141:58 `\|\|` → `&&` in `bezier` | (a) | same (count off by one) |
| 146:9 `\|\|` → `&&` in `bezier` | (b) | A non-finite `lo` or `hi` makes `hi − lo` non-finite, refused by the clause on line 148 |
| 147:9 `\|\|` → `&&` in `bezier` | (a) | same (decreasing span) |
| 148:9 `\|\|` → `&&` in `bezier` | (a) | same (decreasing span) |
| 148:17 `-` → `+` in `bezier` | (a) | same (`±1e308`) |
| 149:9 `\|\|` → `&&` in `bezier` | (a) | same (interior low knot) |
| 150:9 `\|\|` → `&&` in `bezier` | (a) | same (interior high knot) |
| 159:9, 159:58, 160:9 `\|\|` → `&&` in `curve_h` | (a) ×3 | `unvalidated_section_weights_are_invalid_input` |
| 176:36 `>` → `>=` in `check_residual` | (a) | `check_residual_accepts_a_bound_equal_to_the_tolerance_and_refuses_above` |
| 188:35, 189:9 `\|\|` → `&&` in `patch` | (a) ×2 | `unvalidated_patch_grids_are_invalid_input` (control rows of three) |
| 190:9, 192:76 `\|\|` → `&&` in `patch` | (a) ×2 | same (zero-weight row) |
| 192:39 `\|\|` → `&&` in `patch` | (a) | same (weight rows of three) |
| 206:9, 207:9 `\|\|` → `&&` in `patch` | (a) ×2 | `a_patch_reported_periodic_in_u_refuses_although_regular` |
| 285:13 `>` → `>=` in `budget` | (a) | `work_budget_admits_exactly_its_segment_count` |
| 311:42 `\|\|` → `&&` in `rectangles` | (a) | `a_loop_that_is_not_four_closed_uses_is_an_invalid_boundary` (open loop) |
| 311:67 `\|\|` → `&&` in `rectangles` | (a) | same (triangle) |
| 337:17 `\|\|` → `&&` in `rectangles` | (a) | `boundary_charts_must_be_exact_axis_lines_from_parameter_zero` (range `[1, 1+len]`) |
| 337:40 `&&` → `\|\|` in `rectangles` | (a) | same (`(2^-1074, ±1)`) |
| 338:42 `&&` → `\|\|` in `rectangles` | (a) | same (`(±1, 2^-1074)`) |
| 339:17, 340:17 `\|\|` → `&&` in `rectangles` | (a) ×2 | same (`0.1 + 0.2`) |
| 407:29, 407:64 `\|\|` → `&&` in `rectangles` | (a) ×2 | `a_chart_corner_one_ulp_off_its_neighbour_is_an_invalid_boundary` |
| 443:29, 443:49, 443:69 `\|\|` → `&&` in `rectangles` | (a) ×3 | `a_hole_touching_any_outer_side_is_an_invalid_boundary` |
| 458:43 `<` → `<=`, 458:58 `>` → `>=`, 461:25 `\|\|` → `&&` in `inside` | (a) ×3 | `rectangle_membership_is_ambiguous_exactly_when_the_interval_touches_a_side` |
| 482:46 `\|\|` → `&&` in `clip_section` | (a) | `nonpositive_or_nonfinite_tolerance_is_invalid_input` |
| 517:31 `<` → `<=` in `clip_section` | (b) | `v == bounds[2]` already returned `AmbiguousBoundary` on line 514 |
| 517:31 `<` → `==` in `clip_section` | (a) | `trims_wholly_above_or_below_the_trace_contribute_no_events` |
| 517:45 `\|\|` → `&&` in `clip_section` | (a) | same |
| 517:63 `>` → `==` in `clip_section` | (a) | same |
| 517:63 `>` → `>=` in `clip_section` | (b) | `v == bounds[3]` already returned on line 514 |
| 540:45 `\|\|` → `&&`; 540:51 `>` → `==`, `>=` in `clip_section` | (u) ×3 | The event's surface residual is a float sample of the section/patch pair whose whole-span certificate (`≥` the true distance) already passed `≤ tol`; it can differ only if evaluation error outgrows the certificate's outward rounding. Measured over the 72-case curved matrix: worst sample 0.29 of the certificate |
| 546:58 `*` → `/` in `clip_section` | (a) | `nurbs_side_edges_map_the_trace_through_their_own_trim` |
| 555:51 `\|\|` → `&&`; 555:72 `>` → `==`, `>=` in `clip_section` | (u) ×3 | Same argument with the per-side certificate in `rectangles` (edge vs exact boundary, `≤ tol`); measured worst sample `3.8e-8 · tol` |
| 572:54 `-` → `+` in `clip_section` | (b) | `pcurve_range[0]` is exactly 0 (line 336 refuses anything else), so `pr1 − pr0 == pr1 + pr0` |
| 588:17 `&&` → `\|\|` in `clip_section` | (a) | `overlapping_events_need_a_shared_chart_coordinate_or_source_end` (8 ulps) |
| 589:17 `&&` → `\|\|` in `clip_section` | (a) | same (charts sharing an upper end) |
| 591:36, 591:57, 591:78 `&&` → `\|\|` in `clip_section` | (a) ×3 | `an_inexact_source_end_coincidence_is_not_a_shared_end` (near 0, both orders) |
| 592:21 `\|\|` → `&&` in `clip_section` | (a) | `full_source_ends_on_foreign_charts_keep_both_uses` |
| 593:25, 594:25, 595:25 `&&` → `\|\|` in `clip_section` | (a) ×3 | `an_inexact_source_end_coincidence_is_not_a_shared_end` (near 1, both orders) |
| 596:39 `&&` → `\|\|` in `clip_section` | (a) | `coincident_interior_events_on_one_chart_are_one_proven_cut` |
| 608:24 `\|\|` → `&&` in `clip_section` | (a) | `adjacent_cuts_without_a_representable_midpoint_refuse` |

### `curved_section_clip/bernstein.rs` (8)

| Mutant | Verdict | Killed by / proof |
| --- | --- | --- |
| 56:32 `>=` → `<` in `I::div` | (a) | `interval_division_follows_the_divisor_sign` |
| 76:5 `choose` → `1.0`, → `-1.0`; 78:33 `-` → `+` | (a) ×3 | `bernstein_product_uses_binomial_degree_elevation` |
| 99:54 `-` → `+`, `-` → `/` in `derivative` | (a) ×2 | `ruled_normals_are_the_hodograph_cross_the_ruling` |
| 176:24 `>` → `>=` in `restrict` | (u) | Differs only when `a.lo == b.hi` (touching enclosures); then both branches compute enclosures of the same polynomial `h(a + (b − a)s)` — de Casteljau at a negative fraction is the same reparameterization as the reversed split — so only the enclosure widths differ |
| 197:33 `\|\|` → `&&` in `residual` | (a) | `residual_has_no_certificate_when_a_weight_enclosure_reaches_zero` |

### `phase_ef.rs` (15)

| Mutant | Verdict | Killed by |
| --- | --- | --- |
| 763:67 `<=` → `>` in `check_edge_face_pairs` | (a) | `pave_filler/tests.rs::ef_grazing_contact_reuses_the_edges_own_on_wall_extra_pave` (mutant mints a new vertex) |
| 855:5 `edge_extra_pave_within` → `None` | (a) | `extra_pave_within_keeps_the_radius_and_the_acceptance_test` |
| 868:25 `>` → `<`, `==`, `>=` | (a) ×3 | same (radii 6, 5, 4) |
| 868:34 `\|\|` → `&&`; 868:37 delete `!` | (a) ×2 | same (refusing `accept`) |
| 873:31 `\|=` → `&=` | (a) | `extra_pave_within_returns_the_nearest_unless_candidates_are_distinct` (tolerance 1) |
| 873:60 `>` → `<`, `==`, `>=` | (a) ×3 | same (tolerances 5 and 4) |
| 874:33 `<` → `<=`, `==`, `>` | (a) ×3 | same (both orders, equidistant pair) |
| 882:6 delete `!` | (a) | `extra_pave_within_keeps_the_radius_and_the_acceptance_test` |

### `phase_ff.rs` (30)

| Mutant | Verdict | Killed by / proof |
| --- | --- | --- |
| 477:16 delete `!`, 479:21 delete `!` in `perform_with_context` | (n) ×2 | See Not reached |
| 940:17, 942:21 `&&` → `\|\|` in `perform_with_context` | (b) ×2 | They widen `torus_wall_pair`, but every widened pair reaches `section_notches_one_rim`, which returns `false` for every pair except torus × cone/cylinder, and the non-split branch pushes `raw` unchanged |
| 947:63 `>` → `>=` in `perform_with_context` | (u) | Differs only for a marched section whose endpoint gap computes to exactly `tol.linear`; no fixture can place a marcher's output on that double |
| 3189:54, 3192:48, 3197:49, 3214:26, 3214:87 (`<=`, `==`), 3223:77 in `trim_torus_oval_to_box_face` | (n) ×7 | See Not reached |
| 4970:25 `&&` → `\|\|` in `conic_arc_axis_extrema` | (a) | `helper_oracle_tests.rs::conic_extrema_need_a_finite_window` |
| 4983:37 `+` → `-` in `conic_arc_axis_extrema` | (u) | `t* + π` and `t* − π` differ by τ, so the first periodic copy at or after `lo` is the same angle up to rounding; only an extremum within ulps of `hi` can flip, and the arc's endpoint already bounds that side |
| 5474:30, 5479:38 (`+`, `/`), 5479:53 (`%`, `*`) in `is_marched_trace_of_tangent_rim` | (a) ×5 | `every_sample_of_the_trace_domain_is_checked` |
| 5488:34, 5488:38 (`+`, `/`) | (a) ×3 | `a_trace_beside_the_rim_is_kept_at_any_normal_angle` |
| 5489:38 (`%`, `*`) | (a) ×2 | `the_angle_band_is_the_tangency_conditioning_sqrt_two_delta_over_r` |
| 5493:29 `-` → `+`; 5494:36 `>` → `>=` | (a) ×2 | `a_trace_reaching_the_position_band_exactly_is_still_a_rim_duplicate` |
| 5494:36 `>` → `==` | (a) | `a_trace_beside_the_rim_is_kept_at_any_normal_angle` |
| 6807:42 `+` → `-` in `single_hit_circle_is_graze` | (u) | `Circle3D::evaluate` is `c + r(cos t·u + sin t·v)`, so `t ± π` name the same antipode up to an ulp of `r`, below the extent's margin and weld band |
| 7761:21, 7762:21 `-` → `+` in `clip_line_to_face` | (b) ×2 | The plane frame is anchored at `circle.center()`, so `c2 = frame.project(center)` is `(±0, ±0)` and `s − c == s + c` |

## Not reached

9 mutants, all in `phase_ff.rs`; none is called equivalent.

- `perform_with_context` 477:16 and 479:21 (delete `!` around the tangent-rim
  filter). Killing them needs an FF run in `remus-algo` on a torus face tangent to a
  cylinder face whose marcher emits a trace beside the exact rim (the hammer-holder
  shape). That run exists only at the io layer (`crates/io/tests/regress_hammer_*`,
  `step_roundtrip_hammer_holder_repro.rs`), which the per-package oracle does not
  count. Next step: a native torus × tangent-cylinder face pair in
  `phase_ff/helper_oracle_tests.rs`, asserting the FF curve set holds the exact rim
  and no marched duplicate.
- `trim_torus_oval_to_box_face` 3189:54, 3192:48, 3197:49 (`< 0.0` → `<= 0.0` on the
  torus implicit), 3214:26 (`min_d <` → `<=`), 3214:87 (`< dedup_tol` → `<=`, `==`),
  3223:77 (`< r` → `<=`). Six are exact ties: a rim sample, bisection midpoint or
  distance landing exactly on zero, the on-oval band, the dedup radius or the rim
  radius. 3214:87 `==` disables the crossing dedup; a probe with two rim arcs
  overlapping around a crossing (both bracket it) still passed under that mutant —
  the duplicate crossing is absorbed downstream — so the distinguishing input needs a
  duplicate that survives into the kept arcs. Not attempted further in the box.

## After

The reported triage classifies 90 candidates as killed, 9 as equivalent,
10 as distinguishable only at rounding, enclosure or certificate margins, and
9 as not reached. These are classification totals, not verified final-run
`caught`/`missed`/`timeout` counts. The final 118-mutant run artifact was not
available to this review; the nine not-reached candidates remain open.

## Verification

Independent review at PR #933 head `5b646c1e` ran
`cargo test --offline --locked -q -p remus-algo pave_filler`: 214 passed.
The weekly Mutation Testing run on `main` remains the whole-list proof.
