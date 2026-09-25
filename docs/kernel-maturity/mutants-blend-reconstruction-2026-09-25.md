# Exact blend reconstruction mutation triage — 2026-09-25

B19 survivor tranche for the four exact blend-reconstruction modules owned by
P-Class 6.2/6.3/6.5 (see [exact blend reconstruction](../roadmap/exact-blend-reconstruction.md)).
One row per survivor.

## Source

GitHub Actions run 36075171651 (Mutation Testing, sharded, 2026-09-25, head
`1fa7d150`): 1,835 mutants examined, 825 missed. 230 of the missed mutants are in
this tranche's files: `resize_blend.rs` 103, `local_wound.rs` 90,
`affine_blend_caps.rs` 25, `remove_blends.rs` 12. None of the four files changed
between `1fa7d150` and this branch's base `9a4d4a7`. The run's artifacts are not
reachable from the session's network, so the survivor list was rebuilt from the
19 shard job logs' `MISSED` lines; it matches the per-file counts above exactly.

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly workflow's pin), Rust 1.96.0, `ci-test`
  profile, nextest. Every run selects exactly the CI-missed mutants of one file
  (`--re` over their full names) and nothing else.
- Oracle (disclosed, narrower than CI): the unit tests of the four modules'
  `tests` modules, plus the `regress_towel_rack_r1_removal` integration binary for
  `resize_blend.rs`. A narrower oracle can only report a caught mutant as missed,
  never hide a survivor. The "before" column uses the same oracle on the pristine
  base, so before and after are comparable; the CI column is the full-package
  oracle.
- `CARGO_PROFILE_CI_TEST_INCREMENTAL=true` for local runs only (13 s per mutant
  rebuild instead of 107 s); it does not change code generation semantics.
- Verdicts: **(a)** killed by a new test whose oracle is independent of the code
  under test (closed-form geometry, hand-built one-clause refusal inputs, or
  construction-history resolution); **(b)** equivalent, unreachable, or a pure
  tolerance-boundary flip, with a one-line proof; **(c)** not killed, a scope or
  rounding finding stated in the row.
- "Tolerance boundary" rows flip `>`/`>=` (or `<`/`<=`) on a comparison against a
  modeling gate. They change behavior only when a measured float equals the gate
  exactly; following the #639 precedent they are recorded as (b), not pinned.
- Reconciliation: the verdict map was written before the authoritative after-sweep,
  and every row's "After" column is the measured outcome. The sweep left four (a)
  rows alive (`local_wound.rs` 240:27 `/`, 640:47, 642:47; `resize_blend.rs`
  2090:44). Two follow-up test commits closed them; those rows show a re-run on
  the final commit. The rim-orientation fix inserts a helper above
  `oriented_replacement`, which moves the CI names at `resize_blend.rs` 3855–3888
  down by 11 lines. Those seven rows were re-selected at 3866–3899 and re-run on
  the final commit.
- Measured, verdict and outcome agree on every row: all 114 (a) rows are killed,
  and none of the 116 (b)/(c) rows is.

## Findings

- **Defect fixed (c → fixed):** plane/cylinder and cylinder/cone rim removal
  oriented the rebuilt sharp circle from the contact circle's stored normal only.
  A contact stored on the opposite normal with a decreasing trim is the same arc,
  traversed the same way, but it was spliced backwards and the reconstruction
  refused with inconsistent face orientations. Both sites now use the travel
  normal. Regression tests: `resize_blend::tests::plane_cylinder_rim_removal_is_independent_of_circle_storage`
  and `...cylinder_cone_rim_removal_...`. Both fail on the previous code. The
  surviving `oriented_replacement` mutant is killed by the same tests.
- **Scope gap (recorded, not fixed):** a cylindrical band split into two faces on
  one carrier is recognized as one band by `remove_blends`. Its reconstruction
  refuses ("two equally near sharp corners"): the surgical healer qualifies
  single-face bands only, and the positional fallback cannot close the split wound.
  The evidence table in the exact-blend roadmap page records it.
- **Scope boundary (survivor kept):** `affine_blend_caps.rs:200:9` would admit a
  split cross arc on the planar cap. The surgical path reconstructs that exactly,
  so pinning the refusal would pin a limitation.
- **Dead state:** `curved_cylinder_endfaces` in the surgical healer is written but
  never read (`resize_blend.rs:2529:20`).
- **Not killed, documented:** `resize_blend.rs:3270:41` (`/`) depends on the
  rounding sign of P2's own arc parameter. The displacement bound at
  `resize_blend.rs:3107`/`3114` never binds on consistent strips; killing it needs
  an acute-wedge fixture.

## Before / after

| File | CI survivors | Reproduced locally (before) | Survivors after | Killed | (a) | (b) | (c) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `resize_blend.rs` | 103 | 103 | 72 | 31 | 31 | 67 | 5 |
| `local_wound.rs` | 90 | 90 | 31 | 59 | 59 | 31 | 0 |
| `affine_blend_caps.rs` | 25 | 25 | 6 | 19 | 19 | 5 | 1 |
| `remove_blends.rs` | 12 | 12 | 7 | 5 | 5 | 7 | 0 |
| **Total** | **230** | **230** | **116** | **114** | **114** | **110** | **6** |

Per function:

| File | Function | Survivors | Killed | Left (b) | Left (c) |
| --- | --- | ---: | ---: | ---: | ---: |
| `resize_blend.rs` | `heal_cylinder_plane_band_surgical` | 66 | 6 | 55 | 5 |
| `resize_blend.rs` | `prove_spring_chain` | 15 | 7 | 8 | 0 |
| `resize_blend.rs` | `add_certified_closed_circle_edge` | 6 | 5 | 1 | 0 |
| `resize_blend.rs` | `face_contains_contiguous_chain` | 5 | 4 | 1 | 0 |
| `resize_blend.rs` | `point_line_distance` | 3 | 3 | 0 | 0 |
| `resize_blend.rs` | `defeature_curved_band` | 2 | 1 | 1 | 0 |
| `resize_blend.rs` | `sharp_triple_corner` | 1 | 0 | 1 | 0 |
| `resize_blend.rs` | `unit_plane_of` | 1 | 1 | 0 | 0 |
| `resize_blend.rs` | `orient_corners` | 1 | 1 | 0 | 0 |
| `resize_blend.rs` | `line_line_intersection` | 1 | 1 | 0 | 0 |
| `resize_blend.rs` | `line_plane_intersection` | 1 | 1 | 0 | 0 |
| `resize_blend.rs` | `oriented_replacement` | 1 | 1 | 0 | 0 |
| `local_wound.rs` | `extended_circle_trim` | 21 | 15 | 6 | 0 |
| `local_wound.rs` | `classify_sphere_end` | 17 | 12 | 5 | 0 |
| `local_wound.rs` | `register_pcurve` | 14 | 6 | 8 | 0 |
| `local_wound.rs` | `locally_extends_circle_edge` | 12 | 8 | 4 | 0 |
| `local_wound.rs` | `certify_plane_sphere_circle` | 10 | 7 | 3 | 0 |
| `local_wound.rs` | `line_sphere_roots` | 7 | 7 | 0 | 0 |
| `local_wound.rs` | `triple_plane_corner` | 3 | 2 | 1 | 0 |
| `local_wound.rs` | `unit_plane` | 2 | 2 | 0 | 0 |
| `local_wound.rs` | `splice_face` | 2 | 0 | 2 | 0 |
| `local_wound.rs` | `incident_pair_edge` | 1 | 0 | 1 | 0 |
| `local_wound.rs` | `certify_result` | 1 | 0 | 1 | 0 |
| `affine_blend_caps.rs` | `certify_cross_trim` | 11 | 9 | 2 | 0 |
| `affine_blend_caps.rs` | `certify_source_line` | 6 | 5 | 1 | 0 |
| `affine_blend_caps.rs` | `classify_affine_caps` | 5 | 3 | 1 | 1 |
| `affine_blend_caps.rs` | `set_affine_line_pcurves` | 2 | 1 | 1 | 0 |
| `affine_blend_caps.rs` | `restore_face_pcurves` | 1 | 1 | 0 | 0 |
| `remove_blends.rs` | `carrier_band` | 4 | 4 | 0 | 0 |
| `remove_blends.rs` | `validate_boundary_history` | 3 | 0 | 3 | 0 |
| `remove_blends.rs` | `remove_blends_inner` | 2 | 0 | 2 | 0 |
| `remove_blends.rs` | `recognize_group` | 2 | 0 | 2 | 0 |
| `remove_blends.rs` | `connected_component_count` | 1 | 1 | 0 | 0 |

### `resize_blend.rs`

| Mutant | Verdict | After | Evidence |
| --- | --- | --- | --- |
| 459:13 replace \|\| with && in defeature_curved_band | a | killed | `multi_face_band_selection_keeps_the_established_defeature_logic` |
| 460:13 replace \|\| with && in defeature_curved_band | b | survives | a single-face band with a non-planar support passes the mutated guard but `heal_cylinder_plane_band_surgical` declines non-planar supports itself, so the result is the same `Ok(None)` |
| 2032:18 replace < with <= in sharp_triple_corner | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2053:26 replace / with * in unit_plane_of | a | killed | `unit_plane_of_normalizes_the_stored_equation` |
| 2067:5 replace point_line_distance -> f64 with -1.0 | a | killed | `split_chain_proof_refuses_each_violated_clause` (split vertex 1e-5 off the line) |
| 2067:5 replace point_line_distance -> f64 with 0.0 | a | killed | `split_chain_proof_refuses_each_violated_clause` (split vertex 1e-5 off the line) |
| 2072:48 replace / with % in point_line_distance | a | killed | `split_generatrix_chain_is_proved_in_walk_order` (5e-8 in-tolerance drift over a 10 mm chain) |
| 2090:44 replace && with \|\| in orient_corners | a | killed | `orient_corners_requires_both_endpoints` |
| 2132:5 replace face_contains_contiguous_chain -> Result<bool, OperationsError> with Ok(true) | a | killed | `chain_must_be_the_same_contiguous_run_on_both_faces` |
| 2136:32 replace > with == in face_contains_contiguous_chain | b | survives | a closed wire cannot consist only of an open chain, so a chain at least as long as the wire never matches in either version |
| 2146:21 replace && with \|\| in face_contains_contiguous_chain | a | killed | `chain_must_be_the_same_contiguous_run_on_both_faces` (parallel duplicate edge, both traversals) |
| 2147:21 replace && with \|\| in face_contains_contiguous_chain | a | killed | `chain_must_be_the_same_contiguous_run_on_both_faces` (parallel duplicate edge, both traversals) |
| 2160:25 replace && with \|\| in face_contains_contiguous_chain | a | killed | `chain_must_be_the_same_contiguous_run_on_both_faces` (parallel duplicate edge, both traversals) |
| 2184:32 replace > with >= in prove_spring_chain | a | killed | `single_contact_outside_the_proof_declines_to_the_fallback` |
| 2220:29 replace \|\| with && in prove_spring_chain | b | survives | a chain with two ends and a degree-3 vertex also fails the walk ("ambiguous walk"), so the verdict is the same typed reconstruction refusal |
| 2278:40 replace > with == in prove_spring_chain | a | killed | `single_contact_outside_the_proof_declines_to_the_fallback` (tilt 1e-5 rad inside the support plane) |
| 2278:40 replace > with >= in prove_spring_chain | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2285:80 replace + with - in prove_spring_chain | a | killed | `split_chain_proof_refuses_each_violated_clause` (zero-length middle segment) |
| 2286:26 replace < with <= in prove_spring_chain | b | survives | unreachable: the walk starts at parameter 0 and every later vertex is proved strictly increasing, so no parameter is below -tol |
| 2286:26 replace < with == in prove_spring_chain | b | survives | unreachable: the walk starts at parameter 0 and every later vertex is proved strictly increasing, so no parameter is below -tol |
| 2287:13 replace \|\| with && in prove_spring_chain | b | survives | unreachable clauses: parameters are strictly increasing and the last one is the chain end at `span` |
| 2287:26 replace > with >= in prove_spring_chain | b | survives | unreachable: the last vertex is the chain end at exactly `span`, and monotonicity bounds every earlier one below it |
| 2295:51 replace > with == in prove_spring_chain | a | killed | `split_chain_proof_refuses_each_violated_clause` (split vertex 1e-5 off the line) |
| 2295:51 replace > with >= in prove_spring_chain | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2298:79 replace > with == in prove_spring_chain | a | killed | `split_chain_proof_refuses_each_violated_clause` (chain turned 0.01 rad about the axis) |
| 2302:56 replace > with == in prove_spring_chain | a | killed | `split_chain_proof_refuses_each_violated_clause` (chain slid 1e-3 along the plane) |
| 2302:56 replace > with >= in prove_spring_chain | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2308:9 replace \|\| with && in prove_spring_chain | a | killed | `chain_must_be_the_same_contiguous_run_on_both_faces` |
| 2342:54 replace / with * in line_line_intersection | a | killed | `line_solvers_are_invariant_to_direction_length` |
| 2352:76 replace / with * in line_plane_intersection | a | killed | `line_solvers_are_invariant_to_direction_length` |
| 2393:9 replace \|\| with && in heal_cylinder_plane_band_surgical | b | survives | both callers already pass only planar supports (`heal_planar_band`: all planar; `defeature_curved_band`: exactly two planar), `blend_region` guarantees at least two, and a single cylindrical face has exactly two boundary generatrices |
| 2496:12 replace \|\| with && in heal_cylinder_plane_band_surgical | b | survives | unreachable for a simple band wire: both band-wire uses of an interior chain vertex are chain edges, so no cross can touch it |
| 2503:34 replace > with < in heal_cylinder_plane_band_surgical | b | survives | unreachable: `malformed` needs a spring endpoint with no cross use, i.e. two parallel generatrices sharing a vertex; the spring-chain proof refuses such a chain first |
| 2503:34 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: `malformed` needs a spring endpoint with no cross use, i.e. two parallel generatrices sharing a vertex; the spring-chain proof refuses such a chain first |
| 2503:34 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | unreachable: `malformed` needs a spring endpoint with no cross use, i.e. two parallel generatrices sharing a vertex; the spring-chain proof refuses such a chain first |
| 2522:34 replace != with == in heal_cylinder_plane_band_surgical | b | survives | the later end-grouping refuses the same non-cylindrical curved end with the same typed `curved_end_refusal` |
| 2529:20 delete ! in heal_cylinder_plane_band_surgical | b | survives | `curved_cylinder_endfaces` is written but never read (dead state) |
| 2676:37 replace \|\| with && in heal_cylinder_plane_band_surgical | b | survives | role vertices are trivalent in a simple band wire: B's two band uses are the R8 and oblique contacts, so no spring meets B and A always meets exactly one |
| 2705:58 replace \|\| with && in heal_cylinder_plane_band_surgical | b | survives | at a trivalent role vertex the only kept line is E_z / E_o / E_yo, which satisfies both the original and the mutated filter; a fourth face makes the original decline to the positional healer, which also refuses R8-ending bands, so the public verdict is the same refusal |
| 2733:38 replace == with != in heal_cylinder_plane_band_surgical | b | survives | at a trivalent role vertex the only kept line is E_z / E_o / E_yo, which satisfies both the original and the mutated filter; a fourth face makes the original decline to the positional healer, which also refuses R8-ending bands, so the public verdict is the same refusal |
| 2734:21 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | at a trivalent role vertex the only kept line is E_z / E_o / E_yo, which satisfies both the original and the mutated filter; a fourth face makes the original decline to the positional healer, which also refuses R8-ending bands, so the public verdict is the same refusal |
| 2737:61 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | at a trivalent role vertex the only kept line is E_z / E_o / E_yo, which satisfies both the original and the mutated filter; a fourth face makes the original decline to the positional healer, which also refuses R8-ending bands, so the public verdict is the same refusal |
| 2766:55 replace == with != in heal_cylinder_plane_band_surgical | b | survives | at a trivalent role vertex the only kept line is E_z / E_o / E_yo, which satisfies both the original and the mutated filter; a fourth face makes the original decline to the positional healer, which also refuses R8-ending bands, so the public verdict is the same refusal |
| 2781:21 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | at a trivalent role vertex the only kept line is E_z / E_o / E_yo, which satisfies both the original and the mutated filter; a fourth face makes the original decline to the positional healer, which also refuses R8-ending bands, so the public verdict is the same refusal |
| 2816:51 replace > with == in heal_cylinder_plane_band_surgical | a | killed | `r8_generatrix_proofs_refuse_their_own_violations` (a: E_z tilted 9.4e-5 rad on the carrier) |
| 2816:51 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2825:66 replace > with == in heal_cylinder_plane_band_surgical | a | killed | `r8_generatrix_proofs_refuse_their_own_violations` (b: far endpoint 1e-5 radially off R8) |
| 2825:66 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2876:48 replace > with == in heal_cylinder_plane_band_surgical | a | killed | `r8_generatrix_proofs_refuse_their_own_violations` (c: skewed solve) |
| 2877:13 replace \|\| with && in heal_cylinder_plane_band_surgical | a | killed | `r8_generatrix_proofs_refuse_their_own_violations` (c: skewed solve) |
| 2877:65 replace + with - in heal_cylinder_plane_band_surgical | b | survives | `p2` is built on the sharp line (`point_a + direction_a * t`), so its distance to that line is zero whichever second point defines it |
| 2877:84 replace > with == in heal_cylinder_plane_band_surgical | b | survives | `p2` lies on the sharp line by construction, so this clause is never true in either version |
| 2877:84 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | `p2` lies on the sharp line by construction, so this clause is never true in either version |
| 2887:30 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable after the generatrix and skew certificates: P2 is within 1 mm of a certified R8 point along a line inside the 4.5e-5 rad direction gate, so its radial residual is second order (< 1e-8) |
| 2918:31 replace > with == in heal_cylinder_plane_band_surgical | a | killed | `r8_oblique_corner_proof_refuses_an_offset_oblique_face` |
| 2918:31 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 2982:31 replace \|\| with && in heal_cylinder_plane_band_surgical | b | survives | verdict-equivalent: a strip with a split cross arc declines to the positional healer, which rebuilds the same sharp cube (measured: (6, 12, 8) and V = 1000 through either path) |
| 3040:55 replace > with < in heal_cylinder_plane_band_surgical | b | survives | unreachable: each terminal vertex belongs to exactly one planar cross in a simple band wire, so the entry is never occupied |
| 3040:55 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: each terminal vertex belongs to exactly one planar cross in a simple band wire, so the entry is never occupied |
| 3040:55 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | unreachable: each terminal vertex belongs to exactly one planar cross in a simple band wire, so the entry is never occupied |
| 3065:55 replace > with < in heal_cylinder_plane_band_surgical | b | survives | unreachable: the compound roles A, B, D are not endpoints of any other planar cross, so the entry is never occupied |
| 3065:55 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: the compound roles A, B, D are not endpoints of any other planar cross, so the entry is never occupied |
| 3065:55 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | unreachable: the compound roles A, B, D are not endpoints of any other planar cross, so the entry is never occupied |
| 3107:28 replace * with + in heal_cylinder_plane_band_surgical | c | survives | not killed: the displacement bound never binds on consistent strips (recovered corners lie within one radius); a far corner needs an acute-wedge or inconsistent end plane, which the line-extension check also refuses |
| 3107:28 replace * with / in heal_cylinder_plane_band_surgical | c | survives | not killed: the displacement bound never binds on consistent strips (recovered corners lie within one radius); a far corner needs an acute-wedge or inconsistent end plane, which the line-extension check also refuses |
| 3114:41 replace > with == in heal_cylinder_plane_band_surgical | c | survives | not killed: same displacement bound as 3107:28 |
| 3114:41 replace > with >= in heal_cylinder_plane_band_surgical | c | survives | not killed: same displacement bound as 3107:28 |
| 3249:24 replace < with <= in heal_cylinder_plane_band_surgical | b | survives | unreachable after the transverse certificate (\|axial\| >= 1 - 1e-9) |
| 3249:24 replace < with == in heal_cylinder_plane_band_surgical | b | survives | unreachable after the transverse certificate (\|axial\| >= 1 - 1e-9) |
| 3255:92 replace / with * in heal_cylinder_plane_band_surgical | b | survives | `axial` is +-1 on the transverse family admitted by the 1e-9 gate; `/` and `*` differ by at most 2e-9 relative |
| 3270:41 replace - with + in heal_cylinder_plane_band_surgical | b | survives | P2 is the arc's reference direction, so t0 = 0 up to one rounding and t1 + t0 = t1 - t0 |
| 3270:41 replace - with / in heal_cylinder_plane_band_surgical | c | survives | not killed, rounding-dependent: t1 / t0 with t0 = +-1e-16 picks the branch by the rounding sign of P2's own parameter; both towel bands round to the benign sign |
| 3270:70 replace + with - in heal_cylinder_plane_band_surgical | b | survives | `pi + 1e-9` vs `pi - 1e-9`: differs only for an arc within 1e-9 of antipodal |
| 3271:20 replace - with + in heal_cylinder_plane_band_surgical | b | survives | t0 = 0 up to rounding, so the mutated antipodal test differs only for an antipodal P2/Q* pair, which no blend strip produces |
| 3271:20 replace - with / in heal_cylinder_plane_band_surgical | b | survives | t0 = 0 up to rounding, so the mutated antipodal test differs only for an antipodal P2/Q* pair, which no blend strip produces |
| 3271:25 replace - with + in heal_cylinder_plane_band_surgical | b | survives | t0 = 0 up to rounding, so the mutated antipodal test differs only for an antipodal P2/Q* pair, which no blend strip produces |
| 3271:25 replace - with / in heal_cylinder_plane_band_surgical | b | survives | t0 = 0 up to rounding, so the mutated antipodal test differs only for an antipodal P2/Q* pair, which no blend strip produces |
| 3280:19 replace - with + in heal_cylinder_plane_band_surgical | b | survives | s1 is P2's parameter (0 up to rounding), so s1 + s0 = -(minor span) never reaches pi; differs only for an antipodal pair |
| 3280:48 replace - with + in heal_cylinder_plane_band_surgical | b | survives | differs only for an arc within 1e-9 of antipodal, which no blend strip produces |
| 3280:48 replace - with / in heal_cylinder_plane_band_surgical | b | survives | differs only for an arc within 1e-9 of antipodal, which no blend strip produces |
| 3341:90 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | chain segments are collinear, so the dot product is +-\|a\|\|b\|, never 0 |
| 3377:78 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 3383:33 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | only the D-side spring reaches this branch and its corners are exactly {west, Q*}, so each mutated clause evaluates to the original value |
| 3385:33 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | only the D-side spring reaches this branch and its corners are exactly {west, Q*}, so each mutated clause evaluates to the original value |
| 3386:33 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | only the D-side spring reaches this branch and its corners are exactly {west, Q*}, so each mutated clause evaluates to the original value |
| 3431:29 replace && with \|\| in heal_cylinder_plane_band_surgical | b | survives | a one-corner match that the mutant admits is refused by `orient_corners`, which checks both endpoints, with the same typed error |
| 3484:77 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: a kept line at a terminal vertex separates two planes (a support and the end face, or two certified R8 neighbours); the recovered corner lies on both, hence on the line |
| 3484:77 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | unreachable: a kept line at a terminal vertex separates two planes (a support and the end face, or two certified R8 neighbours); the recovered corner lies on both, hence on the line |
| 3485:21 replace \|\| with && in heal_cylinder_plane_band_surgical | b | survives | unreachable: a kept line at a terminal vertex separates two planes (a support and the end face, or two certified R8 neighbours); the recovered corner lies on both, hence on the line |
| 3485:79 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: a kept line at a terminal vertex separates two planes (a support and the end face, or two certified R8 neighbours); the recovered corner lies on both, hence on the line |
| 3485:79 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | unreachable: a kept line at a terminal vertex separates two planes (a support and the end face, or two certified R8 neighbours); the recovered corner lies on both, hence on the line |
| 3490:30 replace == with != in heal_cylinder_plane_band_surgical | b | survives | unreachable: every adjacent carrier contains the recovered corner (see 3484:77), so skipping the other face changes nothing |
| 3503:80 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: the recovered corner lies on every adjacent support/end plane by construction |
| 3503:80 replace > with >= in heal_cylinder_plane_band_surgical | b | survives | unreachable: the recovered corner lies on every adjacent support/end plane by construction |
| 3515:80 replace > with == in heal_cylinder_plane_band_surgical | b | survives | unreachable: P2 and Q* are certified on R8 before any extension |
| 3608:48 replace == with != in heal_cylinder_plane_band_surgical | a | killed | `journaled_plus_y_r8_removal_resolves_the_oblique_spring_to_line_and_arc` (the +y band stores its oblique spring from D) |
| 3855:13 delete ! in oriented_replacement | a | killed | `plane_cylinder_rim_removal_is_independent_of_circle_storage` (ReversedEdge) |
| 3873:38 replace \|\| with && in add_certified_closed_circle_edge | a | killed | `closed_circle_certificate_checks_seam_and_vertex_tolerance` (negative and NaN tolerance) |
| 3873:58 replace < with <= in add_certified_closed_circle_edge | a | killed | `closed_circle_certificate_checks_seam_and_vertex_tolerance` (zero tolerance accepted) |
| 3873:58 replace < with == in add_certified_closed_circle_edge | a | killed | `closed_circle_certificate_checks_seam_and_vertex_tolerance` (zero tolerance accepted) |
| 3888:34 replace \|\| with && in add_certified_closed_circle_edge | a | killed | `closed_circle_certificate_checks_seam_and_vertex_tolerance` (seam 1e-3 off) |
| 3888:46 replace > with == in add_certified_closed_circle_edge | a | killed | `closed_circle_certificate_checks_seam_and_vertex_tolerance` (seam 1e-3 off) |
| 3888:46 replace > with >= in add_certified_closed_circle_edge | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |

### `local_wound.rs`

| Mutant | Verdict | After | Evidence |
| --- | --- | --- | --- |
| 76:32 replace / with * in unit_plane | a | killed | `unit_plane_normalizes_the_stored_plane_equation`, `non_unit_plane_storage_heals_to_the_same_sharp_corner` |
| 77:15 replace / with * in unit_plane | a | killed | `unit_plane_normalizes_the_stored_plane_equation` |
| 155:35 replace && with \|\| in incident_pair_edge | b | survives | at a manifold terminal the spring and cross are excluded and the only remaining edge is the support/end pair edge, so `\|\|` adds no candidate; a fourth face makes both versions return `Ok(None)` (here or at the three-face corner scan) with no side effect |
| 170:18 replace < with <= in triple_plane_corner | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 170:18 replace < with == in triple_plane_corner | a | killed | `triple_plane_corner_solves_oblique_planes_and_refuses_near_parallel` (\|det\| ~ 1e-8) |
| 175:57 replace / with * in triple_plane_corner | a | killed | `triple_plane_corner_solves_oblique_planes_and_refuses_near_parallel` (oblique planes, \|det\| != 1) |
| 182:56 replace * with + in line_sphere_roots | a | killed | `line_sphere_roots_are_invariant_to_direction_length` |
| 185:52 replace / with * in line_sphere_roots | a | killed | `line_sphere_roots_are_invariant_to_direction_length` |
| 192:48 replace * with / in line_sphere_roots | a | killed | `line_sphere_roots_separate_distinct_roots_from_tangency` |
| 193:51 replace * with + in line_sphere_roots | a | killed | `line_sphere_roots_separate_distinct_roots_from_tangency` |
| 193:51 replace * with / in line_sphere_roots | a | killed | `line_sphere_roots_separate_distinct_roots_from_tangency` |
| 194:41 replace + with - in line_sphere_roots | a | killed | `line_sphere_roots_separate_distinct_roots_from_tangency` |
| 199:40 replace / with * in line_sphere_roots | a | killed | `line_sphere_roots_are_invariant_to_direction_length` |
| 218:5 replace certify_plane_sphere_circle -> bool with true | a | killed | `plane_sphere_circle_certificate_refuses_each_violated_clause`, `uncertified_sphere_boundary_circle_refuses` |
| 219:54 replace > with == in certify_plane_sphere_circle | a | killed | `plane_sphere_circle_certificate_refuses_each_violated_clause` (one clause violated per case) |
| 219:54 replace > with >= in certify_plane_sphere_circle | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 228:9 replace > with == in certify_plane_sphere_circle | a | killed | `plane_sphere_circle_certificate_refuses_each_violated_clause` (one clause violated per case) |
| 228:9 replace > with >= in certify_plane_sphere_circle | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 234:47 replace > with == in certify_plane_sphere_circle | a | killed | `plane_sphere_circle_certificate_refuses_each_violated_clause` (one clause violated per case) |
| 234:47 replace > with >= in certify_plane_sphere_circle | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 239:9 replace && with \|\| in certify_plane_sphere_circle | a | killed | `plane_sphere_circle_certificate_refuses_each_violated_clause` (radius off by 1e-4) |
| 240:27 replace * with + in certify_plane_sphere_circle | a | killed | `plane_sphere_circle_certificate_refuses_each_violated_clause` (radius 1e-4 large is outside 1e-7 * R) |
| 240:27 replace * with / in certify_plane_sphere_circle | a | killed | `plane_sphere_circle_certificate_accepts_the_exact_section` (squared radius 2e-7 off, inside the R-scaled tolerance) |
| 250:9 replace \|\| with && in classify_sphere_end | b | survives | equal supports leave three band edges outside `springs`, so the two-cross destructure refuses; a band face outside the solid has no contacts in the solid's adjacency. Both versions return `Ok(None)` from read-only classification |
| 400:13 replace \|\| with && in classify_sphere_end | a | killed | `uncertified_sphere_boundary_circle_refuses` (same vertices, carrier centre 1e-3 off the sphere axis) |
| 434:29 replace \|\| with && in classify_sphere_end | a | killed | `terminal_vertex_touched_by_a_fourth_face_refuses` |
| 475:29 replace < with <= in classify_sphere_end | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 475:29 replace < with == in classify_sphere_end | a | killed | `opposed_spring_travel_refuses` |
| 493:9 replace \|\| with && in classify_sphere_end | a | killed | `terminals_on_opposite_cap_hemispheres_refuse` |
| 513:13 replace \|\| with && in classify_sphere_end | a | killed | `deep_column_far_root_is_excluded_by_the_cap_hemisphere_alone` (far root passes the wound-direction and local-trim proofs) |
| 518:13 replace && with \|\| in classify_sphere_end | a | killed | `root_that_extends_only_one_boundary_circle_refuses` |
| 547:44 replace > with == in classify_sphere_end | a | killed | `band_carrier_violations_refuse` (b: horizontal cylinder through all four spring endpoints) |
| 552:60 replace > with == in classify_sphere_end | a | killed | `band_carrier_violations_refuse` (a: carrier radius 1e-4 large) |
| 552:60 replace > with >= in classify_sphere_end | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 553:17 replace \|\| with && in classify_sphere_end | a | killed | `band_carrier_violations_refuse` (a, c) |
| 558:21 replace > with == in classify_sphere_end | a | killed | `band_carrier_violations_refuse` (c: spring turned 0.01 rad about the axis) |
| 558:21 replace > with >= in classify_sphere_end | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 572:43 replace * with + in classify_sphere_end | a | killed | `sharp_corner_outside_the_local_patch_refuses` (scale 0.1: 4 x span < displacement < span + 4) |
| 582:55 replace > with == in classify_sphere_end | a | killed | `sharp_corner_outside_the_local_patch_refuses` |
| 582:55 replace > with >= in classify_sphere_end | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 615:33 replace - with + in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 615:33 replace - with / in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 623:35 replace + with - in extended_circle_trim | b | survives | the shift range -3..=3 is symmetric, so `+ shift*tau` and `- shift*tau` generate the same candidate set |
| 629:29 replace - with + in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 631:18 replace > with >= in extended_circle_trim | b | survives | `span > 0.0` vs `>= 0.0` differs only for a zero-length candidate domain, which `strict_domain` never produces for a positive-span arc and which the half-turn test then rejects in both versions |
| 632:17 replace && with \|\| in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 633:47 replace + with * in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 633:47 replace + with - in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 635:47 replace - with + in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 638:18 replace < with <= in extended_circle_trim | b | survives | `span < 0.0` vs `<= 0.0` differs only for a zero-length candidate domain (see 631:18) |
| 638:18 replace < with == in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle` (decreasing-parameter cases) |
| 638:18 replace < with > in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle` (decreasing-parameter cases) |
| 639:17 replace && with \|\| in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 640:47 replace - with + in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle` (zero growth on a decreasing trim) |
| 642:31 replace <= with > in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle` (decreasing end growth) |
| 642:47 replace + with * in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle` (zero growth on a decreasing trim) |
| 642:47 replace + with - in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle` (zero growth on a decreasing trim) |
| 645:20 replace && with \|\| in extended_circle_trim | a | killed | `extended_circle_trim_grows_the_named_terminal_by_the_exact_angle`, `extended_circle_trim_refuses_retraction_and_overturn`, `shifted_decreasing_and_reversed_trims_heal_to_the_same_extension` |
| 645:41 replace + with - in extended_circle_trim | b | survives | `tau + angular` vs `tau - angular`: differs only for a trim within 1e-12 of a full turn |
| 646:41 replace - with + in extended_circle_trim | b | survives | at most one candidate passes: admissible candidates lie in an interval of width tau - \|old span\| < tau and are spaced tau apart, so the sort key never decides |
| 646:41 replace - with / in extended_circle_trim | b | survives | at most one candidate passes: admissible candidates lie in an interval of width tau - \|old span\| < tau and are spaced tau apart, so the sort key never decides |
| 664:5 replace locally_extends_circle_edge -> Result<bool, OperationsError> with Ok(true) | a | killed | `locally_extends_circle_edge_refuses_non_local_roots` |
| 684:57 replace - with + in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_accepts_growth_below_half_a_turn`, `..._refuses_non_local_roots` (domains off zero) |
| 684:57 replace - with / in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_accepts_growth_below_half_a_turn`, `..._refuses_non_local_roots` (domains off zero) |
| 684:73 replace - with + in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_accepts_growth_below_half_a_turn`, `..._refuses_non_local_roots` (domains off zero) |
| 685:18 replace < with <= in locally_extends_circle_edge | b | survives | unreachable: `extended_circle_trim` only returns domains that extend the old terminal by at least -angular, so `extension < -angular` never holds |
| 685:18 replace < with == in locally_extends_circle_edge | b | survives | unreachable: `extended_circle_trim` only returns domains that extend the old terminal by at least -angular, so `extension < -angular` never holds |
| 685:20 delete - in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_accepts_growth_below_half_a_turn` (zero growth) |
| 686:9 replace \|\| with && in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_refuses_non_local_roots` |
| 686:22 replace > with == in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_refuses_non_local_roots` (growth pi + 1e-3) |
| 686:22 replace > with >= in locally_extends_circle_edge | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 686:45 replace - with + in locally_extends_circle_edge | b | survives | `pi + angular` vs `pi - angular`: differs only for growth within 1e-12 of half a turn |
| 686:45 replace - with / in locally_extends_circle_edge | a | killed | `locally_extends_circle_edge_refuses_non_local_roots` |
| 810:62 replace && with \|\| in splice_face | b | survives | `mapped_vertex` sends a spring's two ends to exactly the planar and sphere corners, the sharp edge's two endpoints, so each conjunct pair is both true or both false |
| 811:60 replace && with \|\| in splice_face | b | survives | `mapped_vertex` sends a spring's two ends to exactly the planar and sphere corners, the sharp edge's two endpoints, so each conjunct pair is both true or both false |
| 874:28 replace / with * in register_pcurve | b | survives | the anchor stays on the line through the origin along the plane normal for any scale, and `PlaneFrame` axes depend only on the normal, so projected (u, v) are unchanged |
| 876:28 replace / with * in register_pcurve | b | survives | the anchor stays on the line through the origin along the plane normal for any scale, and `PlaneFrame` axes depend only on the normal, so projected (u, v) are unchanged |
| 915:32 replace + with - in register_pcurve | b | survives | `a1 + k*tau` and `a1 - k*tau` over k in {-1, 0, 1} are the same candidate set; the true branch has zero midpoint error and is unique |
| 917:33 replace - with + in register_pcurve | a | killed | `rotated_bodies_heal_across_planar_chart_seams` (arcs crossing the principal-angle seam) |
| 917:33 replace - with / in register_pcurve | a | killed | `rotated_bodies_heal_across_planar_chart_seams` (arcs crossing the principal-angle seam) |
| 917:70 replace + with - in register_pcurve | b | survives | `tau + angular` vs `tau - angular`: differs only for a candidate within 1e-12 of a full turn |
| 930:31 replace > with == in register_pcurve | b | survives | a branch that misses the analytic midpoint also fails `validate_same_parameter` (1025 samples) before the transaction publishes, so the refusal is the same typed reconstruction error |
| 930:31 replace > with >= in register_pcurve | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 958:26 replace - with / in register_pcurve | a | killed | `sphere_chart_seam_in_either_longitude_direction_heals` |
| 958:37 replace > with == in register_pcurve | a | killed | `sphere_chart_seam_in_either_longitude_direction_heals` |
| 958:37 replace > with >= in register_pcurve | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 959:27 replace -= with += in register_pcurve | a | killed | `sphere_chart_seam_in_either_longitude_direction_heals` |
| 959:27 replace -= with /= in register_pcurve | a | killed | `sphere_chart_seam_in_either_longitude_direction_heals` |
| 960:44 replace < with <= in register_pcurve | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 1051:5 replace certify_result -> Result<(), OperationsError> with Ok(()) | b | survives | postcondition only: no input `classify_sphere_end` accepts reconstructs to an invalid solid or an uncovered p-curve (every accepted fixture validates); the check guards against a future construction defect |

### `affine_blend_caps.rs`

| Mutant | Verdict | After | Evidence |
| --- | --- | --- | --- |
| 88:5 replace certify_source_line -> Result<bool, OperationsError> with Ok(true) | a | killed | `patch_holding_the_cross_arc_but_not_the_cap_lines_declines` |
| 103:26 replace + with - in certify_source_line | a | killed | `tight_patch_around_the_sharp_cap_heals_exactly` (reflected midpoints leave a 0.25 mm margin) |
| 105:9 replace && with \|\| in certify_source_line | a | killed | `source_line_certificate_refuses_a_collapsed_line` |
| 105:31 replace - with + in certify_source_line | a | killed | `source_line_certificate_refuses_a_collapsed_line` |
| 105:31 replace - with / in certify_source_line | a | killed | `source_line_certificate_refuses_a_collapsed_line` |
| 105:37 replace > with >= in certify_source_line | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 113:5 replace certify_cross_trim -> Result<bool, OperationsError> with Ok(true) | a | killed | `patch_holding_the_cap_lines_but_not_the_cross_arc_declines` |
| 124:62 replace > with == in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (0.01 mm circle tilted 5e-6 rad) |
| 124:62 replace > with >= in certify_cross_trim | b | survives | tolerance boundary: differs only when the measured value equals the gate exactly; no contract assigns that single float |
| 139:19 replace - with + in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (rotated, asymmetric patch; one extremum exposed per case) |
| 139:19 replace - with / in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (rotated, asymmetric patch; one extremum exposed per case) |
| 139:40 replace - with + in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (rotated, asymmetric patch; one extremum exposed per case) |
| 139:40 replace - with / in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (rotated, asymmetric patch; one extremum exposed per case) |
| 140:40 replace - with + in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (rotated, asymmetric patch; one extremum exposed per case) |
| 140:40 replace - with / in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` (rotated, asymmetric patch; one extremum exposed per case) |
| 148:23 replace + with * in certify_cross_trim | a | killed | `cross_trim_certificate_checks_the_true_carrier_extrema` |
| 148:23 replace + with - in certify_cross_trim | b | survives | `phase - pi` and `phase + pi` evaluate the same circle point |
| 164:35 replace \|\| with && in classify_affine_caps | a | killed | `non_cylindrical_band_or_curved_support_declines` (spherical band) |
| 169:9 replace \|\| with && in classify_affine_caps | a | killed | `non_cylindrical_band_or_curved_support_declines` (cylindrical support) |
| 178:37 replace \|\| with && in classify_affine_caps | b | survives | a band outside the solid has no neighbour in the solid's adjacency (`other_face` fails), and a foreign support gets no contact edge, so the contact census refuses in both versions |
| 200:9 replace \|\| with && in classify_affine_caps | c | survives | scope boundary, not killed: the mutant accepts a split cross arc on the *planar* cap, which the surgical path would reconstruct exactly; widening the adapter's one-contact-per-cap rule is a scope decision, so no test pins the refusal |
| 209:40 replace \|\| with && in classify_affine_caps | a | killed | `affine_cap_with_an_inner_wire_declines` |
| 285:32 replace \|\| with && in set_affine_line_pcurves | b | survives | unreachable: every restored cap edge was certified by `certify_source_line` (finite, > 64 eps) or is a sharp extension of such a line to a certified corner |
| 291:30 replace + with - in set_affine_line_pcurves | a | killed | `tight_patch_around_the_sharp_cap_heals_exactly` |
| 347:5 replace restore_face_pcurves -> Result<(), OperationsError> with Ok(()) | a | killed | `no_live_face_keeps_the_nurbs_carrier_without_its_pcurves` |

### `remove_blends.rs`

| Mutant | Verdict | After | Evidence |
| --- | --- | --- | --- |
| 114:9 replace \|\| with && in remove_blends_inner | b | survives | postcondition on `remove_recognized_blend_faces`: every producer path returns a total face bijection (the journaled totality tests prove it per family); only a second defect could violate it |
| 115:9 replace \|\| with && in remove_blends_inner | b | survives | postcondition on `remove_recognized_blend_faces`: every producer path returns a total face bijection (the journaled totality tests prove it per family); only a second defect could violate it |
| 230:25 replace && with \|\| in recognize_group | b | survives | a spherical corner patch meets its region only along blend-band edges (a sphere touches a plane tangentially at points), so the dropped clauses exclude nothing for any vertex blend the kernel builds |
| 231:25 replace && with \|\| in recognize_group | b | survives | a spherical corner patch meets its region only along blend-band edges (a sphere touches a plane tangentially at points), so the dropped clauses exclude nothing for any vertex blend the kernel builds |
| 290:30 replace == with != in carrier_band | a | killed | `any_seed_on_a_split_band_recognizes_the_whole_carrier_band` (watchdog turns the re-queue loop into a failure) |
| 291:21 replace \|\| with && in carrier_band | a | killed | `any_seed_on_a_split_band_recognizes_the_whole_carrier_band` (watchdog turns the re-queue loop into a failure) |
| 292:21 replace \|\| with && in carrier_band | a | killed | `any_seed_on_a_split_band_recognizes_the_whole_carrier_band` (watchdog turns the re-queue loop into a failure) |
| 292:24 delete ! in carrier_band | a | killed | `any_seed_on_a_split_band_recognizes_the_whole_carrier_band` (watchdog turns the re-queue loop into a failure) |
| 342:5 replace connected_component_count -> Result<usize, OperationsError> with Ok(1) | a | killed | `disconnected_seeds_name_the_number_of_groups` |
| 404:9 replace \|\| with && in validate_boundary_history | b | survives | postcondition on the reconstruction's boundary history: every producer path records total, kind-preserving history (journaled totality tests); only a second defect could violate it |
| 405:9 replace \|\| with && in validate_boundary_history | b | survives | postcondition on the reconstruction's boundary history: every producer path records total, kind-preserving history (journaled totality tests); only a second defect could violate it |
| 407:44 replace \|\| with && in validate_boundary_history | b | survives | postcondition on the reconstruction's boundary history: every producer path records total, kind-preserving history (journaled totality tests); only a second defect could violate it |
