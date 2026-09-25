# remus-algo phase-FF and builder mutation triage — 2026-09-25

One row per survivor of the 2026-09-25 weekly mutation run in
`crates/algo/src/pave_filler/phase_ff.rs`, `crates/algo/src/builder/builder_solid.rs` and
`crates/algo/src/builder/fill_images_faces.rs`. Every survivor is either killed by a new test whose
oracle is independent of the code under test, or proven equivalent or unkillable in one line. No
survivor exposed a defect. There is no production change.

## Source and scope

- Source of truth: GitHub Actions run 36075171651 (Mutation Testing, sharded, 2026-09-25, head
  `1fa7d150`). Its 19 shards mutate only that week's diff (`--in-diff` from `3586082a`).
- The survivor list was read from the 19 shard job logs (`MISSED` lines), not the uploaded
  artifacts, whose blob host this environment cannot reach. The logs give 827 missed mutants: the
  825 of the 18 archived shards plus 2 from shard 9, whose runner shut down mid-run and uploaded no
  artifact.
- In scope: 172 survivors — 153 in `phase_ff.rs` (the 151 of the archived shards plus
  shard 9's `trim_torus_oval_to_box_face` 3154:32 and `single_hit_circle_is_graze` 6678:45),
  10 in `builder_solid.rs`, 9 in `fill_images_faces.rs`.
- The three files are byte-identical between `1fa7d150` and this branch's base. The in-diff
  selection was rebuilt locally from `git diff 3586082a 1fa7d150` restricted to the three files:
  234 mutants, a superset of every CI survivor.

## Method

- Tool: `cargo-mutants 27.1.0` locally (CI pins 27.0.0), Rust 1.96.0, the committed
  `.cargo/mutants.toml` (profile `ci-test`, nextest, `--tests`, first-failure stop, per-package
  oracle). The oracle is therefore the whole `remus-algo` suite, as in CI.
- Before: the CI run. After: one authoritative run of all 234 in-diff mutants on the final
  test tree (`2c02bda`, `-j 3`, baseline green): 188 caught, 33 unviable, 13 missed, 0 timeout.
- Kill attribution: the three files are unchanged from the CI tree, so a CI survivor caught after
  can only be caught by a new test. The table names the first failing test from each mutant's log.
- Verdicts: (a) killed by a new test; (b) equivalent (no input distinguishes it) or unkillable
  (distinguishable only below a stated resolution); (c) real defect. There are no (c) rows.

## New tests and their oracles

| Test module | What it pins | Oracle |
| --- | --- | --- |
| `pave_filler/phase_ff/helper_oracle_tests.rs` | B32 disc-cap miss test in `clip_line_to_face` | Clamped point-to-segment distance against `r + tol`, on 1,200 generated segments at three scales plus named grazes, point segments, and exact single-point touches |
| same | B33 one-hit graze veto (`single_hit_circle_is_graze`) | Antipode's hand-computed distance outside the face, including a sliver face where the weld band, not the margin, decides |
| same | B39 single-rim notch detection (`section_notches_one_rim`) | Hand-placed points on the wall's band bounds, within and beyond the weld band |
| same | B39 torus oval trim on disk caps and straight boxes (`trim_torus_oval_to_box_face`) | A plane cutting a torus off its mid-height gives two exact circles, so kept arcs are checked against circle–circle crossings, the in-face angular span, and seam and domain invariance |
| same | B39 split of a one-rim notch section (`perform_with_context`) | A native repro of the B39 oblique cell: both far ends on the exact top-rim × torus crossing, no section running rim to the same rim |
| `builder/builder_solid/helper_oracle_tests.rs` | Conic support identity, closed-pair traversal, revolved-face wire winding | Constructed geometry (normals, radii), and Newell vector area of each loop against the hand-derived outward normal |
| `builder/fill_images_faces/helper_oracle_tests.rs` | Cap-disc and equal-radius recognition, closed-section containment and coincidence, curved-face segment classifier | The fixtures' own construction: which circle bounds which face, which axial band a generator occupies |

## Before and after, by function

| File | Function | CI survivors | (a) killed | (b) equivalent / unkillable |
| --- | --- | ---: | ---: | ---: |
| `builder_solid.rs` | `orient_revolved_face_wires` | 6 | 6 | 0 |
| `builder_solid.rs` | `closed_pair_traversal_flipped` | 3 | 3 | 0 |
| `builder_solid.rs` | `conics_share_support` | 1 | 1 | 0 |
| `fill_images_faces.rs` | `segment_between_boundary_arcs` | 4 | 4 | 0 |
| `fill_images_faces.rs` | `circle_inside_face` | 2 | 2 | 0 |
| `fill_images_faces.rs` | `equal_radius_cylinder_pair` | 1 | 1 | 0 |
| `fill_images_faces.rs` | `cap_disc_circle` | 1 | 1 | 0 |
| `fill_images_faces.rs` | `closed_curve_coincides_with_boundary` | 1 | 1 | 0 |
| `phase_ff.rs` | `clip_line_to_face` | 70 | 68 | 2 |
| `phase_ff.rs` | `trim_torus_oval_to_box_face` | 62 | 55 | 7 |
| `phase_ff.rs` | `section_notches_one_rim` | 10 | 10 | 0 |
| `phase_ff.rs` | `perform_with_context` | 6 | 3 | 3 |
| `phase_ff.rs` | `single_hit_circle_is_graze` | 5 | 4 | 1 |
| **Total** | | **172** | **159** | **13** |

Survivors after: 13 of 172. The weekly run's per-file counts for these three files go from
153 / 10 / 9 missed to 13 / 0 / 0.

## Equivalent and unkillable survivors

| Mutant | Function | Mutation | Verdict | Proof |
| --- | --- | --- | --- | --- |
| `phase_ff.rs:931:17` | `perform_with_context` | replace && with \|\| | (b) equivalent | Widens `torus_wall_pair`, but the split still requires `section_notches_one_rim`, whose own exhaustive pair match returns false for every pair except torus × cone/cylinder. |
| `phase_ff.rs:933:21` | `perform_with_context` | replace && with \|\| | (b) equivalent | Same as 931:17: `section_notches_one_rim` re-checks the pair and rejects every non torus × wall pair. |
| `phase_ff.rs:938:63` | `perform_with_context` | replace > with >= | (b) equivalent | Differs only when a section's endpoint gap is exactly `tol.linear` (1e-7) in floating point. |
| `phase_ff.rs:3180:54` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when the torus implicit is exactly 0.0 at the rim's start sample. |
| `phase_ff.rs:3183:48` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when the torus implicit is exactly 0.0 at a rim sample. |
| `phase_ff.rs:3188:49` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when the implicit is exactly 0.0 at a bisection midpoint, i.e. at the root itself, so the crossing moves by less than one final bisection step (below roundoff). |
| `phase_ff.rs:3205:26` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when a crossing's sampled distance to the oval equals the 0.1·r band exactly. |
| `phase_ff.rs:3205:87` | `trim_torus_oval_to_box_face` | replace < with == | (b) unkillable | Unkillable: one rim root falls in exactly one sample interval, and two roots closer than the 1e-5 band share one of the 720 rim intervals and cancel, so near-duplicates arise only from sub-band noise between rim arcs, where keeping both moves the kept arc by less than that noise. |
| `phase_ff.rs:3205:87` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when two crossings are exactly the 100·tol dedup distance apart. |
| `phase_ff.rs:3214:77` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only for a point at exactly the rim radius; kept-arc midpoints lie strictly between crossings. |
| `phase_ff.rs:6673:42` | `single_hit_circle_is_graze` | replace + with - | (b) equivalent | Circle evaluation is 2π-periodic: `t − π` and `t + π` give the same antipode. |
| `phase_ff.rs:7627:21` | `clip_line_to_face` | replace - with + | (b) equivalent | The plane frame is built at the circle center, so the projected center is exactly (0, 0) and `sx + cx == sx - cx`. |
| `phase_ff.rs:7628:21` | `clip_line_to_face` | replace - with + | (b) equivalent | As 7627:21, for the y coordinate. |

## Run provenance (disclosed)

- An intermediate after-run (at `f9f0a53`) left 20 missed. Six were fixture gaps, since closed:
  the rim's starting side next to the oval (3180:54 `==`/`>`), off-branch crossings inside the kept
  arc's span (3205:40), an oval domain starting below zero (3197:61), exact single-point touches of
  the disc's tolerance circle (7640:17, 7646:24), and a notched box face (3292:39 `/`→`%`, first
  filed as equivalent and then shown killable: a short excursion's second sample can fall inside the
  face's boundary margin).
- 3180:54 `<`→`>` was caught in CI but missed at `f9f0a53` locally; it is caught in the final run by
  the near-oval seams.
- `main` was merged after the final run (#644 adds a tangent-rim helper and edits
  `perform_with_context` near line 470). The mutated lines are untouched; line numbers here are the
  CI run's.

## Every survivor

| Mutant | Function | Mutation | Verdict | Evidence |
| --- | --- | --- | --- | --- |
| `builder_solid.rs:379:5` | `orient_revolved_face_wires` | replace orient_revolved_face_wires -> Result<(), AlgoError> with Ok(()) | (a) killed | `a_hole_in_a_pointed_cone_whose_rim_has_no_uv_area_is_still_rewound` |
| `builder_solid.rs:483:43` | `orient_revolved_face_wires` | replace match guard a.is_sign_positive() == b.is_sign_positive() with true | (a) killed | `revolved_holes_are_rewound_against_their_outer_loop` |
| `builder_solid.rs:483:43` | `orient_revolved_face_wires` | replace match guard a.is_sign_positive() == b.is_sign_positive() with false | (a) killed | `revolved_holes_are_rewound_against_their_outer_loop` |
| `builder_solid.rs:483:64` | `orient_revolved_face_wires` | replace == with != | (a) killed | `revolved_holes_are_rewound_against_their_outer_loop` |
| `builder_solid.rs:493:21` | `orient_revolved_face_wires` | delete match arm (None, _) | (a) killed | `a_hole_in_a_pointed_cone_whose_rim_has_no_uv_area_is_still_rewound` |
| `builder_solid.rs:496:33` | `orient_revolved_face_wires` | replace \|= with &= | (a) killed | `a_hole_in_a_pointed_cone_whose_rim_has_no_uv_area_is_still_rewound` |
| `builder_solid.rs:3154:5` | `conics_share_support` | replace conics_share_support -> bool with true | (a) killed | `conics_share_support_only_for_the_same_carrier_curve` |
| `builder_solid.rs:3709:5` | `closed_pair_traversal_flipped` | replace closed_pair_traversal_flipped -> Option<bool> with Some(false) | (a) killed | `closed_pair_traversal_is_flipped_exactly_when_the_rings_run_opposite` |
| `builder_solid.rs:3709:5` | `closed_pair_traversal_flipped` | replace closed_pair_traversal_flipped -> Option<bool> with Some(true) | (a) killed | `closed_pair_traversal_is_flipped_exactly_when_the_rings_run_opposite` |
| `builder_solid.rs:3709:5` | `closed_pair_traversal_flipped` | replace closed_pair_traversal_flipped -> Option<bool> with None | (a) killed | `closed_pair_traversal_is_flipped_exactly_when_the_rings_run_opposite` |
| `fill_images_faces.rs:1692:5` | `equal_radius_cylinder_pair` | replace equal_radius_cylinder_pair -> bool with false | (a) killed | `equal_radius_cylinder_pairs_are_recognised_by_radius_alone` |
| `fill_images_faces.rs:1995:5` | `cap_disc_circle` | replace cap_disc_circle -> Option<remus_math::curves::Circle3D> with None | (a) killed | `a_cap_disc_is_a_plane_bounded_by_one_closed_circle` |
| `fill_images_faces.rs:2569:5` | `circle_inside_face` | replace circle_inside_face -> Result<bool, AlgoError> with Ok(false) | (a) killed | `a_closed_section_is_inside_a_face_only_when_it_stays_within_its_extent` |
| `fill_images_faces.rs:2569:5` | `circle_inside_face` | replace circle_inside_face -> Result<bool, AlgoError> with Ok(true) | (a) killed | `a_closed_section_is_inside_a_face_only_when_it_stays_within_its_extent` |
| `fill_images_faces.rs:2642:5` | `closed_curve_coincides_with_boundary` | replace closed_curve_coincides_with_boundary -> bool with false | (a) killed | `a_closed_section_coincides_with_a_boundary_only_when_it_is_that_ring` |
| `fill_images_faces.rs:3330:5` | `segment_between_boundary_arcs` | replace segment_between_boundary_arcs -> bool with false | (a) killed | `a_generator_segment_is_in_a_wall_exactly_when_it_lies_between_the_rims` |
| `fill_images_faces.rs:3330:5` | `segment_between_boundary_arcs` | replace segment_between_boundary_arcs -> bool with true | (a) killed | `a_generator_segment_is_in_a_wall_exactly_when_it_lies_between_the_rims` |
| `fill_images_faces.rs:3342:77` | `segment_between_boundary_arcs` | replace + with * | (a) killed | `a_generator_segment_is_in_a_wall_exactly_when_it_lies_between_the_rims` |
| `fill_images_faces.rs:3342:77` | `segment_between_boundary_arcs` | replace + with - | (a) killed | `a_generator_segment_is_in_a_wall_exactly_when_it_lies_between_the_rims` |
| `phase_ff.rs:931:17` | `perform_with_context` | replace && with \|\| | (b) equivalent | Widens `torus_wall_pair`, but the split still requires `section_notches_one_rim`, whose own exhaustive pair match returns false for every pair except torus × cone/cylinder. |
| `phase_ff.rs:932:17` | `perform_with_context` | replace \|\| with && | (a) killed | `a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex` |
| `phase_ff.rs:933:21` | `perform_with_context` | replace && with \|\| | (b) equivalent | Same as 931:17: `section_notches_one_rim` re-checks the pair and rejects every non torus × wall pair. |
| `phase_ff.rs:938:63` | `perform_with_context` | replace > with == | (a) killed | `a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex` |
| `phase_ff.rs:938:63` | `perform_with_context` | replace > with >= | (b) equivalent | Differs only when a section's endpoint gap is exactly `tol.linear` (1e-7) in floating point. |
| `phase_ff.rs:938:63` | `perform_with_context` | replace > with < | (a) killed | `a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex` |
| `phase_ff.rs:3103:44` | `trim_torus_oval_to_box_face` | replace && with \|\| | (a) killed | `torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight` |
| `phase_ff.rs:3103:63` | `trim_torus_oval_to_box_face` | replace >= with < | (a) killed | `torus_oval_on_a_notched_box_face_drops_the_short_excursion_through_the_notch` |
| `phase_ff.rs:3104:41` | `trim_torus_oval_to_box_face` | replace && with \|\| | (a) killed | `torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight` |
| `phase_ff.rs:3104:44` | `trim_torus_oval_to_box_face` | delete ! | (a) killed | `torus_oval_wholly_inside_a_cap_is_kept_whole_and_wholly_outside_defers` |
| `phase_ff.rs:3104:65` | `trim_torus_oval_to_box_face` | replace && with \|\| | (a) killed | `torus_oval_on_a_notched_box_face_drops_the_short_excursion_through_the_notch` |
| `phase_ff.rs:3105:8` | `trim_torus_oval_to_box_face` | delete ! | (a) killed | `torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight` |
| `phase_ff.rs:3105:22` | `trim_torus_oval_to_box_face` | replace && with \|\| | (a) killed | `torus_oval_on_a_notched_box_face_drops_the_short_excursion_through_the_notch` |
| `phase_ff.rs:3105:25` | `trim_torus_oval_to_box_face` | delete ! | (a) killed | `torus_oval_wholly_inside_a_cap_is_kept_whole_and_wholly_outside_defers` |
| `phase_ff.rs:3153:52` | `trim_torus_oval_to_box_face` | replace <= with > | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3153:66` | `trim_torus_oval_to_box_face` | replace * with / | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3153:66` | `trim_torus_oval_to_box_face` | replace * with + | (a) killed | `torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight` |
| `phase_ff.rs:3154:17` | `trim_torus_oval_to_box_face` | replace && with \|\| | (a) killed | `torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight` |
| `phase_ff.rs:3154:32` | `trim_torus_oval_to_box_face` | replace - with / | (a) killed | `torus_oval_wholly_inside_a_cap_is_kept_whole_and_wholly_outside_defers` |
| `phase_ff.rs:3154:32` | `trim_torus_oval_to_box_face` | replace - with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3154:56` | `trim_torus_oval_to_box_face` | replace <= with > | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3154:70` | `trim_torus_oval_to_box_face` | replace * with / | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3154:70` | `trim_torus_oval_to_box_face` | replace * with + | (a) killed | `torus_oval_defers_on_outlines_that_are_not_one_rim_circle_or_all_straight` |
| `phase_ff.rs:3156:12` | `trim_torus_oval_to_box_face` | delete ! | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3173:25` | `trim_torus_oval_to_box_face` | replace - with + | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3173:60` | `trim_torus_oval_to_box_face` | replace - with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3173:60` | `trim_torus_oval_to_box_face` | replace - with / | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3174:38` | `trim_torus_oval_to_box_face` | replace * with / | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3174:38` | `trim_torus_oval_to_box_face` | replace * with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3174:47` | `trim_torus_oval_to_box_face` | replace - with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3174:47` | `trim_torus_oval_to_box_face` | replace - with / | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3174:70` | `trim_torus_oval_to_box_face` | replace * with / | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3174:70` | `trim_torus_oval_to_box_face` | replace * with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3180:54` | `trim_torus_oval_to_box_face` | replace < with == | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3180:54` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when the torus implicit is exactly 0.0 at the rim's start sample. |
| `phase_ff.rs:3182:25` | `trim_torus_oval_to_box_face` | replace - with / | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3182:25` | `trim_torus_oval_to_box_face` | replace - with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3182:52` | `trim_torus_oval_to_box_face` | replace / with % | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3182:52` | `trim_torus_oval_to_box_face` | replace / with * | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3183:48` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when the torus implicit is exactly 0.0 at a rim sample. |
| `phase_ff.rs:3183:48` | `trim_torus_oval_to_box_face` | replace < with == | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3183:48` | `trim_torus_oval_to_box_face` | replace < with > | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3184:23` | `trim_torus_oval_to_box_face` | replace != with == | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3188:49` | `trim_torus_oval_to_box_face` | replace < with > | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3188:49` | `trim_torus_oval_to_box_face` | replace < with == | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3188:49` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when the implicit is exactly 0.0 at a bisection midpoint, i.e. at the root itself, so the crossing moves by less than one final bisection step (below roundoff). |
| `phase_ff.rs:3188:56` | `trim_torus_oval_to_box_face` | replace == with != | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3197:44` | `trim_torus_oval_to_box_face` | replace + with * | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3197:44` | `trim_torus_oval_to_box_face` | replace + with - | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3197:61` | `trim_torus_oval_to_box_face` | replace - with / | (a) killed | `a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex` |
| `phase_ff.rs:3197:61` | `trim_torus_oval_to_box_face` | replace - with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3197:78` | `trim_torus_oval_to_box_face` | replace * with + | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3197:93` | `trim_torus_oval_to_box_face` | replace / with % | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3197:93` | `trim_torus_oval_to_box_face` | replace / with * | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3205:26` | `trim_torus_oval_to_box_face` | replace < with == | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3205:26` | `trim_torus_oval_to_box_face` | replace < with > | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3205:26` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when a crossing's sampled distance to the oval equals the 0.1·r band exactly. |
| `phase_ff.rs:3205:40` | `trim_torus_oval_to_box_face` | replace && with \|\| | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3205:43` | `trim_torus_oval_to_box_face` | delete ! | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3205:87` | `trim_torus_oval_to_box_face` | replace < with == | (b) unkillable | Unkillable: one rim root falls in exactly one sample interval, and two roots closer than the 1e-5 band share one of the 720 rim intervals and cancel, so near-duplicates arise only from sub-band noise between rim arcs, where keeping both moves the kept arc by less than that noise. |
| `phase_ff.rs:3205:87` | `trim_torus_oval_to_box_face` | replace < with > | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3205:87` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only when two crossings are exactly the 100·tol dedup distance apart. |
| `phase_ff.rs:3214:77` | `trim_torus_oval_to_box_face` | replace < with == | (a) killed | `torus_oval_on_a_disk_cap_keeps_the_exact_in_cap_arc_for_any_rim_seam` |
| `phase_ff.rs:3214:77` | `trim_torus_oval_to_box_face` | replace < with > | (a) killed | `torus_oval_wholly_inside_a_cap_is_kept_whole_and_wholly_outside_defers` |
| `phase_ff.rs:3214:77` | `trim_torus_oval_to_box_face` | replace < with <= | (b) equivalent | Differs only for a point at exactly the rim radius; kept-arc midpoints lie strictly between crossings. |
| `phase_ff.rs:3292:12` | `trim_torus_oval_to_box_face` | delete ! | (a) killed | `torus_oval_on_a_disk_cap_tolerates_weld_scale_noise_between_rim_arcs` |
| `phase_ff.rs:3292:39` | `trim_torus_oval_to_box_face` | replace / with * | (a) killed | `a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex` |
| `phase_ff.rs:3292:39` | `trim_torus_oval_to_box_face` | replace / with % | (a) killed | `torus_oval_on_a_notched_box_face_drops_the_short_excursion_through_the_notch` |
| `phase_ff.rs:5075:5` | `section_notches_one_rim` | replace section_notches_one_rim -> bool with false | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5075:5` | `section_notches_one_rim` | replace section_notches_one_rim -> bool with true | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5112:27` | `section_notches_one_rim` | replace * with / | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5112:27` | `section_notches_one_rim` | replace * with + | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5115:26` | `section_notches_one_rim` | replace - with / | (a) killed | `a_torus_section_notching_one_frustum_rim_is_split_at_an_interior_vertex` |
| `phase_ff.rs:5115:26` | `section_notches_one_rim` | replace - with + | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5115:41` | `section_notches_one_rim` | replace <= with > | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5115:49` | `section_notches_one_rim` | replace && with \|\| | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5115:56` | `section_notches_one_rim` | replace - with + | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:5115:71` | `section_notches_one_rim` | replace <= with > | (a) killed | `a_section_notches_one_rim_only_when_both_ends_sit_on_the_same_band_bound` |
| `phase_ff.rs:6673:42` | `single_hit_circle_is_graze` | replace + with * | (a) killed | `one_hit_circle_on_a_sliver_face_honors_the_weld_band_not_just_the_margin` |
| `phase_ff.rs:6673:42` | `single_hit_circle_is_graze` | replace + with - | (b) equivalent | Circle evaluation is 2π-periodic: `t − π` and `t + π` give the same antipode. |
| `phase_ff.rs:6678:45` | `single_hit_circle_is_graze` | replace + with * | (a) killed | `one_hit_circle_on_a_sliver_face_honors_the_weld_band_not_just_the_margin` |
| `phase_ff.rs:6678:45` | `single_hit_circle_is_graze` | replace + with - | (a) killed | `one_hit_circle_on_a_sliver_face_honors_the_weld_band_not_just_the_margin` |
| `phase_ff.rs:6679:39` | `single_hit_circle_is_graze` | replace && with \|\| | (a) killed | `one_hit_circle_on_a_sliver_face_honors_the_weld_band_not_just_the_margin` |
| `phase_ff.rs:7606:31` | `clip_line_to_face` | replace == with != | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7609:25` | `clip_line_to_face` | replace == with != | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7622:37` | `clip_line_to_face` | replace + with - | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7622:37` | `clip_line_to_face` | replace + with * | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7625:21` | `clip_line_to_face` | replace - with + | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7626:21` | `clip_line_to_face` | replace - with + | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7626:21` | `clip_line_to_face` | replace - with / | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7627:21` | `clip_line_to_face` | replace - with + | (b) equivalent | The plane frame is built at the circle center, so the projected center is exactly (0, 0) and `sx + cx == sx - cx`. |
| `phase_ff.rs:7627:21` | `clip_line_to_face` | replace - with / | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7628:21` | `clip_line_to_face` | replace - with / | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7628:21` | `clip_line_to_face` | replace - with + | (b) equivalent | As 7627:21, for the y coordinate. |
| `phase_ff.rs:7629:20` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7629:20` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7629:25` | `clip_line_to_face` | replace + with * | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7629:25` | `clip_line_to_face` | replace + with - | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7629:30` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7629:30` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7631:14` | `clip_line_to_face` | replace <= with > | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:19` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:19` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:24` | `clip_line_to_face` | replace + with - | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:24` | `clip_line_to_face` | replace + with * | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:29` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:34` | `clip_line_to_face` | replace <= with > | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:43` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7632:43` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_decides_point_segments_by_distance_to_the_center` |
| `phase_ff.rs:7637:21` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7637:21` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7637:27` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7637:27` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7637:32` | `clip_line_to_face` | replace + with - | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7637:32` | `clip_line_to_face` | replace + with * | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7637:37` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7637:37` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7638:20` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7638:20` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7638:25` | `clip_line_to_face` | replace + with * | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7638:25` | `clip_line_to_face` | replace + with - | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7638:30` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7638:35` | `clip_line_to_face` | replace - with + | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7638:35` | `clip_line_to_face` | replace - with / | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7638:43` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7638:43` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7639:22` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7639:22` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7639:26` | `clip_line_to_face` | replace - with + | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7639:26` | `clip_line_to_face` | replace - with / | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7639:32` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7639:32` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7639:36` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7639:36` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7640:17` | `clip_line_to_face` | replace < with == | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7640:17` | `clip_line_to_face` | replace < with <= | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7640:17` | `clip_line_to_face` | replace < with > | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7644:19` | `clip_line_to_face` | delete - | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7644:22` | `clip_line_to_face` | replace - with / | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7644:22` | `clip_line_to_face` | replace - with + | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7644:28` | `clip_line_to_face` | replace / with * | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7644:28` | `clip_line_to_face` | replace / with % | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7644:35` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7645:19` | `clip_line_to_face` | delete - | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7645:22` | `clip_line_to_face` | replace + with - | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7645:22` | `clip_line_to_face` | replace + with * | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7645:28` | `clip_line_to_face` | replace / with % | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7645:28` | `clip_line_to_face` | replace / with * | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7645:35` | `clip_line_to_face` | replace * with / | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7645:35` | `clip_line_to_face` | replace * with + | (a) killed | `disc_clip_matches_the_distance_oracle_on_generated_segments` |
| `phase_ff.rs:7646:24` | `clip_line_to_face` | replace < with <= | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
| `phase_ff.rs:7646:24` | `clip_line_to_face` | replace < with > | (a) killed | `disc_clip_drops_lines_that_miss_and_keeps_lines_that_touch` |
| `phase_ff.rs:7646:24` | `clip_line_to_face` | replace < with == | (a) killed | `disc_clip_keeps_segments_that_touch_the_tolerance_circle_at_one_point` |
