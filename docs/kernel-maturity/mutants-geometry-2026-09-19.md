# remus-geometry mutation triage — 2026-09-19

One row per survivor, for every file in the crate that had one.

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly workflow's pinned version), Rust 1.96.0 toolchain family.
- `.cargo/mutants.toml` `examine_globs` cover only math/algo/blend/offset/operations, so
  `cargo mutants -p remus-geometry` examines **zero** mutants. This campaign overrides discovery with
  `--no-config` (local runs only; the committed CI scope is unchanged).
- Test scope is crate-only: `-- -p remus-geometry` (lib + the four `tests/` targets). Downstream
  crates' suites were NOT run per mutant, so these survivor counts over-approximate the
  workspace-scope survivors the weekly CI would report.
- `--baseline skip` (the crate suite was verified green first). `--timeout 60` on the runs that
  needed it: several `point_surface` and `lipschitz` mutations break a convergence loop and hang.
- `--in-place` was NOT used: cargo-mutants 27 rejects `--in-place` together with `--jobs`, and the
  parallelism was worth more than the shared build cache. Runs used `-j 4` / `-j 8` over copied trees.
- Verdicts: (a) missing assertion — new unit test fails under the mutant, passes on real code;
  (b) equivalent — one-line reason; (c) needs geometry judgment; timeout — mutant hangs the suite.

## Run provenance (disclosed)

- The before-sweep was interrupted twice by the harness's background-task limit. It is therefore the
  merge of two runs, joined by mutant name: the main run (`geom-base`, all files except a partial
  `extrema/segment.rs`) plus a completion run (`geom-rest2`, `extrema/segment.rs` and the five
  `sampling/*` files). The partial `segment.rs` rows from the first run were discarded, not merged.
- One mutant, `extrema/point_surface.rs:490:52: replace * with + in point_to_surface`, was never
  tested on the pristine tree (the first run died between its build and its test). It is **excluded
  from the "before" survivor count** — before totals are over 3003 of the crate's 3004 mutants. It is
  caught on the final tree, but is not counted as a kill, because its pre-state is unknown.
- Per-file triage was fanned out to one agent per file, all sharing one worktree. A sibling's
  work-in-progress can be copied into a concurrent mutants run and make unrelated tests red, which
  marks mutants "caught" spuriously; several agents' per-file after-runs were affected. **Those
  per-file runs are not the numbers reported here.** Every "after" figure below comes from one
  authoritative full-crate re-sweep of the final committed tree.
- Cross-check: of the 741 mutants classified (a), **741 are killed** in the authoritative re-sweep and
  0 still survive; of the 397 classified (b)/(c)/timeout, **0** were killed. Verdicts and measurements
  agree exactly, and there are 0 new survivors relative to the before-sweep.

## Before / after per file (measured)

| File | Mutants | Survivors before | Survivors after | Killed | (a) | (b) | (c) | timeout |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `convert/recognize_surface.rs` | 725 | 306 | 109 | 197 | 197 | 104 | 5 | 0 |
| `convert/recognize_curve.rs` | 792 | 120 | 69 | 51 | 51 | 49 | 20 | 0 |
| `convert/curve_to_nurbs.rs` | 116 | 47 | 11 | 36 | 36 | 11 | 0 | 0 |
| `convert/surface_to_nurbs.rs` | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| `extrema/point_surface.rs` | 426 | 273 | 51 | 222 | 222 | 37 | 14 | 0 |
| `extrema/lipschitz.rs` | 267 | 144 | 62 | 82 | 82 | 29 | 33 | 0 |
| `extrema/curve_curve.rs` | 216 | 77 | 53 | 24 | 24 | 38 | 15 | 0 |
| `extrema/point_curve.rs` | 128 | 27 | 9 | 18 | 18 | 9 | 0 | 0 |
| `extrema/segment.rs` | 83 | 27 | 5 | 22 | 22 | 5 | 0 | 0 |
| `sampling/arc_length.rs` | 106 | 53 | 24 | 29 | 29 | 24 | 0 | 0 |
| `sampling/curvature.rs` | 56 | 30 | 3 | 27 | 27 | 2 | 0 | 1 |
| `sampling/surface.rs` | 34 | 20 | 0 | 20 | 20 | 0 | 0 | 0 |
| `sampling/uniform.rs` | 21 | 10 | 0 | 10 | 10 | 0 | 0 | 0 |
| `sampling/deflection.rs` | 30 | 4 | 1 | 3 | 3 | 1 | 0 | 0 |
| **Total** | **3004** | **1138** | **397** | **741** | **741** | **309** | **87** | **1** |

Crate baseline: 3004 mutants, 1138 survivors over 3003 tested (37.9% miss rate under crate-scoped
tests). 741 killed by 119 new unit tests (lib suite 107 -> 226). No production code changed; no
existing assertion weakened or removed — every hunk lands inside an existing `#[cfg(test)] mod tests`.

## crates/geometry/src/convert/recognize_surface.rs

- before: 306 survivors; after: 109 survivors, 528 caught, 88 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/convert/recognize_surface.rs:112:23: replace \|\| with && in try_recognize_plane | (b) | `cps` and `cps[0]` are never empty for a constructible `NurbsSurface` (degree 0 is rejected), so both operands are false either way |
| crates/geometry/src/convert/recognize_surface.rs:124:22: replace < with == in try_recognize_plane | (b) | the smallest constructible control grid is 2x2 = 4 points, so `all_pts.len()` is never 3 or fewer |
| crates/geometry/src/convert/recognize_surface.rs:124:22: replace < with <= in try_recognize_plane | (b) | the smallest constructible control grid is 2x2 = 4 points, so `all_pts.len()` is never 3 or fewer |
| crates/geometry/src/convert/recognize_surface.rs:133:41: replace + with - in try_recognize_plane | (b) | the extra pairs the mutated `skip` visits were already tried at an earlier outer index and found degenerate, so the same first non-degenerate normal is found |
| crates/geometry/src/convert/recognize_surface.rs:133:41: replace + with * in try_recognize_plane | (b) | the extra pairs the mutated `skip` visits were already tried at an earlier outer index and found degenerate, so the same first non-degenerate normal is found |
| crates/geometry/src/convert/recognize_surface.rs:136:27: replace > with >= in try_recognize_plane | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:150:61: replace - with + in try_recognize_plane | (a) | recognized_plane_off_origin_matches_the_analytic_plane |
| crates/geometry/src/convert/recognize_surface.rs:150:61: replace - with / in try_recognize_plane | (a) | recognized_plane_off_origin_matches_the_analytic_plane |
| crates/geometry/src/convert/recognize_surface.rs:151:23: replace > with >= in try_recognize_plane | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:167:28: replace match guard n.dot(du_cross_dv) < 0.0 with true in try_recognize_plane | (a) | recognized_plane_off_origin_matches_the_analytic_plane |
| crates/geometry/src/convert/recognize_surface.rs:166:30: replace * with + in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:30: replace * with / in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:36: replace + with - in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:36: replace + with * in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:47: replace * with + in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:47: replace * with / in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:53: replace + with - in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:166:53: replace + with * in try_recognize_plane | (b) | the surface is already known planar at this point, so du x dv has a constant direction over the whole domain; moving the sample parameter cannot change the sign test |
| crates/geometry/src/convert/recognize_surface.rs:167:47: replace < with <= in try_recognize_plane | (b) | differs only when the recognized normal is exactly perpendicular to du x dv, impossible for a recognized plane |
| crates/geometry/src/convert/recognize_surface.rs:167:66: delete - in try_recognize_plane | (a) | recognized_plane_off_origin_matches_the_analytic_plane |
| crates/geometry/src/convert/recognize_surface.rs:185:18: replace < with == in try_recognize_cylinder | (b) | `NurbsSurface::new` rejects degree 0, so the smallest grid is 2x2; a 2-row grid cannot represent a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:185:18: replace < with <= in try_recognize_cylinder | (b) | `NurbsSurface::new` rejects degree 0, so the smallest grid is 2x2; a 2-row grid cannot represent a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:189:22: replace < with > in try_recognize_cylinder | (a) | recognize_cylinder_with_three_control_point_columns |
| crates/geometry/src/convert/recognize_surface.rs:198:18: replace += with -= in try_recognize_cylinder | (a) | recognize_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:201:36: replace / with % in try_recognize_cylinder | (b) | the averaged axis vector is immediately normalized, so any positive rescale is invisible; the only other use is a magnitude threshold a non-degenerate axis clears either way |
| crates/geometry/src/convert/recognize_surface.rs:201:36: replace / with * in try_recognize_cylinder | (b) | the averaged axis vector is immediately normalized, so any positive rescale is invisible; the only other use is a magnitude threshold a non-degenerate axis clears either way |
| crates/geometry/src/convert/recognize_surface.rs:203:17: replace < with == in try_recognize_cylinder | (b) | differs only at exact equality with the tolerance; a degenerate axis is rejected by `normalize` on the next line anyway |
| crates/geometry/src/convert/recognize_surface.rs:203:17: replace < with <= in try_recognize_cylinder | (b) | differs only at exact equality with the tolerance; a degenerate axis is rejected by `normalize` on the next line anyway |
| crates/geometry/src/convert/recognize_surface.rs:219:32: replace * with / in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:219:26: replace - with + in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:219:52: replace - with + in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:219:52: replace - with / in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:222:50: replace / with % in try_recognize_cylinder | (a) | recognize_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:222:50: replace / with * in try_recognize_cylinder | (a) | recognize_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:222:36: replace * with / in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:222:30: replace - with + in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:222:56: replace - with + in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:236:39: replace < with == in try_recognize_cylinder | (b) | the else branch picks an equally valid perpendicular seed, and the least-squares circle fit is invariant under rotation of the 2D frame |
| crates/geometry/src/convert/recognize_surface.rs:222:56: replace - with / in try_recognize_cylinder | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:236:39: replace < with > in try_recognize_cylinder | (a) | recognize_x_axis_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:236:39: replace < with <= in try_recognize_cylinder | (b) | the else branch picks an equally valid perpendicular seed, and the least-squares circle fit is invariant under rotation of the 2D frame |
| crates/geometry/src/convert/recognize_surface.rs:241:23: replace - with + in try_recognize_cylinder | (a) | recognize_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:261:24: replace * with + in try_recognize_cylinder | (a) | recognize_exact_cylinder_patch_with_both_centre_coordinates_non_zero |
| crates/geometry/src/convert/recognize_surface.rs:274:25: replace + with - in try_recognize_cylinder | (a) | recognize_exact_cylinder_patch_with_both_centre_coordinates_non_zero |
| crates/geometry/src/convert/recognize_surface.rs:292:20: replace < with == in try_recognize_cylinder | (b) | differs only at exact equality with the tolerance, or on a degenerate zero-radius fit no cylinder can produce |
| crates/geometry/src/convert/recognize_surface.rs:292:20: replace < with <= in try_recognize_cylinder | (b) | differs only at exact equality with the tolerance, or on a degenerate zero-radius fit no cylinder can produce |
| crates/geometry/src/convert/recognize_surface.rs:300:16: replace > with >= in try_recognize_cylinder | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:323:32: replace * with / in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:323:26: replace - with + in try_recognize_sphere | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:323:52: replace - with + in try_recognize_sphere | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:323:52: replace - with / in try_recognize_sphere | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:326:30: replace - with + in try_recognize_sphere | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:326:56: replace - with / in try_recognize_sphere | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:326:56: replace - with + in try_recognize_sphere | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:331:22: replace < with == in try_recognize_sphere | (b) | the sample grid is always 8x8 = 64 points, so `samples.len()` is never 4 or fewer |
| crates/geometry/src/convert/recognize_surface.rs:331:22: replace < with <= in try_recognize_sphere | (b) | the sample grid is always 8x8 = 64 points, so `samples.len()` is never 4 or fewer |
| crates/geometry/src/convert/recognize_surface.rs:350:27: replace - with + in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:351:27: replace - with + in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:352:27: replace - with + in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:358:27: replace += with -= in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:360:20: replace += with -= in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:360:20: replace += with *= in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:370:20: replace - with + in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:371:20: replace - with + in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:372:20: replace - with + in try_recognize_sphere | (a) | recognize_off_origin_sphere, recognize_sampled_off_origin_sphere_within_chord_error |
| crates/geometry/src/convert/recognize_surface.rs:382:20: replace < with == in try_recognize_sphere | (b) | differs only at exact equality with the tolerance, or on a degenerate zero-radius fit no sphere can produce |
| crates/geometry/src/convert/recognize_surface.rs:382:20: replace < with <= in try_recognize_sphere | (b) | differs only at exact equality with the tolerance, or on a degenerate zero-radius fit no sphere can produce |
| crates/geometry/src/convert/recognize_surface.rs:391:16: replace > with >= in try_recognize_sphere | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:417:18: replace < with == in try_recognize_cone | (b) | `NurbsSurface::new` rejects degree 0, so the smallest grid is 2x2; a 2-row grid cannot represent a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:421:22: replace < with == in try_recognize_cone | (a) | recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:417:18: replace < with <= in try_recognize_cone | (b) | `NurbsSurface::new` rejects degree 0, so the smallest grid is 2x2; a 2-row grid cannot represent a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:421:22: replace < with <= in try_recognize_cone | (a) | recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:435:36: replace && with \|\| in try_recognize_cone | (b) | for every recognizable cone grid both operands are true, so `&&` and `\|\|` agree |
| crates/geometry/src/convert/recognize_surface.rs:435:81: replace < with <= in try_recognize_cone | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:443:18: replace += with -= in try_recognize_cone | (b) | negating the estimated axis negates `axials` and hence `slope`; apex, half-angle and the final `if slope > 0.0` flip compensate exactly |
| crates/geometry/src/convert/recognize_surface.rs:446:36: replace / with % in try_recognize_cone | (b) | the averaged axis vector is immediately normalized, so any positive rescale is invisible; the only other use is a magnitude threshold a non-degenerate axis clears either way |
| crates/geometry/src/convert/recognize_surface.rs:446:36: replace / with * in try_recognize_cone | (b) | the averaged axis vector is immediately normalized, so any positive rescale is invisible; the only other use is a magnitude threshold a non-degenerate axis clears either way |
| crates/geometry/src/convert/recognize_surface.rs:447:26: replace < with == in try_recognize_cone | (b) | differs only at exact equality with the tolerance; a degenerate axis is rejected by `normalize` on the next line anyway |
| crates/geometry/src/convert/recognize_surface.rs:447:26: replace < with <= in try_recognize_cone | (b) | differs only at exact equality with the tolerance; a degenerate axis is rejected by `normalize` on the next line anyway |
| crates/geometry/src/convert/recognize_surface.rs:462:26: replace - with + in try_recognize_cone | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:465:50: replace / with % in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:465:50: replace / with * in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:465:36: replace * with / in try_recognize_cone | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:465:30: replace - with + in try_recognize_cone | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:465:56: replace - with + in try_recognize_cone | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:465:56: replace - with / in try_recognize_cone | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:477:21: replace / with % in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:477:21: replace / with * in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:482:18: replace += with -= in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:482:18: replace += with *= in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:483:18: replace += with -= in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:483:18: replace += with *= in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:484:18: replace += with -= in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:484:18: replace += with *= in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:486:39: replace * with + in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:486:39: replace * with / in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:486:57: replace * with + in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:486:57: replace * with / in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:486:75: replace * with + in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:486:75: replace * with / in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:507:22: replace < with == in try_recognize_cone | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:507:22: replace < with <= in try_recognize_cone | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:507:14: replace - with + in try_recognize_cone | (b) | the mutated value stays far above the tolerance for any cone with a non-degenerate radius range, so the degeneracy guard behaves identically |
| crates/geometry/src/convert/recognize_surface.rs:507:14: replace - with / in try_recognize_cone | (b) | the mutated value stays far above the tolerance for any cone with a non-degenerate radius range, so the degeneracy guard behaves identically |
| crates/geometry/src/convert/recognize_surface.rs:524:24: replace / with % in try_recognize_cone | (b) | the anchor is the sample centroid, so `sum_a` is exactly 0 and `mean_a` is 0 under every mutated operator |
| crates/geometry/src/convert/recognize_surface.rs:524:24: replace / with * in try_recognize_cone | (b) | the anchor is the sample centroid, so `sum_a` is exactly 0 and `mean_a` is 0 under every mutated operator |
| crates/geometry/src/convert/recognize_surface.rs:529:28: replace - with + in try_recognize_cone | (b) | `mean_a` is exactly 0 because the anchor is the sample centroid, so subtracting and adding it are identical |
| crates/geometry/src/convert/recognize_surface.rs:530:29: replace - with + in try_recognize_cone | (b) | `Sum da` is exactly 0, so the constant shift of `dr` cancels out of `s_ar` and the fitted slope is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:534:13: replace < with == in try_recognize_cone | (b) | differs only at exact equality with 1e-30, or on a zero-variance fit no cone can produce |
| crates/geometry/src/convert/recognize_surface.rs:534:13: replace < with <= in try_recognize_cone | (b) | differs only at exact equality with 1e-30, or on a zero-variance fit no cone can produce |
| crates/geometry/src/convert/recognize_surface.rs:538:28: replace - with + in try_recognize_cone | (b) | `mean_a` is exactly 0, so `mean_r - slope*mean_a` and `mean_r + slope*mean_a` are identical |
| crates/geometry/src/convert/recognize_surface.rs:539:20: replace < with == in try_recognize_cone | (b) | differs only at exact equality with the tolerance; a near-zero slope is a cylinder, which is recognized earlier |
| crates/geometry/src/convert/recognize_surface.rs:539:20: replace < with <= in try_recognize_cone | (b) | differs only at exact equality with the tolerance; a near-zero slope is a cylinder, which is recognized earlier |
| crates/geometry/src/convert/recognize_surface.rs:548:38: replace > with == in try_recognize_cone | (a) | recognize_tilted_off_origin_cone, recognize_exact_cone_patch_on_a_two_column_grid |
| crates/geometry/src/convert/recognize_surface.rs:548:38: replace > with >= in try_recognize_cone | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:562:27: replace && with \|\| in try_recognize_cone | (b) | `atan` of a positive finite slope is always in (0, pi/2), so both operands are true and `&&` and `\|\|` agree |
| crates/geometry/src/convert/recognize_surface.rs:562:14: replace < with <= in try_recognize_cone | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:562:41: replace < with <= in try_recognize_cone | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:573:30: replace > with == in try_recognize_cone | (a) | recognize_cone_sampled_with_decreasing_v |
| crates/geometry/src/convert/recognize_surface.rs:573:30: replace > with < in try_recognize_cone | (a) | recognize_cone_sampled_with_decreasing_v |
| crates/geometry/src/convert/recognize_surface.rs:573:30: replace > with >= in try_recognize_cone | (b) | differs only when the slope is exactly 0, which the `slope.abs() < tolerance` guard above already rejected |
| crates/geometry/src/convert/recognize_surface.rs:573:52: delete - in try_recognize_cone | (a) | recognize_cone_sampled_with_decreasing_v |
| crates/geometry/src/convert/recognize_surface.rs:601:18: replace < with == in try_recognize_torus | (b) | `NurbsSurface::new` rejects degree 0, so the smallest grid is 2x2; a 2-row grid cannot represent a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:601:18: replace < with <= in try_recognize_torus | (b) | `NurbsSurface::new` rejects degree 0, so the smallest grid is 2x2; a 2-row grid cannot represent a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:605:22: replace < with == in try_recognize_torus | (b) | a 2-column grid cannot represent the closed minor circle of a torus, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:605:22: replace < with <= in try_recognize_torus | (b) | a 2-column grid cannot represent the closed minor circle of a torus, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:620:15: replace < with == in try_recognize_torus | (b) | 3 rows is the minimum the axis search needs and cannot describe a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:620:15: replace < with <= in try_recognize_torus | (b) | 3 rows is the minimum the axis search needs and cannot describe a revolution, so the guard's outcome is unchanged |
| crates/geometry/src/convert/recognize_surface.rs:627:21: replace + with - in try_recognize_torus | (b) | the extra pairs the mutated `skip` visits were already tried at an earlier outer index and found degenerate, so the same first non-degenerate normal is found |
| crates/geometry/src/convert/recognize_surface.rs:627:21: replace + with * in try_recognize_torus | (b) | the extra pairs the mutated `skip` visits were already tried at an earlier outer index and found degenerate, so the same first non-degenerate normal is found |
| crates/geometry/src/convert/recognize_surface.rs:630:31: replace > with < in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:630:31: replace > with >= in try_recognize_torus | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:646:26: replace - with + in try_recognize_torus | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:649:30: replace - with + in try_recognize_torus | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:649:56: replace - with + in try_recognize_torus | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:649:56: replace - with / in try_recognize_torus | (b) | every (u,v) of this analytic surface evaluates to a point ON it, so changing which points are sampled cannot change the fit; the domain is [0,1], where `u1 - u0` equals `u1 + u0` |
| crates/geometry/src/convert/recognize_surface.rs:656:21: replace / with % in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:656:21: replace / with * in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:661:12: replace += with -= in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:661:12: replace += with *= in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:662:12: replace += with -= in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:662:12: replace += with *= in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:663:12: replace += with -= in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:663:12: replace += with *= in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:665:33: replace * with + in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:665:33: replace * with / in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:665:45: replace * with + in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:665:45: replace * with / in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:665:57: replace * with + in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:665:57: replace * with / in try_recognize_torus | (a) | recognize_tilted_off_origin_torus, recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:673:28: replace - with + in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:686:24: replace * with + in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:686:24: replace * with / in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:699:18: replace + with - in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:699:33: replace * with + in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:700:20: replace \|\| with && in try_recognize_torus | (b) | for a valid torus both operands are false; when only one is true the residual check below rejects the surface anyway |
| crates/geometry/src/convert/recognize_surface.rs:706:37: replace - with + in try_recognize_torus | (c) | separable only with a horn torus (minor ~ major); whether the recognizer should accept one is an open geometry question |
| crates/geometry/src/convert/recognize_surface.rs:706:37: replace - with / in try_recognize_torus | (c) | separable only with a horn torus (minor ~ major); whether the recognizer should accept one is an open geometry question |
| crates/geometry/src/convert/recognize_surface.rs:712:28: replace - with + in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:715:40: replace > with == in try_recognize_torus | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:715:40: replace > with >= in try_recognize_torus | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:720:25: replace + with - in try_recognize_torus | (a) | recognize_exact_torus_patch_off_the_major_circle_plane |
| crates/geometry/src/convert/recognize_surface.rs:733:9: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:733:40: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:733:40: replace - with / in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:733:30: replace * with / in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:736:18: replace < with == in solve_3x3 | (a) | solve_3x3_rejects_a_singular_system |
| crates/geometry/src/convert/recognize_surface.rs:736:18: replace < with <= in solve_3x3 | (b) | differs only when the determinant's magnitude is exactly 1e-30 |
| crates/geometry/src/convert/recognize_surface.rs:744:9: replace + with - in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:744:9: replace + with * in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:743:9: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:743:37: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:743:37: replace - with / in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:743:27: replace * with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:743:27: replace * with / in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:743:47: replace * with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:744:37: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:744:37: replace - with / in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:749:9: replace + with - in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:749:37: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:749:30: replace * with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:753:9: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:753:37: replace - with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:753:30: replace * with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:753:44: replace * with + in solve_3x3 | (a) | solve_3x3_recovers_a_known_dense_solution |
| crates/geometry/src/convert/recognize_surface.rs:780:9: replace DetectedSurfaceKind::as_str -> &'static str with "" | (a) | detected_surface_kind_tags |
| crates/geometry/src/convert/recognize_surface.rs:780:9: replace DetectedSurfaceKind::as_str -> &'static str with "xyzzy" | (a) | detected_surface_kind_tags |
| crates/geometry/src/convert/recognize_surface.rs:806:27: replace + with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:27: replace + with - in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:58: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:58: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:45: replace * with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:45: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:36: replace - with + in detect_surface_kind | (b) | the parameter domain is [0,1], where `u_max - u_min` equals `u_max + u_min` |
| crates/geometry/src/convert/recognize_surface.rs:806:36: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:64: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:806:64: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:27: replace + with - in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:27: replace + with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:58: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:58: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:45: replace * with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:45: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:36: replace - with + in detect_surface_kind | (b) | the parameter domain is [0,1], where `v_max - v_min` equals `v_max + v_min` |
| crates/geometry/src/convert/recognize_surface.rs:807:36: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:64: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:807:64: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:817:12: replace += with -= in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:817:12: replace += with *= in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:818:12: replace += with -= in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:818:12: replace += with *= in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:819:12: replace += with -= in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:819:12: replace += with *= in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:822:33: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:822:33: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:822:42: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:822:42: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:822:51: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:822:51: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane, detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:828:21: replace + with - in detect_surface_kind | (b) | the extra pairs give either a zero cross product or the same plane normal up to sign, and the normal is only used through `.abs()` |
| crates/geometry/src/convert/recognize_surface.rs:828:21: replace + with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane |
| crates/geometry/src/convert/recognize_surface.rs:844:54: replace < with == in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane |
| crates/geometry/src/convert/recognize_surface.rs:844:54: replace < with > in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_plane |
| crates/geometry/src/convert/recognize_surface.rs:844:54: replace < with <= in detect_surface_kind | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:852:50: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:852:50: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:854:17: replace < with == in detect_surface_kind | (c) | separable only by pinning the undocumented 1e-10 degeneracy threshold; a patch small enough to reach it is already classified by the planar test above |
| crates/geometry/src/convert/recognize_surface.rs:854:17: replace < with > in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:854:17: replace < with <= in detect_surface_kind | (c) | separable only by pinning the undocumented 1e-10 degeneracy threshold; a patch small enough to reach it is already classified by the planar test above |
| crates/geometry/src/convert/recognize_surface.rs:858:24: replace * with + in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:858:24: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:859:67: replace < with == in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:859:67: replace < with > in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:859:67: replace < with <= in detect_surface_kind | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:859:49: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:859:49: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_off_origin_sphere, detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:873:27: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:873:27: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:873:42: replace * with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:873:42: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:874:27: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:874:27: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:874:42: replace * with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:874:42: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:875:27: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:875:27: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:875:42: replace * with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:875:42: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:881:61: replace / with % in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:882:18: replace > with == in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:882:18: replace > with < in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:881:61: replace / with * in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:882:18: replace > with >= in detect_surface_kind | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:883:31: replace * with + in detect_surface_kind | (c) | separable only by pinning the undocumented 0.1% relative tolerance against an absolute one, which the campaign's numerical rule forbids |
| crates/geometry/src/convert/recognize_surface.rs:883:31: replace * with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:886:44: replace < with == in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:886:44: replace < with > in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:886:29: replace - with + in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:886:29: replace - with / in detect_surface_kind | (a) | detect_surface_kind_of_tilted_off_origin_cylinder, detect_surface_kind_of_cone_is_bspline |
| crates/geometry/src/convert/recognize_surface.rs:886:44: replace < with <= in detect_surface_kind | (b) | differs only when the two compared floats are exactly equal, which no input the contract admits can produce |
| crates/geometry/src/convert/recognize_surface.rs:900:5: replace estimate_cylinder_axis -> Option<Vec3> with None | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:908:24: replace - with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:908:24: replace - with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:909:24: replace - with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:909:24: replace - with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:910:24: replace - with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:910:24: replace - with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:911:13: replace += with -= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:911:13: replace += with *= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:911:19: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:912:13: replace += with -= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:911:19: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:912:13: replace += with *= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:912:19: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:912:19: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:913:13: replace += with -= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:913:13: replace += with *= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:913:19: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:913:19: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:914:13: replace += with -= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:914:13: replace += with *= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:914:19: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:914:19: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:915:13: replace += with -= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:915:13: replace += with *= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:915:19: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:915:19: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:916:13: replace += with -= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:916:13: replace += with *= in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:916:19: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:916:19: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:923:57: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:923:57: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:924:57: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:924:57: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:925:57: replace * with + in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:925:57: replace * with / in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:928:16: replace < with == in estimate_cylinder_axis | (b) | differs only at exact equality with 1e-15, or on a zero covariance matrix that yields no axis either way |
| crates/geometry/src/convert/recognize_surface.rs:928:16: replace < with > in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:928:16: replace < with <= in estimate_cylinder_axis | (b) | differs only at exact equality with 1e-15, or on a zero covariance matrix that yields no axis either way |
| crates/geometry/src/convert/recognize_surface.rs:931:33: replace / with % in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:931:33: replace / with * in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:931:50: replace / with % in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:931:50: replace / with * in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:931:67: replace / with % in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |
| crates/geometry/src/convert/recognize_surface.rs:931:67: replace / with * in estimate_cylinder_axis | (a) | detect_surface_kind_of_tilted_off_origin_cylinder |

## crates/geometry/src/convert/recognize_curve.rs

- before: 120 survivors; after: 69 survivors, 610 caught, 113 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/convert/recognize_curve.rs:97:26: replace \|\| with && in normalized_conic_discriminant | (a) | degenerate_quadratic_part_has_no_discriminant |
| crates/geometry/src/convert/recognize_curve.rs:129:55: replace - with + in recognize_curve | (a) | recognize_curve_samples_the_whole_parameter_domain |
| crates/geometry/src/convert/recognize_curve.rs:129:55: replace - with / in recognize_curve | (a) | recognize_curve_samples_the_whole_parameter_domain |
| crates/geometry/src/convert/recognize_curve.rs:186:22: replace < with == in try_recognize_line | (a) | every_recognizer_rejects_an_empty_sample_set |
| crates/geometry/src/convert/recognize_curve.rs:186:22: replace < with <= in try_recognize_line | (a) | line_accepts_its_two_sample_minimum |
| crates/geometry/src/convert/recognize_curve.rs:195:12: replace < with == in try_recognize_line | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:195:12: replace < with <= in try_recognize_line | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:205:17: replace > with >= in try_recognize_line | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:222:22: replace < with == in try_recognize_circle | (a) | every_recognizer_rejects_an_empty_sample_set |
| crates/geometry/src/convert/recognize_curve.rs:222:22: replace < with <= in try_recognize_circle | (a) | circle_accepts_its_three_sample_minimum |
| crates/geometry/src/convert/recognize_curve.rs:231:41: replace + with - in try_recognize_circle | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:231:41: replace + with * in try_recognize_circle | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:234:27: replace > with >= in try_recognize_circle | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:248:23: replace > with == in try_recognize_circle | (a) | a_sample_lifted_out_of_plane_is_rejected_by_every_coplanar_recognizer |
| crates/geometry/src/convert/recognize_curve.rs:248:23: replace > with >= in try_recognize_circle | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:257:14: replace < with == in try_recognize_circle | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:257:14: replace < with <= in try_recognize_circle | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:260:32: replace / with % in try_recognize_circle | (a) | circle_accepts_its_three_sample_minimum |
| crates/geometry/src/convert/recognize_curve.rs:260:32: replace / with * in try_recognize_circle | (b) | `normalize()` divides out any positive scale factor, so `1.0 * u_len` gives the same unit u-axis |
| crates/geometry/src/convert/recognize_curve.rs:283:28: replace - with + in try_recognize_circle | (b) | `pts2d[0]` is the projection origin by construction, so x0 = y0 = sq0 = 0 and the mutated term is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:283:45: replace - with + in try_recognize_circle | (b) | `pts2d[0]` is the projection origin by construction, so x0 = y0 = sq0 = 0 and the mutated term is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:284:31: replace - with + in try_recognize_circle | (b) | `pts2d[0]` is the projection origin by construction, so x0 = y0 = sq0 = 0 and the mutated term is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:295:18: replace < with == in try_recognize_circle | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:295:18: replace < with <= in try_recognize_circle | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:322:20: replace < with == in try_recognize_circle | (a) | a_circle_smaller_than_the_tolerance_is_degenerate |
| crates/geometry/src/convert/recognize_curve.rs:322:20: replace < with <= in try_recognize_circle | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:331:16: replace > with >= in try_recognize_circle | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:362:22: replace < with == in try_recognize_ellipse | (a) | every_recognizer_rejects_an_empty_sample_set |
| crates/geometry/src/convert/recognize_curve.rs:362:22: replace < with <= in try_recognize_ellipse | (a) | conic_recognizers_accept_their_five_sample_minimum |
| crates/geometry/src/convert/recognize_curve.rs:371:41: replace + with - in try_recognize_ellipse | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:371:41: replace + with * in try_recognize_ellipse | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:374:27: replace > with >= in try_recognize_ellipse | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:387:23: replace > with == in try_recognize_ellipse | (a) | a_sample_lifted_out_of_plane_is_rejected_by_every_coplanar_recognizer |
| crates/geometry/src/convert/recognize_curve.rs:387:23: replace > with >= in try_recognize_ellipse | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:444:16: delete - in try_recognize_ellipse | (b) | widening the ellipse gate to admit the parabola band changes nothing: those fits are rejected by the following m_det / K / eigenvalue / residual checks (the parabola round-trip still passes) |
| crates/geometry/src/convert/recognize_curve.rs:451:20: replace < with == in try_recognize_ellipse | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:451:20: replace < with <= in try_recognize_ellipse | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:479:23: replace \|\| with && in try_recognize_ellipse | (b) | disc < 0 forces both eigenvalues to share a sign, so the two clauses are never split |
| crates/geometry/src/convert/recognize_curve.rs:491:46: replace + with - in try_recognize_ellipse | (b) | rotating by -pi/2 instead of +pi/2 flips u_axis to -u_axis; the doc fixes no sign for the semi-major direction and the residual uses \|lu\|,\|lv\| |
| crates/geometry/src/convert/recognize_curve.rs:502:24: replace > with == in try_recognize_ellipse | (a) | samples_displaced_off_the_fitted_conic_fail_the_residual_check |
| crates/geometry/src/convert/recognize_curve.rs:502:24: replace > with >= in try_recognize_ellipse | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:511:41: replace + with - in try_recognize_ellipse | (a) | recovered_conic_axes_match_the_original_axes |
| crates/geometry/src/convert/recognize_curve.rs:531:22: replace < with == in try_recognize_hyperbola | (a) | every_recognizer_rejects_an_empty_sample_set |
| crates/geometry/src/convert/recognize_curve.rs:531:22: replace < with <= in try_recognize_hyperbola | (a) | conic_recognizers_accept_their_five_sample_minimum |
| crates/geometry/src/convert/recognize_curve.rs:540:41: replace + with * in try_recognize_hyperbola | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:540:41: replace + with - in try_recognize_hyperbola | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:543:27: replace > with >= in try_recognize_hyperbola | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:555:23: replace > with == in try_recognize_hyperbola | (a) | a_sample_lifted_out_of_plane_is_rejected_by_every_coplanar_recognizer |
| crates/geometry/src/convert/recognize_curve.rs:555:23: replace > with >= in try_recognize_hyperbola | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:605:20: replace < with == in try_recognize_hyperbola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:605:20: replace < with <= in try_recognize_hyperbola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:618:16: replace < with == in try_recognize_hyperbola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:618:16: replace < with <= in try_recognize_hyperbola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:634:26: replace \|\| with && in try_recognize_hyperbola | (c) | separating the clauses needs a hyperbola whose fit has the "wrong sign" K — the case the comment says is rejected conservatively; whether that rejection is right is the open question |
| crates/geometry/src/convert/recognize_curve.rs:658:30: replace > with == in try_recognize_hyperbola | (a) | samples_displaced_off_the_fitted_conic_fail_the_residual_check |
| crates/geometry/src/convert/recognize_curve.rs:658:30: replace > with >= in try_recognize_hyperbola | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:664:41: replace + with - in try_recognize_hyperbola | (a) | recovered_conic_axes_match_the_original_axes |
| crates/geometry/src/convert/recognize_curve.rs:692:22: replace < with == in try_recognize_parabola | (a) | every_recognizer_rejects_an_empty_sample_set |
| crates/geometry/src/convert/recognize_curve.rs:692:22: replace < with <= in try_recognize_parabola | (a) | conic_recognizers_accept_their_five_sample_minimum |
| crates/geometry/src/convert/recognize_curve.rs:701:41: replace + with - in try_recognize_parabola | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:701:41: replace + with * in try_recognize_parabola | (b) | the plane search re-visits only samples whose cross product with `v1` is zero (`i*1`) or the mirror of an already-rejected pair (`i-1`), so the first accepted normal is unchanged |
| crates/geometry/src/convert/recognize_curve.rs:704:27: replace > with >= in try_recognize_parabola | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:716:23: replace > with == in try_recognize_parabola | (a) | a_sample_lifted_out_of_plane_is_rejected_by_every_coplanar_recognizer |
| crates/geometry/src/convert/recognize_curve.rs:716:23: replace > with >= in try_recognize_parabola | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:733:62: replace / with % in try_recognize_parabola | (b) | the centroid shift is an arbitrary conditioning offset that the code adds back when returning to 3D |
| crates/geometry/src/convert/recognize_curve.rs:746:27: replace += with -= in try_recognize_parabola | (b) | negating the normal matrix (or its rhs) negates theta; axis_2d, perp_2d, d_p, e_p all flip with it and vertex/focal/axis_dir come out identical |
| crates/geometry/src/convert/recognize_curve.rs:748:20: replace += with -= in try_recognize_parabola | (b) | negating the normal matrix (or its rhs) negates theta; axis_2d, perp_2d, d_p, e_p all flip with it and vertex/focal/axis_dir come out identical |
| crates/geometry/src/convert/recognize_curve.rs:765:19: replace > with == in try_recognize_parabola | (c) | disabling the gate changed nothing for every non-parabolic conic tried (circular arcs 0.02-1.0 rad, tolerances 1e-4..1e-2 all still rejected downstream); separating it needs a conic that passes the whole parabola recovery while failing the discriminant |
| crates/geometry/src/convert/recognize_curve.rs:765:19: replace > with >= in try_recognize_parabola | (c) | boundary inclusivity at exactly `tolerance`: the fn doc says "within tolerance" (inclusive), the module doc says "< tolerance" (exclusive) — pinning either encodes an undecided convention |
| crates/geometry/src/convert/recognize_curve.rs:780:24: delete - in try_recognize_parabola | (a) | parabola_recognition_is_rotation_invariant_in_its_plane |
| crates/geometry/src/convert/recognize_curve.rs:782:33: replace + with - in try_recognize_parabola | (b) | the fitted quadratic form is rank-1 PSD (A' > 0), so v1 = 2A'n1(n2,-n1) and v2 = 2A'n2(-n2,n1); the mutated len1 is always NaN or smaller than len2, so the v2 branch runs with its own correct length and the parallel null vector gives the same axis |
| crates/geometry/src/convert/recognize_curve.rs:782:40: replace * with + in try_recognize_parabola | (b) | the fitted quadratic form is rank-1 PSD (A' > 0), so v1 = 2A'n1(n2,-n1) and v2 = 2A'n2(-n2,n1); the mutated len1 is always NaN or smaller than len2, so the v2 branch runs with its own correct length and the parallel null vector gives the same axis |
| crates/geometry/src/convert/recognize_curve.rs:783:33: replace + with - in try_recognize_parabola | (a) | parabola_recognition_is_rotation_invariant_in_its_plane |
| crates/geometry/src/convert/recognize_curve.rs:785:37: replace >= with < in try_recognize_parabola | (b) | both candidates are parallel null vectors of the same rank-1 form; preferring the other one flips the axis sign, which opening_sign cancels |
| crates/geometry/src/convert/recognize_curve.rs:790:16: replace < with == in try_recognize_parabola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:790:16: replace < with <= in try_recognize_parabola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:822:26: replace \|\| with && in try_recognize_parabola | (b) | needs a fit with a vanishing quadratic or linear coefficient, which the discriminant gate and solve_5x5 already exclude |
| crates/geometry/src/convert/recognize_curve.rs:822:18: replace < with == in try_recognize_parabola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:822:18: replace < with <= in try_recognize_parabola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:822:39: replace < with == in try_recognize_parabola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:822:39: replace < with <= in try_recognize_parabola | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:830:20: replace / with % in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:830:20: replace / with * in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:830:15: delete - in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:830:27: replace * with + in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:830:27: replace * with / in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:831:20: replace + with - in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:831:32: replace / with % in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:831:32: replace / with * in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:831:26: replace * with + in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:831:39: replace * with + in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:831:39: replace * with / in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:836:25: delete - in try_recognize_parabola | (b) | the value is taken through `.abs()` |
| crates/geometry/src/convert/recognize_curve.rs:837:21: replace < with == in try_recognize_parabola | (c) | rejecting a parabola whose focal length is below the linear tolerance is itself questionable (the arc may be large); pinning it would enshrine that rule |
| crates/geometry/src/convert/recognize_curve.rs:837:21: replace < with <= in try_recognize_parabola | (c) | rejecting a parabola whose focal length is below the linear tolerance is itself questionable (the arc may be large); pinning it would enshrine that rule |
| crates/geometry/src/convert/recognize_curve.rs:847:30: replace > with == in try_recognize_parabola | (c) | the residual and the discriminant come from the same fit, so no sample set separates this gate from the one at 765 without pinning fit round-off |
| crates/geometry/src/convert/recognize_curve.rs:847:30: replace > with >= in try_recognize_parabola | (c) | the residual and the discriminant come from the same fit, so no sample set separates this gate from the one at 765 without pinning fit round-off |
| crates/geometry/src/convert/recognize_curve.rs:864:30: replace / with % in try_recognize_parabola | (b) | a_p > 0 for every reachable sample set (the F=-1 origin is the sample centroid, always inside the convex parabola) and Rust's remainder takes the dividend sign, so % and / agree in sign |
| crates/geometry/src/convert/recognize_curve.rs:864:30: replace / with * in try_recognize_parabola | (b) | sign(-e*a) == sign(-e/a) for all non-zero a |
| crates/geometry/src/convert/recognize_curve.rs:864:25: delete - in try_recognize_parabola | (a) | recognize_asymmetric_parabola_arc_recovers_vertex_and_opening |
| crates/geometry/src/convert/recognize_curve.rs:880:21: replace + with * in solve_5x5 | (b) | `i*1 == i`: the pivot scan starts from max_val = \|m[i][i]\| (no change) and back-substitution subtracts m[i][i]*x[i] while x[i] is still 0 |
| crates/geometry/src/convert/recognize_curve.rs:881:30: replace > with == in solve_5x5 | (a) | solve_5x5_pivots_past_a_zero_leading_entry |
| crates/geometry/src/convert/recognize_curve.rs:881:30: replace > with < in solve_5x5 | (a) | solve_5x5_pivots_past_a_zero_leading_entry |
| crates/geometry/src/convert/recognize_curve.rs:881:30: replace > with >= in solve_5x5 | (b) | differs only on an exact tie in \|m[k][i]\|, where either row gives the same solution |
| crates/geometry/src/convert/recognize_curve.rs:886:20: replace < with == in solve_5x5 | (a) | solve_5x5_reports_a_singular_system |
| crates/geometry/src/convert/recognize_curve.rs:886:20: replace < with <= in solve_5x5 | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:889:20: replace != with == in solve_5x5 | (a) | solve_5x5_pivots_past_a_zero_leading_entry |
| crates/geometry/src/convert/recognize_curve.rs:906:21: replace + with * in solve_5x5 | (b) | `i*1 == i`: the pivot scan starts from max_val = \|m[i][i]\| (no change) and back-substitution subtracts m[i][i]*x[i] while x[i] is still 0 |
| crates/geometry/src/convert/recognize_curve.rs:933:9: replace DetectedCurveKind::as_str -> &'static str with "" | (a) | detected_curve_kind_tags_are_stable |
| crates/geometry/src/convert/recognize_curve.rs:933:9: replace DetectedCurveKind::as_str -> &'static str with "xyzzy" | (a) | detected_curve_kind_tags_are_stable |
| crates/geometry/src/convert/recognize_curve.rs:958:27: replace && with \|\| in detect_curve_kind | (a) | detect_non_rational_quadratic_is_bspline |
| crates/geometry/src/convert/recognize_curve.rs:958:23: replace < with <= in detect_curve_kind | (a) | detect_non_rational_quadratic_is_bspline |
| crates/geometry/src/convert/recognize_curve.rs:970:45: replace * with / in detect_curve_kind | (a) | detect_curve_kind_samples_the_start_of_the_domain |
| crates/geometry/src/convert/recognize_curve.rs:970:36: replace - with + in detect_curve_kind | (a) | detect_curve_kind_samples_the_whole_parameter_domain |
| crates/geometry/src/convert/recognize_curve.rs:970:72: replace - with + in detect_curve_kind | (a) | detect_curve_kind_samples_the_whole_parameter_domain |
| crates/geometry/src/convert/recognize_curve.rs:970:72: replace - with / in detect_curve_kind | (a) | detect_curve_kind_samples_the_whole_parameter_domain |
| crates/geometry/src/convert/recognize_curve.rs:978:12: replace < with == in detect_curve_kind | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:978:12: replace < with <= in detect_curve_kind | (b) | equality-only against an internal degeneracy epsilon; no reachable fixture puts the guarded value bit-exactly on it, and the `?`/normalize guards downstream reject the same inputs |
| crates/geometry/src/convert/recognize_curve.rs:992:5: replace sample_extent -> f64 with 1.0 | (a) | detect_uses_the_sample_extent_for_its_deviation_budget |
| crates/geometry/src/convert/recognize_curve.rs:1000:26: replace - with + in sample_extent | (a) | detect_extent_is_a_span_not_a_coordinate_sum |

## crates/geometry/src/convert/curve_to_nurbs.rs

- before: 47 survivors; after: 11 survivors, 86 caught, 19 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/convert/curve_to_nurbs.rs:34:22: replace - with + in circle_to_nurbs | (a) | circle_arc_with_offset_start_uses_span_not_sum |
| crates/geometry/src/convert/curve_to_nurbs.rs:35:19: replace < with <= in circle_to_nurbs | (b) | Differs only when span.abs() is bit-exactly the 1e-15 implementation epsilon; the contract distinguishes zero from non-zero span, not that value |
| crates/geometry/src/convert/curve_to_nurbs.rs:46:52: replace < with == in circle_to_nurbs | (a) | circle_segment_count_snaps_span_near_multiple_of_half_pi |
| crates/geometry/src/convert/curve_to_nurbs.rs:46:52: replace < with > in circle_to_nurbs | (a) | circle_segment_count_snaps_span_near_multiple_of_half_pi |
| crates/geometry/src/convert/curve_to_nurbs.rs:46:52: replace < with <= in circle_to_nurbs | (b) | Differs only when the jitter term is bit-exactly the 1e-9 snap epsilon |
| crates/geometry/src/convert/curve_to_nurbs.rs:46:29: replace - with + in circle_to_nurbs | (a) | circle_segment_count_snaps_span_near_multiple_of_half_pi |
| crates/geometry/src/convert/curve_to_nurbs.rs:46:29: replace - with / in circle_to_nurbs | (a) | circle_segment_count_snaps_span_near_multiple_of_half_pi |
| crates/geometry/src/convert/curve_to_nurbs.rs:79:22: replace - with + in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:79:22: replace - with / in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:80:27: replace \|\| with && in circle_to_nurbs_with_segments | (a) | circle_with_segments_rejects_zero_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:80:19: replace < with == in circle_to_nurbs_with_segments | (a) | circle_with_segments_rejects_zero_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:80:19: replace < with > in circle_to_nurbs_with_segments | (a) | circle_with_segments_rejects_zero_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:80:19: replace < with <= in circle_to_nurbs_with_segments | (b) | Differs only when span.abs() is bit-exactly the 1e-15 implementation epsilon |
| crates/geometry/src/convert/curve_to_nurbs.rs:80:39: replace == with != in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:88:37: replace > with == in circle_to_nurbs_with_segments | (a) | circle_with_segments_rejects_segment_span_over_half_pi |
| crates/geometry/src/convert/curve_to_nurbs.rs:88:37: replace > with >= in circle_to_nurbs_with_segments | (b) | Differs only when the segment span is bit-exactly FRAC_PI_2 + 1e-6; the 1e-6 is an undocumented jitter margin, not a contract value |
| crates/geometry/src/convert/curve_to_nurbs.rs:88:19: replace / with % in circle_to_nurbs_with_segments | (a) | circle_with_segments_rejects_segment_span_over_half_pi |
| crates/geometry/src/convert/curve_to_nurbs.rs:88:19: replace / with * in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:88:49: replace + with - in circle_to_nurbs_with_segments | (a) | circle_with_segments_accepts_exactly_half_pi_per_segment |
| crates/geometry/src/convert/curve_to_nurbs.rs:88:49: replace + with * in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:94:22: replace / with % in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:94:22: replace / with * in circle_to_nurbs_with_segments | (a) | circle_with_segments_honours_count_and_offset_span |
| crates/geometry/src/convert/curve_to_nurbs.rs:119:19: replace < with <= in ellipse_to_nurbs | (b) | Differs only when span.abs() is bit-exactly the 1e-15 implementation epsilon |
| crates/geometry/src/convert/curve_to_nurbs.rs:123:31: replace / with * in ellipse_to_nurbs | (a) | ellipse_full_turn_uses_four_quadratic_arcs |
| crates/geometry/src/convert/curve_to_nurbs.rs:156:19: replace < with <= in line_to_nurbs | (b) | Requires d.length() to be bit-exactly 1e-15, i.e. an exact sqrt hit; not distinguishable from the documented start == end contract |
| crates/geometry/src/convert/curve_to_nurbs.rs:191:28: replace + with - in arc_segments_to_nurbs | (b) | n_cps feeds only Vec::with_capacity — a capacity hint with no observable effect |
| crates/geometry/src/convert/curve_to_nurbs.rs:191:28: replace + with * in arc_segments_to_nurbs | (b) | n_cps feeds only Vec::with_capacity — a capacity hint with no observable effect |
| crates/geometry/src/convert/curve_to_nurbs.rs:191:19: replace * with + in arc_segments_to_nurbs | (b) | n_cps feeds only Vec::with_capacity — a capacity hint with no observable effect |
| crates/geometry/src/convert/curve_to_nurbs.rs:191:19: replace * with / in arc_segments_to_nurbs | (b) | n_cps feeds only Vec::with_capacity — a capacity hint with no observable effect |
| crates/geometry/src/convert/curve_to_nurbs.rs:276:18: delete - in tangent_intersection | (a) | tangent_intersection_xz_rows_hits_known_point |
| crates/geometry/src/convert/curve_to_nurbs.rs:276:44: delete - in tangent_intersection | (a) | tangent_intersection_xz_rows_hits_known_point |
| crates/geometry/src/convert/curve_to_nurbs.rs:279:18: delete - in tangent_intersection | (a) | tangent_intersection_yz_rows_hits_known_point |
| crates/geometry/src/convert/curve_to_nurbs.rs:279:44: delete - in tangent_intersection | (a) | tangent_intersection_yz_rows_hits_known_point |
| crates/geometry/src/convert/curve_to_nurbs.rs:283:18: replace < with == in tangent_intersection | (a) | tangent_intersection_parallel_rays_return_none |
| crates/geometry/src/convert/curve_to_nurbs.rs:283:18: replace < with <= in tangent_intersection | (b) | Differs only when det.abs() is bit-exactly the 1e-30 parallel-test epsilon |
| crates/geometry/src/convert/curve_to_nurbs.rs:294:25: replace * with + in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:294:16: replace + with - in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:294:25: replace * with / in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:294:16: replace + with * in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:295:25: replace * with + in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:295:25: replace * with / in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:295:16: replace + with - in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:295:16: replace + with * in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:296:25: replace * with + in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:296:25: replace * with / in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:296:16: replace + with - in midpoint | (a) | midpoint_is_componentwise_average |
| crates/geometry/src/convert/curve_to_nurbs.rs:296:16: replace + with * in midpoint | (a) | midpoint_is_componentwise_average |

## crates/geometry/src/extrema/point_surface.rs

- before: 273 survivors; after: 51 survivors, 343 caught, 32 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/extrema/point_surface.rs:17:5: replace normalize_angle -> f64 with 1.0 | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:17:5: replace normalize_angle -> f64 with 0.0 | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:17:5: replace normalize_angle -> f64 with -1.0 | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:17:14: replace < with == in normalize_angle | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:17:14: replace < with > in normalize_angle | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:17:14: replace < with <= in normalize_angle | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:18:15: replace + with - in normalize_angle | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:18:15: replace + with * in normalize_angle | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:41:41: replace < with <= in point_to_plane | (a) | plane_normal_x_exactly_at_candidate_threshold |
| crates/geometry/src/extrema/point_surface.rs:48:14: replace < with == in point_to_plane | (a) | plane_zero_normal_returns_finite_zero_uv |
| crates/geometry/src/extrema/point_surface.rs:48:14: replace < with <= in point_to_plane | (b) | equivalent: boundary-only, differs only if \|n x candidate\| is exactly 1e-15, which the unit-normal contract excludes |
| crates/geometry/src/extrema/point_surface.rs:57:31: replace / with * in point_to_plane | (a) | plane_uv_axes_are_normalized_not_scaled |
| crates/geometry/src/extrema/point_surface.rs:59:31: replace / with * in point_to_plane | (b) | equivalent: v_raw = n x u_axis is unit by construction, so 1/\|v_raw\| and \|v_raw\| agree to within one ulp |
| crates/geometry/src/extrema/point_surface.rs:78:19: replace - with + in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:79:19: replace - with + in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:80:19: replace - with + in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:86:16: replace - with + in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:87:16: replace - with + in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:87:16: replace - with / in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:88:20: replace * with / in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:92:14: replace < with <= in point_to_cylinder | (b) | equivalent: boundary-only, differs only if the radial length is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:106:51: replace + with - in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:107:51: replace + with - in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:107:51: replace + with * in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:107:30: replace + with - in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:107:30: replace + with * in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:107:43: replace * with / in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:108:30: replace + with - in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:108:30: replace + with * in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:108:43: replace * with / in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:108:55: replace * with / in point_to_cylinder | (a) | cylinder_normal_offset_recovers_parameters, cylinder_zero_angle_and_axis_point |
| crates/geometry/src/extrema/point_surface.rs:126:19: replace - with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:126:19: replace - with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:127:19: replace - with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:127:19: replace - with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:128:19: replace - with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:128:19: replace - with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:134:16: replace - with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:134:16: replace - with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:134:20: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:134:20: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:135:16: replace - with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:135:16: replace - with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:135:20: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:135:20: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:136:16: replace - with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:136:16: replace - with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:136:20: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:136:20: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:140:17: replace && with \|\| in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:140:10: replace <= with > in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:140:26: replace < with == in point_to_cone | (b) | equivalent: with r_len < 1e-15 and h <= 0 the fall-through recomputes v <= 0 and returns the same apex result |
| crates/geometry/src/extrema/point_surface.rs:140:26: replace < with <= in point_to_cone | (b) | equivalent: boundary-only, differs only if r_len is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:140:26: replace < with > in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:152:36: replace * with + in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:152:36: replace * with / in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:154:10: replace <= with > in point_to_cone | (a) | cone_point_behind_apex_projects_to_apex |
| crates/geometry/src/extrema/point_surface.rs:165:20: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:165:20: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:166:20: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:166:20: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:168:33: replace < with == in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:168:33: replace < with > in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:168:33: replace < with <= in point_to_cone | (b) | equivalent: boundary-only, differs only if r_len is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:170:29: replace + with - in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:170:29: replace + with * in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:170:38: replace * with + in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:170:38: replace * with / in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:171:29: replace + with - in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:171:29: replace + with * in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:171:38: replace * with + in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:171:38: replace * with / in point_to_cone | (a) | cone_interior_axis_point_keeps_generator_parameter, cone_below_apex_plane_but_outside_uses_generator |
| crates/geometry/src/extrema/point_surface.rs:172:29: replace + with - in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:172:29: replace + with * in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:172:38: replace * with + in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:172:38: replace * with / in point_to_cone | (c) | geometry judgment: the on-axis degenerate branch returns a point that is NOT on the cone (the radial term is dropped, and the reported distance is h*cos^2(a) where the true distance is h*cos(a)); pinning the returned point requires deciding what the projection of an axis point should be |
| crates/geometry/src/extrema/point_surface.rs:176:39: replace / with % in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:176:39: replace / with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:177:39: replace / with % in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:177:39: replace / with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:178:39: replace / with % in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:178:39: replace / with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:56: replace + with - in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:56: replace + with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:29: replace + with - in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:29: replace + with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:38: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:38: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:65: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:180:65: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:56: replace + with - in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:56: replace + with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:29: replace + with - in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:29: replace + with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:38: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:38: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:65: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:181:65: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:56: replace + with - in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:56: replace + with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:29: replace + with - in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:29: replace + with * in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:38: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:38: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:65: replace * with + in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:182:65: replace * with / in point_to_cone | (a) | cone_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:214:23: replace < with == in point_to_sphere | (a) | sphere_center_query_returns_a_point_on_the_sphere |
| crates/geometry/src/extrema/point_surface.rs:214:23: replace < with <= in point_to_sphere | (b) | equivalent: boundary-only, differs only if \|P - centre\| is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:217:33: replace + with - in point_to_sphere | (b) | equivalent: the doc contracts only that "an arbitrary surface point is returned"; centre - radius*x is equally on the sphere |
| crates/geometry/src/extrema/point_surface.rs:217:33: replace + with * in point_to_sphere | (a) | sphere_center_query_returns_a_point_on_the_sphere |
| crates/geometry/src/extrema/point_surface.rs:231:29: replace + with * in point_to_sphere | (a) | sphere_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:231:29: replace + with - in point_to_sphere | (a) | sphere_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:231:38: replace * with / in point_to_sphere | (a) | sphere_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:232:29: replace + with - in point_to_sphere | (a) | sphere_normal_offset_recovers_parameters |
| crates/geometry/src/extrema/point_surface.rs:253:19: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:254:19: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:255:19: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:263:16: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:263:20: replace * with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:264:16: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:264:20: replace * with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:265:16: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:265:20: replace * with / in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:273:54: replace < with == in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:273:54: replace < with > in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:273:54: replace < with <= in point_to_torus | (b) | equivalent: boundary-only, differs only if r_len is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:276:32: replace + with - in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:276:32: replace + with * in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:276:42: replace * with + in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:276:42: replace * with / in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:277:32: replace + with - in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:277:32: replace + with * in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:277:42: replace * with + in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:277:42: replace * with / in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:278:32: replace + with - in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:278:32: replace + with * in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:278:42: replace * with + in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:278:42: replace * with / in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:286:32: replace + with - in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:286:32: replace + with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:286:45: replace * with / in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:287:32: replace + with - in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:287:32: replace + with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:287:45: replace * with / in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:295:19: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:296:19: replace - with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:300:18: replace < with == in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:300:18: replace < with <= in point_to_torus | (b) | equivalent: boundary-only, differs only if the tube distance is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:302:28: replace < with == in point_to_torus | (b) | equivalent: the branch is reached only with r_len = major_radius, so == is false exactly where < is false |
| crates/geometry/src/extrema/point_surface.rs:302:28: replace < with > in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:305:34: replace / with % in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:302:28: replace < with <= in point_to_torus | (b) | equivalent: boundary-only, differs only if r_len is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:305:34: replace / with * in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:305:54: replace / with % in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:305:74: replace / with % in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:305:74: replace / with * in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:305:54: replace / with * in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:308:22: replace + with - in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:308:22: replace + with * in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:308:32: replace * with + in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:308:32: replace * with / in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:309:22: replace + with - in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:309:32: replace * with + in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:309:32: replace * with / in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:309:22: replace + with * in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:310:22: replace + with - in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:310:22: replace + with * in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:310:32: replace * with + in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:310:32: replace * with / in point_to_torus | (a) | torus_points_on_the_major_circle_return_the_minor_radius |
| crates/geometry/src/extrema/point_surface.rs:320:30: replace / with % in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:322:18: replace + with - in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:322:18: replace + with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:320:30: replace / with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:322:33: replace * with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:323:18: replace + with - in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:322:33: replace * with / in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:323:18: replace + with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:323:33: replace * with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:323:33: replace * with / in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:324:18: replace + with - in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:324:18: replace + with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:324:33: replace * with + in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:324:33: replace * with / in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:329:38: replace < with == in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:329:38: replace < with > in point_to_torus | (a) | torus_axis_point_seeds_the_x_axis_meridian, torus_axis_point_with_rotated_frames |
| crates/geometry/src/extrema/point_surface.rs:329:38: replace < with <= in point_to_torus | (b) | equivalent: boundary-only, differs only if r_len is exactly 1e-15 |
| crates/geometry/src/extrema/point_surface.rs:332:47: replace / with % in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:332:47: replace / with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:332:67: replace / with % in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:332:67: replace / with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:332:87: replace / with * in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:332:87: replace / with % in point_to_torus | (a) | torus_normal_offset_recovers_parameters, torus_point_inside_the_ring_hole |
| crates/geometry/src/extrema/point_surface.rs:367:27: replace + with - in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:27: replace + with * in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:58: replace / with % in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:58: replace / with * in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:45: replace * with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:45: replace * with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:36: replace - with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:367:36: replace - with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:27: replace + with * in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:27: replace + with - in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:58: replace / with % in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:58: replace / with * in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:45: replace * with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:45: replace * with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:36: replace - with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:368:36: replace - with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:370:29: replace - with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:370:29: replace - with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:371:29: replace - with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:371:29: replace - with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:372:29: replace - with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:372:29: replace - with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:373:60: replace * with + in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:373:60: replace * with / in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:374:24: replace < with > in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:374:24: replace < with == in point_to_nurbs_surface | (a) | nurbs_grid_search_matches_the_documented_11x11_grid |
| crates/geometry/src/extrema/point_surface.rs:374:24: replace < with <= in point_to_nurbs_surface | (b) | equivalent: differs only on an exact distance tie, where either grid sample is equally closest |
| crates/geometry/src/extrema/point_surface.rs:425:32: replace * with + in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:425:32: replace * with / in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:425:26: replace + with - in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:425:26: replace + with * in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:426:32: replace * with + in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:426:32: replace * with / in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:426:26: replace + with - in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:426:26: replace + with * in point_to_surface | (b) | equivalent: dead store, the grid loop always overwrites best_u/best_v on its first sample (d2 < INFINITY) |
| crates/geometry/src/extrema/point_surface.rs:430:46: replace / with % in point_to_surface | (a) | generic_solver_grid_seed_beats_the_range_midpoint, generic_solver_grid_must_span_the_u_range |
| crates/geometry/src/extrema/point_surface.rs:430:32: replace * with + in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:430:46: replace / with * in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:430:26: replace - with + in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:430:26: replace - with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:430:57: replace - with + in point_to_surface | (c) | geometry judgment: re-spaces the seed grid (the last sample is no longer at the range end); every fixture still seeds the same Newton basin, so killing it requires deciding how much of the range the seeding grid must cover |
| crates/geometry/src/extrema/point_surface.rs:430:57: replace - with / in point_to_surface | (c) | geometry judgment: re-spaces the seed grid (the last sample is no longer at the range end); every fixture still seeds the same Newton basin, so killing it requires deciding how much of the range the seeding grid must cover |
| crates/geometry/src/extrema/point_surface.rs:432:24: replace + with - in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:432:50: replace / with % in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:432:50: replace / with * in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:432:36: replace * with + in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:432:36: replace * with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:432:61: replace - with + in point_to_surface | (c) | geometry judgment: re-spaces the seed grid (the last sample is no longer at the range end); every fixture still seeds the same Newton basin, so killing it requires deciding how much of the range the seeding grid must cover |
| crates/geometry/src/extrema/point_surface.rs:432:61: replace - with / in point_to_surface | (c) | geometry judgment: re-spaces the seed grid (the last sample is no longer at the range end); every fixture still seeds the same Newton basin, so killing it requires deciding how much of the range the seeding grid must cover |
| crates/geometry/src/extrema/point_surface.rs:435:42: replace + with - in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:435:42: replace + with * in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:435:31: replace * with + in point_to_surface | (a) | generic_solver_grid_seed_beats_the_range_midpoint, generic_solver_grid_must_span_the_u_range |
| crates/geometry/src/extrema/point_surface.rs:435:31: replace * with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:435:53: replace * with + in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:435:53: replace * with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:436:19: replace < with == in point_to_surface | (a) | generic_solver_grid_seed_beats_the_range_midpoint, generic_solver_grid_must_span_the_u_range |
| crates/geometry/src/extrema/point_surface.rs:436:19: replace < with > in point_to_surface | (a) | generic_solver_grid_seed_beats_the_range_midpoint, generic_solver_grid_must_span_the_u_range |
| crates/geometry/src/extrema/point_surface.rs:436:19: replace < with <= in point_to_surface | (b) | equivalent: differs only on an exact distance tie, where either grid sample is equally closest |
| crates/geometry/src/extrema/point_surface.rs:457:36: replace - with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:457:55: replace - with + in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:467:29: replace - with + in point_to_surface | (b) | equivalent: rescales the Newton step but preserves its fixed point (f1 = f2 = 0), so any converged projection is identical |
| crates/geometry/src/extrema/point_surface.rs:467:23: replace * with + in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:467:23: replace * with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:467:35: replace * with + in point_to_surface | (b) | equivalent: rescales the Newton step but preserves its fixed point (f1 = f2 = 0), so any converged projection is identical |
| crates/geometry/src/extrema/point_surface.rs:469:22: replace < with == in point_to_surface | (b) | equivalent: only reached when \|det\| < f64::EPSILON, where Su and Sv are degenerate and the resulting step is a clamped no-op (exercised by generic_solver_at_a_sphere_pole_stays_finite) |
| crates/geometry/src/extrema/point_surface.rs:469:22: replace < with <= in point_to_surface | (b) | equivalent: boundary-only, differs only if \|det\| is exactly f64::EPSILON |
| crates/geometry/src/extrema/point_surface.rs:473:28: replace - with + in point_to_surface | (b) | equivalent: rescales the Newton step but preserves its fixed point (f1 = f2 = 0), so any converged projection is identical |
| crates/geometry/src/extrema/point_surface.rs:473:22: replace * with / in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:473:33: replace * with + in point_to_surface | (a) | generic_solver_meets_stationarity_on_a_non_orthogonal_patch |
| crates/geometry/src/extrema/point_surface.rs:474:28: replace - with + in point_to_surface | (b) | equivalent: rescales the Newton step but preserves its fixed point (f1 = f2 = 0), so any converged projection is identical |
| crates/geometry/src/extrema/point_surface.rs:474:33: replace * with + in point_to_surface | (a) | generic_solver_meets_stationarity_on_a_non_orthogonal_patch |
| crates/geometry/src/extrema/point_surface.rs:479:30: replace < with == in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:30: replace < with <= in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:19: replace - with + in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:19: replace - with / in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:63: replace < with == in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:63: replace < with > in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |
| crates/geometry/src/extrema/point_surface.rs:479:63: replace < with <= in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:52: replace - with + in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:479:52: replace - with / in point_to_surface | (b) | equivalent: removes only the early exit; the remaining iterations are fixed-point no-ops, so the returned u/v are unchanged |
| crates/geometry/src/extrema/point_surface.rs:490:41: replace + with - in point_to_surface | (a) | generic_solver_matches_the_analytic_torus_distance, generic_solver_matches_the_analytic_cylinder_distance |

## crates/geometry/src/extrema/lipschitz.rs

- before: 144 survivors; after: 62 survivors, 194 caught, 11 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/extrema/lipschitz.rs:76:36: replace * with + in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:76:36: replace * with / in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:76:30: replace + with - in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:76:30: replace + with * in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:77:36: replace * with + in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:77:36: replace * with / in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:77:30: replace + with - in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:77:30: replace + with * in LipschitzOptimizer::minimize_2d | (b) | Equivalent: dead store. `best` starts at `f64::INFINITY`, so the first grid sample overwrites both initialisers, and the documented Lipschitz contract forces `f` to be finite on the grid. |
| crates/geometry/src/extrema/lipschitz.rs:80:30: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:82:34: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:84:30: replace * with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_is_invariant_under_a_constant_offset |
| crates/geometry/src/extrema/lipschitz.rs:84:30: replace * with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_is_invariant_under_a_constant_offset |
| crates/geometry/src/extrema/lipschitz.rs:84:35: replace + with - in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_is_invariant_under_a_constant_offset |
| crates/geometry/src/extrema/lipschitz.rs:84:35: replace + with * in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_is_invariant_under_a_constant_offset |
| crates/geometry/src/extrema/lipschitz.rs:85:24: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:95:36: replace / with % in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_refines_far_below_the_cell_tolerance |
| crates/geometry/src/extrema/lipschitz.rs:95:36: replace / with * in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_refines_far_below_the_cell_tolerance |
| crates/geometry/src/extrema/lipschitz.rs:95:30: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum |
| crates/geometry/src/extrema/lipschitz.rs:95:30: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_refines_far_below_the_cell_tolerance |
| crates/geometry/src/extrema/lipschitz.rs:96:36: replace / with % in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_refines_far_below_the_cell_tolerance |
| crates/geometry/src/extrema/lipschitz.rs:96:36: replace / with * in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_refines_far_below_the_cell_tolerance |
| crates/geometry/src/extrema/lipschitz.rs:96:30: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum |
| crates/geometry/src/extrema/lipschitz.rs:96:30: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_refines_far_below_the_cell_tolerance |
| crates/geometry/src/extrema/lipschitz.rs:104:34: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:111:24: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:118:35: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:125:24: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:133:28: replace / with % in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:133:28: replace / with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:133:22: replace - with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:133:22: replace - with / in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:134:28: replace / with % in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:134:28: replace / with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:134:22: replace - with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:134:22: replace - with / in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:138:40: replace * with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:138:40: replace * with / in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:138:45: replace + with - in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:138:45: replace + with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:139:56: replace + with - in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:139:46: replace * with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:139:46: replace * with / in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:139:41: replace + with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:139:51: replace + with - in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:139:51: replace + with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:140:40: replace * with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:140:40: replace * with / in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:140:45: replace + with - in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:140:45: replace + with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:140:56: replace + with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:141:45: replace / with % in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:141:45: replace / with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:141:32: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_is_invariant_under_a_constant_offset |
| crates/geometry/src/extrema/lipschitz.rs:141:32: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:142:45: replace / with % in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:142:45: replace / with * in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it perturbs the heuristic Lipschitz estimate but not the answer, because the x2 safety margin absorbs it. Measured with a mutation-aware simulator over ~150 fixtures: every fixture that separates this mutant sits on the pruning cliff (fails at bound x1.5, passes at x2), so the test would be brittle and output-derived rather than contract-derived. Killing it means first deciding how tight the estimated bound is required to be. |
| crates/geometry/src/extrema/lipschitz.rs:142:32: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_is_invariant_under_a_constant_offset |
| crates/geometry/src/extrema/lipschitz.rs:142:32: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:144:30: replace > with == in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:144:30: replace > with < in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:144:30: replace > with >= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:150:13: replace *= with += in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the fixture is scaled by 1e-3, so replacing the x2 margin with +2 inflates the bound by about 1e5 and pruning collapses) |
| crates/geometry/src/extrema/lipschitz.rs:150:13: replace *= with /= in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the fixture is scaled by 1e-3, so replacing the x2 margin with +2 inflates the bound by about 1e5 and pruning collapses) |
| crates/geometry/src/extrema/lipschitz.rs:151:16: replace < with == in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_short_circuits_a_constant_objective |
| crates/geometry/src/extrema/lipschitz.rs:151:16: replace < with > in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:151:16: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:162:27: replace > with == in LipschitzOptimizer::minimize_2d | (b) | Equivalent: off-by-one in the budget break (stops at cell 500 000 instead of 500 001). |
| crates/geometry/src/extrema/lipschitz.rs:162:27: replace > with < in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:161:24: replace += with *= in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_honours_the_documented_evaluation_budget |
| crates/geometry/src/extrema/lipschitz.rs:162:27: replace > with >= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: off-by-one in the budget break (stops at cell 500 000 instead of 500 001). |
| crates/geometry/src/extrema/lipschitz.rs:166:34: replace * with + in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:166:34: replace * with / in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:166:27: replace + with - in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:166:27: replace + with * in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:167:34: replace * with + in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:167:34: replace * with / in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:167:27: replace + with - in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:167:27: replace + with * in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum` and `minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see` (the mutated cell centre leaves the requested domain) |
| crates/geometry/src/extrema/lipschitz.rs:169:19: replace < with == in LipschitzOptimizer::minimize_2d | (b) | Equivalent in effect: the identical incumbent update is repeated at the terminal branch (line 187), so the returned optimum is unchanged; only how early the incumbent improves, and hence pruning aggressiveness, differs. |
| crates/geometry/src/extrema/lipschitz.rs:169:19: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent in effect: the identical incumbent update is repeated at the terminal branch (line 187), so the returned optimum is unchanged; only how early the incumbent improves, and hence pruning aggressiveness, differs. |
| crates/geometry/src/extrema/lipschitz.rs:175:31: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:175:31: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:176:31: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:176:31: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:177:73: replace * with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:177:73: replace * with / in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it only weakens pruning (a larger or NaN cell radius means fewer prunes and later termination); the branch-and-bound still returns the same global optimum, and separating it would mean deciding how much search effort the radius rule is required to save. |
| crates/geometry/src/extrema/lipschitz.rs:177:45: replace + with - in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it only weakens pruning (a larger or NaN cell radius means fewer prunes and later termination); the branch-and-bound still returns the same global optimum, and separating it would mean deciding how much search effort the radius rule is required to save. |
| crates/geometry/src/extrema/lipschitz.rs:177:45: replace + with * in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:177:35: replace * with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it only weakens pruning (a larger or NaN cell radius means fewer prunes and later termination); the branch-and-bound still returns the same global optimum, and separating it would mean deciding how much search effort the radius rule is required to save. |
| crates/geometry/src/extrema/lipschitz.rs:177:35: replace * with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:177:55: replace * with + in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it only weakens pruning (a larger or NaN cell radius means fewer prunes and later termination); the branch-and-bound still returns the same global optimum, and separating it would mean deciding how much search effort the radius rule is required to save. |
| crates/geometry/src/extrema/lipschitz.rs:177:55: replace * with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:178:28: replace - with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:178:28: replace - with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:178:34: replace * with + in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:178:34: replace * with / in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:181:22: replace > with == in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:181:22: replace > with < in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:181:22: replace > with >= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:186:35: replace \|\| with && in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it subdivides deeper than necessary, giving the same optimum for more cells. Separating it would mean deciding what the terminal rule must guarantee beyond radius convergence. |
| crates/geometry/src/extrema/lipschitz.rs:186:23: replace < with == in LipschitzOptimizer::minimize_2d | (c) | Needs geometry judgment: it subdivides deeper than necessary, giving the same optimum for more cells. Separating it would mean deciding what the terminal rule must guarantee beyond radius convergence. |
| crates/geometry/src/extrema/lipschitz.rs:186:23: replace < with > in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:186:23: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:186:44: replace >= with < in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:187:23: replace < with == in LipschitzOptimizer::minimize_2d | (b) | Equivalent: unreachable/no-op. Line 169 already guarantees `fc >= best` at this point, so `<=` and `==` re-assign exactly the same values. |
| crates/geometry/src/extrema/lipschitz.rs:187:23: replace < with > in LipschitzOptimizer::minimize_2d | (a) | `minimize_2d_refines_far_below_the_cell_tolerance` (`fc > best` lets the incumbent get worse) |
| crates/geometry/src/extrema/lipschitz.rs:187:23: replace < with <= in LipschitzOptimizer::minimize_2d | (b) | Equivalent: unreachable/no-op. Line 169 already guarantees `fc >= best` at this point, so `<=` and `==` re-assign exactly the same values. |
| crates/geometry/src/extrema/lipschitz.rs:196:24: replace >= with < in LipschitzOptimizer::minimize_2d | (a) | minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see |
| crates/geometry/src/extrema/lipschitz.rs:197:54: replace + with * in LipschitzOptimizer::minimize_2d | (b) | Equivalent for every reachable use: the depth cap only binds for a tolerance below roughly 7e-8 on a domain this size, and every in-tree caller passes 1e-4, so `depth` is never read. |
| crates/geometry/src/extrema/lipschitz.rs:198:54: replace + with * in LipschitzOptimizer::minimize_2d | (b) | Equivalent for every reachable use: the depth cap only binds for a tolerance below roughly 7e-8 on a domain this size, and every in-tree caller passes 1e-4, so `depth` is never read. |
| crates/geometry/src/extrema/lipschitz.rs:200:54: replace + with - in LipschitzOptimizer::minimize_2d | (a) | every `minimize_2d` test (`minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see`, `minimize_2d_is_invariant_under_a_constant_offset`, `minimize_2d_refines_far_below_the_cell_tolerance`, `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum`, `minimize_2d_honours_the_documented_evaluation_budget`): usize underflow panic, because the `else` subdivision branch runs at depth 0 when the v-extent is the longer one. |
| crates/geometry/src/extrema/lipschitz.rs:200:54: replace + with * in LipschitzOptimizer::minimize_2d | (b) | Equivalent for every reachable use: the depth cap only binds for a tolerance below roughly 7e-8 on a domain this size, and every in-tree caller passes 1e-4, so `depth` is never read. |
| crates/geometry/src/extrema/lipschitz.rs:201:54: replace + with * in LipschitzOptimizer::minimize_2d | (b) | Equivalent for every reachable use: the depth cap only binds for a tolerance below roughly 7e-8 on a domain this size, and every in-tree caller passes 1e-4, so `depth` is never read. |
| crates/geometry/src/extrema/lipschitz.rs:201:54: replace + with - in LipschitzOptimizer::minimize_2d | (a) | every `minimize_2d` test (`minimize_2d_finds_deep_well_that_the_grid_phase_cannot_see`, `minimize_2d_is_invariant_under_a_constant_offset`, `minimize_2d_refines_far_below_the_cell_tolerance`, `minimize_2d_clamps_the_search_to_the_domain_at_a_corner_optimum`, `minimize_2d_honours_the_documented_evaluation_budget`): usize underflow panic, because the `else` subdivision branch runs at depth 0 when the v-extent is the longer one. |
| crates/geometry/src/extrema/lipschitz.rs:224:5: replace estimate_curve_curve_lipschitz -> f64 with 0.0 | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:224:5: replace estimate_curve_curve_lipschitz -> f64 with 1.0 | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:224:5: replace estimate_curve_curve_lipschitz -> f64 with -1.0 | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:233:26: replace / with % in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:233:26: replace / with * in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:234:20: replace + with - in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:234:20: replace + with * in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:234:32: replace * with + in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:234:32: replace * with / in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:234:26: replace - with + in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:238:20: replace + with - in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:238:20: replace + with * in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:238:32: replace * with + in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:238:32: replace * with / in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:238:26: replace - with + in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:247:19: replace * with + in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:247:19: replace * with / in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:247:9: replace * with + in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:247:9: replace * with / in estimate_curve_curve_lipschitz | (a) | curve_curve_lipschitz_matches_the_documented_bound |
| crates/geometry/src/extrema/lipschitz.rs:263:12: replace < with == in nurbs_curve_curve_distance | (b) | Equivalent: for `lip` exactly 0 both paths return the same distance (the bound is 0 only for coincident or point-like curves, where the optimiser path recovers the same answer). |
| crates/geometry/src/extrema/lipschitz.rs:263:12: replace < with > in nurbs_curve_curve_distance | (a) | nurbs_distance_between_skew_segments_is_not_the_midpoint_distance |
| crates/geometry/src/extrema/lipschitz.rs:263:12: replace < with <= in nurbs_curve_curve_distance | (b) | Equivalent: tie-break only. It differs solely on an exact float tie, where both candidates are equally valid minimisers, or where both branches shrink the bracket by the same factor toward the same limit. |
| crates/geometry/src/extrema/lipschitz.rs:265:44: replace * with + in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:265:44: replace * with / in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:265:38: replace + with - in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:265:38: replace + with * in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:266:44: replace * with + in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:266:44: replace * with / in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:266:38: replace + with - in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |
| crates/geometry/src/extrema/lipschitz.rs:266:38: replace + with * in nurbs_curve_curve_distance | (a) | nurbs_distance_between_coincident_curves_is_zero |

## crates/geometry/src/extrema/curve_curve.rs

- before: 77 survivors; after: 53 survivors, 149 caught, 14 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/extrema/curve_curve.rs:61:18: replace && with \|\| in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; both operands stay false so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:61:10: replace < with == in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `a == 1e-30` is false, so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:61:10: replace < with > in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `a > 1e-30` is true but `e < 1e-30` is false, so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:61:10: replace < with <= in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `a <= 1e-30` is false, so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:61:23: replace < with > in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; the first operand is already false, so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:61:23: replace < with <= in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `e <= 1e-30` is false, so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:61:23: replace < with == in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `e == 1e-30` is false, so the condition is still false |
| crates/geometry/src/extrema/curve_curve.rs:65:17: replace < with == in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `a` is never exactly 1e-30, so the branch is still not taken |
| crates/geometry/src/extrema/curve_curve.rs:65:17: replace < with > in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:65:17: replace < with <= in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `a <= 1e-30` is false, so the branch is still not taken |
| crates/geometry/src/extrema/curve_curve.rs:67:16: replace / with % in line_to_line | (b) | inside the `a < 1e-30` degenerate-point branch, which is unreachable for any constructible `Line3D` |
| crates/geometry/src/extrema/curve_curve.rs:67:16: replace / with * in line_to_line | (b) | inside the `a < 1e-30` degenerate-point branch, which is unreachable for any constructible `Line3D` |
| crates/geometry/src/extrema/curve_curve.rs:70:14: replace < with == in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `e` is never exactly 1e-30, so the branch is still not taken |
| crates/geometry/src/extrema/curve_curve.rs:70:14: replace < with <= in line_to_line | (b) | `Line3D::new` rejects zero directions and normalises them, so `a` and `e` (the squared direction lengths) are 1 for every constructible input; `e <= 1e-30` is false, so the branch is still not taken |
| crates/geometry/src/extrema/curve_curve.rs:72:21: replace / with % in line_to_line | (b) | inside the `e < 1e-30` degenerate-point branch, which is unreachable for any constructible `Line3D` |
| crates/geometry/src/extrema/curve_curve.rs:72:21: replace / with * in line_to_line | (b) | inside the `e < 1e-30` degenerate-point branch, which is unreachable for any constructible `Line3D` |
| crates/geometry/src/extrema/curve_curve.rs:72:18: delete - in line_to_line | (b) | inside the `e < 1e-30` degenerate-point branch, which is unreachable for any constructible `Line3D` |
| crates/geometry/src/extrema/curve_curve.rs:75:31: replace - with + in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:75:31: replace - with / in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:75:27: replace * with + in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:75:27: replace * with / in line_to_line | (b) | a / e` equals `a * e` because unit directions give `a == e == 1 |
| crates/geometry/src/extrema/curve_curve.rs:75:35: replace * with + in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:75:35: replace * with / in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:78:40: replace > with == in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:78:40: replace > with >= in line_to_line | (b) | differs only when `denom.abs()` is exactly 1e-30 |
| crates/geometry/src/extrema/curve_curve.rs:79:33: replace / with % in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:79:33: replace / with * in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:79:24: replace - with + in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:79:20: replace * with + in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:79:28: replace * with + in line_to_line | (a) | skew_lines_common_perpendicular_closed_form |
| crates/geometry/src/extrema/curve_curve.rs:79:28: replace * with / in line_to_line | (b) | `c / e` equals `c * e` to within one ulp for a unit direction; the proof re-run flipped it only via a knife-edge test in convert/recognize_surface.rs, not via this module |
| crates/geometry/src/extrema/curve_curve.rs:86:37: replace / with * in line_to_line | (b) | dividing and multiplying by `e == 1` agree to within one ulp; flipped only by the same foreign knife-edge test |
| crates/geometry/src/extrema/curve_curve.rs:92:18: replace > with == in line_to_line | (a) | clamping_the_second_parameter_refits_the_first |
| crates/geometry/src/extrema/curve_curve.rs:92:18: replace > with < in line_to_line | (a) | clamping_the_second_parameter_refits_the_first |
| crates/geometry/src/extrema/curve_curve.rs:92:18: replace > with >= in line_to_line | (b) | provable no-op: `a` is 1 for unit directions, so both forms take the recompute branch and produce identical floats; flipped only by the foreign knife-edge test |
| crates/geometry/src/extrema/curve_curve.rs:95:22: replace / with * in line_to_line | (b) | dividing and multiplying by `a == 1` agree to within one ulp; flipped only by the foreign knife-edge test |
| crates/geometry/src/extrema/curve_curve.rs:152:27: replace \|\| with && in curve_to_curve | (a) | degenerate_first_range_yields_a_finite_endpoint_solution |
| crates/geometry/src/extrema/curve_curve.rs:165:37: replace / with % in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided. Flipped by the foreign knife-edge test, not by this module |
| crates/geometry/src/extrema/curve_curve.rs:165:25: replace - with + in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:165:50: replace - with + in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:165:50: replace - with / in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:166:37: replace / with % in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:166:37: replace / with * in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:166:25: replace - with + in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:166:50: replace - with + in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:175:26: replace == with != in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:175:39: replace - with + in curve_to_curve | (b) | the mutated index test can never hold for `i < N_SAMPLES`, so the final sample falls through to `t_start + 31*step`, which is `t_end` |
| crates/geometry/src/extrema/curve_curve.rs:175:39: replace - with / in curve_to_curve | (b) | the mutated index test can never hold for `i < N_SAMPLES`, so the final sample falls through to `t_start + 31*step`, which is `t_end` |
| crates/geometry/src/extrema/curve_curve.rs:178:26: replace + with * in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:186:39: replace - with + in curve_to_curve | (b) | the mutated index test can never hold for `i < N_SAMPLES`, so the final sample falls through to `t_start + 31*step`, which is `t_end` |
| crates/geometry/src/extrema/curve_curve.rs:186:39: replace - with / in curve_to_curve | (b) | the mutated index test can never hold for `i < N_SAMPLES`, so the final sample falls through to `t_start + 31*step`, which is `t_end` |
| crates/geometry/src/extrema/curve_curve.rs:189:26: replace + with - in curve_to_curve | (a) | tilted_circles_closest_approach_on_the_centre_line |
| crates/geometry/src/extrema/curve_curve.rs:189:26: replace + with * in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:189:37: replace * with + in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:189:37: replace * with / in curve_to_curve | (c) | perturbs only the 32-sample seed grid; Gauss-Newton converges to the same local minimum from any seed, and separating the two needs a multiple-local-minimum configuration where which minimum a sample-then-refine solver should report is itself undecided |
| crates/geometry/src/extrema/curve_curve.rs:198:64: replace + with - in curve_to_curve | (a) | tilted_circles_closest_approach_on_the_centre_line |
| crates/geometry/src/extrema/curve_curve.rs:198:64: replace + with * in curve_to_curve | (c) | perturbs only the metric used to pick the seed pair; the refined result is unchanged for a single-minimum configuration, and a multiple-minimum fixture would pin implementation behaviour rather than the contract |
| crates/geometry/src/extrema/curve_curve.rs:198:75: replace * with + in curve_to_curve | (c) | perturbs only the metric used to pick the seed pair; the refined result is unchanged for a single-minimum configuration, and a multiple-minimum fixture would pin implementation behaviour rather than the contract |
| crates/geometry/src/extrema/curve_curve.rs:199:19: replace < with <= in curve_to_curve | (b) | changes only the tie-break among grid pairs with exactly equal squared distance; every tied seed is equally good and the contract does not fix which minimiser is reported |
| crates/geometry/src/extrema/curve_curve.rs:215:23: replace - with + in curve_to_curve | (b) | `h1` is only the finite-difference step; the mutated value is still a small positive step (or the 1e-9 floor), so the converged extremum is unchanged |
| crates/geometry/src/extrema/curve_curve.rs:216:23: replace - with + in curve_to_curve | (b) | `h2` is only the finite-difference step; the mutated value is still a small positive step (or the 1e-9 floor), so the converged extremum is unchanged |
| crates/geometry/src/extrema/curve_curve.rs:235:33: replace * with / in curve_to_curve | (a) | tilted_circles_closest_approach_on_the_centre_line |
| crates/geometry/src/extrema/curve_curve.rs:235:22: replace - with + in curve_to_curve | (a) | tilted_circles_closest_approach_on_the_centre_line |
| crates/geometry/src/extrema/curve_curve.rs:246:33: replace * with / in curve_to_curve | (a) | tilted_circles_closest_approach_on_the_centre_line |
| crates/geometry/src/extrema/curve_curve.rs:246:22: replace - with + in curve_to_curve | (a) | tilted_circles_closest_approach_on_the_centre_line |
| crates/geometry/src/extrema/curve_curve.rs:258:22: replace < with <= in curve_to_curve | (b) | differs only when `det.abs()` is exactly `f64::EPSILON` |
| crates/geometry/src/extrema/curve_curve.rs:268:44: replace && with \|\| in curve_to_curve | (a) | second_domain_narrower_than_param_tol_still_refines_the_first |
| crates/geometry/src/extrema/curve_curve.rs:268:32: replace < with == in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:32: replace < with > in curve_to_curve | (a) | second_domain_narrower_than_param_tol_still_refines_the_first |
| crates/geometry/src/extrema/curve_curve.rs:268:32: replace < with <= in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:20: replace - with + in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:20: replace - with / in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:67: replace < with == in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:67: replace < with > in curve_to_curve | (a) | first_domain_narrower_than_param_tol_still_refines_the_second |
| crates/geometry/src/extrema/curve_curve.rs:268:67: replace < with <= in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:55: replace - with + in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |
| crates/geometry/src/extrema/curve_curve.rs:268:55: replace - with / in curve_to_curve | (b) | only moves when the refinement loop stops; the mutated condition never fires earlier than the real one, so the loop runs to MAX_ITER and returns the same fixed point |

## crates/geometry/src/extrema/point_curve.rs

- before: 27 survivors; after: 9 survivors, 104 caught, 15 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/extrema/point_curve.rs:48:63: replace + with - in point_to_line | (a) | line_distance_uses_every_component_of_the_offset |
| crates/geometry/src/extrema/point_curve.rs:48:74: replace * with + in point_to_line | (a) | line_distance_uses_every_component_of_the_offset |
| crates/geometry/src/extrema/point_curve.rs:138:23: replace - with + in point_to_curve | (a) | generic_multi_turn_helix_picks_the_closest_turn |
| crates/geometry/src/extrema/point_curve.rs:138:47: replace - with + in point_to_curve | (b) | Sample count 63 to 65 intervals: the scan stays uniform over the same domain and the last sample is still forced to t_end; only a curve engineered against the exact grid alignment could see it, which pins sample positions rather than the documented contract |
| crates/geometry/src/extrema/point_curve.rs:138:47: replace - with / in point_to_curve | (b) | Sample count 63 to 64 intervals: the /64 grid falls within 1/4032 of the domain of every /63 sample, so no contract-derived fixture separates them |
| crates/geometry/src/extrema/point_curve.rs:143:35: replace - with + in point_to_curve | (b) | The i == N_SAMPLES-1 guard never fires, so the last sample becomes t_start + 63*step, which differs from t_end by at most one ulp and is clamped identically; only a bit-exact float assertion could detect it |
| crates/geometry/src/extrema/point_curve.rs:143:35: replace - with / in point_to_curve | (b) | Same as the + variant: the guard never fires and the last sample differs from t_end by at most one ulp |
| crates/geometry/src/extrema/point_curve.rs:150:60: replace + with - in point_to_curve | (a) | generic_multi_turn_helix_picks_the_closest_turn |
| crates/geometry/src/extrema/point_curve.rs:150:60: replace + with * in point_to_curve | (a) | generic_multi_turn_helix_picks_the_closest_turn |
| crates/geometry/src/extrema/point_curve.rs:150:38: replace + with - in point_to_curve | (a) | generic_circle_off_plane_query_matches_closed_form |
| crates/geometry/src/extrema/point_curve.rs:150:49: replace * with / in point_to_curve | (a) | generic_curve_coplanar_with_query_finds_global_minimum |
| crates/geometry/src/extrema/point_curve.rs:150:71: replace * with + in point_to_curve | (a) | generic_multi_turn_helix_picks_the_closest_turn |
| crates/geometry/src/extrema/point_curve.rs:168:20: replace - with + in point_to_curve | (a) | generic_offset_parameter_domain_uses_domain_scaled_step |
| crates/geometry/src/extrema/point_curve.rs:183:45: replace * with / in point_to_curve | (a) | generic_steep_descending_helix_keeps_interior_minimum |
| crates/geometry/src/extrema/point_curve.rs:186:53: replace + with - in point_to_curve | (a) | generic_steep_descending_helix_keeps_interior_minimum |
| crates/geometry/src/extrema/point_curve.rs:186:64: replace * with + in point_to_curve | (a) | generic_circle_off_plane_query_matches_closed_form |
| crates/geometry/src/extrema/point_curve.rs:189:52: replace + with - in point_to_curve | (a) | generic_steep_descending_helix_keeps_interior_minimum |
| crates/geometry/src/extrema/point_curve.rs:189:36: replace + with - in point_to_curve | (a) | generic_circle_off_plane_query_matches_closed_form |
| crates/geometry/src/extrema/point_curve.rs:189:60: replace * with + in point_to_curve | (a) | generic_steep_descending_helix_keeps_interior_minimum |
| crates/geometry/src/extrema/point_curve.rs:190:19: replace < with == in point_to_curve | (a) | generic_zero_velocity_curve_returns_finite_result |
| crates/geometry/src/extrema/point_curve.rs:190:19: replace < with <= in point_to_curve | (b) | Differs only when vel_sq equals f64::EPSILON exactly |
| crates/geometry/src/extrema/point_curve.rs:197:30: replace < with == in point_to_curve | (b) | The early exit effectively never fires, so the loop merely runs all 50 iterations from a fixed point it has already reached (the clamp makes endpoints fixed too); the returned t moves by less than PARAM_TOL |
| crates/geometry/src/extrema/point_curve.rs:197:30: replace < with <= in point_to_curve | (b) | Differs only when the parameter step magnitude equals PARAM_TOL exactly |
| crates/geometry/src/extrema/point_curve.rs:197:19: replace - with + in point_to_curve | (b) | The exit test becomes \|t_new + t\| < 1e-10, which never fires on a one-signed domain (the loop runs to MAX_ITER at the same fixed point) and can only fire when both iterates are already within 1e-10 of an answer at zero |
| crates/geometry/src/extrema/point_curve.rs:197:19: replace - with / in point_to_curve | (b) | The exit test becomes \|t_new / t\| < 1e-10, which fires only once the iterate has landed on t close to zero, which is already the returned answer; otherwise the loop runs to MAX_ITER unchanged |
| crates/geometry/src/extrema/point_curve.rs:206:63: replace + with - in point_to_curve | (a) | generic_circle_off_plane_query_matches_closed_form |
| crates/geometry/src/extrema/point_curve.rs:206:74: replace * with + in point_to_curve | (a) | generic_circle_off_plane_query_matches_closed_form |

## crates/geometry/src/extrema/segment.rs

- before: 27 survivors; after: 5 survivors, 59 caught, 19 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/extrema/segment.rs:47:63: replace + with * in segment_segment_distance | (a) | degenerate_both_points_distance_uses_every_component |
| crates/geometry/src/extrema/segment.rs:47:63: replace + with - in segment_segment_distance | (a) | degenerate_both_points_distance_uses_every_component |
| crates/geometry/src/extrema/segment.rs:47:41: replace + with - in segment_segment_distance | (a) | degenerate_both_points_distance_uses_every_component |
| crates/geometry/src/extrema/segment.rs:47:52: replace * with + in segment_segment_distance | (a) | degenerate_both_points_distance_uses_every_component |
| crates/geometry/src/extrema/segment.rs:47:74: replace * with + in segment_segment_distance | (a) | degenerate_both_points_distance_uses_every_component |
| crates/geometry/src/extrema/segment.rs:55:16: replace / with % in segment_segment_distance | (a) | degenerate_a_point_projects_to_interior_of_b |
| crates/geometry/src/extrema/segment.rs:55:16: replace / with * in segment_segment_distance | (a) | degenerate_a_point_projects_to_interior_of_b |
| crates/geometry/src/extrema/segment.rs:61:21: replace / with % in segment_segment_distance | (b) | Dead store: the branch is inside `else { a > 1e-30 }`, so this `s` is always shadowed by the line-81 recompute, which with `t = 0` and `b = d1·0 = 0` yields the same `(-c/a).clamp`. |
| crates/geometry/src/extrema/segment.rs:61:21: replace / with * in segment_segment_distance | (b) | Same dead store: the line-61 `s` is never read because line 81 unconditionally shadows it in this branch. |
| crates/geometry/src/extrema/segment.rs:61:18: delete - in segment_segment_distance | (b) | Same dead store: the line-61 `s` is never read because line 81 unconditionally shadows it in this branch. |
| crates/geometry/src/extrema/segment.rs:65:31: replace - with + in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:65:31: replace - with / in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:65:27: replace * with + in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:65:27: replace * with / in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:65:35: replace * with + in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:65:35: replace * with / in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:68:32: replace > with == in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:68:32: replace > with >= in segment_segment_distance | (b) | Differs only when `denom.abs()` is bit-exactly 1e-30; no realistic geometry lands on that single double value. |
| crates/geometry/src/extrema/segment.rs:69:34: replace / with % in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:34: replace / with * in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:25: replace - with + in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:25: replace - with / in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:21: replace * with + in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:21: replace * with / in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:29: replace * with + in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:69:29: replace * with / in segment_segment_distance | (a) | skew_non_perpendicular_interior_closest_approach |
| crates/geometry/src/extrema/segment.rs:81:18: replace > with >= in segment_segment_distance | (b) | Differs only when `a == 1e-30` exactly (`\|d1\| = 1e-15`), where the resulting point shift is bounded by 1e-15 — below any legitimate assertion tolerance. |

## crates/geometry/src/sampling/arc_length.rs

- before: 53 survivors; after: 24 survivors, 79 caught, 3 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/sampling/arc_length.rs:45:21: replace + with - in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:45:32: replace * with + in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:45:41: replace - with + in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:51:28: replace - with + in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (needs a fixture whose z varies) |
| crates/geometry/src/sampling/arc_length.rs:52:32: replace + with - in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| crates/geometry/src/sampling/arc_length.rs:52:17: replace * with / in sample_arc_length | (a) | arc_length_beats_uniform_parameter_on_an_ellipse (chord becomes ~constant, degrading to uniform-parameter sampling, which is invisible on a circle) |
| crates/geometry/src/sampling/arc_length.rs:52:27: replace * with / in sample_arc_length | (a) | arc_length_beats_uniform_parameter_on_an_ellipse (chord becomes ~constant, degrading to uniform-parameter sampling, which is invisible on a circle) |
| crates/geometry/src/sampling/arc_length.rs:52:37: replace * with + in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| crates/geometry/src/sampling/arc_length.rs:66:32: replace - with + in sample_arc_length | (b) | `i == n + 1` is never true, so `target` becomes `total_len*(n-1)/(n-1)` == `total_len`; and `t_final` at `i == n-1` is snapped to `t_end` by a separate comparison (line 97) regardless. |
| crates/geometry/src/sampling/arc_length.rs:66:32: replace - with / in sample_arc_length | (b) | `i == n / 1` is never true for `i in 0..n`; same reasoning as the `-` -> `+` variant at this site. |
| crates/geometry/src/sampling/arc_length.rs:74:37: replace < with <= in sample_arc_length | (b) | `seg_idx` shifts only when a target lands exactly on a chord-table knot; both brackets bisect to that same parameter, and the `i == 0` / `i == n-1` cases are clamped and snapped identically. |
| crates/geometry/src/sampling/arc_length.rs:76:23: replace - with + in sample_arc_length | (b) | The `.min(segs - 1)` clamp never binds: `target <= total_len == arc_table[segs]`, so `partition_point <= segs` and `seg_idx <= segs - 1` already. |
| crates/geometry/src/sampling/arc_length.rs:76:23: replace - with / in sample_arc_length | (b) | Same dead clamp: `.min(segs)` and `.min(segs - 1)` both leave the already-bounded `seg_idx` unchanged. |
| crates/geometry/src/sampling/arc_length.rs:79:36: replace + with * in sample_arc_length | (b) | `s1` is read only by the `t_approx` computation, which is discarded at line 101. |
| crates/geometry/src/sampling/arc_length.rs:81:34: replace + with * in sample_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (`t1` collapses to `t0`, so bisection returns the left edge of the chord-table segment: a ~1.2% angle-step error) |
| crates/geometry/src/sampling/arc_length.rs:85:43: replace < with == in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:85:43: replace < with > in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:85:43: replace < with <= in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:85:31: replace - with + in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:85:31: replace - with / in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:16: replace + with - in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:16: replace + with * in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:44: replace * with + in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:44: replace * with / in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:32: replace / with % in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:32: replace / with * in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:26: replace - with + in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:26: replace - with / in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:38: replace - with + in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:38: replace - with / in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:50: replace - with + in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:88:50: replace - with / in sample_arc_length | (b) | Dead code: the mutated expression only feeds `t_approx`, which is discarded at line 101 (`let _ = t_approx;`). |
| crates/geometry/src/sampling/arc_length.rs:133:28: replace - with + in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (off-origin centre makes `x + x` differ from `x - x`) |
| crates/geometry/src/sampling/arc_length.rs:133:28: replace - with / in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:134:28: replace - with + in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (off-origin centre makes `y + y` differ from `y - y`) |
| crates/geometry/src/sampling/arc_length.rs:134:28: replace - with / in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:135:28: replace - with + in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (off-origin centre makes `z + z` differ from `z - z`) |
| crates/geometry/src/sampling/arc_length.rs:135:28: replace - with / in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:40: replace + with - in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:40: replace + with * in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:30: replace + with - in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:30: replace + with * in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:25: replace * with + in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:25: replace * with / in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:35: replace * with + in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:35: replace * with / in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:136:45: replace * with + in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| crates/geometry/src/sampling/arc_length.rs:136:45: replace * with / in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle (needs dz != 0) |
| crates/geometry/src/sampling/arc_length.rs:137:36: replace + with - in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:137:36: replace + with * in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:139:23: replace < with == in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:139:23: replace < with > in bisect_arc_length | (a) | arc_length_on_tilted_circle_is_equal_angle |
| crates/geometry/src/sampling/arc_length.rs:139:23: replace < with <= in bisect_arc_length | (b) | Flips only on the exact float equality `arc_at_mid == target_arc` (measure zero); the bracket converges to the same root either way. |

## crates/geometry/src/sampling/curvature.rs

- before: 30 survivors; after: 3 survivors, 50 caught, 3 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/sampling/curvature.rs:16:19: replace < with > in curvature_at | (b) | `derivatives(t, 2)` always returns exactly `d+1 = 3` entries, so `< 3` and `> 3` are both constantly false — no observable change. |
| crates/geometry/src/sampling/curvature.rs:22:15: replace < with == in curvature_at | (a) | curvature_is_zero_where_the_first_derivative_vanishes |
| crates/geometry/src/sampling/curvature.rs:22:15: replace < with <= in curvature_at | (b) | Differs only when `\|C'\|` is exactly `f64::EPSILON`; the contract only covers a near-zero first derivative. |
| crates/geometry/src/sampling/curvature.rs:26:20: replace / with % in curvature_at | (a) | curvature_of_unit_radius_arc_is_one |
| crates/geometry/src/sampling/curvature.rs:26:39: replace * with + in curvature_at | (a) | curvature_of_unit_radius_arc_is_one |
| crates/geometry/src/sampling/curvature.rs:26:20: replace / with * in curvature_at | (a) | curvature_of_unit_radius_arc_is_one |
| crates/geometry/src/sampling/curvature.rs:26:39: replace * with / in curvature_at | (a) | curvature_of_unit_radius_arc_is_one |
| crates/geometry/src/sampling/curvature.rs:26:30: replace * with + in curvature_at | (a) | curvature_of_unit_radius_arc_is_one |
| crates/geometry/src/sampling/curvature.rs:26:30: replace * with / in curvature_at | (a) | curvature_of_unit_radius_arc_is_one |
| crates/geometry/src/sampling/curvature.rs:31:5: replace chord -> f64 with 1.0 | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:31:20: replace - with + in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:31:20: replace - with / in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:33:20: replace - with + in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:32:20: replace - with + in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:32:20: replace - with / in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:24: replace + with - in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:24: replace + with * in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:14: replace + with * in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:14: replace + with - in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:9: replace * with + in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:19: replace * with + in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:9: replace * with / in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:29: replace * with + in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:34:19: replace * with / in chord | (a) | chord_matches_the_euclidean_closed_form |
| crates/geometry/src/sampling/curvature.rs:62:40: replace + with * in subdivide | (a) | stop_rule_compares_curvature_times_interval_length |
| crates/geometry/src/sampling/curvature.rs:69:18: replace * with + in subdivide | (a) | stop_rule_compares_curvature_times_interval_length |
| crates/geometry/src/sampling/curvature.rs:74:59: replace + with * in subdivide | (a) | `subdivide_increments_depth_on_both_branches` (curvature.rs) |
| crates/geometry/src/sampling/curvature.rs:53:14: replace >= with < in subdivide | timeout | Inverts the recursion-depth guard; the pre-existing `recursion_limit_stops_without_appending_points` test then recurses unboundedly, so cargo-mutants reports Timeout (a distinct outcome from Missed) — not claimed as a kill. |
| crates/geometry/src/sampling/curvature.rs:69:18: replace * with / in subdivide | (a) | stop_rule_compares_curvature_times_interval_length |
| crates/geometry/src/sampling/curvature.rs:76:59: replace + with * in subdivide | (a) | `subdivide_increments_depth_on_both_branches` (curvature.rs) |

## crates/geometry/src/sampling/surface.rs

- before: 20 survivors; after: 0 survivors, 33 caught, 1 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/sampling/surface.rs:33:31: replace - with + in surface_grid | (a) | last_row_and_column_sit_exactly_on_the_range_ends |
| crates/geometry/src/sampling/surface.rs:33:31: replace - with / in surface_grid | (a) | last_row_and_column_sit_exactly_on_the_range_ends |
| crates/geometry/src/sampling/surface.rs:36:27: replace + with - in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:27: replace + with * in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:64: replace / with % in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:64: replace / with * in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:38: replace * with / in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:51: replace - with + in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:70: replace - with + in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:36:70: replace - with / in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:43:39: replace - with + in surface_grid | (a) | last_row_and_column_sit_exactly_on_the_range_ends |
| crates/geometry/src/sampling/surface.rs:43:39: replace - with / in surface_grid | (a) | last_row_and_column_sit_exactly_on_the_range_ends |
| crates/geometry/src/sampling/surface.rs:46:35: replace + with - in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:35: replace + with * in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:72: replace / with % in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:72: replace / with * in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:46: replace * with / in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:59: replace - with + in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:78: replace - with + in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |
| crates/geometry/src/sampling/surface.rs:46:78: replace - with / in surface_grid | (a) | interior_grid_parameters_follow_the_documented_formula |

## crates/geometry/src/sampling/uniform.rs

- before: 10 survivors; after: 0 survivors, 17 caught, 4 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/sampling/uniform.rs:42:34: replace / with % in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:42:34: replace / with * in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:42:23: replace - with + in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:42:39: replace - with + in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:42:39: replace - with / in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:45:31: replace - with + in sample_uniform_with_params | (a) | last_param_is_snapped_exactly_to_t_end |
| crates/geometry/src/sampling/uniform.rs:45:31: replace - with / in sample_uniform_with_params | (a) | last_param_is_snapped_exactly_to_t_end |
| crates/geometry/src/sampling/uniform.rs:48:25: replace + with - in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:48:25: replace + with * in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |
| crates/geometry/src/sampling/uniform.rs:48:36: replace * with / in sample_uniform_with_params | (a) | params_are_evenly_spaced_over_an_asymmetric_range |

## crates/geometry/src/sampling/deflection.rs

- before: 4 survivors; after: 1 survivors, 22 caught, 7 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/geometry/src/sampling/deflection.rs:16:15: replace < with == in chord_deviation | (a) | chord_deviation_matches_closed_form_and_degenerate_chord_is_zero |
| crates/geometry/src/sampling/deflection.rs:16:15: replace < with <= in chord_deviation | (b) | Differs only when `\|b - a\|` is exactly `f64::EPSILON`; the contract only specifies coincident `a` and `b`. |
| crates/geometry/src/sampling/deflection.rs:51:64: replace + with * in subdivide | (a) | `subdivide_increments_depth_on_both_branches` (deflection.rs) |
| crates/geometry/src/sampling/deflection.rs:53:64: replace + with * in subdivide | (a) | `subdivide_increments_depth_on_both_branches` (deflection.rs) |

## Notes

- `convert/surface_to_nurbs.rs` had 4 mutants and 0 survivors; it needed no work.
- The biggest single win is `extrema/point_surface.rs` (273 -> 51). Its survivors were almost all
  fixture artifacts: the pre-existing tests used axis-aligned quadrics centred at the origin with
  unit-ish radii, where many mutated operators agree numerically. Off-origin, off-axis fixtures with
  radii that are neither 0 nor 1 killed 222 of them against closed-form distances.
- `convert/recognize_surface.rs` (306 -> 109): exact rational patches built from arc chains and
  surfaces of revolution are what separate the recognizers; the chord error of a sampled patch is too
  loose. Two survivors (`261:24`, `274:25`) were alive purely because `cylinder_to_nurbs` puts the
  circle-fit centre at the origin, so a doubled or sign-flipped centre was invisible.
- `extrema/lipschitz.rs` (144 -> 62): the lever that worked was contract-level invariance — the
  optimizer must be invariant under scaling and under adding a constant to the objective. Fixtures
  that would kill more of the Lipschitz-bound arithmetic all sit on the pruning cliff (fail at bound
  x1.5, pass at x2), so they were left as (c) rather than landed as brittle gates.
- Residual (b) mutants concentrate in three provably unobservable shapes: `Vec::with_capacity` hints,
  strict/non-strict flips at a hard-coded epsilon (1e-15, 1e-30, `f64::EPSILON`) that differ only when
  a float equals the epsilon bit-exactly, and dead stores or unreachable guard branches.
- The one timeout, `sampling/curvature.rs:53:14: replace >= with < in subdivide`, inverts the
  recursion-depth guard; the pre-existing `recursion_limit_stops_without_appending_points` test then
  recurses unboundedly. A test cannot pass against a hang, so it is recorded as a timeout, not a kill.

## Production issues observed while triaging (not fixed here — test-only PR)

- `sampling/arc_length.rs` computes `t_approx` (lines 85-89) and then discards it (`let _ = t_approx;`
  at line 101), despite a comment saying it is a fallback. That single dead expression accounts for
  **17 of the 24** residual survivors in the file; deleting or wiring it up would remove them outright.
- `extrema/point_curve.rs` and `extrema/curve_curve.rs` refine with a Gauss-Newton step that drops the
  curvature term, so the iteration is only stable while the query is within about twice the local
  curvature radius. Beyond that it oscillates or flees to a domain bound and returns a non-stationary
  point. `extrema/point_surface.rs` shows the same shape on a moderately warped bi-quadratic patch
  (returned d = 3.92 against a true minimum of 3.51). This is why several `(c)` rows in those files
  cannot be pinned without first deciding what the right answer is.
- `extrema/point_surface.rs` `point_to_cone` lines 170-172: for a query exactly on the cone axis
  inside the nappe it returns a point on the axis, which is not on the cone, and reports `h*cos^2(a)`
  where the true distance is `h*cos(a)`. Ten `(c)` rows hang on that.

## Reproduce

```
cargo mutants -p remus-geometry --no-config --baseline skip --timeout 60 -j 8 -- -p remus-geometry
```

Raw logs (crate before-sweep, completion run, authoritative after-sweep, per-file agent runs) live in
untracked `mutants.out` directories under the session scratchpad and are not committed.

