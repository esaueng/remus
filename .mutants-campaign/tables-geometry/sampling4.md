| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| `crates/geometry/src/sampling/curvature.rs:16:19: replace < with > in curvature_at` | (b) | `derivatives(t, 2)` always returns exactly `d+1 = 3` entries, so `< 3` and `> 3` are both constantly false — no observable change. |
| `crates/geometry/src/sampling/curvature.rs:22:15: replace < with == in curvature_at` | (a) | `curvature_is_zero_where_the_first_derivative_vanishes` |
| `crates/geometry/src/sampling/curvature.rs:22:15: replace < with <= in curvature_at` | (b) | Differs only when `\|C'\|` is exactly `f64::EPSILON`; the contract only covers a near-zero first derivative. |
| `crates/geometry/src/sampling/curvature.rs:26:20: replace / with % in curvature_at` | (a) | `curvature_of_unit_radius_arc_is_one` |
| `crates/geometry/src/sampling/curvature.rs:26:39: replace * with + in curvature_at` | (a) | `curvature_of_unit_radius_arc_is_one` |
| `crates/geometry/src/sampling/curvature.rs:26:20: replace / with * in curvature_at` | (a) | `curvature_of_unit_radius_arc_is_one` |
| `crates/geometry/src/sampling/curvature.rs:26:39: replace * with / in curvature_at` | (a) | `curvature_of_unit_radius_arc_is_one` |
| `crates/geometry/src/sampling/curvature.rs:26:30: replace * with + in curvature_at` | (a) | `curvature_of_unit_radius_arc_is_one` |
| `crates/geometry/src/sampling/curvature.rs:26:30: replace * with / in curvature_at` | (a) | `curvature_of_unit_radius_arc_is_one` |
| `crates/geometry/src/sampling/curvature.rs:31:5: replace chord -> f64 with 1.0` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:31:20: replace - with + in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:31:20: replace - with / in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:33:20: replace - with + in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:32:20: replace - with + in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:32:20: replace - with / in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:24: replace + with - in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:24: replace + with * in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:14: replace + with * in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:14: replace + with - in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:9: replace * with + in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:19: replace * with + in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:9: replace * with / in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:29: replace * with + in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:34:19: replace * with / in chord` | (a) | `chord_matches_the_euclidean_closed_form` |
| `crates/geometry/src/sampling/curvature.rs:62:40: replace + with * in subdivide` | (a) | `stop_rule_compares_curvature_times_interval_length` |
| `crates/geometry/src/sampling/curvature.rs:69:18: replace * with + in subdivide` | (a) | `stop_rule_compares_curvature_times_interval_length` |
| `crates/geometry/src/sampling/curvature.rs:74:59: replace + with * in subdivide` | (a) | `subdivide_increments_depth_on_both_branches` (curvature.rs) |
| `crates/geometry/src/sampling/curvature.rs:53:14: replace >= with < in subdivide` | timeout | Inverts the recursion-depth guard; the pre-existing `recursion_limit_stops_without_appending_points` test then recurses unboundedly, so cargo-mutants reports Timeout (a distinct outcome from Missed) — not claimed as a kill. |
| `crates/geometry/src/sampling/curvature.rs:69:18: replace * with / in subdivide` | (a) | `stop_rule_compares_curvature_times_interval_length` |
| `crates/geometry/src/sampling/curvature.rs:76:59: replace + with * in subdivide` | (a) | `subdivide_increments_depth_on_both_branches` (curvature.rs) |
| `crates/geometry/src/sampling/surface.rs:33:31: replace - with + in surface_grid` | (a) | `last_row_and_column_sit_exactly_on_the_range_ends` |
| `crates/geometry/src/sampling/surface.rs:33:31: replace - with / in surface_grid` | (a) | `last_row_and_column_sit_exactly_on_the_range_ends` |
| `crates/geometry/src/sampling/surface.rs:36:27: replace + with - in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:27: replace + with * in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:64: replace / with % in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:64: replace / with * in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:38: replace * with / in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:51: replace - with + in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:70: replace - with + in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:36:70: replace - with / in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:43:39: replace - with + in surface_grid` | (a) | `last_row_and_column_sit_exactly_on_the_range_ends` |
| `crates/geometry/src/sampling/surface.rs:43:39: replace - with / in surface_grid` | (a) | `last_row_and_column_sit_exactly_on_the_range_ends` |
| `crates/geometry/src/sampling/surface.rs:46:35: replace + with - in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:35: replace + with * in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:72: replace / with % in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:72: replace / with * in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:46: replace * with / in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:59: replace - with + in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:78: replace - with + in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/surface.rs:46:78: replace - with / in surface_grid` | (a) | `interior_grid_parameters_follow_the_documented_formula` |
| `crates/geometry/src/sampling/uniform.rs:42:34: replace / with % in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:42:34: replace / with * in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:42:23: replace - with + in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:42:39: replace - with + in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:42:39: replace - with / in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:45:31: replace - with + in sample_uniform_with_params` | (a) | `last_param_is_snapped_exactly_to_t_end` |
| `crates/geometry/src/sampling/uniform.rs:45:31: replace - with / in sample_uniform_with_params` | (a) | `last_param_is_snapped_exactly_to_t_end` |
| `crates/geometry/src/sampling/uniform.rs:48:25: replace + with - in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:48:25: replace + with * in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/uniform.rs:48:36: replace * with / in sample_uniform_with_params` | (a) | `params_are_evenly_spaced_over_an_asymmetric_range` |
| `crates/geometry/src/sampling/deflection.rs:16:15: replace < with == in chord_deviation` | (a) | `chord_deviation_matches_closed_form_and_degenerate_chord_is_zero` |
| `crates/geometry/src/sampling/deflection.rs:16:15: replace < with <= in chord_deviation` | (b) | Differs only when `\|b - a\|` is exactly `f64::EPSILON`; the contract only specifies coincident `a` and `b`. |
| `crates/geometry/src/sampling/deflection.rs:51:64: replace + with * in subdivide` | (a) | `subdivide_increments_depth_on_both_branches` (deflection.rs) |
| `crates/geometry/src/sampling/deflection.rs:53:64: replace + with * in subdivide` | (a) | `subdivide_increments_depth_on_both_branches` (deflection.rs) |
