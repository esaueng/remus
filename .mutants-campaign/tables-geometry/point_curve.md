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
