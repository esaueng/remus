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
