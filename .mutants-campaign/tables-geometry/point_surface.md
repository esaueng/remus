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
