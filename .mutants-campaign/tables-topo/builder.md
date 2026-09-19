| Mutant | Verdict | Killing test / reason |
|---|---|---|
| crates/topology/src/builder.rs:34:39: replace < with <= in make_line_edge | (a) | make_line_edge_guard_is_the_squared_linear_tolerance |
| crates/topology/src/builder.rs:34:52: replace * with + in make_line_edge | (a) | make_line_edge_guard_is_the_squared_linear_tolerance |
| crates/topology/src/builder.rs:34:52: replace * with / in make_line_edge | (a) | make_line_edge_guard_is_the_squared_linear_tolerance |
| crates/topology/src/builder.rs:71:31: replace \|\| with && in make_circle_edge_with_ref | (b) | every tolerance the guard rejects (non-finite or negative) is rejected again by the full-turn `periodic_domain_is_valid` check below, so the builder still returns `Err` with no arena mutation; only the reason string differs. |
| crates/topology/src/builder.rs:149:31: replace \|\| with && in make_ellipse_edge_with_ref | (b) | same as the circle builder - the full-turn certification rejects exactly the same tolerance set, so the result is still `Err` with no arena mutation. |
| crates/topology/src/builder.rs:268:9: replace \|\| with && in make_ellipse_arc | (a) | make_ellipse_arc_refuses_endpoints_without_an_angular_span |
| crates/topology/src/builder.rs:267:9: replace \|\| with && in make_ellipse_arc | (b) | `t_start`/`span` can only be non-finite if an endpoint projection is, which the preceding on-ellipse residual test (`!residual.is_finite()`) already rejected; the two unreachable disjuncts cannot change the verdict. |
| crates/topology/src/builder.rs:268:13: delete ! in make_ellipse_arc | (a) | make_ellipse_arc_refuses_endpoints_without_an_angular_span |
| crates/topology/src/builder.rs:307:10: replace < with <= in make_polygon_wire | (a) | make_polygon_wire_links_consecutive_points_and_closes_the_loop |
| crates/topology/src/builder.rs:320:32: replace % with / in make_polygon_wire | (a) | make_polygon_wire_links_consecutive_points_and_closes_the_loop |
| crates/topology/src/builder.rs:320:27: replace + with * in make_polygon_wire | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:348:16: replace < with == in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:348:16: replace < with <= in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:362:47: replace / with % in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:362:34: replace * with + in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:362:29: replace * with + in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:362:29: replace * with / in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:363:32: replace * with + in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:363:32: replace * with / in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:363:54: replace * with + in make_regular_polygon_wire | (a) | make_regular_polygon_wire_places_vertices_on_the_circle |
| crates/topology/src/builder.rs:490:38: replace + with - in sample_wire_for_planarity | (a) | fallback_patch_corners_stay_inside_the_wire_bounds |
| crates/topology/src/builder.rs:554:39: replace + with - in sample_wire_for_planarity | (b) | `ordered` feeds only `newell_normal`, and the reflected sample `1.5*s_i - 0.5*s_(i+1)` contributes `(-0.5 + 1.5) * cross(s_i, s_(i+1))` - identical to the real midpoint's `0.5 + 0.5` - so the winding vector is unchanged. |
| crates/topology/src/builder.rs:580:5: replace sample_open_conic with () | (a) | open_conic_samples_carry_the_non_planarity |
| crates/topology/src/builder.rs:583:31: replace + with - in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:31: replace + with * in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:60: replace / with % in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:60: replace / with * in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:45: replace * with + in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:45: replace * with / in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:38: replace - with + in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:583:38: replace - with / in sample_open_conic | (a) | open_conic_is_sampled_between_its_endpoints |
| crates/topology/src/builder.rs:601:50: replace / with % in sample_conic | (a) | closed_conic_is_sampled_at_its_quadrant_points |
| crates/topology/src/builder.rs:598:5: replace sample_conic with () | (a) | closed_conic_is_sampled_at_its_quadrant_points |
| crates/topology/src/builder.rs:601:50: replace / with * in sample_conic | (a) | closed_conic_samples_span_the_whole_turn |
| crates/topology/src/builder.rs:601:35: replace * with + in sample_conic | (a) | closed_conic_is_sampled_at_its_quadrant_points |
| crates/topology/src/builder.rs:601:35: replace * with / in sample_conic | (a) | closed_conic_samples_span_the_whole_turn |
| crates/topology/src/builder.rs:607:25: replace - with + in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:608:14: replace > with == in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:607:25: replace - with / in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:608:14: replace > with < in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:608:14: replace > with >= in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:609:15: replace -= with += in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:609:15: replace -= with /= in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:610:21: replace < with == in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:610:21: replace < with > in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:610:21: replace < with <= in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:610:23: delete - in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:611:15: replace += with -= in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:611:15: replace += with *= in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:614:31: replace + with - in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:614:31: replace + with * in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:614:54: replace / with % in sample_conic | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:614:54: replace / with * in sample_conic | (a) | closed_conic_is_sampled_at_its_quadrant_points |
| crates/topology/src/builder.rs:614:39: replace * with + in sample_conic | (a) | conic_sub_arc_across_the_seam_samples_the_short_way |
| crates/topology/src/builder.rs:614:39: replace * with / in sample_conic | (a) | closed_conic_samples_span_the_whole_turn |
| crates/topology/src/builder.rs:633:44: replace - with + in verified_plane | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:634:24: replace > with >= in verified_plane | (a) | planarity_accepts_a_sample_exactly_at_the_effective_tolerance |
| crates/topology/src/builder.rs:645:39: replace < with <= in verified_plane | (b) | the winding normal is Newell's normal of samples just verified to lie in the plane, so it is +/- the plane normal and the dot product is +/-1, never exactly 0. |
| crates/topology/src/builder.rs:648:13: delete - in verified_plane | (a) | plane_face_records_the_signed_offset_of_the_wire |
| crates/topology/src/builder.rs:657:22: replace < with == in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:657:22: replace < with <= in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:667:12: replace += with -= in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:667:12: replace += with *= in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:667:37: replace * with + in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:667:25: replace - with + in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:667:49: replace + with - in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:667:49: replace + with * in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:668:12: replace += with -= in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:668:12: replace += with *= in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:668:37: replace * with + in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:668:25: replace - with + in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:668:49: replace + with - in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:668:49: replace + with * in newell_normal | (a) | triangle_plane_normal_follows_the_winding_in_every_axis_plane |
| crates/topology/src/builder.rs:669:49: replace + with * in newell_normal | (a) | conic_sub_arc_is_sampled_along_the_traversed_arc |
| crates/topology/src/builder.rs:678:21: replace < with == in fit_plane | (a) | three_sample_points_are_enough_for_both_plane_and_fallback |
| crates/topology/src/builder.rs:678:21: replace < with <= in fit_plane | (a) | three_sample_points_are_enough_for_both_plane_and_fallback |
| crates/topology/src/builder.rs:687:42: replace > with >= in fit_plane | (b) | only reorders equally distant samples in a tie; the plane through the extreme triple is the same, and its sign is fixed afterwards by the winding step. |
| crates/topology/src/builder.rs:696:42: replace > with >= in fit_plane | (b) | same tie-only reordering for the max-off-line sample; the fitted plane is unchanged. |
| crates/topology/src/builder.rs:703:70: replace * with + in fit_plane | (a) | needle_thin_wire_still_fits_a_plane |
| crates/topology/src/builder.rs:703:38: replace * with / in fit_plane | (a) | needle_thin_wire_still_fits_a_plane |
| crates/topology/src/builder.rs:716:21: replace < with == in bilinear_surface | (a) | three_sample_points_are_enough_for_both_plane_and_fallback |
| crates/topology/src/builder.rs:716:21: replace < with <= in bilinear_surface | (a) | three_sample_points_are_enough_for_both_plane_and_fallback |
| crates/topology/src/builder.rs:784:20: replace / with * in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:785:21: replace / with % in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:785:21: replace / with * in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:787:21: delete - in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:787:26: delete - in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:788:25: delete - in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:790:21: delete - in make_rectangle_face | (a) | make_rectangle_face_corners_are_the_half_extents_in_ccw_order |
| crates/topology/src/builder.rs:854:51: replace && with \|\| in make_nurbs_edge | (a) | make_nurbs_edge_refuses_authority_for_an_unusable_tolerance |
| crates/topology/src/builder.rs:857:9: delete - in make_nurbs_edge | (a) | make_nurbs_edge_refuses_authority_for_an_unusable_tolerance |
| crates/topology/src/builder.rs:862:9: delete match arm (true, false) in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:864:25: replace match guard forward_residual < reverse_residual with true in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:864:25: replace match guard forward_residual < reverse_residual with false in make_nurbs_edge | (b) | with the forward residual smaller, the fall-through stores the same forward domain - via arm 866 when the curve's ends are inside the band, otherwise via the reconstruction block, which returns the full domain and proves it with those same residuals. |
| crates/topology/src/builder.rs:865:25: replace match guard reverse_residual < forward_residual with true in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:865:25: replace match guard reverse_residual < forward_residual with false in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:866:25: replace match guard (natural_start - natural_end).length() <= authority_band with true in make_nurbs_edge | (b) | forcing the arm only matters when the ends are further apart than the band, and there the reconstruction block already returns and proves the same full domain. |
| crates/topology/src/builder.rs:866:25: replace match guard (natural_start - natural_end).length() <= authority_band with false in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:864:42: replace < with == in make_nurbs_edge | (b) | differs only on a residual tie, where the arm and the fall-through both store the forward domain. |
| crates/topology/src/builder.rs:864:42: replace < with > in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:864:42: replace < with <= in make_nurbs_edge | (b) | same tie-only difference - both paths store the forward domain. |
| crates/topology/src/builder.rs:865:42: replace < with == in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:865:42: replace < with > in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:865:42: replace < with <= in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:866:64: replace <= with > in make_nurbs_edge | (a) | make_nurbs_edge_endpoint_match_selects_the_stored_range |
| crates/topology/src/builder.rs:876:65: replace > with >= in make_nurbs_edge | (a) | make_nurbs_edge_interior_recovery_needs_a_gap_beyond_the_band |
| crates/topology/src/builder.rs:890:13: replace && with \|\| in make_nurbs_edge | (a) | make_nurbs_edge_interior_span_is_proven_at_both_ends |
| crates/topology/src/builder.rs:883:17: replace > with >= in make_nurbs_edge | (b) | would need the reconstructed span to equal `1e-12 * domain width` bit-exactly; `reconstruct_domain_from_endpoints` only returns spans wider than `1e-6 *` the domain. |
| crates/topology/src/builder.rs:882:29: replace - with + in make_nurbs_edge | (a) | make_nurbs_edge_interior_span_is_proven_at_both_ends |
| crates/topology/src/builder.rs:882:29: replace - with / in make_nurbs_edge | (a) | make_nurbs_edge_interior_span_is_proven_at_both_ends |
| crates/topology/src/builder.rs:883:25: replace * with / in make_nurbs_edge | (b) | a reconstructed candidate always spans more than `1e-6 *` the domain width, which clears both `1e-12 * width` and `1e-12 / width`, so the guard's verdict cannot change. |
| crates/topology/src/builder.rs:883:39: replace - with + in make_nurbs_edge | (a) | make_nurbs_edge_span_floor_scales_with_the_domain_width |
| crates/topology/src/builder.rs:951:66: replace % with / in make_nurbs_face | (a) | make_nurbs_face_wire_visits_the_domain_corners_in_order |
| crates/topology/src/builder.rs:951:66: replace % with + in make_nurbs_face | (a) | make_nurbs_face_wire_visits_the_domain_corners_in_order |
| crates/topology/src/builder.rs:951:61: replace + with - in make_nurbs_face | (a) | make_nurbs_face_wire_visits_the_domain_corners_in_order |
| crates/topology/src/builder.rs:951:61: replace + with * in make_nurbs_face | (a) | make_nurbs_face_wire_visits_the_domain_corners_in_order |
