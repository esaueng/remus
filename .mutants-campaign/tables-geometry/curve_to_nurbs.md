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
