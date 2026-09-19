| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/edge.rs:61:38: replace && with \|\| in <impl remus_math::diagnostic::ToDiagnostic for EdgeDomainError>::diagnostic | (a) | `invalid_domain_diagnostic_omits_non_finite_bounds` |
| crates/topology/src/edge.rs:161:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn` |
| crates/topology/src/edge.rs:161:43: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn` |
| crates/topology/src/edge.rs:161:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn` |
| crates/topology/src/edge.rs:165:49: replace - with + in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:165:49: replace - with / in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:166:42: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn` |
| crates/topology/src/edge.rs:166:42: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:166:42: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `conic_delta_exactly_at_the_degenerate_threshold_stays_a_sliver_arc` |
| crates/topology/src/edge.rs:167:29: replace + with - in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:167:29: replace + with * in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:171:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn` |
| crates/topology/src/edge.rs:171:43: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn` |
| crates/topology/src/edge.rs:171:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn` |
| crates/topology/src/edge.rs:175:49: replace - with + in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:175:49: replace - with / in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:176:42: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn` |
| crates/topology/src/edge.rs:176:42: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:176:42: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `conic_delta_exactly_at_the_degenerate_threshold_stays_a_sliver_arc` |
| crates/topology/src/edge.rs:177:29: replace + with - in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:177:29: replace + with * in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints` |
| crates/topology/src/edge.rs:192:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Coincident endpoints — the only case the guard exists for — reach the same full domain through the fall-through: a zero parameter span fails the non-degenerate test. Differing needs a nonzero sub-1e-9 chord whose parameter span still exceeds 1e-6 of the domain, i.e. a curve shorter than the 1e-5 weld band. |
| crates/topology/src/edge.rs:192:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Differs only for a chord of exactly 1e-9; below it both branches return the full domain (see the == row). |
| crates/topology/src/edge.rs:197:53: replace && with \|\| in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_needs_both_ends_within_the_band` |
| crates/topology/src/edge.rs:197:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_is_measured_against_the_curve_ends` |
| crates/topology/src/edge.rs:197:43: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_is_measured_against_the_curve_ends` |
| crates/topology/src/edge.rs:197:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_excludes_its_own_edge` |
| crates/topology/src/edge.rs:197:76: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_is_measured_against_the_curve_ends` |
| crates/topology/src/edge.rs:197:76: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_is_measured_against_the_curve_ends` |
| crates/topology/src/edge.rs:197:76: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_excludes_its_own_edge` |
| crates/topology/src/edge.rs:198:55: replace && with \|\| in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_reversed_whole_edge_match_needs_both_ends_within_the_band` |
| crates/topology/src/edge.rs:198:45: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_excludes_its_own_edge` |
| crates/topology/src/edge.rs:198:80: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_whole_edge_match_band_excludes_its_own_edge` |
| crates/topology/src/edge.rs:214:36: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_on_curve_weld_band_excludes_its_own_edge` |
| crates/topology/src/edge.rs:215:40: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_on_curve_weld_band_excludes_its_own_edge` |
| crates/topology/src/edge.rs:216:37: replace > with >= in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Differs only when the projected \|dt\| equals 1e-6 × the domain span bit-exactly; dt comes out of Newton refinement, so no fixture can pin it to that value. |
| crates/topology/src/edge.rs:216:44: replace * with / in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_degenerate_span_threshold_is_relative_to_the_knot_domain` |
| crates/topology/src/edge.rs:216:50: replace - with + in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_degenerate_span_threshold_is_relative_to_the_knot_domain` |
| crates/topology/src/edge.rs:217:32: replace > with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | `nurbs_forward_sub_span_on_a_closed_curve_is_trimmed` |
| crates/topology/src/edge.rs:217:32: replace > with >= in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Unreachable: >= only differs at dt == 0.0, which the preceding \|dt\| > 1e-6 × (d1 - d0) test has already rejected. |
| crates/topology/src/edge.rs:336:9: replace Edge::set_start with () | (a) | `set_start_and_set_end_rewire_the_bounding_vertices` |
| crates/topology/src/edge.rs:341:9: replace Edge::set_end with () | (a) | `set_start_and_set_end_rewire_the_bounding_vertices` |
| crates/topology/src/edge.rs:400:21: replace && with \|\| in Edge::strict_domain | (a) | `strict_domain_rejects_a_line_trim_matching_only_one_end` |
| crates/topology/src/edge.rs:419:13: replace \|\| with && in Edge::strict_domain | (a) | `strict_domain_rejects_a_single_non_finite_bound` |
| crates/topology/src/edge.rs:432:82: replace \|\| with && in Edge::strict_domain | (a) | `strict_domain_rejects_every_out_of_range_nurbs_bound` |
| crates/topology/src/edge.rs:432:60: replace \|\| with && in Edge::strict_domain | (a) | `strict_domain_rejects_every_out_of_range_nurbs_bound` |
| crates/topology/src/edge.rs:437:61: delete ! in Edge::strict_domain | (a) | `strict_domain_accepts_a_finite_open_conic_span` |
| crates/topology/src/edge.rs:440:69: delete ! in Edge::strict_domain | (a) | `strict_domain_accepts_a_finite_open_conic_span` |
| crates/topology/src/edge.rs:445:61: delete ! in Edge::strict_domain | (a) | `strict_domain_accepts_a_finite_open_conic_span` |
| crates/topology/src/edge.rs:448:69: delete ! in Edge::strict_domain | (a) | `strict_domain_accepts_a_finite_open_conic_span` |
| crates/topology/src/edge.rs:519:34: replace * with + in periodic_domain_is_valid | (a) | `strict_domain_rejects_a_closed_turn_overrunning_by_more_than_roundoff` |
| crates/topology/src/edge.rs:526:9: replace \|\| with && in periodic_domain_is_valid | (a) | `periodic_domain_guard_refuses_a_negative_tolerance_claim` |
| crates/topology/src/edge.rs:527:49: replace > with >= in periodic_domain_is_valid | (b) | Unreachable: for any span in [4, 8) the difference span - TAU is an integer multiple of 2^-50, while the allowance 4 * EPSILON * TAU is 6.28 * 2^-50 — equality is not representable; a span outside that range misses TAU by more than 2. |
| crates/topology/src/edge.rs:581:49: replace * with + in periodic_curve_is_finite | (a) | `periodic_curve_is_finite_multiplies_axis_components_by_their_extent` |
| crates/topology/src/edge.rs:582:49: replace * with + in periodic_curve_is_finite | (a) | `periodic_curve_is_finite_multiplies_axis_components_by_their_extent` |
| crates/topology/src/edge.rs:582:49: replace * with / in periodic_curve_is_finite | (a) | `periodic_curve_is_finite_rejects_a_center_plus_extent_overflow` |
| crates/topology/src/edge.rs:583:37: replace + with - in periodic_curve_is_finite | (a) | `periodic_curve_is_finite_rejects_a_center_plus_extent_overflow` |
| crates/topology/src/edge.rs:584:56: replace + with * in periodic_curve_is_finite | (a) | `periodic_curve_is_finite_adds_the_center_to_the_tangent_bound` |
