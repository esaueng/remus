Mutant | Verdict | Killing test / reason
--- | --- | ---
crates/topology/src/validation.rs:37:5: replace validate_wire_closed -> Result<(), TopologyError> with Ok(()) | (a) | wire_closure_rejects_an_open_chain_and_a_broken_ring
crates/topology/src/validation.rs:43:18: replace == with != in validate_wire_closed | (a) | wire_closure_rejects_an_open_chain_and_a_broken_ring
crates/topology/src/validation.rs:48:35: replace <= with > in validate_wire_closed | (a) | wire_closure_accepts_coincident_but_distinct_endpoint_vertices
crates/topology/src/validation.rs:162:33: replace == with != in validate_shell_closed | (a) | closed_shell_report_names_free_and_over_shared_edges
crates/topology/src/validation.rs:210:13: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_diverging_use_count
crates/topology/src/validation.rs:209:13: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_diverging_closure_flag
crates/topology/src/validation.rs:218:17: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_flipped_compatibility_orientation
crates/topology/src/validation.rs:217:17: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_flipped_compatibility_orientation
crates/topology/src/validation.rs:731:5: replace pcurve_type_label -> &'static str with "" | (a) | proof_refusal_names_the_stored_pcurve_type
crates/topology/src/validation.rs:731:5: replace pcurve_type_label -> &'static str with "xyzzy" | (a) | proof_refusal_names_the_stored_pcurve_type
crates/topology/src/validation.rs:740:5: replace pcurve_definition_is_finite -> bool with true | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:743:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:746:69: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:752:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:751:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:750:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:757:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:756:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation
crates/topology/src/validation.rs:800:24: replace \|\| with && in check_same_parameter | (a) | sampled_parameter_reports_max_for_a_half_open_parameter_range
crates/topology/src/validation.rs:804:30: replace + with - in check_same_parameter | (a) | sampled_parameter_reports_max_for_a_half_open_parameter_range
crates/topology/src/validation.rs:804:30: replace + with * in check_same_parameter | (a) | sampled_parameter_reports_max_for_a_half_open_parameter_range
crates/topology/src/validation.rs:809:26: replace / with % in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:809:26: replace / with * in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:810:25: replace * with / in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:810:31: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:812:28: replace \|\| with && in check_same_parameter | (a) | a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines
crates/topology/src/validation.rs:823:20: replace * with / in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:823:26: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:825:16: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map
crates/topology/src/validation.rs:825:16: replace - with / in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map
crates/topology/src/validation.rs:825:20: replace * with + in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map
crates/topology/src/validation.rs:825:20: replace * with / in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map
crates/topology/src/validation.rs:825:26: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map
crates/topology/src/validation.rs:825:26: replace - with / in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map
crates/topology/src/validation.rs:832:13: replace \|\| with && in check_same_parameter | (a) | an_overflowing_sampled_deviation_reports_the_fail_closed_sentinel
crates/topology/src/validation.rs:831:13: replace \|\| with && in check_same_parameter | (b) | Mutant reads `... \|\| (!on_surface.finite && !on_curve.finite) \|\| !deviation.finite`; same implication chain — a non-finite operand always forces a non-finite deviation — so the regrouped form has the same value.
crates/topology/src/validation.rs:830:13: replace \|\| with && in check_same_parameter | (b) | `&&` binds tighter, so the mutant reads `(!g.finite && !on_surface.finite) \|\| !on_curve.finite \|\| !deviation.finite`; a non-finite g, surface point or curve point each force a non-finite deviation, so both forms reduce to the last disjunct.
crates/topology/src/validation.rs:838:22: replace > with >= in check_same_parameter | (a) | sampled_parameter_witnesses_the_first_maximal_sample
crates/topology/src/validation.rs:846:26: replace + with - in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:846:26: replace + with * in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map
crates/topology/src/validation.rs:878:24: replace \|\| with && in check_same_parameter_strict | (a) | non_finite_parameter_bounds_are_caught_before_evaluation
crates/topology/src/validation.rs:896:53: replace \|\| with && in check_same_parameter_strict | (a) | a_single_non_finite_pcurve_endpoint_is_caught_before_the_surface
crates/topology/src/validation.rs:921:13: replace \|\| with && in check_same_parameter_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof
crates/topology/src/validation.rs:920:13: replace \|\| with && in check_same_parameter_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof
crates/topology/src/validation.rs:919:13: replace \|\| with && in check_same_parameter_strict | (b) | Mutant reads `(!on_surface_start.finite && !on_surface_end.finite) \|\| !d0.finite \|\| !d1.finite`; a non-finite surface image forces its own endpoint distance non-finite, so both forms reduce to `!d0.finite \|\| !d1.finite`.
crates/topology/src/validation.rs:952:29: replace > with >= in check_same_parameter_strict | (a) | strict_parameter_certifies_exactly_at_the_linear_tolerance
crates/topology/src/validation.rs:962:56: replace >= with < in check_same_parameter_strict | (a) | strict_parameter_reports_the_larger_endpoint_deviation_plus_its_bound
crates/topology/src/validation.rs:964:47: replace + with - in check_same_parameter_strict | (a) | strict_parameter_reports_the_larger_endpoint_deviation_plus_its_bound
crates/topology/src/validation.rs:1000:33: replace > with >= in validate_same_parameter_strict | (a) | a_deviation_exactly_at_the_bound_is_inside_every_band
crates/topology/src/validation.rs:1032:31: replace \|\| with && in validate_same_parameter | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound
crates/topology/src/validation.rs:1040:33: replace > with >= in validate_same_parameter | (a) | a_deviation_exactly_at_the_bound_is_inside_every_band
crates/topology/src/validation.rs:1081:9: replace \|\| with && in check_same_range | (a) | a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines
crates/topology/src/validation.rs:1080:9: replace \|\| with && in check_same_range | (a) | a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines
crates/topology/src/validation.rs:1079:9: replace \|\| with && in check_same_range | (b) | Mutant reads `(!t_start.finite && !t_end.finite) \|\| !uv0.finite \|\| !uv1.finite`; a non-finite bound forces its own uv evaluation non-finite, so both forms reduce to the uv tests.
crates/topology/src/validation.rs:1094:9: replace \|\| with && in check_same_range | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof
crates/topology/src/validation.rs:1093:9: replace \|\| with && in check_same_range | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof
crates/topology/src/validation.rs:1092:9: replace \|\| with && in check_same_range | (a) | a_single_non_finite_surface_endpoint_fails_the_range_checks_closed
crates/topology/src/validation.rs:1130:38: replace \|\| with && in check_same_range_strict | (a) | non_finite_parameter_bounds_are_caught_before_evaluation
crates/topology/src/validation.rs:1148:53: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_pcurve_endpoint_is_caught_before_the_surface
crates/topology/src/validation.rs:1171:9: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof
crates/topology/src/validation.rs:1170:9: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof
crates/topology/src/validation.rs:1169:9: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_surface_endpoint_fails_the_range_checks_closed
crates/topology/src/validation.rs:1208:31: replace \|\| with && in validate_same_range_strict | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound
crates/topology/src/validation.rs:1213:26: replace > with >= in validate_same_range_strict | (a) | a_deviation_exactly_at_the_bound_is_inside_every_band
crates/topology/src/validation.rs:1256:31: replace \|\| with && in validate_solid_pcurve_contracts | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound
crates/topology/src/validation.rs:1281:45: replace > with >= in validate_solid_pcurve_contracts | (a) | solid_pcurve_contracts_accept_a_use_exactly_at_the_bound
crates/topology/src/validation.rs:1293:38: replace > with == in validate_solid_pcurve_contracts | (b) | Dead comparison: on the one proved path the SameParameter deviation is this SameRange deviation plus a strictly positive arithmetic bound, so any range violation has already returned at the SameParameter check above.
crates/topology/src/validation.rs:1293:38: replace > with >= in validate_solid_pcurve_contracts | (b) | Same dead comparison: `range >= tolerance` implies `parameter > tolerance`, which returns at the SameParameter check above, so this arm can never decide the outcome.
crates/topology/src/validation.rs:1303:40: replace && with \|\| in validate_solid_pcurve_contracts | (b) | `parameter` and `range` are both `Some` exactly when `pcurve_oriented` resolves the use, which the `coedge.pcurve().is_none()` guard above already established, so the two options are never in disagreement and `&&`/`\|\|` coincide.
crates/topology/src/validation.rs:1327:5: replace validate_same_range -> Result<(), TopologyError> with Ok(()) | (a) | same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound
crates/topology/src/validation.rs:1327:31: replace \|\| with && in validate_same_range | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound
crates/topology/src/validation.rs:1335:26: replace > with == in validate_same_range | (a) | same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound
crates/topology/src/validation.rs:1335:26: replace > with >= in validate_same_range | (a) | same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound
crates/topology/src/validation.rs:1356:31: replace \|\| with && in effective_edge_validation_tolerance | (a) | entity_tolerance_guards_reject_negative_vertex_and_edge_claims
crates/topology/src/validation.rs:1365:38: replace \|\| with && in effective_edge_validation_tolerance | (a) | entity_tolerance_guards_reject_negative_vertex_and_edge_claims
crates/topology/src/validation.rs:1430:36: replace != with == in check_vertex_ball | (a) | vertex_ball_reports_exactly_the_incident_edge_ends
crates/topology/src/validation.rs:1438:34: replace && with \|\| in check_vertex_ball | (a) | vertex_ball_reports_max_for_a_non_finite_curve_evaluation
crates/topology/src/validation.rs:1593:29: replace > with >= in validate_edge_tube | (a) | edge_tube_accepts_a_declared_bound_exactly_at_its_measured_deviation
