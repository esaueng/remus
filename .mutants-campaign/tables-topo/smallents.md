| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/pcurve.rs:168:48: replace && with \|\| in PCurveRegistry::remove_for_retired_entities | (a) | `pcurve::tests::remove_for_retired_entities_drops_an_entry_when_either_side_retires` — entry with a retired edge and a live face (and the mirror case) must be dropped; `\|\|` retains it |
| crates/topology/src/pcurve.rs:174:9: replace PCurveRegistry::remove_face with () | (a) | `pcurve::tests::remove_face_drops_exactly_that_face_and_keeps_the_others` — len falls 2 → 1 |
| crates/topology/src/pcurve.rs:174:44: replace != with == in PCurveRegistry::remove_face | (a) | `pcurve::tests::remove_face_drops_exactly_that_face_and_keeps_the_others` — asserts the removed face has no entry and the other face's entry survives |
| crates/topology/src/pcurve.rs:181:9: replace PCurveRegistry::len -> usize with 0 | (a) | `pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — two indexed uses, `len() == 2` |
| crates/topology/src/pcurve.rs:181:9: replace PCurveRegistry::len -> usize with 1 | (a) | `pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — two indexed uses, `len() == 2` |
| crates/topology/src/pcurve.rs:187:9: replace PCurveRegistry::is_empty -> bool with true | (a) | `pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — non-empty fixture asserts `!is_empty()` |
| crates/topology/src/pcurve.rs:187:9: replace PCurveRegistry::is_empty -> bool with false | (a) | `pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — fresh registry asserts `is_empty()` |
| crates/topology/src/coedge.rs:46:9: replace PeriodicWinding::u -> i32 with 0 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `u() == 3` |
| crates/topology/src/coedge.rs:46:9: replace PeriodicWinding::u -> i32 with 1 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `u() == 3` |
| crates/topology/src/coedge.rs:46:9: replace PeriodicWinding::u -> i32 with -1 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `u() == 3` |
| crates/topology/src/coedge.rs:52:9: replace PeriodicWinding::v -> i32 with 0 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `v() == -4`, distinct from u |
| crates/topology/src/coedge.rs:52:9: replace PeriodicWinding::v -> i32 with 1 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `v() == -4`, distinct from u |
| crates/topology/src/coedge.rs:52:9: replace PeriodicWinding::v -> i32 with -1 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `v() == -4`, distinct from u |
| crates/topology/src/wire.rs:117:9: replace Wire::edges_mut -> &mut[OrientedEdge] with Vec::leak(Vec::new()) | (a) | `wire::tests::edges_mut_exposes_the_stored_edges_for_in_place_replacement` — slice len 2 and a write through it is visible in `edges()` |
| crates/topology/src/wire.rs:123:9: replace Wire::is_closed -> bool with true | (a) | `wire::tests::closed_flag_round_trips_both_ways` — open-wire fixture asserts `!is_closed()` |
| crates/topology/src/wire.rs:133:9: replace Wire::set_body_class with () | (a) | `wire::tests::set_body_class_replaces_the_stored_class` — set Sheet then Solid, read back each time |
| crates/topology/src/shell.rs:59:9: replace Shell::is_empty -> bool with true | (a) | `shell::tests::is_empty_separates_the_sentinel_from_a_real_shell` — one-face shell asserts `!is_empty()` |
| crates/topology/src/shell.rs:59:9: replace Shell::is_empty -> bool with false | (a) | `shell::tests::is_empty_separates_the_sentinel_from_a_real_shell` — `Shell::empty()` asserts `is_empty()` |
| crates/topology/src/shell.rs:73:9: replace Shell::faces_mut -> &mut[FaceId] with Vec::leak(Vec::new()) | (a) | `shell::tests::faces_mut_exposes_the_stored_faces_for_in_place_replacement` — slice len 2 and a write through it is visible in `faces()` |
| crates/topology/src/solid.rs:39:9: replace Solid::set_outer_shell with () | (a) | `solid::tests::set_outer_shell_replaces_the_stored_boundary` — outer shell reads back as the new handle |
| crates/topology/src/solid.rs:50:9: replace Solid::add_inner_shell with () | (a) | `solid::tests::add_inner_shell_appends_without_disturbing_the_outer_shell` — cavity count grows 0 → 1 → 2 in order, outer shell untouched |
| crates/topology/src/compsolid.rs:46:9: replace CompSolid::shared_faces -> &[FaceId] with Vec::leak(Vec::new()) | (a) | `compsolid::tests::populated_compsolid_reports_its_solids_and_shared_faces` — two shared faces returned in order |
| crates/topology/src/compsolid.rs:52:9: replace CompSolid::num_solids -> usize with 0 | (a) | `compsolid::tests::populated_compsolid_reports_its_solids_and_shared_faces` — `num_solids() == 2` |
| crates/topology/src/vertex.rs:44:9: replace Vertex::set_point with () | (a) | `vertex::tests::set_point_replaces_the_stored_position` — moved point reads back component-wise, tolerance unchanged |
| crates/topology/src/face_loop.rs:53:9: replace Loop::is_closed -> bool with true | (a) | `face_loop::tests::loop_closed_flag_round_trips_both_ways` — a `Loop::new(.., false)` fixture asserts `!is_closed()` alongside the derived closed loop |
