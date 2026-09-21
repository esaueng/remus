# remus-topology mutation triage — 2026-09-20

One row per survivor, for every file in the crate that had one (`test_utils.rs` excluded by construction — see Method).

## Method

- Tool: `cargo-mutants 27.0.0` (the weekly workflow's pinned version), Rust 1.96.0 toolchain family.
- `.cargo/mutants.toml` `examine_globs` cover only math/algo/blend/offset/operations, so
  `cargo mutants -p remus-topology` examines **zero** mutants. This campaign overrides discovery with
  `--no-config` (local runs only; the committed CI scope is unchanged).
- Test scope is crate-only: `-- -p remus-topology`. Downstream crates' suites were NOT run per mutant,
  so these survivor counts over-approximate the workspace-scope survivors the weekly CI would report.
- `--baseline skip` (the crate suite was verified green first). `--timeout 60`: several mutations break
  a convergence loop and hang.
- `--in-place` was NOT used: cargo-mutants 27 rejects `--in-place` together with `--jobs`, and the
  parallelism was worth more than the shared build cache. Runs used `-j 8` over copied trees.
- Verdicts: (a) missing assertion — new unit test fails under the mutant, passes on real code;
  (b) equivalent — one-line reason; (c) needs geometry judgment; timeout — mutant hangs the suite.
- `crates/topology/src/test_utils.rs` (47 baseline survivors) is behind the `test-utils` feature, which
  the crate's own test build does not enable, so the module is never compiled and no test can kill
  those mutants. They are missed by construction and excluded from every figure below except the
  crate-total row, where they are shown separately.

## Run provenance (disclosed)

- The before-sweep was taken on the pristine tree (`1c1e145c`); this branch is based on `origin/main` @
  `b13ff97b`, and `crates/topology` was verified byte-identical between the two before the campaign, so
  the parked baseline is valid for this branch. Re-measuring it here is impossible: the new tests are
  already committed.
- Per-file triage was fanned out to one agent per file group, all sharing one worktree. To keep a
  sibling's work-in-progress out of the copied trees, most agents narrowed their proof-run test command
  to their own module — which also drops kill credit the crate-wide baseline had. **Those per-file
  runs are not the numbers reported here.** Every "after" figure below comes from one authoritative
  full-crate re-sweep of the final tree (`/tmp/topo-after`, 1557 mutants in 8 min: 78 missed,
  1136 caught, 343 unviable, 0 timeout).
- Two initial after-sweep attempts died on `No space left on device` (the host disk was 100% full,
  almost entirely other sessions' worktrees and scratch). The successful run adds `--gitignore true`
  `--copy-vcs false`, so each job copy excludes gitignored build dirs and `.git`. No topology test reads
  those paths, so the exclusion cannot change a verdict; it only makes the copies fit on disk.
- Cross-check: of the 468 mutants classified (a), **437 are killed** in the authoritative re-sweep and
  0 still survive; of the 31 classified (b), **0** were killed. Verdicts and measurements agree exactly,
  and there are 0 new survivors relative to the before-sweep.

## Before / after per file (measured)

| File | Mutants | Survivors before | Survivors after | Killed | (a) | (b) | (c) | timeout |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `adjacency.rs` | 24 | 13 | 1 | 12 | 12 | 1 | 0 | 0 |
| `arena.rs` | 68 | 11 | 2 | 9 | 9 | 2 | 0 | 0 |
| `attributes.rs` | 36 | 29 | 0 | 29 | 29 | 0 | 0 | 0 |
| `builder.rs` | 322 | 116 | 13 | 103 | 103 | 13 | 0 | 0 |
| `coedge.rs` | 17 | 6 | 0 | 6 | 6 | 0 | 0 | 0 |
| `compsolid.rs` | 6 | 2 | 0 | 2 | 2 | 0 | 0 | 0 |
| `edge.rs` | 228 | 58 | 5 | 53 | 53 | 5 | 0 | 0 |
| `explorer.rs` | 44 | 23 | 0 | 23 | 23 | 0 | 0 | 0 |
| `face.rs` | 63 | 41 | 0 | 41 | 41 | 0 | 0 | 0 |
| `face_loop.rs` | 5 | 1 | 0 | 1 | 1 | 0 | 0 | 0 |
| `journal.rs` | 59 | 7 | 0 | 7 | 7 | 0 | 0 | 0 |
| `naming.rs` | 132 | 48 | 2 | 46 | 46 | 2 | 0 | 0 |
| `pcurve.rs` | 35 | 7 | 0 | 7 | 7 | 0 | 0 | 0 |
| `shell.rs` | 9 | 3 | 0 | 3 | 3 | 0 | 0 | 0 |
| `solid.rs` | 5 | 2 | 0 | 2 | 2 | 0 | 0 | 0 |
| `topology.rs` | 152 | 19 | 1 | 18 | 18 | 1 | 0 | 0 |
| `validation.rs` | 279 | 78 | 7 | 71 | 71 | 7 | 0 | 0 |
| `vertex.rs` | 8 | 1 | 0 | 1 | 1 | 0 | 0 | 0 |
| `wire.rs` | 13 | 3 | 0 | 3 | 3 | 0 | 0 | 0 |
| `test_utils.rs` (excluded) | 47 | 47 | 47 | 0 | — | — | — | 0 |
| **Total** | **1557** | **515** | **78** | **437** | **437** | **31** | **0** | **0** |

Crate baseline: 1557 mutants, 515 survivors over the full set (33.1% miss rate under crate-scoped
tests; 468 real survivors once the 47 `test_utils.rs` missed-by-construction mutants are excluded).
437 killed by 161 new unit tests (lib suite 200 -> 361). No production code changed; no
existing assertion weakened or removed — every hunk lands inside an existing `#[cfg(test)] mod tests`
(plus two test-module allow-header widenings to the repo-standard lint set).

## crates/topology/src/adjacency.rs

- before: 13 survivors; after: 1 survivors, 14 caught, 9 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/adjacency.rs:93:17: delete match arm 1 in AdjacencyIndex::build_from_faces | (a) | adjacency::index_contract_tests::open_sheet_has_free_edges_and_is_not_manifold` — the 6 single-use rim edges must land in `boundary_edges`, not `non_manifold_edges |
| crates/topology/src/adjacency.rs:94:17: delete match arm 2 in AdjacencyIndex::build_from_faces | (a) | `adjacency::index_contract_tests::closed_manifold_cube_index` — every 2-face edge must produce neighbour links and leave `non_manifold_edges` empty |
| crates/topology/src/adjacency.rs:123:9: replace AdjacencyIndex::faces_for_edge -> &[FaceId] with Vec::leak(Vec::new()) | (a) | `adjacency::index_contract_tests::closed_manifold_cube_index` — each of the 12 cube edges maps to 2 faces |
| crates/topology/src/adjacency.rs:92:17: delete match arm 0 in AdjacencyIndex::build_from_faces | (b) | Equivalent: `edge_faces` entries are only ever created by `or_default().push(face_id)`, so a length-0 entry cannot exist and the arm is unreachable |
| crates/topology/src/adjacency.rs:131:9: replace AdjacencyIndex::neighbors_of_face -> &[FaceId] with Vec::leak(Vec::new()) | (a) | `adjacency::index_contract_tests::closed_manifold_cube_index` — each cube face reports 4 neighbours (and `open_sheet_has_free_edges_and_is_not_manifold` pins the exact pair) |
| crates/topology/src/adjacency.rs:140:9: replace AdjacencyIndex::is_manifold -> bool with true | (a) | adjacency::index_contract_tests::open_sheet_has_free_edges_and_is_not_manifold` and `t_junction_edge_is_reported_non_manifold` — both assert `!is_manifold() |
| crates/topology/src/adjacency.rs:140:44: replace && with \|\| in AdjacencyIndex::is_manifold | (a) | `adjacency::index_contract_tests::open_sheet_has_free_edges_and_is_not_manifold` — the open sheet violates exactly one condition (no non-manifold edges, 6 free edges), so `\|\|` returns true where `&&` returns false |
| crates/topology/src/adjacency.rs:146:9: replace AdjacencyIndex::non_manifold_edges -> &[EdgeId] with Vec::leak(Vec::new()) | (a) | `adjacency::index_contract_tests::t_junction_edge_is_reported_non_manifold` — the 3-face fan edge must be reported |
| crates/topology/src/adjacency.rs:152:9: replace AdjacencyIndex::boundary_edges -> &[EdgeId] with Vec::leak(Vec::new()) | (a) | `adjacency::index_contract_tests::open_sheet_has_free_edges_and_is_not_manifold` — 6 free edges reported (also asserted in the T-junction fixture) |
| crates/topology/src/adjacency.rs:158:9: replace AdjacencyIndex::edge_faces -> Option<&[FaceId]> with Some(Vec::leak(Vec::new())) | (a) | adjacency::index_contract_tests::closed_manifold_cube_index` — `edge_faces(e)` must be 2 faces long and equal `faces_for_edge(e) |
| crates/topology/src/adjacency.rs:158:9: replace AdjacencyIndex::edge_faces -> Option<&[FaceId]> with None | (a) | adjacency::index_contract_tests::closed_manifold_cube_index` — a known cube edge must resolve to `Some`, while an unindexed dangling edge must be `None |
| crates/topology/src/adjacency.rs:164:9: replace AdjacencyIndex::edge_count -> usize with 0 | (a) | `adjacency::index_contract_tests::closed_manifold_cube_index` — 12 edges (open sheet and T-junction fixtures both pin 7) |
| crates/topology/src/adjacency.rs:169:9: replace AdjacencyIndex::edge_faces_iter -> impl Iterator<Item =(EdgeId, &[FaceId])> with ::std::iter::empty() | (a) | `adjacency::index_contract_tests::closed_manifold_cube_index` — the iterator must yield all 12 edges (open-sheet test also iterates it to classify rim edges) |

## crates/topology/src/arena.rs

- before: 11 survivors; after: 2 survivors, 45 caught, 21 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/arena.rs:21:9: replace <impl std::fmt::Debug for Id<T>>::fmt -> std::fmt::Result with Ok(Default::default()) | (a) | `arena::tests::ids_order_by_index_and_render_distinguishably` — the rendering is non-empty, carries the index, and differs between two handles. |
| crates/topology/src/arena.rs:43:9: replace <impl PartialOrd for Id<T>>::partial_cmp -> Option<std::cmp::Ordering> with None | (a) | `arena::tests::ids_order_by_index_and_render_distinguishably` — `partial_cmp` returns Less/Equal/Greater by index and `<` agrees with allocation order. |
| crates/topology/src/arena.rs:55:9: replace <impl std::hash::Hash for Id<T>>::hash with () | (a) | `arena::tests::ids_hash_by_index_for_map_and_set_use` — equal ids hash equal and distinct indices hash differently; a `HashMap`/`HashSet` round-trip alone cannot kill this (`Eq` resolves the collisions), so the distinctness assertion is the kill. |
| crates/topology/src/arena.rs:102:9: replace Arena<T>::with_capacity -> Self with Default::default() | (a) | `arena::tests::with_capacity_preallocates_both_slot_vectors` — both the value and liveness vectors must be pre-allocated to the requested capacity, while ids/len match a default arena. |
| crates/topology/src/arena.rs:129:51: replace - with + in Arena<T>::reserve | (a) | `arena::tests::reserve_covers_the_liveness_vector_independently` — on a 1000-slot arena with ample item headroom and a tight liveness vector, the inflated availability would short-circuit the hint away. |
| crates/topology/src/arena.rs:130:60: replace >= with < in Arena<T>::reserve | (a) | `arena::tests::reserve_covers_the_liveness_vector_independently` — the flipped test returns early in exactly the case that needs the liveness vector grown. |
| crates/topology/src/arena.rs:197:9: replace Arena<T>::slot_len -> usize with 0 | (a) | `arena::tests::slot_len_counts_retired_slots_but_len_does_not` — three allocations are three slots. |
| crates/topology/src/arena.rs:197:9: replace Arena<T>::slot_len -> usize with 1 | (a) | `arena::tests::slot_len_counts_retired_slots_but_len_does_not` — a fresh arena has zero slots. |
| crates/topology/src/arena.rs:242:9: replace Arena<T>::iter_mut -> impl Iterator<Item =(Id<T>, &mut T)> with ::std::iter::empty() | (a) | `arena::tests::iter_mut_visits_every_live_entry_once_and_writes_through` — every live entry is visited exactly once, retired slots skipped, and the writes are observable through `get`. |
| crates/topology/src/arena.rs:283:27: replace > with >= in Arena<T>::restore_preserving_slots | (b) | At the boundary `previous_slots == self.items.len()` both added branches are no-ops: `previous_items[len..]` is empty and `live.resize(previous_slots, false)` targets the length `live` already has (`live.len() == items.len()` always, both cloned from the snapshot). No input distinguishes the two forms. |
| crates/topology/src/arena.rs:308:27: replace > with >= in Arena<T>::restore_for_rollback | (b) | Same boundary argument as `restore_preserving_slots`: at equality the extend copies an empty slice and the resize is to the current length, so the mutation cannot change observable state. |

## crates/topology/src/attributes.rs

- before: 29 survivors; after: 0 survivors, 34 caught, 2 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/attributes.rs:53:9: replace ColorRgb::r -> f64 with 0.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:53:9: replace ColorRgb::r -> f64 with 1.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:53:9: replace ColorRgb::r -> f64 with -1.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:59:9: replace ColorRgb::g -> f64 with 0.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:59:9: replace ColorRgb::g -> f64 with 1.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:59:9: replace ColorRgb::g -> f64 with -1.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:65:9: replace ColorRgb::b -> f64 with 0.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:65:9: replace ColorRgb::b -> f64 with 1.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:65:9: replace ColorRgb::b -> f64 with -1.0 | (a) | attributes::tests::color_channels_are_stored_and_read_back_independently |
| crates/topology/src/attributes.rs:84:9: replace EntityAttributes::is_empty -> bool with false | (a) | attributes::tests::entity_attributes_are_empty_only_when_nothing_is_set |
| crates/topology/src/attributes.rs:105:9: replace AttributeStore::solid -> Option<&EntityAttributes> with None | (a) | attributes::tests::set_then_get_round_trips_and_unset_entities_read_none |
| crates/topology/src/attributes.rs:105:9: replace AttributeStore::solid -> Option<&EntityAttributes> with Some(Box::leak(Box::new(Default::default()))) | (a) | `attributes::tests::set_then_get_round_trips_and_unset_entities_read_none` (unset solid must read `None`) |
| crates/topology/src/attributes.rs:116:9: replace AttributeStore::set_solid with () | (a) | attributes::tests::set_then_get_round_trips_and_unset_entities_read_none`, also `setting_empty_attributes_clears_the_entry |
| crates/topology/src/attributes.rs:134:9: replace AttributeStore::remove_solid -> Option<EntityAttributes> with None | (a) | attributes::tests::remove_returns_the_stored_record_and_none_when_absent |
| crates/topology/src/attributes.rs:134:9: replace AttributeStore::remove_solid -> Option<EntityAttributes> with Some(Default::default()) | (a) | attributes::tests::remove_returns_the_stored_record_and_none_when_absent |
| crates/topology/src/attributes.rs:139:9: replace AttributeStore::remove_face -> Option<EntityAttributes> with None | (a) | attributes::tests::remove_returns_the_stored_record_and_none_when_absent |
| crates/topology/src/attributes.rs:145:9: replace AttributeStore::len -> usize with 0 | (a) | attributes::tests::len_and_is_empty_count_solids_and_faces_together |
| crates/topology/src/attributes.rs:139:9: replace AttributeStore::remove_face -> Option<EntityAttributes> with Some(Default::default()) | (a) | attributes::tests::remove_returns_the_stored_record_and_none_when_absent |
| crates/topology/src/attributes.rs:145:9: replace AttributeStore::len -> usize with 1 | (a) | attributes::tests::len_and_is_empty_count_solids_and_faces_together |
| crates/topology/src/attributes.rs:145:27: replace + with - in AttributeStore::len | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` (2 solids + 1 face must be 3) |
| crates/topology/src/attributes.rs:145:27: replace + with * in AttributeStore::len | (a) | attributes::tests::len_and_is_empty_count_solids_and_faces_together |
| crates/topology/src/attributes.rs:151:9: replace AttributeStore::is_empty -> bool with true | (a) | attributes::tests::len_and_is_empty_count_solids_and_faces_together |
| crates/topology/src/attributes.rs:151:9: replace AttributeStore::is_empty -> bool with false | (a) | attributes::tests::len_and_is_empty_count_solids_and_faces_together` (fresh store), `setting_empty_attributes_clears_the_entry |
| crates/topology/src/attributes.rs:151:32: replace && with \|\| in AttributeStore::is_empty | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` (face-only and solid-only stores are not empty) |
| crates/topology/src/attributes.rs:157:9: replace AttributeStore::faces_with_attributes -> Vec<(FaceId, &EntityAttributes)> with vec![] | (a) | attributes::tests::listings_yield_every_entry_sorted_by_index |
| crates/topology/src/attributes.rs:165:9: replace AttributeStore::solids_with_attributes -> Vec<(SolidId, &EntityAttributes)> with vec![] | (a) | attributes::tests::listings_yield_every_entry_sorted_by_index |
| crates/topology/src/attributes.rs:176:36: delete ! in AttributeStore::remove_for_retired_entities | (a) | `attributes::tests::retiring_entities_removes_exactly_their_entries` (live solid must survive) |
| crates/topology/src/attributes.rs:176:9: replace AttributeStore::remove_for_retired_entities with () | (a) | attributes::tests::retiring_entities_removes_exactly_their_entries |
| crates/topology/src/attributes.rs:177:35: delete ! in AttributeStore::remove_for_retired_entities | (a) | `attributes::tests::retiring_entities_removes_exactly_their_entries` (live face must survive) |

## crates/topology/src/builder.rs

- before: 116 survivors; after: 13 survivors, 213 caught, 96 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
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

## crates/topology/src/coedge.rs

- before: 6 survivors; after: 0 survivors, 11 caught, 6 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/coedge.rs:46:9: replace PeriodicWinding::u -> i32 with 0 | (a) | coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `u() == 3 |
| crates/topology/src/coedge.rs:46:9: replace PeriodicWinding::u -> i32 with 1 | (a) | coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `u() == 3 |
| crates/topology/src/coedge.rs:46:9: replace PeriodicWinding::u -> i32 with -1 | (a) | coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `u() == 3 |
| crates/topology/src/coedge.rs:52:9: replace PeriodicWinding::v -> i32 with 0 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `v() == -4`, distinct from u |
| crates/topology/src/coedge.rs:52:9: replace PeriodicWinding::v -> i32 with 1 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `v() == -4`, distinct from u |
| crates/topology/src/coedge.rs:52:9: replace PeriodicWinding::v -> i32 with -1 | (a) | `coedge::tests::periodic_winding_reports_u_and_v_independently` — winding (3, -4): `v() == -4`, distinct from u |

## crates/topology/src/compsolid.rs

- before: 2 survivors; after: 0 survivors, 4 caught, 2 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/compsolid.rs:46:9: replace CompSolid::shared_faces -> &[FaceId] with Vec::leak(Vec::new()) | (a) | `compsolid::tests::populated_compsolid_reports_its_solids_and_shared_faces` — two shared faces returned in order |
| crates/topology/src/compsolid.rs:52:9: replace CompSolid::num_solids -> usize with 0 | (a) | compsolid::tests::populated_compsolid_reports_its_solids_and_shared_faces` — `num_solids() == 2 |

## crates/topology/src/edge.rs

- before: 58 survivors; after: 5 survivors, 190 caught, 33 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/edge.rs:61:38: replace && with \|\| in <impl remus_math::diagnostic::ToDiagnostic for EdgeDomainError>::diagnostic | (a) | invalid_domain_diagnostic_omits_non_finite_bounds |
| crates/topology/src/edge.rs:161:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn |
| crates/topology/src/edge.rs:161:43: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn |
| crates/topology/src/edge.rs:161:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn |
| crates/topology/src/edge.rs:165:49: replace - with + in EdgeCurve::reconstruct_domain_from_endpoints | (a) | circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:165:49: replace - with / in EdgeCurve::reconstruct_domain_from_endpoints | (a) | circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:166:42: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn |
| crates/topology/src/edge.rs:166:42: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:166:42: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | conic_delta_exactly_at_the_degenerate_threshold_stays_a_sliver_arc |
| crates/topology/src/edge.rs:167:29: replace + with - in EdgeCurve::reconstruct_domain_from_endpoints | (a) | circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:167:29: replace + with * in EdgeCurve::reconstruct_domain_from_endpoints | (a) | circle_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:171:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn |
| crates/topology/src/edge.rs:171:43: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | closed_conic_edges_use_the_curves_own_full_domain_not_an_anchored_turn |
| crates/topology/src/edge.rs:171:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn |
| crates/topology/src/edge.rs:175:49: replace - with + in EdgeCurve::reconstruct_domain_from_endpoints | (a) | ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:175:49: replace - with / in EdgeCurve::reconstruct_domain_from_endpoints | (a) | ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:176:42: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | conic_endpoints_on_the_closed_threshold_stay_open_and_take_a_full_turn |
| crates/topology/src/edge.rs:176:42: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:176:42: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | conic_delta_exactly_at_the_degenerate_threshold_stays_a_sliver_arc |
| crates/topology/src/edge.rs:177:29: replace + with - in EdgeCurve::reconstruct_domain_from_endpoints | (a) | ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:177:29: replace + with * in EdgeCurve::reconstruct_domain_from_endpoints | (a) | ellipse_arc_domain_is_the_ccw_span_between_the_projected_endpoints |
| crates/topology/src/edge.rs:192:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Coincident endpoints — the only case the guard exists for — reach the same full domain through the fall-through: a zero parameter span fails the non-degenerate test. Differing needs a nonzero sub-1e-9 chord whose parameter span still exceeds 1e-6 of the domain, i.e. a curve shorter than the 1e-5 weld band. |
| crates/topology/src/edge.rs:192:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Differs only for a chord of exactly 1e-9; below it both branches return the full domain (see the == row). |
| crates/topology/src/edge.rs:197:53: replace && with \|\| in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_needs_both_ends_within_the_band |
| crates/topology/src/edge.rs:197:43: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_is_measured_against_the_curve_ends |
| crates/topology/src/edge.rs:197:43: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_is_measured_against_the_curve_ends |
| crates/topology/src/edge.rs:197:43: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_excludes_its_own_edge |
| crates/topology/src/edge.rs:197:76: replace < with > in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_is_measured_against_the_curve_ends |
| crates/topology/src/edge.rs:197:76: replace < with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_is_measured_against_the_curve_ends |
| crates/topology/src/edge.rs:197:76: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_excludes_its_own_edge |
| crates/topology/src/edge.rs:198:55: replace && with \|\| in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_reversed_whole_edge_match_needs_both_ends_within_the_band |
| crates/topology/src/edge.rs:198:45: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_excludes_its_own_edge |
| crates/topology/src/edge.rs:198:80: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_whole_edge_match_band_excludes_its_own_edge |
| crates/topology/src/edge.rs:214:36: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_on_curve_weld_band_excludes_its_own_edge |
| crates/topology/src/edge.rs:215:40: replace < with <= in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_on_curve_weld_band_excludes_its_own_edge |
| crates/topology/src/edge.rs:216:37: replace > with >= in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Differs only when the projected \|dt\| equals 1e-6 × the domain span bit-exactly; dt comes out of Newton refinement, so no fixture can pin it to that value. |
| crates/topology/src/edge.rs:216:44: replace * with / in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_degenerate_span_threshold_is_relative_to_the_knot_domain |
| crates/topology/src/edge.rs:216:50: replace - with + in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_degenerate_span_threshold_is_relative_to_the_knot_domain |
| crates/topology/src/edge.rs:217:32: replace > with == in EdgeCurve::reconstruct_domain_from_endpoints | (a) | nurbs_forward_sub_span_on_a_closed_curve_is_trimmed |
| crates/topology/src/edge.rs:217:32: replace > with >= in EdgeCurve::reconstruct_domain_from_endpoints | (b) | Unreachable: >= only differs at dt == 0.0, which the preceding \|dt\| > 1e-6 × (d1 - d0) test has already rejected. |
| crates/topology/src/edge.rs:336:9: replace Edge::set_start with () | (a) | set_start_and_set_end_rewire_the_bounding_vertices |
| crates/topology/src/edge.rs:341:9: replace Edge::set_end with () | (a) | set_start_and_set_end_rewire_the_bounding_vertices |
| crates/topology/src/edge.rs:400:21: replace && with \|\| in Edge::strict_domain | (a) | strict_domain_rejects_a_line_trim_matching_only_one_end |
| crates/topology/src/edge.rs:419:13: replace \|\| with && in Edge::strict_domain | (a) | strict_domain_rejects_a_single_non_finite_bound |
| crates/topology/src/edge.rs:432:82: replace \|\| with && in Edge::strict_domain | (a) | strict_domain_rejects_every_out_of_range_nurbs_bound |
| crates/topology/src/edge.rs:432:60: replace \|\| with && in Edge::strict_domain | (a) | strict_domain_rejects_every_out_of_range_nurbs_bound |
| crates/topology/src/edge.rs:437:61: delete ! in Edge::strict_domain | (a) | strict_domain_accepts_a_finite_open_conic_span |
| crates/topology/src/edge.rs:440:69: delete ! in Edge::strict_domain | (a) | strict_domain_accepts_a_finite_open_conic_span |
| crates/topology/src/edge.rs:445:61: delete ! in Edge::strict_domain | (a) | strict_domain_accepts_a_finite_open_conic_span |
| crates/topology/src/edge.rs:448:69: delete ! in Edge::strict_domain | (a) | strict_domain_accepts_a_finite_open_conic_span |
| crates/topology/src/edge.rs:519:34: replace * with + in periodic_domain_is_valid | (a) | strict_domain_rejects_a_closed_turn_overrunning_by_more_than_roundoff |
| crates/topology/src/edge.rs:526:9: replace \|\| with && in periodic_domain_is_valid | (a) | periodic_domain_guard_refuses_a_negative_tolerance_claim |
| crates/topology/src/edge.rs:527:49: replace > with >= in periodic_domain_is_valid | (b) | Unreachable: for any span in [4, 8) the difference span - TAU is an integer multiple of 2^-50, while the allowance 4 * EPSILON * TAU is 6.28 * 2^-50 — equality is not representable; a span outside that range misses TAU by more than 2. |
| crates/topology/src/edge.rs:581:49: replace * with + in periodic_curve_is_finite | (a) | periodic_curve_is_finite_multiplies_axis_components_by_their_extent |
| crates/topology/src/edge.rs:582:49: replace * with + in periodic_curve_is_finite | (a) | periodic_curve_is_finite_multiplies_axis_components_by_their_extent |
| crates/topology/src/edge.rs:582:49: replace * with / in periodic_curve_is_finite | (a) | periodic_curve_is_finite_rejects_a_center_plus_extent_overflow |
| crates/topology/src/edge.rs:583:37: replace + with - in periodic_curve_is_finite | (a) | periodic_curve_is_finite_rejects_a_center_plus_extent_overflow |
| crates/topology/src/edge.rs:584:56: replace + with * in periodic_curve_is_finite | (a) | periodic_curve_is_finite_adds_the_center_to_the_tangent_bound |

## crates/topology/src/explorer.rs

- before: 23 survivors; after: 0 survivors, 24 caught, 20 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/explorer.rs:46:5: replace solid_edges -> Result<Vec<EdgeId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — asserts 12 edges on a cube solid and 24 on a cavity solid |
| crates/topology/src/explorer.rs:66:5: replace solid_vertices -> Result<Vec<VertexId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — asserts 8 vertices on a cube solid and 16 on a cavity solid |
| crates/topology/src/explorer.rs:90:5: replace face_edges -> Result<Vec<EdgeId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::face_edges_and_vertices_include_inner_wires` — 8 edges on a holed face, 4 on each cube face |
| crates/topology/src/explorer.rs:114:5: replace face_vertices -> Result<Vec<VertexId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::face_edges_and_vertices_include_inner_wires` — 8 vertices on a holed face, 4 on each cube face |
| crates/topology/src/explorer.rs:151:5: replace edge_to_face_map -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> with Ok(BTreeMap::new()) | (a) | `explorer::traversal_tests::edge_to_face_map_indexes_every_shell_edge_twice` — asserts the map has 24 entries |
| crates/topology/src/explorer.rs:151:5: replace edge_to_face_map -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> with Ok(BTreeMap::from_iter([(0, SmallVec::new())])) | (a) | `explorer::traversal_tests::edge_to_face_map_indexes_every_shell_edge_twice` — map length 24 and every entry has exactly 2 face uses |
| crates/topology/src/explorer.rs:151:5: replace edge_to_face_map -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> with Ok(BTreeMap::from_iter([(1, SmallVec::new())])) | (a) | `explorer::traversal_tests::edge_to_face_map_indexes_every_shell_edge_twice` — map length 24 and every entry has exactly 2 face uses |
| crates/topology/src/explorer.rs:167:5: replace edge_to_face_map_for_faces -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> with Ok(BTreeMap::new()) | (a) | `explorer::traversal_tests::edge_to_face_map_for_faces_indexes_only_the_given_faces` — 12 entries for the outer shell, 4 for a single face |
| crates/topology/src/explorer.rs:167:5: replace edge_to_face_map_for_faces -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> with Ok(BTreeMap::from_iter([(0, SmallVec::new())])) | (a) | `explorer::traversal_tests::edge_to_face_map_for_faces_indexes_only_the_given_faces` — entry count plus a 2-face-use assertion per entry |
| crates/topology/src/explorer.rs:167:5: replace edge_to_face_map_for_faces -> Result<BTreeMap<usize, SmallVec<[FaceId; 2]>>, TopologyError> with Ok(BTreeMap::from_iter([(1, SmallVec::new())])) | (a) | `explorer::traversal_tests::edge_to_face_map_for_faces_indexes_only_the_given_faces` — entry count plus a 2-face-use assertion per entry |
| crates/topology/src/explorer.rs:199:5: replace shared_edges -> Result<Vec<EdgeId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::adjacent_faces_and_shared_edges_on_a_cube` — adjacent cube faces must share exactly 1 edge |
| crates/topology/src/explorer.rs:223:5: replace adjacent_faces -> Result<Vec<FaceId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::adjacent_faces_and_shared_edges_on_a_cube` — a cube face has exactly 4 neighbours |
| crates/topology/src/explorer.rs:229:32: replace != with == in adjacent_faces | (a) | `explorer::traversal_tests::adjacent_faces_and_shared_edges_on_a_cube` — the mutant yields only the face itself; the test asserts 4 neighbours and that none is the face |
| crates/topology/src/explorer.rs:229:48: replace && with \|\| in adjacent_faces | (a) | `explorer::traversal_tests::adjacent_faces_and_shared_edges_on_a_cube` — short-circuiting drops the dedup and re-admits the face itself, giving 5; the test pins 4 and excludes self |
| crates/topology/src/explorer.rs:249:5: replace face_wires -> Result<Vec<WireId>, TopologyError> with Ok(vec![]) | (a) | `explorer::traversal_tests::face_wires_returns_outer_then_inner` — 2 wires on a holed face, 1 on a cube face, outer first |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((0, 0, 0)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((0, 0, 1)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((0, 1, 0)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((0, 1, 1)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((1, 0, 0)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((1, 0, 1)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((1, 1, 0)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |
| crates/topology/src/explorer.rs:269:5: replace solid_entity_counts -> Result<(usize, usize, usize), TopologyError> with Ok((1, 1, 1)) | (a) | `explorer::traversal_tests::solid_entity_counts_sum_over_every_shell` — pins (6, 12, 8) and (12, 24, 16) |

## crates/topology/src/face.rs

- before: 41 survivors; after: 0 survivors, 49 caught, 14 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with None | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((0.0, 0.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((0.0, 1.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((0.0, -1.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((1.0, 0.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((1.0, 1.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((1.0, -1.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((-1.0, 0.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((-1.0, 1.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:90:9: replace FaceSurface::project_point -> Option<(f64, f64)> with Some((-1.0, -1.0)) | (a) | `project_point_round_trips_on_every_parametric_variant` — plane must project to `None`, and every parametric variant must project to a `(u, v)` that re-evaluates back onto the sampled point |
| crates/topology/src/face.rs:107:9: replace FaceSurface::partial_u -> Option<Vec3> with None | (a) | `partials_span_the_tangent_plane_and_planes_have_none` — cylinder `partial_u` is `Some`, has length = radius and is perpendicular to the axis |
| crates/topology/src/face.rs:124:9: replace FaceSurface::partial_v -> Option<Vec3> with None | (a) | `partials_span_the_tangent_plane_and_planes_have_none` — cylinder `partial_v` is `Some` and equals the unit axis |
| crates/topology/src/face.rs:141:9: replace FaceSurface::estimate_radius -> f64 with 1.0 | (a) | `estimate_radius_reports_each_variant_defining_radius` — plane is +inf; cylinder 2.75, sphere 3.5, torus 4.25, cone cos(0.6) |
| crates/topology/src/face.rs:141:9: replace FaceSurface::estimate_radius -> f64 with 0.0 | (a) | `estimate_radius_reports_each_variant_defining_radius` — plane is +inf; cylinder 2.75, sphere 3.5, torus 4.25, cone cos(0.6) |
| crates/topology/src/face.rs:141:9: replace FaceSurface::estimate_radius -> f64 with -1.0 | (a) | `estimate_radius_reports_each_variant_defining_radius` — plane is +inf; cylinder 2.75, sphere 3.5, torus 4.25, cone cos(0.6) |
| crates/topology/src/face.rs:162:33: replace - with + in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:162:33: replace - with / in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:163:33: replace - with + in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:163:33: replace - with / in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:164:33: replace - with + in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:164:33: replace - with / in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:165:40: replace * with + in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:165:40: replace * with / in FaceSurface::estimate_radius | (a) | estimate_radius_of_nurbs_is_half_the_control_box_diagonal` — control box dx=6, dy=6, dz=4 gives 0.5*sqrt(88); each extent differs from `max+min` and `max/min |
| crates/topology/src/face.rs:173:9: replace FaceSurface::type_tag -> &'static str with "" | (a) | `type_tag_is_distinct_and_stable_across_all_six_variants` — each tag asserted by value and all six asserted pairwise distinct |
| crates/topology/src/face.rs:173:9: replace FaceSurface::type_tag -> &'static str with "xyzzy" | (a) | `type_tag_is_distinct_and_stable_across_all_six_variants` — each tag asserted by value and all six asserted pairwise distinct |
| crates/topology/src/face.rs:186:9: replace FaceSurface::is_planar -> bool with true | (a) | `is_planar_and_is_analytic_classify_every_variant` — plane planar, other five not; plane/cylinder/cone/sphere/torus analytic, NURBS not |
| crates/topology/src/face.rs:186:9: replace FaceSurface::is_planar -> bool with false | (a) | `is_planar_and_is_analytic_classify_every_variant` — plane planar, other five not; plane/cylinder/cone/sphere/torus analytic, NURBS not |
| crates/topology/src/face.rs:192:9: replace FaceSurface::is_analytic -> bool with true | (a) | `is_planar_and_is_analytic_classify_every_variant` — plane planar, other five not; plane/cylinder/cone/sphere/torus analytic, NURBS not |
| crates/topology/src/face.rs:192:9: replace FaceSurface::is_analytic -> bool with false | (a) | `is_planar_and_is_analytic_classify_every_variant` — plane planar, other five not; plane/cylinder/cone/sphere/torus analytic, NURBS not |
| crates/topology/src/face.rs:192:9: delete ! in FaceSurface::is_analytic | (a) | `is_planar_and_is_analytic_classify_every_variant` — plane planar, other five not; plane/cylinder/cone/sphere/torus analytic, NURBS not |
| crates/topology/src/face.rs:201:9: replace FaceSurface::as_analytic -> Option<remus_math::analytic_intersection::AnalyticSurface<'_>> with None | (a) | as_analytic_exposes_the_quadrics_and_only_the_quadrics` — cylinder/cone/sphere/torus return the matching `AnalyticSurface` variant; plane and NURBS return `None |
| crates/topology/src/face.rs:318:9: replace Face::inner_loops -> &[LoopId] with Vec::leak(Vec::new()) | (a) | adding_a_hole_leaves_the_outer_boundary_alone` — a holed face reports exactly one inner loop, equal to `boundary_loops()[1]` and distinct from `outer_loop() |
| crates/topology/src/face.rs:333:9: replace Face::inner_wires_mut -> &mut Vec<WireId> with Box::leak(Box::new(vec![])) | (a) | inner_wires_mut_edits_the_face_in_place` — a hole pushed through the mutable accessor is visible through `inner_wires() |
| crates/topology/src/face.rs:367:9: replace Face::is_reversed -> bool with true | (a) | `reversed_flag_defaults_off_and_round_trips` — `Face::new` is unreversed, `Face::new_reversed` is reversed, and `set_reversed` toggles both ways |
| crates/topology/src/face.rs:367:9: replace Face::is_reversed -> bool with false | (a) | `reversed_flag_defaults_off_and_round_trips` — `Face::new` is unreversed, `Face::new_reversed` is reversed, and `set_reversed` toggles both ways |
| crates/topology/src/face.rs:373:9: replace Face::set_reversed with () | (a) | `reversed_flag_defaults_off_and_round_trips` — `Face::new` is unreversed, `Face::new_reversed` is reversed, and `set_reversed` toggles both ways |
| crates/topology/src/face.rs:381:9: replace Face::effective_plane_normal -> Option<Vec3> with None | (a) | effective_plane_normal_follows_the_reversed_flag` — a planar face reports the stored normal, a reversed one the negated normal, and a cylindrical face `None |
| crates/topology/src/face.rs:382:13: delete match arm FaceSurface::Plane{normal, ..} in Face::effective_plane_normal | (a) | effective_plane_normal_follows_the_reversed_flag` — a planar face reports the stored normal, a reversed one the negated normal, and a cylindrical face `None |
| crates/topology/src/face.rs:384:26: delete - in Face::effective_plane_normal | (a) | effective_plane_normal_follows_the_reversed_flag` — a planar face reports the stored normal, a reversed one the negated normal, and a cylindrical face `None |
| crates/topology/src/face.rs:395:9: replace Face::compose_orientation with () | (a) | `compose_orientation_toggles_only_when_flipping` — `flip=true` toggles the flag on then back off; `flip=false` leaves it unchanged in both states |
| crates/topology/src/face.rs:396:29: delete ! in Face::compose_orientation | (a) | `compose_orientation_toggles_only_when_flipping` — `flip=true` toggles the flag on then back off; `flip=false` leaves it unchanged in both states |

## crates/topology/src/face_loop.rs

- before: 1 survivors; after: 0 survivors, 3 caught, 2 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/face_loop.rs:53:9: replace Loop::is_closed -> bool with true | (a) | `face_loop::tests::loop_closed_flag_round_trips_both_ways` — a `Loop::new(.., false)` fixture asserts `!is_closed()` alongside the derived closed loop |

## crates/topology/src/journal.rs

- before: 7 survivors; after: 0 survivors, 40 caught, 19 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/journal.rs:194:9: replace RecordedOrigin::as_str -> &'static str with "xyzzy" | (a) | journal::tests::recorded_origin_names_are_stable |
| crates/topology/src/journal.rs:194:9: replace RecordedOrigin::as_str -> &'static str with "" | (a) | journal::tests::recorded_origin_names_are_stable |
| crates/topology/src/journal.rs:394:9: replace JournalEntry::is_barrier -> bool with true | (a) | `journal::tests::only_barrier_payloads_are_barriers` (evolution entries must not sever) |
| crates/topology/src/journal.rs:405:9: replace JournalEntry::ticks_after -> u64 with 0 | (a) | journal::tests::entries_carry_distinct_tick_counts_derived_from_their_position |
| crates/topology/src/journal.rs:405:9: replace JournalEntry::ticks_after -> u64 with 1 | (a) | journal::tests::entries_carry_distinct_tick_counts_derived_from_their_position |
| crates/topology/src/journal.rs:455:9: replace PendingOp::add_scope with () | (a) | journal::tests::pre_operation_scope_from_the_pending_token_reaches_the_entry |
| crates/topology/src/journal.rs:493:9: replace Journal::is_empty -> bool with true | (a) | journal::tests::a_recorded_journal_is_not_empty |

## crates/topology/src/naming.rs

- before: 48 survivors; after: 2 survivors, 110 caught, 20 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/naming.rs:294:5: replace resolve_face_attributes -> Result<Vec<(EntityKey, Option<&'t crate::attributes::EntityAttributes>)>, crate::TopologyError> with Ok(vec![]) | (a) | `resolved_face_attributes_follow_the_binding` — asserts one binding is returned, not an empty vec |
| crates/topology/src/naming.rs:298:40: replace == with != in resolve_face_attributes | (a) | resolved_face_attributes_follow_the_binding` — the face binding must carry `Some(attributes)`; `!=` yields `None |
| crates/topology/src/naming.rs:436:57: replace && with \|\| in resolve | (a) | `a_lineage_hop_into_another_kind_binds_nothing_rather_than_the_wrong_entity` — with `\|\|` an out-of-kind edge key survives and binds |
| crates/topology/src/naming.rs:449:9: delete match arm [] in bind | (a) | a_lineage_hop_into_another_kind_binds_nothing_rather_than_the_wrong_entity` — an empty surviving set must be `NoMatch`, not an empty `BoundMany |
| crates/topology/src/naming.rs:518:41: replace match guard *subject == ordinal with true in chase_one_entry | (a) | `sibling_claims_in_one_entry_do_not_leak_into_this_lineage` — a `Deleted` claim about another face must not mark this one deleted |
| crates/topology/src/naming.rs:525:49: replace match guard from.contains(&ordinal) with true in chase_one_entry | (a) | `sibling_claims_in_one_entry_do_not_leak_into_this_lineage` — a `Merged` claim about other faces must not capture this lineage |
| crates/topology/src/naming.rs:570:26: replace == with != in apply_discriminators | (a) | `discriminators_filter_by_kind_and_by_geometry_type` — a face key with a matching `SurfaceType` must survive |
| crates/topology/src/naming.rs:574:71: replace == with != in apply_discriminators | (a) | `discriminators_filter_by_kind_and_by_geometry_type` — asserts both directions (matching tag keeps, non-matching drops) |
| crates/topology/src/naming.rs:578:21: replace && with \|\| in apply_discriminators | (a) | `discriminators_filter_by_kind_and_by_geometry_type` — with `\|\|` a face key passes a `CurveType` filter via the same-index edge |
| crates/topology/src/naming.rs:577:26: replace == with != in apply_discriminators | (a) | `discriminators_filter_by_kind_and_by_geometry_type` — an edge key with a matching `CurveType` must survive |
| crates/topology/src/naming.rs:581:69: replace == with != in apply_discriminators | (a) | `discriminators_filter_by_kind_and_by_geometry_type` — asserts both directions (matching tag keeps, non-matching drops) |
| crates/topology/src/naming.rs:661:9: replace EntitySignature::context_quantum -> f64 with 0.0 | (a) | `context_quantum_is_the_context_linear_tolerance` — reads back a context linear tolerance of 2.5e-4 |
| crates/topology/src/naming.rs:661:9: replace EntitySignature::context_quantum -> f64 with 1.0 | (a) | `context_quantum_is_the_context_linear_tolerance` — reads back a context linear tolerance of 2.5e-4 |
| crates/topology/src/naming.rs:661:9: replace EntitySignature::context_quantum -> f64 with -1.0 | (a) | `context_quantum_is_the_context_linear_tolerance` — reads back a context linear tolerance of 2.5e-4 |
| crates/topology/src/naming.rs:781:38: replace \|\| with && in EntitySignature::matches_raw | (a) | `signatures_never_match_across_curve_types` — line vs hyperbola: equal (empty) params and adjacency, only the tag differs |
| crates/topology/src/naming.rs:788:34: replace && with \|\| in EntitySignature::matches_raw | (a) | `a_quantum_that_is_not_positive_never_matches_anything` — quantum 0.0 must refuse; `\|\|` lets the zero-width window admit the origin |
| crates/topology/src/naming.rs:788:45: replace > with >= in EntitySignature::matches_raw | (a) | `a_quantum_that_is_not_positive_never_matches_anything` — same: `>=` admits quantum 0.0 |
| crates/topology/src/naming.rs:805:30: replace && with \|\| in quantize | (a) | `quantize_refuses_a_negative_quantum` — `quantize(2.0, -1.0)` must be poison, `\|\|` returns -2 |
| crates/topology/src/naming.rs:805:41: replace > with >= in quantize | (b) | Differs only at `quantum == 0.0`, where `value / 0.0` is `inf` or `NaN` and the next non-finite check returns the same `i64::MAX`. Unobservable. |
| crates/topology/src/naming.rs:827:28: replace > with == in canonical_direction | (a) | `canonical_direction_flips_on_the_first_component_beyond_the_quantum` — `(-1, 0, 0)` must canonicalize to `(1, 0, 0)`; the `>=` arm is killed by the `(-1e-7, 0, 1)` boundary case |
| crates/topology/src/naming.rs:827:28: replace > with < in canonical_direction | (a) | `canonical_direction_flips_on_the_first_component_beyond_the_quantum` — `(-1, 0, 0)` must canonicalize to `(1, 0, 0)`; the `>=` arm is killed by the `(-1e-7, 0, 1)` boundary case |
| crates/topology/src/naming.rs:827:28: replace > with >= in canonical_direction | (a) | `canonical_direction_flips_on_the_first_component_beyond_the_quantum` — `(-1, 0, 0)` must canonicalize to `(1, 0, 0)`; the `>=` arm is killed by the `(-1e-7, 0, 1)` boundary case |
| crates/topology/src/naming.rs:828:33: replace < with == in canonical_direction | (a) | `canonical_direction_flips_on_the_first_component_beyond_the_quantum` — `(-1, 0, 0)` must flip |
| crates/topology/src/naming.rs:828:33: replace < with > in canonical_direction | (a) | `canonical_direction_flips_on_the_first_component_beyond_the_quantum` — `(-1, 0, 0)` must flip |
| crates/topology/src/naming.rs:828:33: replace < with <= in canonical_direction | (b) | Reached only when `component.abs() > quantum`, so with any positive quantum `component != 0.0` and `<`/`<=` agree. A non-positive quantum is rejected by `matches_raw` and `quantize` before it can be observed. |
| crates/topology/src/naming.rs:828:41: delete - in canonical_direction | (a) | `canonical_direction_flips_on_the_first_component_beyond_the_quantum` — dropping the negation stops `(-1, 0, 0)` from flipping |
| crates/topology/src/naming.rs:836:5: replace face_raw_params -> Vec<f64> with vec![] | (a) | face_signatures_separate_planes_by_their_surface_parameters` — constant/empty params make two distinct planes `Ambiguous |
| crates/topology/src/naming.rs:836:5: replace face_raw_params -> Vec<f64> with vec![0.0] | (a) | face_signatures_separate_planes_by_their_surface_parameters` — constant/empty params make two distinct planes `Ambiguous |
| crates/topology/src/naming.rs:836:5: replace face_raw_params -> Vec<f64> with vec![1.0] | (a) | face_signatures_separate_planes_by_their_surface_parameters` — constant/empty params make two distinct planes `Ambiguous |
| crates/topology/src/naming.rs:836:5: replace face_raw_params -> Vec<f64> with vec![-1.0] | (a) | face_signatures_separate_planes_by_their_surface_parameters` — constant/empty params make two distinct planes `Ambiguous |
| crates/topology/src/naming.rs:849:74: replace * with + in face_raw_params | (a) | `cylinder_signatures_are_stable_across_axis_reparameterization` — a wrong axis anchor breaks the two-parameterization match |
| crates/topology/src/naming.rs:849:74: replace * with / in face_raw_params | (a) | `cylinder_signatures_are_stable_across_axis_reparameterization` — a wrong axis anchor breaks the two-parameterization match |
| crates/topology/src/naming.rs:851:42: delete - in face_raw_params | (a) | `cylinder_signatures_are_stable_across_axis_reparameterization` — same |
| crates/topology/src/naming.rs:852:42: delete - in face_raw_params | (a) | `cylinder_signatures_are_stable_across_axis_reparameterization` — same |
| crates/topology/src/naming.rs:853:42: delete - in face_raw_params | (a) | `cylinder_signatures_are_stable_across_axis_reparameterization` — same |
| crates/topology/src/naming.rs:907:5: replace edge_raw_params -> Vec<f64> with vec![] | (a) | edge_signatures_separate_circles_sharing_their_endpoints` — constant/empty params make two circles on one pair of endpoints `Ambiguous |
| crates/topology/src/naming.rs:907:5: replace edge_raw_params -> Vec<f64> with vec![0.0] | (a) | edge_signatures_separate_circles_sharing_their_endpoints` — constant/empty params make two circles on one pair of endpoints `Ambiguous |
| crates/topology/src/naming.rs:907:5: replace edge_raw_params -> Vec<f64> with vec![1.0] | (a) | edge_signatures_separate_circles_sharing_their_endpoints` — constant/empty params make two circles on one pair of endpoints `Ambiguous |
| crates/topology/src/naming.rs:907:5: replace edge_raw_params -> Vec<f64> with vec![-1.0] | (a) | edge_signatures_separate_circles_sharing_their_endpoints` — constant/empty params make two circles on one pair of endpoints `Ambiguous |
| crates/topology/src/naming.rs:951:9: replace AdjacencyCounts::edge_face_uses -> u32 with 0 | (a) | edge_signatures_separate_twin_edges_by_face_uses` — a constant face-use count makes the bounding and free twin edges `Ambiguous |
| crates/topology/src/naming.rs:951:9: replace AdjacencyCounts::edge_face_uses -> u32 with 1 | (a) | edge_signatures_separate_twin_edges_by_face_uses` — a constant face-use count makes the bounding and free twin edges `Ambiguous |
| crates/topology/src/naming.rs:955:9: replace AdjacencyCounts::vertex_edge_uses -> u32 with 0 | (a) | vertex_signatures_separate_coincident_vertices_by_incident_edge_uses` — a constant edge-use count makes the coincident vertices `Ambiguous |
| crates/topology/src/naming.rs:955:9: replace AdjacencyCounts::vertex_edge_uses -> u32 with 1 | (a) | vertex_signatures_separate_coincident_vertices_by_incident_edge_uses` — a constant edge-use count makes the coincident vertices `Ambiguous |
| crates/topology/src/naming.rs:970:81: replace += with -= in adjacency_counts | (a) | `edge_signatures_separate_twin_edges_by_face_uses` — `0u32 -= 1` overflows and panics while walking the test's face |
| crates/topology/src/naming.rs:970:81: replace += with *= in adjacency_counts | (a) | edge_signatures_separate_twin_edges_by_face_uses` — `*=` pins every face-use count at 0, making the twins `Ambiguous |
| crates/topology/src/naming.rs:978:68: replace += with *= in adjacency_counts | (a) | `vertex_signatures_separate_coincident_vertices_by_incident_edge_uses` — the start-side half of the test loses its discriminator |
| crates/topology/src/naming.rs:979:66: replace += with *= in adjacency_counts | (a) | `vertex_signatures_separate_coincident_vertices_by_incident_edge_uses` — the end-side half of the test loses its discriminator |
| crates/topology/src/naming.rs:1030:25: replace && with \|\| in resolve_signature | (a) | `edge_signatures_require_both_endpoints_to_match` — with `\|\|` a shared start alone matches the sibling edge |

## crates/topology/src/pcurve.rs

- before: 7 survivors; after: 0 survivors, 24 caught, 11 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/pcurve.rs:168:48: replace && with \|\| in PCurveRegistry::remove_for_retired_entities | (a) | `pcurve::tests::remove_for_retired_entities_drops_an_entry_when_either_side_retires` — entry with a retired edge and a live face (and the mirror case) must be dropped; `\|\|` retains it |
| crates/topology/src/pcurve.rs:174:9: replace PCurveRegistry::remove_face with () | (a) | `pcurve::tests::remove_face_drops_exactly_that_face_and_keeps_the_others` — len falls 2 → 1 |
| crates/topology/src/pcurve.rs:174:44: replace != with == in PCurveRegistry::remove_face | (a) | `pcurve::tests::remove_face_drops_exactly_that_face_and_keeps_the_others` — asserts the removed face has no entry and the other face's entry survives |
| crates/topology/src/pcurve.rs:181:9: replace PCurveRegistry::len -> usize with 0 | (a) | pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — two indexed uses, `len() == 2 |
| crates/topology/src/pcurve.rs:181:9: replace PCurveRegistry::len -> usize with 1 | (a) | pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — two indexed uses, `len() == 2 |
| crates/topology/src/pcurve.rs:187:9: replace PCurveRegistry::is_empty -> bool with true | (a) | pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — non-empty fixture asserts `!is_empty() |
| crates/topology/src/pcurve.rs:187:9: replace PCurveRegistry::is_empty -> bool with false | (a) | pcurve::tests::registry_len_and_is_empty_track_indexed_uses` — fresh registry asserts `is_empty() |

## crates/topology/src/shell.rs

- before: 3 survivors; after: 0 survivors, 5 caught, 4 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/shell.rs:59:9: replace Shell::is_empty -> bool with true | (a) | shell::tests::is_empty_separates_the_sentinel_from_a_real_shell` — one-face shell asserts `!is_empty() |
| crates/topology/src/shell.rs:59:9: replace Shell::is_empty -> bool with false | (a) | shell::tests::is_empty_separates_the_sentinel_from_a_real_shell` — `Shell::empty()` asserts `is_empty() |
| crates/topology/src/shell.rs:73:9: replace Shell::faces_mut -> &mut[FaceId] with Vec::leak(Vec::new()) | (a) | shell::tests::faces_mut_exposes_the_stored_faces_for_in_place_replacement` — slice len 2 and a write through it is visible in `faces() |

## crates/topology/src/solid.rs

- before: 2 survivors; after: 0 survivors, 3 caught, 2 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/solid.rs:39:9: replace Solid::set_outer_shell with () | (a) | `solid::tests::set_outer_shell_replaces_the_stored_boundary` — outer shell reads back as the new handle |
| crates/topology/src/solid.rs:50:9: replace Solid::add_inner_shell with () | (a) | `solid::tests::add_inner_shell_appends_without_disturbing_the_outer_shell` — cavity count grows 0 → 1 → 2 in order, outer shell untouched |

## crates/topology/src/topology.rs

- before: 19 survivors; after: 1 survivors, 115 caught, 36 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/topology.rs:262:9: replace Topology::set_wire_body_class -> Result<(), TopologyError> with Ok(()) | (a) | `topology::tests::wire_body_class_accepts_only_the_wire_tag` — a non-wire tag and a stale handle must both return an error, not `Ok`. |
| crates/topology/src/topology.rs:262:23: replace != with == in Topology::set_wire_body_class | (a) | `topology::tests::wire_body_class_accepts_only_the_wire_tag` — `BodyClass::Wire` must be accepted and stored; Solid/Sheet/General must be refused. |
| crates/topology/src/topology.rs:289:9: replace Topology::allocated_slot_count -> usize with 0 | (a) | `topology::tests::allocated_slot_count_is_a_lifetime_high_water_mark` — 2 vertices + 1 edge is 3 slots. |
| crates/topology/src/topology.rs:289:9: replace Topology::allocated_slot_count -> usize with 1 | (a) | `topology::tests::allocated_slot_count_is_a_lifetime_high_water_mark` — an empty topology reports 0 slots. |
| crates/topology/src/topology.rs:376:37: delete ! in Topology::restore_preserving_handle_slots | (a) | `topology::tests::restore_preserving_handle_slots_keeps_live_face_authority_handles` — a face whose Loop/Coedge authority survived the restore keeps its handles instead of being re-promoted onto fresh ones (`num_loops`/`num_coedges` unchanged). |
| crates/topology/src/topology.rs:471:9: replace Topology::reserve with () | (b) | Pure `Vec` capacity hint; `Arena` exposes no capacity accessor and its fields are private to `crate::arena`, so no behaviour observable from this module changes. Still missed in the proof run, as expected. |
| crates/topology/src/topology.rs:538:9: replace Topology::set_solid_attributes -> Result<(), TopologyError> with Ok(()) | (a) | `topology::tests::solid_attributes_are_stored_cleared_and_validated` — attributes must read back, an empty value must clear, and a non-live handle must yield `SolidNotFound`. |
| crates/topology/src/topology.rs:706:25: delete match arm [] in Topology::propagate_attributes_for_op | (a) | `topology::tests::a_merge_with_no_attributed_inputs_is_not_a_conflict` — a merge with nothing to carry reports `merge_conflicts == 0`, not 1. |
| crates/topology/src/topology.rs:740:9: replace Topology::load_journal with () | (a) | `topology::tests::load_journal_installs_the_given_history` — the loaded entry must be present (op id, kind) and the tick sync must suppress a barrier on the next `journal_begin`. |
| crates/topology/src/topology.rs:831:9: replace Topology::face_id_from_index -> Option<FaceId> with None | (a) | `topology::tests::face_id_from_index_resolves_live_faces_only` — a live face's index resolves back to the same `FaceId`. |
| crates/topology/src/topology.rs:882:61: replace match guard replacement_id == wire_id with true in Topology::boundary_loop_specs | (a) | `topology::tests::wire_replacement_rebuilds_only_the_replaced_wires_loop` — replacing a face's inner wire must not rewrite the outer loop with the replacement's edges. |
| crates/topology/src/topology.rs:1032:48: delete ! in Topology::replace_boundary_wire | (a) | `topology::tests::wire_replacement_leaves_unrelated_face_authority_untouched` — a face that does not reference the replaced wire keeps its exact Loop/Coedge handles. |
| crates/topology/src/topology.rs:1354:9: replace Topology::is_empty_solid -> bool with true | (a) | `topology::tests::empty_result_solids_are_distinguished_from_faced_ones` — a faced solid is not an empty-result sentinel. |
| crates/topology/src/topology.rs:1354:9: replace Topology::is_empty_solid -> bool with false | (a) | `topology::tests::empty_result_solids_are_distinguished_from_faced_ones` — `add_empty_solid` is one. |
| crates/topology/src/topology.rs:1356:17: replace && with \|\| in Topology::is_empty_solid | (a) | `topology::tests::empty_result_solids_are_distinguished_from_faced_ones` — a faced outer shell with no inner shells satisfies only the first conjunct and must still be non-empty. |
| crates/topology/src/topology.rs:1436:13: delete match arm [] in Topology::pcurve | (a) | `topology::tests::an_edge_absent_from_a_face_boundary_is_not_a_seam` — no use at all is `Ok(None)`, not `SeamPcurveAmbiguous`. |
| crates/topology/src/topology.rs:1479:13: delete match arm [] in Topology::set_pcurve | (a) | `topology::tests::an_edge_absent_from_a_face_boundary_is_not_a_seam` — storing on an unused edge is `NonManifold`, not `SeamPcurveAmbiguous`. |
| crates/topology/src/topology.rs:1503:13: delete match arm [] in Topology::remove_pcurve | (a) | `topology::tests::an_edge_absent_from_a_face_boundary_is_not_a_seam` — removing from an unused edge is `Ok(None)`, not `SeamPcurveAmbiguous`. |
| crates/topology/src/topology.rs:1624:9: replace Topology::validate_coedge_authority -> Result<(), TopologyError> with Ok(()) | (a) | `topology::tests::coedge_writes_are_refused_without_loop_ownership` — `set_coedge_pcurve` / `remove_coedge_pcurve` / `set_coedge_periodic_winding` all refuse a coedge its named loop does not own. |

## crates/topology/src/validation.rs

- before: 78 survivors; after: 7 survivors, 237 caught, 35 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/validation.rs:37:5: replace validate_wire_closed -> Result<(), TopologyError> with Ok(()) | (a) | wire_closure_rejects_an_open_chain_and_a_broken_ring |
| crates/topology/src/validation.rs:43:18: replace == with != in validate_wire_closed | (a) | wire_closure_rejects_an_open_chain_and_a_broken_ring |
| crates/topology/src/validation.rs:48:35: replace <= with > in validate_wire_closed | (a) | wire_closure_accepts_coincident_but_distinct_endpoint_vertices |
| crates/topology/src/validation.rs:162:33: replace == with != in validate_shell_closed | (a) | closed_shell_report_names_free_and_over_shared_edges |
| crates/topology/src/validation.rs:210:13: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_diverging_use_count |
| crates/topology/src/validation.rs:209:13: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_diverging_closure_flag |
| crates/topology/src/validation.rs:218:17: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_flipped_compatibility_orientation |
| crates/topology/src/validation.rs:217:17: replace \|\| with && in validate_face_loops | (a) | face_loops_reject_a_flipped_compatibility_orientation |
| crates/topology/src/validation.rs:731:5: replace pcurve_type_label -> &'static str with "" | (a) | proof_refusal_names_the_stored_pcurve_type |
| crates/topology/src/validation.rs:731:5: replace pcurve_type_label -> &'static str with "xyzzy" | (a) | proof_refusal_names_the_stored_pcurve_type |
| crates/topology/src/validation.rs:740:5: replace pcurve_definition_is_finite -> bool with true | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:743:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:746:69: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:752:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:751:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:750:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:757:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:756:17: replace && with \|\| in pcurve_definition_is_finite | (a) | non_finite_pcurve_definitions_are_caught_before_evaluation |
| crates/topology/src/validation.rs:800:24: replace \|\| with && in check_same_parameter | (a) | sampled_parameter_reports_max_for_a_half_open_parameter_range |
| crates/topology/src/validation.rs:804:30: replace + with - in check_same_parameter | (a) | sampled_parameter_reports_max_for_a_half_open_parameter_range |
| crates/topology/src/validation.rs:804:30: replace + with * in check_same_parameter | (a) | sampled_parameter_reports_max_for_a_half_open_parameter_range |
| crates/topology/src/validation.rs:809:26: replace / with % in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:809:26: replace / with * in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:810:25: replace * with / in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:810:31: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:812:28: replace \|\| with && in check_same_parameter | (a) | a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines |
| crates/topology/src/validation.rs:823:20: replace * with / in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:823:26: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:825:16: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map |
| crates/topology/src/validation.rs:825:16: replace - with / in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map |
| crates/topology/src/validation.rs:825:20: replace * with + in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map |
| crates/topology/src/validation.rs:825:20: replace * with / in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map |
| crates/topology/src/validation.rs:825:26: replace - with + in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map |
| crates/topology/src/validation.rs:825:26: replace - with / in check_same_parameter | (a) | sampled_parameter_pins_the_reversed_map |
| crates/topology/src/validation.rs:832:13: replace \|\| with && in check_same_parameter | (a) | an_overflowing_sampled_deviation_reports_the_fail_closed_sentinel |
| crates/topology/src/validation.rs:831:13: replace \|\| with && in check_same_parameter | (b) | Mutant reads `... \|\| (!on_surface.finite && !on_curve.finite) \|\| !deviation.finite`; same implication chain — a non-finite operand always forces a non-finite deviation — so the regrouped form has the same value. |
| crates/topology/src/validation.rs:830:13: replace \|\| with && in check_same_parameter | (b) | `&&` binds tighter, so the mutant reads `(!g.finite && !on_surface.finite) \|\| !on_curve.finite \|\| !deviation.finite`; a non-finite g, surface point or curve point each force a non-finite deviation, so both forms reduce to the last disjunct. |
| crates/topology/src/validation.rs:838:22: replace > with >= in check_same_parameter | (a) | sampled_parameter_witnesses_the_first_maximal_sample |
| crates/topology/src/validation.rs:846:26: replace + with - in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:846:26: replace + with * in check_same_parameter | (a) | sampled_parameter_pins_the_forward_map |
| crates/topology/src/validation.rs:878:24: replace \|\| with && in check_same_parameter_strict | (a) | non_finite_parameter_bounds_are_caught_before_evaluation |
| crates/topology/src/validation.rs:896:53: replace \|\| with && in check_same_parameter_strict | (a) | a_single_non_finite_pcurve_endpoint_is_caught_before_the_surface |
| crates/topology/src/validation.rs:921:13: replace \|\| with && in check_same_parameter_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof |
| crates/topology/src/validation.rs:920:13: replace \|\| with && in check_same_parameter_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof |
| crates/topology/src/validation.rs:919:13: replace \|\| with && in check_same_parameter_strict | (b) | Mutant reads `(!on_surface_start.finite && !on_surface_end.finite) \|\| !d0.finite \|\| !d1.finite`; a non-finite surface image forces its own endpoint distance non-finite, so both forms reduce to `!d0.finite \|\| !d1.finite`. |
| crates/topology/src/validation.rs:952:29: replace > with >= in check_same_parameter_strict | (a) | strict_parameter_certifies_exactly_at_the_linear_tolerance |
| crates/topology/src/validation.rs:962:56: replace >= with < in check_same_parameter_strict | (a) | strict_parameter_reports_the_larger_endpoint_deviation_plus_its_bound |
| crates/topology/src/validation.rs:964:47: replace + with - in check_same_parameter_strict | (a) | strict_parameter_reports_the_larger_endpoint_deviation_plus_its_bound |
| crates/topology/src/validation.rs:1000:33: replace > with >= in validate_same_parameter_strict | (a) | a_deviation_exactly_at_the_bound_is_inside_every_band |
| crates/topology/src/validation.rs:1032:31: replace \|\| with && in validate_same_parameter | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound |
| crates/topology/src/validation.rs:1040:33: replace > with >= in validate_same_parameter | (a) | a_deviation_exactly_at_the_bound_is_inside_every_band |
| crates/topology/src/validation.rs:1081:9: replace \|\| with && in check_same_range | (a) | a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines |
| crates/topology/src/validation.rs:1080:9: replace \|\| with && in check_same_range | (a) | a_non_finite_uv_on_a_plane_is_reported_before_the_surface_declines |
| crates/topology/src/validation.rs:1079:9: replace \|\| with && in check_same_range | (b) | Mutant reads `(!t_start.finite && !t_end.finite) \|\| !uv0.finite \|\| !uv1.finite`; a non-finite bound forces its own uv evaluation non-finite, so both forms reduce to the uv tests. |
| crates/topology/src/validation.rs:1094:9: replace \|\| with && in check_same_range | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof |
| crates/topology/src/validation.rs:1093:9: replace \|\| with && in check_same_range | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof |
| crates/topology/src/validation.rs:1092:9: replace \|\| with && in check_same_range | (a) | a_single_non_finite_surface_endpoint_fails_the_range_checks_closed |
| crates/topology/src/validation.rs:1130:38: replace \|\| with && in check_same_range_strict | (a) | non_finite_parameter_bounds_are_caught_before_evaluation |
| crates/topology/src/validation.rs:1148:53: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_pcurve_endpoint_is_caught_before_the_surface |
| crates/topology/src/validation.rs:1171:9: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof |
| crates/topology/src/validation.rs:1170:9: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_endpoint_distance_refuses_the_strict_parameter_proof |
| crates/topology/src/validation.rs:1169:9: replace \|\| with && in check_same_range_strict | (a) | a_single_non_finite_surface_endpoint_fails_the_range_checks_closed |
| crates/topology/src/validation.rs:1208:31: replace \|\| with && in validate_same_range_strict | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound |
| crates/topology/src/validation.rs:1213:26: replace > with >= in validate_same_range_strict | (a) | a_deviation_exactly_at_the_bound_is_inside_every_band |
| crates/topology/src/validation.rs:1256:31: replace \|\| with && in validate_solid_pcurve_contracts | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound |
| crates/topology/src/validation.rs:1281:45: replace > with >= in validate_solid_pcurve_contracts | (a) | solid_pcurve_contracts_accept_a_use_exactly_at_the_bound |
| crates/topology/src/validation.rs:1293:38: replace > with == in validate_solid_pcurve_contracts | (b) | Dead comparison: on the one proved path the SameParameter deviation is this SameRange deviation plus a strictly positive arithmetic bound, so any range violation has already returned at the SameParameter check above. |
| crates/topology/src/validation.rs:1293:38: replace > with >= in validate_solid_pcurve_contracts | (b) | Same dead comparison: `range >= tolerance` implies `parameter > tolerance`, which returns at the SameParameter check above, so this arm can never decide the outcome. |
| crates/topology/src/validation.rs:1303:40: replace && with \|\| in validate_solid_pcurve_contracts | (b) | `parameter` and `range` are both `Some` exactly when `pcurve_oriented` resolves the use, which the `coedge.pcurve().is_none()` guard above already established, so the two options are never in disagreement and `&&`/`\|\|` coincide. |
| crates/topology/src/validation.rs:1327:5: replace validate_same_range -> Result<(), TopologyError> with Ok(()) | (a) | same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound |
| crates/topology/src/validation.rs:1327:31: replace \|\| with && in validate_same_range | (a) | every_tolerance_guard_rejects_a_negative_and_a_nan_bound |
| crates/topology/src/validation.rs:1335:26: replace > with == in validate_same_range | (a) | same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound |
| crates/topology/src/validation.rs:1335:26: replace > with >= in validate_same_range | (a) | same_range_rejects_an_offset_pcurve_and_accepts_it_at_its_own_bound |
| crates/topology/src/validation.rs:1356:31: replace \|\| with && in effective_edge_validation_tolerance | (a) | entity_tolerance_guards_reject_negative_vertex_and_edge_claims |
| crates/topology/src/validation.rs:1365:38: replace \|\| with && in effective_edge_validation_tolerance | (a) | entity_tolerance_guards_reject_negative_vertex_and_edge_claims |
| crates/topology/src/validation.rs:1430:36: replace != with == in check_vertex_ball | (a) | vertex_ball_reports_exactly_the_incident_edge_ends |
| crates/topology/src/validation.rs:1438:34: replace && with \|\| in check_vertex_ball | (a) | vertex_ball_reports_max_for_a_non_finite_curve_evaluation |
| crates/topology/src/validation.rs:1593:29: replace > with >= in validate_edge_tube | (a) | edge_tube_accepts_a_declared_bound_exactly_at_its_measured_deviation |

## crates/topology/src/vertex.rs

- before: 1 survivors; after: 0 survivors, 7 caught, 1 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/vertex.rs:44:9: replace Vertex::set_point with () | (a) | `vertex::tests::set_point_replaces_the_stored_position` — moved point reads back component-wise, tolerance unchanged |

## crates/topology/src/wire.rs

- before: 3 survivors; after: 0 survivors, 7 caught, 6 unviable.

| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/wire.rs:117:9: replace Wire::edges_mut -> &mut[OrientedEdge] with Vec::leak(Vec::new()) | (a) | wire::tests::edges_mut_exposes_the_stored_edges_for_in_place_replacement` — slice len 2 and a write through it is visible in `edges() |
| crates/topology/src/wire.rs:123:9: replace Wire::is_closed -> bool with true | (a) | wire::tests::closed_flag_round_trips_both_ways` — open-wire fixture asserts `!is_closed() |
| crates/topology/src/wire.rs:133:9: replace Wire::set_body_class with () | (a) | `wire::tests::set_body_class_replaces_the_stored_class` — set Sheet then Solid, read back each time |

## Notes

- `compound.rs`, `lib.rs` and `transaction.rs` had mutants but zero survivors; they needed no work.
- This crate has **zero (c) verdicts** — unlike geometry (87), nothing here needed a geometry judgment
  call. That is expected: remus-topology is a data-structure layer, so a survivor is either a genuine
  coverage gap (now pinned by a test) or a provable no-op (recorded with its equivalence argument).
- The one arguable verdict, `arena.rs:55:9 replace Hash::hash with ()`, is kept as (a). A
  `HashMap`/`HashSet` round-trip *cannot* kill it (`Eq` resolves every collision, so the map stays
  correct, just degenerate). The landed test asserts the real contract instead — the hash depends on the
  index, or every handle in a CAD-scale model lands in one bucket and lookup degrades to a linear scan —
  via a `DefaultHasher` distinctness check (no exact hash values asserted), alongside a round-trip that
  documents the `Hash`/`Eq` consistency half. The authoritative sweep confirms the kill.
- Residual (b) mutants cluster in the shapes a data-structure layer predicts: `Vec` capacity hints with
  no observable effect (`with_capacity`, `reserve`), strict/non-strict flips at a boundary where both
  sides are provably no-ops (empty slices, resize-to-same-length, unrepresentable float equalities),
  guards re-checked downstream (tolerance rejected twice, same domain returned twice), `||`/`&&`
  regroupings over operands linked by an implication chain, and unreachable match arms (a `Vec`-push-only
  map with no empty entry). Each row carries its own argument; none is a bare "equivalent".
- No bit-exact float assertions on computed values: topology tests assert on stored-and-read-back values,
  structural properties (counts, ids, ordering, error variants), and closed-form integer/rational
  expectations, all exact by construction.
- Production-region check: `git diff b13ff97b HEAD -- crates/topology/src/` touches 19 files with 5888
  insertions and 81 deletions; every deletion is inside a `#[cfg(test)]` module (a two-module merge in
  `validation.rs` plus import/allow-header updates), no `pub` item line changed, and no `#[test]` fn
  present at the base is missing at the head.

## Reproduce

```
cargo mutants -p remus-topology --no-config --baseline skip --timeout 60 -j 8 -- -p remus-topology
```

Raw logs (parked baseline, per-file agent runs, authoritative after-sweep) live in untracked
`mutants.out` directories under the session scratchpad and are not committed.
