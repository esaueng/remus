| Mutant | Verdict | Killing test / reason |
| --- | --- | --- |
| crates/topology/src/attributes.rs:53:9: replace ColorRgb::r -> f64 with 0.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:53:9: replace ColorRgb::r -> f64 with 1.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:53:9: replace ColorRgb::r -> f64 with -1.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:59:9: replace ColorRgb::g -> f64 with 0.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:59:9: replace ColorRgb::g -> f64 with 1.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:59:9: replace ColorRgb::g -> f64 with -1.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:65:9: replace ColorRgb::b -> f64 with 0.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:65:9: replace ColorRgb::b -> f64 with 1.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:65:9: replace ColorRgb::b -> f64 with -1.0 | (a) | `attributes::tests::color_channels_are_stored_and_read_back_independently` |
| crates/topology/src/attributes.rs:84:9: replace EntityAttributes::is_empty -> bool with false | (a) | `attributes::tests::entity_attributes_are_empty_only_when_nothing_is_set` |
| crates/topology/src/attributes.rs:105:9: replace AttributeStore::solid -> Option<&EntityAttributes> with None | (a) | `attributes::tests::set_then_get_round_trips_and_unset_entities_read_none` |
| crates/topology/src/attributes.rs:105:9: replace AttributeStore::solid -> Option<&EntityAttributes> with Some(Box::leak(Box::new(Default::default()))) | (a) | `attributes::tests::set_then_get_round_trips_and_unset_entities_read_none` (unset solid must read `None`) |
| crates/topology/src/attributes.rs:116:9: replace AttributeStore::set_solid with () | (a) | `attributes::tests::set_then_get_round_trips_and_unset_entities_read_none`, also `setting_empty_attributes_clears_the_entry` |
| crates/topology/src/attributes.rs:134:9: replace AttributeStore::remove_solid -> Option<EntityAttributes> with None | (a) | `attributes::tests::remove_returns_the_stored_record_and_none_when_absent` |
| crates/topology/src/attributes.rs:134:9: replace AttributeStore::remove_solid -> Option<EntityAttributes> with Some(Default::default()) | (a) | `attributes::tests::remove_returns_the_stored_record_and_none_when_absent` |
| crates/topology/src/attributes.rs:139:9: replace AttributeStore::remove_face -> Option<EntityAttributes> with None | (a) | `attributes::tests::remove_returns_the_stored_record_and_none_when_absent` |
| crates/topology/src/attributes.rs:145:9: replace AttributeStore::len -> usize with 0 | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` |
| crates/topology/src/attributes.rs:139:9: replace AttributeStore::remove_face -> Option<EntityAttributes> with Some(Default::default()) | (a) | `attributes::tests::remove_returns_the_stored_record_and_none_when_absent` |
| crates/topology/src/attributes.rs:145:9: replace AttributeStore::len -> usize with 1 | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` |
| crates/topology/src/attributes.rs:145:27: replace + with - in AttributeStore::len | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` (2 solids + 1 face must be 3) |
| crates/topology/src/attributes.rs:145:27: replace + with * in AttributeStore::len | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` |
| crates/topology/src/attributes.rs:151:9: replace AttributeStore::is_empty -> bool with true | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` |
| crates/topology/src/attributes.rs:151:9: replace AttributeStore::is_empty -> bool with false | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` (fresh store), `setting_empty_attributes_clears_the_entry` |
| crates/topology/src/attributes.rs:151:32: replace && with \|\| in AttributeStore::is_empty | (a) | `attributes::tests::len_and_is_empty_count_solids_and_faces_together` (face-only and solid-only stores are not empty) |
| crates/topology/src/attributes.rs:157:9: replace AttributeStore::faces_with_attributes -> Vec<(FaceId, &EntityAttributes)> with vec![] | (a) | `attributes::tests::listings_yield_every_entry_sorted_by_index` |
| crates/topology/src/attributes.rs:165:9: replace AttributeStore::solids_with_attributes -> Vec<(SolidId, &EntityAttributes)> with vec![] | (a) | `attributes::tests::listings_yield_every_entry_sorted_by_index` |
| crates/topology/src/attributes.rs:176:36: delete ! in AttributeStore::remove_for_retired_entities | (a) | `attributes::tests::retiring_entities_removes_exactly_their_entries` (live solid must survive) |
| crates/topology/src/attributes.rs:176:9: replace AttributeStore::remove_for_retired_entities with () | (a) | `attributes::tests::retiring_entities_removes_exactly_their_entries` |
| crates/topology/src/attributes.rs:177:35: delete ! in AttributeStore::remove_for_retired_entities | (a) | `attributes::tests::retiring_entities_removes_exactly_their_entries` (live face must survive) |
| crates/topology/src/journal.rs:194:9: replace RecordedOrigin::as_str -> &'static str with "xyzzy" | (a) | `journal::tests::recorded_origin_names_are_stable` |
| crates/topology/src/journal.rs:194:9: replace RecordedOrigin::as_str -> &'static str with "" | (a) | `journal::tests::recorded_origin_names_are_stable` |
| crates/topology/src/journal.rs:394:9: replace JournalEntry::is_barrier -> bool with true | (a) | `journal::tests::only_barrier_payloads_are_barriers` (evolution entries must not sever) |
| crates/topology/src/journal.rs:405:9: replace JournalEntry::ticks_after -> u64 with 0 | (a) | `journal::tests::entries_carry_distinct_tick_counts_derived_from_their_position` |
| crates/topology/src/journal.rs:405:9: replace JournalEntry::ticks_after -> u64 with 1 | (a) | `journal::tests::entries_carry_distinct_tick_counts_derived_from_their_position` |
| crates/topology/src/journal.rs:455:9: replace PendingOp::add_scope with () | (a) | `journal::tests::pre_operation_scope_from_the_pending_token_reaches_the_entry` |
| crates/topology/src/journal.rs:493:9: replace Journal::is_empty -> bool with true | (a) | `journal::tests::a_recorded_journal_is_not_empty` |
