# PERF-T02 / PERF-T03: mutation-local rollback and append-only construction

## Contract (written before implementation, 2026-09-28)

PERF-T01 shares unchanged nested entry states but keeps an outer full
`Topology` clone plus genuine full clones for every changed-state savepoint:
all rollback storage remains O(document size). This slice replaces per-scope
full clones with mutation-local undo records (PERF-T02) and adds a guarded
append-only fast path for qualified construction (PERF-T03). It is not another
snapshot-sharing wrapper: after this slice, no transaction scope clones the
document for its entry state on any covered path.

`TopologyError` gains exactly one variant, `AppendOnlyGuardTrip`: the guard
must refuse through the existing `Result`-shaped mutators, and a typed trip
keeps guard-violation tests able to distinguish a trip from a genuine
failure. The trip never surfaces through `run_append_only` (tripped scopes
rewind and retry internally); the only out-of-scope touch is the
compiler-forced exhaustive-match arm in `crates/wasm/src/error.rs` (one
line, batch dispatch untouched). On current-main integration, the public
`RollbackSnapshot::restore_preserving_handle_slots` compatibility method is
removed: an O(1) scope mark cannot supply the full entry snapshot required
for checkpoint-barrier retirement semantics. Its two in-repository failure
callers now use `restore`, which undoes their uncommitted retirements. External
checkpoint barriers still use `Topology::restore_preserving_handle_slots`
with an explicit `Topology` clone.

## 1. State components under the transaction

Every component below is covered by rollback. High-water marks are
deliberately never rewound.

1. **Entity arenas (10):** vertices, edges, wires, faces, shells, solids,
   compounds, compsolids, loops, coedges. Each arena holds `items: Vec<T>`,
   `live: Vec<bool>`, `live_len: usize`. Rollback restores values and
   liveness; allocation high-water (`items.len()` per arena, and the summed
   `allocated_slot_count`) only grows.
2. **Liveness:** retiring an entity clears its bit; rollback of a transaction
   revives entities the failed scope retired (unlike the checkpoint barrier,
   which keeps them retired). Active-scope allocations rolled back become
   dead tombstones whose values are retained, exactly as
   `Arena::restore_for_rollback` leaves post-snapshot values behind.
3. **Wires / coedges / loops derivation:** `Face::boundary_loops`,
   `Loop { face, coedges, closed }`, `Coedge { edge, forward, parent_loop,
   pcurve, periodic_winding }`, plus the compatibility `Wire` facade
   (`outer_wire`, `inner_wires`). Derivation installs allocate fresh
   loops/coedges and retire replaced ones; all of it is ordinary
   arena allocation/retirement/value traffic and is logged as such.
4. **PCurve registry:** the `(edge, face, forward) -> CoedgeId` compatibility
   index. Geometry lives on coedges (covered by arena logging); the index
   itself logs per-key old values (`None` = key absent).
5. **Attributes:** solid/face relational maps. Logs per-key old values
   (`None` = key absent, i.e. entry added by the scope).
6. **Journal:** append-only entries plus the ordinal index. A scope mark
   records `entries.len()`; rollback truncates entries and the ordinal maps
   to the mark while `next_op` / `next_ordinal` high-water marks are
   preserved (never reissued), mirroring `Journal::restore_preserving_ids`.
   `mutation_ticks` is saved at the mark and restored with the state, so a
   rolled-back scope never leaves a journal gap (a later `journal_begin`
   must not synthesize a global barrier for rolled-back work).
7. **Derivations:** the adjacency index is computed on demand and stored
   nowhere; there is no cached derivation to invalidate. Persistent
   references (`naming`) resolve against live topology plus the journal, so
   they follow automatically once arenas and journal are restored.
8. **Transaction coordination:** the undo log, scope marks, the append-only
   guard flag, and a lineage generation counter. Coordination state is reset
   (not rolled back) by foreign full restores and by `Clone`, exactly as the
   PERF-T01 coordinator resets on clone. It is excluded from logical-state
   equality, like the coordinator's weak reference today.

Serialization formats, arena slot reuse policy, and compaction are untouched:
no ID is ever reissued, tombstones stay allocated, checkpoints keep today's
sticky-retirement barrier semantics.

## 2. Mutable access routes (audit)

External crates reach topology state only through `Topology` methods (arena
fields are private). The complete mutating surface:

- **Allocation:** `add_vertex/edge/wire/shell/solid/compound/compsolid`
  (via `arena_api!`), `add_face` (allocates the face and installs fresh
  loops/coedges plus index entries), `add_empty_solid` (shell + solid).
- **Exclusive entity references:** `vertex_mut/edge_mut/wire_mut/face_mut/
  shell_mut/solid_mut/compound_mut/compsolid_mut` (via `arena_get_mut!`).
  Each returns `&mut T`; all further in-place writes go through entity
  setters (`Vertex::set_point/set_tolerance`, `Edge::set_start/set_end/
  set_curve/set_trim/set_tolerance`, `Wire::edges_mut`,
  `Face::set_surface/set_reversed/compose_orientation`,
  `Shell::faces_mut`, `Solid::set_outer_shell/add_inner_shell`) or the
  deprecated wire/body facades (`Face::set_outer_wire/inner_wires_mut`,
  `Shell::faces_mut`). There is deliberately **no** public `loops_mut` /
  `coedges_mut` / `*_mut` for the pcurve registry: loops, coedges and the
  index mutate only through `Topology` methods.
- **Boundary mutation:** `replace_boundary_wire`,
  `set_face_boundary_wires`, `build_face_loops` (derivation fallback that
  allocates when a legacy face has no authority yet).
- **Retirement:** `delete_solid` (retires the unshared subtree: solid,
  shells, faces, wires, edges, vertices, plus owned loops/coedges, and
  prunes pcurve/attribute entries).
- **PCurves:** `set_pcurve_oriented/remove_pcurve_oriented`,
  `set_pcurve/remove_pcurve` (adapters over the oriented pair),
  `set_coedge_pcurve/remove_coedge_pcurve`,
  `set_coedge_periodic_winding`.
- **Attributes:** `set_solid_attributes/set_face_attributes`,
  `propagate_attributes_for_op` (writes only through `set_face_attributes`).
- **Journal:** `journal_begin` (may append a global-barrier entry),
  `journal_record_evolution/journal_record_barrier`, `load_journal`.
- **Body class:** `set_shell_body_class/set_wire_body_class`.
- **Capacity:** `reserve` (no logical change; still a scope boundary event).
- **Restores:** `restore_for_rollback` (transaction: undoes retirements)
  and `restore_preserving_handle_slots` (checkpoint barrier: retirements
  stay retired). Both remain supported as foreign full restores (ad-hoc
  clone/restore pairs in heal, offset, io, wasm helpers, and tests call
  these directly with same-lineage snapshots).
- **In-crate direct arena traffic** (`topology.rs` private helpers):
  `install_face_loop_specs_carrying` (retires replaced loops/coedges,
  allocates new ones, `get_mut` writes to loops/faces/coedges, index
  writes), `replace_boundary_wire` / `set_face_boundary_wires`
  (`wires/faces.get_mut` commits), `set_wire_body_class`
  (`wires.get_mut`), `delete_solid` (all `retire` calls),
  `propagate_attributes_for_op` (attribute writes). Every one of these is
  reachable only from a public method above, so guarding the public methods
  **and** these helpers covers the surface. Crate-internal tests that poke
  `topo.coedges` / `topo.loops` / `topo.pcurves` fields directly are
  test-only; they use explicit clone/restore oracles and are updated to keep
  the undo log consistent (see section 6).

## 3. Write-guard rule (PERF-T02)

A write guard captures old state **before** a mutable reference escapes or a
mutation lands:

- `*_mut(id)`: when a scope is active, clone the live entity value into the
  log before returning `&mut T`. All downstream in-place writes through that
  reference (including `edges_mut` / `faces_mut` / `inner_wires_mut` slices)
  are then covered by the single capture. When no scope is active, `*_mut`
  keeps today's cost (a tick bump and sharing invalidation); logging is
  scoped because there is nothing to roll back to. Failed lookups (`None`)
  record nothing and return the typed not-found error unchanged.
- Allocation records `(arena, index)` with no old value; rollback retires
  the slot (tombstone retained, never reused).
- Retirement and in-place arena writes (`get_mut` commits, `retire`) record
  the previous live value first.
- Registry/attribute writes record the previous per-key value first.
- Journal appends need no per-entry record (truncation to the mark length
  suffices); `load_journal` replaces the journal wholesale and is treated
  as a scope-boundary event that also resyncs ticks, as today.
- Unsupported mutation paths retain a correct fallback: any path the audit
  missed would have to bypass all of the above, which is impossible from
  outside the crate (private fields) and enumerated inside it. If a future
  accessor is added, the rule is: log before escape, else the new path must
  take an eager full snapshot **before** writing (never after an unrecorded
  write). The `only_unchanged_entry_states_share_storage`-era sharing
  invalidation sites are replaced by these guards one-for-one.

## 4. Nesting, failure, validation, checkpoint, and restore semantics

- **Nested success:** an inner scope's mark is dropped; its records stay in
  the log and belong to the outer scope. Commit is O(1).
- **Caught inner failure:** rewind to the inner mark (apply record inverses
  in reverse), keep outer records. The outer scope may then succeed; a later
  `journal_begin` sees no gap.
- **Outer failure after inner success:** rewind to the outer mark, discarding
  inner-committed work too.
- **Validation failure (`run_validated`):** identical to operation failure at
  that scope; the validator observes staged state before the decision.
- **Retained checkpoints:** user checkpoints are independent full clones
  (`Rc<Topology>`); undo rewind never touches them. The WASM checkpoint
  first-write copy (`Rc::make_mut`) and the non-boolean batch dispatch
  clones remain O(document size) and are disclosed as remaining costs, not
  claimed as wins.
- **Restore behavior:** `RollbackSnapshot::capture` / `restore` and
  `run_transacted` / `run_validated` keep their signatures. `restore`
  rewinds to its mark. The former snapshot compatibility method for sticky
  checkpoint retirements is removed as described above. Foreign full restores (`restore_for_rollback`,
  `restore_preserving_handle_slots`) truncate to the clone-time log prefix
  only when lineage identity and unchanged state certify that prefix.
  Changed clone chains and unrelated values take one full pre-restore inverse
  while a scope is live; that document-sized cost preserves complete outer
  rollback, including edits made before the restore. Restoring a scope mark
  into a different `Topology` value remains outside the snapshot contract.
- **Handles allocated in failed scopes remain permanently stale:** rollback
  retires their slots; later allocations append above the preserved
  high-water mark; typed lookups and `id_from_index` fail for them forever,
  including after checkpoint restores. No compaction, no ID reuse.

## 5. Storage-mechanism selection (undo vs chunk/page COW)

Both are prototyped narrowly against the measured whole-clone baseline and
only the winner ships in production:

- **Mutation-local undo:** O(touched slots x old-value size + touched
  keys + journal tail). Intercepts at the existing `&mut`-escape points, so
  entity setters need no changes and aliasing is safe by construction (one
  capture covers every write through the escaped reference). Metadata cost
  is per touched slot/key; retained storage is the log itself, bounded by
  scope writes and dropped on commit.
- **Chunk/page COW:** O(touched chunks). Requires chunk indirection in the
  arena layout and a write barrier at the arena-index level — but entity
  writes after `*_mut` escape never pass through the arena, so COW cannot
  see them without changing every accessor's return shape. Scattered writes
  copy whole chunks; large NURBS payloads sharing a chunk with a touched
  slot are copied anyway.

Selection gate: the mechanism must (a) cover every route in section 2 with
no silent bypass, (b) keep all public signatures, (c) beat the whole-clone
baseline on the fixed-local-edit benchmark with large NURBS payloads and
retained checkpoints, accounting for metadata and retained storage, not
just payload bytes. The prototype comparison and the kept implementation
are recorded in the evidence report.

## 6. Append-only construction (PERF-T03)

`run_append_only(op)` arms a guard plus an undo mark, then runs the
constructor:

- Allocations (any arena), new-entity registry/attribute indexing, and
  journal appends are allowed and logged as usual.
- Any write to **pre-existing** state — `*_mut` on a pre-mark slot,
  retirement, overwrite of a pre-existing registry/attribute key, journal
  replacement — trips the guard **before** the write lands. The scope is
  rewound (O(new content): retire new slots, drop new keys, truncate the
  journal tail) and the operation is re-executed once under the full
  `run_transacted` path. The closure must therefore be re-runnable (pure
  construction); the returned `AppendPath::{AppendOnly, FullFallback}`
  names the path taken so tests observe guard trips without a separate
  error channel.
- Rollback of a failed append-only scope retires every allocation the scope
  made without reissuing IDs; stale-handle guarantees are identical to the
  full path.
- Qualification: a native primitive path (`make_box` under the append-only
  scope) whose transaction cost follows new content; 150 box constructions
  over increasing pre-existing document sizes; injected failure after each
  allocation stage (vertices, edges, wires/faces, shell/solid); guard
  violation exercises (pre-existing vertex/edge/face mutation, retirement,
  attribute/pcurve/journal writes to pre-existing keys) proving fallback.

## 7. Equivalence and benchmark gates

- **Reference:** the existing full-snapshot implementation stays as the test
  oracle (legacy clone + `restore_for_rollback`), plus independent
  logical-state renderers and stale-handle assertions — never only live
  counts.
- **Deterministic mutation sequences** cover allocation, in-place mutation,
  retirement, nested rollback (success, caught, propagated, outer-after-
  inner, validation veto, repeated failure), attributes, pcurves (including
  seam-oriented uses and periodic windings), loop/coedge derivation, and
  journal edits (evolution, barrier, global-barrier gap detection).
- **Failure injection** fires at every meaningful mutation boundary,
  including after allocation, in-place edits, attribute/pcurve changes,
  boundary re-derivation, retirement, and journal recording.
- **State comparison** covers all ten arenas (values and liveness),
  loop/coedge authority and the pcurve index contents, attributes, journal
  entries plus ordinal index, and persistent-reference resolution; only
  intentionally monotonic identity high-water marks are excluded, and
  failed handles are asserted stale after later allocation and restore.
- **Baselines** measure a fixed local edit and repeated primitive
  construction against increasing unrelated document sizes (including large
  NURBS payloads and retained checkpoints), recording snapshot/clone
  counts, copied payload bytes, allocations, peak live memory, and elapsed
  time from identical fixture/harness sources.
- **Integration:** topology tests, representative native modeling/import
  failure paths, native and WASM transaction contracts, workspace checks
  (`clippy -D warnings`, `check-boundaries.sh`, `check-det-hash.sh`,
  `check-doc-paths.sh`), and relevant package qualification. Per-item batch
  commit semantics are preserved (batch dispatch itself is untouched; the
  boolean batch arms pick up the new mechanism through the unchanged
  `RollbackSnapshot` signatures, while other arms keep their clone policy).

A mechanism that cannot meet both the correctness gate (section 7,
equivalence) and the measurable-benefit gate (section 5, benchmark) is not
shipped: the evidence is retained and the milestone is reported undelivered.
