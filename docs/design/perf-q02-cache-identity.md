# PERF-Q02: persistent classification-cache identity and mutation audit

## 1. Identity contract

Persistent spatial-query preparation is keyed by whole-topology
[`CacheIdentity`](../../../crates/topology/src/topology.rs) (`lineage`,
`generation`) plus the queried solid and all numerical query options.
An entry is usable only when its stored identity equals the querying
topology's current identity and the topology is not poisoned.

### Independent instances

Every `Topology::new()` allocates a fresh non-zero lineage from a
process-local atomic. Two independently constructed documents never share
a lineage, so the same numeric `SolidId` index in different documents
never aliases one preparation.

### Clones

`Topology::clone()` copies every arena, the journal, and `mutation_ticks`,
but allocates a fresh cache lineage and starts with an empty undo log
(carrying the source undo lineage only for same-lineage restore
optimisation). The clone carries the source cache generation and poison
state so a saturated source cannot become cacheable by cloning. Two
diverging clones therefore miss each other from birth; a needless rebuild
is acceptable, stale reuse is not.

### Deserialization

Arena documents carry entities only. `deserialize_*` in `crates/io` takes
`&mut Topology` and allocates through the ordinary `add_*`/`set_*` paths,
each of which bumps the destination generation. A document loaded into a
fresh `Topology::new()` keeps that fresh lineage; a document appended to
an existing topology bumps that topology's generation. Runtime identity is
never written to or read from persistent formats.

### Checkpoint restore (`restore_preserving_handle_slots`)

Keeps the destination lineage, bumps the cache generation forward
(including via `undo_truncate_for_foreign_restore`), and restores
`mutation_ticks` backwards with the model state for journal continuity.
The tick rewind is exactly why the cache cannot key by ticks: the
generation forward-bump closes the ABA case where two different
geometries reach the same old tick.

### Rollback (`restore_for_rollback`, `restore_rollback_fields`, `undo_rewind_scope`)

All three bump the cache generation forward while restoring ticks
backwards and undoing retirements. Failed operations therefore invalidate
even when the restored tick equals an earlier generation. Handles
allocated inside the rolled-back scope stay stale via high-water
preservation, so a later solid can never reuse their numeric index within
one lineage.

### Foreign restore (changed-clone or unrelated snapshot inside live scopes)

`undo_truncate_for_foreign_restore` bumps the generation and either
truncates to a same-lineage prefix or records a full `ForeignRestore`
inverse. Callers (`restore_*`) bump again; extra bumps are harmless.
A changed intermediate clone therefore never aliases its source.

### ID retirement (`delete_solid`)

Retires without reusing slots and bumps the generation once at entry
(conservative: even a refused delete that changes nothing invalidates).
Within one lineage a retired numeric ID can never alias a later entity;
across lineages the lineage differs anyway.

### When entries become unusable

- Any generation mismatch for the querying lineage: stale generations are
  dropped on the next lookup for that lineage so rolled-back states cannot
  pin dead preparations.
- Any lineage mismatch: entries for other documents are kept (they may
  still serve their documents) up to the capacity bound.
- Poisoned topology (`is_cache_poisoned()`): every lookup misses without
  storing. Poison covers `u64::MAX` generation and the reserved lineage 0.
- Zero capacity: every lookup misses without storing.
- Invalid solid or missing entity: errors, never cached.

### Overflow without aliasing

`bump_cache_generation` saturates instead of wrapping. The transition
into `u64::MAX` sets `cache_poisoned = true`, and the saturated counter
itself is never cacheable. Two different geometries therefore never share
the saturated value. Lineage allocation skips zero forever; wrapping the
64-bit lineage counter is not reachable in practice (one allocation per
`Topology` value) and still never allocates the reserved zero.

## 2. Mutation-access audit (all routes bump before escape)

| Route | File | Invalidation |
|---|---|---|
| `vertex_mut`, `edge_mut`, `wire_mut`, `face_mut`, `shell_mut`, `solid_mut`, `compound_mut`, `compsolid_mut`, `compsolid_mut` | `topology.rs` (`arena_get_mut!`) | `record_*` then ticks + cache bump before `&mut` escapes |
| `add_vertex`, `add_edge`, `add_wire`, `add_shell`, `add_solid`, `add_compound`, `add_compsolid` | `topology.rs` (`arena_api!`) | ticks + cache bump before alloc |
| `add_face` (plus internal loop/coedge allocs) | `topology.rs` | ticks + cache bump before alloc; `install_*` bumps again (harmless) |
| `add_empty_solid` | `topology.rs` | via inner `add_shell`/`add_solid` |
| `set_shell_body_class` | `topology.rs` | via `shell_mut` |
| `set_wire_body_class` | `topology.rs` | cache-only bump after `record_*`, before `get_mut` (ticks preserved: journal gap detection historically excludes this tag) |
| `set_solid_attributes`, `set_face_attributes` | `topology.rs` | cache-only bump after `record_*` (conservative; attributes never affect bounds) |
| `replace_boundary_wire`, `set_face_boundary_wires` | `topology.rs` | ticks + cache bump after validation, before writes; `install_*` bumps again |
| `build_face_loops` (derivation fallback) | `topology.rs` | via `install_face_loop_specs_carrying` entry bump |
| `install_face_loop_specs_carrying` (all loop/coedge rebuilds) | `topology.rs` | cache bump at entry, before any retire/alloc/write |
| `delete_solid` (retirement) | `topology.rs` | ticks + cache bump at entry (conservative: refused deletes also invalidate) |
| `set_coedge_pcurve`, `remove_coedge_pcurve`, `set_coedge_periodic_winding` | `topology.rs` | ticks + cache bump after authority check, before `get_mut` |
| `set_pcurve`, `set_pcurve_oriented`, `remove_pcurve`, `remove_pcurve_oriented` | `topology.rs` | via `set/remove_coedge_*`; plus `build_face_loops` path when loops missing |
| `restore_preserving_handle_slots` | `topology.rs` | cache bump at entry, lineage kept, ticks restored backwards |
| `restore_for_rollback`, `restore_rollback_fields` | `topology.rs` | cache bump at entry (double bump via helper is harmless) |
| `load_journal` | `topology.rs` | cache bump before tick sync (journal-only but tick can move backwards) |
| `propagate_attributes_for_op` | `topology.rs` | via `set_face_attributes` |
| `reserve` | `topology.rs` | no bump (capacity only, no logical change) |
| `journal_begin`, `journal_record_evolution`, `journal_record_barrier` | `topology.rs` | no cache bump (journal-only, no geometry; geometry paths already bumped) |
| `undo_rewind_scope` (transaction rollback) | `undo_log.rs` | cache bump at entry, ticks restored backwards |
| `undo_truncate_for_foreign_restore` | `undo_log.rs` | cache bump at entry (callers bump again; harmless) |
| `Face::set_surface`, `set_reversed`, `compose_orientation`, `Edge::set_trim`, `set_curve`, `Vertex::set_point`, etc. | `face.rs`, `edge.rs`, `vertex.rs`, `wire.rs`, `shell.rs`, `solid.rs` | covered by `*_mut` acquisition bump before the `&mut` escapes; `Shell::faces_mut`, `Wire::edges_mut`, `Face::inner_wires_mut` mutate only through an already-bumped `&mut` owner |
| Direct `Arena::alloc`/`get_mut`/`retire` | `arena.rs` | unreachable externally (arenas private); in-crate uses route through the bumped `Topology` wrappers above |
| `builder.rs` constructors | `builder.rs` | via `add_*` |
| Undo record helpers (`record_*`, `record_alloc`, `record_pcurve_write`, attribute/journal records) | `undo_log.rs` | no independent bump; they run inside already-bumped callers before writes land |

Face reversal, edge domains (trims), loops/coedges, pcurves, and periodic
windings are all covered: they either go through `*_mut` (bump before
escape) or the dedicated `set_coedge_*`/`install_*` paths above.

## 3. Conservative whole-topology choice

The first slice invalidates the whole topology on any mutation. A needless
rebuild is acceptable; stale reuse is not. Per-face dependency tracking,
automatic mesh invalidation, per-face property caching, and arena
compaction are explicitly out of scope for this assignment.

## 4. Cache keys include all relevant settings

`ClassificationCache` keys by `(lineage, generation, solid_index,
tolerance_bits, max_recovery_attempts)`. Preparation itself is
tolerance-independent, so this duplicates preparation across tolerances;
the 16-entry FIFO bound keeps that duplication finite and deterministic.
Future tolerance-dependent preparation cannot alias across options.
