# Topology-evolution contracts for direct editing

Status: characterized baseline. This document states what the kernel
actually guarantees about source-to-result mappings when operations
preserve, modify, delete, split, or merge faces and edges — no more.
It draws a hard line between **operation-local evolution** (journal
entries in one session) and **persistent naming** (references that
survive recomputation or file round trips). For the persistent-naming
design, see `rfc-0003-persistent-naming.md`; for the WASM face-evolution
payload history, see `../wasm-face-evolution.md`.

Scope: shared contracts only. `resize_blend.rs`, `defeature.rs`, and
their fixture tests are intentionally out of scope (owned by parallel
fillet-removal work).

## 1. Which entity types are tracked

The journal tracks exactly three entity kinds — vertex, edge, face
(`crates/topology/src/journal.rs:102-111`). Shells, solids, wires,
loops, and coedges are never journal subjects. Identity rides on
`EntityKey { kind, index }` (`journal.rs:125-135`), interned per entry
into `JournalOrdinal`s that are never reused (`journal.rs:85-100`).

The legacy operations-layer map is faces-only: `EvolutionMap` carries
`modified / generated / deleted / unresolved` keyed by face arena index
(`crates/operations/src/evolution.rs:111-127`). Its ingestion into the
journal (`record_face_evolution`, `crates/operations/src/journal_ops.rs:262-269`)
produces a faces-only entry: edges and vertices sever across it by
design, while other solids' entities carry through. A faces-only entry
from a blend therefore leaves that solid's edge and vertex references
unresolvable — fail closed, never implicit preservation
(`journal.rs:40-46`).

Pinned by: `journaled_boolean_records_total_construction_history`,
`chamfer_journaled_severs_edge_refs_like_any_faces_only_entry`.

## 2. How outcomes are represented

One event per subject; duplicates are refused
(`TopologyError::JournalDuplicateEvent`, refused at `journal.rs:619`
and in the snapshot path at `:945`).
Subjects of `Preserved / Modified / Generated / Merged / Unresolved`
are *result* entities; a `Deleted` subject is an *input* entity that
ceased to exist (`journal.rs:201-207`):

| Event | Meaning |
|---|---|
| `Preserved { from }` | Subject IS `from`, carried through unchanged (e.g. imprint tool entities, `crates/operations/src/imprint.rs:80-85`). |
| `Modified { from }` | Subject is a modified piece of `from`. A split is several `Modified` subjects sharing one `from` (`journal.rs:215-220`). |
| `Generated { sources }` | Subject is new geometry built from `sources` (possibly cross-kind: a section edge names its generating faces; empty when synthesised from nothing nameable). Adjacency, not identity — lineage never follows it (`naming.rs:31-33`). |
| `Merged { from }` | Many inputs flowed into one output (same-domain merge). References to any input resolve to the subject. |
| `Deleted` | The subject (an input) was deleted. |
| `Unresolved { candidates }` | Origin could not be established; candidates are inseparable inputs (empty if none was plausible). The resolver fails closed here. |

Every entry carries a **scope**: entities the operation may have
touched. Out-of-scope entities carry through; in-scope entities without
a claim are severed (`journal.rs:35-53`). Scope auto-augments to a
superset of every mentioned entity (`journal.rs:605-614`).

## 3. Splits and merges

Both cardinalities are first-class, and the resolver distinguishes them:

- **Split (1→N):** several `Modified` subjects share one `from`;
  resolution is `BoundMany` over all pieces
  (`naming.rs:23-25,184-192`). Pinned geometrically by
  `imprint_preserves_tool_and_splits_target_face` (new in
  `crates/operations/tests/evolution_contracts.rs`): a pierced target
  face resolves to ≥2 patches, none of which is the consumed input.
- **Merge (N→1):** one `Merged` subject names all inputs; any input
  reference resolves to it. Ingestion groups same-output `modified`
  claims into one `Merged` (`journal_ops.rs:300-319`). Pinned
  geometrically by
  `verified_unification_journals_merged_faces_and_consumed_center`
  (heal unification, `tests/journal.rs`) and at ingestion level by
  `merged_outputs_journal_as_one_merged_event`.
- **No merge is ever invented.** Boolean fuse keeps coplanar patches as
  separate `Modified` faces (10 faces for two adjacent 4-cubes, not 6):
  pinned by `adjacent_fuse_keeps_coplanar_patches_separate`. A future
  same-domain merge must arrive as a producer claim; the resolver will
  not infer one. This protects the `Bound` vs `BoundMany` cardinality
  consumers branch on.

## 4. Composition across operations

There is no `compose`/`chain` API for journal entries. Composition is
implicit: each operation appends one entry, and the resolver chases
lineage forward through every subsequent entry (`naming.rs:419-426`,
via `chase_one_entry` at `:469-558`).
Identity flows only through `Preserved / Modified / Merged`; crossing
a barrier, an in-scope entry with no claim, or an `Unresolved` event
that might absorb the entity stops with
`UnresolvedAcrossOperation` naming the operation (`naming.rs:18-22`).

Pinned by `lineage_composes_across_move_then_replace` (planar pull
followed by exact support replacement resolves one pre-edit reference
to one live face with `Construction` provenance throughout) and by
`successive_draft_preserves_all_original_entity_references`
(draft∘draft across scales and rotations, `tests/journal.rs`).

Two related limits:

- Healing folds per-step histories into one entry before recording
  (`compose_healing_history`, `journal_ops.rs:838-883`); that is
  pipeline-internal, not a general chaining API.
- Per-feature fillet fallbacks that cannot compose histories return a
  `Geometry` map instead (`blend_ops.rs:1462-1469`) — see §5.

## 5. What "exact" means

Two separate senses, never conflated:

- **Exact geometry** (analytic, no approximation): e.g. arena documents
  replay byte-identical f64 values (`crates/io/src/arena_io.rs:1-21`);
  `shell` with `approximation: None` is exact-only
  (`shell_op.rs:211-229`).
- **Exact mapping** (construction-recorded provenance vs geometric
  inference): `EvolutionOrigin::Construction` is "what the operation
  itself recorded while building the result. Exact."
  (`evolution.rs:39-55`); `Geometry` is "an inference that can be
  wrong even when it reports no ambiguity". The journal mirrors this
  as `RecordedOrigin::{Construction, Geometry}` (`journal.rs:176-199`),
  and the chase marks provenance `Inferred` after hopping a geometry
  entry (`naming.rs:547-549`). `FaceEvolutionPayloadV1` deliberately
  encodes non-exact maps as `unavailable` with everything unresolved
  (`crates/wasm/src/types.rs:278-298`).

"Exact" never means "complete across entity kinds": a faces-only exact
entry still severs edges and vertices (`journal_ops.rs:231-237`).

## 6. Rollback and stale handles

- `run_transacted` clones, runs, and on `Err` restores topology *and*
  journal together, returning the error unchanged
  (`topology/src/transaction.rs:40-51`); all `*_journaled` wrappers
  share one transaction for geometry + recording (e.g.
  `boolean_journaled` at `journal_ops.rs:133`, `fillet_journaled` at
  `:405`), so a recording refusal restores unpublished history too.
- Arena slots and journal ids (`OpId`, ordinals) are never reused,
  including across restores — rolled-back ids stay dead, so a
  reference to a rolled-back operation dangles as `UnknownOperation`
  rather than rebinding (`journal.rs:59-64,677-687`;
  `topology.rs:421-453`). Stale handles return typed `*NotFound`
  errors, never aliases (`arena.rs:264-300`).
- Checkpoint `restore` is barrier-semantics, not rollback:
  post-snapshot retirements stay retired, journal truncates with ids
  preserved (`topology.rs:305-348`).
- Every allocation/mutation bumps `mutation_ticks`; the next
  `journal_begin` inserts a synthetic `GlobalBarrier` for anything the
  journal was not told about (`journal.rs:19-31`;
  `topology.rs:587-597`). Pinned by
  `unjournaled_operation_surfaces_as_a_global_barrier` and, for the
  failure path, by `failed_imprint_rolls_back_topology_and_journal`
  (new): a refused imprint publishes no entry, changes no topology,
  keeps pre-existing handles live, and leaves no gap for the next
  entry.

## 7. What survives serialization and reimport

Arena handles and STEP numbers are not persistent IDs:

- **Arena documents** (`serializeSolids` / `deserializeSolids`) remap
  to dense local indices; deserialization always allocates fresh ids
  (`arena_io.rs:17-21`). The journal is an additive optional section
  (`SerJournal`, `arena_io.rs:323-332,1576-1645`); ticks are re-derived
  so a clean load is not a gap (`journal.rs:759-763`). Entities absent
  from the document restore as `UNMAPPED` placeholders that fail typed
  lookups instead of aliasing (`arena_io.rs:2486-2533`). Session state,
  retired slots, assemblies, sketches, and checkpoints are excluded.
- **`PersistentRef` is a value object** holding no arena ids
  (`naming_io.rs:1-11`); it resolves "in any session holding the
  model's journal" — i.e. after an arena-document round trip, not
  after STEP.
- **STEP carries geometry, not history.** The writers take only
  `(topo, solids)` — there is no journal channel in the export API
  (`step/writer.rs:72-134`) — and the reader performs no
  `journal_begin`/`journal_record` calls, so imports arrive with an
  empty journal. Display names round-trip onto attributes
  (`reader.rs:1881-1890`, `writer.rs:1413-1423`); history does not.
  Pinned by `step_round_trip_severs_persistent_naming_but_keeps_geometry`
  (new): edited geometry survives with equal volume while the same
  reference value resolves `UnknownOperation` in the fresh session.
- **Signatures are recovery-only.** The `EntitySignature` tier is
  inference-tier, always `Provenance::Inferred`, multi-match →
  `Ambiguous`, never first-match (`naming.rs:612-631`); WASM surfaces
  it as `captureSignatureRef`, tested to report `"inferred"`.

## 8. WASM boundary contracts

- Two evolution surfaces with different completeness (consumers must
  not assume a uniform shape):
  - Issue-12 boolean JSON (`fuse/cut/intersectWithEntityEvolution`):
    `faces[]` with nullable `source`, `edges[]` / `vertices[]` with
    event strings. **No deleted or merged bucket**: a consumed input
    is simply absent — unknown, never implicitly surviving. Pinned by
    `entity_evolution_face_indices_are_live_in_the_result` (new):
    every reported face is live on the result solid, every named
    source is a pre-operation operand face.
  - `FaceEvolutionPayloadV1` (fillet/chamfer with evolution): carries
    `deleted` and decodes with domain/duplicate/disjointness checks
    (`types.rs:329-454`).
- `journalSummary` is counts-only (`events: <len>`,
  `affected: <len>`); it cannot enumerate severed identities — resolve
  each reference individually (`bindings/evolution.rs:502-531`).
- Handles are session-local `u32`s over arena indices
  (`handles.rs:103-140`); deserialized entities receive fresh handles
  while old handles stay live, so consumers must adopt the returned
  arrays. Overflow saturates to `u32::MAX` at the evolution projection
  (`bindings/evolution.rs:20-22`) while op conversion errors
  (`bindings/naming.rs:52-56`): unreachable in practice (4B live
  entities), documented here so a future cleanup has the full picture.
- `fuseWithEntityEvolution` and friends return in-band lineage but
  journal nothing (`boolean/mod.rs` contains no journal calls): like
  any unjournaled mutation, they surface as a `GlobalBarrier` at the
  next journaled operation (§6). Journaled history requires the
  `*Journaled` entry points (`fuseJournaled`, `moveFacesJournaled`,
  …), whose returned `op` feeds `resolveOperationOutput` — pinned by
  `journaled_move_op_resolves_through_naming_and_summary` (new) and
  `move_faces_journaled_preserves_all_entity_refs_in_direct_and_batch_calls`.

## 9. Explicit limitations for downstream selections and replay

1. Treat `Dangling` **and** `UnresolvedAcrossOperation` as terminal.
   A contested deletion — an explicit `Deleted` subject plus an
   `Unresolved` candidacy naming the same input (the shell rim names
   the opened face, `shell_opening_deletes_exactly_the_opened_face`)
   — severs instead of reporting `Dangling`. Both spellings refuse to
   rebind; only the diagnostic differs.
2. The boolean paths spell deletion differently by construction: the
   faithful map synthesizes `deleted` (`boolean/mod.rs:2189-2195`)
   while the journaled entry scope-severs consumed inputs (the GFA
   records no face deletions, `journal_ops.rs:134-137`). Pinned by
   `cut_consumed_faces_are_deleted_in_map_and_severed_in_journal`.
   Never treat "absent from the payload" as "surviving".
3. Faces-only entries sever edge/vertex references across the
   operation. Rebind those only through entries with explicit
   edge/vertex claims (boolean, imprint, planar/coaxial moves,
   replace-surface) — never by proximity, traversal order, or surface
   matching. Nearest-face guessing is not an exact mapping and must
   not be introduced as one.
4. Feature replay across recomputation or STEP/mesh import must
   re-anchor: replay the operation sequence (operation-local
   evolution) or re-derive selections; signature refs are inferred
   recovery, not identity.
5. Do not persist arena indices, `u32` handles, or STEP entity numbers
   as stable IDs. Persist `PersistentRef` values (arena-document
   channel) or semantic attributes (STEP-name channel), and expect
   `UnknownOperation` / `NoMatch` after any history break.

## 10. Follow-up proposals (not this change)

Documented here so they are not lost; each needs its own design and
is explicitly out of scope for this audit:

- **Unify the boolean deletion spelling.** Either synthesize `Deleted`
  events for consumed inputs in `boolean_journaled` (set-difference of
  pre-scope minus claimed sources — an inference wearing a
  `Construction` badge, so it needs care), or drop the synthesized
  `deleted` from the faithful map in favor of documented severing.
  Requires GFA-side provenance work; touches boolean behavior.
- **Rim-as-`Generated` for shell openings.** Recording the new rim as
  `Generated { sources: [opened] }` instead of `Unresolved` would let
  the explicit `Deleted` claim report `Dangling`. Changes resolution
  outcomes for existing consumers — needs a migration note, not a
  drive-by fix.
- **Persistent naming across STEP.** Would require a journal channel
  in the STEP writer/reader (or a sidecar), plus identity policy for
  merge/split across applications. Large; the arena-document channel
  is the supported path today.
