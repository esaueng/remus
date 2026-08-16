# RFC 0003: Persistent topological naming

Status: accepted design; Stage 1 (journal) implemented — see "Staging" and
"Stage 1 implementation notes". Stages 2–5 remain design.

## Problem

Arena handles (`FaceId`, `EdgeId`, …) identify entities within one topology
for one session. They are deliberately not persistent names: a regeneration,
a boolean, a copy-compaction (`deferred-e6b`), or a STEP round trip produces
new handles for what the user considers "the same face." Anything that
stores a selection — an application's fillet-on-this-edge, a color, a
constraint — needs a reference that survives model edits, and the kernel
program's Milestone 5 requires it to survive, when unambiguous:
regeneration, boolean splitting, fillets/chamfers, patterns, copy
compaction, STEP round trips, and direct face edits.

The foundation now exists: construction-derived face provenance
(`operations::evolution`, GFA face origins) and, since Issue 12,
construction-derived edge and vertex events
(`gfa::boolean_with_entity_evolution`). This RFC designs the reference
model that rides on that data.

## Principles

Inherited from the evolution discipline (`operations/src/evolution.rs`) and
binding here:

1. **Wrong is worse than none.** A reference must never silently rebind to
   the wrong entity. Every resolution failure mode is a typed, reportable
   state.
2. **Construction beats inference.** Resolution follows construction
   lineage wherever records exist; geometric/adjacency signatures are a
   declared *inference* tier, always marked as such, never silently mixed
   with construction facts.
3. **Fail closed on ambiguity.** When several entities satisfy a reference
   equally, resolution returns the candidate set — it does not pick one.
4. **Determinism.** Identical model history plus identical reference
   resolves identically, native and WASM.
5. **The kernel names mechanisms, the application names meanings.** The
   kernel has no feature tree. It provides operation-scoped evolution
   records and the resolver; *which* operation is "Pocket 3" is the
   application's vocabulary, carried through the attribute store
   (Issue 14).

## Reference model

A persistent reference is an **anchor** plus optional **discriminators**,
under a versioned schema:

```text
PersistentRef v1 {
    anchor:          Anchor,
    discriminators:  [Discriminator],   // applied in order, fail-closed
    entity_kind:     Face | Edge | Vertex,
}

Anchor =
  | OperationOutput { operation: OpId, index: OutputOrdinal }
      // "the k-th face this operation generated", in the operation's own
      // deterministic output ordering. Exact; construction-derived.
  | LineageOf { base: Box<PersistentRef> }
      // "whatever the entity referenced by `base` evolved into", chased
      // through the journal (below).
  | Signature { sig: EntitySignature }
      // Inference tier: typed geometric + adjacency signature. Marked
      // inferred at every resolution.

Discriminator =
  | SurfaceType(tag) | CurveType(tag)
  | AdjacentToRef(PersistentRef)
  | NearestTo(quantized point, tolerance)   // inference tier
  | OutputRole(role)                         // e.g. blend band, cap, wall —
                                             // roles an operation declares
```

`OpId` is an application-scoped identifier for one operation *invocation*
(the batch already has a deterministic operation index; native callers get
a monotonically issued id from the journal). It is not an arena handle and
never reused.

### The evolution journal

Persistent resolution requires history, not just the final state. The
journal is an append-only, per-topology log:

```text
JournalEntry {
    op: OpId,
    kind: operation name (stable string),
    faces:    per-entity events (from the operation's evolution map),
    edges:    per-entity events (Issue 12),
    vertices: per-entity events (Issue 12),
    origin:   Construction | Geometry   (per map, as today),
}
```

Events reuse the landed vocabulary: preserved, modified, generated (with
sources), split (a modified event with multiple outputs), merged (multiple
inputs, one output), deleted, unresolved. Entries store entity identity as
*journal-local* stable ordinals mapped to current arena ids in a live
index, so compaction or restore rewrites only the index, never the journal
(this is what makes the journal the persistent spine while arena ids stay
session-local).

Operations that produce no evolution data today (offset, shell, draft,
split, defeature, direct edits — the stability matrix's declared gaps)
journal a single **barrier** entry: every entity of the affected solid is
`unresolved` across it. A reference chased through a barrier fails closed
with `UnresolvedAcrossOperation { op }` rather than pretending continuity —
coverage grows operation by operation, and the failure names the operation
whose records are missing.

### Resolution

```text
resolve(ref) ->
  | Bound(entities, provenance: Construction | Inferred)
  | BoundMany(entities, provenance)       // a split target: all pieces
  | Ambiguous { candidates, reason }      // fail closed; caller narrows
  | Dangling { deleted_at: OpId }         // target was deleted
  | UnresolvedAcrossOperation { op }      // journal gap (barrier / unresolved)
```

Rules:

- **Split**: a reference to an entity that later splits resolves
  `BoundMany` over all pieces, ordered by journal record order
  (deterministic). Callers wanting one piece add discriminators.
- **Merge**: references to any pre-merge input resolve to the merged
  entity; the provenance stays `Construction` when the merge was recorded.
- **Preserved/modified**: follow the chain; `modified` keeps binding (that
  is the claim the evolution discipline reserves for carried-forward
  entities).
- **Signatures** resolve against the *current* model only, tolerance-aware
  (quantization derives from `OperationContext.tolerance`, never raw float
  equality), and always return `Inferred` provenance. A signature that
  matches more than one entity is `Ambiguous` — never first-match.
- Discriminators filter candidate sets in order; if a discriminator empties
  the set, resolution reports which one did (diagnosable, not just "not
  found").

New diagnostic codes (registry-additive, categories per the taxonomy):
`ref_ambiguous` (`invalid_input`), `ref_dangling` (`invalid_input`),
`ref_unresolved_across_operation` (`unsupported` — the operation's records
are a declared capability gap).

### Signatures (inference tier)

`EntitySignature` v1, deliberately small and typed:

- entity kind + surface/curve type tag;
- quantized analytic parameters (e.g. cylinder radius, plane normal) using
  tolerance-derived quantization;
- adjacency counts (faces per edge, edges per face-loop);
- for edges: endpoint-vertex signature references (structural, not
  positional) when available.

Signatures are for *recovery* (imported models with no journal; journal
gaps the caller accepts) — never the primary path. Everything about them is
marked `Inferred`, and consumers such as the attribute store must decide
per policy whether inferred rebinding is acceptable.

## Surviving specific events

- **Regeneration** (same operations replayed): `OperationOutput` anchors
  are replay-stable because operation output ordering is deterministic (a
  kernel postcondition since the operation contract).
- **Boolean splitting**: `BoundMany` via the GFA's construction events
  (faces today; edges/vertices per Issue 12).
- **Fillet/chamfer**: the v2 blend evolution payloads already enumerate
  complete face domains; the journal ingests them directly.
- **Patterns**: pattern evolution (construction-derived) gives per-instance
  generated records; `OperationOutput{op, ordinal}` addresses instances.
- **Copy compaction** (`deferred-e6b`): compaction returns explicit
  remaps; only the journal's live index is rewritten. References
  themselves contain no arena ids, so they survive untouched by design.
- **Checkpoint restore**: entries after the checkpoint are truncated with
  the restore (journal and topology stay consistent because both roll back
  together under the transaction/restore machinery).
- **STEP round trip**: STEP has no native persistent identity. Issue 14 uses
  representation-item name fields for user-visible semantic names, so a
  reference key must never overwrite or masquerade as that name. Persistent
  references require a separately namespaced property or external-identification
  encoding whose entity references target the same representation items.
  Defining and implementing that encoding is part of stage 5. Until then, STEP
  round trips preserve semantic names but import with an empty journal;
  signature anchors are the only (inferred) recovery, which is exactly what
  the provenance marking is for.
- **Direct face edits**: direct-modeling operations must emit construction
  evolution (their capability gate includes it); until an operation does,
  it journals a barrier and references across it fail closed.

## Explicit non-goals (v1)

- No feature tree, no rollback/reorder semantics — application concerns.
- No cross-document identity (a ref is scoped to one model's journal).
- No geometric best-effort auto-repair of dangling refs.
- Generational arena handles remain a separate, orthogonal design
  (`deferred-e6b`); nothing here changes handle semantics.

## Staging

1. **Journal** — append-only log + live index, populated first by the
   operations that already produce construction evolution (booleans via
   Issue 12, v2 blends, patterns); barrier entries for the rest.
   **Implemented** (`brepkit_topology::journal` +
   `brepkit_operations::journal_ops`); see the implementation notes below.
2. **Resolver** — `PersistentRef` v1 with `OperationOutput` and
   `LineageOf` anchors over the journal; typed resolution results and the
   three diagnostic codes; determinism tests native/WASM.
3. **Signature tier** — `EntitySignature` v1 with `Inferred` provenance
   and ambiguity semantics.
4. **Attribute store integration** (Issue 14) — attributes keyed by
   references, with per-event propagation driven by the journal. Semantic
   names continue to use STEP representation-item name fields.
5. **Serialization** — journal + refs in the native format (versioned,
   additive), a separately namespaced STEP persistent-reference encoding,
   and repro-bundle support so a naming regression is a replayable fixture.

Each stage lands with the standard gates; stage 1's exit is that every
operation either journals real evolution or an explicit barrier — no
operation is silently absent from history.

## Stage 1 implementation notes

Refinements the implementation added to the design above:

- **The exit gate is structural, not caller discipline.** Recording is
  caller-driven (`journal_begin` → run the operation →
  `journal_record_evolution` / `journal_record_barrier`), but a caller
  cannot create a silent gap: `Topology` counts every mutation
  (allocations, exclusive accesses, retires, pcurve changes — deliberately
  conservative, since a false gap fails closed while a missed mutation
  would fake continuity), each entry records the count at completion, and
  `journal_begin` compares. Any unaccounted mutation — an unjournaled
  operation, a failed operation's partial work, a direct edit — inserts a
  synthetic **global barrier** (`unjournaled_mutations`) that severs
  continuity for every entity. "Not silently absent" is therefore
  guaranteed even for operations that never heard of the journal;
  *usefully present* (real evolution instead of a barrier) grows
  operation by operation, as designed.
- **Identifier no-reuse extends the arena discipline.** A checkpoint
  restore truncates entries and the live index to the snapshot (journal
  and model roll back together, so a clean rollback is not a gap) while
  `OpId` and ordinal counters are high-water preserved — an identifier
  issued by a rolled-back operation dangles forever rather than ever
  rebinding. A plain clone-assign restore does not preserve counters;
  persistent-reference safety requires `restore_preserving_handle_slots`,
  exactly as raw-handle safety already did.
- **Events are one flat list per entry, ordinals carry kind.** Ordinals
  live in one shared space (`EntityKey` = kind + arena index interns to
  one ordinal), so cross-kind sources — a section edge generated by two
  faces — need no special casing. One event per subject per entry;
  duplicate claims are refused whole (`journal_duplicate_event`,
  `invalid_input`) rather than letting a resolver pick.
- **Partial entries claim exactly what they claim.** A faces-only entry
  (an `EvolutionMap` from a v2 blend) leaves edge and vertex references
  unresolvable across that operation — absent claims are gaps, never
  implicit preservation. The `EvolutionMap` ingestion also normalizes the
  map's merge encoding (one output under several inputs' `modified`
  lists) into a first-class `Merged` event naming every input.
- **Journaled booleans are the exact path only.** `boolean_journaled`
  wraps `gfa::boolean_with_entity_evolution`; a mesh fallback has no
  construction records to journal, so a caller accepting approximate
  results journals that operation as a barrier instead.
- **Not yet serialized.** The journal lives only in memory; the native
  format and repro bundles ignore it until Stage 5. Attribute-store writes
  do not count as mutations (they never change which entity an entity
  is), so attaching names and colors does not sever continuity.

## Resolved questions

- References are **value objects** (serializable, hashable), not handles;
  nothing about them dangles when arenas change.
- Set-valued resolution (`BoundMany`) is first-class rather than an error:
  splitting is normal modeling, and forcing pre-emptive disambiguation on
  every caller would push them to fragile positional picks.
- Barrier-on-missing-records was chosen over silently skipping journal
  entries: a gap that looks like continuity is the silent-wrong-binding
  failure this design exists to prevent.
- The journal records events by journal-local ordinals, not arena indices,
  so `deferred-e6b` compaction and checkpoint restore need no journal
  rewrite.
