# RFC 0003: Persistent topological naming

Status: implemented — all five stages (journal, resolver, signature
tier, attribute integration, serialization) plus the WASM reference API
(`wasm/src/bindings/naming.rs`: journaled booleans, barriers,
journal-driven propagation, resolution, face names, and — with the `io`
feature — the serialized reference codec; resolution outcomes are data,
not errors, and every method has an `executeBatch` companion). See
"Staging" and the implementation-notes sections.

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

## Stage 4 implementation notes

Refinements the attribute-integration implementation added to the design
above:

- **Propagation is journal-driven and per-event.**
  `Topology::propagate_attributes_for_op(op, allow_inferred)` copies face
  attributes forward across one journaled operation, claim for claim:
  `Preserved`/`Modified` subjects receive their source's attributes (a
  split's pieces each keep the name unchanged — never suffixed);
  `Merged` subjects receive attributes only when every attributed input
  agrees, with disagreement counted as a conflict and left bare (a merge
  does not toss coins between names); `Generated` and `Unresolved`
  subjects receive nothing (`Unresolved` counted). Inputs keep their own
  attributes (copy-forward only). Non-face subjects are skipped — the
  attribute store's v1 scope is solids and faces; edge/vertex attributes
  remain queued in `deferred-e3b`.
- **Inference is an explicit opt-in.** A geometry-origin entry propagates
  only under `allow_inferred = true`; the refusal is reported
  (`refused_inferred`), never silent. Barrier entries carry nothing —
  they have no claims to ride. An unknown `OpId` (rolled back) is the
  typed `ref_unknown_operation` failure.
- **"Keyed by references" is composition, not re-keying.** The store
  stays arena-keyed (correct within a session, and lifecycle-integrated
  since Issue 14); the durable key is the reference:
  `naming::resolve_face_attributes` resolves a `PersistentRef` and reads
  the bound faces' attributes in one step, and every non-binding
  resolution converts to its typed `ref_*` error — an attribute can never
  be read through a dangling, severed, or ambiguous reference.
- **Propagation never severs.** Attribute writes are not model mutations
  (Stage 1 rule), so propagating between journaled operations does not
  create unjournaled-mutation gaps.
- The map-driven `operations::evolution::propagate_face_attributes`
  (Issue 14) remains for callers holding an `EvolutionMap` without a
  journal; the journal path is the general one.

## Stage 5 implementation notes

Refinements the serialization implementation added to the design above:

- **The journal rides the arena document, additively.** The v2 arena
  format gains optional `journal` and `attributes` fields, absent when
  empty — journal-less output stays byte-identical to historical output.
  Version stays 2 (the format's documented additive policy). Attributes
  travel with the document because a naming round trip that drops names
  would be hollow; both solids' and faces' entries ride by dense local
  index.
- **Ordinals are the persistent identity; arena indices never touch the
  file.** The writer remaps live-index keys to the document's dense local
  indices; the reader remaps them to the freshly allocated ids. An entity
  in journal history but outside the document keeps its *kind* with the
  `EntityKey::UNMAPPED` placeholder — anchor output ordering stays
  replay-stable, and a reference to such an entity resolves `NoMatch`
  ("not present"), never a stale index.
- **Snapshots exclude mutation ticks.** Ticks are session-local
  gap-detection state; `Journal::from_snapshot` re-derives a consistent
  sequence and `Topology::load_journal` syncs the topology's counter, so
  a clean load is not an unjournaled gap while any real post-load
  mutation still severs (pinned by test).
- **Snapshots are validated whole.** Duplicate or out-of-range ordinals,
  duplicate keys, non-increasing `OpId`s, events referencing unknown
  ordinals, or double claims refuse with the typed
  `journal_snapshot_invalid` diagnostic (`invalid_input`) — corrupt
  history is never installed for a resolver to mis-follow.
- **References serialize context-free** (`io::naming_io`, versioned JSON,
  version 1 read forever): they hold no arena ids, so a reference written
  in one session resolves in any session holding the model's journal —
  and signature references resolve in sessions holding no journal at all
  (pinned by a cross-session recovery test).
- **The replayable-fixture property is native-first.** The round-trip
  test pins that every reference resolves identically across save/load;
  WASM repro-bundle reference expectations land with the queued WASM
  reference API.

## Explicit non-goals (v1)

- No feature tree, no rollback/reorder semantics — application concerns.
- No cross-document identity (a ref is scoped to one model's journal).
- No geometric best-effort auto-repair of dangling refs.
- Generational arena handles remain a separate, orthogonal design
  (`deferred-e6b`); nothing here changes handle semantics.

## Staging

1. **Journal** — append-only log + live index, populated first by the
   operations that already produce construction evolution (booleans via
   Issue 12, v2 blends, patterns, and default V2 offsets); barrier entries
   for operations without evolution records.
   **Implemented** (`remus_topology::journal` +
   `remus_operations::journal_ops`); see the implementation notes below.
2. **Resolver** — `PersistentRef` v1 with `OperationOutput` and
   `LineageOf` anchors over the journal; typed resolution results and the
   three diagnostic codes; determinism tests native/WASM.
   **Implemented** (`remus_topology::naming`); see the Stage 2
   implementation notes below.
3. **Signature tier** — `EntitySignature` v1 with `Inferred` provenance
   and ambiguity semantics.
   **Implemented** (`Anchor::Signature`); see the Stage 3 implementation
   notes below.
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
- **Default V2 offsets carry construction identity.** The intersection-joint
  assembler returns the exact one-to-one source-face map; `offset_journaled`
  records it transactionally as `Construction` evolution. Arc-joint and
  self-intersection-removal variants may add or replace faces after that step,
  so the face-map entry point refuses them until those generated/replaced-face
  records exist.
- **Not yet serialized.** The journal lives only in memory; the native
  format and repro bundles ignore it until Stage 5. Attribute-store writes
  do not count as mutations (they never change which entity an entity
  is), so attaching names and colors does not sever continuity.

## Stage 2 implementation notes

Refinements the resolver implementation added to the design above:

- **Entries gained a declared scope.** Stage 1's "entities an entry does
  not mention have no continuity across it" is unusable on multi-body
  models (any entry would sever every other solid's references), so the
  rule became scoped: every entry carries the set of entities its
  operation may have touched — captured half before the operation runs
  (`journal_ops::begin_scoped`, so consumed operand entities are covered)
  and half at record time (the result solids), always a superset of what
  the events mention. An entity **outside** the scope carries through
  unchanged; **inside** the scope it follows its claim or is severed. The
  scope is a construction claim held to the same honesty standard as the
  events: omitting a touched entity would fake continuity.
- **Resolution chases to the present.** Every anchor resolves to what the
  entity is *now*: the anchor establishes a starting ordinal, and every
  subsequent entry's claims are applied. Identity flows only through
  identity claims (`Preserved`/`Modified`/`Merged`); `Generated` is an
  adjacency claim and is never followed. A split fans the ordinal set out
  (`BoundMany`, pieces sorted); a piece's deletion ends that branch
  (survivors continue; all gone → `Dangling{deleted_at}`); an
  `unresolved` output whose candidates include the entity contests every
  other claim about it and severs (a wrong binding is worse than none);
  an entry claiming an entity both deleted and carried is a recording
  defect and severs. Barriers sever on contact, global barriers always.
- **Provenance is the meet over the chain.** Anchoring in or hopping
  through a `Geometry`-origin entry downgrades the resolution to
  `Inferred`; `Construction` survives only an all-construction chain.
- **`LineageOf` is pass-through today.** Since every anchor already
  chases to the present, `LineageOf{base}` adds only its own
  discriminators; it exists as the composition point for the Stage 3
  signature tier.
- **Two codes beyond the three designed.** `ref_unknown_operation`
  (`invalid_input`): the anchor's `OpId` is not in the journal — never
  journaled, or truncated by a rollback; because OpIds are never reissued
  the reference dangles rather than rebinding. `ref_no_match`
  (`invalid_input`): the anchor or a discriminator eliminated every
  candidate, naming which. `ref_ambiguous` is reserved for the signature
  tier (Stage 2 anchors never guess between candidates: splits are
  `BoundMany`, unresolved events sever).
- **`OperationOutput` addresses entry subjects.** Outputs are the entry's
  subjects of the reference's kind except `Deleted` ones (inputs), in the
  entry's deterministic event order; `Unresolved` subjects count (their
  existence is a journal fact even when their lineage is not).
- **Discriminators shipped: `SurfaceType`, `CurveType`.** Adjacency,
  proximity, and operation-declared roles are queued with Stage 3.
- **WASM surfacing queued.** References resolve natively; the WASM API
  lands with the operations/WASM evolution-surfacing work. Determinism is
  pinned by native double-run tests (the WASM build shares the same
  deterministic code paths).

## Stage 3 implementation notes

Refinements the signature-tier implementation added to the design above:

- **Signatures are hashable by quantization.** `EntitySignature` stores
  parameters as integer multiples of a capture quantum (the RFC-designated
  source is the operation tolerance — `EntitySignature::context_quantum`
  reads `OperationContext.tolerance.linear`), with the quantum itself kept
  as `f64` bits. Matching compares a candidate's *raw* parameters against
  the stored multiples within one quantum — tolerance-aware, never raw
  float equality, and free of the double-rounding boundary miss that
  quantize-and-compare would have.
- **Parameters per type, fixed order.** Plane: normal + signed distance
  (orientation-bearing, no sign canonicalization). Cylinder: axis
  (sign-canonicalized), axis anchor nearest the world origin (so two
  parameterizations of one cylinder sign identically), radius. Cone:
  axis, apex, half-angle. Sphere: center, radius. Torus: axis
  (canonicalized), center, radii. Circle/ellipse edges: normal
  (canonicalized), center, radii. **Line edges carry no parameters** —
  their geometry is their endpoints, and the mandatory endpoint vertex
  signatures (structural, position + incident-edge count) do the
  discriminating; NURBS and the open conics are tag-plus-adjacency only,
  which usually resolves `Ambiguous` and is meant to.
- **Adjacency counts are uses, not sets.** Faces: boundary edge uses (a
  seam edge counts twice) + wire count; edges: face boundary uses;
  vertices: incident live edge uses. Computed by one model walk at
  capture and at match, from the same helper.
- **Ambiguity semantics differ from lineage on purpose.** A signature
  matching several entities after discriminators is
  `Ambiguous {candidates, reason}` (`ref_ambiguous`) — an identity
  question with several answers — never `BoundMany` (which is one
  lineage's split pieces) and never a first-match or nearest pick.
  Discriminators filter the candidate set first, so a caller *can* narrow
  an ambiguity with a type constraint, but identical twins stay ambiguous.
- **No journal interaction.** Signature anchors resolve against the
  current model only; they are the recovery path for severed or
  journal-less references, always `Inferred`, and the integration tests
  pin exactly that flow (an edge reference severed by a faces-only blend
  entry, re-anchored by signature, answered as inference or refused as
  ambiguous — never guessed).

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
