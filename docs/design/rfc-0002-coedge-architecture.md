# RFC 0002: Coedge architecture

Status: accepted design; implementation staged as backlog Issues 6–8.
Characterization anchors: `crates/topology/src/pcurve.rs`, module
`seam_characterization` — three tests pin the current defect and state how
they must flip.

## Problem

A face boundary is an ordered `Vec<OrientedEdge>` (`wire.rs`), and p-curves
are keyed by `(EdgeId, FaceId)` (`pcurve.rs`). One 3D edge used twice by the
same face — the seam of every closed cylinder, cone, sphere, torus, and
periodic NURBS surface — therefore cannot carry per-use data:

- the second seam p-curve **silently overwrites** the first
  (`PCurveRegistry::set` is a plain map insert);
- `pcurves_for_edge` reports one use where the face has two;
- there is no identity to hang per-use trim intervals, periodic-branch
  winding, or per-use tolerance on (needed by Issue 8's explicit trims and
  SameParameter validation).

Every seam-crossing capability cell is at best Partial until this is fixed
(`docs/kernel-maturity/capability-matrix.md`, cross-family limitation 1).

## Design

Two new arena entities in `brepkit-topology`:

```rust
/// One directed use of an edge by one face boundary.
pub struct Coedge {
    /// The underlying 3D edge.
    edge: EdgeId,
    /// Traversal orientation relative to the edge's natural direction.
    forward: bool,
    /// The loop this use belongs to (owner).
    parent_loop: LoopId,
    /// This use's 2D curve in the owning face's parameter space, with its
    /// own trim interval. `None` only where the surface type does not
    /// require a p-curve (planar faces may derive it).
    pcurve: Option<PCurve>,
}
pub type CoedgeId = Id<Coedge>;

/// An ordered, closed (or open, for future sheet boundaries) cycle of
/// coedge uses bounding one face.
pub struct Loop {
    /// The owning face.
    face: FaceId,
    /// Ordered traversal. Adjacent coedges connect end-vertex to
    /// start-vertex under their orientations.
    coedges: Vec<CoedgeId>,
    closed: bool,
}
pub type LoopId = Id<Loop>;
```

Held invariants (validator-enforced, see below):

- `coedge.parent_loop` and `loop.face` are always live; a coedge belongs to
  exactly one loop, a loop to exactly one face.
- Two coedges may reference the same `EdgeId`; a seam is exactly two uses on
  one face with opposite `forward` and different p-curve branches.
- P-curve identity is the coedge. `PCurveKey {edge, face}` remains only
  inside the compatibility layer.

Deliberately **not** in this RFC: explicit 3D edge trim intervals and
winding counts (Issue 8 adds them — the coedge is where they will live);
radial edge lists for non-manifold bodies (Milestone 8); generational
handles (separately versioned per `deferred-e6b`).

### Handle semantics

`CoedgeId`/`LoopId` are ordinary arena `Id<T>`s with the existing append-only
no-reuse tombstone contract. Retiring a face retires its loops and coedges;
retiring an edge with live coedges is a validation error (the reverse index
`edge → coedges` makes this checkable). Checkpoint restore treats the new
arenas exactly like the existing ones (high-water retirement).

WASM exposure: coedges are not exposed as public numeric handles in the
first release; JS callers keep face/wire/edge handles. Exposure (for
per-use queries) is an additive binding decision after Issue 7.

## Migration

The migration never has two authoritative representations. It has one
authority and one derived view, and flips them once.

### Stage 1 — additive entities (Issue 6)

- Add the `Coedge`/`Loop` arenas, constructors, traversal
  (`loops_of_face`, `coedges_of_edge`), and validators.
- **Wires remain authoritative** for face boundaries. Derivation is
  **explicit**: `Topology::build_face_loops(face)` derives and stores the
  face's loops (one coedge per `OrientedEdge` occurrence — a seam edge gets
  two coedges naturally), retiring any previous derivation. *Refined during
  implementation:* automatic derivation at face creation was dropped for
  Stage 1 because faces are mutated in place throughout L2/L3
  (`face_mut`, `set_outer_wire`), so eager derivation would guarantee stale
  loops; automatic derivation joins the Stage 2 authority flip, where
  mutation goes through the loop representation itself.
- A consistency validator (`validate_face_loops`) asserts loop ↔ wire
  agreement; faces without a derivation pass vacuously, so it is safe in
  any validation pass today. Divergence is a bug, not a state.
- P-curves stay in the registry; each derived coedge that has a registry
  entry caches nothing yet (no dual storage of geometry).

Exit gate: every face constructed through public APIs has loops; the
seam-characterization face has **two** coedges for its seam edge; no
consumer behavior changes.

### Stage 2 — authority flip (Issue 7)

- Loops become authoritative. `Face` stores `outer_loop` + `inner_loops`;
  the wire the face was built from becomes an input artifact, not state.
- P-curves move into `Coedge.pcurve`. The `(edge, face)` registry API
  becomes a compatibility adapter:
  - `get(edge, face)`: answers only when the face has exactly **one** use
    of that edge; two uses return a typed ambiguity error
    (`invalid_topology` / `seam_pcurve_ambiguous` in the diagnostic
    registry) — the accessor fails closed instead of answering arbitrarily.
  - `set(edge, face)`: same rule; seam p-curves must be set per coedge.
  - This flips the characterization tests, which is the acceptance
    evidence for the stage.
- Compatibility adapters for readers:
  - `face_oriented_edges(topo, face) -> impl Iterator<Item = OrientedEdge>`
    derived from the loop (cheap: each coedge yields `(edge, forward)`);
  - `Face::outer_wire()` remains during the stage, backed by a wire
    materialized from the loop at mutation time, so `&[OrientedEdge]`
    slice-consumers keep compiling.
- Free wires (sweep paths, profiles, wire bodies) are untouched: `Wire`
  remains the representation for wires that are not face boundaries.

Exit gate: the seam face round-trips two independent p-curve branches;
`solid_faces`-based consumers pass unchanged through the adapters; the GFA
boolean suite, blend builders, and tessellation pass on loop-backed faces.

### Stage 3 — trims and SameParameter (Issue 8)

Explicit 3D trim intervals on `Edge`, per-use p-curve trims and periodic
winding on `Coedge`, and SameParameter/SameRange validators with a
non-silent repair operation. Specified separately; this RFC only reserves
the fields' home.

### Migration ratchet

Once Stage 2 lands, new code must not construct face boundaries from raw
`OrientedEdge` lists. Enforcement:

- `Wire::new` stays public (free wires are legitimate); the ratchet is on
  the face constructors: the wire-taking `Face::new` becomes
  `#[deprecated]` in favor of the loop-taking constructor, and CI treats
  new deprecation warnings as errors (already implied by `-D warnings`).
- The adapter module carries a tracking comment and a deletion gate: the
  facade is removed when `rg` finds no `outer_wire()` consumers outside
  `brepkit-topology` and the deprecation has been through one release.

## Serialization

The arena/JSON BREP transfer format gains `loops` and `coedges` arrays under
a schema-version bump. Compatibility:

- Old documents (no loops): loader derives loops from wires exactly as the
  Stage 1 builder does — every legacy file remains loadable forever.
- New documents: loops are authoritative; a wire array is still written for
  old readers during a deprecation window, then dropped with a major schema
  bump.

Repro bundles (schema 1) are unaffected: they replay operations, not
serialized topology.

## STEP mapping

STEP's model already matches this design: an `EDGE_LOOP` of
`ORIENTED_EDGE`s where a seam edge legitimately appears twice, and per-use
2D geometry via `SURFACE_CURVE`/`PCURVE` associated geometry.

- **Reader** (after Stage 2): a repeated oriented edge in an edge loop maps
  to two coedges; each `PCURVE` binds to its coedge by loop position, not by
  `(edge, face)`. Today's reader collapses these — the RFC 0002 fixture
  (write/read/write of the seam face) becomes an active I/O regression at
  Stage 2, not before.
- **Writer**: emits one `ORIENTED_EDGE` per coedge and one per-use
  `PCURVE`. Deterministic entity ordering follows loop order.

## Validation additions

New structural checks (stable codes in the diagnostic registry):

| Check | Code | Category |
| --- | --- | --- |
| Coedge references retired edge/loop | `coedge_dangling_reference` | `invalid_topology` |
| Loop not connected under orientations | `loop_not_connected` | `invalid_topology` |
| Loop/wire divergence (Stage 1 only) | `loop_wire_mismatch` | `internal` |
| Seam uses without distinct p-curve branches (Stage 2+) | `seam_branch_missing` | `invalid_topology` |
| `(edge, face)` p-curve access on a seam (Stage 2+) | `seam_pcurve_ambiguous` | `invalid_topology` |

## Consequences

- **Cost**: `FaceSurface`/`EdgeCurve`-scale ripple. Face-boundary iteration
  appears throughout L2/L3; the adapters exist precisely so Stage 2 is a
  flip of authority, not a big-bang rewrite of ~100 consumers. Consumers
  migrate to loop traversal incrementally after the flip.
- **Memory**: one `Coedge` per boundary use (~40 bytes + p-curve). P-curves
  move rather than duplicate.
- **Unblocks**: periodic seams (booleans, blends, offsets on closed
  surfaces), Issue 8 trims/SameParameter, faithful STEP seam round-trips,
  and per-use evolution events in Milestone 5.

## Resolved questions

- One loop entity for both outer and inner boundaries (a face has one outer
  loop and zero-or-more inner loops); no separate hole type.
- Coedge stores `parent_loop`, loop stores `face` — reverse lookups are one
  hop, and a coedge cannot be shared between loops by construction.
- No `mate`/`partner` pointer between the two uses of a seam edge in v1;
  `coedges_of_edge` answers the query, and a stored pointer is one more
  invariant to break. Revisit only if profiling shows the lookup hot.
- Free wires keep `Wire`. Only face boundaries migrate.
