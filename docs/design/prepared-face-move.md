# Prepared face moves

Owner: [B30](../kernel-maturity/roadmap.md#b30) (profile-selected face-drag
latency). Tier: core — a prepared move is a session handle that edits the
live topology and writes the journal on every call.

## Problem

An interactive face drag restores the same checkpoint every frame and moves
the same face selection to a new source-relative distance
(`restore` → `moveFacesJournaled` → validation, display mesh, measurements).
Every frame repeated work that depends only on the source: the selection's
blend classification, the blend-aware translation's region, carriers and
boundary classification, the swept-region boxes, the prismatic volume
oracle's source readings, the journal scope, and — on bodies whose
remove/rebuild attempt refuses on a support it cannot rebuild — that whole
attempt and its rollback.

## API

```text
prepareMoveFaces(solid, faces) -> handle
moveFacesPrepared(handle, distance) -> solid            // = moveFaces
moveFacesPreparedJournaled(handle, distance) -> "{solid, op}"  // = moveFacesJournaled
releasePreparedMove(handle) -> bool
```

Native: `remus_operations::prepared_move::PreparedFaceMove`
(`prepare`, `move_faces`, `move_faces_journaled`, `is_current`).
Preparation reads the topology only. A kernel holds at most 16 live prepared
moves; handles are never reused.

## Contract

- **Same answers.** A prepared call runs the distance-dependent rest of the
  very code the unprepared move runs, so for the same source content,
  selection and distance it returns the same result content and history, or
  the same refusal text, under the same strict postconditions (volume
  oracles, strict validation, total face maps). Result handles may differ:
  the unprepared move allocates and rolls back entities the prepared one
  never creates.
- **Learned refusal.** A blend move's remove/rebuild attempt that refuses
  before reaching the distance refuses every distance alike; the first call
  records its message and later calls go straight to the translation, whose
  combined refusal text is unchanged.
- **Staleness.** Preparation records the source's complete content key
  (every entity, handle and real a move reads — the volume memo's key). Each
  call recomputes it and refuses with an `invalid input: prepared face move
  is stale …` error, editing nothing, when it differs. A `restore` that
  brings the same content back under the same handles keeps the preparation
  valid, although it advances the topology's cache generation.
- **Uncovered selections** (a bore move, an empty or invalid selection) run
  the unprepared move on every call.

### Why an explicit handle, not a transparent memo

Both must prove, every call, that the source is the prepared one; the cache
generation moves on every restore, so only a content comparison can, and it
costs the same either way. The explicit handle adds what a hidden memo
cannot: an owned lifetime (released with the drag, bounded per kernel), no
memory held for one-off moves, and an unchanged default `moveFacesJournaled`
for every other caller.

## Measured effect and what remains

Native, per frame, on the drag loop in `crates/wasm/examples/face_drag_loop.rs`
(numbers in the PR): the prepared hammer-holder move skips the refused
remove/rebuild attempt, the seed and region classification and the scope
walk; the 0.3 ms content check is the price of the staleness contract. What
remains per frame is distance-dependent: the result's copy or rebuild and its
strict postconditions. On the hammer holder the two strict validators are
about 8 ms native, and ~3 ms of that is the orientation probe re-integrating
the ~5 trimmed cylinder carriers whose trims the move changes; on a filleted
box ~70% of the move is the fillet postcondition's whole-body mesh volume of
the rebuilt body. A boundary reduction of the analytic volume terms (as the
face areas now have) is the next lever for both.

## Adoption (OpenZCAD)

At drag start, after the checkpoint: `prepareMoveFaces(source, faces)`.
Per frame: `restore(checkpoint)`, then
`moveFacesPreparedJournaled(handle, distance)` in place of
`moveFacesJournaled(source, faces, distance)` — same JSON, same refusals.
On a stale refusal (the source was edited), prepare again. On drag end or
cancel: `releasePreparedMove(handle)`. Distances stay source-relative, as
the [face-drag preview contract](face-drag-preview-contract.md) requires.
