---
row: B287
---
Tier: core (edits live solid handles, writes the journal). Native
`remus_offset::move_faces_multi_with_entity_map` (`offset/src/move_faces/multi.rs`)
offsets every selected planar support by its own signed distance (one for all
faces, or one per face; 1 to 64 faces) and rebuilds each edge and vertex once
from its final supports through the coplanar move's own construction
(`rebuild_planar_moves`), with per-face shifts in place of the one group
translation; edges shared by two moved faces are re-intersected from both
planes. Opposite-normal selections are allowed (each face moves along its own
outward normal); a distance at which edges collapse or turn around refuses
`TopologyChange` naming a selected face on that edge. A coplanar group moved
by one distance runs the coplanar move unchanged, bit for bit. Operations:
`multi_move::{move_faces_multi, move_faces_multi_with_evolution,
move_faces_multi_journaled}` add per-face swept-region refusals against fixed
faces and strict validation, with an exact total face map and edge/vertex
history in one `move_faces` entry; `PreparedFaceMove::prepare_multi` (weights
per face) gives the same answers across restores and refuses a stale source.
WASM: `moveFacesMulti`, `moveFacesMultiJournaled`, `moveFacesMultiDetailed`
(direct and batch), `prepareMoveFacesMulti` used with `moveFacesPrepared*` and
`releasePreparedMove`. Witnesses: `operations/tests/move_faces_multi.rs`
(stepped tops ±1.5 and ±5 past the 4-unit step, pocket walls in 1 to 4 and out
1 to 6, boss walls +2, +5, −2, −4, each against chained single moves in
volume, F/E/V, sorted vertices and edge lengths, journal map, prepared
preview, strict validation, watertight mesh; collapse refusals at 5, 6, 12),
`offset/tests/move_faces_multi.rs`, `wasm/src/bindings/move_faces_multi/tests.rs`.
Issue #1889.
