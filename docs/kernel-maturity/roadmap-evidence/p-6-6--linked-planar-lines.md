---
row: p-6-6
---

Tier: core: linked sketch inputs consume live persistent references and update
checkpointed GCS entity handles. This bounded slice attaches a sketch to one
construction-resolved planar face and projects construction-resolved line edges
into fixed external GCS lines. Explicit start/end vertex construction references
preserve endpoint constraints across reversed edge storage; endpoint mismatch
refuses. Refresh is atomic and read-only on topology;
parallel plane moves retain the original in-plane frame. The explicit sketch frame is independent of the face normal sense.
Rotation, split face
or edge references, inferred/ambiguous/deleted references and other curve types
refuse explicitly. Existing GCS constraints consume the returned line and point
handles. Direct and batch APIs share the same implementation.

Full 6.6 remains open: circle/ellipse references, split-face plane continuity,
rotated attachments, portable complete sketch documents and consumer adoption.
References use the existing versioned persistent-reference codec; sketches are
retained by session checkpoints, not by solid-only arena serialization.

Evidence: `crates/operations/tests/linked_sketch.rs` covers real journaled box
face growth and attached-plane moves at 1e-3/1/1e3, exact arena/reference
round-trip, edge-storage reversal, endpoint mismatch, splits, deletion,
ambiguity, inference, stale history and frame refusal. Binding contracts in
`bindings/linked_sketch.rs` pin driven midpoint constraints after edits, both
read-only endpoint setters, checkpoint handle retirement, and resource-error
rollback after input refresh. `scripts/linked-sketch-packaged.mjs` runs the
same public workflow on direct, `executeBatch` and `executeBatchV2`, against
a fresh build or an installed package directory. Package qualification is
recorded separately against the immutable source commit; this source delivery
does not refresh the committed generated packages.
