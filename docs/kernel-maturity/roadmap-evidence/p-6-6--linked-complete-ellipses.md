# P-Class 6.6 — complete native ellipse inputs

Tier: core — persistent model references, GCS handles, sketch publication and
checkpoint ownership are live kernel-session state. This extends the existing
linked planar line/circle binding; it does not add a module or dependency.

Qualified family: construction-bound complete native `Ellipse` edges on a
plane parallel to a linked planar attachment, with finite positive axes,
`a - b > 1e-7`, right-handed orthonormal carriers, physically bounded frame
roundoff, a strict signed full-period native domain and both seam endpoints
within the authoritative vertex tolerance. Reversed carriers/traversals,
shifted seams and opposite attachment normal sense retain the same full
geometric ellipse. Partial/multiple turns, circular/near-circular axes,
noncanonical swapped axes, oblique images and inferred/nonunique/deleted
construction references refuse without rebinding by geometry.

`gcsAddExternalEllipse` returns ordinary `{ ellipse, center }` handles for the
existing nine ellipse constraints. The center is read-only; semiaxes and
angle are genuinely absent from the solver parameter map. Constraint
removal, failed solves, resource refusals, native mutable-access attempts,
deletion and checkpoint restore preserve that ownership. Ownership records
are bounded by live ellipses and cloned with sketch state. Existing GCS
dense-matrix/work limits apply; checkpoint telemetry still excludes GCS state
and this change makes no new persistent-state byte-cap claim.

Native oracles live in `crates/sketch/src/gcs/system/tests.rs`,
`crates/operations/tests/linked_sketch.rs` and the ellipse test module in
`crates/wasm/src/bindings/linked_sketch.rs`. They include independent conic
equations over three scales, carrier/attachment senses, principal-axis
reversal and signed shifted trims, a free constrained consumer, native
explicit-history scalar growth, stale/deleted/split references, resource
rollback, mixed-input atomicity after an earlier input can refresh, and
retired handles.

The permanent `scripts/linked-sketch-packaged.mjs` helper is already wired into
both canonical consumer drivers. Its ellipse oracle defines 108 cells over
direct/legacy/V2 routes, three scales, three rigid placements and both
carrier/attachment senses. It proves public creation, constraints,
attachment-plane edits, locked conflicts, typed refusals and undo. Fresh and
installed package qualification is recorded with the immutable build source
in the PR validation receipts.

Remaining: raw `transformSolid` and `transformDetailed` do not publish
construction history. Their source edits must become stale and refuse
atomically; they are negative controls, not a public source-growth workflow.
A separately journaled transform/direct-edit family is needed for turnkey
public ellipse semiaxis growth. General attachment rotations, partial conic
references, splines and nonplanar projections remain outside this slice.

Design: `docs/design/linked-sketch-ellipses.md`.
