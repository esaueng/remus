# Construction-linked complete ellipses

Tier: core — this extends P-Class 6.6 by editing live GCS handles, persistent
construction references, attachment refresh and checkpoint-owned sketch state.
It cannot cross an arena-document boundary without losing that session state.

The bounded family is one complete native `EdgeCurve::Ellipse` on a plane
parallel to the existing planar attachment. Its stored native interval must
certify one signed full turn and both seam evaluations must agree with the
authoritative vertex. Carriers must have finite positive semiaxes, finite
right-handed orthonormal axes, and physically bounded frame roundoff. The
major semiaxis must exceed the minor semiaxis by the unchanged linear
tolerance: circular and unresolved near-circular principal directions refuse.
No partial/multiple turns, oblique projection, inferred identity or geometric
rebinding is introduced.

`gcsAddExternalEllipse(sketch, ref)` returns `{ ellipse, center }`. The center
is fixed and read-only; all three ellipse scalars (`a`, `b`, `angle`) are
excluded from the solver parameter map. The source's major-axis direction
defines the angle in the attachment's explicit frame, modulo the existing
ellipse representation equivalence. Axis reversal, stored traversal and seam
phase do not select a different geometric ellipse. Outside-scope swapped-axis
carriers refuse rather than choosing a principal axis from geometry.

Refresh first resolves every linked line, circle and ellipse, then publishes
one cloned sketch. Any unresolved input preserves the last published state.
Constraint removal and failed plain/detailed solves cannot unlock driven
scalars. Entity deletion removes its ownership metadata; checkpoints clone
the metadata and preserve retired-handle high-water marks. Existing GCS
matrix/work budgets remain authoritative; no checkpoint telemetry or new
memory-cap claim is added.

Acceptance uses native geometry and GCS tests plus real direct, legacy batch
and V2 exports. Existing ellipse construction and journaled extrusion seed
construction references. Native tests use explicit construction events for
semiaxis/center refresh. Public tests prove creation, constraints, parallel
attachment-plane edits and undo. Raw `transformSolid`/`transformDetailed`
does not publish construction history: its source edits must become stale
and refuse atomically. A turnkey public source-growth workflow remains a
separate journaled-transform/direct-edit prerequisite, outside this slice.
An independent free ellipse consumer follows through
concentric/equal-semiaxis constraints. Three scales,
rigid placements, both carrier/attachment normal senses, signed shifted full
turns, seam damage, near-circular/noncanonical swapped axes, mixed-reference refusal,
resource rollback, conflicts, deletion and undo are explicit controls. The
permanent packaged helper must run against both canonical fresh output and
the installed npm tarball; native helpers alone do not qualify the JS API.
