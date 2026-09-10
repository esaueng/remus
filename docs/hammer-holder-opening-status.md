# Hammer-holder opening: kernel repair checkpoint

The imported Shapr3D holder has a nominal 46 mm opening, 74 mm outside width,
58 mm height and two 5 mm mounting bores. Import is supported. The complete
46 -> 50 mm opening reconstruction is **not yet supported**; no application
parameter or AI editing feature is enabled by this repair.

## Verified first partition

The left mask spans x = [-18, 11], y = [-10, 43], z = [0, 70] mm.
Subtracting it from the source previously returned exact-quality geometry with
24 inconsistently oriented shared edges. Planar split loops were wound against
the reversal-adjusted normal instead of the stored surface normal, applying
reversal twice.

Correcting orientation exposed a separate rim defect: CommonBlock replacement
matched boundary edges only by their endpoints, confusing the opposite halves
of the mounting-hole rim circles. A branch midpoint check preserves the
complementary arcs. The repaired cut passes strict validation and produces a
watertight mesh. Its original source remains byte-identical in arena
serialization. The regression checks both bore positions, diameters and axial
extents, STEP export/reimport, and round-trip volume stability.

The native regression is `crates/io/tests/hammer_opening_partition.rs`.
`scripts/test-wasm-smoke.mjs` also exercises the shipped split kernel/translator
API, exact-only cut, source preservation, strict validation, bore census,
watertight mesh and STEP round trip. Browser-worker verification on the supplied
holder reports zero errors/warnings and zero boundary/non-manifold mesh edges
both before and after STEP round trip.

## Remaining blocker and reproduction

Run:

```sh
cargo run --profile ci-test -p remus-io --example hammer_opening
```

The diagnostic passes the first cut, then currently exits with
`ExactOnlyUnattainable` at `source intersect left_mask`. The rejected GFA
candidate has 99 faces in three components. Its boundary audit finds a point
near (-12.8034951482, 28.5433214964, 16.9921788191) mm outside the original
holder. The candidate must not be accepted by relaxing the audit or permitting
a mesh fallback.

The audit previously tessellated each operand again for every sample. Its
batch classifier now retains one operand mesh while keeping the same boundary
test, winding thresholds, analytic fallback and failure semantics. This makes
the refusal practical to reproduce; it does not turn the invalid candidate
into an accepted result.

## Next acceptance gate

Repair the mixed-support intersection, then complete both symmetric partitions
and validate the full result: 50 mm opening, unchanged 74 mm width and 58 mm
height, both 5 mm bores, original rounds and raised lettering, closed manifold
geometry, and an exact STEP round trip. Repeat at the original dimension and
nearby edited dimensions. Only then expose a replayable document parameter,
undo/redo and the existing AI command path in OpenZCAD. Remus remains the sole
production geometry kernel.
