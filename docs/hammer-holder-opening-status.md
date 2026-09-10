# Hammer-holder opening: kernel repair checkpoint

The imported Shapr3D holder has a nominal 46 mm opening, 74 mm outside width,
58 mm height and two 5 mm mounting bores. Import is supported. The complete
46 -> 50 mm opening reconstruction is **not yet supported**; no application
parameter or AI editing feature is enabled by this repair.

## Verified cut and intersection

The left mask spans x = [-18, 11], y = [-10, 43], z = [0, 70] mm.
Both `source - mask` and `source intersect mask` now pass strict validation,
produce watertight meshes, preserve the source's arena serialization, and
survive STEP export/reimport. The cut preserves both mounting bores. The
intersection retains the complementary NURBS lettering branches and their
planar faces: 101 faces, matching the independently computed reference.

The repairs address several separate defects:

- Planar split loops were wound against the reversal-adjusted normal instead
  of the stored normal, applying reversal twice.
- CommonBlock replacement and edge merging confused distinct curved branches
  sharing endpoints. Each NURBS branch now retains its own representative;
  geometric membership handles different parameter speeds on the same curve.
- The splitter dropped holes bounded by two open curved edges because their
  vertex count was below three. Such loops can enclose real area.
- Rebuilt and vertex-welded boundaries discarded the source edge's recorded
  tolerance. Copied edges retain that contract; generated edges retain the
  operation tolerance. The source allowance is not multiplied by the normal
  endpoint guard.
- Coplanar NURBS sampling reconstructed parameter intervals instead of using
  stored intervals, folding reversed trimmed loops during comparison.
- A perpendicular cylinder and torus with merely touching axial slabs were
  marched into spurious loops. An exact slab check recognizes that these
  carriers have no one-dimensional intersection. Even a small true overlap
  retains the general intersection path.

The native fixture regressions are in
`crates/io/tests/hammer_opening_partition.rs`. Focused unit tests exercise the
branch, hole, parameter-range, tolerance and tangency cases independently.
`scripts/test-wasm-smoke.mjs` checks the cut and intersection through the
shipped split kernel/translator API, including strict validation, source
preservation, mesh closure and STEP round trips.

## Boundary-audit corrections

The original refusal near (-12.8034951482, 28.5433214964, 16.9921788191) mm was
not evidence of an outside-source point. An independent reference places it on
the original trimming curve. The distance query returned its carrier-surface
projection before considering the closer trim. It now considers both, plus
stored topological vertices when curve endpoints differ within the import's
recorded tolerance.

The audit also used standalone face tessellation, which can sample the full
carrier of a trimmed toroidal patch. It now uses the owning solid's trimmed
mesh and visits each referenced vertex once. The existing sample and distance
work limits, containment tolerance, classification rules and independent
volume audit remain in place. The cached operand classifiers avoid repeated
operand tessellation.

## Remaining blocker and reproduction

Run:

```sh
cargo run --profile ci-test -p remus-io --example hammer_opening
```

The diagnostic passes the first cut and intersection, then exits with
`ExactOnlyUnattainable` when intersecting that partition with the holder
translated by -2 mm in X. The candidate still has open boundaries and fails
strict acceptance. The failure spans coplanar overlap selection and curved
boundaries; preserving the lettering holes alone does not qualify the result.
No candidate is accepted by relaxing checks or permitting a mesh fallback.

An equivalent subtraction construction was also examined:
`current - (mask - translated_source)`. Its first tool cut fails strict sphere
inner-wire orientation checks, so it is not a supported alternative.

## Next acceptance gate

Repair the remaining holder-to-holder boolean, then complete both symmetric
edits and validate the full result: 50 mm opening, unchanged 74 mm width and
58 mm height, both 5 mm bores, original rounds and raised lettering, closed
manifold geometry, and an exact STEP round trip. Repeat at the original and
nearby edited dimensions. Only then expose a replayable document parameter,
undo/redo and the existing AI command path in OpenZCAD. Remus remains the sole
production geometry kernel.
