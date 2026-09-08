# Boundary-aware resizing of partial cylindrical faces

Status: planned; investigation and acceptance criteria only, with no target date.

## Intended behavior

Change a selected trimmed cylindrical wall's diameter while keeping its axis
and axial extent fixed. Adjust adjoining face boundaries to retain one valid
solid, without expanding the selected wall into a full cylinder or changing
unrelated material. Preserve exact analytic cylindrical geometry.

## Reproduction and evidence

The OpenZCAD Jolly Fox model is a stepped extruded solid with a quarter-cylinder
adjoining planar faces. The reported kernel is Remus 2.130.0 at
`c557ef5b37544cb451d9d24c8b9ce68e8c8bb39c`.

- Selected radius: 20.5 mm (diameter 41 mm); axis +Z; origin (61, 41, 0) mm.
- Wall area: 386.4158963915445 mm², consistent with a 90° wall, 12 mm high.
- STEP import: one solid, zero validation errors or warnings.
- Original volume: 32296.13970707588 mm³.

Running the supplied `repro.cjs` against the supplied packaged WASM on
2026-09-06 reproduced these results from independent imports:

| Target radius (mm) | Result |
| --- | --- |
| 20 | `non-manifold result` |
| 21 | Produced volume 42099.40444876422; guard expected 33078.39627781974 mm³ |
| 22 | Produced volume 43567.0196096035; guard expected 34699.45808707207 mm³ |
| 28 | `non-manifold result` |
| 32 | `non-manifold result` |

Those expected volumes are outputs of the current guard, not acceptance targets.
The local evidence bundle contains `Jolly-Fox.step`,
`Jolly-Fox.openzcad-diagnostic.json` (modeling history),
`openzcad-interaction-log-2026-09-06.json` (captured attempts), and `repro.cjs`.
The STEP SHA-256 is
`cc8984c9d485f6ee050b54b29112caf0b2dbeaca9ff577eee4c2227f2dbf2267`.
These files are not committed by this roadmap change; retaining the model as a
regression fixture is part of the implementation work.

## Investigation and scope

In `crates/operations/src/push_pull.rs`,
`resize_cylindrical_face_aligned` constructs a full cylinder or tube boolean
tool. Its signed expected volume change uses
`PI * (new_radius² - old_radius²) * height`, assuming a complete 360° wall.
This matches the reported source-level finding and is also present in the
checkout inspected for this entry. Neither assumption respects this face's
angular trims. The precise cause of each boolean topology failure remains
unresolved.

Implementation must:

- Classify the selected face's trims and adjacent support faces, and define
  the supported boundary configurations before choosing a construction.
  Start with the supplied quarter-cylinder adjoining planar faces.
- Recompute intersections with adjacent support faces and rebuild shared
  edges, wires, and trims consistently, including cap boundaries. Keep the
  cylinder analytic and the axis and axial extent fixed.
- Define explicit refusal conditions for unsupported cases, including boundary
  collapse, collisions with unrelated material, and edits requiring topology
  changes outside the supported contract. Larger radii in the evidence are
  investigation cases, not promised successes.
- Retain closed-shell, volume, and analytic-surface guards. Derive an independent
  volume expectation from the intended bounded geometry and adjacent supports.
  Multiplying the existing formula by an angular fraction alone does not fix
  the boolean tool or reconstruct the boundaries; support intersections may
  also change the angular span as the radius changes.

## Acceptance criteria

- Add the supplied STEP as a reproducible regression fixture, identified by
  the hash above. Verify the original import's solid count, clean validation,
  selected cylinder parameters, area, and volume within documented tolerances.
- Independent small inward and outward edits, including diameter 41 → 40 mm
  and 41 → 42 mm, produce the intended single valid solid through both native
  operations and the packaged WASM `resizeCylindricalFace` entry point.
- Verify the unchanged axis and 12 mm axial extent, connected adjoining faces,
  consistent shared boundaries, and retained analytic cylindrical surfaces at
  the requested radius. Check unrelated material remains unchanged and no
  unintended full-cylinder material is added.
- Verify closed manifold topology, no validation errors or warnings, volume
  against an independent bounded-geometry reference, and watertight
  tessellation. Mesh success alone is insufficient; include STEP round-trip
  checks of topology, volume, and analytic surface retention.
- Exercise unsupported configurations and require explicit refusal without
  weakening validation to admit the current incorrect results.
- Preserve full-cylinder boss and bore resize behavior, including existing
  inward/outward, repeated-edit, rotated-axis, scale, and collision regressions
  in `push_pull.rs` and the bracket cylindrical-resize regression.
