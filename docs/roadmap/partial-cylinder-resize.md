# Boundary-aware resizing of partial cylindrical faces

Status: implementation in progress; native and packaged WASM quarter-wall
qualification pass. Full-workspace verification is pending.

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
- Original reported volume in Remus 2.130.0: 32296.13970707588 mm³.
- Current exact reference volume: 32296.762938013333 mm³, independently
  decomposed as `73*41*8 + 12*30.5*12 + PI*20.5²*12/4`. The earlier value
  used a chorded boundary approximation and is not the acceptance oracle.

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
The STEP is retained unchanged as
`crates/io/tests/data/jolly_fox_partial_cylinder.step`; diagnostic/history files
are not needed by the regression and are not committed.

## Investigation and scope

In `crates/operations/src/push_pull.rs`,
`resize_cylindrical_face_aligned` constructs a full cylinder or tube boolean
tool. Its signed expected volume change uses
`PI * (new_radius² - old_radius²) * height`, assuming a complete 360° wall.
Those assumptions do not respect this face's angular trims. The implementation
now routes partial walls to support-surface re-limitation before the full-turn
boolean path; the full-turn path remains for qualified complete boss/bore walls.

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


## Implemented contract and current qualification

The initial supported partial family is an outward quarter-cylinder with two
coaxial authoritative quarter-circle rims, two axial sides, four distinct planar
supports, perpendicular caps, and radial side planes through the cylinder axis.
The rest of the source solid must have planar faces with straight boundaries.
The clearance proof projects each nonadjacent face into the quarter-sector
frame and conservatively excludes its entire bounding box from the swept
annulus, including face interiors. Possible contact is a typed topology-change
refusal. Unsupported carriers or boundary families refuse explicitly.

The replacement engine reconstructs shared intersection edges and cap wires.
Persistent plane p-curves use canonical surface frames; circular rim p-curves
use angular spans and coedge direction. These are required for exact STEP
round trips. Closed-shell, volume, analytic-radius, axis, and axial-extent
checks remain enabled, with transactional rollback on any failure.

`crates/io/tests/partial_cylinder_resize.rs` currently passes eight native test
groups. They cover independent r20/r21 resizes and direct support replacement;
r20/r21/r22/r28 across scales 0.1/1/10 and rigid placements; shoulder contact at
r30.5 and collision at r32; a collision with the interior of a nonadjacent
planar face; and explicit refusal of a valid half-cylinder with unsupported
boundary curves. Success checks include strict validation, independent volume,
material probes, analytic radius, fixed axis and z8..20 extent, wall area,
watertight meshes at two deflections, and STEP round trips. Refusals preserve
serialized source geometry and arena allocation counts.

An offset-layer witness uses a validated quarter prism and a planar obstacle
whose entire boundary is outside the swept wall. The larger radius refuses on
face-interior contact; a smaller radius succeeds. The public fixture separately
pins transactional collision refusal on a connected solid.

Both rebuilt WASM packages pass smoke and installed-tarball consumer suites,
including eight exact direct/batch resizes with STEP round trips and four
collision refusals. All 20 existing push/pull tests, six replacement tests,
and four generalized move tests pass. Replacement p-curve assertions now use
the canonical plane frame and oriented parameter intervals required by STEP;
legacy blend-preserving projections retain their existing test convention.
Workspace Clippy and the unchanged 52-row approximation census pass.
Full-workspace verification must complete before this slice is ready. General angular trims, nonradial side
supports, inward partial bores, blend-adjacent walls, and topology-changing edits
remain unqualified; journaled direct-edit completion is not claimed by this
geometry slice.
