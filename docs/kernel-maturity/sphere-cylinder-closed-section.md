# Shared closed sphere/cylinder section

Tier: core. Boolean construction edits the live kernel topology; tessellation
uses that topology's shared boundary samples. This is a bounded qualification,
not a claim that general quadric booleans are complete.

The regression uses a radius-73.5 sphere centered at `(25.5, -1.5, 0)` and a
radius-6, height-64.1 cylinder starting at `z = 58`. The off-axis intersection
is one closed NURBS section. The cylinder seam partitions it into two edges,
while the sphere's remainder initially carries a single edge. Previously the
common winding-cut pass skipped that single loop, leaving an open sphere
remainder. Exact-quality validation correctly refused the incomplete result.

The winding-cut prepass now admits a single closed loop only on a complete,
hole-free lateral with two full-turn native circular rims and a doubled line
seam. The section must remain strictly between the rims. Both carriers then
receive the same three meridian cuts before face splitting; the section stays
NURBS and both face carriers stay analytic.

The holed-hemisphere mesher also accepts a closed NURBS hole when bounded
positive-weight subdivision qualifies the whole curve against the sphere and
selected hemisphere. Invalid domains, badly conditioned weights, off-sphere
control geometry and exhausted work budgets decline. It uses the existing
shared boundary samples and triangle incidence/orientation checks. A sphere
equator's centroid is a projection singularity, so winding repair preserves
its selected hemisphere when the centroid is indistinguishable from the
sphere center at floating-point precision.

## Regression oracles

- `regress_sphere_cylinder_closed_section.rs`: exact union in both operand
  orders, unchanged inputs, strict validity, analytic carriers, opposite
  two-face edge uses, material witnesses and closed meshes at two deflections.
  Additional cases vary primitive segmentation, scale, seam phase and rigid
  placement. The large rotated case pins the equator orientation failure.
- Independent integration over the cylinder disk gives overlap
  `1219.0344613486054` and added volume `6030.504746075203`. At the original
  dimensions, the volume error must stay below `0.05`. Scaled cases take the
  larger of the area-scaled modeling allowance and volume-scaled trim
  integration allowance. The original fixture's bound is 8.3 ppm of its boss.
- The sphere-chart unit tests exercise valid rational conics plus malformed,
  off-sphere, wrong-hemisphere and ill-conditioned NURBS holes, with no mesh
  mutation on refusal. The winding unit test preserves equators while still
  distinguishing the two senses of ordinary spherical caps.
- `sphere-cylinder-closed-section-packaged.mjs` runs the original dimensions
  through direct, batch and V2 batch APIs in both orders, in fresh build and
  installed-tarball smoke suites. It checks exact quality, native carriers,
  strict validity, edge incidence, material, volume and input immutability.

OpenZCAD's consumer regression additionally covers undo/redo, native project
rebuild and STEP export/reimport. Local evidence and hosted CI are reported
separately in the implementation pull requests; this document does not assert
production deployment.
