---
row: 5.8
---
Tier: core (edits live solid handles and construction history). An isolated
straight ridge between two orthogonal whole-body planar supports now chamfers
through convex cylindrical endpoint rounds by an exact plane cut. The existing
corner builder supplies validation, complete face origins, no consumed or split
source faces, and unchanged boundaries outside the qualified neighborhood.
`regress_chamfer_single_rounded_edge` checks seven distances from 0.03281 to
2.1 against independent section integrals, retained cylinders, STEP round trips,
rotations, scales, mirrors, input immutability, and refusal before opening an
unselected bore. Arbitrary curved supports, concave ridges, two-edge corners,
and asymmetric setbacks are not qualified by this slice.
