# F10 sphere–cylinder fuse qualification

The original campaign witness fuses a unit sphere with an equal-radius cylinder
starting on its equator, with cylinder height one. Its exterior is the lower
hemisphere, the cylinder wall and the top disc: volume `5π/3`, area `5π`.
Before this repair, both unsplit hemispheres were sampled below the equator.
The fuse retained the wrong spherical side and had 16 same-sense shared edges.

The GFA sampler now recognizes a hole-free sphere boundary formed by a complete,
connected partition of one equatorial circle. Stored circle carriers must agree
exactly; sphere-to-circle center and radius agreement uses the existing caller's
linear tolerance. Strict native domains, signed coedge traversal, connected
vertices and a complete turn determine the represented hemisphere. Unqualified
boundaries retain the previous sampling path. This is a bounded qualification,
not a general spherical arrangement algorithm.

A second demonstrated defect affected quantity refinement: cylinder shared-rim
synchronization applied the full face angular sample count to every partial rim
arc. The oversized shared boundary exhausted the sphere filler's unchanged grid
budget. Qualified coaxial cylinder arcs now receive their native fraction of the
face angular count. The original edge sampler remains a lower bound; full rims,
other carriers, tolerances and resource budgets retain their previous behavior.

`regress_opseq_sphere_cylinder_fuse.rs` checks exact analytic carriers, opposing
B-Rep uses, strict validation, Cartesian material membership, independent Gauss
volume, volume and exterior area at relative deflections `1e-4` and `5e-5`
(with the unchanged `1e-4` relative accuracy assertion), and raw mesh closure and
orientation at relative deflections `0.1` and `0.01`. Controls cover scales
`0.001`, `1`, `1000`, both operand orders, sphere partitions 4/8/16/32, cylinder
heights 1 and 1.25, and translation composed with `Rx(0.37) * Rz(0.23)`.
Cancellation and work-budget refusals preserve operands and topology counts.
`scripts/opseq-f10-packaged.mjs` applies the same successful geometry matrix to
the direct API, `executeBatch` and `executeBatchV2`, in fresh and installed
packages. The original campaign replay and native/batch cross-check are active.

## Open fine-grid counterexample

The tiny placed case at relative mesh deflection `1e-5` remains open. Run:

```sh
cargo test -p remus-operations --test regress_opseq_sphere_cylinder_fuse \
  hemisphere_union_tiny_placed_fine_mesh_polar_dedup_open -- --ignored --exact
```

Its original zero-boundary assertion currently fails with 16 boundary edges.
The mesh is closed before triangle deduplication. Four groups each contain two
thin polar triangles with the same position-quantized key and six distinct
actual vertex identities; cancellation opens the mesh. A shared-edge-only
noncoincidence proof cannot resolve these groups. Changing the general
near-coincidence cancellation policy needs separate qualification and is not
part of this repair. The ignored reproduction retains the failing assertion;
the successful packaged matrix does not claim this finer raw mesh closes.

The broad X9 sphere–cylinder campaign exclusion remains. F13 and the translated
frustum-pair F17 are also open; F17 lacks a cone–cone wall section in raw GFA and
is not attributed to the hemisphere sampler.
