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
budget. Qualified complete two-rim circular cylinder bands now receive each arc’s
native fraction of the face angular count. Admission requires a closed wire,
two connected full-turn circular rims at distinct axial levels, one doubled
axial seam with opposite uses and no holes. Missing or unresolved native data
declines to the old density; stepped, partial and noncircular trims retain their
established synchronization. Existing stepped-wall normal and cross-drilled
replay-count assertions remain unchanged. The original edge sampler remains a lower bound; full rims,
other carriers, tolerances and resource budgets retain their previous behavior.

`regress_opseq_sphere_cylinder_fuse.rs` checks exact analytic carriers, opposing
B-Rep uses, strict validation, Cartesian material membership, independent Gauss
volume, volume and exterior area at relative deflections `1e-4` and `5e-5`
(with the unchanged `1e-4` relative accuracy assertion), and raw mesh closure and
orientation at relative deflections `0.1` and `0.01`. Controls cover scales
`0.001`, `1`, `1000`, both operand orders, sphere partitions 4/8/16/32, cylinder
heights 1 and 1.25, and translation composed with `Rx(0.37) * Rz(0.23)`.
The two `1000×` placed operand orders retain the exact
`ExactOnlyUnattainable` refusal and unchanged state: immutable base `0551bb432`
also discards incomplete section loops and rejects a two-face result for these
cells. The 32-partition result passes the geometry, material, preview-mesh and
mesh-quantity assertions; its independent Gauss route retains the exact
`IntegrationFailed("face trim exceeds the 4096-point integration budget")`
refusal. These cells do not count as successful Gauss/boolean qualification.
Cancellation and work-budget refusals preserve operands and topology counts.
`scripts/opseq-f10-packaged.mjs` applies the same successful geometry matrix to
the direct API, `executeBatch` and `executeBatchV2`, in fresh and installed
packages. The original campaign replay and native/batch cross-check are active.

## Fine-grid polar facets

The retained tiny placed case now closes at relative mesh deflections `2e-5`
and `1e-5`, through both plain and grouped tessellation. Run:

```sh
cargo test --profile ci-test -p remus-operations \
  --test regress_opseq_sphere_cylinder_fuse \
  hemisphere_union_tiny_placed_fine_mesh_polar_dedup -- --exact
```

The parent reproduction fails its unchanged zero-boundary assertion with 16
boundary edges at `1e-5`. The mesh was closed before duplicate removal: four
candidate groups each contained two thin polar facets with six distinct vertex
IDs but the same position-quantized corner triple. Cancelling those distinct
facets opened the mesh.

The 1 µm grid remains a candidate filter. For groups of exactly two triangles,
matching dominant projections and filtered exact orientation predicates can now
prove strict separation of the filled facets. Such pairs remain unchanged;
this proof does not rely on shared vertex IDs or optional face attribution.
Touching, overlapping, degenerate, non-finite, unresolved and larger groups keep
the previous cancellation policy. A displacement between supporting planes
alone is insufficient. Existing exact and normal-offset duplicate controls and
the damaged/coarse-mesh reference comparisons retain their assertions.

The new unit witness includes the literal six positions, all three coordinate
permutations, both winding classes, plain/grouped attribution and changed
positions within the same grid cells. The cleanup cache admits only unique
nondegenerate candidate keys, so a preserved collision cannot become a stale
no-removal certificate. Its position-change regression verifies that a later
truly coincident pair still cancels.

The active full-shape witness retains material, exact carrier, strict validity,
quantity, raw mesh incidence and outward-normal assertions. Fresh and installed
package smoke tests add the fine cell for direct construction, `executeBatch`
and `executeBatchV2`. Their independent edge oracle sorts exact integer encodings
of raw vertex IDs; it performs no position welding. This qualifies the retained
fine-grid cell and separation mechanism, not arbitrary fine meshes or all X9.

Qualification at source `8775ec2b58a33143f51c9a488802e8f6bb849ee4` passed
all 8,871 native workspace tests (37 skipped), 9 doctests (2 ignored),
warning-free documentation and all 22 repository policy checks. The 52-row
approximation census matches both its committed snapshot and executed parent
`ca25dececf0dc2845c27b8939cac043fdc4d5d2a`. Existing mesh-pass differential
oracles and all three F10 integration groups also pass.

Canonical `cargo xtask wasm-build` passed both fresh-package and installed-tarball
smoke tests, including all six fine-grid API cells at both deflections. The
generated kernel is 10,491,846 bytes (gzip 3,860,856), SHA-256
`dcd1eb68ec0ab4cda5ed414036d85dbcb4a97a693687faa0aac572ae157c1512`;
the translator is 3,088,530 bytes (gzip 1,167,888), SHA-256
`17b2ca73fb3d268b2495dfa8cb6a2f74c9ba5a44ac2cfc8f1860559c06b40a67`.
These are local source-qualified artifacts, not a published package. Generated
files were preserved separately and the original 20 tracked package files
restored byte for byte. An initial compiler internal error in unchanged
`volume_memo.rs` is retained as a failed attempt; rerunning the same canonical
command without source or tool-flag changes passed.

The broad X9 sphere–cylinder campaign exclusion remains. F13 and the translated
frustum-pair F17 are also open; F17 lacks a cone–cone wall section in raw GFA and
is not attributed to the hemisphere sampler.
