# Exact mesh-stage reuse during interactive edits

The solid tessellator can retain shared-boundary plans, raw edge samples,
holed-planar CDT results, and mesh connectivity between calls. This supplements
the existing per-face mesh cache: a face move still creates new B-Rep handles,
but unchanged mesh dependencies can reuse their previous work.

Native callers opt in with `remus_operations::tessellate::enable_mesh_reuse()`.
`BrepKernel` enables it by default, so existing grouped/ungrouped solid
meshing calls benefit without changing the application. The ordinary API
still returns an owned complete mesh; this change makes its CPU construction
incremental, rather than introducing a GPU or worker-transfer delta format.

## Reuse and invalidation

A reusable boundary plan records exact floating-point input bits, all curve
and surface data, trim authority, directed pcurves, sampling quality and
policy, wire structure, seam uses, face order, and shared-edge incidence.
Only a complete match can reuse a plan. Sorted edge ordinals remap its
identities onto a newly allocated or restored arena. On a miss, unchanged
individual edges reuse their raw samples; circle synchronization, torus
refinement, seam splits, contact discovery and shared-pool construction run
normally. No translation tolerance is added by this cache.

A local planar CDT result depends on its complete projected input points,
outer-ring length, and inner-ring ranges. A hit reuses local triangles and
Steiner points. The pipeline still lifts those points against the current
plane, reconciles requests from every incident face, and repairs triangles
against the final shared subdivision. The existing per-face cache then
reuses unchanged curved/simple faces against those final chains.

The weld pass may reuse its boundary-vertex list only for the identical
index buffer and vertex count. It still checks current positions at the
existing weld distance and performs the same union operations. Coincident
triangle cleanup can reuse a no-removal certificate only after recomputing
the exact quantized-position equivalence classes and matching the complete
post-weld index buffer. New contacts, different winding or new shared
subdivisions invalidate the corresponding result. Gap filling and solid
validation retain their existing dispatch and acceptance checks.

Keys retain content, rather than trusting a hash, handle, revision counter or
operation label. Hash collisions require full key comparison. In-place
mutation, deletion, checkpoint restore, reverse edits and a different kernel
arena cannot make changed content hit. Oversized sampling and refinement
requests still reach the original refusal paths.

## Storage and diagnostics

The stage cache is thread-local, bounded by 4,096 entries and 16 MiB of
estimated retained bytes, including keys and buffers. It is separate from
the existing 32 MiB face-mesh cache, geometry/measurement memos, application
checkpoint accounting and WebAssembly linear memory. These are admission
estimates, not process RSS ceilings. Quantity requests may not evict display
entries; an unfit quantity request does not flush reusable quantity entries.

Native APIs expose `enable_mesh_reuse_with_limits`, `clear_mesh_reuse`,
`disable_mesh_reuse`, and `mesh_reuse_stats`. WASM exposes `meshReuseStats()`
and `setMeshReuseEnabled(bool)` for diagnostics and ablation. The latter is
module-local and constructing a new kernel enables the default cache again.
The existing `setPerformanceTracing`/`drainPerformanceTrace` scopes measure
boundary sampling, planar CDT, curved faces, normals and mesh cleanup.

Regression tests compare fresh and incremental meshes, including position
and normal bits, triangle indices, face offsets, closure and winding. They
exercise the public 160-face filleted Hammer Holder fixture, both directions
of face motion, repeat values, rebuilt arenas, quality changes, mutable trim
authority and the original sampling resource refusals. Cleanup tests change
contact positions, quality and winding while retaining index layouts, and
compare the resulting weld/deduplication decisions against uncached passes.
