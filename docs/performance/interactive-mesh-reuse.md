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

Expensive connectivity storage is admitted only after a mesh layout repeats;
a first sighting retains a small marker. This avoids capturing large buffers
for one-off construction results. The weld pass retains its directed-edge table, complete previous index
buffer, and a bounded map of edge-count changes. Every triangle is compared;
only changed triangles update the counts. Only the presence of a changed
edge or its reverse can change boundary membership. Current positions and
the existing weld distance still determine every spatial union decision.
Coincident-triangle cleanup recomputes the exact position-equivalence
partition on every call. A retained no-removal proof requires the same
partition, then removes the old keys and checks every changed triangle
against both unchanged and newly added triangles. A new coincident pair
falls back to the complete winding/cancellation/removal policy. Large
connectivity changes or exhausted delta bounds also use fresh analysis.
Gap filling and exact solid validation retain their acceptance checks.

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
estimates, not process RSS ceilings. One quarter of the total bound is
reserved for boundary/sample/CDT
entries; the remaining three quarters hold incremental mesh connectivity.
Quantity and boolean-fallback paths use the ordinary stages and existing
face/measurement memos, without creating or looking up new stage entries.
This avoids redundant capture of one-off fine measurement meshes. Exact
measurement quality, validation and refusal checks are unchanged.

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
