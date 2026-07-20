# Troubleshooting

## A boolean returns an error

Validate both operands, confirm they use the same topology and unit scale, and
distinguish `EmptyResult` from invalid input. Preserve a minimal reproduction
before changing tolerances. For a cut, verify target/tool order.

## Imported geometry is huge or tiny

brepkit does not infer file or application units. Normalize coordinates at the
integration boundary and scale linear tolerances and mesh deflection by the
same factor.

## Mesh output is faceted

Reduce tessellation deflection. This increases triangle count and runtime but
does not change exact curves or surfaces. Do not increase topology tolerances
to improve display quality.

## A WASM method rejects a handle

Handles are kernel-local and can become invalid after using the wrong kernel
instance or mixing face, edge, and solid handles. Keep each model's handles
with its `BrepKernel` instance.

## Rendering tests skip locally

Install a supported GPU driver or a software Vulkan implementation such as
Mesa lavapipe. Set `WGPU_BACKEND=vulkan` when selecting lavapipe. CI additionally
sets `BREPKIT_REQUIRE_WGPU_ADAPTER=1`, which turns an unavailable adapter into
a hard failure.

## An importer rejects a large file

Production import limits are deliberate denial-of-service boundaries. For a
trusted workload, call the reader's `*_with_limits` form with an explicit,
reviewed budget; do not remove limits globally.
