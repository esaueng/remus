# Operation Reference

Most modeling functions take a mutable `brepkit_topology::Topology` and return
a typed handle. Handles belong to the topology that created them; passing one
to another topology is an error.

## Create and transform

| Module | Main operations |
| --- | --- |
| `primitives` | Box, cylinder, cone/frustum, sphere, torus, ellipsoid, convex hull, Minkowski sum |
| `extrude`, `revolve` | Sweep a planar face linearly or about an axis |
| `sweep`, `pipe`, `loft`, `helix` | Build solids from profiles and paths |
| `transform`, `copy`, `mirror`, `pattern` | Place or duplicate existing solids |

Primitives use positive dimensions. Boxes occupy `(0, 0, 0)` through
`(dx, dy, dz)`; cylinders and cones begin at `z = 0` and extend along `+Z`.
Use a transform when a centered or differently oriented primitive is needed.

## Boolean and modification

```rust
use brepkit_operations::boolean::{boolean, BooleanOp};

let result = boolean(&mut topo, BooleanOp::Cut, target, tool)?;
```

Operand order matters for `Cut`; `Fuse` and `Intersect` are set-symmetric. An
empty algebraic result is reported as `OperationsError::EmptyResult`, distinct
from invalid input.

| Module | Purpose |
| --- | --- |
| `fillet`, `chamfer`, `blend_ops` | Edge blends; consult stability labels before curved blends |
| `shell_op`, `thicken` | Hollow or thicken geometry |
| `offset_face`, `offset_v2`, `offset_wire` | Offset faces, solids, or wires |
| `draft` | Draft planar faces |
| `section`, `split` | Cross-section or divide geometry |
| `heal`, `sew`, `untrim`, `defeature` | Repair and simplify |

## Measure and validate

`brepkit-operations::measure` provides tessellation-backed volume, surface
area, center of mass, bounding box, edge length, and face perimeter. The
`brepkit-check::properties::solid_properties` path additionally returns the
uniform-density inertia tensor about the center of mass.

Validate imported or heavily modified geometry before export. Strict
validation checks manifold topology; relaxed validation is intended only for
known assembled geometry and should not be used to hide a failed operation.

## Tessellation

`tessellate::tessellate_solid(topo, solid, deflection)` returns positions,
normals, and triangle indices. Deflection is a linear chord-error target in
the model's length unit. Smaller values increase triangle count and runtime.

The Rust API documentation generated with `cargo doc --workspace --no-deps`
is the authoritative signature reference.
