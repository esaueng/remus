# Integration Tests

End-to-end tests live in crate `tests/` targets and module tests. They exercise
real create → operate → measure/export workflows. This directory holds shared
documentation and fixtures rather than a separate workspace crate.

## Boolean workflow

```rust
use brepkit_operations::boolean::{boolean, BooleanOp};
use brepkit_operations::measure::{solid_bounding_box, solid_volume};
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_topology::Topology;

let mut topo = Topology::new();
let target = make_box(&mut topo, 10.0, 10.0, 10.0)?;
let tool = make_cylinder(&mut topo, 2.0, 12.0)?;
let result = boolean(&mut topo, BooleanOp::Cut, target, tool)?;

let volume = solid_volume(&topo, result, 0.05)?;
let bounds = solid_bounding_box(&topo, result)?;
assert!(volume > 0.0 && volume < 1_000.0);
assert!((bounds.max.x() - 10.0).abs() < 1e-6);
# Ok::<(), Box<dyn std::error::Error>>(())
```

`Cut` takes `(target, tool)` after the operation enum. Keep both handles in the
same `Topology`.

## STEP round trip

```rust
use brepkit_io::step::{read_step, write_step};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::make_box;
use brepkit_topology::Topology;

let mut source = Topology::new();
let solid = make_box(&mut source, 5.0, 3.0, 2.0)?;
let expected = solid_volume(&source, solid, 0.01)?;
let step = write_step(&source, &[solid])?;

let mut imported = Topology::new();
let solids = read_step(&step, &mut imported)?;
assert_eq!(solids.len(), 1);
let actual = solid_volume(&imported, solids[0], 0.01)?;
assert!((actual - expected).abs() < 1e-6);
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Test principles

1. Verify geometry with volume, bounds, center of mass, validation, and surface
   type—not a brittle face count unless topology count is the contract.
2. State the unit and reason for each tolerance or deflection.
3. Re-import exported files into a fresh topology before claiming a round trip.
4. Preserve minimal failing operands as fixtures for robustness regressions.
