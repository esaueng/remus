# remus

The native Rust facade for the Remus exact B-Rep modeling kernel. `Model`
owns the topology, operation policy, and evolution journal behind one curated
API surface.

```rust
use remus::prelude::*;

# fn main() -> Result<(), Box<dyn std::error::Error>> {
let mut model = Model::new();

// Primitives are anchored at the origin, so this cylinder rounds off the
// block's corner. Transform it first to place the cut elsewhere.
let block = model.make_box(30.0, 20.0, 10.0)?;
let cutter = model.make_cylinder(5.0, 15.0)?;
let notched = model.cut(block, cutter)?;

// Every policy-aware boolean discloses whether its result stayed exact.
assert_eq!(notched.quality, BooleanQuality::Exact);
let volume = model.volume(notched.solid, 0.1)?;
let step = model.write_step(&[notched.solid])?;

assert!(volume > 0.0);
assert!(step.starts_with("ISO-10303-21;"));
# Ok(())
# }
```
