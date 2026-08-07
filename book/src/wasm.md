# WebAssembly

Install the generated package and construct one kernel per independent model:

```bash
npm install brepkit-wasm
```

```javascript
import { BrepKernel } from "brepkit-wasm";

const kernel = new BrepKernel();
const box = kernel.makeBox(20, 10, 5);
const volume = kernel.volume(box, 0.05);
const inertia = kernel.inertiaTensor(box); // row-major 3x3, about the CoM
```

JavaScript receives opaque numeric handles. A handle is valid only for the
kernel instance that created it. Methods throw JavaScript errors for invalid
input or failed kernel operations; do not continue with a missing handle.

The default build includes STEP, IGES, STL, 3MF, OBJ, PLY, and GLB I/O. Build
with `--no-default-features` for a smaller package without file exchange:

```bash
cargo build -p brepkit-wasm --target wasm32-unknown-unknown \
  --release --no-default-features
```

Large sequences can use `executeBatch` to reduce JavaScript/WASM crossings.
Checkpoints use copy-on-write topology snapshots. `deleteSolid(handle)` retires
a solid and any topology entities not shared with another live solid. Retired
handles remain permanently invalid and are never reused. Deletion does not
compact the topology or reclaim its arena memory; create a new kernel when
memory reclamation is required.

`serializeSolid` and `serializeSolids` write version 2 arena documents; the
latter supports several solid roots with shared topology encoded once. Both are
bounded debug replay mechanisms, not geometry-interchange contracts. Frozen
version 1 input will remain readable by `deserializeSolid` and
`deserializeSolids`; new schema changes use additive versioned readers. Loads
always create fresh handles and do not restore unrelated kernel session state,
retired slots, assemblies, sketches, or checkpoints.
