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
Checkpoints use copy-on-write topology snapshots. `serializeSolid` is a bounded
debug replay mechanism and not a stable interchange contract.
