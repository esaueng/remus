# WebAssembly

The package is not yet published to npm. Pin a reviewed Remus commit, then
construct one kernel per independent model:

```bash
pnpm add 'remus-wasm@github:esaueng/remus#<commit>&path:/crates/wasm/pkg'
```

```javascript
import { BrepKernel } from 'remus-wasm';

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
cargo build -p remus-wasm --target wasm32-unknown-unknown \
  --release --no-default-features
```

Large sequences can use `executeBatch` to reduce JavaScript/WASM crossings.
`executeBatch` is the permanent compatibility entry point: each failure stays
`{"error":"human-readable message"}` and existing error text is not replaced
with a machine code.

Call `executeBatchV2` when code needs to branch on failures. It accepts the
same input and returns the same bare array and success envelopes, but errors
are structured:

```ts
type BatchResultV2 =
  | { ok: unknown }
  | {
      error: {
        code: BatchErrorCodeV2;
        message: string;
        details: Record<string, string | number | boolean | null>;
      };
    };
```

```json
{
  "error": {
    "code": "invalid_handle",
    "message": "invalid solid handle: index 42 is out of bounds",
    "details": {
      "entity": "solid",
      "index": 42,
      "operation": "volume",
      "operationIndex": 3
    }
  }
}
```

`details` is always an object. Per-operation errors always include `operation`
and zero-based `operationIndex`; other fields depend on the code. Parse and
whole-batch limit errors have no operation index because no individual
operation was dispatched.

| Code                      | Meaning                                                             | Stable details                                                        |
| ------------------------- | ------------------------------------------------------------------- | --------------------------------------------------------------------- |
| `invalid_json`            | The batch document is not valid JSON                                | `line`, `column`                                                      |
| `batch_limit_exceeded`    | The JSON byte or operation-count limit was exceeded                 | `resource`, `limit`, `actual`                                         |
| `missing_operation`       | An item has no valid `op` field                                     | `operationIndex`                                                      |
| `unknown_operation`       | The operation name is unsupported                                   | `operation`, `operationIndex`                                         |
| `invalid_argument`        | An argument is missing, non-finite, out of range, or the wrong type | `operation`, `operationIndex`, and `argument` when known              |
| `invalid_handle`          | A handle does not resolve to a live required entity                 | `operation`, `operationIndex`, `entity`, `index`                      |
| `topology_error`          | Referenced topology is absent or inconsistent                       | `operation`, `operationIndex`, and safe entity context                |
| `operation_failed`        | A modeling algorithm refused or failed the request                  | `operation`, `operationIndex`                                         |
| `resource_limit_exceeded` | A typed lower-layer import or model budget was exceeded             | `resource`, `limit`, `actual`; operation context when dispatched      |
| `internal_error`          | A failure cannot be safely classified                               | operation context when dispatched; no unstable internals are promised |

Codes are lowercase ASCII snake case. New codes may be added, but an existing
code's meaning will not be broadened or reassigned. The `message` remains for
people and is not a branching contract. Direct WASM methods keep their current
thrown `JsError` behavior; E5b changes only the additive batch-v2 entry point.

Checkpoints use copy-on-write topology snapshots. `deleteSolid(handle)` retires
a solid and any topology entities not shared with another live solid. Retired
handles remain permanently invalid and are never reused. Deletion does not
compact the topology or reclaim its arena memory; create a new kernel when
memory reclamation is required. Deletion is rejected atomically while a live
compound, comp-solid, or assembly references the solid, and restoring an older
checkpoint never revives a retired handle.

`serializeSolid` and `serializeSolids` write version 2 arena documents; the
latter supports several solid roots with shared topology encoded once. Both are
bounded debug replay mechanisms, not geometry-interchange contracts. Frozen
version 1 input will remain readable by `deserializeSolid` and
`deserializeSolids`; new schema changes use additive versioned readers. Loads
always create fresh handles and do not restore unrelated kernel session state,
retired slots, assemblies, sketches, or checkpoints.
