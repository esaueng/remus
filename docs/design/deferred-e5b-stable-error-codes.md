# E5b: Stable WASM error codes (deferred design)

Status: deferred. This document defines the decisions and migration work that
must precede implementation. It does not change the current wire format.

## Context

`executeBatch` currently returns one JSON object per operation, using
`{"ok": value}` for success and `{"error": string}` for failure. Parse and
batch-limit failures use the same string-only error field. Direct WASM methods
surface `JsError` messages derived from `WasmError` and lower-layer error
display strings.

Those messages are useful to people but are not a stable machine contract.
Changing `error` from a string to an object would break existing consumers, so
stable codes require a versioned, additive API rather than an in-place edit.

## Goals

- Give callers a documented, stable code suitable for branching and telemetry.
- Preserve a human-readable message and optional structured context.
- Keep the existing `executeBatch` response byte shape unchanged.
- Decouple public codes from Rust enum names and internal error prose.
- Use the same code registry for future structured direct-method errors.

This work does not redesign operation results, add localization, expose Rust
backtraces, or promise that every internal failure has a unique code.

## Proposed contract

Add `executeBatchV2` while retaining `executeBatch` indefinitely. A v2 item is
one of:

```json
{"ok": 42}
```

```json
{
  "error": {
    "code": "invalid_handle",
    "message": "invalid solid handle: index 42 is out of bounds",
    "details": {"entity": "solid", "index": 42}
  }
}
```

The initial registry should stay deliberately small:

| Code | Meaning | Expected details |
|---|---|---|
| `invalid_json` | The batch document is not valid JSON | parser location when available |
| `batch_limit_exceeded` | An input-size or operation-count budget was exceeded | limit and actual |
| `missing_operation` | An item has no valid `op` field | operation index |
| `unknown_operation` | The operation name is unsupported | operation name |
| `invalid_argument` | An argument is missing, non-finite, out of range, or the wrong type | argument name when known |
| `invalid_handle` | A handle does not resolve to a live entity of the required kind | entity and index |
| `topology_error` | Referenced topology is absent or inconsistent | entity context when safe |
| `operation_failed` | A modeling algorithm refused or failed the request | operation name |
| `resource_limit_exceeded` | A lower-layer import or model budget was exceeded | resource, limit, and actual |
| `internal_error` | A failure cannot be safely classified | no unstable internals required |

Codes are lowercase ASCII snake case and never contain operation-specific
prose. New codes may be added. Existing meanings must not be broadened or
reassigned; a replacement code is required when semantics diverge.

## Mapping architecture

Introduce an internal structured error value at the WASM boundary with
`code`, `message`, and serializable `details`. Conversion should match on
typed errors (`WasmError`, `TopologyError`, `OperationsError`, and I/O errors)
before any error becomes a display string. The registry must not derive codes
from `Debug`, `Display`, type names, or message substring matching.

Batch dispatch should return the typed value internally. The legacy method
then serializes only its `message`; the v2 method serializes the full object.
This keeps both APIs on one execution path and prevents diagnostic drift.

Some lower-layer enums currently lack enough context for a precise public
mapping. Those variants should map to a broad stable code until typed fields
are added. Adding fields to internal errors must not silently change a public
code.

## Direct-method errors

Changing every thrown `JsError` is a separate compatibility decision.
Possible follow-up options are a JS `Error` with enumerable `code` and
`details`, or additive `*Detailed` methods returning a result envelope. The
batch registry should be designed for reuse, but E5b should ship batch v2
first and leave existing thrown errors untouched.

## Compatibility and rollout

1. Publish the code registry and v2 JSON schema in the book.
2. Add `executeBatchV2`; do not add a mode flag to `executeBatch`.
3. Add TypeScript types for the v2 success and error envelopes.
4. Keep legacy tests that assert `error` is a string.
5. Add contract tests pinning every initial code and its required details.
6. Fuzz malformed batches and verify both entry points execute the same valid
   operations and never panic.

## Decisions required before implementation

- Whether v2 returns a bare array like v1 or a top-level object carrying a
  schema version.
- Which lower-layer conditions deserve their own stable code at launch.
- Whether `details` is always an object or may be omitted.
- Whether operation indices belong on every error or only batch-level errors.
- The long-term direct-method error representation in JavaScript.

Until those decisions are approved, `{"error": string}` remains the public
batch contract.
