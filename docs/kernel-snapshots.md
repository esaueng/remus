# Kernel session snapshots

`RemusKernel.snapshot()` and `RemusKernel.fromSnapshot(bytes)` let a consumer
reopen a modeling session without replaying its feature history. A snapshot is
the whole session, not a body: every topology arena slot verbatim (retired
handles and their liveness included), the pcurve use index, names and colors,
the whole evolution journal with its continuity counters, and the high-water
marks of the kernel's own handle namespaces (assemblies, legacy sketches, GCS
sketches, checkpoints).

A restored kernel therefore answers exactly like the one that was snapshotted:

- every live solid, face, edge, vertex, wire, shell, loop and coedge handle
  resolves to an identical entity;
- every retired handle is still refused;
- the next allocation of every kind (entities, checkpoints, GCS sketches,
  assemblies) takes the same handle as it would have in the original session;
- journal and evolution queries (`journalSummary`, `resolveOperationOutput`,
  `resolveRef`, attribute propagation) answer identically, and a journaled
  edit after the restore produces the same results, handles and journal
  entries as in the original session.

Tier: `core`. Restoring creates a live session (handles and journal), which a
lazy module cannot own.

## API

| Call | Returns | Notes |
|------|---------|-------|
| `kernel.snapshot()` | `Uint8Array` | Read-only. Deterministic: the same session gives the same bytes. |
| `RemusKernel.fromSnapshot(bytes)` | new `RemusKernel` | All or nothing. Runs the same module set-up as `new RemusKernel()`. |
| `RemusKernel.snapshotDigest(bytes)` | 32 hex characters | The verified content digest, from any build. |
| `RemusKernel.snapshotBuildId()` | string | The identity stamped into snapshots and required by `fromSnapshot`, e.g. `remus-wasm 2026.1.0 / snapshot format 1`. |

Every refusal throws an `Error` whose message starts `snapshot:<code>:`.

| Code | From | Meaning |
|------|------|---------|
| `poisoned` | `snapshot` | The kernel panicked earlier; replay instead. |
| `transient_state` | `snapshot` | A GCS sketch, legacy sketch or assembly is live (see below). |
| `limit_exceeded` | both | Over 256 MiB or 3,000,000 arena slots. |
| `not_a_snapshot` | `fromSnapshot` | Wrong magic. |
| `unsupported_format` | `fromSnapshot` | Another snapshot format version. |
| `corrupt` | `fromSnapshot`, `snapshotDigest` | Truncated, extended, digest mismatch, or undecodable. |
| `build_mismatch` | `fromSnapshot` | Written by another build identity. |
| `invalid` | `fromSnapshot` | Decodes, but the topology is inconsistent (dangling or out-of-range references, loops or coedges disagreeing with their owners, malformed NURBS, an invalid journal). |

## When to snapshot

Snapshot after a rebuild has completed successfully, when the history kernel
holds only modeling state. A snapshot is refused while a GCS sketch, legacy
sketch or assembly is live: their contents are not carried. OpenZCAD solves
sketches on scratch kernels and never creates assemblies in the history
kernel, so this does not arise there. A sketch retired by a checkpoint restore
is fine; only its counter is carried.

Prepared face moves and prepared blend-group edits do not block a snapshot,
but they are not carried: release them before snapshotting, or expect their
handles to be unknown after a restore.

## What to store

Store, next to the project:

1. the snapshot bytes;
2. `RemusKernel.snapshotDigest(bytes)`;
3. the Remus package version (`package.json` `version` of `remus-wasm`) and
   `RemusKernel.snapshotBuildId()`;
4. whatever identifies the modeling history the snapshot was taken from (the
   document revision), so a stale snapshot is never loaded against a changed
   history.

The build identity is the kernel crate version plus the snapshot format
version. It changes when the format or the crate version changes, but two
package builds of the same crate version share it, so key stored snapshots by
the package version as well and discard a snapshot written by a different
package version: a newer kernel must rebuild the history to give that
history's newer results.

## Restore flow

```ts
function openKernel(stored, history) {
  if (stored
      && stored.packageVersion === REMUS_PACKAGE_VERSION
      && stored.buildId === RemusKernel.snapshotBuildId()
      && stored.historyRevision === history.revision) {
    try {
      const kernel = RemusKernel.fromSnapshot(stored.bytes);
      return { kernel, restored: true };
    } catch (error) {
      // Any snapshot:<code> refusal: fall back to replay.
    }
  }
  const kernel = new RemusKernel();
  replay(kernel, history);
  return { kernel, restored: false };
}
```

`fromSnapshot` verifies the digest itself; comparing the stored digest with
`RemusKernel.snapshotDigest(bytes)` first only detects storage damage earlier.
After a fallback replay, take a fresh snapshot.

## What is not carried

- Checkpoints. The restored kernel has none; checkpoint ids issued before the
  snapshot stay stale, and the next checkpoint takes the next id. An undo stack
  built on checkpoints must be rebuilt (take a new base checkpoint after the
  restore).
- Prepared face moves and blend-group edits (`prepareMoveFaces`,
  `prepareResizeBlend`, ...). Their handle counters restart.
- Caches: the classification cache, the per-session cache identity and any
  warm content caches. The first queries after a restore recompute what the
  replayed session would have had cached; results are unchanged.
- Shared geometry storage. Curves and surfaces that one topology shared
  between entities are restored as separate copies, so a restored session can
  use somewhat more memory than the original.

## Format

The binary layout is specified in `crates/io/src/snapshot.rs`: a magic
(`REMUSSNP`), the format version, the build identity, the payload length, a
canonical little-endian payload (LEB128 integers, raw `f64` bits) and a 128-bit
digest (FNV-1a 64 and a SplitMix64 word chain) over every preceding byte. The
digest detects corruption; it is not a signature. Restore validates the
decoded topology structurally before building a kernel, so a malformed or
hostile snapshot is refused rather than loaded.
