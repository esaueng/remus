# W9 preflight slice

Run `bash scripts/test-w9-preflight.sh` from a checkout with the repository's
Rust toolchain, Node, and wasm-pack installed. The report is written to
`$CARGO_TARGET_DIR/w9-preflight.json` (default `target/w9-preflight.json`).
The command exits nonzero on any failed stage, build, or oracle test.

This is the first **partial** O1.2e workflow slice, not qualification of W9
or W1–W9 as a whole. Four committed hostile STEP inputs exercise input-byte
limits, entity limits, duplicate entity IDs, and a missing DATA terminator.
The same versioned case, bytes, and explicit limits drive the native `Model`
facade and a real Node-hosted WebAssembly `executeBatchV2` call. The native
entry imports solids; the batch entry also supports sheets. This preallocation
case set does not reach that difference. Both sessions start with a 2×3×4 box
and successfully export/reimport it as a positive parser control;
the WASM session additionally holds sketch, GCS, assembly, and checkpoint
sentinels because these are separate WASM session fields.

Each case runs twice per surface, in fresh child processes. A 30-second
process deadline covers initialization, seeding, parsing, and snapshotting;
it is a harness safety ceiling, not a measured parser latency or competitive
performance band. Timeout and process failure are recorded as such and fail
the run. Per-surface stage rows separately gate bounded completion, expected
typed refusal, byte-identical snapshot text, and repeat agreement. A separate
surface-agreement row checks native/WASM outcome, code, and rollback parity. Both
surfaces must match the manifest's expected stable error code. The report
retains actual diagnostics, observations, and the manifest SHA-256.

The diagnostic-only `workflow-probes` feature adds `workflowStateSnapshot`.
Its derived Debug representation includes every logical `BrepKernel` field,
including topology arenas and retired slots, attributes, journal, sketches,
assemblies, checkpoints, and poison state. The native oracle uses the complete
`Model` Debug representation, including its operation context. Snapshot bytes
are compared only before/after within one process, never across processes or
architectures: Debug formatting is not portable. The pcurve compatibility
map has sorted Debug output because rollback rebuilds it with a fresh hash
seed; other maps in this bounded fixture set are preserved or empty. This is
not raw allocator memory, persistence, a restore format, or a shipped API.
Mutation controls check that the oracle notices geometry and auxiliary state
changes; the scoring tests reject missing, changed, or contradictory evidence.

The actual WASM run uses a **compatibility build with `io` enabled**, emitted
under the target directory. The shipped kernel instead uses a separate
translator module and has no STEP batch operation. No committed package is
rebuilt or changed by this script. The probe feature is absent from shipped
builds. CI runs this workflow alongside the existing package smoke checks.

Still pending: post-allocation refusal cases (handle high-water preservation
needs an explicit reconciliation with W9's byte-identity requirement), the
shipped split-translator path, other hostile formats, W1–W8, reference-kernel
runners, O1.2d scorecard integration, and the results-page workflow table.
No performance or overall W9 completion claim follows from this report.
