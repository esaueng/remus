# Reusable measurements during exact edits

Area queries repeatedly integrated unchanged faces during dragging. Whole-body
volume readings were already memoized, but their keys included raw arena IDs:
a recipe rebuild allocated new IDs and measured unchanged scaffolds again.

The opt-in `measure::enable_measurement_reuse()` adds a scalar face-area memo
and allows the existing volume memo to match rebuilt arenas. The browser kernel
enables the default once; native callers retain the original opt-in behavior.
Every retained scalar comes from the original measurement algorithm. Misses use
the same integration orders, numerical controls, measurement tessellation,
closed-mesh requirements and refusal paths. Display meshes never seed area or
volume. No algebraically predicted volume, translation shortcut, cached error,
coarser tolerance or skipped geometry check is introduced.

## Exact matching and invalidation

An identity hit requires the topology lineage, monotonically advancing generation,
entity ID and request. All topology mutation, rollback and retirement invalidates
that identity. Poisoned identities bypass this path.

A content hit compares complete words, not only a hash: surface and curve data,
vertex coordinates and tolerances, edge trim and optional tolerance, directed
pcurves and their parameter bounds, entity sharing, face reversal, body classes,
closed flags, and ordered shells, faces, wires and edges. The key replaces IDs
with their sorted rank **within each entity type**. This preserves equality and
relative ID order used by measurement tessellation, while removing irrelevant
arena padding. Reordering IDs, changing a tolerance or losing a shared edge
cannot match the old key. No geometric rounding is used.

Area requests include the exact deflection bits. Volume preserves its existing
request/clamp matching policy. Keys have distinct format versions for original
volume, reusable volume and reusable area. Unresolvable or oversized keys fall
back to the original computation; errors are never retained.

## Storage and diagnostics

The new area memo has independent limits of 512 readings and 8 MiB estimated
retained storage (key words, scalar, entry and bookkeeping allowance). Key word
construction is bounded before retention. Limits of zero disable retention;
oversized entries do not evict useful readings. Entries are promoted on hits.

The existing volume memo remains limited to 64 readings / 16 MiB of key bytes;
reusable matching promotes its hits. Existing NURBS-area, face-integral, display
mesh, mesh-stage and history budgets are unchanged. These cache estimates are
not process RSS or a total WASM memory cap.

`set_measurement_reuse_enabled(false)` disables the new paths and releases only
the added area entries. It preserves the pre-existing memos, enabling a controlled
same-binary comparison. Explicit preferences and area limits survive later kernel
construction. `measurementReuseStats()` exposes area counters/bounds and the
existing volume memo counters/bounds as JSON without model content or handles.

## Qualification

`measure/reuse/tests.rs` covers fresh-result parity after rebuilding, mutation and
restore, quality separation, complete-key collision checking, failed measurement
retry, limits, and explicit disable. `io/tests/measurement_reuse.rs` compares all
160 face areas and whole-body volumes bit for bit with fresh measurements on the
public filleted Hammer Holder, including unique offsets, reversals, a repeated
offset, a padded rebuilt arena and tighter quality requests. Existing topology,
watertightness, winding, analytic-carrier and volume regression checks remain.

The browser qualification uses the same application source and complex direct
and recipe fixtures as the preceding native-meshing baseline. It compares that
frozen package against the new package with measurement reuse off and on, in
three rotating matched triples. Timings, provenance, independent geometry proof,
cache bounds and the remaining latency are recorded with the benchmark report.

Risk tier: core kernel measurement caching and additive WASM diagnostics. The
public measurement contracts and existing numerical algorithms are unchanged.
