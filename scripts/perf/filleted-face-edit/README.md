# Filleted face-edit WASM comparison

This harness qualifies outputs and times the public Node WASM APIs for a face
edit on the imported 160-face Shapr3D hammer holder. It runs matched kernel and
translator packages in separate old/new Node subprocesses, with warmed repeats
and alternating process order. Run after builds and other intensive work stop.
It contains no measured performance claims.

Build both current WASM packages with the repository's normal package workflow.
Keep an immutable matched pair of original packages in separate directories,
including their Node wrappers and WASM binaries. Python 3 and Node are required;
the runner uses only Python's standard library.

```sh
python3 scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py \
  --old-kernel /path/to/original-wasm-pkg \
  --old-io /path/to/original-wasm-io-pkg \
  --warmup 2 --samples 6 --rounds 2 --order both \
  --out /path/to/new-output-directory
```

The old package directories are required. New package and fixture defaults are
relative to this repository. Override them with `--new-kernel`, `--new-io`, and
`--fixture`. Without `--out`, a timestamped output directory is created in the
caller's working directory. The output directory must be new. `--cpu auto`
pins the runner and children to the lowest allowed Linux CPU; an explicit CPU
index or `none` is also accepted. Package/runtime versions, module/wrapper/fixture
SHA256 hashes, and affinity are recorded.

The selected face is the outward +X planar cap at x=48, with area approximately
1045.93 mm². Source selection allows 1e-4 mm² difference from the native reference
area; that guard does not relax old/new output comparisons. Each process imports
the fixture once and warms its source queries. Each edit creates a kernel and
restores the source through `serializeSolids`/`deserializeSolids`, preserving
the source body while module caches survive consecutive edits. It calls
`moveFacesJournaled` at distance `5 + iteration * .02`, including warmups.

The outer stages are movement, validation, bounding box, grouped binary display
tessellation at .02/.35, bulk Float32/Uint32 transfer, volume at .08, and complete
feature recognition at .08. Both geometry-first and volume-first schedules run
by default; `--order` can select either. `kernelPipeline` excludes bulk getters;
`pipelineWithTransfer` includes them; `restoredPipeline` also includes kernel
construction and source restoration. Import/cold source queries are reported
separately. Diagnostic reads and file writes occur outside the stage intervals.
This measures kernel calls and transfer, excluding ZCAD orchestration and browser
rendering. Production cache defaults are retained, including the 32 MiB display
cache; that is not a process-memory bound.

Every output must validate, have a watertight indexed display mesh, increase
volume, and place the selected cap at x=48+distance. Old/new qualification requires
exact Float64 volume bits, full feature equality, exact indices/grouping/counts
and analytic radii. Full reachable arena documents are compared byte-exactly and
with a separately reported bounded Float64 numeric comparison. Float32 display
positions/normals report both exact byte identity and bounded near differences
to distinguish existing translated-mesh rounding from topology or quantity
changes. The near bounds are recorded in the manifest; near volume agreement
alone never qualifies a result.

`comparison.json`, `summary.md`, and per-process `results.json` contain the
results. Bit-preserving arena documents and little-endian mesh arrays are saved
for every snapshot. Exit 2 means qualification failed and timing ratios are
invalid for performance claims. Worker errors stop the runner immediately.

When comparing source revisions, state the baseline's origin. The investigation's
native baseline is `61ed862` (main after #976), isolating the current patch. Its
original shipped WASM pair `2026.1.45` was built at `a04b5c86` (#975). Comparing
those packages against current source also includes #976's face-AABB regression
fix, degree 9–10 NURBS evaluation stack buffers, and small circle-pool scan work.
That WASM comparison does not isolate this patch's contribution.

The separate native investigation probes run all-face area queries and a repeat
volume query between drags. Those diagnostics are excluded from native stage
durations but change the next drag's cache history. This WASM harness performs
neither pass; report native and WASM executions as distinct workloads.

Static checks do not instantiate WASM or benchmark:

```sh
node --check scripts/perf/filleted-face-edit/wasm_edit_bench.cjs
python3 -m py_compile scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py
python3 scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py --help
```
