# Real holder parameter rebuild

This maintained Node driver rebuilds the real 31-feature Hammer Holder
opening-width recipe through the public kernel API. It accepts any compatible
kernel package directory, without a consumer checkout, dependency install,
browser build, or generated-package changes. The exact fixture and adapted
plan's origin, copyright, license and hashes are in [NOTICE.md](NOTICE.md).

The recipe holds a 46 mm source opening, four carved rigid end pieces, three
rounded line/arc bridges, and seven separate raised-text solids. It holds
height at 58 mm and varies only `opening_width`. Seventeen fixed features
(four base imports, four masks, four placements, four intersections and the
text import) precede fourteen dependent features: three sketch/extrude pairs,
six body placements, a union, and a text placement. Arcs remain exact circles.

Default `--rebuild full` restores an empty checkpoint and rebuilds all 31
geometry features on every request, including exact input restoration and the
four mask intersections. Explicit `--rebuild suffix` prepares all seventeen
fixed features once and restores that prefix before rebuilding the fourteen
dependent features. This second mode is a partial suffix replay; it does not
establish how much prefix the consumer actually caches. Both modes keep one
checkpoint only, in the same kernel owner, and preserve retirement tombstones.

The driver is a kernel replay of that construction. It does not execute the
consumer's document expression engine, undo history, feature-warning system,
incremental-prefix cache selection, topology witnesses, worker or renderer.
Original STEP translation, opening recognition and lettering separation are
replaced by the immutable exact arena inputs. A cold first tick follows
separately reported module loading, construction and checkpoint setup. In full
mode it includes all 31 geometry features; it is not a cold application launch.

```sh
node scripts/perf/parameter-rebuild/parameter_bench.test.cjs
node scripts/perf/parameter-rebuild/verify_recipe.cjs crates/wasm/pkg /tmp/new-quality.json
node scripts/perf/parameter-rebuild/parameter_bench.cjs \
  --kernel-pkg crates/wasm/pkg --out /tmp/new-preview.json \
  --mode preview --pattern unique --ticks 10 --warmup 2
node scripts/perf/parameter-rebuild/parameter_bench.cjs \
  --kernel-pkg crates/wasm/pkg --out /tmp/new-completion.json \
  --mode completion --pattern unique --ticks 10 --warmup 2
node scripts/perf/parameter-rebuild/compare_runs.cjs \
  /tmp/new-preview.json /tmp/new-completion.json
```

Repeat the two processes and comparison with `--pattern repeated`, then both
patterns with explicit `--rebuild suffix`. Unique
widths are `47 + .02 * tick`; the repeated cycle is `47,48,49,48,47,49` mm.
`revisited` means previously requested in this process, including refusals;
it does not identify a cache hit. The first tick is cold; all ticks with index
less than `--warmup` are excluded from retained pools. First-request and
revisited retained pools are separate, with their actual denominators.
Summaries use empirical nearest-rank p50/p95/max; tiny samples establish no
tail-confidence claim. Output files must be new and are written exclusively.
Errors or failed quality checks stop without publishing a successful result.

Preview stages are restore, fixed prefix (full mode only), dependent exact
rebuild (including checked face unification),
strict validation of all eight solids, grouped display mesh at .02/.35, and
bulk transfer of positions, normals, indices and face offsets. Completion adds
volume and complete kernel feature recognition for every solid at .08.
`previewStageSum` and `stageSum` sum replay stage intervals, including the
in-block numeric plan/profile work, validation and prefix serialization.
They are not isolated native function timings. `observedWall` also includes
between-stage memory probes and JavaScript scheduling overhead.
All geometry/inventory/material/arena reads happen afterwards and may warm
subsequent ticks. `observedWall` ends before its final memory probe. There is
no browser input-to-frame or application completion claim.

Arena allocated/retired slots, WASM linear memory, process RSS, JS heap,
external and array-buffer observations are recorded before/after preparation
and around each stage. Deltas may decrease with GC and cannot attribute
process memory solely to a kernel. Linear memory is module-wide; it need not
shrink after freeing an owner. Fixture buffers, mesh transfers and diagnostic
allocations contribute to process observations. This is a short session,
with production cache defaults and no automatic compaction or slot reuse.

Every tick must retain the 108-face holder partition and seven separate text
solids, strict validity, finite/closed/oriented indexed meshes, exact expected
opening envelope, solid material in the floor and arms, empty opening space,
and empty centers of both original mounting bores. The cross-mode comparison
requires matching complete reachable arena and mesh SHA-256 values, thereby
covering full carriers, trims and per-use boundary authority rather than only
sampled agreement. These hashes are computed over live output bytes; payloads
are reproducible from the immutable fixtures instead of copied into reports.

`verify_recipe.cjs` independently checks the rounded-floor section's analytic
area `142 + 4.5*pi` against order-8 mass-property volume changes over
46→47→49→47→46 mm. Its 0.002 mm³ absolute diagnostic bound covers this
quadrature/trim oracle; it changes no operation tolerance or measurement
deflection. All 42 canonical-input NURBS carriers (8 body, 34 lettering) retain exact
knots, weights and degrees, with expected translated control points within 1e-11 mm.
Both 5 mm mounting bores remain recognized; the whole recognized-feature
inventory and lettering volumes stay invariant. Reverse/revisited and a fresh
kernel must reproduce the entire exact arena digest. Full and suffix replays
must produce the same complete arena at each width; full replay must retain
only its one empty checkpoint across requests. This is a fixture
contract, not general imported-model parameterization.

The offline verifier accepts an optional captured run array as its third path:
`node scripts/perf/parameter-rebuild/verify_recipe.cjs crates/wasm/pkg /tmp/new-audit.json docs/performance/evidence/parameter-rebuild-2026-10-20/runs.json`.
It reconstructs every distinct captured width and matches full arena/mesh
digests to every actual record. Independent numerical volume re-query residuals
are reported separately from exact paired capture comparisons.

The optional `check_plan_fidelity.cjs` takes the exact pinned compiler source,
a captured run array and a new JSON output path. It verifies the source hash
before erasing TypeScript and executing only the unchanged pure planner. It
requires Node TypeScript erasure support (used here with Node 24). This checks
profiles, union order and numeric plans; it does not execute the complete
consumer command compiler or build graph. Adapted arithmetic may differ in
final binary rounding, which is reported explicitly.

Metadata records package version and WASM/wrapper/package-JSON SHA-256,
fixture/source identity, all driver hashes, Node/runtime, CPU and affinity.
Optional `--package-source-sha` is externally supplied and is not automatically
verified; reports must attest package bytes against their committed revision
separately. `--timing-status contended-smoke-excluded` labels development
observations that must not enter measured timing claims. Coordinate quiet or
explicitly reserved-core runs and record co-resident activity honestly.
