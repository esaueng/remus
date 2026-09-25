# Maintained performance baseline

From the repository root:

```bash
python3 scripts/performance/run.py
```

This is the first bounded implementation slice of the [performance audit](kernel-optimization-roadmap.md): M01 workload identity, M06 sampling, and M10 maintained commands. It does not complete those architecture-wide items. It preserves the three priority workload families in a repeatable runner; profiles and optimization changes follow separately.

The runner builds `remus-wasm`'s native `performance_baseline` example using `--locked --profile profiling --no-default-features --features io,perf-counters`, then starts separate worker processes. Rust/Cargo, Python 3.10+, and Node 22+ are required. No new package dependencies or browser setup are needed. The package under `crates/wasm/pkg` must be present. Use `--offline` when Cargo dependencies are cached.

Results go into a new `target/performance/<UTC timestamp>` directory. A completed run prints its location and one median per case. The default retains **five samples in each of three independent processes**, after one discarded warmup in each process, at one Rayon thread. The NURBS family alone takes roughly two minutes at the original audit's speed; compilation and other workloads add time. Runtime varies with hardware and contention.

| Family    | Cases                                                                                                        | Correctness checked after each operation                                                                                                                                                      |
| --------- | ------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| NURBS     | Strict validation and mass properties of the existing hammer-holder STEP fixture                             | Exactly one solid, 160 total faces and 42 NURBS faces; valid strict report; mass within the existing measured reference's 0.1% tolerance; finite center/inertia and positive diagonal moments |
| Transform | 150 translations with 50/200/400 preexisting unit boxes; native direct/batch and installed WASM direct/batch, each with zero or one retained checkpoint | Every call/batch item succeeds; translated bounds, unit volume, and an untouched solid's bounds match expectations                                                                            |
| Chain     | Straight chains of 128/512/2048 points, threshold 1.1                                                        | One component, every point exactly once in expected order, positions and carried parameters preserved                                                                                         |

The hammer-holder mass reference is the existing `50_240.482_8` regression value in `crates/io/tests/regress_shapr3d_reversed_nurbs_faces.rs`; it is a measured reference, not a new independent exact oracle for all mass properties. Center and inertia checks detect invalid output but do not independently establish their numerical accuracy. The chain is intentionally a sparse scaling witness, not coverage of branches, loops or tangencies.

## Timing boundaries

- NURBS import and shape census happen once per process, before warmup. Only the selected validation or property call is timed. Warm samples reuse the imported topology, matching the existing Criterion fixture's lifetime.
- Each transform sample gets a freshly seeded kernel. The timed region contains exactly 150 transforms. Direct calls include argument-buffer creation and wrapper transaction cost; batch input is serialized beforehand, while kernel-side parse/dispatch/output serialization is inside the timer. Native and JS batch-result parsing, numerical checks, and kernel destruction are outside it.
- Chain point construction is outside the timer. One chaining call is timed; output checks and destruction follow.
- Warmup records are retained and explicitly labelled. No sample is silently dropped as an outlier. JSON output and correctness assertions are outside the measured region, except direct-call success handling inside the loop.

These boundaries are not the same as the old audit's batch-normalized scratch chain probe. Compare future runs of this runner to each other. Independent processes are sequential, not concurrent; within-process samples share allocator/runtime state. Setup, startup, validation and teardown still affect process wall time and cache state. A CPU profile of the entire worker includes those phases; this command is not yet a scoped CPU/allocation profiler.

## Results and provenance

- `run.json`: manifest copy/hash, selected cases, fixture hashes, source commit, dirty status and working-tree content hash, toolchain/platform/CPU, relevant environment overrides, native executable hash, package file hashes and package-refresh commit message.
- `build.jsonl` / `build.stderr.txt`: Cargo's build output and exact executable location. The explicit command fixes the target/feature union; the source hash also covers Cargo configuration and the lockfile. Ambient build overrides are recorded, not silently cleared.
- `<case>.<process>.stdout.jsonl`, `.stderr.txt`, `.process.json`: raw worker records, errors, exit status and whole-process duration.
- `samples.jsonl`: validated raw records including warmups and process identity. Inspect `run.json` before using partial samples from an interrupted/failed run.
- `summary.json` and `summary.md`: min/median/max, sample standard deviation, and per-process medians for every selected case; emitted only after all selected workers pass and the source identity remains unchanged. No p95/p99 estimates or automatic performance pass/fail threshold are claimed.

A native worker uses current source. A WASM worker executes the **already committed package**, which may come from an earlier source. Package hashes, version and refresh message remain separate from native provenance. There is no automatic rebuild, parity claim, or cross-runtime speedup calculation. Browser timing, cold-load time, memory attribution, broader gauntlet coverage, and CI regression gates remain follow-up work.

The [workload manifest](../../scripts/performance/workloads.json) reuses the committed fixture and its existing regression reference; it downloads no new corpus. Existing [gauntlet manifests](../../tools/gauntlet/manifests) and [scorecard tooling](../../tools/gauntlet/README.md) remain the wider corpus inventory. This first slice has explicit gaps for other operations, models, rendering and browser engines.

## Focused and comparison runs

```bash
# Smoke execution of every case (not a timing-quality baseline).
python3 scripts/performance/run.py --offline --processes 1 --samples 1

# Retain more samples for the next profiling target.
python3 scripts/performance/run.py --family nurbs --processes 3 --samples 10

# Repeat --family to select several families; choose a new output directory.
python3 scripts/performance/run.py --family transform --family chain --output /tmp/remus-baseline-candidate

# Validate collector failure handling and sample accounting.
python3 -m unittest discover -s scripts/performance -p 'test_*.py' -v
```

Use separate clean checkouts and fresh result directories for baseline/candidate comparisons. Keep the manifest, fixture hashes, compiler, profile/features, environment, hardware and thread count fixed. Alternate which checkout runs first across repetitions to expose drift; compare per-process medians as well as the pooled median. Pin matching package source/build settings before interpreting a native/WASM comparison. This runner deliberately does not claim to control affinity, CPU frequency, thermal state or machine contention.

Any failed operation, invalid sample, worker timeout, changed fixture, or source change rejects the run and exits nonzero. Existing result directories are refused. A forcibly killed parent may leave `status: running`; it is not a successful baseline. Preserve the entire result directory with performance evidence; logs may contain local paths and should be reviewed before publishing.

The legacy `batch_profile` example remains a minimum-only diagnostic. This maintained baseline is the supported raw-sample replacement for its transform scaling scenario; its other assembly/query/build scenarios have not been migrated in this slice. The dated audit evidence stays immutable.

## Transaction scaling diagnostics

The transform cases reuse the same fixed 150-call edit at 50/200/400 unit boxes.
The `_checkpoint` cases retain one checkpoint created after seeding; creation,
restore, and restore correctness checks are outside the timer. Both variants
verify moved/untouched bounds and volume; checkpoint variants additionally verify
restored bounds and volume. Each sample starts from a fresh kernel.

Native builds enable the nondefault `perf-counters` feature. `topology_before`
reports measured live entities, allocated arena slots, solids and PCurves; the
census must stay unchanged across the fixed edit. `transaction_counters` records
transaction entries, deep rollback snapshots, maximum simultaneous scope depth,
and separate copies caused by `Rc::make_mut`. Counters are reset after setup and
read before correctness queries or restore. An active scope prevents reset.
These are thread-local counters for synchronous instrumented scopes in
`run_transacted`, `run_validated`, `with_topology_transaction`, and
`dispatch_with_rollback`; they do not count arbitrary clones, bytes, allocation
calls, or clone duration. Feature-disabled builds contain no counter hooks.
Native diagnostic timings include counter overhead and must be compared only
with the same instrumentation settings.

The installed WASM package is not rebuilt or instrumented. Its checkpoint count
is measured through the public API, while its transaction counters are explicitly
null. Native counters must not be copied onto installed WASM results: package
source equivalence remains unestablished. The runner records both artifact
identities. These measurements identify scaling and snapshot frequency; they do
not establish that snapshot cloning dominates wall time or justify an undo/COW
redesign without CPU/allocation attribution.

For a static census alongside the dynamic witness:

```bash
rg -n 'run_(transacted|validated)|with_topology_transaction|dispatch_with_rollback' crates/
```

The manifest fixture SHA includes the metadata-only STEP export-path anonymization
from commit `0b2a146e`; geometry and the existing numerical oracle are unchanged.
