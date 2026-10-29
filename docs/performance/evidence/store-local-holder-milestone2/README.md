# Store-local qualification and holder rebuild milestone 2

The accepted runtime is `7364ab7be5e598b7c868bd63239dc831cbceb933`, compared
with fixed baseline `c754a9b7f89971691ab8867bf7d7bafcc022ad14`. Qualification
finished on 2026-10-29. Later report commits change documentation/evidence.
The [full report](../../store-local-holder-rebuild-2026-10-28.md) defines scope,
phase boundaries, quantiles, limitations and rejected experiments.

The final source repairs stored tolerance authority in opt-in store-local
splices and carries exact raw samples during torus boundary preparation. The
FMA shim and NURBS ray solver are byte-identical to the baseline. Locality stays
off by default. No tolerance, approximation or resource budget is relaxed.

| Retained holder kernel replay p50 | Baseline ms | Candidate ms | Reduction |
|---|---:|---:|---:|
| Preview stage sum | 268.750 | 259.649 | 3.39% |
| Completion stage sum | 426.483 | 414.960 | 2.70% |
| Completion suffix/rebuild stage | 129.525 | 115.731 | 10.65% |

Both balanced blocks improve. This is a modest local Node replay gain, not a
browser, cold-application, native or broad-kernel speed claim. Primitive native
Criterion controls are flat. The proposed 10% complete-workflow target remains
open; recognition is still about 142 ms. All sixteen raw driver outputs reconstruct
from the portable rows; this claim does not cover raw CPU profiles, launch logs,
compiler logs or other local-only artifacts.

## Final evidence

- `baseline-package.json` and `candidate-package.json`: actual canonical build,
  source tree, paired npm archives, every packed distribution file across
  generated source/archive/installation, hashes, sizes and permission normalization. Both versions are 2026.1.84; the
  hashes distinguish them. Packages are local qualification artifacts.
- `candidate-qualification.json`: 24 direct/batch cells and four holder runs,
  with actual route proof required; zero exact-content failures. Baseline
  qualification files retain the two expected old cross-drilled failures.
- `holder-profile.json`: shipping-byte-qualified baseline CPU attribution,
  7,514 mapped names/zero unmapped WASM ancestors, with diagnostic timings excluded.
- `qualified-summary.json`, `processes.jsonl`, `samples.jsonl`: sixteen fresh
  processes, 96 raw / 64 retained ticks, ABBA/BAAB per preview/completion mode,
  exact recorded outputs and empirical nearest-rank quantiles.
- `capture-environment.json`: source/package/harness/Node/tool hashes,
  CPU 31 / sibling 15 team reservation, per-process host observations and limitations.
- `baseline-independent-audit.json`, `candidate-independent-audit.json`:
  48 matched records/six widths per variant, all 42 NURBS carriers and independent
  material/quantity/checkpoint checks. The audit files are byte-identical.
- [native-final](native-final/README.md): 9,237 workspace tests, 9 docs, 33 store tests,
  52 census rows and 14 complex smoke cases / 28 records. The first LLVM debug-info
  compiler crash and identical successful retry are disclosed separately.
- [native-torus-controls](native-torus-controls/README.md): actual matched Cargo
  producers and eight fresh Criterion processes, all 240 iteration-batch samples,
  every process interval and exact stdout. These primitive controls do not
  establish imported-holder native performance.
- `tooling-checks.json`: 38 Node and 7 Python maintained-tool contracts.
- `scheduled-proof-status.json`: older-head repository proof snapshot; incomplete
  mutation coverage is not used as candidate qualification.

## Earlier experiments

`rejected-uv-cache/`, `rejected-jacobian-cache/` and `rejected-fma-shortcuts/`
retain their own source-bound observations, including exact-output passes that
did not satisfy performance acceptance. They are not final-source results.
[fma-dispatch-investigation](fma-dispatch-investigation/README.md) is an
instrumented custom-shim observation with runtime suites explicitly deferred.
[rejected-fma-dispatch](rejected-fma-dispatch/README.md) retains reproducible
standalone microbenchmarks whose shuffled sampled mixes established no win.
[torus-preliminary](torus-preliminary/README.md) binds the tested pre-lint source
and before-failure work guard. Its production bytes match the final source;
final gates remain separately source-bound.

Some raw patches and text logs are losslessly encoded in JSON to retain context
whitespace/EOF bytes without introducing whitespace failures. Decode the named
payload before comparing its original hash. Their original local artifacts and
historical receipts remain unchanged; each directory explains its transport.

## Verify and reproduce

Run from the repository checkout:

```sh
python3 -B docs/performance/evidence/store-local-holder-milestone2/verify_evidence.py
node docs/performance/evidence/store-local-holder-milestone2/reconstruct_runs.cjs
python3 -B docs/performance/evidence/store-local-holder-milestone2/native-final/verify.py --repo .
```

`EVIDENCE-MANIFEST.json` hashes every shared file except itself. Nested manifests
and original local-only artifact hashes retain their separate stated scope.
The verifier checks the shared inventory; reconstruction checks all 16 original
Node driver JSON hashes without executing the model.

For new timings, use isolated checkouts of the two source commits, canonical
`cargo xtask wasm-build`, npm pack and a separate installed consumer. Verify the
artifact identities first. The maintained driver replays all 31 features:

```sh
env -u NODE_OPTIONS -u NODE_V8_COVERAGE taskset -c 31 node \
  scripts/perf/parameter-rebuild/parameter_bench.cjs \
  --kernel-pkg /path/to/installed/remus-wasm \
  --package-source-sha 7364ab7be5e598b7c868bd63239dc831cbceb933 \
  --out /tmp/new-holder-preview.json --mode preview --rebuild full \
  --pattern unique --ticks 6 --warmup 2 --timing-status measured
```

Use fresh output files/processes, both modes, the recorded ABBA/BAAB order and
paired artifact labels. Keep tracing/profiling off during timings and reserve
the physical core from other team work. Re-run the strict store-local qualifier
with `--require-route-proof on` and the independent `verify_recipe.cjs` audit
outside timing windows. Source labels alone are explicitly external provenance;
the build/package receipts establish their binding. See the maintained driver
README and the full report for the precise benchmark and consumer boundary.
