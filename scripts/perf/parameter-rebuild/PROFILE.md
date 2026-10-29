# Holder CPU and method diagnostics

`profile_holder.cjs` observes the existing parameter replay without changing
`parameter_bench.cjs` or its timing boundaries. It records public kernel method
calls, recipe preparation/rebuild/transfer spans, and an in-process Node
Inspector CPU profile. Use a fresh Node process for each capture. Full replay
remains the default; suffix replay must be requested explicitly.

These are diagnostic observations. The observer, CPU sampler and retained raw
data add overhead. The tool requires `contended-smoke-excluded` timing status
and rejects `measured` before loading a kernel or creating output. Run the
original timing driver separately for latency comparisons. The observer loads
the package before the original module-load timer, so the resulting
`moduleLoadMs` is not a startup measurement. No browser presentation is measured.

First build and freeze the intended canonical package and retain its source,
build and installed-package receipts. The diagnostic hashes every package file,
the driver, recipe, quality helper and fixtures before and after execution. A
changed file fails the capture. The optional source SHA remains an external
label: package hashing does not establish source/build lineage. An optional
package receipt is copied verbatim and identified by its original hash; the
profiler does not independently validate its assertions.

```sh
taskset -c 31 node scripts/perf/parameter-rebuild/profile_holder.cjs \
  --kernel-pkg /path/to/verified/kernel --out /tmp/new-holder-completion \
  --mode completion --pattern unique --rebuild full --ticks 6 --warmup 2 \
  --package-source-sha 0123456789012345678901234567890123456789 \
  --package-receipt /path/to/verified-package-receipt.json
```

Repeat with `--mode preview` and a new output directory. Coordinate the capture
core and its SMT sibling with other work; CPU affinity alone does not establish
a quiet host. The default CPU sample interval is 500 microseconds; use
`--sample-interval-us` to select 100–10000. The original driver retains every
geometry, material, mesh, inventory, prefix and checkpoint check. Its exact
quantities/features and arena/mesh hashes remain in `run.json`.

Output directories must be new. Each gets an initial receipt before package
initialization. Failures preserve a `capture.json` marked failed and any raw
observations already obtained; they never become successful summaries. The
successful receipt binds `run.json`, `methods.json`, `profile.cpuprofile` and
the other raw files by size and SHA-256. Keep these original files local: CPU
profiles contain absolute script URLs, and external receipts can contain local
paths or other information unsuitable for publication.

The shipping module can lack function names. Generate a matching named module
from the exact canonical build's raw Rust WASM using the commands in
[recognition-graph/README.md](../recognition-graph/README.md). The summarizer
reuses its strict `qualify_names` implementation: removing only the custom name
section must produce the exact supplied shipping bytes. A similar module,
different build, symbol-map index guess or source label does not qualify.

```sh
python3 scripts/perf/parameter-rebuild/summarize_profile.py \
  --capture /tmp/new-holder-completion \
  --named-wasm /path/to/qualified/named-optimized.wasm \
  --shipping-wasm /path/to/verified/kernel/remus_wasm_bg.wasm \
  --out /tmp/new-holder-completion-summary.json
```

The summarizer verifies the original raw hashes, shipping package and driver
identities, complete accepted tick count, strict validation results and CPU
clock containment. It rejects inconsistent cold/warmup flags, negative or
invalid deltas, absent CPU nodes and overlapping top-level method windows.
It includes only non-cold, non-warmup calls in the recorded measured phases.
Setup, memory probes, post-timing qualification and teardown stay excluded.
Recipe spans cover preparation/rebuild JavaScript and bulk extraction; other
JavaScript outside observed call/span windows is unassigned. This is explicit
coverage, not a claim to assign every in-stage instruction.

CPU output reports retained sample denominators, excluded samples, per-method
calls, per-solid recognition/volume samples and unmapped leaf/ancestry samples.
Inclusive sample shares overlap and must not be added. Samples and instrumented
intervals identify candidates; they establish no production speedup or tail
latency. Preserve exact arithmetic, root order, tolerances, solver budgets and
acceptance checks when acting on a profile.

The summary uses relative artifact names and omits CPU frame URLs. It retains
the original local raw hashes and does not claim that raw files can be
reconstructed from the summary. Inspect the summary before sharing, including
package filenames and externally supplied metadata. Keep canonical build and
source receipts with the performance report; a hash-only diagnostic does not
verify those claims.

```sh
node scripts/perf/parameter-rebuild/profile_holder.test.cjs
python3 -B scripts/perf/parameter-rebuild/test_summarize_profile.py
```
