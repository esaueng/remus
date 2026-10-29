# Bounded complex-boolean baseline

This native extension complements the maintained baseline with generic plate
slots, ribs and windows at 0/36/144 existing holes, sparse and densely overlapping
N-body fuses, and a cylinder cut in half by a box. The versioned manifest fixes
the procedural cases and lists coverage gaps. It changes no operation defaults,
geometry tolerances, package bytes or refusal policy.

The 256-body disconnected fuse remains an explicit **unqualified** witness:
strict validation fails closed after its existing 4096 component-pair proof
budget. Its 32640 pairs exceed that budget. Default timing uses 16/64 disjoint
bodies and lists the excluded witness in run metadata; an explicit
`--case sparse-fuse-256` retains the failure and rejects the run. No acceptance
check or resource bound is removed or relaxed.

```sh
python3 -m unittest discover -s scripts/performance/complex-boolean -p 'test_*.py'
python3 scripts/performance/complex-boolean/run.py --offline --processes 1 --samples 1
python3 scripts/performance/complex-boolean/run.py --case slot-cut-144 --store-local on \
  --processes 3 --samples 10 --output /tmp/new-store-local-baseline
```

Each sample starts with fresh topology. `setup` constructs the plate and drills
its holes, or constructs/places the N bodies; fixture generation keeps store-local
execution off. `preflight` measures input volumes, outside the kernel timer.
`kernel` contains only the requested boolean or `fuse_solids` call, including that
API's transactions and result gates. `qualification` independently checks strict
validity, closed-form volume, expected edge-connected components, material probes
and unchanged input-operand volume bits. `teardown` destroys the topology.
`generation_ns` is exactly setup plus kernel; it excludes preflight, qualification
and teardown and is not application completion or browser latency.

The runner builds the profiling worker without `perf-counters` by default in its
own checkout's target directory. `--counters` opts into instrumented work/route
captures, resetting counters after setup/preflight; their timings include
counter overhead even with phase clocks off and must not be labeled production
timings. Counter-free rows explicitly carry null work/routes and an empty phase
inventory. Compare otherwise matched feature-off and feature-on small and large
cases to quantify instrumentation overhead. The `content_memos` setting refers specifically to the GFA pcurve and revolved
wire-area memos. Those two are on by default and persist across fresh-topology
samples in one worker; `--memos off` leaves both disabled. Face integrals,
volume/measurement reuse, NURBS area, face boxes, ray geometry, crossings and
mesh caches retain their native library defaults. This is not the full WASM
constructor cache policy; direct/batch WASM needs its own baseline. `--store-local on` requests
the existing opt-in path; drained route counters record actual attempts and
successes separately from the requested setting. `--phase-timings` additionally
enables counters and inclusive GFA phase clocks and labels the run instrumented; keep those
captures separate from clock-disabled comparisons. Nested phase durations must
not be summed into a disjoint total; `gfa.build_nway` includes the N-way split,
classification and assembly work. Caller-thread counters are
not a complete aggregate of any parallel workers. Every
case/settings pair uses separate sequential worker processes, with labelled warmup
samples, raw JSONL, per-process medians and min/median/max stage summaries. No tail
confidence, release pass/fail threshold or cross-runtime speedup is claimed.

Every run records source commit/dirty-tree hash, manifest and worker hashes,
toolchain, features/profile, environment, CPU affinity and executable SHA-256.
It retains Cargo's actual compiler-artifact receipt plus worker and counter-source
hashes, and freezes the executable as `native-worker` in the new result directory
before launching any sample. This prevents a later build from replacing a running
capture's worker; the frozen executable's identity is checked again at completion.
`--binary PATH` reuses a separately built executable and labels source equivalence
as external; never infer that executable's source from the current checkout.
`--capture-cpu N` pins worker processes while builds retain the caller's affinity.
Reserve the core and its SMT sibling and coordinate other workloads before using
captures as performance evidence. Development smoke on busy cores is correctness
evidence only. Use identical harnesses and balanced A–B–B–A / B–A–A–B process
ordering for comparisons; keep all raw records and disclose host contention.

Missing samples, errors, malformed durations, altered oracle bounds, failed
qualification, changed source or a changed executable reject the run. Existing
output directories are refused. Output belongs under target/performance or outside
the checkout. Interrupted or failed directories are retained with their status;
their partial samples are not a successful baseline.

The simple cylinder is an analytic-carrier case; it does not cover NURBS SSI,
cross-drilled geometry, cavities or tangencies. Pattern generation has its own
probe. WASM distribution/installed-tarball parity and full holder replay remain
separate qualification requirements.
