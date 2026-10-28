---
row: PERF-M02
---
Tier: native. A versioned bounded complex-boolean manifest and maintained
collector add 14 procedural cases: generic slot cut/rib fuse/window intersect
at 0/36/144 holes, 16/64 disjoint-body fuses, 8/16 overlapping-body fuses and a
half-cylinder cut. Each sample uses a fresh topology and reports setup,
input-volume preflight, kernel, independent qualification and destruction
separately. Generation time is explicitly setup plus kernel, not application
completion. Closed-form volumes, material probes, strict validity, component
counts and input-volume preservation qualify every retained sample. The runner
records source/tree/manifest/harness/executable identity and raw independent
process samples, rejects changed source/executable and failed/invalid samples,
and retains failure logs. Production builds are counter-free by default;
`--counters` and `--phase-timings` capture separately labeled instrumentation.
Collector protocol tests and adversarial store-local boundary-comparator tests
run in `kernel-performance-contracts.yml`; no noisy latency threshold is added.
Native-only timings, selected success cases and fixed live documents leave
WASM/runtime parity, imported NURBS, tangencies, refusal rates, allocation bytes,
long sessions and full consumer completion explicit gaps. Pattern scaling and
real-package store-local/holder correctness have separate probes. Commands and
measurement boundaries: `scripts/performance/complex-boolean/README.md`.

The 256-disconnected-body case remains an explicitly selected, known-unqualified
manifest witness. Its strict result validation exceeds the existing 4096
component-pair budget (32640 pairs); it is not a default timing case, and an
explicit invocation still fails and preserves its logs. The guard and budget
are unchanged. Initial instrumented smoke receipts qualify the 14 eligible
cases and three store-local/phase-clock requests, but deliberately exclude
contended timings and label their externally supplied executables honestly.
Receipts: `docs/performance/evidence/complex-boolean-milestone/`.

Subsequent Cargo-artifact-bound captures qualify all 14 eligible cases in a
28-record counter-free smoke. Balanced independent-process controls retain
160 samples for counter overhead and 240 samples for the existing store-local
setting, plus 12 separate route/phase proofs. Instrumentation costs about
1.0–1.5% on the two selected plates; enabled locality reduces observed kernel
medians 33–60% on the three declared 144-hole operations. These are scoped
native observations with documented host-activity limitations. Store-local
remains off by default, and its separate stored-tolerance adoption failure
remains open. Report and portable raw evidence:
`docs/performance/complex-boolean-baseline-2026-10-28.md`.
