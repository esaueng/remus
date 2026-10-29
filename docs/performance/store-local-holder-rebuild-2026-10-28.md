# Store-local authority and holder rebuild performance

Started 2026-10-28; final qualification 2026-10-29. The stored-authority
blocker is repaired. Matched packaged measurements show 3.39% lower preview
replay p50 and 2.70% lower completion p50, with an exact-output gate.

Tier: core. Store-local splicing preserves stored boundary authority and
sharing. The performance candidate reuses raw edge/projection samples only
within one immutable edge's torus trim preparation. Store-local stays off by
default. Public APIs, tolerances, root policies and sampling budgets are unchanged.

## Source and package boundary

The fixed baseline is `c754a9b7f89971691ab8867bf7d7bafcc022ad14`.
Runtime candidate `7364ab7be5e598b7c868bd63239dc831cbceb933` is frozen.
Canonical smoke and installed-tarball suites pass. A second npm pack/install
verifies every packed distribution file across generated source, archive and
installation; captured permission modes document normalization. The matched measurements below use these artifacts.
The baseline kernel is 10,877,608 bytes, SHA-256
`8e9c54487facde0fae7225ab9348373c6b032230de5e983988bba5e7816d0ef9`.
The candidate kernel is 10,881,041 bytes (+3,433), SHA-256
`19a091a8bf8c8333f08249f14ffb3cb1bb30731ede8d0b00020b9b5dd58ba4df`.
The candidate translator is 3,192,494 bytes, SHA-256
`1e68aec1a6fbf1fb6427fd31aade9b298d82657f3033cde57bfe1af53673f538`.
Both package pairs use local version `2026.1.84`; versions alone do not identify
the artifacts. Generated packages are local qualification artifacts and are not
committed consumer releases.

The [evidence directory](evidence/store-local-holder-milestone2/) keeps earlier
experiments separate from the final candidate. Raw logs and CPU profiles remain
local with hashes. Portable timing rows can reconstruct their original driver
JSON only where the corresponding receipt explicitly makes that claim.

## Repairing the stored-authority blocker

The old opt-in cross-drilled shaft changed a seam edge from explicit `1e-7` to
inherited tolerance. Current effective tolerance and material happened to agree,
but stored authority differed. The source-matched baseline's full package matrix
still fails exactly the enabled direct and batch cells, at
`root.incidence.edges.0.tolerance`. The explicitly run pre-fix native regression
also fails after its effective-tolerance comparison passes.

The splice previously substituted geometry-equal source entities throughout
the rebuilt patch. It now reuses source entities only at the boundary shared
with retained faces. Interior patch entities keep the GFA builder's new stored
authority. At the shared boundary, the full builder's sorted original-edge
precedence supplies the reference behavior. Matching must be injective;
ambiguous coincident entities, incompatible vertex authority and unsupported
rebuilt per-use data decline without changing the inputs. Kept-face pcurves and
periodic winding are preserved.

All 33 focused native store-local tests pass with `perf-counters`, with no
ignored strict witness. The original cross-drilled regression requires an
actual local success; fallback cannot make it pass. A raised boundary-vertex
tolerance is a separate negative eligibility case: direct locality declines
without mutation, and the public fallback exactly matches general GFA. Mixed
inherited and explicit edge tolerances, alias collisions and kept periodic
authority have dedicated regressions. Existing local copy-work guards remain
8/8 versus 54 general faces for Cut, 7/7 versus 55 for Fuse, and 12/12 versus
38 for Intersect as the blank grows.

The package comparator now also encodes sharing and incidence across all eight
entity tables, including orphan entries. Equal encodings establish a concrete
bijection while preserving numeric bits, geometry, trims, tolerance authority,
winding and per-use pcurves. It is not a complete graph-isomorphism solver:
indistinguishable ordering ties can reject an otherwise equivalent graph.
Supported documents have solid roots; attributes and journal/naming behavior
have separate checks. Eighteen adversarial/tool tests include degree-preserving
coincident rewires and edge, wire, loop and coedge alias changes.

Actual routes use the existing opt-in performance trace. Synthetic operation
traces exclude setup and quality checks; holder preparation and rebuild are
recorded separately. Missing or truncated observations are unavailable evidence,
not zero-success counters. `--require-route-proof on` makes the current package
qualification fail if its expected local/general/decline routes are unproven.
Evolution and journaled operations keep their existing general path. This
bounded repair does not qualify default enablement or arbitrary imported,
cavity or NURBS edits.

Both historical repaired packages (`d7b1544a` and `292f084f`) passed all 24 synthetic direct/batch cells and four
holder runs, with zero exact-content failures and route proof required. The
baseline retains the two historical failures. At each holder width, enabling
locality produces four prefix attempts, two local successes and two general
calls. The suffix takes the existing n-ary route once in both modes, with no
local attempt. The final timing comparison retains the production default of locality off.
The opt-in repair is qualified separately from the torus sampling optimization.

Final installed source `7364ab7b` also passes the full 24-cell/four-holder
matrix with route proof required and no exact-content failure. Its package
hash is bound independently by the canonical build and second install receipt;
the qualifier's source label remains explicitly external provenance.

## Selecting the holder cost

The maintained full 31-feature holder recipe is profiled at the fixed baseline,
with preview and completion separate. Two of six ticks per process are warmup;
only retained method windows contribute to the reported CPU attribution. The
observer's module preload, setup, warmup, quality checks and teardown are
excluded. Its timings are diagnostic, including concurrent validation activity,
and do not enter the untraced performance comparison.

Removing only the `name` custom section from the symbol-bearing optimized WASM
produces the exact shipping bytes. All 7,514 mapped function names qualify;
retained stacks have zero unmapped WASM ancestors. There are 2,520 retained
preview samples and 4,720 completion samples. Recognition is the largest
completion method family, especially the seven lettering solids. Among 1,499
recognition samples, edge relations appear in 80.8%, prepared classification in
79.5%, ray crossings in 68.9%, line–NURBS intersection in 54.8%, and Newton
refinement in 49.6%; face-area work appears in 13.1%. These are nested inclusive
shares, not additive percentages. Preview is led by `fuseAll`; the four prefix
intersections are another substantial family.

The existing WASM FMA shim is a leading leaf in recognition and Boolean work.
The arithmetic candidates below did not qualify. The next measured setup family
is `fuseAll`: collecting face geometry appears in 242 of 927 retained samples,
with trimmed-torus preparation in 236 of 927 (26.1% and 25.5%, inclusive).
This selects repeated authoritative-edge projection in torus trim preparation.

## Exact reuse of torus trim samples

The adaptive sampler projects each edge's quarter, half and three-quarter
parameters, then can evaluate its midpoint again before recursion. Child
midpoint probes can also repeat their parent's quarter probes. The candidate
uses three probe slots per recursion frame containing parameter bits, raw UV
and projection residual. The existing depth limit bounds the stack state.
The immutable per-edge closure supplies geometry identity;
no sample crosses an edge, operation or topology mutation.

Reuse requires exact parameter bits. The midpoint uses `f64::midpoint`, while
quarter probes use fused affine arithmetic; mathematically equal parameters
can round differently, so a mismatch recomputes the original evaluation.
Periodic unwrapping uses the current anchor on every logical call, and the
projection-margin update executes in the original order. Only raw curve
sampling, torus projection and reconstruction residual computation are reused.
The depth-20 and 16,384-point limits and all typed refusal behavior are retained.

The independent oracle keeps the original trim and recursion algorithms.
Tests compare every output bit, logical parameter/anchor sequence and refusal
result, including partial polygons and uncertainty on budget exhaustion.
A deterministic parabola requires 1,788 raw evaluations originally and 1,023
with reuse (42.79% fewer). Disabling reuse makes that work guard fail after
its output/order comparisons pass. Four torus boundaries reconstructed from
the existing public holder fixture check exact results and reduce raw work
from 91,267 to 52,654 (42.31% fewer).
These work counts do not establish elapsed-time gains.

## Rejected experiments

The first runtime prototype, `d7b1544a`, cached residual and unclamped-step
arithmetic by exact ray/UV/point bits. Its 240-ray probe reduced delta evaluations
35.03% with exact roots, but controlled native tests regressed 6–8% on three
small carrier cases. All four candidate process medians exceeded all four
baseline medians on those cases. The full packaged recipe was flat: retained
nearest-rank stage-sum p50 preview 280.965 → 279.674 ms (0.46% lower), completion
609.190 → 612.028 ms (0.47% higher), and observed completion wall p50
612.808 → 625.237 ms (2.03% higher). Completion also had a substantial temporal
slowdown in both variants. Its exactness and independent recipe audits passed,
but it demonstrated no pipeline gain. The table was removed, rather than
retaining a work-count result as a speed claim.

A second, one-entry exact-partial Jacobian prototype saved only 20 of 71,636
preparations on 408 generated rays over the actual 34 lettering NURBS carriers
(0.028%); its general probe saved 6 of 3,685. These are generated rays on real
carriers, not a replay of recognition's actual ray inputs. Roots and requested
work remained exact, but reuse was insufficient to justify adoption. Its
reapplicable source patch and work receipt are retained. No cache growth or
numerical relaxation followed. Both rejected experiments remain under
`rejected-uv-cache/` and `rejected-jacobian-cache/` in the evidence directory.


The `292f084f` zero-addend/unit-factor FMA specialization preserved outputs but
regressed the matched packaged recipe: preview stage p50 294.310 -> 311.800 ms
(+5.94%), completion 468.761 -> 516.506 ms (+10.19%). Both balanced blocks agreed.
Those complete captures and source-qualified correctness results remain under
`rejected-fma-shortcuts/`; they are not final-source results.

A diagnostic observer then recorded operand families on the actual recipe
for the custom `remus_math::fma::wasm_fma` shim. Direct `f64::mul_add` callers
elsewhere, including the torus sampler parameter calculations, are outside
these counts. Recognition's instrumented shim calls were 82.12% remaining emulated inputs, 10.36%
nonzero zero-addend inputs and only 0.016% nonzero unit-factor inputs. The
observer's counters and first-sixteen samples per family describe instrumented
execution, whose compiler behavior can differ from shipping code. They are not
a guaranteed shipping dynamic-call census or representative sequence sample.
Its exact holder outputs match the baseline; runtime package suites were
explicitly deferred. See `fma-dispatch-investigation/`.

All four dispatcher variants passed the standalone numerical oracle, but
zero-first/bit-range/combined guards did not improve shuffled sampled mixes.
An inside-range zero-addend case sped its own family while leaving the total
mix flat and preview/completion mixes slower. These are microbenchmarks of
first-N-weighted shuffled operands, not holder latency results. They justify no
adoption or workflow-regression claim. Sources, reproducible input derivation,
raw process records and limitations remain under `rejected-fma-dispatch/`.
Both `fma.rs` and the NURBS ray solver are restored byte-for-byte to the baseline.

## Untraced final package comparison

The unchanged maintained driver recomputes all 31 features on each request
(`--rebuild full --pattern unique`). It clocks the 17-feature prefix and
14-feature suffix/rebuild separately; the suffix is not the whole workflow.
Preview and completion each use eight fresh Node processes in ABBA then
BAAB order: four processes and sixteen retained ticks per variant/mode.
Six ticks per process include two warmups, for 96 raw and 64 retained ticks.
Profiling, tracing and option toggles are off. The twelve source-bound
timing/fixture files are identical across variants. Node is 24.22.1.

| Mode | Variant | Stage sum p50 ms | Stage sum p95 ms | Observed wall p50 ms | Observed wall p95 ms |
|---|---|---:|---:|---:|---:|
| preview | baseline | 268.750 | 393.301 | 272.937 | 396.099 |
| preview | candidate | 259.649 | 378.874 | 264.769 | 381.693 |
| completion | baseline | 426.483 | 602.427 | 434.397 | 606.949 |
| completion | candidate | 414.960 | 596.710 | 422.474 | 600.325 |

Stage sums are computed per tick before taking quantiles. Quantiles use
empirical nearest rank. At n=16, p95 is the largest retained tick; these
samples establish no tail-confidence or broad-workload claim.

| Mode | Pooled stage p50 reduction | Observed wall p50 reduction | ABBA stage reduction | BAAB stage reduction |
|---|---:|---:|---:|---:|
| preview | 3.39% | 2.99% | 2.63% | 4.50% |
| completion | 2.70% | 2.74% | 2.32% | 2.79% |

Both process orders improve. Every candidate process stage p50 is below
every baseline process stage p50 in each mode. The gain is modest for the
complete kernel replay: **3.39% preview and 2.70% completion**. It does not
reach the proposed 10% complete-workflow reduction. Recognition remains
about 142 ms and is still the largest completion cost.

| Completion stage | Baseline p50 ms | Candidate p50 ms | Change |
|---|---:|---:|---:|
| prefix | 107.299 | 107.770 | +0.44% |
| rebuild | 129.525 | 115.731 | -10.65% |
| validate | 1.895 | 1.829 | -3.49% |
| mesh | 29.515 | 30.029 | +1.74% |
| volume | 17.896 | 17.824 | -0.40% |
| recognize | 141.734 | 142.262 | +0.37% |
| transfer | 0.184 | 0.192 | +4.55% |

The suffix/rebuild stage falls **10.65%** in completion mode and **8.59%**
in preview. The other stage medians are reported without individual gain
claims. In particular, recognition was not accelerated by the rejected
arithmetic experiments.

| Mode | Variant | Module load p50 ms | Constructor p50 ms | Preparation p50 ms | First replay stage p50 ms |
|---|---|---:|---:|---:|---:|
| preview | baseline | 59.673 | 0.533 | 0.153 | 2896.079 |
| preview | candidate | 59.221 | 0.542 | 0.148 | 2845.662 |
| completion | baseline | 59.772 | 0.536 | 0.149 | 3650.884 |
| completion | candidate | 59.684 | 0.554 | 0.149 | 3635.451 |

Setup and first replay have four samples per variant/mode. First replay
is not cold application launch. The two warmups are retained in raw data
and excluded from warmed quantiles. Observed wall is synchronous Node
kernel/transfer replay; application expression work, worker/renderer and
browser presentation are outside this measurement.

The team reserved CPU31 and its SMT sibling15; other team builds and checks
were idle. Observed one-minute load ranged 0.35-1.88; CPU-pressure
avg10 ranged 0.00-46.87. These observations do not establish an exclusive
host or identify the source of pressure. No CPU-frequency cause is inferred.

All 96 records pass the maintained gates. Thirty-two within-variant
cross-mode pairs and sixteen cross-variant completion pairs preserve exact
arenas, mesh hashes/quality, all captured Float64 volume bits, complete
features, material/inventory and checkpoint policy. Portable process/sample
JSONL reconstructs all sixteen original driver JSON outputs byte-for-byte.

Both independent recipe audits match 48 captured records across six widths.
They retain all 8 holder and 34 lettering NURBS carriers, exact degrees,
knots and weights, and bounded control-point residuals under the declared
translations. Whole captured arenas are identical across variants. The
maximum closed-form volume-delta residual is 0.000205769635 mm3 under the
unchanged 0.002 mm3 bound. Audit JSON is byte-identical between variants.

## Native timing controls

The unchanged Criterion torus-notch Cut/Fuse/Intersect primitives use eight
fresh ABBA/BAAB processes, four per variant, with 10 iteration-batch samples
per case, 500 ms warmup and 2 s measurement. Actual Cargo artifacts match
features, flags, profiling opt3/debug2/LTO-off, benchmark/config and tool
identities. The table shows medians of process point estimates; it is not
a pooled latency percentile or confidence interval.

| Primitive native control | Baseline ms | Candidate ms | Change |
|---|---:|---:|---:|
| torus_notch_cut | 5.715985 | 5.709424 | -0.11% |
| torus_notch_fuse | 5.763292 | 5.683275 | -1.39% |
| torus_notch_intersect | 5.577127 | 5.619244 | +0.76% |

These controls are effectively flat; no native gain is claimed. Analytic
primitive curves can bypass the expensive imported NURBS-edge torus lane.
Full per-process Criterion intervals, raw sample batches, producer and host
receipts are in `native-torus-controls/`; they do not prove native elapsed
performance on the imported holder.

## Correctness and remaining scope

Immutable runtime `7364ab7b` passes all **9,237 native workspace tests** with
34 existing skips, nine documentation tests (two existing ignored examples),
all 33 focused store-local feature-counter tests, all 52 unchanged approximation
census rows, and the 14 eligible native complex-Boolean smoke cases (28 semantic
records). The first profiling-mode smoke producer build crashed in LLVM debug
information generation; an identical retry passed. Failed and successful
producer captures are retained and compared; no source, profile, resource,
assertion or budget changed to get a pass.

Canonical `cargo xtask wasm-build` passes both smoke and installed-tarball
consumer suites. A second independent npm install matches every packed file's
bytes; permission normalization is recorded. The final installed qualifier
passes all 24 synthetic cells and four holder runs with actual-route proof.
Normal commit hooks pass workspace all-target/all-feature Clippy, formatting,
Taplo and dependency checks. The maintained tool contracts pass 38 Node and
seven Python tests. Boundary, module-map, documentation and roadmap checks
remain separate repository checks.

[Final native receipts](evidence/store-local-holder-milestone2/native-final/README.md)
record actual Cargo producer messages, source and executable hashes. Historical
results remain bound to their own source commits under the rejected directories
and are never relabeled as final runs. The existing scheduled proof snapshot
has successful older-head gauntlet/fuzz runs, while the mutation run is incomplete
(one cancelled shard, judge/upload skipped); it supplies no candidate mutation
qualification. Hosted PR checks and subsequent package publication are separate.


The sparse 256-body fuse fixture remains explicitly unqualified. Its exclusion
and 4,096 validation budget are unchanged. Store-local default promotion,
arbitrary imported-body/cavity/NURBS adoption and general lineage adoption
remain open.

This full 31-feature kernel replay includes eight solids with all 42 retained
NURBS carriers (eight in the holder, 34 in the lettering). It excludes original
STEP translation/history, application expression processing, worker/renderer
boundaries, browser presentation and GPU work. Stage memory readings are
snapshots, not true allocation peaks or long-session reclamation evidence.
Full B28 stage budgets and consumer timing remain open.
