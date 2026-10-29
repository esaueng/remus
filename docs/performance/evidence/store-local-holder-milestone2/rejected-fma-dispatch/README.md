# Rejected standalone FMA dispatcher experiments

None of these candidates is included in the PR runtime. These experiments tested a measured hot leaf, but did not establish a mixed-workload microbenchmark gain. They do not establish a holder or kernel regression: the standalone loop repeats bounded, biased first-N operand samples, shuffles their order, adds common volatile-load/checksum work, and has a different optimizer context from the shipping kernel.

The source baseline is `c754a9b7f89971691ab8867bf7d7bafcc022ad14`, with native dispatch unchanged. Four modules first compared the original dispatcher, zero-product-first ordering, an equivalent integer range predicate, and both changes. A separate pair compared the original dispatcher with `c == 0` multiplication **inside an already-confirmed emulated range**. The rejected broader zero/unit fronts from source `292f084f9165f8f31bcaeb1581324eba69835994` are absent from these modules. The retained `candidate-range-c0.patch.json` losslessly encodes the historical unified diff and its original payload hash; it was not applied to shipping source. JSON encoding preserves blank context lines without introducing repository trailing-whitespace findings.

## Observed micro results

Percent changes use the median of four fresh-process medians per variant. Positive values mean longer duration. Every row is a standalone sampled loop, not a model-generation timing.

| Candidate | Combined mix | Recognition mix | Preview mix | Completion mix |
|---|---:|---:|---:|---:|
| Zero-product first | +7.76% | +15.06% | +6.64% | +10.85% |
| Integer range | +3.75% | +18.08% | +19.82% | +4.79% |
| Both guard changes | +23.42% | +49.71% | +18.60% | +26.60% |
| Range-confirmed zero addend, separate pair | +0.36% | +0.04% | +4.45% | +1.88% |

For the separate zero-addend pair, completion changed +2.85% and +1.01% in the two blocks; preview changed +3.64% and +5.36%. Recognition was effectively flat (+0.004% and +1.74%). The isolated zero-addend family improved 47.15% (blocks 40.68% and 49.51%), while the remaining-emulated family grew 2.22%. The new branch therefore lacked evidence of a mixed micro gain. Complete per-process values and block effects remain in the two summaries and raw process receipts.

The first experiment used 16 fresh processes, four per variant, in two balanced reversed blocks. The second used eight processes in ABBA/BAAB order. Baseline-only calibration froze the same operation counts for every variant within each experiment. Each workload had four unrecorded prewarm calls, two recorded warmups, and six retained samples. All calls were pinned to CPU31; the team reserved SMT sibling15. External host work remained possible. During the second experiment host load rose from 17.48 to 32.54 and CPU pressure avg10 rose from 19.13% to 29.05%; process variation is retained without a causal claim.

## Inputs and correctness

The compressed input is a lossless 1.4MB JSON of bounded actual operands and diagnostic counts from retained holder ticks 2–5. It selects only prefix, rebuild, validation, mesh, volume, and recognition phase windows, totaling 131,032,477 instrumented calls. Setup, restore, qualification, after-timing and teardown are excluded. The full diagnostic capture hashes are retained in the input metadata; upstream observer qualification belongs to its own receipt.

The observer can change compiler inlining or call elimination, so counts describe instrumented execution only. Micro inputs repeat bounded first-N samples with approximate family proportions and deterministic shuffling. They neither replay the actual operation stream nor reproduce within-family operand distributions. In particular, their branch entropy can differ from the real holder workload.

The predicate proof covers all binary64 encodings mathematically; native tests sample 405,224 critical triples, 442,368 exponent-grid triples, and two million full-width random triples. Each first-experiment module passed four existing native tests, with one existing long sweep ignored. The range-confirmed candidate passed ten active native tests in default and SIMD configurations, with the same existing ignored sweep; direct standalone Clippy used the repository lint settings and denied warnings. This is not a Cargo workspace Clippy run.

All five modules passed 578,952 native `mul_add` oracle triples: exact non-NaN bits, including signed zero and infinities, plus the repository's existing NaN-equivalence policy. The complete non-NaN-output subset also matched volatile-loop/check checksums. Every sampled workload was checked against a separate native oracle before timing, and all timed checksums agreed across variants. The original NaN-inclusive checksum policy error and its correction are disclosed in the proof receipt. Arithmetic helpers and the native public function remain byte-identical; the range-confirmed candidate changes only its guarded zero-addend return path.

## Verification and reproduction

Run `python3 verify.py` to verify the manifest, lossless input hash, process and source hashes, and recomputed medians without building or timing. Native test logs whose original trailing blank lines would fail repository whitespace checks are losslessly encoded as `.log.json`; the verifier checks their decoded bytes against the unchanged historical receipt hashes. All retained paths are portable. Generated WASM/native executables, the 18.5MB native oracle corpus, and the roughly 23MB sampled binary inputs/results are omitted.

`python3 reproduce.py --output NEW_DIRECTORY --wasm-opt BINARYEN_117_EXECUTABLE` rebuilds and validates the standalone modules and regenerates the sampled binary inputs/results. It requires the recorded Rust compiler and Binaryen bytes, checks the optimized module hashes, and decodes the historical patch after verifying its payload hash. Native oracle executable bytes can vary with the new source path; regenerated sampled input and expected-result hashes must still match. Set `REMUS_BUILD_CPUS` to restrict compilation. Reproduction performs no timing by default. The tested reproducer from before patch encoding remains in `proofs/reproduce-before-patch-encoding.py`; its original proof receipt is unchanged. The new adapter only decodes the same patch bytes, and its separate check records that equivalence without rerunning benchmarks or rewriting historical results.

The copied `source/run_balanced_micro.py` and `source/run_paired_micro.py` can be run from the reproduction directory against `actual-workloads` for a new capture after coordinating CPU31 and sibling15. Their results remain experimental micro evidence; an accepted kernel optimization still needs separate canonical package correctness and matched holder measurements.
