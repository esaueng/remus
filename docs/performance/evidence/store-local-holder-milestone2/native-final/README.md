# Native correctness qualification

Runtime source `7364ab7be5e598b7c868bd63239dc831cbceb933` stayed clean throughout the captured runs. The FMA file equals the task baseline, and the torus classifier fingerprint is `6439e3d7b658cac1f32c6339f4c1a96347d7030d251e4f3fc33ccad5842974c7`.

| Gate | Observed result |
|---|---|
| Full workspace nextest, `ci-test`, 12 test threads | 9237 passed; 34 skipped |
| Workspace documentation tests | 9 passed; 2 ignored examples |
| Store-local feature-counter tests | 33 passed; 0 failed or ignored |
| Required release approximation census | 52 fresh rows equal the unchanged snapshot |
| Maintained complex-boolean smoke | 14 eligible cases; 28 semantic records |

The first profiling producer build crashed in rustc/LLVM debug-info generation with SIGSEGV, before any semantic smoke ran. Its original source receipt, Cargo output, stderr and failure status remain preserved. An identical-source/profile retry used a new capture directory and produced fresh actual Cargo JSON, a frozen executable, and all 28 semantic records. Source, settings, build environment, build command, harness/counter hashes, compiler and manifest match between attempts; [the comparison](compiler-retry-comparison.json) records these checks. No source, profile, tolerance, resource guard or proof budget was changed to recover. [The recovery receipt](compiler-retry.json) and the [complex qualification](complex-qualification.json) disclose both attempts.

These are correctness gates. Every test, smoke and census duration is diagnostic and excluded from performance claims. The smoke uses one warmup and one retained sample per eligible case, with store-local requested off, pcurve/wire-area memos on, phase clocks off, and performance counters off. It checks strict result validation, fixed closed-form volume bounds, expected components, material probes and input operand volume-bit preservation. It does not establish arbitrary graph equivalence, whole-input identity or observed route counters. The excluded 256-body witness remains unqualified under the unchanged 4096 component-pair budget.

[The source-bound summary](qualification.json), [workspace receipt](workspace-tests.json), [census receipt](census-qualification.json), [normalized census rows](census.normalized.tsv), and [full report](report.md) retain commands, source/tree/toolchain fingerprints, source-file hashes and producer hashes. Actual Cargo compiler-artifact metadata identifies both semantic producers, their profiles, sizes, modes and frozen executable hashes. Original raw logs, build JSON and executable bytes remain in the local capture; filenames and hashes in these portable receipts refer to those retained originals, not additional bundled files. Placeholder paths preserve the receipt structure without private absolute paths. [The receipt mapping](receipt-links.json) distinguishes the original recovery receipt hash from its path-redacted public file hash; the redaction was checked against the retained original before copying.

Run `python3 verify.py --repo CHECKOUT` to verify this manifest, receipt links, metrics, source fingerprints and the current checkout's recorded source files. Omit `--repo` to verify the portable evidence alone. This verifier performs no build, kernel test or performance measurement. Canonical WASM/package qualification and controlled benchmarks are separate evidence.
