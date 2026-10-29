# Rejected FMA shortcut packaged candidate

The candidate preserves exact outputs but regresses the maintained full-holder workload. Reject it for performance. These results describe immutable source 292f and its frozen installed package, regardless of later source changes. No mechanism for the regression is inferred from elapsed timing alone.

Baseline: `c754a9b7f89971691ab8867bf7d7bafcc022ad14`, WASM `8e9c54487facde0fae7225ab9348373c6b032230de5e983988bba5e7816d0ef9`, 10877608 bytes. Candidate: `292f084f9165f8f31bcaeb1581324eba69835994`, WASM `34d4f4d51f2d5b23961427072e7e6510f1fd6a38877bd240d51c6596b315fb73`, 10880592 bytes. Both declare package version `2026.1.84`; source and artifact hashes identify the variants.

The unchanged maintained full 31-feature driver uses six unique-width ticks and two warmups in each fresh process. Each preview/completion mode has eight processes in ABBA then BAAB order: four processes and sixteen retained ticks per variant/mode. Total: 16 processes, 96 raw ticks, 64 retained ticks and 768 raw solid instances. All twelve frozen harness files are byte/hash/mode identical to the prior S1 capture. Profiler, tracing and option toggles are off; production cache defaults are retained.

| Mode | Variant | stageSum p50 ms | stageSum p95 ms | observedWall p50 ms | observedWall p95 ms |
|---|---|---:|---:|---:|---:|
| preview | baseline | 294.310421 | 448.240765 | 300.073332 | 452.928904 |
| preview | candidate | 311.799601 | 454.910197 | 318.992554 | 457.822472 |
| completion | baseline | 468.760948 | 660.942511 | 477.060763 | 665.541451 |
| completion | candidate | 516.505928 | 711.264323 | 526.136862 | 716.461415 |

Quantiles use empirical nearest rank. At n=16 the p95 is the largest retained tick; no tail-confidence claim is supported. Positive percentages below mean slower.

| Mode | Pooled stageSum increase | Pooled observedWall increase | ABBA stageSum increase | BAAB stageSum increase |
|---|---:|---:|---:|---:|
| preview | 5.942% | 6.305% | 5.942% | 6.640% |
| completion | 10.185% | 10.287% | 10.030% | 10.329% |

Both process orders show regression. The proposed 10% completion reduction was not achieved.

| Mode | Variant | Process ordinals | stageSum medians ms |
|---|---|---|---|
| preview | baseline | 0, 3, 5, 6 | 294.310421, 293.225430, 288.573822, 287.277149 |
| preview | candidate | 1, 2, 4, 7 | 311.799601, 311.524209, 307.120097, 307.734218 |
| completion | baseline | 0, 3, 5, 6 | 457.529872, 468.760948, 467.882136, 468.150583 |
| completion | candidate | 1, 2, 4, 7 | 505.626921, 515.779780, 511.560252, 516.505928 |

| Completion stage | Baseline p50 ms | Candidate p50 ms | Increase |
|---|---:|---:|---:|
| prefix | 122.194775 | 124.378582 | 1.787% |
| rebuild | 141.062713 | 149.978494 | 6.320% |
| validate | 2.007264 | 1.985492 | -1.085% |
| mesh | 34.263494 | 34.624497 | 1.054% |
| volume | 19.434069 | 22.235884 | 14.417% |
| recognize | 154.361326 | 180.122141 | 16.689% |
| transfer | 0.270020 | 0.236958 | -12.244% |

Module load, construction and checkpoint preparation are separate setup intervals. First replay and warmups are retained but excluded from the warmed quantiles. First replay is not cold application launch. stageSum excludes setup and output qualification; observedWall is synchronous Node kernel/transfer replay, excluding application expression processing, worker/renderer and browser presentation. Memory observations are stage snapshots, not lifetime process peaks or long-session reclamation evidence.

| Mode | Variant | Module load p50 ms | Constructor p50 ms | Preparation p50 ms | First replay stageSum p50 ms |
|---|---|---:|---:|---:|---:|
| preview | baseline | 64.301431 | 0.614401 | 0.164801 | 3116.909097 |
| preview | candidate | 64.193958 | 0.592670 | 0.164691 | 3241.318384 |
| completion | baseline | 66.358199 | 0.644338 | 0.163209 | 4122.465833 |
| completion | candidate | 65.781958 | 0.591387 | 0.161926 | 4198.382137 |

CPU31 and sibling15 were reserved from team work during the timed window. Aggregate load declined from about 20.0 to 7.86; CPU-pressure avg10 ranged about 2.6–11.3; memory-pressure avg10 remained zero. These observations do not prove absence of unrelated host activity or identify a specific cause. No CPU-frequency or process-inventory attribution is available.

All 96 records pass the maintained gates, with 32 within-variant cross-mode pairs and 16 cross-variant completion pairs. Exact arenas, mesh hashes and quality records, Float64 volume bits, complete features, material/inventory and checkpoint policy match. All 16 original driver JSON files reconstruct byte-for-byte from portable process/sample JSONL.

Both independent recipe audits match all 48 captured records per variant across six widths. All 8 body +34 text NURBS carriers preserve exact degrees, knots, weights and control points. Maximum analytic volume-delta residual is 0.00020576963493113 mm³ under the unchanged 0.002 mm³ bound. Maximum quantity reread difference is 2.1100277081131935e-10 mm³, reported separately. Audit JSON files are byte-identical across variants and to the prior S1 audits.

No native elapsed gain is claimed. Separate final-source native code-control proof reports identical executable sections and addresses to the baseline; the S1 native regression belongs to its rejected earlier source.

Capture window: 2026-10-29T01:56:50.186699+00:00 through 2026-10-29T01:58:47.325805+00:00.

| Artifact | SHA-256 |
|---|---|
| qualified-summary.json | `c4f24dd7879fbb6f229e6a3255517e890eb2bd5838dd9d96a5df940f6889db0a` |
| processes.jsonl | `bb4a1f87465e565d381f06e59e690de2af9e9788676c2afaf8afecd519409729` |
| samples.jsonl | `b28a1b90db7db413cc07b2ef2fa77f90a48322d5f835f604bb6cc6b24f54f9a6` |
| baseline-independent-audit.json | `6f8bd3ce94ca2a838069e87668bc402a5df271d968b8e5cfb05884d347a1cde7` |
| candidate-independent-audit.json | `6f8bd3ce94ca2a838069e87668bc402a5df271d968b8e5cfb05884d347a1cde7` |
| capture-environment.json | `8c9b4ae4f3a1509752860e41dcf5ac985a8c655060151574e5e2f0f4280316b4` |
| capture_ab.py | `acc7f42ce5522c8e0fb05e998c22f4d1fc2ca0369188579b2cd1b4416098cb71` |
| qualify.cjs | `ef58e6b7da1424e649b8b7f53ec17284c6cd872fc869963330bc1bf75faa6a0f` |
| ready.json | `aa97dab93766ebf46555142d23e6fcef3a1fa62e1c7d50ff65005c3217dae3d8` |
