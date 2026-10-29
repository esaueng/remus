# Native torus-notch controls

The primitive controls are effectively flat; no native speedup is claimed. These cases do not establish performance or coverage of the imported holder NURBS edge curves. Full-holder WASM evidence is reported separately.

Baseline source `c754a9b7f89971691ab8867bf7d7bafcc022ad14`; candidate `7364ab7be5e598b7c868bd63239dc831cbceb933`. Actual Cargo producer JSON, fingerprints and unchanged benchmark/Cargo/lock/FMA/tool inputs bind both frozen executables. Features and rustflags are empty. The matched profiling build uses optimization 3, debug symbols 2, LTO disabled, debug assertions disabled and overflow checks enabled.

Eight fresh processes ran ABBA then BAAB. Each ran the three unchanged native cases with ten samples, 500 ms warmup and a two-second measurement budget. Each iteration includes a fresh topology, torus (10,3,32), box (8,8,8), tool translation (6,-4,-4), boolean and drop. There were 24 case results and 240 timed iteration-batch samples; each batch can contain many operations. Only four processes per variant are independent process observations.

| Case | Baseline median point ms | Candidate median point ms | Candidate increase | ABBA increase | BAAB increase |
|---|---:|---:|---:|---:|---:|
| boolean/torus_notch_cut | 5.715985 | 5.709424 | -0.115% | -0.847% | +0.101% |
| boolean/torus_notch_fuse | 5.763292 | 5.683275 | -1.388% | -0.593% | -2.058% |
| boolean/torus_notch_intersect | 5.577127 | 5.619244 | +0.755% | +2.998% | -0.536% |

Process medians use the average of the two middle process point estimates; block effects use means of two process points per variant. All Criterion estimates here use the recorded slope estimator and linear sampling. Per-process intervals remain separate; no pooled confidence interval or high-confidence tail claim is invented.

Workers were pinned to CPU31 and observed affinity was checked. Executable, receipt and collector hashes were checked before and after every process. Criterion output directories were exclusive. Team scheduling reserved sibling15 as well, but unrelated host activity is not ruled out.

The benchmark unwraps boolean success. Native geometry qualification and canonical package suites are separate evidence. The primitive analytic inputs may bypass parts of the imported torus/NURBS trim-sampling path; these measurements are controls, not proof of a win on that path.

`summary.json` retains every process point and Criterion interval plus matched build provenance. `processes/` contains byte-identical benchmark/sample/estimate JSON and losslessly encoded stdout/stderr. `criterion-stdout.json` provides the exact original Criterion lines for review. `capture-environment.json` retains host observations and hashes of omitted local launch/build paths. Original private raw files remain unchanged; omitted logs are not claimed reconstructable.
