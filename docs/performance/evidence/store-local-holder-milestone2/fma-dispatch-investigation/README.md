# FMA operand-family diagnostic

**Disposition: no FMA arithmetic or dispatch change was accepted from this investigation.** The full-holder shortcut candidate regressed performance, and the subsequent dispatcher microbenchmarks did not justify adoption. These files preserve diagnostic evidence, not an accepted optimization or a canonical package qualification.

An optimized private observer replayed the unchanged full 31-feature holder recipe once in preview and once in completion. Each fresh process used six unique-width ticks and two warmups. The tables cover retained ticks 2–5 and only outer kernel calls in measured stages. All elapsed values are excluded.

Build identity: source `292f084f9165f8f31bcaeb1581324eba69835994` plus restoration of `fma.rs` from `c754a9b7f89971691ab8867bf7d7bafcc022ad14` and the included two-file observer overlay. The literal original arithmetic body remains unchanged after the counter call. The private binding drains exact integer counts and bounded operand-bit samples. Kernel WASM SHA-256 is `36ec3d7d0d52dfd6ea99f94bee9908802d77da952fbcca1b7eaa8f1e1f31ef21`.

The optimized paired build completed with `--defer-package-tests`; canonical runtime suites were not run for this diagnostic artifact. No production speedup, native gain, tail-latency, browser or canonical-pass claim is made.

Counters are exact for the instrumented execution. Observer side effects may alter compiler CSE, elimination or inlining, so this is not a guaranteed census of shipping calls. Families are disjoint in the listed priority order. In particular, remaining-emulated/fallback exclude earlier zero-addend/unit candidate families and do not equal the complete original dispatch-route counts.

| Family | Preview calls | Share | Completion calls | Share |
|---|---:|---:|---:|---:|
| total | 44,466,091 | 100.000% | 86,566,386 | 100.000% |
| zero_product_a_only | 8,477,987 | 19.066% | 9,967,141 | 11.514% |
| zero_product_b_only | 2,676,502 | 6.019% | 3,917,667 | 4.526% |
| zero_product_both | 3,313,788 | 7.452% | 4,342,830 | 5.017% |
| finite_nonzero_zeroaddend | 9,810,532 | 22.063% | 14,583,329 | 16.846% |
| finite_nonzero_unitfactor | 78,281 | 0.176% | 100,514 | 0.116% |
| remaining_emulated | 20,089,445 | 45.179% | 53,626,981 | 61.949% |
| remaining_fallback | 19,556 | 0.044% | 27,924 | 0.032% |

| Completion stage | Calls | Remaining emulated | Zero products (all three bins) | Nonzero zero-addend | Nonzero unit-factor |
|---|---:|---:|---:|---:|---:|
| prefix | 12,851,904 | 43.397% | 37.118% | 19.226% | 0.190% |
| rebuild | 27,513,013 | 41.940% | 33.821% | 24.015% | 0.186% |
| validate | 28,286 | 34.254% | 17.390% | 48.300% | 0.057% |
| mesh | 4,073,532 | 72.750% | 9.534% | 17.649% | 0.067% |
| volume | 5,718,305 | 64.029% | 18.154% | 17.531% | 0.286% |
| recognize | 36,381,346 | 82.120% | 7.479% | 10.363% | 0.016% |

Recognition contains 36,381,346 retained calls: 82.120% are remaining-emulated, while the disjoint unit-factor candidate family is 0.016%. These counts select hypotheses for separate validation; they do not identify the cause of the earlier FMA candidate regression.

Sampling keeps the first at most 16 bit triplets per category between drains, then the first at most 16 per selected stage or recipe root across retained ticks. These are biased witnesses, not random or frequency-weighted samples. Method/root/recipe windows overlap; only outer-stage partitions plus unassigned counts add to the observed total. Counts use decimal strings and operands use unsigned 16-digit hexadecimal strings.

Exact output qualification passed all 12 raw ticks (96 solid instances), one preview/completion pair and comparisons against all 16 frozen c754/292f reference runs. Arenas, mesh hashes/quality, Float64 volume bits, material/inventory, checkpoints and complete feature objects match. This is comparison to previously qualified reference outputs, not a new independent recipe replay.

`counts-and-samples.json` is a compact derivative with retained per-tick stage counts, per-root recognition counts, bounded samples and original raw hashes. The original 18 MB dispatch data and path-bearing capture logs remain local and are not reconstructable from this derivative. The two driver JSON files are exact original bytes; their diagnostic elapsed fields remain excluded. `build-receipt.json`, the source overlay and `output-qualification.json` bind the artifact and checks. Collector source and its unchanged test log are included; the original unit-test source stays local because it contains synthetic absolute-path fixtures.

No branch source, default behavior, tolerance or shipping package was changed by this observer.

The source overlay is stored in `observer-overlay.patch.json` as a lossless UTF-8 `payload` string. Decoding that string reproduces the exact original patch bytes and SHA-256 recorded in the build receipt. The raw patch remains unchanged locally; this encoded public file is not a verbatim file copy. `source-bundle-receipt.json` records the mapping.
