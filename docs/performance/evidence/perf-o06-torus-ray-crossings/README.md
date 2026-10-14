Each lane contains the same measured baseline runtime d78230e1 and its independent
candidate. Timings remain pinned to ae9bfa21 (NURBS) or ad46fac5 (torus), even if
later commits only finalize documentation. Normalized final v61 package archives
were timed: NURBS kernel e4a3ed0a…, torus edd2f2c1…, baseline 9e290c4e…;
the identical translator is a30fd51f… throughout. Both worktrees, tracked source,
harness, native executables and all package files were frozen before/after.

Native builds use `--profile profiling` with identical Rust compiler, default
features and empty Rustflags. Benchmark source is exactly identical for each
baseline/candidate pair. To rebuild a baseline, copy the candidate's maintained
benchmark file into an isolated d78230e1 checkout (register the new torus bench
with `harness = false`); do not copy candidate runtime files. Build with:

```
CARGO_TARGET_DIR="$baseline_target" cargo bench --offline --locked --profile profiling \
  -p remus-math --bench intersection_inner_loops --no-run
CARGO_TARGET_DIR="$baseline_target" cargo bench --offline --locked --profile profiling \
  -p remus-math --bench torus_ray_crossings --no-run
```

After intensive work stops, run the frozen native baseline followed by candidate:

```
CRITERION_HOME="$results/criterion" "$baseline_exe" --bench nurbs_ray_cached --save-baseline baseline
CRITERION_HOME="$results/criterion" "$candidate_exe" --bench nurbs_ray_cached --baseline baseline
```

For torus, use `torus_ray_roots` instead. NURBS has 10 samples/group, 300 ms
warmup and 1 s measurement. Torus has 20 samples/group, 1 s warmup and 2 s
measurement. Native results time isolated ray queries, not recognition or a
browser pipeline. Criterion time estimates and relative-change estimates are
retained verbatim; their estimators differ, so do not derive change percentages
from rounded time estimates. The displayed `p = 0.00` is rounded, not exact zero.

Build both WASM packages together per isolated source using `cargo xtask
wasm-build`, preserve the complete package directories and their hashes, then
run the maintained untraced worker in a separate quiet window:

```
python3 scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py \
  --old-kernel "$baseline_kernel" --old-io "$baseline_io" \
  --new-kernel "$candidate_kernel" --new-io "$candidate_io" \
  --out "$results/wasm" --warmup 2 --samples 6 --rounds 2 --order both --cpu auto
```

This gives 12 retained edits per variant/workflow across two processes. Keep
source and warmups in the raw evidence but exclude them from the reported stage
statistics. The process/JIT and production module-local memos stay warm within
a process; each edit restores a fresh kernel/body. Node kernel and bulk transfer
measurements exclude browser/GPU/presentation. These small correlated samples
are not deployment tail estimates; nearest-rank p95 for n=12 equals the max.

After timing, run separate untimed replay processes and the strict comparator:

```
node scripts/perf/recognition-graph/replay_quantities.cjs "$baseline_kernel" "$baseline_run" > "$baseline_replay"
node scripts/perf/recognition-graph/replay_quantities.cjs "$candidate_kernel" "$candidate_run" > "$candidate_replay"
node scripts/perf/recognition-graph/compare_runs.cjs "$baseline_run" "$candidate_run" \
  "$baseline_replay" "$candidate_replay"
```

Both lanes preserve 36 snapshot pairs (4 source + 8 warmup + 24 retained), 180
arena/display-buffer byte pairs, complete features/volume bits, and 5,760
per-face area-bit pairs plus complete area/bbox/carrier/edge-relation replay.
Raw WASM numbers and native sample arrays are retained. Nine exactly identical
replay geometries are stored once with every process-row reference retained.
Original native logs are exact JSON strings with verified original SHA256,
including their terminal newlines; public files need no whitespace exceptions.

`wasm-samples.json` retains every process's metadata, source readings, warmup and
retained timing records. A record's `qualificationCase` selects its exact full
non-timing fields from `qualificationCases`. Reconstruct a saved record with
`{...record, ...qualificationCases[record.qualificationCase]}` and omit the
reference key; reconstruction was checked against all eight normalized original
results. This removes duplicated geometry/features, not sample values. The nine
complete replay states in `quantity-replays.json` likewise retain every process
row through `processReferences`, along with original replay hashes and policy.
`manifest.json` hashes every public file; JSON native logs preserve exact UTF-8
payloads, including intentional terminal newlines.

Large measurement JSON files use compact formatting to keep the review diff
focused. Their data and exact decoded native log strings are unchanged. Where
`capturedPrettySha256` is present, recover the original captured public file
with Python `json.dumps(json.loads(data), ensure_ascii=False, indent=2,
allow_nan=False) + "\n"`; its UTF-8 hash must equal that field. The manifest
also hashes this README and the separate local gate-qualification file.
