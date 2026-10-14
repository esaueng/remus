# Recognition graph diagnostics

These tools profile recognition inside the maintained holder face-edit workflow.
They do not change kernel policy or contain performance thresholds.
`profile_recognition.cjs` observes retained recognition calls only; source and
warmup queries remain unobserved. CPU/count modes require
`--timing-status contended-smoke-excluded` and reject measured mode before
loading packages. Run untraced timing separately after intensive work stops.

The [source-32 report](../../../docs/performance/perf-o06-recognition-graph.md)
contains the actual package/source identities, raw evidence and qualifications.
Its four timed processes used the worker from `32d39948` (SHA256
`22103aafc45cbf1d23cf992b2861370e7223da32d0dc5960e1fc4439ab543b79`).
The current worker's default path is separately qualified against that version;
the observer is inactive unless this diagnostic CLI supplies it.

## Fresh packages and shipping CPU samples

Use an isolated checkout of the desired source. Build both packages together
and save their complete package directories before instrumentation. Record the
source SHA, clean `git status --porcelain` and `git diff HEAD --name-only`, tool
versions and every package file hash. A committed package is not proof it was
built from the current source.

```sh
recognition_artifacts=$(mktemp -d -t remus-recognition.XXXXXX)
CARGO_BUILD_JOBS=1 BINARYEN_CORES=8 CARGO_TARGET_DIR="$recognition_artifacts/target" \
  cargo xtask wasm-build
cp -R crates/wasm/pkg "$recognition_artifacts/kernel"
cp -R crates/wasm-io/pkg "$recognition_artifacts/io"
node scripts/perf/recognition-graph/profile_recognition.cjs \
  --kernel-pkg "$recognition_artifacts/kernel" --io-pkg "$recognition_artifacts/io" \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --out "$recognition_artifacts/cpu" --warmup 2 --samples 2 \
  --order geometry-first --timing-status contended-smoke-excluded
```

For symbol attribution, retain the raw Rust WASM from the canonical target.
Use wasm-bindgen 0.2.126 and the same wasm-opt as the canonical build (117 for the
recorded evidence). The following creates names without changing the profiled
shipping module. `summarize_wasm_profile.py` refuses mapping unless removing
only the custom name section produces exact shipping bytes. It extracts function
indices from that same qualified name section and reports missing mappings,
including zero. Inclusive sample shares overlap; they are not additive.

```sh
wasm-bindgen "$recognition_artifacts/target/wasm32-unknown-unknown/release/remus_wasm.wasm" \
  --target nodejs --keep-debug --out-dir "$recognition_artifacts/named" --out-name remus_wasm
BINARYEN_CORES=8 wasm-opt "$recognition_artifacts/named/remus_wasm_bg.wasm" \
  -Oz --enable-simd -g --symbolmap -o "$recognition_artifacts/named-optimized.wasm" \
  > "$recognition_artifacts/function-map.txt"
python3 scripts/perf/recognition-graph/summarize_wasm_profile.py \
  --named-wasm "$recognition_artifacts/named-optimized.wasm" \
  --shipping-wasm "$recognition_artifacts/kernel/remus_wasm_bg.wasm" \
  --profiles "$recognition_artifacts/cpu" --out "$recognition_artifacts/attribution.json"
```

## Observation-only work counts

The report's exact seven-file instrumentation patch is preserved as a JSON
string with its original SHA256 in
`docs/performance/evidence/perf-o06-recognition-graph/diagnostic-overlay.json`.
It applies to the source-32 runtime, including this PR's unchanged runtime files.
Extract it, verify its SHA, and apply it in an isolated source checkout. Preserve
original source hashes and the resulting diff hash. Restore those seven files
after capture; the patch is not a proposed production change.

```sh
python3 - "$recognition_artifacts/overlay.patch" <<'PY'
import hashlib, json, pathlib, sys
v = json.loads(pathlib.Path('docs/performance/evidence/perf-o06-recognition-graph/diagnostic-overlay.json').read_text())
payload = v['patch'].encode()
assert hashlib.sha256(payload).hexdigest() == v['sha256']
pathlib.Path(sys.argv[1]).write_bytes(payload)
PY
git apply "$recognition_artifacts/overlay.patch"
```

Build the diagnostic kernel with the canonical Rust flags but without wasm-opt.
This is for deterministic counts and separate phase observations only; its
binary and durations must not be presented as the shipping build. The translator
snapshot is unchanged. The snippet uses the installed pinned wasm-bindgen CLI.

```sh
python3 - "$recognition_artifacts" <<'PY'
import os, pathlib, shutil, subprocess, sys
root = pathlib.Path.cwd()
artifacts = pathlib.Path(sys.argv[1])
sysroot = subprocess.check_output(['rustc', '--print', 'sysroot'], text=True).strip()
cargo_home = pathlib.Path(os.environ.get('CARGO_HOME', str(pathlib.Path.home() / '.cargo')))
flags = ['-Dwarnings', f'--remap-path-prefix={root}=/remus',
         f'--remap-path-prefix={cargo_home}=/cargo', f'--remap-path-prefix={sysroot}=/rust',
         '-C', 'target-feature=+simd128']
env = os.environ.copy()
env.pop('RUSTFLAGS', None)
env.update(CARGO_BUILD_JOBS='1', CARGO_TARGET_DIR=str(artifacts / 'target'),
           CARGO_ENCODED_RUSTFLAGS='\x1f'.join(flags))
subprocess.run(['cargo', 'build', '--offline', '--locked', '--release', '--target',
                'wasm32-unknown-unknown', '-p', 'remus-wasm', '--no-default-features'],
               env=env, check=True)
out = artifacts / 'count-kernel'
subprocess.run(['wasm-bindgen', str(artifacts / 'target/wasm32-unknown-unknown/release/remus_wasm.wasm'),
                '--target', 'nodejs', '--out-dir', str(out), '--out-name', 'remus_wasm'], check=True)
(out / 'remus_wasm.js').rename(out / 'remus_wasm_node.cjs')
shutil.copyfile(artifacts / 'kernel/package.json', out / 'package.json')
PY
node scripts/perf/recognition-graph/profile_recognition.cjs --mode counts \
  --kernel-pkg "$recognition_artifacts/count-kernel" --io-pkg "$recognition_artifacts/io" \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --out "$recognition_artifacts/counts" --warmup 2 --samples 2 \
  --order geometry-first --timing-status contended-smoke-excluded
```

Count mode requires the overlay's explicit counts/notes object, and rejects the
production trace array. Before interpreting counters, compare every saved arena
and mesh buffer to a matched untraced shipping run, plus complete feature JSON,
Float64 volume bits, watertightness, face counts, radii and cap placement. The
recorded report performs those comparisons and records the diagnostic kernel's
own source/diff/binary hashes. No shipping function-index map applies to it.

## Untraced timing and checks

Restore the observation-only source patch after capturing counts:

```sh
git apply --reverse "$recognition_artifacts/overlay.patch"
```

Use the original maintained worker directly, with no observer. Run separate fresh
processes for each schedule, retain warmups/raw samples, alternate schedule order,
and preserve normal cache policy. Repeat source/package/input hashes and
`git diff HEAD --name-only` plus `git status --porcelain` after timing. Measure
browser presentation separately from this Node kernel/transfer pipeline.

```sh
node scripts/perf/filleted-face-edit/wasm_edit_bench.cjs \
  --kernel-pkg "$recognition_artifacts/kernel" --io-pkg "$recognition_artifacts/io" \
  --fixture crates/io/tests/data/shapr3d_hammer_holder.step \
  --out "$recognition_artifacts/untraced" --warmup 2 --samples 6 \
  --order geometry-first --timing-status measured
node scripts/perf/recognition-graph/profile_recognition.test.cjs
python3 scripts/perf/recognition-graph/test_summarize_wasm_profile.py
```

The report's native workload uses the unchanged
`cargo build --profile profiling -p remus --example recognition_perf` executable,
then two serial processes with `RECOG_PERF_RUNS=8` and separate `RECOG_PERF_DUMP`
directories. Retain the first two readings as warmup evidence and exclude them
from each API's six retained readings. Its native default memos are disabled,
printed precision is 0.1 ms, and it performs source queries rather than edited
WASM workflows; do not derive a cross-backend speed ratio from those readings.
