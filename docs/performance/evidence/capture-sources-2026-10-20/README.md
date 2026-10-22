# Sources for the 2026-10-20 captures

These `.cjs.txt` files preserve the exact runtime source bytes identified by
the [parameter-rebuild provenance](../parameter-rebuild-2026-10-20/provenance.json)
and [face-drag provenance](../face-drag-session-2026-10-20/provenance.json).
The maintained drivers subsequently added guards requiring a retained sample
after cold/warmup exclusions. The historical commands use valid counts:
10 ticks / 2 warmups for parameter rebuild and 24 / 2 for face drag.
Use the maintained drivers for general benchmark work.

[manifest.json](manifest.json) binds the snapshots, immutable inputs and
original provenance files. `sourceCommit` identifies where each matching
source was recovered; it is not the capture timestamp or the package's build
source. Existing provenance, raw records and measured values remain unchanged.

Check archive hashes, evidence links and materialized driver behavior from the
repository root with `node scripts/perf/capture_sources.test.cjs`.

Run the following from the repository root. It verifies every archived source,
input and recorded evidence file before writing a temporary replay tree. It
also checks the supplied kernel/IO package bytes against the separately
recorded distribution verification. The example uses the checkout's package
directories; if they differ, point the variables to the captured distribution
`0d942168550b6dd5101d049f7a10de0230bb5848` instead.

```sh
capture_root=$(mktemp -d /tmp/remus-captures-20261020.XXXXXX)
captured_kernel=$(realpath crates/wasm/pkg)
captured_io=$(realpath crates/wasm-io/pkg)
python3 - "$capture_root" "$captured_kernel" "$captured_io" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

repo = Path.cwd()
archive = repo / 'docs/performance/evidence/capture-sources-2026-10-20'
manifest = json.loads((archive / 'manifest.json').read_text())
target, kernel, io = (Path(arg).resolve() for arg in sys.argv[1:])
if not target.is_dir() or any(target.iterdir()):
    raise SystemExit('Use an empty temporary directory')

def checked(path, expected):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected:
        raise SystemExit(f'Capture hash mismatch: {path}')
    return data

sources = {source['path']: source for source in manifest['sources']}
files = [(source['path'], checked(archive / source['snapshot'], source['sha256']))
         for source in manifest['sources']]
files += [(item['path'], checked(repo / item['path'], item['sha256']))
          for item in manifest['inputs']]
for capture in manifest['captures']:
    provenance_path = repo / capture['provenance']
    provenance = json.loads(checked(provenance_path, capture['provenanceSha256']))
    hashes = provenance.get('evidenceSha256', provenance.get('evidenceFiles'))
    for name, expected in hashes.items():
        checked(provenance_path.parent / name, expected)
    runs = json.loads((repo / capture['runs']).read_text())
    expected_driver = sources[capture['driverPath']]['sha256']
    if any(run['metadata']['harnessSha256'] != expected_driver for run in runs):
        raise SystemExit(f'Capture driver mismatch: {capture["name"]}')
package_record = repo / 'docs/performance/evidence/face-drag-session-2026-10-20/package-verification.json'
for item in json.loads(package_record.read_text())['packages']:
    directory = io if item['file'].startswith('crates/wasm-io/pkg/') else kernel
    checked(directory / Path(item['file']).name, item['sha256'])
for relative, data in files:
    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(data)
    destination.chmod(0o644)
print(f'Verified capture tree: {target}')
PY
```

The copied files retain the original relative module layout and file names,
so imports and the drivers' `__filename` hash reads use the captured bytes.
Run each recorded case in a separate process, using a reserved core as
described in the reports. For example, with the recorded logical CPU 31:

```sh
taskset -c 31 node "$capture_root/scripts/perf/parameter-rebuild/parameter_bench.cjs" \
  --kernel-pkg "$captured_kernel" --out "$capture_root/parameter-preview.json" \
  --rebuild full --mode preview --pattern unique --ticks 10 --warmup 2 \
  --package-source-sha 62626997c6584a531b3c8f3c9103fe5b73c7e137
taskset -c 31 node "$capture_root/scripts/perf/face-drag-session/drag_session_bench.cjs" \
  --kernel-pkg "$captured_kernel" --io-pkg "$captured_io" \
  --fixture "$capture_root/crates/io/tests/data/shapr3d_hammer_holder.step" \
  --out "$capture_root/face-preview.json" --ticks 24 --warmup 2 \
  --move prepared --pattern unique --timing-status measured \
  --package-source-sha 62626997c6584a531b3c8f3c9103fe5b73c7e137
```

Repeat the other modes and request patterns from the original provenance.
These commands recreate the captured code/input/package identity and produce
new observations. They do not regenerate the historical timing or memory
values, establish a speedup, or add a new qualification result.
