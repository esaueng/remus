#!/usr/bin/env python3
"""Verify Cargo's runtime source dependencies against reproduced instrumentation.

Arguments: diagnostic checkout, clean source archive of claimed commit, output.
First run instrument.py on the reference archive using this harness revision.
Paths in the emitted manifest are repository-relative; no local user paths.
"""
import hashlib
import json
import pathlib
import sys

actual, reference, output = map(lambda p: pathlib.Path(p).resolve(), sys.argv[1:])
sources = {}
for dep in [actual / 'target/release/examples/perf-t01.d', actual / 'target/wasm32-unknown-unknown/release/remus_wasm.d']:
    for name in dep.read_text().split(': ', 1)[1].strip().split():
        path = pathlib.Path(name)
        if not path.is_file() or not path.is_relative_to(actual):
            continue
        relative = path.relative_to(actual)
        assert path.read_bytes() == (reference / relative).read_bytes(), str(relative)
        sources[str(relative)] = hashlib.sha256(path.read_bytes()).hexdigest()
manifest = {
    'matched_source_files': len(sources),
    'source_file_manifest_sha256': hashlib.sha256(json.dumps(sources, sort_keys=True).encode()).hexdigest(),
    'native_binary_sha256': hashlib.sha256((actual / 'target/release/examples/perf-t01').read_bytes()).hexdigest(),
    'wasm_module_sha256': hashlib.sha256((actual / 'pkg-probe/remus_wasm_bg.wasm').read_bytes()).hexdigest(),
}
output.write_text(json.dumps(manifest, indent=2) + '\n')
