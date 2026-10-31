import os, subprocess, sys, pathlib
sys.path.insert(0, os.path.dirname(__file__)); from perf_cases import cases
pkg = pathlib.Path(sys.argv[1]); env = dict(os.environ, WASM_SPLIT_PROFILE_DIR=sys.argv[2])
for name, cmd in cases(pkg, 1, 1):
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    print(name, 'ok' if r.returncode == 0 else f'FAIL {r.returncode} {r.stderr[-300:]}', flush=True)
