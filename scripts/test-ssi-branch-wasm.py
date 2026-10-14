#!/usr/bin/env python3
"""Execute the independent SSI branch-preservation oracles on wasm32.

This builds a temporary math-only module from the same regression tests run
natively. It does not refresh or certify the committed browser kernel package.
Requires Python 3.11+, the repository Rust toolchain with wasm32-unknown-unknown,
and Node.js.
"""

import hashlib
import json
import os
import shutil
from pathlib import Path
import subprocess
import tempfile
import tomllib


def main():
    repo = Path(__file__).resolve().parents[1]
    oracle = repo / "crates/math/tests/regress_ssi_branch_preservation.rs"
    source = oracle.read_text().replace("#[test]\n", "")
    source += """
#[unsafe(no_mangle)]
pub extern "C" fn verify_ssi_branch_preservation() {
    separate_quadratic_plane_sections_survive_model_scaling();
    separate_sections_keep_identity_under_charts_placement_and_order();
    dense_seeds_do_not_duplicate_the_two_sections();
}
"""
    with tempfile.TemporaryDirectory(prefix="remus-ssi-branch-wasm-") as temporary:
        harness = Path(temporary)
        (harness / "src").mkdir()
        (harness / "src/lib.rs").write_text(source)
        (harness / "Cargo.toml").write_text(
            '[package]\nname="ssi-branch-oracle"\nversion="0.0.0"\nedition="2024"\n'
            '[lib]\ncrate-type=["cdylib"]\n[dependencies]\n'
            f'remus-math={{path={json.dumps(str(repo / "crates/math"))}}}\n'
        )
        # Preserve the repository's dependency versions in this independent
        # harness rather than resolving a different cached dependency set.
        shutil.copyfile(repo / "Cargo.lock", harness / "Cargo.lock")
        target = harness / "target"
        env = dict(os.environ, CARGO_BUILD_JOBS="1", CARGO_TARGET_DIR=str(target))
        subprocess.run(
            ["cargo", "build", "--offline", "--manifest-path", str(harness / "Cargo.toml"),
             "--target", "wasm32-unknown-unknown"],
            cwd=repo, env=env, check=True,
        )
        def identities(lock):
            return {(p["name"], p["version"], p.get("source"))
                    for p in tomllib.loads(lock.read_text())["package"]}
        unexpected = identities(harness / "Cargo.lock") - identities(repo / "Cargo.lock")
        if unexpected != {("ssi-branch-oracle", "0.0.0", None)}:
            raise RuntimeError("WASM oracle dependency versions differ from Cargo.lock")
        module = target / "wasm32-unknown-unknown/debug/ssi_branch_oracle.wasm"
        subprocess.run(
            ["node", "-e", """
const fs = require('node:fs');
const moduleBytes = fs.readFileSync(process.argv[1]);
const moduleObject = new WebAssembly.Module(moduleBytes);
if (WebAssembly.Module.imports(moduleObject).length !== 0) {
  throw new Error('SSI math oracle unexpectedly requires host imports');
}
new WebAssembly.Instance(moduleObject, {}).exports.verify_ssi_branch_preservation();
console.log('SSI branch-preservation wasm32 oracles passed');
""", str(module)], cwd=repo, check=True,
        )
        print("oracle_wasm_sha256=" + hashlib.sha256(module.read_bytes()).hexdigest())
    print("source_lock_sha256=" + hashlib.sha256((repo / "Cargo.lock").read_bytes()).hexdigest())
    print("rustc=" + subprocess.check_output(["rustc", "--version"], cwd=repo, text=True).strip())
    print("rustflags=" + os.environ.get("RUSTFLAGS", ""))
    print("encoded_rustflags=" + os.environ.get("CARGO_ENCODED_RUSTFLAGS", ""))
    print("source_head=" + subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip())
    print("source_dirty=" + str(bool(subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=repo, text=True).strip())).lower())


if __name__ == "__main__":
    main()
