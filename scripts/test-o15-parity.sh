#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

node --test tools/parity/provenance.test.mjs
node --test tools/parity/contract-matrix.test.mjs
node --test tools/parity/quadric-boolean.test.mjs
node --test tools/parity/parity-matrix.test.mjs
cargo test -p remus-parity
cargo build --release -p remus-parity

target_dir=$(cargo metadata --no-deps --format-version 1 | node -e 'let s=""; process.stdin.on("data", d => s += d); process.stdin.on("end", () => console.log(JSON.parse(s).target_directory));')
wasm_dir=$(mktemp -d /tmp/remus-o15-wasm.XXXXXX)
trap 'rm -rf "$wasm_dir"' EXIT
wasm-pack build crates/wasm --target nodejs --release --no-opt --out-dir "$wasm_dir" -- --no-default-features
# Committed/distributed bytes stay read-only: the contract slice installs
# crates/wasm/pkg into its own disposable consumer for the third evidence
# mode, never overlaying or rewriting the committed directory.
status=0
node tools/parity/quadric-boolean.mjs \
  "$target_dir/release/remus-parity-native" \
  "$wasm_dir" > "$target_dir/o15-parity.json" || status=$?
echo "first-slice status: $status"
status2=0
node tools/parity/parity-matrix.mjs \
  "$target_dir/release/remus-parity-native" \
  "$wasm_dir" > "$target_dir/o15-parity-extended.json" || status2=$?
echo "extended-slice status: $status2"
status3=0
node tools/parity/contract-matrix.mjs \
  "$target_dir/release/remus-parity-native" \
  "$wasm_dir" \
  "crates/wasm/pkg" > "$target_dir/o15-contract.json" || status3=$?
echo "contract-slice status: $status3"
node -e '
const fs = require("fs");
const target = process.argv[1];
for (const name of ["o15-parity.json", "o15-parity-extended.json", "o15-contract.json"]) {
  const report = JSON.parse(fs.readFileSync(`${target}/${name}`, "utf8"));
  const summary = name === "o15-contract.json"
    ? { passed: report.passed, staleness: report.staleness?.verdict ?? null }
    : { passed: report.passed };
  console.log(`${name}: ${JSON.stringify(summary)}`);
}' "$target_dir"
cat "$target_dir/o15-contract.json"
if [ "$status" -ne 0 ]; then exit "$status"; fi
if [ "$status2" -ne 0 ]; then exit "$status2"; fi
exit "$status3"
