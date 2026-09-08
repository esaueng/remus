#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

node --test tools/vs-bench/workflows/w9-preflight.test.mjs
cargo test -p remus-wasm --features workflow-probes --lib workflow_probes
cargo build -p remus-wasm --features workflow-probes --example w9-native
target_dir=$(cargo metadata --no-deps --format-version 1 | node -e 'let s=""; process.stdin.on("data", d => s += d); process.stdin.on("end", () => console.log(JSON.parse(s).target_directory));')
wasm-pack build crates/wasm --dev --target nodejs --out-dir "$target_dir/w9-wasm" -- --features workflow-probes
status=0
node tools/vs-bench/workflows/w9-preflight.mjs \
  "$target_dir/debug/examples/w9-native" \
  "$target_dir/w9-wasm/remus_wasm.js" > "$target_dir/w9-preflight.json" || status=$?
cat "$target_dir/w9-preflight.json"
exit "$status"
