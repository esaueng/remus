#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

node --test tools/parity/quadric-boolean.test.mjs
cargo test -p remus-parity
cargo build --release -p remus-parity

target_dir=$(cargo metadata --no-deps --format-version 1 | node -e 'let s=""; process.stdin.on("data", d => s += d); process.stdin.on("end", () => console.log(JSON.parse(s).target_directory));')
wasm_dir=$(mktemp -d /tmp/remus-o15-wasm.XXXXXX)
trap 'rm -rf "$wasm_dir"' EXIT
wasm-pack build crates/wasm --target nodejs --release --no-opt --out-dir "$wasm_dir" -- --no-default-features
status=0
node tools/parity/quadric-boolean.mjs \
  "$target_dir/release/remus-parity-native" \
  "$wasm_dir" > "$target_dir/o15-parity.json" || status=$?
cat "$target_dir/o15-parity.json"
exit "$status"
