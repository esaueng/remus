#!/usr/bin/env bash
# O1.5 split import/export workflow qualification.
#
# Verifies the real distributed split packages together: the native facade
# (`remus::Model` + `remus_io`) versus freshly packed/installed `remus-wasm`
# plus `remus-wasm-io` direct calls, with a separately identified
# committed-package mode. Bodies cross between the two installed modules as
# exact arena documents only.
#
# Usage:
#   bash scripts/test-o15-split-io.sh [--fresh-only]
#
# Default runs fresh + committed (both evidence modes). --fresh-only skips
# the committed install for quick iteration. The machine-readable report is
# written to $CARGO_TARGET_DIR/o15-split-io.json (normally target/).
set -euo pipefail
cd "$(dirname "$0")/.."

fresh_only=0
for arg in "$@"; do
  case "$arg" in
    --fresh-only) fresh_only=1 ;;
    -h|--help)
      echo "usage: test-o15-split-io.sh [--fresh-only]"
      exit 0
      ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

node --test tools/parity/split-io-matrix.test.mjs
cargo test -p remus-parity --bin remus-split-io-native
cargo build --release -p remus-parity --bin remus-split-io-native

target_dir=$(cargo metadata --no-deps --format-version 1 | node -e 'let s=""; process.stdin.on("data", d => s += d); process.stdin.on("end", () => console.log(JSON.parse(s).target_directory));')
native_runner="$target_dir/release/remus-split-io-native"

# Fresh tarballs are built from this source tree in temp dirs: the committed
# package directories are never overlaid or rewritten. The kernel ships
# without legacy I/O (--no-default-features); the translator carries it.
fresh_kernel_dir=$(mktemp -d /tmp/remus-split-kernel.XXXXXX)
fresh_io_dir=$(mktemp -d /tmp/remus-split-io-translator.XXXXXX)
trap 'rm -rf "$fresh_kernel_dir" "$fresh_io_dir"' EXIT
wasm-pack build crates/wasm --target nodejs --release --no-opt --out-dir "$fresh_kernel_dir" -- --no-default-features
wasm-pack build crates/wasm-io --target nodejs --release --no-opt --out-dir "$fresh_io_dir"

status=0
if [ "$fresh_only" -eq 1 ]; then
  node tools/parity/split-io-matrix.mjs \
    "$native_runner" \
    "$fresh_kernel_dir" \
    "$fresh_io_dir" > "$target_dir/o15-split-io.json" || status=$?
else
  node tools/parity/split-io-matrix.mjs \
    "$native_runner" \
    "$fresh_kernel_dir" \
    "$fresh_io_dir" \
    "crates/wasm/pkg" \
    "crates/wasm-io/pkg" > "$target_dir/o15-split-io.json" || status=$?
fi
echo "split-slice status: $status"
node -e '
const fs = require("fs");
const report = JSON.parse(fs.readFileSync(process.argv[1], "utf8"));
console.log(JSON.stringify({
  passed: report.passed,
  cases: report.matrix.cases,
  failedStages: report.failed_stages.length,
  freshKernel: report.provenance.freshKernel?.packageVersion ?? report.provenance.fresh?.packageVersion ?? null,
  stalenessKernel: report.provenance.stalenessKernel?.verdict ?? null,
  stalenessIo: report.provenance.stalenessIo?.verdict ?? null,
}, null, 2));
' "$target_dir/o15-split-io.json"
exit "$status"
