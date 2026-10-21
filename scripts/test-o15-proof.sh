#!/usr/bin/env bash
# Bounded O1.5 recurring proof: contract and split-translator slices only.
set -euo pipefail
cd "$(dirname "$0")/.."

if [ "$#" -ne 0 ]; then
  echo 'usage: test-o15-proof.sh (optional O15_PROOF_OUT output directory)' >&2
  exit 2
fi

proof_root=${O15_PROOF_OUT:-target/o15-proof}
mkdir -p "$proof_root"
# Separate invocations cannot reuse an earlier passing report.
proof_dir=$(mktemp -d "$proof_root/run.XXXXXX")
proof_dir=$(cd "$proof_dir" && pwd)
printf 'O1.5 reports: %s\n' "$proof_dir"
printf 'contract\tnot_run\nsplit-io\tnot_run\n' > "$proof_dir/status.tsv"
{
  printf 'RUSTFLAGS=%q\n' "${RUSTFLAGS-}"
  printf 'CARGO_ENCODED_RUSTFLAGS=%q\n' "${CARGO_ENCODED_RUSTFLAGS-}"
  printf 'CARGO_BUILD_TARGET=%q\n' "${CARGO_BUILD_TARGET-}"
  printf 'CARGO_TARGET_DIR=%q\n' "${CARGO_TARGET_DIR-}"
} > "$proof_dir/build-config.txt"
build() {
  printf '%q ' "$@" >> "$proof_dir/build-config.txt"
  printf '\n' >> "$proof_dir/build-config.txt"
  "$@"
}

node --test tools/parity/provenance.test.mjs tools/parity/contract-matrix.test.mjs tools/parity/split-io-matrix.test.mjs tools/parity/proof-verdict.test.mjs
build cargo test --locked --profile ci-test -p remus-parity
build cargo build --locked --release -p remus-parity
target_dir=$(cargo metadata --locked --no-deps --format-version 1 | node -e 'let s=""; process.stdin.on("data", d => s += d); process.stdin.on("end", () => console.log(JSON.parse(s).target_directory));')

fresh_root=$(mktemp -d "${TMPDIR:-/tmp}/remus-o15-proof.XXXXXX")
trap 'rm -rf "$fresh_root"' EXIT
# Both fresh packages come from this checkout and configuration; committed
# distributions remain read-only and separately identified by the harnesses.
build wasm-pack build crates/wasm --target nodejs --release --no-opt --out-dir "$fresh_root/kernel" -- --locked --no-default-features
build wasm-pack build crates/wasm-io --target nodejs --release --no-opt --out-dir "$fresh_root/io" -- --locked

contract_status=0
node tools/parity/contract-matrix.mjs \
  "$target_dir/release/remus-parity-native" "$fresh_root/kernel" \
  crates/wasm/pkg > "$proof_dir/o15-contract.json" || contract_status=$?
# A contract failure must not suppress the independent translator evidence.
split_status=0
node tools/parity/split-io-matrix.mjs \
  "$target_dir/release/remus-split-io-native" "$fresh_root/kernel" "$fresh_root/io" \
  crates/wasm/pkg crates/wasm-io/pkg > "$proof_dir/o15-split-io.json" || split_status=$?
printf 'contract\t%s\nsplit-io\t%s\n' "$contract_status" "$split_status" > "$proof_dir/status.tsv"
cat "$proof_dir/status.tsv"

verdict_status=0
node tools/parity/proof-verdict.mjs "$proof_dir/o15-contract.json" "$proof_dir/o15-split-io.json" > "$proof_dir/verdict.json" || verdict_status=$?
if [ "$contract_status" -ne 0 ] || [ "$split_status" -ne 0 ] || [ "$verdict_status" -ne 0 ]; then
  exit 1
fi
