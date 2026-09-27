#!/usr/bin/env bash
set -euo pipefail

fixture=$(mktemp -d)
trap 'rm -rf "$fixture"' EXIT
mkdir -p "$fixture/scripts" "$fixture/crates/math" "$fixture/crates/wasm"
cp "$(dirname "$0")/pin-sibling-versions.sh" "$fixture/scripts/"
git -C "$fixture" init -q

cat > "$fixture/crates/math/Cargo.toml" <<'EOF'
[package]
name = "remus-math"
version = "2026.2.0"
EOF
cat > "$fixture/Cargo.toml" <<'EOF'
remus = {path = "crates/remus", version = "=2026.1.0"}
remus-io = {path = "crates/io", version = "=2026.1.0"}
external = {path = "external", version = "=1.0.0"}
EOF
cat > "$fixture/crates/wasm/Cargo.toml" <<'EOF'
[package]
name = "remus-wasm"
version = "2026.1.0"

[dependencies]
remus-io = {path = "../io", version = "=2026.1.0", default-features = false}
EOF

bash "$fixture/scripts/pin-sibling-versions.sh" >/dev/null
grep -Fqx 'remus = {path = "crates/remus", version = "=2026.2.0"}' "$fixture/Cargo.toml"
grep -Fqx 'remus-io = {path = "crates/io", version = "=2026.2.0"}' "$fixture/Cargo.toml"
grep -Fqx 'external = {path = "external", version = "=1.0.0"}' "$fixture/Cargo.toml"
grep -Fqx 'remus-io = {path = "../io", version = "=2026.2.0", default-features = false}' "$fixture/crates/wasm/Cargo.toml"
grep -Fqx 'version = "2026.1.0"' "$fixture/crates/wasm/Cargo.toml"
