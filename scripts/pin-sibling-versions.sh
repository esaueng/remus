#!/usr/bin/env bash
set -euo pipefail

# Re-pin every workspace remus dependency, including the facade crate, to the
# exact remus-math version. Math is the source of truth for the Rust release
# train; the generated WASM npm packages may advance patches between cuts.

command -v perl >/dev/null || {
  echo "perl is required but not installed."
  exit 1
}

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MANIFEST="$ROOT/Cargo.toml"
WASM_MANIFEST="$ROOT/crates/wasm/Cargo.toml"
VERSION_MANIFEST="$ROOT/crates/math/Cargo.toml"

WS_VERSION=$(sed -n 's/^version *= *"\([^"]*\)".*/\1/p' "$VERSION_MANIFEST" | head -1)

if [ -z "$WS_VERSION" ]; then
  echo "could not read the package version from $VERSION_MANIFEST"
  exit 1
fi

# Rewrite sibling path dependencies, including the WASM crate's direct
# remus-io requirement, without changing third-party or package versions.
perl -i -pe '
  s/^(remus(?:-[a-z]+)? *= *\{path *= *"[^"]*", *version *= *")=?[^"]*(")/$1='"$WS_VERSION"'$2/
' "$MANIFEST" "$WASM_MANIFEST"

echo "pinned workspace sibling requirements to =$WS_VERSION"

if ! git -C "$ROOT" diff --quiet -- Cargo.toml crates/wasm/Cargo.toml; then
  git -C "$ROOT" --no-pager diff --stat -- Cargo.toml crates/wasm/Cargo.toml
fi
