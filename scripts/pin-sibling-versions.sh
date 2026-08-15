#!/usr/bin/env bash
set -euo pipefail

# Re-pin every workspace brepkit dependency to the exact brepkit-math crate
# version. The Apache line keeps per-crate versions rather than a shared
# workspace package version, so math is the release-version source of truth
# for the Rust workspace crates.

command -v perl >/dev/null || {
  echo "perl is required but not installed."
  exit 1
}

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MANIFEST="$ROOT/Cargo.toml"
VERSION_MANIFEST="$ROOT/crates/math/Cargo.toml"

WS_VERSION=$(sed -n 's/^version *= *"\([^"]*\)".*/\1/p' "$VERSION_MANIFEST" | head -1)

if [ -z "$WS_VERSION" ]; then
  echo "could not read the package version from $VERSION_MANIFEST"
  exit 1
fi

# Rewrite only workspace sibling entries, leaving third-party requirements and
# the independently versioned WASM package untouched.
perl -i -pe '
  s/^(brepkit-[a-z]+ *= *\{path *= *"[^"]*", *version *= *")=?[^"]*(")/$1='"$WS_VERSION"'$2/
' "$MANIFEST"

echo "pinned workspace sibling requirements to =$WS_VERSION"

if ! git -C "$ROOT" diff --quiet -- Cargo.toml; then
  git -C "$ROOT" --no-pager diff --stat -- Cargo.toml
fi
