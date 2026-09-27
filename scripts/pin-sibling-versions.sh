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
VERSION_MANIFEST="$ROOT/crates/math/Cargo.toml"

WS_VERSION=$(sed -n 's/^version *= *"\([^"]*\)".*/\1/p' "$VERSION_MANIFEST" | head -1)

if [ -z "$WS_VERSION" ]; then
  echo "could not read the package version from $VERSION_MANIFEST"
  exit 1
fi

# Rewrite only workspace sibling entries, leaving third-party requirements and
# the WASM crate manifests untouched.
perl -i -pe '
  s/^(remus(?:-[a-z]+)? *= *\{path *= *"[^"]*", *version *= *")=?[^"]*(")/$1='"$WS_VERSION"'$2/
' "$MANIFEST"

echo "pinned workspace sibling requirements to =$WS_VERSION"

if ! git -C "$ROOT" diff --quiet -- Cargo.toml; then
  git -C "$ROOT" --no-pager diff --stat -- Cargo.toml
fi
