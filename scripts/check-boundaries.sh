#!/usr/bin/env bash
set -euo pipefail

# Verify crate dependency graph matches layer rules.
#
# Layer rules:
#   L0   (math)       — no workspace deps
#   L1   (geometry)   — depends on math only
#   L1   (topology)   — depends on math only
#   L2   (algo)       — depends on math, topology, geometry
#   L2   (blend)      — depends on math, topology, geometry
#   L2   (heal)       — depends on math, topology, geometry
#   L2   (check)      — depends on math, topology, geometry
#   L2   (offset)     — depends on math, topology, geometry
#   L2   (sketch)     — no workspace deps
#   L3   (operations) — depends on math, topology, algo, blend, heal, check, geometry, offset, sketch
#   L3   (io)         — depends on math, topology, operations
#   L4   (render)     — depends on math, topology, operations
#   L4   (wasm)       — depends on all
#   L4   (wasm-io)    — file-format translator module; depends on io, operations, topology, math
#   L5   (remus)      — native facade; depends on operations, io, check, heal, math, topology, sketch

FAIL=0

check_deps() {
  local crate="$1"
  shift
  local allowed=("$@")

  local cargo_toml="crates/${crate}/Cargo.toml"
  if [ ! -f "$cargo_toml" ]; then
    echo "SKIP: $cargo_toml not found"
    return
  fi

  # Extract [dependencies] section only (dev-dependencies are allowed to
  # reference higher-layer crates for integration tests)
  local deps_section
  deps_section=$(sed -n '/^\[dependencies\]/,/^\[/p' "$cargo_toml" 2>/dev/null || true)

  # Every workspace crate name must be listed here, or a dependency on it is
  # never checked. The remus-ops-* through remus-features names are the planned
  # RFC 0007 split of remus-operations; none exists yet.
  for dep in remus remus-math remus-topology remus-algo remus-blend remus-heal remus-check remus-geometry remus-offset remus-sketch remus-operations remus-io remus-render \
    remus-ops-core remus-transform remus-tessellate remus-measure remus-construct remus-boolean remus-features; do
    if echo "$deps_section" | grep -Eq "^${dep}[[:space:]]*="; then
      local is_allowed=false
      for a in "${allowed[@]}"; do
        if [ "$dep" = "$a" ]; then
          is_allowed=true
          break
        fi
      done
      if [ "$is_allowed" = false ]; then
        echo "VIOLATION: crates/${crate} depends on ${dep} (not allowed)"
        FAIL=1
      fi
    fi
  done
}

echo "Checking crate boundary rules..."

if grep -En 'path[[:space:]]*=[[:space:]]*"[^"]*tools/' crates/*/Cargo.toml; then
  echo "VIOLATION: crates/* must not depend on tools/*"
  FAIL=1
fi

check_deps "math"
check_deps "topology"   "remus-math"
check_deps "geometry"   "remus-math"
check_deps "algo"       "remus-math" "remus-topology" "remus-geometry"
check_deps "blend"      "remus-math" "remus-topology" "remus-geometry"
check_deps "heal"       "remus-math" "remus-topology" "remus-geometry"
check_deps "check"      "remus-math" "remus-topology" "remus-geometry"
check_deps "offset"     "remus-math" "remus-topology" "remus-geometry"
check_deps "sketch"
check_deps "operations" "remus-math" "remus-topology" "remus-algo" "remus-blend" "remus-heal" "remus-check" "remus-geometry" "remus-offset" "remus-sketch"
check_deps "io"         "remus-math" "remus-topology" "remus-operations"
check_deps "render"     "remus-math" "remus-topology" "remus-operations"
check_deps "wasm"       "remus-math" "remus-topology" "remus-algo" "remus-blend" "remus-heal" "remus-check" "remus-geometry" "remus-offset" "remus-sketch" "remus-operations" "remus-io"
check_deps "wasm-io"    "remus-math" "remus-topology" "remus-operations" "remus-io"
check_deps "remus"      "remus-operations" "remus-io" "remus-check" "remus-heal" "remus-math" "remus-topology" "remus-sketch"

# Module-level order inside remus-operations (RFC 0007): keeps the planned
# crate split possible while the code still lives in one crate.
if ! python3 scripts/test-ops-module-graph.py >/dev/null 2>&1; then
  echo "VIOLATION: scripts/test-ops-module-graph.py fails; the module-graph check is untrusted"
  FAIL=1
fi
if ! python3 scripts/check-ops-module-graph.py; then
  FAIL=1
fi

if [ $FAIL -ne 0 ]; then
  echo "❌ Boundary check failed."
  exit 1
fi

echo "✅ All crate boundaries valid."
