#!/usr/bin/env bash
# Ratchet on std hash collections in the geometry crates.
#
# `std::collections::HashMap`/`HashSet` seed their hasher per instance, so any
# construction whose output depends on the order it visits map entries — vertex
# welds, shell assembly, triangulation — is nondeterministic run-to-run.
# `remus_math::det_hash::{DetHashMap, DetHashSet}` exist for exactly that. The
# files listed in scripts/det-hash-grandfather.txt predate the rule and are NOT
# audited; nothing new may join them. Migrate a listed file to `det_hash` (or
# `BTreeMap`) and remove it from the manifest in the same change.
#
# Test files (`tests.rs`, `tests/`) are out of scope: a test's map order never
# feeds a shipped result.
set -euo pipefail

cd "$(dirname "$0")/.."
MANIFEST=scripts/det-hash-grandfather.txt
CRATES=(algo blend check heal offset operations)
PATTERN='std::collections::(\{[^}]*\b(HashMap|HashSet)\b[^}]*\}|HashMap|HashSet)\b'

paths=()
for c in "${CRATES[@]}"; do paths+=("crates/$c/src"); done

current=$(rg -l --glob '*.rs' -e "$PATTERN" "${paths[@]}" \
  | grep -vE '/tests\.rs$|/tests/' | sort || true)
listed=$(grep -vE '^\s*(#|$)' "$MANIFEST" | sort)

status=0
new=$(comm -23 <(echo "$current") <(echo "$listed"))
if [ -n "$new" ]; then
  echo "VIOLATION: std HashMap/HashSet in geometry crates outside the grandfather manifest:"
  echo "$new" | sed 's/^/  /'
  echo "  Use remus_math::det_hash::{DetHashMap, DetHashSet} (or BTreeMap) instead."
  status=1
fi
gone=$(comm -13 <(echo "$current") <(echo "$listed"))
if [ -n "$gone" ]; then
  echo "STALE: listed in $MANIFEST but no longer using std hash collections (remove the entry):"
  echo "$gone" | sed 's/^/  /'
  status=1
fi
if [ "$status" -eq 0 ]; then
  echo "✅ det-hash ratchet OK ($(echo "$listed" | grep -c . ) grandfathered files, no new std hash use)."
fi
exit "$status"
