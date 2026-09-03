#!/usr/bin/env bash
set -euo pipefail

# Ratchet on std hash collections in the geometry crates.
#
# `std::collections::HashMap`/`HashSet` seed their hasher per instance, so any
# construction whose output depends on the order it visits map entries — vertex
# welds, shell assembly, triangulation — is nondeterministic run to run.
# `remus_math::det_hash::{DetHashMap, DetHashSet}` exist for exactly that, and
# say so in their own module docs. The files listed in
# scripts/det-hash-grandfather.txt predate the rule and are NOT audited; nothing
# new may join them. Migrate a listed file to `det_hash` (or `BTreeMap`) and
# remove it from the manifest in the same change, so the list only shrinks.
#
# Test files (`tests.rs`, `tests/`) are out of scope: a test's map iteration
# order never feeds a shipped result.
#
# Two rules are inherited from check-remus-rename.sh, which paid for both:
#
#   1. Scan with `git grep`, never `rg`. Ripgrep is NOT installed on the GitHub
#      runner. An `rg` call there exits 127, and a scan whose failure is
#      swallowed reports an empty tree — which for a ratchet means every
#      grandfathered entry reads as newly clean and the gate fails on a tree it
#      never actually read.
#   2. Never let a scan's exit status be interpreted as a result. 0 means found,
#      1 means clean; anything else means the tool failed and the gate must say
#      so loudly rather than vouch for a tree it could not scan.
#
# Pattern note: an earlier revision anchored a trailing `\b` after the closing
# brace of the `{HashMap, HashSet}` import form. `}` followed by `;` is not a
# word boundary, so that form never matched and the manifest under-counted by 24
# files. Match `std::collections::` followed, before the statement's semicolon,
# by either name — which covers the bare, braced, and inline-path forms alike.

cd "$(dirname "$0")/.."

readonly MANIFEST=scripts/det-hash-grandfather.txt
readonly PATTERN='std::collections::[^;]*Hash(Map|Set)'
readonly CRATES=(algo blend check heal offset operations)

pathspecs=()
for crate in "${CRATES[@]}"; do
  pathspecs+=(":(glob)crates/${crate}/src/**/*.rs")
done
pathspecs+=(":(exclude,glob)crates/*/src/**/tests.rs")
pathspecs+=(":(exclude,glob)crates/*/src/**/tests/**")

# Run a search whose only meaningful outcomes are "matched" (0) and "did not
# match" (1). Anything else is the tool failing, which must abort the gate.
scan() {
  local label="$1"
  shift
  local out rc
  set +e
  out=$("$@")
  rc=$?
  set -e
  if ((rc > 1)); then
    echo "check-det-hash: ${label} exited ${rc}; the gate could not run" >&2
    exit 2
  fi
  printf '%s' "$out"
}

# LC_ALL=C on both sides: `comm` requires a shared collation, and the runner's
# locale is not the author's.
current=$(scan "hash-collection scan" \
  git grep -l -E "$PATTERN" -- "${pathspecs[@]}" | LC_ALL=C sort)
listed=$(scan "manifest read" \
  grep -vE '^[[:space:]]*(#|$)' "$MANIFEST" | LC_ALL=C sort)

status=0

new=$(LC_ALL=C comm -23 <(printf '%s\n' "$current") <(printf '%s\n' "$listed"))
if [[ -n $new ]]; then
  echo "VIOLATION: std HashMap/HashSet in geometry crates outside the grandfather manifest:"
  echo "$new" | sed 's/^/  /'
  echo "  Use remus_math::det_hash::{DetHashMap, DetHashSet} (or BTreeMap) instead."
  status=1
fi

gone=$(LC_ALL=C comm -13 <(printf '%s\n' "$current") <(printf '%s\n' "$listed"))
if [[ -n $gone ]]; then
  echo "STALE: listed in ${MANIFEST} but no longer using std hash collections (remove the entry):"
  echo "$gone" | sed 's/^/  /'
  status=1
fi

if [[ $status -eq 0 ]]; then
  count=$(printf '%s\n' "$listed" | grep -c . || true)
  echo "✅ det-hash ratchet OK (${count} grandfathered files, no new std hash use)."
fi

exit "$status"
