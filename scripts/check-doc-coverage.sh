#!/usr/bin/env bash
set -euo pipefail

# The other direction of check-doc-paths.sh.
#
# That script asks "does every path the docs name still exist?". This one asks
# "does every module the tree contains appear in the docs at all?" — and only
# the second catches a whole subsystem being added without ever reaching the
# Module Map. When this was first measured, 27 top-level modules were missing,
# including every part of the RFC-0002/0003 work (topology's coedge, journal,
# naming, attributes, transaction) and io's arena_io, naming_io and limits.
# CLAUDE.md is loaded as ground truth at the start of every session, so a
# subsystem absent from it is one that sessions do not know exists.
#
# A module counts as documented when its file name appears anywhere in the doc,
# OR when its parent directory is named — a row like `pave_filler/phase_*.rs`
# legitimately covers a family without listing each member. The check therefore
# only fires on a module whose whole neighbourhood is unmentioned.
#
# Uses grep, not rg: ripgrep is NOT installed on the GitHub runner, and a gate
# that cannot run there is a gate that always passes (see check-remus-rename.sh,
# which spent its whole life doing exactly that).

readonly DOC="AGENTS.md"

# Never let an unreadable doc, or a find that returns nothing, be mistaken for a
# clean tree — the failure mode that left check-remus-rename.sh inert in CI for
# its whole life. Both abort loudly instead.
if [ ! -r "$DOC" ]; then
  echo "check-doc-coverage: cannot read ${DOC}; the gate could not run" >&2
  exit 2
fi

# Modules that are deliberately not in the map.
#   mod.rs / lib.rs  — the directory row covers them
#   tests.rs         — test-only
readonly SKIP_NAMES=("mod.rs" "lib.rs" "tests.rs")

is_skipped_name() {
  local base="$1"
  for s in "${SKIP_NAMES[@]}"; do
    [ "$base" = "$s" ] && return 0
  done
  return 1
}

missing=()
scanned=0

while IFS= read -r file; do
  scanned=$((scanned + 1))
  base="${file##*/}"
  is_skipped_name "$base" && continue

  # Documented by its own name?
  grep -qF -- "$base" "$DOC" && continue

  # Documented by its directory (e.g. a `foo/bar/` row covering a family)?
  rel="${file#crates/}"                 # <crate>/src/<sub>/<file>
  sub="${rel#*/src/}"
  sub="${sub%/*}"
  if [ "$sub" != "${rel#*/src/}" ]; then
    grep -qF -- "${sub}/" "$DOC" && continue
  fi

  missing+=("$file")
done < <(find crates -path '*/src/*' -name '*.rs' -not -path '*/tests/*' | sort)

if [ "$scanned" -eq 0 ]; then
  echo "check-doc-coverage: found no source modules to check; the gate could not run" >&2
  exit 2
fi

if [ ${#missing[@]} -ne 0 ]; then
  echo "❌ Modules missing from ${DOC}'s Module Map:"
  printf '   %s\n' "${missing[@]}"
  echo
  echo "   Add a row for each under its crate's section, or name its directory"
  echo "   if it belongs to a family the map already covers as a group."
  exit 1
fi

echo "✅ Every module appears in ${DOC}."
