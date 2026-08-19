#!/usr/bin/env bash
set -euo pipefail

# Guards the Remus product identity: no stale "brepkit" naming may re-enter the
# tree, in file content or in tracked paths.
#
# The exemptions below are not laziness — each is a place where the historical
# name is the correct name:
#
#   CHANGELOG.md, crates/wasm/CHANGELOG.md
#       Release history. Their links resolve to commits that really do live in
#       the predecessor repository; rewriting them would break real references.
#
#   NOTICE
#       Attribution. A NOTICE file credits the contributors it names, and
#       renaming them defeats its purpose.
#
#   apache-replay-provenance.{md,json}, check-apache-replay-provenance.py
#       The lineage ledger. It pins the upstream repository, the fork cutoff,
#       and per-PR replay records by name and SHA. These are historical facts.
#
#   crates/io/tests/data/*.step
#       Captured fixtures. These are byte-faithful records of files real
#       exporters produced, headers included. Editing one makes it a forgery
#       rather than a fixture.
#
#   scripts/check-remus-rename.sh
#       This file, which must name what it forbids.
readonly ALLOWED_HISTORY=(
  "CHANGELOG.md"
  "crates/wasm/CHANGELOG.md"
  "NOTICE"
  "docs/production-readiness/apache-replay-provenance.json"
  "docs/production-readiness/apache-replay-provenance.md"
  "scripts/check-apache-replay-provenance.py"
  "scripts/check-remus-rename.sh"
  "crates/io/tests/data/*.step"
)

search_args=()
for path in "${ALLOWED_HISTORY[@]}"; do
  search_args+=(--glob "!${path}")
done

# References that name OTHER repositories, not this project's own identity.
# The release-flow skill deliberately warns readers not to confuse this repo
# with `esaueng/brepkit`, a separate fork of `andymai/brepkit` checked out
# alongside it. Rewriting those would erase the very distinction the warning
# exists to draw. Matched as whole repo/path references, so ordinary stale
# prose in the same file is still caught.
readonly OTHER_REPOS='esaueng/brepkit|andymai/brepkit|/claude/brepkit'

# Identifiers owned by the downstream brepjs repository: its type-sync script,
# its generated types module, its adapter directory, and the kernel id its
# benchmark harness uses. They are named over there, so renaming them here
# would have these docs describe a script and paths that do not exist. They
# lose the old name when brepjs renames them, not before.
readonly EXTERNAL_IDENTIFIERS='sync-brepkit-types|sync:brepkit-types|brepkitWasmTypes|src/kernel/brepkit/|Brepkit\*|BREPJS_KERNEL=brepkit'

status=0

if stale_content=$(rg -n -i --hidden --glob '!.git' 'brepkit|brep-kit' "${search_args[@]}" 2>/dev/null \
  | rg -v "$OTHER_REPOS" | rg -v "$EXTERNAL_IDENTIFIERS"); then
  echo "Remus rename violation: stale product identity found in content"
  echo "$stale_content"
  status=1
fi

if stale_paths=$(git ls-files | rg -i 'brepkit|brep-kit' 2>/dev/null); then
  echo "Remus rename violation: stale product identity found in tracked paths"
  echo "$stale_paths"
  status=1
fi

if [[ $status -eq 0 ]]; then
  echo "✅ Remus naming OK."
fi

exit "$status"
