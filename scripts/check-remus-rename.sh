#!/usr/bin/env bash
set -euo pipefail

# Guards the Remus product identity: no stale "brepkit" naming may re-enter the
# tree, in file content or in tracked paths.
#
# Scans run through `git grep`, not `rg`. Ripgrep is NOT installed on the GitHub
# runner, and both scans used to sit in an `if var=$(...)` condition, which
# swallows a command's exit status wholesale: the resulting 127 read as "no
# matches", so the job printed the success line and exited 0 every time. This
# gate had never been able to fail in CI, and a real violation reached `main`
# past it. Two rules keep that from recurring:
#
#   1. Scan with a tool a git checkout is guaranteed to have. `git grep` also
#      gives us pathspec excludes and binary skipping natively, so it needs no
#      install step that could itself be dropped.
#   2. Never let a scan's exit status be interpreted as a result. 0 means found,
#      1 means clean; anything else means the tool failed and the gate must say
#      so loudly rather than vouch for a tree it never read.
#
# Scope note: `git grep` reads tracked files, where the previous `rg` invocation
# read the working tree. That is the right scope — the second check below is
# already tracked-only, a CI checkout has no untracked files, and untracked
# scratch files are not part of the tree's published identity.
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
#   AI-DISCLOSURE-ETHICS.md
#       The maintainer's signed statement of where this project came from and
#       how it is built. Naming the predecessor is the disclosure — the same
#       category as NOTICE, and the file says outright that it is human-written
#       and not for agents to edit. Its references are a record of origin, not
#       naming that failed to get updated.
#
#   scripts/check-remus-rename.sh
#       This file, which must name what it forbids.
readonly ALLOWED_HISTORY=(
  "CHANGELOG.md"
  "crates/wasm/CHANGELOG.md"
  "NOTICE"
  "AI-DISCLOSURE-ETHICS.md"
  "docs/production-readiness/apache-replay-provenance.json"
  "docs/production-readiness/apache-replay-provenance.md"
  "scripts/check-apache-replay-provenance.py"
  "scripts/check-remus-rename.sh"
  "crates/io/tests/data/*.step"
)

# `glob` magic anchors each pattern at the repo root and stops `*` at a `/`, so
# `CHANGELOG.md` exempts the root file without also exempting the one under
# crates/wasm/ (both are listed, deliberately and separately).
exclude_pathspecs=()
for path in "${ALLOWED_HISTORY[@]}"; do
  exclude_pathspecs+=(":(exclude,glob)${path}")
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

readonly STALE='brepkit|brep-kit'

# Run a search whose only meaningful outcomes are "matched" (0) and "did not
# match" (1). Anything else is the tool failing, which must abort the gate —
# never be mistaken for a clean tree. Emits the matches on stdout.
scan() {
  local label="$1"
  shift
  local out rc
  set +e
  out=$("$@")
  rc=$?
  set -e
  if ((rc > 1)); then
    echo "check-remus-rename: ${label} exited ${rc}; the gate could not run" >&2
    exit 2
  fi
  printf '%s' "$out"
}

status=0

content_hits=$(scan "content scan" \
  git grep -n -I -i -E "$STALE" -- "${exclude_pathspecs[@]}")

if [[ -n $content_hits ]]; then
  # -v with several -e patterns drops a line matching ANY of them, so this is
  # the two chained inverse greps the previous implementation ran.
  stale_content=$(scan "content filter" \
    grep -Ev -e "$OTHER_REPOS" -e "$EXTERNAL_IDENTIFIERS" <<<"$content_hits")
  if [[ -n $stale_content ]]; then
    echo "Remus rename violation: stale product identity found in content"
    echo "$stale_content"
    status=1
  fi
fi

tracked=$(git ls-files)
stale_paths=$(scan "tracked-path scan" grep -Ei -e "$STALE" <<<"$tracked")

if [[ -n $stale_paths ]]; then
  echo "Remus rename violation: stale product identity found in tracked paths"
  echo "$stale_paths"
  status=1
fi

if [[ $status -eq 0 ]]; then
  echo "✅ Remus naming OK."
fi

exit "$status"
