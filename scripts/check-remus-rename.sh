#!/usr/bin/env bash
set -euo pipefail

# Historical names are retained only where they are required for attribution
# or release provenance. Everything else must use the Remus identity.
readonly ALLOWED_HISTORY=(
  "CHANGELOG.md"
  "crates/wasm/CHANGELOG.md"
  "docs/PROVENANCE.md"
  "docs/production-readiness/apache-replay-provenance.json"
  "docs/production-readiness/apache-replay-provenance.md"
  "NOTICE"
  "scripts/check-apache-replay-provenance.py"
  "scripts/check-remus-rename.sh"
)

search_args=()
for path in "${ALLOWED_HISTORY[@]}"; do
  search_args+=(--glob "!${path}")
done

if stale_content=$(rg -n -i --hidden 'brepkit|brep-kit' "${search_args[@]}"); then
  echo "Remus rename violation: stale product identity found"
  echo "$stale_content"
  exit 1
fi

if stale_paths=$(git ls-files | rg -i 'brepkit|brep-kit'); then
  echo "Remus rename violation: stale product identity found in tracked paths"
  echo "$stale_paths"
  exit 1
fi

test -f LICENSE-APACHE
test ! -e LICENSE-MIT
test "$(readlink CLAUDE.md)" = "AGENTS.md"
grep -Fq 'repository = "https://github.com/esaueng/remus"' Cargo.toml
grep -Fq 'name = "remus-wasm"' crates/wasm/Cargo.toml
grep -Fq '"name": "remus-wasm"' crates/wasm/pkg/package.json
grep -Fq '"url": "https://github.com/esaueng/remus"' \
  crates/wasm/pkg/package.json
grep -Fq 'software developed by the brepkit contributors' NOTICE
grep -Fq 'historical provenance' CHANGELOG.md
grep -Fq 'historical provenance' crates/wasm/CHANGELOG.md
grep -Fq 'The post-license BrepKit line is not part of Remus.' docs/PROVENANCE.md
grep -Fq '"source_repository": "https://github.com/esaueng/brepkit"' \
  docs/production-readiness/apache-replay-provenance.json
grep -Fq 'engineering provenance record' \
  docs/production-readiness/apache-replay-provenance.md

echo "Remus package identity and historical-name allowlist verified."
