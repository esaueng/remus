#!/usr/bin/env bash
set -euo pipefail

# These commits introduced the upstream AGPL line and first merged it into the
# production fork. An Apache release must never descend from either commit.
readonly UPSTREAM_AGPL_COMMIT="bd7d1ba7"
readonly FORK_AGPL_MERGE="8fbaea57"

for forbidden in "$UPSTREAM_AGPL_COMMIT" "$FORK_AGPL_MERGE"; do
  if git cat-file -e "${forbidden}^{commit}" 2>/dev/null \
    && git merge-base --is-ancestor "$forbidden" HEAD; then
    echo "Apache lineage violation: forbidden commit $forbidden is an ancestor of HEAD"
    exit 1
  fi
done

test -f LICENSE-APACHE
test -f NOTICE
test ! -e LICENSE-MIT
test ! -e COMMERCIAL-LICENSE.md
test ! -e .github/CLA.md

if ! grep -Eq '^license = "Apache-2.0"$' Cargo.toml; then
  echo "Cargo workspace license must be Apache-2.0"
  exit 1
fi

if rg -n 'AGPL-3.0-only|MIT OR Apache-2.0|LICENSE-MIT' \
  --glob 'Cargo.toml' \
  --glob 'package.json' \
  --glob '*.toml' \
  --glob '*.json'; then
  echo "Apache lineage violation: incompatible project license metadata found"
  exit 1
fi

python3 scripts/check-apache-replay-provenance.py

echo "Apache lineage and project-license metadata verified."
