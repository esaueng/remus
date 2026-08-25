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

# Scanned with `git grep`, not `rg`: ripgrep is not installed on the GitHub
# runner, so this check silently did nothing there — `rg` exited 127, the `if`
# swallowed it, and the incompatible-license scan was skipped on every run while
# the job still reported success. `git grep` is guaranteed present in a checkout;
# 0 means found and 1 means clean, and any other status aborts rather than pass.
# The pathspecs cover the two named manifests, since `*` spans directories here.
set +e
license_hits=$(git grep -n -I -E 'AGPL-3.0-only|MIT OR Apache-2.0|LICENSE-MIT' -- '*.toml' '*.json')
license_rc=$?
set -e
if [ "$license_rc" -gt 1 ]; then
  echo "check-apache-lineage: license metadata scan exited ${license_rc}; the gate could not run" >&2
  exit 2
fi
if [ -n "$license_hits" ]; then
  echo "Apache lineage violation: incompatible project license metadata found"
  echo "$license_hits"
  exit 1
fi

python3 scripts/check-apache-replay-provenance.py

echo "Apache lineage and project-license metadata verified."
