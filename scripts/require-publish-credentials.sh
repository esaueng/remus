#!/usr/bin/env bash
set -euo pipefail

# The App ID is an identifier, not a credential — GitHub displays it in plain
# text — so it lives in a repository *variable* and is read via `vars`. Only
# the private key is a secret. Naming the store per item matters: a value put
# in the wrong tab reads as empty to the other context, which is the exact
# misconfiguration this check exists to report.
missing=()
[[ -n "${REMUS_BOT_APP_ID:-}" ]] || missing+=("REMUS_BOT_APP_ID (repository variable)")
[[ -n "${REMUS_BOT_PRIVATE_KEY:-}" ]] || missing+=("REMUS_BOT_PRIVATE_KEY (repository secret)")

if (( ${#missing[@]} > 0 )); then
  printf -v missing_names '%s, ' "${missing[@]}"
  missing_names=${missing_names%, }
  echo "::error title=Missing publish credentials::This release requires: ${missing_names}"
  exit 1
fi
