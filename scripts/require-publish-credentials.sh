#!/usr/bin/env bash
set -euo pipefail

missing=()
[[ -n "${REMUS_BOT_APP_ID:-}" ]] || missing+=(REMUS_BOT_APP_ID)
[[ -n "${REMUS_BOT_PRIVATE_KEY:-}" ]] || missing+=(REMUS_BOT_PRIVATE_KEY)

if (( ${#missing[@]} > 0 )); then
  printf -v missing_names '%s, ' "${missing[@]}"
  missing_names=${missing_names%, }
  echo "::error title=Missing publish credentials::This release requires repository secrets: ${missing_names}"
  exit 1
fi
