#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
readonly REPO_ROOT
readonly CHECK_SCRIPT="$SCRIPT_DIR/require-publish-credentials.sh"
readonly PUBLISH_WORKFLOW="$REPO_ROOT/.github/workflows/publish.yml"

assert_failure() {
  local expected=$1
  local app_id=${2-}
  local private_key=${3-}
  local output
  local status

  set +e
  output=$(REMUS_BOT_APP_ID="$app_id" REMUS_BOT_PRIVATE_KEY="$private_key" "$CHECK_SCRIPT" 2>&1)
  status=$?
  set -e

  if [[ $status -eq 0 ]]; then
    echo "expected credential check to fail"
    return 1
  fi
  if [[ $output != *"repository secrets: ${expected}"* ]]; then
    echo "unexpected credential error: ${output}"
    return 1
  fi
  if [[ $output == *test-app-id* || $output == *test-private-key* ]]; then
    echo "credential check exposed a secret value"
    return 1
  fi
}

assert_failure "REMUS_BOT_APP_ID, REMUS_BOT_PRIVATE_KEY"
assert_failure "REMUS_BOT_PRIVATE_KEY" test-app-id
assert_failure "REMUS_BOT_APP_ID" "" test-private-key

output=$(REMUS_BOT_APP_ID=test-app-id REMUS_BOT_PRIVATE_KEY=test-private-key "$CHECK_SCRIPT")
if [[ -n $output ]]; then
  echo "successful credential check should be silent"
  exit 1
fi

if grep -Fq '|| github.token' "$PUBLISH_WORKFLOW"; then
  echo "publish workflow must not fall back to github.token"
  exit 1
fi
if ! grep -Fq "GH_TOKEN: \${{ steps.app-token.outputs.token }}" "$PUBLISH_WORKFLOW"; then
  echo "publish workflow must use only the GitHub App token for writes"
  exit 1
fi
if ! grep -Fq 'run: ./scripts/require-publish-credentials.sh' "$PUBLISH_WORKFLOW"; then
  echo "publish workflow must run the credential preflight"
  exit 1
fi
publish_gate_count=$(grep -Fc "if: \${{ steps.package-diff.outputs.needs_publish == 'true' }}" "$PUBLISH_WORKFLOW")
if [[ $publish_gate_count -ne 3 ]]; then
  echo "publish workflow must gate credential validation, token creation, and commit on a package diff"
  exit 1
fi

echo "Publish credential contract OK."
