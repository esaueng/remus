#!/usr/bin/env bash
# Keep the existing local entry point; every host uses the portable Node driver.
set -euo pipefail
cd "$(dirname "$0")/.."
exec node scripts/test-o15-proof.mjs "$@"
