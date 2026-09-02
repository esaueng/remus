#!/usr/bin/env bash
# bench-compare.sh — Run the O3.1 and reference Criterion suites.
#
# Criterion retains the previous sample in target/criterion and reports the
# statistically significant change on the next run. The retired brepjs/OCCT
# harness is deliberately not invoked here; its historical numbers are not a
# release gate for this repository.
#
# Usage:
#   ./scripts/bench-compare.sh

set -euo pipefail

REMUS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RESULTS_DIR="$REMUS_ROOT/bench-results"
CRITERION_LOG="$RESULTS_DIR/criterion.log"

mkdir -p "$RESULTS_DIR"
: > "$CRITERION_LOG"

run_bench() {
    echo ""
    echo "=== $* ==="
    "$@" 2>&1 | tee -a "$CRITERION_LOG"
}

cd "$REMUS_ROOT"

run_bench cargo bench -p remus-math --benches
run_bench cargo bench -p remus-algo --benches
run_bench cargo bench -p remus-blend --benches --features bench-internals
run_bench cargo bench -p remus-operations --bench boolean_tracking
run_bench cargo bench -p remus-operations --bench boolean_perf
run_bench cargo bench -p remus-operations --bench cad_operations -- gridfinity

echo ""
echo "Native benchmark comparison complete."
echo "  Criterion data: $REMUS_ROOT/target/criterion"
echo "  Combined log:   $CRITERION_LOG"
