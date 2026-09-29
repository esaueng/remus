#!/usr/bin/env bash
# P-Class 8.1 sustained-campaign driver (local runs and CI alike).
#
# Usage:
#   tools/opseq81-campaign.sh [cases [seed [partition]]]
#
# Deterministic seed partitions: run `tools/opseq81-campaign.sh 256 8478446 0/4`
# .. `3/4` across four runners with zero overlap. Resumable via the
# checkpoint file under the output dir. Every case runs in an isolated child
# process (OPSEQ81_ISOLATE=1) with a wall-clock kill, so a hung kernel costs
# one case, not the campaign.
set -euo pipefail

CASES="${1:-256}"
SEED="${2:-8478446}"
PARTITION="${3:-0/1}"
OUT="${OPSEQ81_OUT:-$PWD/opseq81-findings-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$OUT"

export OPSEQ81_CASES="$CASES"
export OPSEQ81_SEED="$SEED"
export OPSEQ81_PARTITION="$PARTITION"
export OPSEQ81_OUT="$OUT"
export OPSEQ81_CHECKPOINT="$OUT/checkpoint.jsonl"
export OPSEQ81_ISOLATE=1
export OPSEQ81_REVISION="${OPSEQ81_REVISION:-$(git rev-parse --short HEAD 2>/dev/null || echo unknown)}"

PROFILE="${OPSEQ81_PROFILE:-release}"

echo "opseq81 campaign: cases=$CASES seed=$SEED partition=$PARTITION out=$OUT profile=$PROFILE" >&2

# Static grammar-matrix gate first (fast, no kernel): fails before any
# expensive execution when the generator narrows.
cargo test "--$PROFILE" -p remus-operations --test op_seq_81 campaign_coverage -- --nocapture

# The campaign itself. Findings (bundles + per-finding reports + replay
# commands) land under $OUT; the checkpoint lets a re-run resume.
cargo test "--$PROFILE" -p remus-operations --test op_seq_81 bounded_campaign -- --nocapture

echo "opseq81 campaign done: $OUT" >&2
