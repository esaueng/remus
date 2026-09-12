#!/usr/bin/env bash
set -euo pipefail

# Ratchet on wildcard match arms over the kernel's geometry enums.
#
# A `match` over `EdgeCurve` (`topology/src/edge.rs`) or `FaceSurface`
# (`topology/src/face.rs`) that carries a `_ =>` arm compiles clean when a new
# variant lands and silently routes it down the approximate branch — a face
# skipped, an exact path declined for a mesh fallback, a surface treated as
# non-planar. Most of these arms are a deliberate "anything else is not my
# special case" (`_ => None`, `_ => false`, `_ => return Ok(None)`), which is
# why they exist; the gate does not judge them, it pins them. Every arm
# counted here was hand-audited for B24 (`docs/kernel-maturity/roadmap.md`):
# converting one to exhaustive matches (or deleting it) must shrink its file's
# count in the SAME change, so the manifest below only ever records reviewed
# reductions, never silent growth.
#
# Test files (`tests.rs`, `tests/`) are out of scope, exactly as in
# `check-det-hash.sh`: a test's fallback never feeds a shipped result.
#
# Two rules are inherited from check-remus-rename.sh, which paid for both:
#
#   1. Scan with `git grep`, never `rg`. Ripgrep is NOT installed on the GitHub
#      runner. An `rg` call there exits 127, and a scan whose failure is
#      swallowed reports an empty tree — which for a ratchet means every
#      baselined entry reads as newly clean and the gate fails on a tree it
#      never actually read.
#   2. Never let a scan's exit status be interpreted as a result. 0 means found,
#      1 means clean; anything else means the tool failed and the gate must say
#      so loudly rather than vouch for a tree it could not scan.
#
# Counting rule: a `_ =>` arm counts when it opens its own line inside a
# `match` block whose arm PATTERNS mention `EdgeCurve::` or `FaceSurface::`.
# Pattern-side only — an `EdgeCurve::` constructor in an arm BODY (e.g. a
# `RecognizedCurve` match that rebuilds an `EdgeCurve::Line`) does not make
# the block an enum match, and neither do matches over tuples, slices,
# `Result`s, `AnalyticSurface`, or `OperationsError` that merely sit near one.
# A bare binding catch-all (`surface =>`) is not spelled `_ =>` and is not
# counted; three remain in `algo/src/pave_filler/phase_ff.rs` (`curve =>` x2,
# `other =>`) plus one in `operations/src/resize_blend.rs` (`surface =>`) and
# are tracked for B24 follow-up, not by this gate.
#
# Manifest format: `<count> <path>`, one line per production file with at
# least one arm, `LC_ALL=C sort`ed. Growth (observed above baseline, including
# a previously unlisted file) fails; shrinkage without a manifest update in
# the same change fails too, so reductions are always recorded where they land.

cd "$(dirname "$0")/.."

readonly MANIFEST=scripts/wildcard-arms-baseline.txt

# Run a search whose only meaningful outcomes are "matched" (0) and "did not
# match" (1). Anything else is the tool failing, which must abort the gate.
scan() {
  local label="$1"
  shift
  local out rc
  set +e
  out=$("$@")
  rc=$?
  set -e
  if ((rc > 1)); then
    echo "check-wildcard-arms: ${label} exited ${rc}; the gate could not run" >&2
    exit 2
  fi
  printf '%s' "$out"
}

# Per-file counter (stdin = one Rust source file; prints the arm count).
# A stack of `match` frames tracks brace/paren/bracket depth plus a
# pattern-vs-body state per frame: `=>` at the direct level closes a pattern
# (recording whether it mentioned an enum), `,` at the direct level or the
# close of a body block reopens pattern position, and a `_ =>` arm counts only
# in pattern position at the direct level of a frame whose patterns mentioned
# an enum. Strings, character literals, and line/block comments are stripped
# first so braces inside them cannot shift the depth tracking.
count_arms() {
  awk '
    function push_frame() {
      nstack++
      fb[nstack] = 0; fp[nstack] = 0; fq[nstack] = 0
      fstate[nstack] = 0; fseg[nstack] = 0; fhas[nstack] = 0; fwild[nstack] = 0
    }
    function pop_frame() {
      if (fhas[nstack]) total += fwild[nstack]
      nstack--
    }
    BEGIN { nstack = 0; total = 0; in_block = 0; pending = 0 }
    {
      clean = ""
      i = 1
      in_str = 0
      while (i <= length($0)) {
        c = substr($0, i, 1)
        nx = substr($0, i+1, 1)
        if (in_block) {
          if (c == "*" && nx == "/") { in_block = 0; i += 2 } else { i++ }
        } else if (in_str) {
          if (c == "\\") { i += 2 } else if (c == "\"") { in_str = 0; i++ } else { i++ }
        } else {
          if (c == "/" && nx == "/") { break }
          else if (c == "/" && nx == "*") { in_block = 1; i += 2 }
          else if (c == "\"") { in_str = 1; i++ }
          else if (c == SQ && substr($0, i+2, 1) == SQ) { clean = clean "   "; i += 3 }
          else { clean = clean c; i++ }
        }
      }
      if (clean ~ /^[ \t]*_[ \t]*=>/ && nstack > 0 && fb[nstack] == 0 && fp[nstack] == 0 && fq[nstack] == 0 && fstate[nstack] == 0) {
        fwild[nstack]++
      }
      j = 1
      L = length(clean)
      while (j <= L) {
        if (substr(clean, j, 5) == "match" && substr(clean, j-1, 1) !~ /[A-Za-z0-9_]/ && substr(clean, j+5, 1) !~ /[A-Za-z0-9_]/) {
          pending++
          j += 5
          continue
        }
        c = substr(clean, j, 1)
        nx = substr(clean, j+1, 1)
        if (c == "{") {
          if (pending > 0) { pending--; push_frame() }
          else if (nstack > 0) { fb[nstack]++ }
          j++
        } else if (c == "}") {
          if (nstack > 0 && fb[nstack] == 0 && fp[nstack] == 0 && fq[nstack] == 0) { pop_frame() }
          else if (nstack > 0 && fb[nstack] > 0) {
            fb[nstack]--
            if (fb[nstack] == 0 && fstate[nstack] == 1) fstate[nstack] = 0
          }
          j++
        } else if (c == "(") { if (nstack > 0) fp[nstack]++; j++ }
        else if (c == ")") { if (nstack > 0 && fp[nstack] > 0) fp[nstack]--; j++ }
        else if (c == "[") { if (nstack > 0) fq[nstack]++; j++ }
        else if (c == "]") { if (nstack > 0 && fq[nstack] > 0) fq[nstack]--; j++ }
        else if (c == "=" && nx == ">") {
          if (nstack > 0 && fb[nstack] == 0 && fp[nstack] == 0 && fq[nstack] == 0 && fstate[nstack] == 0) {
            if (fseg[nstack]) fhas[nstack] = 1
            fseg[nstack] = 0
            fstate[nstack] = 1
          }
          j += 2
        } else if (c == ",") {
          if (nstack > 0 && fb[nstack] == 0 && fp[nstack] == 0 && fq[nstack] == 0) fstate[nstack] = 0
          j++
        } else if (substr(clean, j, 11) == "EdgeCurve::" || substr(clean, j, 13) == "FaceSurface::") {
          if (nstack > 0 && fstate[nstack] == 0) fseg[nstack] = 1
          j++
        } else { j++ }
      }
    }
    END { print total + 0 }
  '
}

# `glob` magic (as in check-det-hash.sh): `*` stops at `/`, and the bare
# pathspec form would silently skip files directly under `src/` (e.g.
# `operations/src/resize_blend.rs`). The `(glob)` form matches them.
files=$(scan "wildcard-arm file scan" \
  git grep -l "_ =>" -- \
    ":(glob)crates/*/src/**/*.rs" \
    ":(exclude,glob)crates/*/src/**/tests.rs" \
    ":(exclude,glob)crates/*/src/**/tests/**")

observed=$(
  while IFS= read -r file; do
    [[ -n $file ]] || continue
    count=$(scan "arm count for ${file}" count_arms <"$file")
    if ((count > 0)); then
      printf '%s %s\n' "$count" "$file"
    fi
  done <<<"$files"
  exit 0
)
listed=$(scan "manifest read" \
  grep -vE '^[[:space:]]*(#|$)' "$MANIFEST")

status=0

# Join on the path: growth (or a brand-new file) vs unrecorded shrinkage.
grown=$(LC_ALL=C join -j 2 -o 1.1,1.2,2.1 \
  <(printf '%s\n' "$observed" | LC_ALL=C sort -k2) \
  <(printf '%s\n' "$listed" | LC_ALL=C sort -k2) |
  awk '$1 > $3 { print $2 ": baseline " $3 ", observed " $1 }')
if [[ -n $grown ]]; then
  echo "VIOLATION: wildcard-arm count grew past the B24 baseline (convert the new arm to exhaustive matches or, if it is genuinely required, record why and re-baseline deliberately):"
  echo "$grown" | sed 's/^/  /'
  status=1
fi

new_files=$(LC_ALL=C comm -23 \
  <(printf '%s\n' "$observed" | awk '{ print $2 }' | LC_ALL=C sort) \
  <(printf '%s\n' "$listed" | awk '{ print $2 }' | LC_ALL=C sort))
if [[ -n $new_files ]]; then
  echo "VIOLATION: files with wildcard arms over EdgeCurve/FaceSurface outside the baseline manifest:"
  echo "$new_files" | sed 's/^/  /'
  echo "  Convert the arms to exhaustive matches instead of extending the manifest."
  status=1
fi

shrunk=$(LC_ALL=C join -j 2 -o 1.1,1.2,2.1 \
  <(printf '%s\n' "$observed" | LC_ALL=C sort -k2) \
  <(printf '%s\n' "$listed" | LC_ALL=C sort -k2) |
  awk '$1 < $3 { print $2 ": baseline " $3 ", observed " $1 }')
if [[ -n $shrunk ]]; then
  echo "STALE: wildcard-arm count shrank without a manifest update in ${MANIFEST} (record the reduction in the same PR):"
  echo "$shrunk" | sed 's/^/  /'
  status=1
fi

gone=$(LC_ALL=C comm -13 \
  <(printf '%s\n' "$observed" | awk '{ print $2 }' | LC_ALL=C sort) \
  <(printf '%s\n' "$listed" | awk '{ print $2 }' | LC_ALL=C sort))
if [[ -n $gone ]]; then
  echo "STALE: listed in ${MANIFEST} but no longer carrying wildcard arms (remove the entry):"
  echo "$gone" | sed 's/^/  /'
  status=1
fi

if [[ $status -eq 0 ]]; then
  total=$(printf '%s\n' "$observed" | awk '{ s += $1 } END { print s + 0 }')
  echo "✅ wildcard-arm ratchet OK (${total} audited arms, no growth past the baseline)."
fi

exit "$status"
