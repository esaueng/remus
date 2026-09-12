#!/usr/bin/env bash
set -euo pipefail

# Ratchet on EdgeCurve/FaceSurface wildcard arms in volume.rs (B24, file 1/4).
#
# A `match` over `EdgeCurve` (`topology/src/edge.rs`) or `FaceSurface`
# (`topology/src/face.rs`) that carries a `_ =>` arm compiles clean when a new
# variant lands and silently routes it down the approximate branch — a skipped
# face, an exact path declined for a mesh fallback. volume.rs is converted to
# exhaustive matches by this change; the gate pins that: it counts production
# `_ =>` arms inside `match` blocks whose arm PATTERNS mention `EdgeCurve::`
# or `FaceSurface::`, and fails on any growth. Pattern-side only — an
# `EdgeCurve::` constructor in an arm BODY does not make the block an enum
# match, and neither do matches over `RecognizedCurve` that merely sit near
# one. A bare binding catch-all (`surface =>`) is not spelled `_ =>` and is
# not counted. Matches over tuples, slices, `Result`s, or `Option`s whose
# patterns destructure an enum carrier (e.g. `(FaceSurface::Plane { .. }, _)`)
# count as enum matches: adding a variant still compiles silent there, so the
# conversion PR must name it too. (No such composite match exists in volume.rs
# today; the rule is stated so the first one cannot sneak past the gate.)
#
# Test code (`#[cfg(test)]`, `tests.rs`, `tests/`) is out of scope, exactly as
# in `check-det-hash.sh`: a test's fallback never feeds a shipped result.
#
# Two rules are inherited from check-det-hash.sh, which paid for both:
#
#   1. Scan with `git grep`, never `rg`. Ripgrep is NOT installed on the GitHub
#      runner. An `rg` call there exits 127, and a scan whose failure is
#      swallowed reports an empty tree — which for a ratchet means a converted
#      file reads as clean for the wrong reason and the gate vouches for a
#      tree it never actually read.
#   2. Never let a scan's exit status be interpreted as a result. 0 means found,
#      1 means clean; anything else means the tool failed and the gate must say
#      so loudly rather than vouch for a tree it could not scan.

cd "$(dirname "$0")/.."

readonly FILE=crates/operations/src/measure/volume.rs

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

# Count production `_ =>` arms that open their own line inside a `match` block
# whose arm PATTERNS mention `EdgeCurve::` or `FaceSurface::` (stdin = one
# Rust source file; prints the arm count).
#
# A stack of `match` frames tracks brace/paren/bracket depth plus a
# pattern-vs-body state per frame: `=>` at the direct level closes a pattern
# (recording whether it mentioned an enum), `,` at the direct level or the
# close of a body block reopens pattern position, and a `_ =>` arm counts only
# in pattern position at the direct level of a frame whose patterns mentioned
# an enum. Strings, character literals, and line/block comments are stripped
# first so braces inside them cannot shift the depth tracking. `#[cfg(test)]`
# module bodies are skipped so test fallbacks never count.
count_arms() {
  awk -v SQ="'" '
    function push_frame() {
      nstack++
      fb[nstack] = 0; fp[nstack] = 0; fq[nstack] = 0
      fstate[nstack] = 0; fseg[nstack] = 0; fhas[nstack] = 0; fwild[nstack] = 0
    }
    function pop_frame() {
      if (fhas[nstack]) total += fwild[nstack]
      nstack--
    }
    BEGIN { nstack = 0; total = 0; in_block = 0; pending = 0; skip_depth = 0; skip_pending = 0 }
    {
      raw = $0
      if (skip_depth == 0 && raw ~ /#\[cfg\(test\)\]/) { skip_pending = 1 }
      clean = ""
      i = 1
      in_str = 0
      while (i <= length(raw)) {
        c = substr(raw, i, 1)
        nx = substr(raw, i+1, 1)
        if (in_block) {
          if (c == "*" && nx == "/") { in_block = 0; i += 2 } else { i++ }
        } else if (in_str) {
          if (c == "\\") { i += 2 } else if (c == "\"") { in_str = 0; i++ } else { i++ }
        } else {
          if (c == "/" && nx == "/") { break }
          else if (c == "/" && nx == "*") { in_block = 1; i += 2 }
          else if (c == "\"") { in_str = 1; i++ }
          else if (c == SQ && substr(raw, i+2, 1) == SQ) { clean = clean "   "; i += 3 }
          else { clean = clean c; i++ }
        }
      }
      if (skip_pending && clean ~ /\{/) {
        skip_depth = 1
        skip_pending = 0
        next
      }
      skip_pending = 0
      if (skip_depth > 0) {
        j = 1
        while (j <= length(clean)) {
          cc = substr(clean, j, 1)
          if (cc == "{") skip_depth++
          else if (cc == "}") {
            skip_depth--
            if (skip_depth == 0) break
          }
          j++
        }
        if (skip_depth > 0) next
        clean = substr(clean, j + 1)
        if (clean ~ /^[ \t]*$/) next
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

count=$(scan "arm count for ${FILE}" count_arms <"$FILE")

if [[ $count != 0 ]]; then
  echo "VIOLATION: ${FILE} carries ${count} EdgeCurve/FaceSurface wildcard arm(s):"
  scan "wildcard-arm locations" git grep -n "_ =>" -- "$FILE" | sed 's/^/  /'
  echo "  Convert the arms to exhaustive matches (one arm per EdgeCurve/FaceSurface variant)"
  echo "  instead of extending the wildcard. The other B24 dense files"
  echo "  (phase_ff.rs, nonplanar.rs, resize_blend.rs) convert in their own PRs."
  exit 1
fi

echo "✅ wildcard-arm ratchet OK (${FILE} has no EdgeCurve/FaceSurface wildcards)."
