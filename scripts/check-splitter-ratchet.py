#!/usr/bin/env python3
"""Ratchet on the face splitter's hand-written special cases (Open Kernel O2.3d).

`crates/algo/src/builder/face_splitter/special_cases.rs` is the retirement
target of the UV-arrangement splitter. It grew from 4,368 lines and ten entry
points at the O2.3a baseline to about 7,960 lines and sixteen entry points
with nothing measuring it, and the growth also went into the emitters and the
dispatch chain in `face_splitter/mod.rs`. This gate pins all three surfaces.
It does not judge them: it makes every rise a recorded decision.

Metrics:

- `special_cases.production_lines`: physical lines of `special_cases.rs`
  outside `#[cfg(test)]` items (the measure of the O2.3a design note).
- `special_cases.entry_points`: functions callable from outside the file. A
  top-level function with any `pub` visibility counts, and so does a
  function in a nested module unless it is `pub(super)` or `pub(self)`,
  which cannot leave the file.
- `special_cases.pub_fns`: every production function with any `pub`
  visibility, nested helpers included.
- `mod.emitter_lines.<name>`: lines of each named special-case emitter in
  `face_splitter/mod.rs`, signature through closing brace. A deleted emitter
  counts zero.
- `mod.dispatch_returns.split_face_2d_impl`: `return` statements of
  `split_face_2d_impl` outside closures and nested functions, which is the
  dispatch chain's early-return sites.

Baseline format (`scripts/splitter-ratchet-baseline.txt`): one `<floor>
<metric>` line per metric, then ledger lines `raise <metric> +<n>
<roadmap-anchor> <reason>`. The allowed count of a metric is its floor plus
its ledger raises, and the observed count must equal it:

- growth past the allowed count fails. Lower the code, or add a ledger line
  whose anchor is a row of `docs/kernel-maturity/roadmap.md`;
- shrinkage fails too until the baseline records it in the same change (set
  the floor to the observed count and delete that metric's ledger lines), so
  a reduction is banked where it lands.

Like `check-wildcard-arms.sh`, a scan that cannot run exits 2 rather than
vouching for a tree it did not read.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
SPECIAL_CASES = "crates/algo/src/builder/face_splitter/special_cases.rs"
SPLITTER_MOD = "crates/algo/src/builder/face_splitter/mod.rs"
BASELINE = "scripts/splitter-ratchet-baseline.txt"
ROADMAP = "docs/kernel-maturity/roadmap.md"
DISPATCH_FN = "split_face_2d_impl"
EMITTERS = (
    "split_periodic_face_by_winding_chain",
    "split_periodic_face_by_rim_chains",
    "arrangement_regions_from_combined",
    "arrangement_regions_from_inputs",
    "split_plane_face_by_arrangement",
    "split_cylinder_band_by_arrangement",
)

FN = re.compile(
    r"(?P<vis>\bpub(?:\s*\([^)]*\))?\s+)?"
    r"(?:(?:const|async|unsafe)\s+)*"
    r"\bfn\s+(?P<name>[A-Za-z_]\w*)"
)
CFG_TEST = re.compile(r"#\s*\[\s*cfg\s*\(\s*test\s*\)\s*\]")
OPEN, CLOSE = "([{", ")]}"


class ScanError(Exception):
    """The source could not be read as Rust; the gate must not pass."""


def blank(text: str) -> str:
    """Replace comments, strings and character literals with spaces.

    Newlines survive, so offsets and line numbers match the source, and a
    brace inside a string or a comment can no longer shift depth tracking.
    """
    out = list(text)
    i, n = 0, len(text)

    def wipe(start: int, end: int) -> None:
        for k in range(start, min(end, n)):
            if out[k] != "\n":
                out[k] = " "

    while i < n:
        c = text[i]
        two = text[i : i + 2]
        if two == "//":
            end = text.find("\n", i)
            end = n if end < 0 else end
            wipe(i, end)
            i = end
        elif two == "/*":
            depth, k = 1, i + 2
            while k < n and depth:
                if text[k : k + 2] == "/*":
                    depth, k = depth + 1, k + 2
                elif text[k : k + 2] == "*/":
                    depth, k = depth - 1, k + 2
                else:
                    k += 1
            if depth:
                raise ScanError("unterminated block comment")
            wipe(i, k)
            i = k
        elif c == '"' or (c in "rb" and re.match(r'(?:b?r#*"|b")', text[i:]) and not _ident_before(text, i)):
            raw = re.match(r'b?r(#*)"', text[i:])
            if raw:
                close = '"' + raw.group(1)
                end = text.find(close, i + raw.end())
                if end < 0:
                    raise ScanError("unterminated raw string")
                end += len(close)
            else:
                k = text.index('"', i) + 1
                while k < n and text[k] != '"':
                    k += 2 if text[k] == "\\" else 1
                if k >= n:
                    raise ScanError("unterminated string")
                end = k + 1
            wipe(i, end)
            i = end
        elif c == "'":
            # A character literal ('x', '\n', '\u{1F600}'); anything else that
            # starts with a quote is a lifetime and is left alone.
            lit = re.match(r"'(?:\\(?:u\{[0-9a-fA-F_]+\}|x[0-9a-fA-F]{2}|.)|[^\\'\n])'", text[i:])
            if lit:
                wipe(i, i + lit.end())
                i += lit.end()
            else:
                i += 1
        else:
            i += 1
    return "".join(out)


def _ident_before(text: str, i: int) -> bool:
    return i > 0 and (text[i - 1].isalnum() or text[i - 1] == "_")


def matching_brace(clean: str, start: int) -> int:
    """Offset of the `}` closing the `{` at `start`."""
    depth = 0
    for k in range(start, len(clean)):
        if clean[k] == "{":
            depth += 1
        elif clean[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    raise ScanError("unbalanced braces")


def item_end(clean: str, start: int) -> int:
    """Offset of the last character of the item starting at `start`: its
    `;`, or the brace closing its body. Brackets in the head (generics,
    parameter lists, array types) are skipped."""
    depth = 0
    for k in range(start, len(clean)):
        c = clean[k]
        if c in "([":
            depth += 1
        elif c in ")]":
            depth -= 1
        elif depth == 0 and c == ";":
            return k
        elif depth == 0 and c == "{":
            return matching_brace(clean, k)
    raise ScanError("item without an end")


def line_of(clean: str, offset: int) -> int:
    return clean.count("\n", 0, offset) + 1


def test_spans(clean: str) -> list[tuple[int, int]]:
    """Offset spans of the outermost `#[cfg(test)]` items."""
    spans: list[tuple[int, int]] = []
    for match in CFG_TEST.finditer(clean):
        if spans and match.start() <= spans[-1][1]:
            continue
        spans.append((match.start(), item_end(clean, match.end())))
    return spans


def brace_depths(clean: str) -> list[int]:
    depths, depth = [], 0
    for c in clean:
        if c == "}":
            depth -= 1
        depths.append(depth)
        if c == "{":
            depth += 1
    if depth != 0:
        raise ScanError("unbalanced braces")
    return depths


def scan_special_cases(text: str) -> dict[str, int]:
    clean = blank(text)
    spans = test_spans(clean)
    test_lines: set[int] = set()
    for start, end in spans:
        test_lines.update(range(line_of(clean, start), line_of(clean, end) + 1))
    total = len(text.splitlines())
    depths = brace_depths(clean)
    entry_points = pub_fns = 0
    for match in FN.finditer(clean):
        vis = match.group("vis")
        if not vis or any(start <= match.start() <= end for start, end in spans):
            continue
        pub_fns += 1
        scope = re.sub(r"\s", "", vis)
        if depths[match.start()] == 0 or scope not in ("pub(super)", "pub(self)"):
            entry_points += 1
    return {
        "special_cases.production_lines": total - len(test_lines),
        "special_cases.entry_points": entry_points,
        "special_cases.pub_fns": pub_fns,
    }


def find_fn(clean: str, name: str) -> tuple[int, int, int] | None:
    """`(item start, body open, body close)` of the top-level function `name`."""
    depths = brace_depths(clean)
    for match in FN.finditer(clean):
        if match.group("name") != name or depths[match.start()] != 0:
            continue
        end = item_end(clean, match.end())
        if clean[end] != "}":
            continue
        depth, k = 0, end
        while True:
            if clean[k] == "}":
                depth += 1
            elif clean[k] == "{":
                depth -= 1
                if depth == 0:
                    return match.start(), k, end
            k -= 1
    return None


def dispatch_returns(clean: str, open_at: int, close_at: int) -> list[int]:
    """Lines of the `return` statements of one function body that belong to
    the function itself, not to a closure or a nested function.

    A closure is entered at `|params|` (or `||`) in expression position and
    left when its nesting level closes or a `,` or `;` ends it at that level,
    which covers block bodies and expression bodies alike.
    """
    lines: list[int] = []
    level = 0
    closures: list[int] = []
    k = open_at + 1
    prev = "{"  # last significant character before k
    prev_word = ""
    while k < close_at:
        c = clean[k]
        if c.isspace():
            k += 1
            continue
        if c.isalpha() or c == "_":
            word = re.match(r"\w+", clean[k:]).group(0)
            if word == "fn":
                k = item_end(clean, k) + 1
                prev, prev_word = "}", ""
                continue
            if word == "return" and not closures:
                lines.append(line_of(clean, k))
            k += len(word)
            prev, prev_word = word[-1], word
            continue
        if c == "|":
            operand_before = prev.isalnum() or prev in "_)]}\"'?"
            if prev_word in ("move", "return", "else", "in"):
                operand_before = False
            if not operand_before:
                if clean[k + 1] == "|":
                    k += 2
                else:
                    end = clean.find("|", k + 1, close_at)
                    if end < 0:
                        raise ScanError("closure parameter list without an end")
                    k = end + 1
                # An annotated return type always precedes a block body; its
                # generic arguments carry commas that end nothing.
                arrow = re.match(r"\s*->", clean[k:close_at])
                if arrow:
                    body = clean.find("{", k + arrow.end(), close_at)
                    if body < 0:
                        raise ScanError("closure return type without a body")
                    k = body
                closures.append(level)
                prev, prev_word = "|", ""
                continue
            k += 2 if clean[k + 1] == "|" else 1
            prev, prev_word = "|", ""
            continue
        if c in OPEN:
            level += 1
        elif c in CLOSE:
            level -= 1
            while closures and closures[-1] > level:
                closures.pop()
        elif c in ",;":
            while closures and closures[-1] >= level:
                closures.pop()
        prev, prev_word = c, ""
        k += 1
    return lines


def scan_splitter_mod(text: str) -> tuple[dict[str, int], list[int]]:
    clean = blank(text)
    counts: dict[str, int] = {}
    for name in EMITTERS:
        found = find_fn(clean, name)
        counts[f"mod.emitter_lines.{name}"] = (
            0 if found is None else line_of(clean, found[2]) - line_of(clean, found[0]) + 1
        )
    found = find_fn(clean, DISPATCH_FN)
    sites = [] if found is None else dispatch_returns(clean, found[1], found[2])
    counts[f"mod.dispatch_returns.{DISPATCH_FN}"] = len(sites)
    return counts, sites


def read(root: Path, relative: str) -> str:
    path = root / relative
    return path.read_text(encoding="utf-8") if path.is_file() else ""


def observe(root: Path) -> tuple[dict[str, int], list[int]]:
    counts = scan_special_cases(read(root, SPECIAL_CASES))
    mod_counts, sites = scan_splitter_mod(read(root, SPLITTER_MOD))
    counts.update(mod_counts)
    return counts, sites


def parse_baseline(text: str) -> tuple[dict[str, int], list[tuple[str, int, str, str]]]:
    floors: dict[str, int] = {}
    ledger: list[tuple[str, int, str, str]] = []
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        raise_line = re.fullmatch(r"raise\s+(\S+)\s+\+(\d+)\s+(\S+)\s+(\S.*)", line)
        floor_line = re.fullmatch(r"(\d+)\s+(\S+)", line)
        if raise_line:
            metric, amount, anchor, reason = raise_line.groups()
            ledger.append((metric, int(amount), anchor, reason))
        elif floor_line and floor_line.group(2) not in floors:
            floors[floor_line.group(2)] = int(floor_line.group(1))
        else:
            raise ScanError(f"{BASELINE}:{number}: not a floor or ledger line: {line}")
    return floors, ledger


def check(root: Path) -> int:
    observed, _ = observe(root)
    baseline = root / BASELINE
    roadmap = root / ROADMAP
    if not baseline.is_file():
        raise ScanError(f"{BASELINE} is missing")
    if not roadmap.is_file():
        raise ScanError(f"{ROADMAP} is missing")
    floors, ledger = parse_baseline(baseline.read_text(encoding="utf-8"))
    anchors = set(re.findall(r'<a id="([^"]+)"></a>', roadmap.read_text(encoding="utf-8")))

    failures: list[str] = []
    allowed = dict(floors)
    for metric, amount, anchor, _reason in ledger:
        if metric not in floors:
            failures.append(f"VIOLATION: ledger line raises `{metric}`, which has no floor line.")
        elif anchor not in anchors:
            failures.append(
                f"VIOLATION: ledger line for `{metric}` names `{anchor}`, which is not a row "
                f"anchor in {ROADMAP} (use the row's `<a id>`, for example `o-o2-3d`)."
            )
        else:
            allowed[metric] += amount

    for metric in sorted(set(observed) | set(floors)):
        if metric not in observed:
            failures.append(f"STALE: `{metric}` is in {BASELINE} but is no longer measured; remove it.")
        elif metric not in floors:
            failures.append(
                f"VIOLATION: `{metric}` (observed {observed[metric]}) has no floor line in {BASELINE}."
            )
        elif observed[metric] > allowed[metric]:
            failures.append(
                f"VIOLATION: `{metric}` grew: allowed {allowed[metric]}, observed {observed[metric]}. "
                "Move the case onto the arrangement instead, or record the rise with a ledger line "
                f"`raise {metric} +{observed[metric] - allowed[metric]} <roadmap-anchor> <reason>`."
            )
        elif observed[metric] < allowed[metric]:
            failures.append(
                f"STALE: `{metric}` shrank: allowed {allowed[metric]}, observed {observed[metric]}. "
                f"Set its floor to {observed[metric]} and delete its ledger lines in the same change."
            )

    for line in failures:
        print(line)
    if failures:
        return 1
    raised = sum(amount for _, amount, _, _ in ledger)
    print(
        f"splitter ratchet OK ({len(observed)} metrics, "
        f"{observed['special_cases.production_lines']} production lines and "
        f"{observed['special_cases.entry_points']} entry points in special_cases.rs, "
        f"{raised} ledgered rise)."
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", type=Path, default=ROOT, help="Repository root to scan")
    parser.add_argument(
        "--list", action="store_true", help="Print the observed counts and dispatch sites, then exit"
    )
    args = parser.parse_args(argv)
    try:
        if args.list:
            observed, sites = observe(args.root)
            for metric in sorted(observed):
                print(f"{observed[metric]} {metric}")
            print(f"# {DISPATCH_FN} return sites: " + " ".join(f"{SPLITTER_MOD}:{n}" for n in sites))
            return 0
        return check(args.root)
    except (ScanError, OSError, UnicodeDecodeError) as error:
        print(f"check-splitter-ratchet: {error}; the gate could not run", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
