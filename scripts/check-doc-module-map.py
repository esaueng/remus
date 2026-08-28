#!/usr/bin/env python3
"""Verify every crate module appears in the AGENTS.md Module Map.

`check-doc-paths.sh` catches the other direction: a documented path that no
longer resolves. Nothing caught a module that exists but was never written
down, so the map drifted silently — 42 modules had accumulated by the time
anyone counted, across every crate in the workspace.

The map is what an agent reads to find the right file for a task, so a
module missing from it is invisible: the session never learns the file is
there. Reading `//!` headers to describe a module takes a minute; noticing
that one is absent takes a script.

Matching mirrors how each crate's table is actually written:
  * a backticked path            -- `nurbs/curve.rs`
  * a backticked bare filename   -- `vec.rs`
  * a glob                       -- `pave_filler/phase_*.rs`
  * a directory shorthand        -- `boolean/` (mod, types, classify)

Tokens are scoped to the crate section that contains them. Otherwise a row for
`heal/context.rs`, for example, silently counts `math/context.rs` as covered.
"""

from __future__ import annotations

import fnmatch
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC = os.path.join(REPO, "AGENTS.md")
CRATES = os.path.join(REPO, "crates")

# `lib.rs` and `mod.rs` are wiring, not destinations; `tests.rs` files are
# test-only companions rather than production task entry points.
SKIP_STEMS = {"lib", "mod", "tests"}

# Modules deliberately absent from the map. Keep this short and justified.
ALLOWLIST: set[str] = set()


def crate_sections(module_map: str) -> dict[str, str]:
    """Return each crate's own Module Map subsection."""
    headings = list(
        re.finditer(
            r"^### .*?\(`crates/([^/]+)/src/`\)\s*$", module_map, re.MULTILINE
        )
    )
    return {
        match.group(1): module_map[
            match.end() : headings[index + 1].start()
            if index + 1 < len(headings)
            else len(module_map)
        ]
        for index, match in enumerate(headings)
    }


def documented_tokens(crate_section: str) -> tuple[set[str], list[str]]:
    """Paths one crate table names, split into exact tokens and globs."""
    exact: set[str] = set()
    globs: list[str] = []

    for row in crate_section.splitlines():
        if not row.startswith("|"):
            continue

        # A bare filename after a qualified one continues in that directory:
        # `pave_filler/make_blocks.rs`, `make_split_edges.rs`.
        directory = ""
        for token in re.findall(r"`([A-Za-z0-9_.*/-]+\.rs)`", row):
            if "/" in token:
                directory = token.rsplit("/", 1)[0] + "/"
            resolved = token if "/" in token else f"{directory}{token}"
            if "*" in resolved:
                globs.append(resolved)
            else:
                exact.add(resolved)

        for shorthand, inner in re.findall(
            r"`([A-Za-z0-9_./-]+/)`\s*\(([^)]*)\)", row
        ):
            for part in inner.split(","):
                name = part.strip()
                if name:
                    exact.add(f"{shorthand}{name}.rs")

    return exact, globs


def main() -> int:
    doc = open(DOC, encoding="utf-8").read()
    try:
        start = doc.index("## Module Map")
        end = doc.index("## Ripple-Effect Checklists")
    except ValueError:
        print("VIOLATION: AGENTS.md is missing its Module Map section markers")
        return 1

    sections = crate_sections(doc[start:end])

    missing: list[str] = []
    for crate in sorted(os.listdir(CRATES)):
        src = os.path.join(CRATES, crate, "src")
        if not os.path.isdir(src):
            continue
        if crate not in sections:
            print(f"VIOLATION: AGENTS.md Module Map has no section for crate {crate}")
            return 1
        exact, globs = documented_tokens(sections[crate])
        for root, _, files in os.walk(src):
            for filename in sorted(files):
                if not filename.endswith(".rs"):
                    continue
                stem = filename[: -len(".rs")]
                if stem in SKIP_STEMS:
                    continue
                rel = os.path.relpath(os.path.join(root, filename), src)
                if rel in exact:
                    continue
                if any(
                    fnmatch.fnmatch(rel, pattern) for pattern in globs
                ):
                    continue
                if f"{crate}/{rel}" in ALLOWLIST:
                    continue
                missing.append(f"{crate}/src/{rel}")

    if missing:
        for path in missing:
            print(f"VIOLATION: crates/{path} is absent from the AGENTS.md Module Map")
        print()
        print("❌ Module map check failed.")
        print("   Add a row under the crate's section describing what the module")
        print("   is for -- take the wording from its own `//!` header, not a guess.")
        print("   If a module genuinely does not belong in the map, add it to")
        print("   ALLOWLIST in scripts/check-doc-module-map.py with a reason.")
        return 1

    print("✅ Module map covers every crate module.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
