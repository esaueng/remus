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

Matching mirrors how the map is actually written:
  * a backticked path            -- `nurbs/curve.rs`
  * a backticked bare filename   -- `vec.rs`
  * a glob                       -- `pave_filler/phase_*.rs`
  * a directory shorthand        -- `boolean/` (mod, types, classify)
"""

from __future__ import annotations

import fnmatch
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOC = os.path.join(REPO, "AGENTS.md")
CRATES = os.path.join(REPO, "crates")

# `lib.rs` and `mod.rs` are wiring, not destinations; the map names the
# modules they re-export.
SKIP_STEMS = {"lib", "mod"}

# Modules deliberately absent from the map. Keep this short and justified.
ALLOWLIST: set[str] = set()


def documented_tokens(module_map: str) -> tuple[set[str], list[str]]:
    """Everything the map names, split into exact tokens and glob patterns."""
    exact: set[str] = set()
    globs: list[str] = []

    for match in re.findall(r"`([A-Za-z0-9_.*/-]+\.rs)`", module_map):
        if "*" in match:
            globs.append(match)
            globs.append(match.split("/")[-1])
        else:
            exact.add(match)
            exact.add(match.split("/")[-1][: -len(".rs")])

    for directory, inner in re.findall(
        r"`([A-Za-z0-9_./-]+/)`\s*\(([^)]*)\)", module_map
    ):
        for part in inner.split(","):
            name = part.strip()
            if name:
                exact.add(name)
                exact.add(f"{directory}{name}.rs")

    return exact, globs


def main() -> int:
    doc = open(DOC, encoding="utf-8").read()
    try:
        start = doc.index("## Module Map")
        end = doc.index("## Ripple-Effect Checklists")
    except ValueError:
        print("VIOLATION: AGENTS.md is missing its Module Map section markers")
        return 1

    exact, globs = documented_tokens(doc[start:end])

    missing: list[str] = []
    for crate in sorted(os.listdir(CRATES)):
        src = os.path.join(CRATES, crate, "src")
        if not os.path.isdir(src):
            continue
        for root, _, files in os.walk(src):
            for filename in sorted(files):
                if not filename.endswith(".rs"):
                    continue
                stem = filename[: -len(".rs")]
                if stem in SKIP_STEMS:
                    continue
                rel = os.path.relpath(os.path.join(root, filename), src)
                if rel in exact or stem in exact:
                    continue
                if any(
                    fnmatch.fnmatch(rel, g) or fnmatch.fnmatch(filename, g)
                    for g in globs
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
