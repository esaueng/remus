#!/usr/bin/env python3
"""Classify a Git diff for CI without changing required check names.

Outputs (all lowercase booleans except ``mode``):

* ``heavy`` — source changed: build, lint, and test the workspace.
* ``docs`` — documentation changed: build rustdoc and the book.
* ``full`` — run the second tier (coverage, macOS, MSRV, fuzz compile,
  render, deny, audit). True only for heavy changes on non-PR events or PRs
  carrying the ``ci:full`` label (``--full``).
* ``wasm`` — build and validate the distributable WASM packages. True for
  heavy changes when ``full`` is set or the diff touches a WASM-affecting
  path.
* ``mode`` — ``full``, ``pr``, ``docs``, ``package``, or ``ci-only``.
"""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import PurePosixPath


DOC_DIRECTORIES = ("book/", "docs/", "rfcs/")
DOC_FILENAMES = {
    "CHANGELOG",
    "CHANGELOG.md",
    "CONTRIBUTING",
    "CONTRIBUTING.md",
    "LICENSE",
    "LICENSE-APACHE",
    "NOTICE",
    "README",
    "README.md",
}
# Agent instructions and skills: text read by tooling, never compiled.
AGENT_DIRECTORIES = (".claude/",)
# The committed distributable packages. A diff confined to them is the
# publisher's refresh PR: its bytes were built and smoke-tested by
# `cargo xtask wasm-build` from an already-validated main commit, and the
# separate WASM version guard checks the version bump.
PACKAGE_DIRECTORIES = ("crates/wasm/pkg/", "crates/wasm-io/pkg/")
# Paths whose change can alter the distributable WASM binaries or their
# packaging in a way the native suite does not exercise.
WASM_DIRECTORIES = ("crates/wasm/", "crates/wasm-io/", "xtask/", "tools/vs-bench/")
WASM_FILENAMES = {"Cargo.lock", "Cargo.toml", "rust-toolchain.toml"}


@dataclass(frozen=True)
class Classification:
    heavy: bool
    docs: bool
    full: bool
    wasm: bool
    mode: str


def is_documentation(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix()
    parsed = PurePosixPath(normalized)
    return (
        normalized.startswith(DOC_DIRECTORIES)
        or (
            len(parsed.parts) == 1
            and (normalized.endswith(".md") or normalized in DOC_FILENAMES)
        )
    )


def is_lightweight_github_metadata(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix()
    return normalized.startswith(".github/") and not normalized.startswith(
        ".github/workflows/"
    )


def is_agent_instruction(path: str) -> bool:
    return PurePosixPath(path).as_posix().startswith(AGENT_DIRECTORIES)


def is_committed_package(path: str) -> bool:
    return PurePosixPath(path).as_posix().startswith(PACKAGE_DIRECTORIES)


def affects_wasm(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix()
    name = PurePosixPath(normalized).name
    return (
        normalized.startswith(WASM_DIRECTORIES)
        or normalized in WASM_FILENAMES
        or (normalized.startswith("scripts/") and ("wasm" in name or "w9" in name))
    )


def classify_paths(paths: list[str], force_full: bool = False) -> Classification:
    # An empty or unrecognized diff must receive the complete suite.
    if not paths:
        return Classification(heavy=True, docs=True, full=True, wasm=True, mode="full")

    if all(is_committed_package(path) for path in paths):
        return Classification(heavy=False, docs=False, full=False, wasm=False, mode="package")

    has_docs = any(is_documentation(path) for path in paths)
    lightweight = all(
        is_documentation(path)
        or is_lightweight_github_metadata(path)
        or is_agent_instruction(path)
        for path in paths
    )
    if not lightweight:
        wasm = force_full or any(affects_wasm(path) for path in paths)
        return Classification(
            heavy=True,
            docs=True,
            full=force_full,
            wasm=wasm,
            mode="full" if force_full else "pr",
        )

    return Classification(
        heavy=False,
        docs=has_docs,
        full=False,
        wasm=False,
        mode="docs" if has_docs else "ci-only",
    )


def changed_paths(base: str, head: str) -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--name-only", "-z", base, head],
        check=True,
        capture_output=True,
    )
    return [
        raw.decode("utf-8", errors="surrogateescape")
        for raw in result.stdout.split(b"\0")
        if raw
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True)
    parser.add_argument("--head", required=True)
    parser.add_argument(
        "--full",
        action="store_true",
        help="select the second tier for heavy changes (main pushes, ci:full label)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = changed_paths(args.base, args.head)
    result = classify_paths(paths, force_full=args.full)
    print(f"heavy={str(result.heavy).lower()}")
    print(f"docs={str(result.docs).lower()}")
    print(f"full={str(result.full).lower()}")
    print(f"wasm={str(result.wasm).lower()}")
    print(f"mode={result.mode}")
    print(f"changed_count={len(paths)}")


if __name__ == "__main__":
    main()
