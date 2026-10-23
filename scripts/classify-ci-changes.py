#!/usr/bin/env python3
"""Classify a Git diff for CI without changing required check names.

Outputs (all lowercase booleans except ``mode``):

* ``heavy`` — source changed: build, lint, and test the workspace.
* ``docs`` — documentation changed: build rustdoc and the book.
* ``full`` — run the second tier (coverage, MSRV, deny, audit). True only
  for heavy changes on non-PR events or PRs carrying the ``ci:full`` label
  (``--full``). A merge group keeps it only
  for a diff that touches a second-tier path (see ``needs_second_tier``).
* ``wasm`` — build and validate the distributable WASM packages. True for
  heavy changes when ``full`` is set or the diff touches a WASM-affecting
  path, which includes every crate under ``crates/``.
* ``mode`` — ``full``, ``pr``, ``queue``, ``package``, ``docs``, or
  ``ci-only``.
"""

from __future__ import annotations

import argparse
import os
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
# The committed distributable packages. A diff confined to them (a refresh
# PR from publish.yml) leaves every source, script and workflow at the base
# commit, which already passed the full suite on its main push. Paths alone
# do not prove the committed bytes came from that source; the required
# "Committed packages match source" job in wasm-version.yml does, by
# deleting both package trees, rebuilding them with `cargo xtask wasm-build`
# (which also runs the package smoke and installed-tarball consumer tests)
# and comparing every file. So a package-only diff runs no heavy tier here:
# re-running the suites on an unchanged source tree proves nothing new, and
# did cost three full runs (PR, merge group, main push) per refresh. Any
# other path in the diff, including the checker's own scripts and
# workflows, makes it heavy again.
PACKAGE_DIRECTORIES = ("crates/wasm/pkg/", "crates/wasm-io/pkg/")
# Paths whose change can alter the distributable WASM binaries or their
# packaging in a way the native suite does not exercise.
#
# Every crate sits below the wasm bindings, so any change under `crates/`
# can break the packaged kernel. This used to list only `crates/wasm/` and
# `crates/wasm-io/`, which made the package smoke a main-only gate for engine
# changes: PR #618 (algo, math, operations) passed PR CI with WASM
# Build & Validate skipped, then failed that job on main at 8539b266.
# The job runs beside Test (median 13.6 vs 20.6 minutes on hosted runners,
# 2026-09), so selecting it for every kernel PR does not lengthen the PR.
WASM_DIRECTORIES = ("crates/", "xtask/", "tools/vs-bench/")
# `.cargo/config.toml` defines the `cargo xtask` alias the package build runs.
WASM_FILENAMES = {"Cargo.lock", "Cargo.toml", "rust-toolchain.toml", ".cargo/config.toml"}
# Paths whose effect a merge group cannot leave to the main push: dependency
# and toolchain inputs (Cargo Deny, Security Audit, MSRV) and the CI
# definitions. Fuzz compile and render run in the first tier. Any other
# heavy merge group runs the PR tier. The main push of the same squashed tree
# still runs the second tier, so Coverage (about 40 of a queue entry's 60-70
# minutes, 2026-10) reports after the merge instead of before it.
SECOND_TIER_DIRECTORIES = (".cargo/", ".config/", ".github/workflows/")
SECOND_TIER_FILENAMES = {
    "Cargo.lock", "Cargo.toml", "clippy.toml", "deny.toml", "rust-toolchain.toml",
}


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


def needs_second_tier(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix()
    return (
        # A queue batch can pair a package refresh with source PRs, so its diff
        # is not package-only; the refresh still needs the full suite.
        is_committed_package(normalized)
        or normalized.startswith(SECOND_TIER_DIRECTORIES)
        or normalized in SECOND_TIER_FILENAMES
        or PurePosixPath(normalized).name == "Cargo.toml"
    )


def classify_paths(
    paths: list[str], force_full: bool = False, merge_group: bool = False
) -> Classification:
    # An empty or unrecognized diff must receive the complete suite.
    if not paths:
        return Classification(heavy=True, docs=True, full=True, wasm=True, mode="full")

    # A package refresh: wasm-version.yml rebuilds and compares the packages,
    # whatever the event or --full asks for (see PACKAGE_DIRECTORIES).
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
        full = force_full and (
            not merge_group or any(needs_second_tier(path) for path in paths)
        )
        wasm = full or any(affects_wasm(path) for path in paths)
        if full:
            mode = "full"
        elif force_full:
            mode = "queue"
        else:
            mode = "pr"
        return Classification(heavy=True, docs=True, full=full, wasm=wasm, mode=mode)

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
    # The pinned callee passes only --full; the event name comes from the runner.
    merge_group = os.environ.get("GITHUB_EVENT_NAME") == "merge_group"
    result = classify_paths(paths, force_full=args.full, merge_group=merge_group)
    print(f"heavy={str(result.heavy).lower()}")
    print(f"docs={str(result.docs).lower()}")
    print(f"full={str(result.full).lower()}")
    print(f"wasm={str(result.wasm).lower()}")
    print(f"mode={result.mode}")
    print(f"changed_count={len(paths)}")


if __name__ == "__main__":
    main()
