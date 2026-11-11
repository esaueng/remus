#!/usr/bin/env python3
"""Classify a Git diff into the CI tiers that `ci-checks.yml` runs.

Outputs (lowercase booleans except ``mode``):

* ``heavy`` — source changed: build, lint and test the workspace.
* ``docs`` — build rustdoc and the book.
* ``full`` — dependency and toolchain checks (MSRV, Cargo Deny, Security
  Audit). Selected when the diff touches one of their inputs, by the
  ``ci:full`` PR label, and by a manual dispatch.
* ``wasm`` — build and validate the distributable WASM packages.
* ``coverage`` — instrumented coverage, only on request.
* ``mode`` — ``full``, ``pr``, ``main``, ``queue``, ``package``, ``docs`` or
  ``ci-only``.

A merge group runs the light gates only (Repository Policy, Secrets Scan and
Documentation here, plus the privacy, dependency-review and WASM version
workflows). Its pull request already ran every selected tier on its merge
ref, and the main push of the squashed tree runs them again. Two pull
requests that pass alone but break together therefore turn `main` red after
the merge instead of blocking the queue entry.
"""

from __future__ import annotations

import argparse
import os
import subprocess
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
# commit, which already passed the full suite on its main push. The
# "Committed packages match source" job in wasm-version.yml proves the
# committed bytes came from that source by rebuilding and comparing them, and
# the required "Changed kernel requires a new version" check needs it. So a
# package-only diff runs no heavy tier here. Any other path in the diff makes
# it heavy again.
PACKAGE_DIRECTORIES = ("crates/wasm/pkg/", "crates/wasm-io/pkg/")
# Paths whose change can alter the distributable WASM binaries or their
# packaging in a way the native suite does not exercise. Every crate sits
# below the wasm bindings, so any change under `crates/` can break the
# packaged kernel (PR #618 passed with the package build skipped, then failed
# it on main).
WASM_DIRECTORIES = ("crates/", "xtask/", "tools/vs-bench/")
# `.cargo/config.toml` defines the `cargo xtask` alias the package build runs.
WASM_FILENAMES = {
    "Cargo.lock", "Cargo.toml", "rust-toolchain.toml", ".cargo/config.toml",
    "LICENSE-APACHE", "NOTICE",
    ".github/workflows/ci.yml", ".github/workflows/ci-checks.yml",
    ".github/workflows/publish.yml", ".github/workflows/wasm-size-report.yml",
    ".github/workflows/wasm-version.yml",
}
# Inputs of the dependency and toolchain checks, and the CI definitions.
SECOND_TIER_DIRECTORIES = (".cargo/", ".config/", ".github/workflows/")
SECOND_TIER_FILENAMES = {
    "Cargo.lock", "Cargo.toml", "clippy.toml", "deny.toml", "rust-toolchain.toml",
}

FLAGS = ("heavy", "docs", "full", "wasm", "coverage")


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
        normalized.startswith(SECOND_TIER_DIRECTORIES)
        or normalized in SECOND_TIER_FILENAMES
        or PurePosixPath(normalized).name == "Cargo.toml"
    )


def selection(mode: str, **selected: bool) -> dict:
    return {**{flag: bool(selected.get(flag, False)) for flag in FLAGS}, "mode": mode}


def classify(paths: list[str], event: str, force_full: bool = False, coverage: bool = False) -> dict:
    """Select tiers for ``paths`` on ``event``; an unknown diff gets everything."""
    everything = selection("full", heavy=True, docs=True, full=True, wasm=True, coverage=coverage)
    # A manual dispatch and the `ci:full` label ask for every tier, whatever
    # the diff. An empty or unavailable diff cannot be classified.
    if event == "workflow_dispatch" or force_full or not paths:
        return everything

    # A package refresh is verified by wasm-version.yml (see PACKAGE_DIRECTORIES).
    if all(is_committed_package(path) for path in paths):
        return selection("package")

    has_docs = any(is_documentation(path) for path in paths)
    lightweight = all(
        is_documentation(path)
        or is_lightweight_github_metadata(path)
        or is_agent_instruction(path)
        for path in paths
    )
    package_input = any(affects_wasm(path) for path in paths)
    if lightweight and not package_input:
        return selection("docs" if has_docs else "ci-only", docs=has_docs)

    if event == "merge_group":
        # Rustdoc still builds for the combined tree; see the module docstring.
        return selection("queue", docs=True)

    return selection(
        "main" if event == "push" else "pr",
        heavy=True,
        docs=True,
        full=any(needs_second_tier(path) for path in paths),
        wasm=package_input,
        coverage=coverage,
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", required=True)
    parser.add_argument("--head", required=True)
    parser.add_argument("--full", action="store_true", help="select every tier (the ci:full label)")
    args = parser.parse_args()
    paths = changed_paths(args.base, args.head)
    result = classify(
        paths,
        os.environ.get("GITHUB_EVENT_NAME", ""),
        force_full=args.full,
        coverage=os.environ.get("COVERAGE_REQUESTED") == "true",
    )
    for key, value in result.items():
        print(f"{key}={str(value).lower() if isinstance(value, bool) else value}")
    print(f"base={args.base}")
    print(f"changed_count={len(paths)}")


if __name__ == "__main__":
    main()
