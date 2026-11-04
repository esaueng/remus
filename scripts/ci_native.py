#!/usr/bin/env python3
"""Plan the fast native suite, build once, and run verified archive partitions."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ".github/ci-test-suites.json"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def suite_manifest(root: Path = ROOT) -> dict:
    data = json.loads((root / MANIFEST).read_text())
    if set(data) != {"schema", "critical", "focused_crates"} or type(data["schema"]) is not int or data["schema"] != 1:
        raise ValueError("Unsupported native suite manifest")
    paths = data["critical"]
    if not isinstance(paths, list) or not paths or len(set(paths)) != len(paths):
        raise ValueError("Critical tests must be a nonempty unique list")
    for path in paths:
        if not isinstance(path, str) or not re.fullmatch(r"crates/[a-z-]+/tests/[a-zA-Z0-9_]+\.rs", path):
            raise ValueError("Critical test path is malformed")
        if not (root / path).is_file():
            raise ValueError(f"Critical test is missing: {path}")
    crates = data["focused_crates"]
    if not isinstance(crates, list) or not crates or len(set(crates)) != len(crates):
        raise ValueError("Focused crates must be a nonempty unique list")
    for crate in crates:
        if not isinstance(crate, str) or not re.fullmatch(r"[a-z-]+", crate) or not (root / "crates" / crate / "Cargo.toml").is_file():
            raise ValueError("Focused crate is malformed or missing")
    return data


def package(crate: str) -> str:
    return "remus" if crate == "remus" else "remus-" + crate


def binary_filter(path: str) -> str:
    parts = PurePosixPath(path).parts
    return f"(package(={package(parts[1])}) & kind(test) & binary(={parts[3][:-3]}))"


def plan(paths: list[str], manifest: dict, *, full: bool = False, queue: bool = False) -> dict:
    """Unknown, deleted, fixture and foundational paths select the full suite."""
    critical = list(manifest["critical"])
    targets = set(critical)
    modules = set()
    reason = "explicit full run" if full else "critical and affected module tests"
    for path in paths:
        p = PurePosixPath(path)
        if p.is_absolute() or ".." in p.parts or str(p) != path:
            raise ValueError("Changed path is not a normalized repository path")
        if path.endswith(".md") or path.startswith(("docs/", "book/", ".claude/")):
            continue
        parts = p.parts
        if len(parts) >= 4 and parts[0] == "crates" and parts[1] in manifest["focused_crates"]:
            if parts[2] == "tests" and path.endswith(".rs"):
                # Nested test support files select their containing integration binary.
                target = "/".join(parts[:3]) + "/" + parts[3].removesuffix(".rs") + ".rs"
                targets.add(target)
                continue
            if parts[2] == "src" and path.endswith(".rs") and parts[3] not in ("lib.rs", "main.rs"):
                module = parts[3].removesuffix(".rs")
                if re.fullmatch(r"[a-zA-Z0-9_]+", module):
                    modules.add((package(parts[1]), module))
                    continue
        # Package artifacts are tested independently, but a mixed diff must
        # never accidentally turn into an untested native path.
        full, reason = True, f"broad validation for {path}"
    if not paths:
        full, reason = True, "empty or unavailable diff"
    # Queue smoke is selected only for a known leaf-source/test diff. A
    # foundational change still receives full native tests on its candidate.
    if queue and not full:
        modules.clear()
        reason = "critical integration tests on the combined queue candidate"
    return {"schema": 1, "mode": "full" if full else "fast", "reason": reason,
            "targets": sorted(targets), "modules": [list(x) for x in sorted(modules)]}


def expression(selection: dict) -> str:
    if selection["mode"] == "full":
        return "all()"
    terms = [binary_filter(path) for path in selection["targets"]]
    terms.extend(f"(package(={name}) & kind(lib) & test(/^{re.escape(module)}::/))"
                 for name, module in selection["modules"])
    return " | ".join(terms)


def build_args(selection: dict) -> list[str]:
    args = ["--cargo-profile", "ci-test", "--locked"]
    if selection["mode"] == "full":
        args += ["--workspace"]
    else:
        packages = {package(PurePosixPath(path).parts[1]) for path in selection["targets"]}
        packages.update(name for name, _module in selection["modules"])
        for name in sorted(packages):
            args += ["--package", name]
        args += ["--lib"]
        for target in sorted({PurePosixPath(p).stem for p in selection["targets"]}):
            args += ["--test", target]
    return args


def verify_inventory(selection: dict, inventory: dict) -> bool:
    """Reject missing critical binaries; request full fallback for empty modules."""
    suites = inventory["rust-suites"]
    for path in selection["targets"]:
        parts = PurePosixPath(path).parts
        matches = [s for s in suites.values() if s["package-name"] == package(parts[1])
                   and s["binary-name"] == PurePosixPath(path).stem and s["kind"] == "test"]
        if len(matches) != 1 or not matches[0]["testcases"]:
            raise ValueError(f"Selected integration binary has no tests: {path}")
    for name, module in selection["modules"]:
        if not any(s["package-name"] == name and s["kind"] == "lib"
                   and any(test.startswith(module + "::") and not case["ignored"]
                           for test, case in s["testcases"].items()) for s in suites.values()):
            return False
    return True


def run(command: list[str], **kwargs):
    print("+ " + " ".join(command), flush=True)
    return subprocess.run(command, cwd=ROOT, check=True, **kwargs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("plan", "archive", "run"))
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--base")
    parser.add_argument("--head", default="HEAD")
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--queue", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="critical-suite proof; not a substitute for PR classification")
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--shard", type=int, default=1)
    parser.add_argument("--total", type=int, default=4)
    args = parser.parse_args()
    manifest = suite_manifest()
    if args.command == "plan":
        if not args.base:
            parser.error("plan requires --base")
        raw = subprocess.check_output(["git", "diff", "--name-only", "-z", args.base, args.head], cwd=ROOT)
        paths = [p.decode() for p in raw.split(b"\0") if p]
        selection = plan(paths, manifest, full=args.full, queue=args.queue)
        if args.smoke:
            selection = plan(["README.md"], manifest)
            selection["reason"] = "staged critical-suite proof"
        # Deleted targets and non-Cargo test layout changes require broad validation.
        if not args.smoke and (any(not (ROOT / p).is_file() for p in paths) or any(not (ROOT / p).is_file() for p in selection["targets"])):
            selection["mode"], selection["reason"] = "full", "deleted path or unsupported nested test layout"
        selection["source_sha"] = git("rev-parse", args.head)
        selection["paths"] = paths
        args.plan.write_text(json.dumps(selection, indent=2) + "\n")
    else:
        selection = json.loads(args.plan.read_text())
        if selection.get("schema") != 1 or selection.get("source_sha") != git("rev-parse", "HEAD") or selection.get("mode") not in ("fast", "full"):
            raise ValueError("Native test plan does not match this source")
        if not args.archive:
            parser.error("archive/run requires --archive")
        if args.command == "archive":
            run(["cargo", "nextest", "archive", *build_args(selection), "--archive-file", str(args.archive)])
            # Query actual binaries instead of assuming a source module has tests.
            inventory = json.loads(run(["cargo", "nextest", "list", "--archive-file", str(args.archive),
                                        "--workspace-remap", str(ROOT), "--message-format", "json"],
                                       capture_output=True, text=True).stdout)
            if selection["mode"] == "fast" and not verify_inventory(selection, inventory):
                selection["mode"], selection["reason"] = "full", "affected module has no runnable unit tests"
                run(["cargo", "nextest", "archive", *build_args(selection), "--archive-file", str(args.archive)])
                args.plan.write_text(json.dumps(selection, indent=2) + "\n")
        else:
            if not 1 <= args.shard <= args.total:
                raise ValueError("Invalid native partition")
            run(["cargo", "nextest", "run", "--archive-file", str(args.archive), "--workspace-remap", str(ROOT),
                 "-E", expression(selection), "--partition", f"hash:{args.shard}/{args.total}",
                 "--test-threads", os.environ.get("RUST_TEST_THREADS", "num-cpus")])
    print(json.dumps(selection, indent=2))
    if output := os.environ.get("GITHUB_OUTPUT"):
        with open(output, "a") as stream:
            stream.write(f"mode={selection['mode']}\n")
    if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(summary, "a") as stream:
            stream.write(f"### Native test selection\n\nMode: **{selection['mode']}**. {selection['reason']}.\n\n")


if __name__ == "__main__":
    main()
