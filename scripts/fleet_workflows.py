"""Resolve active workflow versions and enforce their immutable main pins."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FAMILIES = frozenset({
    "benchmark", "ci", "fuzz", "gauntlet", "mutants-sharded",
    "mutation-scope", "osv", "wasm-candidate", "wasm-size",
})
PREFIX = "esaueng/remus/.github/workflows/"
# Superseded callees removed from the tree, with the main commit that landed
# their final bytes. Historical stage proofs read them from there.
RETIRED = {
    "fleet-benchmark-v3.yml": "4e5bd846fae03e87e3d4ca1169e35b18952d4fb9",
    "fleet-ci-v7.yml": "3d648c419a95680ff5d6c9f46cb749b6a8fe0e5a",
    "fleet-mutants-sharded-v4.yml": "4e5bd846fae03e87e3d4ca1169e35b18952d4fb9",
    "fleet-mutation-scope-v3.yml": "4e5bd846fae03e87e3d4ca1169e35b18952d4fb9",
    "fleet-osv-v5.yml": "4e5bd846fae03e87e3d4ca1169e35b18952d4fb9",
    "fleet-wasm-candidate-v4.yml": "4e5bd846fae03e87e3d4ca1169e35b18952d4fb9",
}


def workflow_text(filename: str, root: Path = ROOT) -> str:
    """Read a checked-in workflow, or a retired one from its landed commit."""
    path = root / ".github/workflows" / filename
    if filename not in RETIRED:
        return path.read_text()
    if path.exists():
        raise ValueError(f"Retired workflow is back in the tree: {filename}")
    return subprocess.check_output(
        ["git", "show", f"{RETIRED[filename]}:.github/workflows/{filename}"],
        cwd=root, text=True,
    )


def is_ci_workflow(path: Path) -> bool:
    return re.fullmatch(r"fleet-ci(?:-v[1-9][0-9]*)?\.yml", path.name) is not None


def workflow_manifest(root: Path = ROOT) -> dict[str, str]:
    manifest = json.loads((root / ".github/fleet-workflows.json").read_text())
    if not isinstance(manifest, dict) or set(manifest) != FAMILIES:
        raise ValueError("The active workflow manifest must declare every fleet family")
    for family, filename in manifest.items():
        if not isinstance(filename, str) or not re.fullmatch(
            rf"fleet-{re.escape(family)}(?:-v[1-9][0-9]*)?\.yml", filename
        ):
            raise ValueError(f"Invalid workflow filename for {family}")
        if not (root / ".github/workflows" / filename).is_file():
            raise ValueError(f"Active workflow is missing: {filename}")
    return manifest


def workflow_path(family: str, root: Path = ROOT) -> Path:
    return root / ".github/workflows" / workflow_manifest(root)[family]


def parse_reference(reference: str, manifest: dict[str, str]) -> tuple[str, str]:
    match = re.fullmatch(re.escape(PREFIX) + r"(fleet-[\w-]+\.yml)@([0-9a-f]{40})", reference)
    if not match:
        raise ValueError("Fleet calls require a full immutable commit SHA")
    filename, sha = match.groups()
    if filename not in manifest.values():
        raise ValueError(f"Caller does not select an active workflow: {filename}")
    return ".github/workflows/" + filename, sha


def workflow_calls(root: Path = ROOT) -> list[tuple[Path, str, str]]:
    manifest = workflow_manifest(root)
    calls = []
    for caller in sorted((root / ".github/workflows").glob("*.yml")):
        for reference in re.findall(r"^\s+uses:\s*([^\s#]+)", caller.read_text(), re.M):
            reference = reference.strip("\"'")
            if reference.startswith("./.github/workflows/fleet-"):
                raise ValueError("Fleet calls must use an immutable main-ancestor reference")
            if reference.startswith(PREFIX + "fleet-"):
                path, sha = parse_reference(reference, manifest)
                calls.append((caller, path, sha))
    if not calls:
        raise ValueError("No immutable fleet workflow calls were found")
    return calls


def validate_reference(
    path: str, sha: str, content: bytes, root: Path = ROOT,
    main_ref: str = "origin/main",
) -> None:
    if not re.fullmatch(r"[0-9a-f]{40}", sha):
        raise ValueError("Fleet calls require a full immutable commit SHA")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", sha, main_ref],
        cwd=root, capture_output=True,
    )
    if ancestor.returncode:
        raise ValueError("Workflow pin is not a reachable main ancestor")
    pinned = subprocess.check_output(["git", "show", f"{sha}:{path}"], cwd=root)
    if pinned != content:
        raise ValueError("Checked-in workflow differs from its immutable pinned bytes")


def validate_calls(root: Path = ROOT) -> int:
    calls = workflow_calls(root)
    for _caller, path, sha in calls:
        validate_reference(path, sha, (root / path).read_bytes(), root=root)
    return len(calls)


if __name__ == "__main__":
    print(f"Validated {validate_calls()} immutable main-ancestor workflow calls")
