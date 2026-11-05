#!/usr/bin/env python3
"""Reuse only a protected-main artifact with identical inputs and tested bytes."""

import hashlib
import io
import json
import os
from pathlib import Path
import re
import runpy
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = "esaueng/remus"
PACKAGE_ROOTS = ("crates/wasm/pkg", "crates/wasm-io/pkg")
REQUIRED_JOBS = {"checks / WASM Build & Validate", "checks / WASM runtime (smoke)",
                 "checks / WASM runtime (tarball-consumer)", "checks / CI Pass"}
MAX_BYTES = 128 * 1024 * 1024


def eligible_run(run):
    return (run.get("event") == "push" and run.get("head_branch") == "main"
            and run.get("path") == ".github/workflows/ci.yml"
            and run.get("status") == "completed" and run.get("conclusion") == "success"
            and run.get("run_attempt") == 1
            and run.get("head_repository", {}).get("full_name", "").lower() == REPOSITORY.lower()
            and re.fullmatch(r"[0-9a-f]{40}", run.get("head_sha", "")) is not None)


def tested_artifact(jobs, artifact):
    # Restrict reuse to first attempts. A rebuilt artifact from a partial rerun
    # must never borrow runtime successes belonging to the previous artifact.
    if jobs.get("total_count") != len(jobs.get("jobs", [])) or artifact.get("expired") is not False:
        return False
    required = [job for job in jobs["jobs"] if job.get("name") in REQUIRED_JOBS]
    if len(required) != len(REQUIRED_JOBS) or {j.get("name") for j in required} != REQUIRED_JOBS:
        return False
    if any(j.get("status") != "completed" or j.get("conclusion") != "success" for j in required):
        return False
    created = artifact.get("created_at", "")
    runtime = [job for job in required if job["name"].startswith("checks / WASM runtime (")]
    return bool(created and all(created <= job.get("started_at", "") for job in runtime))


def package_archive(payload, artifact):
    if len(payload) > MAX_BYTES:
        raise ValueError("qualification artifact exceeds its size limit")
    digest = "sha256:" + hashlib.sha256(payload).hexdigest()
    if artifact.get("digest") != digest:
        raise ValueError("qualification artifact digest is missing or does not match")
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        entries = archive.infolist()
        if len(entries) != 1 or entries[0].filename != "committed-wasm-package.tar.gz":
            raise ValueError("unexpected qualification artifact payload")
        if entries[0].file_size > MAX_BYTES:
            raise ValueError("expanded qualification artifact exceeds its size limit")
        return archive.read(entries[0])


def api(path, binary=False):
    output = subprocess.check_output(["gh", "api", f"repos/{REPOSITORY}/{path}"], cwd=ROOT)
    return output if binary else json.loads(output)


def identical_inputs(source):
    for args in (("merge-base", "--is-ancestor", source, "origin/main"),
                 ("diff", "--quiet", source, "HEAD", "--", ".",
                  ":(exclude)crates/wasm/pkg/**", ":(exclude)crates/wasm-io/pkg/**"),
                 ("diff", "--quiet", "HEAD", "--", ".",
                  ":(exclude)crates/wasm/pkg/**", ":(exclude)crates/wasm-io/pkg/**")):
        if subprocess.run(["git", *args], cwd=ROOT, capture_output=True, check=False).returncode:
            return False
    return True


def reuse():
    if os.environ.get("GITHUB_REPOSITORY", "").lower() != REPOSITORY.lower():
        return None
    if api("branches/main").get("protected") is not True:
        return None
    history = api("actions/workflows/ci.yml/runs?branch=main&event=push&status=success&per_page=20")
    for run in history.get("workflow_runs", []):
        if not eligible_run(run) or not identical_inputs(run["head_sha"]):
            continue
        artifacts = api(f"actions/runs/{run['id']}/artifacts?per_page=100")
        if artifacts.get("total_count") != len(artifacts.get("artifacts", [])):
            continue
        candidates = [a for a in artifacts["artifacts"] if a.get("name") == "committed-wasm-package"]
        if len(candidates) != 1:
            continue
        artifact = candidates[0]
        jobs = api(f"actions/runs/{run['id']}/jobs?filter=latest&per_page=100")
        if not tested_artifact(jobs, artifact):
            continue
        payload = package_archive(api(f"actions/artifacts/{artifact['id']}/zip", binary=True), artifact)
        with tempfile.TemporaryDirectory(prefix="remus-qualified-package-") as temporary:
            directory = Path(temporary)
            archive = directory / "packages.tar.gz"
            archive.write_bytes(payload)
            # Never extract an Actions artifact into the source checkout. The
            # existing installer validates every path/type before extraction.
            subprocess.run([sys.executable, str(ROOT / "scripts/install-wasm-package-archive.py"),
                            str(archive), *PACKAGE_ROOTS], cwd=directory, check=True)
            checker = runpy.run_path(str(ROOT / "scripts/check-wasm-package-integrity.py"))
            checker["check"]("HEAD", ROOT, rebuilt_directory=directory)
            # The size gate must consume the verified bytes, even if an
            # earlier local test modified the checkout's generated files.
            subprocess.run([sys.executable, str(ROOT / "scripts/install-wasm-package-archive.py"),
                            str(archive), *PACKAGE_ROOTS], cwd=ROOT, check=True)
        return {"source": run["head_sha"], "run": run["html_url"],
                "artifact": artifact["id"], "digest": artifact["digest"]}
    return None


def main():
    proof = None
    try:
        proof = reuse()
    except Exception:
        # An unavailable, expired, malformed or mismatched proof requires the
        # unchanged clean rebuild. Never turn retrieval failure into success.
        print("No complete protected-main qualification proof; rebuilding both packages.")
    if output := os.environ.get("GITHUB_OUTPUT"):
        with open(output, "a") as stream:
            stream.write(f"reused={str(proof is not None).lower()}\n")
    if proof:
        print(json.dumps(proof, indent=2))
        if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
            with open(summary, "a") as stream:
                stream.write("### Exact package qualification reused\n\n"
                             f"Source: `{proof['source']}`; [successful main run]({proof['run']}).\n\n"
                             "All non-package tracked inputs and both committed package contracts/bytes match.\n")


if __name__ == "__main__":
    main()
