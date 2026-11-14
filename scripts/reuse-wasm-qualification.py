#!/usr/bin/env python3
"""Reuse only a protected-main artifact with identical inputs and tested bytes.

CI runs the integration base's copy of this script with ``--checkout`` naming
the pull request checkout, so a change under test cannot weaken the checks
that qualify its own committed packages.
"""

import argparse
import hashlib
from collections import Counter
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

# The scripts that do the qualifying always come from this script's own tree;
# the checkout being qualified defaults to the same tree.
SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parent
REPOSITORY = "esaueng/remus"
PACKAGE_ROOTS = ("crates/wasm/pkg", "crates/wasm-io/pkg")
# Every part of both runtime suites; scripts/test-packaged-suite-parts.mjs
# keeps this list in step with the matrix in ci-checks.yml.
REQUIRED_JOBS = {"checks / WASM Build & Validate", "checks / CI Pass",
                 *(f"checks / WASM runtime ({suite} {part}/3)"
                   for suite in ("smoke", "tarball-consumer") for part in (1, 2, 3))}
MAX_BYTES = 128 * 1024 * 1024
INTEGRITY_REASONS = {"provenance_mismatch", "package_mismatch", "invalid_package_layout"}


class QualificationTrace:
    """Describe proof selection using controlled stages and aggregate counts."""

    def __init__(self):
        self.stage = "repository"
        self.rejections = Counter()

    def reject(self, reason):
        self.rejections[reason] += 1

    def report(self, reused, reason=None):
        return {"schema": 1, "outcome": "reused" if reused else "rebuild",
                "stage": self.stage, "reason": reason or ("qualified" if reused else "no_matching_proof"),
                "rejections": dict(sorted(self.rejections.items()))}


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


def identical_inputs(source, checkout=ROOT):
    for args in (("merge-base", "--is-ancestor", source, "origin/main"),
                 ("diff", "--quiet", source, "HEAD", "--", ".",
                  ":(exclude)crates/wasm/pkg/**", ":(exclude)crates/wasm-io/pkg/**"),
                 ("diff", "--quiet", "HEAD", "--", ".",
                  ":(exclude)crates/wasm/pkg/**", ":(exclude)crates/wasm-io/pkg/**")):
        if subprocess.run(["git", *args], cwd=checkout, capture_output=True, check=False).returncode:
            return False
    return True


def reuse(trace=None, checkout=ROOT):
    trace = QualificationTrace() if trace is None else trace
    if os.environ.get("GITHUB_REPOSITORY", "").lower() != REPOSITORY.lower():
        trace.reject("repository_mismatch")
        return None
    trace.stage = "protected_main"
    if api("branches/main").get("protected") is not True:
        trace.reject("main_unprotected")
        return None
    trace.stage = "candidate_history"
    history = api("actions/workflows/ci.yml/runs?branch=main&event=push&status=success&per_page=20")
    for run in history.get("workflow_runs", []):
        if not eligible_run(run):
            trace.reject("ineligible_run")
            continue
        trace.stage = "source_inputs"
        if not identical_inputs(run["head_sha"], checkout):
            trace.reject("inputs_or_ancestry_differ")
            continue
        trace.stage = "artifact_metadata"
        artifacts = api(f"actions/runs/{run['id']}/artifacts?per_page=100")
        if artifacts.get("total_count") != len(artifacts.get("artifacts", [])):
            trace.reject("incomplete_artifact_listing")
            continue
        candidates = [a for a in artifacts["artifacts"] if a.get("name") == "committed-wasm-package"]
        if len(candidates) != 1:
            trace.reject("missing_or_duplicate_artifact")
            continue
        artifact = candidates[0]
        trace.stage = "required_jobs"
        jobs = api(f"actions/runs/{run['id']}/jobs?filter=latest&per_page=100")
        if not tested_artifact(jobs, artifact):
            trace.reject("artifact_expired" if artifact.get("expired") is True else "artifact_or_jobs_unqualified")
            continue
        trace.stage = "artifact_download"
        downloaded = api(f"actions/artifacts/{artifact['id']}/zip", binary=True)
        trace.stage = "archive_contract"
        payload = package_archive(downloaded, artifact)
        with tempfile.TemporaryDirectory(prefix="remus-qualified-package-") as temporary:
            directory = Path(temporary)
            archive = directory / "packages.tar.gz"
            archive.write_bytes(payload)
            # Never extract an Actions artifact into the source checkout. The
            # existing installer validates every path/type before extraction.
            trace.stage = "archive_install"
            subprocess.run([sys.executable, str(SCRIPTS / "install-wasm-package-archive.py"),
                            str(archive), *PACKAGE_ROOTS], cwd=directory, check=True)
            checker = runpy.run_path(str(SCRIPTS / "check-wasm-package-integrity.py"))
            trace.stage = "package_integrity"
            checker["check"]("HEAD", checkout, rebuilt_directory=directory)
            # The size gate must consume the verified bytes, even if an
            # earlier local test modified the checkout's generated files.
            trace.stage = "verified_package_install"
            subprocess.run([sys.executable, str(SCRIPTS / "install-wasm-package-archive.py"),
                            str(archive), *PACKAGE_ROOTS], cwd=checkout, check=True)
        return {"source": run["head_sha"], "run": run["html_url"],
                "artifact": artifact["id"], "digest": artifact["digest"]}
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--checkout", type=Path, default=ROOT,
                        help="checkout whose committed packages are qualified (default: this tree)")
    checkout = parser.parse_args(argv).checkout.resolve()
    proof = None
    trace = QualificationTrace()
    reason = None
    try:
        proof = reuse(trace, checkout)
    except Exception as error:
        # An unavailable, expired, malformed or mismatched proof requires the
        # unchanged clean rebuild. Never turn retrieval failure into success.
        # Never echo an exception body: remote responses or local paths may
        # contain private data. Keep the existing fail-closed rebuild fallback.
        code = getattr(error, "reason", None)
        reason = code if isinstance(code, str) and code in INTEGRITY_REASONS else "proof_unavailable"
    if proof is None:
        print("No complete protected-main qualification proof; rebuilding both packages.")
    report = trace.report(proof is not None, reason)
    print(json.dumps({"qualification": report}, sort_keys=True))
    if output := os.environ.get("GITHUB_OUTPUT"):
        with open(output, "a") as stream:
            stream.write(f"reused={str(proof is not None).lower()}\n")
    if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(summary, "a") as stream:
            stream.write("### WASM qualification selection\n\n"
                         f"Outcome: **{report['outcome']}**. Stage: `{report['stage']}`. "
                         f"Reason: `{report['reason']}`.\n\n")
            for code, count in report["rejections"].items():
                stream.write(f"- `{code}`: {count}\n")
    if proof:
        print(json.dumps(proof, indent=2))
        if summary := os.environ.get("GITHUB_STEP_SUMMARY"):
            with open(summary, "a") as stream:
                stream.write("### Exact package qualification reused\n\n"
                             f"Source: `{proof['source']}`; [successful main run]({proof['run']}).\n\n"
                             "All non-package tracked inputs and both committed package contracts/bytes match.\n")


if __name__ == "__main__":
    main()
