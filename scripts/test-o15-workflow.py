#!/usr/bin/env python3
"""Guard the scheduled O1.5 proof's triggers, privileges and failure evidence.

These structural checks use the same fixed indentation conventions as the
workflow. Actionlint supplies full YAML/expression validation separately.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
WORKFLOW = ROOT / ".github/workflows/parity.yml"
HOSTS = [
    ("ubuntu-24.04", "linux", "x64", "x86_64-unknown-linux-gnu"),
    ("ubuntu-24.04-arm", "linux", "arm64", "aarch64-unknown-linux-gnu"),
    ("macos-15-intel", "darwin", "x64", "x86_64-apple-darwin"),
    ("macos-15", "darwin", "arm64", "aarch64-apple-darwin"),
    ("windows-2025", "win32", "x64", "x86_64-pc-windows-msvc"),
]


def block(text: str, key: str, indent: int = 0) -> str:
    """Extract one uniquely declared mapping at the requested indentation."""
    prefix = " " * indent
    matches = list(re.finditer(rf"(?m)^{prefix}{re.escape(key)}:\s*$", text))
    if len(matches) != 1:
        raise ValueError(f"expected exactly one {key} mapping at indent {indent}")
    start = matches[0].end()
    for line in text[start:].splitlines(keepends=True):
        if line.strip() and not line.lstrip().startswith("#"):
            depth = len(line) - len(line.lstrip())
            if depth <= indent:
                break
        start += len(line)
    return text[matches[0].end():start]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_policy(text: str) -> None:
    events = block(text, "on")
    schedule = block(events, "schedule", 2)
    require(re.search(r'cron: [\'\"]\d+ \d+ \* \* \*[\'\"]', schedule) is not None,
            "a daily UTC schedule is required")
    require(re.search(r"(?m)^  workflow_dispatch:\s*$", events) is not None,
            "manual proof dispatch is required")
    pr = block(events, "pull_request", 2)
    for path in (".github/workflows/parity.yml", "tools/parity/**", "scripts/test-o15*"):
        require(f"'{path}'" in pr, f"PR validation must cover {path}")
    require("pull_request_target" not in events and "secrets." not in text,
            "proof execution cannot receive privileged event credentials")
    require(block(text, "permissions").strip() == "contents: read",
            "proof permissions must remain contents: read only")
    permission_lines = re.findall(r"(?m)^\s*permissions:", text)
    require(len(permission_lines) == 1 and not re.search(r"(?m)^\s*[^#\n]+:\s*write\s*$", text),
            "job/step write permissions cannot widen the read-only token")
    require("persist-credentials: false" in text, "checkout credentials must not persist")
    require("O15_PROOF_OUT: target/o15-proof" in text, "artifact paths must work in native Windows and Git Bash")
    require("git config --global core.autocrlf false" in text
            and text.index("git config --global core.autocrlf false") < text.index("uses: actions/checkout@"),
            "Windows checkout must preserve canonical package bytes before checkout")
    require("cancel-in-progress: false" in block(text, "concurrency"),
            "new events must not cancel in-flight proof evidence")
    require('RUST_MIN_STACK: "33554432"' in block(text, "env"),
            "parity builds require the qualified Rust compiler stack size")
    require('rust_min_stack=%s\\n' in text and '"$RUST_MIN_STACK"' in text,
            "workflow artifacts must record the Rust compiler stack size")

    job = block(block(text, "jobs"), "proof", 2)
    strategy = block(job, "strategy", 4)
    require("fail-fast: false" in strategy, "a failed host must not cancel peer proofs")
    matrix = block(strategy, "matrix", 6)
    includes = block(matrix, "include", 8)
    hosts = re.findall(
        r"(?m)^          - os: ([\w.-]+)\n"
        r"            platform: (\w+)\n"
        r"            arch: (\w+)\n"
        r"            rust_host: ([\w-]+)$",
        includes,
    )
    require(hosts == HOSTS and len(re.findall(r"(?m)^          - ", includes)) == len(HOSTS),
            "the five native Linux/macOS/Windows architecture tuples must remain explicit")
    expected = block(job, "env", 4)
    for field, axis in (("RUNNER", "os"), ("PLATFORM", "platform"), ("ARCH", "arch"), ("RUST_HOST", "rust_host")):
        require(f"O15_EXPECT_{field}: ${{{{ matrix.{axis} }}}}" in expected,
                f"actual host validation requires the matrix {field} expectation")
    require("runs-on: ${{ matrix.os }}" in job, "runner selection must use the declared host")
    require("O15_PYTHON: ${{ matrix.platform == 'win32' && 'python' || 'python3' }}" in expected,
            "policy tests require the native Python executable on each host")
    require("runner." not in expected, "runner context is unavailable in job environment expressions")
    require("shell: bash" in job, "the proof runner requires bash")
    job_budget = re.findall(r"(?m)^    timeout-minutes: (\d+)$", job)
    require(len(job_budget) == 1 and 1 <= int(job_budget[0]) <= 120,
            "proof job must have a bounded timeout")
    steps = re.split(r"(?m)^      - ", block(job, "steps", 4))[1:]
    proofs = [step for step in steps if "node scripts/test-o15-proof.mjs" in step]
    require(len(proofs) == 1, "execute the shared proof runner exactly once")
    proof = proofs[0]
    require("continue-on-error" not in proof and "||" not in proof and "if:" not in proof,
            "proof failures must fail the job")
    require("set -o pipefail" in proof,
            "log pipelines must propagate proof failures")
    require('node scripts/test-o15-proof.mjs 2>&1 | tee "$O15_PROOF_OUT/run.log"' in proof,
            "proof stdout and stderr must be retained in the uploaded log")
    proof_budget = re.findall(r"(?m)^        timeout-minutes: (\d+)$", proof)
    require(len(proof_budget) == 1 and 1 <= int(proof_budget[0]) < int(job_budget[0]),
            "proof timeout must leave job time for failure uploads")
    uploads = [step for step in steps if "uses: actions/upload-artifact@" in step]
    require(len(uploads) == 1, "one proof artifact upload is required")
    upload = uploads[0]
    require(steps.index(upload) > steps.index(proof), "upload must follow proof execution")
    require("if: always()" in upload, "failed proof reports and logs must upload")
    require(re.search(r"(?m)^          path: \$\{\{ env.O15_PROOF_OUT \}\}/$", upload) is not None
            and "if-no-files-found: error" in upload,
            "upload must retain the entire proof directory and reject missing evidence")
    for identity in ("matrix.os", "runner.arch", "github.sha", "github.run_attempt"):
        require(f"${{{{ {identity} }}}}" in upload, f"artifact identity must include {identity}")
    for reference in re.findall(r"(?m)^\s*-?\s*uses:\s*(\S+)", text):
        require(re.search(r"@[0-9a-f]{40}$", reference) is not None,
                f"Action reference is not immutable: {reference}")


class ParityWorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.text = WORKFLOW.read_text(encoding="utf-8")

    def test_registered_proof_policy(self) -> None:
        validate_policy(self.text)

    def test_proof_driver_semantics_run_before_the_real_build(self) -> None:
        proof = self.text.index("node scripts/test-o15-proof.mjs")
        for command in ('"$O15_PYTHON" scripts/test-o15-workflow.py', '"$O15_PYTHON" scripts/test-o15-proof.py'):
            self.assertLess(self.text.index(command), proof)

    def reject_edit(self, before: str, after: str, reason: str) -> None:
        self.assertIn(before, self.text)
        with self.assertRaisesRegex(ValueError, reason):
            validate_policy(self.text.replace(before, after, 1))

    def test_removing_daily_schedule_is_rejected(self) -> None:
        self.reject_edit('cron: "43 5 * * *"', 'cron: "43 5 * * 0"', "daily")

    def test_removing_manual_dispatch_is_rejected(self) -> None:
        self.reject_edit("  workflow_dispatch:\n", "", "manual")

    def test_missing_workflow_pr_path_is_rejected(self) -> None:
        self.reject_edit("      - '.github/workflows/parity.yml'\n", "", "PR validation")

    def test_write_permissions_are_rejected(self) -> None:
        self.reject_edit("  contents: read", "  contents: write", "contents: read")

    def test_persistent_checkout_credentials_are_rejected(self) -> None:
        self.reject_edit("persist-credentials: false", "persist-credentials: true", "must not persist")

    def test_platform_fail_fast_is_rejected(self) -> None:
        self.reject_edit("fail-fast: false", "fail-fast: true", "peer proofs")

    def test_missing_platform_tuple_is_rejected(self) -> None:
        self.reject_edit(
            "          - os: macos-15\n            platform: darwin\n"
            "            arch: arm64\n            rust_host: aarch64-apple-darwin\n",
            "", "five native",
        )

    def test_wrong_platform_architecture_is_rejected(self) -> None:
        self.reject_edit("            rust_host: aarch64-apple-darwin",
                         "            rust_host: x86_64-apple-darwin", "five native")

    def test_missing_runtime_architecture_expectation_is_rejected(self) -> None:
        self.reject_edit("      O15_EXPECT_ARCH: ${{ matrix.arch }}\n", "", "matrix ARCH")

    def test_missing_compiler_stack_configuration_is_rejected(self) -> None:
        self.reject_edit('  RUST_MIN_STACK: "33554432"\n', "", "compiler stack size")

    def test_softened_proof_failures_are_rejected(self) -> None:
        self.reject_edit('tee "$O15_PROOF_OUT/run.log"', 'tee "$O15_PROOF_OUT/run.log" || true',
                         "failures must fail")

    def test_missing_pipefail_is_rejected(self) -> None:
        self.reject_edit("          set -o pipefail\n", "", "propagate proof failures")

    def test_missing_stderr_capture_is_rejected(self) -> None:
        self.reject_edit("test-o15-proof.mjs 2>&1 | tee", "test-o15-proof.mjs | tee", "stdout and stderr")

    def test_normalized_windows_checkout_bytes_cannot_replace_canonical_bytes(self) -> None:
        self.reject_edit("git config --global core.autocrlf false", "git config --global core.autocrlf true", "canonical package bytes")

    def test_windows_tuple_cannot_disappear(self) -> None:
        self.reject_edit("          - os: windows-2025\n            platform: win32\n"
                         "            arch: x64\n            rust_host: x86_64-pc-windows-msvc\n",
                         "", "five native")

    def test_job_environment_cannot_use_unavailable_runner_context(self) -> None:
        self.reject_edit("matrix.platform == 'win32'", "runner.os == 'Windows'", "native Python executable")

    def test_missing_proof_log_is_rejected(self) -> None:
        self.reject_edit(' | tee "$O15_PROOF_OUT/run.log"', "", "uploaded log")

    def test_proof_timeout_must_allow_uploads(self) -> None:
        self.reject_edit("        timeout-minutes: 75", "        timeout-minutes: 90", "leave job time")

    def test_success_only_upload_is_rejected(self) -> None:
        self.reject_edit("if: always()", "if: success()", "failed proof reports")

    def test_report_only_upload_is_rejected(self) -> None:
        self.reject_edit("path: ${{ env.O15_PROOF_OUT }}/", "path: ${{ env.O15_PROOF_OUT }}/*.json",
                         "entire proof directory")

    def test_mutable_action_pin_is_rejected(self) -> None:
        self.reject_edit("actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1", "actions/checkout@v7",
                         "not immutable")

    def test_artifact_without_platform_identity_is_rejected(self) -> None:
        self.reject_edit("o15-proof-${{ matrix.os }}-${{ runner.arch }}", "o15-proof-linux", "matrix.os")


if __name__ == "__main__":
    unittest.main()
