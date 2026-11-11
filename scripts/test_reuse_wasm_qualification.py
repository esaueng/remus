"""Negative controls for protected-main package qualification reuse."""

import copy
from contextlib import redirect_stdout
import hashlib
import io
import os
import json
from pathlib import Path
import runpy
import subprocess
import tempfile
import unittest
from unittest import mock
import zipfile

subject = runpy.run_path(str(Path(__file__).with_name("reuse-wasm-qualification.py")))


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.run = {"id": 42, "event": "push", "head_branch": "main", "status": "completed",
                    "conclusion": "success", "path": ".github/workflows/ci.yml", "run_attempt": 1,
                    "head_repository": {"full_name": "esaueng/remus"}, "head_sha": "a" * 40}
        self.jobs = {"total_count": len(subject["REQUIRED_JOBS"]), "jobs": [
            {"name": name, "status": "completed", "conclusion": "success", "started_at": "2026-11-01T12:01:00Z"}
            for name in subject["REQUIRED_JOBS"]]}
        self.artifact = {"expired": False, "created_at": "2026-11-01T12:00:00Z"}

    def test_only_first_attempt_completed_protected_main_ci_runs_are_eligible(self):
        self.assertTrue(subject["eligible_run"](self.run))
        for key, value in (("event", "pull_request"), ("event", "workflow_dispatch"),
                           ("head_branch", "topic"), ("status", "in_progress"),
                           ("conclusion", "failure"), ("run_attempt", 2),
                           ("path", ".github/workflows/other.yml"), ("head_sha", "main"),
                           ("head_repository", {"full_name": "fixture/fork"})):
            with self.subTest(key=key, value=value):
                self.assertFalse(subject["eligible_run"](dict(self.run, **{key: value})))

    def test_each_required_job_must_have_succeeded_on_the_actual_artifact(self):
        self.assertTrue(subject["tested_artifact"](self.jobs, self.artifact))
        for state in ("failure", "cancelled", "skipped", None):
            for index in range(len(self.jobs["jobs"])):
                jobs = copy.deepcopy(self.jobs)
                jobs["jobs"][index]["conclusion"] = state
                self.assertFalse(subject["tested_artifact"](jobs, self.artifact))
        self.assertFalse(subject["tested_artifact"](dict(self.jobs, total_count=len(self.jobs["jobs"]) + 1), self.artifact))
        self.assertFalse(subject["tested_artifact"](self.jobs, dict(self.artifact, expired=True)))
        self.assertFalse(subject["tested_artifact"](self.jobs, dict(self.artifact, created_at="2026-11-01T12:02:00Z")))

    def archive(self, name="committed-wasm-package.tar.gz", extra=False):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, "w") as archive:
            archive.writestr(name, b"qualified tar bytes")
            if extra:
                archive.writestr("extra.txt", b"unexpected")
        payload = stream.getvalue()
        return payload, {"digest": "sha256:" + hashlib.sha256(payload).hexdigest()}

    def test_exact_digest_and_single_expected_payload_are_required(self):
        payload, artifact = self.archive()
        self.assertEqual(subject["package_archive"](payload, artifact), b"qualified tar bytes")
        for digest in (None, "sha256:" + "0" * 64):
            with self.assertRaisesRegex(ValueError, "digest"):
                subject["package_archive"](payload, {"digest": digest})
        for name, extra in (("../committed-wasm-package.tar.gz", False),
                            ("committed-wasm-package.tar.gz", True)):
            with self.assertRaisesRegex(ValueError, "unexpected"):
                subject["package_archive"](*self.archive(name, extra))

    def test_source_comparison_covers_every_tracked_input_except_the_generated_pair(self):
        with mock.patch("subprocess.run") as run:
            run.return_value.returncode = 0
            self.assertTrue(subject["identical_inputs"]("a" * 40))
            self.assertEqual(run.call_args_list[0].args[0],
                             ["git", "merge-base", "--is-ancestor", "a" * 40, "origin/main"])
            self.assertEqual(run.call_args_list[1].args[0][-3:],
                             [".", ":(exclude)crates/wasm/pkg/**", ":(exclude)crates/wasm-io/pkg/**"])
            self.assertEqual(run.call_args_list[2].args[0][:4], ["git", "diff", "--quiet", "HEAD"])
            run.side_effect = [mock.Mock(returncode=0), mock.Mock(returncode=1)]
            self.assertFalse(subject["identical_inputs"]("a" * 40))
            run.side_effect = None
            run.return_value.returncode = 1
            self.assertFalse(subject["identical_inputs"]("a" * 40))

    def test_integrity_reads_the_committed_source_and_separate_rebuilt_tree(self):
        checker = runpy.run_path(str(Path(__file__).with_name("check-wasm-package-integrity.py")))
        contract = {"name": "fixture", "version": "1", "homepage": checker["REPOSITORY"],
                    "repository": {"type": "git", "url": "git+" + checker["REPOSITORY"] + ".git"}}
        entries = {}
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "source"
            rebuilt = Path(temporary) / "rebuilt"
            source.mkdir()
            for root in checker["ROOTS"]:
                package = rebuilt / root
                package.mkdir(parents=True)
                for name, data in (("package.json", json.dumps(contract).encode()), ("kernel.wasm", b"oracle bytes")):
                    (package / name).write_bytes(data)
                    entries[root + "/" + name] = data

            def git(command, **kwargs):
                self.assertEqual(kwargs["cwd"], source)
                if command[1] == "rev-parse":
                    return ("a" * 40).encode()
                if command[1] == "ls-tree":
                    return b"\0".join(b"100644 blob " + b"b" * 40 + b"\t" + name.encode()
                                       for name in entries if name.startswith(command[-1] + "/")) + b"\0"
                return entries[command[2].split(":", 1)[1]]

            with mock.patch("subprocess.check_output", side_effect=git):
                checker["check"]("HEAD", source, rebuilt_directory=rebuilt)
                (rebuilt / checker["ROOTS"][0] / "kernel.wasm").write_bytes(b"substituted bytes")
                with self.assertRaisesRegex(ValueError, "differ from the source rebuild") as caught:
                    checker["check"]("HEAD", source, rebuilt_directory=rebuilt)
                self.assertEqual(caught.exception.reason, "package_mismatch")

    def test_provenance_failures_have_a_stable_reason_without_weakening_the_contract(self):
        checker = runpy.run_path(str(Path(__file__).with_name("check-wasm-package-integrity.py")))
        with self.assertRaisesRegex(ValueError, "provenance") as caught:
            checker["manifest_contract"](json.dumps({"name": "fixture", "homepage": "https://example.invalid"}))
        self.assertEqual(caught.exception.reason, "provenance_mismatch")

    def test_trace_distinguishes_ineligible_runs_from_changed_inputs(self):
        trace = subject["QualificationTrace"]()
        namespace = subject["reuse"].__globals__
        history = {"workflow_runs": [dict(self.run, run_attempt=2), self.run]}
        with mock.patch.dict(namespace, api=mock.Mock(side_effect=[{"protected": True}, history]),
                             identical_inputs=mock.Mock(return_value=False)), \
                mock.patch.dict(os.environ, GITHUB_REPOSITORY=subject["REPOSITORY"]):
            self.assertIsNone(subject["reuse"](trace))
        self.assertEqual(trace.report(False)["rejections"],
                         {"ineligible_run": 1, "inputs_or_ancestry_differ": 1})

    def test_expiry_and_unqualified_runtime_suites_are_reported_but_never_reused(self):
        for expired in (False, True):
            trace = subject["QualificationTrace"]()
            artifact = dict(self.artifact, id=7, name="committed-wasm-package", expired=expired)
            jobs = copy.deepcopy(self.jobs)
            if not expired:
                jobs["jobs"][0]["conclusion"] = "failure"
            replies = [{"protected": True}, {"workflow_runs": [self.run]},
                       {"total_count": 1, "artifacts": [artifact]}, jobs]
            with mock.patch.dict(subject["reuse"].__globals__, api=mock.Mock(side_effect=replies),
                                 identical_inputs=mock.Mock(return_value=True)), \
                    mock.patch.dict(os.environ, GITHUB_REPOSITORY=subject["REPOSITORY"]):
                self.assertIsNone(subject["reuse"](trace))
            reason = "artifact_expired" if expired else "artifact_or_jobs_unqualified"
            self.assertEqual(trace.report(False)["rejections"], {reason: 1})

    def test_success_trace_still_requires_the_existing_digest_and_integrity_checks(self):
        payload, digest = self.archive()
        artifact = dict(self.artifact, **digest, id=7, name="committed-wasm-package")
        run = dict(self.run, html_url="https://example.invalid/qualified-run")
        replies = [{"protected": True}, {"workflow_runs": [run]},
                   {"total_count": 1, "artifacts": [artifact]}, self.jobs, payload]
        trace = subject["QualificationTrace"]()
        checker = mock.Mock()
        namespace = subject["reuse"].__globals__
        with mock.patch.dict(namespace, api=mock.Mock(side_effect=replies),
                             identical_inputs=mock.Mock(return_value=True)), \
                mock.patch("runpy.run_path", return_value={"check": checker}), \
                mock.patch("subprocess.run", return_value=subprocess.CompletedProcess([], 0)), \
                mock.patch.dict(os.environ, GITHUB_REPOSITORY=subject["REPOSITORY"]):
            proof = subject["reuse"](trace)
        self.assertEqual(proof, {"source": run["head_sha"], "run": run["html_url"],
                                 "artifact": 7, "digest": digest["digest"]})
        checker.assert_called_once_with("HEAD", subject["ROOT"], rebuilt_directory=mock.ANY)
        self.assertEqual(trace.report(True)["outcome"], "reused")
        self.assertEqual(trace.report(True)["stage"], "verified_package_install")

    def test_fallback_reports_only_allowlisted_reason_codes_and_never_exception_payloads(self):
        checker = runpy.run_path(str(Path(__file__).with_name("check-wasm-package-integrity.py")))
        private = "private-response-material-must-not-be-logged"
        malformed = ValueError(private)
        malformed.reason = [private]
        for error, expected in ((RuntimeError(private), "proof_unavailable"),
                                (malformed, "proof_unavailable"),
                                (checker["PackageIntegrityError"]("provenance_mismatch", private), "provenance_mismatch")):
            def unavailable(trace):
                trace.stage = "package_integrity"
                raise error

            with tempfile.TemporaryDirectory() as temporary:
                output, summary = (Path(temporary) / name for name in ("outputs", "summary"))
                with mock.patch.dict(subject["main"].__globals__, reuse=unavailable), \
                        mock.patch.dict(os.environ, GITHUB_OUTPUT=str(output), GITHUB_STEP_SUMMARY=str(summary)), \
                        redirect_stdout(io.StringIO()) as captured:
                    subject["main"]()
                self.assertEqual(output.read_text(), "reused=false\n")
                self.assertIn(expected, captured.getvalue())
                self.assertIn("package_integrity", summary.read_text())
                self.assertNotIn(private, captured.getvalue() + summary.read_text())

    def test_workflow_retains_clean_rebuild_and_size_gate_when_proof_is_missing(self):
        workflow = (Path(__file__).resolve().parents[1] / ".github/workflows/wasm-version.yml").read_text()
        self.assertIn("steps.qualification.outputs.reused != 'true'", workflow)
        self.assertIn("rm -rf -- crates/wasm/pkg crates/wasm-io/pkg", workflow)
        self.assertIn("cargo xtask wasm-build", workflow)
        self.assertIn("python3 scripts/check-wasm-package-integrity.py --revision HEAD", workflow)
        self.assertIn("node \"$base/scripts/wasm-size-gate.mjs\"", workflow)
        self.assertNotIn("contents: write", workflow)
        self.assertNotIn("secrets.", workflow)

    def test_any_unavailable_proof_emits_false_so_the_clean_build_runs(self):
        namespace = subject["main"].__globals__
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "outputs"
            with mock.patch.dict(namespace, reuse=mock.Mock(side_effect=RuntimeError("unavailable"))), \
                    mock.patch.dict(os.environ, GITHUB_OUTPUT=str(output), GITHUB_STEP_SUMMARY=""):
                subject["main"]()
            self.assertEqual(output.read_text(), "reused=false\n")


if __name__ == "__main__":
    unittest.main()
