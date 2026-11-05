"""Negative controls for protected-main package qualification reuse."""

import copy
import hashlib
import io
import os
import json
from pathlib import Path
import runpy
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
        self.jobs = {"total_count": 4, "jobs": [
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
            for index in range(4):
                jobs = copy.deepcopy(self.jobs)
                jobs["jobs"][index]["conclusion"] = state
                self.assertFalse(subject["tested_artifact"](jobs, self.artifact))
        self.assertFalse(subject["tested_artifact"](dict(self.jobs, total_count=5), self.artifact))
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
                with self.assertRaisesRegex(ValueError, "differ from the source rebuild"):
                    checker["check"]("HEAD", source, rebuilt_directory=rebuilt)

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
                    mock.patch.dict(os.environ, GITHUB_OUTPUT=str(output)):
                subject["main"]()
            self.assertEqual(output.read_text(), "reused=false\n")


if __name__ == "__main__":
    unittest.main()
