#!/usr/bin/env python3
"""Exercise proof orchestration with failing subprocesses and isolated outputs."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parent.parent
RUNNER = ROOT / "scripts/test-o15-proof.sh"


class ProofRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.calls = self.root / "calls.jsonl"
        self.env = {
            **os.environ,
            "PATH": f"{self.bin}{os.pathsep}{os.environ['PATH']}",
            "O15_PROOF_OUT": str(self.root / "reports"),
            "PROOF_TEST_CALLS": str(self.calls),
            "PROOF_TEST_TARGET": str(self.root / "build"),
            "TMPDIR": str(self.root),
        }
        # The stubs exercise the real shell's control flow, exit handling,
        # artifact paths and cleanup, independently of geometric assertions.
        stub = f"""#!{shutil.which('python3')}
import json, os, pathlib, sys
command = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['PROOF_TEST_CALLS'], 'a') as output:
    output.write(json.dumps([command, *args]) + '\\n')
if command == 'cargo' and args[0] == 'metadata':
    print(json.dumps({{'target_directory': os.environ['PROOF_TEST_TARGET']}}))
elif command == 'node' and args[0] == '-e':
    print(json.load(sys.stdin)['target_directory'])
elif command == 'node' and args[0].endswith('-matrix.mjs'):
    failed = os.environ.get('PROOF_TEST_FAIL') == args[0]
    print(json.dumps({{'passed': not failed}}))
    sys.exit(1 if failed else 0)
elif command == 'node' and args[0] == 'tools/parity/proof-verdict.mjs':
    sys.exit(4 if os.environ.get('PROOF_TEST_FAIL') == 'verdict' else 0)
elif command == 'wasm-pack':
    pathlib.Path(args[args.index('--out-dir') + 1]).mkdir(parents=True)
    sys.exit(3 if os.environ.get('PROOF_TEST_FAIL') == 'build' else 0)
"""
        for command in ("node", "cargo", "wasm-pack"):
            path = self.bin / command
            path.write_text(stub)
            path.chmod(0o755)

    def run_proof(self, failure=""):
        return subprocess.run(
            ["bash", str(RUNNER)], cwd=ROOT, env={**self.env, "PROOF_TEST_FAIL": failure},
            text=True, capture_output=True, check=False,
        )

    def report_dirs(self):
        return sorted((self.root / "reports").glob("run.*"))

    def test_both_reports_and_exact_distribution_arguments(self):
        result = self.run_proof()
        self.assertEqual(result.returncode, 0, result.stderr)
        reports = self.report_dirs()[0]
        self.assertEqual((reports / "status.tsv").read_text(), "contract\t0\nsplit-io\t0\n")
        for name in ("o15-contract.json", "o15-split-io.json"):
            self.assertTrue(json.loads((reports / name).read_text())["passed"])
        calls = [json.loads(line) for line in self.calls.read_text().splitlines()]
        builds = [call for call in calls if call[0] == "wasm-pack"]
        self.assertEqual(len(builds), 2)
        self.assertTrue(all("--no-opt" in call and "--locked" in call for call in builds))
        self.assertIn("--no-default-features", builds[0])
        self.assertNotIn("--no-default-features", builds[1])
        config = (reports / "build-config.txt").read_text()
        self.assertIn("wasm-pack build crates/wasm ", config)
        self.assertIn("--no-opt", config)
        self.assertIn("--locked", config)
        split = next(call for call in calls if call[1] == "tools/parity/split-io-matrix.mjs")
        self.assertEqual(split[-2:], ["crates/wasm/pkg", "crates/wasm-io/pkg"])
        self.assertFalse(list(self.root.glob("remus-o15-proof.*")), "temporary builds must be cleaned")

    def test_contract_failure_retains_both_reports_and_fails(self):
        result = self.run_proof("tools/parity/contract-matrix.mjs")
        self.assertEqual(result.returncode, 1, result.stderr)
        reports = self.report_dirs()[0]
        self.assertEqual((reports / "status.tsv").read_text(), "contract\t1\nsplit-io\t0\n")
        self.assertFalse(json.loads((reports / "o15-contract.json").read_text())["passed"])
        self.assertTrue(json.loads((reports / "o15-split-io.json").read_text())["passed"])

    def test_split_failure_remains_a_failure(self):
        result = self.run_proof("tools/parity/split-io-matrix.mjs")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual((self.report_dirs()[0] / "status.tsv").read_text(), "contract\t0\nsplit-io\t1\n")

    def test_incomplete_report_verdict_cannot_pass_on_successful_process_exits(self):
        result = self.run_proof("verdict")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual((self.report_dirs()[0] / "status.tsv").read_text(), "contract\t0\nsplit-io\t0\n")

    def test_build_failure_marks_unexecuted_slices_and_cleans(self):
        result = self.run_proof("build")
        self.assertEqual(result.returncode, 3, result.stderr)
        reports = self.report_dirs()[0]
        self.assertEqual((reports / "status.tsv").read_text(), "contract\tnot_run\nsplit-io\tnot_run\n")
        self.assertFalse(list(reports.glob("*.json")))
        self.assertFalse(list(self.root.glob("remus-o15-proof.*")))

    def test_repeated_invocations_cannot_reuse_green_artifacts(self):
        self.assertEqual(self.run_proof().returncode, 0)
        self.assertEqual(self.run_proof("build").returncode, 3)
        directories = self.report_dirs()
        self.assertEqual(len(directories), 2)
        self.assertEqual(sum((path / "o15-contract.json").exists() for path in directories), 1)


if __name__ == "__main__":
    unittest.main()
