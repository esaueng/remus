#!/usr/bin/env python3
"""Exercise package substitution and legitimate refreshes using local fixtures."""

import json
from pathlib import Path
import runpy
import unittest

HERE = Path(__file__).resolve().parent
checker = runpy.run_path(str(HERE / "check-wasm-package-integrity.py"))
Fixture = runpy.run_path(str(HERE / "test-package-refresh-pr.py"))["PackageRefreshTests"]


class IntegrityTests(unittest.TestCase):
    def setUp(self):
        fixture = Fixture()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.git = fixture.git
        self.repo = fixture.repo
        self.source = fixture.source
        self.git("reset", "--hard", self.source)
        self.contract = {"version": "2026.1.3", "name": "fixture-package",
                         "repository": {"type": "git", "url": "git+" + checker["REPOSITORY"] + ".git"},
                         "homepage": checker["REPOSITORY"],
                         "exports": {".": {"node": "./kernel.js", "default": "./browser.js"}}}
        for root in checker["ROOTS"]:
            (self.repo / root / "package.json").write_text(json.dumps(self.contract))
            (self.repo / root / "kernel.js").write_text("// generated glue\n")
        self.git("add", "crates")
        self.git("commit", "-m", "package contract fixture")
        self.revision = self.git("rev-parse", "HEAD")

    def check(self):
        checker["check"](self.revision, self.repo)

    def test_identical_payload_passes_with_version_and_publisher_stamping(self):
        for root in checker["ROOTS"]:
            generated = dict(self.contract, version="2026.1.4")
            generated.pop("homepage")
            generated["repository"] = checker["REPOSITORY"]
            (self.repo / root / "package.json").write_text(json.dumps(generated))
        self.check()

    def test_either_binary_or_glue_substitution_fails(self):
        for root in checker["ROOTS"]:
            for name in ("kernel.wasm", "kernel.js"):
                with self.subTest(root=root, name=name):
                    path = self.repo / root / name
                    original = path.read_bytes()
                    path.write_bytes(b"substituted payload")
                    with self.assertRaisesRegex(ValueError, "differ from the source rebuild"):
                        self.check()
                    path.write_bytes(original)

    def test_changed_package_contract_and_export_order_fail(self):
        path = self.repo / checker["ROOTS"][0] / "package.json"
        for altered in [dict(self.contract, scripts={"postinstall": "unexpected"}),
                        dict(self.contract, exports={".": {"default": "./browser.js", "node": "./kernel.js"}})]:
            path.write_text(json.dumps(altered))
            with self.assertRaises(ValueError):
                self.check()

    def test_extra_committed_file_is_not_generated_and_fails(self):
        extra = self.repo / checker["ROOTS"][0] / "injected.js"
        extra.write_text("unexpected executable payload")
        self.git("add", "crates")
        self.git("commit", "-m", "extra payload fixture")
        self.revision = self.git("rev-parse", "HEAD")
        extra.unlink()  # The workflow cleans the package before rebuilding.
        with self.assertRaisesRegex(ValueError, "injected.js"):
            self.check()

    def test_missing_and_extra_rebuilt_files_fail(self):
        path = self.repo / checker["ROOTS"][1] / "kernel.js"
        path.unlink()
        with self.assertRaises(ValueError):
            self.check()
        path.write_text("// generated glue\n")
        (path.parent / "extra.js").write_text("unexpected")
        with self.assertRaises(ValueError):
            self.check()

    def test_committed_provenance_cannot_redirect_consumers(self):
        path = self.repo / checker["ROOTS"][0] / "package.json"
        path.write_text(json.dumps(dict(self.contract, homepage="https://example.invalid")))
        self.git("add", "crates")
        self.git("commit", "-m", "altered provenance fixture")
        self.revision = self.git("rev-parse", "HEAD")
        with self.assertRaisesRegex(ValueError, "provenance"):
            self.check()

    def test_rebuilt_symlink_is_rejected(self):
        path = self.repo / checker["ROOTS"][0] / "kernel.js"
        path.unlink()
        path.symlink_to("package.json")
        with self.assertRaisesRegex(ValueError, "Symlink"):
            self.check()

    def test_existing_version_gate_depends_on_integrity(self):
        workflow = (HERE.parent / ".github/workflows/wasm-version.yml").read_text()
        self.assertIn("    needs: integrity", workflow.split("  version:\n", 1)[1])
        clean = workflow.index("rm -rf -- crates/wasm/pkg crates/wasm-io/pkg")
        build = workflow.index("cargo xtask wasm-build")
        compare = workflow.index("python3 scripts/check-wasm-package-integrity.py --revision HEAD")
        self.assertLess(clean, build)
        self.assertLess(build, compare)
        self.assertNotIn("pull_request_target", workflow)
        self.assertNotIn("secrets.", workflow)


if __name__ == "__main__":
    unittest.main()
