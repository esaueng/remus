#!/usr/bin/env python3
"""Regression coverage for the exact package-attribution exemption."""

from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("check-remus-rename.sh")


class RenameTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="remus-naming-")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        self.write("NOTICE", "Historical " + "brep" + "kit attribution\n")
        self.write("source.rs", "// Remus\n")

    def write(self, path, content):
        file = self.root / path
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(content)
        subprocess.run(["git", "add", "--", path], cwd=self.root, check=True)

    def check(self):
        return subprocess.run(["bash", str(SCRIPT)], cwd=self.root,
                              capture_output=True, text=True)

    def test_exact_notices_are_allowed(self):
        for package in ("crates/wasm/pkg", "crates/wasm-io/pkg"):
            self.write(package + "/NOTICE", (self.root / "NOTICE").read_text())
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_changed_package_notice_is_rejected(self):
        self.write("crates/wasm/pkg/NOTICE", "Different attribution\n")
        result = self.check()
        self.assertEqual(result.returncode, 1)
        self.assertIn("differs from source attribution", result.stdout)

    def test_stale_source_identity_is_still_rejected(self):
        self.write("source.rs", "// " + "brep" + "kit\n")
        self.assertEqual(self.check().returncode, 1)

    def test_unlisted_notice_is_not_exempt(self):
        self.write("other/NOTICE", (self.root / "NOTICE").read_text())
        self.assertEqual(self.check().returncode, 1)


if __name__ == "__main__":
    unittest.main()
