#!/usr/bin/env python3
"""Exercise the wildcard ratchet against tracked Rust fixtures."""
import pathlib
import subprocess
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).with_name("check-wildcard-arms.sh")


class WildcardRatchetTests(unittest.TestCase):
    def check_fixture(self, literal, baseline):
        with tempfile.TemporaryDirectory(prefix="wildcard-ratchet-") as directory:
            root = pathlib.Path(directory)
            (root / "scripts").mkdir()
            (root / "crates/probe/src").mkdir(parents=True)
            (root / "scripts/check-wildcard-arms.sh").write_bytes(SCRIPT.read_bytes())
            (root / "scripts/wildcard-arms-baseline.txt").write_text(baseline)
            (root / "crates/probe/src/lib.rs").write_text(
                "fn probe(curve: EdgeCurve) -> char {\n"
                "    match curve {\n"
                f"        EdgeCurve::Line => {literal},\n"
                "        _ => 'x',\n"
                "    }\n}\n"
            )
            subprocess.run(["git", "init", "-q", directory], check=True)
            subprocess.run(["git", "-C", directory, "add", "."], check=True)
            return subprocess.run(
                ["bash", "scripts/check-wildcard-arms.sh"], cwd=root,
                text=True, capture_output=True, check=False,
            )

    def test_character_braces_cannot_hide_new_wildcards(self):
        for literal in ("'{'", "'}'", "'('", "')'", "'['", "']'"):
            with self.subTest(literal=literal):
                result = self.check_fixture(literal, "")
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("outside the baseline", result.stdout)

    def test_character_braces_do_not_change_existing_count(self):
        for literal in ("'{'", "'}'"):
            with self.subTest(literal=literal):
                result = self.check_fixture(literal, "1 crates/probe/src/lib.rs\n")
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("1 audited arms", result.stdout)

    def check_repo_tree(self, mutate=None):
        """Run the gate against a copy of THIS repo tree (proves it can fail)."""
        import shutil
        with tempfile.TemporaryDirectory(prefix="wildcard-ratchet-repo-") as directory:
            root = pathlib.Path(directory)
            for name in ("scripts", "crates"):
                shutil.copytree(
                    pathlib.Path(__file__).parent.parent / name,
                    root / name,
                    ignore=shutil.ignore_patterns("target"),
                )
            subprocess.run(["git", "init", "-q", directory], check=True)
            subprocess.run(["git", "-C", directory, "add", "."], check=True)
            if mutate is not None:
                mutate(root)
                subprocess.run(["git", "-C", directory, "add", "."], check=True)
            return subprocess.run(
                ["bash", "scripts/check-wildcard-arms.sh"], cwd=root,
                text=True, capture_output=True, check=False,
            )

    def test_repo_tree_reports_current_counts(self):
        # The checked-in tree is mid-B24: the phase_ff/nonplanar conversions
        # ride stacked PRs and the fallout re-baseline rides this one, so the
        # gate may report growth or staleness here. What it must never do is
        # fail to RUN (exit 2) or pass vacuously (exit 0 with no report).
        result = self.check_repo_tree()
        self.assertIn(result.returncode, (0, 1), result.stdout + result.stderr)
        self.assertTrue(
            "wildcard-arm ratchet OK" in result.stdout
            or "VIOLATION" in result.stdout
            or "STALE" in result.stdout,
            result.stdout + result.stderr,
        )

    def test_repo_tree_fails_on_added_wildcard(self):
        def add_arm(root):
            probe = root / "crates/probe/src/lib.rs"
            probe.parent.mkdir(parents=True)
            probe.write_text(
                "fn probe(curve: EdgeCurve) -> u8 {\n"
                "    match curve {\n"
                "        EdgeCurve::Line => 0,\n"
                "        _ => 1,\n"
                "    }\n}\n"
            )
        result = self.check_repo_tree(mutate=add_arm)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("outside the baseline", result.stdout)


if __name__ == "__main__":
    unittest.main()
