"""Superseded PR package builds must not cancel merge-candidate verdicts."""

from pathlib import Path
import re
import unittest


class PackageConcurrencyTests(unittest.TestCase):
    def test_only_prs_share_a_cancellable_group(self):
        workflow = (Path(__file__).resolve().parents[1] / ".github/workflows/wasm-version.yml").read_text()
        group = re.search(r"^  group: (.+)$", workflow, re.M)
        self.assertIsNotNone(group, "package rebuilds need superseded-PR cancellation")
        self.assertEqual(group[1], "wasm-version-${{ github.event_name == 'pull_request' && github.ref || github.run_id }}")
        self.assertIn("  cancel-in-progress: ${{ github.event_name == 'pull_request' }}\n", workflow)
        self.assertIn("  pull_request:\n  merge_group:\n", workflow)
        version = workflow.split("\n  version:\n", 1)[1]
        self.assertIn("    needs: integrity\n", version)
        self.assertIn("    if: ${{ always() }}\n", version)
        self.assertIn('if [[ "$INTEGRITY_RESULT" != "success" ]]', version)


if __name__ == "__main__":
    unittest.main()
