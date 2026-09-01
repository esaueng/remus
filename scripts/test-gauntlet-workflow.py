#!/usr/bin/env python3
"""Contract tests for the scheduled corpus-gauntlet workflow."""

from __future__ import annotations

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
WORKFLOW = ROOT / ".github" / "workflows" / "gauntlet.yml"


class GauntletWorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.text = WORKFLOW.read_text(encoding="utf-8")
        cls.run_job, cls.publish_job = cls.text.split("\n  publish:\n", maxsplit=1)

    def test_declares_daily_smoke_and_weekly_abc_schedules(self) -> None:
        self.assertIn('- cron: "17 3 * * *"', self.text)
        self.assertIn('- cron: "47 3 * * 0"', self.text)
        self.assertIn('if [ "$SCHEDULE" = "47 3 * * 0" ]', self.run_job)
        self.assertIn("tier=abc-1k", self.run_job)
        self.assertIn("tier=smoke", self.run_job)

    def test_runner_is_read_only_and_publisher_is_main_only(self) -> None:
        self.assertIn("permissions: {}", self.text)
        self.assertIn("contents: read", self.run_job)
        self.assertNotIn("contents: write", self.run_job)
        self.assertNotIn("secrets.GITHUB_TOKEN", self.run_job)
        self.assertIn("contents: write", self.publish_job)
        self.assertIn("github.ref == 'refs/heads/main'", self.publish_job)
        self.assertNotIn("actions/checkout@", self.publish_job)
        self.assertNotIn("cargo ", self.publish_job)

    def test_corpus_execution_is_bounded_and_ratchets_every_stage(self) -> None:
        self.assertIn("--timeout-ms 30000", self.run_job)
        self.assertIn("--jobs 2", self.run_job)
        self.assertIn('MAX_STAGE_DROP_BPS: "50"', self.text)
        self.assertIn("remus-gauntlet trend", self.run_job)
        self.assertIn("--max-drop-bps", self.run_job)

    def test_only_aggregate_outputs_are_published(self) -> None:
        self.assertIn("scoreboard.json", self.text)
        self.assertIn("scoreboard.md", self.text)
        self.assertIn("trend-row.json", self.text)
        self.assertNotIn("models.jsonl", self.text)
        self.assertIn("HEAD:results", self.publish_job)
        self.assertIn("trends/${tier}.jsonl", self.publish_job)

    def test_actions_are_immutable_and_credentials_do_not_persist(self) -> None:
        self.assertIn("persist-credentials: false", self.run_job)
        for line in self.text.splitlines():
            if "uses:" not in line:
                continue
            reference = line.split("uses:", maxsplit=1)[1].strip().split()[0]
            self.assertRegex(reference, r"@[0-9a-f]{40}$")


if __name__ == "__main__":
    unittest.main()
