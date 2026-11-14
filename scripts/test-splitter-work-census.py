#!/usr/bin/env python3
"""Regression coverage for the work log's input identity and aggregation."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("splitter-work-census.py")
spec = importlib.util.spec_from_file_location("splitter_work_census", SCRIPT)
census = importlib.util.module_from_spec(spec)
spec.loader.exec_module(census)


def row(identity, march):
    return dict(site="crossing", test="same_test", face=0, sections=2,
                uses=10, input=identity, march=march, adapter_ns=100)


class WorkCensusTests(unittest.TestCase):
    def test_distinct_inputs_with_equal_local_ids_stay_separate_in_any_order(self):
        # Reviewer repro: whichever input came first used to decide the max.
        rows = [row("small", 935), row("unit", 985),
                row("offset", 985), row("large", 985)]
        for ordered in (rows, list(reversed(rows))):
            retained = census.unique_faces(ordered)
            self.assertEqual(len(retained), 4)
            self.assertEqual(max(r["march"] for r in retained), 985)

    def test_repeated_input_folds_even_when_timing_changes(self):
        original = row("unit", 985)
        rerun = dict(original, adapter_ns=200)
        self.assertEqual(census.unique_faces([original, rerun]), [original])
        # The same input at different sites/tests is still separate evidence.
        self.assertEqual(len(census.unique_faces([
            original, dict(original, site="other"), dict(original, test="other"),
        ])), 3)

    def test_old_logs_retain_every_row_without_guessing_identity(self):
        rows = [row(None, 935), row(None, 985), row(None, 985)]
        self.assertEqual(census.unique_faces(rows), rows)

    def test_parser_accepts_new_and_old_schema_and_rejects_malformed_lines(self):
        fields = ["-"] * len(census.COLUMNS)
        fields[census.COLUMNS.index("input")] = "v1:0123456789abcdef"
        scratch = SCRIPT.parent.parent / "target/diag/tmp"
        scratch.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch) as directory:
            log = Path(directory) / "work.tsv"
            log.write_text("\t".join(fields) + "\n" + "\t".join(fields[:-1]) + "\n")
            rows = census.parse([log])
            self.assertEqual(rows[0]["input"], "v1:0123456789abcdef")
            self.assertIsNone(rows[1]["input"])
            self.assertIsNone(rows[0]["march"])
            err, out = io.StringIO(), io.StringIO()
            with contextlib.redirect_stderr(err), contextlib.redirect_stdout(out):
                census.report(rows, False)
            self.assertIn("1 legacy rows lack input identity", err.getvalue())
            self.assertIn("2 probed faces", out.getvalue())
            log.write_text("bad\tline\n")
            with self.assertRaises(ValueError):
                census.parse([log])


if __name__ == "__main__":
    unittest.main()
