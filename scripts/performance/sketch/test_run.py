"""Failure-path tests for the sketch baseline collector; no wall-clock assertions."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("sketch_baseline", Path(__file__).with_name("run.py"))
sketch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sketch)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.case = {
            "id": "independent_solved_100",
            "workload": "independent_solved",
            "size": 100,
            "num_params": 100,
            "num_equations": 100,
            "expected": "solved",
        }
        self.rows = [
            {
                "schema": "remus-sketch-perf-sample-v1",
                "workload": "independent_solved",
                "size": 100,
                "mode": "solve",
                "sample": i,
                "warmup": i == 0,
                "validation": "passed",
                "resource": "solved",
                "operation_ns": ns,
                "num_params": 100,
                "num_equations": 100,
                "classification": "solved",
                "metrics": {"oracle": "hypot_distance_5"},
            }
            for i, ns in enumerate([900000, 1000, 3000])
        ]

    def read(self, rows):
        return sketch.validate_records("\n".join(map(json.dumps, rows)), self.case, "solve", 2, 1)

    def test_does_not_turn_missing_or_duplicate_samples_into_success(self):
        for rows in [self.rows[:2], self.rows + [self.rows[-1]], [self.rows[0]] * 3]:
            with self.subTest(rows=len(rows)), self.assertRaises(ValueError):
                self.read(rows)

    def test_rejects_failed_wrong_case_or_unlabelled_samples(self):
        for key, value in [
            ("validation", "failed"),
            ("size", 1000),
            ("workload", "other"),
            ("mode", "detailed"),
            ("warmup", True),
            ("metrics", {}),
            ("classification", "underConstrained"),
            ("num_params", 10),
            ("num_equations", 50),
        ]:
            rows = copy.deepcopy(self.rows)
            rows[1][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.read(rows)

    def test_rejects_nonfinite_nonpositive_and_boolean_timings(self):
        for value in [float("nan"), float("inf"), -1, 0, True, "100", None]:
            rows = copy.deepcopy(self.rows)
            rows[1]["operation_ns"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.read(rows)

    def test_warmup_does_not_bias_summary_and_processes_remain_visible(self):
        rows = self.read(self.rows)
        for row in rows:
            row["process"] = 0
        other = copy.deepcopy(rows)
        for row in other:
            row["process"] = 1
            row["operation_ns"] *= 10
        summary = sketch.summarize(rows + other)
        self.assertEqual(summary["samples"], 4)
        self.assertAlmostEqual(summary["median"], 0.0065)
        self.assertEqual(summary["process_medians"], [0.002, 0.02])
        self.assertIsNone(summary["tail_percentiles"])

    def test_refused_rows_carry_no_timing_and_summarize_without_it(self):
        case = {
            "id": "coupled_chain_10000",
            "workload": "coupled_chain",
            "size": 10000,
            "num_params": 10000,
            "num_equations": 10000,
            "expected": "resource_refused",
        }
        rows = [
            {
                "schema": "remus-sketch-perf-sample-v1",
                "workload": "coupled_chain",
                "size": 10000,
                "mode": "solve",
                "sample": i,
                "warmup": False,
                "validation": "passed",
                "resource": "refused",
                "operation_ns": None,
                "num_params": 10000,
                "num_equations": 10000,
                "classification": "resource_refused",
                "metrics": {"budget_checked": True},
            }
            for i in range(2)
        ]
        out = sketch.validate_records("\n".join(map(json.dumps, rows)), case, "solve", 2, 0)
        for row in out:
            row["process"] = 0
        summary = sketch.summarize(out)
        self.assertEqual(summary["samples"], 0)
        self.assertEqual(summary["refused"], 2)
        self.assertIsNone(summary["median"])

    def test_solved_row_with_null_timing_is_rejected(self):
        rows = copy.deepcopy(self.rows)
        rows[1]["operation_ns"] = None
        with self.assertRaises(ValueError):
            self.read(rows)


if __name__ == "__main__":
    unittest.main()
