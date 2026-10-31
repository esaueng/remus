"""Failure-path tests for the baseline collector; no wall-clock performance assertions."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest
import tempfile
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("baseline", Path(__file__).with_name("run.py"))
baseline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(baseline)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.case = {"scenario": "native_chain", "size": 128}
        self.rows = [{"schema": "remus-performance-sample-v1", **self.case, "sample": i,
                      "warmup": i == 0, "operation_ns": ns, "validation": "passed",
                      "metrics": {"points": 128}} for i, ns in enumerate([900000, 1000, 3000])]

    def read(self, rows):
        return baseline.validate_records("\n".join(map(json.dumps, rows)), self.case, 2, 1)

    def test_does_not_turn_missing_or_duplicate_samples_into_success(self):
        for rows in [self.rows[:2], self.rows + [self.rows[-1]], [self.rows[0]] * 3]:
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.read(rows)

    def test_rejects_failed_wrong_case_or_unlabelled_samples(self):
        for key, value in [("validation", "failed"), ("size", 512), ("scenario", "other"),
                           ("warmup", True), ("metrics", {})]:
            rows = copy.deepcopy(self.rows)
            rows[1][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.read(rows)

    def test_rejects_nonfinite_nonpositive_and_boolean_timings(self):
        for value in [float("nan"), float("inf"), -1, 0, True, "100"]:
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
        summary = baseline.summarize(rows + other)
        self.assertEqual(summary["samples"], 4)
        self.assertAlmostEqual(summary["median"], 0.0065)
        self.assertEqual(summary["process_medians"], [0.002, 0.02])
        self.assertIsNone(summary["tail_percentiles"])

    def test_nonzero_worker_preserves_output_and_cannot_supply_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            stem = Path(directory) / "failed.0"
            with self.assertRaisesRegex(ValueError, "exit 7"):
                baseline.capture_worker([sys.executable, "-c", "print('partial sample'); raise SystemExit(7)"], stem, None, 5)
            self.assertIn("partial sample", Path(str(stem) + ".stdout.jsonl").read_text())
            metadata = json.loads(Path(str(stem) + ".process.json").read_text())
            self.assertEqual(metadata["exit_code"], 7)
            self.assertEqual(metadata["status"], "failed")

    def test_timeout_is_a_recorded_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            stem = Path(directory) / "timeout.0"
            with self.assertRaises(subprocess.TimeoutExpired):
                baseline.capture_worker([sys.executable, "-c", "import time; time.sleep(10)"], stem, None, 0.05)
            metadata = json.loads(Path(str(stem) + ".process.json").read_text())
            self.assertEqual(metadata["status"], "failed")
            self.assertIn("timed out", metadata["error"])

    def test_source_change_rejects_complete_samples_without_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            manifest_path = directory / "workloads.json"
            manifest = {"schema": "remus-performance-workloads-v1", "fixtures": [],
                        "cases": [{"id": "case", "family": "transform", "scenario": "wasm_transform_direct", "size": 50}]}
            manifest_path.write_text(json.dumps(manifest))
            rows = copy.deepcopy(self.rows)
            for row in rows:
                row.update(scenario="wasm_transform_direct", size=50)
            args = SimpleNamespace(output=directory / "out", family=None, threads=1, jobs=1,
                                   samples=2, warmup=1, processes=1, timeout=5)
            with patch.object(baseline, "MANIFEST", manifest_path), \
                 patch.object(baseline, "source_identity", side_effect=[{"head": "before"}, {"head": "after"}]), \
                 patch.object(baseline, "command", return_value="test toolchain"), \
                 patch.object(baseline, "capture_worker", return_value="\n".join(map(json.dumps, rows))), \
                 self.assertRaisesRegex(ValueError, "source changed"):
                baseline.run(args)
            status = json.loads((args.output / "run.json").read_text())
            self.assertEqual(status["status"], "failed")
            self.assertFalse((args.output / "summary.json").exists())
            self.assertTrue((args.output / "samples.jsonl").exists())

    def test_manifest_preserves_all_four_families_and_artifact_separation(self):
        manifest = json.loads(baseline.MANIFEST.read_text())
        cases = manifest["cases"]
        self.assertEqual(len(cases), len({c["id"] for c in cases}))
        self.assertEqual({c["family"] for c in cases}, {"nurbs", "transform", "chain", "boolean", "warm"})
        for path in ["native_transform_direct", "native_transform_batch", "wasm_transform_direct", "wasm_transform_batch"]:
            self.assertEqual([c["size"] for c in cases if c["scenario"] == path], [50, 200, 400])
        for path in ["native_bool_fuse", "native_bool_refused", "wasm_bool_fuse", "wasm_bool_refused"]:
            self.assertEqual([c["size"] for c in cases if c["scenario"] == path], [50, 200, 400])
        for fixture in manifest["fixtures"]:
            self.assertEqual(baseline.digest(baseline.ROOT / fixture["path"]), fixture["sha256"])

    def test_warm_cases_pair_native_and_wasm_on_the_same_workload(self):
        cases = [c for c in json.loads(baseline.MANIFEST.read_text())["cases"] if c["family"] == "warm"]
        native = {(c["scenario"].removeprefix("native_"), c["size"]) for c in cases if c["scenario"].startswith("native_")}
        wasm = {(c["scenario"].removeprefix("wasm_"), c["size"]) for c in cases if c["scenario"].startswith("wasm_")}
        self.assertEqual(native, wasm)
        self.assertEqual({w for w, _ in native}, {"warm_measure", "warm_edit", "warm_cuts", "warm_plate"})


class CacheStatsTests(unittest.TestCase):
    def rows(self, stats):
        return [{"warmup": i == 0, "metrics": {"cache_stats": s, "cache_entries": {"pcurve": 3},
                 "phases_ms": {"a": 1.0 + i, "b": 2.0}}} for i, s in enumerate(stats)]

    def test_equal_counters_are_reported_once(self):
        stats = {"pcurve": {"hits": 4, "misses": 2}}
        self.assertEqual(baseline.cache_stats(self.rows([stats] * 3)),
                         {"cache_stats": stats, "cache_entries": {"pcurve": 3}})

    def test_counters_that_differ_between_samples_reject_the_case(self):
        rows = self.rows([{"pcurve": {"hits": 4}}, {"pcurve": {"hits": 4}}, {"pcurve": {"hits": 5}}])
        with self.assertRaisesRegex(ValueError, "differ"):
            baseline.cache_stats(rows)

    def test_counters_missing_from_one_sample_reject_the_case(self):
        rows = self.rows([{"pcurve": {"hits": 4}}] * 2)
        del rows[1]["metrics"]["cache_stats"]
        with self.assertRaisesRegex(ValueError, "missing"):
            baseline.cache_stats(rows)

    def test_cases_without_counters_report_none(self):
        self.assertIsNone(baseline.cache_stats([{"metrics": {"points": 1}}]))

    def test_phase_medians_skip_warmup(self):
        self.assertEqual(baseline.phase_medians(self.rows([{}] * 3)), {"a": 2.5, "b": 2.0})


if __name__ == "__main__":
    unittest.main()
