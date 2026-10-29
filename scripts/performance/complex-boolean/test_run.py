"""Collector correctness and failure contracts; no timing thresholds."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("complex_boolean", Path(__file__).with_name("run.py"))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.case = {"id": "slot-intersect-36", "scenario": "slot_intersect", "size": 6}
        self.rows = [{"schema": "remus-complex-boolean-sample-v1", "scenario": "slot_intersect", "size": 6,
                      "sample": i, "warmup": i == 0, "validation": "passed",
                      "settings": {"store_local_requested": False, "content_memos": True, "phase_clock": False, "perf_counters": True},
                      "routes": {key: 0 for key in runner.ROUTES}, "work": {key: 0 for key in runner.WORK}, "phases": [],
                      "stages_ns": {"setup": 1000000, "preflight": 1000, "kernel": ns, "qualification": 10000, "teardown": 500},
                      "generation_ns": 1000000 + ns,
                      "metrics": {"volume": 120, "expected_volume": 120, "volume_bound": 120e-8,
                                  "input_operands_preserved": True, "faces": 6, "material_probes": 3,
                                  "edge_connected_components": 1}}
                     for i, ns in enumerate([1000000000, 2000, 4000])]

    def validate(self, rows):
        return runner.validate_records("\n".join(map(json.dumps, rows)), self.case, 2, 1, False, True)

    def test_rejects_missing_duplicate_or_misidentified_records(self):
        for rows in [self.rows[:2], self.rows + [self.rows[-1]], [self.rows[0]] * 3]:
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.validate(rows)
        for key, value in [("validation", "failed"), ("scenario", "slot_cut"), ("size", 12), ("sample", True), ("warmup", 0)]:
            rows = copy.deepcopy(self.rows)
            rows[1][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.validate(rows)

    def test_rejects_bad_timings_and_forged_totals(self):
        for bad in [float("nan"), float("inf"), -1, 0, True, "100"]:
            rows = copy.deepcopy(self.rows)
            rows[1]["stages_ns"]["kernel"] = bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.validate(rows)
        rows = copy.deepcopy(self.rows)
        rows[1]["generation_ns"] += 1
        with self.assertRaises(ValueError):
            self.validate(rows)

    def test_rejects_different_route_settings_and_timed_diagnostics(self):
        for key, value in [("store_local_requested", True), ("content_memos", False), ("phase_clock", True)]:
            rows = copy.deepcopy(self.rows)
            rows[1]["settings"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.validate(rows)

    def test_rechecks_independent_oracle_and_topology_metrics(self):
        for key, value in [("volume", 121), ("expected_volume", 121), ("volume_bound", 1000),
                           ("input_operands_preserved", False), ("material_probes", 0),
                           ("faces", 0), ("edge_connected_components", 2)]:
            rows = copy.deepcopy(self.rows)
            rows[1]["metrics"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.validate(rows)

    def test_requires_raw_counters_and_separates_phase_diagnostics(self):
        for field, value in [("work", None), ("routes", {}), ("phases", None),
                             ("phases", [{"name": "pave.ff", "calls": 1, "duration_ms": 0.5}])]:
            rows = copy.deepcopy(self.rows)
            rows[1][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.validate(rows)
        rows = copy.deepcopy(self.rows)
        rows[1]["routes"]["store_local_successes"] = 1
        with self.assertRaises(ValueError):
            self.validate(rows)

    def test_rejects_unknown_phase_labels_and_impossible_survivor_counts(self):
        rows = copy.deepcopy(self.rows)
        rows[1]["phases"] = [{"name": "unexpected.private.label", "calls": 1, "duration_ms": 0}]
        with self.assertRaises(ValueError):
            self.validate(rows)
        for kind in ("ee", "ff"):
            rows = copy.deepcopy(self.rows)
            rows[1]["work"][f"{kind}_pairs_surviving"] = 1
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.validate(rows)

    def test_warmup_excluded_and_process_medians_retained(self):
        rows = self.validate(self.rows)
        for row in rows:
            row["process"] = 0
        more = copy.deepcopy(rows)
        for row in more:
            row["process"] = 1
            row["stages_ns"]["kernel"] *= 10
        summary = runner.summarize(rows + more)
        self.assertEqual(summary["samples"], 4)
        self.assertEqual(summary["stages"]["kernel"]["process_medians"], [0.003, 0.03])
        self.assertIsNone(summary["tail_percentiles"])

    def test_manifest_has_distinct_bounded_cases_and_explicit_gaps(self):
        manifest = json.loads(runner.MANIFEST.read_text())
        self.assertEqual(len(manifest["cases"]), len({case["id"] for case in manifest["cases"]}))
        self.assertEqual({case["scenario"] for case in manifest["cases"]},
                         {"slot_cut", "slot_fuse", "slot_intersect", "fuse_sparse", "fuse_dense", "curved_half_cylinder"})
        self.assertTrue(manifest["coverage_gaps"])
        witness = next(case for case in manifest["cases"] if case["id"] == "sparse-fuse-256")
        self.assertFalse(witness["measurement_eligible"])
        self.assertIn("4096", witness["qualification_gap"])
        for case in manifest["cases"]:
            self.assertGreater(runner.expected_volume(case), 0)

    def test_nonzero_worker_is_failed_and_preserves_logs(self):
        import sys
        with tempfile.TemporaryDirectory() as directory:
            stem = Path(directory) / "failure"
            with self.assertRaisesRegex(ValueError, "exit 7"):
                runner.common.capture_worker([sys.executable, "-c", "print('partial'); raise SystemExit(7)"], stem, None, 5)
            metadata = json.loads(Path(str(stem) + ".process.json").read_text())
            self.assertEqual(metadata["status"], "failed")
            self.assertIn("partial", Path(str(stem) + ".stdout.jsonl").read_text())

    def test_source_change_rejects_samples_without_success_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            binary = directory / "worker"
            binary.write_bytes(b"fixture executable")
            args = SimpleNamespace(case=[self.case["id"]], output=directory / "out", threads=1, jobs=1,
                                   store_local="off", memos="on", phase_timings=False, counters=True, processes=1,
                                   samples=2, warmup=1, capture_cpu=None, timeout=5, binary=binary)
            with patch.object(runner.common, "source_identity", side_effect=[{"head": "before"}, {"head": "after"}]), \
                 patch.object(runner.common, "command", return_value="toolchain"), \
                 patch.object(runner.common, "capture_worker", return_value="\n".join(map(json.dumps, self.rows))), \
                 self.assertRaisesRegex(ValueError, "source changed"):
                runner.run(args)
            info = json.loads((args.output / "run.json").read_text())
            self.assertEqual(info["status"], "failed")
            self.assertFalse((args.output / "summary.json").exists())
            self.assertTrue((args.output / "samples.jsonl").exists())

    def test_initial_provenance_failure_keeps_failed_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(case=[self.case["id"]], output=Path(directory) / "out", threads=1, jobs=1,
                                   store_local="off", memos="on", phase_timings=False, counters=True, processes=1,
                                   samples=2, warmup=1, capture_cpu=None, timeout=5, binary=None)
            with patch.object(runner.common, "source_identity", side_effect=OSError("provenance unavailable")), \
                 self.assertRaisesRegex(OSError, "provenance unavailable"):
                runner.run(args)
            receipt = json.loads((args.output / "run.json").read_text())
            self.assertEqual(receipt["status"], "failed")
            self.assertIn("provenance unavailable", receipt["error"])
            self.assertIn("finished_utc", receipt)
            self.assertFalse((args.output / "summary.json").exists())

    def test_counter_free_captures_require_no_instrumentation(self):
        rows = copy.deepcopy(self.rows)
        for row in rows:
            row["settings"]["perf_counters"] = False
            row.update(work=None, routes=None, phases=[])
        text = "\n".join(map(json.dumps, rows))
        runner.validate_records(text, self.case, 2, 1, False, True, counters=False)
        rows[1]["work"] = {key: 0 for key in runner.WORK}
        with self.assertRaises(ValueError):
            runner.validate_records("\n".join(map(json.dumps, rows)), self.case, 2, 1, False, True, counters=False)

    def test_executable_replacement_rejects_samples_without_success_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            binary = directory / "worker"
            binary.write_bytes(b"original executable")
            args = SimpleNamespace(case=[self.case["id"]], output=directory / "out", threads=1, jobs=1,
                                   store_local="off", memos="on", phase_timings=False, counters=True, processes=1,
                                   samples=2, warmup=1, capture_cpu=None, timeout=5, binary=binary)
            def replace_executable(*_args):
                Path(_args[0][0]).write_bytes(b"replacement executable")
                return "\n".join(map(json.dumps, self.rows))
            with patch.object(runner.common, "source_identity", return_value={"head": "fixed"}), \
                 patch.object(runner.common, "command", return_value="toolchain"), \
                 patch.object(runner.common, "capture_worker", side_effect=replace_executable), \
                 self.assertRaisesRegex(ValueError, "executable changed"):
                runner.run(args)
            self.assertEqual(json.loads((args.output / "run.json").read_text())["status"], "failed")
            self.assertFalse((args.output / "summary.json").exists())


if __name__ == "__main__":
    unittest.main()
