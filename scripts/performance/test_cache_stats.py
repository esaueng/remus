"""Tests for the cache-counter comparison; no performance assertions."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("cache_stats", Path(__file__).with_name("cache_stats.py"))
cache_stats = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cache_stats)

CASE = {"cache_stats": {"pcurve": {"hits": 4, "misses": 2}}, "cache_entries": {"pcurve": 2}}


class CompareTests(unittest.TestCase):
    def test_identical_runs_have_no_differences(self):
        self.assertEqual(cache_stats.differences({"a": CASE}, {"a": CASE}), [])

    def test_a_moved_counter_is_named(self):
        moved = json.loads(json.dumps(CASE))
        moved["cache_stats"]["pcurve"]["hits"] = 3
        self.assertEqual(cache_stats.differences({"a": CASE}, {"a": moved}),
                         ["a: cache_stats.pcurve.hits: 4 -> 3"])

    def test_a_new_or_dropped_counter_is_a_difference(self):
        grown = json.loads(json.dumps(CASE))
        grown["cache_stats"]["crossing"] = {"hits": 0}
        self.assertEqual(cache_stats.differences({"a": CASE}, {"a": grown}),
                         ["a: cache_stats.crossing.hits: None -> 0"])

    def test_a_case_measured_by_one_run_only_is_a_difference(self):
        self.assertEqual(cache_stats.differences({"a": CASE}, {}), ["a: only in baseline"])

    def test_load_rejects_an_unknown_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "cache_stats.json").write_text(json.dumps({"schema": "other", "cases": {}}))
            with self.assertRaises(ValueError):
                cache_stats.load(directory)


if __name__ == "__main__":
    unittest.main()
