"""Reject incomplete, invalid or misattributed preview measurements."""

import copy
import unittest

from compare_preview import validate_capture


def capture(background_slots=200):
    initial = background_slots + 3
    result = {
        "metadata": {"timingStatus": "measured", "timingsExcluded": False,
                     "kernel": {"wasmSha256": "kernel"}, "io": {"wasmSha256": "io"},
                     "harnessSha256": "driver", "sourceTopologySlots": 3},
        "records": [{"tick": i, "accepted": True, "stagesMs": {"tick": 12.0},
                     "quality": {"watertight": True, "boundaryEdges": 0,
                                 "nonManifoldEdges": 0, "faces": 160,
                                 "capAtRequestedOffset": True}} for i in range(6)],
        "summary": {"refused": 0, "warmedSamples": 4, "warmed": {"tick": {"n": 4}},
                    "cold": {"stagesMs": {}}, "memoryBefore": {"allocatedSlots": initial, "retiredSlots": 0}}
    }
    allocated, retired = initial, 0
    for i, row in enumerate(result["records"]):
        before = {"allocatedSlots": allocated, "retiredSlots": retired}
        if i:
            retired += 3
        restore = {"allocatedSlots": allocated, "retiredSlots": retired}
        allocated += initial + 3 if i == 0 else 3
        move = {"allocatedSlots": allocated, "retiredSlots": retired}
        row["memoryStages"] = {"before": before, "restore": restore, "move": move,
                               **{stage: move.copy() for stage in ["validate", "mesh", "transfer"]}}
        row["memoryAfter"] = move.copy()
    result["summary"]["memoryLast"] = result["records"][-1]["memoryAfter"].copy()
    return result


class PreviewCaptureTests(unittest.TestCase):
    def validate(self, run):
        return validate_capture(run, 6, 2, "kernel", "io", "driver")

    def test_valid_capture_retains_actual_sample_count(self):
        self.assertEqual(self.validate(capture())["samples"], 4)
        self.assertEqual(self.validate(capture())["workContract"]["maximumRecurringSlots"], 3)

    def test_raw_topology_snapshots_override_a_claimed_passing_contract(self):
        run = capture()
        run["workContract"] = {"passed": True, "maximumRecurringSlots": 3}
        run["records"][1]["memoryStages"]["move"]["allocatedSlots"] += 203
        with self.assertRaisesRegex(ValueError, "topology work budget"):
            self.validate(run)

    def test_topology_gate_rejects_read_allocations_retirement_and_tampered_totals(self):
        for mutate in [
            lambda r: r["records"][2]["memoryStages"]["validate"].update(allocatedSlots=500),
            lambda r: r["records"][2]["memoryStages"]["move"].update(retiredSlots=4),
            lambda r: r["records"][2]["memoryAfter"].update(allocatedSlots=1),
            lambda r: r["summary"]["memoryLast"].update(allocatedSlots=1000),
            lambda r: r["metadata"].update(sourceTopologySlots=True),
            lambda r: r["records"][2]["memoryStages"]["mesh"].update(allocatedSlots=float("nan")),
            lambda r: r["records"][2]["memoryStages"].pop("mesh"),
        ]:
            run = capture()
            mutate(run)
            with self.assertRaises(ValueError):
                self.validate(run)

    def test_fixed_append_budget_cannot_hide_unretired_live_results(self):
        run = capture(0)
        # Model a small document so retaining just one previous result exceeds
        # its live budget. Its append counters and every stage delta stay valid.
        for row in run["records"]:
            for snapshot in [*row["memoryStages"].values(), row["memoryAfter"]]:
                snapshot["retiredSlots"] = 0
        run["summary"]["memoryLast"] = run["records"][-1]["memoryAfter"].copy()
        with self.assertRaisesRegex(ValueError, "Live topology"):
            self.validate(run)

    def test_rejects_artifact_and_timing_policy_mismatches(self):
        base = capture()
        for key, value in [("timingStatus", "contended-smoke-excluded"),
                           ("timingsExcluded", True), ("harnessSha256", "different")]:
            with self.subTest(key=key):
                run = copy.deepcopy(base)
                run["metadata"][key] = value
                with self.assertRaises(ValueError):
                    self.validate(run)
        for package in ["kernel", "io"]:
            run = capture()
            run["metadata"][package]["wasmSha256"] = "different"
            with self.assertRaises(ValueError):
                self.validate(run)

    def test_rejects_missing_and_refused_samples(self):
        for mutation in [lambda r: r["records"].pop(),
                         lambda r: r["summary"].update(refused=1),
                         lambda r: r["records"][2].update(accepted=False),
                         lambda r: r["summary"].update(warmedSamples=3)]:
            run = capture()
            mutation(run)
            with self.assertRaises(ValueError):
                self.validate(run)

    def test_rejects_invalid_geometry(self):
        for key, value in [("watertight", False), ("boundaryEdges", 1),
                           ("nonManifoldEdges", 1), ("faces", 159),
                           ("capAtRequestedOffset", False)]:
            run = capture()
            run["records"][2]["quality"][key] = value
            with self.assertRaises(ValueError):
                self.validate(run)

    def test_rejects_nonfinite_or_negative_durations(self):
        for value in [float("nan"), float("inf"), -1.0]:
            run = capture()
            run["records"][2]["stagesMs"]["tick"] = value
            with self.assertRaises(ValueError):
                self.validate(run)


if __name__ == "__main__":
    unittest.main()
