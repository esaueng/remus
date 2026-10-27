"""Reject incomplete, invalid or misattributed preview measurements."""

import copy
import unittest

from compare_preview import validate_capture


def capture():
    return {
        "metadata": {"timingStatus": "measured", "timingsExcluded": False,
                     "kernel": {"wasmSha256": "kernel"}, "io": {"wasmSha256": "io"},
                     "harnessSha256": "driver"},
        "records": [{"tick": i, "accepted": True, "stagesMs": {"tick": 12.0},
                     "quality": {"watertight": True, "boundaryEdges": 0,
                                 "nonManifoldEdges": 0, "faces": 160,
                                 "capAtRequestedOffset": True}} for i in range(6)],
        "summary": {"refused": 0, "warmedSamples": 4, "warmed": {"tick": {"n": 4}},
                    "cold": {"stagesMs": {}}, "memoryBefore": {}, "memoryLast": {}}
    }


class PreviewCaptureTests(unittest.TestCase):
    def validate(self, run):
        return validate_capture(run, 6, 2, "kernel", "io", "driver")

    def test_valid_capture_retains_actual_sample_count(self):
        self.assertEqual(self.validate(capture())["samples"], 4)

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
