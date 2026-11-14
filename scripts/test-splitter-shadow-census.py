#!/usr/bin/env python3
"""Check the shadow census's reading of the log the kernel writes.

The log format is a contract between
`crates/algo/src/builder/face_splitter/shadow.rs::log_event` and
`scripts/splitter-shadow-census.py`. These tests pin the aggregation on a
hand-written log; they run no cargo command.
"""
import importlib.util
import pathlib
import sys
import unittest

SCRIPT = pathlib.Path(__file__).with_name("splitter-shadow-census.py")
SHADOW = SCRIPT.parent.parent / "crates/algo/src/builder/face_splitter/shadow.rs"
spec = importlib.util.spec_from_file_location("splitter_shadow_census", SCRIPT)
census = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = census  # dataclasses resolve annotations through it
spec.loader.exec_module(census)


def line(budgets, entry, outcome, reason, test="foil_test", face="7", regions=("2", "2")):
    return "\t".join(
        [budgets, entry, outcome, reason, test, face, "plane", "2", regions[0], regions[1], "(1.000000,2.000000,3.000000)"]
    )


CROSSING, DISK, SECTORS, BANDS = census.ENTRIES
LOG = "\n".join(
    [
        line("default", CROSSING, "declined-on-budget", "work-budget", regions=("4", "-")),
        line("raised", CROSSING, "agreed", "-", regions=("4", "4")),
        line("raised", CROSSING, "agreed", "-", face="9", regions=("3", "3")),
        line("raised", DISK, "disagreed", "segmentation-only(vertex)", regions=("3", "3")),
        line("raised", DISK, "disagreed", "region-count", face="11", regions=("3", "2")),
        line("raised", BANDS, "disagreed", "span-direction", regions=("3", "3")),
        line("raised", BANDS, "disagreed", "span-direction", regions=("3", "3")),
        line("raised", SECTORS, "refused", "FaceSplitFailed(\"cylinder arrangement refused: OpenRegion\")"),
        line("raised", SECTORS, "declined", "out-of-domain", regions=("2", "-")),
    ]
)


class ShadowCensusTests(unittest.TestCase):
    def setUp(self):
        self.events = census.parse_log(LOG, "nub/some_fixture")

    def test_every_entry_point_the_kernel_names_has_a_row(self):
        source = SHADOW.parent.parent.parent / "gfa.rs"
        text = source.read_text(encoding="utf-8")
        for entry in census.ENTRIES:
            self.assertIn(f'"{entry}"', text, f"{entry} is not a name gfa.rs gives a shadowed entry point")

    def test_the_log_has_the_columns_the_kernel_documents(self):
        self.assertEqual(len(census.COLUMNS), 11)
        self.assertEqual(self.events[0]["foil"], "nub/some_fixture")
        self.assertEqual(self.events[0]["shadow_regions"], "-")
        with self.assertRaises(ValueError):
            census.parse_log("default\tonly\tthree", "x")

    def test_tally_separates_budget_modes_and_outcomes(self):
        default = census.tally(self.events, "default")
        self.assertEqual(default[CROSSING], {"fired": 1, "declined-on-budget": 1})
        self.assertEqual(sum(default[DISK].values()), 0)
        raised = census.tally(self.events, "raised")
        self.assertEqual(raised[CROSSING], {"fired": 2, "agreed": 2})
        self.assertEqual(raised[DISK], {"fired": 2, "disagreed": 2, "segmentation": 1})
        self.assertEqual(raised[BANDS], {"fired": 2, "disagreed": 2})
        self.assertEqual(raised[SECTORS], {"fired": 2, "refused": 1, "declined": 1})

    def test_table_rows_read_fired_agreed_disagreed_declined_budget_refused(self):
        table = census.render_table(self.events, "raised").splitlines()
        self.assertIn(f"| `{CROSSING}` | 2 | 2 | 0 (0) | 0 | 0 | 0 |", table)
        self.assertIn(f"| `{DISK}` | 2 | 0 | 2 (1) | 0 | 0 | 0 |", table)
        self.assertIn(f"| `{SECTORS}` | 2 | 0 | 0 (0) | 1 | 0 | 1 |", table)
        default = census.render_table(self.events, "default").splitlines()
        self.assertIn(f"| `{CROSSING}` | 1 | 0 | 0 (0) | 0 | 1 | 0 |", default)

    def test_findings_name_foil_face_and_reason_and_fold_repeats(self):
        found = census.render_findings(self.events, "disagreed").splitlines()
        self.assertEqual(len(found), 3)
        self.assertTrue(any("region-count" in row and "face 11" in row and "arrangement 2" in row for row in found))
        self.assertTrue(any("span-direction" in row and row.endswith("x2") for row in found))
        self.assertTrue(all("nub/some_fixture :: foil_test" in row for row in found))
        self.assertIn("OpenRegion", census.render_findings(self.events, "refused"))
        self.assertEqual(census.render_findings([], "disagreed"), "none")

    def test_every_foil_family_of_the_roadmap_skill_is_run(self):
        families = {foil.family for foil in census.FOILS}
        self.assertEqual(
            families,
            {"d4", "honeycomb-pcut3", "divider-lip", "nub", "cylinder-slot", "groove-mouth", "junction-disc"},
        )
        for foil in census.FOILS:
            if foil.package == "remus-io":
                path = SCRIPT.parent.parent / "crates/io/tests" / f"{foil.label}.rs"
                self.assertTrue(path.is_file(), f"{path} is not a fixture")


if __name__ == "__main__":
    unittest.main()
