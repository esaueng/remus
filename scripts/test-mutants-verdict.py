#!/usr/bin/env python3
"""Prove the weekly mutation verdict and shard planner can fail (B19).

Synthetic `mutants.out` directories in cargo-mutants 27's shapes drive
scripts/mutants-verdict.py through every clean and failing case, including the
2026-09-13/16 shape that used to pass: a run stopped by `timeout` during the
baseline, with mutants listed but none examined.
"""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VERDICT = load("mutants_verdict", "mutants-verdict.py")
PLAN = load("plan_mutation_shards", "plan-mutation-shards.py")


def mutant(name, package="remus-algo"):
    return {"name": name, "package": package, "file": name.split(":", 1)[0]}


def outcome(name, summary, package="remus-algo", build=30.0, test=3.0):
    phases = [{"phase": "Build", "duration": build, "process_status": "Success", "argv": []}]
    if summary != "Unviable":
        phases.append({"phase": "Test", "duration": test, "process_status": "Success", "argv": []})
    return {"scenario": {"Mutant": {"name": name, "package": package}}, "summary": summary,
            "phase_results": phases}


BASELINE = {"scenario": "Baseline", "summary": "Success", "phase_results": [
    {"phase": "Build", "duration": 300.0, "process_status": "Success", "argv": []},
    {"phase": "Test", "duration": 150.0, "process_status": "Success", "argv": []},
]}
A = "crates/algo/src/a.rs:1:1: replace f -> bool with true"
B = "crates/algo/src/b.rs:2:2: replace + with - in g"
C = "crates/operations/src/c.rs:3:3: replace h with ()"


class VerdictTests(unittest.TestCase):
    def run_verdict(self, listed, outcomes, exit_code, write_outcomes=True, make_dir=True):
        with tempfile.TemporaryDirectory(prefix="remus-verdict-") as tmp:
            out = Path(tmp) / "mutants.out"
            if make_dir:
                out.mkdir()
                if listed is not None:
                    (out / "mutants.json").write_text(json.dumps(listed))
                if write_outcomes:
                    (out / "outcomes.json").write_text(json.dumps({"outcomes": outcomes}))
            summary = Path(tmp) / "summary.md"
            with contextlib.redirect_stdout(io.StringIO()):
                code = VERDICT.main(["--output", str(out), "--exit-code", str(exit_code),
                                     "--label", "test shard", "--summary", str(summary)])
            unexamined = (out / "unexamined.txt").read_text() if (out / "unexamined.txt").exists() else ""
            return code, summary.read_text(), unexamined

    def test_complete_clean_shard_passes(self):
        listed = [mutant(A), mutant(B), mutant(C, "remus-operations")]
        outcomes = [BASELINE, outcome(A, "CaughtMutant"), outcome(B, "Unviable"),
                    outcome(C, "CaughtMutant", "remus-operations", 200.0, 40.0)]
        code, text, unexamined = self.run_verdict(listed, outcomes, 0)
        self.assertEqual(code, 0, text)
        self.assertIn("| examined | 3 |", text)
        self.assertIn("| remus-operations | 1 | 200 | 40 |", text)
        self.assertEqual(unexamined, "")

    def test_no_mutants_in_scope_passes(self):
        self.assertEqual(self.run_verdict(None, [], 0, make_dir=False)[0], 0)
        self.assertEqual(self.run_verdict([], [], 0, write_outcomes=False)[0], 0)

    def test_budget_expiring_after_the_last_mutant_still_passes(self):
        code, text, _ = self.run_verdict([mutant(A)], [BASELINE, outcome(A, "CaughtMutant")], 124)
        self.assertEqual(code, 0, text)
        self.assertIn("coverage is complete", text)

    def test_baseline_cut_off_by_the_budget_fails(self):
        # The 2026-09-13/16 shape: thousands listed, the baseline still
        # running when `timeout` fired, zero outcomes counted, job green.
        listed = [mutant(A), mutant(B)]
        code, text, unexamined = self.run_verdict(listed, [], 124)
        self.assertEqual(code, 1, text)
        self.assertEqual(unexamined.splitlines(), [A, B])

    def test_partial_coverage_fails_and_names_the_rest(self):
        code, text, unexamined = self.run_verdict(
            [mutant(A), mutant(B)], [BASELINE, outcome(A, "CaughtMutant")], 124)
        self.assertEqual(code, 1, text)
        self.assertIn("| not examined | 1 |", text)
        self.assertEqual(unexamined.splitlines(), [B])

    def test_missed_and_timeout_mutants_fail(self):
        for summary, exit_code in (("MissedMutant", 2), ("Timeout", 3)):
            with self.subTest(summary=summary):
                code, text, _ = self.run_verdict([mutant(A)], [BASELINE, outcome(A, summary)], exit_code)
                self.assertEqual(code, 1, text)
                self.assertIn(f"`{A}`", text)

    def test_missed_mutant_fails_even_if_exit_code_claims_clean(self):
        code, _, _ = self.run_verdict([mutant(A)], [BASELINE, outcome(A, "MissedMutant")], 0)
        self.assertEqual(code, 1)

    def test_unattributed_mutant_state_fails(self):
        for summary in ("Failure", "Success"):
            with self.subTest(summary=summary):
                self.assertEqual(self.run_verdict([mutant(A)], [BASELINE, outcome(A, summary)], 0)[0], 1)

    def test_mutants_without_a_green_baseline_fail(self):
        # `--baseline=skip` would examine mutants with nothing proving the
        # unmutated tree green: every "caught" could be a pre-existing failure.
        code, _, _ = self.run_verdict([mutant(A)], [outcome(A, "CaughtMutant")], 0)
        self.assertEqual(code, 1)

    def test_failed_baseline_fails(self):
        failed = dict(BASELINE, summary="Failure")
        self.assertEqual(self.run_verdict([mutant(A)], [failed], 4)[0], 1)

    def test_unreadable_or_missing_output_fails(self):
        self.assertEqual(self.run_verdict(None, [], 70, make_dir=False)[0], 1)
        self.assertEqual(self.run_verdict([mutant(A)], [], 0, write_outcomes=False)[0], 1)
        self.assertEqual(self.run_verdict(None, [BASELINE], 0)[0], 1)

    def test_unexpected_exit_codes_fail_even_with_clean_outcomes(self):
        for exit_code in (1, 5, 6, 70, 137, 255):
            with self.subTest(exit_code=exit_code):
                result = self.run_verdict([mutant(A)], [BASELINE, outcome(A, "CaughtMutant")], exit_code)
                self.assertEqual(result[0], 1)

    def test_outcomes_outside_the_listing_fail(self):
        self.assertEqual(self.run_verdict(
            [mutant(A)], [BASELINE, outcome(A, "CaughtMutant"), outcome(B, "CaughtMutant")], 0)[0], 1)
        self.assertEqual(self.run_verdict(
            [mutant(A)], [BASELINE, outcome(A, "CaughtMutant"), outcome(A, "CaughtMutant")], 0)[0], 1)


class PlanTests(unittest.TestCase):
    def run_plan(self, mutants, budget=300, raw=None):
        with tempfile.TemporaryDirectory(prefix="remus-plan-") as tmp:
            listing = Path(tmp) / "week.json"
            listing.write_text(raw if raw is not None else json.dumps(mutants))
            output = Path(tmp) / "out"
            with contextlib.redirect_stdout(io.StringIO()):
                PLAN.main([str(listing), "--budget-minutes", str(budget), "--github-output", str(output),
                           "--summary", str(Path(tmp) / "summary.md")])
            return dict(line.split("=", 1) for line in output.read_text().splitlines())

    def test_empty_listing_plans_no_shards(self):
        for raw in ("", "[]\n"):
            with self.subTest(raw=raw):
                self.assertEqual(self.run_plan(None, raw=raw),
                                 {"mutants": "0", "shards": "0", "matrix": "[]"})

    def test_shards_cover_the_estimated_cost(self):
        week = ([mutant(f"a{i}") for i in range(2067)]
                + [mutant(f"o{i}", "remus-operations") for i in range(718)]
                + [mutant(f"m{i}", "remus-math") for i in range(420)]
                + [mutant(f"b{i}", "remus-blend") for i in range(170)])
        counts, estimate, capacity, needed, shards = PLAN.plan(week, 300)
        self.assertEqual(sum(counts.values()), 3375)
        self.assertGreaterEqual(shards * capacity, estimate)
        self.assertLess((shards - 1) * capacity, estimate)
        out = self.run_plan(week)
        self.assertEqual(json.loads(out["matrix"]), list(range(int(out["shards"]))))

    def test_one_mutant_still_gets_one_shard_and_unknown_packages_cost_the_most(self):
        self.assertEqual(self.run_plan([mutant("x", "remus-new")])["shards"], "1")
        _, estimate, _, _, _ = PLAN.plan([mutant("x", "remus-new")], 300)
        self.assertEqual(estimate, max(PLAN.SECONDS_PER_MUTANT.values()))

    def test_overflowing_week_is_capped_and_warned(self):
        huge = [mutant(f"o{i}", "remus-operations") for i in range(20000)]
        _, _, _, needed, shards = PLAN.plan(huge, 300)
        self.assertGreater(needed, PLAN.MAX_SHARDS)
        self.assertEqual(shards, PLAN.MAX_SHARDS)

    def test_probe_cap_bounds_the_matrix(self):
        week = [mutant(f"o{i}", "remus-operations") for i in range(2000)]
        with tempfile.TemporaryDirectory(prefix="remus-plan-") as tmp:
            listing = Path(tmp) / "week.json"
            listing.write_text(json.dumps(week))
            output = Path(tmp) / "out"
            with contextlib.redirect_stdout(io.StringIO()):
                PLAN.main([str(listing), "--budget-minutes", "300", "--max-shards", "3",
                           "--github-output", str(output), "--summary", str(Path(tmp) / "s.md")])
            self.assertIn("matrix=[0, 1, 2]", output.read_text())
        with self.assertRaises(ValueError):
            PLAN.plan(week, 300, max_shards=0)

    def test_budget_below_overhead_is_rejected(self):
        with self.assertRaises(ValueError):
            PLAN.plan([mutant("a")], PLAN.SHARD_OVERHEAD_SECONDS // 60)

    def test_non_list_listing_is_rejected(self):
        with self.assertRaises(ValueError):
            self.run_plan(None, raw='{"name": "x"}')


class WorkflowContractTests(unittest.TestCase):
    """The staged sharded workflow must feed these scripts what they expect."""

    TEXT = (ROOT / ".github/workflows/fleet-mutants-sharded.yml").read_text()

    def test_budget_and_shards_are_shared_between_plan_and_shards(self):
        text = self.TEXT
        self.assertIn('--budget-minutes "$MUTANTS_BUDGET_MINUTES"', text)
        self.assertIn('"${MUTANTS_BUDGET_MINUTES}m"', text)
        self.assertIn('--shard "$SHARD/$SHARDS" --sharding round-robin', text)
        self.assertIn("shard: ${{ fromJSON(needs.plan.outputs.matrix) }}", text)
        self.assertIn("SINCE: ${{ needs.plan.outputs.since }}", text)
        budget = int(text.split('MUTANTS_BUDGET_MINUTES: "', 1)[1].split('"', 1)[0])
        job = int(text.split("timeout-minutes: ", 3)[2].split("\n", 1)[0])
        self.assertGreater(job, budget + 2)
        self.assertLessEqual(job, 360)

    def test_verdict_always_runs_and_judges_the_real_exit_code(self):
        text = self.TEXT
        judge = text.split("- name: Judge this shard", 1)[1].split("- name:", 1)[0]
        self.assertIn("if: always()", judge)
        self.assertIn("scripts/mutants-verdict.py", judge)
        self.assertIn('echo "$?" > mutants-exit-code', text)
        self.assertNotIn("|| true", text)


if __name__ == "__main__":
    unittest.main(verbosity=1)
