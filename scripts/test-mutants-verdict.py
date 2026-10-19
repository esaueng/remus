#!/usr/bin/env python3
"""Prove the weekly mutation verdict and shard planner can fail (B19).

Synthetic `mutants.out` directories in cargo-mutants 27's shapes drive
scripts/mutants-verdict.py through every clean, sampled and failing case,
including the 2026-09-13/16 shape that used to pass: a run stopped by
`timeout` during the baseline, with mutants listed but none examined.
"""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
from fleet_workflows import workflow_path
import re
import tempfile
import unittest
from unittest import mock

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
            with contextlib.redirect_stdout(io.StringIO()) as stdout:
                code = VERDICT.main(["--output", str(out), "--exit-code", str(exit_code),
                                     "--label", "test shard", "--summary", str(summary)])
            self.stdout = stdout.getvalue()
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

    def test_cost_mean_includes_missed_and_unviable_slot_times(self):
        listed = [mutant(A), mutant(B), mutant(C)]
        outcomes = [BASELINE, outcome(A, "CaughtMutant", build=10.0, test=10.0),
                    outcome(B, "MissedMutant", build=40.0, test=100.0),
                    outcome(C, "Unviable", build=5.0)]
        code, text, _ = self.run_verdict(listed, outcomes, 2)
        self.assertEqual(code, 0, text)
        # (20 + 140 + 5) / 3 = 55 occupied slot-seconds. Adding phase medians
        # would give 65 and dropping missed mutants would give 12.5.
        self.assertIn("| remus-algo | 3 | 10 | 55 | 55 |", text)
        # Unviable mutants are not judged: 1 caught of 2 viable.
        self.assertIn("Mutation score: 50.0% of 2 examined viable mutants caught", text)

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

    def test_partial_coverage_is_a_sample_and_names_the_rest(self):
        code, text, unexamined = self.run_verdict(
            [mutant(A), mutant(B)], [BASELINE, outcome(A, "CaughtMutant")], 124)
        self.assertEqual(code, 0, text)
        self.assertIn("| not sampled | 1 |", text)
        self.assertIn("Sample: 1 of 2 listed mutants examined (50%)", text)
        self.assertEqual(unexamined.splitlines(), [B])

    def test_unreached_mutants_after_a_finished_run_fail(self):
        # cargo-mutants exiting by itself (0 clean, 2 missed, 3 timeout) has
        # examined its whole list; a missing outcome is bad output, not a sample.
        for exit_code in (0, 2, 3):
            with self.subTest(exit_code=exit_code):
                code, text, unexamined = self.run_verdict(
                    [mutant(A), mutant(B)], [BASELINE, outcome(A, "CaughtMutant")], exit_code)
                self.assertEqual(code, 1, text)
                self.assertIn("have no outcome although cargo-mutants finished", self.stdout)
                self.assertEqual(unexamined.splitlines(), [B])

    def test_listed_mutants_with_none_examined_fail(self):
        # A green baseline followed by the budget expiring before any mutant
        # finished is no sample at all.
        code, text, unexamined = self.run_verdict([mutant(A), mutant(B)], [BASELINE], 124)
        self.assertEqual(code, 1, text)
        self.assertIn("none was examined", self.stdout)
        self.assertEqual(unexamined.splitlines(), [A, B])

    def test_missed_mutants_are_reported_as_warnings_without_failing(self):
        for exit_code in (2, 0, 124):
            with self.subTest(exit_code=exit_code):
                code, text, _ = self.run_verdict([mutant(A)], [BASELINE, outcome(A, "MissedMutant")], exit_code)
                self.assertEqual(code, 0, text)
                self.assertIn(f"`{A}`", text)
                self.assertIn("::warning::test shard: 1 mutants missed", self.stdout)
                self.assertNotIn("::error::", self.stdout)

    def test_timeout_mutants_fail(self):
        code, text, _ = self.run_verdict([mutant(A)], [BASELINE, outcome(A, "Timeout")], 3)
        self.assertEqual(code, 1, text)
        self.assertIn(f"`{A}`", text)
        self.assertIn("::error::test shard: 1 mutants timed out", self.stdout)

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

    def test_repeated_names_are_counted_as_a_multiset(self):
        listed = [mutant(A), mutant(A)]
        code, text, unexamined = self.run_verdict(listed, [BASELINE, outcome(A, "CaughtMutant")], 124)
        self.assertEqual(code, 0, text)
        self.assertEqual(unexamined.splitlines(), [A])
        both = [BASELINE, outcome(A, "CaughtMutant"), outcome(A, "Unviable")]
        self.assertEqual(self.run_verdict(listed, both, 0)[0], 0)

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
        week = ([mutant(f"a{i}") for i in range(500)]
                + [mutant(f"o{i}", "remus-operations") for i in range(150)]
                + [mutant(f"m{i}", "remus-math") for i in range(100)]
                + [mutant(f"b{i}", "remus-blend") for i in range(30)])
        counts, estimate, capacity, needed, shards = PLAN.plan(week, 300)
        self.assertEqual(sum(counts.values()), 780)
        self.assertLessEqual(shards, PLAN.MAX_SHARDS)
        self.assertGreaterEqual(shards * capacity, estimate)
        self.assertLess((needed - 1) * capacity, estimate)
        self.assertLessEqual(max(PLAN.round_robin_costs(week, shards)), capacity)
        out = self.run_plan(week)
        self.assertEqual(json.loads(out["matrix"]), list(range(int(out["shards"]))))

    def test_one_mutant_still_gets_one_shard_and_unknown_packages_cost_the_most(self):
        self.assertEqual(self.run_plan([mutant("x", "remus-new")])["shards"], "1")
        _, estimate, _, _, _ = PLAN.plan([mutant("x", "remus-new")], 300)
        self.assertEqual(estimate, max(PLAN.SECONDS_PER_MUTANT.values()))

    def test_round_robin_imbalance_requires_more_than_the_aggregate_minimum(self):
        week = [mutant("h0", "heavy"), mutant("l0", "light"),
                mutant("h1", "heavy"), mutant("l1", "light")]
        with mock.patch.dict(PLAN.SECONDS_PER_MUTANT, {"heavy": 40.0, "light": 10.0}, clear=True), \
                mock.patch.object(PLAN, "SHARD_OVERHEAD_SECONDS", 0), \
                mock.patch.object(PLAN, "JOBS_PER_SHARD", 1), \
                mock.patch.object(PLAN, "TARGET_UTILIZATION", 1):
            # 100 / 60 suggests two shards, but i % 2 assigns both expensive
            # mutants to one shard (80/20). Three fit (50/10/40).
            _, _, capacity, needed, shards = PLAN.plan(week, 1, max_shards=4)
            self.assertEqual((needed, shards), (2, 3))
            self.assertEqual(PLAN.round_robin_costs(week, 2), [80.0, 20.0])
            self.assertLessEqual(max(PLAN.round_robin_costs(week, shards)), capacity)

    def test_capped_imbalance_is_reported_as_sampled_even_when_total_cost_fits_the_cap(self):
        week = [mutant("h0", "heavy"), mutant("l0", "light"),
                mutant("h1", "heavy"), mutant("l1", "light")]
        with mock.patch.dict(PLAN.SECONDS_PER_MUTANT, {"heavy": 40.0, "light": 10.0}, clear=True), \
                mock.patch.object(PLAN, "SHARD_OVERHEAD_SECONDS", 0), \
                mock.patch.object(PLAN, "JOBS_PER_SHARD", 1), \
                mock.patch.object(PLAN, "TARGET_UTILIZATION", 1), \
                tempfile.TemporaryDirectory(prefix="remus-plan-imbalance-") as tmp:
            listing = Path(tmp) / "week.json"
            listing.write_text(json.dumps(week))
            summary = Path(tmp) / "summary.md"
            output = Path(tmp) / "out"
            with contextlib.redirect_stdout(io.StringIO()) as stdout:
                PLAN.main([str(listing), "--budget-minutes", "1", "--max-shards", "2",
                           "--summary", str(summary), "--github-output", str(output)])
            self.assertIn("shards=2", output.read_text())
            self.assertIn("::notice::", stdout.getvalue())
            self.assertIn("total-cost lower bound 2, cap 2", summary.read_text())
            # Shard 0 carries 80 s against 60 s capacity, shard 1 20 s: (60 + 20) / 100.
            self.assertIn("about 80% of the listed mutants", summary.read_text())
            self.assertIn("| 0 | 0.0 | no |", summary.read_text())

    def test_single_over_budget_mutant_does_not_create_empty_shards(self):
        with mock.patch.dict(PLAN.SECONDS_PER_MUTANT, {"heavy": 100.0}, clear=True), \
                mock.patch.object(PLAN, "SHARD_OVERHEAD_SECONDS", 0), \
                mock.patch.object(PLAN, "JOBS_PER_SHARD", 1), \
                mock.patch.object(PLAN, "TARGET_UTILIZATION", 1):
            self.assertEqual(PLAN.plan([mutant("h", "heavy")], 1)[-1], 1)

    def test_round_robin_zero_shards_is_rejected(self):
        with self.assertRaises(ValueError):
            PLAN.round_robin_costs([], 0)

    def test_cap_is_one_wave_of_the_callee_parallelism(self):
        text = workflow_path("mutants-sharded").read_text()
        parallel = int(re.search(r"^      max-parallel: (\d+)$", text, re.M)[1])
        self.assertEqual(PLAN.MAX_SHARDS, parallel)

    def test_overflowing_week_is_capped_and_sampled(self):
        huge = [mutant(f"o{i}", "remus-operations") for i in range(20000)]
        _, _, _, needed, shards = PLAN.plan(huge, 300)
        self.assertGreater(needed, PLAN.MAX_SHARDS)
        self.assertEqual(shards, PLAN.MAX_SHARDS)

    def test_former_3375_mutant_fixture_is_now_explicitly_over_capacity(self):
        # Preserve the former planner fixture: the updated measured costs
        # must expose its insufficient budget rather than shrink its scope.
        week = ([mutant(f"a{i}") for i in range(2067)]
                + [mutant(f"o{i}", "remus-operations") for i in range(718)]
                + [mutant(f"m{i}", "remus-math") for i in range(420)]
                + [mutant(f"b{i}", "remus-blend") for i in range(170)])
        counts, estimate, capacity, needed, shards = PLAN.plan(week, 300)
        self.assertEqual(sum(counts.values()), 3375)
        self.assertGreater(estimate, shards * capacity)
        self.assertGreater(needed, PLAN.MAX_SHARDS)
        self.assertEqual(shards, PLAN.MAX_SHARDS)
        self.assertGreater(max(PLAN.round_robin_costs(week, shards)), capacity)

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

    TEXT = workflow_path("mutants-sharded").read_text()

    def test_budget_and_shards_are_shared_between_plan_and_shards(self):
        text = self.TEXT
        self.assertIn('--budget-minutes "$MUTANTS_BUDGET_MINUTES"', text)
        self.assertIn('"${MUTANTS_BUDGET_MINUTES}m"', text)
        self.assertIn('--shard "$SHARD/$SHARDS" --sharding round-robin', text)
        self.assertIn("shard: ${{ fromJSON(needs.plan.outputs.matrix) }}", text)
        self.assertIn("SINCE: ${{ needs.plan.outputs.since }}", text)
        budget = int(re.search(r'MUTANTS_BUDGET_MINUTES: "(\d+)"', text)[1])
        shard_job = text.split("\n  mutants:\n", 1)[1]
        job = int(re.search(r"^    timeout-minutes: (\d+)$", shard_job, re.M)[1])
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
