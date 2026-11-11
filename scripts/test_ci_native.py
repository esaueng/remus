#!/usr/bin/env python3
"""Contracts for the CI Pass gate and the merged native test build."""

import importlib.util
from pathlib import Path
import re
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
CHECKS = (ROOT / ".github/workflows/ci-checks.yml").read_text(encoding="utf-8").replace("\r\n", "\n")


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


GATE = load("ci_gate")
NATIVE = load("ci_native")


def flags(**selected):
    return {name: "true" if selected.get(name) else "false" for name in GATE.FLAGS}


def results(succeeded):
    jobs = set.union(*GATE.GROUPS.values())
    return {job: {"result": "success" if job in succeeded else "skipped"} for job in jobs}


class GateTests(unittest.TestCase):
    ALWAYS = GATE.GROUPS["always"]

    def test_gate_needs_exactly_the_workflow_jobs(self):
        jobs = set(re.findall(r"(?m)^  ([a-z0-9-]+):\n", CHECKS.split("\njobs:\n", 1)[1])) - {"ci-pass"}
        self.assertEqual(jobs, set.union(*GATE.GROUPS.values()))
        needs = re.search(r"(?m)^  ci-pass:\n(?:.*\n)*?    needs: \[(.+)\]$", CHECKS)[1]
        self.assertEqual(set(needs.split(", ")), jobs)
        self.assertRegex(CHECKS, r"(?m)^  ci-pass:\n    name: CI Pass\n    if: always\(\)\n")

    def test_light_selection_passes_with_only_the_light_gates(self):
        GATE.validate(results(self.ALWAYS), flags(), "merge_group")
        GATE.validate(results(self.ALWAYS | {"docs"}), flags(docs=True), "merge_group")

    def test_every_selected_job_must_succeed(self):
        selected = flags(heavy=True, wasm=True, docs=True)
        passing = self.ALWAYS | GATE.GROUPS["heavy"] | GATE.GROUPS["wasm"] | GATE.GROUPS["docs"]
        GATE.validate(results(passing), selected, "push")
        GATE.validate(results(passing | {"wasm-size-gate"}), selected, "pull_request")
        with self.assertRaises(ValueError):  # the size gate is part of a wasm PR
            GATE.validate(results(passing), selected, "pull_request")
        for job in sorted(passing):
            for outcome in ("skipped", "failure", "cancelled"):
                with self.subTest(job=job, outcome=outcome), self.assertRaises(ValueError):
                    broken = results(passing)
                    broken[job] = {"result": outcome}
                    GATE.validate(broken, selected, "push")

    def test_an_unselected_job_may_skip_but_never_fail(self):
        broken = results(self.ALWAYS)
        broken["msrv"] = {"result": "failure"}
        with self.assertRaises(ValueError):
            GATE.validate(broken, flags(), "pull_request")

    def test_malformed_input_fails_closed(self):
        with self.assertRaises(ValueError):
            GATE.validate(results(self.ALWAYS), flags(), "")
        with self.assertRaises(ValueError):
            GATE.validate(results(self.ALWAYS), {**flags(), "heavy": "yes"}, "push")
        with self.assertRaises(ValueError):
            GATE.validate(results(self.ALWAYS), flags(wasm=True), "push")
        missing = results(self.ALWAYS)
        del missing["clippy"]
        with self.assertRaises(ValueError):
            GATE.validate(missing, flags(), "push")


class NativeBuildTests(unittest.TestCase):
    def package(self, root, files, **target):
        tests = root / "tests"
        tests.mkdir(parents=True)
        targets = []
        for name, source in files.items():
            path = tests / f"{name}.rs"
            path.write_text(source, encoding="utf-8")
            targets.append({"kind": ["test"], "name": name, "src_path": str(path), "test": True, **target})
        return {"manifest_path": str(root / "Cargo.toml"), "targets": targets}

    def test_plain_tests_merge_and_identity_dependent_ones_stay_standalone(self):
        with tempfile.TemporaryDirectory() as tmp:
            package = self.package(Path(tmp), {
                "boolean_invariants": "#![allow(dead_code)]\n#[test]\nfn ok() {}\n",
                "prose": "// `crate::util` is only mentioned here\n#[test]\nfn ok() {}\n",
                "cli": 'const BIN: &str = env!("CARGO_BIN_EXE_tool");\n',
                "rooted": "fn helper() {}\n#[test]\nfn ok() { crate::helper() }\n",
                "limit": "#![recursion_limit = \"256\"]\n",
            })
            merged, standalone = NATIVE.split_targets(package)
            self.assertEqual([path.stem for path in merged], ["boolean_invariants", "prose"])
            self.assertEqual(standalone, ["cli", "limit", "rooted"])

    def test_targets_cargo_does_not_discover_from_a_top_level_file_stay_standalone(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            package = self.package(root, {"gated": "#[test]\nfn ok() {}\n"}, **{"required-features": ["x"]})
            nested = root / "tests" / "suite" / "main.rs"
            nested.parent.mkdir()
            nested.write_text("#[test]\nfn ok() {}\n", encoding="utf-8")
            package["targets"].append({"kind": ["test"], "name": "suite", "src_path": str(nested), "test": True})
            package["targets"].append({"kind": ["lib"], "name": "lib", "src_path": str(root / "src/lib.rs")})
            self.assertEqual(NATIVE.split_targets(package), ([], ["gated", "suite"]))

    def test_harnesses_declare_every_merged_file_once(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            big = "#[test]\nfn ok() {}\n" + "// filler\n" * NATIVE.CHUNK_LINES
            package = self.package(root, {"a": big, "b": big, "match": "#[test]\nfn ok() {}\n",
                                          "cli": "fn exe() { std::env::current_exe().unwrap(); }\n"})
            harnesses, standalone = NATIVE.write_harnesses([package])
            self.assertEqual(harnesses, [root / "tests" / f"{NATIVE.HARNESS_PREFIX}{n}.rs" for n in (1, 2, 3)])
            self.assertEqual(standalone, ["cli"])
            written = "".join(path.read_text(encoding="utf-8") for path in harnesses)
            self.assertEqual(sorted(re.findall(r'#\[path = "(\w+)\.rs"\]\nmod (\w+);', written)),
                             [("a", "a"), ("b", "b"), ("match", "t_match")])
            args = NATIVE.build_args(harnesses, standalone)
            self.assertEqual(args[:7], ["--cargo-profile", "ci-test", "--locked", "--workspace",
                                        "--lib", "--bins", "--examples"])
            self.assertEqual(args[7:], ["--test", "ci_harness_1", "--test", "ci_harness_2",
                                        "--test", "ci_harness_3", "--test", "cli"])

    def test_a_harness_cargo_does_not_discover_fails_the_build(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            package = self.package(root, {"a": "#[test]\nfn ok() {}\n"})
            harnesses, _ = NATIVE.write_harnesses([package])
            # `autotests = false`: Cargo still lists only the declared target.
            with self.assertRaisesRegex(ValueError, "ci_harness_1"):
                NATIVE.verify_discovered(harnesses, [package])
            package["targets"].append({"kind": ["test"], "name": "ci_harness_1",
                                       "src_path": str(harnesses[0]), "test": True})
            NATIVE.verify_discovered(harnesses, [package])

    def test_stale_harnesses_are_removed_before_cargo_lists_targets(self):
        with tempfile.TemporaryDirectory() as tmp:
            stale = Path(tmp) / "crates" / "demo" / "tests" / f"{NATIVE.HARNESS_PREFIX}9.rs"
            stale.parent.mkdir(parents=True)
            stale.write_text("stale", encoding="utf-8")
            kept = stale.with_name("real.rs")
            kept.write_text("", encoding="utf-8")
            NATIVE.remove_harnesses(Path(tmp))
            self.assertFalse(stale.exists())
            self.assertTrue(kept.exists())

    def test_generated_harnesses_are_never_committed(self):
        self.assertIn("**/tests/ci_harness_*.rs", (ROOT / ".gitignore").read_text(encoding="utf-8"))

    def test_workflow_builds_once_and_runs_every_partition(self):
        shards = re.search(r"shard: \[([0-9, ]+)\]", CHECKS)[1].split(", ")
        total = len(shards)
        self.assertEqual(shards, [str(n) for n in range(1, total + 1)])
        self.assertIn(f'ci_native.py run --archive "$RUNNER_TEMP/native-tests.tar.zst" --shard "$SHARD" --total {total}', CHECKS)
        self.assertIn(f"name: Native tests (${{{{ matrix.shard }}}}/{total})", CHECKS)
        self.assertEqual(CHECKS.count("ci_native.py archive"), 1)
        self.assertIn("cargo test --workspace --profile ci-test --doc", CHECKS)


if __name__ == "__main__":
    unittest.main()
