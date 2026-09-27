#!/usr/bin/env python3
"""Negative fixtures for the O4.7 twin-coverage gate.

Proves `scripts/check-wasm-o47-coverage.py` fails loudly on every defect
class it exists to catch -- a removed export, a missing batch arm, a
missing witness, a newly added unclassified mutation, a misspelled
`js_name`, an obsolete exception -- and that unknown source syntax never
silently disappears from discovery. A final test runs the gate against
the real tree and requires it to pass there.

Usage: python3 scripts/test-wasm-o47-coverage.py
"""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

GATE_PATH = Path(__file__).with_name("check-wasm-o47-coverage.py")
_SPEC = importlib.util.spec_from_file_location("o47_gate", GATE_PATH)
gate = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gate)  # type: ignore[union-attr]

REPO = Path(__file__).resolve().parents[1]

MUT_EXPORT = """#[wasm_bindgen]
impl BrepKernel {{
    #[wasm_bindgen(js_name = "{js}")]
    pub fn {rust}(&mut self, solid: u32) -> Result<u32, JsError> {{
        Ok(solid)
    }}
}}
"""

QUERY_EXPORT = """#[wasm_bindgen]
impl BrepKernel {{
    #[wasm_bindgen(js_name = "{js}")]
    pub fn {rust}(&self, solid: u32) -> Result<u32, JsError> {{
        Ok(solid)
    }}
}}
"""

ASYNC_MUT_EXPORT = MUT_EXPORT.replace("pub fn", "pub async fn")

BATCH_RS = """use wasm_bindgen::prelude::*;
impl BrepKernel {{
    fn dispatch_op(&mut self, op: &str) -> u32 {{
        match op {{
            "{ops}" => 0,
            _ => 1,
        }}
    }}
}}
"""

TWIN_EXPORT = """#[wasm_bindgen]
impl BrepKernel {{
    #[wasm_bindgen(js_name = "{js}")]
    pub fn {rust}(&mut self, a: u32, b: u32) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {{
        todo!()
    }}
}}
"""

WITNESS_RS = """#[cfg(test)]
mod tests {{
    #[test]
    fn {name}() {{
    }}
}}
"""


def make_tree(files: dict[str, str]) -> Path:
    """Create a temp dir holding a synthetic crates/wasm/src tree."""
    directory = Path(tempfile.mkdtemp(prefix="o47-coverage-"))
    src = directory / "crates" / "wasm" / "src"
    for rel, text in files.items():
        target = src / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8")
    return directory


def write_baseline(directory: Path, rows: list[dict]) -> Path:
    path = directory / "baseline.json"
    path.write_text(
        json.dumps({"meta": {}, "rows": rows}, indent=2), encoding="utf-8"
    )
    return path


def uncovered_row(js: str, rust: str, file: str = "bindings/operations.rs") -> dict:
    return {
        "js": js,
        "rust": rust,
        "file": file,
        "gate": "shipped",
        "class": "geometry_mutation",
        "coverage": "uncovered",
        "owner": "O4.7",
        "reason": "O4.7 solid twin pending",
    }


def covered_row(
    js: str,
    rust: str,
    twin: str,
    ops: list[str],
    witnesses: list[str],
    twin_file: str = "bindings/booleans.rs",
) -> dict:
    return {
        "js": js,
        "rust": rust,
        "file": "bindings/booleans.rs",
        "gate": "shipped",
        "class": "geometry_mutation",
        "coverage": "covered",
        "owner": "O4.7",
        "reason": f"typed by {twin}",
        "twin": twin,
        "twin_file": twin_file,
        "batch_ops": ops,
        "witnesses": witnesses,
        "schema": "solid_envelope",
    }


def run_gate(directory: Path, baseline_rows: list[dict]):
    root = directory / "crates" / "wasm" / "src"
    exports = gate.discover_from_files(root)
    unknown = gate.find_unknown_syntax(root)
    baseline = gate.load_baseline(write_baseline(directory, baseline_rows))
    ops = gate.batch_ops(root)
    violations, stale, counts = gate.verify(exports, baseline, ops, root)
    return violations, stale, counts, unknown


class O47CoverageFixtures(unittest.TestCase):
    def test_clean_tree_passes(self):
        directory = make_tree(
            {
                "bindings/operations.rs": MUT_EXPORT.format(
                    js="doThing", rust="do_thing"
                )
                + QUERY_EXPORT.format(js="getThing", rust="get_thing"),
                "bindings/batch.rs": BATCH_RS.format(ops="doThing"),
            }
        )
        violations, stale, counts, unknown = run_gate(
            directory, [uncovered_row("doThing", "do_thing")]
        )
        self.assertEqual(unknown, [])
        self.assertEqual(violations, [])
        self.assertEqual(stale, [])
        self.assertEqual(counts["uncovered"], 1)

    def test_removed_export_is_stale(self):
        directory = make_tree(
            {"bindings/batch.rs": BATCH_RS.format(ops="doThing")}
        )
        violations, stale, _, _ = run_gate(
            directory, [uncovered_row("doThing", "do_thing")]
        )
        self.assertEqual(violations, [])
        self.assertEqual(len(stale), 1)
        self.assertIn("doThing", stale[0])

    def test_obsolete_exception_is_stale(self):
        directory = make_tree(
            {
                "bindings/operations.rs": MUT_EXPORT.format(
                    js="keptOp", rust="kept_op"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="keptOp"),
            }
        )
        _, stale, _, _ = run_gate(
            directory,
            [
                uncovered_row("keptOp", "kept_op"),
                {
                    "js": "goneOp",
                    "rust": "gone_op",
                    "file": "bindings/operations.rs",
                    "gate": "shipped",
                    "class": "special_case",
                    "coverage": "special",
                    "owner": "O4.7",
                    "reason": "removed upstream",
                },
            ],
        )
        self.assertEqual(len(stale), 1)
        self.assertIn("goneOp", stale[0])

    def test_new_unclassified_mutation_fails(self):
        directory = make_tree(
            {
                "bindings/operations.rs": MUT_EXPORT.format(
                    js="doThing", rust="do_thing"
                )
                + MUT_EXPORT.format(js="surpriseOp", rust="surprise_op"),
                "bindings/batch.rs": BATCH_RS.format(ops="doThing"),
            }
        )
        violations, _, _, _ = run_gate(
            directory, [uncovered_row("doThing", "do_thing")]
        )
        self.assertEqual(len(violations), 1)
        self.assertIn("surpriseOp", violations[0])
        self.assertIn("unclassified", violations[0])

    def test_async_mutation_is_discovered_and_requires_a_row(self):
        directory = make_tree(
            {
                "bindings/operations.rs": ASYNC_MUT_EXPORT.format(
                    js="asyncOp", rust="async_op"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="asyncOp"),
            }
        )
        violations, _, counts, unknown = run_gate(directory, [])
        self.assertEqual(unknown, [])
        self.assertEqual(counts["discovered_mutating"], 1)
        self.assertTrue(any("asyncOp" in item for item in violations), violations)

        violations, stale, counts, unknown = run_gate(
            directory, [uncovered_row("asyncOp", "async_op")]
        )
        self.assertEqual((violations, stale, unknown), ([], [], []))
        self.assertEqual(counts["uncovered"], 1)

    def test_misspelled_js_name_fails(self):
        # The source spells `doThin` where the baseline records `doThing`:
        # one unclassified mutation plus one stale row.
        directory = make_tree(
            {
                "bindings/operations.rs": MUT_EXPORT.format(
                    js="doThin", rust="do_thing"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="doThin"),
            }
        )
        violations, stale, _, _ = run_gate(
            directory, [uncovered_row("doThing", "do_thing")]
        )
        self.assertTrue(any("doThin" in v for v in violations))
        self.assertTrue(any("doThing" in s for s in stale))

    def test_missing_batch_arm_fails(self):
        directory = make_tree(
            {
                "bindings/booleans.rs": MUT_EXPORT.format(js="fuse", rust="fuse")
                + TWIN_EXPORT.format(js="fuseDetailed", rust="fuse_detailed"),
                "bindings/batch.rs": BATCH_RS.format(ops="otherOp"),
                "witness.rs": WITNESS_RS.format(name="fuse_success"),
            }
        )
        violations, _, _, _ = run_gate(
            directory,
            [
                covered_row(
                    "fuse", "fuse", "fuseDetailed", ["fuse"], ["fuse_success"]
                ),
                {
                    "js": "fuseDetailed",
                    "rust": "fuse_detailed",
                    "file": "bindings/booleans.rs",
                    "gate": "shipped",
                    "class": "special_case",
                    "coverage": "special",
                    "owner": "O4.7",
                    "reason": "O4.7 twin; the typed surface itself",
                },
            ],
        )
        self.assertTrue(
            any("batch dispatch" in v and "fuse" in v for v in violations),
            violations,
        )

    def test_missing_witness_fails(self):
        directory = make_tree(
            {
                "bindings/booleans.rs": MUT_EXPORT.format(js="fuse", rust="fuse")
                + TWIN_EXPORT.format(js="fuseDetailed", rust="fuse_detailed"),
                "bindings/batch.rs": BATCH_RS.format(ops="fuse"),
            }
        )
        violations, _, _, _ = run_gate(
            directory,
            [
                covered_row(
                    "fuse",
                    "fuse",
                    "fuseDetailed",
                    ["fuse"],
                    ["ghost_witness"],
                ),
                {
                    "js": "fuseDetailed",
                    "rust": "fuse_detailed",
                    "file": "bindings/booleans.rs",
                    "gate": "shipped",
                    "class": "special_case",
                    "coverage": "special",
                    "owner": "O4.7",
                    "reason": "O4.7 twin; the typed surface itself",
                },
            ],
        )
        self.assertTrue(
            any("ghost_witness" in v for v in violations), violations
        )

    def test_missing_twin_fails_as_false_coverage(self):
        directory = make_tree(
            {
                "bindings/booleans.rs": MUT_EXPORT.format(js="fuse", rust="fuse"),
                "bindings/batch.rs": BATCH_RS.format(ops="fuse"),
                "witness.rs": WITNESS_RS.format(name="fuse_success"),
            }
        )
        violations, _, _, _ = run_gate(
            directory,
            [covered_row("fuse", "fuse", "fuseDetailed", ["fuse"], ["fuse_success"])],
        )
        # No fuseDetailed in source: the twin row is both an unclassified
        # gap (nothing to classify) and, primarily, false coverage on fuse.
        self.assertTrue(
            any("fuseDetailed" in v for v in violations), violations
        )

    def test_optional_io_twin_cannot_cover_shipped_legacy_export(self):
        directory = make_tree(
            {
                "bindings/booleans.rs": MUT_EXPORT.format(js="fuse", rust="fuse"),
                "bindings/io.rs": TWIN_EXPORT.format(
                    js="fuseDetailed", rust="fuse_detailed"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="fuse"),
                "witness.rs": WITNESS_RS.format(name="fuse_success"),
            }
        )
        violations, _, _, _ = run_gate(
            directory,
            [
                covered_row(
                    "fuse", "fuse", "fuseDetailed", ["fuse"],
                    ["fuse_success"], "bindings/io.rs",
                ),
                {
                    "js": "fuseDetailed", "rust": "fuse_detailed",
                    "file": "bindings/io.rs", "gate": "io",
                    "class": "special_case", "coverage": "special",
                    "owner": "O4.7", "reason": "typed twin",
                },
            ],
        )
        self.assertTrue(
            any("only available under 'io'" in item for item in violations),
            violations,
        )

    def test_attribute_gated_twin_cannot_cover_shipped_legacy_export(self):
        twin = TWIN_EXPORT.format(js="fuseDetailed", rust="fuse_detailed")
        sources = {
            "method": twin.replace(
                '    #[wasm_bindgen(js_name',
                '    #[cfg(feature = "io")]\n    #[wasm_bindgen(js_name',
            ),
            "impl": '#[cfg(feature = "io")]\n' + twin,
            "module": '#[cfg(feature = "io")]\nmod optional {\n' + twin + '}\n',
        }
        for placement, gated_twin in sources.items():
            with self.subTest(placement=placement):
                directory = make_tree(
                    {
                        "bindings/booleans.rs": MUT_EXPORT.format(
                            js="fuse", rust="fuse"
                        ) + gated_twin,
                        "bindings/batch.rs": BATCH_RS.format(ops="fuse"),
                        "witness.rs": WITNESS_RS.format(name="fuse_success"),
                    }
                )
                exports = gate.discover_from_files(directory / "crates/wasm/src")
                twin_export = next(e for e in exports if e["js"] == "fuseDetailed")
                self.assertEqual(twin_export["gate"], "io")
                violations, _, _, _ = run_gate(
                    directory,
                    [
                        covered_row("fuse", "fuse", "fuseDetailed", ["fuse"], ["fuse_success"]),
                        {
                            "js": "fuseDetailed", "rust": "fuse_detailed",
                            "file": "bindings/booleans.rs", "gate": "io",
                            "class": "special_case", "coverage": "special",
                            "owner": "O4.7", "reason": "typed twin",
                        },
                    ],
                )
                self.assertTrue(
                    any("only available under 'io'" in item for item in violations),
                    violations,
                )

    def test_optional_io_batch_arm_cannot_cover_shipped_method(self):
        directory = make_tree(
            {
                "bindings/booleans.rs": MUT_EXPORT.format(js="fuse", rust="fuse")
                + TWIN_EXPORT.format(js="fuseDetailed", rust="fuse_detailed"),
                "bindings/batch.rs": BATCH_RS.format(ops="fuse").replace(
                    '            "fuse" =>',
                    '            #[cfg(feature = "io")]\n            "fuse" =>',
                ),
                "witness.rs": WITNESS_RS.format(name="fuse_success"),
            }
        )
        root = directory / "crates/wasm/src"
        self.assertEqual(gate.batch_ops(root)["fuse"], "io")
        violations, _, _, _ = run_gate(
            directory,
            [
                covered_row("fuse", "fuse", "fuseDetailed", ["fuse"], ["fuse_success"]),
                {
                    "js": "fuseDetailed", "rust": "fuse_detailed",
                    "file": "bindings/booleans.rs", "gate": "shipped",
                    "class": "special_case", "coverage": "special",
                    "owner": "O4.7", "reason": "typed twin",
                },
            ],
        )
        self.assertTrue(
            any("arm is only available under 'io'" in item for item in violations),
            violations,
        )

    def test_negated_io_cfg_and_doc_example_stay_shipped(self):
        twin = TWIN_EXPORT.format(js="fuseDetailed", rust="fuse_detailed").replace(
            '    #[wasm_bindgen(js_name',
            '    /// Example: #[cfg(feature = "io")] is optional.\n'
            '    #[cfg(not(feature = "io"))]\n'
            '    #[wasm_bindgen(js_name',
        )
        directory = make_tree(
            {
                "bindings/booleans.rs": MUT_EXPORT.format(js="fuse", rust="fuse") + twin,
                "bindings/batch.rs": BATCH_RS.format(ops="fuse"),
                "witness.rs": WITNESS_RS.format(name="fuse_success"),
            }
        )
        exports = gate.discover_from_files(directory / "crates/wasm/src")
        self.assertEqual(next(e for e in exports if e["js"] == "fuseDetailed")["gate"], "shipped")

    def test_non_solid_forced_into_solid_schema_fails(self):
        for return_type in ("Result<Vec<u32>, JsError>", "Result<String, JsError>", "Result<(), JsError>"):
            with self.subTest(return_type=return_type):
                directory = make_tree(
                    {
                        "bindings/operations.rs": MUT_EXPORT.format(
                            js="split", rust="split_solid"
                        ).replace("Result<u32, JsError>", return_type)
                        + TWIN_EXPORT.format(
                            js="splitDetailed", rust="split_detailed"
                        ),
                        "bindings/batch.rs": BATCH_RS.format(ops="splitDetailed"),
                        "witness.rs": WITNESS_RS.format(name="split_success"),
                    }
                )
                violations, _, _, _ = run_gate(
                    directory,
                    [
                        covered_row(
                            "split", "split_solid", "splitDetailed",
                            ["splitDetailed"], ["split_success"],
                        ),
                        {
                            "js": "splitDetailed", "rust": "split_detailed",
                            "file": "bindings/operations.rs", "gate": "shipped",
                            "class": "special_case", "coverage": "special",
                            "owner": "O4.7", "reason": "O4.7 twin; the typed surface itself",
                        },
                    ],
                )
                self.assertTrue(
                    any("solid envelope" in v for v in violations), violations
                )

    def test_twin_must_return_one_solid_envelope(self):
        directory = make_tree(
            {
                "bindings/operations.rs": MUT_EXPORT.format(js="split", rust="split_solid")
                + TWIN_EXPORT.format(js="splitDetailed", rust="split_detailed").replace(
                    "Result<tsify::Ts<SolidOperationDetailedResult>, JsError>",
                    "Result<Vec<SolidOperationDetailedResult>, JsError>",
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="splitDetailed"),
                "witness.rs": WITNESS_RS.format(name="split_success"),
            }
        )
        violations, _, _, _ = run_gate(
            directory,
            [
                covered_row("split", "split_solid", "splitDetailed", ["splitDetailed"], ["split_success"]),
                {
                    "js": "splitDetailed", "rust": "split_detailed",
                    "file": "bindings/operations.rs", "gate": "shipped",
                    "class": "special_case", "coverage": "special",
                    "owner": "O4.7", "reason": "typed twin",
                },
            ],
        )
        self.assertTrue(any("not the solid envelope" in v for v in violations), violations)

    def test_unknown_syntax_fails_loudly(self):
        directory = make_tree(
            {
                "bindings/operations.rs": (
                    "#[wasm_bindgen]\n"
                    "impl BrepKernel {\n"
                    "    #[wasm_bindgen]\n"
                    "    pub fn bare_export(&mut self, solid: u32) -> Result<u32, JsError> {\n"
                    "        Ok(solid)\n"
                    "    }\n"
                    "}\n"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="bare_export"),
            }
        )
        _, _, _, unknown = run_gate(directory, [])
        self.assertEqual(len(unknown), 1)
        self.assertIn("bare_export", unknown[0])

    def test_async_bare_export_fails_loudly(self):
        directory = make_tree(
            {
                "bindings/operations.rs": (
                    "#[wasm_bindgen]\n"
                    "impl BrepKernel {\n"
                    "    #[wasm_bindgen]\n"
                    "    pub async fn bare_async(&mut self, solid: u32) -> Result<u32, JsError> {\n"
                    "        Ok(solid)\n"
                    "    }\n"
                    "}\n"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="bare_async"),
            }
        )
        _, _, _, unknown = run_gate(directory, [])
        self.assertEqual(len(unknown), 1)
        self.assertIn("bare_async", unknown[0])

    def test_default_named_method_in_exported_impl_fails_loudly(self):
        for qualifier in ("", "async "):
            with self.subTest(qualifier=qualifier):
                directory = make_tree(
                    {
                        "bindings/operations.rs": (
                            "#[wasm_bindgen]\n"
                            "impl BrepKernel {\n"
                            f"    pub {qualifier}fn default_named(&mut self, solid: u32) -> Result<u32, JsError> {{\n"
                            "        Ok(solid)\n"
                            "    }\n"
                            "}\n"
                        ),
                        "bindings/batch.rs": BATCH_RS.format(ops="default_named"),
                    }
                )
                _, _, _, unknown = run_gate(directory, [])
                self.assertEqual(len(unknown), 1)
                self.assertIn("default_named", unknown[0])

    def test_plain_impl_method_is_not_an_export(self):
        directory = make_tree(
            {
                "bindings/operations.rs": (
                    "impl BrepKernel {\n"
                    "    pub fn internal_helper(&mut self) {}\n"
                    "}\n"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="internal_helper"),
            }
        )
        _, _, _, unknown = run_gate(directory, [])
        self.assertEqual(unknown, [])

    def test_indented_impl_with_braces_in_literal_fails_loudly(self):
        directory = make_tree(
            {
                "bindings/operations.rs": (
                    "mod feature {\n"
                    "    #[wasm_bindgen]\n"
                    "    impl BrepKernel {\n"
                    "        #[wasm_bindgen(js_name = \"named\")]\n"
                    "        pub fn named(&self) -> &'static str { \"}\" }\n"
                    "        pub fn default_mutation(&mut self, solid: u32) -> u32 { solid }\n"
                    "    }\n"
                    "}\n"
                ),
                "bindings/batch.rs": BATCH_RS.format(ops="default_mutation"),
            }
        )
        _, _, _, unknown = run_gate(directory, [])
        self.assertEqual(len(unknown), 1)
        self.assertIn("default_mutation", unknown[0])

    def test_batch_arm_must_be_in_dispatch_op_match(self):
        directory = make_tree(
            {
                "bindings/operations.rs": MUT_EXPORT.format(js="doThing", rust="do_thing")
                + TWIN_EXPORT.format(js="doThingDetailed", rust="do_thing_detailed"),
                "bindings/batch.rs": BATCH_RS.format(ops="otherOp").replace(
                    "            _ => 1,",
                    "            _ => {\n"
                    "                match op {\n"
                    "                    \"doThing\" => 0,\n"
                    "                    _ => 1,\n"
                    "                }\n"
                    "            },",
                )
                + '\n// "doThing" => is no longer a dispatch arm\n'
                + 'fn unrelated(value: &str) { match value { "doThing" => {}, _ => {} } }\n',
                "witness.rs": WITNESS_RS.format(name="do_thing_witness"),
            }
        )
        rows = [
            covered_row("doThing", "do_thing", "doThingDetailed", ["doThing"], ["do_thing_witness"], "bindings/operations.rs"),
            {
                "js": "doThingDetailed", "rust": "do_thing_detailed",
                "file": "bindings/operations.rs", "gate": "shipped",
                "class": "special_case", "coverage": "special", "owner": "O4.7",
                "reason": "typed twin",
            },
        ]
        violations, _, _, _ = run_gate(directory, rows)
        self.assertTrue(any("batch dispatch" in item for item in violations), violations)

    def test_repo_tree_passes(self):
        root = REPO / "crates" / "wasm" / "src"
        exports = gate.discover_from_files(root)
        unknown = gate.find_unknown_syntax(root)
        baseline = gate.load_baseline()
        ops = gate.batch_ops(root)
        violations, stale, counts = gate.verify(exports, baseline, ops, root)
        self.assertEqual(unknown, [], f"unknown syntax: {unknown}")
        self.assertEqual(violations, [], "\n".join(violations))
        self.assertEqual(stale, [], "\n".join(stale))
        self.assertGreater(counts["covered"], 0)
        self.assertGreater(counts["uncovered"], 0)


if __name__ == "__main__":
    unittest.main()
