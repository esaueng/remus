#!/usr/bin/env python3
"""Fixtures for scripts/check-ops-module-graph.py."""

from __future__ import annotations

import importlib.util
import os
import sys
import tempfile
import textwrap
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("check_ops_module_graph", os.path.join(HERE, "check-ops-module-graph.py"))
graph = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = graph
_spec.loader.exec_module(graph)


class ModuleGraphTests(unittest.TestCase):
    def crate(self, files: dict[str, str]) -> str:
        root = tempfile.mkdtemp()
        self.addCleanup(lambda: __import__("shutil").rmtree(root))
        for rel, text in files.items():
            path = os.path.join(root, rel)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(textwrap.dedent(text))
        return root

    def run_check(self, files: dict[str, str]) -> list[str]:
        return graph.check(self.crate(files))

    LIB = "pub mod tessellate;\npub mod fillet;\npub mod transform;\n"

    def test_downward_reference_passes(self):
        self.assertEqual(self.run_check({
            "lib.rs": self.LIB,
            "tessellate.rs": "pub fn t() {}\n",
            "fillet.rs": "use crate::tessellate::t;\n",
            "transform.rs": "",
        }), [])

    def test_upward_reference_fails_with_location(self):
        problems = self.run_check({
            "lib.rs": self.LIB,
            "tessellate.rs": "pub fn t() { crate::fillet::f(); }\n",
            "fillet.rs": "pub fn f() {}\n",
            "transform.rs": "",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("tessellate.rs:1: `tessellate`", problems[0])
        self.assertIn("`fillet::f`", problems[0])

    def test_use_groups_are_expanded(self):
        problems = self.run_check({
            "lib.rs": self.LIB,
            "tessellate/mod.rs": "use crate::{fillet::{f, g as h}, transform};\n",
            "fillet.rs": "",
            "transform.rs": "",
        })
        items = sorted(p.split("uses `")[1].split("`")[0] for p in problems)
        self.assertEqual(items, ["fillet::f", "fillet::g", "transform"])

    def test_siblings_may_not_use_each_other(self):
        problems = self.run_check({
            "lib.rs": self.LIB,
            "tessellate.rs": "",
            "fillet.rs": "",
            "transform.rs": "use crate::tessellate as t;\n",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("(remus-tessellate), which is not below it", problems[0])

    def test_test_code_comments_and_strings_are_exempt(self):
        self.assertEqual(self.run_check({
            "lib.rs": self.LIB,
            "tessellate/mod.rs": """\
                // crate::fillet::f in a comment
                const S: &str = "crate::fillet::f";
                #[cfg(test)]
                mod tests {
                    use crate::fillet::f;
                }
                #[cfg(test)]
                mod oracle;
            """,
            "tessellate/oracle.rs": "use crate::fillet::f;\n",
            "fillet.rs": "",
            "transform.rs": "",
        }), [])

    def test_feature_gated_test_code_is_checked(self):
        problems = self.run_check({
            "lib.rs": self.LIB,
            "tessellate.rs": '#[cfg(any(test, feature = "test-utils"))]\npub mod utils { use crate::fillet::f; }\n',
            "fillet.rs": "",
            "transform.rs": "",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("`fillet::f`", problems[0])

    def test_super_resolves_through_inline_modules(self):
        # `super::helpers` inside `mod inner {}` of tessellate/mod.rs names
        # `tessellate::helpers`, not a crate-root module.
        self.assertEqual(self.run_check({
            "lib.rs": self.LIB,
            "tessellate/mod.rs": "mod helpers;\nmod inner { use super::helpers::h; }\n",
            "tessellate/helpers.rs": "pub fn h() {}\n",
            "fillet.rs": "",
            "transform.rs": "",
        }), [])
        problems = self.run_check({
            "lib.rs": self.LIB,
            "tessellate/mod.rs": "mod deep;\n",
            "tessellate/deep.rs": "use super::super::fillet::f;\n",
            "fillet.rs": "",
            "transform.rs": "",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("deep.rs:1", problems[0])

    def test_path_attribute_is_followed(self):
        problems = self.run_check({
            "lib.rs": self.LIB,
            "tessellate.rs": '#[path = "tessellate/impl_detail.rs"]\nmod detail;\n',
            "tessellate/impl_detail.rs": "use crate::fillet::f;\n",
            "fillet.rs": "",
            "transform.rs": "",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("impl_detail.rs:1", problems[0])

    def test_root_items_are_core_but_root_reexports_are_facade(self):
        self.assertEqual(self.run_check({
            "lib.rs": self.LIB + "pub use fillet::f;\n",
            "tessellate.rs": "",
            "fillet.rs": "pub fn f() {}\n",
            "transform.rs": "",
        }), [])
        problems = self.run_check({
            "lib.rs": self.LIB + "pub enum E { A(fillet::FilletError) }\n",
            "tessellate.rs": "",
            "fillet.rs": "pub struct FilletError;\n",
            "transform.rs": "",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("`ROOT` (remus-ops-core) uses `fillet::FilletError`", problems[0])

    def test_unassigned_module_fails(self):
        problems = self.run_check({
            "lib.rs": self.LIB + "pub mod brand_new;\n",
            "tessellate.rs": "",
            "fillet.rs": "",
            "transform.rs": "",
            "brand_new.rs": "",
        })
        self.assertEqual(len(problems), 1)
        self.assertIn("`brand_new` has no tier", problems[0])

    def test_every_tier_is_ordered(self):
        allowed = graph.allowed_tiers()
        self.assertEqual(set(allowed), set(graph.TIERS))
        self.assertNotIn("remus-transform", allowed["remus-measure"])
        self.assertEqual(allowed["remus-operations"], set(graph.TIERS) - {"remus-operations"})


if __name__ == "__main__":
    unittest.main()
