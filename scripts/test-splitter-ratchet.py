#!/usr/bin/env python3
"""Exercise the splitter special-case ratchet against small Rust fixtures."""
import pathlib
import subprocess
import sys
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).with_name("check-splitter-ratchet.py")
REPO = SCRIPT.parent.parent
SPLITTER = "crates/algo/src/builder/face_splitter"

SPECIAL_CASES = """\
//! Fixture: braces in comments { and strings must not shift depth.
pub(super) fn entry_a() -> &'static str {
    "} pub fn not_a_function() {"
}

/* pub fn commented_out() {} */
pub fn entry_b<'a>(text: &'a str) -> char {
    let _ = text;
    '{'
}

fn private_helper() {}

mod nested {
    pub(super) fn stays_inside_the_file() {}
    pub(in crate::builder::face_splitter) fn entry_c() {}
}

#[cfg(test)]
mod tests {
    pub fn test_only() {}
}
"""

SPLITTER_MOD = """\
fn split_plane_face_by_arrangement() -> usize {
    1
}

fn split_face_2d_impl(input: Option<usize>) -> Result<usize, ()> {
    let Some(value) = input else {
        return Ok(0);
    };
    let annotated = [1usize]
        .iter()
        .map(|v| -> Result<Option<usize>, ()> {
            if *v == 0 {
                return Ok(None);
            }
            Ok(Some(*v))
        })
        .count();
    let expression_body = [1usize].iter().filter(|v| match v {
        0 => return false,
        _ => true,
    });
    let either = value > 1 || annotated > 2;
    fn nested() -> usize {
        return 3;
    }
    if either {
        return Ok(nested());
    }
    let _ = expression_body;
    Ok(value | 1)
}
"""

ROADMAP = '| <a id="o-o2-3d"></a>O2.3d Special-case migration + ratchet | B | Pending | — |\n'

# The fixture as the gate must read it: three production functions with a
# `pub` visibility, two of them reachable from outside (`nested`'s
# `pub(super)` helper is not), and two `return`s that belong to the dispatch
# function itself.
FLOORS = {
    "mod.dispatch_returns.split_face_2d_impl": 2,
    "mod.emitter_lines.arrangement_regions_from_combined": 0,
    "mod.emitter_lines.arrangement_regions_from_inputs": 0,
    "mod.emitter_lines.split_cylinder_band_by_arrangement": 0,
    "mod.emitter_lines.split_periodic_face_by_rim_chains": 0,
    "mod.emitter_lines.split_periodic_face_by_winding_chain": 0,
    "mod.emitter_lines.split_plane_face_by_arrangement": 3,
    "special_cases.entry_points": 3,
    "special_cases.production_lines": 18,
    "special_cases.pub_fns": 4,
}


def baseline(floors=None, ledger=()):
    floors = FLOORS if floors is None else floors
    lines = ["# fixture baseline"] + [f"{count} {metric}" for metric, count in floors.items()]
    return "\n".join(lines + list(ledger)) + "\n"


class SplitterRatchetTests(unittest.TestCase):
    def run_gate(self, *, special=SPECIAL_CASES, mod=SPLITTER_MOD, base=None, roadmap=ROADMAP):
        with tempfile.TemporaryDirectory(prefix="splitter-ratchet-") as directory:
            root = pathlib.Path(directory)
            files = {
                f"{SPLITTER}/special_cases.rs": special,
                f"{SPLITTER}/mod.rs": mod,
                "scripts/splitter-ratchet-baseline.txt": baseline() if base is None else base,
                "docs/kernel-maturity/roadmap.md": roadmap,
            }
            for relative, text in files.items():
                if text is None:
                    continue
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text, encoding="utf-8")
            return subprocess.run(
                [sys.executable, str(SCRIPT), "--root", directory],
                text=True, capture_output=True, check=False,
            )

    def assert_exit(self, result, code, needle=None):
        self.assertEqual(result.returncode, code, result.stdout + result.stderr)
        if needle is not None:
            self.assertIn(needle, result.stdout + result.stderr)

    def test_fixture_counts_match_the_hand_count(self):
        self.assert_exit(self.run_gate(), 0, "splitter ratchet OK")

    def test_new_entry_point_fails(self):
        grown = SPECIAL_CASES.replace("fn private_helper() {}", "pub(super) fn private_helper() {}")
        result = self.run_gate(special=grown)
        self.assert_exit(result, 1, "VIOLATION: `special_cases.entry_points` grew: allowed 3, observed 4")
        self.assertIn("`special_cases.pub_fns` grew", result.stdout)

    def test_new_production_line_fails_but_a_test_line_does_not(self):
        grown = SPECIAL_CASES.replace("fn private_helper() {}", "fn private_helper() {}\nfn another() {}")
        self.assert_exit(
            self.run_gate(special=grown), 1,
            "VIOLATION: `special_cases.production_lines` grew: allowed 18, observed 19",
        )
        tested = SPECIAL_CASES.replace("    pub fn test_only() {}", "    pub fn test_only() {}\n    pub fn more() {}")
        self.assert_exit(self.run_gate(special=tested), 0)

    def test_new_dispatch_return_fails_but_a_closure_return_does_not(self):
        dispatch = SPLITTER_MOD.replace("    let _ = expression_body;", "    if value == 9 {\n        return Err(());\n    }\n    let _ = expression_body;")
        self.assert_exit(
            self.run_gate(mod=dispatch), 1,
            "VIOLATION: `mod.dispatch_returns.split_face_2d_impl` grew: allowed 2, observed 3",
        )
        closure = SPLITTER_MOD.replace("        0 => return false,", "        0 => return false,\n        7 => return true,")
        self.assert_exit(self.run_gate(mod=closure), 0)

    def test_emitter_growth_fails(self):
        grown = SPLITTER_MOD.replace("    1\n}", "    let one = 1;\n    one\n}")
        self.assert_exit(
            self.run_gate(mod=grown), 1,
            "VIOLATION: `mod.emitter_lines.split_plane_face_by_arrangement` grew: allowed 3, observed 4",
        )

    def test_ledger_line_naming_a_roadmap_row_admits_a_rise(self):
        grown = SPECIAL_CASES.replace("fn private_helper() {}", "fn private_helper() {}\nfn another() {}")
        ledger = ["raise special_cases.production_lines +1 o-o2-3d fixture: one more helper"]
        self.assert_exit(self.run_gate(special=grown, base=baseline(ledger=ledger)), 0, "1 ledgered rise")

    def test_ledger_line_must_name_an_existing_row(self):
        grown = SPECIAL_CASES.replace("fn private_helper() {}", "fn private_helper() {}\nfn another() {}")
        ledger = ["raise special_cases.production_lines +1 o-no-such-row fixture"]
        result = self.run_gate(special=grown, base=baseline(ledger=ledger))
        self.assert_exit(result, 1, "names `o-no-such-row`, which is not a row anchor")
        # The unbacked raise buys nothing: the growth is still reported.
        self.assertIn("`special_cases.production_lines` grew", result.stdout)

    def test_unrecorded_shrinkage_is_stale(self):
        shrunk = SPECIAL_CASES.replace("fn private_helper() {}\n", "")
        self.assert_exit(
            self.run_gate(special=shrunk), 1,
            "STALE: `special_cases.production_lines` shrank: allowed 18, observed 17",
        )

    def test_deleted_emitter_and_deleted_file_are_stale_not_silent(self):
        gone = SPLITTER_MOD.replace("fn split_plane_face_by_arrangement", "fn renamed_emitter")
        self.assert_exit(
            self.run_gate(mod=gone), 1,
            "STALE: `mod.emitter_lines.split_plane_face_by_arrangement` shrank: allowed 3, observed 0",
        )
        self.assert_exit(
            self.run_gate(special=None), 1,
            "STALE: `special_cases.production_lines` shrank: allowed 18, observed 0",
        )

    def test_metric_without_a_floor_fails(self):
        floors = {k: v for k, v in FLOORS.items() if k != "special_cases.pub_fns"}
        self.assert_exit(
            self.run_gate(base=baseline(floors)), 1,
            "VIOLATION: `special_cases.pub_fns` (observed 4) has no floor line",
        )

    def test_a_scan_that_cannot_run_exits_two(self):
        unbalanced = SPECIAL_CASES + "\nfn broken() {\n"
        self.assert_exit(self.run_gate(special=unbalanced), 2, "the gate could not run")
        self.assert_exit(self.run_gate(roadmap=None), 2, "roadmap.md is missing")
        self.assert_exit(self.run_gate(base="not a baseline line\n"), 2, "not a floor or ledger line")

    def test_repo_tree_matches_its_baseline(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--root", str(REPO)],
            text=True, capture_output=True, check=False,
        )
        self.assert_exit(result, 0, "splitter ratchet OK")

    def test_repo_tree_fails_on_an_added_special_case(self):
        """Run the gate against a copy of THIS tree (proves it can fail)."""
        with tempfile.TemporaryDirectory(prefix="splitter-ratchet-repo-") as directory:
            root = pathlib.Path(directory)
            for relative in (
                f"{SPLITTER}/special_cases.rs",
                f"{SPLITTER}/mod.rs",
                "scripts/splitter-ratchet-baseline.txt",
                "docs/kernel-maturity/roadmap.md",
            ):
                target = root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes((REPO / relative).read_bytes())
            special = root / SPLITTER / "special_cases.rs"
            special.write_text(
                special.read_text(encoding="utf-8") + "\npub(super) fn one_more_special_case() {}\n",
                encoding="utf-8",
            )
            result = subprocess.run(
                [sys.executable, str(SCRIPT), "--root", directory],
                text=True, capture_output=True, check=False,
            )
        self.assert_exit(result, 1, "`special_cases.entry_points` grew: allowed 16, observed 17")


if __name__ == "__main__":
    unittest.main()
