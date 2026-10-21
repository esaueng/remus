#!/usr/bin/env python3
"""Exercise roadmap evidence fragments against a miniature roadmap and the real one."""
import importlib.util
import pathlib
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).with_name("roadmap-evidence.py")
SPEC = importlib.util.spec_from_file_location("roadmap_evidence", SCRIPT)
evidence = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evidence)

# The table shapes the real roadmap uses: an Evidence column, a state column
# with prose, and the PERF inventory whose second column holds parsed codes.
ROADMAP = "\n".join([
    "# Remus master roadmap",
    "",
    "| Issue | State and remaining scope | Evidence |",
    "| --- | --- | --- |",
    '| <a id="p-8-1"></a>8.1 Differential testing harness | Partial — stage 2 | Campaign `a \\| b` harness. |',
    '| <a id="p-8-2"></a>8.2 Performance budget gates | Pending | — |',
    "",
    "| ID | Priority / phase / evidence | Work | State |",
    "| --- | --- | --- | --- |",
    '| <a id="perf-n04"></a>PERF-N04 | P1 / 3 / M | Prune seeds. | Pending |',
    "",
    "Prose mentioning | 8.1 | outside a table.",
])


def fragment(row, text, state=None, name=None):
    return {"name": name or f"{row.lower()}--x.md", "row": row, "state": state, "evidence": text}


def row_of(markdown, anchor):
    return next(line for line in markdown.split("\n") if f'id="{anchor}"' in line)


class FragmentParsingTests(unittest.TestCase):
    def test_parses_front_matter_and_folds_line_breaks(self):
        parsed = evidence.parse_fragment("8.1--x.md", "---\nrow: 8.1\nstate: Done\n---\nFirst line\nsecond.\n")
        self.assertEqual(parsed, {"name": "8.1--x.md", "row": "8.1", "state": "Done", "evidence": "First line second."})

    def test_refuses_fragments_the_fold_could_not_place(self):
        for source, message in (
            ("no front matter", "front matter"),
            ("---\nstate: Done\n---\ntext", "needs a row"),
            ("---\nrow: 8.1\nowner: me\n---\ntext", "unknown"),
            ("---\nrow: 8.1\n---\n  \n", "no evidence"),
        ):
            with self.assertRaisesRegex(evidence.FragmentError, message):
                evidence.parse_fragment("a.md", source)

    def test_keeps_escaped_and_code_span_pipes_in_their_cell(self):
        self.assertEqual(evidence.split_row("| a | `x | y` | b \\| c |"), ["a", "`x | y`", "b \\| c"])


class FoldTests(unittest.TestCase):
    def test_appends_evidence_and_leaves_other_lines_identical(self):
        out = evidence.fold_evidence(ROADMAP, [fragment("8.1", "Stage 3 campaign.")])
        self.assertEqual(
            evidence.split_row(row_of(out, "p-8-1"))[2], "Campaign `a \\| b` harness. Stage 3 campaign."
        )
        changed = [(a, b) for a, b in zip(ROADMAP.split("\n"), out.split("\n")) if a != b]
        self.assertEqual(len(changed), 1)

    def test_replaces_empty_evidence_and_escapes_pipes(self):
        out = evidence.fold_evidence(ROADMAP, [fragment("p-8-2", "Gate a|b.", state="Partial")])
        self.assertEqual(evidence.split_row(row_of(out, "p-8-2"))[1:], ["Partial", "Gate a\\|b."])

    def test_never_writes_prose_into_the_parsed_perf_codes(self):
        out = evidence.fold_evidence(ROADMAP, [fragment("PERF-N04", "Measured.")])
        self.assertEqual(
            evidence.split_row(row_of(out, "perf-n04")),
            ['<a id="perf-n04"></a>PERF-N04', "P1 / 3 / M", "Prune seeds.", "Pending Measured."],
        )

    def test_keeps_earlier_evidence_when_a_later_fragment_sets_a_shared_state_cell(self):
        out = evidence.fold_evidence(ROADMAP, [
            fragment("PERF-N04", "Measured.", name="perf-n04--a.md"),
            fragment("PERF-N04", "Closed.", state="Done", name="perf-n04--b.md"),
        ])
        self.assertEqual(evidence.split_row(row_of(out, "perf-n04"))[3], "Done Measured. Closed.")

    def test_orders_fragments_for_one_row_by_file_name(self):
        out = evidence.fold_evidence(ROADMAP, [
            fragment("8.1", "Second.", name="8.1--b.md"),
            fragment("8.1", "First.", name="8.1--a.md"),
        ])
        self.assertIn("harness. First. Second.", row_of(out, "p-8-1"))

    def test_refuses_unknown_rows_and_competing_states(self):
        with self.assertRaisesRegex(evidence.FragmentError, "matches 0 roadmap rows"):
            evidence.fold_evidence(ROADMAP, [fragment("9.9", "x")])
        with self.assertRaisesRegex(evidence.FragmentError, "already sets the state"):
            evidence.fold_evidence(ROADMAP, [
                fragment("8.1", "x", state="Done", name="8.1--a.md"),
                fragment("8.1", "y", state="Blocked", name="8.1--b.md"),
            ])


class RepositoryTests(unittest.TestCase):
    def make_repo(self, directory, fragments):
        root = pathlib.Path(directory)
        (root / evidence.ROADMAP_PATH).parent.mkdir(parents=True)
        (root / evidence.ROADMAP_PATH).write_text(ROADMAP, encoding="utf-8")
        (root / evidence.FRAGMENT_DIR).mkdir(parents=True)
        (root / evidence.FRAGMENT_DIR / "README.md").write_text("# not a fragment")
        for name, source in fragments.items():
            (root / evidence.FRAGMENT_DIR / name).write_text(source, encoding="utf-8")
        return root

    def test_folds_and_deletes_only_folded_fragments(self):
        with tempfile.TemporaryDirectory(prefix="roadmap-evidence-") as directory:
            root = self.make_repo(directory, {"8.1--stage-3.md": "---\nrow: 8.1\n---\nStage 3.\n"})
            self.assertEqual(len(evidence.check_repo(root)), 1)
            self.assertEqual(len(evidence.fold_repo(root)), 1)
            self.assertIn("harness. Stage 3. |", (root / evidence.ROADMAP_PATH).read_text(encoding="utf-8"))
            self.assertFalse((root / evidence.FRAGMENT_DIR / "8.1--stage-3.md").exists())
            self.assertTrue((root / evidence.FRAGMENT_DIR / "README.md").exists())

    def test_refuses_badly_named_fragments(self):
        with tempfile.TemporaryDirectory(prefix="roadmap-evidence-") as directory:
            root = self.make_repo(directory, {"8.1 notes.md": "---\nrow: 8.1\n---\nx\n"})
            with self.assertRaisesRegex(evidence.FragmentError, "<row>--<slug>"):
                evidence.check_repo(root)

    def test_every_real_anchored_row_can_receive_evidence(self):
        markdown = (evidence.ROOT / evidence.ROADMAP_PATH).read_text(encoding="utf-8")
        anchors = evidence.ANCHOR.findall(
            "\n".join(line for line in markdown.split("\n") if evidence.is_table_row(line))
        )
        self.assertGreater(len(anchors), 300)
        out = evidence.fold_evidence(markdown, [fragment(anchor, "probe") for anchor in anchors])
        self.assertEqual(out.count("probe"), markdown.count("probe") + len(anchors))

    def test_accepts_every_pending_fragment_in_this_repository(self):
        evidence.check_repo()


if __name__ == "__main__":
    unittest.main()
