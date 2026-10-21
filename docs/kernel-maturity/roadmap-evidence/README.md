# Roadmap evidence fragments

An implementation PR records its delivery evidence here, one file per PR,
instead of editing its row in the [master roadmap](../roadmap.md). Roadmap rows
are single-line table cells, so two open PRs that both edit a row (or
neighbouring rows) always conflict, and each merge used to make the rest of the
open PRs un-mergeable. Separate files never conflict.

## Format

Name the file `<row>--<slug>.md` in lowercase, for example
`8.1--stage-3-campaign.md` or `perf-n04--seed-pruning.md`:

```markdown
---
row: 8.1
state: Partial — stage-3 campaign remaining
---
Stage-2 campaign extended to sweeps (`tests/differential.rs`, PR #1235).
```

- `row` (required): the displayed ID (`8.1`, `PERF-N04`, `B82`, `O4.5`) or
  the anchor id (`p-8-1`, `perf-n04`). It must match exactly one table row.
- `state` (optional): replaces the row's state cell outright, so write the
  complete new state and remaining scope. Only one pending fragment may set
  the state of a given row.
- Body: the evidence text, written as you would have appended it to the row.
  Line breaks are folded into spaces.

The text lands in the row's Evidence column. Rows without one (the PERF
inventory, whose `Priority / phase / evidence` column holds codes that
`scripts/sync-roadmap-inventory.py` parses) receive it in their State column.

`python3 scripts/roadmap-evidence.py` checks every fragment against the
roadmap; `python3 scripts/test-roadmap-evidence.py` tests the tool.

## Folding

`python3 scripts/roadmap-evidence.py --fold` appends each fragment to its row
(in file-name order), applies any `state`, and deletes the folded files. Run it
on a fresh branch from `main` and deliver the result as a standalone
`docs(roadmap): fold evidence` PR, so the roadmap rewrite never rides along
with kernel work.
