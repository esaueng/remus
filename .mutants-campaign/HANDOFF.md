# cargo-mutants campaign handoff — 2026-09-19

**This directory is handoff scaffolding. Delete it before opening the topology PR.**
It was committed with `git add -f` because it sits outside the repo's normal tracked
paths; it exists only so the campaign can resume on another machine.

---

## 1. Geometry — DONE

PR **https://github.com/esaueng/remus/pull/528** (`test/mutants-geometry-triage`), ready for
review, not merged.

- 1138 survivors -> 397. **741 killed** by 119 new unit tests (lib 107 -> 226).
- Cross-check passed: all 741 mutants classified `(a)` are killed, all 397 classified
  `(b)`/`(c)`/timeout still survive, 0 new survivors.
- Triage doc committed at `docs/kernel-maturity/mutants-geometry-2026-09-19.md` (1138 rows).

**Nothing outstanding.** The tables under `tables-geometry/` and the three geometry
`outcomes` files here are its backing evidence, already reflected in the committed doc.

---

## 2. Topology — UNFINISHED

Branch `test/mutants-topology-triage`, based on `origin/main` @ `b13ff97b`.
Tests are committed; **no kill count is verified.**

### Baseline (the measurement that cannot be re-taken on this branch)

`outcomes/topo-base.outcomes.json.gz` — take this seriously: once the topology tests are
committed, the before-state is gone from this branch. Re-measuring needs a checkout of
`origin/main` (`crates/topology` was byte-identical between `1c1e145c`, where the baseline
was taken, and `b13ff97b`, where this branch is based — verified, so the baseline is valid).

- 1557 mutants: 699 caught, **515 missed**, 343 unviable.
- **47 of those 515 are in `test_utils.rs` and are missed by construction** — that module is
  behind the `test-utils` feature, which the crate's own test build does not enable, so the
  code is never compiled and no test can kill it. Exclude it.
- **Real survivor pool: 468, across 19 files.**

### Per-file status

| Group | Files | Survivors | Tests written | Triage table | Proof run |
|---|---|---:|---|---|---|
| builder | `builder.rs` | 116 | yes | **yes** (116 rows, validated) | complete, 322 mutants |
| validation | `validation.rs` | 78 | yes | **yes** (78 rows, validated) | complete, module-scoped |
| edge | `edge.rs` | 58 | yes | **yes** (58 rows, validated) | complete, module-scoped |
| naming | `naming.rs` | 48 | yes | **yes** (48 rows, validated) | complete, module-scoped |
| face | `face.rs` | 41 | yes | **yes** (41 rows, validated) | complete, module-scoped |
| sidetables | `attributes.rs`, `journal.rs` | 36 | yes | **yes** (36 rows, validated) | complete, unscoped |
| traversal | `explorer.rs`, `adjacency.rs` | 36 | yes | **yes** (36 rows, validated) | complete, module-scoped |
| smallents | `pcurve/coedge/wire/shell/solid/compsolid/vertex/face_loop` | 25 | yes | **yes** (25 rows, validated) | complete, module-scoped |
| **arenacore** | `topology.rs`, `arena.rs` | 30 | yes | **NO** | complete but **unanalyzed** |

"validated" means every row's mutant string was matched, in order, against the baseline
survivor list for that file by `parse_tables.py`. 8 of 9 groups: 438 of the 468 survivors
carry a finished, checked table.

### builder.rs — finished (correcting an earlier read of its status)

Its agent did **not** die mid-run. It completed and reported while the session was being
wound down: 322-mutant proof run, **116 -> 13 missed (103 killed)**, all 13 residuals
classified `(b)` and the claimed-`(b)` set matches the measured still-missed set exactly.
Table and outcomes are parked here. No further work needed beyond the authoritative sweep.

### arenacore — the one real gap

Tests are written and the proof run completed (220 mutants), but the agent never analyzed it
and never wrote a table. Its `missed.txt` shows **36**, which is more than the 30 baseline
survivors — that looked alarming and was reconciled here:

- 27 of the 30 baseline survivors are **killed**.
- 3 of the 30 still survive.
- The other **33 "missed" are scoping artifacts**: mutants the crate-wide baseline caught via
  tests in other modules, which the agent's narrowed test command no longer runs.

So the anomaly is explained and benign, but it is **not** a substitute for the table. Someone
still has to classify those 30 survivors as (a)/(b)/(c).

### Why no kill count is claimed

Most per-file proof runs narrowed their inner test command to their own module
(`-- -p remus-topology --lib <module>::`). That was deliberate — nine agents shared one
worktree, and a sibling's work-in-progress being copied mid-run marks mutants "caught"
spuriously. But narrowing also drops kill credit the crate-wide baseline had, so those
per-file numbers are conservative and **not comparable to the baseline**. Do not add them up
and publish the total.

---

## 3. Remaining work, in order

1. Write the `arenacore` triage table (30 rows: `topology.rs` 19 + `arena.rs` 11).
   Survivor lists: `survivors/topo-topology.txt`, `survivors/topo-arena.txt`.
   Its completed proof run is `outcomes/topo-after-arenacore.outcomes.json.gz`.
2. Run **one authoritative full-crate after-sweep** on the final tree — unscoped, so it is
   comparable to the baseline:
   ```
   cargo mutants -p remus-topology --no-config --baseline skip --timeout 60 -j 8 \
     --output <dir> -- -p remus-topology
   ```
3. Cross-check verdicts against it, exactly as the geometry campaign did: every mutant
   classified `(a)` must be killed and every `(b)`/`(c)` must survive. Reclassify honestly
   where they disagree; do not restate an agent's claim the sweep does not support.
4. Generate `docs/kernel-maturity/mutants-topology-<date>.md` — same shape as
   `docs/kernel-maturity/mutants-geometry-2026-09-19.md` and `mutants-check-2026-09-18.md`:
   method, run provenance, before/after per file, then one row per survivor.
   `parse_tables.py` here parses the tables (it tolerates both table styles the agents
   emitted: with and without outer pipes, and backtick-quoted cells containing raw `||`).
5. Amend or follow up the WIP commit so the message no longer says kills are unmeasured.
6. `git rm -r .mutants-campaign`, then open a ready-for-review PR. **Do not merge.**

## 4. What the next machine needs

- `cargo install cargo-mutants --version 27.0.0 --locked` (not present by default; the
  weekly workflow pins this version).
- Toolchain: cargo only lives under `/opt/homebrew/opt/rustup/bin` on the current box —
  `export PATH="$HOME/.cargo/bin:/opt/homebrew/opt/rustup/bin:$PATH"`.
- **`.cargo/mutants.toml` `examine_globs` exclude `remus-topology`**, so a plain
  `cargo mutants -p remus-topology` examines *zero* mutants. `--no-config` is required.
  Local runs only — do not change the committed CI scope.
- **cargo-mutants 27 rejects `--in-place` together with `--jobs`.** Pick one. This campaign
  used `--jobs` over copied trees; that also keeps the worktree editable during a run.
- Long runs get killed by the harness's background-task limit (it happened twice). Shard by
  file and merge by mutant name, or be ready to resume.

## 5. Contents

- `tables-topo/*.md` — 8 finished topology triage tables.
- `tables-geometry/*.md` — 10 geometry tables backing PR #528.
- `survivors/topo-*.txt` — per-file baseline survivor lists (the `-all`, `-arenacore`,
  `-sidetables`, `-smallents`, `-traversal` files are concatenations used for grouped agents).
- `outcomes/*.outcomes.json.gz` — full cargo-mutants output, gzipped.
- `outcomes/*.tsv` — the same runs slimmed to `summary / file / mutant name`, which is all
  the counting needs. Every figure in this note is reproducible from these, e.g.
  `awk -F'\t' '$1=="MissedMutant" && $2!="crates/topology/src/test_utils.rs"' outcomes/topo-base.tsv | wc -l` -> 468.
- `briefs/` — the instructions the per-file agents were given.
- `parse_tables.py` — table parser/validator.
