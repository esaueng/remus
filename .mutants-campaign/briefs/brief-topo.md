## Shared context (read this first)

Repo: esaueng/remus, a Rust B-Rep CAD kernel. You are working in the git worktree
`/Users/userzero/claude/remus/.claude/worktrees/mutants-geometry-topology-triage-7d1543`
on branch `test/mutants-topology-triage`. Run every command from that directory. Do NOT cd elsewhere.

Toolchain PATH (prepend in every shell you spawn):
  export PATH="$HOME/.cargo/bin:/opt/homebrew/opt/rustup/bin:$PATH"

### The campaign

We ran `cargo mutants` over remus-topology. You own EXACTLY ONE source file. For every surviving
mutant listed in your assignment, classify it as one of:

(a) **missing assertion** — a real coverage gap. Add a unit test to that file's `#[cfg(test)] mod tests`
    module that FAILS under the mutant and PASSES on the real code.
(b) **equivalent mutant** — the mutation cannot change observable behaviour (or only on an input the
    function's contract excludes). Record a one-line reason. Write NO test.
(c) **needs geometry judgment** — killing it would require deciding a genuine open question about what
    the geometry SHOULD do. Leave it, record a one-line reason. Write NO test.

### Hard rules

- Touch ONLY your assigned source file. Never edit another file, never edit Cargo.toml, never edit
  production code (only the `#[cfg(test)] mod tests` block at the bottom of your file).
- NEVER weaken, delete, or relax an existing assertion or test.
- NEVER change production code to make a test pass. If a test fails on the real code, your test is wrong.
- Tests go in the existing `#[cfg(test)] mod tests` module in your file. Keep the existing
  `#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]` header; add to it only if clippy
  demands it.
- Workspace lints deny `unwrap_used` and `panic` outside that allow header.

### Domain rules (this crate is the arena-based B-Rep data layer)

- remus-topology owns the arena, typed `Id<T>` handles, and the Vertex/Edge/Wire/Face/Shell/Solid
  entities, plus validation, the builder, the journal, persistent naming, attributes and pcurves.
  It depends only on `remus-math`.
- Most survivors here are plain data-structure contracts: an accessor returning the wrong slice, a
  mutator that does nothing, a counter returning 0, a comparison flipped in an index or validation
  rule. These are category (a) and are killed by short, direct unit tests.
- Build fixtures through the crate's own public API (`Topology`, `builder`, the entity constructors).
  `crates/topology/src/test_utils.rs` is behind the `test-utils` feature, which the crate's own test
  build does NOT enable — do not rely on it and do not enable it.
- Where a float is involved (vertex positions, tolerances, pcurve params), the geometry crate's rules
  still apply: do not assert bit-exact floats produced by a computation, and derive any tolerance you
  pin from the documented contract rather than from current output. Exact equality on a coordinate you
  literally stored and read back is fine.
- A mutant that only removes a `debug_assert`, changes a `Vec::with_capacity` hint, or flips a
  comparison at an unreachable index is an honest (b) equivalent. Say so rather than forcing a test.

### Your workflow

1. Read your whole source file, including its existing tests.
2. For each survivor, decide (a)/(b)/(c). Be honest: a forced test that does not actually kill the
   mutant is worse than an honest (b)/(c).
3. Write the new tests. Aim for a small number of high-value tests that each kill many mutants;
   name them descriptively.
4. Verify on the real code:
     cargo test -p remus-topology --lib <your module path>
   All tests must pass.
5. **Prove the kills.** Re-run mutants on just your file:
     cargo mutants -p remus-topology --no-config --baseline skip -j 3 \
       --timeout 60 -f <your file> --output <OUTDIR> -- -p remus-topology
   Then read `<OUTDIR>/mutants.out/outcomes.json` and compare the missed set against your list.
   Use the `--output` dir given in your assignment. This step is MANDATORY — do not report kill
   counts you did not measure.
6. If a claimed kill did not reproduce, either fix the test to actually kill it, or reclassify the
   mutant honestly as (b)/(c).
7. Run `cargo clippy -p remus-topology --all-targets -- -D warnings`, and run `rustfmt` on YOUR
   file(s) only (not `cargo fmt --all`) — sibling agents are editing other files in this same
   worktree at the same time. Your file must be clean.

NOTE ON THE SHARED WORKTREE: several agents work in this worktree concurrently, so a sibling's
work-in-progress can be copied into your mutants run and make unrelated tests red, which marks
mutants "caught" spuriously. Guard against this by scoping the test command of your proof re-run to
your own module, e.g. `-- -p remus-topology --lib <your::module>`. Narrowing can only remove kill
credit, never add it, so the result stays honest. Say in your report if you did this.

### What to report back

- The measured before/after missed counts for your file (from the two outcomes.json files).
- A markdown table, one row per survivor, with columns: `Mutant | Verdict | Killing test / reason`.
  The `Mutant` cell is the exact mutant name string from your assignment.
  Verdict is literally `(a)`, `(b)`, or `(c)`.
- The list of test function names you added.
- Anything you could not kill and why.

Keep your final report compact — the table plus a few lines. Do not paste source code.
