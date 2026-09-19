## Shared context (read this first)

Repo: esaueng/remus, a Rust B-Rep CAD kernel. You are working in the git worktree
`/Users/userzero/claude/remus/.claude/worktrees/mutants-geometry-topology-triage-7d1543`
on branch `test/mutants-geometry-triage`. Run every command from that directory. Do NOT cd elsewhere.

Toolchain PATH (prepend in every shell you spawn):
  export PATH="$HOME/.cargo/bin:/opt/homebrew/opt/rustup/bin:$PATH"

### The campaign

We ran `cargo mutants` over remus-geometry. You own EXACTLY ONE source file. For every surviving
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

### Numerical rules (this crate is float-heavy — these are not optional)

- **Do NOT assert bit-exact floats.** No `assert_eq!` on an `f64` produced by a computation.
- A test that pins a tolerance must derive its expected value from the function's **documented
  contract** (the doc comment / the closed-form maths), NEVER from "whatever the current code prints".
  If you cannot justify the expected number from the contract, the mutant is (c), not (a).
- Prefer assertions on invariants the contract guarantees: monotonicity, endpoint inclusion,
  count/ordering, a closed-form analytic value (circle circumference, distance to a known point),
  a bound that the doc comment states.
- Fixture-value traps: avoid radii/params of 0 or 1, and avoid values where two different operators
  coincide numerically (e.g. `a*b == a+b` at a=b=2). Pick fixture constants so each mutated operator
  gives a *different* answer. Verify this, do not assume it.

### Your workflow

1. Read your whole source file, including its existing tests.
2. For each survivor, decide (a)/(b)/(c). Be honest: a forced test that does not actually kill the
   mutant is worse than an honest (b)/(c).
3. Write the new tests. Aim for a small number of high-value tests that each kill many mutants;
   name them descriptively.
4. Verify on the real code:
     cargo test -p remus-geometry --lib <your module path>
   All tests must pass.
5. **Prove the kills.** Re-run mutants on just your file:
     cargo mutants -p remus-geometry --no-config --baseline skip -j 3 \
       -f <your file> --output <OUTDIR> -- -p remus-geometry
   Then read `<OUTDIR>/mutants.out/outcomes.json` and compare the missed set against your list.
   Use the `--output` dir given in your assignment. This step is MANDATORY — do not report kill
   counts you did not measure.
6. If a claimed kill did not reproduce, either fix the test to actually kill it, or reclassify the
   mutant honestly as (b)/(c).
7. Run `cargo clippy -p remus-geometry --all-targets -- -D warnings` and `cargo fmt --all`.
   Your file must be clean.

### What to report back

- The measured before/after missed counts for your file (from the two outcomes.json files).
- A markdown table, one row per survivor, with columns: `Mutant | Verdict | Killing test / reason`.
  The `Mutant` cell is the exact mutant name string from your assignment.
  Verdict is literally `(a)`, `(b)`, or `(c)`.
- The list of test function names you added.
- Anything you could not kill and why.

Keep your final report compact — the table plus a few lines. Do not paste source code.
