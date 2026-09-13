# Open Kernel strategy reference

The [Remus master roadmap](roadmap.md) owns all O1–O7 priorities, dependencies,
implementation status and public-release gates. The former strategy roadmap has
been consolidated there; start with its [Open Kernel register](roadmap.md#open-kernel-register).

The [implementation specifications](open-kernel-implementation.md) retain the
issue-level scope, repository conventions, dependencies and typed acceptance gates.
The seven public claims S1–S7 are defined once in the
[master roadmap](roadmap.md#open-kernel-public-claims).

The purpose remains to make Remus provably robust, adoptable and durable:

- Publish reproducible corpus and comparison evidence, including losses.
- Preserve exact geometry and disclose every unsupported or approximate result.
- Measure native and browser workloads before optimizing.
- Provide usable Rust, JavaScript and Python entry points with stable contracts.
- Preserve STEP product structure, attributes and declared interchange semantics.
- Support independent consumers with documentation and repeatable fixtures.
- Keep hybrid modeling behind an explicit body-model design gate.

**R9 — Public claims are reproducible claims.** Every published pass rate, speedup
or conformance claim includes the harness, corpus manifest and source/artifact pins
needed to rerun it. Numeric parity bands require the O1.2f baseline; hypothetical
market positioning is not a completion criterion.

Distribution, hosting and outreach decisions remain in the
[master decision register](roadmap.md#decisions-and-exclusions). Corpora are pinned
by manifests; publish aggregates and permitted artifacts, not unlicensed source models.
