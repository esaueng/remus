# AI Disclosure

This project is developed with substantial AI assistance. Most of the code,
tests, and documentation in this repository were written by AI coding agents
working under my direction. I set the goals, review the direction, and decide
what ships; the agents do most of the typing.

I want to be straightforward about that rather than let you discover it and
wonder what else isn't being said. A solid modeling kernel lives or dies on
correctness, so the honest question isn't "was AI involved?" — it's "how do
you know the result is right?" This repository's answer is that nothing is
trusted because it compiles or because an agent said it works:

- Every operation is gated by tests: unit, property-based, golden-file,
  integration, and replayable reproduction bundles for every discovered
  defect.
- CI enforces the architecture (layer boundaries), the lints (`unsafe`,
  `unwrap`, and `panic` are all denied), the license lineage, and the full
  test suite on every change.
- Feature labels are backed by the [capability matrix](docs/kernel-maturity/capability-matrix.md)
  and [stability matrix](docs/production-readiness/stability-matrix.md), not
  by optimism. Where the evidence is thin, the label says so.
- Results are verified against ground truth — measured volumes, watertight
  manifold checks, and head-to-head comparison with an established kernel —
  not against the code's own opinion of itself.

The upstream project this fork continues was also built AI-first, and its
author was equally open about it. I've kept that spirit: the process is
disclosed, the guardrails are in the repository where you can read them, and
the standard the code is held to doesn't depend on who — or what — wrote it.

If you find something that fails that standard, that's a bug and I want to
know: open an issue, or see [SECURITY.md](./SECURITY.md) for anything
security-relevant.

— Peter, Esau Engineering
