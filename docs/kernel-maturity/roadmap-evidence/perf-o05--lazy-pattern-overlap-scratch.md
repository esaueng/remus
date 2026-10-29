---
row: PERF-O05
---
Bounded pattern-overlap scratch allocation: linear, circular and grid patterns
now copy topology only when their conservative sweep finds positive overlap
in all three axes, retaining one isolated scratch for exact material probes.
The exact-only policy, overlap threshold, typed refusal, rollback and lineage
remain unchanged. A deterministic observer pins zero scratch clones for
box-rejected disjoint/touching patterns with unrelated document contents;
scaled overlap and unresolved-candidate refusals retain their guards.
[Matched native-only pattern measurements](../../performance/perf-o05-lazy-pattern-overlap-scratch.md)
compare original source and the isolated patch on all nine 10/100/1000-instance
by 0/100/1000-unrelated-box combinations, with exact arena identity and
source/harness/binary hash receipts. This does not deliver shared-instance
architecture, delayed materialization or a WASM/browser speed claim.
