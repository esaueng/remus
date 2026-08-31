# Fuzzing

`fuzz/` has bounded libFuzzer targets for public readers and kernel operations.
Reader inputs use strict `ImportLimits`; engine targets use small structured
generators and independent geometry or topology oracles, so a plausible but
wrong result is a finding as well as a panic.

`nurbs_surface` constructs 2–4 control-point rational patches, tests typed
rejection of corrupted construction data, evaluates points and derivatives,
and intersects every valid patch with a horizontal NURBS plane. Returned SSI
points, parameters, and fitted curves must satisfy the plane equation and
re-evaluate on both input surfaces. Its corpus includes a clustered marching
section that previously made a cubic refit leave the known plane.

`topology_mutation` builds a bounded box (census and `dx * dy * dz` known by
construction) and drives byte-selected topology mutations over it: face-loop
derivation and re-derivation, validated rollback of a deliberately broken
wire, rollback of staged allocations and of in-transaction
re-derivations/deletions, checkpoint restore, unreferenced-solid deletion,
and referenced-deletion refusal. The oracles are the mutation contracts:
comprehensive validation and the closed-manifold census hold after every
step, rollback reproduces the exact live state while checkpoint restore
keeps window retirements tombstoned without dangling the derivation map,
retired handles fail typed lookups forever and are never reissued, a refused
deletion leaves no partial mutation, and an accepted one retires exactly the
unshared tree — a guard box and an unrelated compound must survive. Its
corpus includes a checkpoint re-derivation seed that previously left the
loop-derivation map referencing retired loops.

Run one target locally with nightly Rust and `cargo-fuzz`:

```bash
cargo +nightly fuzz run nurbs_surface -- -max_total_time=60 -rss_limit_mb=2048
```

PR CI compiles every target. The scheduled `Fuzz Smoke` workflow runs the
public model-reader, boolean-tree, modifier, NURBS-surface, and
topology-mutation campaigns for two minutes each and retains crash
artifacts. `arena_reader` and `wasm_batch` currently compile in PR CI but
are not scheduled; native serialization, curve-intersection, and
offset-specific campaigns remain separate S4 follow-ups.
