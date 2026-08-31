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

Run one target locally with nightly Rust and `cargo-fuzz`:

```bash
cargo +nightly fuzz run nurbs_surface -- -max_total_time=60 -rss_limit_mb=2048
```

PR CI compiles every target. The scheduled `Fuzz Smoke` workflow runs the
public model-reader, boolean-tree, modifier, and NURBS-surface campaigns for
two minutes each and retains crash artifacts. `arena_reader` and `wasm_batch`
currently compile in PR CI but are not scheduled; topology mutation, native
serialization, curve-intersection, and offset-specific campaigns remain
separate S4 follow-ups.
