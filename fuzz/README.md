# Importer fuzzing

The seven public model readers have bounded libFuzzer targets. Each target
uses strict `ImportLimits` so malformed inputs cannot turn a CI smoke campaign
into unbounded allocation or topology work.

Run one target locally with nightly Rust and `cargo-fuzz`:

```bash
cargo +nightly fuzz run step_reader -- -max_total_time=60
```

CI runs short smoke campaigns; the scheduled workflow runs longer campaigns
and retains crash artifacts.
