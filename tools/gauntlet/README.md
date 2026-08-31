# Remus corpus gauntlet

The gauntlet runs each STEP model in its own bounded subprocess and records
five stages: import, validation, centered probe cut, tessellation, and STEP
round-trip. It reports stable failure-taxonomy codes and keeps exact versus
approximate boolean outcomes separate.

```bash
cargo run -p remus-gauntlet -- run \
  --output gauntlet-results \
  --timeout-ms 60000 \
  model-a.step model-b.step
```

The output directory contains one row per model in `models.jsonl` plus
`scoreboard.json` and `scoreboard.md`. Corpus manifests, downloading, and
scheduled CI are separate roadmap stages and are intentionally not part of
this crate yet.
