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
`scoreboard.json` and `scoreboard.md`.

## Corpus manifests

`manifests/` contains three byte-verified STEP tiers. No corpus model or
archive bytes are committed to Remus.

| Manifest | Models | Provenance |
|---|---:|---|
| `smoke.json` | 50 | `sha256-rank-v1`, seed `20260831`, over MAMBO's 113 models |
| `abc-1k.json` | 1,000 | `sha256-rank-v1`, seed `20260831`, over ABC chunk 0000's 10,000 models |
| `mambo.json` | 113 | MAMBO commit `302b8bf33f5126d0c749f60226b76dbe94f21728` |

Every entry records its stable id, pinned URL, SHA-256, upstream license or
terms class, and uncompressed byte size. The ABC manifest additionally pins
the complete source archive's SHA-256 and size. Its `creator-owned` class is
disclosure of the [ABC licensing terms][abc], not relicensing. MAMBO declares
Apache-2.0 at the pinned source commit.

Fetch a whole tier into a content-addressed cache:

```bash
cargo run -p remus-gauntlet -- fetch \
  tools/gauntlet/manifests/mambo.json \
  --cache /tmp/remus-gauntlet-cache \
  --output-list /tmp/mambo-models.txt
```

Runtime samples use the same stable SHA-256 ranking and preserve manifest
order:

```bash
cargo run -p remus-gauntlet -- fetch \
  tools/gauntlet/manifests/mambo.json \
  --cache /tmp/remus-gauntlet-cache \
  --sample 10 --seed 42
```

If an upstream archive requires manual acquisition, map its exact manifest
URL to a local file. The fetcher verifies that file against the declared
archive size and SHA-256 before extracting any selected member, then verifies
each extracted model independently:

```bash
cargo run -p remus-gauntlet -- fetch \
  tools/gauntlet/manifests/abc-1k.json \
  --cache /tmp/remus-gauntlet-cache \
  --sample 50 --seed 42 \
  --source-file \
  https://archive.nyu.edu/bitstream/2451/44309/3/abc_0000_step_v00.7z \
  /path/to/abc_0000_step_v00.7z
```

Regenerate `abc-1k.json` from a verified local archive without retaining
extracted models:

```bash
cargo run -p remus-gauntlet -- manifest-archive \
  --archive /path/to/abc_0000_step_v00.7z \
  --output tools/gauntlet/manifests/abc-1k.json \
  --name abc-1k \
  --url https://archive.nyu.edu/bitstream/2451/44309/3/abc_0000_step_v00.7z \
  --license-class creator-owned:onshape-terms-1.g.ii \
  --id-prefix abc- --sample 1000 --seed 20260831
```

The cache is fail-closed: every reuse rehashes the object, corrupt or
truncated sources are refused, archive member paths cannot escape the cache,
and failed downloads leave no object behind. Scheduled corpus execution is
the separate O1.1c roadmap item.

[abc]: https://deep-geometry.github.io/abc-dataset/
