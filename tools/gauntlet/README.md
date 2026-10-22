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
and failed downloads leave no object behind.

## Scheduled scoreboards

`.github/workflows/gauntlet.yml` runs `smoke` every night and `abc-1k`
weekly, with a manual tier selector for diagnostics. Each model has a 30
second subprocess budget and at most two models execute concurrently. The
workflow uploads only `scoreboard.json`, `scoreboard.md`, and the trend row;
corpus bytes and per-model rows stay on the ephemeral runner.

The append-only `results` branch stores historical and latest aggregate
scoreboards plus `trends/<tier>.jsonl`. Each trend row pins the UTC date,
kernel SHA, manifest SHA-256, integer stage counts, derived pass rates,
failure taxonomy, and exact/approximate boolean counts. A missing results
branch is the explicit first-run baseline. After that, any per-stage drop
larger than 50 basis points (0.50 percentage points) fails the run. The
regressed aggregate is still published, so the trend cannot hide a red run.

The untrusted corpus job has read-only repository credentials. A separate
main-only job receives the write token and publishes aggregate artifacts
without checking out or executing repository code.

### Import triage retained in aggregate artifacts

New scoreboards and trend rows include optional `failure_details`:

- `first_failure_codes` counts one primary error per failed model, grouped
  by its first failing stage. Warnings and propagated prerequisite failures
  do not multiply the count.
- `import_families` counts one hint per failed import: input access, resource
  limit, syntax, units/context, missing reference, endpoint authority, curve
  domain, pcurve, face boundary, unsupported entity, no solids, or unknown.

The family labels are conservative **heuristics**, not judgments that a file
is malformed or valid. Parse-error prose has no stable subcode, so recognized
reader templates supply hints; new or oversized messages remain `unknown`.
Use the stable stage/code counts to preserve the error contract, then replay
the pinned models before selecting an importer repair. No tolerance, resource
limit, or acceptance policy is changed by these hints.

The retained details contain only counts, fixed family labels and bounded
ASCII diagnostic codes. They contain no model paths, entity labels, source
text, or error prose. Aggregate details reconcile to the failed-model and
failed-import denominators before a new trend row is accepted. Historical
scoreboards/trends without this additive field remain readable, and existing
stage regression ratchets keep their original behavior.

Older runs cannot be classified retrospectively from their aggregate
`invalid_input` count: per-model rows were discarded. In particular, the
2026-10-18 ABC run's 193/1,000 imports and 801 `invalid_input` failures do not
identify an importer family. A new pinned ABC run is required to establish
that distribution and any later acceptance improvement; the source archive
is 1,594,129,754 bytes, so the smoke tier is a separate, smaller diagnostic
run rather than a substitute denominator.

Local verification on 2026-10-20 used kernel `de6352906` plus this diagnostic
change, the complete smoke manifest
`779fcc7fc7488b9f1a05d0361aec94acc3c2d5cf44e81729e670b58bc24c3186`,
the `ci-test` profile, default import limits, two workers, and a 120-second
per-model budget. All 50 verified inputs stayed in the denominator: 44
imported, 42 passed every stage, and the 6 import refusals yielded 5
`endpoint_authority` hints and 1 `face_boundary` hint. The other first
failures were 1 boolean topology error and 1 non-watertight tessellation.
This measures the new diagnostics on smoke; it establishes no ABC acceptance
improvement or timing comparison with the scheduled release-profile jobs.

## P-Class 8.5 operation/export slice

`manifests/p85-slice.json` pins 11 MAMBO models (Apache-2.0, same pinned
commit as `mambo.json`) selected by a declared rule before any operation
outcome was observed: the 6 lowest `sha256-rank-v1` basic models, the 3
lowest simple models, and the 2 lowest medium models at seed `8505`,
manifest order preserved. Entries reuse the parent manifest verbatim; no
corpus bytes are committed. Every selected model stays in the denominator.

Each model runs six stages in its own bounded subprocess: import (body,
cavity, carrier, unit, and CAx-IF counting), validation, a rigid transform
(30° about Z plus a fixed diagonal-fraction shift, checked by vertex-set,
volume/area, census, cavity, occupancy, and re-validation oracles), a
declared exact-only disjoint-box fuse per solid (two regions, closed-form
`s³`/`6s²` oracles, inclusion identity, occupancy), STEP export
(millimetre units, root count, carrier entities), and STEP reimport
(property bounds, census, cavities, occupancy, mesh-volume cross-check,
trim metrics). Same-kernel round-trip agreement is only one check among
several independent oracles; MAMBO fixtures carry no CAx-IF validation
properties, which the slice records as an explicit oracle gap.

Per-model verdicts distinguish `pass` from `failed` (incorrect success or
ordinary stage error), `refused` (typed `unsupported`/`quality_refused`),
`crashed` (worker-level failure), and `resource` (budget/limit failure).
The first failing stage and full provenance (kernel SHA, manifest bytes
SHA-256, model SHA-256, recipe actuals, limits, deflection, timeout) are
recorded on every row for later regression bisection.

```bash
cargo run -p remus-gauntlet -- p85-run \
  --manifest tools/gauntlet/manifests/p85-slice.json \
  --cache /tmp/remus-gauntlet-cache \
  --output p85-results \
  --timeout-ms 120000 \
  --kernel-sha "$(git rev-parse HEAD)"
```

A 120 second per-model budget is recommended: medium fixtures tessellate
and classify for tens of seconds. To replay one fixture and recipe at any
source revision, check out the revision and run:

```bash
cargo run -p remus-gauntlet -- p85-replay \
  --model <model.step> --model-id <id> --model-sha256 <sha> \
  --kernel-sha <revision> --manifest-sha256 <manifest-sha> \
  --bundle-out replay-bundle.json
```

`p85-replay` refuses model bytes whose SHA-256 differs from the pinned
value and writes a deterministic reproduction bundle to compare against
the published row.

[abc]: https://deep-geometry.github.io/abc-dataset/
