# Mutation planning cost review, 2026-10-13

The 2026-10-11 weekly mutation run planned 21 shards but left unexamined
mutants. Its 2026-09-24 cost model underestimated current operations work;
an average matrix-wide cost also could not detect an uneven round-robin
assignment. This change repairs those estimates and reports every shard's
estimated load. It preserves the resource cap and every failing verdict.

## Pinned evidence

[Run 38123404817](https://github.com/esaueng/remus/actions/runs/38123404817),
source `82f79173291eeddb2532fb0fe65c80705296d32c`, finished 2026-10-11.
The inspected artifacts were `mutants-plan` (11696131983),
`mutants-report-shard-0` (11702853912) and `mutants-report-shard-5`
(11702499099). This is a two-shard sample, not a census of all 21 shards.

| Shard | Listed | Examined | Missed | Timeout | Unexamined | Baseline build + test (s) |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 211 | 115 | 30 | 1 | 96 | 285 + 503 |
| 5 | 211 | 182 | 47 | 0 | 29 | 207 + 314 |

Occupied slot time is the sum of each mutant's build and test phase durations,
including caught, missed, timed-out and unviable outcomes. Phase medians do
not estimate the mean cost when surviving mutants run much longer.

| Package | Shard 0 samples / mean (s) | Shard 5 samples / mean (s) | Old estimate (s) | New estimate (s) |
| --- | ---: | ---: | ---: | ---: |
| remus-operations | 42 / 637.59 | 62 / 463.49 | 300 | 650 |
| remus-algo | 22 / 84.62 | 47 / 56.91 | 60 | 90 |
| remus-math | 51 / 102.81 | 73 / 37.91 | 50 | 110 |

The new estimates round each package's larger observed shard mean upward.
Other shards or future code may cost more. Blend and offset have no samples
in these artifacts and retain their existing estimates. The overhead reserve
rises from 12 to 20 minutes: shard 0's baseline alone took 788 seconds, before
setup and the second scratch directory's cold build.

## Planning behavior and limits

The planner now calculates the actual `i % shards` assignment that
`cargo-mutants 27 --sharding round-robin` uses. Its `--shuffle` changes order
after partitioning. Starting at the aggregate cost lower bound, the planner
tests each eligible matrix size and selects the first whose estimated peak
fits the capacity. A capped matrix reports over-capacity shards explicitly,
including when the aggregate estimate would have fit.

The archived full listing contains 4,422 mutants: algo 1,065, math 1,830,
operations 1,527. SHA-256 of `week-mutants.json`:
`51380c2037fe0c19b8004fba15579e1852cf5825f73fbd066dfe29d650187a09`.
Replaying it now estimates 358.2 slot-hours, a total-cost lower bound of
46 shards, and a peak of 15.0 slot-hours in the capped 24-shard matrix versus
7.9 capacity. The cap remains 24; this scope does not promise completion of
that week or silently increase CI resources.

The verdict additionally reports each package's observed mean occupied slot
time, retaining its phase medians and all existing failure conditions.
Both archived shards still fail with the same missed, timed-out and
unexamined counts. Retain their `unexamined.txt` and original source/listing
for an explicitly scoped catch-up run. Automatic carry-forward across the
weekly seven-day window is still open; this change does not implement it.

## Validation

`python3 scripts/test-mutants-verdict.py` covers imbalanced round-robin
assignment, capped imbalance, an individually over-budget mutant, observed
mean cost including survivors/unviable outcomes, and the existing clean and
fail-closed verdict cases. The archived listing was replanned and both
archived shard verdicts replayed locally. No mutation campaign was rerun;
hosted completion remains separate evidence.
