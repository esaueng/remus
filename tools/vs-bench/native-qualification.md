# Native qualification runner (O1.2a first slice)

The first **executable** Remus qualification runner on the existing O1.2d
protocol. It executes a bounded representative set natively through the
`Model` facade and maps each real execution/oracle result to one versioned
observation. It is **not** a head-to-head comparison: one native kernel runs,
no competitor is assumed available, and no ranking or speed claim is made
(the protocol itself strips timing columns for single-kernel reports).

- Runner binary: `tools/vs-bench/src/bin/remus_native_qualification.rs`
  (`cargo run -p remus-vs-bench --bin remus-native-qualification -- …`).
- Mapping and case library: `tools/vs-bench/src/qualification.rs` (the O1.2d
  schema in `src/lib.rs` is untouched).
- Regression tests: `tools/vs-bench/tests/native_qualification.rs` (11 tests).
- Reference run: `tools/vs-bench/runs/remus-native-75cce2c/`
  (`job.json`, `report.json`, `attempts.json`).

## Reproduce

From the repository root (needs the repo's Rust toolchain; no Node,
wasm-pack, or proprietary SDK):

```sh
cargo run -p remus-vs-bench --bin remus-native-qualification -- \
  --out /tmp/job.json --attempts /tmp/attempts.json
cargo run -p remus-vs-bench -- < /tmp/job.json > /tmp/report.json
cargo test -p remus-vs-bench
```

Parent exit 0 means a complete job was emitted (kernel refusals inside are
data, not failure); exit 2 means usage, harness-identity, or
incomplete-evidence failure. Scorecard exit 0/1/2 keeps the existing CLI
contract (pass / gates failed / invalid input).

## Case set

Every case declares supported scope, independent oracle, tolerance/error
intent, a 60-second per-repetition wall-clock ceiling (harness safety bound,
not a latency band), and a pinned reproduction identity. The oracle is always
closed-form arithmetic or the algebraic empty set — computed without kernel
geometry — checked against one kernel measurement; validation and mesh
watertightness are separate independent gates, not second measurements of the
same value. Each case runs twice in fresh child processes; disagreement
becomes `repeat_agrees: false`, never a silent pick.

| Case | Scope | Oracle | Tolerance intent | Repro |
| --- | --- | --- | --- | --- |
| `box-fuse-half-overlap` | axis-aligned box/box fuse, exact journaled path | volume 1.5 = 1 + 1 − 0.5 | rel vol err ≤ 1e-6, exact, zero budget | 1³ + 1³@(0.5,0,0), Fuse |
| `box-fuse-identical` | coincident fuse incl. unchanged-stock guard | volume 1.0 | rel vol err ≤ 1e-6, exact, zero budget | 1³ + 1³, Fuse |
| `box-cut-contained-cavity` | full-containment cut → hollow solid + cavity shell | volume 7.0 = 8 − 1, area 30.0 = 24 + 6 | rel vol/area err ≤ 1e-6, exact, zero budget | 2³ − 1³@(0.5,0.5,0.5), Cut |
| `box-cut-identical-empty` | algebraic empty via subtraction, plain facade path | A − A = {}; typed `EmptyResult` only | outcome identity, no numeric tol | 1³ − 1³, Cut |
| `box-intersect-disjoint-empty` | algebraic empty via disjoint intersection, plain path | zero faces + ~0 volume, or `EmptyResult` | vol ≤ 1e-6 + zero faces; none on refusals | 1³ ∩ 1³@(5,5,5) |
| `step-cylinder-preservation` | STEP import → measure → write → reimport | cylinder 160π (r=4, h=10) + stability ≤ 1e-9 | rel import err ≤ 1e-6, exact, zero budget | `crates/io/tests/data/axis2_optional_attrs_cylinder.step` |

A timed-out, crashed, or otherwise incomplete case yields **no** observation:
the schema requires measured values for every applicable metric, so the
runner records the attempt and refuses to emit a partial job instead. A typed
kernel refusal is kept as a refusal; a claimed success that misses its oracle
keeps success with `oracle_agrees: false` (the scorecard derives
`silent_wrong`). Nothing is relabeled.

## Baseline result at `75cce2c9`

Source `75cce2c9cfdb2c6f3b185c00045f75dc792d91b3`, rustc 1.96.0,
`tools/gauntlet/manifests/smoke.json` pinned at
`779fcc7f…c3186`, STEP fixture blob `8200e82d…41f08`. Attempts 12/12 with
evidence; 0 timeouts, 0 crashes, 0 resource failures. Scorecard `passed:
true` (exit 0).

| Outcome | Denominator |
| --- | --- |
| `exact_success` | 5/6 |
| `typed_refusal` (`EmptyResult: Cut of identical solids`) | 1/6 |
| disclosed approximation, repaired success, correct-generic success | 0/6 |
| `silent_wrong`, `invalid_success`, `untyped_error`, `nondeterminism` | 0/6 |
| `crash`, `hang_or_budget_overrun` | 0/6 |

Measured agreement (relative volume error): fuse-half 8.9e-16, fuse-identical
8.9e-16, cavity 1.3e-15 with area error exactly 0.0 (24 + 6 whole-boundary),
STEP import 1.5e-13 against 160π with bit-identical round-trip (all four
fidelity errors 0.0). Validation accepts every produced solid including the
zero-face disjoint result; evolution completeness is 1.0 on all three
journaled cases; tessellation watertightness holds at 0.01 mm deflection.
Single-kernel report carries no runtime columns, so no speed claim follows.

## Explicit non-claims

- No competitor ran: no head-to-head ranking, no parity band, no O1.2f
  baseline pin. Reference runners, W1–W8 scenarios, and the results page
  remain O1.2a–c/e/f work.
- WASM/package parity is a separate agent's scope; this run is native only
  (`remus-native@…`), and native/WASM agreement is never used as geometric
  proof here.
- The two mass-property routes (exact-integrated vs tessellated volume) are
  not treated as independent oracles of each other; independence comes from
  the closed-form arithmetic.

## Out-of-scope observations (witnesses for owners, not cases)

**O-1 — journaled empty-outcome divergence (Boolean API contracts owner).**
`Model::boolean_journaled` on either empty input returns
`Algo(AssemblyFailed("no faces selected"))` instead of the documented
`EmptyResult`/zero-face contract the plain `Model::boolean` path honors
(verified: `--worker box-cut-identical-empty` and
`--worker box-intersect-disjoint-empty` on the journaled path, same binary).
Source pin: `crates/operations/src/journal_ops.rs` `boolean_journaled` calls
`gfa::boolean_with_entity_evolution` directly, bypassing the
trivial-relation short-circuits in `crates/operations/src/boolean/mod.rs`
(identical-solids and containment arms). Not repaired here; empty cases in
this set run on the plain path where the contract is qualified.

**O-2 — STEP import records no journal evolution (evolution coverage
owner).** The STEP worker measures 0.0 evolution coverage after import, so
the scenario declares `topology_producing: false` and carries only
interchange/geometry metrics; the 0.0 observation is retained in
`attempts.json` (`journal_note`). Import transcribes foreign topology rather
than constructing it; whether it should journal is the owner's call.

## Runner self-tests (fault mapping, no production injection)

`tests/native_qualification.rs` covers: real end-to-end run judged by the
real scorecard; injected wrong success → `silent_wrong`; refusal after a
sibling success (both rows kept, report passes); dropped observation →
contract rejection; `--timeout-ms 0` child → timeouts recorded, no job
emitted, exit 2; synthetic signal-death → `Crash` classification; unknown
worker case → exit 2. The production binary has no fault-injection flags:
timeouts and identities are ordinary CLI configuration.
