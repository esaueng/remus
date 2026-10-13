# PERF-D01: prune fine quantity-mesh boundary distances

On the imported 160-face holder, preparing immutable boundary segments and
rejecting impossible contacts reduces packaged-WASM volume time by 48.4–48.8%
and the measured kernel pipeline including transfer by 25.5–25.7%. Every
compared arena, display-mesh buffer and feature record remains byte-identical;
volume bits remain identical. Requested linear/angular controls, the quantity
clamp and cache keys are unchanged. Browser rendering and presentation are
outside this experiment.

## Source and artifact identity

Timings compare original runtime source `353c957204e27e41d9403c2ca5db73061c9d3230`
with the reviewed implementation `8a40740dc764f57aa3796012f24d49dce7693135`.
Both canonical local package pairs are 2026.1.58. This matched comparison isolates
the boundary-filter implementation; the harness README's older 61ed862/package 45
investigation is historical context, not this experiment's baseline.

The delivery source `f2d4ddddd34ee0e18602efed8994b7d8ff64d68b` rebases 8a onto
main's automatic package-only refresh `cc1ea99cc2124fd873a8bfe6ebf46a232764181c`.
Only the two package manifests and committed kernel binary differ between 8a
and 414a; runtime source, Cargo files, benchmark and harness are identical.
Current main's two module binaries are byte-identical to the measured original
pair. [Source equivalence](evidence/perf-d01-boundary-clearance/delivery-source-equivalence.json)
and [artifact/build provenance](evidence/perf-d01-boundary-clearance/provenance.json)
retain the actual build heads; rebasing does not relabel the measurements.

| Module | Measured original SHA256 | Measured candidate SHA256 | Raw bytes original → candidate |
|---|---|---|---:|
| Kernel | `368d2eb930828e9bcc265343f71550cf4141b53b2cbfa6b04c632a0a7ebc9c6f` | `ccf8437b0ad95cbda83c6173aab3d6c1508ae42ac6255d27ded3ee1065de2a93` | 9,560,400 → 9,563,178 |
| Translator | `e6e689baea1029087f6c4cfbd9522fa4a7cf30b60d1e1b9f2e0d6c5240ecb0d7` | `67733cd58e704acf863d4ee4bede774a5109e5b2bc058a4c14bb77740b5382c3` | 2,705,941 → 2,708,719 |

Final delivery qualification at 414a builds a canonical local pair 2026.1.59,
then applies the exact publisher repository/homepage fields and passes the
installed-tarball consumer again. All 18 non-manifest package files, including
both modules and both Node wrappers, are byte-identical to the measured v58
candidate. The two manifest contracts differ only in version. This v59 pair
was not timed; [delivery artifact identity](evidence/perf-d01-boundary-clearance/delivery-artifacts.json)
records the separate qualification and actual build head.

Each module grows 2,778 B against fresh 611/main 1f8. The trusted current-main
[size gate](evidence/perf-d01-boundary-clearance/size-gate.md) passes. Generated
packages are local validation artifacts and have been restored; the existing
main publisher performs the consumer refresh after a source merge.

## Attribution before changing implementation

The initial diagnostic used unmodified source `4152fc07` and the then-shipped
2026.1.57 kernel `f2636194857e0cb48d6753fca0729c91f9e3abbb497a5f5f210b85b285f5864a`.
Its nonplanar source file is byte-identical to 611's baseline. A symbol-bearing
optimized module, after removing only its name custom section, matches every
shipping byte. This qualifies function-index mapping to the actual shipping
module; symbol names alone would not establish that mapping.

Two isolated volume CPU profiles contain 3,271 / 3,282 samples. Nonplanar CDT owns
85.3% of samples inclusively; the boundary-scan iterator owns 49.1–49.8%, and
its software fused-multiply-add leaf owns 43.4%. Inspecting the mapped iterator's
code identifies the original point-to-segment distance predicate. CDT insertion
owns 17.1–17.2% inclusively; normals 3.4–3.8% and NURBS evaluation 2.4–2.5% are
smaller. These nested sample shares are not additive. Wall recognition has no
named sampled frame in these profiles.

A separate, temporary counter overlay qualifies five source/edit snapshots
against the uninstrumented package. It observes 96 generic nonplanar CDT faces,
6,658 outer segments and no holes in this holder's generic faces. At edits 5.04
and 5.06, 66,890 / 66,824 grid queries evaluate 9,965,836 / 9,943,924 segment distances.
The worst cylinder faces each scan 194 segments for 5,209 queries (1,010,546
distances); two large NURBS faces have 332 segments. Physical candidate filtering
keeps all 68,533 / 68,467 candidates and evaluates point+normal twice across
filtering and final assembly. These counts identify bounded work to remove;
they do not make the other stages unnecessary.

All diagnostic profiles/counters ran outside the quiet timed comparison,
under other build load. Their wall durations are excluded from performance
claims. The temporary source overlay was restored before implementation and
qualification. [CPU samples](evidence/perf-d01-boundary-clearance/shipping-attribution.json),
[exact name-map check](evidence/perf-d01-boundary-clearance/mapping-qualification.json),
[work counts](evidence/perf-d01-boundary-clearance/work-count-summary.json) and
[counter provenance](evidence/perf-d01-boundary-clearance/work-counter-provenance.json)
retain their distinct source/artifact identities.

## Bounded implementation and correctness controls

`PreparedBoundarySegments` stores each original segment's start, rounded edge
and squared length once per face. It rejects a query only outside conservative
bounds for the **computed nearest point**. Bounding ideal endpoints is unsafe:
for a segment 1e16→1, rounded subtraction/addition makes the original fraction 1
nearest point 0. The regression pins that cancellation witness.

Rounded multiplication/addition are monotone for a clamped finite fraction in
[0,1], so the nearest coordinate lies between start and start + rounded edge.
The degenerate branch uses start only. Bounds expand this hull outward; the
broad-phase radius uses two upward ULPs and verifies its rounded squared norm
exceeds the original clearance. Overflow, nonfinite or unknown bounds/query
ranges retain the complete scan. The narrow division, clamp, nearest-point
arithmetic, square root, threshold and segment/hole order stay unchanged.
This margin does not change the acceptance threshold or deflection.

Seven focused tests pin directed segments at ULP boundaries and many scales,
empty/duplicate/hole loops, unwrapped seams, degeneracy and invalid arithmetic.
A 512-segment interior-query control reduces evaluated distances 524,288→0.
The actual holder control requires more than 9 million original distances and
more than 20× fewer evaluated distances, with identical raw Float64 quantity
mesh positions/normals/indices and fresh volume bits at both width edits.

Complete mesh comparisons cover all carriers, required ridge blends, holes,
cavities, converted periodic NURBS, seam-crossing bores, two deflections,
grouped face offsets and cold/warm quantity/display replay. The optional
filleted-box fixture can refuse construction; it is not guaranteed fillet
coverage. Existing independent mesh/geometry oracles remain unchanged.

## Quiet packaged-WASM measurement

The maintained [comparison runner](../../scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py)
and [worker](../../scripts/perf/filleted-face-edit/wasm_edit_bench.cjs) run eight
fresh Node processes: original/candidate × two schedules × two rounds, reversing
variant launch order in the second round. Each process imports once, warms
source queries, performs two warmup edits and retains six measured edits.
Each edit restores the source into a new kernel while module/JIT caches and
production cache policy persist. These are first post-edit quantity readings
in warmed processes, not process/JIT-cold or universally cache-cold reads.

Node 24.14.0/Linux/x86_64 on a Ryzen 9 5900XT, pinned to logical CPU 0, with builds
and other heavy jobs held. The quiet provenance records UTC boundaries. The
unchanged policy requests display 0.02 / 0.35 and measurement 0.08; volume retains
its exact bbox-derived clamp and deflection key bits. Timed stages include
movement, validation, bounding box, grouped mesh, bulk transfer, volume and
complete feature recognition. Import, output diagnostics and artifact writes
are outside these stage intervals.

There are 12 retained edits per variant/schedule, across only two processes.
The table gives median [min–max] milliseconds, descriptive of this small,
correlated workload sample. P95 at this sample count is the observed maximum,
not a calibrated production-tail estimate.

| Schedule | Stage | Original median [min–max] ms | Candidate median [min–max] ms |
|---|---|---:|---:|
| geometry-first | Volume | 492.39 [489.62–499.82] | 254.23 [250.70–260.60] |
| geometry-first | Recognition | 346.28 [344.40–351.25] | 348.52 [346.94–355.99] |
| geometry-first | Pipeline including transfer | 934.53 [914.76–972.22] | 696.54 [679.51–727.95] |
| volume-first | Volume | 495.58 [490.38–510.58] | 253.75 [249.27–258.20] |
| volume-first | Recognition | 347.74 [345.59–351.84] | 346.88 [346.27–351.71] |
| volume-first | Pipeline including transfer | 937.74 [918.57–987.09] | 697.11 [677.53–727.88] |

Geometry-first geometry+transfer readiness is 92.941→93.512 ms; no stable display
change is established. The small mesh-time movement (~0.2 ms), and the
volume-first transfer median ratio, do not support acceleration claims for
unchanged display/transfer code. Volume-first scheduled geometry+transfer
readiness includes the earlier volume read: 588.683→350.472 ms. This prefix
excludes restoration, GPU rendering and actual presentation. Restored pipeline
medians are 941.434→703.491 ms and 944.730→704.130 ms respectively.

All 36 paired snapshots pass strict comparison: 4 source + 8 warmup + 24 measured
pairs, rather than 36 timed samples. All 180 arena/display-buffer byte pairs and
360 recorded artifact digest claims agree; all 36 volume Float64 bits, full
feature JSON hashes, counts, validity and watertightness checks agree.
Float32 position/normal comparisons permit no near-only qualification here.
Native tests additionally compare raw Float64 quantity mesh fields.

[Every raw WASM timing sample](evidence/perf-d01-boundary-clearance/wasm-raw-samples.json),
[all paired qualification records/distributions](evidence/perf-d01-boundary-clearance/wasm-comparison.json)
and [clean/input checks](evidence/perf-d01-boundary-clearance/quiet-source-input-verification.json)
are committed as portable evidence; full geometry buffers are represented by
original SHA256 digests and reproducible fixture/harness identities.

## Native workload check

The committed [Criterion benchmark](../../crates/operations/benches/perf_d01_quantity.rs)
uses the same imported holder after prepared width edits +5 / +5.06. Import/edit
setup is excluded. Every timed iteration clears the thread volume memo and
face mesh cache, with that cache disabled, then requests volume 0.08 including
the complete fresh quantity mesh workflow. The process is warmed; other
persistent caches are not globally described as cold.

The original production source 611 is compiled with the same benchmark-only
overlay as 8a. The profiling configuration and benchmark SHA match exactly.
Each width has 20 flat statistical samples, one iteration each, in one original
process then one candidate process. The 1 s warmup/3 s measurement target is
unchanged; Criterion extends collection to fit 20 samples (~4.9 s original,
~3.7–3.8 s candidate). Raw warnings/outliers are retained.

| Width edit | Original Criterion estimate ms | Candidate estimate ms | Reported change 95% CI |
|---|---:|---:|---:|
|+5|245.881|186.616|−24.724% to −23.545%|
|+5.06|249.189|186.059|−25.815% to −24.871%|

These confidence intervals are conditional on this native session; native
numbers do not establish a WASM, browser or cross-kernel comparison. The log's
p=0.00 is formatted rounding. All 80 raw sample times/iteration counts and
estimates are in [native raw samples](evidence/perf-d01-boundary-clearance/native-raw-samples.json);
[verbatim native log payloads](evidence/perf-d01-boundary-clearance/native-logs.json)
preserve the complete original/candidate text, terminal newlines and SHA256
digests through JSON encoding. Build provenance fields
`timingExecuted:false` were captured before these later measurement runs.

## Validation and reproduction

At immutable 8a: 7,973 native tests pass, 47 committed skips, including all 54
required-Vulkan renderer tests; docs 9 pass / 2 ignored; 52 semantic census rows
match; 43 no-I/O contracts pass. Full all-feature/all-target Clippy, fmt, module
map, boundaries, Taplo, machete and privacy/project hooks pass. Canonical
local v58 paired build, runtime regressions and installed-tarball consumer pass.

For reproduction, require clean tracked/staged inputs (`git diff HEAD`) and
empty `git status --porcelain` before timing. Build original 611 and the public
delivery source 414a (or the reviewed PR head with the same runtime source).
The measured 8a commit is preserved locally and is not required for public
reproduction. Its v58 timing identity stays recorded; delivery v59 metadata
differs while runtime source equivalence is checked separately. Preserve
binaries/wrappers/manifests and use the
existing runner with the exact arguments:

```sh
python3 scripts/perf/filleted-face-edit/run_wasm_edit_comparison.py \
  --old-kernel <original-pair>/crates/wasm/pkg \
  --old-io <original-pair>/crates/wasm-io/pkg \
  --new-kernel <candidate-pair>/crates/wasm/pkg \
  --new-io <candidate-pair>/crates/wasm-io/pkg \
  --warmup 2 --samples 6 --rounds 2 --order both --cpu auto --out <new-directory>
```

The public raw evidence retains the exact expanded command and runtime hashes.
Upgrade every reported near comparison to byte identity for this experiment.
For native reproduction, register/copy the identical committed benchmark into
original 611 without changing production source; the identical benchmark is
available at delivered 414a. Compile both using
`cargo bench --offline --locked --profile profiling -p remus-operations --bench perf_d01_quantity --no-run`,
then run the saved binaries on the same pinned CPU with a shared
`CRITERION_HOME`, original `--bench --noplot --save-baseline before` then
candidate `--bench --noplot --baseline before`.

The diagnostic [profile client](evidence/perf-d01-boundary-clearance/profile_volume.cjs)
and [counter client](evidence/perf-d01-boundary-clearance/count_volume.cjs)
accept the maintained worker's package/fixture/output arguments. Always pass
`--timing-status contended-smoke-excluded`; their durations are diagnostic only.
The counter [source overlay](evidence/perf-d01-boundary-clearance/diagnostic-overlay.json)
is JSON-encoded with its original SHA256. Extract its `patch` string verbatim
to reproduce the original bytes; it applies to 4152, not the delivered
implementation. Retain exact shipping-byte
identity before interpreting a name-derived function index. Sampled CPU data
and work counters qualify this holder; allocation profiles and broader
adversarial workload coverage remain open under PERF-M07.

## Next bounded performance work

Recognition remains 346.9–348.5 ms and now occupies about half this pipeline.
Profile graph construction's per-face area and relation work before choosing
its next optimization. Shipped prepared contexts/candidate setup and face-mesh
caching are already present. This result does not justify tolerance bucketing,
coarser quantity policy, broad tessellation-plan reuse, general history-driven
property recombination or a browser/UI latency promise. PERF-D01/V02 remain
partial beyond this measured boundary-distance acceleration.
