# PERF-O06: recognition graph attribution

The next performance target on the 160-face holder is quadrant classification's
ray intersections. Fresh source `32d3994860fab2de13d6c9f3caaab836bab40db2`
measures recognition at **347.6 ms** after a width edit. Two sampled profiles
attribute about **79%** of recognition samples to ray crossings, led by
line–NURBS intersections and the torus solver. Face-area work accounts for about
14%; prepared-context setup is under 1%. This change ships profiling tools and
evidence, with no runtime optimization, tolerance or cache-policy change.

## Source and workload

The source includes the F5/F12 repair and the PERF-D01 quantity-mesh repair.
The committed version 2026.1.58 packages at this revision came from older runtime
`611a9116`; they were not used. A fresh canonical paired build produced local
version **2026.1.59**, then passed the package and installed-consumer suites,
including a second installed-consumer run with the publisher's stable
homepage/repository fields. During delivery, automatic refresh `5438f0ef`
(#1047) published version 59 from source 32. All 18 non-manifest package files
are byte-identical to the measured pair; both manifests decode to exactly the
same metadata (UTF-8 punctuation versus equivalent JSON escapes). This PR uses
that artifact-only base; measurements remain attributed to source 32. No
generated packages are included.

| artifact | SHA256 | bytes |
|---|---|---:|
| fresh kernel | `4177922b20d6dc4ace45a7a986ba06ded1348c390f80533d0559579864ba4816` | 9,590,279 |
| fresh translator | `67733cd58e704acf863d4ee4bede774a5109e5b2bc058a4c14bb77740b5382c3` | 2,708,719 |

The committed STEP fixture is `shapr3d_hammer_holder.step`, SHA256
`2403b4c9868cbb56292a09ec2f2d37baca16cc4b789d11e97ff8df3f1e46bef8`:
52 planes, 42 cylinders, 42 NURBS, 8 spheres, 14 tori and 2 cones; 386 edges.
The packaged workflow imports once, warms source queries, then restores a new
kernel and moves the +X cap by `5 + iteration * .02`. Display uses .02/.35 and
measurement/recognition uses .08. Production memo and display-cache defaults
remain enabled; module/JIT state persists within a process. These are warmed
processes with fresh restored edit bodies, not process-cold latency samples.

## Quiet untraced measurements

A 41-second serial window ran on Linux, Node v24.14.0, Ryzen 9 5900XT, pinned
to allowed CPU 0. Four fresh WASM processes covered two schedules, with two
warmup edits and six retained edits per process: **12 retained edits per
schedule**, across two processes each. The native and WASM executables,
packages, fixture and worker were hashed before/after; both tracked/index state
checks were clean. Neither Inspector nor kernel tracing ran in these calls.

| packaged stage | geometry first, median [min–max] ms | volume first, median [min–max] ms |
|---|---:|---:|
| recognition | 347.606 [344.962–351.262] | 347.060 [345.748–351.818] |
| kernel pipeline plus bulk mesh transfer | 693.827 [676.845–724.096] | 697.489 [676.778–724.806] |

The pipeline includes move, validation, bounding box, display tessellation,
volume, recognition and mesh getters. It excludes source restoration, browser
orchestration, GPU work and presentation. This is a baseline, not a speedup
comparison. Twelve correlated edits from two processes do not establish a
production tail-latency distribution.

Two separate native processes ran the unmodified `recognition_perf` example on
the imported source body at .08. Native default memos were disabled; there was
no edit/display pipeline. Each API ran eight times per process, with the first
two excluded: 12 retained calls per API. The example prints to 0.1 ms.
Recognition measured **191.3 [190.3–193.5] ms**; whole-solid edge relations
**113.1 [112.7–117.6] ms**. These different native/WASM workloads do not establish
a backend speed ratio. Complete native feature and relation dumps match between
processes.

## Shipping-module attribution

Two additional retained recognition calls were sampled with Inspector at
100 microseconds while other work was running. Their durations are excluded
from latency claims. A named wasm-opt 117 `-Oz` build, after removing only its
custom `name` section, matches the actual fresh kernel byte for byte. Function
indices are read from that qualified name section; both profiles have **zero
unmapped WASM functions or affected samples**.

| inclusive frame/category | sample counts, calls 1 / 2 | sample shares |
|---|---:|---:|
| edge relations | 2,012 / 2,065 | 84.2–84.6% |
| prepared ray crossings | 1,892 / 1,929 | 79.1–79.2% |
| line–NURBS intersections | 1,118 / 1,142 | 46.8% |
| torus ray solver | 614 / 625 | 25.6–25.7% |
| per-face areas | 341 / 350 | 14.3% |
| normal sampling | 49 / 66 | 2.1–2.7% |
| prepared-context setup | 15 / 15 | 0.6% |
| exact emulated FMA, nested in the above work | 1,300 / 1,318 | 54.0–54.4% |

Denominators are 2,390 and 2,440 total samples. Categories are nested and must
not be added. The area branch is predominantly fixed-order quadrature; existing
NURBS scalar-area reuse already handles most faces.

A separate temporary observation-only overlay counted work and retained each
face's area bits, boundary/pole sizes, memo state and phase events. It used Rust
release and wasm-bindgen without wasm-opt; it is **not** the shipping module,
and its phase durations are diagnostic only. Both edited calls report:

| work | count per recognition |
|---|---:|
| prepared solids / faces | 1 / 160 |
| unique edges / two-face relations / tangent relations | 386 / 382 / 179 |
| normal reads / NURBS normal projections | 2,292 / 504 |
| quadrant classification calls / rays | 771 / 1,538 |
| candidate-face ray calls | 14,549 |
| candidate NURBS / torus ray calls | 1,536 / 5,804 |
| NURBS boundary projections / support-hull rejects | 450 / 31,058 |
| NURBS area memo hits / misses / evictions | 40 / 2 / 0 |
| fixed-integral memo hits / misses / evictions | 50 / 21 / 0 |

The 21 fixed-integral misses comprise 13 cylinders, 3 planes, 3 tori and
2 spheres. Forty-seven additional planar area requests bypass that memo. Early
boundary/ambiguity returns are retained; 771 quadrant calls is not an assumption
of four calls for every non-tangent edge.

## Qualification and next work

Each initial diagnostic run matches its untraced shipping control on five
snapshots: source, two warmups and two retained edits. Both sets preserve all
25 arena/display-buffer artifact pairs, complete feature JSON and Float64 volume
bits. The maintained CPU/count/default modes add 15 snapshot comparisons and
75 byte-exact artifact pairs. Quiet repeats preserve 18 snapshot pairs and
90 artifact pairs, including exact features and volume bits. Every saved display
mesh validates as watertight; cap position, analytic radii and all 160 faces are
qualified. All temporary source instrumentation was restored.

The measured next step is to attribute the **line–NURBS Newton and torus-root
work inside quadrant rays**: count refinement starts/iterations, repeated
immutable evaluations and trim-rejected intersections before choosing a narrow
change. Any proposed reuse or conservative rejection must preserve complete
roots, quadrant verdicts, feature records, area bits and geometry/quantity
contracts on periodic, holed, grazing, translated and ambiguous controls.
Replacing exact FMA or changing solver tolerances is outside this evidence.

Fixed-order area work is the secondary target. Further area-cache work must
preserve fresh per-face bits; the current exact NURBS-area memo already answers
40/42 requests. Rebuilding prepared contexts, hoisting tiny candidate matching,
or prioritizing planar candidate buckets is not supported by this fixture.
Allocation profiling, broader workloads and actual browser presentation remain
open.

[Portable evidence](evidence/perf-o06-recognition-graph/provenance.json) retains
all raw WASM timings/features/digests, native readings and exact dumps, sampled
stacks, counted per-face observations, the exact diagnostic patch and qualification
records. [Harness instructions](../../scripts/perf/recognition-graph/README.md)
describe reproduction and the strict separation of tracing from timing. The
timed worker is the public source-32 version; the maintained worker adds an
opt-in observer and is separately qualified without changing its default outputs.
