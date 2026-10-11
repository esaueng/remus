# Face-drag qualification — 2026-10-11

Final local qualification passed for source
`b7d4c68f7468e8e9a5e51ed5f128fdde698699ec` and paired WASM packages
`2026.1.52`. Across 36 old/new snapshot pairs, reachable arenas, volume
readings, feature records and display mesh arrays are exact. In the
geometry-first schedule, the median sum of kernel work through display mesh
transfer decreases from 123.6 to 92.7 ms. The complete kernel pipeline decreases
from 1108.3 to 969.0 ms. Volume medians change by less than 1% in either schedule.

[The compact JSON evidence](evidence/face-drag-2026-10-11.json) retains every
native and WASM stage record, including warmups, package/module/wrapper/fixture
hashes, runtime metadata and output comparisons. Machine-specific paths are
replaced by portable location identifiers. Binary and arena snapshot payloads
are not embedded; their original filenames and hashes remain in the records.

## Source and package identity

| Variant | Source | Matched kernel / translator packages | Scope |
| --- | --- | --- | --- |
| Baseline | `8af9a738` | Locally compiled pair `2026.1.46` | Frozen source baseline for this investigation. |
| Merged optimization base | `36d22c036d3ba0551e686ec4ef0a37d78fa78a37`, merged PR #982 | Committed pair `2026.1.51` | Includes the existing face-edit optimizations, tessellation admission refinement and promotion only after successful display replay. |
| Follow-up | `b7d4c68f7468e8e9a5e51ed5f128fdde698699ec` | Built and packaged pair `2026.1.52` | Adds the finite-reference guard, independent oracles and reproducible native probe. |

The baseline-to-follow-up timing comparison includes the changes merged in
PR #982. It cannot isolate the runtime cost of the follow-up's 21 added
production lines. The raw evidence identifies both package modules and Node
wrappers by hash and records the source provenance.
PR #982 was squash-merged as `aab51010`; its tree is identical to the recorded
optimization base `0f925b8d`.

| WASM module | Baseline `2026.1.46`, bytes | Follow-up `2026.1.52`, bytes | Change |
| --- | ---: | ---: | ---: |
| Kernel | 10,367,007 | 10,368,772 | +1,765 |
| Translator | 3,050,094 | 3,051,698 | +1,604 |

Both variants ran on Linux 7.1.5 x86_64 with an AMD Ryzen 9 5900XT 16-Core
Processor, pinned to CPU 0. WASM used Node 24.14.0. Final timing processes ran
serially after competing compilation finished. Small dependency bookkeeping
overlapped an excluded early native warmup only. Initial CPU-contended
exploratory trials are discarded and absent from this evidence.

## Workload and limits of the reproduction

The public input is
`crates/io/tests/data/shapr3d_hammer_holder.step`: one solid with 160 faces,
including 42 NURBS support surfaces, and approximate dimensions
74 × 53 × 58 mm. The reviewed screenshot also showed 160 faces, but its body was
144 × 87 × 70 mm and the selected planar face had area 1469.93 mm². The exact
edited document was not available. Equal face counts do not establish that
these are the same geometry or editing history.

The probes select the public fixture's outward +X planar arm face at x = 48 mm.
The maintained WASM harness checks its area against approximately 1045.93 mm².
They exercise explicit blend-preserving `move_faces` / `moveFacesJournaled`.
Results do not automatically describe the alternate `pushPullFace` operation
or the screenshot's selected face.

This is a kernel-call workload. It excludes application scheduling, worker
queues, React updates, mesh installation, GPU submission and browser
input-to-visible-frame latency. A lower kernel duration alone does not establish
a real-time browser interaction.

## Implemented scope

The merged optimization base already includes the fixed-rule volume-only
orientation path, first-order NURBS solve reuse, the exact scalar memo for
standalone NURBS areas, and display priority in the bounded face-mesh cache.
Their contracts and qualification history are described in
[Filleted face-edit responsiveness](perf-face-edit-responsiveness.md). This
follow-up does not introduce a second volume integration implementation.

The additional production change validates a requested integration reference
before either a direct cache lookup or a thread-cache lookup. A warm affine
volume entry could previously return a successful NaN reading for a non-finite
reference, whereas fresh integration rejected it. Cold, warm and disabled
caches now preserve that existing input-validation failure. Rejected requests
leave cache statistics unchanged.

No new geometric approximation, quadrature rule, tolerance, default quality
setting or validation exemption is introduced. Full mass-property requests
retain their existing path; orientation still uses its existing volume,
physical-area and vector-area arithmetic and original sample policy.

## Native probe

The maintained example is
`crates/io/examples/face_drag_probe.rs`:

```sh
RAYON_NUM_THREADS=1 cargo run --locked --profile profiling \
  -p remus-io --example face_drag_probe
```

It imports the fixture once and emits four JSON sample records, with the first
marked as warmup. Each edit starts from a clone of the imported topology; cloning
is outside the movement interval. It times source strict validation,
`move_faces` by +1 mm, result strict validation and display tessellation at
linear/angular tolerances 0.0148/0.06. Volume is checked at deflection 0.08
outside those timing intervals. Every result must remain valid, retain 160
faces, have zero boundary and non-manifold mesh edges, and have finite positive
volume.

This example does not construct `BrepKernel` or enable its opt-in thread caches.
Repeated native validation therefore remains an uncached integration workload,
while reusable scratch within an integration can still apply. It provides
phase diagnosis and regression checks, not a matched browser benchmark. Two
fresh processes per variant ran in old/new then new/old order, retaining three
samples from each after its warmup: six samples per variant. Both native
binaries used the `profiling` profile, release optimization level 3 with LTO
disabled, and `RAYON_NUM_THREADS=1`.

The retained native candidate was built at
`13bc2f58a161d9d42d3790a64f4ccdb6205ade81`. The later display-cache promotion
correction affects only an opt-in cache that this probe leaves disabled; these
native records retain their original source provenance.

| Native stage, milliseconds | Baseline median [range] | Follow-up median [range] |
| --- | ---: | ---: |
| Source strict validation | 388.5 [387.5–390.2] | 359.0 [357.9–362.7] |
| Blend-preserving face move | 537.8 [537.5–538.3] | 501.4 [500.7–502.4] |
| Result strict validation | 388.3 [387.4–390.7] | 358.8 [357.8–359.6] |
| Display tessellation | 332.8 [331.6–336.7] | 319.0 [316.5–320.5] |

## Maintained WASM comparison

The [paired-package harness](../../scripts/perf/filleted-face-edit/README.md)
used immutable old/new kernel and translator directories. It ran two rounds
per schedule with alternating old/new process order, two warmup edits and six
retained edits per process. Eight processes supply 12 retained edits per
variant per schedule. The command, runtime, affinity, ordering and package and
fixture hashes are recorded in the raw evidence.

The harness imports once per process, warms source queries, and creates each
edit from an exact serialized arena document. Separate kernel instances share
the module's enabled, bounded thread caches. Both geometry-first and
volume-first schedules operate at distances `5 + iteration * 0.02` mm; display
tolerances are 0.02/0.35 and volume/recognition deflection is 0.08. This differs
from the native probe's distance, display policy and cache history; native and
WASM timings must be reported as separate workloads.

Movement, strict validation, bounding box, grouped display tessellation, bulk
mesh transfer, volume and complete feature recognition are timed separately.
The kernel pipeline excludes bulk getters; the pipeline with transfer includes
them; the restored pipeline also includes kernel construction and source
restoration. Import and source-query warmup are separate measurements. Tables
show median and observed minimum–maximum in milliseconds, without extrapolating
percentiles from these small samples.

| Schedule | Stage, milliseconds | Baseline median [range] | Follow-up median [range] |
| --- | --- | ---: | ---: |
| Geometry first | Face move | 60.1 [51.2–94.7] | 56.5 [48.7–86.6] |
| Geometry first | Strict validation | 1.50 [1.40–1.95] | 1.46 [1.39–4.60] |
| Geometry first | Bounding box | 0.77 [0.75–3.46] | 0.77 [0.75–5.14] |
| Geometry first | Display tessellation | 47.3 [29.2–223.3] | 29.8 [29.2–63.5] |
| Geometry first | Bulk mesh transfer | 0.15 [0.13–0.27] | 0.14 [0.13–0.17] |
| Geometry first | Display mesh and transfer | 47.5 [29.3–223.4] | 30.0 [29.3–63.6] |
| Geometry first | Volume | 535.2 [528.5–544.9] | 530.3 [524.1–536.4] |
| Geometry first | Feature recognition | 454.1 [448.2–458.8] | 347.3 [345.7–358.2] |
| Geometry first | Kernel pipeline | 1108.3 [1062.2–1289.6] | 969.0 [953.6–1009.4] |
| Geometry first | Pipeline with transfer | 1108.4 [1062.3–1289.9] | 969.1 [953.8–1009.6] |
| Geometry first | Restored pipeline | 1114.3 [1066.5–1295.1] | 977.5 [961.8–1014.9] |
| Volume first | Face move | 60.1 [51.5–92.7] | 56.6 [48.7–91.4] |
| Volume first | Strict validation | 1.80 [1.39–1.90] | 1.67 [1.37–1.89] |
| Volume first | Bounding box | 0.76 [0.75–2.29] | 0.76 [0.75–2.32] |
| Volume first | Display tessellation | 30.9 [28.9–255.3] | 30.0 [28.6–63.8] |
| Volume first | Bulk mesh transfer | 0.16 [0.15–0.23] | 0.16 [0.15–2.59] |
| Volume first | Display mesh and transfer | 31.1 [29.0–255.5] | 30.2 [28.8–64.0] |
| Volume first | Volume | 531.9 [526.1–544.5] | 528.0 [522.3–547.1] |
| Volume first | Feature recognition | 450.5 [446.2–455.4] | 347.3 [345.5–352.0] |
| Volume first | Kernel pipeline | 1082.0 [1063.7–1335.3] | 969.3 [952.6–1023.2] |
| Volume first | Pipeline with transfer | 1082.2 [1064.0–1335.5] | 971.4 [952.8–1023.4] |
| Volume first | Restored pipeline | 1088.6 [1068.1–1339.6] | 978.7 [956.9–1028.8] |

The geometry-first subtotal through display mesh transfer is calculated per
sample as movement + validation + bounding box + display tessellation + bulk
transfer, then summarized. Its median [range] is 123.6 [85.2–294.8] ms before
and 92.7 [80.9–124.8] ms after. This sum excludes kernel construction and source
restoration; it is kernel/transfer work before quantity analysis, not measured
browser input-to-visible latency. The volume-first schedule performs volume
before display, so the same geometry subtotal would omit a preceding wait.

Recognition medians improve from 454.1 to 347.3 ms in geometry-first order and
450.5 to 347.3 ms in volume-first order. Volume remains the largest stage: its
medians change from 535.2 to 530.3 ms and 531.9 to 528.0 ms, respectively, each
less than 1%. Its volume-first maximum increases from 544.5 to 547.1 ms. Short
movement and validation stages still have variable tails. Warm WASM validation
often reuses the integral cache; its roughly 1–2 ms medians must not be compared
to uncached native validation as a platform scaling ratio.

## Output qualification

Every compared snapshot must validate, have a watertight indexed display mesh,
increase volume and move the selected cap to the requested x coordinate. The
maintained runner requires exact Float64 volume bits, complete feature-record
equality, exact indices/grouping/counts and analytic radii. It separately reports
reachable-arena byte identity and Float32 position/normal byte identity, alongside
bounded numeric comparisons. In this run, every exact flag passes; qualification
does not depend on the bounded comparison allowance. The maximum position and
normal component differences are both zero.

| Final old/new comparison | Outcome |
| --- | --- |
| All source, warmup and measured snapshot pairs qualify | 36/36 |
| Reachable Float64 arena documents byte-exact | 36/36 |
| Float64 volume bits exact | 36/36 |
| Complete feature records exact | 36/36 |
| Display positions and normals byte-exact / maximum difference | 36/36 / 0 |
| Indices, grouping, face/surface counts and analytic radii exact | 36/36 |
| All edits valid and display meshes watertight | 36/36 pairs |

## Independent correctness checks and delivery gates

The added NURBS oracle integrates the polynomial pillow
`z = 9 L u(1-u)v(1-v)`, `x = u`, `y = v`: its signed divergence contribution
about reference R is `L/4 - R_z/3`, with sign reversed for reversed faces.
It covers multiple lifts, references and Gauss orders independently of the
implementation's full-property integrator.

Independent analytic checks use a quarter-cylinder, hemisphere and cone with
origin-based signed volume terms 2π, 18π and zero. Additional comparisons pin
volume, physical area and vector-area bits against full integration over
multiple orders, references and face reversals. The non-finite-reference
regression covers NaN, positive infinity and negative infinity across cold,
warm, disabled, direct and thread-cache paths.

Local `cargo test -p remus-check` validation passed 295 tests
(217 unit and 78 integration), and `cargo clippy -p remus-check --all-targets
-- -D warnings` passed. The I/O example also passed clippy. Formatting, layer
boundaries, deterministic hashing and documentation-path checks passed.
The display-promotion regression also passed after integration. The normal paired
`cargo xtask wasm-build`, package runtime checks and installed-tarball consumer
regressions passed for `2026.1.52`. These are local source and artifact checks;
hosted CI and deployment are not established by this report.
