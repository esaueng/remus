# Imported video workflow acceptance repro

Tier: native. Test and qualification tooling only; no kernel implementation,
WASM package, tolerance or public API changes.

`crates/io/tests/video_workflow_baseline.rs` carries forward the diagnostic
harness from `eb32a2ee`, with assertions for the target geometry. It is an
ignored ready-repro: known-open failures are labelled `XFAIL (open)` in the
table but still make the acceptance run fail. Normal test discovery compiles
the suite and leaves it ignored. The W3 spike is report-only and returns success
when evidence collection completes, independently of geometry acceptance.

## Reproduction

On the Windows GNU toolchain:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run-video-workflow-baseline.ps1 -Mode all -OutputDirectory target/video-workflow-acceptance
```

Other useful arguments: `-Mode acceptance` (default, includes cumulative),
`-Mode cumulative`, `-Mode spike`, `-Case A1_rim_shoulder`, `-Case A5`,
`-CompileOnly`, and `-Lint`. A single acceptance case asserts too; a spike case
prints its measured violations without asserting construction support.

With a normal native linker:

```sh
cargo test -p remus-io --profile ci-test --test video_workflow_baseline -- --ignored --nocapture
REMUS_VIDEO_MODE=spike cargo test -p remus-io --profile ci-test --test video_workflow_baseline -- --ignored --nocapture
```

Fixtures default to `crates/io/tests/data`; `REMUS_VIDEO_FIXTURES` can override
them. `REMUS_VIDEO_OUTPUT` selects an evidence directory. The Windows runner
uses the installed Rust LLD and an import-library adapter, with a temporary
test-only manifest under `target/` to avoid unused benchmark dependencies that
need a C compiler. Registry dependencies remain pinned by the repository lock;
the runner copies the workspace lint policy. It changes no tracked manifest.

## Fixtures and assertions

Both supplied STEP files are retained verbatim. The sharp file is 12,030 bytes;
W-Fillet is 604,137 bytes, smaller than the existing 946,386-byte STEP fixture.
No applicable fixture-size ceiling was found. STEP author fields are empty.
The STEP files retain exact planes, cylinders and tori; the two NURBS corners
in W-Fillet are original sampled geometry. They are not a reader fallback.

A0 rebuilds the five-edge R4 network from the sharp STEP, then adds the R1 cap
rim. Its census and volume must match W-Fillet plus that same R1 rim. A2-A8
isolated rows start from W-Fillet plus R1. Historical no-rim A1 probes and the
raw reference import remain explicitly separate rows for baseline comparison.

Each candidate records and checks:

- Both B-Rep validators, closed shells, zero free/non-manifold B-Rep edges.
- Two explicit meshes at diagonal times `1e-5` and `5e-6`, both finer than the
  volume clamp: zero welded boundary/non-manifold edges.
- API versus independently summed mesh volume at each deflection, and mesh
  convergence, within `1e-4` relative. Tetrahedra use a body-local origin.
- Analytic census, bounded face count, surviving tori, and requested analytic
  cylinder/torus radii. Input census and volumes are separately qualified.
- Ray-classification probes, including points relative to the selected cap
  plane rather than global z-max, and inside/outside an edited boss wall.
- Exact source document preservation, typed errors, and unchanged live arena
  entity/pcurve counts on refusal. Retired handle slots and mutation ticks may
  grow by the transaction contract; they are also recorded, not hidden.
- Preview preparation/call/cancel timing and exact document equality after
  cancellation. This is native evidence, not browser latency or app undo proof.

The coordinator runs four isolated cases at a time, each with a three-minute
limit. `ACCEPTANCE.md` contains the whole pass/fail table even when assertions
fail; JSON and logs retain all intermediate results, engines, typed errors,
corner diagnostics, inventories and timing. Use a new output directory for
independently attributable runs. Timings include contention between cases.

## Cumulative scope

The rows follow the corrected spec of the round-2 application run. The
cumulative case starts at A0 plus R1 and runs the application's order on one
evolving body: cap z61.2 then z26 with sampled permission (256 stations), A2
on the convex band along x = 0 (R4 -> 1.9), A3 R1 on the x = 0 / y = 18
vertical edge, A4's 0.7 free-end chamfer (the single edge, a typed refusal
naming its tangent chain, then the chain), the A5 negative-radius refusal,
A6's 0.8 bottom chamfer from its seed's tangent chain, and A7 Ø14.6 -> 17 ->
(21.1 preview, discarded) -> 14.7. The concave seam R4 -> 2 is skipped: no
construction of it exists under the R4 shoulder (W3), so its isolated rows
stay open. The case stops on the first refused or geometrically invalid
prerequisite and marks later steps blocked; it never substitutes an isolated
reference body.

Isolated rows: `A1_quality_short` lowers the rimmed cap to z26 and
`A1_quality_shoulder` takes it to z61.2 and then z26, both with sampled
permission. The exact-only `move_faces` cap rows (`A1_short`, `A1_sequence`,
`A1_shoulder`, `A1_rim_*`) are typed refusals by contract when they shorten
into the shoulder. `A2_convex_*` resize to R1.9, `A2_sequence` drags the x = 0
band R4 -> 1.5 -> 1.9, and `A3` is A2 followed by the corrected vertical edge
(the four proxy rows are gone).

The same order, with every oracle and the twelve-edge A6 outline, is the CI
test `crates/operations/tests/video_parity_full_order.rs`.

## Sharp-body rebuild spike

The matrix covers the sharp body, cap z23/z26/z30/z61.2, and sharp boss diameters
14.6/17/21.1/14.7. Each starts fresh, edits through existing exact APIs, and
then attempts all R4; convex R1.5/R1.9 with seam/shoulder R4; seam R2 with
shoulder/convex R4; and both combinations. Every completed network also gets
an R1 cap rim. R2 seam cases run both seam-first and shoulder-first orders.

For diameters, retain the convenience resize refusal, then try exact carrier
replacement and exact Boolean growth of the actual imported solid. None of
these probes permits an approximate Boolean fallback or replaces the source
with a native proxy. The Boolean target's volume is independently specified
as `12960 + 30*pi*r^2`. A refusal in an earlier optional edit route is allowed
only for the spike; source/arena preservation still must pass, and failure to
produce the sharp target prevents testing its blend set. W2 A7 always tests the
original convenience resize API.

The public result reports the engine. Existing debug diagnostics identify the
observed equal-radius corner or general ridge-chain family (the latter is the
family exercised by `fillet_into_seam_band`). Diagnostics may include refused
earlier cascade attempts; they are not a new stable public corner-family API.

## Acceptance results with the joined-cap group route (W14)

Measured on the W14 branch (the local integration of #1572 and #1574-#1585
plus this change) under WSL, ci-test profile. 32 of 34 rows pass; the two
concave R2 seam rows remain the open XFAIL (no construction under the R4
shoulder). The cumulative row passes when run alone and in the full table.

| Step (cumulative, one body) | Result | Census (plane/cyl/cone/torus/nurbs) | API volume |
|---|---|---|---|
| A0 + R1 rim | built | 7/5/0/2/2 | 16339.8724 |
| A1 z61.2 (sampled permission) | built | 7/5/0/2/2 | 19189.9235 |
| A1 z26 (sampled cap join) | built | 7/5/0/3/4 | 15207.7253 |
| A2 x = 0 band R4 -> 1.9 | built (cap lifted, edited, lowered) | 7/5/0/4/4 | 15241.6912 |
| A3 x = 0 / y = 18 edge R1 | built | 7/7/0/4/5 | 15230.3409 |
| A4 single free-end edge | typed refusal `chamfer-tangent-chain-required`, names the five-edge chain | unchanged | unchanged |
| A4 chain 0.7 | built | 10/7/2/4/5 | 15214.8760 |
| A5 R -1 | typed `invalid-input` refusal | unchanged | unchanged |
| A6 seed chain 0.8 | built | 13/7/6/4/5 | 15185.9944 |
| A7 Ø14.6 / 17 / 21.1 (discarded) / 14.7 | built | 13/7/6/4/5 each | 16290.2523 / 17496.6934 / 19974.0237 / 16336.9303 |

Every built row passes both validators, the dual-deflection watertight mesh
checks, API/mesh volume agreement and the feature probes. The sampled faces
are the two original corner patches, A1's two rim bands and A3's lofted turn;
A2's walked transition replaces one corner patch.

## Acceptance results at kernel f4b5d747

| Step | Current result | Evidence |
|---|---|---|
| A0, plus R1 rim | PASS | Rebuild matches imported reference census and volume; full geometry checks pass |
| A1 lengthen to z61.2 | PASS | Both no-rim and R1-rim rows pass |
| A1 shorten to z23 | FAIL | Typed, unchanged-body refusal, including sampled-quality entry point |
| A1 shorten to z26 | FAIL | R1-rim move returns API success but has 12 mesh boundary edges at both deflections and 2.3817% API/mesh volume disagreement; quality API returns the same candidate as `Exact` |
| A2 convex/concave | FAIL | Curved corner termination / open support-contact refusals |
| A3 | PARTIAL | Two isolated free-end proxies pass; untouched boss seams are tangent and filtered; post-A2 video selection remains unqualified |
| A4 | PASS, isolated | R1.5/R3.7 fillets and 0.7/5 chamfers pass |
| A5 | PASS | Negative radius is typed `InvalidInput`; source and live arena preserved |
| A6 | FAIL | Seed-only probe passes; propagated chain and full outline refuse trimming the shared bottom face |
| A7 | FAIL | Diameter sequence refuses adjacent NURBS support; after-bottom row is blocked by A6 |
| A8 | PASS, native only | Source and cancel document equality pass; browser/history/latency target is not qualified |
| Cumulative A1-A7 | FAIL | Stops at invalid z26 candidate; A2-A7 are explicitly blocked |

The acceptance run completes evidence collection and exits nonzero as intended.
The STEP rows reproduce the earlier baseline; adding the R1 rim to downstream
isolated inputs does not remove the known refusals. There are no child crashes
or timeouts in the measured run. The generated table keeps each individual
row, including supplemental no-rim A1 and both A3 proxy selections.

## W3 results at kernel f4b5d747

81 process-isolated probes completed without crashes or timeouts. `P` means
the entire target including R1 rim passes. `B` means the API builds the entire
target, but the analytic-radius requirement fails. `F` means blend construction
fails. `E` means the sharp edit is refused, so blend construction is not reached.
All unspecified shoulder/convex/seam radii are R4. Both orders were tried for
each of the three sets containing an R2 vertical seam.

| Sharp target | All R4 | Convex 1.5, seam 4 | Convex 1.9, seam 4 | Seam 2, shoulder 4 | Convex 1.5 + seam 2 | Convex 1.9 + seam 2 |
|---|---|---|---|---|---|---|
| Original, cap z36 | P | B | B | F | F | F |
| Cap z23 | E | E | E | E | E | E |
| Cap z26 | F | F | F | F | F | F |
| Cap z30 | P | B | B | F | F | F |
| Cap z61.2 | P | B | B | F | F | F |
| Diameter 14.6 | E | E | E | E | E | E |
| Diameter 17 | E | E | E | E | E | E |
| Diameter 21.1 | E | E | E | E | E | E |
| Diameter 14.7 | E | E | E | E | E | E |

The three P targets use the public `Walking` engine with the
`equal_radius_boss_corner` diagnostic, retaining 7 planes, 5 cylinders,
2 tori and 2 sampled NURBS corner patches. All B targets use `Walking` with
general ridge-chain diagnostics, the `fillet_into_seam_band` family. They retain
7 planes, 3 cylinders, 2 tori and 2 NURBS faces: the two convex bands are stored
as NURBS over their straight portions too, so there is no analytic R1.5/R1.9
cylinder. They do not meet the spec's analytic preservation requirement.

Every P/B target passes both validators, B-Rep free/non-manifold counts,
both welded mesh boundary/non-manifold counts, feature classification probes,
source preservation, API/mesh volume agreement and volume convergence.
All four edge counts are zero. The largest API/mesh volume disagreement
among these completed targets is below `5.65e-6` relative.

| Cap / convex radius | API volume, coarse / fine | Independent mesh volume, coarse / fine |
|---|---|---|
| z36 / R4 | 16339.872377 / 16339.872377 | 16339.845060 / 16339.858523 |
| z36 / R1.5 | 16451.495932 / 16451.495932 | 16451.456497 / 16451.475780 |
| z36 / R1.9 | 16441.351467 / 16441.371447 | 16441.351467 / 16441.371447 |
| z30 / R4 | 15661.288773 / 15661.288773 | 15661.272898 / 15661.280094 |
| z30 / R1.5 | 15772.912524 / 15772.912524 | 15772.884771 / 15772.898018 |
| z30 / R1.9 | 15762.779786 / 15762.793716 | 15762.779786 / 15762.793716 |
| z61.2 / R4 | 19189.923511 / 19189.923511 | 19189.831106 / 19189.877138 |
| z61.2 / R1.5 | 19301.546246 / 19301.546246 | 19301.437376 / 19301.492746 |
| z61.2 / R1.9 | 19291.331893 / 19291.388180 | 19291.331893 / 19291.388180 |

Specific failures:

- R2 seam first: the intermediate seam API call succeeds but its meshes have
  8 boundary edges. API/mesh volume disagreement is about 10.7-12.5% over the
  tested heights. Adding the R4 shoulder refuses with `UnsupportedVertexBlend`
  (`2 stripes meet`). A successful intermediate API call is not a qualified
  intermediate body. Both combined sets encounter this before convex rounding.
- Shoulder first: R4 shoulder construction succeeds at the separated heights,
  then the R2 seam refuses with `TrimmingFailure`. At z26, the shoulder itself
  can refuse a wire self-intersection. Neither order reaches a complete unequal
  seam/shoulder corner family.
- z26: the sharp cap edit succeeds, but R4 rebuilding refuses vertex blending;
  convex variants can refuse disconnected/open-shell postconditions. This is
  distinct from the W2 imported-rim z26 API-success corruption.
- z23: the sharp cap edit refuses the swept intersection with the block top.
- All four diameters: both resize and exact carrier replacement refuse
  `quarter-wall replacement requires two circular rims and two axial sides`.
  Exact Boolean growth returns `ExactOnlyUnattainable`, after rejecting an
  invalid exact result; no approximate fallback is enabled. Thus the blend
  engine at these changed diameters remains **unmeasured**, not proven unable.

## W6/W7 decision

**Stop the full W6/W7 implementation as currently premised on existing rebuild
coverage.** The spike does not establish a qualifying rebuild for the R2
seam/R4 shoulder network. Treat its unequal corner construction as a focused
research spike before committing to the group-rebuild planner. This is evidence
about the tested orders and APIs, not a proof that every construction is impossible.

A constrained W6 convex-only path has an engineering basis: the existing ridge
walker constructs it and passes closure/volume/probe checks. It still needs an
analytic straight-span split rather than shipping the whole stripe as NURBS.
Equal-R4 rebuilding is already available when the cap remains separated.

W7 additionally needs a sharp-body radius edit path for this trim layout before
the changed-diameter blend sets can even be qualified. The premise that this
Gecko can already be resized sharply through those public exact routes is false
at this kernel revision. Do not infer unequal-radius success from A0 or browser
parity from any of these native results.

## Validation and artifacts

Local: ignored-suite compilation, touched-crate formatting, focused Clippy with
the workspace lint policy, layer/module boundary checks, normal discovery
(`1 ignored`), explicit passing A5, and explicit failing z26 acceptance assertions.
The 34 acceptance rows record 15 local geometry passes (including 2 A3 proxies)
and 19 failures; the complete assertions intentionally fail. W3's separate
81-case report-only run exits successfully after collecting its measured failures.
Full workspace tests, hosted CI, WASM and deployed/browser behavior are separate.

Evidence in this session is under ignored `target/video-workflow-acceptance/`
(acceptance), `target/video-workflow-spike/` (final W3, with feature probes and
all three diameter edit routes), and `target/video-final-focused/` (single-case
assertions). The earlier combined run's W3 rows precede the extra z30 and exact
edit-route probes; use the final spike directory for W3 conclusions. The test
runner emits fixture/source ref/compiler/lock provenance in `run-manifest.json`;
an invocation directly through the compiled test executable emits the case
evidence without the PowerShell wrapper's provenance manifest.
