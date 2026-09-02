# RFC 0006 `FaceSurface` wildcard audit

Baseline: `origin/main` at `96422993` on 2026-09-01. This is the committed
O2.1a audit artifact required by
[RFC 0006](rfc-0006-swept-analytic-surfaces.md). Line numbers identify the
baseline; the enclosing expression and disposition are authoritative when
later edits move them.

The repository doctrine's broad inventory command remains the first pass:

```bash
rg -n --multiline '(EdgeCurve|FaceSurface)::[\s\S]{0,600}?^\s+_ =>' crates/*/src/
```

That expression deliberately over-reports across adjacent and nested matches.
This audit then inspected actual Rust `match` blocks, excluded code inside
`#[cfg(test)]` modules and `tests.rs`, and required a production match to
contain both a `FaceSurface::` pattern and a wildcard arm. Result: **92 match
sites in 40 production files**. This agrees with the program's approximate
"~93" census and replaces the older "~72" estimate.

Dispositions:

- **DELEGATE** — the new variants must take a common checked delegate path;
  retaining a wildcard that can absorb them is forbidden.
- **ADD** — add explicit revolution/extrusion semantics at this site.
- **EQUALITY** — add structural same-carrier/equivalence arms for both native
  variants.
- **NARROW** — the routine is intentionally specialized; the new variants do
  not belong in it. Preserve that decline/refusal and make it explicit where
  a future reader could mistake it for support.
- **REFUSE** — keep the operation unsupported for swept faces with a stable,
  typed error and both operand/input sides where applicable.

O2.1c may split or move a site, but it must close every DELEGATE, ADD, and
EQUALITY row and preserve every NARROW/REFUSE decision with tests. Re-running
the inventory and updating this file is part of that PR's exit gate.

| # | Baseline site | Current wildcard role | O2.1 disposition |
|---:|---|---|---|
| 1 | `crates/algo/src/builder/face_splitter/conversion.rs:426` | Lists surfaces with periodic u. | **DELEGATE** — use `u_period()`; revolution is `2π`, extrusion follows its profile. |
| 2 | `crates/algo/src/builder/face_splitter/special_cases.rs:2042` | Applies an inward offset only to a coplanar special case. | **NARROW** — non-planes intentionally receive no plane offset. |
| 3 | `crates/algo/src/builder/fill_images_faces.rs:539` | Extracts a plane normal after an `is Plane` guard. | **NARROW** — keep the plane-only arrangement branch. |
| 4 | `crates/algo/src/builder/fill_images_faces.rs:1561` | Detects equal-radius cylinder pairs. | **NARROW** — this degeneracy predicate is cylinder-specific. |
| 5 | `crates/algo/src/builder/fill_images_faces.rs:3215` | Enables plane clipping for an arc/line rescue. | **NARROW** — do not present the rescue as a general surface clipper. |
| 6 | `crates/algo/src/builder/fill_images_faces.rs:3342` | Builds a `PlaneFrame` for a planar arrangement. | **NARROW** — swept surfaces stay out of the planar arrangement. |
| 7 | `crates/algo/src/builder/mod.rs:1337` | Falls back to world Z when projection does not yield a normal. | **DELEGATE** — checked projection/normal must handle both variants; no fixed-axis fallback. |
| 8 | `crates/algo/src/builder/same_domain.rs:1462` | Projects only cylinder/cone overlap samples. | **NARROW** — leave this calibrated quadric-overlap helper narrow. |
| 9 | `crates/algo/src/builder/same_domain.rs:1470` | Computes the radius scale for that cylinder/cone helper. | **NARROW** — same helper and scope as #8. |
| 10 | `crates/algo/src/builder/same_domain.rs:1738` | Estimates representative area only for plane/cylinder/cone faces. | **ADD** — use swept UV integration for same-domain representative selection. |
| 11 | `crates/algo/src/builder/same_domain.rs:2024` | Returns unknown for unlisted carrier pairs. | **EQUALITY** — compare profile, axis/vector, periods, and orientation structurally. |
| 12 | `crates/algo/src/pave_filler/phase_ef.rs:338` | Builds cached seed grids only for NURBS. | **NARROW** — native swept projection does not need a NURBS seed grid. |
| 13 | `crates/algo/src/pave_filler/phase_ef.rs:398` | Uses an exact plane crossing, generic surface solver otherwise. | **DELEGATE** — both variants take the generic checked surface solver. |
| 14 | `crates/algo/src/pave_filler/phase_ef.rs:432` | Uses a plane normal or projected surface normal. | **DELEGATE** — both variants take checked projection/normal. |
| 15 | `crates/algo/src/pave_filler/phase_ff.rs:479` | Reads axial v only for cylinder/cone clipping. | **NARROW** — this closed-conic patch remains cylinder/cone-only. |
| 16 | `crates/algo/src/pave_filler/phase_ff.rs:1929` | Selects the torus×plane oval special case. | **NARROW** — do not widen a torus-specific topology patch. |
| 17 | `crates/algo/src/pave_filler/phase_ff.rs:2635` | Selects the plane×cylinder/cone closed-conic patch. | **NARROW** — swept pairs use their disclosed general/closed-form routes. |
| 18 | `crates/algo/src/pave_filler/phase_ff.rs:2812` | Has exact line crossings for cylinder/cone, otherwise uniform-t fallback. | **NARROW** — retain the named fallback until a swept line solver is qualified; do not label it exact. |
| 19 | `crates/algo/src/pave_filler/phase_ff.rs:3399` | Adds carrier bounds for faces with degenerate boundary samples. | **ADD** — use conservative bounded swept-face AABBs so full-period seams cannot vanish. |
| 20 | `crates/algo/src/pave_filler/phase_vf.rs:103` | Handles planes directly and every parametric surface generically. | **DELEGATE** — both variants take checked evaluate/project. |
| 21 | `crates/blend/src/chamfer_builder.rs:527` | Selects a plane×cylinder/cone closed-rim rebuild. | **NARROW** — decline swept supports; the assembler is not general. |
| 22 | `crates/blend/src/chamfer_builder.rs:574` | Extracts the axis of that cylinder/cone wall. | **NARROW** — same closed-rim scope as #21. |
| 23 | `crates/blend/src/fillet_builder.rs:1025` | Selects a plane×cylinder/cone closed-rim rebuild. | **NARROW** — decline swept supports. |
| 24 | `crates/blend/src/fillet_builder.rs:1074` | Extracts the wall axis for that rebuild. | **NARROW** — same scope as #23. |
| 25 | `crates/blend/src/fillet_builder.rs:1141` | Uses torus minor radius or a setback fallback for a fillet stripe. | **NARROW** — a swept carrier is not a produced fillet stripe in this path. |
| 26 | `crates/blend/src/fillet_builder.rs:1215` | Applies cone-specific convexity, cylinder behavior otherwise. | **NARROW** — only surfaces admitted by the earlier closed-rim gate reach it. |
| 27 | `crates/blend/src/fillet_builder.rs:1265` | Requires the produced stripe to be a torus. | **REFUSE** — preserve `TrimmingFailure` for any other stripe type. |
| 28 | `crates/blend/src/query.rs:211` | Compares known analytic carriers. | **EQUALITY** — add structural native swept carrier comparison. |
| 29 | `crates/blend/src/trimmer.rs:982` | `AwayFrom` trimming derives keep-side only on planes. | **REFUSE** — swept-face trimming remains typed `TrimmingFailure` until B4 qualifies it. |
| 30 | `crates/heal/src/fix/face.rs:139` | Uses Newell normal as a non-planar orientation proxy. | **DELEGATE** — prefer checked surface normal for swept faces; retain Newell only as disclosed fallback. |
| 31 | `crates/io/src/step/reader.rs:2036` | Resolves multiple bounds for planes or periodic surfaces. | **DELEGATE** — use period delegates so both variants enter the correct resolver. |
| 32 | `crates/io/src/step/reader.rs:2536` | Builds exact seams for supported periodic winding axes. | **ADD** — profile seam for revolution; sweep-direction seam for periodic extrusion profile; otherwise typed refusal. |
| 33 | `crates/offset/src/inter3d.rs:190` | Converts only quadric carriers to `AnalyticSurface`. | **NARROW** — use `as_analytic()`; general swept surfaces are not quadrics. |
| 34 | `crates/offset/src/inter3d.rs:354` | Compares only plane/cylinder/sphere same domains. | **EQUALITY** — add both swept carriers or share the common carrier comparator. |
| 35 | `crates/offset/src/inter3d.rs:394` | Returns the input point for unlisted surface projection. | **DELEGATE** — checked swept projection is mandatory; identity is forbidden. |
| 36 | `crates/offset/src/loops.rs:598` | Requires a plane normal in the line-intersection loop builder. | **REFUSE** — retain the typed non-planar assembly error. |
| 37 | `crates/offset/src/loops.rs:751` | Leaves non-planar boundary endpoints unprojected as an approximation. | **DELEGATE** — project swept endpoints or return a typed projection failure. |
| 38 | `crates/offset/src/move_faces.rs:212` | Selects exact plane×cylinder reconstruction. | **NARROW** — do not widen the support-pair algorithm. |
| 39 | `crates/operations/src/boolean/assembly.rs:1041` | Filters a planar opposing-normal heuristic. | **NARROW** — non-planar faces are intentionally absent from this heuristic. |
| 40 | `crates/operations/src/boolean/assembly.rs:2228` | Approximates non-plane representative normals from vertices. | **DELEGATE** — use checked swept projection/normal before the geometric fallback. |
| 41 | `crates/operations/src/compound_ops.rs:159` | Recognizes a simple capped cylinder. | **NARROW** — a native extrusion/revolution is not this primitive signature. |
| 42 | `crates/operations/src/compound_ops.rs:599` | Recognizes a line-bounded polyhedron. | **NARROW** — curved swept faces correctly decline. |
| 43 | `crates/operations/src/defeature.rs:662` | Requires a planar kept face. | **REFUSE** — preserve the stable non-planar unsupported result. |
| 44 | `crates/operations/src/draft.rs:318` | Collects planar support equations. | **NARROW** — swept faces remain `None` until curved draft is qualified. |
| 45 | `crates/operations/src/feature_recognition.rs:452` | Recognizes cylindrical holes. | **NARROW** — do not misclassify a general swept wall as a cylinder. |
| 46 | `crates/operations/src/fillet/mod.rs:164` | Builds planar polygons for legacy trimming. | **NARROW** — non-planar supports continue through the existing non-polygon path. |
| 47 | `crates/operations/src/fillet/mod.rs:487` | Same planar polygon gate in the variable-radius path. | **NARROW** — same decision as #46. |
| 48 | `crates/operations/src/fillet/rolling_ball.rs:319` | Builds planar polygons for rolling-ball trimming. | **NARROW** — non-planar carriers remain untrimmed/declined by the existing contract. |
| 49 | `crates/operations/src/heal.rs:516` | Flips only plane surfaces; non-planar parameterization is left intact. | **NARROW** — swept orientation lives in `Face::reversed`, not carrier mutation. |
| 50 | `crates/operations/src/heal.rs:977` | Plane comparison is special; curved faces use projected normals below. | **DELEGATE** — swept faces take the existing curved checked-normal path. |
| 51 | `crates/operations/src/loft.rs:1368` | Recognizes a planar circular face. | **NARROW** — the loft fast path is intentionally planar. |
| 52 | `crates/operations/src/measure/area.rs:251` | Asserts a cone-only helper precondition. | **NARROW** — keep the typed helper guard. |
| 53 | `crates/operations/src/measure/area.rs:323` | Asserts a torus-only helper precondition. | **NARROW** — keep the typed helper guard. |
| 54 | `crates/operations/src/measure/volume.rs:92` | Detects a torus notch-band workaround. | **NARROW** — native revolution does not inherit a torus defect signature. |
| 55 | `crates/operations/src/measure/volume.rs:184` | Detects a notched cylinder/cone wall. | **NARROW** — keep the calibrated quadric predicate. |
| 56 | `crates/operations/src/measure/volume.rs:235` | Detects full-period cylinder/cone boundary winding. | **NARROW** — swept periods are handled by the general integrator. |
| 57 | `crates/operations/src/measure/volume.rs:338` | Selects which holed analytic faces the per-face integrator can trust. | **ADD** — route swept faces through period-aware UV trimming or decline explicitly; never fall through as proven. |
| 58 | `crates/operations/src/measure/volume.rs:490` | Validates a quadric-only revolution-solid closed form. | **ADD** — native revolution faces use the general face integrator; they must not be accepted by the quadric shortcut accidentally. |
| 59 | `crates/operations/src/measure/volume.rs:686` | Recognizes the two-cylinder Steinmetz lens. | **NARROW** — reject all other carriers from this exact formula. |
| 60 | `crates/operations/src/measure/volume.rs:1664` | Asserts a cylinder-only volume helper precondition. | **NARROW** — keep the typed helper guard. |
| 61 | `crates/operations/src/measure/volume.rs:2005` | Asserts a cone-only volume helper precondition. | **NARROW** — keep the typed helper guard. |
| 62 | `crates/operations/src/measure/volume.rs:2126` | Asserts a sphere-only volume helper precondition. | **NARROW** — keep the typed helper guard. |
| 63 | `crates/operations/src/measure/volume.rs:2254` | Asserts a torus-only volume helper precondition. | **NARROW** — keep the typed helper guard. |
| 64 | `crates/operations/src/measure/volume.rs:2460` | Accepts known per-face analytic formulas and declines the rest. | **ADD** — integrate both swept variants over their trimmed UV domains, independently of mesh deflection. |
| 65 | `crates/operations/src/offset_wire.rs:89` | Requires a planar face for 2D wire offset. | **REFUSE** — preserve the typed non-planar error. |
| 66 | `crates/operations/src/push_pull.rs:429` | Checks neighbor invariance only for plane/cylinder supports. | **NARROW** — return false transactionally for swept neighbors until qualified. |
| 67 | `crates/operations/src/push_pull.rs:548` | Recognizes a simple capped cylinder push/pull case. | **NARROW** — reject other carriers from the primitive fast path. |
| 68 | `crates/operations/src/resize_blend.rs:315` | Recognizes cylinder/torus/sphere blend bands. | **NARROW** — a general swept face is not a proven blend band. |
| 69 | `crates/operations/src/resize_blend.rs:1412` | Classifies plane×cylinder supports after healing. | **REFUSE** — retain reconstruction failure if the support types change. |
| 70 | `crates/operations/src/resize_blend.rs:1461` | Rechecks the plane support. | **REFUSE** — same transactional reconstruction contract as #69. |
| 71 | `crates/operations/src/resize_blend.rs:1465` | Rechecks the cylinder support. | **REFUSE** — same contract as #69. |
| 72 | `crates/operations/src/resize_blend.rs:1806` | Classifies cylinder×cone supports after healing. | **REFUSE** — swept supports remain outside this reconstruction. |
| 73 | `crates/operations/src/resize_blend.rs:1836` | Rechecks the cylinder support. | **REFUSE** — same contract as #72. |
| 74 | `crates/operations/src/resize_blend.rs:1840` | Rechecks the cone support. | **REFUSE** — same contract as #72. |
| 75 | `crates/operations/src/revolve.rs:743` | Requires a planar input face for the analytic profile fast path. | **NARROW** — this is an operation-input gate, not a carrier-consumer omission. |
| 76 | `crates/operations/src/revolve.rs:1136` | Requires a planar single-circle torus fast path. | **NARROW** — keep the primitive-specific gate. |
| 77 | `crates/operations/src/sweep.rs:495` | Requires a planar profile for straight extrusion. | **NARROW** — surface variants do not broaden face-profile support. |
| 78 | `crates/operations/src/sweep.rs:1703` | Refuses non-planar profiles in miter sweep. | **REFUSE** — preserve the typed invalid-input result. |
| 79 | `crates/operations/src/sweep.rs:2566` | Uses Newell normal for non-planar multi-section profiles. | **NARROW** — retain the boundary-derived profile orientation. |
| 80 | `crates/operations/src/tessellate/mesh_ops.rs:333` | Compares known carrier equivalence. | **EQUALITY** — add structural comparison for both swept variants. |
| 81 | `crates/operations/src/tessellate/nonplanar.rs:45` | Selects a cylinder/cone curved-wire structured mesher. | **NARROW** — native swept bands get a separate structured mesher. |
| 82 | `crates/operations/src/tessellate/nonplanar.rs:310` | Selects a cylinder/cone rim-sweep mesher. | **NARROW** — same decision as #81. |
| 83 | `crates/operations/src/tessellate/nonplanar.rs:1188` | Selects the sphere/torus holed-band mesher. | **NARROW** — do not force arbitrary swept holes into that topology. |
| 84 | `crates/operations/src/tessellate/nonplanar.rs:1209` | Chooses the radius scale for that sphere/torus mesher. | **NARROW** — same scope as #83. |
| 85 | `crates/operations/src/tessellate/nonplanar.rs:1890` | Enumerates periodic-u surfaces for boundary unwrapping. | **DELEGATE** — use `u_period()` and the finite face-domain branch target. |
| 86 | `crates/operations/src/tessellate/solid.rs:283` | Synchronizes circular edges to cylinder/cone face-grid density. | **ADD** — the new structured mesher supplies profile/sweep density and shared boundary samples. |
| 87 | `crates/operations/src/untrim.rs:138` | Requires a NURBS surface for NURBS untrimming. | **REFUSE** — preserve the typed NURBS-only contract. |
| 88 | `crates/topology/src/face.rs:355` | Returns an effective normal only for planes. | **NARROW** — `effective_plane_normal` remains plane-only by definition. |
| 89 | `crates/wasm/src/bindings/operations.rs:1911` | Duplicates the NURBS-only untrim gate. | **REFUSE** — keep direct WASM parity with native typed refusal. |
| 90 | `crates/wasm/src/bindings/query.rs:1357` | Sends every non-plane/non-NURBS surface to a generic analytic wildcard. | **DELEGATE** — add checked native swept projection with stable convergence errors. |
| 91 | `crates/wasm/src/bindings/query.rs:1442` | Grid evaluation enumerates four quadrics and skips the rest. | **DELEGATE** — remove the nested variant list; use bounded checked surface evaluation. |
| 92 | `crates/wasm/src/bindings/query.rs:1457` | Final projection enumerates four quadrics and returns the input point otherwise. | **DELEGATE** — identity fallback is forbidden; return evaluated projection or typed refusal. |

## Compiler-flagged companion sweep

This checklist is only the wildcard half of the variant ripple. O2.1c must
also resolve every exhaustive match the compiler names. High-risk exhaustive
sites measured on this baseline include `topology/src/face.rs` delegates,
`check/src/properties/face_integrator.rs`, `operations/src/transform.rs`,
`operations/src/tessellate/face.rs`, `io/src/arena_io.rs`, and
`io/src/step/writer.rs`. Compilation is the inventory for those sites; this
document must not be used to waive them.
