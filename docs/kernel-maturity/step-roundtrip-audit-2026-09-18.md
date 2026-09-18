# STEP round-trip audit — 2026-09-18

Scope: every STEP fixture under `crates/io/tests/data/` (31 files). For each
fixture: import → record per-solid face count, `FaceSurface` / `EdgeCurve`
histograms, volume and surface area via `remus_operations::measure` →
export via `remus_io::step::writer::write_step` → re-import → repeat.
Finding threshold (per task): analytic degrades to NURBS on the round trip,
or face count changes, or volume relative drift > 1e-9. Area drift is
reported alongside volume; one area-only drift is filed as a finding because
it exceeds any reasonable mass-property tolerance.

Status: audit complete, 2026-09-18. 30/31 fixtures import; 28/30 readable
fixtures round-trip exactly (identical histograms, face counts, volume and
area to ≤3e-16 relative). 3 findings filed below, each with an `#[ignore]`
ready-repro in `crates/io/tests/`. No reader/writer fix in this PR (read-only
audit). Roadmap rows that own STEP fidelity point here.

Method (per the io-formats skill's analytic round-trip check):
- Reader: `remus_io::step::reader::read_step(&str, &mut Topology) -> Vec<SolidId>`.
- Writer: `remus_io::step::writer::write_step(&Topology, &[SolidId]) -> String`.
- Census: `remus_topology::explorer::solid_faces` + `solid_edges`;
  `face.surface()` matched on `Plane/Cylinder/Cone/Sphere/Torus/Nurbs`;
  `edge.curve()` matched on `Line/Circle/Ellipse/Parabola/Hyperbola/NurbsCurve`.
- Measure: `remus_operations::measure::solid_volume(&topo, solid, 0.01)` and
  `solid_surface_area(&topo, solid, 0.01)`; deflection = 0.01 throughout;
  per-solid values summed; rel delta = |after−before|/|before|.
- Limits: `remus_io::ImportLimits::default()` (`max_input_bytes` 256 MiB,
  `max_model_entities` 3,000,000). All fixtures are 1.7 KB–931.8 KB, far
  under the byte limit; no limit was hit or changed.
- No healing after import (`heal_solid`, `convert_to_elementary`, and
  `unify_same_domain` were NOT applied): this is a pure writer/reader
  fidelity check, not a heal-after-import check. NURBS that were already
  NURBS in the file are faithful, not degradations (see observations).
- Batches were fanned out (~10 fixtures each) via throwaway integration
  tests that were deleted before commit; the parent re-verified all three
  findings plus the lemon-torus observation natively (see finding sections
  for exact re-verified numbers).

## Summary table

Hist key: Pl=Plane, Cyl=Cylinder, Cone=Cone, Sph=Sphere, Tor=Torus,
Nurb=NURBS surface; Ln=Line, Cir=Circle, Ell=Ellipse, Par=Parabola,
Hyp=Hyperbola, NurbC=NURBS curve. Hists aggregated over all solids in the
file. Volume/area at deflection 0.01.

| Fixture | Solids before→after | Faces before→after | FaceSurface before→after | EdgeCurve before→after | Volume before → after (rel) | Area before → after (rel) | Verdict |
|---|---|---|---|---|---|---|---|
| axis2_optional_attrs_cylinder.step | 1→1 | 3→3 | Pl2 Cyl1 → identical | Ln1 Cir2 → identical | 5.026548245744e2 → same (0.0) | 3.518583772020e2 → same (0.0) | pass |
| conical_surface_base_radius_frustum.step | 1→1 | 4→4 | Pl2 Cone2 → identical | Ln2 Cir4 → identical | 5.730265000148e3 → same (0.0) | 2.412743157957e3 → same (0.0) | pass |
| jolly_fox_partial_cylinder.step | 1→1 | 10→10 | Pl9 Cyl1 → identical | Ln30 Cir2 → identical | 3.229676293801e4 → same (0.0) | 9.216415896391e3 → same (0.0) | pass |
| lemon-torus-band.step | 1→1 | 3→3 | Pl2 Nurb1 → identical | Cir3 → identical | 1.371225985403e2 → same (0.0) | 1.326941026019e2 → same (0.0) | pass (see observation 1) |
| lipfuse_2x1_body.step | 1→1 | 19→19 | Pl11 Cyl8 → identical | Ln32 Cir16 → identical | 8.402520894854e3 → same (0.0) | 1.429647582531e4 → same (0.0) | pass |
| lipfuse_2x1_lip.step | 1→1 | 49→49 | Pl25 Cyl12 Cone12 → identical | Ln76 Cir30 → identical | 2.983109067246e3 → same (1.5e-16) | 4.006402254711e3 → same (0.0) | pass |
| lipfuse_3x3_body.step | 1→1 | 27→27 | Pl11 Cyl16 → identical | Ln40 Cir32 → identical | 2.762004089634e4 → same (0.0) | 4.662807582531e4 → same (0.0) | pass |
| lipfuse_3x3_lip.step | 1→1 | 49→49 | Pl25 Cyl12 Cone12 → identical | Ln76 Cir28 → identical | 6.153269067247e3 → same (3.0e-16) | 8.254329525583e3 → same (0.0) | pass |
| lipfuse_body.step | 1→1 | 19→19 | Pl11 Cyl8 → identical | Ln32 Cir16 → identical | 1.410276089485e4 → same (0.0) | 2.389767582531e4 → same (0.0) | pass |
| lipfuse_lip.step | 1→1 | 49→49 | Pl25 Cyl12 Cone12 → identical | Ln76 Cir28 → identical | 4.039829067246e3 → same (1.1e-16) | 5.422378011669e3 → same (0.0) | pass |
| mambo_b12_period_winding_torus.step | 1→1 | 3→3 | Pl2 Tor1 → identical | Cir3 → identical | 1.2337005501361697e1 → same (0.0) | 6.283185307179599e0 → same (0.0) | pass |
| mambo_b1_untrimmed_nurbs.step | 0→— | — | — | — | — | — | FINDING 1 (read error, no round trip) |
| multicavity_bin_box.step | 1→1 | 10→10 | Pl6 Cyl4 → identical | Ln16 Cir8 → identical | 1.113628583470578e5 → same (0.0) | 1.9161348411812767e4 → same (0.0) | pass |
| multicavity_cavity_0.step | 1→1 | 9→9 | Pl3 Cyl2 Nurb4 → identical | Ln9 Cir4 NurbC8 → identical | 5.114687451766128e4 → same (0.0) | 1.0264902974243138e4 → same (0.0) | pass |
| multicavity_cavity_1.step | 1→1 | 9→9 | Pl4 Cyl2 Nurb3 → identical | Ln11 Cir4 NurbC6 → identical | 5.114687451766233e4 → same (0.0) | 1.0264902974243145e4 → same (0.0) | pass |
| openzcad_a_export_bored_plate.step | 1→1 | 7→7 | Pl6 Cyl1 → identical | Ln13 Cir2 → identical | 8.814601836602553e3 → same (0.0) | 3.3570796326792765e3 → same (0.0) | pass |
| openzcad_e_analytic_fillet_plate.step | 1→1 | 10→10 | Pl6 Cyl4 → identical | Ln16 Cir8 → identical | 9.52274333882308e3 → same (0.0) | 3.1330442269800415e3 → same (0.0) | pass |
| openzcad_e_nurbs_fillet_plate.step | 1→1 | 10→10 | Pl6 Nurb4 → identical | Ln24 → identical | 9.539115868770154e3 → same (0.0) | 3.118708877127896e3 → same (0.0) | pass (faithful NURBS source) |
| oring_nested_holes.step | 1→1 | 47→47 | Pl23 Cyl24 → identical | Ln88 Cir48 → identical | 4.515885667455405e3 → same (0.0) | 5.722910263355824e4 → same (0.0) | pass |
| scale_platform_v1_linear_spline_edge.step | 1→1 | 6→6 | Pl6 → identical | Ln12 → identical | 4.608037978666666e6 → same (0.0) | 2.624001800514508e5 → same (0.0) | pass |
| scoop_bin_box.step | 1→1 | 10→10 | Pl6 Cyl4 → identical | Ln16 Cir8 → identical | 1.11362858347057801e5 → same (0.0) | 1.91613484118127672e4 → same (0.0) | pass |
| scoop_cavity_0.step | 1→1 | 9→9 | Pl3 Cyl2 Nurb4 → identical | Ln9 Cir4 NurbC8 → identical | 5.11468745176612792e4 → same (0.0) | 1.02649029742431376e4 → same (0.0) | pass |
| scoop_cavity_1.step | 1→1 | 9→9 | Pl4 Cyl2 Nurb3 → identical | Ln11 Cir4 NurbC6 → identical | 5.11468745176623343e4 → same (0.0) | 1.02649029742431449e4 → same (0.0) | pass |
| scoop_scoop_0.step | 1→1 | 56→56 | Pl56 → identical | Ln156 → identical | 6.37612080524401517e3 → same (0.0) | 7.98525208325882249e3 → same (0.0) | pass |
| shapr3d_hammer_holder.step | 1→1 | 160→160 | Pl52 Cyl42 Cone2 Sph8 Tor14 Nurb42 → identical | Ln200 Cir96 NurbC90 → identical | 5.02406431268449305e4 → 5.02406460872056996e4 (5.89e-8) | 1.39388151664662237e4 → same (0.0) | FINDING 2 (volume drift) |
| shapr3d_walking_stick_foot.step | 1→1 | 11→11 | Pl3 Cyl3 Cone2 Tor3 → identical | Ln5 Cir17 → identical | 3.23649016740484731e4 → same (0.0) | 1.08920564412322947e4 → same (1.7e-16) | pass |
| shapr_untrimmed_nurbs_domain.step | 1→1 | 4→4 | Pl3 Nurb1 → identical | Ln4 NurbC2 → identical | 1.06290637561401069e0 → 1.06290637561401002e0 (6.3e-16) | 1.94950549898607264e1 → 1.76719603545463393e1 (9.35e-2) | FINDING 3 (area drift) |
| wallcut_body.step | 1→1 | 19→19 | Pl11 Cyl8 → identical | Ln32 Cir16 → identical | 1.41027608948542875e4 → same (0.0) | 2.38976758253055596e4 → same (0.0) | pass |
| wallcut_tool_0.step | 1→1 | 56→56 | Pl24 Cyl32 → identical | Ln80 Cir64 → identical | 7.24309284579545056e3 → same (0.0) | 6.03454981586510166e3 → same (0.0) | pass |
| wide_sphere_cap.step | 1→1 | 2→2 | Pl1 Sph1 → identical | Cir1 → identical | 2.99333758546055424e3 → same (0.0) | 1.01080743629251526e3 → same (0.0) | pass |
| wide_sphere_cap_with_seam.step | 1→1 | 2→2 | Pl1 Sph1 → identical | Cir2 → identical | 2.99333758546055424e3 → same (0.0) | 1.62544777961459630e2 → same (0.0) | pass |

Totals: 31 fixtures, 30 import, 28 pass exactly, 3 findings. Zero
analytic→NURBS degradations on the export→re-import leg; zero face-count
changes on any readable fixture.

## Finding 1 — `mambo_b1_untrimmed_nurbs.step` import rejection

- Fixture: `crates/io/tests/data/mambo_b1_untrimmed_nurbs.step` (39,442 bytes).
- What degraded: nothing round-tripped — the first import fails, so no
  export/re-import leg exists. Error (re-verified by parent):
  `parse error: EDGE_CURVE #253 start endpoint misses its carrier by
  6.164947e-5 mm (local recovery cap 1.000000e-6 mm)`.
  The file stores 8 `CYLINDRICAL_SURFACE` carriers with 8
  `B_SPLINE_CURVE_WITH_KNOTS` trims over 32 `EDGE_CURVE`s; the #253 trim
  start misses its carrier by ~62× the local cap.
- Code path suspected (read-only, not fixed): `crates/io/src/step/reader.rs`
  curved-edge import adapter (`endpoint_error` constructor near line 4641 and
  the `Circle`/NURBS residual checks that follow, lines ~4673–4958). The
  declared-trim check compares against `tolerance_cap`
  (vertex tolerances ∪ `model_tolerance_cap`), while the projected-recovery
  path is additionally clamped by `MAX_PROJECTED_NURBS_RECOVERY_TOLERANCE_MM`
  (1e-6) and the untrimmed path by
  `MAX_UNTRIMMED_NURBS_RECOVERY_TOLERANCE_MM` (1e-4) — see lines ~56–67 and
  ~4820–4846. The 6.16e-5 mm miss exceeds the 1e-6 projected cap but would
  fit the 1e-4 untrimmed cap; whether #253 qualifies for the wider path (or
  for a tolerance-carrying repair via `heal_solid` / `convert_to_elementary`
  per the heal-after-import recipe) is the open question for the owning row.
- Ready-repro: `crates/io/tests/step_roundtrip_mambo_b1_repro.rs`
  (`mambo_b1_import_is_refused`, `#[ignore = "open: ..."]`). Its doc comment
  carries this finding. It currently asserts successful import and is
  ignored because import fails; when the reader (or the documented
  heal-after-import path) accepts the file, remove the `#[ignore]` and
  extend it with the round-trip histogram/volume asserts.
- Heal-after-import note: per the io-formats skill, the next step for the
  owner is to try `heal_solid`/`repair_solid` and `convert_to_elementary`
  after a (currently impossible) import — or to decide the file is genuinely
  out of tolerance. This audit did not heal; it records the refusal.

## Finding 2 — `shapr3d_hammer_holder.step` volume drift 5.89e-8

- Fixture: `crates/io/tests/data/shapr3d_hammer_holder.step` (931,835 bytes,
  the largest STEP fixture; 160 faces: Pl52 Cyl42 Cone2 Sph8 Tor14 Nurb42;
  90 NURBS edges).
- What degraded: topology and types are bit-identical across the round trip
  (160→160 faces, identical `FaceSurface`/`EdgeCurve` histograms), area is
  bit-identical (1.39388151664662237e4 → same, rel 0.0), but volume drifts:
  5.02406431268449305e4 → 5.02406460872056996e4 (abs +0.00296, rel
  5.892362e-8 > 1e-9 threshold). Re-verified by parent at deflection 0.01.
- Code path suspected (read-only, not fixed): the NURBS
  write→read leg. The writer emits full double precision
  (`fmt_f64`/`fmt_weight` use `{:.17E}` in `crates/io/src/step/writer.rs`,
  ~line 1660), so decimal rounding is unlikely to explain +0.003 on 5e4.
  With 42 NURBS faces the volume goes through tessellation at the 0.01
  deflection; re-parameterized NURBS (knot/weight round-trip through STEP
  text, edge-domain re-derivation in
  `crates/io/src/step/reader.rs::build_bspline_surface` and the
  periodic/untrimmed domain helpers ~lines 2335–2680) can re-tessellate
  slightly differently. Whether the drift is I/O parameter churn or
  measure-side tessellation noise at 0.01 is the open question — the owner
  should re-measure both sides at finer deflection and compare
  `oriented_solid_volume` before calling it a writer bug.
- Ready-repro: `crates/io/tests/step_roundtrip_hammer_holder_repro.rs`
  (`hammer_holder_volume_survives_roundtrip`, `#[ignore = "open: ..."]`).
  Asserts rel drift < 1e-9; fails today (≈5.9e-8), passes when the drift
  closes. Doc comment carries this finding.

## Finding 3 — `shapr_untrimmed_nurbs_domain.step` area drift 9.35e-2

- Fixture: `crates/io/tests/data/shapr_untrimmed_nurbs_domain.step` (6,600
  bytes; 4 faces: Pl3 Nurb1; edges Ln4 NurbC2). The single NURBS face is
  genuine source NURBS (`B_SPLINE_SURFACE_WITH_KNOTS` in the file), so the
  NURBS import itself is faithful.
- What degraded: topology and types identical (4→4 faces, identical hists),
  volume stable (1.06290637561401069e0 → ...1002e0, rel 6.3e-16), but surface
  area collapses 19.495054989860726 → 17.67196035454634 (abs −1.823, rel
  9.351575e-2). Re-verified by parent at deflection 0.01. A 9% area move
  with volume pinned means the NURBS patch's parameterization (not its
  closed volume contribution) changed across the write→read leg.
- Code path suspected (read-only, not fixed): untrimmed-NURBS domain
  handling. Reader side: `MAX_UNTRIMMED_NURBS_RECOVERY_CONTROL_POINTS`
  (4,096), `MAX_UNTRIMMED_NURBS_RECOVERY_TOLERANCE_MM` (1e-4), and the
  `step_untrimmed_nurbs_domain_recovered` probe (lines ~56–133), plus
  `build_bspline_surface` and the periodic/untrimmed UV-domain helpers
  (~lines 2335–2680, `periodic_uv_domain`, unwrapped-domain interior probes).
  Writer side: NURBS surface/edge-domain emission in
  `crates/io/src/step/writer.rs` (NURBS arms; `edge.strict_domain()` reads
  at ~lines 1050/1996/2057/2075/2115 in reader tests show how domains are
  re-derived). The re-imported face likely recovers a different (smaller)
  sub-domain than the first import. The owner should dump both NURBS
  carriers (control net, knots, recovered domain) before and after.
- Ready-repro: `crates/io/tests/step_roundtrip_untrimmed_nurbs_repro.rs`
  (`untrimmed_nurbs_area_survives_roundtrip`, `#[ignore = "open: ..."]`).
  Asserts area rel < 1e-9 (and volume rel < 1e-9); fails today on area
  (≈9.35e-2), passes when the domain round-trips. Doc comment carries this
  finding.

## Observations (not findings)

1. `lemon-torus-band.step` imports its band as NURBS (Pl2 Nurb1, Tor0) on
   BOTH sides with exact round-trip (rel 0.0). The source file stores
   `DEGENERATE_TOROIDAL_SURFACE('',#8,2.,5.,.F.)` — a lemon/spindle torus
   (minor 5 > major 2). The reader maps that entity to NURBS by design
   (`reader.rs` `"DEGENERATE_TOROIDAL_SURFACE"` arm → `degenerate_torus_nurbs`
   → `FaceSurface::Nurbs`, ~line 3420–3452). This is correct output, not a
   degradation; analytic `Torus` only covers non-degenerate toroids
   (`TOROIDAL_SURFACE` arm → `FaceSurface::Torus`, ~line 3454–3486, as
   exercised by `mambo_b12_period_winding_torus.step` which stays Tor1).
2. `openzcad_e_nurbs_fillet_plate.step` (Pl6 Nurb4) vs its analytic sibling
   `openzcad_e_analytic_fillet_plate.step` (Pl6 Cyl4): both round-trip
   exactly. The NURBS variant genuinely stores B-splines; per the
   io-formats pitfalls table this is faithful, not a reader bug. If an
   analytic comparison is ever needed, `convert_to_elementary` is the
   documented recovery (not applied here).
3. `scale_platform_v1_linear_spline_edge.step` imports its "linear spline"
   edge as 12 `Line` edges / 0 NURBS — factored at import, preserved exactly.
4. `multicavity_*` / `scoop_*` NURBS-bearing cavities (3–4 NURBS faces, 6–8
   NURBS edges) all round-trip bit-exact at 0.01, including volume to 0.0.
5. No fixture changed face count; no readable fixture changed any
   `FaceSurface`/`EdgeCurve` histogram entry on the export→re-import leg.

## Repro pointers

- `crates/io/tests/step_roundtrip_mambo_b1_repro.rs` (Finding 1)
- `crates/io/tests/step_roundtrip_hammer_holder_repro.rs` (Finding 2)
- `crates/io/tests/step_roundtrip_untrimmed_nurbs_repro.rs` (Finding 3)

All three follow the testing skill's ready-repro pattern: file-level doc
comment states the finding, `#![allow(clippy::unwrap_used,
clippy::expect_used)]` header, `#[ignore = "open: ..."]` on the failing
assert, runnable via
`cargo test -p remus-io --test <name> -- --ignored`.
