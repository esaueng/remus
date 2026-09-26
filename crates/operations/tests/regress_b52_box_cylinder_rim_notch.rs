//! B52 box–cylinder rim-notch regressions (2026-09-24).
//!
//! Box 2.5×1×1 against a cylinder r = 2, h = 1 placed by
//! `translation(0.5, −1.5, −0.5) · rotation_z(rot)`. The box notches the
//! cylinder's lateral from its top rim (z ∈ [0, 0.5], between the x = 0 and
//! y = 0 generators). Rotating the cylinder about its own axis leaves the
//! solid unchanged and only moves its seam, yet at rot = 0.3 and 2.0 all three
//! booleans refused `ExactOnlyUnattainable`.
//!
//! Root cause (first-wrong phase: FF broad phase, not the face splitter): the
//! FF face box was the box of 9 samples per boundary edge. For a full rim
//! circle that is an inscribed octagon, short of the rim by up to
//! r·(1 − cos π/8) on an axis, by an amount set by the seam. At rot = 2.0 the
//! lateral's box topped out at y = 0.374 instead of 0.5, so the x = 0 wall's
//! generator at y = 0.436 failed the exact segment-vs-box gate and was
//! dropped: the x = 0 wall stayed whole (area 1.0 instead of 0.782), the
//! lateral reached the wire builder without its notch sections, and assembly
//! lost the bottom rim and cap (raw fuse V−E+F = 1). Fixed by adding each
//! circle/ellipse arc's per-axis extrema to the face box
//! (`phase_ff::conic_arc_axis_extrema`), so the box is exact wherever the
//! seam sits. No tolerance, healing, or mesh-fallback change.
//!
//! Oracles: closed-form intersect volume (the notch prism, integrated in
//! closed form), inclusion–exclusion and the cut complement, the kernel's
//! measured volume against the exact Gauss integral, strict validation on
//! both validators, watertight meshes at 0.1 / 0.01 / 1e-4 and the proptest
//! harness deflection, translation invariance, and ray-cast material probes.
//! The cone twin (B32) is swept across the same seam rotations for all three
//! operations.
//!
//! Scale × rigid-placement acceptance (2026-09-26) crosses scales 1e-3, 1
//! and 1e3 with origin / rigid whole-body placements over the same seam
//! matrix; see the section at the end of this file.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_8, PI};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const OPS: [BooleanOp; 3] = [BooleanOp::Fuse, BooleanOp::Cut, BooleanOp::Intersect];
const V_BOX: f64 = 2.5;

/// Sixteen evenly spaced seam placements plus the two that refused.
fn seam_rotations() -> Vec<f64> {
    let mut rots: Vec<f64> = (0..16).map(|k| f64::from(k) * FRAC_PI_8).collect();
    rots.extend([0.3, 2.0]);
    rots
}

fn exact_boolean(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
    what: &str,
) -> SolidId {
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| panic!("{what}: {e:?}"));
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{what}: non-exact quality"
    );
    outcome.solid
}

fn build_cylinder(topo: &mut Topology, rot: f64) -> (SolidId, SolidId) {
    let m = Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(rot);
    let a = make_box(topo, 2.5, 1.0, 1.0).expect("box");
    let b = make_cylinder(topo, 2.0, 1.0).expect("cylinder");
    remus_operations::transform::transform_solid(topo, b, &m).expect("place");
    (a, b)
}

/// The B32 sibling cone (frustum r0 = 1.5, r1 = 2.5, h = 1) at the same
/// placement.
fn build_cone(topo: &mut Topology, rot: f64) -> (SolidId, SolidId) {
    let m = Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(rot);
    let a = make_box(topo, 2.5, 1.0, 1.0).expect("box");
    let b = make_cone(topo, 1.5, 2.5, 1.0).expect("cone");
    remus_operations::transform::transform_solid(topo, b, &m).expect("place");
    (a, b)
}

fn gauss_volume(topo: &Topology, s: SolidId) -> f64 {
    remus_operations::measure::mass_properties(topo, s)
        .unwrap()
        .mass
}

/// Closed-form box ∩ cylinder volume. The overlap is the prism
/// z ∈ [0, 0.5] over the region x ≥ 0, 0 ≤ y ≤ −1.5 + √(4 − (x − 0.5)²)
/// (the disc never reaches y = 1). With s = x − 0.5 and
/// F(s) = ½(s√(4 − s²) + 4·asin(s/2)), the area is
/// F(√1.75) − F(−0.5) − 1.5·(0.5 + √1.75).
fn inter_closed_form() -> f64 {
    let f = |s: f64| 0.5 * s.mul_add((4.0 - s * s).sqrt(), 4.0 * (s / 2.0).asin());
    let s1 = 1.75_f64.sqrt();
    let area = 1.5f64.mul_add(-(0.5 + s1), f(s1) - f(-0.5));
    0.5 * area
}

fn expected_cylinder_volume(op: BooleanOp) -> f64 {
    let vi = inter_closed_form();
    match op {
        BooleanOp::Fuse => V_BOX + 4.0 * PI - vi,
        BooleanOp::Cut => V_BOX - vi,
        BooleanOp::Intersect => vi,
    }
}

fn frustum_volume(r0: f64, r1: f64, h: f64) -> f64 {
    PI * h / 3.0 * r0.mul_add(r0, r0.mul_add(r1, r1 * r1))
}

fn assert_strict_valid(topo: &Topology, s: SolidId, what: &str) {
    let strict = remus_operations::validate::validate_solid(topo, s)
        .map_err(|e| format!("{what}: validator error: {e:?}"))
        .unwrap();
    assert!(strict.is_valid(), "{what}: ops validator issues");
    let mut opts = remus_check::validate::ValidateOptions::default();
    opts.disabled_checks
        .insert(remus_check::validate::CheckId::ShellConnected);
    let rep = remus_check::validate::validate_solid(topo, s, &opts).unwrap();
    let errs = rep
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .count();
    assert_eq!(errs, 0, "{what}: check-crate errors");
}

fn assert_watertight_at(topo: &Topology, s: SolidId, d: f64, what: &str) {
    let mesh = tessellate_solid(topo, s, d).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: boundary at d={d}");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold at d={d}"
    );
}

/// Watertight at 0.1, 0.01, 1e-4 and the proptest harness deflection
/// (`diag · 1e-5`, floored at 1e-7).
fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    let diag = remus_operations::measure::solid_bounding_box(topo, s)
        .map(|b| (b.max - b.min).length())
        .unwrap();
    for d in [0.1, 0.01, 1e-4, (diag * 1e-5).max(1e-7)] {
        assert_watertight_at(topo, s, d, what);
    }
}

/// The measured volume equals the exact Gauss integral, here and after a
/// rigid translation (the doubled-boundary detector).
fn assert_measured_and_translation_invariant(topo: &Topology, s: SolidId, gauss: f64, what: &str) {
    let mut moved = topo.clone();
    remus_operations::transform::transform_solid(
        &mut moved,
        s,
        &Mat4::translation(13.0, -7.0, 5.0),
    )
    .unwrap();
    for d in [0.1, 1e-4] {
        for (t, place) in [(topo, "in place"), (&moved, "translated")] {
            let v = solid_volume(t, s, d).unwrap();
            assert!(
                (v - gauss).abs() / gauss <= 1e-4,
                "{what}: measured {v:.9} ({place}, d={d}) vs Gauss {gauss:.9}"
            );
        }
    }
    let gm = gauss_volume(&moved, s);
    assert!(
        (gm - gauss).abs() / gauss <= 1e-9,
        "{what}: Gauss moved {gauss:.12} -> {gm:.12}"
    );
}

/// Every face is an exact plane or cylinder, the result is compact, and the
/// cylinder carrier survives.
fn assert_cylinder_census(topo: &Topology, s: SolidId, what: &str) {
    let faces = remus_topology::explorer::solid_faces(topo, s).unwrap();
    let mut cylinders = 0;
    for &f in &faces {
        match topo.face(f).unwrap().surface() {
            FaceSurface::Cylinder(_) => cylinders += 1,
            FaceSurface::Plane { .. } => {}
            other => panic!(
                "{what}: non-analytic or unexpected face {}",
                other.type_tag()
            ),
        }
    }
    assert!(cylinders >= 1, "{what}: cylinder carrier lost");
    assert!(faces.len() <= 20, "{what}: {} faces", faces.len());
}

/// Every seam placement gives an exact analytic B-Rep for all three
/// operations, with the closed-form volume, strict validity, watertight
/// meshes at every deflection and translation-invariant measurement. Before
/// the fix, rot = 0.3 and 2.0 refused all three legs.
#[test]
fn b52_box_cylinder_notch_every_seam_every_op() {
    for rot in seam_rotations() {
        for op in OPS {
            let what = format!("box-cylinder {op:?} rot={rot:.4}");
            let mut t = Topology::new();
            let (a, b) = build_cylinder(&mut t, rot);
            let s = exact_boolean(&mut t, op, a, b, &what);
            let g = gauss_volume(&t, s);
            let want = expected_cylinder_volume(op);
            assert!(
                (g - want).abs() / want <= 1e-6,
                "{what}: Gauss {g:.9} vs closed form {want:.9}"
            );
            assert_cylinder_census(&t, s, &what);
            assert_strict_valid(&t, s, &what);
            assert_watertight(&t, s, &what);
            assert_measured_and_translation_invariant(&t, s, g, &what);
        }
    }
}

/// Independent identities between separately built results at every seam:
/// inclusion–exclusion (fuse = box + cylinder − inter) and the cut
/// complement (cut + inter = box, fuse = cut + cylinder).
#[test]
fn b52_box_cylinder_notch_identities_hold_at_every_seam() {
    let v_cyl = 4.0 * PI;
    for rot in seam_rotations() {
        let what = format!("box-cylinder rot={rot:.4}");
        let run = |op: BooleanOp| {
            let mut t = Topology::new();
            let (a, b) = build_cylinder(&mut t, rot);
            let s = exact_boolean(&mut t, op, a, b, &format!("{what} {op:?}"));
            gauss_volume(&t, s)
        };
        let (gf, gc, gi) = (
            run(BooleanOp::Fuse),
            run(BooleanOp::Cut),
            run(BooleanOp::Intersect),
        );
        assert!(
            (gf - (V_BOX + v_cyl - gi)).abs() / gf <= 1e-6,
            "{what}: fuse {gf:.9} != box + cylinder − inter {gi:.9}"
        );
        assert!(
            (gc + gi - V_BOX).abs() / V_BOX <= 1e-6,
            "{what}: cut {gc:.9} + inter {gi:.9} != box"
        );
        assert!(
            (gf - (gc + v_cyl)).abs() / gf <= 1e-6,
            "{what}: fuse {gf:.9} != cut {gc:.9} + cylinder"
        );
    }
}

/// Ray-cast material probes at the two seam placements that refused: the
/// below-box cylinder extent and bottom rim (dropped by the refusing fuse),
/// the notch (box ∩ cylinder), the box beyond the cylinder, and clear
/// outside points. Coordinates are unit-scale base placement; the
/// scale × placement tests carry them through the same map as the operands.
const MATERIAL_PROBES: [(
    [f64; 3],
    PointClassification,
    PointClassification,
    PointClassification,
); 10] = [
    (
        [0.5, -1.5, -0.25],
        PointClassification::Inside,
        PointClassification::Outside,
        PointClassification::Outside,
    ),
    (
        [0.5, -1.5, -0.45],
        PointClassification::Inside,
        PointClassification::Outside,
        PointClassification::Outside,
    ),
    (
        [2.3, -1.5, -0.4],
        PointClassification::Inside,
        PointClassification::Outside,
        PointClassification::Outside,
    ),
    (
        [0.5, 0.2, 0.25],
        PointClassification::Inside,
        PointClassification::Outside,
        PointClassification::Inside,
    ),
    (
        [1.0, 0.1, 0.4],
        PointClassification::Inside,
        PointClassification::Outside,
        PointClassification::Inside,
    ),
    (
        [0.5, 0.2, 0.75],
        PointClassification::Inside,
        PointClassification::Inside,
        PointClassification::Outside,
    ),
    (
        [2.0, 0.8, 0.8],
        PointClassification::Inside,
        PointClassification::Inside,
        PointClassification::Outside,
    ),
    (
        [0.5, -1.5, -0.6],
        PointClassification::Outside,
        PointClassification::Outside,
        PointClassification::Outside,
    ),
    (
        [0.5, -3.0, 0.7],
        PointClassification::Outside,
        PointClassification::Outside,
        PointClassification::Outside,
    ),
    (
        [3.0, 2.0, 0.5],
        PointClassification::Outside,
        PointClassification::Outside,
        PointClassification::Outside,
    ),
];

#[test]
fn b52_box_cylinder_notch_material_probes() {
    let opts = ClassifyOptions::default();
    for rot in [0.3, 2.0] {
        for (col, op) in OPS.into_iter().enumerate() {
            let mut t = Topology::new();
            let (a, b) = build_cylinder(&mut t, rot);
            let s = exact_boolean(&mut t, op, a, b, &format!("{op:?} rot={rot}"));
            // (point, fuse, cut, intersect)
            for (coords, f, c, i) in MATERIAL_PROBES {
                let p = Point3::new(coords[0], coords[1], coords[2]);
                let want = [f, c, i][col];
                let got = classify_point(&t, s, p, &opts).unwrap();
                assert_eq!(got, want, "{op:?} rot={rot}: probe {p:?}");
            }
        }
    }
}

/// Seam angles at which the cone's seam generator crosses a SLANTED side of
/// the notch. The frustum's notch sides are hyperbolas (the y = 0 and x = 0
/// planes), sweeping the seam angle over
/// `[atan2(1.5, √(2.5² − 2.25)), atan2(1.5, √(2² − 2.25))]` ≈ [0.6435, 0.8481]
/// (y = 0 side, top rim down to the box floor at z = 0) and
/// `[atan2(√(2.5² − 0.25), −0.5), atan2(√(2² − 0.25), −0.5)]` ≈
/// [1.7722, 1.8235] (x = 0 side). A cylinder's notch sides are generators,
/// parallel to its seam, so the cylinder has no such band.
fn cone_seam_crosses_slanted_side(rot: f64) -> bool {
    let y_side = (
        1.5_f64.atan2(2.5_f64.mul_add(2.5, -2.25).sqrt()),
        1.5_f64.atan2(2.0_f64.mul_add(2.0, -2.25).sqrt()),
    );
    let x_side = (
        2.5_f64.mul_add(2.5, -0.25).sqrt().atan2(-0.5),
        2.0_f64.mul_add(2.0, -0.25).sqrt().atan2(-0.5),
    );
    let a = rot.rem_euclid(std::f64::consts::TAU);
    (y_side.0..=y_side.1).contains(&a) || (x_side.0..=x_side.1).contains(&a)
}

/// Exact-result battery for the cone twin at the given seam placements:
/// all three legs exact and analytic, strict-valid, watertight at 0.01,
/// the intersect volume seam-invariant, and inclusion–exclusion and
/// the cut complement holding.
fn check_cone_seams(rotations: &[f64]) {
    let v_cone = frustum_volume(1.5, 2.5, 1.0);
    let mut reference_inter: Option<f64> = None;
    for &rot in rotations {
        let mut g = [0.0; 3];
        for (k, op) in OPS.into_iter().enumerate() {
            let what = format!("box-cone {op:?} rot={rot:.4}");
            let mut t = Topology::new();
            let (a, b) = build_cone(&mut t, rot);
            let s = exact_boolean(&mut t, op, a, b, &what);
            g[k] = gauss_volume(&t, s);
            let faces = remus_topology::explorer::solid_faces(&t, s).unwrap();
            assert!(
                faces.iter().all(|&f| matches!(
                    t.face(f).unwrap().surface(),
                    FaceSurface::Plane { .. } | FaceSurface::Cone(_)
                )),
                "{what}: non-analytic face"
            );
            assert!(faces.len() <= 20, "{what}: {} faces", faces.len());
            assert_strict_valid(&t, s, &what);
            assert_watertight_at(&t, s, 0.01, &what);
        }
        let [gf, gc, gi] = g;
        let what = format!("box-cone rot={rot:.4}");
        let vi = *reference_inter.get_or_insert(gi);
        assert!(
            (gi - vi).abs() / vi <= 1e-6,
            "{what}: inter {gi:.9} vs first seam {vi:.9}"
        );
        assert!(
            (gf - (V_BOX + v_cone - gi)).abs() / gf <= 1e-6,
            "{what}: fuse {gf:.9} != box + cone − inter"
        );
        assert!(
            (gc + gi - V_BOX).abs() / V_BOX <= 1e-6,
            "{what}: cut {gc:.9} + inter {gi:.9} != box"
        );
    }
}

/// The B32 cone twin across the same seam placements (all but π/4, which
/// falls in a slanted-side band) and all three operations. B32 pinned only
/// the fuse leg, at eight rotations; this extends the sweep to every leg.
#[test]
fn b52_box_cone_notch_every_seam_every_op() {
    let rotations: Vec<f64> = seam_rotations()
        .into_iter()
        .filter(|&r| !cone_seam_crosses_slanted_side(r))
        .collect();
    assert_eq!(rotations.len(), 17, "only π/4 falls in a slanted-side band");
    check_cone_seams(&rotations);
}

/// Ready-repro, discovered extending the B52 sweep to the cone twin
/// (2026-09-24): when the cone's seam generator crosses a slanted notch side
/// (see [`cone_seam_crosses_slanted_side`]), all three legs refuse
/// `ExactOnlyUnattainable` — 24 of 768 cells in a 256-rotation sweep, with
/// and without the B52 face-box fix. Not the B52 root: the FF section set is
/// identical to a passing seam's. The side hyperbola crosses the seam
/// mid-band, but the seam-meridian section split runs only for WINDING
/// chains, and a periodic lateral's seam edge is not expanded through its EF
/// pave, so the lateral's splitter receives the crossing section and the seam
/// both unsplit; it closes the notch the long way round the top rim and the
/// kept lateral and bottom cap are lost (raw fuse V−E+F = 1). Splitting the
/// crossing section at the seam alone keeps the cap but merges the notch into
/// the kept piece (one 20-edge loop), so the fix needs the splitter to trace a
/// notch region that straddles the seam. Acceptance target encoded below.
#[test]
#[ignore = "open: box-cone notch refuses when the cone seam crosses a slanted notch side (found closing B52)"]
fn b52_box_cone_seam_crossing_slanted_side() {
    let rotations = [0.7, std::f64::consts::FRAC_PI_4, 1.8];
    assert!(rotations.iter().all(|&r| cone_seam_crosses_slanted_side(r)));
    check_cone_seams(&rotations);
}

// ---------------------------------------------------------------------------
// Scale × rigid-placement acceptance (2026-09-26).
//
// The unit-scale tests above cover one scale and one placement. Seam
// invariance and rigid-motion invariance are DISTINCT claims, and this
// section covers both as independent matrix dimensions:
//
// * seam (`rot`): spins the cylinder about its own axis. The solid is
//   unchanged; only its seam moves. This is the B52 dimension.
// * rigid placement: a non-axis-aligned rotation plus translation applied
//   to BOTH operands together (and to every probe through the same map).
//   The relative arrangement — and the seam under test — is unchanged; the
//   physical assembly moves. This catches axis-aligned assumptions and
//   world-origin-referenced arithmetic (see B41, B56, B58).
//
// Scales are 1e-3, 1 and 1e3 with dimensionally scaled dimensions, base
// placement offsets and rigid translations. Expectations reuse the
// independent clipped-circle/rectangle area-times-height closed form
// (`inter_closed_form`), scaled dimensionally (lengths × s, volumes × s³).
//
// Tolerance accounting (no production tolerance is touched by this file):
// * FIXED physical quantities: the engine's linear (1e-7) and angular
//   (1e-12) tolerances are absolute model units and stay put; the test
//   never widens a band to accommodate a scale.
// * SCALED test quantities: every dimension, placement offset, probe
//   coordinate and tessellation deflection is an absolute length and is
//   scaled with the body (× s); expected volumes scale × s³.
// * SCALE-INVARIANT comparisons: every pass band is a dimensionless
//   relative ratio (1e-6 Gauss-vs-closed-form, 1e-4 measured-vs-Gauss,
//   1e-6 inclusion–exclusion) and is identical at every scale.
//
// Matrix denominator: 3 scales × 2 placements × 18 seams × 3 ops = 324
// exact boolean cells, plus 3 × 2 × 2 seams × 3 ops = 36 probe cells.
// Every cell drives the PUBLIC path (`boolean_with_context` + `ExactOnly`,
// never raw GFA), and every result keeps its analytic carriers (planes +
// cylinders only, ≤ 20 faces), passes strict validation on BOTH validators
// (oriented closed topology), and meshes welded-manifold (zero boundary,
// zero non-manifold edges) at scale-relative coarse/mid/fine deflections
// plus the bbox-derived harness deflection.

/// Scales of the acceptance matrix.
const ACCEPT_SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

/// Whole-body rigid placement for a scale: a non-axis-aligned rotation (x-
/// then y-, so the composed axis is not a coordinate axis) plus a
/// translation scaled with the body, keeping the off-origin distance
/// proportional across scales.
fn rigid_placement(scale: f64) -> Mat4 {
    Mat4::translation(13.0 * scale, -7.0 * scale, 5.0 * scale)
        * Mat4::rotation_y(0.5)
        * Mat4::rotation_x(0.7)
}

/// The cylinder-notch pair rebuilt at `scale` with the cylinder seam at
/// `rot`, then optionally carried through a whole-body rigid placement
/// applied to BOTH operands together.
fn build_scaled_cylinder(
    topo: &mut Topology,
    scale: f64,
    rot: f64,
    rigid: Option<&Mat4>,
) -> (SolidId, SolidId) {
    let m = Mat4::translation(0.5 * scale, -1.5 * scale, -0.5 * scale) * Mat4::rotation_z(rot);
    let a = make_box(topo, 2.5 * scale, 1.0 * scale, 1.0 * scale).expect("box");
    let b = make_cylinder(topo, 2.0 * scale, 1.0 * scale).expect("cylinder");
    remus_operations::transform::transform_solid(topo, b, &m).expect("place");
    if let Some(p) = rigid {
        remus_operations::transform::transform_solid(topo, a, p).expect("rigid box");
        remus_operations::transform::transform_solid(topo, b, p).expect("rigid cylinder");
    }
    (a, b)
}

/// Dimensionally scaled volume expectations: the unit-scale closed form and
/// operand volumes times s³.
fn expected_scaled_cylinder_volume(op: BooleanOp, scale: f64) -> f64 {
    expected_cylinder_volume(op) * scale.powi(3)
}

fn scaled_operand_volumes(scale: f64) -> (f64, f64) {
    (V_BOX * scale.powi(3), 4.0 * PI * scale.powi(3))
}

/// Welded-manifold meshes at scale-relative coarse/mid/fine deflections plus
/// the bbox-derived proptest harness deflection. Deflections are absolute
/// model units, so each is scaled with the body.
fn assert_watertight_scaled(topo: &Topology, s: SolidId, scale: f64, what: &str) {
    let diag = remus_operations::measure::solid_bounding_box(topo, s)
        .map(|b| (b.max - b.min).length())
        .unwrap();
    for d in [
        0.1 * scale,
        0.01 * scale,
        1e-4 * scale,
        (diag * 1e-5).max(1e-7),
    ] {
        assert_watertight_at(topo, s, d, what);
    }
}

/// The tessellation-measured volume equals the exact Gauss integral at two
/// scale-relative deflections. Coarse requests clamp to `diag · 5e-5`
/// inside `solid_volume`, so the two legs stay genuinely different.
/// Rigidly placed cells ARE translated bodies, so this subsumes the
/// translation-invariance detector at every scale.
fn assert_measured_scaled(topo: &Topology, s: SolidId, scale: f64, gauss: f64, what: &str) {
    for d in [0.1 * scale, 1e-4 * scale] {
        let v = solid_volume(topo, s, d).unwrap();
        assert!(
            (v - gauss).abs() / gauss <= 1e-4,
            "{what}: measured {v:.9} (d={d}) vs Gauss {gauss:.9}"
        );
    }
}

/// Scale × rigid-placement acceptance over the full seam matrix: every cell
/// exact and analytic with the closed-form volume, strict-valid on both
/// validators, welded-manifold at every deflection, measured against Gauss,
/// and satisfying inclusion–exclusion and the cut complement between its
/// three separately built legs.
#[test]
fn b52_box_cylinder_notch_scale_and_placement_every_seam_every_op() {
    for &scale in &ACCEPT_SCALES {
        let rigid = rigid_placement(scale);
        for (place_name, rigid_opt) in [("origin", None), ("rigid", Some(&rigid))] {
            for rot in seam_rotations() {
                let mut legs = [0.0; 3];
                for (k, op) in OPS.into_iter().enumerate() {
                    let what = format!(
                        "box-cylinder {op:?} scale={scale} place={place_name} rot={rot:.4}"
                    );
                    let mut t = Topology::new();
                    let (a, b) = build_scaled_cylinder(&mut t, scale, rot, rigid_opt);
                    let s = exact_boolean(&mut t, op, a, b, &what);
                    let g = gauss_volume(&t, s);
                    legs[k] = g;
                    let want = expected_scaled_cylinder_volume(op, scale);
                    assert!(
                        (g - want).abs() / want <= 1e-6,
                        "{what}: Gauss {g:.9} vs closed form {want:.9}"
                    );
                    assert_cylinder_census(&t, s, &what);
                    assert_strict_valid(&t, s, &what);
                    assert_watertight_scaled(&t, s, scale, &what);
                    assert_measured_scaled(&t, s, scale, g, &what);
                }
                let (v_box, v_cyl) = scaled_operand_volumes(scale);
                let [gf, gc, gi] = legs;
                let what = format!("box-cylinder scale={scale} place={place_name} rot={rot:.4}");
                assert!(
                    (gf - (v_box + v_cyl - gi)).abs() / gf <= 1e-6,
                    "{what}: fuse {gf:.9} != box + cylinder − inter {gi:.9}"
                );
                assert!(
                    (gc + gi - v_box).abs() / v_box <= 1e-6,
                    "{what}: cut {gc:.9} + inter {gi:.9} != box"
                );
                assert!(
                    (gf - (gc + v_cyl)).abs() / gf <= 1e-6,
                    "{what}: fuse {gf:.9} != cut {gc:.9} + cylinder"
                );
            }
        }
    }
}

/// Material occupancy across the scale × placement matrix at the two
/// historically refusing seams: every base probe scaled with the body and
/// carried through the same rigid map as the operands.
#[test]
fn b52_box_cylinder_notch_material_probes_scaled_and_placed() {
    let opts = ClassifyOptions::default();
    for &scale in &ACCEPT_SCALES {
        let rigid = rigid_placement(scale);
        for (place_name, rigid_opt) in [("origin", None), ("rigid", Some(&rigid))] {
            let map = |coords: [f64; 3]| {
                let q = Point3::new(coords[0] * scale, coords[1] * scale, coords[2] * scale);
                match rigid_opt {
                    Some(p) => p.mul_point(q),
                    None => q,
                }
            };
            for rot in [0.3, 2.0] {
                for (col, op) in OPS.into_iter().enumerate() {
                    let mut t = Topology::new();
                    let (a, b) = build_scaled_cylinder(&mut t, scale, rot, rigid_opt);
                    let s = exact_boolean(
                        &mut t,
                        op,
                        a,
                        b,
                        &format!("{op:?} scale={scale} place={place_name} rot={rot}"),
                    );
                    // (point, fuse, cut, intersect)
                    for (coords, f, c, i) in MATERIAL_PROBES {
                        let p = map(coords);
                        let want = [f, c, i][col];
                        let got = classify_point(&t, s, p, &opts).unwrap();
                        assert_eq!(
                            got, want,
                            "{op:?} scale={scale} place={place_name} rot={rot}: probe {p:?}"
                        );
                    }
                }
            }
        }
    }
}
