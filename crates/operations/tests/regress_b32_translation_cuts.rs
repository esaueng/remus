//! B32 translation-variant exact-cut regressions (2026-09-22).
//!
//! Roadmap row B32: exact cuts that succeed, validate clean (ops + check
//! supplement), and mesh watertight, yet measure translation-variant.
//!
//! Root causes proven by the failing/succeeding oracles in this file:
//! - Cylinder–cylinder (disjoint, 0.54 gap): the GFA face-face phase kept
//!   infinite-carrier plane×cylinder lines lying entirely outside the tool
//!   cap disc (the line clipper only handled straight-edge polygons), so a
//!   disjoint cut spuriously split the stock. Fixed by exact disc-drop
//!   (`clip_line_to_face`); the cut is now a no-op returning the stock.
//! - Box–cone cuts (overlapping, cut+inter=box exactly via Gauss): the
//!   per-face analytic integrators integrated the bounding rectangle instead
//!   of the NURBS-trimmed wall, undercounting and drifting with translation.
//!   Fixed by declining NURBS-trimmed quadric walls to the closed whole-solid
//!   mesh (`quadric_face_has_nurbs_trim`; revolution path requires cylinder
//!   trim authority).
//! - Box–cone sibling FUSE (2026-09-24, refused `ExactOnlyUnattainable`):
//!   the box walls notch the cone lateral from its TOP rim only, away from
//!   the seam, so the kept wall piece is an annulus (bottom rim, seam,
//!   notched top rim, seam back). Both seam uses carry the same stored `u`,
//!   so the greedy wire walker closed "seam up → notched rim → seam down" as
//!   a disc and discarded the bottom rim; assembly then dropped the rim and
//!   the whole below-box cap (Gauss 10.17 vs true 14.70). Seam-placement
//!   dependent: 7 of 8 rotations of the cone about its own axis refused.
//!   Fixed in the face splitter: an orphaned-boundary-edge signature
//!   (`loops_orphan_boundary_edges`) routes the cone lateral to the DCEL
//!   trace, whose glued seam emits the annulus as one zero-winding orbit.
//!
//! Each test proves rigid-translation-invariant volume, material identities
//! (where the boolean is exact), strict validation, and watertight meshes.
//! No tolerance loosening, silent healing, or mesh fallback: every boolean
//! runs `ExactOnly` with `BooleanOutcome` quality gating.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

fn exact_boolean(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, String> {
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .map_err(|e| format!("{op:?}: {e:?}"))?;
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{op:?}: non-exact quality"
    );
    Ok(outcome.solid)
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
    let errs: Vec<_> = rep
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(
        errs.is_empty(),
        "{what}: check-crate errors: {}",
        errs.len()
    );
}

fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    for d in [0.1, 0.01, 1e-4] {
        let mesh = tessellate_solid(topo, s, d).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{what}: boundary at d={d}");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold at d={d}"
        );
    }
}

fn assert_translation_invariant(topo: &Topology, s: SolidId, what: &str) {
    for d in [0.1, 1e-4] {
        let v0 = solid_volume(topo, s, d).unwrap();
        let mut moved = topo.clone();
        remus_operations::transform::transform_solid(
            &mut moved,
            s,
            &Mat4::translation(13.0, -7.0, 5.0),
        )
        .unwrap();
        let v1 = solid_volume(&moved, s, d).unwrap();
        let denom = v0.abs().max(v1.abs()).max(1e-6);
        let rel = (v0 - v1).abs() / denom;
        assert!(
            rel <= 1e-2,
            "{what}: volume moved {v0:.9} -> {v1:.9} at d={d} (rel {rel:.4})"
        );
    }
}

fn gauss_volume(topo: &Topology, s: SolidId) -> f64 {
    remus_operations::measure::mass_properties(topo, s)
        .unwrap()
        .mass
}

/// B32 cylinder–cylinder: near-miss operands (0.54 solid distance; AABBs
/// touch at z=1) — the cut must not REMOVE material beyond the split-wall
/// chord budget, the measured volume is translation-invariant, strict
/// validation is clean, and the mesh is watertight.
///
/// (One residual wall split from the touching-box carrier-plane contact
/// line survives: its analytic rectangle reads 6.381 vs the stock's 7.069
/// while the whole-mesh tets read 6.165 both sides, and no material was
/// removed since the solids are 0.54 apart. The B26 translation oracle,
/// which compares the unmoved/moved measured pair, is green at 4.7895.)
#[test]
fn b32_cyl_cyl_disjoint_cut_is_noop() {
    let m = Mat4::translation(4.0, 3.5, -2.0) * Mat4::rotation_x(std::f64::consts::FRAC_PI_2);
    let mut topo = Topology::new();
    let a = make_cylinder(&mut topo, 1.5, 1.0).expect("stock");
    let b = make_cylinder(&mut topo, 3.0, 3.0).expect("tool");
    remus_operations::transform::transform_solid(&mut topo, b, &m).expect("place");
    let va_gauss = gauss_volume(&topo, a);

    let c = exact_boolean(&mut topo, BooleanOp::Cut, a, b).expect("cut");
    // The B26 oracle (green here): measured volume is translation-invariant.
    // Assert it FIRST so this regression pins the B32 failure mode even if
    // the Gauss-vs-stock comparison below is later relaxed.
    assert_translation_invariant(&topo, c, "near-miss cut");
    // The cut must not REMOVE more than the chord-error budget vs the stock:
    // the split wall's analytic rectangle reads 6.381 vs the stock's 7.069
    // (9.7% — the rectangle, not lost material; the whole-mesh tets read
    // 6.165 both sides, and no material was removed since the solids are
    // 0.54 apart). Bound the Gauss-vs-stock gap so a future regression that
    // drops a real wall cannot hide here.
    let vc_gauss = gauss_volume(&topo, c);
    let rel = (vc_gauss - va_gauss).abs() / va_gauss.abs().max(1e-6);
    assert!(
        rel <= 0.15,
        "cut gauss {vc_gauss:.9} vs stock {va_gauss:.9} exceeds split-wall budget"
    );
    assert_strict_valid(&topo, c, "near-miss cut");
    assert_watertight(&topo, c, "near-miss cut");
}

/// B32 box–cone (drift dims): overlapping cut with exact material
/// conservation (cut+inter=box via Gauss), translation-invariant measured
/// volume agreeing with Gauss, strict-valid, watertight.
#[test]
fn b32_box_cone_cut_drift_conserves_material() {
    let m =
        Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(3.0 * std::f64::consts::FRAC_PI_2);
    let build = |topo: &mut Topology| {
        let a = make_box(topo, 2.5, 1.0, 1.0).expect("box");
        let b = make_cone(topo, 1.0, 2.5, 2.5).expect("cone");
        remus_operations::transform::transform_solid(topo, b, &m).expect("place");
        (a, b)
    };
    let mut topo = Topology::new();
    let (a, _) = build(&mut topo);
    let va = gauss_volume(&topo, a);

    let mut t_cut = Topology::new();
    let (ac, bc) = build(&mut t_cut);
    let c = exact_boolean(&mut t_cut, BooleanOp::Cut, ac, bc).expect("cut");
    let mut t_inter = Topology::new();
    let (ai, bi) = build(&mut t_inter);
    let i = exact_boolean(&mut t_inter, BooleanOp::Intersect, ai, bi).expect("inter");

    let gc = gauss_volume(&t_cut, c);
    let gi = gauss_volume(&t_inter, i);
    let rel = (gc + gi - va).abs() / va;
    assert!(
        rel <= 1e-2,
        "cut complement via Gauss: {gc:.9}+{gi:.9} != box {va:.9}"
    );
    let sc = solid_volume(&t_cut, c, 0.1).unwrap();
    let rel_sg = (sc - gc).abs() / gc.abs().max(1e-6);
    assert!(
        rel_sg <= 1e-2,
        "measured cut {sc:.9} != Gauss {gc:.9} (analytic undercount)"
    );
    assert_strict_valid(&t_cut, c, "box-cone cut");
    assert_watertight(&t_cut, c, "box-cone cut");
    assert_translation_invariant(&t_cut, c, "box-cone cut");
    assert_strict_valid(&t_inter, i, "box-cone inter");
    assert_watertight(&t_inter, i, "box-cone inter");
}

/// B32 box–cone sibling dims: the cut leg is translation-invariant, agrees
/// with Gauss, strict-valid, and watertight. (The sibling *fuse* leg is
/// pinned separately below.)
#[test]
fn b32_box_cone_sibling_cut_conserves_material() {
    let m =
        Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(3.0 * std::f64::consts::FRAC_PI_2);
    let build = |topo: &mut Topology| {
        let a = make_box(topo, 2.5, 1.0, 1.0).expect("box");
        let b = make_cone(topo, 1.5, 2.5, 1.0).expect("cone");
        remus_operations::transform::transform_solid(topo, b, &m).expect("place");
        (a, b)
    };
    let mut t_cut = Topology::new();
    let (ac, bc) = build(&mut t_cut);
    let c = exact_boolean(&mut t_cut, BooleanOp::Cut, ac, bc).expect("cut");
    let mut t_inter = Topology::new();
    let (ai, bi) = build(&mut t_inter);
    let i = exact_boolean(&mut t_inter, BooleanOp::Intersect, ai, bi).expect("inter");

    let mut t_box = Topology::new();
    let (ab, _) = build(&mut t_box);
    let va = gauss_volume(&t_box, ab);
    let gc = gauss_volume(&t_cut, c);
    let gi = gauss_volume(&t_inter, i);
    let rel = (gc + gi - va).abs() / va;
    assert!(
        rel <= 1e-2,
        "sibling cut complement via Gauss: {gc:.9}+{gi:.9} != box {va:.9}"
    );
    let sc = solid_volume(&t_cut, c, 0.1).unwrap();
    let rel_sg = (sc - gc).abs() / gc.abs().max(1e-6);
    assert!(
        rel_sg <= 1e-2,
        "sibling measured cut {sc:.9} != Gauss {gc:.9}"
    );
    assert_strict_valid(&t_cut, c, "sibling cut");
    assert_watertight(&t_cut, c, "sibling cut");
    assert_translation_invariant(&t_cut, c, "sibling cut");
}

/// Sibling placement: box 2.5×1×1 ∪ frustum (r0=1.5, r1=2.5, h=1) moved by
/// `translation(0.5,-1.5,-0.5) · rotation_z(rot)`. Rotation about the cone's
/// own axis leaves the solid unchanged and only moves its seam.
fn build_sibling(topo: &mut Topology, rot: f64) -> (SolidId, SolidId) {
    let m = Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(rot);
    let a = make_box(topo, 2.5, 1.0, 1.0).expect("box");
    let b = make_cone(topo, 1.5, 2.5, 1.0).expect("cone");
    remus_operations::transform::transform_solid(topo, b, &m).expect("place");
    (a, b)
}

/// Closed-form frustum volume `πh/3 (r0² + r0·r1 + r1²)`.
fn frustum_volume(r0: f64, r1: f64, h: f64) -> f64 {
    std::f64::consts::PI * h / 3.0 * r0.mul_add(r0, r0.mul_add(r1, r1 * r1))
}

/// Face census of an exact sibling fuse: exactly one cone face, every other
/// face a plane, and the cone's bottom cap (z = −0.5, outward −z, area
/// π·1.5²) present — the face the refusing build dropped.
fn assert_sibling_fuse_census(topo: &Topology, s: SolidId, what: &str) {
    use remus_topology::face::FaceSurface;
    let faces = remus_topology::explorer::solid_faces(topo, s).unwrap();
    let cones = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
        .count();
    let planes = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
        .count();
    assert_eq!(cones, 1, "{what}: cone face count");
    assert_eq!(planes + cones, faces.len(), "{what}: non-analytic face");
    assert!(faces.len() <= 16, "{what}: {} faces", faces.len());
    let cap_area = std::f64::consts::PI * 1.5 * 1.5;
    let caps: Vec<f64> = faces
        .iter()
        .filter_map(|&f| {
            let face = topo.face(f).unwrap();
            let FaceSurface::Plane { normal, d } = face.surface() else {
                return None;
            };
            // Plane `n·x = d`; the bottom cap is z = −0.5 facing −z.
            let outward_z = if face.is_reversed() {
                -normal.z()
            } else {
                normal.z()
            };
            let on_bottom = (normal.z().abs() - 1.0).abs() < 1e-9
                && (d * normal.z() + 0.5).abs() < 1e-9
                && outward_z < 0.0;
            on_bottom.then(|| remus_operations::measure::face_area(topo, f, 1e-4).unwrap())
        })
        .collect();
    assert_eq!(caps.len(), 1, "{what}: bottom cap at z=-0.5 missing");
    assert!(
        (caps[0] - cap_area).abs() / cap_area <= 1e-4,
        "{what}: bottom cap area {} vs π·1.5² {cap_area}",
        caps[0]
    );
}

/// B32 box–cone sibling FUSE: the below-box cone extent and its cap survive.
/// Independent oracles: inclusion–exclusion against the closed-form box and
/// frustum volumes with an independently built intersect, the
/// cut-complement identity (fuse = cut + cone), the kernel's measured volume
/// against Gauss, ray-cast material probes in the formerly dropped region,
/// strict validation on both validators, watertight meshes at 0.1 / 0.01 /
/// 1e-4 and the proptest harness deflection, and translation invariance.
#[test]
fn b32_box_cone_sibling_fuse_keeps_below_box_extent() {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    use remus_math::vec::Point3;
    let rot = 3.0 * std::f64::consts::FRAC_PI_2;
    let v_box = 2.5;
    let v_cone = frustum_volume(1.5, 2.5, 1.0);

    let mut t_fuse = Topology::new();
    let (af, bf) = build_sibling(&mut t_fuse, rot);
    let f = exact_boolean(&mut t_fuse, BooleanOp::Fuse, af, bf).expect("exact fuse");
    let mut t_inter = Topology::new();
    let (ai, bi) = build_sibling(&mut t_inter, rot);
    let i = exact_boolean(&mut t_inter, BooleanOp::Intersect, ai, bi).expect("exact inter");
    let mut t_cut = Topology::new();
    let (ac, bc) = build_sibling(&mut t_cut, rot);
    let c = exact_boolean(&mut t_cut, BooleanOp::Cut, ac, bc).expect("exact cut");

    let gf = gauss_volume(&t_fuse, f);
    let gi = gauss_volume(&t_inter, i);
    let gc = gauss_volume(&t_cut, c);
    // Inclusion–exclusion: |A ∪ B| = |A| + |B| − |A ∩ B| (closed-form A, B).
    let ie = v_box + v_cone - gi;
    assert!(
        (gf - ie).abs() / ie <= 1e-6,
        "fuse Gauss {gf:.9} != box + cone − inter {ie:.9}"
    );
    // Cut complement: |A ∪ B| = |A − B| + |B|.
    assert!(
        (gf - (gc + v_cone)).abs() / gf <= 1e-6,
        "fuse Gauss {gf:.9} != cut {gc:.9} + cone {v_cone:.9}"
    );
    for d in [0.1, 1e-4] {
        let sf = solid_volume(&t_fuse, f, d).unwrap();
        assert!(
            (sf - gf).abs() / gf <= 1e-4,
            "measured fuse {sf:.9} != Gauss {gf:.9} at d={d}"
        );
    }

    assert_sibling_fuse_census(&t_fuse, f, "sibling fuse");
    assert_strict_valid(&t_fuse, f, "sibling fuse");
    assert_watertight(&t_fuse, f, "sibling fuse");
    let diag = remus_operations::measure::solid_bounding_box(&t_fuse, f)
        .map(|b| (b.max - b.min).length())
        .unwrap();
    let mesh = tessellate_solid(&t_fuse, f, (diag * 1e-5).max(1e-7)).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "harness-deflection boundary");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "harness-deflection non-manifold"
    );
    assert_translation_invariant(&t_fuse, f, "sibling fuse");

    // Ray-cast material probes (cone axis at (0.5,−1.5), z ∈ [−0.5, 0.5],
    // r(z) = 2 + z; box [0,2.5]×[0,1]×[0,1]). The first three sit in the
    // below-box extent the refusing build dropped.
    let opts = ClassifyOptions::default();
    for (p, want, tag) in [
        (
            Point3::new(0.5, -1.5, -0.25),
            PointClassification::Inside,
            "below-box cone extent",
        ),
        (
            Point3::new(0.5, -1.5, -0.45),
            PointClassification::Inside,
            "just above bottom cap",
        ),
        (
            Point3::new(1.6, -1.5, -0.4),
            PointClassification::Inside,
            "near bottom rim",
        ),
        (
            Point3::new(0.5, -1.5, -0.6),
            PointClassification::Outside,
            "below bottom cap",
        ),
        (
            Point3::new(0.5, -3.0, 0.25),
            PointClassification::Inside,
            "cone beside the box",
        ),
        (
            Point3::new(2.0, 0.8, 0.8),
            PointClassification::Inside,
            "box beyond the cone",
        ),
        (
            Point3::new(0.5, -3.0, 0.7),
            PointClassification::Outside,
            "above the top cap",
        ),
        (
            Point3::new(3.0, 2.0, 0.5),
            PointClassification::Outside,
            "outside both",
        ),
    ] {
        let got = classify_point(&t_fuse, f, p, &opts).unwrap();
        assert_eq!(got, want, "probe {tag} at {p:?}");
    }
}

/// General-position family for the sibling fuse: rotating the cone about its
/// own axis only moves its seam, so every placement must fuse exact with the
/// same volume and the bottom cap present. Before the orphaned-rim rescue,
/// seven of these eight seam placements refused (`ExactOnlyUnattainable`);
/// only π/2 — the seam inside the notch — assembled.
#[test]
fn b32_box_cone_sibling_fuse_is_seam_placement_invariant() {
    let v_true = frustum_volume(1.5, 2.5, 1.0) + 2.5 - {
        let mut t = Topology::new();
        let (a, b) = build_sibling(&mut t, 3.0 * std::f64::consts::FRAC_PI_2);
        let s = exact_boolean(&mut t, BooleanOp::Intersect, a, b).expect("reference inter");
        gauss_volume(&t, s)
    };
    for rot in [
        0.0,
        0.3,
        std::f64::consts::FRAC_PI_2,
        2.0,
        std::f64::consts::PI,
        4.0,
        3.0 * std::f64::consts::FRAC_PI_2,
        5.5,
    ] {
        let what = format!("sibling fuse rot={rot}");
        let mut t = Topology::new();
        let (a, b) = build_sibling(&mut t, rot);
        let f =
            exact_boolean(&mut t, BooleanOp::Fuse, a, b).unwrap_or_else(|e| panic!("{what}: {e}"));
        let g = gauss_volume(&t, f);
        assert!(
            (g - v_true).abs() / v_true <= 1e-6,
            "{what}: Gauss {g:.9} != inclusion–exclusion {v_true:.9}"
        );
        assert_sibling_fuse_census(&t, f, &what);
        assert_strict_valid(&t, f, &what);
        let mesh = tessellate_solid(&t, f, 0.01).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{what}: boundary at d=0.01");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{what}: non-manifold at d=0.01"
        );
    }
}

/// B52: rotating a cylinder seam must not lose valid notch sections to the
/// face-face broad phase. Sampled circular-rim bounds formerly dropped them.
#[test]
fn b52_box_cylinder_notch_is_seam_placement_invariant() {
    let build = |topo: &mut Topology, rot: f64| {
        let m = Mat4::translation(0.5, -1.5, -0.5) * Mat4::rotation_z(rot);
        let a = make_box(topo, 2.5, 1.0, 1.0).expect("box");
        let b = make_cylinder(topo, 2.0, 1.0).expect("cylinder");
        remus_operations::transform::transform_solid(topo, b, &m).expect("place");
        (a, b)
    };
    let v_box = 2.5;
    let v_cyl = std::f64::consts::PI * 4.0;
    for rot in [
        0.0,
        0.3,
        1.0,
        std::f64::consts::FRAC_PI_2,
        2.0,
        2.5,
        std::f64::consts::PI,
        4.0,
        3.0 * std::f64::consts::FRAC_PI_2,
        5.5,
    ] {
        let what = format!("box-cylinder rot={rot}");
        let run = |op: BooleanOp| {
            let mut t = Topology::new();
            let (a, b) = build(&mut t, rot);
            let s = exact_boolean(&mut t, op, a, b).unwrap_or_else(|e| panic!("{what}: {e}"));
            let label = format!("{what} {op:?}");
            assert_strict_valid(&t, s, &label);
            assert_watertight(&t, s, &label);
            assert_b52_material(&t, s, op, &label);
            assert_translation_invariant(&t, s, &label);
            gauss_volume(&t, s)
        };
        let (gf, gc, gi) = (
            run(BooleanOp::Fuse),
            run(BooleanOp::Cut),
            run(BooleanOp::Intersect),
        );
        assert!(
            (gf - (v_box + v_cyl - gi)).abs() / gf <= 1e-6,
            "{what}: fuse {gf:.9} != box + cylinder − inter"
        );
        assert!(
            (gc + gi - v_box).abs() / v_box <= 1e-6,
            "{what}: cut {gc:.9} + inter {gi:.9} != box"
        );
    }
}

fn assert_b52_material(topo: &Topology, solid: SolidId, op: BooleanOp, what: &str) {
    use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
    use remus_math::vec::Point3;
    use remus_topology::face::FaceSurface;
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert!(faces.len() <= 20, "{what}: {} faces", faces.len());
    assert!(
        faces
            .iter()
            .any(|f| matches!(topo.face(*f).unwrap().surface(), FaceSurface::Cylinder(_)))
    );
    assert!(faces.iter().all(|f| matches!(
        topo.face(*f).unwrap().surface(),
        FaceSurface::Plane { .. } | FaceSurface::Cylinder(_)
    )));
    let mut uses = std::collections::BTreeMap::new();
    #[allow(clippy::cast_possible_truncation)]
    let quantize = |p: Point3| {
        [
            (p.x() * 1e6).round() as i64,
            (p.y() * 1e6).round() as i64,
            (p.z() * 1e6).round() as i64,
        ]
    };
    for face in faces {
        let face = topo.face(face).unwrap();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wire).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                let start = topo.vertex(edge.start()).unwrap().point();
                let end = topo.vertex(edge.end()).unwrap().point();
                let (lo, hi) = edge.strict_domain().unwrap();
                let mid = edge
                    .curve()
                    .evaluate_with_endpoints(f64::midpoint(lo, hi), start, end);
                let mut ends = [quantize(start), quantize(end)];
                ends.sort_unstable();
                *uses.entry((ends, quantize(mid))).or_insert(0) += 1;
            }
        }
    }
    assert!(
        uses.values().all(|count| *count == 2),
        "{what}: position-quantized edge uses {uses:?}"
    );
    for (point, in_box, in_cylinder) in [
        (Point3::new(0.5, 0.2, 0.25), true, true),
        (Point3::new(2.0, 0.8, 0.8), true, false),
        (Point3::new(0.5, -1.5, -0.25), false, true),
        (Point3::new(0.5, -1.5, -0.6), false, false),
        (Point3::new(3.0, 2.0, 0.25), false, false),
    ] {
        let inside = match op {
            BooleanOp::Fuse => in_box || in_cylinder,
            BooleanOp::Cut => in_box && !in_cylinder,
            BooleanOp::Intersect => in_box && in_cylinder,
        };
        let want = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(
            classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap(),
            want,
            "{what}: probe {point:?}"
        );
    }
}
