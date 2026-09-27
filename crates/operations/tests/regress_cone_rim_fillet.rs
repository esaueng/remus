//! B67 qualification: plane–cone closed-rim fillets build the rolling-ball
//! torus on the material side.
//!
//! The analytic `plane_cone_fillet` arm used to place the ball on the empty
//! side of a frustum base rim (tube centre at `−r`, major radius
//! `r_p + r·cot(α/2)`), growing the cap past the rim and extending the wall
//! below the base — while still passing closed-shell validation. Its
//! small-end sibling (cone flaring away from the plate) was declined outright
//! and refused with `TrimmingFailure`.
//!
//! Every test below measures against an INDEPENDENT meridian-integral
//! expectation (a 1-D Simpson integral of the removed profile revolved about
//! the axis — no kernel 3-D volume code), plus tangent-continuity checks
//! along both contact circles and ray-cast material probes. Closed-shell
//! validation alone missed this defect, so volume agreement is the exit
//! oracle here, not a supplement.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_blend::BlendEngine;
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{fillet_cascade, fillet_v2};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

// ── Independent meridian expectation ────────────────────────────────

/// Removed-material volume for a convex bounded plane–cone rim fillet.
///
/// Meridian coordinates `(ρ, s)`: `ρ` radial from the cone axis, `s` along
/// the inward plate normal (material at `s > 0`, plate at `s = 0`). The rim
/// is `(r_rim, 0)`; the wall is `ρ = r_rim + k·s` with
/// `k = σ·cot α` (`σ = ±1` is the cone-axis/inward-normal alignment, `α`
/// the `ConicalSurface` half-angle from the radial plane). The ball centre
/// is `(ρc, r)` with `ρc = r_rim + k·r − r·csc α`; the wall contact is at
/// `s_foot = r·(1 − σ·cos α)`.
///
/// At height `s` the removed annulus spans from the ball's outer surface
/// `ρc + q(s)` (`q(s) = √(r² − (s−r)²)`) to the wall, so by Pappus
/// `V = π·∫₀^{s_foot} (wall² − (ρc+q)²) ds`, evaluated here with Simpson's
/// rule (N = 2048, smooth integrand — exact to ~1e-12, independent of every
/// kernel 3-D measurement path).
fn removed_volume_meridian(r_rim: f64, alpha: f64, sigma: f64, r: f64) -> f64 {
    let (sin_a, cos_a) = alpha.sin_cos();
    let k = sigma * cos_a / sin_a;
    let csc = 1.0 / sin_a;
    let rho_c = r_rim + k * r - r * csc;
    assert!(rho_c > 0.0, "ball must fit inside the rim: {rho_c}");
    let s_foot = r * (1.0 - sigma * cos_a);
    let n: usize = 2048;
    let h = s_foot / n as f64;
    let mut sum = 0.0;
    for i in 0..=n {
        let s = h * i as f64;
        let wall = r_rim + k * s;
        let q = (r * r - (s - r) * (s - r)).max(0.0).sqrt();
        let outer = rho_c + q;
        let f = wall * wall - outer * outer;
        let w = if i == 0 || i == n {
            1.0
        } else if i % 2 == 0 {
            2.0
        } else {
            4.0
        };
        sum += w * f;
    }
    std::f64::consts::PI * sum * h / 3.0
}

/// The Simpson expectation reproduces the roadmap's Pappus closed forms
/// (`make_cone(2, 1.5, 2)` → 0.39834, `make_cone(3, 1, 4)` → 0.96146 at
/// `r = 0.3`) to their quoted digits. This pins the oracle itself: if the
/// derivation above disagreed with the independent closed form, this fails
/// before any kernel code runs.
#[test]
fn meridian_integral_reproduces_roadmap_closed_forms() {
    // (2, 1.5, 2): apex 6 above the base, cot α = 0.25.
    let alpha_a = 8.0_f64.atan2(2.0);
    // (3, 1, 4): apex 6 above the base, cot α = 0.5.
    let alpha_b = 6.0_f64.atan2(3.0);
    let va = removed_volume_meridian(2.0, alpha_a, -1.0, 0.3);
    let vb = removed_volume_meridian(3.0, alpha_b, -1.0, 0.3);
    assert!(
        (va - 0.39834).abs() < 2e-5,
        "base-rim closed form drifted: {va} vs 0.39834"
    );
    assert!(
        (vb - 0.96146).abs() < 2e-5,
        "base-rim closed form drifted: {vb} vs 0.96146"
    );
}

// ── Topology helpers ────────────────────────────────────────────────

/// Circle edge whose centre height is extremal (bottom rim: minimum z, top
/// rim: maximum z) among the solid's full-circle edges.
fn extremal_circle_edge(topo: &Topology, solid: SolidId, want_top: bool) -> EdgeId {
    let mut best: Option<(EdgeId, f64)> = None;
    for eid in solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(eid).unwrap();
        let EdgeCurve::Circle(c) = edge.curve() else {
            continue;
        };
        if edge.start() != edge.end() {
            continue;
        }
        let z = c.center().z();
        let take = match best {
            None => true,
            Some((_, bz)) => {
                if want_top {
                    z > bz
                } else {
                    z < bz
                }
            }
        };
        if take {
            best = Some((eid, z));
        }
    }
    best.map(|(eid, _)| eid)
        .expect("solid must have a circle rim")
}

/// Every closed circle edge of the solid as `(centre, radius)`.
fn circle_edges(topo: &Topology, solid: SolidId) -> Vec<(Point3, f64)> {
    let mut out = Vec::new();
    for f in solid_faces(topo, solid).unwrap() {
        let face = topo.face(f).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                if let EdgeCurve::Circle(c) = edge.curve()
                    && edge.start() == edge.end()
                {
                    out.push((c.center(), c.radius()));
                }
            }
        }
    }
    out
}

fn has_circle(circles: &[(Point3, f64)], z: f64, radius: f64, tol: f64) -> bool {
    circles
        .iter()
        .any(|(c, r)| (c.z() - z).abs() < tol && (r - radius).abs() < tol)
}

/// Outward normal of `face` at surface point `p`.
fn outward_normal(topo: &Topology, face: remus_topology::face::FaceId, p: Point3) -> Vec3 {
    let face_data = topo.face(face).unwrap();
    let surface = face_data.surface();
    let (u, v) = surface.project_point(p).unwrap_or((0.0, 0.0));
    let n = surface.normal(u, v).normalize().unwrap();
    if face_data.is_reversed() { -n } else { n }
}

/// A closed blend band meets each support tangentially: across every shared
/// contact circle the band's outward normal must coincide with its support's.
fn assert_band_normals_continuous(topo: &Topology, solid: SolidId) {
    let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
    let band = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .expect("toroidal band");
    let mut checked = 0;
    let face = topo.face(band).unwrap();
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oe in topo.wire(wid).unwrap().edges() {
            let EdgeCurve::Circle(circle) = topo.edge(oe.edge()).unwrap().curve() else {
                continue;
            };
            let Some(support) = adjacency
                .faces_for_edge(oe.edge())
                .iter()
                .copied()
                .find(|&f| f != band)
            else {
                continue;
            };
            for t in [0.3, 1.9, 4.4] {
                let p = circle.evaluate(t);
                let (nb, ns) = (
                    outward_normal(topo, band, p),
                    outward_normal(topo, support, p),
                );
                assert!(
                    nb.dot(ns) > 1.0 - 1e-6,
                    "band normal {nb:?} breaks from its support's {ns:?} at {p:?}"
                );
            }
            checked += 1;
        }
    }
    assert_eq!(checked, 2, "a closed band has two contact circles");
}

/// Strict dual validation: both the operations Euler-aware validator and the
/// check-crate report must be clean.
fn assert_strict_valid(topo: &Topology, solid: SolidId) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let check_report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(check_report.is_valid(), "{:#?}", check_report.issues);
}

/// Welded manifold tessellation: zero boundary and zero non-manifold edges.
fn assert_watertight_mesh(topo: &Topology, solid: SolidId, deflection: f64) {
    let mesh = tessellate_solid(topo, solid, deflection).unwrap();
    assert_eq!(
        boundary_edge_count(&mesh),
        0,
        "open mesh at deflection {deflection}"
    );
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "non-manifold mesh at deflection {deflection}"
    );
}

/// Cone half-angle (from the radial plane) of a `make_cone` frustum.
fn frustum_half_angle(r_big: f64, r_small: f64, height: f64) -> f64 {
    let axial_to_apex = r_small * height / (r_big - r_small);
    (axial_to_apex + height).atan2(r_big)
}

// ── Core qualification ──────────────────────────────────────────────

/// Fillet one rim of a frustum through the public `fillet_v2` path and check
/// the full B67 postcondition set: exact torus census, closed-form removal,
/// contact circles, tangent continuity, material probes, strict validation
/// and watertight mesh.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn qualify_cone_rim(
    r_bottom: f64,
    r_top: f64,
    height: f64,
    want_top: bool,
    radius: f64,
    deflection: f64,
    rel_tol: f64,
) {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, r_bottom, r_top, height).unwrap();
    let rim = extremal_circle_edge(&topo, solid, want_top);

    let v_before = solid_volume(&topo, solid, deflection).unwrap();
    let result = fillet_v2(&mut topo, solid, &[rim], radius).unwrap();
    assert!(!result.is_partial && result.failed.is_empty());
    assert_eq!(result.succeeded, vec![rim]);
    let out = result.solid;

    // Exact-analytic census: wall + 2 caps + 1 torus band, nothing else.
    let faces = solid_faces(&topo, out).unwrap();
    assert_eq!(faces.len(), 4, "filleted frustum must have 4 faces");
    let torus_count = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .count();
    assert_eq!(torus_count, 1, "exactly one toroidal band");

    // Closed-form removal: measured vs the independent meridian integral.
    let v_after = solid_volume(&topo, out, deflection).unwrap();
    let removed = v_before - v_after;
    assert!(removed > 0.0, "convex rim fillet must remove material");
    let (r_rim, sigma) = if want_top {
        // Top rim: apex across the cap from the material on a narrowing
        // frustum (σ = +1); mirrored (σ = −1) on a flaring one.
        let axis_sign = if r_bottom >= r_top { -1.0 } else { 1.0 };
        (r_top, -axis_sign)
    } else {
        // Base rim: apex on the material side of a narrowing frustum
        // (σ = −1); across the cap of a flaring one (σ = +1).
        let axis_sign = if r_bottom >= r_top { -1.0 } else { 1.0 };
        (r_bottom, axis_sign)
    };
    let (r_big, r_small) = if r_bottom >= r_top {
        (r_bottom, r_top)
    } else {
        (r_top, r_bottom)
    };
    let alpha = frustum_half_angle(r_big, r_small, height);
    let expected = removed_volume_meridian(r_rim, alpha, sigma, radius);
    assert!(
        (removed - expected).abs() <= rel_tol * expected,
        "removal {removed} vs rolling-ball closed form {expected} (rim {r_rim}, σ {sigma})"
    );

    // Gauss route must agree with the tessellation route: two independent
    // kernel integrators on the same B-Rep.
    let gauss_after = mass_properties(&topo, out).unwrap().mass;
    assert!(
        (gauss_after - v_after).abs() <= 1e-3 * v_after,
        "Gauss {gauss_after} vs tessellated {v_after}"
    );

    // Contact circles: plate contact inside the cap disc, wall contact on
    // the frustum wall — and the original rim gone.
    let (sin_a, cos_a) = alpha.sin_cos();
    let k = sigma * cos_a / sin_a;
    let rho_c = r_rim + k * radius - radius / sin_a;
    let s_foot = radius * (1.0 - sigma * cos_a);
    let rho_foot = rho_c + radius * sin_a;
    let (plate_z, wall_z) = if want_top {
        (height, height - s_foot)
    } else {
        (0.0, s_foot)
    };
    let circles = circle_edges(&topo, out);
    let tol = 1e-6 * r_rim.max(1.0) + 1e-9;
    assert!(
        has_circle(&circles, plate_z, rho_c, tol.max(1e-6)),
        "no plate contact circle (z {plate_z}, ρ {rho_c}): {circles:?}"
    );
    assert!(
        has_circle(&circles, wall_z, rho_foot, tol.max(1e-6)),
        "no wall contact circle (z {wall_z}, ρ {rho_foot}): {circles:?}"
    );
    let rim_z = if want_top { height } else { 0.0 };
    assert!(
        !has_circle(&circles, rim_z, r_rim, tol.max(1e-6)),
        "original rim survived: {circles:?}"
    );

    assert_band_normals_continuous(&topo, out);

    // Material probes (ray-cast ground truth, never winding): the old
    // corner is gone, the interior and the ball centre remain.
    let opts = ClassifyOptions::default();
    let mid_r = 0.5 * r_rim.min(rho_c);
    let mid_z = 0.5 * height;
    assert_eq!(
        classify_point(&topo, out, Point3::new(mid_r, 0.0, mid_z), &opts).unwrap(),
        PointClassification::Inside,
        "deep interior must stay inside"
    );
    // Ball centre: one fillet radius into the material from the plate
    // contact (above the base cap, below the top cap).
    let ball_z = if want_top { height - radius } else { radius };
    assert_eq!(
        classify_point(&topo, out, Point3::new(rho_c, 0.0, ball_z), &opts).unwrap(),
        PointClassification::Inside,
        "ball centre must stay inside"
    );
    let probe_r = r_rim + 0.5 * radius;
    let probe_z = if want_top {
        height + 0.25 * radius
    } else {
        -0.25 * radius
    };
    assert_eq!(
        classify_point(&topo, out, Point3::new(probe_r, 0.0, probe_z), &opts).unwrap(),
        PointClassification::Outside,
        "the old corner wedge must be gone"
    );

    assert_strict_valid(&topo, out);
    assert_watertight_mesh(&topo, out, 0.1);
    assert_watertight_mesh(&topo, out, 0.01);
    assert_watertight_mesh(&topo, out, deflection);
}

/// Roadmap base-rim pair: the incorrect accepted result must now measure the
/// rolling-ball closed form instead of 0.44512.
#[test]
fn cone_base_rim_fillet_matches_meridian_removal() {
    qualify_cone_rim(2.0, 1.5, 2.0, false, 0.3, 1e-4, 1e-3);
}

/// Roadmap base-rim pair, second taper: 1.07704 → 0.96146.
#[test]
fn cone_base_rim_fillet_second_taper_matches_meridian_removal() {
    qualify_cone_rim(3.0, 1.0, 4.0, false, 0.3, 1e-4, 1e-3);
}

/// Roadmap small-end sibling: previously refused with `TrimmingFailure`.
#[test]
fn cone_small_end_rim_fillet_matches_meridian_removal() {
    qualify_cone_rim(2.0, 1.5, 2.0, true, 0.3, 1e-4, 1e-3);
    qualify_cone_rim(3.0, 1.0, 4.0, true, 0.3, 1e-4, 1e-3);
}

/// Flaring frustum (small end at the base): both rims with mirrored σ.
#[test]
fn cone_flared_frustum_both_rims_match_meridian_removal() {
    qualify_cone_rim(1.0, 2.0, 3.0, false, 0.3, 1e-4, 1e-3);
    qualify_cone_rim(1.0, 2.0, 3.0, true, 0.3, 1e-4, 1e-3);
}

/// Varying radius on one taper, both rims.
#[test]
fn cone_rim_fillet_radius_sweep_matches_meridian_removal() {
    for r in [0.1, 0.3, 0.5] {
        qualify_cone_rim(2.0, 1.5, 2.0, false, r, 1e-4, 2e-3);
        qualify_cone_rim(2.0, 1.5, 2.0, true, r, 1e-4, 2e-3);
    }
}

/// Cylinder-rim control: unchanged behavior. Removal is measured through
/// the exact Gauss route (both legs all-analytic) against the meridian
/// integral, which at the cylinder limit (`α = π/2`) is the exact Pappus
/// volume — centroid and all, not the ball-centre approximation
/// `2π(R−r)·r²·(1−π/4)`, which understates it by ~12%.
#[test]
fn cylinder_rim_control_still_matches_pappus_removal() {
    let (big_r, h, r) = (2.0, 4.0, 0.3);
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, big_r, h).unwrap();
    let rim = extremal_circle_edge(&topo, solid, false);
    let g_before = mass_properties(&topo, solid).unwrap().mass;
    let result = fillet_v2(&mut topo, solid, &[rim], r).unwrap();
    assert!(!result.is_partial && result.failed.is_empty());
    let g_after = mass_properties(&topo, result.solid).unwrap().mass;
    let expected = removed_volume_meridian(big_r, std::f64::consts::FRAC_PI_2, 1.0, r);
    let removed = g_before - g_after;
    assert!(
        (removed - expected).abs() <= 1e-4 * expected,
        "control removal {removed} vs Pappus {expected}"
    );
    assert_strict_valid(&topo, result.solid);
    assert_watertight_mesh(&topo, result.solid, 0.01);
}

// ── Placement and scale ─────────────────────────────────────────────

/// Rigid placement: the same fillet after a rotation + translation removes
/// the same volume and carries the torus centre rigidly.
#[test]
fn cone_rim_fillet_survives_rigid_placement() {
    let (r_bottom, r_top, height, r) = (2.0, 1.5, 2.0, 0.3);
    let place = Mat4::translation(13.0, -7.0, 5.0) * Mat4::rotation_y(std::f64::consts::FRAC_PI_4);

    for want_top in [false, true] {
        let mut topo = Topology::new();
        let solid = make_cone(&mut topo, r_bottom, r_top, height).unwrap();
        let rim = extremal_circle_edge(&topo, solid, want_top);
        let v_before = solid_volume(&topo, solid, 1e-4).unwrap();
        let plain = fillet_v2(&mut topo, solid, &[rim], r).unwrap();
        let v_plain = solid_volume(&topo, plain.solid, 1e-4).unwrap();

        let mut moved_topo = Topology::new();
        let moved = make_cone(&mut moved_topo, r_bottom, r_top, height).unwrap();
        let moved_rim = extremal_circle_edge(&moved_topo, moved, want_top);
        remus_operations::transform::transform_solid(&mut moved_topo, moved, &place).unwrap();
        let mv_before = solid_volume(&moved_topo, moved, 1e-4).unwrap();
        assert!(
            (mv_before - v_before).abs() <= 1e-6 * v_before,
            "placement must not move the volume: {mv_before} vs {v_before}"
        );
        let result = fillet_v2(&mut moved_topo, moved, &[moved_rim], r).unwrap();
        assert!(!result.is_partial && result.failed.is_empty());
        let mv_after = solid_volume(&moved_topo, result.solid, 1e-4).unwrap();
        assert!(
            ((mv_before - mv_after) - (v_before - v_plain)).abs() <= 1e-3 * (v_before - v_plain),
            "removal must be placement-invariant"
        );
        assert_strict_valid(&moved_topo, result.solid);
        assert_watertight_mesh(&moved_topo, result.solid, 0.01);

        // Torus centre carried rigidly: compare against the unmoved
        // result's band centre pushed through the placement.
        let band_center = |t: &Topology, s: SolidId| {
            solid_faces(t, s)
                .unwrap()
                .into_iter()
                .find_map(|f| match t.face(f).unwrap().surface() {
                    FaceSurface::Torus(torus) => Some(torus.center()),
                    _ => None,
                })
                .expect("toroidal band")
        };
        let want = place.mul_point(band_center(&topo, plain.solid));
        let got = band_center(&moved_topo, result.solid);
        assert!(
            (got - want).length() <= 1e-6,
            "band centre did not ride the placement: {got:?} vs {want:?}"
        );
    }
}

/// Scales 1e-3 / 1 / 1e3: removal scales with the cube of the scale.
#[test]
fn cone_rim_fillet_scales_cubically() {
    for scale in [1e-3, 1.0, 1e3] {
        for want_top in [false, true] {
            let (r_bottom, r_top, height, r) = (2.0 * scale, 1.5 * scale, 2.0 * scale, 0.3 * scale);
            let deflection = 1e-4 * scale;
            let mut topo = Topology::new();
            let solid = make_cone(&mut topo, r_bottom, r_top, height).unwrap();
            let rim = extremal_circle_edge(&topo, solid, want_top);
            let v_before = solid_volume(&topo, solid, deflection).unwrap();
            let result = fillet_v2(&mut topo, solid, &[rim], r).unwrap();
            assert!(!result.is_partial && result.failed.is_empty());
            let v_after = solid_volume(&topo, result.solid, deflection).unwrap();
            let removed = v_before - v_after;
            let (r_big, r_small) = (r_bottom, r_top);
            let alpha = frustum_half_angle(r_big, r_small, height);
            let r_rim = if want_top { r_top } else { r_bottom };
            // Narrowing frustum (apex above): base σ = −1, top σ = +1.
            let sigma = if want_top { 1.0 } else { -1.0 };
            let expected = removed_volume_meridian(r_rim, alpha, sigma, r);
            assert!(
                (removed - expected).abs() <= 1e-2 * expected,
                "scale {scale} rim {} removal {removed} vs {expected}",
                if want_top { "top" } else { "base" }
            );
            assert_strict_valid(&topo, result.solid);
            assert_watertight_mesh(&topo, result.solid, 0.1 * scale);
        }
    }
}

// ── Public cascade, engine disclosure and typed refusal ─────────────

/// The default cascade reaches the same corrected geometry as `fillet_v2`
/// and discloses the engine that ran.
#[test]
fn cone_rim_default_cascade_agrees_with_fillet_v2() {
    for want_top in [false, true] {
        let mut topo = Topology::new();
        let solid = make_cone(&mut topo, 2.0, 1.5, 2.0).unwrap();
        let rim = extremal_circle_edge(&topo, solid, want_top);
        let via_v2 = fillet_v2(&mut topo, solid, &[rim], 0.3).unwrap();
        let v_v2 = solid_volume(&topo, via_v2.solid, 1e-4).unwrap();

        let mut topo2 = Topology::new();
        let solid2 = make_cone(&mut topo2, 2.0, 1.5, 2.0).unwrap();
        let rim2 = extremal_circle_edge(&topo2, solid2, want_top);
        let via_cascade = fillet_cascade(&mut topo2, solid2, &[rim2], 0.3).unwrap();
        let v_cascade = solid_volume(&topo2, via_cascade.solid, 1e-4).unwrap();

        assert!(
            (v_cascade - v_v2).abs() <= 1e-9,
            "cascade {v_cascade} vs v2 {v_v2}"
        );
        assert!(matches!(
            via_cascade.engine,
            BlendEngine::Walking | BlendEngine::RollingBall | BlendEngine::Mixed
        ));
        assert_strict_valid(&topo2, via_cascade.solid);
    }
}

/// Outside the supported domain the rim refuses with a typed error and
/// leaves the input untouched: a radius taller than the frustum cannot fit
/// its wall contact on the wall.
#[test]
fn cone_rim_oversize_radius_refuses_typed_and_fail_closed() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 2.0, 1.5, 0.5).unwrap();
    let rim = extremal_circle_edge(&topo, solid, false);
    let faces_before = solid_faces(&topo, solid).unwrap().len();
    let v_before = solid_volume(&topo, solid, 1e-4).unwrap();
    let result = fillet_v2(&mut topo, solid, &[rim], 0.5);
    let err = match result {
        Ok(_) => panic!("oversize radius must refuse, but it succeeded"),
        Err(e) => e,
    };
    let msg = format!("{err:?}");
    assert!(
        msg.contains("CliffEncountered"),
        "oversize radius must refuse typed, got: {msg}"
    );
    assert_eq!(
        solid_faces(&topo, solid).unwrap().len(),
        faces_before,
        "refusal must leave the solid untouched"
    );
    assert!(
        (solid_volume(&topo, solid, 1e-4).unwrap() - v_before).abs() <= 1e-12,
        "refusal must leave the volume untouched"
    );
}
