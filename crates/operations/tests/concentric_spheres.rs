//! Concentric-sphere scenarios for boolean robustness.
//!
//! Sphere same-domain requires matching center and radius; the SD
//! detector returns `Some(true)` (always same-direction since spheres
//! have no axis). Like cylinders, the DETECTOR works correctly
//! (see `same_domain.rs::sphere_*` unit tests) but the GFA pipeline
//! integration of sphere SD pairs has known gaps tracked here.

#![allow(clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::PointClassification;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_sphere;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.05;
const SEGMENTS: usize = 32;

fn vol(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).unwrap()
}

/// Mesh-winding point classification.
///
/// The kernel's public winding classifier: the watertight lens mesh is
/// boundary-exact, so its solid-angle winding is decisive. (The pure-B-Rep
/// ray-cast majority vote in `remus_check::classify::classify_point` and the
/// chordal-fan winding in remus-check are both marginal for probes inside one
/// operand's collar on these four-patch lens solids — their fan polygons cut
/// the chord, so the raw winding lands near the threshold; the watertight
/// mesh winding has no such gap.)
fn classify_winding(topo: &Topology, solid: SolidId, p: (f64, f64, f64)) -> PointClassification {
    remus_operations::classify::classify_point_winding(
        topo,
        solid,
        remus_math::vec::Point3::new(p.0, p.1, p.2),
        1e-3,
        1e-7,
    )
    .unwrap()
}

fn sphere_volume(r: f64) -> f64 {
    4.0 * PI * r * r * r / 3.0
}

fn approx_eq(a: f64, b: f64, frac: f64) -> bool {
    (a - b).abs() < a.abs().max(b.abs()).max(1.0) * frac
}

fn sphere_at(topo: &mut Topology, x: f64, y: f64, z: f64, radius: f64) -> SolidId {
    let s = make_sphere(topo, radius, SEGMENTS).unwrap();
    if x != 0.0 || y != 0.0 || z != 0.0 {
        transform_solid(topo, s, &Mat4::translation(x, y, z)).unwrap();
    }
    s
}

// ── 0. Baseline: disjoint spheres ──────────────────────────────────────

#[test]
fn baseline_disjoint_spheres_intersect_empty() {
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 5.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Intersect, a, b);
    if let Ok(sid) = r {
        let v = vol(&topo, sid);
        assert!(
            v < 1e-3,
            "disjoint sphere intersect should be ~zero, got {v}"
        );
    }
}

// ── 1. Identical spheres (degenerate SD) ──────────────────────────────

#[test]
fn identical_spheres_fuse_preserves_volume() {
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let expected = sphere_volume(1.0);
    let got = vol(&topo, r);
    assert!(approx_eq(got, expected, 0.05));
}

#[test]
fn identical_spheres_intersect_preserves_volume() {
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Intersect, a, b).unwrap();
    let expected = sphere_volume(1.0);
    let got = vol(&topo, r);
    assert!(approx_eq(got, expected, 0.05));
}

// ── 2. Concentric different radii (NOT same-domain — must NOT merge) ──

#[test]
fn concentric_spheres_different_radii_fuse() {
    let mut topo = Topology::default();
    let outer = sphere_at(&mut topo, 0.0, 0.0, 0.0, 2.0);
    let inner = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Fuse, outer, inner).unwrap();
    let expected = sphere_volume(2.0);
    let got = vol(&topo, r);
    assert!(approx_eq(got, expected, 0.03));
}

#[test]
fn concentric_spheres_different_radii_intersect_collapses_to_inner() {
    let mut topo = Topology::default();
    let outer = sphere_at(&mut topo, 0.0, 0.0, 0.0, 3.0);
    let inner = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.5);
    let r = boolean(&mut topo, BooleanOp::Intersect, outer, inner).unwrap();
    // Intersection of concentric spheres == smaller sphere.
    let expected = sphere_volume(1.5);
    let got = vol(&topo, r);
    assert!(
        approx_eq(got, expected, 0.05),
        "concentric intersect should collapse to inner sphere: got {got:.3}, expected {expected:.3}"
    );
}

#[test]
fn concentric_spheres_at_offset_center_fuse() {
    // Verify the shortcut handles a non-origin shared center: both spheres
    // translated to (5, -2, 7) before the boolean.
    let mut topo = Topology::default();
    let outer = sphere_at(&mut topo, 5.0, -2.0, 7.0, 2.0);
    let inner = sphere_at(&mut topo, 5.0, -2.0, 7.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Fuse, outer, inner).unwrap();
    let expected = sphere_volume(2.0);
    let got = vol(&topo, r);
    assert!(approx_eq(got, expected, 0.05));
}

#[test]
fn non_concentric_spheres_fuse_exact() {
    // Two unit spheres in general position (centers one radius apart) fuse
    // through the exact pipeline: the sphere-sphere section is the closed-form
    // radical-plane circle, the face splitter carves each hemisphere along it
    // into cap + collar, and the assembler welds the two operands' collars
    // along the section arcs. This used to be the pinned refusal
    // `non_concentric_spheres_fuse_fails_closed_without_shortcut`; the exact
    // gate below is its flip.
    //
    // Oracle: the union is both balls minus the lens, and the lens of two
    // unit spheres at distance d is two caps of height h = 1 - d/2:
    //   V_lens = 2 * pi * h^2 * (3 - h) / 3  (= 5*pi/12 at d = 1)
    //   V_fuse = 8*pi/3 - V_lens            (= 9*pi/4 at d = 1)
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 1.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let expected = 9.0 * PI / 4.0;
    let got = vol(&topo, r);
    assert!(
        (got - expected).abs() < 1e-6,
        "non-concentric sphere fuse must match the closed form exactly: \
         got {got:.9}, expected {expected:.9}"
    );
    // Kept-material witnesses (winding classifier): inside A only, deep in
    // the lens (union interior), and near A's pole.
    for p in [
        (-0.5, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (0.5, 0.0, 0.3),
        (0.0, 0.0, 0.9),
    ] {
        let c = classify_winding(&topo, r, p);
        assert!(
            matches!(c, PointClassification::Inside),
            "fuse probe {p:?} must be Inside, got {c:?}"
        );
    }
}

#[test]
fn non_concentric_spheres_cut_exact() {
    // Cut of two unit spheres at distance 1: V = 4*pi/3 - V_lens = 11*pi/12.
    // The lens region (inside both spheres) must classify Outside — it is
    // exactly the removed material.
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 1.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Cut, a, b).unwrap();
    let expected = 11.0 * PI / 12.0;
    let got = vol(&topo, r);
    assert!(
        (got - expected).abs() < 1e-6,
        "non-concentric sphere cut must match the closed form exactly: \
         got {got:.9}, expected {expected:.9}"
    );
    let inside = classify_winding(&topo, r, (-0.5, 0.0, 0.0));
    assert!(
        matches!(inside, PointClassification::Inside),
        "cut kept-material probe must be Inside, got {inside:?}"
    );
    let removed = classify_winding(&topo, r, (0.5, 0.0, 0.0));
    assert!(
        matches!(removed, PointClassification::Outside),
        "cut lens-region probe (the removed material) must be Outside, got {removed:?}"
    );
}

#[test]
fn non_concentric_spheres_intersect_exact() {
    // Intersect of two unit spheres at distance 1: V = V_lens = 5*pi/12.
    // The lens region is the whole result; A-only material is Outside.
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 1.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Intersect, a, b).unwrap();
    let expected = 5.0 * PI / 12.0;
    let got = vol(&topo, r);
    assert!(
        (got - expected).abs() < 1e-6,
        "non-concentric sphere intersect must match the closed form exactly: \
         got {got:.9}, expected {expected:.9}"
    );
    let inside = classify_winding(&topo, r, (0.5, 0.0, 0.0));
    assert!(
        matches!(inside, PointClassification::Inside),
        "intersect lens-center probe must be Inside, got {inside:?}"
    );
    let outside = classify_winding(&topo, r, (-0.5, 0.0, 0.0));
    assert!(
        matches!(outside, PointClassification::Outside),
        "intersect A-only probe must be Outside, got {outside:?}"
    );
}

#[test]
fn non_concentric_spheres_far_apart_overlap_exact() {
    // A near-tangent proper overlap (d = 1.7, caps of height 0.15): the
    // general-position arm holds away from the unit-scale symmetric case too.
    // V_lens = 2*pi*h^2*(3-h)/3 with h = 0.15.
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 1.7, 0.0, 0.0, 1.0);
    let h = 1.0 - 1.7 / 2.0;
    let v_lens = 2.0 * PI * h * h * (3.0 - h) / 3.0;
    let r = boolean(&mut topo, BooleanOp::Intersect, a, b).unwrap();
    let got = vol(&topo, r);
    assert!(
        (got - v_lens).abs() < 1e-6,
        "near-tangent intersect must match the closed form exactly: \
         got {got:.9}, expected {v_lens:.9}"
    );
    let fused = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let got_fuse = vol(&topo, fused);
    let expected_fuse = 8.0 * PI / 3.0 - v_lens;
    assert!(
        (got_fuse - expected_fuse).abs() < 1e-6,
        "near-tangent fuse must match the closed form exactly: \
         got {got_fuse:.9}, expected {expected_fuse:.9}"
    );
}

#[test]
fn tangent_spheres_intersect_is_empty() {
    // Externally tangent spheres share exactly one point: the intersection is
    // empty (the tangent contact arm defers — a point section bounds no
    // region — and the empty result is the honest outcome).
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 2.0, 0.0, 0.0, 1.0);
    let r = boolean(&mut topo, BooleanOp::Intersect, a, b).unwrap();
    let got = vol(&topo, r);
    assert!(
        got < 1e-9,
        "tangent sphere intersect must be empty, got {got:.9}"
    );
}

// ── 3. Sub-tolerance shifted center (should be SD) ────────────────────

#[test]
fn spheres_sub_tolerance_shifted_fuse() {
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, 4e-8, 0.0, 0.0, 1.0); // < linear tol 1e-7
    let r = boolean(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    let expected = sphere_volume(1.0);
    let got = vol(&topo, r);
    assert!(approx_eq(got, expected, 0.05));
}
