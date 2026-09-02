//! Concentric-sphere scenarios for boolean robustness.
//!
//! Sphere same-domain requires matching center and radius; the SD
//! detector returns `Some(true)` (always same-direction since spheres
//! have no axis). Like cylinders, the DETECTOR works correctly
//! (see `same_domain.rs::sphere_*` unit tests) but the GFA pipeline
//! integration of sphere SD pairs has known gaps tracked here.

#![allow(clippy::expect_used, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_sphere;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.05;
const SEGMENTS: usize = 32;

fn vol(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).unwrap()
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

fn assert_general_position_boolean(
    b_center: remus_math::vec::Point3,
    op: BooleanOp,
    expected_volume: f64,
    inside: &[remus_math::vec::Point3],
    outside: &[remus_math::vec::Point3],
) {
    let mut topo = Topology::default();
    let a = sphere_at(&mut topo, 0.0, 0.0, 0.0, 1.0);
    let b = sphere_at(&mut topo, b_center.x(), b_center.y(), b_center.z(), 1.0);
    let result = boolean(&mut topo, op, a, b).expect("transversal sphere boolean");

    let faces = solid_faces(&topo, result).unwrap();
    assert_eq!(faces.len(), 4, "sphere boolean analytic face census");
    assert!(faces.iter().all(|&face| {
        topo.face(face)
            .is_ok_and(|data| matches!(data.surface(), FaceSurface::Sphere(_)))
    }));

    let report = remus_operations::validate::validate_solid(&topo, result).unwrap();
    assert!(report.is_valid(), "sphere boolean validation: {report:?}");

    for deflection in [0.12, 0.06, 0.03] {
        let mesh = tessellate_solid(&topo, result, deflection).unwrap();
        assert!(!mesh.indices.is_empty(), "deflection={deflection}");
        assert_eq!(boundary_edge_count(&mesh), 0, "deflection={deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "deflection={deflection}");
    }

    let measured = solid_volume(&topo, result, 0.03).unwrap();
    assert!(
        approx_eq(measured, expected_volume, 0.01),
        "sphere boolean volume: got {measured}, expected {expected_volume}"
    );

    for &point in inside {
        assert_eq!(
            classify_point(&topo, result, point, 0.03, 1e-7).unwrap(),
            PointClassification::Inside,
            "inside probe {point:?}"
        );
    }
    for &point in outside {
        assert_eq!(
            classify_point(&topo, result, point, 0.03, 1e-7).unwrap(),
            PointClassification::Outside,
            "outside probe {point:?}"
        );
    }
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
fn non_concentric_spheres_fuse_has_exact_union_volume() {
    assert_general_position_boolean(
        remus_math::vec::Point3::new(1.0, 0.0, 0.0),
        BooleanOp::Fuse,
        9.0 * PI / 4.0,
        &[
            remus_math::vec::Point3::new(-0.5, 0.0, 0.0),
            remus_math::vec::Point3::new(0.5, 0.0, 0.0),
            remus_math::vec::Point3::new(1.5, 0.0, 0.0),
        ],
        &[remus_math::vec::Point3::new(0.5, 1.1, 0.0)],
    );
}

#[test]
fn oblique_non_concentric_spheres_fuse_keeps_exact_spherical_patches() {
    let n = remus_math::vec::Vec3::new(2.0 / 3.0, 2.0 / 3.0, 1.0 / 3.0);
    let perpendicular = remus_math::vec::Vec3::new(
        -std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    );
    assert_general_position_boolean(
        remus_math::vec::Point3::new(n.x(), n.y(), n.z()),
        BooleanOp::Fuse,
        9.0 * PI / 4.0,
        &[
            remus_math::vec::Point3::new(-0.5 * n.x(), -0.5 * n.y(), -0.5 * n.z()),
            remus_math::vec::Point3::new(0.5 * n.x(), 0.5 * n.y(), 0.5 * n.z()),
            remus_math::vec::Point3::new(1.5 * n.x(), 1.5 * n.y(), 1.5 * n.z()),
        ],
        &[remus_math::vec::Point3::new(
            0.5 * n.x() + 1.1 * perpendicular.x(),
            0.5 * n.y() + 1.1 * perpendicular.y(),
            0.5 * n.z(),
        )],
    );
}

#[test]
fn non_concentric_spheres_intersect_has_exact_lens_volume() {
    assert_general_position_boolean(
        remus_math::vec::Point3::new(1.0, 0.0, 0.0),
        BooleanOp::Intersect,
        5.0 * PI / 12.0,
        &[remus_math::vec::Point3::new(0.5, 0.0, 0.0)],
        &[
            remus_math::vec::Point3::new(-0.5, 0.0, 0.0),
            remus_math::vec::Point3::new(1.5, 0.0, 0.0),
        ],
    );
}

#[test]
fn non_concentric_spheres_cut_has_exact_difference_volume() {
    assert_general_position_boolean(
        remus_math::vec::Point3::new(1.0, 0.0, 0.0),
        BooleanOp::Cut,
        11.0 * PI / 12.0,
        &[remus_math::vec::Point3::new(-0.5, 0.0, 0.0)],
        &[
            remus_math::vec::Point3::new(0.5, 0.0, 0.0),
            remus_math::vec::Point3::new(1.5, 0.0, 0.0),
        ],
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
