#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::fill_face::fill_coons_patch;

use super::*;

/// Helper: make a square face at z=offset with given size.
fn make_square_at(topo: &mut Topology, size: f64, z: f64) -> FaceId {
    let hs = size / 2.0;
    let tol_val = 1e-7;
    let v0 = topo.add_vertex(Vertex::new(Point3::new(-hs, -hs, z), tol_val));
    let v1 = topo.add_vertex(Vertex::new(Point3::new(hs, -hs, z), tol_val));
    let v2 = topo.add_vertex(Vertex::new(Point3::new(hs, hs, z), tol_val));
    let v3 = topo.add_vertex(Vertex::new(Point3::new(-hs, hs, z), tol_val));

    let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
    let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
    let e3 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));

    let wire = Wire::new(
        vec![
            OrientedEdge::new(e0, true),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, true),
            OrientedEdge::new(e3, true),
        ],
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);

    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

#[test]
fn loft_refuses_profiles_with_holes_without_mutation() {
    let mut topo = Topology::new();
    let bottom = make_square_at(&mut topo, 4.0, 0.0);
    let top = make_square_at(&mut topo, 4.0, 2.0);
    let hole = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let bottom_face = topo.face(bottom).unwrap();
    let outer = bottom_face.outer_wire();
    let surface = bottom_face.surface().clone();
    let holed_bottom = topo.add_face(Face::new(outer, vec![hole], surface));
    let before = topo.allocated_slot_count();

    for result in [
        loft(&mut topo, &[holed_bottom, top]),
        loft_smooth(&mut topo, &[holed_bottom, top]),
    ] {
        let error = result.unwrap_err();
        assert!(
            error
                .to_string()
                .contains("refusing to discard an inner wire"),
            "unexpected error: {error}"
        );
        assert_eq!(topo.allocated_slot_count(), before);
    }
}

#[test]
fn loft_two_identical_squares_makes_box() {
    let mut topo = Topology::new();
    let bottom = make_square_at(&mut topo, 1.0, 0.0);
    let top = make_square_at(&mut topo, 1.0, 1.0);

    let solid = loft(&mut topo, &[bottom, top]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    // 2 caps + 4 sides = 6 faces
    assert_eq!(sh.faces().len(), 6, "lofted box should have 6 faces");

    // Volume should be 1.0 (unit cube)
    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    let tol = Tolerance::loose();
    assert!(
        tol.approx_eq(vol, 1.0),
        "lofted box volume should be ~1.0, got {vol}"
    );
}

#[test]
fn loft_tapered_frustum() {
    let mut topo = Topology::new();
    let bottom = make_square_at(&mut topo, 2.0, 0.0);
    let top = make_square_at(&mut topo, 1.0, 3.0);

    let solid = loft(&mut topo, &[bottom, top]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    assert_eq!(sh.faces().len(), 6);

    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    // Frustum of a square pyramid: V = h/3 * (A1 + A2 + sqrt(A1*A2))
    // A1 = 4.0, A2 = 1.0, h = 3.0
    // V = 3/3 * (4 + 1 + 2) = 7.0
    let expected = 7.0;
    assert!(
        (vol - expected).abs() / expected < 0.05,
        "tapered frustum volume should be ~{expected}, got {vol} (error: {:.1}%)",
        (vol - expected).abs() / expected * 100.0
    );
}

#[test]
fn loft_three_profiles() {
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 2.0, 0.0);
    let p1 = make_square_at(&mut topo, 1.0, 1.5);
    let p2 = make_square_at(&mut topo, 2.0, 3.0);

    let solid = loft(&mut topo, &[p0, p1, p2]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    // 2 caps + 2 sections × 4 edges = 10 faces
    assert_eq!(sh.faces().len(), 10);

    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    assert!(vol > 0.0, "lofted solid should have positive volume");
}

#[test]
fn loft_single_profile_error() {
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 1.0, 0.0);

    assert!(loft(&mut topo, &[p0]).is_err());
}

#[test]
fn loft_mismatched_vertex_count_error() {
    let mut topo = Topology::new();
    let square = make_square_at(&mut topo, 1.0, 0.0);

    let tol_val = 1e-7;
    let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 1.0), tol_val));
    let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 1.0), tol_val));
    let v2 = topo.add_vertex(Vertex::new(Point3::new(0.5, 1.0, 1.0), tol_val));

    let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
    let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));

    let wire = Wire::new(
        vec![
            OrientedEdge::new(e0, true),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, true),
        ],
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    let tri = topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 1.0,
        },
    ));

    // Profiles with different vertex counts should succeed via resampling.
    let result = loft(&mut topo, &[square, tri]);
    assert!(
        result.is_ok(),
        "loft with different vertex counts should succeed via resampling"
    );
}

/// Helper: make a CW-wound square face at z=offset with given size.
fn make_cw_square_at(topo: &mut Topology, size: f64, z: f64) -> FaceId {
    let hs = size / 2.0;
    let tol_val = 1e-7;
    // CW order: v0→v3→v2→v1 (reversed from make_square_at)
    let v0 = topo.add_vertex(Vertex::new(Point3::new(-hs, -hs, z), tol_val));
    let v1 = topo.add_vertex(Vertex::new(Point3::new(-hs, hs, z), tol_val));
    let v2 = topo.add_vertex(Vertex::new(Point3::new(hs, hs, z), tol_val));
    let v3 = topo.add_vertex(Vertex::new(Point3::new(hs, -hs, z), tol_val));

    let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
    let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
    let e2 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
    let e3 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));

    let wire = Wire::new(
        vec![
            OrientedEdge::new(e0, true),
            OrientedEdge::new(e1, true),
            OrientedEdge::new(e2, true),
            OrientedEdge::new(e3, true),
        ],
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);

    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

#[test]
fn loft_cw_profiles_produces_correct_solid() {
    let mut topo = Topology::new();
    let bottom = make_cw_square_at(&mut topo, 1.0, 0.0);
    let top = make_cw_square_at(&mut topo, 1.0, 1.0);

    let solid = loft(&mut topo, &[bottom, top]).unwrap();

    // CW profiles should be auto-corrected to produce a valid solid
    // with positive volume (not inside-out).
    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    assert!(
        vol > 0.0,
        "CW-wound loft should produce positive volume, got {vol}"
    );
}

#[test]
fn loft_smooth_two_profiles_delegates() {
    // With 2 profiles, loft_smooth delegates to basic loft (ruled surfaces).
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 1.0, 0.0);
    let p1 = make_square_at(&mut topo, 1.0, 1.0);

    let solid = loft_smooth(&mut topo, &[p0, p1]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    assert_eq!(
        sh.faces().len(),
        6,
        "2-profile smooth loft should have 6 faces"
    );
}

#[test]
fn loft_smooth_three_profiles_has_nurbs() {
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 2.0, 0.0);
    let p1 = make_square_at(&mut topo, 1.0, 1.5);
    let p2 = make_square_at(&mut topo, 2.0, 3.0);

    let solid = loft_smooth(&mut topo, &[p0, p1, p2]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    // 2 caps + 4 NURBS sides = 6 faces (one surface per edge, spanning all profiles)
    assert_eq!(
        sh.faces().len(),
        6,
        "3-profile smooth loft should have 6 faces"
    );

    // Verify at least one NURBS face exists (the side surfaces).
    let has_nurbs = sh.faces().iter().any(|&fid| {
        matches!(
            topo.face(fid).expect("face").surface(),
            FaceSurface::Nurbs(_)
        )
    });
    assert!(has_nurbs, "smooth loft should produce NURBS side faces");
}

#[test]
fn loft_smooth_three_profiles_positive_volume() {
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 2.0, 0.0);
    let p1 = make_square_at(&mut topo, 1.0, 1.5);
    let p2 = make_square_at(&mut topo, 2.0, 3.0);

    let solid = loft_smooth(&mut topo, &[p0, p1, p2]).unwrap();

    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    assert!(
        vol > 0.0,
        "smooth loft should have positive volume, got {vol}"
    );
}

#[test]
fn loft_smooth_four_profiles() {
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 2.0, 0.0);
    let p1 = make_square_at(&mut topo, 1.5, 1.0);
    let p2 = make_square_at(&mut topo, 1.0, 2.0);
    let p3 = make_square_at(&mut topo, 1.5, 3.0);

    let solid = loft_smooth(&mut topo, &[p0, p1, p2, p3]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    assert_eq!(
        sh.faces().len(),
        6,
        "4-profile smooth loft should have 6 faces"
    );

    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    assert!(vol > 0.0, "smooth loft should have positive volume");
}

/// Helper: make a planar circle face whose single outer edge is an
/// analytic [`Circle3D`], centered at `(0,0,z)` with axis +Z.
fn make_circle_face_at(topo: &mut Topology, radius: f64, z: f64) -> FaceId {
    let tol_val = 1e-7;
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let center = Point3::new(0.0, 0.0, z);
    let circle = remus_math::curves::Circle3D::new(center, axis, radius).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), tol_val));
    let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane { normal: axis, d: z },
    ))
}

/// Helper: make a planar circle face whose outer edge is stored as a
/// rational NURBS curve that is geometrically a circle (exercises the
/// canonical-recognition branch).
fn make_nurbs_circle_face_at(topo: &mut Topology, radius: f64, z: f64) -> FaceId {
    let tol_val = 1e-7;
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let center = Point3::new(0.0, 0.0, z);
    let circle = remus_math::curves::Circle3D::new(center, axis, radius).unwrap();
    let nurbs =
        remus_geometry::convert::circle_to_nurbs(&circle, 0.0, 2.0 * std::f64::consts::PI).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), tol_val));
    let range = nurbs.domain();
    let mut edge = Edge::new(seam, seam, EdgeCurve::NurbsCurve(nurbs));
    edge.set_trim(Some(range));
    let edge = topo.add_edge(edge);
    let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane { normal: axis, d: z },
    ))
}

fn assert_analytic_frustum_solid(topo: &Topology, solid: SolidId, expected_volume: f64) {
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    assert_eq!(
        sh.faces().len(),
        3,
        "analytic loft must emit exactly 3 faces (two caps + one analytic side)"
    );
    let has_curved_side = sh.faces().iter().any(|&fid| {
        matches!(
            topo.face(fid).unwrap().surface(),
            FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
        )
    });
    assert!(
        has_curved_side,
        "analytic loft side face must be cylindrical/conical, not planar"
    );
    let vol = crate::measure::solid_volume(topo, solid, 0.05).unwrap();
    let rel_err = (vol - expected_volume).abs() / expected_volume;
    assert!(
        rel_err < 0.005,
        "analytic loft volume {vol} should be within 0.5% of {expected_volume} (err {:.3}%)",
        rel_err * 100.0
    );
}

#[test]
fn loft_two_circles_volume_within_0_5pct_of_truncated_cone() {
    let h = 20.0;
    let big = 10.0;
    let small = 5.0;
    let expected = std::f64::consts::PI * h / 3.0 * (big * big + big * small + small * small);

    let mut topo = Topology::new();
    let bottom = make_circle_face_at(&mut topo, big, 0.0);
    let top = make_circle_face_at(&mut topo, small, h);
    let solid = loft(&mut topo, &[bottom, top]).unwrap();
    assert_analytic_frustum_solid(&topo, solid, expected);

    let mut topo2 = Topology::new();
    let bottom2 = make_nurbs_circle_face_at(&mut topo2, big, 0.0);
    let top2 = make_nurbs_circle_face_at(&mut topo2, small, h);
    let solid2 = loft(&mut topo2, &[bottom2, top2]).unwrap();
    assert_analytic_frustum_solid(&topo2, solid2, expected);
}

#[test]
fn loft_two_equal_circles_makes_cylinder() {
    let h = 20.0;
    let r = 10.0;
    let expected = std::f64::consts::PI * r * r * h;

    let mut topo = Topology::new();
    let bottom = make_circle_face_at(&mut topo, r, 0.0);
    let top = make_circle_face_at(&mut topo, r, h);
    let solid = loft(&mut topo, &[bottom, top]).unwrap();
    assert_analytic_frustum_solid(&topo, solid, expected);
}

#[test]
fn loft_non_coaxial_circles_make_exact_oblique_band() {
    let mut topo = Topology::new();
    let bottom = make_circle_face_at(&mut topo, 10.0, 0.0);
    let tol_val = 1e-7;
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let center = Point3::new(8.0, 0.0, 20.0);
    let circle = remus_math::curves::Circle3D::new(center, axis, 5.0).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), tol_val));
    let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let wire = Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap();
    let wid = topo.add_wire(wire);
    let top = topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: axis,
            d: 20.0,
        },
    ));

    let solid = loft(&mut topo, &[bottom, top]).unwrap();
    // The laterally offset pair is an oblique frustum: Cavalieri gives the
    // same π·h·(R² + R·r + r²)/3 as the coaxial one, which the old 32-facet
    // fallback under-filled.
    let expected = std::f64::consts::PI * 20.0 / 3.0 * (100.0 + 50.0 + 25.0);
    assert_oblique_circle_loft(
        &topo,
        solid,
        &[(Point3::new(0.0, 0.0, 0.0), 10.0), (center, 5.0)],
        axis,
        expected,
    );
}

#[test]
fn loft_three_coaxial_circles_two_analytic_bands() {
    let r_big = 10.0;
    let r_mid = 5.0;
    let h = 20.0;
    let band = |ra: f64, rb: f64| std::f64::consts::PI * h / 3.0 * (ra * ra + ra * rb + rb * rb);
    let expected = band(r_big, r_mid) + band(r_mid, r_big);

    let mut topo = Topology::new();
    let p0 = make_circle_face_at(&mut topo, r_big, 0.0);
    let p1 = make_circle_face_at(&mut topo, r_mid, h);
    let p2 = make_circle_face_at(&mut topo, r_big, 2.0 * h);

    let solid = loft(&mut topo, &[p0, p1, p2]).unwrap();
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    let analytic_sides = sh
        .faces()
        .iter()
        .filter(|&&fid| {
            matches!(
                topo.face(fid).unwrap().surface(),
                FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
            )
        })
        .count();
    assert_eq!(
        analytic_sides, 2,
        "three coaxial circles should emit two analytic frustum bands"
    );

    // Measured at deflection 0.01 so the frustum chord deviation is negligible:
    // circular bands tessellate to the exact chord count (no curvature floor),
    // and a coarse mesh of an inscribed polygon legitimately under-fills volume.
    let vol = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
    let rel_err = (vol - expected).abs() / expected;
    assert!(
        rel_err < 0.005,
        "three-circle loft volume {vol} should be within 0.5% of {expected} (err {:.3}%)",
        rel_err * 100.0
    );
}

#[test]
fn loft_smooth_surface_passes_through_profiles() {
    let mut topo = Topology::new();
    let p0 = make_square_at(&mut topo, 2.0, 0.0);
    let p1 = make_square_at(&mut topo, 1.0, 2.0);
    let p2 = make_square_at(&mut topo, 2.0, 4.0);

    let solid = loft_smooth(&mut topo, &[p0, p1, p2]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    // Find a NURBS side face and verify it passes through the middle profile.
    for &fid in sh.faces() {
        let face = topo.face(fid).expect("face");
        if let FaceSurface::Nurbs(surface) = face.surface() {
            // At u=0.5 (middle profile), the surface should pass through
            // the middle profile's vertex positions. Evaluate at u=0.5, v=0.
            let mid_pt = surface.evaluate(0.5, 0.0);
            // The middle profile is at z=2.0.
            assert!(
                (mid_pt.z() - 2.0).abs() < 0.5,
                "surface at u=0.5 should be near z=2.0, got z={:.3}",
                mid_pt.z()
            );
            break;
        }
    }
}

/// Rounded-rect profile (4 Line edges + 4 Circle arc corners), CCW.
fn make_rr_arcs(topo: &mut Topology, hw: f64, hd: f64, r: f64, z: f64) -> FaceId {
    use remus_math::curves::Circle3D;
    let r = r.min(hw.min(hd));
    let cc = [
        Point3::new(hw - r, -hd + r, z),
        Point3::new(hw - r, hd - r, z),
        Point3::new(-hw + r, hd - r, z),
        Point3::new(-hw + r, -hd + r, z),
    ];
    let ap = [
        (Point3::new(hw - r, -hd, z), Point3::new(hw, -hd + r, z)),
        (Point3::new(hw, hd - r, z), Point3::new(hw - r, hd, z)),
        (Point3::new(-hw + r, hd, z), Point3::new(-hw, hd - r, z)),
        (Point3::new(-hw, -hd + r, z), Point3::new(-hw + r, -hd, z)),
    ];
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let mut v = Vec::new();
    for p in &ap {
        v.push(topo.add_vertex(Vertex::new(p.0, 1e-7)));
        v.push(topo.add_vertex(Vertex::new(p.1, 1e-7)));
    }
    let mut e = Vec::new();
    e.push(topo.add_edge(Edge::new(v[7], v[0], EdgeCurve::Line)));
    for i in 0..4 {
        e.push(topo.add_edge(Edge::new(
            v[2 * i],
            v[2 * i + 1],
            EdgeCurve::Circle(Circle3D::new(cc[i], axis, r).unwrap()),
        )));
        if i < 3 {
            e.push(topo.add_edge(Edge::new(v[2 * i + 1], v[2 * i + 2], EdgeCurve::Line)));
        }
    }
    let wire = Wire::new(
        e.iter().map(|&id| OrientedEdge::new(id, true)).collect(),
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane { normal: axis, d: z },
    ))
}

#[test]
fn loft_rounded_rect_preserves_arc_corners() {
    let mut topo = Topology::new();
    let bottom = make_rr_arcs(&mut topo, 10.0, 10.0, 2.0, 0.0);
    let top = make_rr_arcs(&mut topo, 14.0, 14.0, 3.0, 10.0);

    let solid = loft(&mut topo, &[bottom, top]).unwrap();

    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();

    // 2 caps + (4 straight walls + 4 arc corners) = 10 faces; the corners
    // must be NURBS (arcs preserved), not faceted into Line walls.
    let faces = sh.faces();
    assert_eq!(faces.len(), 10, "rounded-rect loft should have 10 faces");
    let nurbs_faces = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count();
    assert_eq!(
        nurbs_faces, 4,
        "the 4 arc corners should be NURBS side faces"
    );

    // Watertight genus-0: every edge shared by exactly 2 faces, euler == 2.
    let manifold = remus_topology::validation::validate_shell_manifold(sh, &topo);
    assert!(
        manifold.is_ok(),
        "loft should be a closed manifold: {manifold:?}"
    );
    let (f, e, v) = remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap();
    #[allow(clippy::cast_possible_wrap)]
    let euler = (v as i64) - (e as i64) + (f as i64);
    assert_eq!(
        euler, 2,
        "rounded-rect frustum should be genus-0 (F={f} E={e} V={v})"
    );

    // Volume sits between the inscribed and circumscribed box frustums and
    // exceeds the straight-chamfer (octagon) approximation — i.e. the arcs
    // genuinely bulge out. Mean cross-section ≈ rr(12,12,2.5) area.
    let vol = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
    let rr_area = |hw: f64, hd: f64, r: f64| 4.0 * hw * hd + (std::f64::consts::PI - 4.0) * r * r;
    let approx = 0.5 * (rr_area(10.0, 10.0, 2.0) + rr_area(14.0, 14.0, 3.0)) * 10.0;
    assert!(
        (vol - approx).abs() / approx < 0.02,
        "rounded-rect frustum volume {vol:.1} should be within 2% of ~{approx:.1}"
    );
}

#[test]
fn loft_multi_section_rounded_rect_preserves_arc_corners() {
    // Regression: a multi-section (>2 profile) rounded-rect loft must keep its
    // arc corners curve-preserved (NURBS/Cone), not facet them to a polygon. A
    // faceted multi-section lip/socket is what drove the gridfinity bin fuse to
    // its non-manifold mesh fallback. Before the fix this was gated out
    // (`num_profiles != 2`) and fell back to the all-planar polygon loft.
    let mut topo = Topology::new();
    let p: Vec<FaceId> = [
        (10.0, 10.0, 2.0, 0.0),
        (11.0, 11.0, 2.2, 3.0),
        (12.0, 12.0, 2.5, 6.0),
        (14.0, 14.0, 3.0, 10.0),
    ]
    .iter()
    .map(|&(hw, hd, r, z)| make_rr_arcs(&mut topo, hw, hd, r, z))
    .collect();

    let solid = loft(&mut topo, &p).unwrap();
    let s = topo.solid(solid).unwrap();
    let sh = topo.shell(s.outer_shell()).unwrap();
    let faces = sh.faces();

    // 2 caps + 3 bands * (4 straight walls + 4 arc corners) = 26 faces.
    assert_eq!(
        faces.len(),
        26,
        "4-section rounded-rect loft should have 26 faces"
    );
    let curved = faces
        .iter()
        .filter(|&&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                FaceSurface::Nurbs(_) | FaceSurface::Cone(_) | FaceSurface::Cylinder(_)
            )
        })
        .count();
    assert_eq!(
        curved, 12,
        "3 bands x 4 arc corners = 12 curve-preserved side faces (not faceted)"
    );

    let manifold = remus_topology::validation::validate_shell_manifold(sh, &topo);
    assert!(
        manifold.is_ok(),
        "multi-section loft should be a closed manifold: {manifold:?}"
    );
    let (f, e, v) = remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap();
    #[allow(clippy::cast_possible_wrap)]
    let euler = (v as i64) - (e as i64) + (f as i64);
    assert_eq!(
        euler, 2,
        "multi-section frustum should be genus-0 (F={f} E={e} V={v})"
    );
}

/// Rounded-rect profile with each 90° corner split into TWO co-circular arcs,
/// mimicking drawn rounded-rects that split corners inconsistently with size.
fn make_rr_arcs_split(topo: &mut Topology, hw: f64, hd: f64, r: f64, z: f64) -> FaceId {
    use remus_math::curves::Circle3D;
    let r = r.min(hw.min(hd));
    let cc = [
        Point3::new(hw - r, -hd + r, z),
        Point3::new(hw - r, hd - r, z),
        Point3::new(-hw + r, hd - r, z),
        Point3::new(-hw + r, -hd + r, z),
    ];
    let ap = [
        (Point3::new(hw - r, -hd, z), Point3::new(hw, -hd + r, z)),
        (Point3::new(hw, hd - r, z), Point3::new(hw - r, hd, z)),
        (Point3::new(-hw + r, hd, z), Point3::new(-hw, hd - r, z)),
        (Point3::new(-hw, -hd + r, z), Point3::new(-hw + r, -hd, z)),
    ];
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let mut v = Vec::new();
    for p in &ap {
        v.push(topo.add_vertex(Vertex::new(p.0, 1e-7)));
        v.push(topo.add_vertex(Vertex::new(p.1, 1e-7)));
    }
    let mut e = Vec::new();
    e.push(topo.add_edge(Edge::new(v[7], v[0], EdgeCurve::Line)));
    for i in 0..4 {
        let circle = Circle3D::new(cc[i], axis, r).unwrap();
        // Bisector midpoint splits the 90° corner into two co-circular sub-arcs.
        let bis = ((ap[i].0 - cc[i]) + (ap[i].1 - cc[i])).normalize().unwrap();
        let vmid = topo.add_vertex(Vertex::new(cc[i] + bis * r, 1e-7));
        e.push(topo.add_edge(Edge::new(v[2 * i], vmid, EdgeCurve::Circle(circle.clone()))));
        e.push(topo.add_edge(Edge::new(vmid, v[2 * i + 1], EdgeCurve::Circle(circle))));
        if i < 3 {
            e.push(topo.add_edge(Edge::new(v[2 * i + 1], v[2 * i + 2], EdgeCurve::Line)));
        }
    }
    let wire = Wire::new(
        e.iter().map(|&id| OrientedEdge::new(id, true)).collect(),
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane { normal: axis, d: z },
    ))
}

#[test]
fn loft_merges_split_arc_corners_for_curve_preservation() {
    // One profile with single-arc corners + one whose corners are split into two
    // co-circular sub-arcs (as drawn rounded-rects do, inconsistently). They must
    // still curve-preserve: the arc-merge pass canonicalizes both so their edges
    // align. Before the fix the 8-vs-12 edge-count mismatch forced the faceted
    // polygon loft — the gridfinity lip's faceting.
    let mut topo = Topology::new();
    let single = make_rr_arcs(&mut topo, 12.0, 12.0, 2.5, 0.0); // 8 edges, corner center 9.5
    let split = make_rr_arcs_split(&mut topo, 11.5, 11.5, 2.0, 5.0); // 12 edges, center 9.5
    let solid = loft(&mut topo, &[single, split]).unwrap();
    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    let curved = sh
        .faces()
        .iter()
        .filter(|&&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                FaceSurface::Nurbs(_) | FaceSurface::Cone(_) | FaceSurface::Cylinder(_)
            )
        })
        .count();
    assert_eq!(
        curved, 4,
        "the 4 corners must stay curve-preserved despite the split-arc profile"
    );
    let manifold = remus_topology::validation::validate_shell_manifold(sh, &topo);
    assert!(
        manifold.is_ok(),
        "split-arc loft should be a closed manifold: {manifold:?}"
    );
    let (f, e, v) = remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap();
    #[allow(clippy::cast_possible_wrap)]
    let euler = (v as i64) - (e as i64) + (f as i64);
    assert_eq!(
        euler, 2,
        "split-arc frustum should be genus-0 (F={f} E={e} V={v})"
    );
}

#[test]
fn loft_planar_caps_unchanged() {
    // Regression: planar profiles must still produce flat `Plane` caps with the
    // section normal (±Z) and no reversal — the non-planar cap work must not
    // touch the planar path.
    let mut topo = Topology::new();
    let bottom = make_square_at(&mut topo, 1.0, 0.0);
    let top = make_square_at(&mut topo, 1.0, 1.0);
    let solid = loft(&mut topo, &[bottom, top]).unwrap();
    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert_eq!(sh.faces().len(), 6);

    // The two z-facing faces are the caps: still planar, normal ±Z, not reversed.
    let cap_reversed: Vec<bool> = sh
        .faces()
        .iter()
        .filter_map(|&fid| {
            let f = topo.face(fid).unwrap();
            match f.surface() {
                FaceSurface::Plane { normal, .. } if normal.z().abs() > 0.99 => {
                    Some(f.is_reversed())
                }
                _ => None,
            }
        })
        .collect();
    assert_eq!(cap_reversed.len(), 2, "two ±Z planar caps");
    assert!(
        cap_reversed.iter().all(|&rev| !rev),
        "planar caps must never be reversed"
    );
}

/// A non-planar profile: a 4-corner Coons patch whose corners are staggered in
/// z (genuinely non-coplanar boundary) and whose surface bulges (non-planar
/// NURBS). Corner order p00,p10,p11,p01 is CCW in XY.
fn make_saddle_patch(topo: &mut Topology, half: f64, z: f64) -> FaceId {
    let h = half;
    let bottom = vec![
        Point3::new(-h, -h, z + 0.3),
        Point3::new(0.0, -h, z + 0.6),
        Point3::new(h, -h, z - 0.3),
    ];
    let right = vec![
        Point3::new(h, -h, z - 0.3),
        Point3::new(h, 0.0, z + 0.6),
        Point3::new(h, h, z + 0.3),
    ];
    let top = vec![
        Point3::new(-h, h, z - 0.3),
        Point3::new(0.0, h, z + 0.6),
        Point3::new(h, h, z + 0.3),
    ];
    let left = vec![
        Point3::new(-h, -h, z + 0.3),
        Point3::new(-h, 0.0, z + 0.6),
        Point3::new(-h, h, z - 0.3),
    ];
    fill_coons_patch(topo, &[bottom, right, top, left]).unwrap()
}

#[test]
fn loft_two_nonplanar_coons_patches_is_valid_solid() {
    let mut topo = Topology::new();
    let bottom = make_saddle_patch(&mut topo, 2.0, 0.0);
    let top = make_saddle_patch(&mut topo, 2.0, 6.0);
    assert!(
        !topo.face(bottom).unwrap().surface().is_planar(),
        "saddle profile must be a non-planar (NURBS) face"
    );

    let solid = loft(&mut topo, &[bottom, top]).unwrap();
    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();

    // 2 caps + 4 ruled sides = 6 faces. The non-planar section boundaries are
    // filled by bilinear (NURBS) caps whose iso-boundaries are the ring chords.
    assert_eq!(sh.faces().len(), 6);
    let nurbs_caps = sh
        .faces()
        .iter()
        .filter(|&&fid| matches!(topo.face(fid).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count();
    assert_eq!(
        nurbs_caps, 2,
        "non-planar ring caps are bilinear NURBS fills"
    );

    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid(),
        "non-planar loft must be a valid solid"
    );
    // ~4×4 cross-section, height ~6 → ≈96. The bilinear caps fill only the
    // section (no parent-surface overfill), so the volume stays near the prism;
    // a reused-surface overfill would inflate it well past this bound.
    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    assert!(
        vol > 85.0 && vol < 110.0,
        "non-planar loft volume out of expected range, got {vol}"
    );
}

/// A profile whose *surface* is non-planar (a sphere) but whose *boundary* is a
/// planar ring at `ring_z`. Previously rejected by the planar-only gate; now
/// accepted and closed with an exact flat (`Plane`) cap.
fn make_sphere_cap_patch(topo: &mut Topology, ring_z: f64, ring_r: f64, sphere_r: f64) -> FaceId {
    let sphere =
        remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), sphere_r).unwrap();
    let pts = [
        Point3::new(ring_r, 0.0, ring_z),
        Point3::new(0.0, ring_r, ring_z),
        Point3::new(-ring_r, 0.0, ring_z),
        Point3::new(0.0, -ring_r, ring_z),
    ];
    let v: Vec<_> = pts
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();
    let e: Vec<_> = (0..4)
        .map(|i| topo.add_edge(Edge::new(v[i], v[(i + 1) % 4], EdgeCurve::Line)))
        .collect();
    let wire = Wire::new(
        e.iter().map(|&id| OrientedEdge::new(id, true)).collect(),
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    topo.add_face(Face::new(wid, vec![], FaceSurface::Sphere(sphere)))
}

#[test]
fn loft_nonplanar_surface_planar_boundary_makes_flat_caps() {
    let mut topo = Topology::new();
    // Two sphere-surfaced sections with planar boundaries (z=4, z=14), same
    // radius-3 diamond cross-section → a square prism of height 10.
    let bottom = make_sphere_cap_patch(&mut topo, 4.0, 3.0, 5.0);
    let top = make_sphere_cap_patch(&mut topo, 14.0, 3.0, 5.0);
    assert!(
        !topo.face(bottom).unwrap().surface().is_planar(),
        "the profile surface is non-planar (a sphere)"
    );

    let solid = loft(&mut topo, &[bottom, top]).unwrap();
    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert_eq!(sh.faces().len(), 6);
    // Planar boundary ⇒ exact flat caps (the curved surface is not reused).
    let planar = sh
        .faces()
        .iter()
        .filter(|&&fid| topo.face(fid).unwrap().surface().is_planar())
        .count();
    assert_eq!(planar, 6, "all faces (caps + sides) are planar");

    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid()
    );
    // Diamond cross-section area = d²/2 = 6²/2 = 18, height 10 → exact prism 180.
    let vol = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
    assert!(
        (vol - 180.0).abs() / 180.0 < 0.01,
        "prism volume should be ~180, got {vol}"
    );
}

#[test]
fn loft_nonplanar_boundary_over_4_edges_caps_with_coons() {
    // A genuinely non-planar 5-edge section boundary is capped by a Coons
    // patch whose boundary iso-curves are exactly the ring chords. The two
    // sections are identical warps 5 apart, so every side ruling is vertical
    // and the solid is a translation sweep: volume = 5 × (XY shoelace area),
    // exactly.
    let mut topo = Topology::new();
    let mk = |topo: &mut Topology, z: f64| -> FaceId {
        let pts = [
            Point3::new(2.0, 0.0, z + 0.4),
            Point3::new(0.6, 1.9, z - 0.4),
            Point3::new(-1.6, 1.2, z + 0.4),
            Point3::new(-1.6, -1.2, z - 0.4),
            Point3::new(0.6, -1.9, z + 0.4),
        ];
        let v: Vec<_> = pts
            .iter()
            .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let e: Vec<_> = (0..5)
            .map(|i| topo.add_edge(Edge::new(v[i], v[(i + 1) % 5], EdgeCurve::Line)))
            .collect();
        let wire = Wire::new(
            e.iter().map(|&id| OrientedEdge::new(id, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(
            wid,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: z,
            },
        ))
    };
    let p0 = mk(&mut topo, 0.0);
    let p1 = mk(&mut topo, 5.0);
    let solid = loft(&mut topo, &[p0, p1]).unwrap();

    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert_eq!(sh.faces().len(), 7, "5 side walls + 2 Coons caps");
    let nurbs_caps = sh
        .faces()
        .iter()
        .filter(|&&fid| matches!(topo.face(fid).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count();
    assert!(nurbs_caps >= 2, "both caps are NURBS Coons fills");

    assert!(
        crate::validate::validate_solid(&topo, solid)
            .unwrap()
            .is_valid(),
        "Coons-capped loft must be a valid solid"
    );

    // Shoelace area of the pentagon's XY projection.
    let xy = [
        (2.0, 0.0),
        (0.6, 1.9),
        (-1.6, 1.2),
        (-1.6, -1.2),
        (0.6, -1.9),
    ];
    let mut area2: f64 = 0.0;
    for i in 0..5 {
        let (x0, y0) = xy[i];
        let (x1, y1) = xy[(i + 1) % 5];
        area2 += x0 * y1 - x1 * y0;
    }
    let expected = 5.0 * area2.abs() / 2.0;
    let vol = crate::measure::solid_volume(&topo, solid, 0.05).unwrap();
    assert!(
        ((vol - expected) / expected).abs() < 0.005,
        "translation-sweep volume should be {expected}, got {vol}"
    );
}

#[test]
fn loft_smooth_three_nonplanar_patches_positive_volume() {
    // loft_smooth's own cap path (3+ profiles → NURBS side faces) must also fill
    // the non-planar section boundaries (bilinear caps) and stay sane.
    let mut topo = Topology::new();
    let p0 = make_saddle_patch(&mut topo, 2.0, 0.0);
    let p1 = make_saddle_patch(&mut topo, 1.5, 4.0);
    let p2 = make_saddle_patch(&mut topo, 2.0, 8.0);

    let solid = loft_smooth(&mut topo, &[p0, p1, p2]).unwrap();
    let sh = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    // 2 caps + 4 NURBS sides = 6 faces.
    assert_eq!(sh.faces().len(), 6);

    let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
    assert!(
        vol > 0.0,
        "smooth non-planar loft must have positive volume, got {vol}"
    );
}

/// Rounded-rect profile whose corner arcs are stored as `curve` NURBS (the
/// form a brepjs sketch delivers) or as native circles.
fn make_rounded_rect_face_at(
    topo: &mut Topology,
    half: f64,
    r: f64,
    z: f64,
    nurbs_arcs: bool,
) -> FaceId {
    let tol_val = 1e-7;
    let c = half - r;
    let corners = [
        (c, c, 0.0_f64),
        (-c, c, 90.0),
        (-c, -c, 180.0),
        (c, -c, 270.0),
    ];
    let pt = |cx: f64, cy: f64, deg: f64| {
        let a = deg.to_radians();
        Point3::new(cx + r * a.cos(), cy + r * a.sin(), z)
    };
    let mut vids = Vec::new();
    for &(cx, cy, a0) in &corners {
        vids.push(topo.add_vertex(Vertex::new(pt(cx, cy, a0), tol_val)));
        vids.push(topo.add_vertex(Vertex::new(pt(cx, cy, a0 + 90.0), tol_val)));
    }
    let mut oes = Vec::new();
    for (k, &(cx, cy, _)) in corners.iter().enumerate() {
        let circle =
            remus_math::curves::Circle3D::new(Point3::new(cx, cy, z), Vec3::new(0.0, 0.0, 1.0), r)
                .unwrap();
        let curve = if nurbs_arcs {
            let (a0, a1) = EdgeCurve::Circle(circle.clone()).domain_with_endpoints(
                topo.vertex(vids[2 * k]).unwrap().point(),
                topo.vertex(vids[2 * k + 1]).unwrap().point(),
            );
            EdgeCurve::NurbsCurve(
                remus_geometry::convert::circle_to_nurbs(&circle, a0, a1).unwrap(),
            )
        } else {
            EdgeCurve::Circle(circle)
        };
        let arc = topo.add_edge(Edge::new(vids[2 * k], vids[2 * k + 1], curve));
        oes.push(OrientedEdge::new(arc, true));
        let line = topo.add_edge(Edge::new(
            vids[2 * k + 1],
            vids[(2 * k + 2) % 8],
            EdgeCurve::Line,
        ));
        oes.push(OrientedEdge::new(line, true));
    }
    let wid = topo.add_wire(Wire::new(oes, true).unwrap());
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

/// A multi-section rounded-rect loft whose corner arcs arrive as NURBS (the
/// brepjs sketch form — the gridfinity socket) must produce the same analytic
/// cone/cylinder corners as one built from native circles, not a faceted
/// polygon frustum.
#[test]
fn loft_rounded_rect_nurbs_arcs_stays_analytic() {
    let sections: [(f64, f64, f64); 5] = [
        (20.75, 3.75, 0.0),
        (20.75, 3.75, 0.9),
        (19.85, 2.85, 1.7),
        (19.85, 2.85, 3.5),
        (18.95, 1.95, 4.75),
    ];
    let mut volumes = Vec::new();
    // (nurbs_arcs, downward): the gridfinity socket combines both — sketch
    // NURBS arcs AND a downward stack (top section at z=0, lofting to −h),
    // which winds every profile CW about the stacking axis.
    for (nurbs_arcs, down) in [(false, false), (true, false), (false, true), (true, true)] {
        let mut topo = Topology::new();
        let profs: Vec<FaceId> = sections
            .iter()
            .map(|&(half, r, z)| {
                let z = if down { -z } else { z };
                make_rounded_rect_face_at(&mut topo, half, r, z, nurbs_arcs)
            })
            .collect();
        let solid = loft(&mut topo, &profs).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        let curved = faces
            .iter()
            .filter(|&&f| {
                matches!(
                    topo.face(f).unwrap().surface(),
                    FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
                )
            })
            .count();
        assert!(
            curved >= 16,
            "nurbs_arcs={nurbs_arcs} down={down}: only {curved} cone/cylinder corner faces — \
             loft faceted the arcs"
        );
        volumes.push(crate::measure::solid_volume(&topo, solid, 0.01).unwrap());
    }
    for (k, v) in volumes.iter().enumerate().skip(1) {
        let diff = (volumes[0] - v).abs() / volumes[0];
        assert!(
            diff < 1e-3,
            "variant {k} volume {v} deviates from native upward loft {}",
            volumes[0]
        );
    }
}

#[test]
fn ruled_circle_rails_follow_reversed_major_seam_authority() {
    use std::f64::consts::FRAC_PI_2;

    let c0 = remus_math::curves::Circle3D::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0,
    )
    .unwrap();
    let c1 = remus_math::curves::Circle3D::new(
        Point3::new(0.75, -0.25, 3.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
    )
    .unwrap();

    for range in [(5.5, 5.5 + 3.0 * FRAC_PI_2), (5.5, 5.5 - 3.0 * FRAC_PI_2)] {
        let surface = ruled_arc_surface(&c0, range, &c1, range).unwrap();
        for (v, angle) in [
            (0.0, range.0),
            (0.5, f64::midpoint(range.0, range.1)),
            (1.0, range.1),
        ] {
            assert!((surface.evaluate(0.0, v) - c0.evaluate(angle)).length() < 1e-9);
            assert!((surface.evaluate(1.0, v) - c1.evaluate(angle)).length() < 1e-9);
        }
    }
}

/// Mutation survivor (2026-09-06): `!finite || <= 0` → `&&` let a finite
/// zero or negative ring radius through the coaxial stack's input guard.
#[test]
fn coaxial_band_stack_refuses_non_positive_and_non_finite_radii() {
    for radii in [
        [1.0, 0.0],
        [1.0, -1.0],
        [1.0, f64::NAN],
        [f64::INFINITY, 1.0],
    ] {
        let mut topo = Topology::new();
        let error = build_coaxial_band_stack(&mut topo, &[0.0, 1.0], &radii).unwrap_err();
        assert!(
            matches!(
                &error,
                crate::OperationsError::InvalidInput { reason } if reason.contains("positive-radius")
            ),
            "{radii:?}: expected the typed input refusal, got {error:?}"
        );
    }
}

/// Every stored pcurve must lift to its owning 3D edge in the same oriented
/// parameter, including curved rails and reversed wall uses.
fn assert_rectangular_wall_boundaries(topo: &Topology, solid: SolidId) -> usize {
    let mut curved = 0;
    for face_id in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        let FaceSurface::Nurbs(surface) = face.surface() else {
            continue;
        };
        curved += 1;
        for oriented in topo.wire(face.outer_wire()).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let pcurve = topo
                .pcurve_oriented(oriented.edge(), face_id, oriented.is_forward())
                .expect("exact wall boundary has an oriented pcurve");
            let (lo, hi) = edge.strict_domain().unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            for fraction in [0.0, 0.2, 0.5, 0.8, 1.0] {
                let parameter = if oriented.is_forward() {
                    lo + fraction * (hi - lo)
                } else {
                    hi - fraction * (hi - lo)
                };
                let point = edge.curve().evaluate_with_endpoints(parameter, start, end);
                let uv = pcurve
                    .evaluate(pcurve.t_start() + fraction * (pcurve.t_end() - pcurve.t_start()));
                let lifted = surface.evaluate(uv.x(), uv.y());
                assert!((point - lifted).length() < Tolerance::new().linear);
            }
        }
    }
    curved
}

#[test]
fn loft_twisted_independent_squares_have_exact_bilinear_walls() {
    use remus_math::mat::Mat4;
    for placement in [0.0, 10.0, 100.0] {
        let mut topo = Topology::new();
        let first = make_square_at(&mut topo, 2.0, 1.0);
        // Match the public fixture's independent square over [0, 2]^2.
        crate::transform::transform_face(&mut topo, first, &Mat4::translation(1.0, 1.0, 0.0))
            .unwrap();
        let second = crate::copy::copy_face(&mut topo, first).unwrap();
        let original = face_polygon(&topo, first).unwrap();
        crate::transform::transform_face(
            &mut topo,
            second,
            &(Mat4::translation(0.0, 0.0, 2.0) * Mat4::rotation_z(0.5)),
        )
        .unwrap();
        assert_eq!(face_polygon(&topo, first).unwrap(), original);
        let rigid = Mat4::translation(placement, -placement, placement) * Mat4::rotation_x(0.3);
        for profile in [first, second] {
            crate::transform::transform_face(&mut topo, profile, &rigid).unwrap();
        }
        let input_points = [
            face_polygon(&topo, first).unwrap(),
            face_polygon(&topo, second).unwrap(),
        ];
        let solid = loft(&mut topo, &[first, second]).unwrap();
        assert!(
            crate::validate::validate_solid(&topo, solid)
                .unwrap()
                .is_valid()
        );
        assert_eq!(assert_rectangular_wall_boundaries(&topo, solid), 4);
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
        assert!(crate::tessellate::is_watertight(&mesh));
        assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap(),
            (6, 12, 8)
        );
        // A(t)=(1-t)I+tR(theta), so integrating the horizontal square area
        // gives h*4*(2+cos(theta))/3. Translation and rigid rotation preserve it.
        let expected = 8.0 * (2.0 + 0.5_f64.cos()) / 3.0;
        let volume = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
        assert!((volume - expected).abs() < 1e-7, "{volume} != {expected}");
        assert_eq!(face_polygon(&topo, first).unwrap(), input_points[0]);
        assert_eq!(face_polygon(&topo, second).unwrap(), input_points[1]);
    }
}

#[test]
fn smooth_loft_middle_offset_has_shared_curved_rails_and_exact_volume() {
    use remus_math::mat::Mat4;
    for placement in [0.0, 10.0, 100.0] {
        let mut topo = Topology::new();
        let first = make_square_at(&mut topo, 2.0, 1.0);
        // Match the public fixture's independent square over [0, 2]^2.
        crate::transform::transform_face(&mut topo, first, &Mat4::translation(1.0, 1.0, 0.0))
            .unwrap();
        let middle = crate::copy::copy_face(&mut topo, first).unwrap();
        let last = crate::copy::copy_face(&mut topo, first).unwrap();
        crate::transform::transform_face(&mut topo, middle, &Mat4::translation(1.0, 0.0, 1.0))
            .unwrap();
        crate::transform::transform_face(&mut topo, last, &Mat4::translation(0.0, 0.0, 2.0))
            .unwrap();
        let rigid = Mat4::translation(placement, -placement, placement) * Mat4::rotation_y(0.2);
        for profile in [first, middle, last] {
            crate::transform::transform_face(&mut topo, profile, &rigid).unwrap();
        }
        let profiles = [first, middle, last];
        let inputs: Vec<_> = profiles
            .iter()
            .map(|&face| face_polygon(&topo, face).unwrap())
            .collect();
        let solid = loft_smooth(&mut topo, &profiles).unwrap();
        let report = crate::validate::validate_solid(&topo, solid).unwrap();
        assert!(report.is_valid(), "{report:?}");
        assert_eq!(assert_rectangular_wall_boundaries(&topo, solid), 4);
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
        assert!(crate::tessellate::is_watertight(&mesh));
        assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
        assert_eq!(
            remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap(),
            (6, 12, 8)
        );
        let edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
        let rails: Vec<_> = edges
            .iter()
            .filter(|&&id| matches!(topo.edge(id).unwrap().curve(), EdgeCurve::NurbsCurve(_)))
            .collect();
        assert_eq!(rails.len(), 4, "one shared rail at each junction");
        for &edge in &rails {
            let edge = topo.edge(*edge).unwrap();
            let midpoint = edge.curve().evaluate_with_endpoints(
                0.5,
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            );
            assert!(
                inputs[1]
                    .iter()
                    .any(|&point| (point - midpoint).length() < 1e-7)
            );
        }
        // Every interpolated cross-section is merely a translated 2x2 square.
        // The axial coordinate is affine from z=1 to z=3, so volume is 4*2.
        let volume = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
        assert!((volume - 8.0).abs() < 1e-7, "volume {volume}");
        for (&face, expected) in profiles.iter().zip(&inputs) {
            assert_eq!(face_polygon(&topo, face).unwrap(), *expected);
        }
    }
}

#[test]
fn smooth_loft_uses_one_profile_parameter_on_every_wall() {
    use remus_math::mat::Mat4;
    let mut topo = Topology::new();
    let first = make_square_at(&mut topo, 2.0, 0.0);
    let middle = make_square_at(&mut topo, 1.0, 1.0);
    let last = make_square_at(&mut topo, 3.0, 4.0);
    crate::transform::transform_face(&mut topo, middle, &Mat4::translation(0.5, 0.0, 0.0)).unwrap();
    crate::transform::transform_face(&mut topo, last, &Mat4::translation(-0.3, 0.0, 0.0)).unwrap();
    let profiles = [first, middle, last];
    let points: Vec<_> = profiles
        .iter()
        .map(|&face| face_polygon(&topo, face).unwrap())
        .collect();
    let first_band: f64 = points[0]
        .iter()
        .zip(&points[1])
        .map(|(&a, &b)| (b - a).length())
        .sum();
    let last_band: f64 = points[1]
        .iter()
        .zip(&points[2])
        .map(|(&a, &b)| (b - a).length())
        .sum();
    let parameters = [0.0, first_band / (first_band + last_band), 1.0];
    let solid = loft_smooth(&mut topo, &profiles).unwrap();
    assert_eq!(assert_rectangular_wall_boundaries(&topo, solid), 4);
    let walls: Vec<_> = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .collect();
    for (index, &face) in walls.iter().enumerate() {
        let FaceSurface::Nurbs(surface) = topo.face(face).unwrap().surface() else {
            unreachable!()
        };
        for (ring, &parameter) in points.iter().zip(&parameters) {
            for v in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let expected = ring[index] + (ring[(index + 1) % 4] - ring[index]) * v;
                assert!((surface.evaluate(parameter, v) - expected).length() < 1e-7);
            }
        }
    }
}

fn loft_live_counts(topo: &Topology) -> [usize; 11] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_compounds(),
        topo.num_compsolids(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    ]
}

#[test]
fn loft_late_failure_rolls_back_live_topology_and_history() {
    use remus_topology::journal::EntityKey;
    let mut topo = Topology::new();
    let first = make_square_at(&mut topo, 2.0, 0.0);
    let second = crate::copy::copy_face(&mut topo, first).unwrap();
    let operation = topo.journal_begin("loft rollback fixture");
    topo.journal_record_barrier(operation, vec![EntityKey::face(first.index())]);
    let counts = loft_live_counts(&topo);
    let journal = topo.journal().snapshot();
    let first_before = format!("{:?}", topo.face(first));
    let second_before = format!("{:?}", topo.face(second));
    let points = face_polygon(&topo, first).unwrap();
    let slots = topo.allocated_slot_count();
    for smooth in [false, true] {
        let result = if smooth {
            loft_smooth(&mut topo, &[first, second])
        } else {
            loft(&mut topo, &[first, second])
        };
        assert!(matches!(
            result,
            Err(crate::OperationsError::Math(
                remus_math::MathError::ZeroVector
            ))
        ));
        assert_eq!(loft_live_counts(&topo), counts);
        assert_eq!(topo.journal().snapshot(), journal);
        assert_eq!(format!("{:?}", topo.face(first)), first_before);
        assert_eq!(format!("{:?}", topo.face(second)), second_before);
        assert_eq!(face_polygon(&topo, first).unwrap(), points);
        assert_eq!(face_polygon(&topo, second).unwrap(), points);
    }
    // Failed handles remain retired above the old allocation high-water mark.
    assert!(topo.allocated_slot_count() > slots);
}

fn face_from_polygon_junctions(topo: &mut Topology, points: &[Point3]) -> FaceId {
    let wire = make_polygon_wire(topo, points, Tolerance::new().linear).unwrap();
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: points[0].z(),
        },
    ))
}

#[test]
fn repeated_polygon_junctions_keep_closed_ruled_and_smooth_lofts() {
    use remus_math::mat::Mat4;
    for smooth in [false, true] {
        for offset in [0.0, 100.0] {
            let mut topo = Topology::new();
            let profiles: Vec<_> = [(1.0, 0.0), (2.0, 1.0), (3.0, 0.0)]
                .into_iter()
                .map(|(z, shift)| {
                    face_from_polygon_junctions(
                        &mut topo,
                        &[
                            Point3::new(shift, 0.0, z),
                            Point3::new(shift + 2.0, 0.0, z),
                            Point3::new(shift + 2.0, 0.0, z),
                            Point3::new(shift + 2.0, 2.0, z),
                            Point3::new(shift, 2.0, z),
                            Point3::new(shift, 0.0, z),
                        ],
                    )
                })
                .collect();
            let rigid = Mat4::translation(offset, -offset, offset) * Mat4::rotation_y(0.2);
            for &profile in &profiles {
                crate::transform::transform_face(&mut topo, profile, &rigid).unwrap();
            }
            let inputs: Vec<_> = profiles
                .iter()
                .map(|&face| face_polygon(&topo, face).unwrap())
                .collect();
            let solid = if smooth {
                loft_smooth(&mut topo, &profiles).unwrap()
            } else {
                loft(&mut topo, &profiles).unwrap()
            };
            assert!(
                crate::validate::validate_solid(&topo, solid)
                    .unwrap()
                    .is_valid()
            );
            let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.05).unwrap();
            assert!(crate::tessellate::is_watertight(&mesh));
            assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
            let expected_counts = if smooth { (6, 12, 8) } else { (10, 20, 12) };
            assert_eq!(
                remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap(),
                expected_counts
            );
            // Repeated junctions add no material. Every section translates a
            // square of area 4, through an axial height of 2.
            let volume = crate::measure::solid_volume(&topo, solid, 0.01).unwrap();
            assert!((volume - 8.0).abs() < 1e-7, "volume {volume}");
            if smooth {
                assert_eq!(assert_rectangular_wall_boundaries(&topo, solid), 4);
            }
            for (&profile, input) in profiles.iter().zip(inputs) {
                assert_eq!(face_polygon(&topo, profile).unwrap(), input);
            }
        }
    }
}

#[test]
fn wholly_repeated_polygon_junctions_refuse_without_mutation() {
    use remus_topology::journal::EntityKey;
    let mut topo = Topology::new();
    let profiles: Vec<_> = [0.0, 1.0, 2.0]
        .into_iter()
        .map(|z| face_from_polygon_junctions(&mut topo, &[Point3::new(0.0, 0.0, z); 4]))
        .collect();
    let operation = topo.journal_begin("collapsed loft fixture");
    topo.journal_record_barrier(operation, vec![EntityKey::face(profiles[0].index())]);
    let counts = loft_live_counts(&topo);
    let slots = topo.allocated_slot_count();
    let journal = topo.journal().snapshot();
    for smooth in [false, true] {
        let result = if smooth {
            loft_smooth(&mut topo, &profiles)
        } else {
            loft(&mut topo, &profiles)
        };
        assert!(matches!(
            result,
            Err(crate::OperationsError::InvalidInput { .. })
        ));
        assert_eq!(loft_live_counts(&topo), counts);
        assert_eq!(topo.allocated_slot_count(), slots);
        assert_eq!(topo.journal().snapshot(), journal);
    }
}

/// A planar profile bounded by one full-turn circle about `axis`, stored as
/// an analytic circle or as its exact rational NURBS (the sketch form).
fn circle_section(
    topo: &mut Topology,
    center: Point3,
    axis: Vec3,
    radius: f64,
    as_nurbs: bool,
) -> FaceId {
    let axis = axis.normalize().unwrap();
    let circle = remus_math::curves::Circle3D::new(center, axis, radius).unwrap();
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let curve = if as_nurbs {
        EdgeCurve::NurbsCurve(
            remus_geometry::convert::circle_to_nurbs(&circle, 0.0, std::f64::consts::TAU).unwrap(),
        )
    } else {
        EdgeCurve::Circle(circle)
    };
    let range = match &curve {
        EdgeCurve::NurbsCurve(curve) => curve.domain(),
        EdgeCurve::Circle(_) => (0.0, std::f64::consts::TAU),
        _ => unreachable!(),
    };
    let mut edge = Edge::new(seam, seam, curve);
    edge.set_trim(Some(range));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: axis,
            d: axis.dot(Vec3::new(center.x(), center.y(), center.z())),
        },
    ))
}

/// Oblique frustum volume by Cavalieri: only the axial height matters.
fn oblique_frustum_volume(lower: (Point3, f64), upper: (Point3, f64), axis: Vec3) -> f64 {
    let height = (upper.0 - lower.0).dot(axis.normalize().unwrap()).abs();
    let (big, small) = (lower.1, upper.1);
    std::f64::consts::PI * height / 3.0 * small.mul_add(small, big.mul_add(big, big * small))
}

/// The closed-form oblique stack at an axial height: the section circle's
/// centre and radius, blended linearly between the bracketing sections.
fn oblique_section_at(sections: &[(Point3, f64)], axis: Vec3, point: Point3) -> (Point3, f64) {
    let origin = sections[0].0;
    let height = |p: Point3| (p - origin).dot(axis);
    let h = height(point);
    let band = sections
        .windows(2)
        .find(|pair| {
            let (a, b) = (height(pair[0].0), height(pair[1].0));
            h >= a.min(b) - 1e-9 && h <= a.max(b) + 1e-9
        })
        .expect("sample lies between two sections");
    let (a, b) = (height(band[0].0), height(band[1].0));
    let t = ((h - a) / (b - a)).clamp(0.0, 1.0);
    (
        band[0].0 + (band[1].0 - band[0].0) * t,
        (band[1].1 - band[0].1).mul_add(t, band[0].1),
    )
}

/// Every #1253 invariant for an oblique circle loft through `sections`
/// (centre, radius) in stacking order:
/// - two planar caps and one curved band per section pair, nothing faceted;
/// - one exact full-turn circle edge per section, at its centre and radius;
/// - every band point on the closed-form section circle at its height, with
///   the oriented B-rep normal pointing out of that circle;
/// - strict validation, the exact Cavalieri volume, and a closed,
///   consistently wound, outward display mesh.
fn assert_oblique_circle_loft(
    topo: &Topology,
    solid: SolidId,
    sections: &[(Point3, f64)],
    axis: Vec3,
    expected_volume: f64,
) {
    let axis = axis.normalize().unwrap();
    let scale = sections.iter().map(|s| s.1).fold(0.0, f64::max);
    let report = crate::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "oblique loft must validate: {report:?}");

    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let planes = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Plane { .. }))
        .count();
    let bands: Vec<FaceId> = faces
        .iter()
        .copied()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .collect();
    assert_eq!(planes, 2, "only the two caps may be planar");
    assert_eq!(
        bands.len(),
        sections.len() - 1,
        "one curved band per section pair"
    );
    assert_eq!(faces.len(), sections.len() + 1, "no faceted side faces");

    let edges = remus_topology::explorer::solid_edges(topo, solid).unwrap();
    let mut circles: Vec<_> = edges
        .iter()
        .filter_map(|&e| match topo.edge(e).unwrap().curve() {
            EdgeCurve::Circle(c) => Some(c.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(
        circles.len(),
        sections.len(),
        "one exact rim circle per section"
    );
    for &(center, radius) in sections {
        let index = circles.iter().position(|c| {
            (c.center() - center).length() <= 1e-9 * scale.max(1.0)
                && (c.radius() - radius).abs() <= 1e-9 * scale.max(1.0)
                && c.normal().dot(axis).abs() >= 1.0 - 1e-12
        });
        assert!(
            index.is_some(),
            "no exact rim for section {center:?} r {radius}"
        );
        circles.remove(index.unwrap());
    }

    for &band in &bands {
        let face = topo.face(band).unwrap();
        let FaceSurface::Nurbs(surface) = face.surface() else {
            unreachable!()
        };
        for i in 0..=12 {
            for j in 0..48 {
                let (u, v) = (f64::from(i) / 12.0, (f64::from(j) + 0.25) / 48.0);
                let point = surface.evaluate(u, v);
                let (center, radius) = oblique_section_at(sections, axis, point);
                let offset = point - center;
                let axial = offset.dot(axis);
                let radial = offset - axis * axial;
                assert!(
                    axial.abs() <= 1e-9 * scale.max(1.0)
                        && (radial.length() - radius).abs() <= 1e-9 * scale.max(1.0),
                    "band point {point:?} leaves the exact section circle: \
                     radial {} vs {radius}",
                    radial.length()
                );
                let mut normal = surface.normal(u, v).unwrap();
                if face.is_reversed() {
                    normal = normal * -1.0;
                }
                assert!(
                    normal.dot(radial) > 0.0,
                    "oriented band normal must point out of the section circle"
                );
            }
        }
    }

    // The face integrals witness each band's full natural rectangle, so the
    // Gauss mass properties read the exact Cavalieri volume. The centroid
    // checks the lateral placement too: each section's area sits at its own
    // centre, so the centroid blends the section centres by ∫ t·r(t)² dt.
    let properties = crate::measure::mass_properties(topo, solid).unwrap();
    let relative = (properties.mass - expected_volume).abs() / expected_volume;
    assert!(
        relative < 1e-9,
        "oblique loft volume {} vs exact {expected_volume} (relative {relative:e})",
        properties.mass
    );
    if let [(c0, big), (c1, small)] = *sections {
        let k = small - big;
        let weight = k.mul_add(k / 3.0, big.mul_add(big, big * k));
        let moment = (k * k).mul_add(0.25, big.mul_add(big * 0.5, 2.0 * big * k / 3.0));
        let centroid = c0 + (c1 - c0) * (moment / weight);
        assert!(
            (properties.center - centroid).length() <= 1e-9 * scale.max(1.0),
            "oblique loft centroid {:?} vs exact {centroid:?}",
            properties.center
        );
    }
    // `solid_volume` reads a body of planar caps and whole full-turn bands
    // off the same face integrals, so it too reads the Cavalieri volume. It
    // used to chord the bands on the whole-solid mesh, 1.6e-4 low here.
    let measured = crate::measure::solid_volume(topo, solid, scale * 1e-4).unwrap();
    let relative = (measured - expected_volume).abs() / expected_volume;
    assert!(
        relative < 1e-9,
        "solid_volume {measured} vs exact {expected_volume} (relative {relative:e})"
    );

    let mesh = crate::tessellate::tessellate_solid(topo, solid, scale * 2e-3).unwrap();
    assert!(crate::tessellate::is_watertight(&mesh));
    assert_eq!(crate::tessellate::boundary_edge_count(&mesh), 0);
    assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
    let signed: f64 = mesh
        .indices
        .chunks_exact(3)
        .map(|tri| {
            let [a, b, c] = [0, 1, 2].map(|k| {
                let p = mesh.positions[tri[k] as usize];
                Vec3::new(p.x(), p.y(), p.z())
            });
            a.dot(b.cross(c)) / 6.0
        })
        .sum();
    assert!(
        (signed - expected_volume).abs() / expected_volume < 1e-2,
        "display mesh must enclose the solid outward: {signed} vs {expected_volume}"
    );
}

/// #1253: the r25 → r10 circles with the top centre moved 12 mm sideways.
/// Ruled and smooth (two sections share the ruled geometry) used to emit 32
/// planar facets 0.64 % short of the exact oblique frustum.
#[test]
fn offset_circle_loft_keeps_circular_rims_and_one_exact_curved_wall() {
    type LoftFn = fn(&mut Topology, &[FaceId]) -> Result<SolidId, crate::OperationsError>;
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let bottom = (Point3::new(0.0, 0.0, 0.0), 25.0);
    let top = (Point3::new(12.0, 0.0, 60.0), 10.0);
    let expected = oblique_frustum_volume(bottom, top, axis);
    assert!((expected - 61_261.056_745_000_96).abs() < 1e-6);
    for (name, method) in [("loft", loft as LoftFn), ("loft_smooth", loft_smooth)] {
        for (first, second) in [(bottom, top), (top, bottom)] {
            for as_nurbs in [false, true] {
                let mut topo = Topology::new();
                let a = circle_section(&mut topo, first.0, axis, first.1, as_nurbs);
                let b = circle_section(&mut topo, second.0, axis, second.1, as_nurbs);
                let solid = method(&mut topo, &[a, b]);
                assert!(
                    solid.is_ok(),
                    "{name} nurbs={as_nurbs}: {:?}",
                    solid.as_ref().err()
                );
                let solid = solid.unwrap();
                assert_oblique_circle_loft(&topo, solid, &[first, second], axis, expected);
            }
        }
    }
}

/// The aligned control still takes the analytic coaxial path.
#[test]
fn aligned_circle_loft_control_stays_one_analytic_cone() {
    let mut topo = Topology::new();
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let a = circle_section(&mut topo, Point3::new(0.0, 0.0, 0.0), axis, 25.0, false);
    let b = circle_section(&mut topo, Point3::new(0.0, 0.0, 60.0), axis, 10.0, false);
    let solid = loft(&mut topo, &[a, b]).unwrap();
    let expected = oblique_frustum_volume(
        (Point3::new(0.0, 0.0, 0.0), 25.0),
        (Point3::new(0.0, 0.0, 60.0), 10.0),
        axis,
    );
    assert_analytic_frustum_solid(&topo, solid, expected);
}

/// Placement and scale independence: a tilted stacking axis, lateral offsets
/// larger than the radii, an oblique cylinder (equal radii) and 1e-2 … 1e3
/// scales all keep the exact rims, band and volume.
#[test]
fn offset_circle_loft_is_exact_across_scales_and_placements() {
    let tilted = Vec3::new(1.0, -2.0, 3.0).normalize().unwrap();
    let side = tilted.cross(Vec3::new(0.0, 0.0, 1.0)).normalize().unwrap();
    for scale in [1e-2, 1.0, 1e3] {
        for (axis, lateral, radii) in [
            (tilted, side * 40.0, (25.0, 10.0)),
            (tilted, side * -7.0, (8.0, 8.0)),
            (
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(30.0, 0.0, -5.0),
                (5.0, 20.0),
            ),
        ] {
            let origin = Point3::new(3.0 * scale, -4.0 * scale, 5.0 * scale);
            let lower = (origin, radii.0 * scale);
            let upper = (origin + (axis * 60.0 + lateral) * scale, radii.1 * scale);
            let expected = oblique_frustum_volume(lower, upper, axis);
            let mut topo = Topology::new();
            let a = circle_section(&mut topo, lower.0, axis, lower.1, false);
            let b = circle_section(&mut topo, upper.0, axis, upper.1, true);
            let solid = loft(&mut topo, &[a, b]).unwrap();
            assert_oblique_circle_loft(&topo, solid, &[lower, upper], axis, expected);
        }
    }
}

/// A zig-zag stack of three offset circles keeps one exact band per pair
/// and shares the middle rim between them.
#[test]
fn three_offset_circles_loft_into_two_exact_bands() {
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let sections = [
        (Point3::new(0.0, 0.0, 0.0), 20.0),
        (Point3::new(10.0, 5.0, 30.0), 12.0),
        (Point3::new(-6.0, 0.0, 50.0), 15.0),
    ];
    let expected = oblique_frustum_volume(sections[0], sections[1], axis)
        + oblique_frustum_volume(sections[1], sections[2], axis);
    let mut topo = Topology::new();
    let faces: Vec<FaceId> = sections
        .iter()
        .map(|&(center, radius)| circle_section(&mut topo, center, axis, radius, false))
        .collect();
    let solid = loft(&mut topo, &faces).unwrap();
    assert_oblique_circle_loft(&topo, solid, &sections, axis, expected);
}

/// Inputs outside the oblique stack's proof keep their legacy route: tilted
/// (non-parallel) circles still loft through the polygon path, and a circle
/// whose stored plane does not contain it is not trusted as a section.
#[test]
fn non_parallel_or_inconsistent_circle_sections_keep_the_legacy_loft() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let mut topo = Topology::new();
    let a = circle_section(&mut topo, Point3::new(0.0, 0.0, 0.0), z, 25.0, false);
    let tilted = Vec3::new(0.2, 0.0, 1.0);
    let b = circle_section(&mut topo, Point3::new(12.0, 0.0, 60.0), tilted, 10.0, false);
    let solid = loft(&mut topo, &[a, b]).unwrap();
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    assert!(
        faces.len() > 3,
        "non-parallel circles are not an oblique stack"
    );

    let mut topo = Topology::new();
    let a = circle_section(&mut topo, Point3::new(0.0, 0.0, 0.0), z, 25.0, false);
    let b = circle_section(&mut topo, Point3::new(12.0, 0.0, 60.0), z, 10.0, false);
    let outer = topo.face(b).unwrap().outer_wire();
    let lifted = topo.add_face(Face::new(
        outer,
        vec![],
        FaceSurface::Plane { normal: z, d: 61.0 },
    ));
    assert!(super::face_full_circle_section(&topo, lifted).is_none());
    assert!(super::face_full_circle_section(&topo, b).is_some());
    let _ = a;
}

#[test]
fn oblique_circle_loft_does_not_flatten_nearly_parallel_sections() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    // The old dot-product gate accepted the first tilt, although flattening
    // its rim moves it 100 times farther than the linear tolerance. The
    // second tilt is below angular tolerance but fails at this large radius.
    for (tilt, radius) in [(1e-6, 10.0), (5e-13, 1e7)] {
        let mut topo = Topology::new();
        let a = circle_section(&mut topo, Point3::new(0.0, 0.0, 0.0), z, 25.0, false);
        let b = circle_section(
            &mut topo,
            Point3::new(12.0, 0.0, 60.0),
            Vec3::new(0.0, tilt, 1.0),
            radius,
            false,
        );
        let before = topo.allocated_slot_count();
        assert!(
            super::try_loft_parallel_circle_stack(&mut topo, &[a, b])
                .unwrap()
                .is_none()
        );
        assert_eq!(topo.allocated_slot_count(), before);
    }
    // A tiny angle between a circle and its own claimed cap plane can
    // still move a large rim beyond linear tolerance. A seam point alone
    // is insufficient to witness that the entire circle lies in the plane.
    let mut topo = Topology::new();
    let center = Point3::new(12.0, 0.0, 60.0);
    let face = circle_section(&mut topo, center, z, 1e7, false);
    let outer = topo.face(face).unwrap().outer_wire();
    let normal = Vec3::new(5e-13, 5e-13, 1.0).normalize().unwrap();
    let inconsistent = topo.add_face(Face::new(
        outer,
        vec![],
        FaceSurface::Plane {
            normal,
            d: normal.dot(Vec3::new(center.x(), center.y(), center.z())),
        },
    ));
    assert!(super::face_full_circle_section(&topo, inconsistent).is_none());
}

fn profile_with_curve_domain(
    topo: &mut Topology,
    center: Point3,
    curve: EdgeCurve,
    start: Point3,
    trim: (f64, f64),
) -> FaceId {
    let seam = topo.add_vertex(Vertex::new(start, 1e-7));
    let mut edge = Edge::new(seam, seam, curve);
    edge.set_trim(Some(trim));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: center.z(),
        },
    ))
}

#[test]
fn oblique_circle_loft_does_not_replace_a_near_circle_with_a_fitted_circle() {
    let center = Point3::new(12.0, 0.0, 60.0);
    let circle = remus_math::curves::Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 10.0).unwrap();
    let canonical =
        remus_geometry::convert::circle_to_nurbs(&circle, 0.0, std::f64::consts::TAU).unwrap();
    // An affine stretch of the exact rational circle is an exact ellipse.
    // The sampled recognizer accepts it at 1e-5, but changing its boundary
    // into that fitted circle would exceed the kernel's 1e-7 tolerance.
    let ellipse = remus_math::nurbs::curve::NurbsCurve::new(
        canonical.degree(),
        canonical.knots().to_vec(),
        canonical
            .control_points()
            .iter()
            .map(|p| {
                Point3::new(
                    center.x() + (p.x() - center.x()) * 1.000_000_2,
                    p.y(),
                    p.z(),
                )
            })
            .collect(),
        canonical.weights().to_vec(),
    )
    .unwrap();
    assert!(matches!(
        remus_geometry::convert::recognize_curve(&ellipse, 1e-5),
        remus_geometry::convert::RecognizedCurve::Circle { .. }
    ));
    let mut topo = Topology::new();
    let start = ellipse.control_points()[0];
    let range = ellipse.domain();
    let face = profile_with_curve_domain(
        &mut topo,
        center,
        EdgeCurve::NurbsCurve(ellipse),
        start,
        range,
    );
    assert!(super::face_full_circle_section(&topo, face).is_none());
}

#[test]
fn oblique_circle_loft_requires_the_original_full_turn_domain() {
    let center = Point3::new(12.0, 0.0, 60.0);
    let circle = remus_math::curves::Circle3D::new(center, Vec3::new(0.0, 0.0, 1.0), 10.0).unwrap();
    let turn = std::f64::consts::TAU;
    for range in [(0.0, turn * 0.5), (0.0, turn * 2.0)] {
        let mut topo = Topology::new();
        let face = profile_with_curve_domain(
            &mut topo,
            center,
            EdgeCurve::Circle(circle.clone()),
            circle.evaluate(0.0),
            range,
        );
        assert!(super::face_full_circle_section(&topo, face).is_none());
    }
    let curve = remus_geometry::convert::circle_to_nurbs(&circle, 0.0, turn).unwrap();
    let mut topo = Topology::new();
    let start = curve.control_points()[0];
    let face = profile_with_curve_domain(
        &mut topo,
        center,
        EdgeCurve::NurbsCurve(curve),
        start,
        (0.25, 1.0),
    );
    assert!(super::face_full_circle_section(&topo, face).is_none());
}
