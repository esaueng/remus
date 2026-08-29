//! Steinmetz intersect: equal-radius cylinder ∩ cylinder with intersecting
//! axes is exact analytic.
//!
//! The intersection curve of two equal-radius cylinders whose axes intersect
//! factors into two planar ellipses (the bisector planes of the axes), so the
//! boolean intersect must produce trimmed `Cylinder` patches bounded by
//! `Ellipse` edges — never the mesh fallback that used to answer here with
//! ~70 planar faces and a ~1% volume error. Oracle: the bicylinder volume is
//! exactly `16/3·r³` for perpendicular axes, `16r³/(3·sinθ)` in general.

#![allow(clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::topology::Topology;

/// Assert the Steinmetz result shape: all-cylinder faces, Line/Ellipse edges
/// only, watertight + manifold mesh at several deflections, exact volume.
fn assert_steinmetz(topo: &Topology, result: SolidId, r: f64, exact_volume: f64) {
    let faces = solid_faces(topo, result).expect("faces");
    assert_eq!(faces.len(), 6, "expected 6 cylinder patches, got {faces:?}");
    for &f in &faces {
        let face = topo.face(f).expect("face");
        assert!(
            matches!(face.surface(), FaceSurface::Cylinder(_)),
            "non-cylinder face in Steinmetz intersect: {:?}",
            face.surface().type_tag()
        );
        let wire = topo.wire(face.outer_wire()).expect("wire");
        for oe in wire.edges() {
            let e = topo.edge(oe.edge()).expect("edge");
            assert!(
                matches!(e.curve(), EdgeCurve::Line | EdgeCurve::Ellipse(_)),
                "unexpected edge curve type: {}",
                e.curve().type_tag()
            );
        }
    }

    // Volume: measured by tessellation, so allow discretization error only —
    // the old mesh fallback was ~0.5–1.2% off, the exact result ~1e-5.
    let vol = solid_volume(topo, result, 0.001).expect("volume");
    let rel = (vol - exact_volume).abs() / exact_volume;
    assert!(
        rel < 1.0e-4,
        "volume {vol} vs exact {exact_volume} (rel {rel:.2e}) at r={r}"
    );

    // Watertight AND manifold at coarse through fine deflection — the lens
    // tips are where duplicated chordal triangles used to appear.
    for defl in [0.05, 0.01, 0.001] {
        let mesh = tessellate_solid(topo, result, defl).expect("tessellate");
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "boundary edges at deflection {defl}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "non-manifold edges at deflection {defl}"
        );
    }
}

fn perpendicular_pair(topo: &mut Topology, r: f64) -> (SolidId, SolidId) {
    let c1 = primitives::make_cylinder(topo, r, 20.0).expect("c1");
    transform_solid(topo, c1, &Mat4::translation(0.0, 0.0, -10.0)).expect("t1");
    let c2 = primitives::make_cylinder(topo, r, 20.0).expect("c2");
    transform_solid(topo, c2, &Mat4::rotation_y(std::f64::consts::FRAC_PI_2)).expect("r2");
    transform_solid(topo, c2, &Mat4::translation(-10.0, 0.0, 0.0)).expect("t2");
    (c1, c2)
}

#[test]
fn perpendicular_equal_radius_intersect_is_exact_analytic() {
    for r in [2.0_f64, 3.0, 5.0] {
        let topo = &mut Topology::new();
        let (c1, c2) = perpendicular_pair(topo, r);
        let result = boolean(topo, BooleanOp::Intersect, c1, c2).expect("intersect");
        assert_steinmetz(topo, result, r, 16.0 / 3.0 * r * r * r);
    }
}

#[test]
fn rotated_off_axis_steinmetz_intersect_is_exact_analytic() {
    // Same configuration under a general rigid motion: no axis-aligned
    // shortcut may be load-bearing.
    let r = 3.0_f64;
    let topo = &mut Topology::new();
    let (c1, c2) = perpendicular_pair(topo, r);
    let motion =
        Mat4::translation(7.0, -4.0, 11.0) * Mat4::rotation_z(0.61) * Mat4::rotation_x(-0.37);
    transform_solid(topo, c1, &motion).expect("m1");
    transform_solid(topo, c2, &motion).expect("m2");
    let result = boolean(topo, BooleanOp::Intersect, c1, c2).expect("intersect");
    assert_steinmetz(topo, result, r, 16.0 / 3.0 * r * r * r);
}

#[test]
fn oblique_equal_radius_intersect_is_exact_analytic() {
    // The bisector-plane factorization holds for ANY axis angle, not just
    // 90°: volume is 16r³/(3·sinθ).
    let r = 2.0_f64;
    let theta = 1.05_f64; // ~60°
    let topo = &mut Topology::new();
    let c1 = primitives::make_cylinder(topo, r, 24.0).expect("c1");
    transform_solid(topo, c1, &Mat4::translation(0.0, 0.0, -12.0)).expect("t1");
    let c2 = primitives::make_cylinder(topo, r, 24.0).expect("c2");
    transform_solid(topo, c2, &Mat4::translation(0.0, 0.0, -12.0)).expect("t2");
    transform_solid(topo, c2, &Mat4::rotation_y(theta)).expect("r2");
    let result = boolean(topo, BooleanOp::Intersect, c1, c2).expect("intersect");

    let faces = solid_faces(topo, result).expect("faces");
    assert!(
        faces.iter().all(|&f| {
            topo.face(f)
                .is_ok_and(|face| matches!(face.surface(), FaceSurface::Cylinder(_)))
        }),
        "oblique Steinmetz intersect must stay all-cylinder"
    );
    let exact = 16.0 * r * r * r / (3.0 * theta.sin());
    let vol = solid_volume(topo, result, 0.001).expect("volume");
    let rel = (vol - exact).abs() / exact;
    assert!(
        rel < 1.0e-4,
        "oblique volume {vol} vs {exact} (rel {rel:.2e})"
    );
    let mesh = tessellate_solid(topo, result, 0.01).expect("tessellate");
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);
}

#[test]
fn perpendicular_equal_radius_fuse_and_cut_stay_exact() {
    // The same exact ellipse sections now feed the union and difference:
    // both must stay exact analytic with correct volumes
    // (V∪ = 2·πr²h − 16/3·r³, V∖ = πr²h − 16/3·r³) and mesh watertight.
    let r = 3.0_f64;
    let h = 20.0_f64;
    let steinmetz = 16.0 / 3.0 * r * r * r;
    for (op, exact) in [
        (
            BooleanOp::Fuse,
            2.0 * std::f64::consts::PI * r * r * h - steinmetz,
        ),
        (BooleanOp::Cut, std::f64::consts::PI * r * r * h - steinmetz),
    ] {
        let topo = &mut Topology::new();
        let (c1, c2) = perpendicular_pair(topo, r);
        let result = boolean(topo, op, c1, c2).expect("boolean");
        let faces = solid_faces(topo, result).expect("faces");
        assert!(
            faces.iter().all(|&f| {
                topo.face(f).is_ok_and(|face| {
                    matches!(
                        face.surface(),
                        FaceSurface::Cylinder(_) | FaceSurface::Plane { .. }
                    )
                })
            }),
            "{op:?} must stay analytic (cylinders + caps)"
        );
        let vol = solid_volume(topo, result, 0.001).expect("volume");
        let rel = (vol - exact).abs() / exact;
        assert!(
            rel < 1.0e-4,
            "{op:?} volume {vol} vs {exact} (rel {rel:.2e})"
        );
        for defl in [0.05, 0.01] {
            let mesh = tessellate_solid(topo, result, defl).expect("tessellate");
            assert_eq!(
                boundary_edge_count(&mesh),
                0,
                "{op:?} boundary edges at deflection {defl}"
            );
            assert_eq!(
                non_manifold_edge_count(&mesh),
                0,
                "{op:?} non-manifold edges at deflection {defl}"
            );
        }
    }
}

#[test]
fn unequal_radius_perpendicular_intersect_still_answers() {
    // The exact arm must decline unequal radii (irreducible quartic); the
    // general path answers — bounded, whatever representation it picks.
    let topo = &mut Topology::new();
    let c1 = primitives::make_cylinder(topo, 3.0, 20.0).expect("c1");
    transform_solid(topo, c1, &Mat4::translation(0.0, 0.0, -10.0)).expect("t1");
    let c2 = primitives::make_cylinder(topo, 2.0, 20.0).expect("c2");
    transform_solid(topo, c2, &Mat4::rotation_y(std::f64::consts::FRAC_PI_2)).expect("r2");
    transform_solid(topo, c2, &Mat4::translation(-10.0, 0.0, 0.0)).expect("t2");
    let result = boolean(topo, BooleanOp::Intersect, c1, c2).expect("intersect");
    let vol = solid_volume(topo, result, 0.01).expect("volume");
    // No closed form asserted here — the point is "no regression to an
    // error", with a plausibility band: the result sits strictly inside the
    // smaller cylinder clipped to the larger one's diameter (π·2²·6 ≈ 75.4).
    assert!(vol > 0.0 && vol < 76.0, "vol={vol}");
}
