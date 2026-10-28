#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::Point3;
use remus_topology::explorer::solid_vertices;
use remus_topology::face::Face;
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::test_utils::{make_unit_cube_manifold_at, make_unit_square_face};

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn attach(topo: &mut Topology, surface: FaceSurface, reversed: bool) -> FaceId {
    // The API is deliberately a carrier query. The reused wire makes that
    // distinction explicit: no test claims this wire trims a curved carrier.
    let template = make_unit_square_face(topo);
    let wire = topo.face(template).unwrap().outer_wire();
    topo.add_face(if reversed {
        Face::new_reversed(wire, vec![], surface)
    } else {
        Face::new(wire, vec![], surface)
    })
}

fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() < 2e-14, "{actual} != {expected}");
}

#[test]
fn slope_angles_and_face_orientation_are_signed() {
    let mut topo = Topology::new();
    for slope in [-3.0_f64, -0.25, 0.0, 0.25, 3.0] {
        for reversed in [false, true] {
            let face = attach(
                &mut topo,
                FaceSurface::Plane {
                    normal: Vec3::new(1.0, 0.0, slope),
                    d: 13.0,
                },
                reversed,
            );
            let report = surface_draft_angle(&topo, face, 0.0, 0.0, Z).unwrap();
            let expected = if reversed {
                -slope.atan()
            } else {
                slope.atan()
            };
            close(report.angle_radians, expected);
            close(report.outward_normal.length(), 1.0);
            close(
                surface_draft_angle(&topo, face, 7.0, -9.0, -Z)
                    .unwrap()
                    .angle_radians,
                -expected,
            );
        }
    }
}

#[test]
fn direction_scaling_including_extreme_magnitudes_does_not_change_angles() {
    let mut topo = Topology::new();
    let face = make_unit_square_face(&mut topo);
    let expected = (1.0_f64 / 2.0).atan();
    for magnitude in [1e-300, 1e-3, 1.0, 1e300] {
        let angle = surface_draft_angle(
            &topo,
            face,
            0.0,
            0.0,
            Vec3::new(2.0 * magnitude, 0.0, magnitude),
        )
        .unwrap()
        .angle_radians;
        close(angle, expected);
    }
    close(
        surface_draft_angle(
            &topo,
            face,
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, f64::from_bits(1)),
        )
        .unwrap()
        .angle_radians,
        std::f64::consts::FRAC_PI_2,
    );
    let axis = Vec3::new(1.0, 2.0, 3.0);
    let oblique = attach(
        &mut topo,
        FaceSurface::Plane {
            normal: axis,
            d: 0.0,
        },
        false,
    );
    close(
        surface_draft_angle(&topo, oblique, 0.0, 0.0, axis)
            .unwrap()
            .angle_radians,
        std::f64::consts::FRAC_PI_2,
    );
}

#[test]
fn analytic_surfaces_match_axis_angle_oracles_across_scale_and_placement() {
    let mut topo = Topology::new();
    for scale in [1e-3, 1.0, 1e3] {
        let origin = Point3::new(37.0 * scale, -91.0 * scale, 13.0 * scale);
        let axis = Vec3::new(0.0, 1.0, 0.0);
        let cylinder = attach(
            &mut topo,
            FaceSurface::Cylinder(CylindricalSurface::new(origin, axis, 2.0 * scale).unwrap()),
            false,
        );
        close(
            surface_draft_angle(&topo, cylinder, 0.7, scale, axis)
                .unwrap()
                .angle_radians,
            0.0,
        );
        let cone = attach(
            &mut topo,
            FaceSurface::Cone(ConicalSurface::new(origin, axis, 0.6).unwrap()),
            false,
        );
        close(
            surface_draft_angle(&topo, cone, 0.7, scale, axis)
                .unwrap()
                .angle_radians,
            0.6 - std::f64::consts::FRAC_PI_2,
        );
        let sphere = attach(
            &mut topo,
            FaceSurface::Sphere(SphericalSurface::with_axis(origin, 2.0 * scale, axis).unwrap()),
            false,
        );
        for latitude in [
            -std::f64::consts::FRAC_PI_2,
            -0.5,
            0.0,
            0.5,
            std::f64::consts::FRAC_PI_2,
        ] {
            close(
                surface_draft_angle(&topo, sphere, 0.7, latitude, axis)
                    .unwrap()
                    .angle_radians,
                latitude,
            );
        }
        let torus = attach(
            &mut topo,
            FaceSurface::Torus(
                ToroidalSurface::with_axis(origin, 4.0 * scale, scale, axis).unwrap(),
            ),
            false,
        );
        for latitude in [-0.5, 0.0, 0.5] {
            close(
                surface_draft_angle(&topo, torus, 0.7, latitude, axis)
                    .unwrap()
                    .angle_radians,
                latitude,
            );
        }
    }
}

fn patch(scale: f64, collapsed: bool) -> NurbsSurface {
    let point = |u: f64, v: f64| Point3::new(scale * u, scale * v, scale * 0.25 * v);
    let p00 = point(0.0, 0.0);
    let p10 = point(1.0, 0.0);
    let p01 = if collapsed { p10 } else { point(0.0, 1.0) };
    let p11 = if collapsed {
        point(2.0, 0.0)
    } else {
        point(1.0, 1.0)
    };
    NurbsSurface::new(
        1,
        1,
        vec![2.0, 2.0, 5.0, 5.0],
        vec![-4.0, -4.0, 7.0, 7.0],
        vec![vec![p00, p01], vec![p10, p11]],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap()
}

#[test]
fn nurbs_normals_use_native_domains_and_refuse_collapsed_charts() {
    let mut topo = Topology::new();
    for scale in [1e-150, 1e-3, 1.0, 1e3, 1e150] {
        let face = attach(&mut topo, FaceSurface::Nurbs(patch(scale, false)), false);
        for (u, v) in [(2.0, -4.0), (3.3, 2.7), (5.0, 7.0)] {
            // z = y/4 gives outward normal proportional to (0, -1/4, 1).
            close(
                surface_draft_angle(&topo, face, u, v, Vec3::new(0.0, -1.0, 0.0))
                    .unwrap()
                    .angle_radians,
                0.25_f64.atan(),
            );
        }
        assert!(matches!(
            surface_draft_angle(&topo, face, 0.0, 0.0, Z),
            Err(DraftAnalysisError::InvalidParameters)
        ));
    }
    let face = attach(&mut topo, FaceSurface::Nurbs(patch(1.0, true)), false);
    assert!(matches!(
        surface_draft_angle(&topo, face, 3.0, 0.0, Z),
        Err(DraftAnalysisError::UndefinedNormal)
    ));
}

#[test]
fn planar_solid_report_covers_cavity_faces_without_mutation() {
    let mut topo = Topology::new();
    let outer = make_unit_cube_manifold_at(&mut topo, 37.0, -91.0, 13.0);
    let inner = make_unit_cube_manifold_at(&mut topo, 37.25, -90.75, 13.25);
    let anchor = Point3::new(37.25, -90.75, 13.25);
    for vertex in solid_vertices(&topo, inner).unwrap() {
        let point = topo.vertex(vertex).unwrap().point();
        topo.vertex_mut(vertex)
            .unwrap()
            .set_point(anchor + (point - anchor) * 0.5);
    }
    // Move each plane consistently with the smaller inner cube. The face
    // normals remain valid and the cavity lies strictly inside the outer cube.
    for face in solid_faces(&topo, inner).unwrap() {
        let surface = topo.face(face).unwrap().surface().clone();
        if let FaceSurface::Plane { normal, d } = surface {
            let d = 0.5 * d + 0.5 * normal.dot(Vec3::new(anchor.x(), anchor.y(), anchor.z()));
            topo.face_mut(face)
                .unwrap()
                .set_surface(FaceSurface::Plane { normal, d });
        }
    }
    let outer_shell = topo.solid(outer).unwrap().outer_shell();
    let inner_shell = topo.solid(inner).unwrap().outer_shell();
    let inner_faces = topo.shell(inner_shell).unwrap().faces().to_vec();
    let ids = inner_faces
        .into_iter()
        .map(|id| {
            let f = topo.face(id).unwrap();
            let face = Face::new_reversed(
                f.outer_wire(),
                f.inner_wires().to_vec(),
                f.surface().clone(),
            );
            topo.add_face(face)
        })
        .collect();
    let cavity = topo.add_shell(Shell::new(ids).unwrap());
    let solid = topo.add_solid(Solid::new(outer_shell, vec![cavity]));
    let ticks = topo.mutation_ticks();
    let faces = solid_faces(&topo, solid).unwrap();
    let report = planar_solid_draft(&topo, solid, Z).unwrap();
    assert_eq!(report.len(), 12);
    assert_eq!(report.iter().map(|r| r.face).collect::<Vec<_>>(), faces);
    for entry in &report {
        assert_eq!(
            entry.sample,
            surface_draft_angle(&topo, entry.face, 0.0, 0.0, Z).unwrap()
        );
    }
    assert_eq!(
        report
            .iter()
            .filter(|r| r.sample.angle_radians > 1.0)
            .count(),
        2
    );
    assert_eq!(
        report
            .iter()
            .filter(|r| r.sample.angle_radians < -1.0)
            .count(),
        2
    );
    assert_eq!(topo.mutation_ticks(), ticks);
}

#[test]
fn refusals_are_typed_and_read_only() {
    let mut topo = Topology::new();
    let face = make_unit_square_face(&mut topo);
    let ticks = topo.mutation_ticks();
    for pull in [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(f64::NAN, 0.0, 1.0),
        Vec3::new(f64::INFINITY, 0.0, 1.0),
    ] {
        assert!(matches!(
            surface_draft_angle(&topo, face, 0.0, 0.0, pull),
            Err(DraftAnalysisError::InvalidPullDirection)
        ));
    }
    for parameter in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(matches!(
            surface_draft_angle(&topo, face, parameter, 0.0, Z),
            Err(DraftAnalysisError::InvalidParameters)
        ));
        assert!(matches!(
            surface_draft_angle(&topo, face, 0.0, parameter, Z),
            Err(DraftAnalysisError::InvalidParameters)
        ));
    }
    assert_eq!(topo.mutation_ticks(), ticks);
    let curved = attach(
        &mut topo,
        FaceSurface::Cylinder(CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, 2.0).unwrap()),
        false,
    );
    let shell = topo.add_shell(Shell::new(vec![curved]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let ticks = topo.mutation_ticks();
    assert!(
        matches!(planar_solid_draft(&topo, solid, Z), Err(DraftAnalysisError::NonPlanarFace { face }) if face == curved)
    );
    assert_eq!(topo.mutation_ticks(), ticks);
}

#[test]
fn singular_and_out_of_domain_analytic_queries_refuse() {
    let mut topo = Topology::new();
    let cone = attach(
        &mut topo,
        FaceSurface::Cone(ConicalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, 0.6).unwrap()),
        false,
    );
    for slant in [0.0, -1.0] {
        assert!(matches!(
            surface_draft_angle(&topo, cone, 0.3, slant, Z),
            Err(DraftAnalysisError::InvalidParameters)
        ));
    }
    let sphere = attach(
        &mut topo,
        FaceSurface::Sphere(SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 2.0).unwrap()),
        false,
    );
    assert!(matches!(
        surface_draft_angle(&topo, sphere, 0.3, 2.0, Z),
        Err(DraftAnalysisError::InvalidParameters)
    ));
    let malformed = attach(
        &mut topo,
        FaceSurface::Plane {
            normal: Vec3::new(f64::NAN, 0.0, 1.0),
            d: 0.0,
        },
        false,
    );
    assert!(matches!(
        surface_draft_angle(&topo, malformed, 0.0, 0.0, Z),
        Err(DraftAnalysisError::UndefinedNormal)
    ));
}

#[test]
fn internal_c0_knots_do_not_fabricate_a_unique_normal() {
    let mut topo = Topology::new();
    for transpose in [false, true] {
        let mut points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(0.5, 0.0, 0.5), Point3::new(0.5, 1.0, 0.5)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ];
        let mut knots_u = vec![0.0, 0.0, 0.5, 1.0, 1.0];
        let mut knots_v = vec![0.0, 0.0, 1.0, 1.0];
        if transpose {
            points = (0..2)
                .map(|j| (0..3).map(|i| points[i][j]).collect())
                .collect();
            std::mem::swap(&mut knots_u, &mut knots_v);
        }
        let weights = points.iter().map(|row| vec![1.0; row.len()]).collect();
        let face = attach(
            &mut topo,
            FaceSurface::Nurbs(NurbsSurface::new(1, 1, knots_u, knots_v, points, weights).unwrap()),
            false,
        );
        let uv = |t| if transpose { (0.3, t) } else { (t, 0.3) };
        let sign = if transpose { -1.0 } else { 1.0 };
        for (t, angle) in [
            (0.25, -std::f64::consts::FRAC_PI_4),
            (0.75, std::f64::consts::FRAC_PI_4),
        ] {
            let (u, v) = uv(t);
            close(
                surface_draft_angle(&topo, face, u, v, Vec3::new(1.0, 0.0, 0.0))
                    .unwrap()
                    .angle_radians,
                sign * angle,
            );
        }
        let (u, v) = uv(0.5);
        assert!(matches!(
            surface_draft_angle(&topo, face, u, v, Z),
            Err(DraftAnalysisError::UndefinedNormal)
        ));
    }
}
