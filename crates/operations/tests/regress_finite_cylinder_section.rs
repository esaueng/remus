//! Finite cylinder section bounds, placement, and unsupported trim foils.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::measure::face_area;
use remus_operations::primitives::make_cylinder;
use remus_operations::section::section;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;

#[test]
fn axial_sections_respect_caps_at_all_scales_and_placements() {
    for scale in [1e-3, 1., 1e3] {
        for placed in [false, true] {
            for height in [-1., 0., 1., 2., 3.] {
                for sense in [-1., 1.] {
                    let mut topo = Topology::new();
                    let solid = make_cylinder(&mut topo, scale, 2. * scale).unwrap();
                    let matrix = if placed {
                        Mat4::translation(10. * scale, -3. * scale, 5. * scale)
                            * Mat4::rotation_x(0.37)
                            * Mat4::rotation_y(0.28)
                    } else {
                        Mat4::identity()
                    };
                    transform_solid(&mut topo, solid, &matrix).unwrap();
                    let point = matrix.mul_point(Point3::new(0., 0., height * scale));
                    let normal = Vec3::new(matrix.0[0][2], matrix.0[1][2], matrix.0[2][2]) * sense;
                    let result = section(&mut topo, solid, point, normal).unwrap();
                    if (0. ..=2.).contains(&height) {
                        assert_eq!(result.faces.len(), 1);
                        // The API retains its existing inscribed 64-chord disc.
                        let area = face_area(&topo, result.faces[0], 1e-3 * scale).unwrap();
                        let expected = 32. * scale * scale * (std::f64::consts::TAU / 64.).sin();
                        assert!((area / expected - 1.).abs() < 1e-10);
                        for oriented in topo
                            .wire(topo.face(result.faces[0]).unwrap().outer_wire())
                            .unwrap()
                            .edges()
                        {
                            let vertex = topo
                                .vertex(topo.edge(oriented.edge()).unwrap().start())
                                .unwrap()
                                .point();
                            assert!((vertex - point).dot(normal).abs() < 1e-7);
                        }
                    } else {
                        assert!(
                            result.faces.is_empty(),
                            "scale={scale}, height={height}, placed={placed}"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn contained_oblique_section_matches_ellipse_area() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1., 4.).unwrap();
    let normal = Vec3::new(0.25, 0., 1.).normalize().unwrap();
    let result = section(&mut topo, solid, Point3::new(0., 0., 2.), normal).unwrap();
    assert_eq!(result.faces.len(), 1);
    let expected = 32. * (std::f64::consts::TAU / 64.).sin() / normal.z();
    assert!((face_area(&topo, result.faces[0], 1e-3).unwrap() / expected - 1.).abs() < 1e-12);
}

#[test]
fn cuts_crossing_caps_or_parallel_to_axis_refuse_without_arena_mutation() {
    for (point, normal) in [
        (Point3::new(0., 0., 0.1), Vec3::new(1., 0., 1.)),
        (Point3::new(0., 0., 1.), Vec3::new(1., 0., 0.)),
    ] {
        let mut topo = Topology::new();
        let solid = make_cylinder(&mut topo, 1., 2.).unwrap();
        let counts = (
            topo.vertices().len(),
            topo.edges().len(),
            topo.wires().len(),
            topo.faces().len(),
        );
        assert!(matches!(
            section(&mut topo, solid, point, normal),
            Err(OperationsError::Unsupported {
                operation: "section",
                ..
            })
        ));
        assert_eq!(
            counts,
            (
                topo.vertices().len(),
                topo.edges().len(),
                topo.wires().len(),
                topo.faces().len()
            )
        );
    }
}

#[test]
fn cylindrical_partial_rim_is_not_promoted_to_a_full_section() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1., 2.).unwrap();
    let lateral = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| matches!(topo.face(id).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let rim = topo
        .wire(topo.face(lateral).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    let trim = topo.edge(rim).unwrap().trim().unwrap();
    topo.edge_mut(rim)
        .unwrap()
        .set_trim(Some((trim.0, trim.0 + std::f64::consts::PI)));
    assert!(
        section(
            &mut topo,
            solid,
            Point3::new(0., 0., 1.),
            Vec3::new(0., 0., 1.)
        )
        .is_err()
    );
}

#[test]
fn cylindrical_band_with_a_hole_refuses() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 1., 2.).unwrap();
    let lateral = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| matches!(topo.face(id).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let boundary = topo.face(lateral).unwrap().outer_wire();
    // A full band shortcut must never silently erase an inner boundary.
    topo.set_face_boundary_wires(lateral, boundary, vec![boundary])
        .unwrap();
    assert!(matches!(
        section(
            &mut topo,
            solid,
            Point3::new(0., 0., 1.),
            Vec3::new(0., 0., 1.)
        ),
        Err(OperationsError::Unsupported { .. })
    ));
}
