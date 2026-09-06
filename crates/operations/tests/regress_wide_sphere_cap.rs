//! Issue #285: a sphere with one planar cut must retain the large spherical cap.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::Circle3D;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn ball_with_flat(radius: f64, cut: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let rim_radius = (radius * radius - cut * cut).sqrt();
    let circle = Circle3D::new(
        Point3::new(0.0, 0.0, cut),
        Vec3::new(0.0, 0.0, 1.0),
        rim_radius,
    )
    .unwrap();
    let vertex = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, std::f64::consts::TAU)));
    let edge = topo.add_edge(edge);
    let cap_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
    let wall_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, false)], true).unwrap());
    let cap = topo.add_face(Face::new(
        cap_wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: cut,
        },
    ));
    let wall = topo.add_face(Face::new(
        wall_wire,
        vec![],
        FaceSurface::Sphere(SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), radius).unwrap()),
    ));
    let shell = topo.add_shell(Shell::new(vec![cap, wall]).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    (topo, solid)
}

fn assert_cap(topo: &Topology, solid: SolidId, radius: f64, cut: f64) {
    assert!(
        remus_operations::validate::validate_solid(topo, solid)
            .unwrap()
            .is_valid()
    );
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    assert_eq!(faces.len(), 2);
    assert_eq!(
        faces
            .iter()
            .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Sphere(_)))
            .count(),
        1
    );
    let height = radius + cut;
    let expected = std::f64::consts::PI * height * height * (3.0 * radius - height) / 3.0;
    for relative_deflection in [0.005, 0.0005] {
        let mesh = tessellate_solid(topo, solid, radius * relative_deflection).unwrap();
        let quality = welded_mesh_quality(&mesh);
        assert!(
            quality.is_watertight(),
            "r={radius}, cut={cut}: {quality:?}"
        );
        let origin = mesh.positions[0];
        let volume: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|tri| {
                let a = mesh.positions[tri[0] as usize] - origin;
                let b = mesh.positions[tri[1] as usize] - origin;
                let c = mesh.positions[tri[2] as usize] - origin;
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        assert!(
            (volume - expected).abs() / expected < 0.01,
            "r={radius}, cut={cut}, deflection={relative_deflection}: {volume} vs {expected}"
        );
    }
}

#[test]
fn wide_sphere_cap_matches_closed_form_volume() {
    for scale in [0.1, 1.0, 10.0] {
        for (radius, cut) in [(2.0_f64, 1.0_f64), (9.0, 7.5), (2.0, -0.1), (2.0, -1.4)] {
            let radius = radius * scale;
            let cut = cut * scale;
            let (mut topo, solid) = ball_with_flat(radius, cut);
            assert_cap(&topo, solid, radius, cut);
            let placement =
                remus_math::mat::Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
                    * remus_math::mat::Mat4::rotation_y(0.37);
            remus_operations::transform::transform_solid(&mut topo, solid, &placement).unwrap();
            assert_cap(&topo, solid, radius, cut);
        }
    }
}

#[test]
fn wide_sphere_cap_survives_step_round_trip() {
    let (topo, solid) = ball_with_flat(9.0, 7.5);
    let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
    let mut imported = Topology::new();
    let solids = remus_io::step::reader::read_step(&step, &mut imported).unwrap();
    assert_eq!(solids.len(), 1);
    assert_cap(&imported, solids[0], 9.0, 7.5);
}

#[test]
fn standalone_wide_sphere_face_has_the_retained_cap_area() {
    let (topo, solid) = ball_with_flat(2.0, 1.0);
    let face = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Sphere(_)))
        .unwrap();
    let expected = 2.0 * std::f64::consts::PI * 2.0 * 3.0;
    for deflection in [0.02, 0.002] {
        let mesh = remus_operations::tessellate::tessellate(&topo, face, deflection).unwrap();
        let area: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|tri| {
                let a = mesh.positions[tri[0] as usize];
                let b = mesh.positions[tri[1] as usize];
                let c = mesh.positions[tri[2] as usize];
                (b - a).cross(c - a).length() * 0.5
            })
            .sum();
        assert!(
            (area - expected).abs() / expected < 0.01,
            "{area} vs {expected}"
        );
    }
}

#[test]
fn wide_sphere_cap_with_pole_seam_matches_closed_form() {
    let radius = 9.0;
    let cut = 7.5;
    let (mut topo, solid) = ball_with_flat(radius, cut);
    let face = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Sphere(_)))
        .unwrap();
    let rim = topo
        .wire(topo.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()[0];
    let rim_vertex = topo.edge(rim.edge()).unwrap().start();
    let pole = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, -radius), 1e-7));
    let meridian = Circle3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        radius,
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let mut seam = Edge::new(pole, rim_vertex, EdgeCurve::Circle(meridian));
    seam.set_trim(Some((-std::f64::consts::FRAC_PI_2, (cut / radius).asin())));
    let seam = topo.add_edge(seam);
    let wire = topo.add_wire(
        Wire::new(
            vec![
                rim,
                OrientedEdge::new(seam, false),
                OrientedEdge::new(seam, true),
            ],
            true,
        )
        .unwrap(),
    );
    topo.set_face_boundary_wires(face, wire, vec![]).unwrap();
    let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
    let mut imported = Topology::new();
    let imported_solids = remus_io::step::reader::read_step(&step, &mut imported).unwrap();
    assert_eq!(imported_solids.len(), 1);
    assert_cap(&imported, imported_solids[0], radius, cut);

    assert_cap(&topo, solid, radius, cut);
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &remus_math::mat::Mat4::rotation_y(0.37),
    )
    .unwrap();
    assert_cap(&topo, solid, radius, cut);
}
