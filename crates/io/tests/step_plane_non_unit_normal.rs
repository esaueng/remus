//! A `FaceSurface::Plane` may store a non-unit normal: `{ normal, d }` means
//! `normal · p = d`, so its chart origin is the foot of the plane,
//! `normal * d / |normal|²` (the reader's `planar_bound_frame` and the
//! SameParameter proof's `PlaneChart::canonical`). The STEP writer once used
//! `normal * d` for both the emitted `AXIS2_PLACEMENT_3D` location and its
//! pcurve endpoint check, which sits off the plane unless `|normal| == 1`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::{reader::read_step, writer::write_step};
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::vec::{Point2, Vec2, Vec3};
use remus_operations::primitives::make_box;
use remus_topology::{
    Topology,
    explorer::{face_vertices, solid_faces},
    face::FaceSurface,
    pcurve::PCurve,
};

const TOL: f64 = 1e-9;

#[test]
fn non_unit_normal_plane_with_pcurves_exports_and_round_trips_on_its_plane() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 5.0, 3.0).unwrap();
    let top = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face_id| match topo.face(face_id).unwrap().surface() {
            FaceSurface::Plane { normal, d } => {
                (normal.z() - 1.0).abs() < TOL && (*d - 3.0).abs() < TOL
            }
            _ => false,
        })
        .expect("box has a top face at z = 3");
    // Same plane z = 3, so the canonical chart is unchanged.
    topo.face_mut(top).unwrap().set_surface(FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 2.5),
        d: 7.5,
    });

    // Canonical chart of z = 3: origin (0, 0, 3), u = n̂ × x̂ = (0, 1, 0),
    // v = n̂ × u = (-1, 0, 0), so a point maps to (y, -x).
    let chart = |p: remus_math::vec::Point3| Point2::new(p.y(), -p.x());
    let loop_id = topo.face(top).unwrap().outer_loop().expect("outer loop");
    let coedges = topo.face_loop(loop_id).unwrap().coedges().to_vec();
    assert_eq!(coedges.len(), 4);
    for coedge_id in coedges {
        let coedge = topo.coedge(coedge_id).unwrap();
        let edge = topo.edge(coedge.edge()).unwrap();
        let (start, end) = if coedge.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        let start = chart(topo.vertex(start).unwrap().point());
        let end = chart(topo.vertex(end).unwrap().point());
        let length = (end - start).length();
        let line = Line2D::new(start, Vec2::new(end.x() - start.x(), end.y() - start.y()))
            .expect("non-degenerate box edge");
        topo.set_coedge_pcurve(coedge_id, PCurve::new(Curve2D::Line(line), 0.0, length))
            .unwrap();
    }

    let step = write_step(&topo, &[solid])
        .expect("a valid non-unit-normal planar face must export without a pcurve deviation");
    assert_eq!(step.matches("PCURVE(").count(), 4);

    let mut imported = Topology::new();
    let solids = read_step(&step, &mut imported).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(imported.num_pcurves(), 4);
    let faces = solid_faces(&imported, solids[0]).unwrap();
    assert_eq!(faces.len(), 6);

    let mut top_planes = 0;
    for face_id in faces {
        let FaceSurface::Plane { normal, d } = *imported.face(face_id).unwrap().surface() else {
            panic!("box faces stay planar");
        };
        let length = normal.length();
        if (normal.z() / length - 1.0).abs() < TOL {
            top_planes += 1;
            let foot = normal * (d / normal.dot(normal));
            assert!(
                foot.x().abs() < TOL && foot.y().abs() < TOL && (foot.z() - 3.0).abs() < TOL,
                "top plane must round-trip at z = 3, got normal {normal:?}, d = {d}"
            );
        }
        for vertex_id in face_vertices(&imported, face_id).unwrap() {
            let p = imported.vertex(vertex_id).unwrap().point();
            let residual = (normal.dot(Vec3::new(p.x(), p.y(), p.z())) - d).abs() / length;
            assert!(residual < TOL, "vertex {p:?} is {residual} off its plane");
        }
    }
    assert_eq!(top_planes, 1);
}
