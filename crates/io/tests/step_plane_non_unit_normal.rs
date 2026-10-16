//! A `FaceSurface::Plane` may store a non-unit normal: `{ normal, d }` means
//! `normal · p = d`. Its canonical chart (`PlaneChart::canonical` in the
//! topology SameParameter proof) has origin at the foot of the plane,
//! `normal * d / |normal|²`, and axes `u = n̂ × seed`, `v = n̂ × u`, where the
//! seed is `x̂` unless `|n̂ · x̂| ≥ 0.9`. The STEP writer and reader once built
//! the origin as `normal * d` and picked the seed from the unnormalized
//! normal, so a non-unit normal moved or rotated their chart away from it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::{reader::read_step, writer::write_step};
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::{
    Topology,
    explorer::{face_vertices, solid_faces},
    face::{FaceId, FaceSurface},
    pcurve::PCurve,
};

const TOL: f64 = 1e-9;

/// Independent restatement of `PlaneChart::canonical`: (origin, u, v).
fn canonical_chart(normal: Vec3, d: f64) -> (Point3, Vec3, Vec3) {
    let foot = normal * (d / normal.dot(normal));
    let n = normal.normalize().unwrap();
    let seed = if n.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = n.cross(seed).normalize().unwrap();
    let v = n.cross(u);
    (Point3::new(foot.x(), foot.y(), foot.z()), u, v)
}

fn plane(topo: &Topology, face: FaceId) -> (Vec3, f64) {
    let FaceSurface::Plane { normal, d } = *topo.face(face).unwrap().surface() else {
        panic!("box faces stay planar");
    };
    (normal, d)
}

/// Give every coedge of `face`'s outer loop a line pcurve in its canonical chart.
fn attach_canonical_pcurves(topo: &mut Topology, face: FaceId) {
    let (normal, d) = plane(topo, face);
    let (origin, u, v) = canonical_chart(normal, d);
    let chart = |p: Point3| Point2::new((p - origin).dot(u), (p - origin).dot(v));
    let loop_id = topo.face(face).unwrap().outer_loop().expect("outer loop");
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
}

/// Export, re-import, and check that every imported plane holds its vertices
/// and every imported pcurve endpoint lands on its vertex through the
/// canonical chart of the imported plane.
fn round_trip(topo: &Topology, solid: remus_topology::solid::SolidId) -> Topology {
    let step = write_step(topo, &[solid])
        .expect("a valid non-unit-normal planar face must export without a pcurve deviation");
    assert_eq!(step.matches("PCURVE(").count(), 4);

    let mut imported = Topology::new();
    let solids = read_step(&step, &mut imported).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(imported.num_pcurves(), 4);
    let faces = solid_faces(&imported, solids[0]).unwrap();
    assert_eq!(faces.len(), 6);

    let mut checked_pcurves = 0;
    for face_id in faces {
        let (normal, d) = plane(&imported, face_id);
        let length = normal.length();
        for vertex_id in face_vertices(&imported, face_id).unwrap() {
            let p = imported.vertex(vertex_id).unwrap().point();
            let residual = (normal.dot(Vec3::new(p.x(), p.y(), p.z())) - d).abs() / length;
            assert!(residual < TOL, "vertex {p:?} is {residual} off its plane");
        }
        let (origin, u, v) = canonical_chart(normal, d);
        let loop_id = imported.face(face_id).unwrap().outer_loop().unwrap();
        for &coedge_id in imported.face_loop(loop_id).unwrap().coedges() {
            let coedge = imported.coedge(coedge_id).unwrap();
            let Some(pcurve) = coedge.pcurve() else {
                continue;
            };
            let edge = imported.edge(coedge.edge()).unwrap();
            let (start, end) = if coedge.is_forward() {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            for (t, vertex) in [(pcurve.t_start(), start), (pcurve.t_end(), end)] {
                let uv = pcurve.evaluate(t);
                let on_chart = origin + u * uv.x() + v * uv.y();
                let expected = imported.vertex(vertex).unwrap().point();
                let miss = (on_chart - expected).length();
                assert!(miss < TOL, "imported pcurve misses its vertex by {miss}");
            }
            checked_pcurves += 1;
        }
    }
    assert_eq!(checked_pcurves, 4);
    imported
}

#[test]
fn non_unit_normal_plane_with_pcurves_exports_and_round_trips_on_its_plane() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 5.0, 3.0).unwrap();
    let top = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            let (normal, d) = plane(&topo, face);
            (normal.z() - 1.0).abs() < TOL && (d - 3.0).abs() < TOL
        })
        .expect("box has a top face at z = 3");
    // Same plane z = 3.
    topo.face_mut(top).unwrap().set_surface(FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 2.5),
        d: 7.5,
    });
    attach_canonical_pcurves(&mut topo, top);

    let imported = round_trip(&topo, solid);
    let solid = imported.solids().iter().next().unwrap().0;
    let top_feet: Vec<_> = solid_faces(&imported, solid)
        .unwrap()
        .into_iter()
        .map(|face| plane(&imported, face))
        .filter(|(normal, _)| (normal.z() / normal.length() - 1.0).abs() < TOL)
        .map(|(normal, d)| normal * (d / normal.dot(normal)))
        .collect();
    assert_eq!(top_feet.len(), 1);
    let foot = top_feet[0];
    assert!(
        foot.x().abs() < TOL && foot.y().abs() < TOL && (foot.z() - 3.0).abs() < TOL,
        "top plane must round-trip at z = 3, got foot {foot:?}"
    );
}

#[test]
fn non_unit_normal_picks_the_chart_seed_from_the_unit_normal() {
    // A side face turned 45° about z has n̂ = (√½, √½, 0): |n̂·x̂| < 0.9 picks
    // the x̂ seed, but the same plane stored with |n| = 2 has |n·x̂| ≈ 1.41.
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 5.0, 3.0).unwrap();
    transform_solid(
        &mut topo,
        solid,
        &Mat4::rotation_z(std::f64::consts::FRAC_PI_4),
    )
    .unwrap();
    let diagonal = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            let (normal, _) = plane(&topo, face);
            (normal.x() - std::f64::consts::FRAC_1_SQRT_2).abs() < TOL
                && (normal.y() - std::f64::consts::FRAC_1_SQRT_2).abs() < TOL
        })
        .expect("rotated box has a face with normal (√½, √½, 0)");
    let (normal, d) = plane(&topo, diagonal);
    topo.face_mut(diagonal)
        .unwrap()
        .set_surface(FaceSurface::Plane {
            normal: normal * 2.0,
            d: d * 2.0,
        });
    attach_canonical_pcurves(&mut topo, diagonal);

    round_trip(&topo, solid);
}

#[test]
fn overflowed_canonical_plane_origin_refuses_export_without_pcurves() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 5.0, 3.0).unwrap();
    let top = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&id| {
            matches!(topo.face(id).unwrap().surface(), FaceSurface::Plane { normal, d }
            if (normal.z()-1.0).abs()<TOL && (*d-3.0).abs()<TOL)
        })
        .unwrap();
    // The normal can be normalized, but the existing SameParameter chart's
    // multiplication order cannot represent this otherwise finite plane foot.
    topo.face_mut(top).unwrap().set_surface(FaceSurface::Plane {
        normal: Vec3::new(0.0, 0.0, 1e154),
        d: 3e154,
    });
    assert!(Vec3::new(0.0, 0.0, 1e154).normalize().is_ok());
    let error = write_step(&topo, &[solid])
        .expect_err("unsupported canonical origins must not emit an infinite STEP placement");
    assert!(matches!(error, remus_io::IoError::InvalidTopology { .. }));
}

#[test]
fn rounded_seed_threshold_matches_the_canonical_normalization() {
    let normal = Vec3::new(0.112_499_999_999_999_95, 0.054_486_236_794_258_42, 0.0);
    let unit = normal.normalize().unwrap();
    // This finite, moderate scaling rounds exactly onto the canonical seed
    // boundary; comparing nx with 0.9*length instead chooses the other seed.
    assert_eq!(unit.x().to_bits(), 0.9_f64.to_bits());
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 5.0, 3.0).unwrap();
    transform_solid(
        &mut topo,
        solid,
        &Mat4::rotation_z(normal.y().atan2(normal.x())),
    )
    .unwrap();
    let side = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            let (n, _) = plane(&topo, face);
            (n.x() - unit.x()).abs() < TOL && (n.y() - unit.y()).abs() < TOL
        })
        .unwrap();
    let point = topo
        .vertex(face_vertices(&topo, side).unwrap()[0])
        .unwrap()
        .point();
    let d = normal.dot(Vec3::new(point.x(), point.y(), point.z()));
    topo.face_mut(side)
        .unwrap()
        .set_surface(FaceSurface::Plane { normal, d });
    attach_canonical_pcurves(&mut topo, side);
    round_trip(&topo, solid);
}
