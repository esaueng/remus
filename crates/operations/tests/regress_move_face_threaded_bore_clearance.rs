//! Moving a block top whose bounding box covers a threaded bore.
//!
//! A boss through the corner of a block, the junction rounded, and a bore
//! threaded inside the boss: the block top wraps the boss, so its box covers
//! the bore, and the R2 junction torus that travels with it covers the bore
//! too. The thread's flanks are NURBS faces, about 18 from the bore axis at
//! most, while the block top and the torus stay at least 40 from the boss
//! axis. Translation along the axes keeps those distances, so moving the top
//! up is clear; a box test alone refused it at the first thread band.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::thread::{CylinderThreadOptions, ThreadForm, ThreadKind, cut_thread};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{face_vertices, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn planar_at(topo: &Topology, solid: SolidId, height: f64, normal: Vec3) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(normal) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .expect("planar face at requested height")
}

fn cylinder_face(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(),
                FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9)
        })
        .expect("cylindrical face of the requested radius")
}

fn nurbs_faces(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count()
}

fn assert_valid_and_watertight(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.05, 0.3).unwrap();
    assert!(is_watertight(&mesh));
}

const W: f64 = 74.0;
const D: f64 = 67.0;
const H: f64 = 35.5;
const R: f64 = 40.0;

/// An R40 boss 60 tall about the z axis through the corner of a block
/// 74 x 67 x 35.5, the boss/block perimeter rounded R2, then a blind bore
/// of radius 11.6 about (3, 2) from the boss cap down to z = 20, threaded
/// internally at pitch 1.5 (the edit's part, rebuilt natively).
fn part() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, W, D, H).unwrap();
    let boss = make_cylinder(&mut topo, R, 60.0).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    // The junction arc on the block top, the block top's edges along the
    // walls the boss crosses, and the two vertical seams of boss and block.
    let perimeter: Vec<_> = solid_edges(&topo, fused)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let at = |a: f64, b: f64| (a - b).abs() < 1e-7;
            let top = at(a.z(), H) && at(b.z(), H);
            let both = |x: bool, v: f64| {
                let c = |p: Point3| if x { p.x() } else { p.y() };
                at(c(a), v) && at(c(b), v)
            };
            match edge.curve() {
                EdgeCurve::Circle(_) => top,
                EdgeCurve::Line => {
                    (top && (both(true, 0.0) || both(false, 0.0)))
                        || ((a.z() - b.z()).abs() > H - 0.01
                            && ((both(true, R) && both(false, 0.0))
                                || (both(false, R) && both(true, 0.0))))
                }
                _ => false,
            }
        })
        .collect();
    assert_eq!(perimeter.len(), 5);
    let rounded = fillet_cascade(&mut topo, fused, &perimeter, 2.0)
        .unwrap()
        .solid;
    let tool = make_cylinder(&mut topo, 11.6, 44.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(3.0, 2.0, 20.0)).unwrap();
    let bored = boolean(&mut topo, BooleanOp::Cut, rounded, tool).unwrap();
    let wall = cylinder_face(&topo, bored, 11.6);
    let form = ThreadForm::iso_metric(1.5, ThreadKind::Internal);
    let threaded = cut_thread(
        &mut topo,
        bored,
        wall,
        &form,
        &CylinderThreadOptions::default(),
    )
    .unwrap()
    .solid;
    (topo, threaded)
}

/// The section of a round of radius `r` in the right-angled concave corner
/// of a wall plane through the centre of a boss of radius `big` and the
/// boss: by Green's theorem along the wall, the boss arc and the round.
fn seam_round_area(big: f64, r: f64) -> f64 {
    let (cx, cy) = (((big + r) * (big + r) - r * r).sqrt(), r);
    let boss_arc = 0.5 * big * big * cy.atan2(cx);
    let (a, b) = ((-cy).atan2(-cx), -PI / 2.0);
    let round_arc = 0.5
        * (r * r).mul_add(
            b - a,
            r * (cx * (b.sin() - a.sin()) - cy * (b.cos() - a.cos())),
        );
    (boss_arc + round_arc).abs()
}

#[test]
fn block_top_moves_up_past_a_threaded_bore_its_box_covers() {
    let (topo, solid) = part();
    assert!(nurbs_faces(&topo, solid) > 50, "the bore is threaded");
    let top = planar_at(&topo, solid, H, UP);
    // The block top's box reaches over the bore's footprint.
    let corners: Vec<Point3> = face_vertices(&topo, top)
        .unwrap()
        .into_iter()
        .map(|v| topo.vertex(v).unwrap().point())
        .collect();
    assert!(corners.iter().any(|p| p.x() < 3.0 + 11.6));
    assert!(corners.iter().any(|p| p.y() < 2.0 + 11.6));
    let before = solid_volume(&topo, solid, 0.01).unwrap();
    // The roof over the block outside the boss rises with the top, and the
    // two R2 rounds in the vertical seams of boss and block stretch with it.
    let roof = W.mul_add(D, -PI * R * R / 4.0) + 2.0 * seam_round_area(R, 2.0);
    for distance in [1.0, 10.0] {
        let mut scratch = topo.clone();
        let moved = move_faces(&mut scratch, solid, &[top], distance).unwrap();
        assert_valid_and_watertight(&scratch, moved);
        let after = solid_volume(&scratch, moved, 0.01).unwrap();
        let expected = roof * distance;
        assert!(
            ((after - before) - expected).abs() < 1e-9 * before,
            "{distance}: volume grew by {} for {expected}",
            after - before
        );
        planar_at(&scratch, moved, H + distance, UP);
    }
}

/// A cup whose lip overhangs an externally threaded post: the lip's
/// underside, moved down, sweeps through the thread's crest.
#[test]
fn lip_moved_onto_a_threaded_post_is_refused() {
    use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
    let mut topo = Topology::new();
    let section = [
        (0., 0.),
        (20., 0.),
        (20., 20.),
        (5.5, 20.),
        (5.5, 16.),
        (16., 16.),
        (16., 4.),
        (6., 4.),
        (6., 12.),
        (0., 12.),
    ]
    .map(|(r, z)| Point3::new(r, 0., z));
    let wire = make_polygon_wire(&mut topo, &section, 1e-7).unwrap();
    let profile = make_planar_face_from_wire(&mut topo, wire).unwrap();
    let cup = remus_operations::revolve::revolve(
        &mut topo,
        profile,
        Point3::new(0., 0., 0.),
        UP,
        std::f64::consts::TAU,
    )
    .unwrap();
    let post = cylinder_face(&topo, cup, 6.0);
    let form = ThreadForm::iso_metric(1.5, ThreadKind::External);
    let threaded = cut_thread(
        &mut topo,
        cup,
        post,
        &form,
        &CylinderThreadOptions::default(),
    )
    .unwrap()
    .solid;
    assert!(nurbs_faces(&topo, threaded) > 0, "the post is threaded");
    let lip = planar_at(&topo, threaded, 16.0, -UP);
    let faces = solid_faces(&topo, threaded).unwrap();
    let before = solid_volume(&topo, threaded, 0.01).unwrap();
    let err = move_faces(&mut topo, threaded, &[lip], 7.0).unwrap_err();
    assert!(
        err.to_string()
            .contains("swept face reaches nonadjacent face"),
        "{err}"
    );
    assert_eq!(solid_faces(&topo, threaded).unwrap(), faces);
    let after = solid_volume(&topo, threaded, 0.01).unwrap();
    assert_eq!(before.to_bits(), after.to_bits());
    // Short of the post's top the lip still moves.
    let mut scratch = topo.clone();
    let moved = move_faces(&mut scratch, threaded, &[lip], 2.0).unwrap();
    assert!(validate_solid(&scratch, moved).unwrap().is_valid());
}
