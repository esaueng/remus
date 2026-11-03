//! A single planar ridge must chamfer through an existing cylindrical end round.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::measure::{face_content_digest, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn rounded_box(topo: &mut Topology) -> SolidId {
    let body = make_box(topo, 36.0, 41.0, 11.5).unwrap();
    let far: Vec<_> = solid_edges(topo, body)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            [(36.0, 0.0), (0.0, 41.0)].iter().any(|&(x, y)| {
                (a.x() - x).abs() + (b.x() - x).abs() + (a.y() - y).abs() + (b.y() - y).abs() < 1e-8
            }) && (a.z() - b.z()).abs() > 10.0
        })
        .collect();
    assert_eq!(far.len(), 2);
    fillet_cascade(topo, body, &far, 1.0).unwrap().solid
}

fn selected_edge(topo: &Topology, body: SolidId) -> EdgeId {
    solid_edges(topo, body)
        .unwrap()
        .into_iter()
        .find(|&id| {
            let edge = topo.edge(id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            a.y().abs() + a.z().abs() + b.y().abs() + b.z().abs() < 1e-8
                && (a.x() - b.x()).abs() > 30.0
        })
        .unwrap()
}

fn fingerprint(topo: &Topology, solid: SolidId) -> Vec<String> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|f| face_content_digest(topo, f).unwrap().to_hex())
        .collect()
}

// Integrate (d-y) times the original section width. For y<1 the width is
// 35+sqrt(2y-y^2); above the quarter-circle end it is exactly 36. This oracle
// does not use the clipping plane, the Boolean result or the kernel mesher.
fn removed_volume(distance: f64) -> f64 {
    let limit = distance.min(1.0);
    let u = limit - 1.0;
    let circle = (1.0 - u * u).sqrt();
    let arc = 0.5 * (u * circle + u.asin() + std::f64::consts::FRAC_PI_2);
    35.0 * (distance * limit - 0.5 * limit * limit)
        + (distance - 1.0) * arc
        + circle.powi(3) / 3.0
        + 18.0 * (distance - 1.0).max(0.0).powi(2)
}

#[test]
fn single_edge_chamfers_preserve_rounds_and_match_section_integrals() {
    for distance in [0.03281, 0.1, 0.2625, 0.5, 1.0, 1.05, 2.1] {
        let mut topo = Topology::new();
        let body = rounded_box(&mut topo);
        let edge = selected_edge(&topo, body);
        let input = fingerprint(&topo, body);
        let before = solid_volume(&topo, body, 0.001).unwrap();
        let result = chamfer_v2(&mut topo, body, &[edge], distance, distance)
            .expect("a single straight ridge ending on an earlier round");
        assert_eq!(result.succeeded, [edge]);
        assert!(!result.is_partial);
        assert!(
            remus_operations::validate::validate_solid(&topo, result.solid)
                .unwrap()
                .is_valid()
        );
        let actual = before - solid_volume(&topo, result.solid, 0.001).unwrap();
        assert!(
            (actual - removed_volume(distance)).abs() < 0.001,
            "d={distance}: expected {}, got {actual}",
            removed_volume(distance)
        );
        let rounds = solid_faces(&topo, result.solid).unwrap().into_iter().filter(|&f|
            matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(c) if (c.radius()-1.0).abs()<1e-9)).count();
        assert_eq!(rounds, 2);
        let origins = result.face_origins.unwrap();
        assert_eq!(origins.created.len(), 1);
        assert_eq!(origins.created[0].1.len(), 2);
        assert_eq!(
            origins.survived.len(),
            solid_faces(&topo, body).unwrap().len()
        );
        assert!(origins.deleted.is_empty());
        assert!(origins.created_unattributed.is_empty());
        assert_eq!(fingerprint(&topo, body), input);
    }
}

#[test]
fn single_edge_round_trip_and_placement_preserve_exact_geometry() {
    for (scale, mirror) in [(0.1, false), (1.0, false), (10.0, true)] {
        let mut topo = Topology::new();
        let body = rounded_box(&mut topo);
        let step = remus_io::step::write_step(&topo, &[body]).unwrap();
        let mut imported = Topology::new();
        let body = remus_io::step::read_step(&step, &mut imported).unwrap()[0];
        let edge = selected_edge(&imported, body);
        let before = solid_volume(&imported, body, 0.001).unwrap();
        transform_solid(
            &mut imported,
            body,
            &(Mat4::translation(13.0, -7.0, 3.0)
                * Mat4::rotation_z(0.37)
                * Mat4::rotation_x(0.23)
                * Mat4::scale(if mirror { -scale } else { scale }, scale, scale)),
        )
        .unwrap();
        let result = chamfer(&mut imported, body, &[edge], 2.1 * scale).unwrap();
        let step = remus_io::step::write_step(&imported, &[result]).unwrap();
        let mut reread = Topology::new();
        let body = remus_io::step::read_step(&step, &mut reread).unwrap()[0];
        assert!(
            remus_operations::validate::validate_solid(&reread, body)
                .unwrap()
                .is_valid()
        );
        let expected = (before - removed_volume(2.1)) * scale.powi(3);
        let actual = solid_volume(&reread, body, 0.001 * scale).unwrap();
        assert!(
            (actual - expected).abs() < 0.001 * scale.powi(3),
            "scale={scale}, mirror={mirror}: expected {expected}, got {actual}"
        );
    }
}

#[test]
fn a_single_bevel_cannot_open_an_unselected_bore_or_consume_a_face() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo);
    let tool = make_cylinder(&mut topo, 0.2, 15.5).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(15.0, 0.5, -2.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, body, tool).unwrap();
    let edge = selected_edge(&topo, body);
    let input = fingerprint(&topo, body);
    for distance in [1.0, 20.0] {
        assert!(chamfer(&mut topo, body, &[edge], distance).is_err());
        assert_eq!(fingerprint(&topo, body), input);
    }
}
