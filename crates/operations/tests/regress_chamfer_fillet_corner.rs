//! Three straight edges at one corner must chamfer after the far corners
//! were filleted, without flattening those cylinders or losing a through bore.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::measure::{edge_length, face_content_digest, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn far_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let e = topo.edge(id).unwrap();
            let a = topo.vertex(e.start()).unwrap().point();
            let b = topo.vertex(e.end()).unwrap().point();
            [(36.0, 0.0), (0.0, 41.0)].iter().any(|&(x, y)| {
                (a.x() - x).abs() + (b.x() - x).abs() + (a.y() - y).abs() + (b.y() - y).abs() < 1e-8
            }) && (a.z() - b.z()).abs() > 10.0
        })
        .collect()
}

fn corner_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let corner = Point3::new(0.0, 0.0, 11.5);
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let e = topo.edge(id).unwrap();
            [e.start(), e.end()]
                .into_iter()
                .any(|v| (topo.vertex(v).unwrap().point() - corner).length() < 1e-8)
        })
        .collect()
}

fn rounded_box(topo: &mut Topology, bore: bool, rim_chamfer: bool) -> SolidId {
    let mut body = make_box(topo, 36.0, 41.0, 11.5).unwrap();
    if bore {
        let tool = make_cylinder(topo, 3.0, 15.5).unwrap();
        transform_solid(topo, tool, &Mat4::translation(27.0, 30.0, -2.0)).unwrap();
        body = boolean(topo, BooleanOp::Cut, body, tool).unwrap();
        if rim_chamfer {
            let rim: Vec<_> = solid_edges(topo, body)
                .unwrap()
                .into_iter()
                .filter(|&id| {
                    let e = topo.edge(id).unwrap();
                    matches!(e.curve(), EdgeCurve::Circle(_))
                        && (topo.vertex(e.start()).unwrap().point().z() - 11.5).abs() < 1e-8
                })
                .collect();
            assert_eq!(rim.len(), 1);
            body = chamfer_v2(topo, body, &rim, 1.0, 1.0).unwrap().solid;
        }
    }
    let far = far_edges(topo, body);
    fillet_cascade(topo, body, &far, 1.0).unwrap().solid
}

fn round_count(topo: &Topology, solid: SolidId, radius: f64) -> usize {
    solid_faces(topo,solid).unwrap().iter().filter(|&&f|
        matches!(topo.face(f).unwrap().surface(),FaceSurface::Cylinder(c) if (c.radius()-radius).abs()<1e-9)).count()
}

fn fingerprint(topo: &Topology, solid: SolidId) -> Vec<String> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|f| face_content_digest(topo, f).unwrap().to_hex())
        .collect()
}

#[test]
fn three_edge_chamfer_preserves_earlier_endpoint_fillets() {
    let mut topo = Topology::new();
    let rounded = rounded_box(&mut topo, false, false);
    let input = fingerprint(&topo, rounded);
    let edges = corner_edges(&topo, rounded);
    assert_eq!(edges.len(), 3);
    let length: f64 = edges.iter().map(|&e| edge_length(&topo, e).unwrap()).sum();
    assert!((length - 86.5).abs() < 1e-8);
    let result =
        chamfer(&mut topo, rounded, &edges, 1.0).expect("chamfer at the still-sharp corner");
    assert!(
        remus_operations::validate::validate_solid(&topo, result)
            .unwrap()
            .is_valid()
    );
    // Two r1 vertical rounds remove 2*(1-pi/4)*11.5. Integrating the
    // bevel through their quarter-circle ends removes exactly 43.25 mm^3.
    let expected = 36.0 * 41.0 * 11.5 - 2.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 11.5 - 43.25;
    let actual = solid_volume(&topo, result, 0.001).unwrap();
    assert!(
        (actual - expected).abs() < 0.001,
        "expected {expected}, got {actual}"
    );
    assert_eq!(round_count(&topo, result, 1.0), 2);
    assert_eq!(fingerprint(&topo, rounded), input);
}

#[test]
fn drilled_chamfered_and_filleted_history_keeps_the_bore_and_exact_origins() {
    for rim_chamfer in [false, true] {
        let mut topo = Topology::new();
        let rounded = rounded_box(&mut topo, true, rim_chamfer);
        let input = fingerprint(&topo, rounded);
        let before = solid_volume(&topo, rounded, 0.001).unwrap();
        let edges = corner_edges(&topo, rounded);
        let result = chamfer_v2(&mut topo, rounded, &edges, 1.0, 1.0).unwrap();
        assert_eq!(result.succeeded, edges);
        assert!(!result.is_partial);
        let origins = result.face_origins.unwrap();
        assert!(origins.deleted.is_empty());
        assert!(origins.created_unattributed.is_empty());
        assert_eq!(
            origins.survived.len(),
            solid_faces(&topo, rounded).unwrap().len()
        );
        assert_eq!(origins.created.len(), 4);
        assert_eq!(
            origins
                .created
                .iter()
                .filter(|(_, sources)| sources.len() == 3)
                .count(),
            1
        );
        assert_eq!(round_count(&topo, result.solid, 1.0), 2);
        assert_eq!(round_count(&topo, result.solid, 3.0), 1);
        assert!((before - solid_volume(&topo, result.solid, 0.001).unwrap() - 43.25).abs() < 0.001);
        assert_eq!(fingerprint(&topo, rounded), input);
    }
}

#[test]
fn smaller_distances_match_independent_quarter_circle_integrals() {
    for distance in [0.1, 0.5, 1.0] {
        let mut topo = Topology::new();
        let rounded = rounded_box(&mut topo, false, false);
        let edges = corner_edges(&topo, rounded);
        let before = solid_volume(&topo, rounded, 0.001).unwrap();
        let result = chamfer(&mut topo, rounded, &edges, distance).unwrap();
        // Integral from 0 to d of (d-y)*sqrt(2*y-y*y), the bevel
        // volume over each quarter-circle end. This is independent of GFA.
        let u = distance - 1.0;
        let circle = (1.0 - u * u).sqrt();
        let integral =
            (distance - 1.0) * 0.5 * (u * circle + u.asin() + std::f64::consts::FRAC_PI_2)
                + circle.powi(3) / 3.0;
        let removed =
            0.5 * 88.5 * distance.powi(2) - 2.0 * distance.powi(3) / 3.0 - distance.powi(2)
                + 2.0 * integral;
        let actual = before - solid_volume(&topo, result, 0.001).unwrap();
        assert!(
            (actual - removed).abs() < 0.001,
            "d={distance}: expected {removed}, got {actual}"
        );
    }
}

#[test]
fn imported_corner_round_trips_without_flattening_the_fillets() {
    let mut topo = Topology::new();
    let rounded = rounded_box(&mut topo, true, false);
    let step = remus_io::step::write_step(&topo, &[rounded]).unwrap();
    let mut imported = Topology::new();
    let body = remus_io::step::read_step(&step, &mut imported).unwrap()[0];
    let edges = corner_edges(&imported, body);
    let result = chamfer(&mut imported, body, &edges, 1.0).unwrap();
    let expected = solid_volume(&imported, result, 0.001).unwrap();
    let output = remus_io::step::write_step(&imported, &[result]).unwrap();
    let mut reread = Topology::new();
    let body = remus_io::step::read_step(&output, &mut reread).unwrap()[0];
    assert_eq!(round_count(&reread, body, 1.0), 2);
    assert_eq!(round_count(&reread, body, 3.0), 1);
    assert!(
        remus_operations::validate::validate_solid(&reread, body)
            .unwrap()
            .is_valid()
    );
    assert!((solid_volume(&reread, body, 0.001).unwrap() - expected).abs() < 0.001);
}

#[test]
fn unsupported_two_edge_and_oversized_selections_leave_the_input_intact() {
    let mut topo = Topology::new();
    let body = rounded_box(&mut topo, true, false);
    let input = fingerprint(&topo, body);
    let edges = corner_edges(&topo, body);
    for (count, distance) in [(2, 1.0), (3, 20.0)] {
        assert!(chamfer(&mut topo, body, &edges[..count], distance).is_err());
        assert_eq!(fingerprint(&topo, body), input);
    }
}

#[test]
fn rotated_scaled_and_mirrored_corners_preserve_the_same_geometry() {
    for (scale, mirror) in [(0.1, false), (1.0, false), (10.0, true)] {
        let mut topo = Topology::new();
        let body = rounded_box(&mut topo, false, false);
        let edges = corner_edges(&topo, body);
        let before = solid_volume(&topo, body, 0.001).unwrap();
        let matrix = Mat4::translation(13.0, -7.0, 3.0)
            * Mat4::rotation_z(0.37)
            * Mat4::rotation_x(0.23)
            * Mat4::scale(if mirror { -scale } else { scale }, scale, scale);
        transform_solid(&mut topo, body, &matrix).unwrap();
        let result = chamfer(&mut topo, body, &edges, scale).unwrap();
        let expected = (before - 43.25) * scale.powi(3);
        let actual = solid_volume(&topo, result, 0.001 * scale).unwrap();
        assert!(
            (actual - expected).abs() < 0.001 * scale.powi(3),
            "scale={scale}, mirror={mirror}: expected {expected}, got {actual}"
        );
    }
}

#[test]
fn a_bevel_that_would_open_an_unselected_bore_refuses_atomically() {
    let mut topo = Topology::new();
    let rounded = rounded_box(&mut topo, false, false);
    let tool = make_cylinder(&mut topo, 0.2, 15.5).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(15.0, 0.5, -2.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, rounded, tool).unwrap();
    let input = fingerprint(&topo, body);
    let edges = corner_edges(&topo, body);
    assert!(chamfer(&mut topo, body, &edges, 1.0).is_err());
    assert_eq!(fingerprint(&topo, body), input);
}
