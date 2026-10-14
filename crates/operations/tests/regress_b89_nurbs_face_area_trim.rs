//! B89 (kernel review 2026-10-14, OPS-4): `face_area` and
//! `solid_surface_area` measured a NURBS face by its untrimmed carrier.
//!
//! `convert_to_bspline` gives each planar box face a carrier patch that
//! overhangs the face, so a converted 10³ box read 144 per face and 864 in
//! total, and a converted face with an inner wire refused outright (the
//! standalone mesher does not support holes). A NURBS face now measures the
//! region its outer wire bounds minus its inner wires.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::convert_to_bspline;
use remus_operations::measure::{face_area, solid_surface_area};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn assert_all_nurbs(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    let faces = solid_faces(topo, solid).unwrap();
    for &face in &faces {
        assert!(
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)),
            "face {face:?} was not converted"
        );
    }
    faces
}

#[test]
fn converted_box_faces_measure_their_trim_at_every_scale_and_placement() {
    for scale in [1e-3, 1.0, 1e3] {
        for offset in [(0.0, 0.0, 0.0), (37.0, -91.0, 13.0)] {
            let mut topo = Topology::new();
            let side = 10.0 * scale;
            let solid = make_box(&mut topo, side, side, side).unwrap();
            transform_solid(
                &mut topo,
                solid,
                &Mat4::translation(offset.0 * scale, offset.1 * scale, offset.2 * scale),
            )
            .unwrap();
            convert_to_bspline(&mut topo, solid).unwrap();
            let faces = assert_all_nurbs(&topo, solid);
            assert_eq!(faces.len(), 6);

            // Planar patches mesh exactly, so the reading is deflection-free.
            let face_exact = side * side;
            for deflection in [0.1 * scale, 1e-3 * scale] {
                for &face in &faces {
                    let area = face_area(&topo, face, deflection).unwrap();
                    assert!(
                        (area - face_exact).abs() <= 1e-9 * face_exact,
                        "scale {scale}, offset {offset:?}, face {face:?}: {area} vs {face_exact}"
                    );
                }
                let total = solid_surface_area(&topo, solid, deflection).unwrap();
                assert!(
                    (total - 6.0 * face_exact).abs() <= 1e-9 * 6.0 * face_exact,
                    "scale {scale}, offset {offset:?}: total {total} vs {}",
                    6.0 * face_exact
                );
            }
        }
    }
}

/// A 20 × 20 × 4 plate with a through-bore of radius 3 at its centre,
/// measured before and after conversion. The two cap faces carry the bore
/// rim as an inner wire.
#[test]
fn converted_holed_plate_matches_the_native_plate() {
    const RADIUS: f64 = 3.0;
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 20.0, 20.0, 4.0).unwrap();
    let tool = make_cylinder(&mut topo, RADIUS, 10.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(10.0, 10.0, -3.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, plate, tool).unwrap();

    let faces = solid_faces(&topo, solid).unwrap();
    let native: Vec<f64> = faces
        .iter()
        .map(|&face| face_area(&topo, face, 0.01).unwrap())
        .collect();
    let native_total = solid_surface_area(&topo, solid, 0.01).unwrap();
    let holed_exact = 400.0 - PI * RADIUS * RADIUS;
    let bore_exact = 2.0 * PI * RADIUS * 4.0;
    assert!((native_total - (2.0 * holed_exact + 4.0 * 80.0 + bore_exact)).abs() < 1e-9);

    convert_to_bspline(&mut topo, solid).unwrap();
    assert_eq!(assert_all_nurbs(&topo, solid), faces);

    let mut holed = 0;
    for deflection in [0.01, 1e-3] {
        let mut total_bound = 0.0;
        for (&face, &exact) in faces.iter().zip(&native) {
            let area = face_area(&topo, face, deflection).unwrap();
            let inner_wires = topo.face(face).unwrap().inner_wires().len();
            // An inscribed mesh: a hole polygon loses at most its perimeter
            // times the sag from the hole, so the face reads high; a curved
            // wall reads low by about 4d/(3r) of its area.
            let (low, high) = if inner_wires == 1 {
                holed += 1;
                (0.0, 2.0 * PI * RADIUS * deflection)
            } else if (exact - bore_exact).abs() < 1e-9 {
                (2.0 * deflection / RADIUS * exact, 0.0)
            } else {
                (1e-9 * exact, 1e-9 * exact)
            };
            assert!(
                area >= exact - low - 1e-12 && area <= exact + high + 1e-12,
                "deflection {deflection}, face {face:?} ({inner_wires} holes): {area} vs native {exact}"
            );
            total_bound += low.max(high);
        }
        let total = solid_surface_area(&topo, solid, deflection).unwrap();
        assert!(
            (total - native_total).abs() <= total_bound,
            "deflection {deflection}: total {total} vs native {native_total}"
        );
    }
    assert_eq!(
        holed, 4,
        "both caps keep their bore rim at both deflections"
    );
}
