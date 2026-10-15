//! `sew_faces` refuses what it cannot rebuild and keeps face orientation.
//!
//! `sew_faces` rebuilds every boundary from its endpoints as straight edges.
//! It used to do so for any input: the three faces of a cylinder came back as
//! a solid whose circular rims were zero-length `Line` self-loops, a holed
//! face lost its hole (a box with a square through-hole measured 1026.67
//! against 960), and a reversed face lost its orientation flag. Those inputs
//! are now refused with the arena unchanged, and the flag is carried.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, missing_docs)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_math::vec::Vec3;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::loft::loft;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::sew::sew_faces;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::validation::check_same_range;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn arena_counts(topo: &Topology) -> (usize, usize, usize, usize) {
    (
        topo.num_faces(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    )
}

fn assert_refused_unchanged(topo: &mut Topology, faces: &[FaceId], label: &str) {
    let before = arena_counts(topo);
    let error = sew_faces(topo, faces, 1e-6).expect_err(label);
    assert!(
        matches!(error, OperationsError::InvalidInput { .. }),
        "{label}: unexpected error {error}"
    );
    assert_eq!(arena_counts(topo), before, "{label}: refusal left entities");
}

#[test]
fn curved_and_closed_boundaries_are_refused() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 1.0, 2.0).unwrap();
    let faces = solid_faces(&topo, cylinder).unwrap();
    assert_refused_unchanged(&mut topo, &faces, "cylinder faces");
}

#[test]
fn holed_faces_are_refused() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_box(&mut topo, 2.0, 2.0, 20.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(4.0, 4.0, -5.0)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, blank, tool).unwrap();
    let faces = solid_faces(&topo, holed).unwrap();
    assert!(
        faces
            .iter()
            .any(|&f| !topo.face(f).unwrap().inner_wires().is_empty()),
        "premise: a holed face"
    );
    assert_refused_unchanged(&mut topo, &faces, "holed box faces");
}

#[test]
fn reversed_face_keeps_its_orientation() {
    // A 10³ box whose top face is stored reversed: plane normal −z with the
    // reversal flag set, so its effective normal is still +z.
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let mut faces = solid_faces(&topo, cube).unwrap();
    let top_index = faces
        .iter()
        .position(|&f| {
            topo.face(f)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.z() > 0.5)
        })
        .expect("top face");
    let top = topo.face(faces[top_index]).unwrap();
    let FaceSurface::Plane { normal, d } = *top.surface() else {
        panic!("top face is planar");
    };
    // A reversed face's wire winds about the stored (here −z) normal; the
    // flag flips its effective sense back to the original traversal.
    let mut edges = topo.wire(top.outer_wire()).unwrap().edges().to_vec();
    edges.reverse();
    let edges = edges
        .into_iter()
        .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    faces[top_index] = topo.add_face(Face::new_reversed(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: -normal,
            d: -d,
        },
    ));
    // Premise: the box with its reversed top face is itself valid.
    let input_shell = topo.add_shell(Shell::new(faces.clone()).unwrap());
    let input = topo.add_solid(Solid::new(input_shell, vec![]));
    let input_report = validate_solid(&topo, input).unwrap();
    assert!(
        input_report.is_valid(),
        "premise: {:#?}",
        input_report.issues
    );

    let solid = sew_faces(&mut topo, &faces, 1e-6).unwrap();
    let report = validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);
    let volume = solid_volume(&topo, solid, 1e-4).unwrap();
    assert!((volume - 1000.0).abs() < 1e-6, "volume {volume}");
    let top_normal = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|f| topo.face(f).unwrap().effective_plane_normal())
        .find(|n| n.dot(Vec3::new(0.0, 0.0, 1.0)) > 0.5);
    assert!(top_normal.is_some(), "the sewn top face still faces +z");
}

#[test]
fn loose_planar_quads_still_sew() {
    // Control: six independent quads of the unit cube.
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let faces = solid_faces(&topo, cube).unwrap();
    let solid = sew_faces(&mut topo, &faces, 1e-6).unwrap();
    assert!(validate_solid(&topo, solid).unwrap().is_valid());
    let volume = solid_volume(&topo, solid, 1e-4).unwrap();
    assert!((volume - 1.0).abs() < 1e-9, "volume {volume}");
}

#[test]
fn edges_shorter_than_the_tolerance_are_refused() {
    // At tolerance 4 every corner of the unit cube rounds to the same merge
    // cell, so every rebuilt edge would join a vertex to itself.
    let mut topo = Topology::new();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let faces = solid_faces(&topo, cube).unwrap();
    let before = arena_counts(&topo);
    let error = sew_faces(&mut topo, &faces, 4.0).expect_err("collapsing edges");
    assert!(
        matches!(error, OperationsError::InvalidInput { .. }),
        "unexpected error {error}"
    );
    assert_eq!(arena_counts(&topo), before, "refusal left entities");
}

/// A planar `z = height` face bounded by straight edges through `corners`.
fn polygon_face(topo: &mut Topology, corners: &[[f64; 2]], height: f64) -> FaceId {
    let vertices: Vec<_> = corners
        .iter()
        .map(|&[x, y]| topo.add_vertex(Vertex::new(Point3::new(x, y, height), 1e-7)))
        .collect();
    let edges = (0..vertices.len())
        .map(|i| {
            let edge = Edge::new(
                vertices[i],
                vertices[(i + 1) % vertices.len()],
                EdgeCurve::Line,
            );
            OrientedEdge::new(topo.add_edge(edge), true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: height,
        },
    ))
}

#[test]
fn corners_merged_by_the_tolerance_are_refused() {
    // A thin diamond whose two side corners lie 0.002 apart: every edge is
    // about 1 long, but at tolerance 0.01 the side corners share a merge cell
    // and the loop would pinch into reused edges.
    let mut topo = Topology::new();
    let diamond = [[0.0, 0.0], [1.0, -0.001], [2.0, 0.0], [1.0, 0.001]];
    let faces = [
        polygon_face(&mut topo, &diamond, 0.0),
        polygon_face(&mut topo, &diamond, 5.0),
    ];
    let before = arena_counts(&topo);
    let error = sew_faces(&mut topo, &faces, 0.01).expect_err("pinched loop");
    assert!(
        matches!(error, OperationsError::InvalidInput { .. }),
        "unexpected error {error}"
    );
    assert_eq!(arena_counts(&topo), before, "refusal left entities");
}

/// Boundary uses of `faces` that carry a pcurve, with each one's SameRange
/// deviation.
fn pcurve_uses(topo: &Topology, faces: &[FaceId]) -> Vec<f64> {
    let mut deviations = Vec::new();
    for &face in faces {
        let wire = topo.wire(topo.face(face).unwrap().outer_wire()).unwrap();
        for oe in wire.edges() {
            if topo
                .pcurve_oriented(oe.edge(), face, oe.is_forward())
                .is_some()
            {
                let deviation = check_same_range(topo, oe.edge(), face, oe.is_forward())
                    .unwrap()
                    .unwrap();
                deviations.push(deviation);
            }
        }
    }
    deviations
}

#[test]
fn ruled_loft_faces_keep_their_pcurves() {
    // A square lofted to a diamond gives twisted ruled sides: NURBS quads
    // bounded by straight edges, each with authored rectangular pcurves.
    let mut topo = Topology::new();
    let bottom = polygon_face(
        &mut topo,
        &[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
        0.0,
    );
    let top = polygon_face(
        &mut topo,
        &[[1.0, -0.4], [2.4, 1.0], [1.0, 2.4], [-0.4, 1.0]],
        3.0,
    );
    let lofted = loft(&mut topo, &[bottom, top]).unwrap();
    let faces = solid_faces(&topo, lofted).unwrap();
    let before = pcurve_uses(&topo, &faces);
    assert!(!before.is_empty(), "the lofted sides should carry pcurves");

    let sewn = sew_faces(&mut topo, &faces, 1e-7).unwrap();
    let after = pcurve_uses(&topo, &solid_faces(&topo, sewn).unwrap());
    assert_eq!(
        after.len(),
        before.len(),
        "every authored pcurve carries over"
    );
    for deviation in after {
        assert!(
            deviation < 1e-9,
            "a carried pcurve misses its use's ends by {deviation}"
        );
    }
}
