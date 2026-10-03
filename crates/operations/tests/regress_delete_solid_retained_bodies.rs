//! Deleting a source solid must preserve independently constructed body roots.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_check::validate::ValidateOptions;
use remus_operations::measure::{face_area, sheet_surface_area, wire_length};
use remus_operations::primitives::make_box;
use remus_operations::sew::make_sheet_body;
use remus_operations::tessellate::{TriangleMesh, tessellate_sheet};
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::validation::validate_boundary_authority;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{BodyClass, BodyId, Topology};

fn mesh_area(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            (b - a).cross(c - a).length() * 0.5
        })
        .sum()
}

#[test]
fn sheets_from_box_faces_survive_deleting_the_box() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let shell = topo.solid(solid).unwrap().outer_shell();
    let faces = topo.shell(shell).unwrap().faces().to_vec();
    let attributes = EntityAttributes {
        name: Some("retained sheet face".into()),
        color: Some(ColorRgb::new(0.2, 0.4, 0.6).unwrap()),
    };
    topo.set_face_attributes(faces[0], attributes.clone())
        .unwrap();
    let sheets: Vec<_> = faces
        .iter()
        .map(|&face| {
            let area = face_area(&topo, face, 0.01).unwrap();
            (make_sheet_body(&mut topo, &[face]).unwrap(), area)
        })
        .collect();
    let complete_sheet = make_sheet_body(&mut topo, &faces).unwrap();

    topo.delete_solid(solid).unwrap();

    assert!(topo.solid(solid).is_err());
    assert!(topo.shell(shell).is_err());
    assert_eq!(topo.num_solids(), 0);
    assert_eq!(topo.num_faces(), 6);
    assert_eq!(topo.num_edges(), 12);
    assert_eq!(topo.num_vertices(), 8);
    for (sheet, expected_area) in sheets {
        assert_eq!(
            topo.body_class_of(BodyId::Shell(sheet)).unwrap(),
            BodyClass::Sheet
        );
        let report =
            remus_check::validate::validate_sheet_body(&topo, sheet, &ValidateOptions::default())
                .unwrap();
        assert!(report.is_valid(), "{report:?}");
        assert!((sheet_surface_area(&topo, sheet, 0.01).unwrap() - expected_area).abs() < 1e-12);
        let mesh = tessellate_sheet(&topo, sheet, 0.01).unwrap();
        assert!((mesh_area(&mesh) - expected_area).abs() < 1e-12);
    }
    assert!((sheet_surface_area(&topo, complete_sheet, 0.01).unwrap() - 52.0).abs() < 1e-12);
    assert_eq!(topo.attributes().face(faces[0]), Some(&attributes));
    validate_boundary_authority(&topo).unwrap();

    let new_solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    assert!(new_solid.index() > solid.index());
    topo.delete_solid(new_solid).unwrap();
    assert_eq!(topo.num_faces(), 6);
    assert_eq!(topo.num_shells(), 7);
}

#[test]
fn standalone_wire_keeps_a_box_edge_after_deleting_the_box() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
    let shell = topo.solid(solid).unwrap().outer_shell();
    let face = topo.shell(shell).unwrap().faces()[0];
    let wire = topo.face(face).unwrap().outer_wire();
    let edge = topo.wire(wire).unwrap().edges()[0].edge();
    let wire_body = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], false).unwrap());

    topo.delete_solid(solid).unwrap();

    assert!((wire_length(&topo, wire_body).unwrap() - 2.0).abs() < 1e-12);
    assert!(
        remus_check::validate::validate_wire_body(&topo, wire_body, &ValidateOptions::default())
            .unwrap()
            .is_valid()
    );
    assert_eq!(topo.num_shells(), 0);
    assert_eq!(topo.num_faces(), 0);
    assert_eq!(topo.num_wires(), 1);
    assert_eq!(topo.num_edges(), 1);
    assert_eq!(topo.num_vertices(), 2);
    validate_boundary_authority(&topo).unwrap();
}
