//! Issue 4.4a qualification for sheet-by-solid keep-side trimming.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{SheetTrimMode, trim_sheet_by_solid};
use remus_operations::measure::{sheet_bounding_box, sheet_surface_area};
use remus_operations::primitives::make_box;
use remus_operations::sew::make_sheet_body;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const AREA_DEFLECTION: f64 = 1.0e-3;

fn planar_face(topo: &mut Topology, lo: f64, hi: f64, z: f64) -> remus_topology::FaceId {
    let points = [
        Point3::new(lo, lo, z),
        Point3::new(hi, lo, z),
        Point3::new(hi, hi, z),
        Point3::new(lo, hi, z),
    ];
    let vertices: Vec<_> = points
        .into_iter()
        .map(|point| topo.add_vertex(Vertex::new(point, 1.0e-7)))
        .collect();
    let edges: Vec<_> = (0..4)
        .map(|index| {
            topo.add_edge(Edge::new(
                vertices[index],
                vertices[(index + 1) % 4],
                EdgeCurve::Line,
            ))
        })
        .collect();
    let wire = topo.add_wire(
        Wire::new(
            edges
                .into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

fn trim_fixture(mode: SheetTrimMode) -> (Topology, remus_topology::ShellId) {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let face = planar_face(&mut topo, -2.0, 12.0, 5.0);
    let sheet = make_sheet_body(&mut topo, &[face]).unwrap();
    let result = trim_sheet_by_solid(&mut topo, sheet, solid, mode).unwrap();
    (topo, result)
}

#[test]
fn keep_inside_returns_the_exact_box_section_as_a_valid_sheet() {
    let (topo, sheet) = trim_fixture(SheetTrimMode::KeepInside);
    let report = remus_check::validate::validate_sheet_body(
        &topo,
        sheet,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);
    assert!((sheet_surface_area(&topo, sheet, AREA_DEFLECTION).unwrap() - 100.0).abs() < 1.0e-9);
    let bounds = sheet_bounding_box(&topo, sheet).unwrap();
    assert!((bounds.min.x() - 0.0).abs() < 1.0e-9);
    assert!((bounds.max.x() - 10.0).abs() < 1.0e-9);
    assert!((bounds.min.y() - 0.0).abs() < 1.0e-9);
    assert!((bounds.max.y() - 10.0).abs() < 1.0e-9);
}

#[test]
fn keep_outside_returns_the_exact_sheet_remainder() {
    let (topo, sheet) = trim_fixture(SheetTrimMode::KeepOutside);
    let report = remus_check::validate::validate_sheet_body(
        &topo,
        sheet,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);
    assert!((sheet_surface_area(&topo, sheet, AREA_DEFLECTION).unwrap() - 96.0).abs() < 1.0e-9);
}

#[test]
fn keep_inside_trim_is_deterministic() {
    let signature = || {
        let (topo, sheet) = trim_fixture(SheetTrimMode::KeepInside);
        let faces = topo.shell(sheet).unwrap().faces().len();
        let area = sheet_surface_area(&topo, sheet, AREA_DEFLECTION).unwrap();
        (faces, (area * 1.0e9).round() as i64)
    };
    assert_eq!(signature(), signature());
}

#[test]
fn coincident_sheet_refuses_transactionally() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[0];
    let sheet = make_sheet_body(&mut topo, &[face]).unwrap();
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let error =
        trim_sheet_by_solid(&mut topo, sheet, solid, SheetTrimMode::KeepInside).unwrap_err();
    assert!(
        error.to_string().contains("unsupported sheet trim"),
        "{error}"
    );
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        )
    );
}

#[test]
fn separately_built_coincident_sheet_refuses_transactionally() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let face = planar_face(&mut topo, 0.0, 10.0, 0.0);
    let sheet = make_sheet_body(&mut topo, &[face]).unwrap();
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let error =
        trim_sheet_by_solid(&mut topo, sheet, solid, SheetTrimMode::KeepInside).unwrap_err();
    assert!(
        error.to_string().contains("unsupported sheet trim"),
        "{error}"
    );
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        )
    );
}
