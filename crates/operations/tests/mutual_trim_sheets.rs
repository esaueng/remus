//! Issue 4.4b qualification for mutual planar sheet trimming and sew.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{SheetSide, mutual_trim_sheets, trim_sheet_by_sheet};
use remus_operations::measure::{sheet_bounding_box, sheet_surface_area, solid_volume};
use remus_operations::sew::{make_sheet_body, sew_faces};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::ShellId;

const TOL: f64 = 1.0e-7;

fn planar_sheet(
    topo: &mut Topology,
    origin: Point3,
    u: Vec3,
    v: Vec3,
    normal: Vec3,
    lo: f64,
    hi: f64,
) -> ShellId {
    assert!(u.cross(v).dot(normal) > 0.999_999);
    let points = [
        origin + u * lo + v * lo,
        origin + u * hi + v * lo,
        origin + u * hi + v * hi,
        origin + u * lo + v * hi,
    ];
    let wire = make_polygon_wire(topo, &points, TOL).unwrap();
    let d = normal.dot(Vec3::new(origin.x(), origin.y(), origin.z()));
    let face = topo.add_face(Face::new(
        wire,
        Vec::new(),
        FaceSurface::Plane { normal, d },
    ));
    make_sheet_body(topo, &[face]).unwrap()
}

#[test]
fn perpendicular_sheets_trim_each_other_by_effective_normal_side() {
    let mut topo = Topology::new();
    let horizontal = planar_sheet(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        -2.0,
        2.0,
    );
    let vertical = planar_sheet(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        -2.0,
        2.0,
    );

    let result = mutual_trim_sheets(
        &mut topo,
        horizontal,
        vertical,
        SheetSide::Positive,
        SheetSide::Negative,
    )
    .unwrap();
    let area_a = sheet_surface_area(&topo, result.sheet_a, 1.0e-3).unwrap();
    let area_b = sheet_surface_area(&topo, result.sheet_b, 1.0e-3).unwrap();
    assert!((area_a - 8.0).abs() < 1.0e-9);
    assert!((area_b - 8.0).abs() < 1.0e-9);
    let bounds_a = sheet_bounding_box(&topo, result.sheet_a).unwrap();
    let bounds_b = sheet_bounding_box(&topo, result.sheet_b).unwrap();
    assert!((bounds_a.min.x() - 0.0).abs() < 1.0e-9);
    assert!((bounds_b.max.z() - 0.0).abs() < 1.0e-9);
}

#[test]
fn reversed_tool_face_reverses_the_selected_positive_side() {
    let mut topo = Topology::new();
    let horizontal = planar_sheet(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        -2.0,
        2.0,
    );
    let vertical = planar_sheet(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        -2.0,
        2.0,
    );
    let tool_face = topo.shell(vertical).unwrap().faces()[0];
    topo.face_mut(tool_face).unwrap().set_reversed(true);

    let result = remus_operations::boolean::trim_sheet_by_sheet(
        &mut topo,
        horizontal,
        vertical,
        SheetSide::Positive,
    )
    .unwrap();
    let bounds = sheet_bounding_box(&topo, result).unwrap();
    assert!((bounds.max.x() - 0.0).abs() < 1.0e-9);
    assert!((bounds.min.x() + 2.0).abs() < 1.0e-9);
}

#[test]
fn identical_mutual_operands_refuse_without_mutation() {
    let mut topo = Topology::new();
    let sheet = planar_sheet(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        -2.0,
        2.0,
    );
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let error = mutual_trim_sheets(
        &mut topo,
        sheet,
        sheet,
        SheetSide::Positive,
        SheetSide::Negative,
    )
    .unwrap_err();
    assert!(error.to_string().contains("unsupported sheet trim"));
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

fn trimmed_box_signature() -> (usize, i64) {
    let mut topo = Topology::new();
    let sheets = [
        planar_sheet(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
            -10.0,
            20.0,
        ),
        planar_sheet(
            &mut topo,
            Point3::new(10.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            -10.0,
            20.0,
        ),
        planar_sheet(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -1.0, 0.0),
            -10.0,
            20.0,
        ),
        planar_sheet(
            &mut topo,
            Point3::new(0.0, 10.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            -10.0,
            20.0,
        ),
        planar_sheet(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
            -10.0,
            20.0,
        ),
        planar_sheet(
            &mut topo,
            Point3::new(0.0, 0.0, 10.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            -10.0,
            20.0,
        ),
    ];
    let adjacent = [
        [2, 3, 4, 5],
        [2, 3, 4, 5],
        [0, 1, 4, 5],
        [0, 1, 4, 5],
        [0, 1, 2, 3],
        [0, 1, 2, 3],
    ];

    let mut faces = Vec::new();
    for (target, tools) in sheets.iter().copied().zip(adjacent) {
        let mut trimmed = target;
        for tool in tools {
            trimmed =
                trim_sheet_by_sheet(&mut topo, trimmed, sheets[tool], SheetSide::Negative).unwrap();
        }
        let [face] = topo.shell(trimmed).unwrap().faces() else {
            panic!("trimmed box side must be one face");
        };
        assert!((sheet_surface_area(&topo, trimmed, 1.0e-3).unwrap() - 100.0).abs() < 1.0e-9);
        faces.push(*face);
    }

    let solid = sew_faces(&mut topo, &faces, 1.0e-6).unwrap();
    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);
    let volume = solid_volume(&topo, solid, 1.0e-3).unwrap();
    let reference = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let reference_volume = solid_volume(&topo, reference, 1.0e-3).unwrap();
    assert!((volume - reference_volume).abs() < 1.0e-9);
    let face_count = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap()
        .faces()
        .len();
    (face_count, (volume * 1.0e9).round() as i64)
}

#[test]
fn mutually_trimmed_sheets_sew_to_the_box_volume_deterministically() {
    let first = trimmed_box_signature();
    let second = trimmed_box_signature();
    assert_eq!(first.0, 6);
    assert_eq!(first.1, 1_000_000_000_000);
    assert_eq!(first, second);
}
