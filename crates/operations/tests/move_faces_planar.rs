//! Phase 4.1 regression coverage for topology-preserving planar face moves.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::analytic_intersection::{
    AnalyticSurface, ExactIntersectionCurve, exact_plane_analytic,
};
use remus_math::mat::Mat4;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_offset::OffsetError;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{face_vertices, solid_entity_counts, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const DEFLECTION: f64 = 0.002;

fn plane_coordinate(topo: &Topology, face: FaceId, direction: Vec3) -> f64 {
    let vertices = face_vertices(topo, face).expect("face vertices");
    let point = topo.vertex(vertices[0]).expect("face vertex").point();
    (point - Point3::new(0.0, 0.0, 0.0)).dot(direction)
}

fn outward_faces(topo: &Topology, solid: SolidId, direction: Vec3) -> Vec<FaceId> {
    let mut candidates: Vec<_> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .expect("face")
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(direction) > 1.0 - 1e-9)
        })
        .collect();
    let max = candidates
        .iter()
        .map(|&face| plane_coordinate(topo, face, direction))
        .fold(f64::NEG_INFINITY, f64::max);
    candidates.retain(|&face| (plane_coordinate(topo, face, direction) - max).abs() < 1e-7);
    candidates.sort_by_key(|face| face.index());
    candidates
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64) {
    let actual = solid_volume(topo, solid, DEFLECTION).expect("solid volume");
    let tolerance = expected.abs().mul_add(2e-4, 1e-6);
    assert!(
        (actual - expected).abs() <= tolerance,
        "volume {actual} != {expected} within {tolerance}"
    );
}

fn positional_edge_health(positions: &[Point3], indices: &[u32]) -> (usize, usize) {
    let quantization = 1e6;
    let mut canonical = HashMap::new();
    let mut remap = vec![0_u32; positions.len()];
    for (index, point) in positions.iter().enumerate() {
        let key = (
            (point.x() * quantization).round() as i64,
            (point.y() * quantization).round() as i64,
            (point.z() * quantization).round() as i64,
        );
        let next = canonical.len() as u32;
        remap[index] = *canonical.entry(key).or_insert(next);
    }
    let mut uses = HashMap::new();
    for triangle in indices.chunks_exact(3) {
        let vertices = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for &(a, b) in &[
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ] {
            let key = if a < b { (a, b) } else { (b, a) };
            *uses.entry(key).or_insert(0_usize) += 1;
        }
    }
    let boundary = uses.values().filter(|&&count| count == 1).count();
    let non_manifold = uses.values().filter(|&&count| count > 2).count();
    (boundary, non_manifold)
}

fn assert_verified(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).expect("strict solid validation");
    assert!(report.is_valid(), "strict validation: {:?}", report.issues);

    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).expect("solid tessellation");
    assert!(is_watertight(&mesh), "index-based mesh is not watertight");
    let (boundary, non_manifold) = positional_edge_health(&mesh.positions, &mesh.indices);
    assert_eq!(boundary, 0, "position-welded mesh has boundary edges");
    assert_eq!(
        non_manifold, 0,
        "position-welded mesh has non-manifold edges"
    );
}

fn assert_classification(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    expected: PointClassification,
) {
    let actual = classify_point(topo, solid, point, &ClassifyOptions::default())
        .expect("ray-cast classification");
    assert_eq!(actual, expected, "classification at {point:?}");
}

fn cylinder_at(topo: &mut Topology, radius: f64, height: f64, x: f64, y: f64) -> SolidId {
    let cylinder = make_cylinder(topo, radius, height).expect("cylinder");
    transform_solid(topo, cylinder, &Mat4::translation(x, y, 0.0)).expect("place cylinder");
    cylinder
}

fn drilled_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 40.0, 40.0, 10.0).expect("plate");
    let drill = cylinder_at(topo, 3.0, 10.0, 20.0, 20.0);
    boolean(topo, BooleanOp::Cut, plate, drill).expect("drill through-hole")
}

fn l_prism(topo: &mut Topology) -> SolidId {
    let profile = remus_topology::builder::make_planar_face(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 2.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(2.0, 8.0, 0.0),
            Point3::new(0.0, 8.0, 0.0),
        ],
        1e-7,
    )
    .expect("L profile");
    extrude(topo, profile, Vec3::new(0.0, 0.0, 1.0), 6.0).expect("L prism")
}

fn sloped_prism(topo: &mut Topology) -> SolidId {
    let profile = remus_topology::builder::make_planar_face(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(8.0, 0.0, 6.0),
            Point3::new(0.0, 0.0, 6.0),
        ],
        1e-7,
    )
    .expect("trapezoid profile");
    extrude(topo, profile, Vec3::new(0.0, 1.0, 0.0), 4.0).expect("sloped prism")
}

fn disconnected_boxes(topo: &mut Topology) -> SolidId {
    let first = make_box(topo, 10.0, 10.0, 10.0).expect("first box");
    let second = make_box(topo, 10.0, 10.0, 10.0).expect("second box");
    transform_solid(topo, second, &Mat4::translation(20.0, 0.0, 0.0)).expect("place second box");

    let mut faces = solid_faces(topo, first).expect("first faces");
    faces.extend(solid_faces(topo, second).expect("second faces"));
    let shell = topo.add_shell(Shell::new(faces).expect("disconnected shell"));
    topo.add_solid(Solid::new(shell, Vec::new()))
}

fn stacked_boxes(topo: &mut Topology) -> (SolidId, FaceId) {
    let lower = make_box(topo, 10.0, 10.0, 10.0).expect("lower box");
    let upper = make_box(topo, 10.0, 10.0, 10.0).expect("upper box");
    transform_solid(topo, upper, &Mat4::translation(0.0, 0.0, 20.0)).expect("place upper box");

    let lower_top = outward_faces(topo, lower, Vec3::new(0.0, 0.0, 1.0))[0];
    let mut faces = solid_faces(topo, lower).expect("lower faces");
    faces.extend(solid_faces(topo, upper).expect("upper faces"));
    let shell = topo.add_shell(Shell::new(faces).expect("stacked shell"));
    (topo.add_solid(Solid::new(shell, Vec::new())), lower_top)
}

fn obliquely_capped_cylinder(topo: &mut Topology) -> (SolidId, FaceId, f64) {
    let radius = 3.0;
    let top_d = 10.0;
    let normal = Vec3::new(0.0, 1.0, 1.0)
        .normalize()
        .expect("oblique plane normal");
    let cylinder =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), radius)
            .expect("cylinder surface");
    let ellipse_at = |d| {
        let curves = exact_plane_analytic(AnalyticSurface::Cylinder(&cylinder), normal, d)
            .expect("plane-cylinder intersection");
        let [ExactIntersectionCurve::Ellipse(ellipse)] = curves.as_slice() else {
            panic!("oblique plane must cut the cylinder in one ellipse");
        };
        ellipse.clone()
    };
    let bottom_ellipse = ellipse_at(0.0);
    let top_ellipse = ellipse_at(top_d);
    let seam_at = |ellipse: &remus_math::curves::Ellipse3D, z| {
        let candidate = Point3::new(radius, 0.0, z);
        ellipse.evaluate(ellipse.project(candidate))
    };
    let bottom_point = seam_at(&bottom_ellipse, 0.0);
    let top_point = seam_at(&top_ellipse, top_d / normal.z());
    let bottom_vertex = topo.add_vertex(Vertex::new(bottom_point, 1e-7));
    let top_vertex = topo.add_vertex(Vertex::new(top_point, 1e-7));
    let add_ellipse_rim =
        |topo: &mut Topology, vertex, point: Point3, ellipse: remus_math::curves::Ellipse3D| {
            let seam = ellipse.project(point);
            let mut edge =
                Edge::with_tolerance(vertex, vertex, EdgeCurve::Ellipse(ellipse), Some(1e-7));
            edge.set_trim(Some((seam, seam + TAU)));
            edge.strict_domain().expect("certified ellipse rim");
            topo.add_edge(edge)
        };
    let bottom_edge = add_ellipse_rim(topo, bottom_vertex, bottom_point, bottom_ellipse);
    let top_edge = add_ellipse_rim(topo, top_vertex, top_point, top_ellipse);
    let seam = topo.add_edge(Edge::new(bottom_vertex, top_vertex, EdgeCurve::Line));

    let lateral_wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(bottom_edge, true),
                OrientedEdge::new(seam, true),
                OrientedEdge::new(top_edge, false),
                OrientedEdge::new(seam, false),
            ],
            true,
        )
        .expect("lateral wire"),
    );
    let lateral = topo.add_face(Face::new(
        lateral_wire,
        Vec::new(),
        FaceSurface::Cylinder(cylinder),
    ));
    let bottom_wire = topo.add_wire(
        Wire::new(vec![OrientedEdge::new(bottom_edge, false)], true).expect("bottom wire"),
    );
    let bottom = topo.add_face(Face::new(
        bottom_wire,
        Vec::new(),
        FaceSurface::Plane {
            normal: -normal,
            d: 0.0,
        },
    ));
    let top_wire =
        topo.add_wire(Wire::new(vec![OrientedEdge::new(top_edge, true)], true).expect("top wire"));
    let top = topo.add_face(Face::new(
        top_wire,
        Vec::new(),
        FaceSurface::Plane { normal, d: top_d },
    ));
    let shell = topo.add_shell(Shell::new(vec![lateral, bottom, top]).expect("oblique shell"));
    (
        topo.add_solid(Solid::new(shell, Vec::new())),
        top,
        normal.z(),
    )
}

#[test]
fn box_face_moves_outward_and_inward_without_topology_change() {
    for (distance, expected_height) in [(5.0, 15.0), (-3.0, 7.0)] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).expect("box");
        let source_counts = solid_entity_counts(&topo, source).expect("source counts");
        let selected = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0));

        let result = move_faces(&mut topo, source, &selected, distance).expect("move box face");

        assert_eq!(
            solid_entity_counts(&topo, result).expect("result counts"),
            source_counts
        );
        assert_volume(&topo, source, 1_000.0);
        assert_volume(&topo, result, 100.0 * expected_height);
        assert_verified(&topo, result);
        assert_classification(
            &topo,
            result,
            Point3::new(5.0, 5.0, expected_height - 0.5),
            PointClassification::Inside,
        );
        assert_classification(
            &topo,
            result,
            Point3::new(5.0, 5.0, expected_height + 0.5),
            PointClassification::Outside,
        );
    }
}

#[test]
fn l_bracket_cap_move_extends_all_six_planar_neighbors() {
    let mut topo = Topology::new();
    let source = l_prism(&mut topo);
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let selected = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(selected.len(), 1, "L prism has one moved cap");

    let result = move_faces(&mut topo, source, &selected, 2.0).expect("move L cap");

    assert_eq!(
        solid_entity_counts(&topo, result).expect("result counts"),
        source_counts
    );
    assert_volume(&topo, source, 32.0 * 6.0);
    assert_volume(&topo, result, 32.0 * 8.0);
    assert_verified(&topo, result);
    assert_classification(
        &topo,
        result,
        Point3::new(1.0, 7.0, 7.5),
        PointClassification::Inside,
    );
    assert_classification(
        &topo,
        result,
        Point3::new(5.0, 5.0, 7.5),
        PointClassification::Outside,
    );
}

#[test]
fn planar_move_retrims_a_sloped_neighbor() {
    let mut topo = Topology::new();
    let source = sloped_prism(&mut topo);
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let selected = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(selected.len(), 1, "sloped prism has one top face");

    let result = move_faces(&mut topo, source, &selected, 2.0).expect("move sloped cap");

    assert_eq!(
        solid_entity_counts(&topo, result).expect("result counts"),
        source_counts
    );
    assert_volume(&topo, source, 216.0);
    assert_volume(&topo, result, 832.0 / 3.0);
    assert_verified(&topo, result);
    assert_classification(
        &topo,
        result,
        Point3::new(7.0, 2.0, 7.5),
        PointClassification::Inside,
    );
    assert_classification(
        &topo,
        result,
        Point3::new(8.0, 2.0, 7.5),
        PointClassification::Outside,
    );
}

#[test]
fn coplanar_face_group_moves_rigidly() {
    let mut topo = Topology::new();
    let source = disconnected_boxes(&mut topo);
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let selected = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(selected.len(), 2, "one top face per component");

    let result = move_faces(&mut topo, source, &selected, 5.0).expect("move face group");

    assert_eq!(
        solid_entity_counts(&topo, result).expect("result counts"),
        source_counts
    );
    assert_volume(&topo, result, 3_000.0);
    assert_verified(&topo, result);
    for x in [5.0, 25.0] {
        assert_classification(
            &topo,
            result,
            Point3::new(x, 5.0, 14.5),
            PointClassification::Inside,
        );
    }
}

#[test]
fn plate_cap_move_lengthens_the_through_hole_cylinder() {
    let mut topo = Topology::new();
    let source = drilled_plate(&mut topo);
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let selected = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0));
    assert_eq!(selected.len(), 1, "plate has one top face");
    assert_eq!(
        topo.face(selected[0])
            .expect("top face")
            .inner_wires()
            .len(),
        1,
        "top face carries the bore loop"
    );

    let result = move_faces(&mut topo, source, &selected, 5.0).expect("move drilled cap");

    assert_eq!(
        solid_entity_counts(&topo, result).expect("result counts"),
        source_counts
    );
    assert_volume(&topo, source, (40.0f64.mul_add(40.0, -(PI * 9.0))) * 10.0);
    assert_volume(&topo, result, (40.0f64.mul_add(40.0, -(PI * 9.0))) * 15.0);
    assert_verified(&topo, result);
    assert_classification(
        &topo,
        result,
        Point3::new(20.0, 20.0, 12.0),
        PointClassification::Outside,
    );
    assert_classification(
        &topo,
        result,
        Point3::new(1.0, 1.0, 12.0),
        PointClassification::Inside,
    );

    let cylinders: Vec<_> = solid_faces(&topo, result)
        .expect("result faces")
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).expect("result face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .collect();
    assert_eq!(cylinders.len(), 1, "the bore remains one analytic cylinder");
    let z_values: Vec<_> = face_vertices(&topo, cylinders[0])
        .expect("cylinder vertices")
        .into_iter()
        .map(|vertex| topo.vertex(vertex).expect("cylinder vertex").point().z())
        .collect();
    let min_z = z_values.iter().copied().fold(f64::INFINITY, f64::min);
    let max_z = z_values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    assert!((min_z - 0.0).abs() < 1e-7, "bore starts at z={min_z}");
    assert!((max_z - 15.0).abs() < 1e-7, "bore ends at z={max_z}");
}

#[test]
fn oblique_cap_move_preserves_the_exact_elliptic_cylinder_boundary() {
    let mut topo = Topology::new();
    let (source, top, normal_z) = obliquely_capped_cylinder(&mut topo);
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let before = solid_volume(&topo, source, DEFLECTION).expect("source volume");
    let moved_area = face_area(&topo, top, DEFLECTION).expect("oblique cap area");
    let analytic_area = PI * 9.0 / normal_z;
    assert!(
        (moved_area - analytic_area).abs() <= analytic_area * 2e-4,
        "oblique cap area {moved_area} != {analytic_area}"
    );
    assert_verified(&topo, source);

    let result = move_faces(&mut topo, source, &[top], 2.0).expect("move oblique cap");

    assert_eq!(
        solid_entity_counts(&topo, result).expect("result counts"),
        source_counts
    );
    let result_volume = solid_volume(&topo, result, DEFLECTION).expect("result volume");
    let expected_change = moved_area * 2.0;
    assert!(
        ((result_volume - before) - expected_change).abs() <= expected_change * 0.003,
        "oblique volume change {} != {expected_change}",
        result_volume - before
    );
    assert_verified(&topo, result);
    let ellipse_count = solid_faces(&topo, result)
        .expect("result faces")
        .into_iter()
        .filter(|&face| {
            let face = topo.face(face).expect("result face");
            topo.wire(face.outer_wire())
                .expect("result wire")
                .edges()
                .iter()
                .any(|edge| {
                    matches!(
                        topo.edge(edge.edge()).expect("result edge").curve(),
                        EdgeCurve::Ellipse(_)
                    )
                })
        })
        .count();
    assert_eq!(ellipse_count, 3, "all three faces share elliptic rims");
}

#[test]
fn refused_move_is_typed_and_transactional() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).expect("box");
    let source_counts = solid_entity_counts(&topo, source).expect("source counts");
    let selected = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0));
    let live_counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );

    let lower_error = remus_offset::move_faces(&mut topo, source, &selected, -10.0)
        .expect_err("lower-level collapsed face");
    assert!(
        matches!(lower_error, OffsetError::TopologyChange { .. }),
        "unexpected lower-level refusal: {lower_error}"
    );
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        ),
        live_counts,
        "lower-level failure must retire every temporary entity"
    );

    let error = move_faces(&mut topo, source, &selected, -10.0).expect_err("collapsed face");
    assert!(
        matches!(
            error,
            OperationsError::Offset(OffsetError::TopologyChange { .. })
        ),
        "unexpected refusal: {error}"
    );
    assert_eq!(
        solid_entity_counts(&topo, source).expect("source after refusal"),
        source_counts
    );
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        ),
        live_counts,
        "failed move must retire every temporary entity"
    );
    assert_volume(&topo, source, 1_000.0);
    assert_verified(&topo, source);
}

#[test]
fn move_refuses_contact_with_nonadjacent_geometry() {
    let mut topo = Topology::new();
    let (source, lower_top) = stacked_boxes(&mut topo);
    let live_counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );

    let error = move_faces(&mut topo, source, &[lower_top], 10.0)
        .expect_err("move must stop at nonadjacent face");
    assert!(
        matches!(
            error,
            OperationsError::Offset(OffsetError::TopologyChange { .. })
        ),
        "unexpected refusal: {error}"
    );
    assert_eq!(
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        ),
        live_counts,
        "collision refusal must not allocate topology"
    );
}

#[test]
fn invalid_selections_return_specific_move_face_errors() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).expect("box");
    let top = outward_faces(&topo, source, Vec3::new(0.0, 0.0, 1.0))[0];
    let bottom = outward_faces(&topo, source, Vec3::new(0.0, 0.0, -1.0))[0];

    let mismatch = move_faces(&mut topo, source, &[top, bottom], 1.0)
        .expect_err("opposed faces are not one rigid move group");
    assert!(matches!(
        mismatch,
        OperationsError::Offset(OffsetError::MoveGroupMismatch { .. })
    ));

    let other = make_box(&mut topo, 1.0, 1.0, 1.0).expect("other box");
    let outsider = outward_faces(&topo, other, Vec3::new(0.0, 0.0, 1.0))[0];
    let membership =
        move_faces(&mut topo, source, &[outsider], 1.0).expect_err("foreign face must be refused");
    assert!(matches!(
        membership,
        OperationsError::Offset(OffsetError::FaceNotInSolid { .. })
    ));

    let cylinder = make_cylinder(&mut topo, 2.0, 5.0).expect("cylinder");
    let curved = solid_faces(&topo, cylinder)
        .expect("cylinder faces")
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).expect("cylinder face").surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .expect("curved face");
    let unsupported = move_faces(&mut topo, cylinder, &[curved], 1.0)
        .expect_err("outward-facing cylinders are not bore moves");
    assert!(matches!(
        unsupported,
        OperationsError::Offset(OffsetError::UnsupportedMoveFace { .. })
    ));
}
