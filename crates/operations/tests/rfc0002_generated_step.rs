//! STEP export authority for curved edges minted by public operations.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_operations::loft::loft;
use remus_operations::offset_face::offset_face;
use remus_operations::offset_wire::{JoinType, offset_wire_with_join};
use remus_operations::primitives::make_cylinder;
use remus_operations::revolve::revolve;
use remus_operations::split::split;
use remus_operations::{measure, validate};
use remus_topology::Topology;
use remus_topology::compound::Compound;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn circle_face(topo: &mut Topology, center: Point3, normal: Vec3, radius: f64) -> FaceId {
    let circle = Circle3D::new(center, normal, radius).expect("circle profile");
    let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, TAU)));
    let edge = topo.add_edge(edge);
    let wire = topo.add_wire(
        Wire::new(vec![OrientedEdge::new(edge, true)], true).expect("circle profile wire"),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: normal.dot(Vec3::new(center.x(), center.y(), center.z())),
        },
    ))
}

fn rounded_rectangle_face(
    topo: &mut Topology,
    center_x: f64,
    half_width: f64,
    half_depth: f64,
    radius: f64,
    z: f64,
) -> FaceId {
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let centers = [
        Point3::new(center_x + half_width - radius, -half_depth + radius, z),
        Point3::new(center_x + half_width - radius, half_depth - radius, z),
        Point3::new(center_x - half_width + radius, half_depth - radius, z),
        Point3::new(center_x - half_width + radius, -half_depth + radius, z),
    ];
    let endpoints = [
        (
            Point3::new(center_x + half_width - radius, -half_depth, z),
            Point3::new(center_x + half_width, -half_depth + radius, z),
        ),
        (
            Point3::new(center_x + half_width, half_depth - radius, z),
            Point3::new(center_x + half_width - radius, half_depth, z),
        ),
        (
            Point3::new(center_x - half_width + radius, half_depth, z),
            Point3::new(center_x - half_width, half_depth - radius, z),
        ),
        (
            Point3::new(center_x - half_width, -half_depth + radius, z),
            Point3::new(center_x - half_width + radius, -half_depth, z),
        ),
    ];
    let vertices: Vec<_> = endpoints
        .iter()
        .flat_map(|(start, end)| [*start, *end])
        .map(|point| topo.add_vertex(Vertex::new(point, 1e-7)))
        .collect();
    let mut edges = vec![topo.add_edge(Edge::new(vertices[7], vertices[0], EdgeCurve::Line))];
    for index in 0..4 {
        let circle = Circle3D::new(centers[index], axis, radius).expect("corner circle");
        let start_parameter = circle.project(endpoints[index].0);
        let span = (circle.project(endpoints[index].1) - start_parameter).rem_euclid(TAU);
        let mut arc = Edge::with_tolerance(
            vertices[2 * index],
            vertices[2 * index + 1],
            EdgeCurve::Circle(circle),
            Some(1e-7),
        );
        arc.set_trim(Some((start_parameter, start_parameter + span)));
        edges.push(topo.add_edge(arc));
        if index < 3 {
            edges.push(topo.add_edge(Edge::new(
                vertices[2 * index + 1],
                vertices[2 * index + 2],
                EdgeCurve::Line,
            )));
        }
    }
    let wire = topo.add_wire(
        Wire::new(
            edges
                .into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .expect("rounded rectangle wire"),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane { normal: axis, d: z },
    ))
}

fn assert_step_round_trip(topo: &Topology, solid: SolidId, expected_volume: f64, label: &str) {
    let report = validate::validate_solid(topo, solid).expect("validate source");
    assert!(
        report.is_valid(),
        "{label} source invalid: {:?}",
        report.issues
    );

    for edge_id in remus_topology::explorer::solid_edges(topo, solid).expect("solid edges") {
        let edge = topo.edge(edge_id).expect("edge");
        if !matches!(edge.curve(), EdgeCurve::Line) {
            edge.strict_domain()
                .unwrap_or_else(|error| panic!("{label} edge {edge_id:?}: {error}"));
        }
    }

    let source_volume = measure::solid_volume(topo, solid, 0.001).expect("source volume");
    assert!(
        (source_volume - expected_volume).abs() <= expected_volume.abs() * 2e-4,
        "{label} source volume {source_volume}, expected {expected_volume}"
    );

    let step = remus_io::step::write_step(topo, &[solid]).expect("public STEP export");
    let mut imported = Topology::new();
    let solids = remus_io::step::read_step(&step, &mut imported).expect("STEP re-import");
    assert_eq!(solids.len(), 1, "{label} round trip solid count");
    let imported_volume =
        measure::solid_volume(&imported, solids[0], 0.001).expect("round-trip volume");
    assert!(
        (imported_volume - expected_volume).abs() <= expected_volume.abs() * 2e-4,
        "{label} round-trip volume {imported_volume}, expected {expected_volume}"
    );
}

#[test]
fn coaxial_circle_loft_result_round_trips_through_step() {
    let mut topo = Topology::new();
    let lower = circle_face(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        2.0,
    );
    let upper = circle_face(
        &mut topo,
        Point3::new(0.0, 0.0, 4.0),
        Vec3::new(0.0, 0.0, 1.0),
        3.0,
    );
    let solid = loft(&mut topo, &[lower, upper]).expect("coaxial circle loft");
    let expected = PI * 4.0 / 3.0 * (2.0_f64.mul_add(2.0, 2.0 * 3.0) + 3.0 * 3.0);
    assert_step_round_trip(&topo, solid, expected, "coaxial loft");
}

#[test]
fn noncoaxial_circle_profile_loft_round_trips_through_step() {
    let mut topo = Topology::new();
    let lower = rounded_rectangle_face(&mut topo, 0.0, 2.0, 1.5, 0.4, 0.0);
    let upper = rounded_rectangle_face(&mut topo, 0.4, 3.0, 2.25, 0.6, 4.0);
    let solid = loft(&mut topo, &[lower, upper]).expect("noncoaxial analytic-profile loft");
    let lower_area = 4.0 * 2.0 * 1.5 - (4.0 - PI) * 0.4_f64.powi(2);
    let upper_area = 4.0 * 3.0 * 2.25 - (4.0 - PI) * 0.6_f64.powi(2);
    let expected = 4.0 / 3.0 * (lower_area + (lower_area * upper_area).sqrt() + upper_area);
    assert_step_round_trip(&topo, solid, expected, "noncoaxial circle-profile loft");
}

#[test]
fn translated_curved_face_offset_then_extrude_round_trips_through_step() {
    let mut topo = Topology::new();
    let source = rounded_rectangle_face(&mut topo, 1.5, 2.0, 1.5, 0.4, 0.0);
    let shifted = offset_face(&mut topo, source, 3.0, 8).expect("planar curved face offset");
    let solid =
        remus_operations::extrude::extrude(&mut topo, shifted, Vec3::new(0.0, 0.0, 1.0), 2.0)
            .expect("extrude offset rounded rectangle");
    let area = 4.0 * 2.0 * 1.5 - (4.0 - PI) * 0.4_f64.powi(2);
    assert_step_round_trip(&topo, solid, area * 2.0, "offset/extrude rounded rectangle");
}

#[test]
fn exact_parallel_cylinder_compound_union_round_trips_through_step() {
    let mut topo = Topology::new();
    let radius = 2.0;
    let height = 5.0;
    let separation = 2.5;
    let first = make_cylinder(&mut topo, radius, height).expect("first cylinder");
    let second = make_cylinder(&mut topo, radius, height).expect("second cylinder");
    remus_operations::transform::transform_solid(
        &mut topo,
        second,
        &remus_math::mat::Mat4::translation(separation, 0.0, 0.0),
    )
    .expect("translate second cylinder");
    let compound = topo.add_compound(Compound::new(vec![first, second]));
    let fused = remus_operations::compound_ops::fuse_all(&mut topo, compound)
        .expect("exact parallel-cylinder union");
    let overlap = 2.0 * radius.powi(2) * (separation / (2.0 * radius)).acos()
        - 0.5 * separation * (4.0 * radius.powi(2) - separation.powi(2)).sqrt();
    let expected = (2.0 * PI * radius.powi(2) - overlap) * height;
    assert_step_round_trip(&topo, fused, expected, "parallel-cylinder compound union");
}

#[test]
fn arc_join_wire_offset_result_round_trips_through_step() {
    let mut topo = Topology::new();
    let source_wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .expect("square wire");
    let source_face = topo.add_face(Face::new(
        source_wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let distance = 0.2;
    let offset_wire = offset_wire_with_join(&mut topo, source_face, distance, JoinType::Arc)
        .expect("arc-join wire offset");
    let offset_face = topo.add_face(Face::new(
        offset_wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let height = 2.0;
    let solid = remus_operations::extrude::extrude(
        &mut topo,
        offset_face,
        Vec3::new(0.0, 0.0, 1.0),
        height,
    )
    .expect("extrude arc-joined offset");
    let expected_area = 1.0 + 4.0 * distance + PI * distance * distance;
    assert_step_round_trip(&topo, solid, expected_area * height, "arc-join wire offset");
}

#[test]
fn analytic_full_revolve_result_round_trips_through_step() {
    let mut topo = Topology::new();
    let wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 4.0),
            Point3::new(2.0, 0.0, 4.0),
        ],
        1e-7,
    )
    .expect("annulus profile");
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        },
    ));
    let solid = revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        TAU,
    )
    .expect("analytic annulus revolve");
    assert_step_round_trip(
        &topo,
        solid,
        PI * (3.0_f64.powi(2) - 2.0_f64.powi(2)) * 4.0,
        "full revolve",
    );
}

#[test]
fn segmented_partial_revolve_result_round_trips_through_step() {
    let mut topo = Topology::new();
    let wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 4.0),
            Point3::new(2.0, 0.0, 4.0),
        ],
        1e-7,
    )
    .expect("partial annulus profile");
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 1.0, 0.0),
            d: 0.0,
        },
    ));
    let solid = revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        PI,
    )
    .expect("segmented partial annulus revolve");
    assert_step_round_trip(&topo, solid, 10.0 * PI, "segmented partial revolve");
}

#[test]
fn partial_torus_revolve_result_round_trips_through_step() {
    let mut topo = Topology::new();
    let major = 6.0;
    let minor = 2.0;
    let angle = 2.0 * PI / 3.0;
    let face = circle_face(
        &mut topo,
        Point3::new(major, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        minor,
    );
    let solid = revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        angle,
    )
    .expect("partial torus revolve");
    assert_step_round_trip(
        &topo,
        solid,
        PI * major * minor * minor * angle,
        "partial torus",
    );
}

#[test]
fn square_cylinder_split_results_round_trip_through_step() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 2.0, 6.0).expect("cylinder");
    let result = split(
        &mut topo,
        cylinder,
        Point3::new(0.0, 0.0, 3.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .expect("square cylinder split");
    let expected_half = PI * 2.0_f64.powi(2) * 3.0;
    assert_step_round_trip(&topo, result.positive, expected_half, "positive split half");
    assert_step_round_trip(&topo, result.negative, expected_half, "negative split half");
}
