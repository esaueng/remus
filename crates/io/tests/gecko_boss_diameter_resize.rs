//! Exact diameter edits of a partial-turn boss wall fused into a block.
//!
//! Gentle-Gecko (sharp) is a 30 x 18 x 24 block with an R6 boss on the
//! block's x = 0, y = 0 corner edge, axis +Z, height 36. Below the block top
//! the wall is a three-quarter turn whose sides lie on the block planes x = 0
//! and y = 0; above it the wall is a full seamed turn. The material outside
//! the block is a three-quarter disk prism over 24 plus a full disk prism over
//! 12, so the volume is `12960 + 30 pi r^2` while `r < 18` keeps the wall
//! inside the block's x = 0 face. The synthetic fixture is the same family
//! built natively: a half-turn wall on the middle of a block face.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_io::arena_io::serialize_solid;
use remus_math::vec::{Point3, Vec3};
use remus_operations::{
    OperationsError, measure, push_pull::resize_cylindrical_face, validate::validate_solid,
};
use remus_topology::{
    Topology,
    adjacency::AdjacencyIndex,
    edge::EdgeCurve,
    explorer::{solid_edges, solid_entity_counts, solid_faces},
    face::{FaceId, FaceSurface},
    solid::SolidId,
};

const GECKO: &str = include_str!("data/Gentle-Gecko.step");

fn gecko_volume(radius: f64) -> f64 {
    30.0f64.mul_add(18.0 * 24.0, 30.0 * PI * radius * radius)
}

/// Box 40 x 25 x 20 plus an R boss centred on its y = 0 face at x = 20,
/// height 32: a half-disk prism over 20 and a full disk prism over 12.
fn half_boss_volume(radius: f64) -> f64 {
    40.0f64.mul_add(25.0 * 20.0, 22.0 * PI * radius * radius)
}

fn wall(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .collect();
    assert_eq!(walls.len(), 1);
    walls[0]
}

fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize) {
    let mut planes = 0;
    let mut cylinders = 0;
    let mut other = 0;
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => planes += 1,
            FaceSurface::Cylinder(_) => cylinders += 1,
            _ => other += 1,
        }
    }
    (planes, cylinders, other)
}

fn classify(topo: &Topology, solid: SolidId, point: Point3) -> PointClassification {
    classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap()
}

/// Validator findings with entity ids and measured values stripped.
type IssueKeys = (Vec<String>, Vec<String>);

fn issue_keys(topo: &Topology, solid: SolidId) -> IssueKeys {
    let report = validate_solid(topo, solid).unwrap();
    assert_eq!(report.error_count(), 0, "{report:?}");
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{:?}", strict.issues);
    let strip = |text: String| {
        text.chars()
            .filter(|c| !c.is_ascii_digit())
            .collect::<String>()
    };
    let mut operations: Vec<_> = report
        .issues
        .iter()
        .map(|issue| strip(format!("{:?} {}", issue.severity, issue.description)))
        .collect();
    let mut check: Vec<_> = strict
        .issues
        .iter()
        .map(|issue| format!("{:?} {:?}", issue.severity, issue.check))
        .collect();
    operations.sort();
    check.sort();
    (operations, check)
}

fn is_topology_change(error: &OperationsError) -> bool {
    matches!(error, OperationsError::Offset(inner) if format!("{inner:?}").starts_with("TopologyChange"))
}

/// The full solid-verification bar for a resized boss on axis `(cx, cy)`.
///
/// `outside_angles` are polar angles where the wall is exposed over the
/// whole height; probes just inside and just outside the new radius there
/// must classify accordingly.
#[allow(clippy::too_many_arguments)]
fn verify(
    topo: &Topology,
    solid: SolidId,
    source_counts: (usize, usize, usize),
    source_issues: &IssueKeys,
    axis: (f64, f64),
    radius: f64,
    height: f64,
    expected_volume: f64,
    outside_angles: &[f64],
) {
    // Census: same entities, still all planes plus one exact cylinder.
    assert_eq!(solid_entity_counts(topo, solid).unwrap(), source_counts);
    let (planes, cylinders, other) = census(topo, solid);
    assert_eq!((cylinders, other), (1, 0));
    assert_eq!(planes + 1, source_counts.0);
    let face = wall(topo, solid);
    let FaceSurface::Cylinder(cylinder) = topo.face(face).unwrap().surface() else {
        unreachable!()
    };
    assert!((cylinder.radius() - radius).abs() < 1e-9);
    assert!((cylinder.axis() - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12);
    let offset = cylinder.origin() - Point3::new(axis.0, axis.1, 0.0);
    assert!(offset.x().hypot(offset.y()) < 1e-9);

    // Every edge on the wall is exact: axial lines on the new radius and
    // coaxial circles of the new radius; every circle in the body is one.
    let wall_edges: Vec<_> = topo
        .wire(topo.face(face).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(remus_topology::OrientedEdge::edge)
        .collect();
    for edge_id in solid_edges(topo, solid).unwrap() {
        let edge = topo.edge(edge_id).unwrap();
        let ends = [edge.start(), edge.end()].map(|v| topo.vertex(v).unwrap().point());
        match edge.curve() {
            EdgeCurve::Line => {
                if wall_edges.contains(&edge_id) {
                    for point in ends {
                        let radial = (point.x() - axis.0).hypot(point.y() - axis.1);
                        assert!((radial - radius).abs() < 1e-9, "{point:?}");
                    }
                    assert!((ends[1] - ends[0]).cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9);
                }
            }
            EdgeCurve::Circle(circle) => {
                assert!(wall_edges.contains(&edge_id));
                assert!((circle.radius() - radius).abs() < 1e-9);
                let centre = circle.center();
                assert!((centre.x() - axis.0).hypot(centre.y() - axis.1) < 1e-9);
                for point in ends {
                    let radial = (point.x() - axis.0).hypot(point.y() - axis.1);
                    assert!((radial - radius).abs() < 1e-9);
                    assert!((point.z() - centre.z()).abs() < 1e-9);
                }
            }
            other => panic!("unexpected {} edge", other.type_tag()),
        }
    }

    // B-Rep closed and manifold, both validators clean.
    // The sources already carry the validators' heuristic warnings for a
    // seamed stepped wall; the edit must add none and introduce no error.
    assert_eq!(&issue_keys(topo, solid), source_issues);
    let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
    assert!(adjacency.boundary_edges().is_empty());
    assert!(adjacency.non_manifold_edges().is_empty());

    // Closed form, convergence at two deflections finer than the clamp, and
    // an independent mesh volume from the welded watertight tessellation.
    let bbox = measure::solid_bounding_box(topo, solid).unwrap();
    let diagonal = (bbox.max - bbox.min).length();
    let deflections = [diagonal * 4e-5, diagonal * 2e-5];
    let mut volumes = Vec::new();
    for deflection in deflections {
        let volume = measure::solid_volume(topo, solid, deflection).unwrap();
        assert!(
            (volume - expected_volume).abs() <= expected_volume * 1e-4,
            "volume {volume} vs closed form {expected_volume}"
        );
        let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        assert_eq!((quality.boundary_edges, quality.non_manifold_edges), (0, 0));
        let origin = bbox.min;
        let mesh_volume: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|ix| {
                let a = mesh.positions[ix[0] as usize] - origin;
                let b = mesh.positions[ix[1] as usize] - origin;
                let c = mesh.positions[ix[2] as usize] - origin;
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        assert!(
            (mesh_volume - volume).abs() <= volume * 1e-4,
            "mesh volume {mesh_volume} vs API volume {volume}"
        );
        volumes.push(volume);
    }
    assert!((volumes[0] - volumes[1]).abs() <= volumes[0] * 1e-4);

    // Material ends exactly at the new radius on the exposed wall, at both
    // heights; the cap is at the original height.
    for &angle in outside_angles {
        for z in [height * 0.3, height * 0.9] {
            let at = |r: f64| {
                Point3::new(
                    r.mul_add(angle.cos(), axis.0),
                    r.mul_add(angle.sin(), axis.1),
                    z,
                )
            };
            assert_eq!(
                classify(topo, solid, at(radius - 0.05)),
                PointClassification::Inside
            );
            assert_eq!(
                classify(topo, solid, at(radius + 0.05)),
                PointClassification::Outside
            );
        }
    }
    let top = |dz: f64| Point3::new(axis.0, axis.1, height + dz);
    assert_eq!(
        classify(topo, solid, top(-0.05)),
        PointClassification::Inside
    );
    assert_eq!(
        classify(topo, solid, top(0.05)),
        PointClassification::Outside
    );

    // The exact result round-trips through STEP with its volume.
    let step = remus_io::step::writer::write_step(topo, &[solid]).unwrap();
    let mut round_topo = Topology::new();
    let round = remus_io::step::reader::read_step(&step, &mut round_topo).unwrap();
    assert_eq!(round.len(), 1);
    assert_eq!(census(&round_topo, round[0]), census(topo, solid));
    let round_volume = measure::solid_volume(&round_topo, round[0], deflections[0]).unwrap();
    assert!((round_volume - volumes[0]).abs() <= volumes[0] * 1e-6);
}

fn gecko() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(GECKO, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// Exposed-wall angles of the Gecko boss below the block top: the three
/// quarters outside the block's first quadrant.
const GECKO_EXPOSED: [f64; 3] = [0.75 * PI, PI, 1.5 * PI];

#[test]
fn gecko_boss_diameter_sequence_resizes_exactly() {
    let (mut topo, source) = gecko();
    let counts = solid_entity_counts(&topo, source).unwrap();
    assert_eq!(census(&topo, source), (7, 1, 0));
    let issues = issue_keys(&topo, source);
    assert!(
        (measure::solid_volume(&topo, source, 1e-3).unwrap() - gecko_volume(6.0)).abs()
            < gecko_volume(6.0) * 1e-6
    );
    let mut current = source;
    // Ø12 -> 14.6 -> 17 -> 21.1 -> 14.7: outward, outward, outward, inward.
    for diameter in [14.6, 17.0, 21.1, 14.7] {
        let radius = diameter / 2.0;
        let before = serialize_solid(&topo, current).unwrap();
        let face = wall(&topo, current);
        let result = resize_cylindrical_face(&mut topo, current, face, radius)
            .unwrap_or_else(|error| panic!("Ø{diameter}: {error}"));
        assert_eq!(before, serialize_solid(&topo, current).unwrap());
        verify(
            &topo,
            result,
            counts,
            &issues,
            (0.0, 0.0),
            radius,
            36.0,
            gecko_volume(radius),
            &GECKO_EXPOSED,
        );
        // The block side planes keep their exact radial contacts.
        for point in [
            Point3::new(radius + 0.05, 0.05, 12.0),
            Point3::new(0.05, radius + 0.05, 12.0),
            Point3::new(radius - 0.05, 0.05, 30.0),
        ] {
            assert_eq!(classify(&topo, result, point), PointClassification::Inside);
        }
        for point in [
            Point3::new(radius + 0.05, 0.05, 30.0),
            Point3::new(0.05, -0.05, 30.0 + 7.0),
        ] {
            assert_eq!(classify(&topo, result, point), PointClassification::Outside);
        }
        current = result;
    }
}

#[test]
fn gecko_boss_resizes_inward_from_the_source() {
    for diameter in [10.0, 4.0] {
        let (mut topo, source) = gecko();
        let counts = solid_entity_counts(&topo, source).unwrap();
        let radius = diameter / 2.0;
        let issues = issue_keys(&topo, source);
        let face = wall(&topo, source);
        let result = resize_cylindrical_face(&mut topo, source, face, radius).unwrap();
        verify(
            &topo,
            result,
            counts,
            &issues,
            (0.0, 0.0),
            radius,
            36.0,
            gecko_volume(radius),
            &GECKO_EXPOSED,
        );
    }
}

#[test]
fn gecko_boss_oversized_or_invalid_diameter_refuses_without_mutation() {
    // r >= 18 meets the block's back face y = 18 and leaves the x = 0 face.
    for (radius, topology_change) in [
        (18.0, true),
        (20.0, true),
        (40.0, true),
        (0.0, false),
        (-3.0, false),
    ] {
        let (mut topo, source) = gecko();
        let face = wall(&topo, source);
        let before = serialize_solid(&topo, source).unwrap();
        let slots = topo.allocated_slot_count();
        let error = resize_cylindrical_face(&mut topo, source, face, radius).unwrap_err();
        if topology_change {
            assert!(is_topology_change(&error), "r={radius}: {error:?}");
        } else {
            assert!(
                matches!(error, OperationsError::InvalidInput { .. }),
                "{error:?}"
            );
        }
        assert_eq!(slots, topo.allocated_slot_count());
        assert_eq!(before, serialize_solid(&topo, source).unwrap());
    }
}

fn half_boss(radius: f64) -> (Topology, SolidId) {
    use remus_math::{
        context::{FallbackPolicy, OperationContext},
        mat::Mat4,
    };
    use remus_operations::{
        boolean::{BooleanOp, boolean_with_context},
        primitives::{make_box, make_cylinder},
        transform::transform_solid,
    };
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 25.0, 20.0).unwrap();
    let boss = make_cylinder(&mut topo, radius, 32.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(20.0, 0.0, 0.0)).unwrap();
    let solid = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        block,
        boss,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap()
    .solid;
    // The exact fuse leaves the bottom split along the old block and disk
    // outlines; merge it back to one face as a modelled part would have.
    remus_operations::heal::unify_faces(&mut topo, solid).unwrap();
    (topo, solid)
}

#[test]
fn synthetic_half_turn_boss_resizes_exactly_both_ways() {
    for target in [7.5, 3.0, 12.25] {
        let (mut topo, source) = half_boss(5.0);
        let issues = issue_keys(&topo, source);
        assert_eq!(census(&topo, source), (8, 1, 0));
        let counts = solid_entity_counts(&topo, source).unwrap();
        assert!(
            (measure::solid_volume(&topo, source, 1e-3).unwrap() - half_boss_volume(5.0)).abs()
                < half_boss_volume(5.0) * 1e-6
        );
        let face = wall(&topo, source);
        let before = serialize_solid(&topo, source).unwrap();
        let result = resize_cylindrical_face(&mut topo, source, face, target)
            .unwrap_or_else(|error| panic!("r={target}: {error}"));
        assert_eq!(before, serialize_solid(&topo, source).unwrap());
        verify(
            &topo,
            result,
            counts,
            &issues,
            (20.0, 0.0),
            target,
            32.0,
            half_boss_volume(target),
            &[1.25 * PI, 1.5 * PI, 1.75 * PI],
        );
    }
}

#[test]
fn synthetic_half_turn_boss_refuses_reaching_the_block_ends() {
    // The front faces' far edges sit 20 from the axis.
    for radius in [20.0, 22.0] {
        let (mut topo, source) = half_boss(5.0);
        let face = wall(&topo, source);
        let before = serialize_solid(&topo, source).unwrap();
        let slots = topo.allocated_slot_count();
        let error = resize_cylindrical_face(&mut topo, source, face, radius).unwrap_err();
        assert!(is_topology_change(&error), "r={radius}: {error:?}");
        assert_eq!(slots, topo.allocated_slot_count());
        assert_eq!(before, serialize_solid(&topo, source).unwrap());
    }
}

#[test]
fn gecko_boss_journaled_resize_uses_the_exact_footprint() {
    let (mut topo, source) = gecko();
    let counts = solid_entity_counts(&topo, source).unwrap();
    let issues = issue_keys(&topo, source);
    let face = wall(&topo, source);
    let result = remus_operations::journal_ops::resize_cylindrical_face_journaled(
        &mut topo, source, face, 7.3,
    )
    .unwrap()
    .solid;
    verify(
        &topo,
        result,
        counts,
        &issues,
        (0.0, 0.0),
        7.3,
        36.0,
        gecko_volume(7.3),
        &GECKO_EXPOSED,
    );
}
