//! Issue #1051: a planar section loop may touch an existing hole without
//! crossing it. The plate top below has the rib footprint as an inner wire.
//! Each cutter encloses that footprint while sharing one edge or touching
//! just one vertex, and must remove the rib and the correct plate volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::extrude::extrude;
use remus_operations::measure::mass_properties;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::edge::EdgeId;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

fn placed_box(topo: &mut Topology, min: [f64; 3], max: [f64; 3]) -> SolidId {
    let solid =
        make_box(topo, max[0] - min[0], max[1] - min[1], max[2] - min[2]).expect("make box");
    transform_solid(topo, solid, &Mat4::translation(min[0], min[1], min[2])).expect("place box");
    solid
}

fn exact(topo: &mut Topology, op: BooleanOp, a: SolidId, b: SolidId) -> SolidId {
    let _ = env_logger::try_init();
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|err| panic!("{op:?} must succeed without a mesh fallback: {err}"));
    assert!(matches!(outcome.quality, BooleanQuality::Exact));
    outcome.solid
}

fn plate_with_rib(topo: &mut Topology) -> SolidId {
    // The fused operand has volume 100 + 4 = 104. Its y = 1 top face
    // carries the square rib footprint [-1,1] × [-1,1] as an inner wire.
    let plate = placed_box(topo, [-5.0, 0.0, -5.0], [5.0, 1.0, 5.0]);
    let rib = placed_box(topo, [-1.0, 1.0, -1.0], [1.0, 2.0, 1.0]);
    let result = exact(topo, BooleanOp::Fuse, plate, rib);
    assert!(solid_faces(topo, result).unwrap().into_iter().any(|fid| {
        let face = topo.face(fid).unwrap();
        !face.inner_wires().is_empty()
    }));
    result
}

fn assert_valid_watertight_volume(topo: &Topology, solid: SolidId, expected: f64) {
    let validation = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(validation.is_valid(), "{:?}", validation.issues);
    let validation = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    let errors: Vec<_> = validation
        .issues
        .iter()
        .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
        .collect();
    assert!(errors.is_empty(), "check-crate errors: {errors:?}");

    let mut edge_uses: HashMap<EdgeId, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                *edge_uses.entry(edge.edge()).or_default() += 1;
            }
        }
    }
    assert!(
        edge_uses.values().all(|&uses| uses == 2),
        "every result edge must have exactly two face uses: {edge_uses:?}"
    );
    let actual = mass_properties(topo, solid).unwrap().mass;
    assert!(
        (actual - expected).abs() < 1e-9,
        "exact volume {actual:.12} must equal closed form {expected:.12}"
    );
    for deflection in [0.1, 0.01] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "deflection {deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "deflection {deflection}");
    }
}

#[test]
fn planar_section_sharing_an_existing_hole_edge_stays_exact() {
    let mut topo = Topology::new();
    let plate = plate_with_rib(&mut topo);
    // The cutter encloses the rib footprint while sharing its z = -1
    // edge. It removes the 4 × 1 × 3 plate prism and the entire rib,
    // so the volume is (100 + 4) - 12 - 4 = 88.
    let cutter = placed_box(&mut topo, [-2.0, -1.0, -1.0], [2.0, 3.0, 2.0]);
    let result = exact(&mut topo, BooleanOp::Cut, plate, cutter);
    assert_valid_watertight_volume(&topo, result, 88.0);
}

#[test]
fn planar_section_touching_an_existing_hole_vertex_stays_exact() {
    let mut topo = Topology::new();
    let plate = plate_with_rib(&mut topo);
    // This polygon encloses the rib footprint and touches its boundary
    // only at the common (x,z) = (1,1) vertex. Its exact shoelace area is
    // 30, so it removes 30 plate volume plus the entire 4-volume rib:
    // (100 + 4) - 30 - 4 = 70.
    let points = [
        Point3::new(-3.0, -1.0, -3.0),
        Point3::new(3.0, -1.0, -3.0),
        Point3::new(3.0, -1.0, 0.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(0.0, -1.0, 3.0),
        Point3::new(-3.0, -1.0, 3.0),
    ];
    let wire = make_polygon_wire(&mut topo, &points, 1e-7).expect("cutter polygon");
    let face = make_planar_face_from_wire(&mut topo, wire).expect("cutter profile");
    let cutter = extrude(&mut topo, face, Vec3::new(0.0, 1.0, 0.0), 4.0)
        .expect("extrude cutter through plate and rib");
    let result = exact(&mut topo, BooleanOp::Cut, plate, cutter);
    assert_valid_watertight_volume(&topo, result, 70.0);
}
