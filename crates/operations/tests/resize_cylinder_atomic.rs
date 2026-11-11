//! Refused full-turn sleeve resizes must leave the live arena unchanged.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::OperationsError;
use remus_operations::blend_cluster::PreparedGroupEdit;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn fixture(off_z: bool) -> (Topology, SolidId, FaceId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let neighbor = make_box(&mut topo, 10.0, 40.0, 20.0).unwrap();
    transform_solid(&mut topo, neighbor, &Mat4::translation(30.0, 0.0, 0.0)).unwrap();
    let block = boolean(&mut topo, BooleanOp::Fuse, block, neighbor).unwrap();
    let boss = make_cylinder(&mut topo, 5.0, 10.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(20.0, 20.0, 10.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Fuse, block, boss).unwrap();
    let shoulder = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(c)
            if (c.radius() - 5.0).abs() < 1e-7 && (c.center().z() - 10.0).abs() < 1e-7)
        })
        .unwrap();
    let blended = fillet_v2(&mut topo, solid, &[shoulder], 1.0).unwrap();
    assert!(!blended.is_partial);
    let solid = blended.solid;
    let face = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(c)
            if (c.radius() - 5.0).abs() < 1e-7)
        })
        .unwrap();
    if off_z {
        transform_solid(&mut topo, solid, &Mat4::rotation_y(0.7)).unwrap();
    }
    (topo, solid, face)
}

/// Live entities and exact source document; retired handle slots are allowed
/// to advance so handles from a failed operation can never alias later ones.
fn fingerprint(t: &Topology, solid: SolidId) -> ([usize; 11], Vec<u8>, String) {
    (
        [
            t.num_vertices(),
            t.num_edges(),
            t.num_wires(),
            t.num_faces(),
            t.num_shells(),
            t.num_solids(),
            t.compounds().len(),
            t.compsolids().len(),
            t.num_loops(),
            t.num_coedges(),
            t.num_pcurves(),
        ],
        remus_io::arena_io::serialize_solids(t, &[solid]).unwrap(),
        format!("{:?}", t.journal().snapshot()),
    )
}

fn assert_refusal(error: &OperationsError, radius: f64) {
    if radius.to_bits() == 25.0_f64.to_bits() {
        assert!(
            matches!(error, OperationsError::ExactOnlyUnattainable),
            "{error}"
        );
    } else {
        assert!(
            matches!(error, OperationsError::InvalidInput { reason }
            if reason.starts_with("cylindrical resize produced volume")),
            "{error}"
        );
    }
}

#[test]
fn push_pull_refused_sleeve_resize_is_atomic_on_and_off_z() {
    for off_z in [false, true] {
        let (mut topo, solid, face) = fixture(off_z);
        let before = fingerprint(&topo, solid);
        // R12 meets the neighboring wall, R25 is refused by the exact
        // boolean, R40 engulfs the step. Each fails after tool allocation.
        for radius in [12.0, 25.0, 40.0] {
            let error = resize_cylindrical_face(&mut topo, solid, face, radius).unwrap_err();
            assert_refusal(&error, radius);
            assert_eq!(
                fingerprint(&topo, solid),
                before,
                "off_z={off_z}, R{radius}"
            );
        }
    }
}

#[test]
fn push_pull_prepared_sleeve_refusals_are_atomic() {
    let (mut topo, solid, face) = fixture(false);
    let prepared = PreparedGroupEdit::prepare_cylinder(&topo, solid, face).unwrap();
    let before = fingerprint(&topo, solid);
    for radius in [12.0, 25.0, 40.0] {
        let error = prepared.preview(&mut topo, radius).unwrap_err();
        assert_refusal(&error, radius);
        assert_eq!(fingerprint(&topo, solid), before, "preview R{radius}");
        let error = prepared.commit(&mut topo, radius).unwrap_err();
        assert_refusal(&error, radius);
        assert_eq!(fingerprint(&topo, solid), before, "commit R{radius}");
        assert!(prepared.is_current(&topo));
    }
    // A refused frame must leave the preparation usable without restoring
    // a checkpoint. Success and its source retain their ordinary behavior.
    let result = prepared.commit(&mut topo, 8.0).unwrap();
    assert_ne!(result, solid);
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
        before.1
    );
}
