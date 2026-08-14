//! Pins the fork's fail-closed contract for an upstream convex-ridge chamfer.
//!
//! Upstream accepts the generated shell and checks its volume. The fork's
//! modifier postcondition additionally detects inconsistent face orientation,
//! so returning that shell would expose invalid geometry to WASM consumers.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::blend_ops::chamfer_v2;
use brepkit_operations::extrude::extrude;
use brepkit_topology::Topology;
use brepkit_topology::builder::make_polygon_wire;
use brepkit_topology::explorer::solid_edges;
use brepkit_topology::face::{Face, FaceSurface};

#[test]
fn chamfer_v2_convex_ridge_fails_closed_on_winding_defect() {
    let mut topo = Topology::new();
    // A shallow ridge: the vertex at (5, 0.05) makes the two top laterals
    // meet at ~178.9 degrees after extrusion.
    let profile = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 2.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, -3.0, 0.0),
            Point3::new(0.0, -3.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let solid = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 8.0).unwrap();

    // The ridge edge runs along (5, 0.05, z).
    let ridge = solid_edges(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&eid| {
            let e = topo.edge(eid).unwrap();
            let s = topo.vertex(e.start()).unwrap().point();
            let t = topo.vertex(e.end()).unwrap().point();
            (s.x() - 5.0).abs() < 1e-9 && (t.x() - 5.0).abs() < 1e-9 && (s.z() - t.z()).abs() > 1.0
        })
        .expect("ridge edge");

    let before = brepkit_operations::measure::solid_volume(&topo, solid, 0.05).unwrap();
    let error = match chamfer_v2(&mut topo, solid, &[ridge], 0.5, 0.5) {
        Ok(_) => panic!("orientation-invalid chamfer must fail closed"),
        Err(error) => error,
    };
    assert!(
        error.to_string().contains("inconsistent face orientations"),
        "unexpected chamfer rejection: {error}"
    );

    // The transactional modifier contract restores the input after rejection.
    let after = brepkit_operations::measure::solid_volume(&topo, solid, 0.05).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "failed chamfer mutated the input: before={before:.12} after={after:.12}"
    );
}
