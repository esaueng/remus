//! A convex ridge chamfer repairs planar winding and retains exact setbacks.
//!
//! This formerly failed the modifier's face-orientation gate. The construction
//! now normalizes the rebuilt planar wires; the validity gate stays enabled.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::chamfer_v2;
use remus_operations::extrude::extrude;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::solid_edges;
use remus_topology::face::{Face, FaceSurface};

#[test]
fn chamfer_v2_convex_ridge_repairs_winding_and_matches_exact_volume() {
    let mut topo = Topology::new();
    // The top laterals meet at (5,2), with sin(interior angle) = 20/29.
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

    // The ridge edge runs along (5, 2, z).
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

    let before = remus_operations::measure::solid_volume(&topo, solid, 0.05).unwrap();
    let source = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
    let result = chamfer_v2(&mut topo, solid, &[ridge], 0.5, 0.5).unwrap();
    let report = remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let shell = topo
        .shell(topo.solid(result.solid).unwrap().outer_shell())
        .unwrap();
    remus_topology::validation::validate_shell_closed(shell, &topo).unwrap();
    remus_topology::validation::validate_shell_manifold(shell, &topo).unwrap();
    // Triangular cross-section: area = d²*sin(angle)/2, extruded 8 mm.
    let expected = 320.0 - 8.0 * 0.5 * 0.5 * 0.5 * 20.0 / 29.0;
    let actual = remus_operations::measure::solid_volume(&topo, result.solid, 0.001).unwrap();
    assert!(
        (actual - expected).abs() < 1e-9,
        "expected {expected}, got {actual}"
    );

    // The accepted construction also leaves its source solid unchanged.
    let after = remus_operations::measure::solid_volume(&topo, solid, 0.05).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "chamfer mutated the source: before={before:.12} after={after:.12}"
    );
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap(),
        source
    );
}
