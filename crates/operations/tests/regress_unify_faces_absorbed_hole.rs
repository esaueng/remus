//! Regression: `unify_faces` on a coaxial-cylinder fuse result must not
//! leave the absorbed hub circle behind as a stale inner wire.
//!
//! Fusing `make_cylinder(45, 12)` with the coaxial `make_cylinder(24, 30)`
//! produces two coplanar z=0 faces: an annulus (outer r45, inner wire r24)
//! and the hub disc (outer r24). `unify_faces` merges them, correctly
//! filtering the shared r24 circle from the outer boundary — but it used to
//! carry the annulus's inner wire over verbatim, so the "merged disc" was
//! still an annulus while the hub disc face had been consumed. The shell
//! was left open (one free edge at the r24 circle), and any subsequent
//! boolean (e.g. a bolt-hole cut, as run by consumers that unify between
//! booleans) failed with `NonManifoldResult`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Count edge uses across all wires of a solid: 1 = free (open shell),
/// 2 = manifold, 3+ = non-manifold.
fn edge_use_counts(topo: &Topology, solid: SolidId) -> HashMap<usize, usize> {
    let mut usage = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let wire = topo.wire(wid).unwrap();
            for oe in wire.edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    usage
}

fn free_edge_count(topo: &Topology, solid: SolidId) -> usize {
    edge_use_counts(topo, solid)
        .values()
        .filter(|&&c| c == 1)
        .count()
}

#[test]
fn unify_faces_after_coaxial_fuse_keeps_shell_closed_and_cut_succeeds() {
    let mut topo = Topology::new();
    let flange = primitives::make_cylinder(&mut topo, 45.0, 12.0).unwrap();
    let hub = primitives::make_cylinder(&mut topo, 24.0, 30.0).unwrap();

    let fused = boolean(&mut topo, BooleanOp::Fuse, flange, hub).unwrap();
    assert_eq!(
        free_edge_count(&topo, fused),
        0,
        "fuse result must be closed"
    );

    let removed = unify_faces(&mut topo, fused).unwrap();
    assert_eq!(removed, 1, "the two coplanar z=0 faces must merge into one");
    assert_eq!(
        free_edge_count(&topo, fused),
        0,
        "unify_faces must not open the shell (stale absorbed-hole inner wire)"
    );

    // The merged bottom face must be a full disc: exactly one z=0 planar
    // face, with no leftover inner wire from the absorbed hub circle.
    let bottom_faces: Vec<_> = solid_faces(&topo, fused)
        .unwrap()
        .into_iter()
        .filter(|&fid| {
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            wire.edges().iter().all(|oe| {
                let edge = topo.edge(oe.edge()).unwrap();
                let sp = topo.vertex(edge.start()).unwrap().point();
                sp.z().abs() < 1e-9 && topo.vertex(edge.end()).unwrap().point().z().abs() < 1e-9
            })
        })
        .collect();
    assert_eq!(bottom_faces.len(), 1, "exactly one merged bottom face");
    let bottom = topo.face(bottom_faces[0]).unwrap();
    assert!(
        bottom.inner_wires().is_empty(),
        "merged bottom disc must not retain the absorbed r24 hole wire"
    );

    // A bolt-hole cut through the flange must still work on the unified
    // solid (the doc contract: unify output is safe for further booleans).
    let bolt = primitives::make_cylinder(&mut topo, 3.5, 18.0).unwrap();
    transform_solid(&mut topo, bolt, &Mat4::translation(34.0, 0.0, -3.0)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, fused, bolt)
        .expect("bolt cut on unified solid must not fail");

    assert_eq!(
        free_edge_count(&topo, result),
        0,
        "cut result must be closed"
    );

    // Analytic result, not mesh fallback: a handful of faces with curved
    // types present.
    let faces = solid_faces(&topo, result).unwrap();
    assert!(
        faces.len() <= 12,
        "expected analytic result, got {} faces (mesh fallback?)",
        faces.len()
    );
    let cylinder_faces = faces
        .iter()
        .filter(|&&fid| topo.face(fid).unwrap().surface().type_tag() == "cylinder")
        .count();
    assert!(
        cylinder_faces >= 3,
        "outer wall, hub wall, and bolt hole wall"
    );

    // Volume: fuse volume minus the bolt hole through the 12mm flange.
    let expected = std::f64::consts::PI
        * (45.0 * 45.0 * 12.0 + 24.0 * 24.0 * (30.0 - 12.0) - 3.5 * 3.5 * 12.0);
    let vol = measure::solid_volume(&topo, result, 0.1).unwrap();
    assert!(
        (vol - expected).abs() / expected < 0.005,
        "volume {vol} vs analytic {expected}"
    );

    // Ray-cast ground truth: hole carved, material where it belongs.
    let opts = ClassifyOptions::default();
    let probe = |topo: &Topology, x: f64, y: f64, z: f64| {
        classify_point(topo, result, Point3::new(x, y, z), &opts).unwrap()
    };
    assert_eq!(
        probe(&topo, 34.0, 0.0, 6.0),
        PointClassification::Outside,
        "inside the bolt hole must be empty"
    );
    assert_eq!(
        probe(&topo, 34.0, 10.0, 6.0),
        PointClassification::Inside,
        "flange material beside the hole"
    );
    assert_eq!(
        probe(&topo, 10.0, 0.0, 0.5),
        PointClassification::Inside,
        "material just above the merged bottom disc"
    );
    assert_eq!(
        probe(&topo, 0.0, 0.0, 20.0),
        PointClassification::Inside,
        "hub core"
    );
}
