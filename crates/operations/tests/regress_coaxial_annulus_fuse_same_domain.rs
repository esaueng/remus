//! Regression: fusing two coaxial revolved annuli that share a cylindrical
//! wall must produce an analytic result, not a mesh fallback.
//!
//! The rim (r24..45) and hub (r12..24) of a pipe flange share the r24
//! cylinder over the rim's height. Both instances of that circle carry the
//! same geometry but different parameterizations, so the same-domain
//! edge-set key — which used a midpoint sampled in stored order — hashed
//! them apart. With no same-domain pair, each coincident face was
//! classified independently by point sampling ON the other solid's
//! boundary: the split one (hub wall) resolved Inside and was dropped, the
//! unsplit one (rim wall) resolved Outside and survived. That leftover
//! interface face made both r24 circles non-manifold (3 faces each), the
//! acceptance gate rejected the GFA result, and the fuse mesh-fell-back to
//! ~1000 planar faces — taking every downstream boolean with it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::revolve::revolve;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;

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

fn free_and_nonmanifold(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let usage = edge_use_counts(topo, solid);
    (
        usage.values().filter(|&&c| c == 1).count(),
        usage.values().filter(|&&c| c >= 3).count(),
    )
}

fn surface_census(topo: &Topology, solid: SolidId) -> HashMap<&'static str, usize> {
    let mut census = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

/// Revolve a rectangular profile in the XZ plane about the Z axis.
fn revolved_annulus(
    topo: &mut Topology,
    r_inner: f64,
    r_outer: f64,
    z_lo: f64,
    z_hi: f64,
) -> SolidId {
    let pts = [
        Point3::new(r_inner, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_hi),
        Point3::new(r_inner, 0.0, z_hi),
    ];
    let wire = make_polygon_wire(topo, &pts, TOL).unwrap();
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
    )
    .unwrap()
}

#[test]
fn coaxial_annulus_fuse_stays_analytic() {
    let mut topo = Topology::new();
    let rim = revolved_annulus(&mut topo, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved_annulus(&mut topo, 12.0, 24.0, 0.0, 26.0);

    let blank = boolean(&mut topo, BooleanOp::Fuse, rim, hub).expect("coaxial fuse must succeed");

    // The shared r24 wall must be gone: 7 faces (r45/r24/r12 cylinders, the
    // merged-later coplanar bottoms, the two z-caps). A mesh fallback lands
    // in the hundreds and is all-planar.
    let census = surface_census(&topo, blank);
    let n: usize = census.values().sum();
    assert!(
        n <= 12,
        "expected an analytic fuse, got {n} faces (mesh fallback?): {census:?}"
    );
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        3,
        "outer r45, hub r24 above the rim, bore r12: {census:?}"
    );

    let (free, nonmanifold) = free_and_nonmanifold(&topo, blank);
    assert_eq!(free, 0, "fuse result must be closed");
    assert_eq!(
        nonmanifold, 0,
        "the coincident r24 interface must not survive as a third face on the r24 circles"
    );

    // unify_faces merges the two coplanar z=0 bottoms into one disc.
    let removed = unify_faces(&mut topo, blank).unwrap();
    assert_eq!(removed, 1, "the coplanar z=0 bottoms must merge");
    let (free, nonmanifold) = free_and_nonmanifold(&topo, blank);
    assert_eq!(
        (free, nonmanifold),
        (0, 0),
        "unify must keep the shell sound"
    );

    // Six bolt holes through the rim must each stay analytic.
    let mut current = blank;
    for i in 0..6 {
        let angle = std::f64::consts::TAU * f64::from(i) / 6.0;
        let bolt = primitives::make_cylinder(&mut topo, 3.0, 16.0).unwrap();
        transform_solid(
            &mut topo,
            bolt,
            &Mat4::translation(34.0 * angle.cos(), 34.0 * angle.sin(), -3.0),
        )
        .unwrap();
        current = boolean(&mut topo, BooleanOp::Cut, current, bolt)
            .unwrap_or_else(|e| panic!("bolt {i} cut failed: {e:?}"));
    }

    let census = surface_census(&topo, current);
    let n: usize = census.values().sum();
    assert!(
        n <= 20,
        "expected an analytic drilled flange, got {n} faces: {census:?}"
    );
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        9,
        "3 body cylinders + 6 bolt walls: {census:?}"
    );
    let (free, nonmanifold) = free_and_nonmanifold(&topo, current);
    assert_eq!(
        (free, nonmanifold),
        (0, 0),
        "drilled result must be watertight"
    );

    // Volume against the closed form.
    let pi = std::f64::consts::PI;
    let expected = pi
        * ((45.0f64 * 45.0 - 24.0 * 24.0) * 10.0 + (24.0f64 * 24.0 - 12.0 * 12.0) * 26.0
            - 6.0 * 3.0 * 3.0 * 10.0);
    let vol = measure::solid_volume(&topo, current, 0.05).unwrap();
    assert!(
        (vol - expected).abs() / expected < 0.005,
        "volume {vol} vs analytic {expected}"
    );

    // Ray-cast ground truth, not volume alone.
    let opts = ClassifyOptions::default();
    let probe = |x: f64, y: f64, z: f64| {
        classify_point(&topo, current, Point3::new(x, y, z), &opts).unwrap()
    };
    assert_eq!(
        probe(34.0, 0.0, 5.0),
        PointClassification::Outside,
        "bolt hole must be carved"
    );
    assert_eq!(
        probe(34.0 * (pi / 6.0).cos(), 34.0 * (pi / 6.0).sin(), 5.0),
        PointClassification::Inside,
        "material must remain between bolt holes"
    );
    assert_eq!(
        probe(18.0, 0.0, 20.0),
        PointClassification::Inside,
        "hub wall above the rim"
    );
    assert_eq!(
        probe(0.0, 0.0, 20.0),
        PointClassification::Outside,
        "bore must be open"
    );
}
