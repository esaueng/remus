//! B275: a bar through the wall of a pointed cone whose rim is split into
//! arcs opens the display mesh (found reviewing #1696, 2026-11-13).
//!
//! A pointed cone r2 h4 is fused onto a 6 × 1 × 1 box that shares its base
//! plane; the box's long edges cross the rim, so the cone wall's outer wire
//! carries nine edges instead of the primitive's three. An r0.3 bar is then
//! fused radially through the wall at z = 1.5. The B-Rep is exact, yet the
//! mesh is open:
//!
//! - a bar on the seam meridian (0 rad) or at 1.0 rad leaves its whole pierce
//!   loop unstitched at deflection 1e-3 (381 open edges), closed at 0.1 and
//!   0.01;
//! - a bar at π/2 opens the seam line from rim to apex (3 open edges at 0.1
//!   and 0.01, 250 at 1e-3).
//!
//! The cone without the box stays closed at every one of those placements,
//! and a bar on the far side of the wall (3.0 rad) stays closed with the box;
//! the controls below pin both, so the ignored witness keeps pointing at the
//! split rim. Pre-existing: identical counts with #1696's cone-wall pre-split
//! reverted.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::FRAC_PI_2;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::primitives::{make_box, make_cone, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

fn exact_fuse(topo: &mut Topology, a: SolidId, b: SolidId) -> SolidId {
    let outcome = boolean_with_context(
        topo,
        BooleanOp::Fuse,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| panic!("exact fuse refused: {e:?}"));
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "ExactOnly returned {:?}",
        outcome.quality
    );
    outcome.solid
}

/// Edge count of the cone wall's outer wire (the body's only curved face
/// before the bar goes in).
fn cone_wall_rim_edges(topo: &Topology, solid: SolidId) -> usize {
    let walls: Vec<usize> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|f| {
            let face = topo.face(f).unwrap();
            (!face.surface().is_planar())
                .then(|| topo.wire(face.outer_wire()).unwrap().edges().len())
        })
        .collect();
    assert_eq!(walls.len(), 1, "one cone wall expected");
    walls[0]
}

/// The pointed cone, on the box when `on_box`, with an r0.3 bar fused
/// radially through its wall at meridian `bar` (radians), z = 1.5.
fn body(on_box: bool, bar: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let cone = make_cone(&mut topo, 2.0, 0.0, 4.0).unwrap();
    let base = if on_box {
        let block = make_box(&mut topo, 6.0, 1.0, 1.0).unwrap();
        transform_solid(&mut topo, block, &Mat4::translation(-3.0, -0.5, -1.0)).unwrap();
        let base = exact_fuse(&mut topo, cone, block);
        assert_eq!(
            cone_wall_rim_edges(&topo, base),
            9,
            "the box splits the rim"
        );
        base
    } else {
        assert_eq!(cone_wall_rim_edges(&topo, cone), 3);
        cone
    };
    let rod = make_cylinder(&mut topo, 0.3, 3.0).unwrap();
    let placement =
        Mat4::translation(0.0, 0.0, 1.5) * Mat4::rotation_z(bar) * Mat4::rotation_y(FRAC_PI_2);
    transform_solid(&mut topo, rod, &placement).unwrap();
    let solid = exact_fuse(&mut topo, base, rod);
    (topo, solid)
}

/// `(deflection, boundary edges, non-manifold edges)` wherever the mesh is
/// not closed.
fn open_meshes(topo: &Topology, solid: SolidId, deflections: &[f64]) -> Vec<(f64, usize, usize)> {
    deflections
        .iter()
        .filter_map(|&d| {
            let mesh = tessellate_solid(topo, solid, d).unwrap();
            let (b, n) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
            (b + n > 0).then_some((d, b, n))
        })
        .collect()
}

const DEFLECTIONS: [f64; 3] = [0.1, 0.01, 1e-3];

#[test]
#[ignore = "open: a bar through a pointed cone wall whose rim the box splits leaves an open mesh (B275)"]
fn split_rim_cone_wall_pierce_mesh_is_closed() {
    let mut failures = Vec::new();
    for bar in [0.0, 1.0, FRAC_PI_2] {
        let (topo, solid) = body(true, bar);
        let open = open_meshes(&topo, solid, &DEFLECTIONS);
        if !open.is_empty() {
            failures.push((bar, open));
        }
    }
    assert!(
        failures.is_empty(),
        "open display meshes (bar meridian, [(deflection, boundary, non-manifold)]): {failures:?}"
    );
}

#[test]
fn controls_without_a_split_rim_or_away_from_the_seam_are_closed() {
    for bar in [0.0, 1.0, FRAC_PI_2] {
        let (topo, solid) = body(false, bar);
        let open = open_meshes(&topo, solid, &DEFLECTIONS);
        assert!(open.is_empty(), "cone alone, bar at {bar}: {open:?}");
    }
    let (topo, solid) = body(true, 3.0);
    let open = open_meshes(&topo, solid, &DEFLECTIONS);
    assert!(open.is_empty(), "split rim, far-side bar: {open:?}");
}
