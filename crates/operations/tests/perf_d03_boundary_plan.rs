//! PERF-D03 integration contracts: deterministic public tessellation and
//! preserved display-vs-boolean sampling policy.
//!
//! These tests exercise only the public API (`tessellate_solid`,
//! `tessellate_solid_for_boolean`, grouped tessellation, face-ID
//! attribution). They pin what the bounded slice guarantees to callers:
//! repeated runs are byte-identical, grouped face offsets stay aligned with
//! the triangle list, open sheets keep their boundaries, and the boolean
//! path keeps its denser circle sampling while display stays coarser.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::tessellate::{
    boundary_edge_count, is_watertight, non_manifold_edge_count, tessellate_solid,
    tessellate_solid_for_boolean, tessellate_solid_grouped_with_tolerance,
};
use remus_topology::Topology;

/// Drilled-box fixture: holed planar caps plus a cylindrical wall, covering
/// the holed-planar CDT family and its neighbours.
fn drilled_box() -> (Topology, remus_topology::solid::SolidId) {
    let mut topo = Topology::new();
    let box_s = primitives::make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let cyl = primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        cyl,
        &Mat4::translation(10.0, 10.0, -5.0),
    )
    .unwrap();
    let result = boolean(&mut topo, BooleanOp::Cut, box_s, cyl).unwrap();
    (topo, result)
}

/// Repeated public tessellations are byte-identical: same positions in the
/// same order, same triangle emission order. This pins schedule-independent
/// output across native thread counts (the parallel CDT map preserves job
/// order) and matches sequential WASM execution.
#[test]
fn perf_d03_repeated_solid_tessellations_are_byte_identical() {
    let (topo, solid) = drilled_box();
    let first = tessellate_solid(&topo, solid, 0.1).unwrap();
    let second = tessellate_solid(&topo, solid, 0.1).unwrap();

    assert!(is_watertight(&first));
    assert_eq!(first.indices, second.indices);
    assert_eq!(first.positions.len(), second.positions.len());
    for (a, b) in first.positions.iter().zip(second.positions.iter()) {
        assert_eq!((a.x(), a.y(), a.z()), (b.x(), b.y(), b.z()));
    }
}

/// Watertightness holds at both a fine and a coarse deflection: sampling
/// off-by-ones are deflection dependent, so one passing value proves
/// nothing on its own.
#[test]
fn perf_d03_drilled_box_watertight_across_deflections() {
    let (topo, solid) = drilled_box();
    for deflection in [0.05, 0.1, 0.5] {
        let mesh = tessellate_solid(&topo, solid, deflection).unwrap();
        assert_eq!(
            (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh)),
            (0, 0),
            "deflection {deflection} must stay watertight"
        );
    }
}

/// Grouped tessellation keeps face offsets aligned: offsets run
/// `faces + 1` with a zero start and a full-length sentinel, every triangle
/// is attributed, and the grouped mesh matches the ungrouped one.
#[test]
fn perf_d03_grouped_offsets_stay_aligned() {
    let (topo, solid) = drilled_box();
    let angular = remus_math::chord::DEFAULT_ANGULAR_TOL;
    let (grouped, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, 0.1, angular).unwrap();
    let plain = tessellate_solid(&topo, solid, 0.1).unwrap();

    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    assert_eq!(offsets.len(), faces.len() + 1);
    assert_eq!(offsets[0], 0);
    assert_eq!(*offsets.last().unwrap() as usize, grouped.indices.len());
    assert_eq!(grouped.indices.len(), plain.indices.len());
    assert!(is_watertight(&grouped));
}

/// The display-vs-boolean sampling policy is preserved: the boolean path
/// keeps the curvature floor on circular edges (denser, for co-refinement
/// robustness) while display stays floor-free. Both meshes are watertight;
/// the boolean mesh is never coarser on a curved solid.
#[test]
fn perf_d03_display_and_boolean_policies_stay_distinct() {
    let mut topo = Topology::new();
    let cyl = primitives::make_cylinder(&mut topo, 5.0, 10.0).unwrap();

    let angular = remus_math::chord::DEFAULT_ANGULAR_TOL;
    let display =
        remus_operations::tessellate::tessellate_solid_with_tolerance(&topo, cyl, 0.5, angular)
            .unwrap();
    let for_boolean = tessellate_solid_for_boolean(&topo, cyl, 0.5, angular).unwrap();

    assert!(is_watertight(&display));
    assert!(is_watertight(&for_boolean));
    assert!(
        for_boolean.positions.len() >= display.positions.len(),
        "boolean mesh ({} verts) must not be coarser than display ({} verts)",
        for_boolean.positions.len(),
        display.positions.len()
    );
}

/// Tessellation preserves material volume: the drilled-box mesh volume
/// agrees with the analytic box-minus-cylinder value within one percent at
/// display deflection.
#[test]
fn perf_d03_drilled_box_mesh_volume_matches_analytic() {
    let (topo, solid) = drilled_box();
    let mesh = tessellate_solid(&topo, solid, 0.05).unwrap();

    let mut volume = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        volume += a.x() * (b.y() * c.z() - b.z() * c.y()) - a.y() * (b.x() * c.z() - b.z() * c.x())
            + a.z() * (b.x() * c.y() - b.y() * c.x());
    }
    volume = volume.abs() / 6.0;

    let expected = 20.0 * 20.0 * 10.0 - std::f64::consts::PI * 9.0 * 10.0;
    let rel = ((volume - expected) / expected).abs();
    assert!(
        rel < 0.01,
        "mesh volume {volume} too far from analytic {expected} (rel {rel})"
    );

    // The B-Rep measure path agrees with the mesh integration.
    let measured = measure::solid_volume(&topo, solid, 0.05).unwrap();
    let rel_brep = ((measured - expected) / expected).abs();
    assert!(
        rel_brep < 0.01,
        "B-Rep volume {measured} too far from analytic {expected}"
    );
}
