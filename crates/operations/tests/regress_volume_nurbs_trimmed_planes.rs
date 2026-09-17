//! `solid_volume` must agree with the closed whole-solid mesh on a body whose
//! planar faces are trimmed by unrecognized marched NURBS sections.
//!
//! Repro: a 1×5×7.5 box cut by an oblique torus (a bore wall) — the 2026-09-13
//! and 2026-09-16 `modifier_ops` fuzz crashes, where `mass_properties`
//! (Gauss, exact on the true boundary) read ~36.41 while `solid_volume`
//! read ~35.27.
//!
//! Root: the bored body routes to `volume_from_direct_face_tessellation`
//! (the reversed torus bore wall triggers `needs_direct_tessellation`), and
//! the exact-analytic gate measured the NURBS-trimmed planes with their
//! chord polygons (`planar_face_signed_volume` silently chords any rim it
//! cannot recognize) while the same faces' own tessellation walks the raw
//! knot domain instead of the edge trim. The two disagreed by ~1.5 mm² per
//! face — far outside any chord budget — yet the consistency probe passed
//! because that budget scales with the *recognized* arc length (zero here).
//!
//! The gate now carries an `exact_boundary` bit: a plane whose boundary the
//! closed form chords without authority declines the exact path, and bodies
//! that then fall back to per-face summation are re-routed to the closed
//! whole-solid mesh, which shares rim vertices and never samples a trim.
//!
//! Ground truth is the closed mesh at two deflections (converged to 1e-4
//! relative) plus the Gauss sum, which integrates the true boundary by
//! construction. Nothing here is a recorded measurement.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_torus};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// The fuzz crash body: box bored by an oblique torus.
///
/// Magnitudes are decoded from the crash artifacts (`BaseBody` lattices in
/// `fuzz/fuzz_targets/shapegen.rs`): stock 1×5×7.5, torus R=6.5/r=3.25 placed
/// by a quarter-turn-plus rotation and a half-unit offset, mode=cut. The
/// fillet from the crash is incidental (any routing that reaches the direct
/// per-face path shows the same disagreement), and the blend drivers' own
/// volume guard measures with the `solid_volume` under test, so building
/// through them would refuse for the very bug this test pins.
fn bored_body(topo: &mut Topology) -> SolidId {
    let stock = make_box(topo, 1.0, 5.0, 7.5).unwrap();
    let tool = make_torus(topo, 6.5, 3.25, 16).unwrap();
    let place = Mat4::translation(-2.5, 2.0, -4.0) * Mat4::rotation_x(std::f64::consts::FRAC_PI_6);
    remus_operations::transform::transform_solid(topo, tool, &place).unwrap();
    boolean(topo, BooleanOp::Cut, stock, tool).unwrap()
}

/// Closed-mesh volume at a deflection, with its boundary-edge count: 0 means
/// the mesh is watertight and the divergence sum is trustworthy.
fn mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> (f64, usize) {
    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
    let mut total = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let p = |k: usize| {
            let v = mesh.positions[tri[k] as usize];
            (v.x(), v.y(), v.z())
        };
        let (ax, ay, az) = p(0);
        let (bx, by, bz) = p(1);
        let (cx, cy, cz) = p(2);
        total += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
    }
    (
        (total / 6.0).abs(),
        remus_operations::tessellate::boundary_edge_count(&mesh),
    )
}

#[test]
fn nurbs_trimmed_planes_agree_with_closed_mesh() {
    let mut topo = Topology::new();
    let solid = bored_body(&mut topo);
    let bb = solid_bounding_box(&topo, solid).unwrap();
    let diag = (bb.max - bb.min).length();

    let (coarse, coarse_open) = mesh_volume(&topo, solid, 1e-3);
    let (fine, fine_open) = mesh_volume(&topo, solid, 1e-4);
    assert_eq!(coarse_open, 0, "reference mesh is open at 1e-3");
    assert_eq!(fine_open, 0, "reference mesh is open at 1e-4");
    let mesh_rel = (coarse - fine).abs() / fine.abs().max(1e-6);
    assert!(
        mesh_rel <= 1e-4,
        "reference mesh unconverged: {coarse:.9} vs {fine:.9}"
    );

    let tessellated = solid_volume(&topo, solid, (diag * 4e-5).max(1e-7)).unwrap();
    let gauss = mass_properties(&topo, solid).unwrap().mass;
    for (label, v) in [("solid_volume", tessellated), ("mass_properties", gauss)] {
        let rel = (v - fine).abs() / fine.abs().max(1e-6);
        assert!(
            rel <= 1e-2,
            "{label} says {v:.9} but the closed mesh says {fine:.9} (relative {rel:.3e})"
        );
    }

    // The two routes must agree with each other, not just with the mesh:
    // that is the fuzz oracle that caught this (1e-2 slack).
    let scale = tessellated.abs().max(gauss.abs()).max(1e-6);
    let rel = (tessellated - gauss).abs() / scale;
    assert!(
        rel <= 1e-2,
        "routes disagree: solid_volume={tessellated:.9} mass_properties={gauss:.9} (relative {rel:.3e})"
    );
}
