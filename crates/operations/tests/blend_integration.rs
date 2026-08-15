//! End-to-end integration tests for the validated fillet and chamfer APIs.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_operations::blend_ops::{chamfer_distance_angle, chamfer_v2, fillet_v2};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::{make_box, make_cone, make_cylinder};
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::explorer::{solid_edges, solid_faces};

const BOX_VOLUME: f64 = 1000.0; // 10 x 10 x 10

/// Create a 10x10x10 box and fillet a single edge.
#[test]
fn fillet_box_single_edge() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    assert!(!edges.is_empty(), "box must have edges");

    let result = fillet_v2(&mut topo, solid, &edges[..1], 1.0).unwrap();

    let faces = solid_faces(&topo, result.solid).unwrap();
    assert!(
        faces.len() > 6,
        "filleted box should have more than 6 faces"
    );
    assert!(
        !result.succeeded.is_empty(),
        "at least one edge should succeed"
    );

    let vol = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - BOX_VOLUME).abs() > 0.01,
        "filleted volume {vol} should differ from original {BOX_VOLUME}"
    );
}

/// Fillet 4 edges of a box (e.g. the first 4 found).
#[test]
fn fillet_box_multiple_edges() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    let n = edges.len().min(4);
    let target = &edges[..n];

    let result = fillet_v2(&mut topo, solid, target, 0.5).unwrap();
    assert!(
        !result.succeeded.is_empty(),
        "at least some edges should succeed"
    );

    let vol = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - BOX_VOLUME).abs() > 0.01,
        "filleted volume {vol} should differ from original {BOX_VOLUME}"
    );
}

/// Symmetric chamfer on a single edge.
#[test]
fn chamfer_box_symmetric() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    let result = chamfer_v2(&mut topo, solid, &edges[..1], 1.0, 1.0).unwrap();

    let faces = solid_faces(&topo, result.solid).unwrap();
    assert!(
        faces.len() > 6,
        "chamfered box should have more than 6 faces"
    );

    let vol = solid_volume(&topo, result.solid, 0.01).unwrap();
    assert!(
        (vol - BOX_VOLUME).abs() > 0.01,
        "chamfered volume {vol} should differ from original {BOX_VOLUME}"
    );
}

/// Distance-angle chamfer on a single edge.
#[test]
fn chamfer_box_distance_angle() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();

    let edges = solid_edges(&topo, solid).unwrap();
    let result = chamfer_distance_angle(
        &mut topo,
        solid,
        &edges[..1],
        1.0,
        std::f64::consts::FRAC_PI_4,
    )
    .unwrap();

    assert!(
        !result.succeeded.is_empty(),
        "distance-angle chamfer should succeed on at least one edge"
    );
}

#[test]
fn fillet_zero_radius_error() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    assert!(fillet_v2(&mut topo, solid, &edges[..1], 0.0).is_err());
}

#[test]
fn chamfer_zero_distance_error() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    assert!(chamfer_v2(&mut topo, solid, &edges[..1], 0.0, 1.0).is_err());
}

#[test]
fn fillet_empty_edges_error() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    assert!(fillet_v2(&mut topo, solid, &[], 1.0).is_err());
}

fn bottom_circle_edge(topo: &Topology, solid: brepkit_topology::solid::SolidId) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            matches!(edge.curve(), EdgeCurve::Circle(_))
                && topo.vertex(edge.start()).unwrap().point().z().abs() < 1e-9
        })
        .expect("primitive must have a bottom circle edge")
}

/// Strict validation for a closed-rim chamfer.
///
/// `expected_cones` counts every conical face in the result, not just the band:
/// a chamfered cylinder has one (the band), a chamfered cone has two (its own
/// wall plus the band).
fn assert_valid_closed_chamfer(
    topo: &Topology,
    result: &brepkit_blend::BlendResult,
    expected_cones: usize,
) {
    assert!(!result.is_partial);
    assert_eq!(result.succeeded.len(), 1);
    assert!(result.failed.is_empty());

    let report = brepkit_check::validate::validate_solid(
        topo,
        result.solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);

    let cone_count = solid_faces(topo, result.solid)
        .unwrap()
        .into_iter()
        .filter(|&face_id| {
            matches!(
                topo.face(face_id).unwrap().surface(),
                brepkit_topology::face::FaceSurface::Cone(_)
            )
        })
        .count();
    assert_eq!(
        cone_count, expected_cones,
        "closed-rim chamfer must add one exact conical band (not a NURBS approximation)"
    );
}

fn assert_valid_closed_fillet(topo: &Topology, result: &brepkit_blend::BlendResult) {
    assert!(!result.is_partial);
    assert_eq!(result.succeeded.len(), 1);
    assert!(result.failed.is_empty());

    let report = brepkit_check::validate::validate_solid(
        topo,
        result.solid,
        &brepkit_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);

    let torus_count = solid_faces(topo, result.solid)
        .unwrap()
        .into_iter()
        .filter(|&face_id| {
            matches!(
                topo.face(face_id).unwrap().surface(),
                brepkit_topology::face::FaceSurface::Torus(_)
            )
        })
        .count();
    assert_eq!(torus_count, 1, "closed-rim fillet must add one torus band");
}

/// Closed circular fillets use the exact analytic rim assembler and must pass
/// the same strict solid validation as planar production fillets.
#[test]
fn fillet_cylinder_closed_rim_is_valid() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    let result = fillet_v2(&mut topo, solid, &[rim], 0.3).unwrap();
    assert_valid_closed_fillet(&topo, &result);
}

#[test]
fn fillet_cone_closed_rim_is_valid() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    let result = fillet_v2(&mut topo, solid, &[rim], 0.3).unwrap();
    assert_valid_closed_fillet(&topo, &result);
}

/// Closed circular chamfers use the rim assembler ported from the fillet
/// builder and must pass the same strict validation as the fillet pair above.
///
/// These two previously asserted the opposite — that a closed-edge chamfer
/// must fail closed with "closed-edge ... invalid solid" — which pinned the
/// `reject_closed_edges` guard that stood in for the missing assembly. The
/// guard is gone now that `chamfer_builder` can build the annular case, so the
/// tests assert the real postcondition instead of the placeholder refusal.
#[test]
fn chamfer_cylinder_closed_rim_is_valid() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    let result = chamfer_v2(&mut topo, solid, &[rim], 0.4, 0.4).unwrap();
    assert_valid_closed_chamfer(&topo, &result, 1);
}

#[test]
fn chamfer_cone_closed_rim_is_valid() {
    let mut topo = Topology::new();
    let solid = make_cone(&mut topo, 3.0, 1.0, 4.0).unwrap();
    let rim = bottom_circle_edge(&topo, solid);
    let result = chamfer_v2(&mut topo, solid, &[rim], 0.4, 0.4).unwrap();
    assert_valid_closed_chamfer(&topo, &result, 2);
}

// ── Seed-selection handling ────────────────────────────────────
//
// Selections reaching `filletV2`/`chamferV2` from JS are commonly built from
// face adjacency (every shared edge named once per face) and commonly cover
// the whole part ("fillet all edges" on a baseplate or heat sink). Neither
// shape of input may be refused: repeats are collapsed, and the only ceiling
// on a selection is the solid's own edge count.

/// Boxes on a grid, fused into one body: a cheap stand-in for the many-edged
/// parts that "fillet all edges" is used on.
fn box_grid(topo: &mut Topology, nx: usize, ny: usize) -> brepkit_topology::solid::SolidId {
    let mut solids = Vec::with_capacity(nx * ny);
    for i in 0..nx {
        for j in 0..ny {
            let s = make_box(topo, 4.0, 4.0, 4.0).unwrap();
            #[allow(clippy::cast_precision_loss)]
            let step = (i as f64 * 10.0, j as f64 * 10.0);
            brepkit_operations::transform::transform_solid(
                topo,
                s,
                &brepkit_math::mat::Mat4::translation(step.0, step.1, 0.0),
            )
            .unwrap();
            solids.push(s);
        }
    }
    let compound = topo.add_compound(brepkit_topology::compound::Compound::new(solids));
    brepkit_operations::compound_ops::fuse_all(topo, compound).unwrap()
}

/// A duplicated seed says nothing new about the geometry, so it must be
/// collapsed rather than refused — and must produce exactly the result the
/// de-duplicated selection produces.
#[test]
fn fillet_duplicate_seed_edges_match_deduplicated_selection() {
    let measure = |selection: fn(&[EdgeId]) -> Vec<EdgeId>| {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();
        let result = fillet_v2(&mut topo, solid, &selection(&edges), 1.0).unwrap();
        let faces = solid_faces(&topo, result.solid).unwrap().len();
        let volume = solid_volume(&topo, result.solid, 0.01).unwrap();
        (faces, volume, result.succeeded.len())
    };

    let (unique_faces, unique_volume, unique_seeds) = measure(|e| vec![e[0], e[1]]);
    let (dup_faces, dup_volume, dup_seeds) = measure(|e| vec![e[0], e[1], e[0], e[1], e[1]]);

    // The fillet must actually have happened, not silently no-opped.
    assert!(
        unique_faces > 6,
        "fillet must add faces, got {unique_faces}"
    );
    assert!(
        unique_volume < BOX_VOLUME,
        "convex fillet must remove material: {unique_volume} vs {BOX_VOLUME}"
    );

    assert_eq!(dup_faces, unique_faces);
    assert!(
        (dup_volume - unique_volume).abs() < 1e-9,
        "duplicated selection changed the result: {dup_volume} vs {unique_volume}"
    );
    assert_eq!(
        dup_seeds, unique_seeds,
        "duplicates must not be reported twice"
    );
}

/// Duplicated chamfer seeds used to bevel the same edge twice and land a
/// non-manifold shell; they are collapsed for the same reason fillet's are.
#[test]
fn chamfer_duplicate_seed_edges_match_deduplicated_selection() {
    let measure = |selection: fn(&[EdgeId]) -> Vec<EdgeId>| {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let edges = solid_edges(&topo, solid).unwrap();
        let result = chamfer_v2(&mut topo, solid, &selection(&edges), 1.0, 1.0).unwrap();
        let volume = solid_volume(&topo, result.solid, 0.01).unwrap();
        (solid_faces(&topo, result.solid).unwrap().len(), volume)
    };

    let (unique_faces, unique_volume) = measure(|e| vec![e[0], e[1]]);
    let (dup_faces, dup_volume) = measure(|e| vec![e[0], e[1], e[0], e[1]]);

    assert!(unique_faces > 6, "chamfer must add faces");
    assert!(
        unique_volume < BOX_VOLUME,
        "chamfer must remove material: {unique_volume} vs {BOX_VOLUME}"
    );
    assert_eq!(dup_faces, unique_faces);
    assert!(
        (dup_volume - unique_volume).abs() < 1e-9,
        "duplicated selection changed the result: {dup_volume} vs {unique_volume}"
    );
}

/// "Fillet every edge" of a many-edged part. The selection here is 300 edges,
/// well past the 256-seed ceiling this path used to impose, and the ceiling is
/// what the assertions below are about: the blend itself must still land a
/// body with more faces and less material than it started with.
#[test]
fn fillet_all_edges_of_a_many_edged_part() {
    let mut topo = Topology::new();
    let solid = box_grid(&mut topo, 5, 5);

    let edges = solid_edges(&topo, solid).unwrap();
    assert!(
        edges.len() > 256,
        "fixture must exceed the old cap, got {} edges",
        edges.len()
    );
    let faces_before = solid_faces(&topo, solid).unwrap().len();
    let volume_before = solid_volume(&topo, solid, 0.05).unwrap();

    let result = fillet_v2(&mut topo, solid, &edges, 0.4).unwrap();

    assert_eq!(result.succeeded.len(), edges.len());
    let faces_after = solid_faces(&topo, result.solid).unwrap().len();
    let volume_after = solid_volume(&topo, result.solid, 0.05).unwrap();
    assert!(
        faces_after > faces_before,
        "fillet must add faces: {faces_before} -> {faces_after}"
    );
    assert!(
        volume_after < volume_before,
        "convex fillet must remove material: {volume_before} -> {volume_after}"
    );
}
