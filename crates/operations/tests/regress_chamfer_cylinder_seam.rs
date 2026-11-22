//! B290: `chamfer_v2` on a cylinder's two rims and its seam.
//!
//! The seam of a closed face has no dihedral: the lateral meets itself
//! smoothly across it. `chamfer_v2` drops such an edge from the selection, as
//! it always has, but it used to drop it silently: the result reported two of
//! three requested edges as succeeded, none as failed and was not partial, so
//! a caller could not tell what had happened to the third (`modifier_ops`
//! fuzz target, completeness oracle I8). The result now names every dropped
//! edge in `BlendResult::skipped` with its reason.
//!
//! The rims themselves must be exact 45° (or two-distance) cone bands, and
//! the body watertight, manifold and of closed-form volume.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_operations::blend_ops::{BlendSkip, chamfer_v2};
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::{boundary_edge_count, non_manifold_edge_count};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 6.0;
const H: f64 = 6.0;

/// The fuzz reproducer's body: an r6 h6 cylinder, its rims and its seam.
fn cylinder() -> (Topology, SolidId, Vec<EdgeId>, EdgeId) {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_cylinder(&mut topo, R, H).unwrap();
    let edges = solid_edges(&topo, solid).unwrap();
    let seam = edges
        .iter()
        .copied()
        .find(|&e| matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Line))
        .unwrap();
    let rims = edges.iter().copied().filter(|&e| e != seam).collect();
    (topo, solid, rims, seam)
}

/// Volume of an r×h cylinder with both rims chamfered `radial` into the caps
/// and `axial` down the lateral: each rim loses a triangle of legs `radial`
/// and `axial` swept about the axis, its centroid `radial / 3` inside the
/// rim (Pappus).
fn chamfered_cylinder_volume(radial: f64, axial: f64) -> f64 {
    PI * R * R * H - 2.0 * (0.5 * radial * axial) * 2.0 * PI * (R - radial / 3.0)
}

/// Every check a chamfered cylinder must pass, for a chamfer of `(d1, d2)`
/// in either order.
fn assert_exact_chamfered_cylinder(topo: &Topology, solid: SolidId, d1: f64, d2: f64) {
    // Census: the lateral, two caps and two exact cone bands.
    let mut cones = Vec::new();
    let (mut planes, mut cylinders) = (0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => planes += 1,
            FaceSurface::Cylinder(_) => cylinders += 1,
            FaceSurface::Cone(cone) => cones.push((face, cone.clone())),
            other => panic!("unexpected {} face", other.type_tag()),
        }
    }
    assert_eq!((planes, cylinders, cones.len()), (2, 1, 2));

    // Each band is a cone about the cylinder's axis whose two boundary
    // circles are the chamfer's contact circles: one on the cap at R - a,
    // one on the lateral a distance b from the cap, {a, b} = {d1, d2}.
    let mut splits = Vec::new();
    for (face, cone) in &cones {
        let axis = cone.axis();
        assert!(axis.x().abs() < 1e-12 && axis.y().abs() < 1e-12, "{axis:?}");
        let apex = cone.apex();
        assert!(apex.x().abs() < 1e-12 && apex.y().abs() < 1e-12, "{apex:?}");
        let mut circles = Vec::new();
        let wire = topo.wire(topo.face(*face).unwrap().outer_wire()).unwrap();
        for oe in wire.edges() {
            if let EdgeCurve::Circle(c) = topo.edge(oe.edge()).unwrap().curve() {
                circles.push((c.radius(), c.center().z()));
                // The circle lies on the cone, to rounding.
                for k in 0..8 {
                    let t = f64::from(k) * PI / 4.0;
                    let p = c.evaluate(t);
                    let (u, v) = cone.project_point(p);
                    assert!((cone.evaluate(u, v) - p).length() < 1e-12);
                }
            }
        }
        assert_eq!(circles.len(), 2, "a band has two contact circles");
        circles.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        let ((inner_r, cap_z), (outer_r, lateral_z)) = (circles[0], circles[1]);
        assert!((outer_r - R).abs() < 1e-12, "{circles:?}");
        assert!(
            cap_z.abs() < 1e-12 || (cap_z - H).abs() < 1e-12,
            "{circles:?}"
        );
        let radial = R - inner_r;
        let axial = (lateral_z - cap_z).abs();
        let matches = |a: f64, b: f64| (radial - a).abs() < 1e-12 && (axial - b).abs() < 1e-12;
        assert!(matches(d1, d2) || matches(d2, d1), "{circles:?}");
        splits.push((radial, axial));
    }

    // B-Rep closed and manifold, valid by the loop-aware validator.
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    remus_topology::validation::validate_shell_closed(shell, topo).unwrap();
    remus_topology::validation::validate_shell_manifold(shell, topo).unwrap();

    // Mesh watertight.
    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, 0.01).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    assert_eq!(non_manifold_edge_count(&mesh), 0);

    // Closed-form volume, at two deflections finer than the clamp
    // (diag * 5e-5 ~ 6e-4 here); the exact routes agree to rounding.
    for (radial, axial) in &splits {
        let expected = chamfered_cylinder_volume(*radial, *axial);
        for deflection in [1e-4, 1e-5] {
            let volume = solid_volume(topo, solid, deflection).unwrap();
            assert!(
                (volume - expected).abs() < 1e-9 * expected,
                "volume {volume} vs closed form {expected} at {deflection}"
            );
        }
    }
}

/// The fuzz case: both rims and the seam at 0.05. Before the fix the result
/// said two of three edges succeeded, none failed, not partial.
#[test]
fn chamfering_a_cylinders_seam_names_it_as_skipped() {
    let (mut topo, solid, rims, seam) = cylinder();
    let mut selection = rims.clone();
    selection.push(seam);
    let result = chamfer_v2(&mut topo, solid, &selection, 0.05, 0.05).unwrap();

    assert!(!result.is_partial);
    assert!(result.failed.is_empty());
    let mut succeeded = result.succeeded.clone();
    succeeded.compact_sort_by_key(|e| e.index());
    let mut expected_rims = rims;
    expected_rims.compact_sort_by_key(|e| e.index());
    assert_eq!(succeeded, expected_rims);
    assert_eq!(result.skipped, vec![(seam, BlendSkip::Seam)]);
    // Every requested edge is accounted for, exactly once.
    assert_eq!(
        result.succeeded.len() + result.failed.len() + result.skipped.len(),
        selection.len()
    );

    assert_exact_chamfered_cylinder(&topo, result.solid, 0.05, 0.05);
}

/// The seam changes nothing about the geometry: the rims alone build the
/// same body and skip nothing, whatever order the seam is named in.
#[test]
fn the_seam_does_not_change_the_chamfered_rims() {
    for (d1, d2) in [(0.05, 0.05), (0.05, 0.15), (0.4, 0.1)] {
        let (mut topo, solid, rims, _) = cylinder();
        let alone = chamfer_v2(&mut topo, solid, &rims, d1, d2).unwrap();
        assert!(alone.skipped.is_empty());
        assert_exact_chamfered_cylinder(&topo, alone.solid, d1, d2);
        let alone_volume = solid_volume(&topo, alone.solid, 1e-5).unwrap();
        let alone_counts = solid_entity_counts(&topo, alone.solid).unwrap();

        let (mut topo, solid, rims, seam) = cylinder();
        let selection = [seam, rims[0], seam, rims[1]];
        let with_seam = chamfer_v2(&mut topo, solid, &selection, d1, d2).unwrap();
        assert_eq!(with_seam.skipped, vec![(seam, BlendSkip::Seam)]);
        assert_eq!(with_seam.succeeded.len(), 2);
        assert_exact_chamfered_cylinder(&topo, with_seam.solid, d1, d2);
        assert_eq!(
            solid_entity_counts(&topo, with_seam.solid).unwrap(),
            alone_counts
        );
        let volume = solid_volume(&topo, with_seam.solid, 1e-5).unwrap();
        assert!((volume - alone_volume).abs() < 1e-9 * alone_volume);
    }
}

/// A selection of nothing but the seam has nothing to chamfer: a typed
/// refusal that leaves the body as it was, never a success.
#[test]
fn chamfering_only_the_seam_refuses() {
    let (mut topo, solid, _, seam) = cylinder();
    let counts = (topo.num_vertices(), topo.num_edges(), topo.num_faces());
    let Err(error) = chamfer_v2(&mut topo, solid, &[seam], 0.05, 0.05) else {
        panic!("a seam-only chamfer must refuse");
    };
    assert!(
        matches!(
            error,
            remus_operations::OperationsError::InvalidInput { .. }
        ),
        "{error}"
    );
    assert_eq!(
        (topo.num_vertices(), topo.num_edges(), topo.num_faces()),
        counts
    );
}

/// A placed cylinder skips the same seam: the reason is read from the
/// topology, not from a world-axis assumption.
#[test]
fn a_placed_cylinder_names_its_seam_too() {
    let (mut topo, solid, rims, seam) = cylinder();
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &(Mat4::translation(13.0, -7.0, 21.0) * Mat4::rotation_x(0.6) * Mat4::rotation_z(0.3)),
    )
    .unwrap();
    let selection = [rims[0], seam, rims[1]];
    let result = chamfer_v2(&mut topo, solid, &selection, 0.1, 0.1).unwrap();
    assert_eq!(result.skipped, vec![(seam, BlendSkip::Seam)]);
    assert_eq!(result.succeeded.len(), 2);
    assert!(!result.is_partial);
    let expected = chamfered_cylinder_volume(0.1, 0.1);
    let volume = solid_volume(&topo, result.solid, 1e-5).unwrap();
    assert!(
        (volume - expected).abs() < 1e-9 * expected,
        "{volume} vs {expected}"
    );
}

/// The fuzz target's second reproducer: a 1 x 1.5 x 1.5 box fused with a
/// 1.5 x 6.5 x 1.5 box turned 45 degrees about x, three of whose edges were
/// chosen. One of them joins two coplanar plane faces the fuse left
/// separate: it has no dihedral either, and is named as a tangent join.
#[test]
fn a_flat_join_left_by_a_fuse_is_named_as_tangent() {
    let mut topo = Topology::new();
    let stock = remus_operations::primitives::make_box(&mut topo, 1.0, 1.5, 1.5).unwrap();
    let tool = remus_operations::primitives::make_box(&mut topo, 1.5, 6.5, 1.5).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        tool,
        &(Mat4::translation(-0.5, -1.5, -2.0) * Mat4::rotation_x(PI / 4.0)),
    )
    .unwrap();
    let solid = remus_operations::boolean::boolean(
        &mut topo,
        remus_operations::boolean::BooleanOp::Fuse,
        stock,
        tool,
    )
    .unwrap();
    assert_eq!(solid_faces(&topo, solid).unwrap().len(), 11);
    // The harness's pick: three edges, every fifth from 110 mod the count.
    let edges = solid_edges(&topo, solid).unwrap();
    let selection: Vec<EdgeId> = (0..3).map(|k| edges[(110 + 5 * k) % edges.len()]).collect();

    let input_volume = solid_volume(&topo, solid, 1e-5).unwrap();
    let result = chamfer_v2(&mut topo, solid, &selection, 0.05, 0.05).unwrap();
    assert!(!result.is_partial && result.failed.is_empty());
    assert_eq!(result.succeeded.len(), 2);
    assert_eq!(result.skipped.len(), 1, "{:?}", result.skipped);
    let (flat, reason) = result.skipped[0];
    assert_eq!(reason, BlendSkip::Tangent);
    assert!(selection.contains(&flat) && !result.succeeded.contains(&flat));

    // The skipped edge really is flat: its two faces are planes with the
    // same outward normal.
    let mut normals = Vec::new();
    for face in solid_faces(&topo, solid).unwrap() {
        if remus_topology::explorer::face_edges(&topo, face)
            .unwrap()
            .contains(&flat)
        {
            let data = topo.face(face).unwrap();
            let FaceSurface::Plane { normal, .. } = data.surface() else {
                panic!("the flat join borders a {} face", data.surface().type_tag());
            };
            normals.push(if data.is_reversed() {
                -*normal
            } else {
                *normal
            });
        }
    }
    assert_eq!(normals.len(), 2);
    assert!(
        (normals[0].dot(normals[1]) - 1.0).abs() < 1e-12,
        "{normals:?}"
    );

    let report = remus_operations::validate::validate_solid(&topo, result.solid).unwrap();
    assert!(report.is_valid(), "{report:?}");
    let volume = solid_volume(&topo, result.solid, 1e-5).unwrap();
    assert!(volume < input_volume, "{volume} vs input {input_volume}");

    // The same body as chamfering the two sharp edges alone.
    let mut alone_topo = Topology::new();
    let stock = remus_operations::primitives::make_box(&mut alone_topo, 1.0, 1.5, 1.5).unwrap();
    let tool = remus_operations::primitives::make_box(&mut alone_topo, 1.5, 6.5, 1.5).unwrap();
    remus_operations::transform::transform_solid(
        &mut alone_topo,
        tool,
        &(Mat4::translation(-0.5, -1.5, -2.0) * Mat4::rotation_x(PI / 4.0)),
    )
    .unwrap();
    let alone_solid = remus_operations::boolean::boolean(
        &mut alone_topo,
        remus_operations::boolean::BooleanOp::Fuse,
        stock,
        tool,
    )
    .unwrap();
    let sharp: Vec<EdgeId> = selection.iter().copied().filter(|&e| e != flat).collect();
    let alone = chamfer_v2(&mut alone_topo, alone_solid, &sharp, 0.05, 0.05).unwrap();
    assert!(alone.skipped.is_empty());
    let alone_volume = solid_volume(&alone_topo, alone.solid, 1e-5).unwrap();
    assert!((volume - alone_volume).abs() < 1e-9 * alone_volume);
    assert_eq!(
        solid_entity_counts(&topo, result.solid).unwrap(),
        solid_entity_counts(&alone_topo, alone.solid).unwrap()
    );
}
