//! Integration tests for the offset pipeline.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_offset::{
    JointType, OffsetError, OffsetOptions, offset_solid, offset_solid_with_face_map,
};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere, make_torus};
use remus_topology::Topology;

fn offset_opts() -> OffsetOptions {
    OffsetOptions {
        remove_self_intersections: false,
        ..Default::default()
    }
}

#[test]
fn offset_box_outward_face_count() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let result = offset_solid(&mut topo, solid, 0.5, offset_opts()).unwrap();
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    assert_eq!(shell.faces().len(), 6, "offset box should have 6 faces");
}

#[test]
fn face_map_refuses_non_one_to_one_options_without_mutation() {
    for options in [
        OffsetOptions {
            joint: JointType::Arc,
            ..Default::default()
        },
        OffsetOptions {
            remove_self_intersections: true,
            ..Default::default()
        },
    ] {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let counts = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        );
        let error = offset_solid_with_face_map(&mut topo, solid, 0.5, options).unwrap_err();
        assert!(matches!(error, OffsetError::InvalidInput { .. }));
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
            ),
            counts
        );
        assert!(topo.solid(solid).is_ok());
    }
}

#[test]
fn offset_box_inward_face_count() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let result = offset_solid(&mut topo, solid, -0.5, offset_opts()).unwrap();
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    assert_eq!(shell.faces().len(), 6);
}

#[test]
fn offset_rectangular_box() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 3.0, 5.0, 7.0).unwrap();
    let result = offset_solid(&mut topo, solid, 1.0, offset_opts()).unwrap();
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    assert_eq!(shell.faces().len(), 6);
}

// Volume tests — for planar offsets, Phase 3 computes exact intersection
// line endpoints using the offset distance as margin (no percentage-based
// approximation), and the wire builder clips edges at exact line-line
// intersection corners.  Corner vertices use the first line's point
// directly (no midpoint averaging) since coplanar lines intersect exactly.
// This gives exact volumes for box offsets.

#[test]
fn offset_box_outward_exact_volume() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let result = offset_solid(&mut topo, solid, 0.5, offset_opts()).unwrap();
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    let expected = 27.0_f64; // (2+1)^3
    let error_pct = ((vol - expected) / expected * 100.0).abs();
    assert!(
        error_pct < 0.01,
        "outward offset volume {vol:.10} should be {expected}, error {error_pct:.6}%"
    );
}

#[test]
fn offset_box_inward_exact_volume() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let result = offset_solid(&mut topo, solid, -0.5, offset_opts()).unwrap();
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    let expected = 27.0_f64; // (4-1)^3
    let error_pct = ((vol - expected) / expected * 100.0).abs();
    assert!(
        error_pct < 0.01,
        "inward offset volume {vol:.10} should be {expected}, error {error_pct:.6}%"
    );
}

#[test]
fn offset_rectangular_box_exact_volume() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 3.0, 5.0, 7.0).unwrap();
    let result = offset_solid(&mut topo, solid, 1.0, offset_opts()).unwrap();
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    let expected = 5.0 * 7.0 * 9.0; // 315
    let error_pct = ((vol - expected) / expected * 100.0).abs();
    assert!(
        error_pct < 0.01,
        "rectangular offset volume {vol:.10} should be {expected}, error {error_pct:.6}%"
    );
}

#[test]
fn offset_rectangular_box_inward_exact_volume() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 3.0, 5.0, 7.0).unwrap();
    let result = offset_solid(&mut topo, solid, -0.5, offset_opts()).unwrap();
    let vol = solid_volume(&topo, result, 0.01).unwrap();
    let expected = 2.0 * 4.0 * 6.0; // 48
    let error_pct = ((vol - expected) / expected * 100.0).abs();
    assert!(
        error_pct < 0.01,
        "inward rectangular offset volume {vol:.10} should be {expected}, error {error_pct:.6}%"
    );
}

#[test]
fn offset_box_outward_volume_larger_than_original() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let original_vol = solid_volume(&topo, solid, 0.1).unwrap();

    let result = offset_solid(&mut topo, solid, 0.5, offset_opts()).unwrap();
    let offset_vol = solid_volume(&topo, result, 0.1).unwrap();
    assert!(
        offset_vol > original_vol,
        "outward offset volume ({offset_vol}) should exceed original ({original_vol})"
    );
}

#[test]
fn offset_box_inward_volume_smaller_than_original() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let original_vol = solid_volume(&topo, solid, 0.1).unwrap();

    let result = offset_solid(&mut topo, solid, -0.5, offset_opts()).unwrap();
    let offset_vol = solid_volume(&topo, result, 0.1).unwrap();
    assert!(
        offset_vol < original_vol,
        "inward offset volume ({offset_vol}) should be less than original ({original_vol})"
    );
}

#[test]
fn offset_box_rebuilds_are_manifold_and_exact() {
    // Regression for randomized HashMap iteration selecting a mix of direct
    // and independently trimmed face loops. The old path intermittently
    // produced free edges and, when it happened to pass topology validation,
    // inconsistent face winding and incorrect volume.
    for _ in 0..64 {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let result = offset_solid(&mut topo, solid, 0.5, offset_opts()).unwrap();
        let volume = solid_volume(&topo, result, 0.01).unwrap();
        assert!(
            (volume - 27.0).abs() < 1e-9,
            "offset rebuild volume must remain exact, got {volume}"
        );
    }
}

// ── Cylinder offset tests ──────────────────────────────────────

#[test]
fn offset_cylinder_outward_produces_solid() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
    let result = offset_solid(&mut topo, solid, 0.5, offset_opts()).unwrap();
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    assert!(
        shell.faces().len() >= 3,
        "offset cylinder should have at least 3 faces, got {}",
        shell.faces().len()
    );
}

#[test]
fn offset_cylinder_volume_increases() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
    let original_vol = solid_volume(&topo, solid, 0.1).unwrap();

    if let Ok(result) = offset_solid(&mut topo, solid, 0.5, offset_opts()) {
        let offset_vol = solid_volume(&topo, result, 0.1).unwrap();
        assert!(
            offset_vol > original_vol,
            "outward offset volume ({offset_vol}) should exceed original ({original_vol})"
        );
    }
}

// ── Thick solid (shell) tests ─────────────────────────────────

#[test]
fn thick_solid_box_produces_hollow() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let faces: Vec<_> = topo.shell(shell_id).unwrap().faces().to_vec();
    let exclude = vec![faces[0]];

    let result =
        remus_offset::thick_solid(&mut topo, solid, -0.2, &exclude, offset_opts()).unwrap();

    let result_shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    assert!(
        result_shell.faces().len() >= 9,
        "thick solid should have >= 9 faces, got {}",
        result_shell.faces().len()
    );

    let vol = solid_volume(&topo, result, 0.1).unwrap();
    assert!(
        vol > 0.0,
        "thick solid should have positive volume, got {vol}"
    );
}

// ── Sphere offset tests ────────────────────────────────────────

#[test]
fn offset_sphere_outward_produces_solid() {
    let mut topo = Topology::new();
    let solid = make_sphere(&mut topo, 3.0, 16_usize).unwrap();
    let result = offset_solid(&mut topo, solid, 0.5, offset_opts()).unwrap();
    let shell = topo
        .shell(topo.solid(result).unwrap().outer_shell())
        .unwrap();
    assert!(!shell.faces().is_empty(), "offset sphere should have faces");
}

#[test]
fn offset_torus_stays_analytic() {
    // Regression: a torus face is doubly-periodic (a fundamental-polygon wire
    // with degenerate v0->v0 seam edges). The offset wire-builder's circle/seam
    // and chaining strategies couldn't rebuild it, so the offset solid ended up
    // with no faces ("no faces could be assembled"). The offset of a torus is a
    // concentric torus, so its fundamental-polygon wire is now rebuilt directly.
    for distance in [0.5_f64, -0.5, 1.0] {
        let mut topo = Topology::new();
        let solid = make_torus(&mut topo, 10.0, 3.0, 32).unwrap();
        let result = offset_solid(&mut topo, solid, distance, offset_opts()).unwrap();
        let shell = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        assert_eq!(
            shell.faces().len(),
            1,
            "offset torus by {distance} should be a single torus face"
        );
        assert!(
            matches!(
                topo.face(shell.faces()[0]).unwrap().surface(),
                remus_topology::face::FaceSurface::Torus(_)
            ),
            "offset torus by {distance} must stay analytic (Torus surface)"
        );
    }
}

#[test]
fn direct_edit_entity_maps_preserve_boundary_incidence() {
    use remus_topology::explorer::{face_wires, solid_edges, solid_faces, solid_vertices};
    use remus_topology::face::FaceSurface;
    use std::collections::BTreeSet;

    for replacement in [false, true] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 3.0, 5.0, 7.0).unwrap();
        let faces = solid_faces(&topo, source).unwrap();
        let selected = faces[0];
        let result = if replacement {
            let FaceSurface::Plane { normal, d } = *topo.face(selected).unwrap().surface() else {
                unreachable!()
            };
            remus_offset::replace_surface_with_entity_map(
                &mut topo,
                source,
                selected,
                FaceSurface::Plane {
                    normal,
                    d: d + 0.25,
                },
            )
            .unwrap()
        } else {
            remus_offset::move_faces_with_entity_map(&mut topo, source, &[selected], 0.25).unwrap()
        };
        let output_faces = solid_faces(&topo, result.solid).unwrap();
        let output_edges = solid_edges(&topo, result.solid).unwrap();
        let output_vertices = solid_vertices(&topo, result.solid).unwrap();
        assert_eq!(result.face_map.len(), faces.len());
        assert_eq!(
            result.edge_map.len(),
            solid_edges(&topo, source).unwrap().len()
        );
        assert_eq!(
            result.vertex_map.len(),
            solid_vertices(&topo, source).unwrap().len()
        );
        assert_eq!(
            result.face_map.values().copied().collect::<BTreeSet<_>>(),
            output_faces.into_iter().collect()
        );
        assert_eq!(
            result.edge_map.values().copied().collect::<BTreeSet<_>>(),
            output_edges.into_iter().collect()
        );
        assert_eq!(
            result.vertex_map.values().copied().collect::<BTreeSet<_>>(),
            output_vertices.into_iter().collect()
        );
        for source_edge in solid_edges(&topo, source).unwrap() {
            let before = topo.edge(source_edge).unwrap();
            let after = topo.edge(result.edge_map[&source_edge.index()]).unwrap();
            assert_eq!(after.start(), result.vertex_map[&before.start().index()]);
            assert_eq!(after.end(), result.vertex_map[&before.end().index()]);
        }
        for face in faces {
            let before = face_wires(&topo, face).unwrap();
            let after = face_wires(&topo, result.face_map[&face.index()]).unwrap();
            assert_eq!(before.len(), after.len());
            for (before_wire, after_wire) in before.into_iter().zip(after) {
                let before_edges = topo.wire(before_wire).unwrap().edges();
                let after_edges = topo.wire(after_wire).unwrap().edges();
                assert_eq!(before_edges.len(), after_edges.len());
                for (before_edge, after_edge) in before_edges.iter().zip(after_edges) {
                    assert_eq!(
                        after_edge.edge(),
                        result.edge_map[&before_edge.edge().index()]
                    );
                    assert_eq!(before_edge.is_forward(), after_edge.is_forward());
                }
            }
        }
    }
}
