//! Exact curved-support boss removal qualification for program item 6.3.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::defeature::{defeature, defeature_with_evolution};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{face_vertices, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use std::collections::BTreeSet;

fn fixture() -> (Topology, SolidId, Vec<FaceId>) {
    let mut topo = Topology::new();
    let base = make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let boss = make_box(&mut topo, 5.0, 4.0, 4.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(8.0, -2.0, 8.0)).unwrap();
    let outcome = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        base,
        boss,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap();
    assert_eq!(outcome.quality, BooleanQuality::Exact);
    let input = outcome.solid;
    let faces = solid_faces(&topo, input).unwrap();
    let selected: Vec<_> = faces
        .iter()
        .copied()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            ) && face_vertices(&topo, face).unwrap().iter().all(|&vertex| {
                let point = topo.vertex(vertex).unwrap().point();
                point.z() > 7.0 && point.z() < 13.0
            })
        })
        .collect();
    assert_eq!(selected.len(), 5);
    let curved: Vec<_> = faces
        .iter()
        .copied()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .collect();
    assert_eq!(curved.len(), 1);
    assert_eq!(
        topo.face(curved[0]).unwrap().inner_wires().len(),
        1,
        "wound must be on the cylinder, not a planar cap"
    );
    (topo, input, selected)
}

#[test]
fn curved_wall_boss_restores_exact_cylinder_across_transforms() {
    for scale in [1e-3_f64, 1.0, 1e3] {
        let (mut topo, input, selected) = fixture();
        let placement = Mat4::translation(17.0 * scale, -23.0 * scale, 31.0 * scale)
            * Mat4::rotation_y(0.37)
            * Mat4::scale(scale, scale, scale);
        transform_solid(&mut topo, input, &placement).unwrap();
        let sources: BTreeSet<_> = solid_faces(&topo, input)
            .unwrap()
            .iter()
            .map(|face| face.index())
            .collect();
        let deleted: BTreeSet<_> = selected.iter().map(|face| face.index()).collect();
        let input_bytes = remus_io::arena_io::serialize_solid(&topo, input).unwrap();
        let (result, history) = defeature_with_evolution(&mut topo, input, &selected).unwrap();
        assert_eq!(
            remus_io::arena_io::serialize_solid(&topo, input).unwrap(),
            input_bytes
        );
        let faces = solid_faces(&topo, result).unwrap();
        assert_eq!(faces.len(), 3);
        assert!(history.origin.is_exact() && history.is_complete());
        assert!(history.generated.is_empty());
        assert_eq!(
            history.deleted.iter().copied().collect::<BTreeSet<_>>(),
            deleted
        );
        assert_eq!(
            history.modified.keys().copied().collect::<BTreeSet<_>>(),
            sources.difference(&deleted).copied().collect()
        );
        assert!(history.modified.values().all(|outputs| outputs.len() == 1));
        assert_eq!(
            history
                .modified
                .values()
                .flatten()
                .copied()
                .collect::<BTreeSet<_>>(),
            faces.iter().map(|face| face.index()).collect()
        );
        assert!(
            faces
                .iter()
                .all(|&face| topo.face(face).unwrap().inner_wires().is_empty())
        );
        assert!(
            remus_operations::validate::validate_solid(&topo, result)
                .unwrap()
                .is_valid()
        );
        let expected = std::f64::consts::PI * 2000.0 * scale.powi(3);
        let volume = solid_volume(&topo, result, 0.01 * scale).unwrap();
        assert!((volume - expected).abs() < expected * 1e-4);
        let mesh = tessellate_solid_with_tolerance(&topo, result, 0.01 * scale, 0.1).unwrap();
        assert!(is_watertight(&mesh));
        let origin = mesh.positions[0];
        let volume: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|triangle| {
                let a = mesh.positions[triangle[0] as usize] - origin;
                let b = mesh.positions[triangle[1] as usize] - origin;
                let c = mesh.positions[triangle[2] as usize] - origin;
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        assert!((volume.abs() - expected).abs() < expected * 0.003);
        for edge in solid_edges(&topo, result).unwrap() {
            let (start, end) = topo.edge(edge).unwrap().strict_domain().unwrap();
            assert!(start.is_finite() && end.is_finite() && (end - start).abs() > 0.0);
        }
    }
}

#[test]
fn incomplete_curved_wall_boss_selection_refuses_without_mutation() {
    let (mut topo, input, selected) = fixture();
    let snapshot = format!("{topo:?}");
    let error = defeature(&mut topo, input, &selected[..selected.len() - 1]).unwrap_err();
    assert!(matches!(
        error,
        remus_operations::OperationsError::Unsupported {
            operation: "defeature",
            ..
        }
    ));
    assert_eq!(format!("{topo:?}"), snapshot);
}
