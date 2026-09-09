//! Bounded cylindrical blend-resize history through repeated exact edits.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_operations::{
    blend_ops::fillet_v2,
    journal_ops::{resize_blend_journaled, solid_entity_keys},
    primitives::make_box,
};
use remus_topology::{
    FaceId, SolidId, Topology,
    explorer::{solid_edges, solid_faces},
    face::FaceSurface,
    journal::{EntityKind, EventDraft, EvolutionDraft},
    naming::{PersistentRef, Provenance, Resolution, resolve},
};

fn band(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap()
}

fn fixture(topo: &mut Topology) -> SolidId {
    let sharp = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(topo, sharp).unwrap()[0];
    fillet_v2(topo, sharp, &[edge], 1.0).unwrap().solid
}

fn references(topo: &mut Topology, solid: SolidId) -> Vec<PersistentRef> {
    let keys = solid_entity_keys(topo, solid).unwrap();
    let pending = topo.journal_begin("fixture");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for &key in &keys {
        draft.push(key, EventDraft::Generated { sources: vec![] });
    }
    let anchor = topo.journal_record_evolution(pending, draft).unwrap();
    [EntityKind::Face, EntityKind::Edge, EntityKind::Vertex]
        .into_iter()
        .flat_map(|kind| {
            (0..keys.iter().filter(|key| key.kind == kind).count())
                .map(move |index| PersistentRef::operation_output(anchor, kind, index))
        })
        .collect()
}

fn assert_bound(topo: &Topology, solid: SolidId, refs: &[PersistentRef]) {
    let mut found = std::collections::BTreeSet::new();
    for reference in refs {
        let outcome = resolve(topo, reference);
        let Resolution::Bound {
            entity,
            provenance: Provenance::Construction,
        } = outcome
        else {
            panic!("lost reference: {outcome:?}");
        };
        assert!(found.insert(entity));
    }
    assert_eq!(
        found,
        solid_entity_keys(topo, solid)
            .unwrap()
            .into_iter()
            .collect()
    );
}

#[test]
fn blend_resize_preserves_all_references_through_repeated_edits() {
    for scale in [0.1, 1.0, 10.0] {
        for imported in [false, true] {
            let mut topo = Topology::new();
            let mut solid = fixture(&mut topo);
            let transform =
                remus_math::mat::Mat4::translation(23.0 * scale, -7.0 * scale, 4.0 * scale)
                    * remus_math::mat::Mat4::rotation_x(0.37)
                    * remus_math::mat::Mat4::scale(scale, scale, scale);
            remus_operations::transform::transform_solid(&mut topo, solid, &transform).unwrap();
            if imported {
                let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
                solid = remus_io::step::reader::read_step(&step, &mut topo).unwrap()[0];
            }
            let refs = references(&mut topo, solid);
            let mut radius = scale;
            for next in [2.0 * scale, 0.75 * scale, 0.75 * scale] {
                let face = band(&topo, solid);
                solid = resize_blend_journaled(&mut topo, solid, face, radius, next)
                    .unwrap()
                    .solid;
                radius = next;
                assert_bound(&topo, solid, &refs);
                let faces = solid_faces(&topo, solid).unwrap();
                assert_eq!(faces.len(), 7);
                assert_eq!(
                    faces
                        .iter()
                        .filter(|&&face| topo.face(face).unwrap().surface().is_planar())
                        .count(),
                    6
                );
                let FaceSurface::Cylinder(cylinder) =
                    topo.face(band(&topo, solid)).unwrap().surface()
                else {
                    unreachable!()
                };
                assert!((cylinder.radius() - next).abs() < scale * 1e-9);

                let expected = (1000.0
                    - 10.0 * (next / scale).powi(2) * (1.0 - std::f64::consts::FRAC_PI_4))
                    * scale.powi(3);
                let volume =
                    remus_operations::measure::solid_volume(&topo, solid, 0.01 * scale).unwrap();
                assert!(
                    (volume - expected).abs() < expected * 1e-6,
                    "{volume} != {expected}"
                );
                assert!(
                    remus_operations::validate::validate_solid(&topo, solid)
                        .unwrap()
                        .is_valid()
                );
                let mesh = remus_operations::tessellate::tessellate_solid_with_tolerance(
                    &topo,
                    solid,
                    0.01 * scale,
                    0.1,
                )
                .unwrap();
                assert!(remus_operations::tessellate::is_watertight(&mesh));
                let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
                let mut roundtrip = Topology::new();
                let restored = remus_io::step::reader::read_step(&step, &mut roundtrip).unwrap()[0];
                let actual =
                    remus_operations::measure::solid_volume(&roundtrip, restored, 0.01 * scale)
                        .unwrap();
                assert!((actual - expected).abs() < expected * 1e-6);
            }
            let bytes = remus_io::arena_io::serialize_solids(&topo, &[solid]).unwrap();
            let mut restored = Topology::new();
            make_box(&mut restored, 1.0, 1.0, 1.0).unwrap();
            let solid = remus_io::arena_io::deserialize_solids(&bytes, &mut restored).unwrap()[0];
            assert_bound(&restored, solid, &refs);
            let face = band(&restored, solid);
            let edited =
                resize_blend_journaled(&mut restored, solid, face, radius, 1.25 * scale).unwrap();
            assert_bound(&restored, edited.solid, &refs);
        }
    }
}

#[test]
fn blend_resize_refusal_preserves_geometry_and_unpublished_history() {
    let mut topo = Topology::new();
    let solid = fixture(&mut topo);
    let refs = references(&mut topo, solid);
    make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    let face = band(&topo, solid);
    let before = topo.journal().snapshot();
    let geometry = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
    for (expected, next) in [(1.0, 0.0), (1.0, -1.0), (3.0, 2.0), (1.0, 50.0)] {
        assert!(resize_blend_journaled(&mut topo, solid, face, expected, next).is_err());
        let after = topo.journal().snapshot();
        assert_eq!(after.entries, before.entries);
        assert_eq!(after.index, before.index);
        assert_eq!(after.next_ordinal, before.next_ordinal);
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[solid]).unwrap(),
            geometry
        );
    }
    // The outstanding unrelated edit remains a barrier when the next call succeeds.
    let result = resize_blend_journaled(&mut topo, solid, face, 1.0, 2.0).unwrap();
    assert_eq!(topo.journal().entries().len(), before.entries.len() + 2);
    assert!(
        refs.iter()
            .all(|reference| !matches!(resolve(&topo, reference), Resolution::Bound { .. }))
    );
    assert!(topo.solid(result.solid).is_ok());
}

#[test]
fn curved_support_blend_history_refuses_without_changing_legacy_geometry() {
    let mut topo = Topology::new();
    let sharp = remus_operations::primitives::make_cylinder(&mut topo, 10.0, 20.0).unwrap();
    let edge = solid_edges(&topo, sharp)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            matches!(
                topo.edge(edge).unwrap().curve(),
                remus_topology::edge::EdgeCurve::Circle(_)
            )
        })
        .unwrap();
    let solid = fillet_v2(&mut topo, sharp, &[edge], 1.0).unwrap().solid;
    let face = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let refs = references(&mut topo, solid);
    let before = topo.journal().snapshot();
    let counts = remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap();
    assert!(resize_blend_journaled(&mut topo, solid, face, 1.0, 2.0).is_err());
    assert_eq!(topo.journal().snapshot().entries, before.entries);
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(&topo, solid).unwrap(),
        counts
    );
    assert_bound(&topo, solid, &refs);
    assert!(remus_operations::resize_blend::resize_blend(&mut topo, solid, face, 1.0, 2.0).is_ok());
}
