#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use super::*;
use remus_math::mat::Mat4;
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityEvent, EntryPayload};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

fn top(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid).unwrap().into_iter().find(|face| {
        matches!(topo.face(*face).unwrap().surface(), FaceSurface::Plane { normal, .. } if normal.z() > 0.99)
    }).unwrap()
}

fn ring(scale: f64) -> Vec<Point3> {
    [
        [3.0, 2.0, 4.0],
        [7.0, 2.0, 4.0],
        [7.0, 6.0, 4.0],
        [3.0, 6.0, 4.0],
    ]
    .map(|p| Point3::new(p[0] * scale, p[1] * scale, p[2] * scale))
    .to_vec()
}

fn seed(topo: &mut Topology, solid: SolidId) -> OpId {
    let pending = topo.journal_begin("imprint_test_seed");
    let mut draft = EvolutionDraft::construction();
    for key in solid_entity_keys(topo, solid).unwrap() {
        draft.push(key, EventDraft::Generated { sources: vec![] });
    }
    topo.journal_record_evolution(pending, draft).unwrap()
}

fn reference(topo: &Topology, op: OpId, key: EntityKey) -> PersistentRef {
    (0..64).map(|ordinal| PersistentRef::operation_output(op, key.kind, ordinal)).find(|reference| {
        matches!(resolve(topo, reference), Resolution::Bound { entity, .. } if entity == key)
    }).unwrap()
}

#[test]
fn box_partition_has_independent_quantities_material_mesh_and_construction_identity() {
    for scale in [1e-3, 1.0, 1e3] {
        for reversed in [false, true] {
            let mut topo = Topology::new();
            let source =
                crate::primitives::make_box(&mut topo, 10.0 * scale, 8.0 * scale, 4.0 * scale)
                    .unwrap();
            let face = top(&topo, source);
            let op = seed(&mut topo, source);
            let old_face_ref = reference(&topo, op, EntityKey::face(face.index()));
            let old_edges = solid_edges(&topo, source).unwrap();
            let old_vertices = solid_vertices(&topo, source).unwrap();
            let edge_ref = reference(&topo, op, EntityKey::edge(old_edges[0].index()));
            let vertex_ref = reference(&topo, op, EntityKey::vertex(old_vertices[0].index()));
            let mut points = ring(scale);
            if reversed {
                points.reverse();
            }
            let result = imprint_planar_polygon(&mut topo, source, face, &points).unwrap();
            assert_eq!(solid_faces(&topo, result.solid).unwrap().len(), 7);
            assert_eq!(solid_edges(&topo, result.solid).unwrap().len(), 16);
            assert_eq!(solid_vertices(&topo, result.solid).unwrap().len(), 12);
            assert_eq!(solid_faces(&topo, source).unwrap().len(), 6);
            for (patch, area) in [(result.inner_face, 16.0), (result.surround_face, 64.0)] {
                let measured = crate::measure::face_area(&topo, patch, scale * 1e-4).unwrap();
                assert!((measured - area * scale * scale).abs() < 1e-9 * scale * scale);
                remus_topology::validation::validate_face_loops(&topo, patch).unwrap();
            }
            let volume = crate::measure::solid_volume(&topo, result.solid, scale * 1e-4).unwrap();
            assert!((volume - 320.0 * scale.powi(3)).abs() < 1e-8 * scale.powi(3));
            let strict = remus_check::validate::validate_solid(
                &topo,
                result.solid,
                &remus_check::validate::ValidateOptions::default(),
            )
            .unwrap();
            assert_eq!(strict.error_count(), 0, "{:?}", strict.issues);
            assert!(strict.issues.iter().any(|issue| issue.check
                == remus_check::validate::CheckId::SolidEulerCharacteristic
                && issue.severity == remus_check::validate::Severity::Warning));
            assert!(
                crate::validate::validate_solid(&topo, result.solid)
                    .unwrap()
                    .is_valid()
            );
            let mesh =
                crate::tessellate::tessellate_solid(&topo, result.solid, scale * 1e-4).unwrap();
            assert_eq!(crate::tessellate::boundary_edge_count(&mesh), 0);
            assert_eq!(crate::tessellate::non_manifold_edge_count(&mesh), 0);
            mesh_oracle(&mesh, scale, 320.0);
            for x in [2.0, 3.0, 5.0, 7.0, 8.0] {
                for z in [3.0, 5.0] {
                    let actual = remus_check::classify::classify_point(
                        &topo,
                        result.solid,
                        Point3::new(x * scale, 4.0 * scale, z * scale),
                        &remus_check::classify::ClassifyOptions::default(),
                    )
                    .unwrap();
                    assert_eq!(
                        actual,
                        if z < 4.0 {
                            remus_check::classify::PointClassification::Inside
                        } else {
                            remus_check::classify::PointClassification::Outside
                        }
                    );
                }
            }
            assert!(
                matches!(resolve(&topo,&old_face_ref), Resolution::BoundMany { entities, provenance: Provenance::Construction } if entities.len()==2 && entities.contains(&EntityKey::face(result.inner_face.index())) && entities.contains(&EntityKey::face(result.surround_face.index())))
            );
            assert!(
                matches!(resolve(&topo,&edge_ref),Resolution::Bound { entity, provenance: Provenance::Construction } if entity==EntityKey::edge(old_edges[0].index()))
            );
            assert!(
                matches!(resolve(&topo,&vertex_ref),Resolution::Bound { entity, provenance: Provenance::Construction } if entity==EntityKey::vertex(old_vertices[0].index()))
            );
            let inner_loop = topo
                .face_loop(topo.face(result.inner_face).unwrap().outer_loop().unwrap())
                .unwrap();
            let surround_loop = topo
                .face_loop(topo.face(result.surround_face).unwrap().inner_loops()[0])
                .unwrap();
            assert_eq!(inner_loop.coedges().len(), 4);
            for coedge in inner_loop.coedges() {
                let coedge = topo.coedge(*coedge).unwrap();
                assert!(surround_loop.coedges().iter().any(|other| {
                    let other = topo.coedge(*other).unwrap();
                    other.edge() == coedge.edge() && other.is_forward() != coedge.is_forward()
                }));
            }
        }
    }
}

// Independent position weld, directed incidence and signed tetrahedron volume.
fn mesh_oracle(mesh: &crate::tessellate::TriangleMesh, scale: f64, volume: f64) {
    use std::collections::BTreeMap;
    let mut positions = BTreeMap::new();
    let keys: Vec<_> = mesh
        .positions
        .iter()
        .map(|point| {
            let key = point.0.map(|v| (v / (scale * 1e-8)).round() as i64);
            let next = positions.len();
            *positions.entry(key).or_insert(next)
        })
        .collect();
    let mut edges: BTreeMap<_, (usize, i32)> = BTreeMap::new();
    let anchor = mesh.positions[0];
    let mut signed_volume = 0.0;
    for triangle in mesh.indices.chunks_exact(3) {
        for i in 0..3 {
            let a = keys[triangle[i] as usize];
            let b = keys[triangle[(i + 1) % 3] as usize];
            assert_ne!(a, b, "collapsed welded triangle");
            let value = edges.entry((a.min(b), a.max(b))).or_default();
            value.0 += 1;
            value.1 += if a < b { 1 } else { -1 };
        }
        let a = mesh.positions[triangle[0] as usize] - anchor;
        let b = mesh.positions[triangle[1] as usize] - anchor;
        let c = mesh.positions[triangle[2] as usize] - anchor;
        signed_volume += a.dot(b.cross(c)) / 6.0;
    }
    assert!(edges.values().all(|value| *value == (2, 0)));
    assert!((signed_volume - volume * scale.powi(3)).abs() < volume * scale.powi(3) * 1e-8);
}

fn attributes(name: &str) -> EntityAttributes {
    EntityAttributes {
        name: Some(name.into()),
        color: Some(ColorRgb::new(0.2, 0.4, 0.7).unwrap()),
    }
}

#[test]
fn rigid_placements_attributes_total_events_and_planar_pcurve_authority() {
    for scale in [1e-3, 1.0, 1e3] {
        let mut topo = Topology::new();
        let source =
            crate::primitives::make_box(&mut topo, 10.0 * scale, 8.0 * scale, 4.0 * scale).unwrap();
        let face = top(&topo, source);
        let matrix =
            Mat4::translation(13.0, -7.0, 4.0) * Mat4::rotation_z(0.37) * Mat4::rotation_y(0.41);
        crate::transform::transform_solid(&mut topo, source, &matrix).unwrap();
        topo.set_face_attributes(face, attributes("partition support"))
            .unwrap();
        topo.set_solid_attributes(source, attributes("body"))
            .unwrap();
        let seed_op = seed(&mut topo, source);
        let old = reference(&topo, seed_op, EntityKey::face(face.index()));
        let input: std::collections::BTreeSet<_> = solid_entity_keys(&topo, source)
            .unwrap()
            .into_iter()
            .collect();
        let points: Vec<_> = ring(scale)
            .into_iter()
            .map(|p| matrix.mul_point(p))
            .collect();
        let result = imprint_planar_polygon(&mut topo, source, face, &points).unwrap();
        assert_eq!(
            topo.attributes().solid(result.solid),
            Some(&attributes("body"))
        );
        for patch in [result.inner_face, result.surround_face] {
            assert_eq!(
                topo.attributes().face(patch),
                Some(&attributes("partition support"))
            );
            assert_eq!(
                format!("{:?}", topo.face(patch).unwrap().surface()),
                format!("{:?}", topo.face(face).unwrap().surface())
            );
            assert!(
                topo.pcurves_for_face(patch).is_empty(),
                "a plane has no intrinsic UV chart"
            );
        }
        let entry = topo
            .journal()
            .entries()
            .iter()
            .find(|entry| entry.op() == result.op)
            .unwrap();
        let EntryPayload::Evolution { events, .. } = entry.payload() else {
            panic!("construction evolution");
        };
        let mut output = std::collections::BTreeSet::new();
        let mut counts = [0; 3];
        for (ordinal, event) in events {
            let key = topo.journal().key_of(*ordinal).unwrap();
            assert!(output.insert(key));
            match event {
                EntityEvent::Modified { from } => {
                    assert_eq!(
                        topo.journal().key_of(*from),
                        Some(EntityKey::face(face.index()))
                    );
                    assert!(
                        [
                            EntityKey::face(result.inner_face.index()),
                            EntityKey::face(result.surround_face.index())
                        ]
                        .contains(&key)
                    );
                    counts[0] += 1;
                }
                EntityEvent::Generated { sources } => {
                    assert_eq!(sources.len(), 1);
                    assert_eq!(
                        topo.journal().key_of(sources[0]),
                        Some(EntityKey::face(face.index()))
                    );
                    assert!(!input.contains(&key));
                    counts[1] += 1;
                }
                EntityEvent::Preserved { from } => {
                    assert_eq!(topo.journal().key_of(*from), Some(key));
                    assert!(input.contains(&key));
                    counts[2] += 1;
                }
                other => panic!("unexpected event {other:?}"),
            }
        }
        assert_eq!(counts, [2, 8, 25]);
        assert_eq!(
            output,
            solid_entity_keys(&topo, result.solid)
                .unwrap()
                .into_iter()
                .collect()
        );
        assert!(
            matches!(resolve(&topo,&old), Resolution::BoundMany { provenance: Provenance::Construction, entities } if entities.len()==2)
        );
        let mesh = crate::tessellate::tessellate_solid(&topo, result.solid, scale * 1e-4).unwrap();
        mesh_oracle(&mesh, scale, 320.0);
    }
}

fn live_state(topo: &Topology) -> String {
    // Live arena content excludes intentionally retained rollback high-water slots.
    let loops: Vec<_> = topo
        .faces()
        .iter()
        .flat_map(|(_, face)| face.boundary_loops())
        .map(|id| topo.face_loop(*id).unwrap())
        .collect();
    let uses: Vec<_> = loops
        .iter()
        .flat_map(|loop_| loop_.coedges())
        .map(|id| topo.coedge(*id).unwrap())
        .collect();
    format!(
        "{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}{:?}",
        topo.vertices().iter().collect::<Vec<_>>(),
        topo.edges().iter().collect::<Vec<_>>(),
        topo.wires().iter().collect::<Vec<_>>(),
        topo.faces().iter().collect::<Vec<_>>(),
        topo.shells().iter().collect::<Vec<_>>(),
        topo.solids().iter().collect::<Vec<_>>(),
        loops,
        uses,
        topo.attributes(),
        topo.journal().entries()
    )
}

#[test]
fn malformed_boundary_star_clearance_caps_and_context_refuse_without_mutation() {
    let mut topo = Topology::new();
    let source = crate::primitives::make_box(&mut topo, 10.0, 8.0, 4.0).unwrap();
    let face = top(&topo, source);
    seed(&mut topo, source);
    let before = live_state(&topo);
    let star: Vec<_> = [0, 2, 4, 1, 3]
        .map(|i| {
            let angle = std::f64::consts::TAU * f64::from(i) / 5.0;
            Point3::new(5.0 + 2.0 * angle.cos(), 4.0 + 2.0 * angle.sin(), 4.0)
        })
        .to_vec();
    // This self-crossing traversal has a consistent local turn sign.
    assert!((0..5).all(|i| {
        side(
            Vec3::new(0., 0., 1.),
            star[i],
            star[(i + 1) % 5],
            star[(i + 2) % 5],
        )
        .lo > 0.0
    }));
    let mut malformed = vec![
        vec![],
        ring(1.0)[..2].to_vec(),
        star,
        vec![Point3::new(3., 2., 4.); 4],
        vec![Point3::new(3., 2., 4.); 17],
    ];
    let mut nonfinite = ring(1.0);
    nonfinite[2] = Point3::new(f64::NAN, 6., 4.);
    malformed.push(nonfinite);
    let mut offplane = ring(1.0);
    offplane[1] = Point3::new(7., 2., 4.001);
    malformed.push(offplane);
    let mut boundary = ring(1.0);
    boundary[0] = Point3::new(0., 2., 4.);
    malformed.push(boundary);
    malformed.push(vec![
        Point3::new(3., 2., 4.),
        Point3::new(7., 2., 4.),
        Point3::new(5., 3., 4.),
        Point3::new(7., 6., 4.),
        Point3::new(3., 6., 4.),
    ]);
    for points in malformed {
        assert!(
            imprint_planar_polygon(&mut topo, source, face, &points).is_err(),
            "{points:?}"
        );
        assert_eq!(live_state(&topo), before);
    }
    for linear in [0., -1., f64::NAN, f64::INFINITY] {
        let mut context = OperationContext::new();
        context.tolerance.linear = linear;
        assert!(matches!(
            imprint_planar_polygon_with_context(&mut topo, source, face, &ring(1.), &context),
            Err(OperationsError::InvalidInput { .. })
        ));
        assert_eq!(live_state(&topo), before);
    }
    for (segments, pairs) in [(7, 32), (8, 31)] {
        let mut context = OperationContext::new();
        context.budgets.segments = segments;
        context.budgets.vertex_pairs = pairs;
        assert!(matches!(
            imprint_planar_polygon_with_context(&mut topo, source, face, &ring(1.), &context),
            Err(OperationsError::Math(
                remus_math::MathError::ResourceLimitExceeded { .. }
            ))
        ));
        assert_eq!(live_state(&topo), before);
    }
    let token = remus_math::context::CancellationToken::new();
    token.cancel();
    assert!(matches!(
        imprint_planar_polygon_with_context(
            &mut topo,
            source,
            face,
            &ring(1.),
            &OperationContext::new().with_cancellation(token)
        ),
        Err(OperationsError::Math(remus_math::MathError::Cancelled))
    ));
    assert_eq!(live_state(&topo), before);
    let mut exact = OperationContext::new();
    exact.budgets.segments = 8;
    exact.budgets.vertex_pairs = 32;
    imprint_planar_polygon_with_context(&mut topo, source, face, &ring(1.), &exact).unwrap();
}

#[test]
fn result_validation_failure_rolls_back_allocations_attributes_and_pending_history() {
    let mut topo = Topology::new();
    let source = crate::primitives::make_box(&mut topo, 10., 8., 4.).unwrap();
    let face = top(&topo, source);
    let broken = solid_faces(&topo, source)
        .unwrap()
        .into_iter()
        .find(|candidate| *candidate != face)
        .unwrap();
    topo.face_mut(broken).unwrap().set_reversed(true);
    topo.set_face_attributes(face, attributes("retained"))
        .unwrap();
    seed(&mut topo, source);
    let before = live_state(&topo);
    let slots = topo.allocated_slot_count();
    assert!(matches!(
        imprint_planar_polygon(&mut topo, source, face, &ring(1.)),
        Err(OperationsError::BodyValidationFailed { .. })
    ));
    assert!(
        topo.allocated_slot_count() > slots,
        "must exercise post-allocation rollback"
    );
    assert_eq!(live_state(&topo), before);
}

#[test]
fn cavity_shell_ownership_and_reversed_face_use_preserve_material() {
    for select_cavity in [false, true] {
        let mut topo = Topology::new();
        let outer = crate::primitives::make_box(&mut topo, 10., 8., 4.).unwrap();
        let inner = crate::primitives::make_box(&mut topo, 2., 2., 2.).unwrap();
        crate::transform::transform_solid(&mut topo, inner, &Mat4::translation(4., 3., 1.))
            .unwrap();
        let inner_top = top(&topo, inner);
        for face in solid_faces(&topo, inner).unwrap() {
            topo.face_mut(face).unwrap().set_reversed(true);
        }
        let outer_shell = topo.solid(outer).unwrap().outer_shell();
        let inner_shell = topo.solid(inner).unwrap().outer_shell();
        let source = topo.add_solid(Solid::new(outer_shell, vec![inner_shell]));
        validate(&topo, source).unwrap();
        seed(&mut topo, source);
        let (face, points) = if select_cavity {
            (
                inner_top,
                vec![
                    Point3::new(4.5, 3.5, 3.),
                    Point3::new(5.5, 3.5, 3.),
                    Point3::new(5.5, 4.5, 3.),
                    Point3::new(4.5, 4.5, 3.),
                ],
            )
        } else {
            (top(&topo, outer), ring(1.))
        };
        let result = imprint_planar_polygon(&mut topo, source, face, &points).unwrap();
        let data = topo.solid(result.solid).unwrap();
        assert_eq!(data.inner_shells().len(), 1);
        assert_eq!(data.outer_shell() == outer_shell, select_cavity);
        assert_eq!(data.inner_shells()[0] == inner_shell, !select_cavity);
        assert_eq!(
            topo.face(result.inner_face).unwrap().is_reversed(),
            select_cavity
        );
        assert!(
            (crate::measure::solid_volume(&topo, result.solid, 1e-4).unwrap() - 312.).abs() < 1e-8
        );
        mesh_oracle(
            &crate::tessellate::tessellate_solid(&topo, result.solid, 1e-4).unwrap(),
            1.,
            312.,
        );
        for (point, expected) in [
            (
                Point3::new(5., 4., 2.),
                remus_check::classify::PointClassification::Outside,
            ),
            (
                Point3::new(2., 4., 2.),
                remus_check::classify::PointClassification::Inside,
            ),
        ] {
            assert_eq!(
                remus_check::classify::classify_point(
                    &topo,
                    result.solid,
                    point,
                    &remus_check::classify::ClassifyOptions::default()
                )
                .unwrap(),
                expected
            );
        }
    }
}

#[test]
fn bounded_ring_extremes_target_cap_and_unsupported_carriers_are_explicit() {
    for size in [3, 16] {
        let mut topo = Topology::new();
        let source = crate::primitives::make_box(&mut topo, 10., 8., 4.).unwrap();
        let face = top(&topo, source);
        let points: Vec<_> = (0..size)
            .map(|i| {
                let a = std::f64::consts::TAU * f64::from(i) / f64::from(size);
                Point3::new(5. + 2. * a.cos(), 4. + 2. * a.sin(), 4.)
            })
            .collect();
        let result = imprint_planar_polygon(&mut topo, source, face, &points).unwrap();
        assert_eq!(
            solid_edges(&topo, result.solid).unwrap().len(),
            12 + size as usize
        );
        mesh_oracle(
            &crate::tessellate::tessellate_solid(&topo, result.solid, 1e-4).unwrap(),
            1.,
            320.,
        );
    }
    for size in [32, 33] {
        let mut topo = Topology::new();
        let points: Vec<_> = (0..size)
            .map(|i| {
                let a = std::f64::consts::TAU * f64::from(i) / f64::from(size);
                Point3::new(5. + 4. * a.cos(), 4. + 4. * a.sin(), 0.)
            })
            .collect();
        let profile = remus_topology::builder::make_planar_face(&mut topo, &points, 1e-7).unwrap();
        let source =
            crate::extrude::extrude(&mut topo, profile, Vec3::new(0., 0., 1.), 4.).unwrap();
        let face = top(&topo, source);
        let before = live_state(&topo);
        let outcome = imprint_planar_polygon(&mut topo, source, face, &ring(1.));
        if size == 32 {
            outcome.unwrap();
        } else {
            assert!(matches!(
                outcome,
                Err(OperationsError::Math(
                    remus_math::MathError::ResourceLimitExceeded {
                        resource: "imprint_target_vertices",
                        limit: 32,
                        actual: 33
                    }
                ))
            ));
            assert_eq!(live_state(&topo), before);
        }
    }
    let mut topo = Topology::new();
    let source = crate::primitives::make_cylinder(&mut topo, 5., 4.).unwrap();
    for face in solid_faces(&topo, source).unwrap() {
        let before = live_state(&topo);
        let outcome = imprint_planar_polygon(&mut topo, source, face, &ring(1.));
        assert!(
            matches!(
                outcome,
                Err(OperationsError::Algo(
                    remus_algo::error::AlgoError::UnsupportedImprint { .. }
                ))
            ),
            "{outcome:?}"
        );
        assert_eq!(live_state(&topo), before);
    }
}
