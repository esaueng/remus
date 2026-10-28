//! B18: full-turn radial-profile history comes from construction identities.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::vec::{Point3, Vec3};
use remus_operations::boundary_evolution::{BoundaryEvent, completeness_for_result_solids};
use remus_operations::journal_ops::{
    linear_pattern_journaled, move_faces_journaled, revolve_journaled,
};
use remus_operations::measure::{solid_surface_area, solid_volume};
use remus_operations::revolve::{revolve, revolve_with_entity_evolution};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::journal::EntityKind;
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

fn profile(
    topo: &mut Topology,
    points: &[(f64, f64)],
    scale: f64,
    placed: bool,
) -> (FaceId, Point3, Vec3) {
    let (sine, cosine) = if placed {
        0.37_f64.sin_cos()
    } else {
        (0.0, 1.0)
    };
    let (tilt_sine, tilt_cosine) = if placed {
        0.23_f64.sin_cos()
    } else {
        (0.0, 1.0)
    };
    let axis = Vec3::new(0.0, -tilt_sine, tilt_cosine);
    let origin = if placed {
        Point3::new(10.0 * scale, -7.0 * scale, 3.0 * scale)
    } else {
        Point3::new(0.0, 0.0, 0.0)
    };
    let vertices: Vec<_> = points
        .iter()
        .map(|&(radius, height)| {
            topo.add_vertex(Vertex::new(
                origin
                    + Vec3::new(
                        radius * scale * cosine,
                        radius * scale * sine * tilt_cosine - height * scale * tilt_sine,
                        radius * scale * sine * tilt_sine + height * scale * tilt_cosine,
                    ),
                1e-7,
            ))
        })
        .collect();
    let edges = (0..vertices.len())
        .map(|index| {
            let edge = topo.add_edge(Edge::new(
                vertices[index],
                vertices[(index + 1) % vertices.len()],
                EdgeCurve::Line,
            ));
            OrientedEdge::new(edge, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let normal = Vec3::new(sine, -cosine * tilt_cosine, -cosine * tilt_sine);
    let face = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal,
            d: normal.x() * origin.x() + normal.y() * origin.y() + normal.z() * origin.z(),
        },
    ));
    (face, origin, axis)
}

fn quantities(points: &[(f64, f64)]) -> (f64, f64) {
    let mut volume = 0.0;
    let mut area = 0.0;
    for (&(r0, z0), &(r1, z1)) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        volume += PI * (r0 * r0 + r0 * r1 + r1 * r1) * (z1 - z0) / 3.0;
        area += PI * (r0 + r1) * (r1 - r0).hypot(z1 - z0);
    }
    (volume.abs(), area)
}

#[test]
fn analytic_revolve_records_every_entity_and_preserves_legacy_geometry() {
    let profiles: &[&[(f64, f64)]] = &[
        &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)],
        &[(1.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 2.0)],
        &[(0.0, 0.0), (3.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
        &[(0.0, 0.0), (3.0, 0.0), (0.0, 2.0)],
        &[
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 1.0),
            (2.0, 1.0),
            (2.0, 2.0),
            (0.0, 2.0),
        ],
    ];
    for points in profiles {
        let (volume, area) = quantities(points);
        for scale in [0.001_f64, 1.0, 1000.0] {
            for placed in [false, true] {
                let mut topo = Topology::new();
                let (face, origin, axis) = profile(&mut topo, points, scale, placed);
                let mut legacy = topo.clone();
                let result =
                    revolve_with_entity_evolution(&mut topo, face, origin, axis, TAU).unwrap();
                let old = revolve(&mut legacy, face, origin, axis, TAU).unwrap();
                assert_eq!(
                    remus_io::arena_io::serialize_solids(&topo, &[result.solid]).unwrap(),
                    remus_io::arena_io::serialize_solids(&legacy, &[old]).unwrap(),
                    "history cannot change the legacy shape bytes"
                );
                assert!(result.completeness.is_resolved());
                assert_eq!(
                    result.boundary.edges.len(),
                    solid_edges(&topo, result.solid).unwrap().len()
                );
                assert_eq!(
                    result.boundary.vertices.len(),
                    solid_vertices(&topo, result.solid).unwrap().len()
                );
                assert!(
                    remus_operations::validate::validate_solid(&topo, result.solid)
                        .unwrap()
                        .is_valid()
                );
                let measured = solid_volume(&topo, result.solid, 0.01 * scale).unwrap();
                assert!((measured / scale.powi(3) - volume).abs() < volume * 1e-7);
                let measured = solid_surface_area(&topo, result.solid, 0.01 * scale).unwrap();
                assert!((measured / scale.powi(2) - area).abs() < area * 1e-7);
                let mesh = tessellate_solid(&topo, result.solid, 0.02 * scale).unwrap();
                assert_eq!(boundary_edge_count(&mesh), 0);
                assert_eq!(non_manifold_edge_count(&mesh), 0);
            }
        }
    }
}

#[test]
fn revolve_completeness_detects_omitted_and_phantom_boundary_claims() {
    let mut topo = Topology::new();
    let (face, origin, axis) = profile(
        &mut topo,
        &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)],
        1.0,
        false,
    );
    let result = revolve_with_entity_evolution(&mut topo, face, origin, axis, TAU).unwrap();
    let mut missing = result.boundary.clone();
    let first = *missing.edges.keys().next().unwrap();
    missing.edges.remove(&first);
    let report =
        completeness_for_result_solids(&topo, &result.faces, &missing, &[result.solid]).unwrap();
    assert_eq!(report.edges.omitted, vec![first]);
    assert!(!report.is_accounted());
    let mut phantom = result.boundary.clone();
    phantom
        .vertices
        .insert(usize::MAX, BoundaryEvent::Modified { from: 0 });
    let report =
        completeness_for_result_solids(&topo, &result.faces, &phantom, &[result.solid]).unwrap();
    assert_eq!(report.vertices.phantom, vec![usize::MAX]);
    assert!(!report.is_accounted());
}

#[test]
fn journaled_revolve_references_survive_later_pattern_and_checkpoint_restore() {
    let mut topo = Topology::new();
    let (face, origin, axis) = profile(
        &mut topo,
        &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)],
        1.0,
        false,
    );
    let before = topo.clone();
    let result = revolve_journaled(&mut topo, face, origin, axis, TAU).unwrap();
    let counts = [
        (
            EntityKind::Face,
            solid_faces(&topo, result.solid).unwrap().len(),
        ),
        (
            EntityKind::Edge,
            solid_edges(&topo, result.solid).unwrap().len(),
        ),
        (
            EntityKind::Vertex,
            solid_vertices(&topo, result.solid).unwrap().len(),
        ),
    ];
    let references: Vec<_> = counts
        .into_iter()
        .flat_map(|(kind, count)| {
            (0..count).map(move |index| PersistentRef::operation_output(result.op, kind, index))
        })
        .collect();
    let checkpoint = topo.clone();
    linear_pattern_journaled(&mut topo, result.solid, Vec3::new(1.0, 0.0, 0.0), 10.0, 2).unwrap();
    for reference in &references {
        assert!(matches!(
            resolve(&topo, reference),
            Resolution::Bound {
                provenance: Provenance::Construction,
                ..
            }
        ));
    }
    topo.restore_preserving_handle_slots(&checkpoint);
    for reference in &references {
        assert!(matches!(
            resolve(&topo, reference),
            Resolution::Bound {
                provenance: Provenance::Construction,
                ..
            }
        ));
    }
    // Redo the later edit by replay, preserving the checkpoint barrier's
    // sticky retirements rather than resurrecting retired numeric handles.
    linear_pattern_journaled(&mut topo, result.solid, Vec3::new(1.0, 0.0, 0.0), 10.0, 2).unwrap();
    for reference in &references {
        assert!(matches!(
            resolve(&topo, reference),
            Resolution::Bound {
                provenance: Provenance::Construction,
                ..
            }
        ));
    }
    topo.restore_preserving_handle_slots(&before);
    assert!(topo.solid(result.solid).is_err());
    assert!(
        topo.journal()
            .entries()
            .iter()
            .all(|entry| entry.op() != result.op)
    );
    for reference in &references {
        assert!(!matches!(
            resolve(&topo, reference),
            Resolution::Bound { .. }
        ));
    }
}

#[test]
fn unqualified_revolve_history_refuses_without_publishing_or_mutating() {
    let mut topo = Topology::new();
    let (face, origin, axis) = profile(
        &mut topo,
        &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)],
        1.0,
        false,
    );
    revolve_journaled(&mut topo, face, origin, axis, TAU).unwrap();
    // A pre-existing unjournaled mutation would make journal_begin publish a
    // global barrier. Refusal must roll that setup back along with geometry.
    let marker = topo.add_vertex(Vertex::new(Point3::new(20.0, 30.0, 40.0), 1e-7));
    for angle in [0.0, PI, TAU + 0.1, f64::NAN] {
        let before = topo.clone();
        let journal = topo.journal().snapshot();
        assert!(revolve_journaled(&mut topo, face, origin, axis, angle).is_err());
        assert_eq!(topo.num_solids(), before.num_solids());
        assert_eq!(topo.num_faces(), before.num_faces());
        assert_eq!(topo.journal().snapshot().entries, journal.entries);
        assert_eq!(
            topo.vertex(marker).unwrap().point(),
            Point3::new(20.0, 30.0, 40.0)
        );
    }
}

#[test]
fn curved_and_holed_profiles_refuse_transactionally() {
    for curved in [false, true] {
        let mut topo = Topology::new();
        let (face, origin, axis) = if curved {
            let (face, origin, axis) =
                profile(&mut topo, &[(0.0, 0.0), (3.0, 0.0), (0.0, 3.0)], 1.0, false);
            let outer = topo.face(face).unwrap().outer_wire();
            let edge = topo.wire(outer).unwrap().edges()[1].edge();
            let circle = remus_math::curves::Circle3D::with_axes(
                origin,
                Vec3::new(0.0, -1.0, 0.0),
                3.0,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap();
            topo.edge_mut(edge)
                .unwrap()
                .set_curve(EdgeCurve::Circle(circle));
            topo.edge_mut(edge)
                .unwrap()
                .set_trim(Some((0.0, std::f64::consts::FRAC_PI_2)));
            (face, origin, axis)
        } else {
            let (outer, origin, axis) = profile(
                &mut topo,
                &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)],
                1.0,
                false,
            );
            let (hole, _, _) = profile(
                &mut topo,
                &[(1.0, 0.5), (1.0, 1.5), (2.0, 1.5), (2.0, 0.5)],
                1.0,
                false,
            );
            let data = topo.face(outer).unwrap();
            let face = Face::new(
                data.outer_wire(),
                vec![topo.face(hole).unwrap().outer_wire()],
                data.surface().clone(),
            );
            (topo.add_face(face), origin, axis)
        };
        let before = topo.clone();
        assert!(matches!(
            revolve_journaled(&mut topo, face, origin, axis, TAU),
            Err(remus_operations::OperationsError::Unsupported { .. })
        ));
        assert_eq!(topo.num_vertices(), before.num_vertices());
        assert_eq!(topo.num_edges(), before.num_edges());
        assert_eq!(topo.num_wires(), before.num_wires());
        assert_eq!(topo.num_faces(), before.num_faces());
        assert_eq!(topo.num_solids(), before.num_solids());
        assert_eq!(
            topo.journal().snapshot().entries,
            before.journal().snapshot().entries
        );
    }
}

#[test]
fn revolve_output_references_follow_cap_edit_undo_and_replay() {
    let mut topo = Topology::new();
    let (face, origin, axis) = profile(
        &mut topo,
        &[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)],
        1.0,
        false,
    );
    let result = revolve_journaled(&mut topo, face, origin, axis, TAU).unwrap();
    let wall = solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .find(|&id| {
            matches!(topo.face(id).unwrap().surface(),
            FaceSurface::Plane { normal, d } if normal.z() > 0.9 && *d > 1.9)
        })
        .unwrap();
    let references: Vec<_> = [
        (
            EntityKind::Face,
            solid_faces(&topo, result.solid).unwrap().len(),
        ),
        (
            EntityKind::Edge,
            solid_edges(&topo, result.solid).unwrap().len(),
        ),
        (
            EntityKind::Vertex,
            solid_vertices(&topo, result.solid).unwrap().len(),
        ),
    ]
    .into_iter()
    .flat_map(|(kind, count)| {
        (0..count).map(move |index| PersistentRef::operation_output(result.op, kind, index))
    })
    .collect();
    let checkpoint = topo.clone();
    for replay in [false, true] {
        if replay {
            topo.restore_preserving_handle_slots(&checkpoint);
        }
        let edited = move_faces_journaled(&mut topo, result.solid, &[wall], 0.5).unwrap();
        assert!((solid_volume(&topo, edited.solid, 0.01).unwrap() - 22.5 * PI).abs() < 1e-8);
        for reference in &references {
            assert!(
                matches!(
                    resolve(&topo, reference),
                    Resolution::Bound {
                        provenance: Provenance::Construction,
                        ..
                    }
                ),
                "{reference:?}: {:?}",
                resolve(&topo, reference)
            );
        }
        topo.restore_preserving_handle_slots(&checkpoint);
        for reference in &references {
            assert!(matches!(
                resolve(&topo, reference),
                Resolution::Bound {
                    provenance: Provenance::Construction,
                    ..
                }
            ));
        }
    }
}
