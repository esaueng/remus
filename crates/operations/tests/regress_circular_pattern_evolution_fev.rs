//! Total circular-pattern construction history: every result face, edge and
//! vertex journals a typed construction disposition by copy-time lineage.
//!
//! At the reviewed baseline `circular_pattern_journaled` did not exist:
//! `circular_pattern_impl` already collected the full correspondence via
//! `copy_solid_with_entity_map` and `PatternTracker::record_instance`, but
//! `circular_pattern_with_evolution` returned only `history.map`, leaving
//! every edge and vertex in scope without a claim. This file pins the upgrade:
//! the journaled circular pattern now records total F/E/V lineage from the
//! copy maps, never by coordinate or centroid matching, using the linear
//! conventions.
//!
//! Dispositions, by construction (mirroring the linear path):
//! - the original instance (unchanged, same arena ids) is
//!   `Modified`-into-itself for faces (legacy) and for edges/vertices;
//! - each copy's entities are `Generated` from their single copy-time source
//!   of the same kind;
//! - no `Preserved` claim is fabricated for moved copies, no `Deleted` and
//!   no `Unresolved` (a pattern deletes nothing and resolves everything);
//! - every subject journals under its own [`EntityKey`] kind, so a face
//!   index never collides with an edge or vertex sharing its number.
//!
//! Lineage semantics are original-only: a reference anchored before the
//! pattern chases to the original instance alone (`Bound`, never
//! `BoundMany`), because copies are `Generated` adjacency, not identity.
//! Copies are addressed through the pattern entry's own `operation_output`
//! anchors. The tests below pin `Bound` (not `BoundMany`), `Construction`
//! provenance throughout, expected rotated positions/carriers, unchanged
//! material, arena round-trip, checkpoint-restore truncation, and a subsequent
//! supported edit.
//!
//! The axis remains the origin-based axis-direction contract: patterns rotate
//! about the line through the origin along `axis_direction`. Rotated
//! placements rotate the operand and the axis consistently; the tests never
//! assume translation invariance about a fixed origin.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeSet, HashMap};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::journal_ops::{circular_pattern_journaled, solid_entity_keys};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityEvent, EntityKey, EntityKind, EntryPayload, OpId};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const COUNTS: [usize; 3] = [2, 3, 6];
const Z_AXIS: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn faces_of(topo: &Topology, solid: remus_topology::SolidId) -> BTreeSet<usize> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn edges_of(topo: &Topology, solid: remus_topology::SolidId) -> BTreeSet<usize> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn vertices_of(topo: &Topology, solid: remus_topology::SolidId) -> BTreeSet<usize> {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(Id::index)
        .collect()
}

fn compound_sets(
    topo: &Topology,
    compound: remus_topology::CompoundId,
) -> (BTreeSet<usize>, BTreeSet<usize>, BTreeSet<usize>) {
    let mut faces = BTreeSet::new();
    let mut edges = BTreeSet::new();
    let mut vertices = BTreeSet::new();
    for &solid in topo.compound(compound).unwrap().solids() {
        faces.extend(faces_of(topo, solid));
        edges.extend(edges_of(topo, solid));
        vertices.extend(vertices_of(topo, solid));
    }
    (faces, edges, vertices)
}

fn hollow_box(topo: &mut Topology, scale: f64) -> remus_topology::SolidId {
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::transform::transform_solid;
    let blank = make_box(topo, 3.0 * scale, 3.0 * scale, 3.0 * scale).unwrap();
    let tool = make_box(topo, scale, scale, scale).unwrap();
    transform_solid(topo, tool, &Mat4::translation(scale, scale, scale)).unwrap();
    boolean(topo, BooleanOp::Cut, blank, tool).unwrap()
}

/// Box placed away from the Z axis so circular copies are disjoint.
/// Side `10*scale`, offset `30*scale` along X: radius ~35*scale, chord for
/// N=6 (~35*scale) clears the ~17*scale diagonal at every scale.
fn disjoint_box(topo: &mut Topology, scale: f64) -> remus_topology::SolidId {
    let source = make_box(topo, 10.0 * scale, 10.0 * scale, 10.0 * scale).unwrap();
    remus_operations::transform::transform_solid(
        topo,
        source,
        &Mat4::translation(30.0 * scale, 0.0, 0.0),
    )
    .unwrap();
    source
}

/// Cylinder placed away from the Z axis so copies are disjoint.
/// Radius `2*scale`, height `5*scale`, offset `20*scale` along X.
fn disjoint_cylinder(topo: &mut Topology, scale: f64) -> remus_topology::SolidId {
    let source = make_cylinder(topo, 2.0 * scale, 5.0 * scale).unwrap();
    remus_operations::transform::transform_solid(
        topo,
        source,
        &Mat4::translation(20.0 * scale, 0.0, 0.0),
    )
    .unwrap();
    source
}

/// Hollow box placed away from the Z axis so copies are disjoint.
fn disjoint_hollow(topo: &mut Topology, scale: f64) -> remus_topology::SolidId {
    let source = hollow_box(topo, scale);
    remus_operations::transform::transform_solid(
        topo,
        source,
        &Mat4::translation(30.0 * scale, 0.0, 0.0),
    )
    .unwrap();
    source
}

fn pattern_entry(topo: &Topology, op: OpId) -> (Vec<EntityKey>, Vec<(EntityKey, EntityEvent)>) {
    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == op)
        .unwrap_or_else(|| panic!("journal has no entry for op {}", op.value()));
    assert_eq!(entry.kind(), "circular_pattern");
    let EntryPayload::Evolution {
        scope: _, events, ..
    } = entry.payload()
    else {
        panic!(
            "pattern op {} recorded a barrier, expected evolution",
            op.value()
        );
    };
    let mut subjects = Vec::new();
    let mut decoded = Vec::new();
    for (ordinal, event) in events {
        let key = topo.journal().key_of(*ordinal).unwrap();
        subjects.push(key);
        decoded.push((key, event.clone()));
    }
    (subjects, decoded)
}

/// Total census over one journaled circular pattern: every result F/E/V is a
/// subject exactly once with the construction-typed disposition, every source
/// is accounted for, and no coordinate matching was involved.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
fn assert_total_pattern_history(
    label: &str,
    topo: &Topology,
    source: remus_topology::SolidId,
    compound: remus_topology::CompoundId,
    op: OpId,
    source_faces: &BTreeSet<usize>,
    source_edges: &BTreeSet<usize>,
    source_vertices: &BTreeSet<usize>,
    count: usize,
) {
    let (result_faces, result_edges, result_vertices) = compound_sets(topo, compound);
    let members = topo.compound(compound).unwrap().solids().to_vec();
    assert_eq!(
        members[0], source,
        "{label}: the source solid is reused as the first instance"
    );
    assert_eq!(
        result_faces.len(),
        source_faces.len() * count,
        "{label}: every instance carries the source faces"
    );
    assert_eq!(
        result_edges.len(),
        source_edges.len() * count,
        "{label}: every instance carries the source edges"
    );
    assert_eq!(
        result_vertices.len(),
        source_vertices.len() * count,
        "{label}: every instance carries the source vertices"
    );

    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == op)
        .unwrap();
    let EntryPayload::Evolution {
        scope: _, events, ..
    } = entry.payload()
    else {
        panic!("{label}: pattern entry is a barrier");
    };
    let origin = match entry.payload() {
        EntryPayload::Evolution { origin, .. } => *origin,
        _ => unreachable!(),
    };
    assert_eq!(
        origin,
        remus_topology::journal::RecordedOrigin::Construction,
        "{label}: pattern history is construction-derived"
    );

    let mut seen: BTreeSet<EntityKey> = BTreeSet::new();
    for (ordinal, _) in events {
        let key = topo.journal().key_of(*ordinal).unwrap();
        assert!(seen.insert(key), "{label}: duplicate subject {key:?}");
    }
    let live: BTreeSet<EntityKey> = result_faces
        .iter()
        .map(|&i| EntityKey::face(i))
        .chain(result_edges.iter().map(|&i| EntityKey::edge(i)))
        .chain(result_vertices.iter().map(|&i| EntityKey::vertex(i)))
        .collect();
    let omitted: Vec<EntityKey> = live.difference(&seen).copied().collect();
    assert!(
        omitted.is_empty(),
        "{label}: result entities with no lineage record: {omitted:?}"
    );
    let phantom: Vec<EntityKey> = seen.difference(&live).copied().collect();
    assert!(
        phantom.is_empty(),
        "{label}: lineage subjects not in the result: {phantom:?}"
    );

    let mut generated_sources: HashMap<EntityKey, Vec<EntityKey>> = HashMap::new();
    for (key, event) in pattern_entry(topo, op).1 {
        match event {
            EntityEvent::Modified { from } => {
                let from_key = topo.journal().key_of(from).unwrap();
                assert_eq!(
                    key, from_key,
                    "{label}: Modified {key:?} must be the original instance into itself"
                );
                assert!(
                    (key.kind == EntityKind::Face && source_faces.contains(&key.index))
                        || (key.kind == EntityKind::Edge && source_edges.contains(&key.index))
                        || (key.kind == EntityKind::Vertex && source_vertices.contains(&key.index)),
                    "{label}: Modified {key:?} is not a source entity"
                );
            }
            EntityEvent::Generated { sources } => {
                assert_eq!(
                    sources.len(),
                    1,
                    "{label}: copy {key:?} must name exactly its copy-time source, got {sources:?}"
                );
                let src = topo.journal().key_of(sources[0]).unwrap();
                assert_eq!(
                    src.kind, key.kind,
                    "{label}: copy {key:?} names a foreign-kind source {src:?}"
                );
                assert!(
                    (key.kind == EntityKind::Face && source_faces.contains(&src.index))
                        || (key.kind == EntityKind::Edge && source_edges.contains(&src.index))
                        || (key.kind == EntityKind::Vertex && source_vertices.contains(&src.index)),
                    "{label}: copy {key:?} names {src:?}, not a source entity"
                );
                assert!(
                    !matches!(
                        (
                            &key.kind,
                            source_faces.contains(&key.index),
                            source_edges.contains(&key.index),
                            source_vertices.contains(&key.index)
                        ),
                        (EntityKind::Face, true, _, _)
                            | (EntityKind::Edge, _, true, _)
                            | (EntityKind::Vertex, _, _, true)
                    ),
                    "{label}: copy subject {key:?} collides with the original instance"
                );
                generated_sources.entry(src).or_default().push(key);
            }
            EntityEvent::Preserved { .. } => {
                panic!("{label}: pattern must not fabricate Preserved for moved copies")
            }
            EntityEvent::Merged { .. } => {
                panic!("{label}: pattern copies are one-to-one; a merge would be invented")
            }
            EntityEvent::Deleted => panic!("{label}: a pattern deletes nothing"),
            EntityEvent::Unresolved { .. } => {
                panic!("{label}: copy-time lineage leaves nothing unresolved")
            }
        }
    }
    for &src in source_faces {
        let copies = generated_sources
            .get(&EntityKey::face(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies,
            count - 1,
            "{label}: source face {src} must generate {n} copies, got {copies}",
            n = count - 1
        );
    }
    for &src in source_edges {
        let copies = generated_sources
            .get(&EntityKey::edge(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies,
            count - 1,
            "{label}: source edge {src} must generate {n} copies",
            n = count - 1
        );
    }
    for &src in source_vertices {
        let copies = generated_sources
            .get(&EntityKey::vertex(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies,
            count - 1,
            "{label}: source vertex {src} must generate {n} copies",
            n = count - 1
        );
    }
}

fn rotate_point_z(point: Point3, angle: f64) -> Point3 {
    let (s, c) = angle.sin_cos();
    Point3::new(
        c.mul_add(point.x(), -(s * point.y())),
        s.mul_add(point.x(), c * point.y()),
        point.z(),
    )
}

fn solid_vertex_positions(topo: &Topology, solid: remus_topology::SolidId) -> Vec<Point3> {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(|vid| topo.vertex(vid).unwrap().point())
        .collect()
}

/// Every copy's vertices are the source vertices rotated about Z by
/// `i * step`. Set-matched within a scale-relative tolerance: the check is a
/// geometry oracle over the pattern rotation, never the history attribution
/// (which the journal census above pins by copy-time maps).
fn assert_rotated_vertex_sets(
    label: &str,
    topo: &Topology,
    source: remus_topology::SolidId,
    compound: remus_topology::CompoundId,
    count: usize,
    scale: f64,
) {
    let source_points = solid_vertex_positions(topo, source);
    let members = topo.compound(compound).unwrap().solids().to_vec();
    assert_eq!(members.len(), count, "{label}: compound size");
    let angle_step = 2.0 * std::f64::consts::PI / (count as f64);
    let tol = 1e-6 * scale.max(1e-12);
    for (i, &solid) in members.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let angle = angle_step * (i as f64);
        let expected: Vec<Point3> = source_points
            .iter()
            .map(|&p| rotate_point_z(p, angle))
            .collect();
        let actual = solid_vertex_positions(topo, solid);
        assert_eq!(
            actual.len(),
            expected.len(),
            "{label}: instance {i} vertex count"
        );
        for want in &expected {
            let found = actual.iter().any(|got| (*got - *want).length() <= tol);
            assert!(
                found,
                "{label}: instance {i} missing rotated vertex {want:?} (tol {tol:e})"
            );
        }
    }
}

fn assert_box_carriers(label: &str, topo: &Topology, solid: remus_topology::SolidId) {
    for fid in solid_faces(topo, solid).unwrap() {
        let surface = topo.face(fid).unwrap().surface().clone();
        assert!(
            matches!(surface, remus_topology::face::FaceSurface::Plane { .. }),
            "{label}: box face must stay planar, got {surface:?}"
        );
    }
}

fn assert_cylinder_carriers(
    label: &str,
    topo: &Topology,
    solid: remus_topology::SolidId,
    expected_radius: f64,
    scale: f64,
) {
    use remus_topology::face::FaceSurface;
    let mut cylinders = 0;
    let mut planes = 0;
    for fid in solid_faces(topo, solid).unwrap() {
        match topo.face(fid).unwrap().surface() {
            FaceSurface::Cylinder(c) => {
                cylinders += 1;
                let rel = (c.radius() - expected_radius).abs() / expected_radius;
                assert!(
                    rel < 1e-9,
                    "{label}: cylinder radius {} != {expected_radius} (rel {rel})",
                    c.radius()
                );
                // Pattern rotation about Z preserves the Z axis direction.
                let axis = c.axis();
                assert!(
                    (axis.x().abs() < 1e-9)
                        && (axis.y().abs() < 1e-9)
                        && ((axis.z() - 1.0).abs() < 1e-9 || (axis.z() + 1.0).abs() < 1e-9),
                    "{label}: cylinder axis must stay ±Z, got {axis:?}"
                );
                let _ = scale;
            }
            FaceSurface::Plane { .. } => planes += 1,
            other => panic!("{label}: cylinder solid carries unexpected {other:?}"),
        }
    }
    assert_eq!(cylinders, 1, "{label}: one lateral cylinder");
    assert_eq!(planes, 2, "{label}: two planar caps");
}

fn assert_volumes_equal(
    label: &str,
    topo: &Topology,
    compound: remus_topology::CompoundId,
    expected_each: f64,
    scale: f64,
) {
    let mut total = 0.0;
    for &solid in topo.compound(compound).unwrap().solids() {
        let vol = remus_operations::measure::solid_volume(topo, solid, 0.01 * scale).unwrap();
        let rel = (vol - expected_each).abs() / expected_each;
        assert!(
            rel < 1e-6,
            "{label}: instance volume {vol} != {expected_each} (rel {rel})"
        );
        total += vol;
        let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
        assert!(report.is_valid(), "{label}: {report:?}");
    }
    let expected_total = expected_each * (topo.compound(compound).unwrap().solids().len() as f64);
    let rel = (total - expected_total).abs() / expected_total;
    assert!(
        rel < 1e-9,
        "{label}: total volume {total} != {expected_total}"
    );
}

#[test]
fn circular_pattern_journal_covers_every_result_entity() {
    for scale in SCALES {
        // Disjoint boxes at every count: offset from the axis.
        for count in COUNTS {
            let mut topo = Topology::new();
            let source = disjoint_box(&mut topo, scale);
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            assert_eq!(source_faces.len(), 6);
            assert_eq!(source_edges.len(), 12);
            assert_eq!(source_vertices.len(), 8);
            let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, count).unwrap();
            let label = format!("disjoint box count {count} at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                count,
            );
            let (result_faces, _, _) = compound_sets(&topo, journaled.compound);
            assert!(
                journaled
                    .map
                    .accounts_for_result(result_faces.iter().copied()),
                "{label}: face map must account for every result face"
            );
            assert!(
                journaled
                    .map
                    .is_construction_resolved_for_result(result_faces.iter().copied()),
                "{label}: face map must be construction-resolved"
            );
            let side = 10.0 * scale;
            assert_volumes_equal(&label, &topo, journaled.compound, side.powi(3), scale);
            assert_rotated_vertex_sets(&label, &topo, source, journaled.compound, count, scale);
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                assert_box_carriers(&label, &topo, solid);
            }
            // BBox centers rotate with the instances: independent material
            // oracle beyond the journal census.
            let members = topo.compound(journaled.compound).unwrap().solids().to_vec();
            let center_of = |solid| {
                let bbox = remus_operations::measure::solid_bounding_box(&topo, solid).unwrap();
                Point3::new(
                    0.5 * (bbox.min.x() + bbox.max.x()),
                    0.5 * (bbox.min.y() + bbox.max.y()),
                    0.5 * (bbox.min.z() + bbox.max.z()),
                )
            };
            let source_center = center_of(source);
            let angle_step = 2.0 * std::f64::consts::PI / (count as f64);
            for (i, &solid) in members.iter().enumerate() {
                #[allow(clippy::cast_precision_loss)]
                let want = rotate_point_z(source_center, angle_step * (i as f64));
                let got = center_of(solid);
                let dist = (got - want).length();
                assert!(
                    dist <= 1e-6 * scale,
                    "{label}: instance {i} center {got:?} != rotated {want:?}"
                );
            }
        }
        // Touching boxes: origin placement, count 2 pinwheels at a point.
        {
            let mut topo = Topology::new();
            let side = 10.0 * scale;
            let source = make_box(&mut topo, side, side, side).unwrap();
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, 2).unwrap();
            let label = format!("touching box count 2 at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                2,
            );
            assert_volumes_equal(&label, &topo, journaled.compound, side.powi(3), scale);
            assert_rotated_vertex_sets(&label, &topo, source, journaled.compound, 2, scale);
        }
        // Curved, seam-bearing source: offset cylinder (lateral seam + caps).
        for count in COUNTS {
            let mut topo = Topology::new();
            let radius = 2.0 * scale;
            let height = 5.0 * scale;
            let source = disjoint_cylinder(&mut topo, scale);
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            assert_eq!(source_faces.len(), 3, "cylinder has lateral + 2 caps");
            assert_eq!(source_edges.len(), 3, "cylinder has 2 rims + seam");
            assert_eq!(source_vertices.len(), 2, "cylinder seam endpoints");
            let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, count).unwrap();
            let label = format!("disjoint cylinder count {count} at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                count,
            );
            let expected = std::f64::consts::PI * radius * radius * height;
            assert_volumes_equal(&label, &topo, journaled.compound, expected, scale);
            assert_rotated_vertex_sets(&label, &topo, source, journaled.compound, count, scale);
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                assert_cylinder_carriers(&label, &topo, solid, radius, scale);
            }
        }
        // Cavity-bearing source: disjoint hollow box with one inner shell.
        for count in COUNTS {
            let mut topo = Topology::new();
            let source = disjoint_hollow(&mut topo, scale);
            assert_eq!(
                topo.solid(source).unwrap().inner_shells().len(),
                1,
                "hollow box must carry one cavity shell"
            );
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            assert_eq!(source_faces.len(), 12);
            assert_eq!(source_edges.len(), 24);
            assert_eq!(source_vertices.len(), 16);
            let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, count).unwrap();
            let label = format!("disjoint hollow count {count} at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                count,
            );
            let expected_each = 26.0 * scale.powi(3);
            assert_volumes_equal(&label, &topo, journaled.compound, expected_each, scale);
            assert_rotated_vertex_sets(&label, &topo, source, journaled.compound, count, scale);
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                assert_eq!(
                    topo.solid(solid).unwrap().inner_shells().len(),
                    1,
                    "{label}: copies must preserve the cavity shell"
                );
            }
        }
        // Touching cavity: origin hollow, count 2.
        {
            let mut topo = Topology::new();
            let source = hollow_box(&mut topo, scale);
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, 2).unwrap();
            let label = format!("touching hollow count 2 at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                2,
            );
            assert_volumes_equal(
                &label,
                &topo,
                journaled.compound,
                26.0 * scale.powi(3),
                scale,
            );
        }
    }
}

fn rotate_vec_by_matrix(mat: &Mat4, v: Vec3) -> Vec3 {
    let m = &mat.0;
    Vec3::new(
        m[0][0].mul_add(v.x(), m[0][1].mul_add(v.y(), m[0][2] * v.z())),
        m[1][0].mul_add(v.x(), m[1][1].mul_add(v.y(), m[1][2] * v.z())),
        m[2][0].mul_add(v.x(), m[2][1].mul_add(v.y(), m[2][2] * v.z())),
    )
}

#[test]
fn circular_pattern_history_survives_consistent_rotation() {
    // Rotate the operand and the axis by the same rigid rotation: the relative
    // pattern geometry is preserved, while a fixed-origin assumption would
    // misplace every copy. Translation along the axis is also preserved
    // (radius unchanged); arbitrary translation is not.
    let placement = Mat4::rotation_x(0.6) * Mat4::rotation_z(0.7);
    let axis = rotate_vec_by_matrix(&placement, Z_AXIS);
    let mut topo = Topology::new();
    let source = disjoint_box(&mut topo, 1.0);
    remus_operations::transform::transform_solid(&mut topo, source, &placement).unwrap();
    // Slide along the (rotated) axis: radius to the axis is unchanged.
    let along = axis * 15.0;
    remus_operations::transform::transform_solid(
        &mut topo,
        source,
        &Mat4::translation(along.x(), along.y(), along.z()),
    )
    .unwrap();
    let source_faces = faces_of(&topo, source);
    let source_edges = edges_of(&topo, source);
    let source_vertices = vertices_of(&topo, source);
    let journaled = circular_pattern_journaled(&mut topo, source, axis, 3).unwrap();
    assert_total_pattern_history(
        "consistently rotated box count 3",
        &topo,
        source,
        journaled.compound,
        journaled.op,
        &source_faces,
        &source_edges,
        &source_vertices,
        3,
    );
    assert_volumes_equal("rotated box", &topo, journaled.compound, 1000.0, 1.0);
    for &solid in topo.compound(journaled.compound).unwrap().solids() {
        assert_box_carriers("rotated box", &topo, solid);
    }
    // Same consistent rotation for a seam-bearing cylinder.
    let mut topo = Topology::new();
    let source = disjoint_cylinder(&mut topo, 1.0);
    remus_operations::transform::transform_solid(&mut topo, source, &placement).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        source,
        &Mat4::translation(along.x(), along.y(), along.z()),
    )
    .unwrap();
    let source_faces = faces_of(&topo, source);
    let source_edges = edges_of(&topo, source);
    let source_vertices = vertices_of(&topo, source);
    let journaled = circular_pattern_journaled(&mut topo, source, axis, 3).unwrap();
    assert_total_pattern_history(
        "consistently rotated cylinder count 3",
        &topo,
        source,
        journaled.compound,
        journaled.op,
        &source_faces,
        &source_edges,
        &source_vertices,
        3,
    );
    let expected = std::f64::consts::PI * 4.0 * 5.0;
    assert_volumes_equal("rotated cylinder", &topo, journaled.compound, expected, 1.0);
}

/// Construction-recorded anchor over every entity of `solid`, mirroring
/// `regress_pattern_evolution_fev.rs`: each live entity becomes a `Generated`
/// subject with no sources, so `operation_output(anchor, kind, index)`
/// addresses the `index`-th entity of that kind in deterministic order.
struct Anchor {
    op: OpId,
    faces: Vec<EntityKey>,
    edges: Vec<EntityKey>,
    vertices: Vec<EntityKey>,
}

fn anchor_all_entities(topo: &mut Topology, solid: remus_topology::SolidId) -> Anchor {
    use remus_topology::journal::{EventDraft, EvolutionDraft};
    let keys = solid_entity_keys(topo, solid).unwrap();
    let pending = topo.journal_begin("anchor");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for &key in &keys {
        draft.push(
            key,
            EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    let op = topo.journal_record_evolution(pending, draft).unwrap();
    let mut anchor = Anchor {
        op,
        faces: Vec::new(),
        edges: Vec::new(),
        vertices: Vec::new(),
    };
    for key in keys {
        match key.kind {
            EntityKind::Face => anchor.faces.push(key),
            EntityKind::Edge => anchor.edges.push(key),
            EntityKind::Vertex => anchor.vertices.push(key),
        }
    }
    anchor
}

#[test]
fn circular_pattern_lineage_is_original_only_and_covers_all_kinds() {
    let mut topo = Topology::new();
    let source = disjoint_box(&mut topo, 1.0);
    let anchor = anchor_all_entities(&mut topo, source);
    let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, 3).unwrap();
    for (kind, keys) in [
        (EntityKind::Face, &anchor.faces),
        (EntityKind::Edge, &anchor.edges),
        (EntityKind::Vertex, &anchor.vertices),
    ] {
        for (index, _) in keys.iter().enumerate() {
            match resolve(
                &topo,
                &PersistentRef::operation_output(anchor.op, kind, index),
            ) {
                Resolution::Bound { entity, provenance } => {
                    assert_eq!(provenance, Provenance::Construction);
                    assert_eq!(entity.kind, kind);
                    let source_key = keys[index];
                    assert_eq!(
                        entity, source_key,
                        "source {kind:?}/{index} must stay bound to the original instance"
                    );
                }
                other => panic!("source {kind:?}/{index} must stay Bound, got {other:?}"),
            }
        }
    }
    let (result_faces, result_edges, result_vertices) = compound_sets(&topo, journaled.compound);
    for (kind, live) in [
        (EntityKind::Face, &result_faces),
        (EntityKind::Edge, &result_edges),
        (EntityKind::Vertex, &result_vertices),
    ] {
        let entry = topo
            .journal()
            .entries()
            .iter()
            .find(|entry| entry.op() == journaled.op)
            .unwrap();
        let EntryPayload::Evolution { events, .. } = entry.payload() else {
            panic!("pattern entry is a barrier");
        };
        let outputs: Vec<EntityKey> = events
            .iter()
            .filter_map(|(subject, event)| {
                if matches!(event, EntityEvent::Deleted) {
                    return None;
                }
                let key = topo.journal().key_of(*subject).unwrap();
                (key.kind == kind).then_some(key)
            })
            .collect();
        assert_eq!(
            outputs.len(),
            live.len(),
            "pattern entry must expose every result {kind:?}"
        );
        let mut resolved = BTreeSet::new();
        for index in 0..outputs.len() {
            match resolve(
                &topo,
                &PersistentRef::operation_output(journaled.op, kind, index),
            ) {
                Resolution::Bound { entity, provenance } => {
                    assert_eq!(provenance, Provenance::Construction);
                    assert!(resolved.insert(entity), "outputs must be distinct");
                }
                other => panic!("pattern output {kind:?}/{index}: {other:?}"),
            }
        }
        let expected: BTreeSet<EntityKey> = live
            .iter()
            .map(|&i| match kind {
                EntityKind::Face => EntityKey::face(i),
                EntityKind::Edge => EntityKey::edge(i),
                EntityKind::Vertex => EntityKey::vertex(i),
            })
            .collect();
        assert_eq!(
            resolved, expected,
            "pattern {kind:?} outputs must census the result"
        );
    }
}

#[test]
fn circular_pattern_refs_survive_arena_round_trip_restore_and_subsequent_edit() {
    let mut topo = Topology::new();
    let source = disjoint_box(&mut topo, 1.0);
    let anchor = anchor_all_entities(&mut topo, source);
    let journaled = circular_pattern_journaled(&mut topo, source, Z_AXIS, 2).unwrap();
    let members = topo.compound(journaled.compound).unwrap().solids().to_vec();
    assert_eq!(members.len(), 2);
    let bytes = remus_io::arena_io::serialize_document(
        &topo,
        &members,
        std::slice::from_ref(&journaled.compound),
    )
    .unwrap();
    let mut restored = Topology::new();
    make_box(&mut restored, 1.0, 1.0, 1.0).unwrap();
    let document = remus_io::arena_io::deserialize_document(&bytes, &mut restored).unwrap();
    assert_eq!(document.compounds.len(), 1);
    assert_eq!(document.solids.len(), 2);
    for (kind, count) in [
        (EntityKind::Face, anchor.faces.len()),
        (EntityKind::Edge, anchor.edges.len()),
        (EntityKind::Vertex, anchor.vertices.len()),
    ] {
        for index in 0..count {
            match resolve(
                &restored,
                &PersistentRef::operation_output(anchor.op, kind, index),
            ) {
                Resolution::Bound { entity, provenance } => {
                    assert_eq!(provenance, Provenance::Construction);
                    assert_eq!(entity.kind, kind);
                }
                other => panic!("restored source {kind:?}/{index}: {other:?}"),
            }
        }
        let entry = restored
            .journal()
            .entries()
            .iter()
            .find(|entry| entry.op() == journaled.op)
            .unwrap_or_else(|| panic!("restored journal lost pattern op"));
        let EntryPayload::Evolution { events, .. } = entry.payload() else {
            panic!("restored pattern entry is a barrier");
        };
        let outputs = events
            .iter()
            .filter(|(subject, event)| {
                !matches!(event, EntityEvent::Deleted)
                    && restored
                        .journal()
                        .key_of(*subject)
                        .is_some_and(|key| key.kind == kind)
            })
            .count();
        for index in 0..outputs {
            match resolve(
                &restored,
                &PersistentRef::operation_output(journaled.op, kind, index),
            ) {
                Resolution::Bound { provenance, .. } => {
                    assert_eq!(provenance, Provenance::Construction);
                }
                other => panic!("restored pattern {kind:?}/{index}: {other:?}"),
            }
        }
    }
    let snapshot = topo.clone();
    let pre_restore_faces = faces_of(&topo, members[0]);
    topo.restore_preserving_handle_slots(&snapshot);
    assert_eq!(faces_of(&topo, members[0]), pre_restore_faces);
    let moved_face = solid_faces(&topo, members[1])
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.9 || normal.x() < -0.9)
        })
        .unwrap_or_else(|| solid_faces(&topo, members[1]).unwrap()[0]);
    let edited = remus_operations::journal_ops::move_faces_journaled(
        &mut topo,
        members[1],
        &[moved_face],
        1.0,
    )
    .unwrap();
    for (kind, count) in [
        (EntityKind::Face, anchor.faces.len()),
        (EntityKind::Edge, anchor.edges.len()),
        (EntityKind::Vertex, anchor.vertices.len()),
    ] {
        for index in 0..count {
            let resolution = resolve(
                &topo,
                &PersistentRef::operation_output(anchor.op, kind, index),
            );
            match resolution {
                Resolution::Bound { provenance, .. } => {
                    assert_eq!(provenance, Provenance::Construction);
                }
                Resolution::UnresolvedAcrossOperation { .. } => {
                    let _ = (kind, index);
                }
                other => panic!("post-edit source {kind:?}/{index}: {other:?}"),
            }
        }
    }
    let _ = edited;
    let _ = Point3::new(0.0, 0.0, 0.0);
}

#[test]
fn circular_pattern_typed_refusals_preserve_geometry_history_and_resolution() {
    use remus_operations::journal_ops::circular_pattern_journaled;
    // Foreign handle: a retired solid refuses before publishing history.
    {
        let mut topo = Topology::new();
        let source = disjoint_box(&mut topo, 1.0);
        let anchor = anchor_all_entities(&mut topo, source);
        let retired = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        topo.delete_solid(retired).unwrap();
        let before = topo.journal().snapshot();
        let geometry = remus_io::step::writer::write_step(&topo, &[source]).unwrap();
        let failed = circular_pattern_journaled(&mut topo, retired, Z_AXIS, 2);
        assert!(failed.is_err(), "retired handle must refuse");
        assert_eq!(topo.journal().snapshot().entries, before.entries);
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[source]).unwrap(),
            geometry
        );
        for (kind, count) in [
            (EntityKind::Face, anchor.faces.len()),
            (EntityKind::Edge, anchor.edges.len()),
            (EntityKind::Vertex, anchor.vertices.len()),
        ] {
            for index in 0..count {
                assert!(
                    matches!(
                        resolve(
                            &topo,
                            &PersistentRef::operation_output(anchor.op, kind, index)
                        ),
                        Resolution::Bound { .. }
                    ),
                    "earlier resolution must survive a foreign-handle refusal"
                );
            }
        }
        assert!(topo.solid(retired).is_err(), "stale handle must not reuse");
    }
    // Count refusals: 0 and 1 both need at least 2.
    for count in [0_usize, 1] {
        let mut topo = Topology::new();
        let source = disjoint_box(&mut topo, 1.0);
        let before = topo.journal().snapshot();
        let failed = circular_pattern_journaled(&mut topo, source, Z_AXIS, count);
        assert!(failed.is_err(), "count {count} refusal must fail");
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "count {count} refusal published history"
        );
        let volume = remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() < 1e-6,
            "count {count} changed source geometry"
        );
    }
    // Zero and non-finite axes refuse without publishing history.
    for (axis, tag) in [
        (Vec3::new(0.0, 0.0, 0.0), "zero"),
        (Vec3::new(f64::NAN, 0.0, 0.0), "nan"),
        (Vec3::new(1.0, 0.0, f64::INFINITY), "infinite"),
    ] {
        let mut topo = Topology::new();
        let source = disjoint_box(&mut topo, 1.0);
        let before = topo.journal().snapshot();
        let failed = circular_pattern_journaled(&mut topo, source, axis, 2);
        assert!(failed.is_err(), "{tag} axis refusal must fail");
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "{tag} axis refusal published history"
        );
        let volume = remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() < 1e-6,
            "{tag} axis changed source geometry"
        );
    }
    // Material overlap refuses with the typed error and rolls back.
    {
        let mut topo = Topology::new();
        let source = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
        let before_counts = [
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
            topo.num_compounds(),
        ];
        let before = topo.journal().snapshot();
        let failed = circular_pattern_journaled(&mut topo, source, Z_AXIS, 2);
        match failed {
            Err(remus_operations::OperationsError::PatternInstancesOverlap { .. }) => {}
            other => panic!("overlap must refuse typed, got {other:?}"),
        }
        assert_eq!(
            [
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
                topo.num_compounds(),
            ],
            before_counts,
            "overlap refusal must retire every staged copy"
        );
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "overlap refusal published history"
        );
        assert!(topo.solid(source).is_ok(), "input must survive rollback");
    }
}
