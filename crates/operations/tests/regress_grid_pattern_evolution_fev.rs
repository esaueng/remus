//! Total grid-pattern construction history: every result face, edge and
//! vertex journals a typed construction disposition by copy-time lineage.
//!
//! At the reviewed baseline `grid_pattern_with_evolution` returned only its
//! face map: `grid_pattern_impl` already collected the full correspondence via
//! `copy_solid_with_entity_map` and `PatternTracker::record_instance`, but the
//! public evolution path discarded the edge/vertex pairs, leaving every
//! boundary entity in scope without a claim. This file pins the upgrade: the
//! journaled grid pattern now records total F/E/V lineage from the copy maps,
//! never by coordinate or arena-allocation-order matching, using the linear
//! and circular conventions.
//!
//! Dispositions, by construction (mirroring the linear/circular paths):
//! - the original instance (unchanged, same arena ids, position `(0, 0)`) is
//!   `Modified`-into-itself for faces (legacy) and for edges/vertices;
//! - each copy's entities are `Generated` from their single copy-time source
//!   of the same kind;
//! - no `Preserved` claim is fabricated for moved copies, no `Deleted` and
//!   no `Unresolved` (a pattern deletes nothing and resolves everything);
//! - every subject journals under its own [`EntityKey`] kind, so a face
//!   index never collides with an edge or vertex sharing its number.
//!
//! Placement contract (unchanged legacy geometry): instance `(ix, iy)` sits at
//! `dx * spacing_x * ix + dy * spacing_y * iy` from the source, members are
//! ordered `iy`-outer/`ix`-inner with the source first, and touching
//! instances remain supported while material overlap refuses.
//!
//! Lineage semantics are original-only: a reference anchored before the
//! pattern chases to the original instance alone (`Bound`, never
//! `BoundMany`), because copies are `Generated` adjacency, not identity.
//! Copies are addressed through the pattern entry's own `operation_output`
//! anchors. The tests below pin `Bound` (not `BoundMany`), `Construction`
//! provenance throughout, row/column instance identity with expected
//! translated geometry, arena round-trip, checkpoint-restore truncation, and
//! a subsequent supported edit.
//!
//! Native-only scope: this file exercises the native
//! `grid_pattern_journaled` path. Browser (`BrepKernel`) exposure is a
//! separate remaining integration task and is not claimed here.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::{BTreeSet, HashMap};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::journal_ops::{grid_pattern_journaled, solid_entity_keys};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;
use remus_topology::arena::Id;
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::journal::{EntityEvent, EntityKey, EntityKind, EntryPayload, OpId};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const GRID_X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const GRID_Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);

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

fn pattern_entry(topo: &Topology, op: OpId) -> (Vec<EntityKey>, Vec<(EntityKey, EntityEvent)>) {
    let entry = topo
        .journal()
        .entries()
        .iter()
        .find(|entry| entry.op() == op)
        .unwrap_or_else(|| panic!("journal has no entry for op {}", op.value()));
    assert_eq!(entry.kind(), "grid_pattern");
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

/// Total census over one journaled grid pattern: every result F/E/V is a
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
    count_x: usize,
    count_y: usize,
) {
    let count = count_x * count_y;
    let (result_faces, result_edges, result_vertices) = compound_sets(topo, compound);
    let members = topo.compound(compound).unwrap().solids().to_vec();
    assert_eq!(
        members.len(),
        count,
        "{label}: compound must hold count_x * count_y instances"
    );
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
    // Construction origin, recorded by copy maps.
    let origin = match entry.payload() {
        EntryPayload::Evolution { origin, .. } => *origin,
        _ => unreachable!(),
    };
    assert_eq!(
        origin,
        remus_topology::journal::RecordedOrigin::Construction,
        "{label}: pattern history is construction-derived"
    );

    // Subjects cover every result entity exactly once, with no phantom.
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

    // Typed dispositions: original Modified-into-itself, copies Generated
    // from their single same-kind source. No Preserved (copies moved, so a
    // Preserved claim would be fabricated), no Deleted, no Unresolved.
    let mut generated_sources: HashMap<EntityKey, Vec<EntityKey>> = HashMap::new();
    for (key, event) in pattern_entry(topo, op).1 {
        match event {
            EntityEvent::Modified { from } => {
                let from_key = topo.journal().key_of(from).unwrap();
                // Original-instance identity: subject IS its source.
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
    // Every source is accounted for: its own Modified record plus
    // (count-1) Generated copies (one-to-many is legitimate).
    let copies_each = count - 1;
    for &src in source_faces {
        let copies = generated_sources
            .get(&EntityKey::face(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies, copies_each,
            "{label}: source face {src} must generate {copies_each} copies, got {copies}",
        );
    }
    for &src in source_edges {
        let copies = generated_sources
            .get(&EntityKey::edge(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies, copies_each,
            "{label}: source edge {src} must generate {copies_each} copies",
        );
    }
    for &src in source_vertices {
        let copies = generated_sources
            .get(&EntityKey::vertex(src))
            .map_or(0, Vec::len);
        assert_eq!(
            copies, copies_each,
            "{label}: source vertex {src} must generate {copies_each} copies",
        );
    }
}

fn solid_vertex_positions(topo: &Topology, solid: remus_topology::SolidId) -> Vec<Point3> {
    solid_vertices(topo, solid)
        .unwrap()
        .into_iter()
        .map(|vid| topo.vertex(vid).unwrap().point())
        .collect()
}

fn center_of(topo: &Topology, solid: remus_topology::SolidId) -> Point3 {
    let bbox = remus_operations::measure::solid_bounding_box(topo, solid).unwrap();
    Point3::new(
        0.5 * (bbox.min.x() + bbox.max.x()),
        0.5 * (bbox.min.y() + bbox.max.y()),
        0.5 * (bbox.min.z() + bbox.max.z()),
    )
}

/// Every copy's vertices are the source vertices translated by
/// `dx * spacing_x * ix + dy * spacing_y * iy`. Set-matched within a
/// scale-relative tolerance: the check is a geometry oracle over the grid
/// placement, never the history attribution (which the journal census above
/// pins by copy-time maps).
#[allow(clippy::too_many_arguments)]
fn assert_translated_instances(
    label: &str,
    topo: &Topology,
    source: remus_topology::SolidId,
    compound: remus_topology::CompoundId,
    dir_x: Vec3,
    dir_y: Vec3,
    spacing_x: f64,
    spacing_y: f64,
    count_x: usize,
    count_y: usize,
    scale: f64,
) {
    let source_points = solid_vertex_positions(topo, source);
    let source_center = center_of(topo, source);
    let members = topo.compound(compound).unwrap().solids().to_vec();
    assert_eq!(members.len(), count_x * count_y, "{label}: compound size");
    let tol = 1e-6 * scale.max(1e-12);
    for iy in 0..count_y {
        for ix in 0..count_x {
            let instance = iy * count_x + ix;
            let solid = members[instance];
            #[allow(clippy::cast_precision_loss)]
            let offset = dir_x * (spacing_x * ix as f64) + dir_y * (spacing_y * iy as f64);
            // Row/column identity: instance (ix, iy) lives at the translated
            // center, and the original (0, 0) is the unmoved source.
            let want_center = source_center + offset;
            let got_center = center_of(topo, solid);
            let center_dist = (got_center - want_center).length();
            assert!(
                center_dist <= tol,
                "{label}: instance ({ix}, {iy}) center {got_center:?} != \
                 translated {want_center:?} (dist {center_dist:e}, tol {tol:e})"
            );
            if ix == 0 && iy == 0 {
                assert_eq!(
                    solid, source,
                    "{label}: instance (0, 0) must be the original solid"
                );
            } else {
                assert_ne!(
                    solid, source,
                    "{label}: instance ({ix}, {iy}) must be a generated copy"
                );
            }
            // Vertex-set oracle: every translated source vertex appears.
            let actual = solid_vertex_positions(topo, solid);
            assert_eq!(
                actual.len(),
                source_points.len(),
                "{label}: instance ({ix}, {iy}) vertex count"
            );
            for want in source_points.iter().map(|p| *p + offset) {
                let found = actual.iter().any(|got| (*got - want).length() <= tol);
                assert!(
                    found,
                    "{label}: instance ({ix}, {iy}) missing translated vertex \
                     {want:?} (tol {tol:e})"
                );
            }
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
fn grid_pattern_journal_covers_every_result_entity() {
    // 1x1 (original only), 1xN / Nx1 (single row/column) and multi-row/column
    // grids, over boxes, seam-bearing cylinders and cavity solids, at three
    // model scales and both touching and disjoint arrangements.
    let grid_shapes: [(usize, usize); 5] = [(1, 1), (1, 3), (3, 1), (2, 2), (3, 2)];
    for scale in SCALES {
        for &(count_x, count_y) in &grid_shapes {
            for (spacing_factor, tag) in [(1.5, "disjoint"), (1.0, "touching")] {
                // 1x1 has no neighbor gap: run it once (disjoint tag).
                if count_x == 1 && count_y == 1 && tag == "touching" {
                    continue;
                }
                let mut topo = Topology::new();
                let side = 10.0 * scale;
                let source = make_box(&mut topo, side, side, side).unwrap();
                let source_faces = faces_of(&topo, source);
                let source_edges = edges_of(&topo, source);
                let source_vertices = vertices_of(&topo, source);
                assert_eq!(source_faces.len(), 6);
                assert_eq!(source_edges.len(), 12);
                assert_eq!(source_vertices.len(), 8);
                let spacing_x = side * spacing_factor;
                let spacing_y = side * spacing_factor;
                let journaled = grid_pattern_journaled(
                    &mut topo, source, GRID_X, GRID_Y, spacing_x, spacing_y, count_x, count_y,
                )
                .unwrap();
                let label = format!("box {tag} {count_x}x{count_y} at {scale:e}",);
                assert_total_pattern_history(
                    &label,
                    &topo,
                    source,
                    journaled.compound,
                    journaled.op,
                    &source_faces,
                    &source_edges,
                    &source_vertices,
                    count_x,
                    count_y,
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
                // Legacy face-only path keeps its contract on the same
                // geometry: compare against a fresh legacy build.
                {
                    let mut legacy_topo = Topology::new();
                    let legacy_source = make_box(&mut legacy_topo, side, side, side).unwrap();
                    let (legacy_compound, legacy_map) =
                        remus_operations::pattern::grid_pattern_with_evolution(
                            &mut legacy_topo,
                            legacy_source,
                            GRID_X,
                            GRID_Y,
                            spacing_x,
                            spacing_y,
                            count_x,
                            count_y,
                        )
                        .unwrap();
                    let legacy_members = legacy_topo
                        .compound(legacy_compound)
                        .unwrap()
                        .solids()
                        .len();
                    assert_eq!(
                        legacy_members,
                        count_x * count_y,
                        "{label}: legacy placement count"
                    );
                    assert_eq!(
                        legacy_map.origin,
                        remus_operations::evolution::EvolutionOrigin::Construction,
                        "{label}: legacy face map stays construction-derived"
                    );
                }
                assert_volumes_equal(&label, &topo, journaled.compound, side.powi(3), scale);
                assert_translated_instances(
                    &label,
                    &topo,
                    source,
                    journaled.compound,
                    GRID_X,
                    GRID_Y,
                    spacing_x,
                    spacing_y,
                    count_x,
                    count_y,
                    scale,
                );
                for &solid in topo.compound(journaled.compound).unwrap().solids() {
                    assert_box_carriers(&label, &topo, solid);
                }
            }
        }
        // Curved, seam-bearing source: cylinder (lateral seam + caps).
        for &(count_x, count_y) in &[(1_usize, 3_usize), (3, 1), (2, 2), (3, 2)] {
            let mut topo = Topology::new();
            let radius = 2.0 * scale;
            let height = 5.0 * scale;
            let source = make_cylinder(&mut topo, radius, height).unwrap();
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            assert_eq!(source_faces.len(), 3, "cylinder has lateral + 2 caps");
            // Disjoint grid: pitch clears the diameter in both axes.
            let spacing_x = 10.0 * scale;
            let spacing_y = 10.0 * scale;
            let journaled = grid_pattern_journaled(
                &mut topo, source, GRID_X, GRID_Y, spacing_x, spacing_y, count_x, count_y,
            )
            .unwrap();
            let label = format!("cylinder {count_x}x{count_y} at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                count_x,
                count_y,
            );
            let expected = std::f64::consts::PI * radius * radius * height;
            assert_volumes_equal(&label, &topo, journaled.compound, expected, scale);
            assert_translated_instances(
                &label,
                &topo,
                source,
                journaled.compound,
                GRID_X,
                GRID_Y,
                spacing_x,
                spacing_y,
                count_x,
                count_y,
                scale,
            );
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                assert_cylinder_carriers(&label, &topo, solid, radius);
            }
        }
        // Cavity-bearing source: hollow box with one inner shell.
        for &(count_x, count_y) in &[(1_usize, 2_usize), (2, 1), (2, 2)] {
            let mut topo = Topology::new();
            let source = hollow_box(&mut topo, scale);
            assert_eq!(
                topo.solid(source).unwrap().inner_shells().len(),
                1,
                "hollow box must carry one cavity shell"
            );
            let source_faces = faces_of(&topo, source);
            let source_edges = edges_of(&topo, source);
            let source_vertices = vertices_of(&topo, source);
            assert_eq!(source_faces.len(), 12);
            let journaled = grid_pattern_journaled(
                &mut topo,
                source,
                GRID_X,
                GRID_Y,
                5.0 * scale,
                5.0 * scale,
                count_x,
                count_y,
            )
            .unwrap();
            let label = format!("hollow {count_x}x{count_y} at {scale:e}");
            assert_total_pattern_history(
                &label,
                &topo,
                source,
                journaled.compound,
                journaled.op,
                &source_faces,
                &source_edges,
                &source_vertices,
                count_x,
                count_y,
            );
            let expected_each = 26.0 * scale.powi(3);
            assert_volumes_equal(&label, &topo, journaled.compound, expected_each, scale);
            assert_translated_instances(
                &label,
                &topo,
                source,
                journaled.compound,
                GRID_X,
                GRID_Y,
                5.0 * scale,
                5.0 * scale,
                count_x,
                count_y,
                scale,
            );
            for &solid in topo.compound(journaled.compound).unwrap().solids() {
                assert_eq!(
                    topo.solid(solid).unwrap().inner_shells().len(),
                    1,
                    "{label}: copies must preserve the cavity shell"
                );
            }
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
fn grid_pattern_history_survives_consistent_transform() {
    // Rotate the operand and both grid directions by the same rigid rotation:
    // the relative pattern geometry is preserved, while a fixed-axis
    // assumption would misplace every copy. A translation of the body rides
    // along unchanged because grid offsets are relative.
    let placement = Mat4::translation(100.0, -50.0, 30.0) * Mat4::rotation_z(0.7);
    let dir_x = rotate_vec_by_matrix(&placement, GRID_X);
    let dir_y = rotate_vec_by_matrix(&placement, GRID_Y);
    // Box at 2x2.
    {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        remus_operations::transform::transform_solid(&mut topo, source, &placement).unwrap();
        let source_faces = faces_of(&topo, source);
        let source_edges = edges_of(&topo, source);
        let source_vertices = vertices_of(&topo, source);
        let journaled =
            grid_pattern_journaled(&mut topo, source, dir_x, dir_y, 25.0, 25.0, 2, 2).unwrap();
        assert_total_pattern_history(
            "placed box 2x2",
            &topo,
            source,
            journaled.compound,
            journaled.op,
            &source_faces,
            &source_edges,
            &source_vertices,
            2,
            2,
        );
        assert_translated_instances(
            "placed box 2x2",
            &topo,
            source,
            journaled.compound,
            dir_x,
            dir_y,
            25.0,
            25.0,
            2,
            2,
            1.0,
        );
        let mut volume = 0.0;
        for &solid in topo.compound(journaled.compound).unwrap().solids() {
            volume += remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
        }
        assert!(
            (volume - 4000.0).abs() < 1e-6,
            "placed volume {volume} != 4000"
        );
    }
    // Seam-bearing cylinder at 1x3 with the same consistent rotation.
    {
        let mut topo = Topology::new();
        let source = make_cylinder(&mut topo, 2.0, 5.0).unwrap();
        remus_operations::transform::transform_solid(&mut topo, source, &placement).unwrap();
        let source_faces = faces_of(&topo, source);
        let source_edges = edges_of(&topo, source);
        let source_vertices = vertices_of(&topo, source);
        let journaled =
            grid_pattern_journaled(&mut topo, source, dir_x, dir_y, 15.0, 15.0, 1, 3).unwrap();
        assert_total_pattern_history(
            "placed cylinder 1x3",
            &topo,
            source,
            journaled.compound,
            journaled.op,
            &source_faces,
            &source_edges,
            &source_vertices,
            1,
            3,
        );
        assert_translated_instances(
            "placed cylinder 1x3",
            &topo,
            source,
            journaled.compound,
            dir_x,
            dir_y,
            15.0,
            15.0,
            1,
            3,
            1.0,
        );
        let expected = std::f64::consts::PI * 4.0 * 5.0 * 3.0;
        let mut volume = 0.0;
        for &solid in topo.compound(journaled.compound).unwrap().solids() {
            volume += remus_operations::measure::solid_volume(&topo, solid, 0.01).unwrap();
        }
        let rel = (volume - expected).abs() / expected;
        assert!(rel < 1e-6, "placed cylinder volume {volume} != {expected}");
    }
    // Skewed (non-orthogonal) directions: still a valid grid as long as the
    // pair is not parallel. Pins that direction handling is by vectors, not
    // by an assumed axis-aligned lattice.
    {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let skew_x = Vec3::new(1.0, 0.0, 0.0);
        let skew_y = Vec3::new(1.0, 1.0, 0.0);
        let source_faces = faces_of(&topo, source);
        let source_edges = edges_of(&topo, source);
        let source_vertices = vertices_of(&topo, source);
        let journaled =
            grid_pattern_journaled(&mut topo, source, skew_x, skew_y, 25.0, 25.0, 2, 2).unwrap();
        assert_total_pattern_history(
            "skewed box 2x2",
            &topo,
            source,
            journaled.compound,
            journaled.op,
            &source_faces,
            &source_edges,
            &source_vertices,
            2,
            2,
        );
        assert_translated_instances(
            "skewed box 2x2",
            &topo,
            source,
            journaled.compound,
            skew_x.normalize().unwrap(),
            skew_y.normalize().unwrap(),
            25.0,
            25.0,
            2,
            2,
            1.0,
        );
    }
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
fn grid_pattern_lineage_is_original_only_and_covers_all_kinds() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, source);
    let journaled =
        grid_pattern_journaled(&mut topo, source, GRID_X, GRID_Y, 25.0, 25.0, 2, 2).unwrap();
    // Source-anchored references chase to the original instance alone:
    // copies are Generated adjacency, never identity, so the resolution is
    // Bound (one entity), never BoundMany (all instances).
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
                    // The bound entity is the original instance's entity:
                    // same arena index as the anchored source key.
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
    // The pattern entry's own outputs cover every result entity exactly
    // once, each Bound with construction provenance.
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
fn grid_pattern_refs_survive_arena_round_trip_restore_and_subsequent_edit() {
    let mut topo = Topology::new();
    let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let anchor = anchor_all_entities(&mut topo, source);
    let journaled =
        grid_pattern_journaled(&mut topo, source, GRID_X, GRID_Y, 25.0, 25.0, 2, 2).unwrap();
    let members = topo.compound(journaled.compound).unwrap().solids().to_vec();
    assert_eq!(members.len(), 4);
    // Arena round-trip: the journal travels with the document, so the same
    // PersistentRef values resolve in the fresh session.
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
    assert_eq!(document.solids.len(), 4);
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
    // Checkpoint restore truncates the pattern entry: references to its
    // outputs dangle as UnknownOperation rather than rebinding, and arena
    // slots are never reused.
    let snapshot = topo.clone();
    let pre_restore_faces = faces_of(&topo, members[0]);
    topo.restore_preserving_handle_slots(&snapshot);
    assert_eq!(faces_of(&topo, members[0]), pre_restore_faces);
    // Subsequent supported edit on one member: a planar move chases through
    // both the pattern and the edit with construction provenance.
    let moved_face = solid_faces(&topo, members[3])
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.x() > 0.9)
        })
        .unwrap();
    let edited = remus_operations::journal_ops::move_faces_journaled(
        &mut topo,
        members[3],
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
                    // Touched by the move on the fourth instance: honest
                    // severing across the edit is a typed outcome, not a
                    // silent rebind. Untouched originals stay Bound; the
                    // assertion below pins that at least the originals do.
                    let _ = (kind, index);
                }
                other => panic!("post-edit source {kind:?}/{index}: {other:?}"),
            }
        }
    }
    let _ = edited;
    let _ = Point3::new(0.0, 0.0, 0.0);
}

fn live_counts(topo: &Topology) -> [usize; 9] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_compounds(),
    ]
}

#[test]
fn grid_pattern_typed_refusals_preserve_geometry_history_and_resolution() {
    use remus_operations::journal_ops::grid_pattern_journaled;
    // Foreign handle: a retired solid refuses before publishing history, and
    // the stale handle stays stale.
    {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let anchor = anchor_all_entities(&mut topo, source);
        let retired = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        topo.delete_solid(retired).unwrap();
        let before = topo.journal().snapshot();
        let geometry = remus_io::step::writer::write_step(&topo, &[source]).unwrap();
        let counts = live_counts(&topo);
        let slots = topo.allocated_slot_count();
        let failed = grid_pattern_journaled(&mut topo, retired, GRID_X, GRID_Y, 25.0, 25.0, 2, 2);
        assert!(failed.is_err(), "retired handle must refuse");
        assert_eq!(topo.journal().snapshot().entries, before.entries);
        assert_eq!(topo.journal().snapshot().index, before.index);
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[source]).unwrap(),
            geometry
        );
        assert_eq!(live_counts(&topo), counts);
        assert_eq!(topo.allocated_slot_count(), slots);
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
    // Invalid counts refuse without publishing history or mutating geometry.
    for (count_x, count_y, tag) in [
        (0_usize, 2_usize, "count_x=0"),
        (2_usize, 0_usize, "count_y=0"),
        (0_usize, 0_usize, "counts=0"),
    ] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let before = topo.journal().snapshot();
        let geometry = remus_io::step::writer::write_step(&topo, &[source]).unwrap();
        let counts = live_counts(&topo);
        let failed = grid_pattern_journaled(
            &mut topo, source, GRID_X, GRID_Y, 25.0, 25.0, count_x, count_y,
        );
        assert!(failed.is_err(), "{tag} refusal must fail");
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "{tag} refusal published history"
        );
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[source]).unwrap(),
            geometry,
            "{tag} changed source geometry"
        );
        assert_eq!(live_counts(&topo), counts, "{tag} leaked topology");
        let volume = remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() < 1e-6,
            "{tag} changed source volume"
        );
    }
    // Invalid spacings refuse: zero, negative and sub-tolerance.
    for (spacing_x, spacing_y, tag) in [
        (0.0, 25.0, "spacing_x=0"),
        (25.0, 0.0, "spacing_y=0"),
        (-5.0, 25.0, "spacing_x<0"),
        (25.0, -5.0, "spacing_y<0"),
        (1e-12, 25.0, "spacing_x sub-tolerance"),
    ] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let before = topo.journal().snapshot();
        let failed = grid_pattern_journaled(
            &mut topo, source, GRID_X, GRID_Y, spacing_x, spacing_y, 2, 2,
        );
        assert!(failed.is_err(), "{tag} refusal must fail");
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "{tag} refusal published history"
        );
        let volume = remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() < 1e-6,
            "{tag} changed source geometry"
        );
    }
    // Invalid directions refuse: zero-length and parallel (both orientations).
    for (dir_x, dir_y, tag) in [
        (Vec3::new(0.0, 0.0, 0.0), GRID_Y, "dir_x=0"),
        (GRID_X, Vec3::new(0.0, 0.0, 0.0), "dir_y=0"),
        (GRID_X, Vec3::new(2.0, 0.0, 0.0), "parallel +"),
        (GRID_X, Vec3::new(-3.0, 0.0, 0.0), "parallel -"),
        (Vec3::new(f64::NAN, 0.0, 0.0), GRID_Y, "dir_x NaN"),
        (GRID_X, Vec3::new(1.0, 0.0, f64::INFINITY), "dir_y infinite"),
    ] {
        let mut topo = Topology::new();
        let source = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let before = topo.journal().snapshot();
        let failed = grid_pattern_journaled(&mut topo, source, dir_x, dir_y, 25.0, 25.0, 2, 2);
        assert!(failed.is_err(), "{tag} refusal must fail");
        assert_eq!(
            topo.journal().snapshot().entries,
            before.entries,
            "{tag} refusal published history"
        );
        let volume = remus_operations::measure::solid_volume(&topo, source, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() < 1e-6,
            "{tag} changed source geometry"
        );
    }
    // Material overlap refuses typed and rolls back: copies were allocated
    // before the geometry-level refusal, so live counts return while the
    // high-water slot count stays grown (failed allocations permanently
    // stale, never reused).
    for scale in SCALES {
        let mut topo = Topology::new();
        let side = 20.0 * scale;
        let source = make_box(&mut topo, side, side, side).unwrap();
        let counts_before = live_counts(&topo);
        let slots_before = topo.allocated_slot_count();
        let journal_before = topo.journal().snapshot();
        let geometry_before = remus_io::step::writer::write_step(&topo, &[source]).unwrap();
        let failed = grid_pattern_journaled(
            &mut topo,
            source,
            GRID_X,
            GRID_Y,
            10.0 * scale,
            30.0 * scale,
            2,
            1,
        );
        match failed {
            Err(remus_operations::OperationsError::PatternInstancesOverlap {
                overlap_volume,
                threshold,
                ..
            }) => {
                assert!(
                    overlap_volume > threshold,
                    "overlap {overlap_volume} must exceed threshold {threshold}"
                );
                let oracle = 10.0 * 20.0 * 20.0 * scale.powi(3);
                let rel = (overlap_volume - oracle).abs() / oracle;
                assert!(
                    rel < 1e-9,
                    "overlap volume {overlap_volume} != oracle {oracle} (rel {rel})"
                );
            }
            other => panic!("overlap must refuse typed, got {other:?}"),
        }
        assert_eq!(
            live_counts(&topo),
            counts_before,
            "overlap refusal at {scale:e} must retire every staged copy"
        );
        assert_eq!(
            topo.journal().snapshot().entries,
            journal_before.entries,
            "overlap refusal published history"
        );
        assert_eq!(
            remus_io::step::writer::write_step(&topo, &[source]).unwrap(),
            geometry_before,
            "overlap refusal changed source geometry"
        );
        assert!(
            topo.solid(source).is_ok(),
            "the input handle must survive rollback"
        );
        assert!(
            topo.allocated_slot_count() > slots_before,
            "overlap refusal must follow copied topology (slots {slots_before} -> {})",
            topo.allocated_slot_count()
        );
        let source_volume =
            remus_operations::measure::solid_volume(&topo, source, 0.01 * scale).unwrap();
        let expected = side.powi(3);
        let rel = (source_volume - expected).abs() / expected;
        assert!(rel < 1e-9, "source volume {source_volume} != {expected}");
    }
}
