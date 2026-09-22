//! Regression coverage for atomic connected blend-group removal.
//!
//! Every fixture is synthetic. Seeds are found by analytic carrier geometry,
//! never by arena index. Successful cases compare against a sharp construction
//! oracle, verify construction history, and exercise the public operation.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::{BTreeMap, BTreeSet};

use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::remove_blends::{RemoveBlendsResult, remove_blends};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::journal::{
    EntityEvent, EntityKey, EntityKind, EntryPayload, EventDraft, EvolutionDraft, RecordedOrigin,
};
use remus_topology::naming::{PersistentRef, Provenance, Resolution, resolve};
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.02;
const BOX_SIZE: f64 = 40.0;
const RADIUS: f64 = 3.0;
type DeterministicOutcome = (
    usize,
    usize,
    usize,
    u64,
    Vec<(EntityKey, Option<EntityKey>)>,
);

#[derive(Debug, PartialEq, Eq)]
struct ArenaCounts {
    vertices: usize,
    edges: usize,
    wires: usize,
    faces: usize,
    shells: usize,
    solids: usize,
    loops: usize,
    coedges: usize,
    pcurves: usize,
    attributes: usize,
}

fn arena_counts(topo: &Topology) -> ArenaCounts {
    ArenaCounts {
        vertices: topo.num_vertices(),
        edges: topo.num_edges(),
        wires: topo.num_wires(),
        faces: topo.num_faces(),
        shells: topo.num_shells(),
        solids: topo.num_solids(),
        loops: topo.num_loops(),
        coedges: topo.num_coedges(),
        pcurves: topo.num_pcurves(),
        attributes: topo.attributes().len(),
    }
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{label}: invalid solid: {:?}",
        report
            .issues
            .iter()
            .map(|issue| &issue.description)
            .collect::<Vec<_>>()
    );
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).unwrap()
}

fn assert_volume_eq(
    actual_topo: &Topology,
    actual: SolidId,
    expected_topo: &Topology,
    expected: SolidId,
    label: &str,
) {
    // Use the exact face-integral path for the sharp oracle comparison. The
    // legacy deflection volume intentionally remains in rollback fingerprints,
    // but its tessellation error is large enough to obscure this small concave
    // fixture even when both bodies have identical exact planar carriers.
    let actual = remus_operations::measure::mass_properties(actual_topo, actual)
        .unwrap()
        .mass;
    let expected = remus_operations::measure::mass_properties(expected_topo, expected)
        .unwrap()
        .mass;
    // The exact face-integral implementation documents <= 1e-6 relative on
    // all-planar bodies whose trimmed polygon is integrated numerically. The
    // carrier/topology assertions below remain exact; this bound applies only
    // to the independent mass-property oracle.
    let tolerance = expected.abs().mul_add(1e-6, 1e-10);
    assert!(
        (actual - expected).abs() <= tolerance,
        "{label}: volume {actual} != sharp oracle {expected} (tol {tolerance})"
    );
}

fn point(topo: &Topology, edge: EdgeId, start: bool) -> Point3 {
    let edge = topo.edge(edge).unwrap();
    topo.vertex(if start { edge.start() } else { edge.end() })
        .unwrap()
        .point()
}

fn edge_at_corner_along(topo: &Topology, solid: SolidId, corner: Point3, axis: Vec3) -> EdgeId {
    let tol = Tolerance::new().linear;
    let mut candidates: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|edge| {
            let edge_data = topo.edge(*edge).unwrap();
            if !matches!(edge_data.curve(), EdgeCurve::Line) {
                return false;
            }
            let a = point(topo, *edge, true);
            let b = point(topo, *edge, false);
            let touches = (a - corner).length() <= tol || (b - corner).length() <= tol;
            let direction = (b - a).normalize().unwrap();
            touches && direction.dot(axis).abs() > 1.0 - 1e-12
        })
        .collect();
    candidates.sort_unstable_by_key(|edge| edge.index());
    let [edge] = candidates.as_slice() else {
        panic!(
            "expected one edge at {corner:?} along {axis:?}, got {}",
            candidates.len()
        );
    };
    *edge
}

fn blend_faces(topo: &Topology, solid: SolidId) -> (Vec<FaceId>, Vec<FaceId>) {
    let mut bands = Vec::new();
    let mut corners = Vec::new();
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder)
                if Tolerance::new().approx_eq(cylinder.radius(), RADIUS) =>
            {
                bands.push(face);
            }
            FaceSurface::Sphere(sphere) if Tolerance::new().approx_eq(sphere.radius(), RADIUS) => {
                corners.push(face);
            }
            _ => {}
        }
    }
    bands.sort_unstable_by_key(|face| face.index());
    corners.sort_unstable_by_key(|face| face.index());
    (bands, corners)
}

fn box_corner_fixture(edge_count: usize) -> (Topology, SolidId, SolidId, Vec<FaceId>) {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, BOX_SIZE, BOX_SIZE, BOX_SIZE).unwrap();
    let corner = Point3::new(BOX_SIZE, BOX_SIZE, BOX_SIZE);
    let axes = [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ];
    let edges: Vec<_> = axes[..edge_count]
        .iter()
        .map(|&axis| edge_at_corner_along(&topo, sharp, corner, axis))
        .collect();
    let blended = fillet_v2(&mut topo, sharp, &edges, RADIUS).unwrap().solid;
    assert_valid(&topo, blended, "box blend baseline");
    let (bands, _) = blend_faces(&topo, blended);
    assert_eq!(bands.len(), edge_count, "one cylindrical band per edge");
    (topo, sharp, blended, bands)
}

fn concave_fixture() -> (Topology, SolidId, SolidId, FaceId) {
    let mut topo = Topology::new();
    let profile = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(5.0, -1.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, -3.0, 0.0),
            Point3::new(0.0, -3.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let source_sharp = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 8.0).unwrap();
    let ridge: Vec<_> = solid_edges(&topo, source_sharp)
        .unwrap()
        .into_iter()
        .filter(|edge| {
            let a = point(&topo, *edge, true);
            let b = point(&topo, *edge, false);
            (a.x() - 5.0).abs() < 1e-9 && (b.x() - 5.0).abs() < 1e-9 && (a.z() - b.z()).abs() > 1.0
        })
        .collect();
    let [ridge] = ridge.as_slice() else {
        panic!("expected one concave ridge, got {}", ridge.len());
    };
    let blended = fillet_v2(&mut topo, source_sharp, &[*ridge], 0.02)
        .unwrap()
        .solid;
    let band = solid_faces(&topo, blended)
        .unwrap()
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), 0.02)
            )
        })
        .expect("concave blend band");
    assert_valid(&topo, blended, "concave blend baseline");

    // Independent sharp oracle: boolean-cut the same triangular notch from a
    // rectangular prism instead of reusing the profile extrusion that was
    // filleted. This construction has separate topology lineage and catches a
    // wound heal that merely reproduces the source builder's segmentation.
    let sharp_box = make_box(&mut topo, 10.0, 3.0, 8.0).unwrap();
    transform_solid(&mut topo, sharp_box, &Mat4::translation(0.0, -3.0, 0.0)).unwrap();
    let notch_wire = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(4.0, 0.0, -1.0),
            Point3::new(6.0, 0.0, -1.0),
            Point3::new(5.0, -1.0, -1.0),
        ],
        1e-7,
    )
    .unwrap();
    let notch_face = topo.add_face(Face::new(
        notch_wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: -1.0,
        },
    ));
    let notch = extrude(&mut topo, notch_face, Vec3::new(0.0, 0.0, 1.0), 10.0).unwrap();
    let sharp = boolean(&mut topo, BooleanOp::Cut, sharp_box, notch).unwrap();
    assert_valid(&topo, sharp, "independent concave sharp oracle");
    (topo, sharp, blended, band)
}

fn assert_exact_history(
    topo: &Topology,
    source: SolidId,
    removed: &[FaceId],
    result: &RemoveBlendsResult,
) {
    assert!(result.evolution.origin.is_exact());
    assert!(result.evolution.unresolved.is_empty());
    let result_faces: BTreeSet<_> = solid_faces(topo, result.solid)
        .unwrap()
        .into_iter()
        .map(remus_topology::arena::Id::index)
        .collect();
    assert!(
        result
            .evolution
            .is_construction_resolved_for_result(result_faces.iter().copied())
    );
    let removed: BTreeSet<_> = removed.iter().map(|face| face.index()).collect();
    assert_eq!(
        result.evolution.deleted,
        removed.iter().copied().collect(),
        "exactly the recognized group is deleted"
    );

    for source_face in solid_faces(topo, source).unwrap() {
        if removed.contains(&source_face.index()) {
            continue;
        }
        let [target] = result
            .evolution
            .modified
            .get(&source_face.index())
            .expect("surviving face has construction successor")
            .as_slice()
        else {
            panic!("surviving face has one successor");
        };
        let target = topo.face_id_from_index(*target).unwrap();
        assert!(
            remus_operations::heal::surfaces_equivalent_pub(
                topo.face(source_face).unwrap().surface(),
                topo.face(target).unwrap().surface(),
            ),
            "surviving face {} changed carrier",
            source_face.index()
        );
    }

    let source_boundaries: BTreeSet<_> =
        remus_operations::journal_ops::solid_entity_keys(topo, source)
            .unwrap()
            .into_iter()
            .filter(|key| key.kind != EntityKind::Face)
            .collect();
    let result_boundaries: BTreeSet<_> =
        remus_operations::journal_ops::solid_entity_keys(topo, result.solid)
            .unwrap()
            .into_iter()
            .filter(|key| key.kind != EntityKind::Face)
            .collect();
    assert_eq!(
        result
            .boundary_history
            .iter()
            .map(|(source, _)| *source)
            .collect::<BTreeSet<_>>(),
        source_boundaries
    );
    assert_eq!(
        result
            .boundary_history
            .iter()
            .filter_map(|(_, target)| *target)
            .collect::<BTreeSet<_>>(),
        result_boundaries
    );
    assert!(
        result
            .boundary_history
            .windows(2)
            .all(|pair| pair[0] < pair[1]),
        "history is strictly sorted and deduplicated"
    );
}

fn source_references(topo: &mut Topology, solid: SolidId) -> Vec<(EntityKey, PersistentRef)> {
    let keys = remus_operations::journal_ops::solid_entity_keys(topo, solid).unwrap();
    let pending = topo.journal_begin("connected_blend_fixture");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for &key in &keys {
        draft.push(key, EventDraft::Generated { sources: vec![] });
    }
    let anchor = topo.journal_record_evolution(pending, draft).unwrap();
    let mut offsets = BTreeMap::new();
    keys.into_iter()
        .map(|key| {
            let offset = offsets.entry(key.kind).or_insert(0_usize);
            let reference = PersistentRef::operation_output(anchor, key.kind, *offset);
            *offset += 1;
            (key, reference)
        })
        .collect()
}

fn assert_matches_sharp_oracle(
    topo: &Topology,
    sharp: SolidId,
    blended: SolidId,
    removed: &[FaceId],
    result: &RemoveBlendsResult,
    label: &str,
) {
    assert_valid(topo, result.solid, label);
    // STEP reimport independently reconstructs the sharp oracle's canonical
    // topology. This avoids treating source-construction wire splits as part
    // of the desired result while retaining exact analytic carriers.
    let oracle_step = remus_io::step::write_step(topo, &[sharp]).unwrap();
    let mut oracle_topo = Topology::new();
    let oracle = remus_io::step::read_step(&oracle_step, &mut oracle_topo).unwrap()[0];
    assert_valid(&oracle_topo, oracle, &format!("{label}: sharp STEP oracle"));
    assert_volume_eq(topo, result.solid, &oracle_topo, oracle, label);
    assert_eq!(
        solid_faces(topo, result.solid).unwrap().len(),
        solid_faces(&oracle_topo, oracle).unwrap().len(),
        "{label}: face census matches sharp oracle"
    );
    assert_eq!(
        solid_edges(topo, result.solid).unwrap().len(),
        solid_edges(&oracle_topo, oracle).unwrap().len(),
        "{label}: edge census matches sharp oracle"
    );
    assert_eq!(
        solid_vertices(topo, result.solid).unwrap().len(),
        solid_vertices(&oracle_topo, oracle).unwrap().len(),
        "{label}: vertex census matches sharp oracle"
    );
    assert!(
        solid_faces(topo, result.solid)
            .unwrap()
            .into_iter()
            .all(|face| matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            ))
    );
    assert_exact_history(topo, blended, removed, result);
}

#[test]
fn convex_and_concave_public_paths_restore_sharp_oracles() {
    let (mut topo, sharp, blended, bands) = box_corner_fixture(1);
    let result = remove_blends(&mut topo, blended, &[bands[0]]).unwrap();
    assert_matches_sharp_oracle(&topo, sharp, blended, &bands, &result, "convex band");

    let (mut topo, sharp, blended, band) = concave_fixture();
    let result = remove_blends(&mut topo, blended, &[band]).unwrap();
    assert_matches_sharp_oracle(&topo, sharp, blended, &[band], &result, "concave band");
}

#[test]
fn adjacent_shared_support_bands_are_one_atomic_wound() {
    // Three incident bands share the three planar supports pairwise. None can
    // be healed independently because the spherical corner joins all three;
    // the public group path removes the shared-support wound in one rebuild.
    let (mut topo, sharp, blended, bands) = box_corner_fixture(3);
    let (_, corners) = blend_faces(&topo, blended);
    assert_eq!(
        corners.len(),
        1,
        "shared-support bands have one corner patch"
    );
    let mut removed = bands.clone();
    removed.extend(corners.iter().copied());

    let result = remove_blends(
        &mut topo,
        blended,
        &[corners[0], bands[2], bands[1], bands[0], bands[1]],
    )
    .unwrap();
    assert_matches_sharp_oracle(
        &topo,
        sharp,
        blended,
        &removed,
        &result,
        "shared-support pair",
    );
}

#[test]
fn trihedral_corner_group_succeeds_only_as_complete_atomic_selection() {
    let (mut topo, sharp, blended, bands) = box_corner_fixture(3);
    let (_, corners) = blend_faces(&topo, blended);
    assert_eq!(corners.len(), 1, "trihedral fixture has one sphere patch");

    let before = arena_counts(&topo);
    let source_volume = volume(&topo, blended).to_bits();
    let error = remove_blends(&mut topo, blended, &[bands[0]]).unwrap_err();
    assert!(
        matches!(error, OperationsError::ResizeBlend(_)),
        "partial corner has typed blend refusal: {error:?}"
    );
    assert_eq!(
        arena_counts(&topo),
        before,
        "partial corner rolls back arena"
    );
    assert_eq!(
        volume(&topo, blended).to_bits(),
        source_volume,
        "partial corner preserves source bits"
    );
    assert_valid(&topo, blended, "partial corner source after rollback");

    let mut removed = bands.clone();
    removed.extend(corners);
    let result = remove_blends(&mut topo, blended, &bands).unwrap();
    assert_matches_sharp_oracle(&topo, sharp, blended, &removed, &result, "trihedral corner");
}

#[test]
fn trihedral_group_journal_resolves_every_source_boundary_and_result_entity() {
    let (mut topo, sharp, blended, bands) = box_corner_fixture(3);
    let source_refs = source_references(&mut topo, blended);
    let result =
        remus_operations::journal_ops::remove_blends_journaled(&mut topo, blended, &bands).unwrap();
    assert_valid(&topo, result.solid, "journaled trihedral result");
    assert_volume_eq(
        &topo,
        result.solid,
        &topo,
        sharp,
        "journaled trihedral result",
    );

    let entry = topo.journal().entries().last().unwrap();
    assert_eq!(entry.op(), result.op);
    assert_eq!(entry.kind(), "remove_blends");
    let EntryPayload::Evolution { origin, events, .. } = entry.payload() else {
        panic!("connected removal must record evolution, not a barrier");
    };
    assert_eq!(*origin, RecordedOrigin::Construction);
    assert!(
        events
            .iter()
            .all(|(_, event)| !matches!(event, EntityEvent::Unresolved { .. })),
        "group removal cannot publish unresolved forward records"
    );
    assert!(
        events
            .iter()
            .any(|(_, event)| matches!(event, EntityEvent::Merged { from } if from.len() > 1)),
        "collapsed corner boundaries need grouped many-to-one records"
    );
    assert!(
        events
            .iter()
            .any(|(_, event)| matches!(event, EntityEvent::Deleted)),
        "blend faces and consumed boundaries are deleted"
    );

    let result_keys: BTreeSet<_> =
        remus_operations::journal_ops::solid_entity_keys(&topo, result.solid)
            .unwrap()
            .into_iter()
            .collect();
    for &key in &result_keys {
        let ordinal = topo
            .journal()
            .ordinal_of(key)
            .unwrap_or_else(|| panic!("result entity {key:?} has no journal ordinal"));
        assert!(
            events
                .binary_search_by_key(&ordinal, |(subject, _)| *subject)
                .is_ok(),
            "result entity {key:?} has no forward record"
        );
    }

    let mut resolved = BTreeSet::new();
    let mut deleted = BTreeSet::new();
    let mut bound_resolutions = 0;
    let mut settled_boundaries = 0;
    for (source, reference) in &source_refs {
        match resolve(&topo, reference) {
            Resolution::Bound {
                entity,
                provenance: Provenance::Construction,
            } => {
                resolved.insert(entity);
                bound_resolutions += 1;
            }
            Resolution::Dangling { deleted_at } => {
                assert_eq!(deleted_at, result.op);
                deleted.insert(*source);
            }
            other => panic!("source {source:?} did not resolve exactly: {other:?}"),
        }
        if source.kind != EntityKind::Face {
            settled_boundaries += 1;
        }
    }
    assert_eq!(
        resolved, result_keys,
        "resolved references equal result census"
    );
    assert!(
        !deleted.is_empty(),
        "blend group faces and consumed boundaries have explicit deletions"
    );
    assert_eq!(
        settled_boundaries,
        source_refs
            .iter()
            .filter(|(key, _)| key.kind != EntityKind::Face)
            .count(),
        "every source edge and vertex has a bound or deleted construction resolution"
    );
    assert!(
        bound_resolutions > resolved.len(),
        "at least one grouped source set resolves through a many-to-one merge"
    );
}

#[test]
fn invalid_and_disconnected_selections_are_exact_no_ops() {
    let (mut topo, _, blended, bands) = box_corner_fixture(1);
    let foreign = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
    let foreign_face = solid_faces(&topo, foreign).unwrap()[0];
    let planar_face = solid_faces(&topo, blended)
        .unwrap()
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .unwrap();

    for seeds in [&[][..], &[foreign_face][..], &[planar_face][..]] {
        let before = arena_counts(&topo);
        let fingerprint = (
            solid_faces(&topo, blended).unwrap(),
            solid_edges(&topo, blended).unwrap(),
            solid_vertices(&topo, blended).unwrap(),
            volume(&topo, blended).to_bits(),
        );
        assert!(remove_blends(&mut topo, blended, seeds).is_err());
        assert_eq!(arena_counts(&topo), before);
        assert_eq!(
            (
                solid_faces(&topo, blended).unwrap(),
                solid_edges(&topo, blended).unwrap(),
                solid_vertices(&topo, blended).unwrap(),
                volume(&topo, blended).to_bits(),
            ),
            fingerprint
        );
        assert_valid(&topo, blended, "invalid selection rollback");
    }

    // A valid seed still succeeds after all refusals, proving handle slots and
    // recognition state were restored rather than merely leaving the old
    // solid readable.
    remove_blends(&mut topo, blended, &bands).unwrap();

    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, BOX_SIZE, BOX_SIZE, BOX_SIZE).unwrap();
    let first = edge_at_corner_along(
        &topo,
        sharp,
        Point3::new(BOX_SIZE, BOX_SIZE, BOX_SIZE),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let second = edge_at_corner_along(
        &topo,
        sharp,
        Point3::new(0.0, 0.0, BOX_SIZE),
        Vec3::new(0.0, 0.0, 1.0),
    );
    let separated = fillet_v2(&mut topo, sharp, &[first, second], RADIUS)
        .unwrap()
        .solid;
    let (separated_bands, corners) = blend_faces(&topo, separated);
    assert_eq!(separated_bands.len(), 2);
    assert!(corners.is_empty());
    let before = arena_counts(&topo);
    let volume_before = volume(&topo, separated).to_bits();
    let error = remove_blends(&mut topo, separated, &separated_bands).unwrap_err();
    assert!(format!("{error}").contains("disconnected"));
    assert_eq!(arena_counts(&topo), before);
    assert_eq!(volume(&topo, separated).to_bits(), volume_before);
    assert_valid(&topo, separated, "disconnected selection rollback");
}

#[test]
fn trihedral_step_round_trip_preserves_exact_group_removal_and_history() {
    let (topo, _, blended, _) = box_corner_fixture(3);
    let step = remus_io::step::write_step(&topo, &[blended]).unwrap();
    assert!(!step.is_empty());
    let mut imported = Topology::new();
    let imported_blended = remus_io::step::read_step(&step, &mut imported).unwrap()[0];
    assert_valid(&imported, imported_blended, "STEP blend baseline");
    let (bands, corners) = blend_faces(&imported, imported_blended);
    assert_eq!(bands.len(), 3);
    assert_eq!(corners.len(), 1);
    let mut removed = bands.clone();
    removed.extend(corners);

    let result = remove_blends(&mut imported, imported_blended, &bands).unwrap();
    assert_valid(&imported, result.solid, "STEP removed result");
    let expected_volume = BOX_SIZE * BOX_SIZE * BOX_SIZE;
    assert!((volume(&imported, result.solid) - expected_volume).abs() <= expected_volume * 1e-8);
    assert_eq!(solid_faces(&imported, result.solid).unwrap().len(), 6);
    assert_exact_history(&imported, imported_blended, &removed, &result);

    let sharp_step = remus_io::step::write_step(&imported, &[result.solid]).unwrap();
    let mut reread = Topology::new();
    let reread_solid = remus_io::step::read_step(&sharp_step, &mut reread).unwrap()[0];
    assert_valid(&reread, reread_solid, "removed STEP reread");
    assert_eq!(solid_faces(&reread, reread_solid).unwrap().len(), 6);
    assert!(
        solid_faces(&reread, reread_solid)
            .unwrap()
            .into_iter()
            .all(|face| matches!(
                reread.face(face).unwrap().surface(),
                FaceSurface::Plane { .. }
            ))
    );
    assert!((volume(&reread, reread_solid) - expected_volume).abs() <= expected_volume * 1e-8);
}

#[test]
fn seed_order_and_duplicates_do_not_change_the_result() {
    fn run(reverse: bool) -> DeterministicOutcome {
        let (mut topo, _, blended, mut bands) = box_corner_fixture(3);
        if reverse {
            bands.reverse();
            bands.push(bands[0]);
        }
        let result = remove_blends(&mut topo, blended, &bands).unwrap();
        (
            solid_faces(&topo, result.solid).unwrap().len(),
            solid_edges(&topo, result.solid).unwrap().len(),
            solid_vertices(&topo, result.solid).unwrap().len(),
            volume(&topo, result.solid).to_bits(),
            result.boundary_history,
        )
    }
    assert_eq!(run(false), run(true));
}
