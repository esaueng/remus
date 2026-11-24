//! Moving a planar face whose rim is chamfered carries the chamfer (#1957).
//!
//! A 60 × 40 × 20 box, sharp or rounded along its vertical edges (R3), has
//! its bottom rim chamfered, wholly or along one edge. Moving the bottom face
//! along its normal must translate the chamfer band with it: the walls change
//! length, the footprint and the chamfer stay as they are, and the volume
//! changes by the distance times the wall section. Before the fix the planar
//! engine re-limited the chamfer as an ordinary neighbour: over rounded
//! corners it refused at any distance, on sharp corners it extended the
//! chamfer planes into a different valid solid (pushed in by 2, the box grew
//! 1.65 sideways and gained volume).
//!
//! The other side of the criterion (`resize_blend/rim_chamfer.rs`): a draft
//! (a frustum side, or a tapered section standing on straight walls) has the
//! shape of a chamfer band but is re-limited, never carried. A band the
//! journal traces to a `chamfer` entry is carried; one it traces to any other
//! entry is a draft. Without that history, geometry cannot tell the two
//! apart (equal setbacks describe a 45° draft too), so the move refuses with
//! `AmbiguousRimChamfer` unless the caller resolves it
//! (`MoveFacesOptions::with_ambiguous_rim`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade, planar_chamfer_with_origins};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::journal_ops::{
    chamfer_journaled, extrude_journaled, move_faces_journaled, move_faces_journaled_with_options,
    solid_entity_keys,
};
use remus_operations::measure::{face_area, solid_bounding_box, solid_volume};
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::make_box;
use remus_operations::push_pull::{
    AmbiguousRimResolution, MoveFacesOptions, move_faces, move_faces_with_options,
};
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::{EventDraft, EvolutionDraft};
use remus_topology::solid::SolidId;

/// Picks edges by their two end points.
type EdgeFilter = fn([f64; 3], [f64; 3]) -> bool;

const X: f64 = 60.0;
const Y: f64 = 40.0;
const Z: f64 = 20.0;
const ROUND: f64 = 3.0;
const CHAMFER: f64 = 0.35;
const DEFLECTION: f64 = 1e-3;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

fn edges_where(
    topo: &Topology,
    solid: SolidId,
    keep: impl Fn([f64; 3], [f64; 3]) -> bool,
) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let edge = topo.edge(edge).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            keep([a.x(), a.y(), a.z()], [b.x(), b.y(), b.z()])
        })
        .collect()
}

fn vertical(a: [f64; 3], b: [f64; 3]) -> bool {
    near(a[0], b[0]) && near(a[1], b[1])
}

fn bottom_rim(a: [f64; 3], b: [f64; 3]) -> bool {
    near(a[2], 0.0) && near(b[2], 0.0)
}

fn front_bottom(a: [f64; 3], b: [f64; 3]) -> bool {
    bottom_rim(a, b) && near(a[1], 0.0) && near(b[1], 0.0)
}

fn back_bottom(a: [f64; 3], b: [f64; 3]) -> bool {
    bottom_rim(a, b) && near(a[1], Y) && near(b[1], Y)
}

/// The planar face whose outward normal is `normal`, largest first.
fn face_facing(topo: &Topology, solid: SolidId, normal: Vec3) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(normal) > 1.0 - 1e-9)
        })
        .max_by(|&a, &b| {
            face_area(topo, a, 0.01)
                .unwrap()
                .total_cmp(&face_area(topo, b, 0.01).unwrap())
        })
        .unwrap_or_else(|| panic!("no face facing {normal:?}"))
}

fn bottom(topo: &Topology, solid: SolidId) -> FaceId {
    face_facing(topo, solid, Vec3::new(0.0, 0.0, -1.0))
}

/// The box rounded along its vertical edges.
fn rounded_box(topo: &mut Topology) -> SolidId {
    let solid = make_box(topo, X, Y, Z).unwrap();
    let edges = edges_where(topo, solid, vertical);
    assert_eq!(edges.len(), 4);
    fillet_cascade(topo, solid, &edges, ROUND).unwrap().solid
}

const AS_CHAMFER: MoveFacesOptions =
    MoveFacesOptions::new().with_ambiguous_rim(AmbiguousRimResolution::TreatAsChamfer);
const AS_DRAFT: MoveFacesOptions =
    MoveFacesOptions::new().with_ambiguous_rim(AmbiguousRimResolution::TreatAsDraft);

const LEGACY_REPLAY: MoveFacesOptions =
    MoveFacesOptions::new().with_ambiguous_rim(AmbiguousRimResolution::LegacyReplay);

/// A symmetric chamfer as the app builds one: the planar bevel first, the
/// general engine after. Unjournaled: the band has no history.
fn chamfer(topo: &mut Topology, solid: SolidId, edges: &[EdgeId], distance: f64) -> SolidId {
    match planar_chamfer_with_origins(topo, solid, edges, distance) {
        Ok((chamfered, _)) => chamfered,
        Err(_) => {
            chamfer_v2(topo, solid, edges, distance, distance)
                .unwrap()
                .solid
        }
    }
}

const SHARP_SECTION: f64 = X * Y;

fn rounded_section() -> f64 {
    (4.0 - PI).mul_add(-ROUND * ROUND, X * Y)
}

/// Total area of the faces a chamfer cuts: tilted planes and cones.
fn band_area(topo: &Topology, solid: SolidId) -> f64 {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            let data = topo.face(face).unwrap();
            match data.surface() {
                FaceSurface::Cone(_) => true,
                FaceSurface::Plane { .. } => data.effective_plane_normal().is_some_and(|n| {
                    let along = n.z().abs();
                    along > 1e-9 && along < 1.0 - 1e-9
                }),
                _ => false,
            }
        })
        .map(|face| face_area(topo, face, DEFLECTION).unwrap())
        .sum()
}

/// Moves the bottom of `solid` by `distance` and checks the chamfer was
/// carried: same topology, footprint unchanged, the bottom shifted by the
/// distance, chamfer area unchanged, volume changed by `distance × section`.
/// The prepared move must give the same body. Returns the moved solid.
fn assert_carried(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    section: f64,
    label: &str,
    options: MoveFacesOptions,
) -> SolidId {
    let face = bottom(topo, solid);
    let counts = solid_entity_counts(topo, solid).unwrap();
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    let band = band_area(topo, solid);
    let before = solid_bounding_box(topo, solid).unwrap();
    let face_before = face_area(topo, face, DEFLECTION).unwrap();

    let moved = move_faces_with_options(topo, solid, &[face], distance, options)
        .unwrap_or_else(|error| panic!("{label}, move {distance}: {error}"))
        .solid;

    assert!(
        validate_solid(topo, moved).unwrap().is_valid(),
        "{label}, move {distance}: invalid result"
    );
    assert_eq!(
        solid_entity_counts(topo, moved).unwrap(),
        counts,
        "{label}, move {distance}: topology changed"
    );
    let mesh = tessellate_solid_with_tolerance(topo, moved, 0.01, 0.08).unwrap();
    assert!(is_watertight(&mesh), "{label}, move {distance}: open mesh");

    let after = solid_bounding_box(topo, moved).unwrap();
    for (axis, (a, b)) in [
        (after.min.x(), before.min.x()),
        (after.max.x(), before.max.x()),
        (after.min.y(), before.min.y()),
        (after.max.y(), before.max.y()),
        (after.max.z(), before.max.z()),
        (after.min.z(), before.min.z() - distance),
    ]
    .into_iter()
    .enumerate()
    {
        assert!(
            (a - b).abs() < 1e-9,
            "{label}, move {distance}: bounding box bound {axis} is {a}, expected {b}"
        );
    }
    let change = solid_volume(topo, moved, DEFLECTION).unwrap() - volume;
    let expected = distance * section;
    assert!(
        (change - expected).abs() <= 1e-6 * expected.abs().max(1.0),
        "{label}, move {distance}: volume changed by {change}, carried is {expected}"
    );
    let band_after = band_area(topo, moved);
    assert!(
        (band_after - band).abs() <= 1e-9 * band.max(1.0),
        "{label}, move {distance}: chamfer area {band} became {band_after}"
    );
    let face_after = face_area(topo, bottom(topo, moved), DEFLECTION).unwrap();
    assert!(
        (face_after - face_before).abs() <= 1e-9 * face_before,
        "{label}, move {distance}: bottom area {face_before} became {face_after}"
    );
    // The source is untouched.
    assert!(
        (solid_volume(topo, solid, DEFLECTION).unwrap() - volume).abs() <= 1e-9 * volume,
        "{label}: source changed"
    );

    // The prepared (drag) path gives the same body.
    let mut prepared =
        PreparedFaceMove::prepare_with_options(topo, solid, &[face], options).unwrap();
    let dragged = prepared.move_faces(topo, distance).unwrap().solid;
    assert_eq!(solid_entity_counts(topo, dragged).unwrap(), counts);
    let dragged_volume = solid_volume(topo, dragged, DEFLECTION).unwrap();
    assert!(
        (dragged_volume - volume - expected).abs() <= 1e-6 * expected.abs().max(1.0),
        "{label}, prepared move {distance}: volume changed by {}",
        dragged_volume - volume
    );
    moved
}

/// The bodies of the issue's repro, each with its wall section. With
/// `journaled`, each chamfer is recorded in the journal; without, the bands
/// have no history (as after an import). The blanks are built first: an
/// unjournaled edit between two journaled ones leaves a barrier that severs
/// the earlier band's history.
fn issue_bodies(topo: &mut Topology, journaled: bool) -> Vec<(&'static str, SolidId, f64)> {
    let rounded = rounded_box(topo);
    let rounded_rim = edges_where(topo, rounded, bottom_rim);
    let sharp = make_box(topo, X, Y, Z).unwrap();
    let sharp_rim = edges_where(topo, sharp, bottom_rim);
    let single = make_box(topo, X, Y, Z).unwrap();
    let one = edges_where(topo, single, front_bottom);
    assert_eq!(one.len(), 1);
    let control_blank = rounded_box(topo);
    let control_rim = edges_where(topo, control_blank, bottom_rim);

    let cut = |topo: &mut Topology, solid: SolidId, edges: &[EdgeId]| {
        if journaled {
            chamfer_journaled(topo, solid, edges, CHAMFER, CHAMFER)
                .unwrap()
                .result
                .solid
        } else {
            chamfer(topo, solid, edges, CHAMFER)
        }
    };
    let rounded_chamfer = cut(topo, rounded, &rounded_rim);
    let sharp_chamfer = cut(topo, sharp, &sharp_rim);
    let one_edge = cut(topo, single, &one);
    let control = fillet_cascade(topo, control_blank, &control_rim, CHAMFER)
        .unwrap()
        .solid;

    vec![
        (
            "R3 vertical corners, bottom chamfer",
            rounded_chamfer,
            rounded_section(),
        ),
        ("sharp box, bottom chamfer", sharp_chamfer, SHARP_SECTION),
        (
            "sharp box, one bottom edge chamfered",
            one_edge,
            SHARP_SECTION,
        ),
        (
            "control: R3 corners, bottom fillet",
            control,
            rounded_section(),
        ),
    ]
}

/// Journaled chamfers travel with the moved face by default; history-free
/// ones when the caller reads them as chamfers.
#[test]
fn chamfered_rims_travel_with_the_moved_face() {
    for distance in [2.0, -2.0, 0.1, -0.1, 20.0, -15.0, -19.6] {
        for (journaled, options) in [(true, MoveFacesOptions::default()), (false, AS_CHAMFER)] {
            let mut topo = Topology::new();
            for (label, solid, section) in issue_bodies(&mut topo, journaled) {
                let label = format!("{label}, journaled {journaled}");
                assert_carried(&mut topo, solid, distance, section, &label, options);
            }
        }
    }
}

/// Without history a chamfer band and a draft have one shape: the default
/// move refuses, naming the band, on the direct, journaled and prepared
/// paths, and leaves the source as it was. The fillet control still moves.
#[test]
fn history_free_chamfered_rims_refuse_by_default() {
    let mut topo = Topology::new();
    for (label, solid, _) in issue_bodies(&mut topo, false) {
        let face = bottom(&topo, solid);
        let counts = solid_entity_counts(&topo, solid).unwrap();
        let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
        let journal = topo.journal().entries().len();
        if label.starts_with("control") {
            move_faces(&mut topo, solid, &[face], 2.0).unwrap();
            continue;
        }
        let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[face]).unwrap();
        let errors = [
            move_faces(&mut topo, solid, &[face], 2.0).unwrap_err(),
            move_faces_journaled(&mut topo, solid, &[face], -2.0).unwrap_err(),
            prepared.move_faces(&mut topo, 2.0).unwrap_err(),
            prepared.move_faces_journaled(&mut topo, -2.0).unwrap_err(),
        ];
        for error in errors {
            let OperationsError::AmbiguousRimChamfer(band) = error else {
                panic!("{label}: expected an ambiguous-rim refusal, got {error}");
            };
            assert_eq!(band.cap, face, "{label}");
            assert!(
                (band.cap_setback - CHAMFER).abs() < 1e-9
                    && (band.wall_setback - CHAMFER).abs() < 1e-9,
                "{label}: setbacks {band:?}"
            );
            let edge = topo.edge(band.edge).unwrap();
            for vertex in [edge.start(), edge.end()] {
                assert!(topo.vertex(vertex).unwrap().point().z().abs() < 1e-9);
            }
            assert!(
                topo.face(band.wall)
                    .unwrap()
                    .effective_plane_normal()
                    .is_none_or(|n| n.z().abs() < 1e-9),
                "{label}: the wall is not vertical"
            );
        }
        assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
        assert!((solid_volume(&topo, solid, DEFLECTION).unwrap() - volume).abs() <= 1e-9 * volume);
        assert_eq!(topo.journal().entries().len(), journal, "{label}");
    }
}

/// Pushed in until the band's top would pass the top face, the move refuses
/// with a typed error and leaves the source as it was.
#[test]
fn a_carried_chamfer_refuses_to_cross_the_far_face() {
    for distance in [-19.7, -20.0, -25.0] {
        let mut topo = Topology::new();
        for (label, solid, _) in issue_bodies(&mut topo, false) {
            let face = bottom(&topo, solid);
            let counts = solid_entity_counts(&topo, solid).unwrap();
            let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
            let error = move_faces_with_options(&mut topo, solid, &[face], distance, AS_CHAMFER)
                .map(|moved| moved.solid)
                .expect_err(&format!("{label}: a move of {distance} must refuse"));
            assert!(
                matches!(
                    error,
                    OperationsError::Offset(_) | OperationsError::ResizeBlend(_)
                ),
                "{label}, move {distance}: untyped refusal {error}"
            );
            assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
            assert!(
                (solid_volume(&topo, solid, DEFLECTION).unwrap() - volume).abs() <= 1e-9 * volume
            );
        }
    }
}

/// Two-distance and distance-angle chamfers carry when the journal records
/// the chamfer that cut them, over sharp and rounded corners, through a
/// second journaled move as well.
#[test]
fn journaled_two_distance_and_distance_angle_chamfers_travel() {
    let angle = 30.0_f64.to_radians();
    for (d1, d2) in [(0.3, 0.6), (0.6, 0.3), (0.4, 0.4 * angle.tan())] {
        for rounded in [false, true] {
            // A single chamfered edge runs into the rounds only on the
            // sharp box.
            let selections: &[EdgeFilter] = if rounded {
                &[bottom_rim]
            } else {
                &[bottom_rim, front_bottom]
            };
            for &keep in selections {
                let mut topo = Topology::new();
                let body = if rounded {
                    rounded_box(&mut topo)
                } else {
                    make_box(&mut topo, X, Y, Z).unwrap()
                };
                let edges = edges_where(&topo, body, keep);
                let solid = chamfer_journaled(&mut topo, body, &edges, d1, d2)
                    .unwrap()
                    .result
                    .solid;
                let section = if rounded {
                    rounded_section()
                } else {
                    SHARP_SECTION
                };
                let label = format!("chamfer {d1}/{d2}, rounded {rounded}");
                // A journaled move first: an unjournaled edit before the next
                // journaled operation leaves a barrier in the journal, and
                // the band's history is then no longer trusted.
                let face = bottom(&topo, solid);
                let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
                let first = move_faces_journaled(&mut topo, solid, &[face], 1.5)
                    .unwrap_or_else(|error| panic!("{label}: journaled move: {error}"))
                    .solid;
                let change = solid_volume(&topo, first, DEFLECTION).unwrap() - volume;
                assert!(
                    (change - 1.5 * section).abs() <= 1e-6 * section,
                    "{label}: journaled move changed the volume by {change}"
                );
                // The moved band keeps its history through the move.
                for distance in [-3.0, 2.0] {
                    let options = MoveFacesOptions::default();
                    assert_carried(&mut topo, first, distance, section, &label, options);
                }
                for distance in [2.0, -2.0] {
                    let options = MoveFacesOptions::default();
                    assert_carried(&mut topo, solid, distance, section, &label, options);
                }
            }
        }
    }
}

/// Without journal history, unequal setbacks are as ambiguous as equal ones:
/// the default move refuses; read as a draft the band is re-limited (or the
/// move refuses over rounded corners), read as a chamfer it is carried.
#[test]
fn unjournaled_unequal_setbacks_follow_the_caller() {
    for rounded in [false, true] {
        let mut topo = Topology::new();
        let body = if rounded {
            rounded_box(&mut topo)
        } else {
            make_box(&mut topo, X, Y, Z).unwrap()
        };
        let edges = edges_where(&topo, body, bottom_rim);
        let solid = chamfer_v2(&mut topo, body, &edges, 0.3, 0.6).unwrap().solid;
        let section = if rounded {
            rounded_section()
        } else {
            SHARP_SECTION
        };
        let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
        let face = bottom(&topo, solid);
        let error = move_faces(&mut topo, solid, &[face], -0.2).unwrap_err();
        let OperationsError::AmbiguousRimChamfer(band) = error else {
            panic!("rounded {rounded}: expected an ambiguous-rim refusal, got {error}");
        };
        let setbacks = [
            band.cap_setback.min(band.wall_setback),
            band.cap_setback.max(band.wall_setback),
        ];
        assert!(
            (setbacks[0] - 0.3).abs() < 1e-9 && (setbacks[1] - 0.6).abs() < 1e-9,
            "rounded {rounded}: {band:?}"
        );
        if let Ok(moved) = move_faces_with_options(&mut topo, solid, &[face], -0.2, AS_DRAFT) {
            let change = solid_volume(&topo, moved.solid, DEFLECTION).unwrap() - volume;
            assert!(
                (change + 0.2 * section).abs() > 0.1,
                "rounded {rounded}: a band read as a draft was carried"
            );
        }
        for distance in [2.0, -2.0] {
            let label = format!("unjournaled 0.3/0.6, rounded {rounded}");
            assert_carried(&mut topo, solid, distance, section, &label, AS_CHAMFER);
        }
    }
}

/// A rim holding a recognized chamfer and a band read as a draft refuses: the
/// translation cannot carry one and re-limit the other.
#[test]
fn a_rim_mixing_a_chamfer_and_a_draft_refuses() {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, X, Y, Z).unwrap();
    let front = edges_where(&topo, sharp, front_bottom);
    let solid = chamfer(&mut topo, sharp, &front, CHAMFER);
    let back = edges_where(&topo, solid, back_bottom);
    assert_eq!(back.len(), 1);
    let solid = chamfer_v2(&mut topo, solid, &back, 0.3, 0.6).unwrap().solid;
    let face = bottom(&topo, solid);
    let counts = solid_entity_counts(&topo, solid).unwrap();
    // The back band, recorded as made by something other than a chamfer, is
    // a draft; the front band, with no history, is read as a chamfer.
    let back_band = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&band| {
            topo.face(band)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.y() > 1e-9 && n.z() < -1e-9)
        })
        .unwrap();
    record_as_made_by(&mut topo, &[back_band], "taper_fixture");
    let error = move_faces_with_options(&mut topo, solid, &[face], -0.2, AS_CHAMFER)
        .expect_err("a chamfer and a draft on one rim must refuse");
    assert!(
        error.to_string().contains("mixes a chamfer"),
        "unexpected refusal: {error}"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
}

/// Records every entity of `faces` as generated by a `kind` entry.
fn record_as_made_by(topo: &mut Topology, faces: &[FaceId], kind: &str) {
    let pending = topo.journal_begin(kind);
    let keys: Vec<_> = faces
        .iter()
        .map(|face| remus_topology::journal::EntityKey::face(face.index()))
        .collect();
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for key in &keys {
        draft.push(
            *key,
            EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    topo.journal_record_evolution(pending, draft).unwrap();
}

/// A prism of the profile `points` (in x–z) extruded 40 along y.
fn prism(topo: &mut Topology, points: &[[f64; 2]]) -> SolidId {
    let points: Vec<Point3> = points
        .iter()
        .map(|p| Point3::new(p[0], 0.0, p[1]))
        .collect();
    let profile = remus_topology::builder::make_planar_face(topo, &points, 1e-7).unwrap();
    let normal = topo
        .face(profile)
        .unwrap()
        .effective_plane_normal()
        .unwrap();
    // Extrude toward +y whichever way the profile faces.
    let direction = if normal.y() < 0.0 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, -1.0, 0.0)
    };
    extrude(topo, profile, direction, Y).unwrap()
}

/// Shoelace area of an x–z profile.
fn profile_area(points: &[[f64; 2]]) -> f64 {
    let mut twice = 0.0;
    for (index, a) in points.iter().enumerate() {
        let b = points[(index + 1) % points.len()];
        twice += a[0].mul_add(b[1], -(b[0] * a[1]));
    }
    twice.abs() / 2.0
}

/// Moves the top of the prism of `before` up by `distance` and checks it was
/// re-limited: the sloped faces keep their planes, so the profile becomes
/// `after`.
fn assert_relimited(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    before: &[[f64; 2]],
    after: &[[f64; 2]],
    label: &str,
    options: MoveFacesOptions,
) {
    let face = face_facing(topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    assert!(
        (volume - Y * profile_area(before)).abs() < 1e-6,
        "{label}: source volume"
    );
    let moved = move_faces_with_options(topo, solid, &[face], distance, options)
        .unwrap_or_else(|error| panic!("{label}: {error}"))
        .solid;
    assert!(validate_solid(topo, moved).unwrap().is_valid(), "{label}");
    let expected = Y * profile_area(after);
    let got = solid_volume(topo, moved, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "{label}: volume {got}, re-limited is {expected}, carried would be {}",
        volume + distance * Y * X
    );
}

/// A frustum side climbs to the far face, not a wall: a draft, re-limited.
#[test]
fn a_frustum_side_is_relimited() {
    let before = [[0.0, 0.0], [X, 0.0], [58.0, 15.0], [2.0, 15.0]];
    let after = [
        [0.0, 0.0],
        [X, 0.0],
        [X - 2.0 * 17.0 / 15.0, 17.0],
        [2.0 * 17.0 / 15.0, 17.0],
    ];
    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    assert_relimited(
        &mut topo,
        solid,
        2.0,
        &before,
        &after,
        "frustum",
        MoveFacesOptions::default(),
    );
}

/// Moves the top of the prism of `before` up by `distance` and checks the
/// sloped band was carried: the profile keeps its shape above the walls,
/// which lengthen by `distance`.
fn assert_section_carried(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    before: &[[f64; 2]],
    options: MoveFacesOptions,
    label: &str,
) {
    let face = face_facing(topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let area = face_area(topo, face, DEFLECTION).unwrap();
    let moved = move_faces_with_options(topo, solid, &[face], distance, options)
        .unwrap_or_else(|error| panic!("{label}: {error}"))
        .solid;
    assert!(validate_solid(topo, moved).unwrap().is_valid(), "{label}");
    let expected = Y * (profile_area(before) + distance * X);
    let got = solid_volume(topo, moved, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "{label}: volume {got}, carried is {expected}"
    );
    let top = face_facing(topo, moved, Vec3::new(0.0, 0.0, 1.0));
    assert!((face_area(topo, top, DEFLECTION).unwrap() - area).abs() < 1e-9 * area);
    let bounds = solid_bounding_box(topo, moved).unwrap();
    assert!((bounds.max.z() - (15.0 + distance)).abs() < 1e-9, "{label}");
    assert!(bounds.min.x().abs() < 1e-9 && (bounds.max.x() - X).abs() < 1e-9);
}

/// A tapered section standing on straight walls (2 on the cap, 5 on the
/// walls) with no history: refused by default, re-limited as a draft,
/// carried as a chamfer, whichever the caller reads it as. Through the
/// prepared path too.
#[test]
fn a_tapered_section_on_straight_walls_follows_the_caller() {
    let before = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [58.0, 15.0],
        [2.0, 15.0],
        [0.0, 10.0],
    ];
    let after = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [57.2, 17.0],
        [2.8, 17.0],
        [0.0, 10.0],
    ];
    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    let face = face_facing(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let error = move_faces(&mut topo, solid, &[face], 2.0).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(_)),
        "unexpected refusal: {error}"
    );
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[face]).unwrap();
    assert!(matches!(
        prepared.move_faces(&mut topo, 2.0),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
    let label = "tapered section read as a draft";
    assert_relimited(&mut topo, solid, 2.0, &before, &after, label, AS_DRAFT);
    let label = "tapered section read as a chamfer";
    assert_section_carried(&mut topo, solid, 2.0, &before, AS_CHAMFER, label);
}

/// A 45° section standing on straight walls has the equal setbacks of a
/// symmetric chamfer. Without history it is ambiguous: refused by default,
/// carried or re-limited when the caller says which. When the journal records
/// it as made by another operation (a fixture entry, or a journaled
/// extrusion of the tapered profile) it is a draft and is re-limited by
/// default.
#[test]
fn journal_history_decides_an_equal_setback_section() {
    let before = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [55.0, 15.0],
        [5.0, 15.0],
        [0.0, 10.0],
    ];
    let after = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [53.0, 17.0],
        [7.0, 17.0],
        [0.0, 10.0],
    ];

    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    let face = face_facing(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let error = move_faces(&mut topo, solid, &[face], 2.0).unwrap_err();
    let OperationsError::AmbiguousRimChamfer(band) = error else {
        panic!("an unrecorded 45° section must refuse, got {error}");
    };
    assert_eq!(band.cap, face);
    assert!((band.cap_setback - 5.0).abs() < 1e-9 && (band.wall_setback - 5.0).abs() < 1e-9);
    let label = "unrecorded 45° section read as a chamfer";
    assert_section_carried(&mut topo, solid, 2.0, &before, AS_CHAMFER, label);
    let label = "unrecorded 45° section read as a draft";
    assert_relimited(&mut topo, solid, 2.0, &before, &after, label, AS_DRAFT);

    assert_relimited(
        &mut topo,
        solid,
        2.0,
        &before,
        &after,
        "saved planar move",
        LEGACY_REPLAY,
    );

    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    let keys = solid_entity_keys(&topo, solid).unwrap();
    let pending = topo.journal_begin("taper_fixture");
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.iter().copied());
    for key in &keys {
        draft.push(
            *key,
            EventDraft::Generated {
                sources: Vec::new(),
            },
        );
    }
    topo.journal_record_evolution(pending, draft).unwrap();
    let options = MoveFacesOptions::default();
    assert_relimited(
        &mut topo,
        solid,
        2.0,
        &before,
        &after,
        "recorded taper",
        options,
    );
    // History outranks the caller's reading: a recorded draft is re-limited
    // even when the caller would read ambiguous bands as chamfers.
    let label = "recorded taper, caller reads ambiguity as chamfer";
    assert_relimited(&mut topo, solid, 2.0, &before, &after, label, AS_CHAMFER);

    // A journaled extrusion of the tapered profile is a draft too, through the
    // journaled and prepared moves as well.
    let mut topo = Topology::new();
    let points: Vec<Point3> = before
        .iter()
        .map(|p| Point3::new(p[0], 0.0, p[1]))
        .collect();
    let profile = remus_topology::builder::make_planar_face(&mut topo, &points, 1e-7).unwrap();
    let normal = topo
        .face(profile)
        .unwrap()
        .effective_plane_normal()
        .unwrap();
    let direction = Vec3::new(0.0, -normal.y().signum(), 0.0);
    let solid = extrude_journaled(&mut topo, profile, direction, Y)
        .unwrap()
        .solid;
    // The journaled move first: an unjournaled move before it would leave a
    // barrier severing the band's history.
    let options = MoveFacesOptions::default();
    let face = face_facing(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let expected = Y * profile_area(&after);
    let journaled = move_faces_journaled_with_options(&mut topo, solid, &[face], 2.0, options)
        .unwrap()
        .solid;
    let got = solid_volume(&topo, journaled, DEFLECTION).unwrap();
    assert!((got - expected).abs() < 1e-6 * expected, "journaled: {got}");
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[face]).unwrap();
    let dragged = prepared.move_faces(&mut topo, 2.0).unwrap().solid;
    let got = solid_volume(&topo, dragged, DEFLECTION).unwrap();
    assert!((got - expected).abs() < 1e-6 * expected, "prepared: {got}");
    assert_relimited(
        &mut topo,
        solid,
        2.0,
        &before,
        &after,
        "extruded taper",
        options,
    );
}

/// The carried chamfer sweeps material the cap's own sweep does not: a
/// cavity just behind the band, beyond the cap's footprint, must stop the
/// move rather than be cut open by the translated chamfer plane.
#[test]
fn a_carried_chamfer_refuses_to_sweep_through_a_cavity() {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, X, Y, Z).unwrap();
    let front = edges_where(&topo, sharp, front_bottom);
    let solid = chamfer(&mut topo, sharp, &front, CHAMFER);
    let cavity = make_box(&mut topo, 2.0, 0.1, 0.1).unwrap();
    transform_solid(&mut topo, cavity, &Mat4::translation(10.0, 0.1, 2.16)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, solid, cavity).unwrap();
    let face = bottom(&topo, solid);
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let error = move_faces_with_options(&mut topo, solid, &[face], -2.0, AS_CHAMFER)
        .expect_err("the chamfer would sweep through the cavity");
    assert!(
        error
            .to_string()
            .contains("swept face reaches nonadjacent face"),
        "unexpected refusal: {error}"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
}

/// A body far from the origin reads the same setbacks: the chamfer is
/// carried exactly as at the origin.
#[test]
fn a_translated_body_carries_its_chamfer_the_same() {
    for offset in [1e3, 4e6] {
        let mut topo = Topology::new();
        for (label, solid, section) in issue_bodies(&mut topo, false) {
            transform_solid(
                &mut topo,
                solid,
                &Mat4::translation(offset, -offset, offset),
            )
            .unwrap();
            for distance in [2.0, -2.0] {
                let face = bottom(&topo, solid);
                let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
                let moved =
                    move_faces_with_options(&mut topo, solid, &[face], distance, AS_CHAMFER)
                        .unwrap_or_else(|error| panic!("{label} at {offset}: {error}"))
                        .solid;
                let change = solid_volume(&topo, moved, DEFLECTION).unwrap() - volume;
                let expected = distance * section;
                assert!(
                    (change - expected).abs() <= 1e-3 * expected.abs(),
                    "{label} at {offset}, move {distance}: volume changed by {change}, carried \
                     is {expected}"
                );
            }
        }
    }
}

/// A journal gap introduced by an unrelated edit severs the band's history
/// for the direct and the prepared journaled moves alike: both refuse it as
/// ambiguous, and read as a draft both build the same body.
#[test]
fn prepared_journaled_move_uses_the_same_chamfer_identity_after_a_gap() {
    let mut topo = Topology::new();
    let body = make_box(&mut topo, X, Y, Z).unwrap();
    let edges = edges_where(&topo, body, front_bottom);
    let solid = chamfer_journaled(&mut topo, body, &edges, 0.3, 0.6)
        .unwrap()
        .result
        .solid;
    make_box(&mut topo, X, Y, Z).unwrap();
    let face = bottom(&topo, solid);

    let mut direct_topo = topo.clone();
    assert!(matches!(
        move_faces_journaled(&mut direct_topo, solid, &[face], 1.5),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
    let direct = move_faces_journaled_with_options(&mut direct_topo, solid, &[face], 1.5, AS_DRAFT)
        .unwrap()
        .solid;
    let direct_volume = solid_volume(&direct_topo, direct, DEFLECTION).unwrap();

    for (multi, qualified) in [(false, false), (false, true), (true, false), (true, true)] {
        for options in [MoveFacesOptions::default(), AS_DRAFT] {
            let mut preview_topo = topo.clone();
            let mut prepared = if multi {
                PreparedFaceMove::prepare_multi_with_options(
                    &preview_topo,
                    solid,
                    &[face],
                    &[],
                    options,
                )
            } else {
                PreparedFaceMove::prepare_with_options(&preview_topo, solid, &[face], options)
            }
            .unwrap();
            let moved = if qualified {
                prepared
                    .move_faces_journaled_with_quality(&mut preview_topo, 1.5, None)
                    .map(|(moved, _)| moved)
            } else {
                prepared.move_faces_journaled(&mut preview_topo, 1.5)
            };
            if options == AS_DRAFT {
                let moved = moved.unwrap();
                let prepared_volume = solid_volume(&preview_topo, moved.solid, DEFLECTION).unwrap();
                assert!(
                    (prepared_volume - direct_volume).abs() < 1e-7,
                    "direct result volume {direct_volume}, prepared result volume \
                     {prepared_volume}"
                );
            } else {
                assert!(
                    matches!(moved, Err(OperationsError::AmbiguousRimChamfer(_))),
                    "multi {multi}, qualified {qualified}: {moved:?}"
                );
            }
        }
    }
}

/// A chamfered bore must not allow the moving cap to carry an unrelated
/// outer band that the geometry could not identify as a chamfer.
#[test]
fn a_chamfered_bore_does_not_override_mixed_outer_rim_refusal() {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, X, Y, Z).unwrap();
    let tool = remus_operations::primitives::make_cylinder(&mut topo, 3.0, Z + 2.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(X / 2.0, Y / 2.0, -1.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, sharp, tool).unwrap();
    let front = edges_where(&topo, drilled, front_bottom);
    let solid = chamfer(&mut topo, drilled, &front, CHAMFER);
    let back = edges_where(&topo, solid, back_bottom);
    let solid = chamfer_v2(&mut topo, solid, &back, 0.3, 0.6).unwrap().solid;
    let mouth: Vec<_> = edges_where(&topo, solid, bottom_rim)
        .into_iter()
        .filter(|&id| {
            matches!(
                topo.edge(id).unwrap().curve(),
                remus_topology::edge::EdgeCurve::Circle(_)
            )
        })
        .collect();
    assert_eq!(mouth.len(), 1);
    let solid = chamfer_v2(&mut topo, solid, &mouth, CHAMFER, CHAMFER)
        .unwrap()
        .solid;
    let face = bottom(&topo, solid);
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let error = move_faces(&mut topo, solid, &[face], -0.5).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(_)),
        "{error}"
    );
    // Read as drafts, the outer bands are re-limited while the mouth cone
    // travels with the cap: the translation cannot do both.
    let error = move_faces_with_options(&mut topo, solid, &[face], -0.5, AS_DRAFT)
        .expect_err("a chamfered bore cannot override the mixed rim refusal");
    assert!(error.to_string().contains("mixes a chamfer"), "{error}");
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
}

/// Before rim disambiguation a chamfered bore mouth seeded a translation,
/// and the unrecorded outer bevel rode with that cap.
#[test]
fn legacy_replay_carries_the_unrecorded_rim_with_a_chamfered_bore() {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, X, Y, Z).unwrap();
    let radius = 5.0;
    let tool = remus_operations::primitives::make_cylinder(&mut topo, radius, Z + 10.0).unwrap();
    transform_solid(&mut topo, tool, &Mat4::translation(20.0, 20.0, -5.0)).unwrap();
    let drilled = boolean(&mut topo, BooleanOp::Cut, sharp, tool).unwrap();
    let edges = edges_where(&topo, drilled, bottom_rim);
    let solid = chamfer(&mut topo, drilled, &edges, CHAMFER);
    let face = bottom(&topo, solid);
    assert!(matches!(
        move_faces(&mut topo, solid, &[face], 2.0),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
    assert_carried(
        &mut topo,
        solid,
        2.0,
        X * Y - PI * radius * radius,
        "saved bore move",
        LEGACY_REPLAY,
    );
}

/// A varying setback cannot seed a chamfer move, but the earlier move did
/// carry this oblique band with the chamfered bore.
#[test]
fn legacy_replay_allows_an_oblique_rider_with_a_chamfered_bore() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, X, Y, Z).unwrap();
    // The cutter removes x + 0.02*y - z > 58.4 along the bottom corner.
    let profile = remus_topology::builder::make_planar_face(
        &mut topo,
        &[
            Point3::new(48.42, -1.0, -10.0),
            Point3::new(80.0, -1.0, -10.0),
            Point3::new(80.0, -1.0, 21.58),
        ],
        1e-7,
    )
    .unwrap();
    let sweep = Vec3::new(-0.02 * (Y + 2.0), Y + 2.0, 0.0);
    let wedge = extrude(
        &mut topo,
        profile,
        sweep * sweep.length().recip(),
        sweep.length(),
    )
    .unwrap();
    let plate = boolean(&mut topo, BooleanOp::Cut, plate, wedge).unwrap();
    let bore = remus_operations::primitives::make_cylinder(&mut topo, 5.0, Z + 10.0).unwrap();
    transform_solid(&mut topo, bore, &Mat4::translation(20.0, 20.0, -5.0)).unwrap();
    let plate = boolean(&mut topo, BooleanOp::Cut, plate, bore).unwrap();
    let mouth: Vec<_> = edges_where(&topo, plate, bottom_rim)
        .into_iter()
        .filter(|&id| {
            matches!(
                topo.edge(id).unwrap().curve(),
                remus_topology::edge::EdgeCurve::Circle(_)
            )
        })
        .collect();
    assert_eq!(mouth.len(), 1);
    let solid = chamfer(&mut topo, plate, &mouth, CHAMFER);
    let face = bottom(&topo, solid);
    let refused = move_faces(&mut topo, solid, &[face], 2.0).unwrap_err();
    assert!(
        refused
            .to_string()
            .contains("not a constant-setback chamfer"),
        "{refused}"
    );
    assert_carried(
        &mut topo,
        solid,
        2.0,
        X * Y - PI * 25.0,
        "saved oblique rider",
        LEGACY_REPLAY,
    );
}

fn regular_prism(topo: &mut Topology, sides: u32) -> SolidId {
    let points: Vec<_> = (0..sides)
        .map(|index| {
            let t = std::f64::consts::TAU * f64::from(index) / f64::from(sides);
            Point3::new(20.0 * t.cos(), 20.0 * t.sin(), 0.0)
        })
        .collect();
    let profile = remus_topology::builder::make_planar_face(topo, &points, 1e-7).unwrap();
    extrude(topo, profile, Vec3::new(0.0, 0.0, 1.0), Z).unwrap()
}

/// A bevel split into many connected patches must not be re-limited as a
/// draft simply because the rim recognizer reached its work limit.
#[test]
fn large_connected_bevel_cannot_fall_through_to_planar_move() {
    const SIDES: u32 = 72;
    let mut topo = Topology::new();
    let solid = regular_prism(&mut topo, SIDES);
    let rim = edges_where(&topo, solid, bottom_rim);
    assert_eq!(rim.len(), SIDES as usize);
    let solid = chamfer(&mut topo, solid, &rim, CHAMFER);
    let face = bottom(&topo, solid);
    let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
    let distance = -0.1;
    assert!(matches!(
        move_faces(&mut topo, solid, &[face], distance),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
    match move_faces_with_options(&mut topo, solid, &[face], distance, AS_CHAMFER)
        .map(|moved| moved.solid)
    {
        Ok(moved) => {
            let area = f64::from(SIDES) * 200.0 * (std::f64::consts::TAU / f64::from(SIDES)).sin();
            let change = solid_volume(&topo, moved, DEFLECTION).unwrap() - volume;
            assert!(
                (change - distance * area).abs() < 1e-6,
                "bevel was re-limited: volume change {change}, expected {}",
                distance * area
            );
        }
        Err(error) => assert!(
            error.to_string().contains("above the carry limit"),
            "unexpected refusal: {error}"
        ),
    }
}

/// Preparation cannot reject a chamfer on history that the eventual
/// journaled move will sever before classifying the rim.
#[test]
fn prepared_move_defers_oversized_chamfer_refusal_until_after_journal_gap() {
    let mut topo = Topology::new();
    let prism = regular_prism(&mut topo, 72);
    let rim = edges_where(&topo, prism, bottom_rim);
    let solid = chamfer_journaled(&mut topo, prism, &rim, 0.3, 0.6)
        .unwrap()
        .result
        .solid;
    make_box(&mut topo, X, Y, Z).unwrap();
    let face = bottom(&topo, solid);
    let distance = -0.1;

    let mut direct_topo = topo.clone();
    assert!(matches!(
        move_faces_journaled(&mut direct_topo, solid, &[face], distance),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
    let direct =
        move_faces_journaled_with_options(&mut direct_topo, solid, &[face], distance, AS_DRAFT)
            .expect("after the journal gap the caller reads the unequal rim as a draft");
    assert!(
        validate_solid(&direct_topo, direct.solid)
            .unwrap()
            .is_valid()
    );
    let expected = solid_volume(&direct_topo, direct.solid, DEFLECTION).unwrap();

    for (multi, qualified) in [(false, false), (false, true), (true, false), (true, true)] {
        let mut preview_topo = topo.clone();
        let mut prepared = if multi {
            PreparedFaceMove::prepare_multi_with_options(
                &preview_topo,
                solid,
                &[face],
                &[],
                AS_DRAFT,
            )
        } else {
            PreparedFaceMove::prepare_with_options(&preview_topo, solid, &[face], AS_DRAFT)
        }
        .expect("preparation must defer the lineage-dependent refusal");
        let result = if qualified {
            prepared
                .move_faces_journaled_with_quality(&mut preview_topo, distance, None)
                .unwrap()
                .0
        } else {
            prepared
                .move_faces_journaled(&mut preview_topo, distance)
                .unwrap()
        };
        let actual = solid_volume(&preview_topo, result.solid, DEFLECTION).unwrap();
        assert!(
            (actual - expected).abs() < 1e-7,
            "prepared {actual}, direct {expected}"
        );
    }
}

/// A prepared move reads a band by its history; a barrier recorded since
/// (the solid unchanged) severs that history, and the prepared handle then
/// refuses as the fresh move does rather than carrying the band.
#[test]
fn a_prepared_move_follows_a_barrier_that_severs_the_band_history() {
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, X, Y, Z).unwrap();
    let rim = edges_where(&topo, blank, bottom_rim);
    let solid = chamfer_journaled(&mut topo, blank, &rim, CHAMFER, CHAMFER)
        .unwrap()
        .result
        .solid;
    let face = bottom(&topo, solid);
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[face]).unwrap();
    prepared.move_faces(&mut topo, 2.0).unwrap();

    let pending = topo.journal_begin("barrier_fixture");
    remus_operations::journal_ops::record_barrier_over_solid(&mut topo, pending, solid).unwrap();
    assert!(prepared.is_current(&topo), "the solid itself is unchanged");
    let error = move_faces(&mut topo, solid, &[face], 2.0).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(_)),
        "unexpected refusal: {error}"
    );
    let error = prepared.move_faces(&mut topo, 2.0).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(_)),
        "unexpected refusal: {error}"
    );
    // Prepared again it agrees with the fresh move.
    let mut prepared = PreparedFaceMove::prepare(&topo, solid, &[face]).unwrap();
    assert!(matches!(
        prepared.move_faces(&mut topo, 2.0),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
}

/// A multi-face move keeps every face it does not move on its support, so it
/// can only re-limit a rim band: history-free bands refuse unless read as
/// drafts, bands read as chamfers refuse; directly and prepared, the
/// prepared handle keeping the options it was prepared with.
#[test]
fn multi_face_moves_refuse_rim_bands_they_cannot_carry() {
    use remus_operations::multi_move::move_faces_multi;
    use remus_operations::push_pull::with_move_options;
    let before = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [58.0, 15.0],
        [2.0, 15.0],
        [0.0, 10.0],
    ];
    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    let top = face_facing(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let floor = face_facing(&topo, solid, Vec3::new(0.0, 0.0, -1.0));
    let faces = [top, floor];
    let distances = [2.0, 1.0];
    let counts = solid_entity_counts(&topo, solid).unwrap();

    let error = move_faces_multi(&mut topo, solid, &faces, &distances).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(band) if band.cap == top),
        "unexpected refusal: {error}"
    );
    let error = with_move_options(AS_CHAMFER, || {
        move_faces_multi(&mut topo, solid, &faces, &distances)
    })
    .unwrap_err();
    assert!(
        matches!(error, OperationsError::Unsupported { .. }),
        "unexpected refusal: {error}"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);

    // Read as a draft, the sloped planes stay and the top is re-limited; the
    // floor moves down by 1.
    let moved = with_move_options(AS_DRAFT, || {
        move_faces_multi(&mut topo, solid, &faces, &distances)
    })
    .unwrap();
    let after = [
        [0.0, -1.0],
        [X, -1.0],
        [X, 10.0],
        [57.2, 17.0],
        [2.8, 17.0],
        [0.0, 10.0],
    ];
    let expected = Y * profile_area(&after);
    let got = solid_volume(&topo, moved, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "volume {got}, expected {expected}"
    );

    // Prepared: the handle runs under the options it was prepared with, never
    // the ambient ones.
    let mut prepared =
        PreparedFaceMove::prepare_multi_with_options(&topo, solid, &faces, &[2.0, 1.0], AS_DRAFT)
            .unwrap();
    let dragged = prepared.move_faces(&mut topo, 1.0).unwrap().solid;
    let got = solid_volume(&topo, dragged, DEFLECTION).unwrap();
    assert!(
        (got - expected).abs() < 1e-6 * expected,
        "prepared volume {got}"
    );
    let mut prepared = with_move_options(AS_DRAFT, || {
        PreparedFaceMove::prepare_multi(&topo, solid, &faces, &[2.0, 1.0]).unwrap()
    });
    assert_eq!(prepared.options(), MoveFacesOptions::default());
    assert!(matches!(
        prepared.move_faces(&mut topo, 1.0),
        Err(OperationsError::AmbiguousRimChamfer(_))
    ));
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
}

/// Two parallel terraces at different heights are two caps to a multi-face
/// move: after a barrier severs the upper terrace's chamfer history, the
/// prepared handle refuses as the fresh move now does.
#[test]
fn a_prepared_multi_move_of_parallel_terraces_tracks_each_rim() {
    use remus_operations::multi_move::move_faces_multi;
    let profile = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [30.0, 10.0],
        [30.0, 15.0],
        [0.0, 15.0],
    ];
    let mut topo = Topology::new();
    let blank = prism(&mut topo, &profile);
    let edge = edges_where(&topo, blank, |a, b| {
        near(a[0], 0.0) && near(b[0], 0.0) && near(a[2], 15.0) && near(b[2], 15.0)
    });
    assert_eq!(edge.len(), 1);
    let solid = chamfer_journaled(&mut topo, blank, &edge, 0.5, 0.5)
        .unwrap()
        .result
        .solid;
    let up = Vec3::new(0.0, 0.0, 1.0);
    let tops: Vec<FaceId> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(up) > 1.0 - 1e-9)
        })
        .collect();
    assert_eq!(tops.len(), 2);
    let weights = [1.0, 2.0];
    let error = move_faces_multi(&mut topo, solid, &tops, &weights).unwrap_err();
    assert!(
        matches!(error, OperationsError::Unsupported { .. }),
        "a journaled chamfer cannot be carried by a multi-face move: {error}"
    );
    let mut prepared = PreparedFaceMove::prepare_multi(&topo, solid, &tops, &weights).unwrap();
    assert!(matches!(
        prepared.move_faces(&mut topo, 1.0),
        Err(OperationsError::Unsupported { .. })
    ));

    let pending = topo.journal_begin("barrier_fixture");
    remus_operations::journal_ops::record_barrier_over_solid(&mut topo, pending, solid).unwrap();
    let error = move_faces_multi(&mut topo, solid, &tops, &weights).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(_)),
        "unexpected refusal: {error}"
    );
    let error = prepared.move_faces(&mut topo, 1.0).unwrap_err();
    assert!(
        matches!(error, OperationsError::AmbiguousRimChamfer(_)),
        "unexpected refusal: {error}"
    );
}

/// A chamfer face moved along with the cap does not hide the strips of its
/// ring that stay fixed: they are still read, and refuse without history.
#[test]
fn selecting_one_strip_of_a_ring_still_reads_the_others() {
    use remus_operations::multi_move::move_faces_multi;
    let mut topo = Topology::new();
    let blank = make_box(&mut topo, X, Y, Z).unwrap();
    let rim = edges_where(&topo, blank, bottom_rim);
    let solid = chamfer(&mut topo, blank, &rim, CHAMFER);
    let cap = bottom(&topo, solid);
    let strip = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.y() < -1e-9 && n.z() < -1e-9)
        })
        .unwrap();
    let wall = face_facing(&topo, solid, Vec3::new(0.0, -1.0, 0.0));
    let faces = [cap, strip, wall];
    let counts = solid_entity_counts(&topo, solid).unwrap();
    let error = move_faces_multi(&mut topo, solid, &faces, &[0.1, 0.1, 0.1]).unwrap_err();
    let OperationsError::AmbiguousRimChamfer(band) = error else {
        panic!("expected an ambiguous-rim refusal, got {error}");
    };
    assert_eq!(band.cap, cap);
    assert!(
        !faces.contains(&band.face),
        "the refusal names a fixed strip"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
}

/// The rim refusal never masks the multi-face solver's own refusal of a
/// zero or non-finite distance.
#[test]
fn an_invalid_multi_move_distance_is_refused_as_such() {
    use remus_operations::multi_move::move_faces_multi;
    let before = [
        [0.0, 0.0],
        [X, 0.0],
        [X, 10.0],
        [58.0, 15.0],
        [2.0, 15.0],
        [0.0, 10.0],
    ];
    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    let top = face_facing(&topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let floor = face_facing(&topo, solid, Vec3::new(0.0, 0.0, -1.0));
    for distances in [[2.0, 0.0], [2.0, f64::NAN]] {
        let error = move_faces_multi(&mut topo, solid, &[top, floor], &distances).unwrap_err();
        assert!(
            !matches!(error, OperationsError::AmbiguousRimChamfer(_)),
            "{distances:?}: {error}"
        );
    }
}
