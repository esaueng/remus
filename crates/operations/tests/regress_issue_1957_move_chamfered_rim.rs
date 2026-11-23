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
//! shape of a chamfer band but is re-limited, never carried. Without journal
//! history a band is a chamfer only with equal setbacks; with history, only
//! when a `chamfer` entry generated it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade, planar_chamfer_with_origins};
use remus_operations::extrude::extrude;
use remus_operations::journal_ops::{chamfer_journaled, move_faces_journaled, solid_entity_keys};
use remus_operations::measure::{face_area, solid_bounding_box, solid_volume};
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::make_box;
use remus_operations::push_pull::move_faces;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
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

/// A symmetric chamfer as the app builds one: the planar bevel first, the
/// general engine after.
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
) -> SolidId {
    let face = bottom(topo, solid);
    let counts = solid_entity_counts(topo, solid).unwrap();
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    let band = band_area(topo, solid);
    let before = solid_bounding_box(topo, solid).unwrap();
    let face_before = face_area(topo, face, DEFLECTION).unwrap();

    let moved = move_faces(topo, solid, &[face], distance)
        .unwrap_or_else(|error| panic!("{label}, move {distance}: {error}"));

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
    let mut prepared = PreparedFaceMove::prepare(topo, solid, &[face]).unwrap();
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

/// The bodies of the issue's repro, each with its wall section.
fn issue_bodies(topo: &mut Topology) -> Vec<(&'static str, SolidId, f64)> {
    let rounded = rounded_box(topo);
    let rim = edges_where(topo, rounded, bottom_rim);
    let rounded_chamfer = chamfer(topo, rounded, &rim, CHAMFER);

    let sharp = make_box(topo, X, Y, Z).unwrap();
    let rim = edges_where(topo, sharp, bottom_rim);
    let sharp_chamfer = chamfer(topo, sharp, &rim, CHAMFER);

    let sharp = make_box(topo, X, Y, Z).unwrap();
    let one = edges_where(topo, sharp, front_bottom);
    assert_eq!(one.len(), 1);
    let one_edge = chamfer(topo, sharp, &one, CHAMFER);

    let rounded = rounded_box(topo);
    let rim = edges_where(topo, rounded, bottom_rim);
    let control = fillet_cascade(topo, rounded, &rim, CHAMFER).unwrap().solid;

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

#[test]
fn chamfered_rims_travel_with_the_moved_face() {
    for distance in [2.0, -2.0, 0.1, -0.1, 20.0, -15.0, -19.6] {
        let mut topo = Topology::new();
        for (label, solid, section) in issue_bodies(&mut topo) {
            assert_carried(&mut topo, solid, distance, section, label);
        }
    }
}

/// Pushed in until the band's top would pass the top face, the move refuses
/// with a typed error and leaves the source as it was.
#[test]
fn a_carried_chamfer_refuses_to_cross_the_far_face() {
    for distance in [-19.7, -20.0, -25.0] {
        let mut topo = Topology::new();
        for (label, solid, _) in issue_bodies(&mut topo) {
            let face = bottom(&topo, solid);
            let counts = solid_entity_counts(&topo, solid).unwrap();
            let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
            let error = move_faces(&mut topo, solid, &[face], distance)
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
                    assert_carried(&mut topo, first, distance, section, &label);
                }
                for distance in [2.0, -2.0] {
                    assert_carried(&mut topo, solid, distance, section, &label);
                }
            }
        }
    }
}

/// Without journal history, unequal setbacks are read as a draft: the band is
/// not carried (the move re-limits it, or refuses over rounded corners).
#[test]
fn unjournaled_unequal_setbacks_are_not_carried() {
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
        if let Ok(moved) = move_faces(&mut topo, solid, &[face], -2.0) {
            let change = solid_volume(&topo, moved, DEFLECTION).unwrap() - volume;
            assert!(
                (change + 2.0 * section).abs() > 1.0,
                "rounded {rounded}: an unrecorded two-distance band was carried"
            );
        }
    }
}

/// A rim holding a recognized chamfer and a band that is not one refuses: the
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
    let error = move_faces(&mut topo, solid, &[face], -2.0)
        .expect_err("a chamfer and a draft on one rim must refuse");
    assert!(
        error.to_string().contains("mixes a chamfer"),
        "unexpected refusal: {error}"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
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
) {
    let face = face_facing(topo, solid, Vec3::new(0.0, 0.0, 1.0));
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    assert!(
        (volume - Y * profile_area(before)).abs() < 1e-6,
        "{label}: source volume"
    );
    let moved = move_faces(topo, solid, &[face], distance)
        .unwrap_or_else(|error| panic!("{label}: {error}"));
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
    assert_relimited(&mut topo, solid, 2.0, &before, &after, "frustum");
}

/// A tapered section standing on straight walls, its setbacks unequal (2 on
/// the cap, 5 on the walls): a draft, re-limited.
#[test]
fn a_tapered_section_on_straight_walls_is_relimited() {
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
    assert_relimited(&mut topo, solid, 2.0, &before, &after, "tapered section");
}

/// A 45° section standing on straight walls has the equal setbacks of a
/// chamfer. Without history it is read as one and carried; when the journal
/// records it as generated by another operation, it is a draft and is
/// re-limited.
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
    let volume = solid_volume(&topo, solid, DEFLECTION).unwrap();
    let moved = move_faces(&mut topo, solid, &[face], 2.0).unwrap();
    let change = solid_volume(&topo, moved, DEFLECTION).unwrap() - volume;
    assert!(
        (change - 2.0 * X * Y).abs() < 1e-6 * X * Y,
        "an unrecorded 45° section is carried as a chamfer: volume changed by {change}"
    );

    let mut topo = Topology::new();
    let solid = prism(&mut topo, &before);
    let pending = topo.journal_begin("taper_fixture");
    let keys = solid_entity_keys(&topo, solid).unwrap();
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
    assert_relimited(&mut topo, solid, 2.0, &before, &after, "recorded taper");
}
