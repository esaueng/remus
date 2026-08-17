//! Import of a `CONICAL_SURFACE` whose `base_radius` is not zero.
//!
//! ISO 10303-42 states a cone's `radius` on its placement plane; remus's
//! `ConicalSurface` is anchored at the apex. The reader dropped the radius
//! and read the placement origin as the apex, so every non-zero-radius cone
//! landed `radius*cot(semi_angle)` too far along its own axis with the wrong
//! radius everywhere. On a customer part with 400 faces, the 2 cones that
//! declared a radius were the entire remaining volume error: their trim
//! circles sat exactly `radius` off their own surfaces.
//!
//! A round trip cannot cover this. remus's writer anchors on the apex and
//! always emits `0.0E0` for the radius, which is a legal and self-consistent
//! statement of the same cone — so a remus-written file never puts a
//! non-zero `base_radius` back through the reader, and the two could not
//! cancel even if the writer had the mirror-image bug. The hand-written
//! fixture is the coverage; the round trip here only confirms the corrected
//! apex survives being written out and read back.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::path::{Path, PathBuf};

use remus_operations::measure;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

const FIXTURE: &str = "conical_surface_base_radius_frustum.step";
const DEFLECTION: f64 = 0.01;

/// Frustum of radius 12 at z=0 and radius 18 at z=8:
/// `pi*h/3 * (R1^2 + R1*R2 + R2^2)` = 1824*pi.
const EXPECTED_VOLUME: f64 = std::f64::consts::PI * 1824.0;

/// Lateral area of the same frustum, `pi*(R1 + R2)*slant` = 300*pi, summed
/// over the two halves the fixture splits the wall into. The apex is what
/// this measures: an apex left on the placement plane makes the wall run
/// from radius 0 to radius 6 and the area comes out 60*pi.
const EXPECTED_LATERAL_AREA: f64 = std::f64::consts::PI * 300.0;

fn read_fixture() -> (Topology, SolidId) {
    let text = std::fs::read_to_string(fixture(FIXTURE)).unwrap();
    assert!(
        text.contains("CONICAL_SURFACE('Cone.1', #48, 1.200000000000000E1,"),
        "the fixture must keep its non-zero base_radius"
    );
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(&text, &mut topo).unwrap();
    assert_eq!(solids.len(), 1, "expected exactly one solid");
    let solid = solids[0];
    (topo, solid)
}

fn cone_faces(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&fid| matches!(topo.face(fid).unwrap().surface(), FaceSurface::Cone(_)))
        .collect()
}

/// The measured volume against the closed form.
///
/// The fixture states its wall as two cone faces on purpose. `solid_volume`
/// recognises a shell of exactly ONE cone plus planar caps as a primitive
/// frustum and reads the cap radii straight off the trim circles, never
/// touching the conical surface — so a single-face frustum measures 1824*pi
/// with the apex in the wrong place and this assertion would prove nothing.
/// Two cone faces defer the measurement to the paths that integrate the
/// surface itself.
#[test]
fn frustum_measures_its_closed_form_volume() {
    let (topo, solid) = read_fixture();

    let faces = explorer::solid_faces(&topo, solid).unwrap();
    assert_eq!(faces.len(), 4, "frustum: two wall halves and two caps");
    assert_eq!(cone_faces(&topo, solid).len(), 2, "wall halves");

    let volume = measure::solid_volume(&topo, solid, DEFLECTION).unwrap();
    let rel_error = (volume - EXPECTED_VOLUME).abs() / EXPECTED_VOLUME;
    assert!(
        rel_error < 1e-3,
        "volume {volume} differs from {EXPECTED_VOLUME} (rel {rel_error})"
    );
}

/// The lateral area against its closed form: the tightest measurement of the
/// surface itself, since it integrates the wall and nothing else.
#[test]
fn frustum_wall_measures_its_closed_form_area() {
    let (topo, solid) = read_fixture();

    let area: f64 = cone_faces(&topo, solid)
        .into_iter()
        .map(|fid| measure::face_area(&topo, fid, 0.001).unwrap())
        .sum();
    let rel_error = (area - EXPECTED_LATERAL_AREA).abs() / EXPECTED_LATERAL_AREA;
    assert!(
        rel_error < 1e-3,
        "lateral area {area} differs from {EXPECTED_LATERAL_AREA} (rel {rel_error})"
    );
}

/// The apex the surface actually carries, and the radii it produces at the
/// two planes the fixture trims it on.
#[test]
fn frustum_cone_apex_sits_below_the_placement_plane() {
    let (topo, solid) = read_fixture();

    for fid in cone_faces(&topo, solid) {
        let FaceSurface::Cone(cone) = topo.face(fid).unwrap().surface() else {
            unreachable!("filtered to cone faces")
        };

        let apex = cone.apex();
        assert!(
            apex.x().abs() < 1e-9 && apex.y().abs() < 1e-9 && (apex.z() + 16.0).abs() < 1e-9,
            "apex should be (0,0,-16), got ({}, {}, {})",
            apex.x(),
            apex.y(),
            apex.z(),
        );

        // `radius_at` takes a generator distance; `h/sin(half_angle)` reaches
        // axial distance `h` from the apex.
        let sin_half = cone.half_angle().sin();
        for (z, expected) in [(0.0_f64, 12.0_f64), (8.0, 18.0)] {
            let radius = cone.radius_at((z + 16.0) / sin_half);
            assert!(
                (radius - expected).abs() < 1e-9,
                "radius at z={z} should be {expected}, got {radius}"
            );
        }
    }
}

/// Writing the imported frustum out and reading it back reproduces the same
/// solid. The written file states the cone at its apex with a zero radius,
/// which is the same surface said the other legal way.
#[test]
fn frustum_round_trips_through_the_writer() {
    let (topo, solid) = read_fixture();
    let volume = measure::solid_volume(&topo, solid, DEFLECTION).unwrap();
    let center = measure::solid_center_of_mass(&topo, solid, DEFLECTION).unwrap();

    let written = remus_io::step::write_step(&topo, &[solid]).unwrap();
    assert!(
        written.contains("CONICAL_SURFACE"),
        "the written file should carry the cone"
    );

    let mut round_topo = Topology::new();
    let round_solids = remus_io::step::reader::read_step(&written, &mut round_topo).unwrap();
    assert_eq!(round_solids.len(), 1);
    assert_eq!(
        explorer::solid_faces(&round_topo, round_solids[0])
            .unwrap()
            .len(),
        4,
        "face count changed over the round trip"
    );

    let round_volume = measure::solid_volume(&round_topo, round_solids[0], DEFLECTION).unwrap();
    assert!(
        (round_volume - volume).abs() / volume < 1e-9,
        "volume changed over the round trip: {volume} then {round_volume}"
    );

    let round_center =
        measure::solid_center_of_mass(&round_topo, round_solids[0], DEFLECTION).unwrap();
    assert!(
        (round_center.x() - center.x()).abs() < 1e-9
            && (round_center.y() - center.y()).abs() < 1e-9
            && (round_center.z() - center.z()).abs() < 1e-9,
        "centre of mass moved over the round trip"
    );
}
