//! Mass properties must read nested inner wires as alternating void and
//! material.
//!
//! `oring_nested_holes.step` is a rounded-square block (half-width 62.75,
//! corner radius 3.75, z 0 to 21) with a rounded-square through hole
//! (half-width 21.25, radius 3.75) and a pocket from z = 1.2 between the
//! outline at half-width 61.55 (radius 2.55) and a standing island at
//! half-width 22.45 (radius 3.75). Its z = 21 top therefore has three nested
//! inner wires: the pocket opening holds the island, which holds the through
//! hole. Subtracting every inner wire read that face as 3207.97 (the absolute
//! value of 590.15 − 2003.94 − 1794.18) and the solid's volume as 18543.46.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::{measure, tessellate};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

/// Area of a rounded square of half-width `h` and corner radius `r`.
fn rounded_square(h: f64, r: f64) -> f64 {
    (2.0 * h).powi(2) - (4.0 - std::f64::consts::PI) * r * r
}

fn load() -> (Topology, SolidId) {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/oring_nested_holes.step"
    );
    let mut topo = Topology::new();
    let text = std::fs::read_to_string(path).unwrap();
    let solid = remus_io::step::reader::read_step(&text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn nested_top(topo: &Topology, solid: SolidId) -> FaceId {
    let faces: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| topo.face(face).unwrap().inner_wires().len() == 3)
        .collect();
    assert_eq!(faces.len(), 1, "one face carries the nested inner wires");
    faces[0]
}

fn mesh_area(topo: &Topology, face: FaceId, deflection: f64) -> f64 {
    let mesh = tessellate::tessellate(topo, face, deflection).unwrap();
    mesh.indices
        .chunks(3)
        .map(|t| {
            let p = |k: usize| mesh.positions[t[k] as usize];
            (p(1) - p(0)).cross(p(2) - p(0)).length() * 0.5
        })
        .sum()
}

#[test]
fn nested_top_face_area_matches_closed_form_and_mesh() {
    let (topo, solid) = load();
    let top = nested_top(&topo, solid);
    // Outline less the pocket opening, plus the island less the hole.
    let want = rounded_square(62.75, 3.75) - rounded_square(61.55, 2.55)
        + rounded_square(22.45, 3.75)
        - rounded_square(21.25, 3.75);
    let area = measure::face_area(&topo, top, 0.01).unwrap();
    assert!(
        (area - want).abs() <= 1e-9 * want,
        "face area {area} != closed form {want}"
    );
    // The mesh chords the corner arcs. Each chord cuts off a circular
    // segment no wider than the deflection, so the mesh is within the
    // deflection times the loops' total arc length (three loops of radius
    // 3.75 and one of 2.55, a full turn each).
    let arc_length = std::f64::consts::TAU * (3.0 * 3.75 + 2.55);
    for deflection in [0.01, 0.001] {
        let mesh = mesh_area(&topo, top, deflection);
        assert!(
            (mesh - area).abs() <= deflection * arc_length,
            "mesh area {mesh} at {deflection} vs face area {area}"
        );
    }
}

#[test]
fn nested_hole_solid_volume_and_centroid_match_closed_form() {
    let (topo, solid) = load();
    // The block less its through hole, z 0 to 21; the pocket, z 1.2 to 21.
    let block = 21.0 * (rounded_square(62.75, 3.75) - rounded_square(21.25, 3.75));
    let pocket = 19.8 * (rounded_square(61.55, 2.55) - rounded_square(22.45, 3.75));
    let want = block - pocket;
    let volume = measure::solid_volume(&topo, solid, 0.01).unwrap();
    assert!(
        (volume - want).abs() <= 1e-9 * want,
        "volume {volume} != closed form {want}"
    );
    // Symmetric about the z axis; the block's centroid sits at z = 10.5 and
    // the pocket's at z = 1.2 + 19.8 / 2.
    let want_z = (block * 10.5 - pocket * 11.1) / want;
    let com = measure::solid_center_of_mass(&topo, solid, 0.01).unwrap();
    let want_com = Point3::new(0.0, 0.0, want_z);
    assert!(
        (com - want_com).length() <= 1e-9 * 62.75,
        "centre of mass {com:?} != closed form {want_com:?}"
    );
}

/// `face_area`, `solid_volume` and `mass_properties` agree with each other
/// and with the closed forms, and the face-integral cache reads them warm
/// exactly as cold.
#[test]
fn nested_hole_measures_agree_and_read_the_same_warm() {
    use remus_check::properties::face_cache::{
        clear_thread_face_cache, enable_thread_face_cache, set_thread_face_cache_limits,
    };
    let (topo, solid) = load();
    let top = nested_top(&topo, solid);
    let block = 21.0 * (rounded_square(62.75, 3.75) - rounded_square(21.25, 3.75));
    let pocket = 19.8 * (rounded_square(61.55, 2.55) - rounded_square(22.45, 3.75));
    let volume = block - pocket;
    let top_area = rounded_square(62.75, 3.75) - rounded_square(61.55, 2.55)
        + rounded_square(22.45, 3.75)
        - rounded_square(21.25, 3.75);
    let read = || {
        (
            measure::face_area(&topo, top, 0.01).unwrap(),
            measure::solid_volume(&topo, solid, 0.01).unwrap(),
            measure::mass_properties(&topo, solid).unwrap().mass,
        )
    };
    let cold = read();
    assert!((cold.0 - top_area).abs() <= 1e-9 * top_area, "{cold:?}");
    assert!((cold.1 - volume).abs() <= 1e-9 * volume, "{cold:?}");
    assert!((cold.2 - volume).abs() <= 1e-9 * volume, "{cold:?}");
    enable_thread_face_cache();
    clear_thread_face_cache();
    let primed = read();
    let warm = read();
    set_thread_face_cache_limits(0, 0);
    for (label, reading) in [("primed", primed), ("warm", warm)] {
        assert_eq!(reading.0.to_bits(), cold.0.to_bits(), "{label} face_area");
        assert_eq!(
            reading.1.to_bits(),
            cold.1.to_bits(),
            "{label} solid_volume"
        );
        assert_eq!(
            reading.2.to_bits(),
            cold.2.to_bits(),
            "{label} mass_properties"
        );
    }
}

/// With the nesting left undecided (no work allowed), the top face and the
/// solid are measured on their meshes, whose parity the tessellator decides:
/// still the closed forms, to the chord error; `mass_properties`, which has
/// no mesh route, refuses rather than guessing.
#[test]
fn an_undecided_nesting_falls_back_to_the_mesh() {
    use remus_check::properties::loop_nesting::with_nesting_work_limit;
    let (topo, solid) = load();
    let top = nested_top(&topo, solid);
    let top_area = rounded_square(62.75, 3.75) - rounded_square(61.55, 2.55)
        + rounded_square(22.45, 3.75)
        - rounded_square(21.25, 3.75);
    let block = 21.0 * (rounded_square(62.75, 3.75) - rounded_square(21.25, 3.75));
    let pocket = 19.8 * (rounded_square(61.55, 2.55) - rounded_square(22.45, 3.75));
    let volume = block - pocket;
    let arc_length = std::f64::consts::TAU * (3.0 * 3.75 + 2.55);
    let deflection = 0.001;
    let (area, solid_volume, mass) = with_nesting_work_limit(0, || {
        (
            measure::face_area(&topo, top, deflection).unwrap(),
            measure::solid_volume(&topo, solid, deflection).unwrap(),
            measure::mass_properties(&topo, solid),
        )
    });
    assert!(
        (area - top_area).abs() <= deflection * arc_length,
        "fallback area {area} against {top_area}"
    );
    let mesh = mesh_area(&topo, top, deflection);
    assert!(
        (area - mesh).abs() <= deflection * arc_length,
        "fallback area {area} against the tessellator's {mesh}"
    );
    // Every face's chord error, over the solid's height.
    assert!(
        (solid_volume - volume).abs() <= deflection * arc_length * 2.0 * 21.0,
        "fallback volume {solid_volume} against {volume}"
    );
    assert!(
        matches!(
            mass,
            Err(remus_operations::OperationsError::Check(
                remus_check::CheckError::UndecidedLoopNesting(_)
            ))
        ),
        "{mass:?}"
    );
}
