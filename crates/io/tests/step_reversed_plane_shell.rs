//! A STEP box whose planes are written with `same_sense = .F.` shells like
//! the native box (OpenZCAD #1247).
//!
//! An independent AP214 producer writes three of the six box faces against their plane's normal. They
//! import as reversed faces, which the shell's fold detector misread as
//! inverted walls, so a 2 mm top-open shell refused with "outside the
//! qualified uniform-L collapse cell" while the same box written by remus
//! (all `.T.`) succeeded. The fixture is an independent AP214 export, including
//! its `SURFACE_CURVE` p-curves. Only its header is normalized.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_operations::measure::solid_volume;
use remus_operations::shell_op::shell;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const REFERENCE_BOX: &str = include_str!("data/reference_box_20x16x12.step");

fn import(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// The planar face whose effective outward normal is +Z at `z`.
fn cap(topo: &Topology, solid: SolidId, z: f64) -> FaceId {
    let caps: Vec<_> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            let face_data = topo.face(face).unwrap();
            let FaceSurface::Plane { normal, d } = face_data.surface() else {
                return false;
            };
            let sense = if face_data.is_reversed() { -1.0 } else { 1.0 };
            (normal.z() * sense - 1.0).abs() < 1e-12 && (d * sense - z).abs() < 1e-9
        })
        .collect();
    assert_eq!(caps.len(), 1);
    caps[0]
}

fn assert_shells(topo: &mut Topology, solid: SolidId, label: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    assert!((solid_volume(topo, solid, 0.005).unwrap() - 3840.0).abs() < 1e-9);
    let top = cap(topo, solid, 6.0);
    // 3840 - 16 x 12 x 10 inner pocket open through the top.
    let open = shell(topo, solid, 2.0, &[top]).unwrap();
    let report = validate_solid(topo, open).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let volume = solid_volume(topo, open, 0.005).unwrap();
    assert!(
        (volume - 1920.0).abs() <= 1920.0 * 1e-9,
        "{label}: {volume}"
    );
    // 3840 - 16 x 12 x 8 closed cavity.
    let closed = shell(topo, solid, 2.0, &[]).unwrap();
    let volume = solid_volume(topo, closed, 0.005).unwrap();
    assert!(
        (volume - 2304.0).abs() <= 2304.0 * 1e-9,
        "{label}: {volume}"
    );
}

#[test]
fn reference_box_with_reversed_planes_shells_like_the_native_box() {
    assert!(REFERENCE_BOX.matches(",.F.);").count() >= 3);
    let (mut topo, solid) = import(REFERENCE_BOX);
    let reversed = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| topo.face(face).unwrap().is_reversed())
        .count();
    assert_eq!(
        reversed, 3,
        "the fixture must keep its same_sense = .F. faces"
    );
    assert_shells(&mut topo, solid, "reference");

    // The writer's own all-`.T.` control and its re-import still succeed.
    let written = write_step(&topo, &[solid]).unwrap();
    let (mut again, solid) = import(&written);
    assert_shells(&mut again, solid, "remus round trip");
}
