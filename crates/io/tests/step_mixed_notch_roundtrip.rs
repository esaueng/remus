//! STEP export/import agreement for the mixed-notch L-bracket boundary.
//!
//! The extruded L-bracket (XY polygon
//! `[(0,0),(40,0),(40,8),(8,8),(8,50),(0,50)]`, 20 mm along +Z) round-trips
//! through STEP with identical topology (8 faces, 18 edges), volume
//! (13120 mm^3), and validity — and the reimported solid keeps the exact
//! kernel boundary contract: whole-edge fillet at r=1 still refuses typed
//! (`unsupported-vertex-blend`, input intact), whole-edge chamfer at d=1
//! still builds with the closed-form volume, and the chamfered solid
//! itself round-trips with volume agreement.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{chamfer_v2, fillet_cascade};
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::query::filter_filletable_edges;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;

fn l_bracket(topo: &mut Topology) -> SolidId {
    let profile = make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(40.0, 0.0, 0.0),
            Point3::new(40.0, 8.0, 0.0),
            Point3::new(8.0, 8.0, 0.0),
            Point3::new(8.0, 50.0, 0.0),
            Point3::new(0.0, 50.0, 0.0),
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
    extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 20.0).unwrap()
}

fn reimport(step: &str) -> (Topology, SolidId) {
    let mut imported = Topology::new();
    let solids = remus_io::step::reader::read_step(step, &mut imported).unwrap();
    assert_eq!(solids.len(), 1, "STEP must carry exactly one solid");
    (imported, solids[0])
}

/// Export/reimport preserves topology, volume, and validity.
#[test]
fn mixed_notch_bracket_step_round_trip_preserves_shape() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();

    let (imported, back) = reimport(&step);
    assert_eq!(solid_faces(&imported, back).unwrap().len(), 8);
    assert_eq!(solid_edges(&imported, back).unwrap().len(), 18);
    let volume = solid_volume(&imported, back, 0.01).unwrap();
    assert!(
        (volume - 13120.0).abs() < 0.5,
        "reimported volume must be 13120, got {volume:.4}"
    );
    let report = validate_solid(&imported, back, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "reimported solid must validate");
}

/// The reimported solid keeps the fillet success (torus corners, closed
/// form) and the chamfer success; the filleted solid round-trips too.
#[test]
fn reimported_bracket_keeps_boundary_contracts() {
    let mut topo = Topology::new();
    let solid = l_bracket(&mut topo);
    let step = remus_io::step::writer::write_step(&topo, &[solid]).unwrap();
    let (mut imported, back) = reimport(&step);

    let all = solid_edges(&imported, back).unwrap();
    let physical = filter_filletable_edges(&imported, back, &all).unwrap();
    assert_eq!(physical.len(), 18, "reimport must keep all physical edges");

    let filleted = fillet_cascade(&mut imported, back, &physical, 1.0)
        .unwrap_or_else(|e| panic!("reimported whole-edge fillet must build: {e}"));
    assert!(!filleted.is_partial);
    let fvol = solid_volume(&imported, filleted.solid, 0.01).unwrap();
    assert!(
        (fvol - 13027.2829).abs() < 0.2,
        "reimported fillet must meet the closed form, got {fvol:.4}"
    );

    let chamfered = chamfer_v2(&mut imported, back, &physical, 1.0, 1.0)
        .unwrap_or_else(|e| panic!("reimported whole-edge chamfer must build: {e}"));
    assert!(!chamfered.is_partial);
    let cvol = solid_volume(&imported, chamfered.solid, 0.01).unwrap();
    assert!(
        (cvol - 12905.3333).abs() < 0.5,
        "reimported chamfer must meet the closed form, got {cvol:.4}"
    );

    // The filleted solid itself round-trips with volume agreement.
    let step2 = remus_io::step::writer::write_step(&imported, &[filleted.solid]).unwrap();
    let (imported2, back2) = reimport(&step2);
    let fvol2 = solid_volume(&imported2, back2, 0.01).unwrap();
    assert!(
        (fvol2 - fvol).abs() < 0.1,
        "fillet round-trip must agree: {fvol:.4} vs {fvol2:.4}"
    );
    let report = validate_solid(&imported2, back2, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "reimported fillet must validate");
}
