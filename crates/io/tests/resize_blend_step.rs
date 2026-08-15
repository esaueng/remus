//! STEP regressions for exact analytic blend resizing and refusal.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use brepkit_check::validate::{ValidateOptions, validate_solid};
use brepkit_io::step::reader::read_step;
use brepkit_io::step::writer::write_step;
use brepkit_math::tolerance::Tolerance;
use brepkit_operations::blend_ops::fillet_v2;
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::make_box;
use brepkit_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use brepkit_topology::Topology;
use brepkit_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

const DEFLECTION: f64 = 0.01;

fn assert_valid(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).expect("validate solid");
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).expect("measure volume")
}

fn blend_face(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let matches: Vec<FaceId> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|face| match topo.face(*face).expect("face").surface() {
            FaceSurface::Cylinder(cylinder) => {
                Tolerance::new().approx_eq(cylinder.radius(), radius)
            }
            FaceSurface::Torus(torus) => Tolerance::new().approx_eq(torus.minor_radius(), radius),
            _ => false,
        })
        .collect();
    assert_eq!(matches.len(), 1, "one analytic r={radius} blend face");
    matches[0]
}

fn imported_box_fillet() -> String {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 10.0, 10.0, 10.0).expect("box");
    let edge = solid_edges(&topo, sharp).expect("box edges")[0];
    let filleted = fillet_v2(&mut topo, sharp, &[edge], 1.0)
        .expect("fillet box")
        .solid;
    write_step(&topo, &[filleted]).expect("write STEP")
}

#[test]
fn imported_step_box_blend_grows_shrinks_and_removes_exactly() {
    let step = imported_box_fillet();

    for new_radius in [2.0, 0.5, 0.0] {
        let mut topo = Topology::new();
        let input = read_step(&step, &mut topo).expect("read STEP")[0];
        assert_valid(&topo, input);
        let band = blend_face(&topo, input, 1.0);
        let before = volume(&topo, input);

        let result = resize_blend(&mut topo, input, band, 1.0, new_radius)
            .expect("resize imported blend")
            .solid;
        assert_valid(&topo, result);
        let after = volume(&topo, result);

        if Tolerance::new().approx_eq(new_radius, 0.0) {
            assert_eq!(solid_entity_counts(&topo, result).expect("counts").0, 6);
            assert!(Tolerance::new().approx_eq(after, 1000.0));
        } else {
            let _ = blend_face(&topo, result, new_radius);
            if new_radius > 1.0 {
                assert!(after < before, "growing a convex blend removes volume");
            } else {
                assert!(after > before, "shrinking a convex blend restores volume");
            }
        }
    }
}

#[test]
fn shapr3d_periodic_step_unfillets_exactly_and_refuses_unimplemented_resize() {
    let mut topo = Topology::new();
    let input = read_step(
        include_str!("data/shapr3d_walking_stick_foot.step"),
        &mut topo,
    )
    .expect("read Shapr3D STEP")[0];
    assert_valid(&topo, input);
    let band = solid_faces(&topo, input)
        .expect("faces")
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).expect("face").surface(),
                FaceSurface::Torus(torus)
                    if Tolerance::new().approx_eq(torus.minor_radius(), 4.0)
            )
        })
        .expect("Shapr3D torus band");
    let counts = solid_entity_counts(&topo, input).expect("counts");
    let before = volume(&topo, input);

    let error = resize_blend(&mut topo, input, band, 4.0, 3.0).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&error),
        "unsupported-support-pair"
    );
    assert_eq!(solid_entity_counts(&topo, input).expect("counts"), counts);
    assert!(Tolerance::new().approx_eq(volume(&topo, input), before));

    let removed =
        resize_blend(&mut topo, input, band, 4.0, 0.0).expect("remove Shapr3D cylinder-cone band");
    assert_valid(&topo, removed.solid);
    assert!(removed.evolution.origin.is_exact());
    assert!(removed.evolution.deleted.contains(&band.index()));
    assert_eq!(
        solid_entity_counts(&topo, removed.solid)
            .expect("result counts")
            .0,
        counts.0 - 1
    );

    let removed_counts = solid_entity_counts(&topo, removed.solid).expect("removed counts");
    let removed_volume = volume(&topo, removed.solid);
    let step = write_step(&topo, &[removed.solid]).expect("write unfilleted Shapr3D STEP");
    let mut roundtrip_topo = Topology::new();
    let roundtrip = read_step(&step, &mut roundtrip_topo).expect("re-read unfilleted STEP")[0];
    assert_valid(&roundtrip_topo, roundtrip);
    assert_eq!(
        solid_entity_counts(&roundtrip_topo, roundtrip)
            .expect("round-trip counts")
            .0,
        removed_counts.0
    );
    assert!(Tolerance::new().approx_eq(volume(&roundtrip_topo, roundtrip), removed_volume));
}

#[test]
fn occt_multi_fillet_step_refuses_without_mutating_input() {
    let mut topo = Topology::new();
    let input = read_step(
        include_str!("data/openzcad_e_analytic_fillet_plate.step"),
        &mut topo,
    )
    .expect("read Open CASCADE STEP")[0];
    assert_valid(&topo, input);
    let band = solid_faces(&topo, input)
        .expect("faces")
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).expect("face").surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), 3.0)
            )
        })
        .expect("Open CASCADE fillet band");
    let counts = solid_entity_counts(&topo, input).expect("counts");
    let before = volume(&topo, input);

    let error = resize_blend(&mut topo, input, band, 3.0, 2.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "resize-blend-failed");
    assert_eq!(solid_entity_counts(&topo, input).expect("counts"), counts);
    assert!(Tolerance::new().approx_eq(volume(&topo, input), before));
    assert_valid(&topo, input);
}
