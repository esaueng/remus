//! First-class STEP sheet-body exchange contracts.

#![allow(clippy::expect_used, clippy::unwrap_used)]

use remus_math::nurbs::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::{BodyClass, BodyId, Topology};

fn make_trimmed_nurbs_sheet(topo: &mut Topology) -> remus_topology::shell::ShellId {
    let face = remus_topology::builder::make_rectangle_face(topo, 2.0, 1.0, 1e-7).unwrap();
    let surface = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(-1.0, -0.5, 0.0), Point3::new(1.0, -0.5, 0.0)],
            vec![Point3::new(-1.0, 0.5, 0.0), Point3::new(1.0, 0.5, 0.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap();
    topo.face_mut(face)
        .unwrap()
        .set_surface(remus_topology::face::FaceSurface::Nurbs(surface));
    remus_operations::sew::make_sheet_body(topo, &[face]).unwrap()
}

#[test]
fn trimmed_nurbs_sheet_round_trips_as_open_surface_model() {
    let mut source = Topology::new();
    let sheet = make_trimmed_nurbs_sheet(&mut source);

    let step = remus_io::step::write_step_sheets(&source, &[sheet]).unwrap();
    assert!(step.contains("SHELL_BASED_SURFACE_MODEL("));
    assert!(step.contains("OPEN_SHELL("));
    assert!(!step.contains("MANIFOLD_SOLID_BREP("));

    let mut restored = Topology::new();
    let result = remus_io::step::read_step_bodies(&step, &mut restored).unwrap();
    assert!(result.solids().is_empty());
    assert_eq!(result.sheets().len(), 1);
    let restored_sheet = result.sheets()[0];
    assert_eq!(
        restored
            .body_class_of(BodyId::Shell(restored_sheet))
            .unwrap(),
        BodyClass::Sheet
    );

    let report = remus_check::validate::validate_sheet_body(
        &restored,
        restored_sheet,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);
    assert_eq!(report.error_count(), 0);
    assert!(
        report.warning_count() > 0,
        "open-by-design must be reported"
    );

    let area = remus_operations::measure::body_surface_area(
        &restored,
        BodyId::Shell(restored_sheet),
        0.05,
    )
    .unwrap();
    assert!((area - 2.0).abs() < 1e-10, "area={area}");
    let bounds = remus_operations::measure::sheet_bounding_box(&restored, restored_sheet).unwrap();
    assert_eq!(bounds.min, Point3::new(-1.0, -0.5, 0.0));
    assert_eq!(bounds.max, Point3::new(1.0, 0.5, 0.0));

    let second = remus_io::step::write_step_sheets(&restored, &[restored_sheet]).unwrap();
    assert_eq!(second, step, "sheet STEP exchange must be deterministic");
}

#[test]
fn body_aware_round_trip_keeps_solid_and_sheet_roots_distinct() {
    let mut source = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut source, 1.0, 2.0, 3.0).unwrap();
    let sheet = make_trimmed_nurbs_sheet(&mut source);
    let step = remus_io::step::write_step_bodies(&source, &[solid], &[sheet]).unwrap();
    assert!(step.contains("MANIFOLD_SOLID_BREP("));
    assert!(step.contains("SHELL_BASED_SURFACE_MODEL("));

    let mut restored = Topology::new();
    let result = remus_io::step::read_step_bodies(&step, &mut restored).unwrap();
    assert_eq!(result.solids().len(), 1);
    assert_eq!(result.sheets().len(), 1);
    assert_eq!(
        restored
            .body_class_of(BodyId::Shell(result.sheets()[0]))
            .unwrap(),
        BodyClass::Sheet
    );
}

#[test]
fn legacy_solid_only_reader_ignores_sheet_roots_without_allocating_them() {
    let mut source = Topology::new();
    let sheet = make_trimmed_nurbs_sheet(&mut source);
    let step = remus_io::step::write_step_sheets(&source, &[sheet]).unwrap();

    let mut restored = Topology::new();
    let solids = remus_io::step::read_step(&step, &mut restored).unwrap();
    assert!(solids.is_empty());
    assert_eq!(restored.num_shells(), 0);
}

#[test]
fn closed_sheet_uses_closed_shell_without_becoming_a_solid() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let outer = topo.solid(solid).unwrap().outer_shell();
    let faces = topo.shell(outer).unwrap().faces().to_vec();
    let sheet = remus_operations::sew::make_sheet_body(&mut topo, &faces).unwrap();

    let step = remus_io::step::write_step_sheets(&topo, &[sheet]).unwrap();
    assert!(step.contains("CLOSED_SHELL("));
    assert!(!step.contains("OPEN_SHELL("));
    assert!(!step.contains("MANIFOLD_SOLID_BREP("));

    let mut restored = Topology::new();
    let result = remus_io::step::read_step_bodies(&step, &mut restored).unwrap();
    assert_eq!(result.sheets().len(), 1);
    assert!(
        remus_topology::validation::validate_shell_closed(
            restored.shell(result.sheets()[0]).unwrap(),
            &restored,
        )
        .is_ok()
    );
    let report = remus_check::validate::validate_sheet_body(
        &restored,
        result.sheets()[0],
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{:#?}", report.issues);
    assert_eq!(
        report.warning_count(),
        0,
        "closed sheet has no free boundary"
    );
}

#[test]
fn wrong_body_class_and_malformed_constituent_fail_closed() {
    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let solid_shell = topo.solid(solid).unwrap().outer_shell();
    let error = remus_io::step::write_step_sheets(&topo, &[solid_shell]).unwrap_err();
    assert!(matches!(
        error,
        remus_io::IoError::Topology(remus_topology::TopologyError::BodyClassMismatch {
            entity: "STEP sheet root",
            expected: "sheet",
            actual: "solid",
        })
    ));

    let sheet = make_trimmed_nurbs_sheet(&mut topo);
    let valid = remus_io::step::write_step_sheets(&topo, &[sheet]).unwrap();
    let malformed = valid.replacen("OPEN_SHELL", "EDGE_LOOP", 1);
    let mut destination = Topology::new();
    let sentinel = remus_operations::primitives::make_box(&mut destination, 2.0, 2.0, 2.0).unwrap();
    let counts = (
        destination.num_vertices(),
        destination.num_edges(),
        destination.num_faces(),
        destination.num_shells(),
        destination.num_solids(),
    );
    assert!(remus_io::step::read_step_bodies(&malformed, &mut destination).is_err());
    assert_eq!(
        (
            destination.num_vertices(),
            destination.num_edges(),
            destination.num_faces(),
            destination.num_shells(),
            destination.num_solids(),
        ),
        counts
    );
    assert!(destination.solid(sentinel).is_ok());
}

#[test]
fn solid_only_validation_properties_are_refused_for_sheet_documents() {
    let mut topo = Topology::new();
    let sheet = make_trimmed_nurbs_sheet(&mut topo);
    let options = remus_io::step::StepWriteOptions {
        validation_properties: true,
        ..Default::default()
    };
    let error =
        remus_io::step::write_step_bodies_with_options(&topo, &[], &[sheet], &options).unwrap_err();
    assert!(matches!(error, remus_io::IoError::InvalidTopology { .. }));
    assert!(
        error
            .to_string()
            .contains("validation properties are not defined for sheet bodies")
    );
}
