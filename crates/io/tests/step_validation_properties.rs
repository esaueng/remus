//! CAx-IF STEP geometric validation-property contract tests.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use remus_io::IoError;
use remus_io::step::{
    StepValidationDiagnosticCode, StepValidationOptions, StepWriteOptions, read_step,
    read_step_with_validation, write_step, write_step_with_options,
};
use remus_operations::measure::mass_properties;
use remus_operations::primitives::make_box;
use remus_topology::Topology;

const GVP_HEADER: &str =
    "CAx-IF Rec.Pracs.---Geometric and Assembly Validation Properties---4.6---2023-04-21";

fn write_step_with_validation(
    topo: &Topology,
    solids: &[remus_topology::solid::SolidId],
) -> String {
    write_step_with_options(
        topo,
        solids,
        &StepWriteOptions {
            validation_properties: true,
            ..StepWriteOptions::default()
        },
    )
    .unwrap()
}

fn assert_close(actual: f64, expected: f64, tolerance: f64, label: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{label}: expected {expected}, got {actual}"
    );
}

fn replace_nth_measure(step: &str, name: &str, occurrence: usize, value: f64) -> String {
    let marker = format!(
        "'{name}', {}_MEASURE(",
        if name == "volume measure" {
            "VOLUME"
        } else {
            "AREA"
        }
    );
    let start = step
        .match_indices(&marker)
        .nth(occurrence)
        .map(|(index, _)| index + marker.len())
        .unwrap_or_else(|| panic!("measure occurrence {occurrence} for {name} not found"));
    let end = step[start..]
        .find(')')
        .map(|offset| start + offset)
        .expect("measure literal terminator");
    format!("{}{value:.17E}{}", &step[..start], &step[end..])
}

fn split_geometry_declaration(step: &str) -> String {
    let representation_line = step
        .lines()
        .filter(|line| line.contains(" = REPRESENTATION('', ("))
        .nth(1)
        .expect("geometry validation representation");
    let representation_id = representation_line
        .split_once(" =")
        .expect("representation id")
        .0;
    let items_start = representation_line.find("('', (").unwrap() + "('', (".len();
    let items_end = representation_line[items_start..].find("), #").unwrap() + items_start;
    let items: Vec<&str> = representation_line[items_start..items_end]
        .split(',')
        .map(str::trim)
        .collect();
    assert_eq!(items.len(), 3);
    let context_start = items_end + "), #".len();
    let context_end = representation_line[context_start..].find(')').unwrap() + context_start;
    let context = &representation_line[context_start..context_end];
    let property_line = step
        .lines()
        .filter(|line| line.contains("PROPERTY_DEFINITION('geometric validation property'"))
        .nth(1)
        .expect("geometry validation property");
    let target_start = property_line.rfind('#').unwrap();
    let target_end = property_line[target_start..].find(')').unwrap() + target_start;
    let target = &property_line[target_start..target_end];

    let volume_only = format!(
        "{representation_id} = REPRESENTATION('volume', ({}), #{context});",
        items[0]
    );
    let separated = step.replacen(representation_line, &volume_only, 1);
    let additions = format!(
        "#9000001 = REPRESENTATION('surface area', ({}), #{context});\n\
         #9000002 = PROPERTY_DEFINITION('geometric validation property', 'area', {target});\n\
         #9000003 = PROPERTY_DEFINITION_REPRESENTATION(#9000002, #9000001);\n\
         #9000004 = REPRESENTATION('centroid', ({}), #{context});\n\
         #9000005 = PROPERTY_DEFINITION('geometric validation property', 'centroid', {target});\n\
         #9000006 = PROPERTY_DEFINITION_REPRESENTATION(#9000005, #9000004);\n",
        items[1], items[2]
    );
    let insertion = separated.rfind("ENDSEC;").expect("DATA ENDSEC");
    format!(
        "{}{}{}",
        &separated[..insertion],
        additions,
        &separated[insertion..]
    )
}

fn reroute_derived_unit(step: &str, unit_type: &str, base_ref: u64) -> String {
    let marker = format!(" = {unit_type}((#");
    let unit_line = step
        .lines()
        .find(|line| line.contains(&marker))
        .unwrap_or_else(|| panic!("{unit_type} declaration"));
    let element_start = unit_line.find(&marker).unwrap() + marker.len();
    let element_end = unit_line[element_start..].find(')').unwrap() + element_start;
    let element_ref = &unit_line[element_start..element_end];
    let element_marker = format!("#{element_ref} = DERIVED_UNIT_ELEMENT(#");
    let element_line = step
        .lines()
        .find(|line| line.starts_with(&element_marker))
        .unwrap_or_else(|| panic!("{unit_type} derived element"));
    let old_base_start = element_marker.len();
    let old_base_end = element_line[old_base_start..].find(',').unwrap() + old_base_start;
    let replacement = format!(
        "{}{}{}",
        &element_line[..old_base_start],
        base_ref,
        &element_line[old_base_end..]
    );
    step.replacen(element_line, &replacement, 1)
}

#[test]
fn cax_if_properties_round_trip_per_solid_against_analytic_oracles() {
    let mut source = Topology::new();
    let first = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let second = make_box(&mut source, 1.0, 1.0, 1.0).unwrap();
    let step = write_step_with_validation(&source, &[first, second]);

    assert!(step.contains(GVP_HEADER));
    assert!(step.contains("ID_ATTRIBUTE("));
    assert!(step.contains("SHAPE_ASPECT("));
    assert!(step.contains("'Shape for Validation Properties'"));
    assert!(step.contains("DERIVED_UNIT_ELEMENT("));
    assert!(step.contains("AREA_UNIT((#"));
    assert!(step.contains("VOLUME_UNIT((#"));
    assert!(step.contains("'SQUARE MILLIMETRE'"));
    assert!(step.contains("'CUBIC MILLIMETRE'"));
    assert_eq!(
        step.matches("PROPERTY_DEFINITION('geometric validation property'")
            .count(),
        3,
        "one part-level aggregate plus one declaration per solid"
    );

    let mut imported = Topology::new();
    let result = read_step_with_validation(
        &step,
        &mut imported,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert_eq!(result.solids().len(), 2);
    assert!(result.diagnostics().is_empty());
    assert_eq!(result.validation().len(), 2);
    assert!(
        result
            .validation()
            .iter()
            .all(|report| report.diagnostics.is_empty())
    );

    let expected = [(24.0, 52.0, [1.0, 1.5, 2.0]), (1.0, 6.0, [0.5, 0.5, 0.5])];
    for (report, (volume, area, centroid)) in result.validation().iter().zip(expected) {
        let declared = report.declared.expect("writer declaration");
        for values in [declared, report.recomputed] {
            assert_close(values.volume, volume, 1e-9, "volume");
            assert_close(values.surface_area, area, 1e-9, "surface area");
            for (axis, expected_axis) in values.centroid.into_iter().zip(centroid) {
                assert_close(axis, expected_axis, 1e-9, "centroid");
            }
        }
    }
}

#[test]
fn relative_volume_bound_accepts_just_inside_and_reports_just_outside() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let step = write_step_with_validation(&source, &[solid]);

    // Occurrence zero is the part-level aggregate; occurrence one is the
    // geometry-level declaration the per-solid reader checks.
    let inside = replace_nth_measure(&step, "volume measure", 1, 24.0 * 1.0049);
    let mut inside_topology = Topology::new();
    let inside_result = read_step_with_validation(
        &inside,
        &mut inside_topology,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert!(inside_result.validation()[0].diagnostics.is_empty());

    let outside = replace_nth_measure(&step, "volume measure", 1, 24.0 * 1.006);
    let mut outside_topology = Topology::new();
    let outside_result = read_step_with_validation(
        &outside,
        &mut outside_topology,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert_eq!(outside_result.validation()[0].diagnostics.len(), 1);
    assert_eq!(
        outside_result.validation()[0].diagnostics[0].code,
        StepValidationDiagnosticCode::VolumeDeviation
    );
    assert_eq!(
        outside_result.validation()[0].diagnostics[0].category,
        "tolerance_violation"
    );
}

#[test]
fn reader_merges_standard_separate_property_representations() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let step = split_geometry_declaration(&write_step_with_validation(&source, &[solid]));
    let mut imported = Topology::new();
    let result = read_step_with_validation(
        &step,
        &mut imported,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert!(result.validation()[0].diagnostics.is_empty());
    assert_close(
        result.validation()[0].declared.unwrap().volume,
        24.0,
        1e-9,
        "separate declaration volume",
    );
}

#[test]
fn reader_resolves_explicit_square_and_cubic_metre_units() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let mut step = write_step_with_validation(&source, &[solid]);
    step = reroute_derived_unit(&step, "AREA_UNIT", 9_000_010);
    step = reroute_derived_unit(&step, "VOLUME_UNIT", 9_000_010);
    let insertion = step.rfind("ENDSEC;").unwrap();
    step.insert_str(
        insertion,
        "#9000010 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );\n",
    );
    for occurrence in (0..=1).rev() {
        step = replace_nth_measure(&step, "volume measure", occurrence, 24e-9);
        step = replace_nth_measure(&step, "surface area measure", occurrence, 52e-6);
    }

    let mut imported = Topology::new();
    let result = read_step_with_validation(
        &step,
        &mut imported,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert!(result.validation()[0].diagnostics.is_empty());
    let declared = result.validation()[0].declared.unwrap();
    assert_close(declared.volume, 24.0, 1e-9, "cubic metre conversion");
    assert_close(declared.surface_area, 52.0, 1e-9, "square metre conversion");
}

#[test]
fn malformed_declaration_refuses_transactionally_but_opt_out_stays_tolerant() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let step = write_step_with_validation(&source, &[solid]);
    let marker = "'surface area measure', AREA_MEASURE(";
    let second = step
        .match_indices(marker)
        .nth(1)
        .map(|(index, _)| index)
        .expect("geometry-level area declaration");
    let malformed = format!(
        "{}{}{}",
        &step[..second],
        "'surface area measure', VOLUME_MEASURE(",
        &step[second + marker.len()..]
    );

    let mut checked = Topology::new();
    let sentinel = make_box(&mut checked, 1.0, 2.0, 3.0).unwrap();
    let error = read_step_with_validation(
        &malformed,
        &mut checked,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap_err();
    assert!(matches!(
        error,
        IoError::InvalidValidationProperties {
            code: "step_validation_invalid_measure",
            ..
        }
    ));
    assert_close(
        mass_properties(&checked, sentinel).unwrap().mass,
        6.0,
        1e-12,
        "sentinel volume after refusal",
    );

    let mut unchecked = Topology::new();
    assert_eq!(read_step(&malformed, &mut unchecked).unwrap().len(), 1);
}

#[test]
fn writer_default_preserves_legacy_output_and_checker_reports_missing_properties() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let step = write_step(&source, &[solid]).unwrap();
    assert!(!step.contains(GVP_HEADER));
    assert!(!step.contains("MEASURE_REPRESENTATION_ITEM"));

    let mut imported = Topology::new();
    let result = read_step_with_validation(
        &step,
        &mut imported,
        remus_io::ImportLimits::default(),
        StepValidationOptions::default(),
    )
    .unwrap();
    assert_eq!(result.validation()[0].diagnostics.len(), 1);
    assert_eq!(
        result.validation()[0].diagnostics[0].code,
        StepValidationDiagnosticCode::PropertiesMissing
    );
}

#[test]
fn invalid_validation_bounds_are_typed_and_non_mutating() {
    let mut source = Topology::new();
    let solid = make_box(&mut source, 2.0, 3.0, 4.0).unwrap();
    let step = write_step_with_validation(&source, &[solid]);
    let mut imported = Topology::new();
    let options = StepValidationOptions {
        volume_relative: -1.0,
        ..StepValidationOptions::default()
    };
    let error = read_step_with_validation(
        &step,
        &mut imported,
        remus_io::ImportLimits::default(),
        options,
    )
    .unwrap_err();
    assert!(matches!(
        error,
        IoError::InvalidValidationProperties {
            code: "step_validation_invalid_options",
            ..
        }
    ));
}
