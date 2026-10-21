//! Public-path assembly round trips and fail-closed graph/placement contracts.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Write as _;

use remus_io::{ImportLimits, IoError, arena_io::serialize_solid, step};
use remus_math::{mat::Mat4, vec::Point3};
use remus_operations::{assembly::Assembly, measure, primitives::make_box};
use remus_topology::{Topology, explorer::solid_faces, face::FaceSurface};

fn witness() -> (Topology, Assembly) {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1., 2., 3.).unwrap();
    let mut assembly = Assembly::new("fixture's assembly");
    let root = assembly.add_root_component(
        "left ' part",
        solid,
        Mat4::translation(10., 2., 0.) * Mat4::rotation_z(std::f64::consts::FRAC_PI_2),
    );
    assembly.add_root_component("right part", solid, Mat4::translation(0., 30., 0.));
    assembly
        .add_child_component(
            root,
            "nested part",
            solid,
            Mat4::translation(2., 0., 3.) * Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
        )
        .unwrap();
    (topo, assembly)
}

fn near(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && (actual - expected).abs() < 1e-8,
        "{actual} != {expected}"
    );
}

#[test]
fn step_assembly_roundtrip_preserves_nested_shared_definitions_and_rigid_placements() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 1);
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), 3);
    assert_eq!(text, step::write_step_assembly(&topo, &assembly).unwrap());
    let mut restored = Topology::new();
    let result = step::read_step_assembly(&text, &mut restored).unwrap();
    assert_eq!(result.solids.len(), 1);
    assert_eq!(result.assembly.name(), assembly.name());
    assert_eq!(result.assembly.component_count(), 3);
    assert_eq!(result.assembly.roots().len(), 2);
    near(result.length_scale_to_mm, 1.);
    let bom = result.assembly.bill_of_materials();
    assert_eq!(bom.len(), 1);
    assert_eq!(bom[0].name, assembly.bill_of_materials()[0].name);
    assert_eq!(bom[0].instance_count, 3);
    for (expected, actual) in assembly.flatten().iter().zip(result.assembly.flatten()) {
        assert_eq!(actual.0, result.solids[0]);
        for (a, b) in actual
            .1
            .0
            .iter()
            .flatten()
            .zip(expected.1.0.iter().flatten())
        {
            near(*a, *b);
        }
    }
    let parent = result
        .assembly
        .component(result.assembly.roots()[0])
        .unwrap();
    assert_eq!(parent.name, "left ' part");
    let nested = result.assembly.component(parent.children[0]).unwrap();
    assert_eq!(nested.name, "nested part");
    let point = result
        .assembly
        .world_transform(parent.children[0])
        .unwrap()
        .mul_point(Point3::new(1., 0., 0.));
    near(point.x(), 10.);
    near(point.y(), 4.);
    near(point.z(), 2.);
    near(
        measure::solid_volume(&restored, result.solids[0], 0.01).unwrap(),
        6.,
    );
    let faces = solid_faces(&restored, result.solids[0]).unwrap();
    assert_eq!(faces.len(), 6);
    assert!(faces.iter().all(|&face| matches!(
        restored.face(face).unwrap().surface(),
        FaceSurface::Plane { .. }
    )));
    let mut legacy = Topology::new();
    assert_eq!(step::read_step(&text, &mut legacy).unwrap().len(), 1);
}

#[test]
fn step_assembly_names_are_part21_encoded_and_decoded() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1., 2., 3.).unwrap();
    let mut assembly = Assembly::new("Baugruppe für café");
    let root = assembly.add_root_component("Gehäuse \\ 'A'", solid, Mat4::identity());
    assembly
        .add_child_component(root, "部品 😀 é", solid, Mat4::translation(5., 0., 0.))
        .unwrap();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    // The header declares edition 2: every string must be printable ASCII.
    assert!(text.is_ascii(), "edition-2 STEP must not carry raw UTF-8");
    assert!(text.contains(r"'Baugruppe f\X2\00FC\X0\r caf\X2\00E9\X0\'"));
    assert!(text.contains(r"'Geh\X2\00E4\X0\use \\ ''A'''"));
    assert!(text.contains(r"'\X2\90E854C1\X0\ \X4\0001F600\X0\ \X2\00E9\X0\'"));

    let result = step::read_step_assembly(&text, &mut Topology::new()).unwrap();
    assert_eq!(result.assembly.name(), "Baugruppe für café");
    let parent = result
        .assembly
        .component(result.assembly.roots()[0])
        .unwrap();
    assert_eq!(parent.name, "Gehäuse \\ 'A'");
    let child = result.assembly.component(parent.children[0]).unwrap();
    assert_eq!(child.name, "部品 😀 é");
}

#[test]
fn step_assembly_declared_metres_scale_geometry_and_placements_once() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly)
        .unwrap()
        .replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)");
    let mut restored = Topology::new();
    let result = step::read_step_assembly(&text, &mut restored).unwrap();
    near(result.length_scale_to_mm, 1000.);
    let bbox = measure::solid_bounding_box(&restored, result.solids[0]).unwrap();
    near(bbox.max.x(), 1000.);
    near(bbox.max.y(), 2000.);
    near(bbox.max.z(), 3000.);
    let parent = result
        .assembly
        .component(result.assembly.roots()[0])
        .unwrap();
    let point = result
        .assembly
        .world_transform(parent.children[0])
        .unwrap()
        .mul_point(Point3::new(1000., 0., 0.));
    near(point.x(), 10000.);
    near(point.y(), 4000.);
    near(point.z(), 2000.);
}

fn refuses_atomically(text: &str) -> IoError {
    refuses_atomically_observed(text).0
}

fn refuses_atomically_observed(text: &str) -> (IoError, usize) {
    refuses_with_limits(text, ImportLimits::default())
}

fn refuses_with_limits(text: &str, limits: ImportLimits) -> (IoError, usize) {
    let mut topo = Topology::new();
    let existing = make_box(&mut topo, 3., 4., 5.).unwrap();
    let before = serialize_solid(&topo, existing).unwrap();
    let counts = (
        topo.num_solids(),
        topo.num_shells(),
        topo.num_faces(),
        topo.num_wires(),
        topo.num_edges(),
        topo.num_vertices(),
        topo.num_loops(),
        topo.num_coedges(),
        topo.num_pcurves(),
    );
    let slots = topo.allocated_slot_count();
    let error = step::read_step_assembly_with_limits(text, &mut topo, limits).unwrap_err();
    assert_eq!(
        counts,
        (
            topo.num_solids(),
            topo.num_shells(),
            topo.num_faces(),
            topo.num_wires(),
            topo.num_edges(),
            topo.num_vertices(),
            topo.num_loops(),
            topo.num_coedges(),
            topo.num_pcurves(),
        )
    );
    assert_eq!(before, serialize_solid(&topo, existing).unwrap());
    (error, topo.allocated_slot_count() - slots)
}

#[test]
fn step_assembly_mixed_or_missing_units_are_atomic_refusals() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    for input in [
        text.replacen("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)", 1),
        text.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.SECOND.)"),
    ] {
        assert!(matches!(
            refuses_atomically(&input),
            IoError::ParseError { .. }
        ));
    }
}

fn refs(line: &str) -> Vec<u64> {
    line.split('#')
        .skip(1)
        .map(|part| {
            part.chars()
                .take_while(char::is_ascii_digit)
                .collect::<String>()
                .parse()
                .unwrap()
        })
        .collect()
}

fn entity_line(text: &str, id: u64) -> &str {
    text.lines()
        .find(|line| line.starts_with(&format!("#{id} =")))
        .unwrap()
}

fn add_representation_item(text: &str, representation: u64, item: u64) -> String {
    let line = entity_line(text, representation);
    let replacement = line.replace("),#", &format!(",#{item}),#"));
    assert_ne!(replacement, line, "fixture must edit representation items");
    text.replace(line, &replacement)
}

#[test]
fn step_assembly_reversed_relationship_is_refused_without_changing_legacy_geometry() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let relation = text
        .lines()
        .find(|line| line.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION("))
        .unwrap();
    let ids = refs(relation);
    let mapping = [(ids[1], ids[2]), (ids[2], ids[1])].into_iter().collect();
    let reversed = text.replace(relation, &replace_refs(relation, &mapping));

    assert!(matches!(
        refuses_atomically(&reversed),
        IoError::ParseError { .. }
    ));
    // The legacy geometry API never materializes product occurrences and
    // retains its historical acceptance of uniquely resolvable memberships.
    let mut restored = Topology::new();
    let solids = step::read_step(&reversed, &mut restored).unwrap();
    assert_eq!(solids.len(), 1);
    near(
        measure::solid_volume(&restored, solids[0], 0.01).unwrap(),
        6.,
    );
}

#[test]
fn step_assembly_shared_placement_memberships_refuse_atomically() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let relation = text
        .lines()
        .find(|line| line.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION("))
        .unwrap();
    let ids = refs(relation);
    let transform = entity_line(&text, ids[3]);
    let transform_ids = refs(transform);

    // A parent placement appearing in both representations leaves one
    // schema orientation, which legacy body import can still resolve. The
    // opt-in assembly API requires each placement to belong only to its side.
    let cross_member = add_representation_item(&text, ids[1], transform_ids[2]);
    assert!(matches!(
        refuses_atomically(&cross_member),
        IoError::ParseError { .. }
    ));
    assert_eq!(
        step::read_step(&cross_member, &mut Topology::new())
            .unwrap()
            .len(),
        1
    );

    // Even identical frames cannot choose a side by their evaluated geometry:
    // reusing one placement entity on both sides is structurally ambiguous.
    let shared_identity = add_representation_item(&text, ids[2], transform_ids[1]);
    let same_item = format!(
        "#{} = ITEM_DEFINED_TRANSFORMATION('','',#{},#{});",
        transform_ids[0], transform_ids[1], transform_ids[1]
    );
    let shared_identity = shared_identity.replace(transform, &same_item);
    assert!(matches!(
        refuses_atomically(&shared_identity),
        IoError::ParseError { .. }
    ));
}

#[test]
fn step_assembly_both_valid_placement_orders_are_refused_by_both_readers() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let relation = text
        .lines()
        .find(|line| line.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION("))
        .unwrap();
    let ids = refs(relation);
    let transform_ids = refs(entity_line(&text, ids[3]));
    let ambiguous = add_representation_item(&text, ids[1], transform_ids[2]);
    let ambiguous = add_representation_item(&ambiguous, ids[2], transform_ids[1]);

    assert!(matches!(
        refuses_atomically(&ambiguous),
        IoError::ParseError { .. }
    ));
    let mut restored = Topology::new();
    let existing = make_box(&mut restored, 3., 4., 5.).unwrap();
    let before = serialize_solid(&restored, existing).unwrap();
    assert!(matches!(
        step::read_step(&ambiguous, &mut restored),
        Err(IoError::ParseError { .. })
    ));
    assert_eq!(before, serialize_solid(&restored, existing).unwrap());
    assert_eq!(restored.num_solids(), 1);
}

#[test]
fn step_assembly_placement_references_require_point_and_direction_entity_types() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let transform = text
        .lines()
        .find(|line| line.contains("ITEM_DEFINED_TRANSFORMATION("))
        .unwrap();
    let placement = refs(transform)[2];
    let frame = refs(entity_line(&text, placement));
    let point = entity_line(&text, frame[1]);
    let direction = entity_line(&text, frame[2]);
    for (original, replacement) in [
        (point, point.replacen("CARTESIAN_POINT(", "DIRECTION(", 1)),
        (
            direction,
            direction.replacen("DIRECTION(", "CARTESIAN_POINT(", 1),
        ),
    ] {
        assert_ne!(original, replacement);
        // The numeric triple remains unchanged: matching coordinates cannot
        // substitute for the entity type required by an assembly placement.
        assert!(matches!(
            refuses_atomically(&text.replace(original, &replacement)),
            IoError::ParseError { .. }
        ));
    }
}

#[test]
fn step_assembly_relationship_components_require_a_complex_entity() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let relation = text
        .lines()
        .find(|line| line.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION("))
        .unwrap();
    let wrapper = relation.replacen("= (", "= WRAPPER(", 1);
    assert_ne!(wrapper, relation);
    // Textually identical component names inside a different simple entity
    // do not establish the transformed representation relationship schema.
    assert!(matches!(
        refuses_atomically(&text.replace(relation, &wrapper)),
        IoError::ParseError { .. }
    ));
}

#[test]
fn step_assembly_reversed_legacy_membership_and_malformed_placements_are_refused() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let transform = text
        .lines()
        .find(|line| line.contains("ITEM_DEFINED_TRANSFORMATION("))
        .unwrap();
    let ids = refs(transform);
    let reversed = format!(
        "#{} = ITEM_DEFINED_TRANSFORMATION('','',#{},#{});",
        ids[0], ids[2], ids[1]
    );
    let reversed = text.replace(transform, &reversed);
    assert!(matches!(
        refuses_atomically(&reversed),
        IoError::ParseError { .. }
    ));
    assert_eq!(
        step::read_step(&reversed, &mut Topology::new())
            .unwrap()
            .len(),
        1
    );
    let placement = text
        .lines()
        .find(|line| line.starts_with(&format!("#{} =", ids[2])))
        .unwrap();
    let placement_ids = refs(placement);
    let bad = format!(
        "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});",
        placement_ids[0], placement_ids[1], placement_ids[2], placement_ids[2]
    );
    assert!(matches!(
        refuses_atomically(&text.replace(placement, &bad)),
        IoError::ParseError { .. }
    ));
    let missing = format!(
        "#{} = ITEM_DEFINED_TRANSFORMATION('','',#{},#999999999);",
        ids[0], ids[1]
    );
    assert!(matches!(
        refuses_atomically(&text.replace(transform, &missing)),
        IoError::ParseError { .. }
    ));
}

#[test]
fn step_assembly_invalid_graph_and_import_limits_leave_existing_geometry_untouched() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let occurrence = text
        .lines()
        .find(|line| line.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE("))
        .unwrap();
    let ids = refs(occurrence);
    let invalid = occurrence.replace(
        &format!("#{},#{}", ids[1], ids[2]),
        &format!("#{},#{}", ids[2], ids[2]),
    );
    assert!(matches!(
        refuses_atomically(&text.replace(occurrence, &invalid)),
        IoError::ParseError { .. }
    ));
    let error = step::read_step_assembly_with_limits(
        &text,
        &mut Topology::new(),
        ImportLimits {
            max_input_bytes: 1,
            ..ImportLimits::default()
        },
    )
    .unwrap_err();
    assert!(matches!(error, IoError::LimitExceeded { .. }));
}

#[test]
fn step_assembly_writer_refuses_nonrigid_nonfinite_and_empty_assemblies() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1., 1., 1.).unwrap();
    assert!(matches!(
        step::write_step_assembly(&topo, &Assembly::new("empty")),
        Err(IoError::InvalidTopology { .. })
    ));
    for matrix in [
        Mat4::scale(2., 1., 1.),
        Mat4::scale(-1., 1., 1.),
        Mat4::translation(f64::NAN, 0., 0.),
        Mat4::translation(f64::INFINITY, 0., 0.),
        Mat4([
            [1., 0.1, 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.],
        ]),
        // A projective or non-unit homogeneous row cannot be represented by
        // AXIS2_PLACEMENT_3D, even when its difference fits angular tolerance.
        Mat4([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [1e-13, 0., 0., 1.],
        ]),
        Mat4([
            [1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1. + 1e-13],
        ]),
    ] {
        let mut assembly = Assembly::new("invalid");
        assembly.add_root_component("part", solid, matrix);
        assert!(matches!(
            step::write_step_assembly(&topo, &assembly),
            Err(IoError::InvalidTopology { .. })
        ));
    }
}

#[test]
fn step_assembly_preserves_arbitrary_rigid_placements_and_small_components() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1., 2., 3.).unwrap();
    let mut assembly = Assembly::new("rigid precision");
    let transform = Mat4::translation(1e-13, -3.125, 2e-13)
        * Mat4::rotation_z(0.37)
        * Mat4::rotation_y(-0.61)
        * Mat4::rotation_x(1e-13);
    assembly.add_root_component("part", solid, transform);
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let result = step::read_step_assembly(&text, &mut Topology::new()).unwrap();
    let actual = result
        .assembly
        .world_transform(result.assembly.roots()[0])
        .unwrap();
    for (actual, expected) in actual.0.iter().flatten().zip(transform.0.iter().flatten()) {
        assert!(
            (actual - expected).abs() < 2e-15,
            "placement component {actual} != {expected}"
        );
    }
    // These two translations must retain their value, rather than satisfy an
    // absolute geometry tolerance after being rounded away in the STEP text.
    assert_eq!(actual.0[0][3].to_bits(), transform.0[0][3].to_bits());
    assert_eq!(actual.0[2][3].to_bits(), transform.0[2][3].to_bits());
}

#[test]
fn step_assembly_writer_refuses_sheet_tagged_and_open_component_shells() {
    use remus_topology::{BodyClass, TopologyError, shell::Shell, solid::Solid};

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 3., 4., 5.).unwrap();
    let outer = topo.solid(solid).unwrap().outer_shell();
    let mut assembly = Assembly::new("invalid component");
    assembly.add_root_component("part", solid, Mat4::identity());

    // A SolidId can still refer to a shell explicitly tagged as a sheet via
    // the public low-level topology API. Export must honor that body class.
    topo.set_shell_body_class(outer, BodyClass::Sheet).unwrap();
    assert!(matches!(
        step::write_step_assembly(&topo, &assembly),
        Err(IoError::Topology(TopologyError::BodyClassMismatch {
            expected: "solid",
            actual: "sheet",
            ..
        }))
    ));
    topo.set_shell_body_class(outer, BodyClass::Solid).unwrap();

    // All five retained faces and their boundary authority are valid; the
    // absent sixth face specifically makes the outer shell open.
    let faces = topo.shell(outer).unwrap().faces()[..5].to_vec();
    let open_shell = topo.add_shell(Shell::new(faces).unwrap());
    let open_solid = topo.add_solid(Solid::new(open_shell, vec![]));
    let mut open_assembly = Assembly::new("open component");
    open_assembly.add_root_component("part", open_solid, Mat4::identity());
    let before = serialize_solid(&topo, solid).unwrap();
    assert!(matches!(
        step::write_step_assembly(&topo, &open_assembly),
        Err(IoError::Topology(TopologyError::NonManifold { .. }))
    ));
    assert_eq!(before, serialize_solid(&topo, solid).unwrap());
}

#[test]
fn step_assembly_shared_cavity_definition_requires_a_closed_inner_shell() {
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_topology::{TopologyError, shell::Shell, solid::Solid};

    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 3., 3., 3.).unwrap();
    let void = make_box(&mut topo, 1., 1., 1.).unwrap();
    remus_operations::transform::transform_solid(&mut topo, void, &Mat4::translation(1., 1., 1.))
        .unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, void).unwrap();
    let mut assembly = Assembly::new("shared hollow parts");
    assembly.add_root_component("left", hollow, Mat4::identity());
    assembly.add_root_component("right", hollow, Mat4::translation(10., 0., 0.));
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    assert_eq!(text.matches("BREP_WITH_VOIDS(").count(), 1);
    let mut restored = Topology::new();
    let result = step::read_step_assembly(&text, &mut restored).unwrap();
    assert_eq!(result.solids.len(), 1);
    assert_eq!(result.assembly.bill_of_materials()[0].instance_count, 2);
    assert_eq!(
        restored
            .solid(result.solids[0])
            .unwrap()
            .inner_shells()
            .len(),
        1
    );
    near(
        measure::solid_volume(&restored, result.solids[0], 0.01).unwrap(),
        26.,
    );

    let solid = topo.solid(hollow).unwrap();
    let outer_shell = solid.outer_shell();
    let inner_shell = solid.inner_shells()[0];
    let faces = topo.shell(inner_shell).unwrap().faces()[..5].to_vec();
    let open_inner = topo.add_shell(Shell::new(faces).unwrap());
    let malformed = topo.add_solid(Solid::new(outer_shell, vec![open_inner]));
    let mut malformed_assembly = Assembly::new("open cavity");
    malformed_assembly.add_root_component("part", malformed, Mat4::identity());
    let original = serialize_solid(&topo, hollow).unwrap();
    assert!(matches!(
        step::write_step_assembly(&topo, &malformed_assembly),
        Err(IoError::Topology(TopologyError::NonManifold { .. }))
    ));
    assert_eq!(original, serialize_solid(&topo, hollow).unwrap());
}

#[test]
fn step_assembly_distinct_products_can_share_formation_and_representation() {
    let (topo, assembly) = witness();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let occurrences: Vec<_> = text
        .lines()
        .filter(|line| line.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE("))
        .collect();
    let first_definition = refs(occurrences[0])[2];
    let second_definition = refs(occurrences[1])[2];
    let first_product = refs(entity_line(&text, first_definition));
    let second_product_line = entity_line(&text, second_definition);
    let second_product = refs(second_product_line);
    let relationships: Vec<_> = text
        .lines()
        .filter(|line| line.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION("))
        .collect();
    let first_relation = refs(relationships[0]);
    let second_relation = refs(relationships[1]);
    let first_transform = refs(entity_line(&text, first_relation[3]));
    let second_transform_line = entity_line(&text, second_relation[3]);
    let second_transform = refs(second_transform_line);
    let second_shape = text
        .lines()
        .find(|line| {
            line.contains("PRODUCT_DEFINITION_SHAPE(")
                && refs(line).get(1) == Some(&second_definition)
        })
        .unwrap();
    let second_shape_id = refs(second_shape)[0];
    let second_mapping = text
        .lines()
        .find(|line| {
            line.contains("SHAPE_DEFINITION_REPRESENTATION(")
                && refs(line).get(1) == Some(&second_shape_id)
        })
        .unwrap();

    let shared_formation = std::iter::once((second_product[1], first_product[1])).collect();
    let shared_representation = std::iter::once((second_relation[1], first_relation[1])).collect();
    let shared_child_frame = std::iter::once((second_transform[1], first_transform[1])).collect();
    let text = text
        .replace(
            second_product_line,
            &replace_refs(second_product_line, &shared_formation),
        )
        .replace(
            second_mapping,
            &replace_refs(second_mapping, &shared_representation),
        )
        .replace(
            relationships[1],
            &replace_refs(relationships[1], &shared_representation),
        )
        .replace(
            second_transform_line,
            &replace_refs(second_transform_line, &shared_child_frame),
        );

    let result = step::read_step_assembly(&text, &mut Topology::new()).unwrap();
    assert_eq!(result.solids.len(), 1);
    assert_eq!(result.assembly.component_count(), 3);
    assert_eq!(result.assembly.roots().len(), 2);
    for (&expected, &actual) in assembly.roots().iter().zip(result.assembly.roots()) {
        assert_eq!(
            result.assembly.component(actual).unwrap().name,
            assembly.component(expected).unwrap().name
        );
    }
    for ((_, expected), (_, actual)) in assembly.flatten().iter().zip(result.assembly.flatten()) {
        for (actual, expected) in actual.0.iter().flatten().zip(expected.0.iter().flatten()) {
            near(*actual, *expected);
        }
    }
}

#[test]
fn step_assembly_unreachable_cycle_refuses_after_build_and_retires_allocated_handles() {
    let (topo, assembly) = witness();
    let mut text = step::write_step_assembly(&topo, &assembly).unwrap();
    let occurrences: Vec<_> = text
        .lines()
        .filter(|line| line.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE("))
        .map(str::to_owned)
        .collect();
    let first = refs(&occurrences[0]);
    let nested = refs(&occurrences[2]);
    let child_shape = text
        .lines()
        .find(|line| {
            line.contains("PRODUCT_DEFINITION_SHAPE(") && refs(line).get(1) == Some(&nested[2])
        })
        .unwrap();
    let shape_id = refs(child_shape)[0];
    let child_link = text
        .lines()
        .find(|line| {
            line.contains("SHAPE_DEFINITION_REPRESENTATION(")
                && refs(line).get(1) == Some(&shape_id)
        })
        .unwrap();
    let child_rep = refs(child_link)[2];
    let relation = text
        .lines()
        .find(|line| line.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION("))
        .unwrap()
        .to_owned();
    let relation_ids = refs(&relation);
    let transform = text
        .lines()
        .find(|line| line.contains("ITEM_DEFINED_TRANSFORMATION("))
        .unwrap();
    let parent_placement = refs(transform)[2];
    let old_parent_rep = relation_ids[2];
    let root_rep_line = text
        .lines()
        .find(|line| line.starts_with(&format!("#{old_parent_rep} =")))
        .unwrap()
        .to_owned();
    let child_rep_line = text
        .lines()
        .find(|line| line.starts_with(&format!("#{child_rep} =")))
        .unwrap()
        .to_owned();
    text = text.replace(
        &occurrences[0],
        &occurrences[0].replace(
            &format!("#{},#{}", first[1], first[2]),
            &format!("#{},#{}", nested[2], first[2]),
        ),
    );
    text = text.replace(
        &relation,
        &relation.replace(&format!("#{old_parent_rep})"), &format!("#{child_rep})")),
    );
    text = text.replace(
        &root_rep_line,
        &root_rep_line.replace(&format!(",#{parent_placement}"), ""),
    );
    text = text.replace(
        &child_rep_line,
        &child_rep_line.replace("),#", &format!(",#{parent_placement}),#")),
    );
    let (error, retired_slots) = refuses_atomically_observed(&text);
    assert!(matches!(error, IoError::ParseError { .. }));
    assert!(
        error.to_string().contains("unreachable or cyclic"),
        "{error}"
    );
    assert!(
        retired_slots > 0,
        "witness must reach the post-geometry graph validation"
    );
}

#[test]
fn step_assembly_nonidentity_child_frame_is_inverted_before_parent_placement() {
    let (topo, assembly) = witness();
    let mut text = step::write_step_assembly(&topo, &assembly).unwrap();
    let transform = text
        .lines()
        .find(|line| line.contains("ITEM_DEFINED_TRANSFORMATION("))
        .unwrap();
    let child_placement = refs(transform)[1];
    let placement = text
        .lines()
        .find(|line| line.starts_with(&format!("#{child_placement} =")))
        .unwrap();
    let frame_ids = refs(placement);
    let point = text
        .lines()
        .find(|line| line.starts_with(&format!("#{} =", frame_ids[1])))
        .unwrap()
        .to_owned();
    let axis = text
        .lines()
        .find(|line| line.starts_with(&format!("#{} =", frame_ids[2])))
        .unwrap()
        .to_owned();
    text = text.replace(
        &point,
        &format!("#{} = CARTESIAN_POINT('',(3.,4.,5.));", frame_ids[1]),
    );
    text = text.replace(
        &axis,
        &format!("#{} = DIRECTION('',(0.,-1.,0.));", frame_ids[2]),
    );
    let result = step::read_step_assembly(&text, &mut Topology::new()).unwrap();
    let world = result
        .assembly
        .world_transform(result.assembly.roots()[0])
        .unwrap();
    // Local (3,4,6) is (0,1,0) in the child frame; the parent rotates it to (-1,0,0).
    let point = world.mul_point(Point3::new(3., 4., 6.));
    near(point.x(), 9.);
    near(point.y(), 2.);
    near(point.z(), 0.);
}

#[test]
fn step_assembly_duplicate_product_shape_mapping_is_an_atomic_refusal() {
    let (topo, assembly) = witness();
    let mut text = step::write_step_assembly(&topo, &assembly).unwrap();
    let mapping = text
        .lines()
        .find(|line| line.contains("SHAPE_DEFINITION_REPRESENTATION("))
        .unwrap();
    let mapping_ids = refs(mapping);
    let duplicate = format!(
        "#99999999 = SHAPE_DEFINITION_REPRESENTATION(#{},#{});\n",
        mapping_ids[1], mapping_ids[2]
    );
    text.insert_str(text.rfind("ENDSEC;").unwrap(), &duplicate);
    assert!(
        refuses_atomically(&text)
            .to_string()
            .contains("ambiguous product shape")
    );
}

fn chain_document(depth: usize) -> String {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1., 1., 1.).unwrap();
    let mut assembly = Assembly::new("chain");
    let mut parent = assembly.add_root_component("0", solid, Mat4::identity());
    for i in 1..depth {
        parent = assembly
            .add_child_component(parent, i.to_string(), solid, Mat4::identity())
            .unwrap();
    }
    step::write_step_assembly(&topo, &assembly).unwrap()
}

#[test]
fn step_assembly_depth_limit_is_atomic_after_geometry_allocation() {
    let (error, slots) = refuses_atomically_observed(&chain_document(257));
    assert!(matches!(
        error,
        IoError::LimitExceeded {
            resource: "STEP assembly depth",
            limit: 256,
            actual: 257
        }
    ));
    assert!(slots > 0);
}

#[test]
fn step_assembly_maximum_supported_depth_is_accepted() {
    let result = step::read_step_assembly(&chain_document(256), &mut Topology::new()).unwrap();
    assert_eq!(result.solids.len(), 1);
    assert_eq!(result.assembly.component_count(), 256);
    assert_eq!(result.assembly.roots().len(), 1);
    assert_eq!(result.assembly.bill_of_materials()[0].instance_count, 256);
}

#[test]
fn step_assembly_failed_expansion_does_not_reuse_retired_solid_slots() {
    let mut restored = Topology::new();
    let existing = make_box(&mut restored, 3., 4., 5.).unwrap();
    let retired_index = existing.index() + 1;
    assert!(matches!(
        step::read_step_assembly(&chain_document(257), &mut restored),
        Err(IoError::LimitExceeded {
            resource: "STEP assembly depth",
            ..
        })
    ));
    assert!(restored.solid_id_from_index(retired_index).is_none());
    let next = make_box(&mut restored, 1., 1., 1.).unwrap();
    assert!(
        next.index() > retired_index,
        "the failed import must retire its constructed solid before later allocation"
    );
    assert!(restored.solid_id_from_index(retired_index).is_none());
    near(
        measure::solid_volume(&restored, existing, 0.01).unwrap(),
        60.,
    );
}

fn replace_refs(line: &str, mapping: &std::collections::BTreeMap<u64, u64>) -> String {
    let mut parts = line.split('#');
    let mut result = parts.next().unwrap().to_string();
    for part in parts {
        let digits = part.bytes().take_while(u8::is_ascii_digit).count();
        let old = part[..digits].parse::<u64>().unwrap();
        write!(
            result,
            "#{}{}",
            mapping.get(&old).unwrap_or(&old),
            &part[digits..]
        )
        .unwrap();
    }
    result
}

fn doubled_chain_document(depth: usize) -> String {
    let mut text = chain_document(depth);
    let lines: Vec<_> = text.lines().collect();
    let mut next_id = lines
        .iter()
        .filter(|line| line.starts_with('#'))
        .map(|line| refs(line)[0])
        .max()
        .unwrap()
        + 1;
    let mut extra = String::new();
    for (i, line) in lines.iter().enumerate() {
        if !line.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE(") {
            continue;
        }
        let group = &lines[i..i + 5];
        let mapping = group
            .iter()
            .map(|line| {
                let old = refs(line)[0];
                let new = next_id;
                next_id += 1;
                (old, new)
            })
            .collect();
        for line in group {
            let line = replace_refs(line, &mapping);
            if line.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE(") {
                let ids = refs(&line);
                writeln!(extra, "#{} = NEXT_ASSEMBLY_USAGE_OCCURRENCE('repeat{}','repeat','',#{},#{},'repeat{}');", ids[0], ids[0], ids[1], ids[2], ids[0]).unwrap();
            } else {
                extra.push_str(&line);
                extra.push('\n');
            }
        }
    }
    text.insert_str(text.rfind("ENDSEC;").unwrap(), &extra);
    text
}

#[test]
fn step_assembly_repeated_product_definitions_expand_with_shared_solids() {
    let result =
        step::read_step_assembly(&doubled_chain_document(3), &mut Topology::new()).unwrap();
    assert_eq!(result.solids.len(), 1);
    assert_eq!(result.assembly.component_count(), 14);
    assert_eq!(result.assembly.roots().len(), 2);
    assert_eq!(result.assembly.bill_of_materials()[0].instance_count, 14);
}

#[test]
fn step_assembly_repeated_subassembly_expansion_is_bounded_independently_of_file_size() {
    let text = doubled_chain_document(12);
    let records = text.lines().filter(|line| line.starts_with('#')).count();
    let (error, slots) = refuses_with_limits(
        &text,
        ImportLimits {
            max_model_entities: records + 1,
            ..ImportLimits::default()
        },
    );
    assert!(
        matches!(
            error,
            IoError::LimitExceeded {
                resource: "expanded STEP occurrences",
                ..
            }
        ),
        "{error}"
    );
    assert!(slots > 0);
}

#[test]
fn step_assembly_preserves_bom_name_when_nested_instance_was_created_after_another_root() {
    let mut topo = Topology::new();
    let first = make_box(&mut topo, 1., 1., 1.).unwrap();
    let second = make_box(&mut topo, 2., 2., 2.).unwrap();
    let mut assembly = Assembly::new("order");
    let parent = assembly.add_root_component("parent", first, Mat4::identity());
    assembly.add_root_component("original representative", second, Mat4::identity());
    assembly
        .add_child_component(parent, "later child", second, Mat4::identity())
        .unwrap();
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let result = step::read_step_assembly(&text, &mut Topology::new()).unwrap();
    let expected: Vec<_> = assembly
        .bill_of_materials()
        .iter()
        .map(|entry| (entry.name.clone(), entry.instance_count))
        .collect();
    let actual: Vec<_> = result
        .assembly
        .bill_of_materials()
        .iter()
        .map(|entry| (entry.name.clone(), entry.instance_count))
        .collect();
    assert_eq!(actual, expected);
}

#[test]
fn step_assembly_refuses_overflow_in_composed_world_placements() {
    let (topo, mut assembly) = witness();
    let solid = assembly.component(assembly.roots()[0]).unwrap().solid;
    let parent =
        assembly.add_root_component("overflow parent", solid, Mat4::translation(1e308, 0., 0.));
    assembly
        .add_child_component(
            parent,
            "overflow child",
            solid,
            Mat4::translation(1e308, 0., 0.),
        )
        .unwrap();
    assert!(matches!(
        step::write_step_assembly(&topo, &assembly),
        Err(IoError::InvalidTopology { .. })
    ));
    let (topo, assembly) = witness();
    let mut text = step::write_step_assembly(&topo, &assembly).unwrap();
    let transforms: Vec<_> = text
        .lines()
        .filter(|line| line.contains("ITEM_DEFINED_TRANSFORMATION("))
        .map(str::to_owned)
        .collect();
    for (index, coordinates) in [(0, "1.E308,0.,0."), (2, "0.,-1.E308,0.")] {
        let placement_id = refs(&transforms[index])[2];
        let placement = text
            .lines()
            .find(|line| line.starts_with(&format!("#{placement_id} =")))
            .unwrap();
        let point_id = refs(placement)[1];
        let point = text
            .lines()
            .find(|line| line.starts_with(&format!("#{point_id} =")))
            .unwrap()
            .to_owned();
        text = text.replace(
            &point,
            &format!("#{point_id} = CARTESIAN_POINT('',({coordinates}));"),
        );
    }
    assert!(matches!(
        refuses_atomically(&text),
        IoError::ParseError { .. }
    ));
}

#[test]
fn step_assembly_repeated_names_have_a_separate_expanded_byte_limit() {
    let text =
        doubled_chain_document(8).replace("'repeat',''", &format!("'{}',''", "x".repeat(1024)));
    let (error, slots) = refuses_with_limits(
        &text,
        ImportLimits {
            max_input_bytes: text.len(),
            ..ImportLimits::default()
        },
    );
    assert!(
        matches!(
            error,
            IoError::LimitExceeded {
                resource: "expanded STEP name bytes",
                ..
            }
        ),
        "{error}"
    );
    assert!(slots > 0);
}
