//! Product structure, placed occurrences, names and colours from STEP.
//!
//! The fixtures under `tests/data/step_structure/` were written by an
//! independent XCAF STEP writer (not by Remus), and every expected volume,
//! centroid and colour below was read back from them by that kernel's own
//! XCAF reader: placed volumes and bounds per occurrence, and surface colours
//! in sRGB. The assembly is a 40 × 30 × 5 plate and a "Pin pair"
//! sub-assembly (two R3 × 12 pins, no geometry of its own) placed twice, the
//! second time turned 90° about Z; the part is one product holding a
//! 20 × 10 × 5 block (coloured, one face overriding it) and an R4 ball.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_io::{ImportLimits, IoError, arena_io::serialize_solid, step};
use remus_math::mat::Mat4;
use remus_operations::{assembly::Assembly, measure, primitives::make_box};
use remus_topology::{Topology, attributes::ColorRgb, explorer::solid_faces, solid::SolidId};

const AP214: &str = include_str!("data/step_structure/nested_assembly_ap214.step");
const AP242: &str = include_str!("data/step_structure/nested_assembly_ap242.step");
const INCH: &str = include_str!("data/step_structure/nested_assembly_inch.step");
const PART: &str = include_str!("data/step_structure/part_solid_and_face_colours.step");
const PART_INCH: &str = include_str!("data/step_structure/part_solid_and_face_colours_inch.step");

/// The plate's and the pins' `COLOUR_RGB` values as the files state them.
const PLATE_RGB: (f64, f64, f64) = (0.484_529_207_832, 0.665_185_089_102, 0.906_331_759_313);
const PIN_RGB: (f64, f64, f64) = (0.954_687_160_741, 0.735_356_983_052, 0.349_190_215_138);

fn pin_volume() -> f64 {
    PI * 9.0 * 12.0
}

fn assert_near(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= 1e-6 * expected.abs().max(1.0),
        "{what}: {actual} != {expected}"
    );
}

fn volume_and_centre(topo: &Topology, solid: SolidId) -> (f64, [f64; 3]) {
    let props = measure::mass_properties(topo, solid).unwrap();
    (
        props.mass,
        [props.center.x(), props.center.y(), props.center.z()],
    )
}

fn colour(topo: &Topology, solid: SolidId) -> Option<(f64, f64, f64)> {
    topo.attributes()
        .solid(solid)
        .and_then(|attributes| attributes.color)
        .map(|colour: ColorRgb| (colour.r(), colour.g(), colour.b()))
}

fn assert_colour(actual: Option<(f64, f64, f64)>, expected: (f64, f64, f64)) {
    let actual = actual.expect("solid has a colour");
    for (a, e) in [
        (actual.0, expected.0),
        (actual.1, expected.1),
        (actual.2, expected.2),
    ] {
        assert!((a - e).abs() < 1e-12, "{actual:?} != {expected:?}");
    }
}

fn node_names(result: &step::StepStructureReadResult) -> Vec<(&str, Option<usize>)> {
    result
        .nodes
        .iter()
        .map(|node| (node.name.as_str(), node.parent))
        .collect()
}

fn check_nested_assembly(text: &str) {
    let mut topo = Topology::new();
    let result = step::read_step_structure(text, &mut topo).unwrap();
    assert_eq!(
        node_names(&result),
        vec![
            ("Fixture", None),
            ("Base plate", Some(0)),
            ("Pin pair", Some(0)),
            ("Pin", Some(2)),
            ("Pin", Some(2)),
            ("Pin pair", Some(0)),
            ("Pin", Some(5)),
            ("Pin", Some(5)),
        ]
    );
    assert_eq!(
        result.solid_nodes,
        vec![Some(1), Some(3), Some(4), Some(6), Some(7)]
    );
    // Independent placed centroids: the bounds' centres per occurrence.
    let expected = [
        (6000.0, [20.0, 15.0, 2.5]),
        (pin_volume(), [5.0, 5.0, 11.0]),
        (pin_volume(), [15.0, 5.0, 11.0]),
        (pin_volume(), [-5.0, 25.0, 11.0]),
        (pin_volume(), [-5.0, 35.0, 11.0]),
    ];
    assert_eq!(result.solids.len(), expected.len());
    for (&solid, (volume, centre)) in result.solids.iter().zip(expected) {
        let (actual_volume, actual_centre) = volume_and_centre(&topo, solid);
        assert_near(actual_volume, volume, "volume");
        for axis in 0..3 {
            assert_near(actual_centre[axis], centre[axis], "centroid");
        }
    }
    assert_colour(colour(&topo, result.solids[0]), PLATE_RGB);
    for &pin in &result.solids[1..] {
        assert_colour(colour(&topo, pin), PIN_RGB);
    }
    // Repeated occurrences are distinct roots, not one shared solid.
    let unique: std::collections::BTreeSet<_> = result.solids.iter().collect();
    assert_eq!(unique.len(), result.solids.len());
    assert_eq!(result.presentation.solid_colours, 2);
    assert_eq!(result.presentation.unapplied_styles, 0);
    assert_eq!(result.presentation.conflicting_styles, 0);
}

#[test]
fn nested_assembly_places_every_occurrence_with_names_and_colours() {
    check_nested_assembly(AP214);
}

#[test]
fn ap242_assembly_reads_like_ap214() {
    check_nested_assembly(AP242);
}

#[test]
fn inch_assembly_scales_geometry_and_placements_once() {
    check_nested_assembly(INCH);
}

#[test]
fn flat_readers_still_return_the_unplaced_definitions() {
    // The structure reader is additive: the legacy readers keep their
    // historical result on the same file.
    let mut topo = Topology::new();
    assert_eq!(step::read_step(AP214, &mut topo).unwrap().len(), 2);
    let mut topo = Topology::new();
    let bodies = step::read_step_bodies(AP214, &mut topo).unwrap();
    assert_eq!(bodies.solids().len(), 2);
    assert!(
        topo.attributes()
            .solids_with_attributes()
            .iter()
            .all(|(_, attributes)| attributes.color.is_none())
    );
}

fn check_part(text: &str) {
    let mut topo = Topology::new();
    let result = step::read_step_structure(text, &mut topo).unwrap();
    assert_eq!(node_names(&result), vec![("Bracket", None)]);
    assert_eq!(result.solid_nodes, vec![Some(0), Some(0)]);
    let (block, ball) = (result.solids[0], result.solids[1]);
    assert_near(volume_and_centre(&topo, block).0, 1000.0, "block volume");
    assert_near(
        volume_and_centre(&topo, ball).0,
        4.0 / 3.0 * PI * 64.0,
        "ball volume",
    );
    assert!(colour(&topo, block).is_some());
    assert!(colour(&topo, ball).is_none());
    // One face of the block carries the overriding colour; the rest inherit.
    let coloured: Vec<_> = solid_faces(&topo, block)
        .unwrap()
        .into_iter()
        .filter_map(|face| topo.attributes().face(face).and_then(|a| a.color))
        .collect();
    assert_eq!(coloured.len(), 1);
    assert_ne!(
        Some(coloured[0]),
        topo.attributes().solid(block).unwrap().color
    );
    assert!(coloured[0].r() > 0.8 && coloured[0].g() < 0.4);
    assert_eq!(result.presentation.solid_colours, 1);
    assert_eq!(result.presentation.face_colours, 1);
}

#[test]
fn part_keeps_solid_colour_and_face_override() {
    check_part(PART);
}

#[test]
fn inch_part_reads_like_millimetre_part() {
    check_part(PART_INCH);
}

/// The text with one entity line (`#id = …;`, possibly wrapped) replaced.
fn replace_entity(text: &str, id: u64, replacement: &str) -> String {
    let start = text
        .find(&format!("\n#{id} = "))
        .unwrap_or_else(|| panic!("no entity #{id}"))
        + 1;
    let end = start + text[start..].find(";\n").unwrap() + 1;
    format!("{}{}{}", &text[..start], replacement, &text[end..])
}

fn entity_text(text: &str, id: u64) -> String {
    let start = text.find(&format!("\n#{id} = ")).unwrap() + 1;
    let end = start + text[start..].find(";\n").unwrap() + 1;
    text[start..end].to_string()
}

/// Inserts entities just before `ENDSEC;` of the DATA section.
fn append_entities(text: &str, lines: &str) -> String {
    let at = text.rfind("ENDSEC;").unwrap();
    format!("{}{}\n{}", &text[..at], lines, &text[at..])
}

fn refuses_atomically(text: &str) -> IoError {
    let mut topo = Topology::new();
    let existing = make_box(&mut topo, 3., 4., 5.).unwrap();
    let before = serialize_solid(&topo, existing).unwrap();
    let counts = |topo: &Topology| {
        (
            topo.num_solids(),
            topo.num_shells(),
            topo.num_faces(),
            topo.num_edges(),
            topo.num_vertices(),
            topo.attributes().len(),
        )
    };
    let counted = counts(&topo);
    let error = step::read_step_structure(text, &mut topo).unwrap_err();
    assert_eq!(counted, counts(&topo));
    assert_eq!(before, serialize_solid(&topo, existing).unwrap());
    error
}

/// `#376` places the plate occurrence `#380`; `#407` places the first
/// "Pin pair" in the root `#5`; `#384` is the "Pin pair" definition.
#[test]
fn an_occurrence_that_places_geometry_without_a_placement_is_refused() {
    let text = replace_entity(AP214, 376, "#376 = DRAUGHTING_MODEL('',(#11),#27);");
    let error = refuses_atomically(&text);
    assert!(
        error
            .to_string()
            .contains("#380 places geometry but has no placement"),
        "{error}"
    );
}

#[test]
fn a_cyclic_product_graph_is_refused() {
    // The pin pair now uses the root fixture as a component.
    let nauo = entity_text(AP214, 524).replacen("#384,#411", "#384,#5", 1);
    let text = replace_entity(AP214, 524, &nauo);
    let error = refuses_atomically(&text);
    assert!(
        matches!(
            error,
            IoError::ParseError { .. } | IoError::LimitExceeded { .. }
        ),
        "{error}"
    );
}

#[test]
fn mapped_item_assemblies_are_refused_by_name() {
    let text = append_entities(
        AP214,
        "#9001 = REPRESENTATION_MAP(#11,#40);\n#9002 = MAPPED_ITEM('',#9001,#15);",
    );
    match refuses_atomically(&text) {
        IoError::UnsupportedEntity { entity } => assert_eq!(entity, "MAPPED_ITEM"),
        other => panic!("{other}"),
    }
}

#[test]
fn reversed_transformation_items_and_relationship_order_place_identically() {
    let mut topo = Topology::new();
    let reference = step::read_step_structure(AP214, &mut topo).unwrap();
    let reference: Vec<_> = reference
        .solids
        .iter()
        .map(|&solid| volume_and_centre(&topo, solid))
        .collect();
    // Historical item order on the second pin pair's transformation, and
    // parent-then-child relationship order on the first pin's.
    let reversed = replace_entity(
        AP214,
        533,
        "#533 = ITEM_DEFINED_TRANSFORMATION('','',#23,#11);",
    );
    let swapped = entity_text(&reversed, 521).replacen("#416,#389", "#389,#416", 1);
    let text = replace_entity(&reversed, 521, &swapped);
    let mut topo = Topology::new();
    let result = step::read_step_structure(&text, &mut topo).unwrap();
    for (&solid, (volume, centre)) in result.solids.iter().zip(reference) {
        let (actual_volume, actual_centre) = volume_and_centre(&topo, solid);
        assert_near(actual_volume, volume, "volume");
        for axis in 0..3 {
            assert_near(actual_centre[axis], centre[axis], "centroid");
        }
    }
}

#[test]
fn presentation_problems_never_refuse_the_geometry() {
    // A second, different plain style on the plate is a conflict: the
    // plate stays uncoloured. An out-of-range pin colour is refused and
    // counted. A style on the plate's shell is not applied.
    let mut text = append_entities(
        AP214,
        "#9010 = COLOUR_RGB('',0.1,0.2,0.3);\n\
         #9011 = FILL_AREA_STYLE_COLOUR('',#9010);\n\
         #9012 = FILL_AREA_STYLE('',(#9011));\n\
         #9013 = SURFACE_STYLE_FILL_AREA(#9012);\n\
         #9014 = SURFACE_SIDE_STYLE('',(#9013));\n\
         #9015 = SURFACE_STYLE_USAGE(.BOTH.,#9014);\n\
         #9016 = PRESENTATION_STYLE_ASSIGNMENT((#9015));\n\
         #9017 = STYLED_ITEM('',(#9016),#41);\n\
         #9018 = STYLED_ITEM('',(#9016),#42);",
    );
    text = replace_entity(
        &text,
        544,
        "#544 = COLOUR_RGB('',1.5,0.735356983052,0.349190215138);",
    );
    let mut topo = Topology::new();
    let result = step::read_step_structure(&text, &mut topo).unwrap();
    assert_eq!(result.solids.len(), 5);
    assert!(
        result
            .solids
            .iter()
            .all(|&solid| colour(&topo, solid).is_none())
    );
    assert_eq!(result.presentation.conflicting_styles, 1);
    assert_eq!(result.presentation.invalid_colours, 1);
    assert_eq!(result.presentation.unapplied_styles, 1);
    assert_eq!(result.presentation.solid_colours, 0);
}

#[test]
fn predefined_colours_resolve() {
    let text = replace_entity(PART, 371, "#371 = DRAUGHTING_PRE_DEFINED_COLOUR('blue');");
    let mut topo = Topology::new();
    let result = step::read_step_structure(&text, &mut topo).unwrap();
    assert_colour(colour(&topo, result.solids[0]), (0.0, 0.0, 1.0));
}

#[test]
fn solids_no_product_reaches_are_kept_outside_every_node() {
    // Without its shape definition the product holds nothing; its solids
    // still import, outside the tree.
    let text = replace_entity(PART, 3, "#3 = DRAUGHTING_MODEL('',(#11),#357);");
    let mut topo = Topology::new();
    let result = step::read_step_structure(&text, &mut topo).unwrap();
    assert_eq!(result.solid_nodes, vec![None, None]);
    assert_eq!(result.solids.len(), 2);
}

#[test]
fn native_assembly_writer_round_trips_through_the_structure_reader() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1., 2., 3.).unwrap();
    let mut assembly = Assembly::new("bench");
    let left = assembly.add_root_component("left", solid, Mat4::translation(10., 0., 0.));
    assembly
        .add_child_component(left, "nested", solid, Mat4::translation(0., 5., 0.))
        .unwrap();
    assembly.add_root_component("right", solid, Mat4::translation(-10., 0., 0.));
    let text = step::write_step_assembly(&topo, &assembly).unwrap();
    let mut restored = Topology::new();
    let result = step::read_step_structure(&text, &mut restored).unwrap();
    assert_eq!(result.solids.len(), 3);
    let centres: Vec<_> = result
        .solids
        .iter()
        .map(|&solid| volume_and_centre(&restored, solid).1)
        .collect();
    let mut expected = vec![[10.5, 1.0, 1.5], [10.5, 6.0, 1.5], [-9.5, 1.0, 1.5]];
    let mut actual = centres;
    let key = |c: &[f64; 3]| (c[0] * 1e6).round() as i64 * 1_000_000 + (c[1] * 1e3).round() as i64;
    actual.sort_by_key(key);
    expected.sort_by_key(key);
    for (a, e) in actual.iter().zip(&expected) {
        for axis in 0..3 {
            assert_near(a[axis], e[axis], "centroid");
        }
    }
    assert_eq!(result.nodes[0].name, "bench");
    assert!(result.nodes.iter().any(|node| node.name == "nested"));
}

#[test]
fn an_empty_budget_refuses_before_building() {
    let limits = ImportLimits {
        max_input_bytes: 100,
        ..ImportLimits::default()
    };
    let mut topo = Topology::new();
    assert!(matches!(
        step::read_step_structure_with_limits(AP214, &mut topo, limits),
        Err(IoError::LimitExceeded { .. })
    ));
    assert_eq!(topo.num_solids(), 0);
}

#[test]
fn an_untyped_length_conversion_factor_reads_in_every_reader() {
    // `LENGTH_MEASURE_WITH_UNIT(25.4,#31)` states its factor as a bare real.
    // The flat reader refused it before; both readers now read 1 in as 25.4 mm.
    assert!(PART_INCH.contains("LENGTH_MEASURE_WITH_UNIT(25.4,#"));
    let mut topo = Topology::new();
    let solids = step::read_step(PART_INCH, &mut topo).unwrap();
    assert_eq!(solids.len(), 2);
    assert_near(
        volume_and_centre(&topo, solids[0]).0,
        1000.0,
        "block volume",
    );
    // A plain MEASURE_WITH_UNIT still needs the typed parameter.
    let untyped = PART_INCH.replacen(
        "LENGTH_MEASURE_WITH_UNIT(25.4,#",
        "MEASURE_WITH_UNIT(25.4,#",
        1,
    );
    assert!(step::read_step(&untyped, &mut Topology::new()).is_err());
}

#[test]
fn shared_geometry_free_definitions_do_not_explode_the_reachability_check() {
    // Forty geometry-free products, each using the next twice without a
    // placement, beside a part with real geometry: 2^40 paths. The graph is
    // walked once, and expansion stops at the occurrence budget.
    let mut extra = String::new();
    let base = 9000;
    for level in 0..41u64 {
        let id = base + level * 10;
        let _ = std::fmt::Write::write_fmt(
            &mut extra,
            format_args!(
                "#{} = PRODUCT('L{level}','L{level}','',(#8));\n#{} = PRODUCT_DEFINITION_FORMATION('','',#{});\n#{} = PRODUCT_DEFINITION('design','',#{},#9);\n",
                id,
                id + 1,
                id,
                id + 2,
                id + 1
            ),
        );
        if level > 0 {
            let parent = base + (level - 1) * 10 + 2;
            for copy in 0..2 {
                let _ = std::fmt::Write::write_fmt(
                    &mut extra,
                    format_args!(
                        "#{} = NEXT_ASSEMBLY_USAGE_OCCURRENCE('{copy}','','',#{parent},#{},$);\n",
                        id + 3 + copy,
                        id + 2
                    ),
                );
            }
        }
    }
    let text = append_entities(PART, &extra);
    let limits = ImportLimits {
        max_model_entities: 20_000,
        ..ImportLimits::default()
    };
    let started = std::time::Instant::now();
    let error =
        step::read_step_structure_with_limits(&text, &mut Topology::new(), limits).unwrap_err();
    assert!(matches!(error, IoError::LimitExceeded { .. }), "{error}");
    assert!(started.elapsed() < std::time::Duration::from_secs(20));
}

#[test]
fn occurrence_copies_are_charged_against_the_model_budget() {
    // Two hundred more placements of the plate in the root: each costs four
    // entities to state but a whole plate (26 faces, edges and vertices) to
    // copy. A budget the file's entities fit refuses the copies by name.
    let mut extra = String::new();
    for copy in 0..200u64 {
        let id = 9000 + copy * 4;
        let _ = std::fmt::Write::write_fmt(
            &mut extra,
            format_args!(
                "#{a} = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{b},#{c});
                 #{b} = ( REPRESENTATION_RELATIONSHIP('','',#40,#10) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#378) SHAPE_REPRESENTATION_RELATIONSHIP() );
                 #{c} = PRODUCT_DEFINITION_SHAPE('','',#{d});
                 #{d} = NEXT_ASSEMBLY_USAGE_OCCURRENCE('x{copy}','','',#5,#35,$);
",
                a = id,
                b = id + 1,
                c = id + 2,
                d = id + 3
            ),
        );
    }
    let text = append_entities(AP214, &extra);
    let mut topo = Topology::new();
    assert_eq!(
        step::read_step_structure(&text, &mut topo)
            .unwrap()
            .solids
            .len(),
        205
    );
    let limits = ImportLimits {
        max_model_entities: 3_000,
        ..ImportLimits::default()
    };
    match step::read_step_structure_with_limits(&text, &mut Topology::new(), limits) {
        Err(IoError::LimitExceeded { resource, .. }) => {
            assert_eq!(resource, "STEP occurrence copy entities");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_style_chain_that_repeats_its_references_is_resolved_once() {
    // Each level lists the next one 300 times: 300^4 paths, one colour.
    let repeat = |id: u64| vec![format!("#{id}"); 300].join(",");
    let extra = format!(
        "#9100 = COLOUR_RGB('',0.1,0.2,0.3);\n\
         #9101 = FILL_AREA_STYLE_COLOUR('',#9100);\n\
         #9102 = FILL_AREA_STYLE('',({}));\n\
         #9103 = SURFACE_STYLE_FILL_AREA(#9102);\n\
         #9104 = SURFACE_SIDE_STYLE('',({}));\n\
         #9105 = SURFACE_STYLE_USAGE(.BOTH.,#9104);\n\
         #9106 = PRESENTATION_STYLE_ASSIGNMENT(({}));\n\
         #9107 = STYLED_ITEM('',({}),#345);",
        repeat(9101),
        repeat(9103),
        repeat(9105),
        repeat(9106)
    );
    let text = append_entities(PART, &extra);
    let started = std::time::Instant::now();
    let mut topo = Topology::new();
    let result = step::read_step_structure(&text, &mut topo).unwrap();
    assert!(started.elapsed() < std::time::Duration::from_secs(20));
    assert_colour(colour(&topo, result.solids[1]), (0.1, 0.2, 0.3));
}
