//! Native saved-shape import: independent physical oracles and refusal controls.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::{ImportLimits, IoError, arena_io, brep, step};
use remus_math::vec::Point3;
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::{measure, primitives, validate::validate_solid};
use remus_topology::{Topology, explorer::solid_faces};

const V1: &str = include_str!("data/brep/freecad-box-v1.brep");
const V2: &str = include_str!("data/brep/box-v2.brep");
const V3: &str = include_str!("data/brep/box-v3.brep");
const TWO_HOLES: &str = include_str!("data/brep/two-holes-v1.brep");
const POCKET: &str = include_str!("data/brep/blind-pocket-v1.brep");
const CONCAVE: &str = include_str!("data/brep/concave-hole-v1.brep");
const OBLIQUE: &str = include_str!("data/brep/oblique-two-holes-v1.brep");

fn hole_count(topo: &Topology, solid: remus_topology::solid::SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .iter()
        .map(|&face| topo.face(face).unwrap().inner_wires().len())
        .sum()
}

#[test]
fn planar_holes_pocket_and_concave_perimeter_keep_material_and_transport() {
    for (source, volume, area, holes, void_probe, solid_probe) in [
        (TWO_HOLES, 864., 728., 4, [3., 3., 4.], [5., 6., 4.]),
        (POCKET, 852., 664., 1, [5., 6., 7.], [5., 6., 4.]),
        (CONCAVE, 752., 620., 2, [3., 3., 4.], [5., 6., 4.]),
        (
            OBLIQUE,
            864.,
            728.,
            4,
            [36.4, -19.68, 16.76],
            [35.2, -17.64, 19.48],
        ),
    ] {
        let mut topo = Topology::new();
        let ids = brep::read_brep(source, &mut topo).unwrap();
        assert_eq!(ids.len(), 1);
        assert!(validate_solid(&topo, ids[0]).unwrap().is_valid());
        assert_eq!(hole_count(&topo, ids[0]), holes);
        near(measure::solid_volume(&topo, ids[0], 0.01).unwrap(), volume);
        near(
            measure::solid_surface_area(&topo, ids[0], 0.01).unwrap(),
            area,
        );
        for (xyz, expected) in [
            (void_probe, PointClassification::Outside),
            (solid_probe, PointClassification::Inside),
        ] {
            assert_eq!(
                classify_point(
                    &topo,
                    ids[0],
                    Point3::new(xyz[0], xyz[1], xyz[2]),
                    0.01,
                    1e-7
                )
                .unwrap(),
                expected
            );
        }
        let mesh = tessellate_solid(&topo, ids[0], 0.01).unwrap();
        let quality = welded_mesh_quality(&mesh);
        assert_eq!(quality.boundary_edges, 0);
        assert_eq!(quality.non_manifold_edges, 0);
        let origin = Point3::new(0., 0., 0.);
        let mesh_volume: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|triangle| {
                let [a, b, c] = [triangle[0], triangle[1], triangle[2]]
                    .map(|index| mesh.positions[index as usize] - origin);
                a.dot(b.cross(c)) / 6.0
            })
            .sum();
        near(mesh_volume, volume);
        let arena = arena_io::serialize_solids(&topo, &ids).unwrap();
        let mut restored = Topology::new();
        let restored_ids = arena_io::deserialize_solids(&arena, &mut restored).unwrap();
        assert_eq!(hole_count(&restored, restored_ids[0]), holes);
        near(
            measure::solid_volume(&restored, restored_ids[0], 0.01).unwrap(),
            volume,
        );
        let exchange = step::write_step(&topo, &ids).unwrap();
        let mut imported = Topology::new();
        let imported_ids = step::read_step(&exchange, &mut imported).unwrap();
        assert_eq!(hole_count(&imported, imported_ids[0]), holes);
        near(
            measure::solid_volume(&imported, imported_ids[0], 0.01).unwrap(),
            volume,
        );
    }
}

fn rewrite_face_loops(source: &str, rewrite: impl Fn(&str) -> String) -> String {
    let mut in_face = false;
    source
        .lines()
        .map(|line| {
            if line == "Fa" {
                in_face = true;
            }
            if in_face && line.ends_with('*') {
                in_face = false;
                rewrite(line)
            } else {
                line.to_owned()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn planar_perimeter_selection_is_independent_of_saved_loop_order() {
    for source in [TWO_HOLES, POCKET, CONCAVE, OBLIQUE] {
        let reordered = rewrite_face_loops(source, |line| {
            let mut pairs: Vec<_> = line
                .trim_end_matches('*')
                .split_ascii_whitespace()
                .collect::<Vec<_>>()
                .chunks_exact(2)
                .map(|pair| pair.join(" "))
                .collect();
            pairs.reverse();
            format!("{} *", pairs.join(" "))
        });
        let mut expected = Topology::new();
        let a = brep::read_brep(source, &mut expected).unwrap();
        let mut actual = Topology::new();
        let b = brep::read_brep(&reordered, &mut actual).unwrap();
        near(
            measure::solid_volume(&actual, b[0], 0.01).unwrap(),
            measure::solid_volume(&expected, a[0], 0.01).unwrap(),
        );
        assert_eq!(hole_count(&actual, b[0]), hole_count(&expected, a[0]));
    }
}

#[test]
fn same_wound_or_missing_holes_refuse_without_changing_existing_solids() {
    let reversed_holes = rewrite_face_loops(TWO_HOLES, |line| {
        if line.split_ascii_whitespace().count() > 3 {
            line.replacen('+', "-", 1)
        } else {
            line.to_owned()
        }
    });
    assert_ne!(reversed_holes, TWO_HOLES.trim());
    assert!(matches!(
        refuses_without_changing_existing_solid(&reversed_holes),
        IoError::ParseError { .. }
    ));
    let missing = rewrite_face_loops(TWO_HOLES, |line| {
        let parts: Vec<_> = line.split_ascii_whitespace().collect();
        if parts.len() > 3 {
            parts[2..].join(" ")
        } else {
            line.to_owned()
        }
    });
    assert!(matches!(
        refuses_without_changing_existing_solid(&missing),
        IoError::InvalidTopology { .. }
    ));
}

#[test]
fn valid_unqualified_edge_flags_are_unsupported_but_malformed_flags_are_errors() {
    for flags in ["0 1 0", "1 0 0", "1 1 1"] {
        let input = V3.replacen("1e-07 1 1 0", &format!("1e-07 {flags}"), 1);
        assert_ne!(input, V3);
        let mut topo = Topology::new();
        assert!(matches!(
            brep::read_brep(&input, &mut topo),
            Err(IoError::UnsupportedEntity { .. })
        ));
        assert_eq!(topo.num_vertices(), 0);
    }
    let input = V3.replacen("1e-07 1 1 0", "1e-07 1 2 0", 1);
    assert_ne!(input, V3);
    let mut topo = Topology::new();
    assert!(matches!(
        brep::read_brep(&input, &mut topo),
        Err(IoError::ParseError { .. })
    ));
}

fn near(value: f64, expected: f64) {
    assert!(
        value.is_finite() && (value - expected).abs() < 1e-8,
        "{value} != {expected}"
    );
}

#[test]
fn freecad_v1_box_preserves_shared_topology_and_physical_properties() {
    let mut topo = Topology::new();
    let solids = brep::read_brep(V1, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    assert_eq!(topo.num_vertices(), 8);
    assert_eq!(topo.num_edges(), 12);
    assert_eq!(solid_faces(&topo, solids[0]).unwrap().len(), 6);
    assert!(validate_solid(&topo, solids[0]).unwrap().is_valid());
    near(
        measure::solid_volume(&topo, solids[0], 0.01).unwrap(),
        1000.,
    );
    near(
        measure::solid_surface_area(&topo, solids[0], 0.01).unwrap(),
        600.,
    );
    let bytes = arena_io::serialize_solids(&topo, &solids).unwrap();
    let mut restored = Topology::new();
    let ids = arena_io::deserialize_solids(&bytes, &mut restored).unwrap();
    near(
        measure::solid_volume(&restored, ids[0], 0.01).unwrap(),
        1000.,
    );
    let exchange = step::write_step(&topo, &solids).unwrap();
    let mut roundtrip = Topology::new();
    let ids = step::read_step(&exchange, &mut roundtrip).unwrap();
    near(
        measure::solid_volume(&roundtrip, ids[0], 0.01).unwrap(),
        1000.,
    );
}

#[test]
fn v3_box_and_optional_draw_header_keep_expected_dimensions() {
    for text in [
        V2.to_owned(),
        V3.to_owned(),
        format!("DBRep_DrawableShape\r\n\r\n{}", V3.replace('\n', "\r\n")),
    ] {
        let mut topo = Topology::new();
        let ids = brep::read_brep(&text, &mut topo).unwrap();
        near(measure::solid_volume(&topo, ids[0], 0.01).unwrap(), 6000.);
        near(
            measure::solid_surface_area(&topo, ids[0], 0.01).unwrap(),
            2200.,
        );
    }
}

fn refuses_without_changing_existing_solid(input: &str) -> IoError {
    let mut topo = Topology::new();
    let existing = primitives::make_box(&mut topo, 2., 3., 4.).unwrap();
    let before = arena_io::serialize_solids(&topo, &[existing]).unwrap();
    let counts = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    );
    let error = brep::read_brep(input, &mut topo).unwrap_err();
    assert_eq!(
        counts,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_solids()
        )
    );
    assert_eq!(
        before,
        arena_io::serialize_solids(&topo, &[existing]).unwrap()
    );
    error
}

#[test]
fn unsupported_encoding_geometry_and_location_refuse() {
    assert!(matches!(
        refuses_without_changing_existing_solid("Open CASCADE Topology V3"),
        IoError::UnsupportedEntity { .. }
    ));
    let curve = V3.replacen("Curves 12\n1 ", "Curves 12\n2 ", 1);
    assert!(matches!(
        refuses_without_changing_existing_solid(&curve),
        IoError::UnsupportedEntity { .. }
    ));
    let surface = V3.replacen("Surfaces 6\n1 ", "Surfaces 6\n2 ", 1);
    assert!(matches!(
        refuses_without_changing_existing_solid(&surface),
        IoError::UnsupportedEntity { .. }
    ));
    let located = V1.replacen(
        "1.000000000000000 0.000000000000000 0.000000000000000 0.000000000000000",
        "1 0 0 5",
        1,
    );
    assert!(matches!(
        refuses_without_changing_existing_solid(&located),
        IoError::UnsupportedEntity { .. }
    ));
}

#[test]
fn byte_and_aggregate_record_budgets_apply_before_materialization() {
    for limits in [
        ImportLimits {
            max_input_bytes: 1,
            ..ImportLimits::default()
        },
        ImportLimits {
            max_model_entities: 100,
            ..ImportLimits::default()
        },
    ] {
        let mut topo = Topology::new();
        assert!(matches!(
            brep::read_brep_with_limits(V1, &mut topo, limits),
            Err(IoError::LimitExceeded { .. })
        ));
        assert_eq!(topo.num_vertices(), 0);
    }
    refuses_without_changing_existing_solid(&V1.replacen(
        "TShapes 34",
        "TShapes 9999999999999999999",
        1,
    ));
}

#[test]
fn malformed_references_truncation_and_trailing_data_refuse() {
    for text in [
        V1.replacen("-34 2 +33 2", "-35 2 +33 2", 1),
        V1.replacen("-34 2 +33 2", "-1 2 +33 2", 1),
        V1[..V1.len() / 2].to_owned(),
        format!("{V1}garbage"),
        V1.replacen("0.000000100000000", "NaN", 1),
    ] {
        refuses_without_changing_existing_solid(&text);
    }
}

#[test]
fn source_pcurve_disagreement_rolls_back_after_allocating_geometry() {
    // The first UV line's origin moves by 2 mm while the 3D geometry is unchanged.
    let text = V1.replacen(
        "Curve2ds 24\n1 0.00000000000000000",
        "Curve2ds 24\n1 2.00000000000000000",
        1,
    );
    let error = refuses_without_changing_existing_solid(&text);
    assert!(error.to_string().contains("pcurve"), "{error}");
}

#[test]
fn strict_validation_refuses_an_open_shell_after_materialization() {
    let text = V1.replacen("-25 0 +15 0", "+15 0", 1);
    assert!(matches!(
        refuses_without_changing_existing_solid(&text),
        IoError::InvalidTopology { .. }
    ));
}

#[test]
fn non_solid_roots_are_not_silently_discarded() {
    let at = V1.rfind("+1 1").unwrap();
    let text = format!("{}+2 1", &V1[..at]);
    assert!(matches!(
        refuses_without_changing_existing_solid(&text),
        IoError::UnsupportedEntity { .. }
    ));
}

#[test]
fn arithmetic_overflow_and_reversed_trims_refuse_atomically() {
    for text in [
        V3.replacen("1  1 0 0 30", "1  1 0 0 1e308", 1),
        V3.replacen("2  1 1 0 0 30", "2  1 1 0 30 0", 1),
    ] {
        refuses_without_changing_existing_solid(&text);
    }
}

fn compound(children: &str) -> String {
    let end = V3.rfind("+1 0").unwrap();
    let body = V3[..end].replacen("TShapes 34", "TShapes 35", 1);
    let body = body
        .lines()
        .map(|line| {
            if !line.ends_with('*') {
                return line.to_owned();
            }
            line.split_ascii_whitespace()
                .map(|token| {
                    if token.starts_with('+') || token.starts_with('-') {
                        format!(
                            "{}{}",
                            &token[..1],
                            token[1..].parse::<usize>().unwrap() + 1
                        )
                    } else {
                        token.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n");
    format!("{body}\nCo\n1100000\n{children} *\n+1 0")
}

#[test]
fn compound_solid_roots_import_but_repeated_or_mixed_roots_refuse() {
    let mut topo = Topology::new();
    let ids = brep::read_brep(&compound("+2 0"), &mut topo).unwrap();
    assert_eq!(ids.len(), 1);
    near(measure::solid_volume(&topo, ids[0], 0.01).unwrap(), 6000.);
    for children in ["+2 0 +2 0", "+2 0 +3 0"] {
        assert!(matches!(
            refuses_without_changing_existing_solid(&compound(children)),
            IoError::UnsupportedEntity { .. }
        ));
    }
}

#[test]
fn planar_edges_without_saved_pcurves_remain_exact_and_exportable() {
    let mut lines = V3.lines();
    let mut output = Vec::new();
    while let Some(line) = lines.next() {
        if line == "Curve2ds 24" {
            output.push("Curve2ds 0");
            for _ in 0..24 {
                lines.next().unwrap();
            }
        } else if !line.starts_with("2  ") {
            output.push(line);
        }
    }
    let mut topo = Topology::new();
    let ids = brep::read_brep(&output.join("\n"), &mut topo).unwrap();
    near(measure::solid_volume(&topo, ids[0], 0.01).unwrap(), 6000.);
    assert_eq!(topo.num_pcurves(), 0);
    let exchange = step::write_step(&topo, &ids).unwrap();
    let mut restored = Topology::new();
    let ids = step::read_step(&exchange, &mut restored).unwrap();
    near(
        measure::solid_volume(&restored, ids[0], 0.01).unwrap(),
        6000.,
    );
}

#[test]
fn shared_shell_occurrences_cannot_expand_into_repeated_native_boundaries() {
    let end = V3.rfind("+1 0").unwrap();
    let body = V3[..end].replacen("TShapes 34", "TShapes 36", 1);
    let body = body
        .lines()
        .map(|line| {
            if !line.ends_with('*') {
                return line.to_owned();
            }
            line.split_ascii_whitespace()
                .map(|token| {
                    if token.starts_with('+') || token.starts_with('-') {
                        format!(
                            "{}{}",
                            &token[..1],
                            token[1..].parse::<usize>().unwrap() + 2
                        )
                    } else {
                        token.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n");
    // Two distinct solid records reference the same shell. They are distinct
    // roots, so merely deduplicating root identities does not bound the work.
    let text = format!("{body}\nSo\n1100000\n+4 0 *\nCo\n1100000\n+3 0 +2 0 *\n+1 0");
    let error = refuses_without_changing_existing_solid(&text);
    assert!(matches!(error, IoError::UnsupportedEntity { .. }));
    assert!(error.to_string().contains("repeated shell"));
}

#[test]
fn excess_edge_representations_refuse_before_unbounded_per_face_search() {
    let line = "2  1 1 0 0 30";
    let text = V3.replacen(line, &format!("{line}\n{line}\n{line}"), 1);
    let error = refuses_without_changing_existing_solid(&text);
    assert!(error.to_string().contains("two pcurves"));
}
