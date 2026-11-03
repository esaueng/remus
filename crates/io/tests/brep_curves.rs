//! Native circular carriers, occurrence placements and independent oracles.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::{IoError, arena_io, brep, step};
use remus_math::vec::Point3;
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::{measure, primitives, validate::validate_solid};
use remus_topology::{
    Topology,
    explorer::{solid_edges, solid_faces, solid_vertices},
};

const PLACED_BOX: &str = include_str!("data/brep/freecad-placed-box-v1.brep");
const CYLINDER: &str = include_str!("data/brep/freecad-cylinder-v1.brep");
const TUBE: &str = include_str!("data/brep/tube-v1.brep");

fn near(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && (actual - expected).abs() <= expected.abs().max(1.) * 1e-10,
        "{actual} != {expected}"
    );
}

#[test]
fn genuine_placed_box_keeps_dimensions_and_root_translation() {
    let mut topo = Topology::new();
    let ids = brep::read_brep(PLACED_BOX, &mut topo).unwrap();
    assert_eq!(ids.len(), 1);
    near(
        measure::solid_volume(&topo, ids[0], 0.01).unwrap(),
        140. * 80. * 110.,
    );
    let bounds = measure::solid_bounding_box(&topo, ids[0]).unwrap();
    near(bounds.min.x(), 0.);
    near(bounds.max.x(), 140.);
    near(bounds.min.y(), -40.);
    near(bounds.max.y(), 40.);
    near(bounds.min.z(), 0.);
    near(bounds.max.z(), 110.);
    assert!(validate_solid(&topo, ids[0]).unwrap().is_valid());
}

#[test]
fn genuine_located_cylinder_preserves_periodic_seam_and_reused_cap() {
    let mut topo = Topology::new();
    let ids = brep::read_brep(CYLINDER, &mut topo).unwrap();
    assert_eq!(ids.len(), 1);
    assert_eq!(solid_vertices(&topo, ids[0]).unwrap().len(), 2);
    assert_eq!(solid_edges(&topo, ids[0]).unwrap().len(), 3);
    assert_eq!(solid_faces(&topo, ids[0]).unwrap().len(), 3);
    near(
        measure::solid_volume(&topo, ids[0], 0.01).unwrap(),
        std::f64::consts::PI * 25. * 25. * 110.,
    );
    near(
        measure::solid_surface_area(&topo, ids[0], 0.01).unwrap(),
        2. * std::f64::consts::PI * 25. * 135.,
    );
    let bounds = measure::solid_bounding_box(&topo, ids[0]).unwrap();
    near(bounds.min.x(), 15.);
    near(bounds.max.x(), 65.);
    near(bounds.min.y(), -25.);
    near(bounds.max.y(), 25.);
    near(bounds.min.z(), 0.);
    near(bounds.max.z(), 110.);
    assert!(validate_solid(&topo, ids[0]).unwrap().is_valid());
    let text = step::write_step(&topo, &ids).unwrap();
    let mut restored = Topology::new();
    let ids = step::read_step(&text, &mut restored).unwrap();
    near(
        measure::solid_volume(&restored, ids[0], 0.01).unwrap(),
        std::f64::consts::PI * 25. * 25. * 110.,
    );
}

#[test]
fn annular_caps_keep_the_bore_material_mesh_and_transport_in_all_text_versions() {
    for source in [
        TUBE,
        include_str!("data/brep/tube-v2.brep"),
        include_str!("data/brep/tube-v3.brep"),
    ] {
        let mut topo = Topology::new();
        let ids = brep::read_brep(source, &mut topo).unwrap();
        assert_eq!(solid_vertices(&topo, ids[0]).unwrap().len(), 4);
        assert_eq!(solid_edges(&topo, ids[0]).unwrap().len(), 6);
        assert_eq!(solid_faces(&topo, ids[0]).unwrap().len(), 4);
        assert!(validate_solid(&topo, ids[0]).unwrap().is_valid());
        near(
            measure::solid_volume(&topo, ids[0], 0.01).unwrap(),
            252. * std::f64::consts::PI,
        );
        near(
            measure::solid_surface_area(&topo, ids[0], 0.01).unwrap(),
            210. * std::f64::consts::PI,
        );
        for (x, expected) in [
            (0., PointClassification::Outside),
            (3., PointClassification::Inside),
        ] {
            assert_eq!(
                classify_point(&topo, ids[0], Point3::new(x, 0., 6.), 0.01, 1e-7).unwrap(),
                expected
            );
        }
        let mesh = tessellate_solid(&topo, ids[0], 0.01).unwrap();
        let quality = welded_mesh_quality(&mesh);
        assert_eq!(quality.boundary_edges, 0);
        assert_eq!(quality.non_manifold_edges, 0);
        let origin = Point3::new(0., 0., 0.);
        let volume: f64 = mesh
            .indices
            .chunks_exact(3)
            .map(|tri| {
                let [a, b, c] =
                    [tri[0], tri[1], tri[2]].map(|i| mesh.positions[i as usize] - origin);
                a.dot(b.cross(c)) / 6.
            })
            .sum();
        assert!((volume / (252. * std::f64::consts::PI) - 1.).abs() < 0.01);
        let bytes = arena_io::serialize_solids(&topo, &ids).unwrap();
        let mut restored = Topology::new();
        let restored_ids = arena_io::deserialize_solids(&bytes, &mut restored).unwrap();
        near(
            measure::solid_volume(&restored, restored_ids[0], 0.01).unwrap(),
            252. * std::f64::consts::PI,
        );
        let text = step::write_step(&topo, &ids).unwrap();
        let mut imported = Topology::new();
        let imported_ids = step::read_step(&text, &mut imported).unwrap();
        near(
            measure::solid_volume(&imported, imported_ids[0], 0.01).unwrap(),
            252. * std::f64::consts::PI,
        );
    }
}

fn first_location(source: &str, matrix: &str) -> String {
    let a = source.find("Locations 3\n1\n").unwrap() + "Locations 3\n1\n".len();
    let b = a + source[a..].match_indices('\n').nth(2).unwrap().0;
    format!("{}{matrix}{}", &source[..a], &source[b..])
}

#[test]
fn oblique_root_rotation_keeps_exact_cylinder_carriers() {
    let source = first_location(CYLINDER, "0.6 0 0.8 37\n0 1 0 -19\n-0.8 0 0.6 11");
    let mut topo = Topology::new();
    let ids = brep::read_brep(&source, &mut topo).unwrap();
    near(
        measure::solid_volume(&topo, ids[0], 0.01).unwrap(),
        68750. * std::f64::consts::PI,
    );
    let bounds = measure::solid_bounding_box(&topo, ids[0]).unwrap();
    for (actual, expected) in [
        (bounds.min.x(), 22.),
        (bounds.max.x(), 140.),
        (bounds.min.y(), -44.),
        (bounds.max.y(), 6.),
        (bounds.min.z(), -9.),
        (bounds.max.z(), 97.),
    ] {
        near(actual, expected);
    }
    assert!(validate_solid(&topo, ids[0]).unwrap().is_valid());
    assert_eq!(
        classify_point(&topo, ids[0], Point3::new(81., -19., 44.), 0.01, 1e-7).unwrap(),
        PointClassification::Inside
    );
    let text = step::write_step(&topo, &ids).unwrap();
    let mut restored = Topology::new();
    let restored_ids = step::read_step(&text, &mut restored).unwrap();
    near(
        measure::solid_volume(&restored, restored_ids[0], 0.01).unwrap(),
        68750. * std::f64::consts::PI,
    );
}

fn compound(locations: &[String], children: &str, root: usize) -> String {
    let source = CYLINDER.replacen(
        "Locations 3",
        &format!("Locations {}", locations.len() + 3),
        1,
    );
    let source = source.replacen(
        "Curve2ds 5",
        &format!("{}\nCurve2ds 5", locations.join("\n")),
        1,
    );
    let end = source.rfind("+1 1").unwrap();
    let body = source[..end].replacen("TShapes 9", "TShapes 10", 1);
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
    format!("{body}\nCo\n0100000\n{children} *\n+1 {root}\n")
}

#[test]
fn parent_rotation_precedes_child_translation() {
    let source = compound(&["1\n0 0 1 7\n0 1 0 -3\n-1 0 0 2".to_owned()], "+2 1", 4);
    let mut topo = Topology::new();
    let ids = brep::read_brep(&source, &mut topo).unwrap();
    let bounds = measure::solid_bounding_box(&topo, ids[0]).unwrap();
    for (actual, expected) in [
        (bounds.min.x(), 7.),
        (bounds.max.x(), 117.),
        (bounds.min.y(), -28.),
        (bounds.max.y(), 22.),
        (bounds.min.z(), -63.),
        (bounds.max.z(), -13.),
    ] {
        near(actual, expected);
    }
    near(
        measure::solid_volume(&topo, ids[0], 0.01).unwrap(),
        68750. * std::f64::consts::PI,
    );
}

#[test]
fn repeated_source_solids_retain_distinct_location_identity_even_when_coincident() {
    for x in [40., 140.] {
        let source = compound(&[format!("1\n1 0 0 {x}\n0 1 0 0\n0 0 1 0")], "+2 1 +2 4", 0);
        let mut topo = Topology::new();
        let ids = brep::read_brep(&source, &mut topo).unwrap();
        assert_eq!(ids.len(), 2);
        let a = solid_vertices(&topo, ids[0]).unwrap();
        let b = solid_vertices(&topo, ids[1]).unwrap();
        assert!(a.iter().all(|id| !b.contains(id)));
        assert_eq!(topo.num_vertices(), 4);
        assert_eq!(topo.num_edges(), 6);
        for id in ids {
            near(
                measure::solid_volume(&topo, id, 0.01).unwrap(),
                68750. * std::f64::consts::PI,
            );
        }
    }
}

fn refuses_atomically(source: &str) -> IoError {
    let mut topo = Topology::new();
    let existing = primitives::make_box(&mut topo, 2., 3., 4.).unwrap();
    let before = arena_io::serialize_solids(&topo, &[existing]).unwrap();
    let counts = |t: &Topology| {
        (
            t.num_vertices(),
            t.num_edges(),
            t.num_wires(),
            t.num_faces(),
            t.num_shells(),
            t.num_solids(),
            t.num_loops(),
            t.num_coedges(),
            t.num_pcurves(),
        )
    };
    let census = counts(&topo);
    let error = brep::read_brep(source, &mut topo).unwrap_err();
    assert_eq!(counts(&topo), census);
    assert_eq!(
        arena_io::serialize_solids(&topo, &[existing]).unwrap(),
        before
    );
    error
}

#[test]
fn malformed_circular_traces_seam_branches_and_locations_refuse_atomically() {
    for (old, new) in [
        ("3  1 2CN", "3  1 1CN"),
        (
            "-0.00000000000000000 1.00000000000000000 25.00000000000000000",
            "0 -1 25",
        ),
        ("+1 1", "+1 4"),
    ] {
        let source = CYLINDER.replacen(old, new, 1);
        assert_ne!(source, CYLINDER);
        assert!(matches!(
            refuses_atomically(&source),
            IoError::ParseError { .. }
        ));
    }
    for matrix in ["0 0 0 0\n0 1 0 0\n0 0 1 0", "1 0 0 NaN\n0 1 0 0\n0 0 1 0"] {
        assert!(matches!(
            refuses_atomically(&first_location(CYLINDER, matrix)),
            IoError::ParseError { .. }
        ));
    }
    for matrix in [
        "2 0 0 0\n0 1 0 0\n0 0 1 0",
        "-1 0 0 0\n0 1 0 0\n0 0 1 0",
        "1 0.5 0 0\n0 1 0 0\n0 0 1 0",
    ] {
        assert!(matches!(
            refuses_atomically(&first_location(CYLINDER, matrix)),
            IoError::UnsupportedEntity { .. }
        ));
    }
}

#[test]
fn repeated_located_occurrences_have_a_materialization_budget() {
    let locations: Vec<_> = (0..500)
        .map(|i| format!("1\n1 0 0 {}\n0 1 0 0\n0 0 1 0", 100 * i))
        .collect();
    let children = (4..504)
        .map(|i| format!("+2 {i}"))
        .collect::<Vec<_>>()
        .join(" ");
    let source = compound(&locations, &children, 0);
    assert!(matches!(
        refuses_atomically(&source),
        IoError::LimitExceeded { .. }
    ));
}

#[test]
fn invalid_circular_frames_radii_and_hole_winding_keep_typed_refusals() {
    for (old, new) in [
        ("2 0 0 1 0 0 1 5", "2 0 0 1 0 1 0 5"),
        ("2 0 0 1 0 0 1 5", "2 0 0 1 0 0 1 -5"),
    ] {
        let source = TUBE.replacen(old, new, 1);
        assert_ne!(source, TUBE);
        assert!(matches!(
            refuses_atomically(&source),
            IoError::ParseError { .. }
        ));
    }
    let zero = TUBE.replacen("2 0 0 1 0 0 1 5", "2 0 0 1 0 0 1 0", 1);
    assert!(matches!(
        refuses_atomically(&zero),
        IoError::UnsupportedEntity { .. }
    ));
    // Reverse only the inner rim wire: the cap then winds both loops alike.
    let mut source = TUBE.to_owned();
    let inner = source.rfind("Wi\n").unwrap();
    let sign = inner + source[inner..].find('-').unwrap();
    source.replace_range(sign..=sign, "+");
    assert!(matches!(
        refuses_atomically(&source),
        IoError::ParseError { .. }
    ));
}

#[test]
fn circular_caps_without_saved_traces_derive_exact_authoritative_pcurves() {
    let source = TUBE
        .lines()
        .filter(|line| {
            let words: Vec<_> = line.split_ascii_whitespace().collect();
            !(words.len() == 6 && words[0] == "2" && matches!(words[2], "1" | "2"))
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert_ne!(source, TUBE);
    let mut topo = Topology::new();
    let ids = brep::read_brep(&source, &mut topo).unwrap();
    near(
        measure::solid_volume(&topo, ids[0], 0.01).unwrap(),
        252. * std::f64::consts::PI,
    );
    assert!(validate_solid(&topo, ids[0]).unwrap().is_valid());
    assert!(step::write_step(&topo, &ids).is_ok());
}
