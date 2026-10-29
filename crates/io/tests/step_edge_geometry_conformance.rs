//! STEP edge geometry must be conformant ISO 10303-42 advanced B-rep geometry.
//!
//! An `EDGE_CURVE`'s `edge_geometry` (or a `SURFACE_CURVE`'s `curve_3d`) is a
//! line, conic, polyline or B-spline, never a `TRIMMED_CURVE`. Readers that
//! follow the advanced_face rules drop trimmed-curve edges: a cylinder's caps
//! import with no edges, unbounded, and the solid has no volume.
//!
//! Each body here is exported, checked structurally, and read back by remus:
//! the read body must be valid, keep its volume and keep every edge's length.
//!
//! Set `REMUS_STEP_CONFORMANCE_DIR` to also write the exports and a
//! `manifest.json` of expected volumes there, for an external reader such as
//! `scripts/check-step-external.py`.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::HashMap;
use std::f64::consts::PI;
use std::fmt::Write as _;

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{edge_length, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::revolve::revolve;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_edges;
use remus_topology::solid::SolidId;

const NURBS_PLATE: &str = include_str!("data/openzcad_e_nurbs_fillet_plate.step");

const DEFLECTION: f64 = 0.05;

/// `id -> (type, attributes)` for one-entity-per-line writer output. A
/// complex instance is keyed by its first partial type.
fn entities(step: &str) -> HashMap<u64, (String, String)> {
    step.lines()
        .filter_map(|line| {
            let rest = line.trim().strip_prefix('#')?;
            let (id, body) = rest.split_once('=')?;
            let body = body.trim();
            let body = body.strip_prefix('(').map_or(body, str::trim_start);
            let (kind, attrs) = body.split_once('(')?;
            Some((
                id.trim().parse().ok()?,
                (kind.trim().to_string(), attrs.to_string()),
            ))
        })
        .collect()
}

fn refs(attrs: &str) -> Vec<u64> {
    attrs
        .split('#')
        .skip(1)
        .filter_map(|tail| {
            let digits: String = tail.chars().take_while(char::is_ascii_digit).collect();
            digits.parse().ok()
        })
        .collect()
}

/// Count each `EDGE_CURVE`'s geometry type, looking through a
/// `SURFACE_CURVE` to its `curve_3d`, and fail on anything outside the
/// advanced B-rep edge geometry set.
fn assert_conformant_edge_geometry(name: &str, step: &str) -> HashMap<String, usize> {
    let entities = entities(step);
    let mut census = HashMap::<String, usize>::new();
    for (kind, attrs) in entities.values() {
        if kind != "EDGE_CURVE" {
            continue;
        }
        let geometry = refs(attrs)[2];
        let (mut geometry_kind, geometry_attrs) = entities[&geometry].clone();
        if geometry_kind == "SURFACE_CURVE" {
            geometry_kind.clone_from(&entities[&refs(&geometry_attrs)[0]].0);
        }
        assert!(
            matches!(
                geometry_kind.as_str(),
                "LINE"
                    | "CIRCLE"
                    | "ELLIPSE"
                    | "HYPERBOLA"
                    | "PARABOLA"
                    | "POLYLINE"
                    | "B_SPLINE_CURVE_WITH_KNOTS"
                    | "BOUNDED_CURVE"
            ),
            "{name}: EDGE_CURVE geometry must not be a {geometry_kind}"
        );
        *census.entry(geometry_kind).or_default() += 1;
    }
    assert!(!census.is_empty(), "{name}: export has no EDGE_CURVE");
    census
}

fn sorted_edge_lengths(topo: &Topology, solid: SolidId) -> Vec<f64> {
    let mut lengths: Vec<f64> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .map(|edge| edge_length(topo, edge).unwrap())
        .collect();
    lengths.sort_by(f64::total_cmp);
    lengths
}

/// Every NURBS edge's `(knot domain, edge domain)`, sorted.
fn nurbs_domains(topo: &Topology, solid: SolidId) -> Vec<[u64; 4]> {
    let mut domains: Vec<[u64; 4]> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|edge| {
            let edge = topo.edge(edge).unwrap();
            let EdgeCurve::NurbsCurve(nurbs) = edge.curve() else {
                return None;
            };
            let (k0, k1) = nurbs.domain();
            let (t0, t1) = edge.strict_domain().unwrap();
            Some([k0.to_bits(), k1.to_bits(), t0.to_bits(), t1.to_bits()])
        })
        .collect();
    domains.sort_unstable();
    domains
}

/// Export, check conformance, read back and compare with the source body.
fn assert_conformant_round_trip(
    name: &str,
    topo: &Topology,
    solid: SolidId,
    expected_volume: f64,
    expected_curves: &[&str],
) -> String {
    let source_report = validate_solid(topo, solid).unwrap();
    assert!(
        source_report.is_valid(),
        "{name}: {:?}",
        source_report.issues
    );
    let source_volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    assert!(
        (source_volume - expected_volume).abs() <= expected_volume * 1e-6,
        "{name}: source volume {source_volume} != {expected_volume}"
    );

    let step = write_step(topo, &[solid]).unwrap();
    let census = assert_conformant_edge_geometry(name, &step);
    for curve in expected_curves {
        assert!(
            census.contains_key(*curve),
            "{name}: expected {curve} edge geometry, got {census:?}"
        );
    }

    let mut read_topo = Topology::new();
    let solids = read_step(&step, &mut read_topo).unwrap();
    assert_eq!(solids.len(), 1, "{name}");
    let read = solids[0];
    let report = validate_solid(&read_topo, read).unwrap();
    assert!(report.is_valid(), "{name}: {:?}", report.issues);
    let read_volume = solid_volume(&read_topo, read, DEFLECTION).unwrap();
    assert!(
        (read_volume - source_volume).abs() <= source_volume * 1e-9,
        "{name}: read volume {read_volume} != source {source_volume}"
    );

    let source_lengths = sorted_edge_lengths(topo, solid);
    let read_lengths = sorted_edge_lengths(&read_topo, read);
    assert_eq!(source_lengths.len(), read_lengths.len(), "{name}");
    for (source, read) in source_lengths.iter().zip(&read_lengths) {
        assert!(
            (source - read).abs() <= 1e-9 * source.max(1.0),
            "{name}: edge lengths diverge: {source_lengths:?} vs {read_lengths:?}"
        );
    }

    // A second write of the read body is the same file: nothing drifts.
    let again = write_step(&read_topo, &[read]).unwrap();
    let mut again_topo = Topology::new();
    let again_solid = read_step(&again, &mut again_topo).unwrap()[0];
    assert_eq!(
        nurbs_domains(&read_topo, read),
        nurbs_domains(&again_topo, again_solid),
        "{name}: NURBS edge domains drift on a second round trip"
    );

    dump(name, &step, expected_volume);
    step
}

/// Write the export for an external reader when asked to.
fn dump(name: &str, step: &str, expected_volume: f64) {
    let Some(dir) = std::env::var_os("REMUS_STEP_CONFORMANCE_DIR") else {
        return;
    };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join(format!("{name}.step")), step).unwrap();
    let manifest = dir.join(format!("{name}.volume"));
    let mut text = String::new();
    let _ = writeln!(text, "{expected_volume:.9}");
    std::fs::write(manifest, text).unwrap();
}

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
}

#[test]
fn cylinder_exports_basis_circles() {
    let mut topo = Topology::new();
    let cylinder = make_cylinder(&mut topo, 6.0, 28.0).unwrap();
    let step = assert_conformant_round_trip(
        "cylinder",
        &topo,
        cylinder,
        PI * 36.0 * 28.0,
        &["CIRCLE", "LINE"],
    );

    // Both rims are closed circles on one vertex, written with the carrier's
    // own sense.
    let entities = entities(&step);
    let closed_circles = entities
        .values()
        .filter(|(kind, attrs)| {
            if kind != "EDGE_CURVE" {
                return false;
            }
            let refs = refs(attrs);
            let mut geometry = entities[&refs[2]].clone();
            if geometry.0 == "SURFACE_CURVE" {
                geometry = entities[&self::refs(&geometry.1)[0]].clone();
            }
            refs[0] == refs[1] && geometry.0 == "CIRCLE" && attrs.contains(".T.)")
        })
        .count();
    assert_eq!(closed_circles, 2);
}

#[test]
fn box_with_through_hole_exports_basis_circles() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 30.0, 20.0).unwrap();
    let drill = make_cylinder(&mut topo, 5.0, 40.0).unwrap();
    translate(&mut topo, drill, 20.0, 15.0, -10.0);
    let bored = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
    assert_conformant_round_trip(
        "box-through-hole",
        &topo,
        bored,
        40.0 * 30.0 * 20.0 - PI * 25.0 * 20.0,
        &["CIRCLE", "LINE"],
    );
}

#[test]
fn filleted_box_edge_exports_partial_arcs() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 40.0, 30.0, 20.0).unwrap();
    let vertical = solid_edges(&topo, block)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let edge = topo.edge(edge).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            (start.x() - 40.0).abs() < 1e-9
                && (end.x() - 40.0).abs() < 1e-9
                && (start.y() - 30.0).abs() < 1e-9
                && (end.y() - 30.0).abs() < 1e-9
        })
        .unwrap();
    #[allow(deprecated)]
    let filleted =
        remus_operations::fillet::fillet_rolling_ball(&mut topo, block, &[vertical], 5.0).unwrap();
    assert_conformant_round_trip(
        "filleted-box-edge",
        &topo,
        filleted,
        40.0 * 30.0 * 20.0 - (1.0 - PI / 4.0) * 25.0 * 20.0,
        &["CIRCLE", "LINE"],
    );
}

fn revolved_rectangle(topo: &mut Topology, angle: f64) -> SolidId {
    let wire = remus_topology::builder::make_polygon_wire(
        topo,
        &[
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 12.0),
            Point3::new(4.0, 0.0, 12.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = remus_topology::builder::make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        angle,
    )
    .unwrap()
}

#[test]
fn full_revolve_exports_basis_circles() {
    let mut topo = Topology::new();
    let tube = revolved_rectangle(&mut topo, 2.0 * PI);
    assert_conformant_round_trip(
        "revolve-full",
        &topo,
        tube,
        PI * (100.0 - 16.0) * 12.0,
        &["CIRCLE"],
    );
}

#[test]
fn partial_revolve_exports_basis_arcs() {
    let mut topo = Topology::new();
    let sector = revolved_rectangle(&mut topo, 1.5 * PI);
    assert_conformant_round_trip(
        "revolve-270",
        &topo,
        sector,
        0.75 * PI * (100.0 - 16.0) * 12.0,
        // The partial revolve's rims are rational B-spline arcs.
        &["BOUNDED_CURVE", "LINE"],
    );
}

#[test]
fn nurbs_fillet_plate_round_trips_through_basis_geometry() {
    let mut topo = Topology::new();
    let plate = read_step(NURBS_PLATE, &mut topo).unwrap()[0];
    let volume = solid_volume(&topo, plate, DEFLECTION).unwrap();
    // No closed-form volume: the fixture's corner bands are parabolas, and
    // independent readers already disagree on the source file's content
    // (see `openzcad_step_validity.rs`), so the dump carries no `.volume`.
    assert_conformant_round_trip("nurbs-fillet-plate", &topo, plate, volume, &["LINE"]);
    let dumped = std::env::var_os("REMUS_STEP_CONFORMANCE_DIR")
        .map(|dir| std::path::PathBuf::from(dir).join("nurbs-fillet-plate.volume"));
    if let Some(path) = dumped {
        let _ = std::fs::remove_file(path);
    }
}
