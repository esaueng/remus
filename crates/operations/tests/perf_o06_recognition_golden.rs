//! O06 equivalence pins for whole-solid feature recognition and edge
//! relations.
//!
//! OpenZCAD re-runs `recognizeFeatures` and `solidEdgeRelations` on an
//! imported body after every direct edit, so both calls were made fast by
//! preparing per-call state once (the analytic point classifier, NURBS seed
//! grids, edge normal samples). None of that may change an answer. These
//! tests pin the complete outputs — every feature with its faces and area,
//! every edge with its verdict and signed angle — on the imported
//! `shapr3d_hammer_holder.step` body and on synthetic fixtures the existing
//! suites already use. The golden files were generated from the code before
//! the O06 change and must not be regenerated to make a performance change
//! pass.
//!
//! Discrete content (feature kinds, face and edge handles, verdicts, counts,
//! `none`) compares exactly. Floating values compare to 1e-9 relative: the
//! goldens are generated on one platform, and CI's libm may round a
//! transcendental differently in the last place. The bit-exact proof is the
//! same-process differential test below and the byte-identical dumps of
//! `crates/remus/examples/recognition_perf.rs` (see
//! `docs/performance/perf-o06-recognition.md`).
//!
//! Regenerate a golden only for an intentional semantic change, with
//! `UPDATE_GOLDEN=1`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::format_push_string,
    // The filleted fixture mirrors `query.rs`'s blend-spring test, which
    // builds its band with the rolling-ball engine.
    deprecated
)]

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use remus_check::classify::{ClassifyOptions, PreparedSolid, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::chamfer::chamfer;
use remus_operations::feature_recognition::{Feature, recognize_features};
use remus_operations::fillet::fillet_rolling_ball;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::query::{
    EdgeConcavity, EdgeRelation, default_concavity_probe, edge_relation, effective_face_normal,
    solid_edge_relations, trimmed_edge_domain,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_edges, solid_vertices};
use remus_topology::solid::SolidId;

const HAMMER_HOLDER: &str = include_str!("../../io/tests/data/shapr3d_hammer_holder.step");

/// The deflection OpenZCAD passes to `recognizeFeatures`
/// (`MEASUREMENT_DEFLECTION`).
const CONSUMER_DEFLECTION: f64 = 0.08;

/// The deflection the feature-recognition qualification suite uses.
const SUITE_DEFLECTION: f64 = 0.05;

// ── Golden helpers ───────────────────────────────────────────────────

fn golden_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/golden/data/o06")
        .join(name)
}

/// Whether two golden tokens agree: `key=value` keys and non-numeric values
/// exactly, numeric values to 1e-9 relative.
fn tokens_agree(expected: &str, actual: &str) -> bool {
    match (expected.split_once('='), actual.split_once('=')) {
        (Some((ek, ev)), Some((ak, av))) => {
            if ek != ak {
                return false;
            }
            match (ev.parse::<f64>(), av.parse::<f64>()) {
                (Ok(e), Ok(a)) => {
                    let scale = 1.0_f64.max(e.abs()).max(a.abs());
                    (e - a).abs() <= 1e-9 * scale
                }
                _ => ev == av,
            }
        }
        (None, None) => expected == actual,
        _ => false,
    }
}

fn assert_golden(name: &str, actual: &str) {
    let path = golden_path(name);
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, actual).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|_| {
        panic!(
            "golden file not found: {}\nRun with UPDATE_GOLDEN=1 to create it.",
            path.display()
        )
    });
    let expected = expected.replace("\r\n", "\n");
    let expected_lines: Vec<&str> = expected.trim().lines().collect();
    let actual_lines: Vec<&str> = actual.trim().lines().collect();
    assert_eq!(
        expected_lines.len(),
        actual_lines.len(),
        "{name}: line count differs\n--- actual ---\n{actual}"
    );
    for (index, (e, a)) in expected_lines.iter().zip(&actual_lines).enumerate() {
        let et: Vec<&str> = e.split_whitespace().collect();
        let at: Vec<&str> = a.split_whitespace().collect();
        assert!(
            et.len() == at.len() && et.iter().zip(&at).all(|(x, y)| tokens_agree(x, y)),
            "{name}: line {} differs\n  expected: {e}\n  actual:   {a}",
            index + 1
        );
    }
}

fn list<T: std::fmt::Display>(items: impl IntoIterator<Item = T>) -> String {
    let items: Vec<String> = items.into_iter().map(|i| i.to_string()).collect();
    format!("[{}]", items.join(","))
}

fn number(value: Option<f64>) -> String {
    value.map_or_else(|| "none".to_string(), |v| format!("{v:?}"))
}

/// One line per feature, in output order, with every handle and value.
fn render_features(features: &[Feature]) -> String {
    let mut out = format!("features={}\n", features.len());
    for feature in features {
        match feature {
            Feature::Hole {
                faces,
                diameter,
                through,
            } => writeln!(
                out,
                "hole faces={} diameter={} through={through}",
                list(faces.iter().map(|f| f.index())),
                number(*diameter)
            ),
            Feature::Chamfer {
                face,
                adjacent,
                angle,
            } => writeln!(
                out,
                "chamfer face={} adjacent={} angle={angle:?}",
                face.index(),
                list([adjacent.0.index(), adjacent.1.index()])
            ),
            Feature::FilletLike { face, area } => {
                writeln!(out, "fillet_like face={} area={area:?}", face.index())
            }
            Feature::Pocket { floor, walls } => writeln!(
                out,
                "pocket floor={} walls={}",
                floor.index(),
                list(walls.iter().map(|w| w.index()))
            ),
            Feature::Pattern {
                feature_indices,
                pattern_type,
                count,
                spacing,
            } => writeln!(
                out,
                "pattern type={pattern_type:?} count={count} members={} spacing={}",
                list(feature_indices.iter().copied()),
                number(*spacing)
            ),
            other => writeln!(out, "other {other:?}"),
        }
        .unwrap();
    }
    out
}

/// One line per edge, in output order: handle, verdict, signed angle.
fn render_relations(relations: &[EdgeRelation]) -> String {
    let mut out = format!("relations={}\n", relations.len());
    for relation in relations {
        writeln!(
            out,
            "edge={} relation={} angle={}",
            relation.edge.index(),
            relation.concavity.as_str(),
            number(relation.dihedral_angle)
        )
        .unwrap();
    }
    out
}

// ── Fixtures ─────────────────────────────────────────────────────────

fn hammer_holder(topo: &mut Topology) -> SolidId {
    remus_io::step::reader::read_step(HAMMER_HOLDER, topo).expect("import")[0]
}

/// The two-hole plate of `qualify_feature_recognition::recognition_is_deterministic`.
fn two_hole_plate(topo: &mut Topology) -> SolidId {
    let cube = make_box(topo, 3.0, 2.0, 1.0).unwrap();
    let d1 = make_cylinder(topo, 0.2, 2.0).unwrap();
    transform_solid(topo, d1, &Mat4::translation(0.7, 1.0, -0.5)).unwrap();
    let d2 = make_cylinder(topo, 0.2, 2.0).unwrap();
    transform_solid(topo, d2, &Mat4::translation(2.3, 1.0, -0.5)).unwrap();
    let b1 = boolean(topo, BooleanOp::Cut, cube, d1).unwrap();
    boolean(topo, BooleanOp::Cut, b1, d2).unwrap()
}

/// The pocketed box of `qualify_feature_recognition::rectangular_pocket_recognized`.
fn pocketed_box(topo: &mut Topology) -> SolidId {
    let cube = make_box(topo, 2.0, 2.0, 1.0).unwrap();
    let tool = make_box(topo, 0.6, 0.8, 0.5).unwrap();
    transform_solid(topo, tool, &Mat4::translation(0.7, 0.6, 0.5)).unwrap();
    boolean(topo, BooleanOp::Cut, cube, tool).unwrap()
}

/// The blind-hole plate of the B16 edge-relation tests.
fn blind_hole_plate(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 20.0, 20.0, 6.0).unwrap();
    let drill = make_cylinder(topo, 3.0, 4.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(10.0, 10.0, 2.0)).unwrap();
    boolean(topo, BooleanOp::Cut, plate, drill).unwrap()
}

/// A unit box with its top +X edge chamfered (the qualification fixture).
fn chamfered_box(topo: &mut Topology) -> SolidId {
    let cube = make_box(topo, 1.0, 1.0, 1.0).unwrap();
    let target = solid_edges(topo, cube)
        .unwrap()
        .into_iter()
        .find(|&edge| {
            let e = topo.edge(edge).unwrap();
            let on = |p: Point3| (p.x() - 1.0).abs() < 1e-9 && (p.z() - 1.0).abs() < 1e-9;
            on(topo.vertex(e.start()).unwrap().point()) && on(topo.vertex(e.end()).unwrap().point())
        })
        .expect("top +X edge");
    chamfer(topo, cube, &[target], 0.15).unwrap()
}

/// A 10 mm box with one rolling-ball fillet (the blend-spring fixture).
fn filleted_box(topo: &mut Topology) -> SolidId {
    let cube = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let edge = solid_edges(topo, cube).unwrap()[0];
    fillet_rolling_ball(topo, cube, &[edge], 1.0).unwrap()
}

type Fixture = (&'static str, fn(&mut Topology) -> SolidId);

const SYNTHETIC: [Fixture; 5] = [
    ("two_hole_plate", two_hole_plate),
    ("pocketed_box", pocketed_box),
    ("blind_hole_plate", blind_hole_plate),
    ("chamfered_box", chamfered_box),
    ("filleted_box", filleted_box),
];

// ── Golden pins ──────────────────────────────────────────────────────

#[test]
fn hammer_holder_recognition_matches_golden() {
    let mut topo = Topology::new();
    let solid = hammer_holder(&mut topo);
    let features = recognize_features(&topo, solid, CONSUMER_DEFLECTION).unwrap();
    assert_golden("hammer_holder_features.golden", &render_features(&features));
}

#[test]
fn hammer_holder_edge_relations_match_golden() {
    let mut topo = Topology::new();
    let solid = hammer_holder(&mut topo);
    let relations = solid_edge_relations(&topo, solid, None).unwrap();
    assert_golden(
        "hammer_holder_relations.golden",
        &render_relations(&relations),
    );
}

#[test]
fn synthetic_recognition_matches_golden() {
    let mut out = String::new();
    for (name, build) in SYNTHETIC {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        let features = recognize_features(&topo, solid, SUITE_DEFLECTION).unwrap();
        writeln!(out, "fixture={name}").unwrap();
        out.push_str(&render_features(&features));
    }
    assert_golden("synthetic_features.golden", &out);
}

#[test]
fn synthetic_edge_relations_match_golden() {
    let mut out = String::new();
    for (name, build) in SYNTHETIC {
        let mut topo = Topology::new();
        let solid = build(&mut topo);
        writeln!(out, "fixture={name} probe=default").unwrap();
        out.push_str(&render_relations(
            &solid_edge_relations(&topo, solid, None).unwrap(),
        ));
        // One caller probe for every edge, small enough to stay local on the
        // smallest fixture so the verdicts stay meaningful.
        writeln!(out, "fixture={name} probe=0.01").unwrap();
        out.push_str(&render_relations(
            &solid_edge_relations(&topo, solid, Some(0.01)).unwrap(),
        ));
    }
    assert_golden("synthetic_relations.golden", &out);
}

/// The single-edge query reaches the bulk row for edges of the import.
#[test]
fn hammer_holder_single_edge_queries_match_bulk_rows() {
    let mut topo = Topology::new();
    let solid = hammer_holder(&mut topo);
    let bulk = solid_edge_relations(&topo, solid, None).unwrap();
    // Every 7th edge keeps the test-profile run short while still covering
    // every surface family on the body.
    for row in bulk.iter().step_by(7) {
        let single = edge_relation(&topo, solid, row.edge, None).unwrap();
        assert_eq!(single.concavity, row.concavity, "edge {}", row.edge.index());
        assert_eq!(
            single.dihedral_angle.map(f64::to_bits),
            row.dihedral_angle.map(f64::to_bits),
            "edge {}",
            row.edge.index()
        );
    }
    assert!(
        bulk.iter()
            .any(|row| row.concavity == EdgeConcavity::Concave)
            && bulk
                .iter()
                .any(|row| row.concavity == EdgeConcavity::Convex)
            && bulk
                .iter()
                .any(|row| row.concavity == EdgeConcavity::Tangent),
        "the import must exercise every verdict"
    );
}

// ── Same-process differential proof ──────────────────────────────────

/// A deterministic corpus around the imported body: the concavity quadrant
/// probes of every manifold edge (what recognition classifies), every vertex
/// (on the boundary), and points just inside and outside the 1e-7 boundary
/// band at each vertex.
fn hammer_probe_corpus(topo: &Topology, solid: SolidId) -> Vec<Point3> {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let mut points = Vec::new();
    for edge in solid_edges(topo, solid).unwrap() {
        let faces = adjacency.faces_for_edge(edge);
        if faces.len() != 2 || faces[0] == faces[1] {
            continue;
        }
        let Ok(probe) = default_concavity_probe(topo, edge, faces[0], faces[1]) else {
            continue;
        };
        let edge_data = topo.edge(edge).unwrap();
        let start = topo.vertex(edge_data.start()).unwrap().point();
        let end = topo.vertex(edge_data.end()).unwrap().point();
        let (t0, t1) = trimmed_edge_domain(topo, edge).unwrap();
        let mid = edge_data
            .curve()
            .evaluate_with_endpoints(f64::midpoint(t0, t1), start, end);
        let (Some(na), Some(nb)) = (
            effective_face_normal(topo, faces[0], mid),
            effective_face_normal(topo, faces[1], mid),
        ) else {
            continue;
        };
        for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
            points.push(mid + (na * sa + nb * sb) * -probe);
        }
    }
    for vertex in solid_vertices(topo, solid).unwrap() {
        let p = topo.vertex(vertex).unwrap().point();
        points.push(p);
        for offset in [0.5e-7, 2.0e-7, 1.0e-5] {
            points.push(Point3::new(p.x() + offset, p.y() - offset, p.z() + offset));
        }
    }
    points
}

/// The prepared classifier (with its NURBS seed grids, cached UV trims and
/// support-hull boundary skip) answers exactly like the one-shot classifier
/// on probes of the imported body. Same process, so libm cannot differ.
#[test]
fn hammer_holder_prepared_classification_matches_one_shot() {
    let mut topo = Topology::new();
    let solid = hammer_holder(&mut topo);
    let options = ClassifyOptions {
        tolerance: 1e-7,
        ..Default::default()
    };
    let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
    let corpus = hammer_probe_corpus(&topo, solid);
    assert!(corpus.len() > 1_000, "corpus too small: {}", corpus.len());
    let mut verdicts = std::collections::BTreeMap::new();
    // Every 3rd point keeps the one-shot reference affordable in the test
    // profile; the prepared passes below still classify all of them.
    for (index, &point) in corpus.iter().enumerate().step_by(3) {
        let expected = classify_point(&topo, solid, point, &options).unwrap();
        let actual = prepared.classify_point(point, &options).unwrap();
        assert_eq!(actual, expected, "probe {index} at {point:?}");
        *verdicts.entry(format!("{expected:?}")).or_insert(0usize) += 1;
    }
    assert_eq!(
        verdicts.len(),
        3,
        "the corpus must reach Inside, Outside and OnBoundary: {verdicts:?}"
    );
    // Lazily filled caches must not change a later answer.
    let first = prepared.classify_points(&corpus, &options).unwrap();
    let again = prepared.classify_points(&corpus, &options).unwrap();
    assert_eq!(first, again);
}
