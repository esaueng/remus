//! Issue #1741: a 60 x 40 x 20 box with only its four top edges rounded
//! (R1 or R5), the usual rounded lid. Moving the top face was refused at
//! +/-1 mm: the blend-aware planar move removes the blend loop first, and the
//! positional heal built a disconnected shell, the top face alone above four
//! walls that stopped `r` short of it.
//!
//! The fillet closes each top corner, where the vertical edge stays sharp,
//! with a sphere octant and a flat triangular setback cap at `z = 20 - r`.
//! The cap is neither blend nor tangent support, so the heal kept it, and its
//! plane pinned the walls' recovered corners at `z = 20 - r`. The heal now
//! removes those caps with the band, so the walls meet the top face again,
//! and the defeature refuses before assembly when the healed faces would
//! fall apart into groups sharing no corner.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::defeature::defeature;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::push_pull::move_faces;
use remus_operations::query::filter_filletable_edges;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const LENGTH: f64 = 60.0;
const WIDTH: f64 = 40.0;
const HEIGHT: f64 = 20.0;

/// Closed-form volume of the lid at height `height` with its top loop
/// rounded at `radius`: the box, less a quarter-round strip along each
/// straight run between the corners, less at each corner the `r` cube above
/// the cap that the sphere octant does not fill.
fn lid_volume(radius: f64, height: f64) -> f64 {
    let runs = 2.0 * (LENGTH - 2.0 * radius) + 2.0 * (WIDTH - 2.0 * radius);
    let strips = (1.0 - PI / 4.0) * radius * radius * runs;
    let corners = 4.0 * radius.powi(3) * (1.0 - PI / 6.0);
    LENGTH * WIDTH * height - strips - corners
}

fn top_loop(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            let start = topo.vertex(data.start()).unwrap().point();
            let end = topo.vertex(data.end()).unwrap().point();
            (start.z() - HEIGHT).abs() < 1e-9 && (end.z() - HEIGHT).abs() < 1e-9
        })
        .collect()
}

/// The lid as the WASM `filletDetailed` binding builds it: filletable edges
/// only, then the cascade.
fn lid(radius: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, LENGTH, WIDTH, HEIGHT).unwrap();
    let edges = top_loop(&topo, sharp);
    assert_eq!(edges.len(), 4);
    let edges = filter_filletable_edges(&topo, sharp, &edges).unwrap();
    let solid = fillet_cascade(&mut topo, sharp, &edges, radius)
        .unwrap()
        .solid;
    (topo, solid)
}

/// The planar face whose outward normal is `(0, 0, nz)` and which lies in
/// the plane `z = height`.
fn horizontal_face(topo: &Topology, solid: SolidId, nz: f64, height: f64) -> Option<FaceId> {
    solid_faces(topo, solid).unwrap().into_iter().find(|&face| {
        let data = topo.face(face).unwrap();
        let FaceSurface::Plane { normal, d } = data.surface() else {
            return false;
        };
        let sign = if data.is_reversed() { -1.0 } else { 1.0 };
        (normal.z() * sign - nz).abs() < 1e-9
            && normal.x().abs() < 1e-9
            && normal.y().abs() < 1e-9
            && (d / normal.z() - height).abs() < 1e-9
    })
}

fn census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(face).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

fn assert_valid(topo: &Topology, solid: SolidId, label: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{label}: result must validate: {:?}",
        report
            .issues
            .iter()
            .map(|issue| issue.description.clone())
            .collect::<Vec<_>>()
    );
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, label: &str) {
    for deflection in [2e-3, 1e-3] {
        let volume = solid_volume(topo, solid, deflection).unwrap();
        let error = (volume - expected).abs() / expected;
        assert!(
            error < 1e-5,
            "{label}: volume {volume} at deflection {deflection} differs from the closed form \
             {expected} (relative {error:.3e})"
        );
    }
}

fn classify(topo: &Topology, solid: SolidId, point: Point3) -> PointClassification {
    classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap()
}

fn assert_moves_top(radius: f64, distance: f64) {
    let label = format!("R{radius} top {distance:+}");
    let (mut topo, solid) = lid(radius);
    assert_valid(&topo, solid, "source lid");
    assert_volume(&topo, solid, lid_volume(radius, HEIGHT), "source lid");
    let source_counts = solid_entity_counts(&topo, solid).unwrap();
    let source_census = census(&topo, solid);
    assert_eq!(
        source_census,
        BTreeMap::from([("cylinder", 4), ("plane", 10), ("sphere", 4)]),
        "fixture: four strips, four sphere corners, six box faces and four setback caps"
    );
    let top = horizontal_face(&topo, solid, 1.0, HEIGHT).unwrap();

    let moved = move_faces(&mut topo, solid, &[top], distance)
        .unwrap_or_else(|error| panic!("{label}: move refused: {error}"));
    let height = HEIGHT + distance;

    assert_valid(&topo, moved, &label);
    assert_eq!(
        solid_entity_counts(&topo, moved).unwrap(),
        source_counts,
        "{label}: the move keeps the topology"
    );
    assert_eq!(
        census(&topo, moved),
        source_census,
        "{label}: every blend face stays exact"
    );
    assert_volume(&topo, moved, lid_volume(radius, height), &label);
    // The top face and the blend loop are carried to the new height, and
    // the walls (with the caps under the corners) changed height with them.
    assert!(
        horizontal_face(&topo, moved, 1.0, height).is_some(),
        "{label}: top face"
    );
    assert!(
        horizontal_face(&topo, moved, -1.0, 0.0).is_some(),
        "{label}: bottom stays"
    );
    let caps = solid_faces(&topo, moved)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Plane { normal, d }
                if normal.x().abs() < 1e-9 && normal.y().abs() < 1e-9
                    && (d / normal.z() - (height - radius)).abs() < 1e-9)
        })
        .count();
    assert_eq!(
        caps, 4,
        "{label}: the four setback caps sit r below the moved top"
    );
    let middle = Point3::new(LENGTH / 2.0, WIDTH / 2.0, height - 0.25 * radius);
    assert_eq!(classify(&topo, moved, middle), PointClassification::Inside);
    let above = Point3::new(LENGTH / 2.0, WIDTH / 2.0, height + 0.25);
    assert_eq!(classify(&topo, moved, above), PointClassification::Outside);
    // Just inside the sharp corner the blend has rounded away.
    let corner = Point3::new(0.1 * radius, 0.1 * radius, height - 0.1 * radius);
    assert_eq!(classify(&topo, moved, corner), PointClassification::Outside);
    // Inside the wall just below the rounded edge.
    let wall = Point3::new(LENGTH / 2.0, 0.1, height - 1.1 * radius);
    assert_eq!(classify(&topo, moved, wall), PointClassification::Inside);
}

#[test]
fn moving_the_r5_lid_top_up_carries_the_blend_loop() {
    assert_moves_top(5.0, 1.0);
}

#[test]
fn moving_the_r5_lid_top_down_carries_the_blend_loop() {
    assert_moves_top(5.0, -1.0);
}

#[test]
fn moving_the_r1_lid_top_up_carries_the_blend_loop() {
    assert_moves_top(1.0, 1.0);
}

#[test]
fn moving_the_r1_lid_top_down_carries_the_blend_loop() {
    assert_moves_top(1.0, -1.0);
}

/// The control from the report: the bottom face of the same lid moved
/// before the fix and still does, to the same closed form.
#[test]
fn moving_the_lid_bottom_still_lengthens_the_walls() {
    for radius in [5.0, 1.0] {
        let (mut topo, solid) = lid(radius);
        let bottom = horizontal_face(&topo, solid, -1.0, 0.0).unwrap();
        let moved = move_faces(&mut topo, solid, &[bottom], 1.0).unwrap();
        assert_valid(&topo, moved, "bottom move");
        assert_volume(
            &topo,
            moved,
            lid_volume(radius, HEIGHT + 1.0),
            "bottom move",
        );
    }
}

/// Defeaturing the blend loop without its setback caps leaves the caps'
/// plane pinning the walls `r` below the top: the healed faces split into
/// the lone top face and the rest. The defeature now refuses that before
/// it assembles anything, rather than assembling and then failing
/// validation, and leaves the input untouched.
#[test]
fn defeature_refuses_a_heal_that_strands_the_top_face() {
    let (mut topo, solid) = lid(5.0);
    let band: Vec<FaceId> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(_) | FaceSurface::Sphere(_)
            )
        })
        .collect();
    assert_eq!(band.len(), 8);
    let before = (topo.num_faces(), topo.num_edges(), topo.num_vertices());
    let counts = solid_entity_counts(&topo, solid).unwrap();
    match defeature(&mut topo, solid, &band) {
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "defeature");
            assert!(
                reason.contains("share no corner"),
                "refusal must come from the pre-assembly connectivity guard: {reason}"
            );
        }
        Ok(result) => panic!(
            "defeature without the setback caps returned {:?}",
            solid_entity_counts(&topo, result)
        ),
        Err(other) => panic!("expected a typed refusal, got {other}"),
    }
    assert_eq!(
        (topo.num_faces(), topo.num_edges(), topo.num_vertices()),
        before,
        "a refused heal allocates nothing that survives"
    );
    assert_eq!(solid_entity_counts(&topo, solid).unwrap(), counts);
}
