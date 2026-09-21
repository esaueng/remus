//! Direct-edit regression corpus: synthetic fixtures for fillet-band removal.
//!
//! ## Coverage matrix
//!
//! | # | Test | Requirement bullet | Outcome class |
//! |---|------|--------------------|---------------|
//! | T1 | convex removal, see below | concave+convex plane-to-plane (convex side) | covered by existing tests (no corpus duplicate): `regress_defeature_heals_or_refuses::plane_plane_fillet_removal_restores_the_sharp_edge` + `research_blend_edit_probe::e1_plane_plane_fillet_structure` (E6 unfillet); T12 executes convex removal on a STEP-reimported band |
//! | T2 | `concave_notch_unfillet_restores_sharp_notch` | concave plane-to-plane | supported success |
//! | T3 | `wall_base_inside_corner_unfillet_restores_l_blank` | fillet ending against planar faces | supported success |
//! | T4 | `band_ending_against_curved_notch_is_refused_at_creation` | fillet ending against a curved face | creation-side boundary: `Blend` refusal (code unpinned), input preserved |
//! | T5 | `trihedral_full_network_reports_region_but_refuses_fail_closed` | connected blends, multi-fillet corner | `blend_region` groups the network (success) + `defeature` refuses fail-closed (known gap) |
//! | T6 | `partial_corner_band_removal_is_refused_fail_closed` | connected blends (negative side) | intentional safe refusal (permanent) |
//! | T7 | `blind_hole_floor_fillet_unfillets_to_sharp_floor` | blind pocket | supported success |
//! | T8 | `boss_base_fillet_unfillets_to_sharp_rim` | boss | supported success |
//! | T9 | `hole_top_rim_unfillet_crosses_bore_seam_cleanly` | periodic surface seam | supported success |
//! | T10 | `outer_band_on_hollow_solid_is_refused_cavity_kept` | cavity shells | intentional safe refusal (declared boundary, permanent) |
//! | T11 | `unfillet_preserves_unrelated_bores_and_boss` | unrelated curved geometry + holes survive | supported success |
//! | T12 | `step_reimported_bands_unfillet_to_sharp_bodies` | STEP import/export/reimport invariants + imported-R1 removal analogue | supported success |
//!
//! ## Conventions (deliberate non-duplication, no proprietary models)
//!
//! * Synthetic fixtures only: `make_box` / `make_cylinder` + `boolean` +
//!   `fillet_v2`. Nothing is imported, nothing is committed as a STEP file,
//!   and no user-supplied geometry appears.
//! * Deliberate non-duplication: where a behavior is already covered (convex
//!   native removal, cavity refusal, STEP analytics), the matrix names the
//!   existing test instead of re-proving it; the corpus adds only the
//!   uncovered delta (T1 row, T10, T12).
//! * Geometric selectors (coordinates, surface/curve type, radius) — never
//!   hard-coded arena handles. Each test establishes baseline validity
//!   (closed solid, exact or recorded volume) *before* the edit, then checks
//!   exact surface/curve types, topology counts, dimensions and volume after
//!   it. Tessellation watertightness is an extra, never the verdict.
//!
//! ## Outcome classes
//!
//! * **Supported success** — `defeature` restores the sharp body and its
//!   volume; the band's analytic carrier is gone and the supports survive.
//! * **Intentional safe refusal** — `defeature` returns a typed refusal
//!   (`Unsupported { operation: "defeature" }` or `ResizeBlend`) and the input
//!   solid is bit-for-bit preserved (face/edge counts, volume, shells,
//!   validity). T6 and T10 are permanent boundaries; the second half of T5 is
//!   a known gap awaiting local reconstruction. T4 pins the creation-side
//!   boundary (a `Blend` refusal whose code is deliberately unpinned).
//! * **Known defects** are never baked in as passing successes: anything that
//!   cannot succeed today is asserted as fail-closed refusal + preservation,
//!   labelled `KNOWN GAP`, and recorded below.
//!
//! ## Limitations (missing capabilities, not passing tests)
//!
//! * Connected multi-band networks (T5: three cylinders + corner sphere) have
//!   no `defeature` path yet — the analytic-band entry requires a torus seed,
//!   so the network falls through to plane extension and refuses. The region
//!   query already groups the four faces, which is the machine-readable hook.
//! * Open bands ending against a curved face (T4) cannot be constructed
//!   natively — the rolling ball has no room where the edge meets the notch
//!   wall, so creation fails (cliff on degenerate input, trimming failure on
//!   clean input; the code is unpinned on purpose). Other CAD systems produce
//!   such bands, so an *imported* curved-ending R1 remains the vehicle for
//!   removal coverage here; no user STEP models are committed by this corpus,
//!   so that case is recorded, not fabricated.
//! * Cavity-bearing solids (T10) are refused by declaration, before any wound
//!   analysis.
//! * Plane-extension heals require every kept face to be planar
//!   (`heal_by_extending`), so extend-path removal on a solid carrying any
//!   unrelated curved geometry refuses structurally. Survival edits must go
//!   through the analytic-band path (T11 proves it preserves far-field curved
//!   features exactly).
//!
//! ## Reuse notes for the active fillet-removal agent
//!
//! * T12 is the minimal imported-R1 analogue: convex cylinder band and torus
//!   band through STEP export/reimport (helpers: `roundtrip`,
//!   volume-matched pairing), then `defeature` on the reimported bodies.
//! * T8 / T9 are the native torus-band analogues (boss base, hole rim across
//!   a bore seam) with major/minor/center assertions to compare a
//!   reconstructed band against.
//! * `cylinder_bands`, `torus_bands`, `closed_circle_rims`, `snapshot` /
//!   `assert_preserved` are reusable across new fixtures.
//! * T4, T5, T11 are the priority open cases: curved-ending open bands (needs
//!   imported R1s), ambiguous network corners, and survival of unrelated
//!   geometry.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::defeature::defeature;
use remus_operations::extrude::extrude;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::resize_blend::blend_region;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.02;
const REL_TOL: f64 = 1e-6;

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{what}: baseline solid must validate, got {:?}",
        report
            .issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).unwrap()
}

fn assert_volume_near(actual: f64, expected: f64, what: &str) {
    let rel = (actual - expected).abs() / expected.abs().max(1e-15);
    assert!(
        rel < REL_TOL,
        "{what}: volume {actual} differs from expected {expected} (rel {rel:.3e})"
    );
}

fn face_count(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid).unwrap().len()
}

fn edge_count(topo: &Topology, solid: SolidId) -> usize {
    solid_edges(topo, solid).unwrap().len()
}

/// Cylinder blend bands of `radius` (within 1e-9), any orientation.
fn cylinder_bands(topo: &Topology, solid: SolidId, radius: f64) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9
            )
        })
        .collect()
}

/// Torus blend bands of minor radius `minor` (within 1e-9).
fn torus_bands(topo: &Topology, solid: SolidId, minor: f64) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                FaceSurface::Torus(t) if (t.minor_radius() - minor).abs() < 1e-9
            )
        })
        .collect()
}

/// Sphere patches of `radius` (within 1e-9).
fn sphere_patches(topo: &Topology, solid: SolidId, radius: f64) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            matches!(
                topo.face(f).unwrap().surface(),
                FaceSurface::Sphere(s) if (s.radius() - radius).abs() < 1e-9
            )
        })
        .collect()
}

/// Reversed (hole-wall) cylinder faces of `radius`.
fn reversed_cylinder_walls(topo: &Topology, solid: SolidId, radius: f64) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            face.is_reversed()
                && matches!(
                    face.surface(),
                    FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9
                )
        })
        .collect()
}

/// Non-reversed (boss-wall) cylinder faces of `radius`. A fused protrusion
/// carries outward normals, so its wall is not reversed — unlike a bore wall.
fn protrusion_cylinder_walls(topo: &Topology, solid: SolidId, radius: f64) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            !face.is_reversed()
                && matches!(
                    face.surface(),
                    FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-9
                )
        })
        .collect()
}

/// Closed (start == end) exact circle edges: (center, radius).
fn closed_circle_rims(topo: &Topology, solid: SolidId) -> Vec<(Point3, f64)> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|e| {
            let edge = topo.edge(e).unwrap();
            if edge.start() != edge.end() {
                return None;
            }
            match edge.curve() {
                EdgeCurve::Circle(c) => Some((c.center(), c.radius())),
                _ => None,
            }
        })
        .collect()
}

fn has_closed_rim_at(
    topo: &Topology,
    solid: SolidId,
    x: f64,
    y: f64,
    z: f64,
    radius: f64,
    what: &str,
) {
    let found = closed_circle_rims(topo, solid)
        .iter()
        .filter(|(c, r)| {
            (c.x() - x).abs() < 1e-9
                && (c.y() - y).abs() < 1e-9
                && (c.z() - z).abs() < 1e-9
                && (r - radius).abs() < 1e-9
        })
        .count();
    assert_eq!(
        found, 1,
        "{what}: expected exactly one closed Circle rim at ({x}, {y}, {z}) r={radius}, found {found}"
    );
}

struct Snapshot {
    faces: usize,
    edges: usize,
    volume_bits: u64,
    inner_shells: usize,
}

fn snapshot(topo: &Topology, solid: SolidId) -> Snapshot {
    Snapshot {
        faces: face_count(topo, solid),
        edges: edge_count(topo, solid),
        volume_bits: volume(topo, solid).to_bits(),
        inner_shells: topo.solid(solid).unwrap().inner_shells().len(),
    }
}

/// A refusal must leave the input untouched: same faces, edges, volume bits,
///
/// shells, and continued validity.
fn assert_preserved(topo: &Topology, solid: SolidId, before: &Snapshot, what: &str) {
    assert_eq!(
        face_count(topo, solid),
        before.faces,
        "{what}: refusal changed the face count"
    );
    assert_eq!(
        edge_count(topo, solid),
        before.edges,
        "{what}: refusal changed the edge count"
    );
    assert_eq!(
        volume(topo, solid).to_bits(),
        before.volume_bits,
        "{what}: refusal changed the input volume"
    );
    assert_eq!(
        topo.solid(solid).unwrap().inner_shells().len(),
        before.inner_shells,
        "{what}: refusal changed the cavity shells"
    );
    assert_valid(topo, solid, &format!("{what}: input after refusal"));
}

/// Accept only the two typed refusal families; anything else (success,
/// `InvalidInput`, untyped errors) fails the test with its payload.
fn assert_typed_refusal(result: Result<SolidId, OperationsError>, what: &str) -> String {
    match result {
        Ok(_) => panic!("{what}: expected a typed refusal, the edit unexpectedly succeeded"),
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "defeature", "{what}: wrong operation tag");
            assert!(!reason.is_empty(), "{what}: refusal must name a reason");
            format!("defeature:Unsupported:{reason}")
        }
        Err(OperationsError::ResizeBlend(err)) => format!("resize-blend:{}", err.code()),
        Err(other) => {
            panic!("{what}: expected a typed Unsupported/ResizeBlend refusal, got {other:?}")
        }
    }
}

fn vertex_point(topo: &Topology, e: EdgeId, take_start: bool) -> Point3 {
    let edge = topo.edge(e).unwrap();
    topo.vertex(if take_start { edge.start() } else { edge.end() })
        .unwrap()
        .point()
}

/// Straight edges whose endpoints both satisfy `predicate`.
fn straight_edges_where(
    topo: &Topology,
    solid: SolidId,
    predicate: impl Fn(Point3, Point3) -> bool,
) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            if edge.start() == edge.end() || !matches!(edge.curve(), EdgeCurve::Line) {
                return false;
            }
            predicate(vertex_point(topo, e, true), vertex_point(topo, e, false))
        })
        .collect()
}

fn roundtrip(topo: &Topology, solids: &[SolidId]) -> (Topology, Vec<SolidId>) {
    let step = remus_io::step::write_step(topo, solids).unwrap();
    assert!(!step.is_empty(), "STEP export must produce content");
    let mut reread = Topology::new();
    let solids = remus_io::step::read_step(&step, &mut reread).unwrap();
    (reread, solids)
}

// ---------------------------------------------------------------------------
// T2: concave notch fillet removal restores the sharp notch
// ---------------------------------------------------------------------------
// (Convex plane-plane removal is covered by existing tests —
// `regress_defeature_heals_or_refuses::plane_plane_fillet_removal_restores_the_sharp_edge`
// and `research_blend_edit_probe::e1_plane_plane_fillet_structure` (E6) — so
// this corpus executes the concave side here and the convex side on a
// STEP-reimported band in T12.)

/// Extruded V-notch block; the ridge at x = 5 is a concave (reflex) edge.
/// Construction mirrors `regress_fillet_concave_notch` (same-repo fixture).
fn notched_block(topo: &mut Topology) -> (SolidId, EdgeId) {
    let profile = make_polygon_wire(
        topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(5.0, -1.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(10.0, -3.0, 0.0),
            Point3::new(0.0, -3.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    let solid = extrude(topo, face, Vec3::new(0.0, 0.0, 1.0), 8.0).unwrap();
    let ridge = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&eid| {
            let (a, b) = (
                vertex_point(topo, eid, true),
                vertex_point(topo, eid, false),
            );
            (a.x() - 5.0).abs() < 1e-9 && (b.x() - 5.0).abs() < 1e-9 && (a.z() - b.z()).abs() > 1.0
        })
        .expect("T2: ridge edge");
    (solid, ridge)
}

#[test]
fn concave_notch_unfillet_restores_sharp_notch() {
    let mut topo = Topology::new();
    let (solid, ridge) = notched_block(&mut topo);
    assert_valid(&topo, solid, "T2 baseline");
    let v0 = volume(&topo, solid);
    let faces0 = face_count(&topo, solid);

    let filleted = fillet_v2(&mut topo, solid, &[ridge], 0.02).unwrap().solid;
    assert_valid(&topo, filleted, "T2 filleted baseline");
    // A concave fillet ADDS the sliver: the filleted body is larger.
    assert!(
        volume(&topo, filleted) > v0,
        "T2: concave fillet must add material"
    );
    let bands = cylinder_bands(&topo, filleted, 0.02);
    assert_eq!(bands.len(), 1, "T2: exactly one exact cylinder band");

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_valid(&topo, healed, "T2 healed");
    assert_volume_near(volume(&topo, healed), v0, "T2 healed");
    assert_eq!(
        face_count(&topo, healed),
        faces0,
        "T2: healed face count matches the sharp notch"
    );
    assert!(
        cylinder_bands(&topo, healed, 0.02).is_empty(),
        "T2: no blend carrier may survive"
    );
}

// ---------------------------------------------------------------------------
// T3: inside-corner (wall base) fillet ending against planar faces
// ---------------------------------------------------------------------------

#[test]
fn wall_base_inside_corner_unfillet_restores_l_blank() {
    let mut topo = Topology::new();
    let base = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let wall = make_box(&mut topo, 40.0, 8.0, 32.0).unwrap();
    transform_solid(&mut topo, wall, &Mat4::translation(0.0, 32.0, 10.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, base, wall).unwrap();
    assert_valid(&topo, fused, "T3 baseline");
    // Exact L volume: base + wall, no overlap.
    assert_volume_near(volume(&topo, fused), 16_000.0 + 10_240.0, "T3 baseline");
    let v0 = volume(&topo, fused);
    let faces0 = face_count(&topo, fused);

    // Concave inside corner along X at y = 32, z = 10.
    let inside: Vec<EdgeId> = straight_edges_where(&topo, fused, |a, b| {
        (a.y() - 32.0).abs() < 1e-9
            && (b.y() - 32.0).abs() < 1e-9
            && (a.z() - 10.0).abs() < 1e-9
            && (b.z() - 10.0).abs() < 1e-9
            && (a.x() - b.x()).abs() > 1.0
    });
    assert_eq!(inside.len(), 1, "T3: one inside corner edge");

    let filleted = fillet_v2(&mut topo, fused, &inside, 2.0).unwrap().solid;
    assert_valid(&topo, filleted, "T3 filleted baseline");
    assert!(
        volume(&topo, filleted) > v0,
        "T3: concave corner fillet must add material"
    );
    let bands = cylinder_bands(&topo, filleted, 2.0);
    assert_eq!(bands.len(), 1, "T3: exactly one exact cylinder band");

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_valid(&topo, healed, "T3 healed");
    assert_volume_near(volume(&topo, healed), v0, "T3 healed");
    assert_eq!(
        face_count(&topo, healed),
        faces0,
        "T3: healed face count matches the sharp L"
    );
}

// ---------------------------------------------------------------------------
// T4: open band ending against a curved notch — refused at creation
// ---------------------------------------------------------------------------

#[test]
fn band_ending_against_curved_notch_is_refused_at_creation() {
    // Boundary: an open band ending against a curved face cannot be built
    // natively. The top front edge runs into a bore notch; the longest
    // remaining segment abuts the curved notch wall, where the rolling ball
    // has no room, so creation fails and the input survives untouched.
    // The failure code is deliberately NOT pinned: on tangent-degenerate
    // input this reports a cliff, on clean input a trimming failure, and a
    // future engine may build it. What is pinned is the fail-closed contract
    // (a `Blend` refusal, never a partial solid) and input preservation.
    // Removal coverage for curved-ending open bands therefore waits on
    // imported (STEP) R1s, which other CAD systems can produce natively —
    // see Limitations.
    // A bore breaking the front face (y = 0) and the top front edge's middle.
    // Centered at y = 1 so the mouth in the front face has finite width
    // (half-width √3): a tangent placement (y = 2) would degenerate into a
    // slit plus sliver edges, and any refusal on that input could come from
    // the degeneracy rather than the claimed cliff.
    let mut topo = Topology::new();
    let body = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let drill = make_cylinder(&mut topo, 2.0, 20.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(20.0, 1.0, -5.0)).unwrap();
    let notched = boolean(&mut topo, BooleanOp::Cut, body, drill).unwrap();
    assert_valid(&topo, notched, "T4 baseline");
    assert!(
        !reversed_cylinder_walls(&topo, notched, 2.0).is_empty(),
        "T4 premise: the notch wall must exist as exact cylinders"
    );

    // Top-edge segments along X at y = 0, z = 10: the bore splits the edge.
    // The longest segment runs from the notch tangent (≈20) to the corner.
    let mut longest: Option<EdgeId> = None;
    let mut best = 0.0;
    for e in solid_edges(&topo, notched).unwrap() {
        let edge = topo.edge(e).unwrap();
        if edge.start() == edge.end() || !matches!(edge.curve(), EdgeCurve::Line) {
            continue;
        }
        let (a, b) = (vertex_point(&topo, e, true), vertex_point(&topo, e, false));
        if (a.y()).abs() > 1e-9
            || (b.y()).abs() > 1e-9
            || (a.z() - 10.0).abs() > 1e-9
            || (b.z() - 10.0).abs() > 1e-9
        {
            continue;
        }
        let len = (b - a).length();
        if len > best {
            best = len;
            longest = Some(e);
        }
    }
    let target = longest.expect("T4: a top-edge segment must remain");
    assert!(
        best > 10.0,
        "T4: the surviving segment must span the corner"
    );
    let (start, end) = (
        vertex_point(&topo, target, true),
        vertex_point(&topo, target, false),
    );
    assert!(
        (start.x() - 40.0).abs() < 1e-9 || (end.x() - 40.0).abs() < 1e-9,
        "T4: the segment must reach the box corner at either endpoint"
    );

    let before = snapshot(&topo, notched);
    match fillet_v2(&mut topo, notched, &[target], 1.0) {
        Ok(_) => panic!("T4: a band running into the notch wall must be refused"),
        Err(OperationsError::Blend(_)) => {}
        Err(other) => panic!("T4: expected a Blend refusal, got {other:?}"),
    }
    assert_preserved(&topo, notched, &before, "T4");
}

// ---------------------------------------------------------------------------
// T5: trihedral corner network — region query groups it, removal refuses
// ---------------------------------------------------------------------------

fn trihedral_filleted(topo: &mut Topology) -> SolidId {
    let sharp = make_box(topo, 40.0, 40.0, 40.0).unwrap();
    let vertical: Vec<EdgeId> = straight_edges_where(&*topo, sharp, |a, b| {
        (a.x() - 40.0).abs() < 1e-9
            && (b.x() - 40.0).abs() < 1e-9
            && (a.y() - 40.0).abs() < 1e-9
            && (b.y() - 40.0).abs() < 1e-9
            && (a.z() - b.z()).abs() > 1.0
    });
    assert_eq!(vertical.len(), 1);
    let top: Vec<EdgeId> = straight_edges_where(&*topo, sharp, |a, b| {
        let at_corner = |p: Point3| {
            (p.x() - 40.0).abs() < 1e-9
                && (p.y() - 40.0).abs() < 1e-9
                && (p.z() - 40.0).abs() < 1e-9
        };
        (at_corner(a) || at_corner(b)) && (a.z() - b.z()).abs() < 1e-9
    });
    assert_eq!(top.len(), 2);
    let selected: Vec<EdgeId> = vertical.into_iter().chain(top).collect();
    fillet_v2(topo, sharp, &selected, 3.0).unwrap().solid
}

#[test]
fn trihedral_full_network_reports_region_but_refuses_fail_closed() {
    // KNOWN GAP: connected multi-band networks (3 cylinders + corner sphere)
    // have no defeature path — the analytic-band entry needs a torus seed, and
    // plane extension refuses the network as an ambiguous multi-way corner
    // (pinned below) rather than guessing. The region query already groups
    // the network (the reusable hook); the removal itself must refuse
    // fail-closed until local reconstruction disambiguates network corners.
    // Upgrade to success when it does.
    let mut topo = Topology::new();
    let filleted = trihedral_filleted(&mut topo);
    assert_valid(&topo, filleted, "T5 filleted baseline");

    let bands = cylinder_bands(&topo, filleted, 3.0);
    let corners = sphere_patches(&topo, filleted, 3.0);
    assert_eq!(bands.len(), 3, "T5: three exact cylinder bands");
    assert_eq!(corners.len(), 1, "T5: one exact spherical corner patch");

    let region = blend_region(&topo, filleted, bands[0]).unwrap();
    assert_eq!(
        region.faces.len(),
        4,
        "T5: the region groups 3 bands + corner sphere"
    );

    let mut network = bands;
    network.extend(corners.iter().copied());
    let before = snapshot(&topo, filleted);
    // The wound crosses only planes meeting at a clean corner, yet the three
    // grown faces can close the patch in more than one way, so the heal
    // refuses as ambiguous rather than guessing. That exact ambiguity is the
    // known gap: local reconstruction must disambiguate network corners.
    let reason = assert_typed_refusal(defeature(&mut topo, filleted, &network), "T5");
    assert!(
        reason.contains("ambiguous") || reason.contains("more than one way"),
        "T5: refusal should name the corner ambiguity, got {reason:?}"
    );
    assert_preserved(&topo, filleted, &before, "T5");
}

// ---------------------------------------------------------------------------
// T6: partial network removal always refuses (permanent boundary)
// ---------------------------------------------------------------------------

#[test]
fn partial_corner_band_removal_is_refused_fail_closed() {
    // Permanent: removing one band of a connected network can never heal —
    // the corner sphere would hang off a kept band. Must always refuse.
    let mut topo = Topology::new();
    let filleted = trihedral_filleted(&mut topo);
    assert_valid(&topo, filleted, "T6 filleted baseline");

    let bands = cylinder_bands(&topo, filleted, 3.0);
    assert_eq!(bands.len(), 3);
    let before = snapshot(&topo, filleted);
    // A cylinder seed never reaches the analytic-band path, so this must be
    // the defeature-tagged refusal, not a resize-blend code.
    match defeature(&mut topo, filleted, &bands[0..1]) {
        Ok(_) => panic!("T6: partial network removal must be refused"),
        Err(OperationsError::Unsupported { operation, reason }) => {
            assert_eq!(operation, "defeature");
            assert!(!reason.is_empty());
        }
        Err(other) => panic!("T6: expected Unsupported, got {other:?}"),
    }
    assert_preserved(&topo, filleted, &before, "T6");
}

// ---------------------------------------------------------------------------
// T7: blind-hole floor-rim fillet removal restores the sharp floor
// ---------------------------------------------------------------------------

#[test]
fn blind_hole_floor_fillet_unfillets_to_sharp_floor() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let drill = make_cylinder(&mut topo, 3.0, 6.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(5.0, 5.0, 5.0)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
    assert_valid(&topo, holed, "T7 baseline");
    // Exact drilled volume: block minus the r=3, h=5 bore.
    assert_volume_near(volume(&topo, holed), 1000.0 - PI * 9.0 * 5.0, "T7 baseline");
    let v0 = volume(&topo, holed);
    let faces0 = face_count(&topo, holed);

    // Floor rim: closed circle at z = 5, radius 3.
    let rims: Vec<EdgeId> = solid_edges(&topo, holed)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            edge.start() == edge.end()
                && matches!(
                    edge.curve(),
                    EdgeCurve::Circle(c)
                        if (c.center().z() - 5.0).abs() < 1e-9 && (c.radius() - 3.0).abs() < 1e-9
                )
        })
        .collect();
    assert_eq!(rims.len(), 1, "T7: one floor rim");

    let filleted = fillet_v2(&mut topo, holed, &rims, 1.0).unwrap().solid;
    assert_valid(&topo, filleted, "T7 filleted baseline");
    let bands = torus_bands(&topo, filleted, 1.0);
    assert_eq!(bands.len(), 1, "T7: exactly one exact torus band");

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_valid(&topo, healed, "T7 healed");
    assert_volume_near(volume(&topo, healed), v0, "T7 healed");
    assert_eq!(
        face_count(&topo, healed),
        faces0,
        "T7: healed face count matches the sharp blind hole"
    );
    assert!(
        torus_bands(&topo, healed, 1.0).is_empty(),
        "T7: no blend carrier may survive"
    );
    assert_eq!(
        reversed_cylinder_walls(&topo, healed, 3.0).len(),
        1,
        "T7: the bore wall survives as one exact cylinder"
    );
    has_closed_rim_at(&topo, healed, 5.0, 5.0, 5.0, 3.0, "T7 floor rim");
}

// ---------------------------------------------------------------------------
// T8: boss-base fillet removal restores the sharp rim
// ---------------------------------------------------------------------------

#[test]
fn boss_base_fillet_unfillets_to_sharp_rim() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 80.0, 40.0, 8.0).unwrap();
    let boss = make_cylinder(&mut topo, 10.0, 32.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(40.0, 20.0, 8.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, plate, boss).unwrap();
    assert_valid(&topo, fused, "T8 baseline");
    assert_volume_near(
        volume(&topo, fused),
        80.0 * 40.0 * 8.0 + PI * 100.0 * 32.0,
        "T8 baseline",
    );
    let v0 = volume(&topo, fused);
    let faces0 = face_count(&topo, fused);

    // Base rim: closed circle at z = 8, radius 10.
    let rims: Vec<EdgeId> = solid_edges(&topo, fused)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            edge.start() == edge.end()
                && matches!(
                    edge.curve(),
                    EdgeCurve::Circle(c)
                        if (c.center().z() - 8.0).abs() < 1e-9 && (c.radius() - 10.0).abs() < 1e-9
                )
        })
        .collect();
    assert_eq!(rims.len(), 1, "T8: one base rim");

    let filleted = fillet_v2(&mut topo, fused, &rims, 2.0).unwrap().solid;
    assert_valid(&topo, filleted, "T8 filleted baseline");
    let bands = torus_bands(&topo, filleted, 2.0);
    assert_eq!(bands.len(), 1, "T8: exactly one exact torus band");
    // Torus frame: major = post + fillet radius, center coaxial with the post.
    let band_surface = topo.face(bands[0]).unwrap().surface().clone();
    match band_surface {
        FaceSurface::Torus(t) => {
            assert!((t.major_radius() - 12.0).abs() < 1e-9, "T8: major radius");
            assert!((t.minor_radius() - 2.0).abs() < 1e-9, "T8: minor radius");
            let c = t.center();
            assert!(
                (c.x() - 40.0).abs() < 1e-9
                    && (c.y() - 20.0).abs() < 1e-9
                    && (c.z() - 10.0).abs() < 1e-9,
                "T8: torus center coaxial with the post, got ({:.6}, {:.6}, {:.6})",
                c.x(),
                c.y(),
                c.z()
            );
            assert!(
                t.z_axis().dot(Vec3::new(0.0, 0.0, 1.0)).abs() > 1.0 - 1e-12,
                "T8: torus axis is the post axis"
            );
        }
        other => panic!("T8: expected a torus band, got {other:?}"),
    }

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_valid(&topo, healed, "T8 healed");
    assert_volume_near(volume(&topo, healed), v0, "T8 healed");
    assert_eq!(
        face_count(&topo, healed),
        faces0,
        "T8: healed face count matches the sharp boss"
    );
    assert_eq!(
        protrusion_cylinder_walls(&topo, healed, 10.0).len(),
        1,
        "T8: the post wall survives as one exact cylinder"
    );
    has_closed_rim_at(&topo, healed, 40.0, 20.0, 8.0, 10.0, "T8 base rim");
}

// ---------------------------------------------------------------------------
// T9: hole-rim fillet removal crosses the bore seam cleanly
// ---------------------------------------------------------------------------

#[test]
fn hole_top_rim_unfillet_crosses_bore_seam_cleanly() {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 80.0, 40.0, 8.0).unwrap();
    let drill = make_cylinder(&mut topo, 10.0, 16.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(40.0, 20.0, -4.0)).unwrap();
    let cut = boolean(&mut topo, BooleanOp::Cut, plate, drill).unwrap();
    assert_valid(&topo, cut, "T9 baseline");
    assert_volume_near(
        volume(&topo, cut),
        80.0 * 40.0 * 8.0 - PI * 100.0 * 8.0,
        "T9 baseline",
    );
    let v0 = volume(&topo, cut);
    let faces0 = face_count(&topo, cut);

    // Top rim of the bore: closed circle at z = 8, radius 10.
    let rims: Vec<EdgeId> = solid_edges(&topo, cut)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            edge.start() == edge.end()
                && matches!(
                    edge.curve(),
                    EdgeCurve::Circle(c)
                        if (c.center().z() - 8.0).abs() < 1e-9 && (c.radius() - 10.0).abs() < 1e-9
                )
        })
        .collect();
    assert_eq!(rims.len(), 1, "T9: one bore top rim");

    let filleted = fillet_v2(&mut topo, cut, &rims, 2.0).unwrap().solid;
    assert_valid(&topo, filleted, "T9 filleted baseline");
    let bands = torus_bands(&topo, filleted, 2.0);
    assert_eq!(bands.len(), 1, "T9: exactly one exact torus band");

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_valid(&topo, healed, "T9 healed");
    assert_volume_near(volume(&topo, healed), v0, "T9 healed");
    assert_eq!(
        face_count(&topo, healed),
        faces0,
        "T9: healed face count matches the sharp drilled plate"
    );
    // The bore wall crosses its periodic seam: it must come back as ONE exact
    // cylinder, not NURBS patches, bounded by TWO exact closed rims.
    assert_eq!(
        reversed_cylinder_walls(&topo, healed, 10.0).len(),
        1,
        "T9: bore wall is one exact cylinder across the seam"
    );
    has_closed_rim_at(&topo, healed, 40.0, 20.0, 8.0, 10.0, "T9 top rim");
    has_closed_rim_at(&topo, healed, 40.0, 20.0, 0.0, 10.0, "T9 bottom rim");
}

// ---------------------------------------------------------------------------
// T10: hollow solid (cavity shell) — refused by declaration, cavity kept
// ---------------------------------------------------------------------------

#[test]
fn outer_band_on_hollow_solid_is_refused_cavity_kept() {
    // Delta over `qualify_defeature::cavity_solid_refused_typed` (which refuses
    // a plain wall removal on a hollow box): here the hollow solid is filleted
    // first, proving fillet creation preserves the cavity shell, and the
    // refusal still names the cavity with the band present.
    let mut topo = Topology::new();
    let outer = make_box(&mut topo, 30.0, 30.0, 10.0).unwrap();
    let void = make_box(&mut topo, 10.0, 10.0, 4.0).unwrap();
    transform_solid(&mut topo, void, &Mat4::translation(10.0, 10.0, 3.0)).unwrap();
    let hollow = boolean(&mut topo, BooleanOp::Cut, outer, void).unwrap();
    assert_valid(&topo, hollow, "T10 baseline");
    assert_eq!(
        topo.solid(hollow).unwrap().inner_shells().len(),
        1,
        "T10: one cavity shell"
    );

    let edges = straight_edges_where(&topo, hollow, |a, b| {
        (a.z() - 10.0).abs() < 1e-9
            && (b.z() - 10.0).abs() < 1e-9
            && (a.y()).abs() < 1e-9
            && (b.y()).abs() < 1e-9
            && (a.x() - b.x()).abs() > 1.0
    });
    assert_eq!(edges.len(), 1, "T10: one outer top edge");
    let filleted = fillet_v2(&mut topo, hollow, &edges, 1.0).unwrap().solid;
    assert_valid(&topo, filleted, "T10 filleted baseline");
    let bands = cylinder_bands(&topo, filleted, 1.0);
    assert_eq!(bands.len(), 1, "T10: one exact cylinder band");

    let before = snapshot(&topo, filleted);
    let code = assert_typed_refusal(defeature(&mut topo, filleted, &bands), "T10");
    assert_preserved(&topo, filleted, &before, "T10");
    assert!(
        code.contains("cavity"),
        "T10: refusal should name the cavity shells, got {code:?}"
    );
}

// ---------------------------------------------------------------------------
// T11: unrelated bores and boss survive a far-away torus unfillet
// ---------------------------------------------------------------------------

#[test]
fn unfillet_preserves_unrelated_bores_and_boss() {
    // Structural note: plane-extension heals require EVERY kept face to be
    // planar (`heal_by_extending`), so an extend-path removal on a solid with
    // unrelated curved geometry refuses by design. Survival edits therefore go
    // through the analytic-band path: a torus unfillet here must leave two far
    // bores and a far boss byte-identical in type and size.
    let mut topo = Topology::new();
    let mut body = make_box(&mut topo, 80.0, 60.0, 6.0).unwrap();
    for &(x, y) in &[(10.0, 10.0), (70.0, 50.0)] {
        let drill = make_cylinder(&mut topo, 2.25, 10.0).unwrap();
        transform_solid(&mut topo, drill, &Mat4::translation(x, y, -2.0)).unwrap();
        body = boolean(&mut topo, BooleanOp::Cut, body, drill).unwrap();
    }
    // Near post (filleted at its base) + far boss (must survive untouched).
    let post = make_cylinder(&mut topo, 10.0, 32.0).unwrap();
    transform_solid(&mut topo, post, &Mat4::translation(40.0, 30.0, 6.0)).unwrap();
    body = boolean(&mut topo, BooleanOp::Fuse, body, post).unwrap();
    let far_boss = make_cylinder(&mut topo, 4.0, 8.0).unwrap();
    transform_solid(&mut topo, far_boss, &Mat4::translation(70.0, 10.0, 6.0)).unwrap();
    body = boolean(&mut topo, BooleanOp::Fuse, body, far_boss).unwrap();
    assert_valid(&topo, body, "T11 baseline");
    let analytic =
        80.0 * 60.0 * 6.0 - 2.0 * PI * 2.25 * 2.25 * 6.0 + PI * 100.0 * 32.0 + PI * 16.0 * 8.0;
    assert_volume_near(volume(&topo, body), analytic, "T11 baseline");
    let v0 = volume(&topo, body);
    let faces0 = face_count(&topo, body);

    // Post base rim: closed circle at z = 6, radius 10. Every other feature
    // clears the r = 2 band by more than 20 mm.
    let rims: Vec<EdgeId> = solid_edges(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            edge.start() == edge.end()
                && matches!(
                    edge.curve(),
                    EdgeCurve::Circle(c)
                        if (c.center().z() - 6.0).abs() < 1e-9 && (c.radius() - 10.0).abs() < 1e-9
                )
        })
        .collect();
    assert_eq!(rims.len(), 1, "T11: one post base rim");
    let filleted = fillet_v2(&mut topo, body, &rims, 2.0).unwrap().solid;
    assert_valid(&topo, filleted, "T11 filleted baseline");
    let bands = torus_bands(&topo, filleted, 2.0);
    assert_eq!(bands.len(), 1, "T11: one exact torus band");

    let healed = defeature(&mut topo, filleted, &bands).unwrap();
    assert_valid(&topo, healed, "T11 healed");
    assert_volume_near(volume(&topo, healed), v0, "T11 healed");
    assert_eq!(
        face_count(&topo, healed),
        faces0,
        "T11: healed face count matches the pre-fillet body"
    );
    // The edited post wall comes back exact...
    assert_eq!(
        protrusion_cylinder_walls(&topo, healed, 10.0).len(),
        1,
        "T11: the post wall survives as one exact cylinder"
    );
    has_closed_rim_at(&topo, healed, 40.0, 30.0, 6.0, 10.0, "T11 post base rim");
    // ...and so does everything far away: both bore walls and the far boss
    // wall survive as exact cylinders...
    assert_eq!(
        reversed_cylinder_walls(&topo, healed, 2.25).len(),
        2,
        "T11: both bore walls survive"
    );
    assert_eq!(
        protrusion_cylinder_walls(&topo, healed, 4.0).len(),
        1,
        "T11: the far boss wall survives"
    );
    // ...with exact closed rims on both caps (bores) and both boss tops...
    for &(x, y) in &[(10.0, 10.0), (70.0, 50.0)] {
        has_closed_rim_at(&topo, healed, x, y, 6.0, 2.25, "T11 bore top rim");
        has_closed_rim_at(&topo, healed, x, y, 0.0, 2.25, "T11 bore bottom rim");
    }
    has_closed_rim_at(&topo, healed, 70.0, 10.0, 14.0, 4.0, "T11 far boss top rim");
    // ...and the holes are still holes.
    let opts = ClassifyOptions::default();
    for &(x, y) in &[(10.0, 10.0), (70.0, 50.0)] {
        assert_eq!(
            classify_point(&topo, healed, Point3::new(x, y, 3.0), &opts).unwrap(),
            PointClassification::Outside,
            "T11: bore at ({x}, {y}) must survive"
        );
    }
    assert_eq!(
        classify_point(&topo, healed, Point3::new(20.0, 45.0, 3.0), &opts).unwrap(),
        PointClassification::Inside,
        "T11: plate material between features is still solid"
    );
}

// ---------------------------------------------------------------------------
// T12: STEP-reimported bands unfillet to sharp bodies (no files committed)
// ---------------------------------------------------------------------------

#[test]
fn step_reimported_bands_unfillet_to_sharp_bodies() {
    // The imported-R1 analogue: build two band classes natively, pass them
    // through STEP export/reimport (in memory — nothing committed), then
    // remove the reimported bands. This is the exact workflow the active
    // fillet-removal agent exercises on foreign R1s.
    let mut topo = Topology::new();
    // Convex cylinder band body.
    let sharp = make_box(&mut topo, 24.0, 16.0, 8.0).unwrap();
    let edge = straight_edges_where(&topo, sharp, |a, b| {
        (a.y()).abs() < 1e-9
            && (b.y()).abs() < 1e-9
            && (a.z() - 8.0).abs() < 1e-9
            && (b.z() - 8.0).abs() < 1e-9
            && (a.x() - b.x()).abs() > 1.0
    });
    assert_eq!(edge.len(), 1, "T12: one target top edge");
    let convex = fillet_v2(&mut topo, sharp, &edge, 2.0).unwrap().solid;

    // Torus boss-base band body.
    let plate = make_box(&mut topo, 80.0, 40.0, 8.0).unwrap();
    let boss = make_cylinder(&mut topo, 10.0, 32.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(40.0, 20.0, 8.0)).unwrap();
    let fused = boolean(&mut topo, BooleanOp::Fuse, plate, boss).unwrap();
    let rim: Vec<EdgeId> = solid_edges(&topo, fused)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            edge.start() == edge.end()
                && matches!(
                    edge.curve(),
                    EdgeCurve::Circle(c)
                        if (c.center().z() - 8.0).abs() < 1e-9 && (c.radius() - 10.0).abs() < 1e-9
                )
        })
        .collect();
    assert_eq!(rim.len(), 1, "T12: one base rim");
    let toroidal = fillet_v2(&mut topo, fused, &rim, 2.0).unwrap().solid;

    for (body, what) in [(convex, "T12 convex"), (toroidal, "T12 toroidal")] {
        assert_valid(&topo, body, &format!("{what} baseline"));
    }
    let v_convex = volume(&topo, convex);
    let v_toroidal = volume(&topo, toroidal);

    // Export both, reimport, match bodies back by volume (never by position —
    // export order is not part of the contract).
    let (mut reread, bodies) = roundtrip(&topo, &[convex, toroidal]);
    assert_eq!(bodies.len(), 2, "T12: both bodies reimport");
    let pick = |v: f64| {
        bodies
            .iter()
            .copied()
            .find(|&s| (volume(&reread, s) - v).abs() <= 1e-7 * v.abs().max(1.0))
            .expect("T12: reimported body matches its pre-export volume")
    };
    let (convex2, toroidal2) = (pick(v_convex), pick(v_toroidal));
    assert_ne!(convex2, toroidal2, "T12: the two bodies are distinct");
    for (body, what) in [(convex2, "T12 convex"), (toroidal2, "T12 toroidal")] {
        assert_valid(&reread, body, &format!("{what} reimported"));
    }
    // Analytic carriers survive with exact radii and exact closed rims — a
    // Circle degraded to NURBS would fail the closed-rim counts, not just a
    // totals comparison.
    assert_eq!(
        cylinder_bands(&reread, convex2, 2.0).len(),
        1,
        "T12: convex cylinder band survives as an exact cylinder"
    );
    let tbands = torus_bands(&reread, toroidal2, 2.0);
    assert_eq!(tbands.len(), 1, "T12: torus band survives");
    match reread.face(tbands[0]).unwrap().surface() {
        FaceSurface::Torus(t) => {
            assert!((t.major_radius() - 12.0).abs() < 1e-9, "T12: major radius");
            assert!((t.minor_radius() - 2.0).abs() < 1e-9, "T12: minor radius");
        }
        other => panic!("T12: expected a torus band, got {other:?}"),
    }
    has_closed_rim_at(
        &reread,
        toroidal2,
        40.0,
        20.0,
        8.0,
        12.0,
        "T12 reimported plate spring",
    );
    has_closed_rim_at(
        &reread,
        toroidal2,
        40.0,
        20.0,
        10.0,
        10.0,
        "T12 reimported wall spring",
    );

    // ...and both reimported bands remove exactly, restoring sharp bodies.
    let convex_bands = cylinder_bands(&reread, convex2, 2.0);
    let healed_convex = defeature(&mut reread, convex2, &convex_bands).unwrap();
    assert_valid(&reread, healed_convex, "T12 healed convex");
    assert_volume_near(volume(&reread, healed_convex), 3072.0, "T12 healed convex");
    assert_eq!(
        face_count(&reread, healed_convex),
        6,
        "T12: reimported convex band restores the sharp box"
    );

    let healed_toroidal = defeature(&mut reread, toroidal2, &tbands).unwrap();
    assert_valid(&reread, healed_toroidal, "T12 healed toroidal");
    // Sharp reference: plate + post, no fillet.
    assert_volume_near(
        volume(&reread, healed_toroidal),
        80.0 * 40.0 * 8.0 + PI * 100.0 * 32.0,
        "T12 healed toroidal",
    );
    assert_eq!(
        face_count(&reread, healed_toroidal),
        8,
        "T12: reimported torus band restores the sharp boss"
    );
    assert_eq!(
        protrusion_cylinder_walls(&reread, healed_toroidal, 10.0).len(),
        1,
        "T12: reimported post wall is one exact cylinder"
    );
    has_closed_rim_at(
        &reread,
        healed_toroidal,
        40.0,
        20.0,
        8.0,
        10.0,
        "T12 restored base rim",
    );
}
