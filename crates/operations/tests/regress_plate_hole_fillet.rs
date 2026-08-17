//! Regression: corner and perimeter fillets on a boolean-result plate.
//!
//! The canonical part is an 80 x 60 x 6 plate with four 4.5 mm holes. On the
//! plain box every fillet class works; on the drilled plate only a single
//! isolated straight edge did, because `fillet_v2`'s planar fast path emitted
//! an OPEN shell for any holed cap and the walking builder it fell back to
//! refuses multi-stripe vertices (`UnsupportedVertexBlend`).
//!
//! The fast path emitted an open shell for a precise reason: every face spec
//! it hands the assembler describes a wire as a list of vertex POSITIONS, and
//! a drilled hole's rim is one closed circle edge — a single position. The
//! assembler dropped every loop shorter than three positions, so the rebuilt
//! cap came back solid while its bore wall kept the rim it no longer shared.
//! Holed caps now travel as topology (`FaceSpec::Existing`) instead.
//!
//! What must NOT happen is the fillet quietly running into a hole: a cap wire
//! that crosses its own inner loop still passes the closed-shell and Euler
//! checks, so it would ship as a self-intersecting body.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::blend_ops;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const W: f64 = 80.0;
const D: f64 = 60.0;
const T: f64 = 6.0;
const HOLE_R: f64 = 2.25;
const INSET: f64 = 10.0;
/// Deflection for the volume comparisons. Fine enough that the drilled plate's
/// bore faceting stays well inside `VOLUME_TOLERANCE`.
///
/// It has to be five times finer than it used to be. The drilled plate is fully
/// analytic before the fillet, so `solid_volume` now integrates it in closed
/// form; the filleted body carries a NURBS vertex-blend patch and is still
/// measured off its mesh. The bores' inscribed-mesh error therefore lands on one
/// end of `before − after` instead of very nearly cancelling across both, and it
/// is that single-sided term — `4·(2/3)·(2πr)·δ·T`, 1.1 mm³ at δ = 0.005 — the
/// deflection has to keep small. The budget below is unchanged.
const VOLUME_DEFLECTION: f64 = 0.001;
/// How far the drilled plate's measured loss may sit from the plain plate's.
///
/// The two bodies lose the same material, so the only difference is
/// quadrature: the drilled plate's four bores are chorded in the filleted
/// result, and the plain plate has no bores to chord. That moves a fixed
/// fraction of the BORES' volume, so the budget is stated against the bores
/// rather than against the fillet.
fn volume_tolerance() -> f64 {
    let bore_volume = 4.0 * std::f64::consts::PI * HOLE_R * HOLE_R * T;
    bore_volume * 2.5e-3
}

/// The screenshot part: plate minus `holes` drilled straight through.
fn plate(topo: &mut Topology, holes: &[(f64, f64)]) -> SolidId {
    let mut body = make_box(topo, W, D, T).expect("plate blank");
    for &(x, y) in holes {
        let drill = make_cylinder(topo, HOLE_R, T + 4.0).expect("drill");
        transform_solid(topo, drill, &Mat4::translation(x, y, -2.0)).expect("place drill");
        body = boolean(topo, BooleanOp::Cut, body, drill).expect("drill hole");
    }
    body
}

fn four_holes() -> Vec<(f64, f64)> {
    vec![
        (INSET, INSET),
        (W - INSET, INSET),
        (INSET, D - INSET),
        (W - INSET, D - INSET),
    ]
}

fn solid_edge_list(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if !seen.contains(&oe.edge()) {
                    seen.push(oe.edge());
                }
            }
        }
    }
    seen
}

/// The straight top edge lying in the plane `x == at` (or `y == at`).
fn top_edge(topo: &Topology, s: SolidId, along_x: bool, at: f64) -> EdgeId {
    solid_edge_list(topo, s)
        .into_iter()
        .find(|&e| {
            let ed = topo.edge(e).unwrap();
            if ed.start() == ed.end() {
                return false;
            }
            let a = topo.vertex(ed.start()).unwrap().point();
            let b = topo.vertex(ed.end()).unwrap().point();
            if (a.z() - T).abs() > 1e-9 || (b.z() - T).abs() > 1e-9 {
                return false;
            }
            if along_x {
                (a.y() - at).abs() < 1e-9 && (b.y() - at).abs() < 1e-9
            } else {
                (a.x() - at).abs() < 1e-9 && (b.x() - at).abs() < 1e-9
            }
        })
        .expect("top edge")
}

/// (free edges, non-manifold edges) in the B-rep.
fn brep_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (
        usage.values().filter(|&&c| c == 1).count(),
        usage.values().filter(|&&c| c >= 3).count(),
    )
}

/// (free edges, non-manifold edges) in the tessellation.
fn mesh_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, s, 0.01, 0.1).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
    )
}

fn assert_watertight(topo: &Topology, s: SolidId, what: &str) {
    let shell_id = topo.solid(s).unwrap().outer_shell();
    let shell = topo.shell(shell_id).unwrap().clone();
    validate_shell_closed(&shell, topo)
        .unwrap_or_else(|e| panic!("{what}: the result shell must be closed, got {e}"));
    assert_eq!(
        brep_edge_health(topo, s),
        (0, 0),
        "{what}: no free or non-manifold B-rep edges"
    );
    assert_eq!(
        mesh_edge_health(topo, s),
        (0, 0),
        "{what}: the tessellation must be watertight"
    );
}

/// Every hole must still be bounded by its two exact circular rims.
///
/// This is the property Phase 1a actually turns on, asserted without a
/// tolerance band: the rebuilt cap must carry the hole's rim through as the
/// same closed `Circle` edge rather than re-minting it as a chord polygon.
/// Point classification alone would not notice a rim silently degraded into a
/// 12-gon — the hole would still read as a hole, just the wrong size — and a
/// volume comparison would absorb it into the quadrature budget.
fn assert_hole_rims_exact(topo: &Topology, s: SolidId, holes: &[(f64, f64)], what: &str) {
    let mut circles: Vec<(Point3, f64)> = Vec::new();
    for e in solid_edge_list(topo, s) {
        let ed = topo.edge(e).unwrap();
        let EdgeCurve::Circle(c) = ed.curve() else {
            continue;
        };
        // A hole rim is a closed edge: one vertex, start == end.
        if ed.start() != ed.end() {
            continue;
        }
        circles.push((c.center(), c.radius()));
    }
    for &(x, y) in holes {
        let rims: Vec<_> = circles
            .iter()
            .filter(|(c, _)| (c.x() - x).abs() < 1e-9 && (c.y() - y).abs() < 1e-9)
            .collect();
        assert_eq!(
            rims.len(),
            2,
            "{what}: the hole at ({x}, {y}) must keep exactly two closed circular \
             rims (top and bottom), found {rims:?}"
        );
        for (c, r) in rims {
            assert!(
                (r - HOLE_R).abs() < 1e-9,
                "{what}: rim at {c:?} has radius {r}, expected {HOLE_R}"
            );
        }
    }
}

/// Every hole must still be a hole, and the material between them still solid.
fn assert_holes_survive(topo: &Topology, s: SolidId, holes: &[(f64, f64)]) {
    let opts = ClassifyOptions::default();
    for &(x, y) in holes {
        assert_eq!(
            classify_point(topo, s, Point3::new(x, y, T / 2.0), &opts).unwrap(),
            PointClassification::Outside,
            "the hole at ({x}, {y}) must survive the fillet"
        );
    }
    assert_eq!(
        classify_point(topo, s, Point3::new(W / 2.0, D / 2.0, T / 2.0), &opts).unwrap(),
        PointClassification::Inside,
        "the plate between the holes is still solid"
    );
}

/// Material the same fillet removes from the UNDRILLED plate.
///
/// Every hole in this fixture sits at least 7.75 mm clear of every filleted
/// edge, so the blend cannot reach one: the drilled plate must lose exactly
/// what the plain plate loses. That is a far sharper statement than any
/// closed-form bound, and it is measured with the same integrator on both
/// sides, so it is not blunted by how precisely the engine's NURBS strips
/// tessellate. The residual gap is the drilled plate's bore faceting, which
/// shrinks with the deflection (0.11 % at 0.02, 0.02 % at 0.001).
fn plain_plate_removal(pick: impl Fn(&Topology, SolidId) -> Vec<EdgeId>, r: f64) -> f64 {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[]);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();
    let edges = pick(&topo, body);
    let result = blend_ops::fillet_v2(&mut topo, body, &edges, r).expect("plain plate fillet");
    before - measure::solid_volume(&topo, result.solid, VOLUME_DEFLECTION).unwrap()
}

/// Baseline: on the plain box every one of these already worked. It must keep
/// working, or a "fix" for the drilled plate has cost the simple case.
#[test]
fn plain_plate_still_fillets_every_class() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[]);
    let cases: Vec<Vec<EdgeId>> = vec![
        vec![top_edge(&topo, body, true, 0.0)],
        vec![
            top_edge(&topo, body, true, 0.0),
            top_edge(&topo, body, false, 0.0),
        ],
        vec![
            top_edge(&topo, body, true, 0.0),
            top_edge(&topo, body, true, D),
            top_edge(&topo, body, false, 0.0),
            top_edge(&topo, body, false, W),
        ],
    ];
    for edges in cases {
        let mut t = topo.clone();
        let n = edges.len();
        let result = blend_ops::fillet_v2(&mut t, body, &edges, 2.0)
            .unwrap_or_else(|e| panic!("plain plate, {n} edge(s): {e}"));
        assert_watertight(&t, result.solid, "plain plate");
    }
}

/// The screenshot case: two adjacent top edges of the drilled plate.
#[test]
fn drilled_plate_fillets_a_corner_pair() {
    let holes = four_holes();
    let mut topo = Topology::new();
    let body = plate(&mut topo, &holes);

    // Guard the premise: without a holed cap this test degrades into
    // re-testing the plain box.
    let holed_caps = solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            face.surface().type_tag() == "plane" && !face.inner_wires().is_empty()
        })
        .count();
    assert_eq!(holed_caps, 2, "both caps must carry the four hole rims");

    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    for r in [2.0_f64, 0.5] {
        let mut t = topo.clone();
        let edges = vec![
            top_edge(&t, body, true, 0.0),
            top_edge(&t, body, false, 0.0),
        ];
        let result = blend_ops::fillet_v2(&mut t, body, &edges, r)
            .unwrap_or_else(|e| panic!("corner pair at r={r}: {e}"));
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        assert_watertight(&t, result.solid, &format!("corner pair r={r}"));
        assert_holes_survive(&t, result.solid, &holes);
        assert_hole_rims_exact(&t, result.solid, &holes, &format!("corner pair r={r}"));

        // Both strips are convex, so material can only leave, and the holes
        // are nowhere near the blend: the loss must match the plain plate's.
        let after = measure::solid_volume(&t, result.solid, VOLUME_DEFLECTION).unwrap();
        let removed = before - after;
        assert!(
            removed > 0.0,
            "a convex fillet must remove material, removed {removed}"
        );
        let want = plain_plate_removal(
            |topo, body| {
                vec![
                    top_edge(topo, body, true, 0.0),
                    top_edge(topo, body, false, 0.0),
                ]
            },
            r,
        );
        assert!(
            (removed - want).abs() < volume_tolerance(),
            "corner pair r={r} removed {removed} from the drilled plate but {want} \
             from the plain one; the holes are 7.75 mm clear of the blend"
        );
    }
}

/// The whole top perimeter of the drilled plate, all four edges at once.
#[test]
fn drilled_plate_fillets_the_top_perimeter() {
    let holes = four_holes();
    let mut topo = Topology::new();
    let body = plate(&mut topo, &holes);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    let r = 2.0;
    let edges = vec![
        top_edge(&topo, body, true, 0.0),
        top_edge(&topo, body, true, D),
        top_edge(&topo, body, false, 0.0),
        top_edge(&topo, body, false, W),
    ];
    let result = blend_ops::fillet_v2(&mut topo, body, &edges, r).expect("top perimeter fillet");
    assert!(result.failed.is_empty(), "{:?}", result.failed);
    assert_watertight(&topo, result.solid, "top perimeter");
    assert_holes_survive(&topo, result.solid, &holes);
    assert_hole_rims_exact(&topo, result.solid, &holes, "top perimeter");

    let after = measure::solid_volume(&topo, result.solid, VOLUME_DEFLECTION).unwrap();
    let removed = before - after;
    let want = plain_plate_removal(
        |topo, body| {
            vec![
                top_edge(topo, body, true, 0.0),
                top_edge(topo, body, true, D),
                top_edge(topo, body, false, 0.0),
                top_edge(topo, body, false, W),
            ]
        },
        r,
    );
    assert!(
        (removed - want).abs() < volume_tolerance(),
        "top perimeter removed {removed} from the drilled plate but {want} from \
         the plain one"
    );
}

/// A second fillet on an already-filleted drilled plate — the exact sequence
/// from the bug report, where 17 of 18 edges refused.
#[test]
fn drilled_plate_accepts_a_second_fillet() {
    let holes = four_holes();
    let mut topo = Topology::new();
    let body = plate(&mut topo, &holes);

    let first_edge = top_edge(&topo, body, true, 0.0);
    let first = blend_ops::fillet_v2(&mut topo, body, &[first_edge], 2.0).expect("first fillet");
    assert_watertight(&topo, first.solid, "first fillet");

    // The opposite long edge is untouched by the first fillet and must round.
    let second_edge = top_edge(&topo, first.solid, true, D);
    let second = blend_ops::fillet_v2(&mut topo, first.solid, &[second_edge], 2.0)
        .expect("second fillet on the already-filleted plate");
    assert_watertight(&topo, second.solid, "second fillet");
    assert_holes_survive(&topo, second.solid, &holes);
    assert_hole_rims_exact(&topo, second.solid, &holes, "second fillet");
}

/// A radius large enough to reach a hole must be refused by radius, and the
/// input must come back untouched.
///
/// The hole centre sits 4 mm from the edge with a 2.25 mm radius, so the rim
/// clears the edge by 1.75 mm. At r = 2 the cap's rebuilt boundary would cross
/// it. The failure is genuinely about the radius — r = 1 rounds the same edge
/// fine — so it must report `RadiusTooLarge`, not a trimming failure and
/// certainly not a closed-but-self-intersecting solid.
#[test]
fn fillet_reaching_a_hole_is_refused_by_radius() {
    let mut topo = Topology::new();
    let body = plate(&mut topo, &[(W / 2.0, 4.0)]);
    let edge = top_edge(&topo, body, true, 0.0);
    let before = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();

    let outcome = blend_ops::fillet_v2(&mut topo, body, &[edge], 2.0);
    let err = outcome
        .err()
        .expect("a radius that reaches the hole must fail");
    assert_eq!(
        blend_ops::blend_failure_code(&err),
        "radius-too-large",
        "the cause is the radius, not the topology: {err}"
    );

    let after = measure::solid_volume(&topo, body, VOLUME_DEFLECTION).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "the refused fillet must leave the input untouched"
    );

    // And the same edge still rounds below the clearance.
    let ok = blend_ops::fillet_v2(&mut topo, body, &[edge], 1.0)
        .expect("a radius inside the clearance must still work");
    assert_watertight(&topo, ok.solid, "sub-clearance fillet");
}
