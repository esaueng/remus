//! Regression: a straight-edge fillet whose contact line runs into a hole in
//! one of its own planar neighbours (kernel review 2026-10-14, BLEND-2 and
//! BLEND-3).
//!
//! A cap rebuilt with its outer wire across its own hole loop still closes,
//! passes Euler, and validates, so every result here is checked loop by loop.
//!
//! - BLEND-2: `fillet_cascade` re-ran the rolling-ball rebuild after
//!   `fillet_v2` refused a plate-hole fillet, without the hole guard that made
//!   `fillet_v2` refuse, and returned the crossing cap.
//! - BLEND-3: the hole guard compared the clearance with the radius `r`, but
//!   the contact line sits `r / tan(α/2)` from an edge of dihedral `α`. On a
//!   60° wedge that is `r·√3`, so a hole between `r` and `r·√3` from the edge
//!   was crossed without complaint.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_ops::{self, BlendError};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::extrude::extrude;
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::wire::WireId;

const HOLE_R: f64 = 1.0;

fn line_edge(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            let p = topo.vertex(edge.start()).unwrap().point();
            let q = topo.vertex(edge.end()).unwrap().point();
            ((p - a).length() < 1e-9 && (q - b).length() < 1e-9)
                || ((p - b).length() < 1e-9 && (q - a).length() < 1e-9)
        })
        .expect("the fixture has this straight edge")
}

/// A loop as an ordered polyline, projected onto the two axes its face's
/// plane normal is least aligned with.
fn loop_polygon(topo: &Topology, wire: WireId, drop_axis: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    for oriented in topo.wire(wire).unwrap().edges() {
        let edge = topo.edge(oriented.edge()).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let (t0, t1) = edge.strict_domain().expect("stored edge domain");
        let mut points: Vec<Point3> = (0..64)
            .map(|k| {
                let t = (t1 - t0).mul_add(f64::from(k) / 64.0, t0);
                edge.curve().evaluate_with_endpoints(t, start, end)
            })
            .collect();
        if !oriented.is_forward() {
            points.push(end);
            points.remove(0);
            points.reverse();
        }
        out.extend(points.into_iter().map(|p| {
            let c = [p.x(), p.y(), p.z()];
            match drop_axis {
                0 => (c[1], c[2]),
                1 => (c[0], c[2]),
                _ => (c[0], c[1]),
            }
        }));
    }
    out
}

/// Signed distance from `p` to the boundary of `poly`: positive inside.
fn signed_depth(poly: &[(f64, f64)], p: (f64, f64)) -> f64 {
    let mut inside = false;
    let mut nearest = f64::INFINITY;
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let len2 = dx.mul_add(dx, dy * dy);
        let t = if len2 > 0.0 {
            (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        nearest = nearest.min((p.0 - (a.0 + t * dx)).hypot(p.1 - (a.1 + t * dy)));
    }
    if inside { nearest } else { -nearest }
}

/// Every hole loop of every planar face lies strictly inside its outer loop.
fn assert_no_loop_crosses_its_outer_wire(topo: &Topology, solid: SolidId) {
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            continue;
        };
        let n = [normal.x().abs(), normal.y().abs(), normal.z().abs()];
        let drop_axis = (0..3).max_by(|&i, &j| n[i].total_cmp(&n[j])).unwrap();
        let outer = loop_polygon(topo, face.outer_wire(), drop_axis);
        for &inner in face.inner_wires() {
            for p in loop_polygon(topo, inner, drop_axis) {
                let depth = signed_depth(&outer, p);
                assert!(
                    depth > 1e-6,
                    "face {face_id:?}: hole point {p:?} lies {depth:.4} outside its outer wire"
                );
            }
        }
    }
}

fn assert_cliff(outcome: Result<blend_ops::BlendResult, OperationsError>, available: f64) {
    match outcome {
        Err(OperationsError::Blend(BlendError::CliffEncountered {
            available_radius, ..
        })) => assert!(
            (available_radius - available).abs() < 1e-2,
            "available radius {available_radius}, expected ≈{available}"
        ),
        Err(other) => panic!("expected a cliff refusal, got {other}"),
        Ok(result) => panic!("expected a cliff refusal, got {:?} result", result.engine),
    }
}

/// What a refused call must leave behind: the arena and the input body as
/// they were.
fn arena_fingerprint(
    topo: &Topology,
    solid: SolidId,
) -> (usize, usize, (usize, usize, usize), f64) {
    (
        topo.num_faces(),
        topo.num_loops(),
        solid_entity_counts(topo, solid).unwrap(),
        measure::solid_volume(topo, solid, 0.05).unwrap(),
    )
}

/// 10×10×4 plate, through hole of radius 1 centred 2.75 from the front face:
/// the hole's near side is 1.75 from the top front edge.
fn plate_with_hole(topo: &mut Topology) -> (SolidId, EdgeId) {
    let plate = primitives::make_box(topo, 10.0, 10.0, 4.0).unwrap();
    let drill = primitives::make_cylinder(topo, HOLE_R, 8.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(5.0, 2.75, -2.0)).unwrap();
    let body = boolean(topo, BooleanOp::Cut, plate, drill).unwrap();
    let edge = line_edge(
        topo,
        body,
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(10.0, 0.0, 4.0),
    );
    (body, edge)
}

/// BLEND-2: the cascade must not fall back past the hole guard.
#[test]
fn cascade_refuses_a_plate_fillet_that_reaches_the_hole() {
    let mut topo = Topology::new();
    let (body, edge) = plate_with_hole(&mut topo);

    assert_cliff(
        blend_ops::fillet_v2(&mut topo.clone(), body, &[edge], 2.0),
        1.75,
    );

    let before = arena_fingerprint(&topo, body);
    assert_cliff(
        blend_ops::fillet_cascade(&mut topo, body, &[edge], 2.0),
        1.75,
    );
    assert_eq!(
        arena_fingerprint(&topo, body),
        before,
        "the refused cascade must leave the arena untouched"
    );

    // Below the clearance the same edge still fillets, around the hole.
    let result = blend_ops::fillet_cascade(&mut topo, body, &[edge], 1.5)
        .expect("a fillet that clears the hole is still supported");
    assert_no_loop_crosses_its_outer_wire(&topo, result.solid);
    assert!(measure::solid_volume(&topo, result.solid, 0.05).unwrap() < before.3);
}

/// A plate of height 6, 12 wide, 10 deep, whose top-left edge along y has the
/// interior dihedral `angle_deg`, with a blind hole of radius 1 and depth 2
/// drilled in the top face 2.5 from that edge.
fn wedge_with_hole(topo: &mut Topology, angle_deg: f64) -> (SolidId, EdgeId) {
    const H: f64 = 6.0;
    let foot = H / angle_deg.to_radians().tan();
    let profile = [
        Point3::new(0.0, 0.0, H),
        Point3::new(foot, 0.0, 0.0),
        Point3::new(12.0, 0.0, 0.0),
        Point3::new(12.0, 0.0, H),
    ];
    let wire = make_polygon_wire(topo, &profile, 1e-7).unwrap();
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    let blank = extrude(topo, face, Vec3::new(0.0, 1.0, 0.0), 10.0).unwrap();
    let drill = primitives::make_cylinder(topo, HOLE_R, 3.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(2.5 + HOLE_R, 5.0, H - 2.0)).unwrap();
    let body = boolean(topo, BooleanOp::Cut, blank, drill).unwrap();
    let edge = line_edge(
        topo,
        body,
        Point3::new(0.0, 0.0, H),
        Point3::new(0.0, 10.0, H),
    );
    (body, edge)
}

/// BLEND-3: on a 60° edge an r = 2 fillet touches the top face 2·√3 ≈ 3.46
/// from the edge, past a hole 2.5 away, though r itself is below 2.5.
#[test]
fn sixty_degree_wedge_fillet_reaching_the_hole_is_refused() {
    let mut topo = Topology::new();
    let (body, edge) = wedge_with_hole(&mut topo, 60.0);
    // The largest radius whose setback r·√3 stays inside the clearance.
    let available = 2.5 / 3.0_f64.sqrt();

    assert_cliff(
        blend_ops::fillet_v2(&mut topo.clone(), body, &[edge], 2.0),
        available,
    );
    let before = arena_fingerprint(&topo, body);
    assert_cliff(
        blend_ops::fillet_cascade(&mut topo, body, &[edge], 2.0),
        available,
    );
    assert_eq!(
        arena_fingerprint(&topo, body),
        before,
        "the refused cascade must leave the arena untouched"
    );

    // Setback 1.3·√3 ≈ 2.25 clears the hole.
    let result = blend_ops::fillet_cascade(&mut topo, body, &[edge], 1.3)
        .expect("a 60° fillet whose setback clears the hole is still supported");
    assert_no_loop_crosses_its_outer_wire(&topo, result.solid);
}

/// The same hole clearance at a right angle: the setback is r, and r = 2
/// clears a hole 2.5 away.
#[test]
fn right_angle_fillet_with_the_same_clearance_still_succeeds() {
    let mut topo = Topology::new();
    let (body, edge) = wedge_with_hole(&mut topo, 90.0);
    let before = measure::solid_volume(&topo, body, 0.05).unwrap();
    let result = blend_ops::fillet_cascade(&mut topo, body, &[edge], 2.0)
        .expect("a right-angle fillet whose setback clears the hole is supported");
    assert!(!result.is_partial);
    assert_no_loop_crosses_its_outer_wire(&topo, result.solid);
    let removed = before - measure::solid_volume(&topo, result.solid, 0.05).unwrap();
    // (1 − π/4)·r² over the 10-long edge.
    let want = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0 * 10.0;
    assert!(
        (removed - want).abs() < 0.05 * want,
        "removed {removed}, closed form {want}"
    );
}
