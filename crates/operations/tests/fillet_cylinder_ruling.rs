//! Fillets along a cylinder ruling: the straight edge where a cylinder meets a
//! plane that runs along its axis.
//!
//! Two shapes produce that edge. A round boss standing against a block meets
//! the block's wall in a CONCAVE line (the fillet adds material in the open
//! corner); a flat ground onto a shaft meets the shaft in a CONVEX one. Both
//! used to fall to the walking engine, which placed the concave ball inside
//! the material and approximated both bands as NURBS. The blend is an exact
//! cylinder, and these tests hold the result to closed-form areas and
//! volumes rather than to the kernel's own measurements of itself.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{PI, TAU};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{blend_failure_code, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const MEASURE_DEFLECTION: f64 = 0.005;

fn edge_between(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-6 && (end - b).length() < 1e-6)
                || ((start - b).length() < 1e-6 && (end - a).length() < 1e-6)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

fn rotation_about_z(degrees: f64) -> Mat4 {
    let (s, c) = degrees.to_radians().sin_cos();
    Mat4([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

/// The faces the fillet created whose surface is a cylinder of radius
/// `radius`: the band.
fn bands(topo: &Topology, input: SolidId, output: SolidId, radius: f64) -> Vec<FaceId> {
    let before = solid_faces(topo, input).unwrap();
    solid_faces(topo, output)
        .unwrap()
        .into_iter()
        .filter(|face| !before.contains(face))
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(c) if (c.radius() - radius).abs() < 1e-12
            )
        })
        .collect()
}

fn assert_valid_closed(topo: &Topology, solid: SolidId, label: &str) {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    assert!(
        validate_shell_closed(shell, topo).is_ok(),
        "{label}: open shell"
    );
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
}

/// `∮ x dy − y dx` along a circle arc of radius `rad` about `(cx, cy)` from
/// angle `t0` to `t1`.
fn arc_moment(cx: f64, cy: f64, rad: f64, t0: f64, t1: f64) -> f64 {
    rad * rad * (t1 - t0) + rad * (cx * (t1.sin() - t0.sin()) - cy * (t1.cos() - t0.cos()))
}

/// Area of the fillet's cross-section where a wall (the half-plane `y = 0`,
/// `x >= big`) meets a cylinder of radius `big` about the z axis, filled by
/// a ball of radius `r` in the open corner (`y < 0`).
///
/// Green's theorem around `S → P → Q → S`: along the wall from the ruling
/// `S = (big, 0)` to the wall contact `P` (no moment on `y = 0`), round the
/// ball from `P` to the cylinder contact `Q`, and back along the cylinder.
fn concave_section_area(big: f64, r: f64) -> f64 {
    let xc = ((big + r).powi(2) - r * r).sqrt();
    // About the ball centre (xc, −r): P is straight up, Q faces the axis.
    let ball = arc_moment(xc, -r, r, PI / 2.0, r.atan2(-xc));
    // About the axis: from Q's angle back up to S at angle 0.
    let cylinder = arc_moment(0.0, 0.0, big, (-r).atan2(xc), 0.0);
    0.5 * (ball + cylinder).abs()
}

/// A round boss standing against a block, filleted where its side meets the
/// block's wall. The body gains exactly the filled corner and the band is an
/// exact cylinder, wherever the cylinder's parameter seam sits and whether
/// the boss ends flush with the block or rises past it.
///
/// Rising past it, the fillet stops where the block's top face crosses the
/// boss, so the boss is trimmed by an L — the contact ruling, then the arc of
/// the boss in that face — rather than by one curve between two of its edges.
/// The mirrored body walks the boss's wire the other way round that L; the
/// upside-down one puts the open end at the bottom.
#[test]
fn concave_line_where_a_boss_meets_a_wall() {
    let (big, r, height) = (6.0, 1.0, 24.0);
    let section = concave_section_area(big, r);
    // Independent bounds: a flat wall would fill (1 − π/4)·r²; the convex
    // cylinder leaves strictly less corner to fill.
    assert!(
        section > 0.0 && section < (1.0 - PI / 4.0) * r * r,
        "{section}"
    );

    let identity = rotation_about_z(0.0);
    let mirrored = Mat4([
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let upside_down = Mat4([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let cases = [
        ("flush", height, identity),
        ("taller", height + 4.0, identity),
        ("taller, mirrored", height + 4.0, mirrored),
        ("taller, upside down", height + 4.0, upside_down),
    ];
    for (shape, boss_height, placement) in cases {
        for seam_degrees in [0.0, -30.0, 180.0] {
            let label = format!("{shape}, seam at {seam_degrees} degrees");
            let mut topo = Topology::new();
            let boss = make_cylinder(&mut topo, big, boss_height).unwrap();
            transform_solid(&mut topo, boss, &rotation_about_z(seam_degrees)).unwrap();
            let block = make_box(&mut topo, 30.0, 18.0, height).unwrap();
            let body = boolean(&mut topo, BooleanOp::Fuse, boss, block)
                .unwrap_or_else(|e| panic!("{label}: fuse: {e:?}"));
            transform_solid(&mut topo, body, &placement).unwrap();
            let line = edge_between(
                &topo,
                body,
                placement.mul_point(Point3::new(big, 0.0, 0.0)),
                placement.mul_point(Point3::new(big, 0.0, height)),
            );
            let before = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();

            let result = fillet_cascade(&mut topo, body, &[line], r)
                .unwrap_or_else(|e| panic!("{label}: {}: {e}", blend_failure_code(&e)));
            assert!(result.failed.is_empty(), "{label}: partial blend");
            assert_valid_closed(&topo, result.solid, &label);

            let band = bands(&topo, body, result.solid, r);
            assert_eq!(band.len(), 1, "{label}: one exact cylindrical band");
            let FaceSurface::Cylinder(band) = topo.face(band[0]).unwrap().surface() else {
                unreachable!()
            };
            assert!(
                band.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12,
                "{label}"
            );

            let gained = solid_volume(&topo, result.solid, MEASURE_DEFLECTION).unwrap() - before;
            let expected = section * height;
            assert!(
                (gained - expected).abs() < 1e-6 * before,
                "{label}: gained {gained}, expected {expected}"
            );
        }
    }
}

/// A flat ground onto a shaft, filleted where the flat meets the round. The
/// band is an exact cylinder, and the flat, the shaft and the band each have
/// the closed-form area.
#[test]
fn convex_line_where_a_flat_meets_a_shaft() {
    let (big, r, height, flat_x) = (6.0, 1.0, 24.0, 4.0);
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, big, height).unwrap();
    let cutter = make_box(&mut topo, 10.0, 20.0, height + 6.0).unwrap();
    transform_solid(&mut topo, cutter, &Mat4::translation(flat_x, -10.0, -3.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, shaft, cutter).unwrap();
    let half_chord = (big * big - flat_x * flat_x).sqrt();
    let line = edge_between(
        &topo,
        body,
        Point3::new(flat_x, half_chord, 0.0),
        Point3::new(flat_x, half_chord, height),
    );

    let result = fillet_cascade(&mut topo, body, &[line], r)
        .unwrap_or_else(|e| panic!("{}: {e}", blend_failure_code(&e)));
    assert!(result.failed.is_empty());
    assert_valid_closed(&topo, result.solid, "flat");

    // The ball rolls inside the material: one radius behind the flat and
    // `big − r` from the axis. It touches the flat at its own height and the
    // shaft along its direction from the axis; the band turns from the
    // flat's normal (+x) to the shaft's there.
    let center_y = ((big - r).powi(2) - (flat_x - r).powi(2)).sqrt();
    let shaft_contact_angle = center_y.atan2(flat_x - r);
    let ruling_angle = half_chord.atan2(flat_x);

    let band = bands(&topo, body, result.solid, r);
    assert_eq!(band.len(), 1, "one exact cylindrical band");
    let band_area = face_area(&topo, band[0], MEASURE_DEFLECTION).unwrap();
    let expected_band = shaft_contact_angle * r * height;
    assert!(
        (band_area - expected_band).abs() < 1e-6,
        "band {band_area} vs {expected_band}"
    );

    let mut flat_area = None;
    let mut shaft_area = None;
    for face in solid_faces(&topo, result.solid).unwrap() {
        let area = face_area(&topo, face, MEASURE_DEFLECTION).unwrap();
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { normal, .. } if normal.z().abs() < 1e-9 => flat_area = Some(area),
            FaceSurface::Cylinder(c) if (c.radius() - big).abs() < 1e-12 => shaft_area = Some(area),
            _ => {}
        }
    }
    let expected_flat = (half_chord + center_y) * height;
    let expected_shaft = (TAU - ruling_angle - shaft_contact_angle) * big * height;
    let (flat_area, shaft_area) = (flat_area.unwrap(), shaft_area.unwrap());
    assert!(
        (flat_area - expected_flat).abs() < 1e-6,
        "flat {flat_area} vs {expected_flat}"
    );
    assert!(
        (shaft_area - expected_shaft).abs() < 1e-6,
        "shaft {shaft_area} vs {expected_shaft}"
    );
}

/// The boss rising past the block at BOTH ends: each end of the fillet stops
/// part-way along the boss, so the strip to remove is bounded by two section
/// arcs and touches the boss's boundary only at the spine's own vertices. Not
/// supported yet; it must refuse by its typed code and leave the input
/// untouched rather than build a wrong body.
#[test]
fn boss_past_the_wall_at_both_ends_refuses_cleanly() {
    let mut topo = Topology::new();
    let boss = make_cylinder(&mut topo, 6.0, 32.0).unwrap();
    transform_solid(&mut topo, boss, &Mat4::translation(0.0, 0.0, -4.0)).unwrap();
    let block = make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
    let body = boolean(&mut topo, BooleanOp::Fuse, boss, block).unwrap();
    let line = edge_between(
        &topo,
        body,
        Point3::new(6.0, 0.0, 0.0),
        Point3::new(6.0, 0.0, 24.0),
    );
    let faces_before = solid_faces(&topo, body).unwrap();
    let Err(error) = fillet_cascade(&mut topo, body, &[line], 1.0) else {
        panic!("the boss past both ends built; tighten this test to the result");
    };
    assert_eq!(blend_failure_code(&error), "trimming-failure", "{error}");
    assert_eq!(solid_faces(&topo, body).unwrap(), faces_before);
}
