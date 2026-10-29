//! Regression: a flange bolt circle with holes on the 45° diagonals must cut
//! exactly.
//!
//! The blank is the pipe-flange demo's: a rim (r24..45, z0..10) and a hub
//! (r12..24, z0..26), each revolved about the vertical axis from a profile on
//! the XZ plane (seam at angle 0), fused and unified. The bolt circle is one
//! r3 cylinder on the r34 circle, circular-patterned about Z, fused into one
//! tool and cut from the blank.
//!
//! Bolt counts 8 and 16 (and any two holes including one at 45°) were refused
//! as `ExactOnlyUnattainable`; 4, 5, 6 and 12 cut. The rim cap (z = 10) has
//! the r24 hub opening as an inner wire, and each bolt stamps a closed circle
//! on it. The gate that sends several closed circles to the internal-loops
//! path proved them clear of that opening by axis-aligned bounding box. A 45°
//! hole's box corner sits at 34/√2 − 3 ≈ 21.04 < 24 on both axes, inside the
//! opening's box, though the discs are 7 apart. The cap fell to the wire
//! builder, which wove the holes into one sub-face, the bore walls had nothing
//! to meet, and the GFA result had free edges. The gate now compares circles
//! as discs.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::compound_ops::fuse_solids;
use remus_operations::heal::unify_faces;
use remus_operations::measure;
use remus_operations::pattern::circular_pattern;
use remus_operations::primitives;
use remus_operations::revolve::revolve;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;
const BOLT_R: f64 = 3.0;
const RIM_T: f64 = 10.0;

/// Revolve a rectangle on the XZ plane about the vertical axis. The axis
/// points down (-Z), as the demo's XZ sketch basis puts it.
fn revolved_annulus(topo: &mut Topology, r0: f64, r1: f64, z0: f64, z1: f64) -> SolidId {
    let pts = [
        Point3::new(r0, 0.0, z0),
        Point3::new(r1, 0.0, z0),
        Point3::new(r1, 0.0, z1),
        Point3::new(r0, 0.0, z1),
    ];
    let wire = make_polygon_wire(topo, &pts, TOL).unwrap();
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        TAU,
    )
    .unwrap()
}

fn flange_blank(topo: &mut Topology) -> SolidId {
    let rim = revolved_annulus(topo, 24.0, 45.0, 0.0, RIM_T);
    let hub = revolved_annulus(topo, 12.0, 24.0, 0.0, 26.0);
    let blank = boolean(topo, BooleanOp::Fuse, rim, hub).expect("blank fuse");
    unify_faces(topo, blank).unwrap();
    blank
}

/// The r3 bolt tool on the bolt circle at `angle`, long enough to pass clear
/// through the rim (z -3..13).
fn bolt(topo: &mut Topology, bolt_circle: f64, angle: f64) -> SolidId {
    let c = primitives::make_cylinder(topo, BOLT_R, RIM_T + 6.0).unwrap();
    transform_solid(
        topo,
        c,
        &Mat4::translation(bolt_circle * angle.cos(), bolt_circle * angle.sin(), -3.0),
    )
    .unwrap();
    c
}

fn blank_volume() -> f64 {
    PI * ((45.0f64 * 45.0 - 24.0 * 24.0) * RIM_T + (24.0f64 * 24.0 - 12.0 * 12.0) * 26.0)
}

fn surface_census(topo: &Topology, solid: SolidId) -> HashMap<&'static str, usize> {
    let mut census = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

fn free_and_nonmanifold(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
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

/// Check a drilled flange: exact analytic faces, closed and valid, the
/// closed-form volume, and every hole open while the rim between holes stays
/// solid.
fn assert_drilled(
    topo: &Topology,
    drilled: SolidId,
    bolt_circle: f64,
    angles: &[f64],
    label: &str,
) {
    let n = angles.len();
    let census = surface_census(topo, drilled);
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        3 + n,
        "{label}: 3 body cylinders + {n} bolt walls, got {census:?}"
    );
    assert_eq!(
        census.values().sum::<usize>(),
        6 + n,
        "{label}: expected an analytic result, got {census:?}"
    );
    assert_eq!(
        free_and_nonmanifold(topo, drilled),
        (0, 0),
        "{label}: drilled flange must be watertight and manifold"
    );
    let report = validate_solid(topo, drilled).unwrap();
    assert!(report.is_valid(), "{label}: invalid result: {report:?}");

    #[allow(clippy::cast_precision_loss)]
    let expected = (n as f64).mul_add(-PI * BOLT_R * BOLT_R * RIM_T, blank_volume());
    let volume = measure::solid_volume(topo, drilled, 0.05).unwrap();
    assert!(
        (volume - expected).abs() / expected < 1e-6,
        "{label}: volume {volume} vs analytic {expected}"
    );

    let opts = ClassifyOptions::default();
    let at = |r: f64, a: f64| Point3::new(r * a.cos(), r * a.sin(), RIM_T / 2.0);
    for &a in angles {
        assert_eq!(
            classify_point(topo, drilled, at(bolt_circle, a), &opts).unwrap(),
            PointClassification::Outside,
            "{label}: hole at {:.1}° must be drilled through",
            a.to_degrees()
        );
        // Beside the hole, outside its r3 wall, the rim is solid.
        assert_eq!(
            classify_point(topo, drilled, at(bolt_circle + 2.0 * BOLT_R, a), &opts).unwrap(),
            PointClassification::Inside,
            "{label}: rim beside the hole at {:.1}° must stay solid",
            a.to_degrees()
        );
    }
}

/// The demo's full chain: pattern, fuse into one tool, unify, cut.
fn drill_bolt_circle(n: usize, bolt_circle: f64) {
    let mut topo = Topology::new();
    let blank = flange_blank(&mut topo);
    let seed = bolt(&mut topo, bolt_circle, 0.0);
    let pattern = circular_pattern(&mut topo, seed, Vec3::new(0.0, 0.0, 1.0), n).unwrap();
    let copies = topo.compound(pattern).unwrap().solids().to_vec();
    let tool = fuse_solids(&mut topo, &copies).expect("bolt circle fuse");
    unify_faces(&mut topo, tool).unwrap();
    let label = format!("N={n} bolt circle {bolt_circle}");
    let drilled = boolean(&mut topo, BooleanOp::Cut, blank, tool)
        .unwrap_or_else(|e| panic!("{label}: cut refused: {e:?}"));
    #[allow(clippy::cast_precision_loss)]
    let angles: Vec<f64> = (0..n).map(|i| TAU * i as f64 / n as f64).collect();
    assert_drilled(&topo, drilled, bolt_circle, &angles, &label);
}

/// Cut the holes at `degrees` as one fused tool.
fn drill_holes_at(degrees: &[f64]) {
    let mut topo = Topology::new();
    let blank = flange_blank(&mut topo);
    let tools: Vec<SolidId> = degrees
        .iter()
        .map(|d| bolt(&mut topo, 34.0, d.to_radians()))
        .collect();
    let tool = if tools.len() == 1 {
        tools[0]
    } else {
        let tool = fuse_solids(&mut topo, &tools).expect("tool fuse");
        unify_faces(&mut topo, tool).unwrap();
        tool
    };
    let label = format!("holes at {degrees:?}°");
    let drilled = boolean(&mut topo, BooleanOp::Cut, blank, tool)
        .unwrap_or_else(|e| panic!("{label}: cut refused: {e:?}"));
    let angles: Vec<f64> = degrees.iter().map(|d| d.to_radians()).collect();
    assert_drilled(&topo, drilled, 34.0, &angles, &label);
}

/// 8 and 16 were refused before the fix; 4, 6 and 12 keep working.
#[test]
fn bolt_circle_counts_cut_exactly() {
    for n in [4usize, 6, 8, 12, 16] {
        drill_bolt_circle(n, 34.0);
    }
}

/// A larger bolt circle moves the 45° box corner (35/√2 − 3 ≈ 21.7) but still
/// inside the opening's box; it was refused too.
#[test]
fn eight_holes_on_a_larger_bolt_circle_cut_exactly() {
    drill_bolt_circle(8, 35.0);
}

/// Holes one at a time (these take the local drilled-hole path) and in
/// opposite pairs (two closed circles per cap, so the face splitter's
/// multi-loop gate decides) at the diagonals and around them.
#[test]
fn single_and_paired_holes_at_any_angle_cut_exactly() {
    for degrees in [
        0.0, 22.5, 30.0, 44.0, 45.0, 46.0, 60.0, 90.0, 135.0, 225.0, 315.0,
    ] {
        drill_holes_at(&[degrees]);
        drill_holes_at(&[degrees, degrees + 180.0]);
    }
    // Two neighbours of an 8-hole circle, one of them on the diagonal.
    drill_holes_at(&[0.0, 45.0]);
}
