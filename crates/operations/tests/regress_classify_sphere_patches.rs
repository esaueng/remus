//! RFC 0008 F2: sphere patches bounded by three or more circles classify by
//! their exact circle-arc trim.
//!
//! A sphere of radius 6 centred on the x = 20 face of a 20 x 20 x 10 box
//! protrudes through that face and the top and bottom faces. Each surviving
//! sphere face is bounded by arcs of three circles and is not convex, so the
//! cap, great-circle and two-plane sphere trims declined, and the chorded 3D
//! polygon fallback got containment wrong: `(23, 10, 5)`, deep inside the
//! sphere, classified `Outside`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives::{make_box, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const CENTER: Point3 = Point3::new(20.0, 10.0, 5.0);
const RADIUS: f64 = 6.0;

fn in_sphere(p: Point3) -> f64 {
    RADIUS - (p - CENTER).length()
}

/// Signed distance-like margin to the box `[lo, hi]`: positive inside.
fn in_box(p: Point3, lo: Point3, hi: Point3) -> f64 {
    (0..3)
        .map(|k| (p.0[k] - lo.0[k]).min(hi.0[k] - p.0[k]))
        .fold(f64::INFINITY, f64::min)
}

/// Grid points whose closed-form margin clears `1e-3`, each with its verdict.
fn closed_form(margin: impl Fn(Point3) -> f64, lo: Point3, hi: Point3) -> Vec<(Point3, bool)> {
    let n = 15;
    let mut points = Vec::new();
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let f = |c: usize, a: f64, b: f64| a + (b - a) * (c as f64 + 0.37) / n as f64;
                let p = Point3::new(
                    f(i, lo.x(), hi.x()),
                    f(j, lo.y(), hi.y()),
                    f(k, lo.z(), hi.z()),
                );
                let m = margin(p);
                if m.abs() > 1e-3 {
                    points.push((p, m > 0.0));
                }
            }
        }
    }
    points
}

fn assert_classifies(topo: &Topology, solid: SolidId, points: &[(Point3, bool)]) {
    let prepared = PreparedSolid::prepare(topo, solid).unwrap();
    let options = ClassifyOptions::default();
    for &(p, inside) in points {
        let want = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(
            prepared.classify_point(p, &options).unwrap(),
            want,
            "prepared at {p:?}"
        );
        assert_eq!(
            classify_point(topo, solid, p, &options).unwrap(),
            want,
            "one-shot at {p:?}"
        );
    }
}

fn block_and_ball(op: BooleanOp) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 20.0, 20.0, 10.0).unwrap();
    let ball = make_sphere(&mut topo, RADIUS, 16).unwrap();
    transform_solid(
        &mut topo,
        ball,
        &Mat4::translation(CENTER.x(), CENTER.y(), CENTER.z()),
    )
    .unwrap();
    let solid = boolean(&mut topo, op, block, ball).unwrap();
    (topo, solid)
}

const BOX_LO: Point3 = Point3::new(0.0, 0.0, 0.0);
const BOX_HI: Point3 = Point3::new(20.0, 20.0, 10.0);

#[test]
fn rfc_0008_f2_repro_point() {
    let (topo, solid) = block_and_ball(BooleanOp::Fuse);
    assert_classifies(&topo, solid, &[(Point3::new(23.0, 10.0, 5.0), true)]);
}

#[test]
fn box_union_sphere_classifies_by_closed_form() {
    let (topo, solid) = block_and_ball(BooleanOp::Fuse);
    let points = closed_form(
        |p| in_box(p, BOX_LO, BOX_HI).max(in_sphere(p)),
        Point3::new(13.0, 3.0, -2.0),
        Point3::new(27.0, 17.0, 12.0),
    );
    assert_classifies(&topo, solid, &points);
}

/// The cut leaves the same patches as reversed faces.
#[test]
fn box_minus_sphere_classifies_by_closed_form() {
    let (topo, solid) = block_and_ball(BooleanOp::Cut);
    let points = closed_form(
        |p| in_box(p, BOX_LO, BOX_HI).min(-in_sphere(p)),
        Point3::new(13.0, 3.0, -1.0),
        Point3::new(21.0, 17.0, 11.0),
    );
    assert_classifies(&topo, solid, &points);
}

/// Three cut planes meet on the sphere: its remaining face is bounded by
/// arcs of three circles.
#[test]
fn sphere_minus_corner_classifies_by_closed_form() {
    let mut topo = Topology::new();
    let ball = make_sphere(&mut topo, RADIUS, 16).unwrap();
    let corner = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    transform_solid(&mut topo, corner, &Mat4::translation(1.0, 1.5, 2.0)).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Cut, ball, corner).unwrap();
    let (lo, hi) = (Point3::new(1.0, 1.5, 2.0), Point3::new(11.0, 11.5, 12.0));
    let origin = Point3::new(0.0, 0.0, 0.0);
    let points = closed_form(
        |p| (RADIUS - (p - origin).length()).min(-in_box(p, lo, hi)),
        Point3::new(-6.5, -6.5, -6.5),
        Point3::new(6.5, 6.5, 6.5),
    );
    assert_classifies(&topo, solid, &points);
}

/// The box-sphere octant comes from an analytic shortcut whose shell winds
/// its faces on the right of their loops: the arc trim reads that side off a
/// planar neighbour instead of assuming one.
#[test]
fn box_intersect_sphere_octant_classifies_by_closed_form() {
    let mut topo = Topology::new();
    let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let ball = make_sphere(&mut topo, 8.0, 32).unwrap();
    let solid = boolean(&mut topo, BooleanOp::Intersect, block, ball).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let points = closed_form(
        |p| (8.0 - (p - origin).length()).min(in_box(p, BOX_LO, Point3::new(10.0, 10.0, 10.0))),
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(9.0, 9.0, 9.0),
    );
    assert_classifies(&topo, solid, &points);
}
