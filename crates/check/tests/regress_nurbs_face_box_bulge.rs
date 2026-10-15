//! Point classification must see the whole of a NURBS face (HC-8).
//!
//! The classifier's ray BVH boxed each face with `face_aabb`, which bounds a
//! NURBS face by a 9x9 sample grid. A ridge whose support falls between two
//! grid lines was outside the box, and the BVH pruned the face from every
//! ray that crossed the ridge, so a point inside it classified as Outside.
//!
//! The cube's top face is replaced by a flat degree-2 NURBS patch over the
//! same unit square with a low ridge along v: interior control points of
//! column 13 are raised by 0.01. With 30 control points per direction on
//! uniform clamped knots, column 13 is supported on u in (11/28, 14/28); the
//! grid samples u = 10.5/28 and 14/28 both read z = 1 exactly, so the sampled
//! box stops at the cube's top. The ridge stays shallower than the
//! classifier's rays (slope at most 0.28 against their 0.35 or more), so each
//! ray leaves it through the crest once.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::test_utils::make_unit_cube_manifold;

const N: usize = 30;
const RIDGE: usize = 13;
const RAISE: f64 = 0.01;

fn knots() -> Vec<f64> {
    let spans = (N - 2) as f64;
    let mut knots = vec![0.0; 3];
    knots.extend((1..N - 2).map(|k| k as f64 / spans));
    knots.extend([1.0; 3]);
    knots
}

/// Greville abscissae: control points there make the patch's (u, v) chart
/// the identity on the unit square, so its boundary rows are the cube's
/// top edges.
fn greville(knots: &[f64], i: usize) -> f64 {
    f64::midpoint(knots[i + 1], knots[i + 2])
}

#[test]
fn ridge_between_samples_classifies_inside() {
    let mut topo = Topology::new();
    let cube = make_unit_cube_manifold(&mut topo);
    let top = remus_topology::explorer::solid_faces(&topo, cube)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(),
                FaceSurface::Plane { normal, d } if normal.z() > 0.5 && (d - 1.0).abs() < 1e-12)
        })
        .expect("cube has a top face at z = 1");
    let knots = knots();
    let control: Vec<Vec<Point3>> = (0..N)
        .map(|i| {
            (0..N)
                .map(|j| {
                    let z = if i == RIDGE && (1..N - 1).contains(&j) {
                        1.0 + RAISE
                    } else {
                        1.0
                    };
                    Point3::new(greville(&knots, i), greville(&knots, j), z)
                })
                .collect()
        })
        .collect();
    let surface = NurbsSurface::new(
        2,
        2,
        knots.clone(),
        knots.clone(),
        control,
        vec![vec![1.0; N]; N],
    )
    .unwrap();
    let crest = surface.evaluate(greville(&knots, RIDGE), 0.5);
    assert!(crest.z() > 1.007, "ridge crest {crest:?}");
    topo.face_mut(top)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(surface));

    let options = ClassifyOptions::default();
    let under_crest = Point3::new(crest.x(), crest.y(), 1.003);
    assert_eq!(
        classify_point(&topo, cube, under_crest, &options).unwrap(),
        PointClassification::Inside
    );
    let over_crest = Point3::new(crest.x(), crest.y(), crest.z() + 0.003);
    assert_eq!(
        classify_point(&topo, cube, over_crest, &options).unwrap(),
        PointClassification::Outside
    );
}
