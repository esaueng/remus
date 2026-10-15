//! Point classification above a steep, narrow NURBS bump must count the
//! bump's far-flank crossing.
//!
//! The cube's top face is a degree-2×2 NURBS surface, flat at `z = 1` except
//! one control point raised to `z = 1.5`, which lifts a bump about three knot
//! spans (`3/28`) wide. A query point inside the bump, below its peak, is
//! inside the solid. The classifier's first base ray leaves it obliquely and
//! exits through the bump's far slope. The 20×20 line-intersection seed grid
//! put no seed in that crossing's Newton basin, so `intersect_line_nurbs`
//! returned only the two crossings behind the origin, the ray counted no
//! forward top-face crossing and the point classified as Outside.
//!
//! Both the line-intersection completion and conservative face BVH bounds
//! are required: this end-to-end witness runs with both fixes integrated.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::test_utils::make_unit_cube_manifold;

const N: usize = 30;
const SPANS: f64 = 28.0;

/// Uniform clamped degree-2 knots on `[0, 1]` and their Greville abscissae.
fn knots_and_greville() -> (Vec<f64>, Vec<f64>) {
    let mut knots = vec![0.0; 3];
    knots.extend((1..28).map(|k| f64::from(k) / SPANS));
    knots.extend([1.0; 3]);
    let greville = (0..N)
        .map(|i| 0.5 * (knots[i + 1] + knots[i + 2]))
        .collect();
    (knots, greville)
}

/// The bump surface: the plane `z = 1` over the unit square, control point
/// `(13, 13)` raised to `z = 1.5`. Greville placement keeps `x = u`, `y = v`.
fn bump_surface() -> NurbsSurface {
    let (knots, g) = knots_and_greville();
    let control_points = (0..N)
        .map(|i| {
            (0..N)
                .map(|j| {
                    let z = if i == 13 && j == 13 { 1.5 } else { 1.0 };
                    Point3::new(g[i], g[j], z)
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(
        2,
        2,
        knots.clone(),
        knots,
        control_points,
        vec![vec![1.0; N]; N],
    )
    .unwrap()
}

fn bumped_cube(topo: &mut Topology) -> remus_topology::solid::SolidId {
    let solid = make_unit_cube_manifold(topo);
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let top = faces
        .into_iter()
        .find(|&f| match topo.face(f).unwrap().surface() {
            FaceSurface::Plane { normal, d } => normal.z() > 0.5 && (d - 1.0).abs() < 1e-12,
            _ => false,
        })
        .expect("cube has a +Z face at z = 1");
    // The surface's boundary rows lie on the cube's top edges, and its
    // natural normal (∂u × ∂v) points +Z, outward.
    topo.face_mut(top)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(bump_surface()));
    solid
}

#[test]
fn point_inside_narrow_bump_classifies_inside() {
    let mut topo = Topology::new();
    let solid = bumped_cube(&mut topo);
    let peak = 12.5 / SPANS;
    // Below the peak (z ≈ 1.28125) and above the flat top: inside.
    let point = Point3::new(peak, peak, 1.1);
    let options = ClassifyOptions::default();

    assert_eq!(
        classify_point(&topo, solid, point, &options).unwrap(),
        PointClassification::Inside
    );
    let prepared = PreparedSolid::prepare(&topo, solid).unwrap();
    assert_eq!(
        prepared.classify_point(point, &options).unwrap(),
        PointClassification::Inside
    );

    // Controls on either side of the bump's surface.
    let above_peak = Point3::new(peak, peak, 1.4);
    assert_eq!(
        classify_point(&topo, solid, above_peak, &options).unwrap(),
        PointClassification::Outside
    );
    let below_top = Point3::new(peak, peak, 0.9);
    assert_eq!(
        classify_point(&topo, solid, below_top, &options).unwrap(),
        PointClassification::Inside
    );
}
