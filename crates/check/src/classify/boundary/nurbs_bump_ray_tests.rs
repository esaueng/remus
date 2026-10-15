//! The classifier's NURBS ray path must count a steep, narrow bump's
//! far-flank crossing.
//!
//! Degree-2×2 surface on uniform clamped knots, control points at the
//! Greville abscissae (`x = u`, `y = v`), flat at `z = 1` except control
//! point `(13, 13)` raised to `z = 1.5`. From a point inside the bump, below
//! its peak, each of the classifier's base rays leaves through the bump's
//! surface exactly once ahead of the origin. With `NURBS_RAY_SAMPLES = 20`
//! the first base ray found only crossings behind the origin.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::{FaceCache, NURBS_RAY_SAMPLES};
use crate::classify::ray_surface::{ray_nurbs, ray_nurbs_with_grid};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};

fn bump_surface() -> NurbsSurface {
    const N: usize = 30;
    let mut knots = vec![0.0; 3];
    knots.extend((1..28).map(|k| f64::from(k) / 28.0));
    knots.extend([1.0; 3]);
    let g: Vec<f64> = (0..N)
        .map(|i| 0.5 * (knots[i + 1] + knots[i + 2]))
        .collect();
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

#[test]
fn every_base_ray_counts_one_forward_bump_crossing() {
    let surface = bump_surface();
    let peak = 12.5 / 28.0;
    let origin = Point3::new(peak, peak, 1.1);
    let (a, b, c) = (
        0.573_576_436_351_046,
        0.740_535_693_464_567_5,
        0.350_889_803_483_932_2,
    );
    let cache = FaceCache::default();
    for dir in [Vec3::new(a, b, c), Vec3::new(-c, a, b), Vec3::new(b, -c, a)] {
        let one_shot = ray_nurbs(origin, dir, &surface, NURBS_RAY_SAMPLES).unwrap();
        let cached = ray_nurbs_with_grid(origin, dir, &surface, cache.line_grid(&surface)).unwrap();
        assert_eq!(one_shot.len(), 1, "{dir:?}: {one_shot:?}");
        assert_eq!(cached.len(), 1, "{dir:?}: {cached:?}");
        let (t, u, v) = one_shot[0];
        // The crossing lies on the bump (its 3-span support), above the flat
        // top, and on the ray.
        let hit = surface.evaluate(u, v);
        assert!(
            hit.z() > 1.0 && (11.0 / 28.0..=14.0 / 28.0).contains(&u),
            "{hit:?}"
        );
        assert!((hit - (origin + dir * t)).length() < 1e-8);
    }
}

#[test]
fn apex_contact_is_preserved_by_math_and_abstains_from_parity() {
    use remus_math::nurbs::intersection::intersect_line_nurbs;
    use remus_topology::Topology;
    use remus_topology::face::{Face, FaceSurface};
    let surface = bump_surface();
    let peak = 12.5 / 28.0;
    let apex = surface.evaluate(peak, peak);
    let origin = Point3::new(peak - 0.5, peak, apex.z());
    let direction = Vec3::new(1.0, 0.0, 0.0);
    let hits = intersect_line_nurbs(&surface, origin, direction, NURBS_RAY_SAMPLES).unwrap();
    assert!(!hits.is_empty(), "math retains the tangent contact");
    let mut topo = Topology::new();
    let corners = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
    ];
    let wire = remus_topology::builder::make_polygon_wire(&mut topo, &corners, 1e-7).unwrap();
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface.clone())));
    for cached in [false, true] {
        let cache = FaceCache::default();
        let result = super::ray_crossings_nurbs_with_trim(
            &topo,
            face,
            None,
            cached.then_some(&cache),
            origin,
            direction,
            &surface,
        );
        assert!(
            matches!(&result, Err(crate::CheckError::ClassificationFailed(message)) if message == crate::classify::NURBS_RAY_GRAZE),
            "{result:?}; {hits:?}; normals {:?}",
            {
                use remus_math::traits::ParametricSurface;
                hits.iter()
                    .map(|hit| {
                        let (u, v) = hit.param1;
                        let n = surface.partial_u(u, v).cross(surface.partial_v(u, v));
                        n.dot(direction).abs() / n.length()
                    })
                    .collect::<Vec<_>>()
            }
        );
    }
}

#[test]
fn arbitrarily_shallow_flat_crossing_remains_qualified() {
    let surface = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0; 2]; 2],
    )
    .unwrap();
    for slope in [1e-4, 1e-8, 1e-12] {
        assert!(!super::nurbs_contact_parity_uncertain(
            &surface,
            Point3::new(0.5, 0.5, 0.0),
            Vec3::new(1.0, 0.0, slope),
            0.5,
            0.5
        ));
    }
}
