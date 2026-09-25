#![allow(clippy::unwrap_used, clippy::expect_used)]

use crate::nurbs::surface::NurbsSurface;
use crate::vec::{Point3, Vec3};

use super::surface_marching::march_intersection;
use super::surface_marching::{SsiScratch, near_existing_segment, second_order_tangent};
use super::surface_seeding::{
    find_ssi_seeds_grid, find_ssi_seeds_subdivision, find_ssi_seeds_subdivision_with_context,
    refine_ssi_point, refine_ssi_point_with_context,
};
use super::*;

/// Create a simple bilinear NURBS surface (flat plane at z=0, from (0,0) to (1,1)).
fn flat_surface() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

/// Create a curved surface (saddle shape).
fn saddle_surface() -> NurbsSurface {
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.5, 0.25),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![
                Point3::new(0.5, 0.0, -0.25),
                Point3::new(0.5, 0.5, 0.0),
                Point3::new(0.5, 1.0, 0.25),
            ],
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.5, -0.25),
                Point3::new(1.0, 1.0, 0.0),
            ],
        ],
        vec![vec![1.0; 3]; 3],
    )
    .unwrap()
}

// -- Plane-NURBS intersection --

#[test]
fn flat_surface_plane_no_intersection() {
    let surface = flat_surface();
    // Plane at z=1 shouldn't intersect surface at z=0.
    let result = intersect_plane_nurbs(&surface, Vec3::new(0.0, 0.0, 1.0), 1.0, 30).unwrap();

    assert!(result.is_empty(), "no intersection expected");
}

#[test]
fn saddle_surface_plane_intersection() {
    let surface = saddle_surface();
    // Plane at z=0 should intersect the saddle surface.
    let result = intersect_plane_nurbs(&surface, Vec3::new(0.0, 0.0, 1.0), 0.0, 50).unwrap();

    assert!(
        !result.is_empty(),
        "saddle surface should intersect z=0 plane"
    );

    // The intersection curve should have points near z=0.
    for curve in &result {
        for pt in &curve.points {
            assert!(
                pt.point.z().abs() < 1e-4,
                "intersection point should be near z=0, got z={}",
                pt.point.z()
            );
        }
    }
}

// -- Line-NURBS intersection --

#[test]
fn line_flat_surface_intersection() {
    let surface = flat_surface();
    // Vertical ray through (0.5, 0.5) should hit the surface at z=0.
    let result = intersect_line_nurbs(
        &surface,
        Point3::new(0.5, 0.5, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
        20,
    )
    .unwrap();

    assert!(!result.is_empty(), "ray should hit flat surface");

    let pt = &result[0];
    assert!(
        (pt.point.x() - 0.5).abs() < 1e-4,
        "x should be ~0.5, got {}",
        pt.point.x()
    );
    assert!(
        (pt.point.y() - 0.5).abs() < 1e-4,
        "y should be ~0.5, got {}",
        pt.point.y()
    );
    assert!(
        pt.point.z().abs() < 1e-4,
        "z should be ~0.0, got {}",
        pt.point.z()
    );
}

#[test]
fn line_misses_surface() {
    let surface = flat_surface();
    // Ray parallel to the surface should miss.
    let result = intersect_line_nurbs(
        &surface,
        Point3::new(0.5, 0.5, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
        20,
    )
    .unwrap();

    assert!(result.is_empty(), "parallel ray should miss");
}

// -- Intersection point quality --

#[test]
fn refined_points_are_on_plane() {
    let surface = saddle_surface();
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let d = 0.1; // Slightly above z=0.
    let result = intersect_plane_nurbs(&surface, normal, d, 50).unwrap();

    for curve in &result {
        for pt in &curve.points {
            let signed_dist = Vec3::new(pt.point.x(), pt.point.y(), pt.point.z()).dot(normal) - d;
            assert!(
                signed_dist.abs() < 1e-4,
                "point should be on plane, signed_dist={signed_dist}"
            );
        }
    }
}

// -- NURBS-NURBS intersection --

/// Create a flat surface at z=0.5 (overlapping region with `flat_surface` at z=0).
fn flat_surface_offset() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, 0.5), Point3::new(0.0, 1.0, 0.5)],
            vec![Point3::new(1.0, 0.0, 0.5), Point3::new(1.0, 1.0, 0.5)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

/// Create a tilted flat surface that intersects the flat z=0 surface.
fn tilted_surface() -> NurbsSurface {
    // Surface tilted in the XZ plane: goes from z=-0.5 at x=0 to z=0.5 at x=1.
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, -0.5), Point3::new(0.0, 1.0, -0.5)],
            vec![Point3::new(1.0, 0.0, 0.5), Point3::new(1.0, 1.0, 0.5)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

#[test]
fn parallel_surfaces_no_intersection() {
    let s1 = flat_surface();
    let s2 = flat_surface_offset();
    let result = intersect_nurbs_nurbs(&s1, &s2, 15, 0.02).unwrap();
    assert!(result.is_empty(), "parallel surfaces should not intersect");
}

#[test]
fn refine_ssi_basic() {
    let s1 = flat_surface();
    let s2 = tilted_surface();
    // At u1=0.5, v1=0.5 on flat -> (0.5, 0.5, 0)
    // At u2=0.5, v2=0.5 on tilted -> (0.5, 0.5, 0)
    // These should refine to an intersection point.
    let result = refine_ssi_point(&s1, &s2, 0.5, 0.5, 0.5, 0.5, 1e-6);
    assert!(
        result.is_some(),
        "refine should find intersection at (0.5, 0.5)"
    );
}

#[test]
fn seed_finding_basic() {
    let s1 = flat_surface();
    let s2 = tilted_surface();

    // Verify surfaces evaluate correctly.
    let p1 = s1.evaluate(0.5, 0.5);
    let p2 = s2.evaluate(0.5, 0.5);
    let dist = (p1 - p2).length();
    assert!(
        dist < 0.01,
        "flat(0.5,0.5)={p1:?} tilted(0.5,0.5)={p2:?} dist={dist}",
    );

    // Verify refine works from off-center guess.
    let refined = refine_ssi_point(&s1, &s2, 0.5263, 0.5, 0.5263, 0.5, 1e-6);
    assert!(
        refined.is_some(),
        "refine should converge from off-center guess"
    );

    let seeds = find_ssi_seeds_grid(&s1, &s2, 10, 1e-6);
    assert!(
        !seeds.is_empty(),
        "should find seeds between flat and tilted surfaces"
    );
}

#[test]
fn tilted_intersects_flat() {
    let s1 = flat_surface();
    let s2 = tilted_surface();

    // First verify seed finding works.
    let seeds = find_ssi_seeds_grid(&s1, &s2, 10, 1e-6);
    assert!(
        !seeds.is_empty(),
        "should find at least one seed point, got 0"
    );

    let result = intersect_nurbs_nurbs(&s1, &s2, 10, 0.05).unwrap();

    assert!(
        !result.is_empty(),
        "tilted surface should intersect flat surface (seeds: {})",
        seeds.len()
    );

    for curve in &result {
        for pt in &curve.points {
            assert!(
                pt.point.z().abs() < 0.15,
                "point should be near z=0, got z={}",
                pt.point.z()
            );
        }
    }
}

#[test]
fn ssi_points_lie_on_both_surfaces() {
    let s1 = flat_surface();
    let s2 = tilted_surface();
    let result = intersect_nurbs_nurbs(&s1, &s2, 10, 0.02).unwrap();

    for curve in &result {
        for pt in &curve.points {
            // Check point lies on surface 1.
            let p1 = s1.evaluate(pt.param1.0, pt.param1.1);
            let dist1 = (p1 - pt.point).length();
            assert!(dist1 < 0.05, "point should lie on surface 1, dist={dist1}");

            // Check point lies on surface 2.
            let p2 = s2.evaluate(pt.param2.0, pt.param2.1);
            let dist2 = (p2 - pt.point).length();
            assert!(dist2 < 0.05, "point should lie on surface 2, dist={dist2}");
        }
    }
}

/// Create a dome-shaped NURBS surface (quadratic, unit domain).
/// High at center (z=2), low at edges (z=-1), so slicing at z=0
/// produces a closed ring-like intersection.
fn dome_surface() -> NurbsSurface {
    NurbsSurface::new(
        2,
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            vec![
                Point3::new(0.0, 0.0, -1.0),
                Point3::new(0.0, 0.5, 0.5),
                Point3::new(0.0, 1.0, -1.0),
            ],
            vec![
                Point3::new(0.5, 0.0, 0.5),
                Point3::new(0.5, 0.5, 2.0),
                Point3::new(0.5, 1.0, 0.5),
            ],
            vec![
                Point3::new(1.0, 0.0, -1.0),
                Point3::new(1.0, 0.5, 0.5),
                Point3::new(1.0, 1.0, -1.0),
            ],
        ],
        vec![vec![1.0; 3]; 3],
    )
    .unwrap()
}

/// Create a flat surface at a given z height, mapping [0,1]^2 to the
/// same XY extent [0,1]x[0,1] as the dome.
fn flat_plane_at_z(z: f64) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(0.0, 0.0, z), Point3::new(0.0, 1.0, z)],
            vec![Point3::new(1.0, 0.0, z), Point3::new(1.0, 1.0, z)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

#[test]
fn ssi_tangential_touch() {
    // Two surfaces that touch tangentially: a dome and a flat plane at the
    // dome's peak height. The normals are parallel at the touch point, so
    // this exercises the singular_tangent_direction fallback.
    let dome = dome_surface();
    // The dome peaks around z=2 at the center. Use a plane slightly below
    // to create a tangential touch region.
    let peak_z = dome.evaluate(0.5, 0.5).z();

    // Place the plane at the peak height -- tangential contact.
    let plane = flat_plane_at_z(peak_z);

    // At the tangent point both normals point in +z, so cross product vanishes.
    // The marching should handle this gracefully via singular_tangent_direction.
    let seed = refine_ssi_point(&dome, &plane, 0.5, 0.5, 0.5, 0.5, 1e-6);
    assert!(
        seed.is_some(),
        "should find a seed at the tangential contact point"
    );

    let seed = seed.unwrap();
    assert!(
        (seed.point.z() - peak_z).abs() < 0.2,
        "seed should be near z={peak_z}, got z={}",
        seed.point.z()
    );

    // March from the tangential point. The key requirement is that this
    // does not panic and handles the singular point.
    let traced = march_intersection(&dome, &plane, &seed, 0.05, 1e-6);

    // At a true tangential touch (single point contact), marching may
    // produce few or no additional points -- that's acceptable. The test
    // ensures we don't crash/panic at the singular point.
    // If the plane is slightly below peak, there may be a small intersection
    // loop.
    for pt in &traced {
        // All traced points should be reasonably close to both surfaces.
        let p1 = dome.evaluate(pt.param1.0, pt.param1.1);
        let p2 = plane.evaluate(pt.param2.0, pt.param2.1);
        let dist1 = (p1 - pt.point).length();
        let dist2 = (p2 - pt.point).length();
        assert!(
            dist1 < 0.5,
            "traced point should be near dome surface, dist={dist1}"
        );
        assert!(
            dist2 < 0.5,
            "traced point should be near plane surface, dist={dist2}"
        );
    }
}

#[test]
fn ssi_closed_loop() {
    // Intersect a dome surface with a horizontal plane.
    // Use a known seed point and march directly to test closed-loop
    // detection without the expensive O(n^4) seed search.
    let dome = dome_surface();
    let plane = flat_plane_at_z(0.0);

    // Find one seed by refining a point we know is on the intersection
    // (from the debug test: the z=0 contour passes through the region
    // around u=0.25 on the dome).
    let seed = refine_ssi_point(&dome, &plane, 0.25, 0.5, 0.25, 0.5, 1e-6)
        .expect("should refine to a seed on the dome-plane intersection");

    // Verify the seed is near z=0.
    assert!(
        seed.point.z().abs() < 0.1,
        "seed should be near z=0, got z={}",
        seed.point.z()
    );

    // March from the seed.
    let traced = march_intersection(&dome, &plane, &seed, 0.05, 1e-6);

    assert!(
        traced.len() >= 5,
        "should trace at least 5 points, got {}",
        traced.len()
    );

    // Check that the curve closes: first and last points should be close.
    let first = &traced[0];
    let last = &traced[traced.len() - 1];
    let gap = (first.point - last.point).length();

    assert!(
        gap < 0.5,
        "expected closed loop (first-last gap < 0.5), got gap={gap:.4}"
    );

    // All points should lie near z=0.
    for pt in &traced {
        assert!(
            pt.point.z().abs() < 0.15,
            "intersection point should be near z=0, got z={}",
            pt.point.z()
        );
    }
}

// -- Subdivision seed finder tests --

#[test]
fn subdivision_finds_seeds() {
    let s1 = flat_surface();
    let s2 = tilted_surface();

    let seeds = find_ssi_seeds_subdivision(&s1, &s2, 1e-6);
    assert!(
        !seeds.is_empty(),
        "subdivision should find seeds between flat and tilted"
    );

    // All seeds should lie on both surfaces
    for seed in &seeds {
        let p1 = s1.evaluate(seed.param1.0, seed.param1.1);
        let p2 = s2.evaluate(seed.param2.0, seed.param2.1);
        assert!(
            (p1 - seed.point).length() < 0.01,
            "seed should lie on surface 1"
        );
        assert!(
            (p2 - seed.point).length() < 0.01,
            "seed should lie on surface 2"
        );
    }
}

#[test]
fn caller_subdivision_depth_budget_is_authoritative_for_ssi_seeding() {
    use crate::context::{OperationContext, WorkBudgets};

    let dome = dome_surface();
    let plane = flat_plane_at_z(0.0);
    let counts: Vec<usize> = (0..=6)
        .map(|depth| {
            let context = OperationContext::new()
                .with_budgets(WorkBudgets::new().with_subdivision_depth(depth));
            find_ssi_seeds_subdivision_with_context(
                &dome,
                &plane,
                1e-6,
                &context,
                &mut SsiScratch::new(),
            )
            .unwrap()
            .len()
        })
        .collect();

    assert_eq!(counts[0], 0, "zero budget must perform no recursive split");
    assert!(
        counts[6] > 0,
        "the legacy depth budget must discover the closed intersection: {counts:?}"
    );
}

// -- Chain building tests --

#[test]
fn chain_separates_branches() {
    // Two clusters of points with a gap between them
    let points = vec![
        IntersectionPoint {
            point: Point3::new(0.0, 0.0, 0.0),
            param1: (0.0, 0.0),
            param2: (0.0, 0.0),
        },
        IntersectionPoint {
            point: Point3::new(0.1, 0.0, 0.0),
            param1: (0.1, 0.0),
            param2: (0.1, 0.0),
        },
        IntersectionPoint {
            point: Point3::new(0.2, 0.0, 0.0),
            param1: (0.2, 0.0),
            param2: (0.2, 0.0),
        },
        // Gap
        IntersectionPoint {
            point: Point3::new(5.0, 0.0, 0.0),
            param1: (0.5, 0.0),
            param2: (0.5, 0.0),
        },
        IntersectionPoint {
            point: Point3::new(5.1, 0.0, 0.0),
            param1: (0.6, 0.0),
            param2: (0.6, 0.0),
        },
    ];

    let chains = chain_intersection_points(&points, 0.5);
    assert_eq!(
        chains.len(),
        2,
        "should separate into 2 branches, got {}",
        chains.len()
    );
}

#[test]
fn chain_detects_single_group() {
    // Points close together: should form 1 chain
    let points: Vec<IntersectionPoint> = (0..5)
        .map(|i| {
            let x = f64::from(i) * 0.1;
            IntersectionPoint {
                point: Point3::new(x, 0.0, 0.0),
                param1: (x, 0.0),
                param2: (x, 0.0),
            }
        })
        .collect();

    let chains = chain_intersection_points(&points, 0.5);
    assert_eq!(chains.len(), 1, "all close points should form 1 chain");
    assert_eq!(chains[0].len(), 5);
}

/// Test second-order tangent analysis with two nearly-tangent surfaces.
#[test]
fn second_order_tangent_finds_direction() {
    // Two surfaces that touch at (0.5, 0.5): one flat, one dome.
    // At the touch point, normals are parallel (both ~+z), so
    // first-order tangent n1 x n2 ~ 0.
    let dome = dome_surface();
    let peak_z = dome.evaluate(0.5, 0.5).z();

    // Place a flat plane at the dome's peak height.
    let plane = flat_plane_at_z(peak_z);

    // Try the second-order analysis.
    let result = second_order_tangent(&dome, &plane, 0.5, 0.5, 0.5, 0.5, &mut SsiScratch::new());

    // The result should be Some (a direction was found) or None
    // (degenerate -- surfaces osculate to second order).
    // For a dome with quadratic curvature vs flat plane, the
    // curvature difference is non-zero, so we should get a direction.
    if let Some(dir) = result {
        // The direction should be a unit vector in the tangent plane.
        let len = dir.length();
        assert!(
            (len - 1.0).abs() < 0.01,
            "tangent direction should be unit length, got {len}"
        );
        // The direction should be roughly in the XY plane (since
        // both surfaces are horizontal at the touch point).
        assert!(
            dir.z().abs() < 0.5,
            "tangent direction should be mostly horizontal, got z={}",
            dir.z()
        );
    }
    // None is also acceptable for this degenerate case -- it means
    // the perturbation fallback will be used.
}

// -- Non-normalized domain tests --

/// Create a bilinear surface over domain [0, 100] x [0, 100].
fn wide_domain_surface(z: f64) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 100.0, 100.0],
        vec![0.0, 0.0, 100.0, 100.0],
        vec![
            vec![Point3::new(0.0, 0.0, z), Point3::new(0.0, 10.0, z)],
            vec![Point3::new(10.0, 0.0, z), Point3::new(10.0, 10.0, z)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

/// Create a tilted surface over domain [0, 100] x [0, 100] that
/// crosses z=0 at x=5.
fn wide_domain_tilted() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 100.0, 100.0],
        vec![0.0, 0.0, 100.0, 100.0],
        vec![
            vec![Point3::new(0.0, 0.0, -5.0), Point3::new(0.0, 10.0, -5.0)],
            vec![Point3::new(10.0, 0.0, 5.0), Point3::new(10.0, 10.0, 5.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

#[test]
fn plane_nurbs_wide_domain() {
    // Surface with knot domain [0, 100] -- should still find the
    // intersection with the z=0 plane.
    let tilted = wide_domain_tilted();

    // Verify domain is actually [0, 100].
    let (u_min, u_max) = tilted.domain_u();
    let (v_min, v_max) = tilted.domain_v();
    assert!(u_min.abs() < 1e-10);
    assert!((u_max - 100.0).abs() < 1e-10);
    assert!(v_min.abs() < 1e-10);
    assert!((v_max - 100.0).abs() < 1e-10);

    let result = intersect_plane_nurbs(&tilted, Vec3::new(0.0, 0.0, 1.0), 0.0, 50).unwrap();

    assert!(
        !result.is_empty(),
        "should find intersection on [0,100] domain surface"
    );

    for curve in &result {
        for pt in &curve.points {
            assert!(
                pt.point.z().abs() < 0.2,
                "intersection point should be near z=0, got z={}",
                pt.point.z()
            );
            // x should be near 5.0 (the midpoint where z crosses 0)
            assert!(
                (pt.point.x() - 5.0).abs() < 1.0,
                "x should be near 5.0, got {}",
                pt.point.x()
            );
        }
    }
}

#[test]
fn ssi_wide_domain_surfaces() {
    // Two surfaces with [0, 100] domains that intersect.
    let s1 = wide_domain_surface(0.0);
    let s2 = wide_domain_tilted();

    // Verify domains.
    assert!((s1.domain_u().1 - 100.0).abs() < 1e-10);
    assert!((s2.domain_u().1 - 100.0).abs() < 1e-10);

    let seeds = find_ssi_seeds_grid(&s1, &s2, 15, 1e-6);
    assert!(
        !seeds.is_empty(),
        "should find seeds between wide-domain surfaces"
    );

    let result = intersect_nurbs_nurbs(&s1, &s2, 15, 0.0).unwrap();
    assert!(
        !result.is_empty(),
        "should find SSI on [0,100] domain surfaces"
    );

    for curve in &result {
        for pt in &curve.points {
            assert!(
                pt.point.z().abs() < 0.5,
                "SSI point should be near z=0, got z={}",
                pt.point.z()
            );
        }
    }
}

/// A flat patch of the size a b-spline-converted box face has.
fn wide_flat_patch() -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(-1.0, 11.0, 0.0)],
            vec![Point3::new(11.0, -1.0, 0.0), Point3::new(11.0, 11.0, 0.0)],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    )
    .unwrap()
}

/// Rays that are OBLIQUE to the surface must be found too.
///
/// Every other ray test here fires along the surface normal, which is the one
/// direction where the refinement's normal matrix is already correct: the
/// tangents are perpendicular to the ray, so projecting them changes nothing.
/// Off that axis the raw matrix is inflated by the ray-parallel component of
/// each tangent, every step is under-relaxed, and the iteration budget runs out
/// with the intersection undiscovered — silently, as an empty result.
///
/// These directions are the ones `remus-check` casts for point-in-solid
/// classification. Before the fix a plain b-spline box misclassified a quarter
/// of its interior points because of it.
#[test]
fn line_nurbs_oblique_rays_are_found() {
    let surface = wide_flat_patch();
    let dirs = [
        Vec3::new(
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
            0.350_889_803_483_932_2,
        ),
        Vec3::new(
            0.267_261_241_912_424_4,
            0.534_522_483_824_849,
            0.801_783_725_737_273,
        ),
        Vec3::new(
            -0.424_264_068_711_928_5,
            0.565_685_424_949_238,
            0.707_106_781_186_547_5,
        ),
    ];
    for (k, dir) in dirs.iter().enumerate() {
        // Aim from below so the ray crosses z = 0 at a point well inside the patch.
        let target = Point3::new(4.0, 6.0, 0.0);
        let origin = target - *dir * 7.0;

        let hits = intersect_line_nurbs(&surface, origin, *dir, 20).unwrap();
        assert!(
            !hits.is_empty(),
            "dir {k}: oblique ray must hit the patch, got no intersection"
        );
        let best = hits
            .iter()
            .map(|h| (h.point - target).length())
            .fold(f64::INFINITY, f64::min);
        assert!(
            best < 1e-6,
            "dir {k}: expected the hit at {target:?}, closest returned was {best:.3e} away"
        );
    }
}

#[test]
fn line_nurbs_wide_domain() {
    // Ray intersection with a surface having [0, 100] domain.
    let surface = wide_domain_surface(0.0);

    let result = intersect_line_nurbs(
        &surface,
        Point3::new(5.0, 5.0, 1.0),
        Vec3::new(0.0, 0.0, -1.0),
        20,
    )
    .unwrap();

    assert!(!result.is_empty(), "ray should hit wide-domain surface");

    let pt = &result[0];
    assert!(
        (pt.point.x() - 5.0).abs() < 0.5,
        "x should be ~5.0, got {}",
        pt.point.x()
    );
    assert!(
        pt.point.z().abs() < 0.1,
        "z should be ~0.0, got {}",
        pt.point.z()
    );
}

/// Create a half-cylinder-like surface with v-domain [0, 2pi].
fn cylinder_nurbs_surface() -> NurbsSurface {
    use std::f64::consts::PI;
    let tau = 2.0 * PI;
    // Approximate a cylinder of radius 1, height 2, with a degree-2
    // NURBS surface in v (angular) and degree-1 in u (height).
    // Use 9 control points in v for a full circle (rational).
    let r = 1.0;
    let w = std::f64::consts::FRAC_1_SQRT_2; // cos(45 deg)

    // v knots for a full circle: [0,0,0, pi/2,pi/2, pi,pi, 3pi/2,3pi/2, 2pi,2pi,2pi]
    let knots_v = vec![
        0.0,
        0.0,
        0.0,
        PI / 2.0,
        PI / 2.0,
        PI,
        PI,
        3.0 * PI / 2.0,
        3.0 * PI / 2.0,
        tau,
        tau,
        tau,
    ];

    // 9 control points around the circle at z=0 and z=2.
    let circle_cps = [
        (r, 0.0, 1.0),
        (r, r, w),
        (0.0, r, 1.0),
        (-r, r, w),
        (-r, 0.0, 1.0),
        (-r, -r, w),
        (0.0, -r, 1.0),
        (r, -r, w),
        (r, 0.0, 1.0),
    ];

    let cps_bottom: Vec<Point3> = circle_cps
        .iter()
        .map(|&(x, y, _)| Point3::new(x, y, 0.0))
        .collect();
    let cps_top: Vec<Point3> = circle_cps
        .iter()
        .map(|&(x, y, _)| Point3::new(x, y, 2.0))
        .collect();

    let weights_row: Vec<f64> = circle_cps.iter().map(|&(_, _, w_)| w_).collect();

    NurbsSurface::new(
        1,
        2,
        vec![0.0, 0.0, 2.0, 2.0], // u: height [0, 2]
        knots_v,
        vec![cps_bottom, cps_top],
        vec![weights_row.clone(), weights_row],
    )
    .unwrap()
}

#[test]
fn plane_nurbs_cylinder_domain() {
    use std::f64::consts::PI;
    let cylinder = cylinder_nurbs_surface();

    // Verify domain is [0,2] x [0, 2pi].
    let (u_min, u_max) = cylinder.domain_u();
    let (v_min, v_max) = cylinder.domain_v();
    assert!((u_min - 0.0).abs() < 1e-10);
    assert!((u_max - 2.0).abs() < 1e-10);
    assert!((v_min - 0.0).abs() < 1e-10);
    assert!((v_max - 2.0 * PI).abs() < 1e-10);

    // Intersect with a plane at z=1 (horizontal slice through cylinder).
    let result = intersect_plane_nurbs(&cylinder, Vec3::new(0.0, 0.0, 1.0), 1.0, 50).unwrap();

    assert!(
        !result.is_empty(),
        "should find intersection of cylinder with z=1 plane"
    );

    // All intersection points should be near z=1 and at radius ~1.
    for curve in &result {
        for pt in &curve.points {
            assert!(
                (pt.point.z() - 1.0).abs() < 0.2,
                "z should be ~1.0, got {}",
                pt.point.z()
            );
            let r = (pt.point.x().powi(2) + pt.point.y().powi(2)).sqrt();
            assert!((r - 1.0).abs() < 0.2, "radius should be ~1.0, got {r}");
        }
    }
}

/// A transversal plane slicing a NURBS tube wall must return the full loop as
/// ONE closed fitted curve — not zero curves, and not an open arc with a
/// grid-sized gap.
///
/// At 32 seed-grid samples the wall's z=1 loop seeds 31 unique crossings.
/// The old per-cell edge scan pushed every crossing twice (once per incident
/// cell), and the doubled cloud drove the chaining threshold's
/// nearest-neighbour average to zero: each twin pair chained alone, deduped
/// to a single point, and was discarded as too short — the loop returned
/// zero curves. Unique-edge emission keeps the average at the true
/// along-curve spacing; the single chain's endpoints then land one grid
/// spacing apart and are closed by an exact re-append refit, so the stored
/// curve is closed (start == end) with fit deviation at the interpolation
/// error, not the grid spacing.
#[test]
fn plane_nurbs_tube_wall_returns_closed_loop() {
    let cylinder = cylinder_nurbs_surface();

    // Coarse grid: 32 samples is what the FF phase passes (`NURBS_SAMPLES`).
    let result = intersect_plane_nurbs(&cylinder, Vec3::new(0.0, 0.0, 1.0), 1.0, 32).unwrap();

    assert_eq!(
        result.len(),
        1,
        "one transversal loop should chain into one curve, got {}",
        result.len()
    );
    let curve = &result[0].curve;
    let domain = curve.domain();
    let (p_start, p_end) = (curve.evaluate(domain.0), curve.evaluate(domain.1));
    assert!(
        (p_start - p_end).length() < 1e-9,
        "loop should be closed, gap = {:.3e}",
        (p_start - p_end).length()
    );
    // Independent geometry oracle: every fitted point lies on the true tube
    // (z=1 plane, unit radius) to the interpolation error — orders below the
    // 0.06 grid spacing a gap would leave.
    for k in 0..=32 {
        #[allow(clippy::cast_precision_loss)]
        let t = domain.0 + (domain.1 - domain.0) * (k as f64) / 32.0;
        let p = curve.evaluate(t);
        assert!(
            (p.z() - 1.0).abs() < 1e-3,
            "fitted point should lie in the z=1 plane, got z={}",
            p.z()
        );
        let r = (p.x().powi(2) + p.y().powi(2)).sqrt();
        assert!(
            (r - 1.0).abs() < 1e-3,
            "fitted point should lie on the unit circle, got r={r}"
        );
    }
}

/// Build an OPEN polygonal tube: `segments` straight quads around an arc that
/// stops `gap_angle` short of closing, so the carrier has a real wedge gap.
///
/// The wall is degree (1, 1): one quad per segment, control points exactly on
/// the cylinder, so every point of the carrier is within chord error of the
/// radius-1 tube and the z=1 slice is a clean near-full arc. The gap is a
/// genuine carrier boundary, not sampling noise.
fn open_polygonal_tube(segments: usize, gap_angle: f64) -> NurbsSurface {
    use std::f64::consts::TAU;
    assert!(segments >= 3);
    assert!(gap_angle > 0.0 && gap_angle < TAU);
    let span = TAU - gap_angle;
    let mut bottom = Vec::with_capacity(segments + 1);
    let mut top = Vec::with_capacity(segments + 1);
    let mut knots = Vec::with_capacity(segments + 4);
    knots.push(0.0);
    knots.push(0.0);
    for i in 0..=segments {
        #[allow(clippy::cast_precision_loss)]
        let a = span * (i as f64) / (segments as f64);
        bottom.push(Point3::new(a.cos(), a.sin(), 0.0));
        top.push(Point3::new(a.cos(), a.sin(), 2.0));
        if i > 0 && i < segments {
            knots.push(a);
        }
    }
    knots.push(span);
    knots.push(span);
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        knots,
        vec![bottom, top],
        vec![vec![1.0; segments + 1]; 2],
    )
    .unwrap()
}

/// A physical wedge gap must survive section fitting.
#[test]
fn plane_nurbs_open_tube_stays_open() {
    use std::f64::consts::TAU;
    let tube = open_polygonal_tube(128, 0.02);

    let result = intersect_plane_nurbs(&tube, Vec3::new(0.0, 0.0, 1.0), 1.0, 32).unwrap();

    assert_eq!(
        result.len(),
        1,
        "one open arc should chain into one curve, got {}",
        result.len()
    );
    let curve = &result[0].curve;
    let domain = curve.domain();
    let (p_start, p_end) = (curve.evaluate(domain.0), curve.evaluate(domain.1));
    let gap = (p_start - p_end).length();
    // The carrier's real gap: chord of the missing 0.02-radian wedge.
    let expected = 2.0_f64 * (0.02_f64 / 2.0).sin();
    assert!(
        gap > 0.01 && (gap - expected).abs() < 0.01,
        "open arc should keep its carrier gap (~{expected:.4}), got gap = {gap:.4}",
    );
    // Sanity: the arc really is the near-full tube (spans the circle), not a
    // short fragment the seed grid happened to catch.
    let mut covered = 0.0_f64;
    let mut prev = p_start;
    for k in 1..=64 {
        #[allow(clippy::cast_precision_loss)]
        let t = domain.0 + (domain.1 - domain.0) * (k as f64) / 64.0;
        let p = curve.evaluate(t);
        covered += (p - prev).length();
        prev = p;
    }
    assert!(
        (covered - (TAU - 0.02)).abs() < 0.5,
        "arc should span the near-full tube (~{:.2}), got length {covered:.2}",
        TAU - 0.02
    );
}

/// Verify that the tangential touch test still works with the new
/// second-order analysis integrated into the main SSI pipeline.
#[test]
fn ssi_tangential_with_second_order() {
    let dome = dome_surface();
    let peak_z = dome.evaluate(0.5, 0.5).z();
    let plane = flat_plane_at_z(peak_z - 0.3); // Below peak but not extremely close

    // This should find an intersection loop near the peak.
    // Use a large march step since we only care about correctness, not density.
    let result = intersect_nurbs_nurbs(&dome, &plane, 5, 0.2).unwrap();

    // Near-tangential: may or may not find an intersection (depends
    // on numerical precision), but should NOT crash.
    for curve in &result {
        for pt in &curve.points {
            // All points should be close to the plane height.
            assert!(
                (pt.point.z() - (peak_z - 0.3)).abs() < 0.5,
                "intersection point should be near z={:.2}, got z={:.4}",
                peak_z - 0.3,
                pt.point.z()
            );
        }
    }
}

/// Line curve through the flat surface at z=0: from (-1,-1,0.5) to (2,2,-0.5).
/// Should cross the unit square plane at one point.
#[test]
fn curve_surface_line_through_flat_plane() {
    use crate::nurbs::curve::NurbsCurve;

    let surf = flat_surface(); // z=0 plane, (0..1, 0..1)
    // Straight line from (-1,-1,0.5) to (2,2,-0.5) as degree-1 NURBS.
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(-1.0, -1.0, 0.5), Point3::new(2.0, 2.0, -0.5)],
        vec![1.0, 1.0],
    )
    .unwrap();

    let hits = intersect_curve_surface(&curve, &surf, 1e-7).unwrap();
    assert_eq!(hits.len(), 1, "expected 1 hit, got {}", hits.len());

    let hit = &hits[0];
    // The line is C(t) = (-1 + 3t, -1 + 3t, 0.5 - t). C(t).z = 0 -> t = 0.5.
    // C(0.5) = (0.5, 0.5, 0.0).
    assert!(
        (hit.point.z()).abs() < 1e-5,
        "z should be ~0, got {}",
        hit.point.z()
    );
    assert!(
        (hit.point.x() - 0.5).abs() < 1e-5,
        "x should be ~0.5, got {}",
        hit.point.x()
    );
    assert!(
        (hit.t - 0.5).abs() < 1e-4,
        "t should be ~0.5, got {}",
        hit.t
    );
}

/// A degree-2 curve (parabola) intersecting a flat plane -- should find 2 points.
#[test]
fn curve_surface_parabola_through_flat_plane() {
    use crate::nurbs::curve::NurbsCurve;

    let surf = flat_surface(); // z=0, (0..1, 0..1)
    // Quadratic curve from (0.2, 0.5, -0.3) through control (0.5, 0.5, 1.0)
    // to (0.8, 0.5, -0.3). The z-component is:
    //   z(t) = (1-t)^2(-0.3) + 2t(1-t)(1.0) + t^2(-0.3)
    //        = -0.3 + 2.6t - 2.6t^2
    // z = 0 at t ~ 0.133 and t ~ 0.867 -- two clear crossings.
    let curve = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.2, 0.5, -0.3),
            Point3::new(0.5, 0.5, 1.0),
            Point3::new(0.8, 0.5, -0.3),
        ],
        vec![1.0, 1.0, 1.0],
    )
    .unwrap();

    let hits = intersect_curve_surface(&curve, &surf, 1e-7).unwrap();
    assert_eq!(hits.len(), 2, "expected 2 hits, got {}", hits.len());

    // Both hits should be on the z=0 plane.
    for hit in &hits {
        assert!(
            hit.point.z().abs() < 1e-4,
            "z should be ~0, got {}",
            hit.point.z()
        );
    }
    // Parameters should be symmetric around 0.5.
    assert!(hits[0].t < 0.5, "first hit t should be < 0.5");
    assert!(hits[1].t > 0.5, "second hit t should be > 0.5");
}

/// Build a cylinder NURBS surface along z-axis, centered at (cx, cy).
fn cylinder_at(cx: f64, cy: f64, r: f64, z_lo: f64, z_hi: f64) -> NurbsSurface {
    use std::f64::consts::PI;
    let tau = 2.0 * PI;
    let w = std::f64::consts::FRAC_1_SQRT_2;

    let knots_v = vec![
        0.0,
        0.0,
        0.0,
        PI / 2.0,
        PI / 2.0,
        PI,
        PI,
        3.0 * PI / 2.0,
        3.0 * PI / 2.0,
        tau,
        tau,
        tau,
    ];

    let circle_cps = [
        (r, 0.0, 1.0),
        (r, r, w),
        (0.0, r, 1.0),
        (-r, r, w),
        (-r, 0.0, 1.0),
        (-r, -r, w),
        (0.0, -r, 1.0),
        (r, -r, w),
        (r, 0.0, 1.0),
    ];

    let cps_lo: Vec<Point3> = circle_cps
        .iter()
        .map(|&(x, y, _)| Point3::new(cx + x, cy + y, z_lo))
        .collect();
    let cps_hi: Vec<Point3> = circle_cps
        .iter()
        .map(|&(x, y, _)| Point3::new(cx + x, cy + y, z_hi))
        .collect();

    let weights: Vec<f64> = circle_cps.iter().map(|&(_, _, w_)| w_).collect();

    NurbsSurface::new(
        1,
        2,
        vec![z_lo, z_lo, z_hi, z_hi],
        knots_v,
        vec![cps_lo, cps_hi],
        vec![weights.clone(), weights],
    )
    .unwrap()
}

/// Build a cylinder NURBS surface along x-axis, centered at (cy, cz).
fn cylinder_along_x(cy: f64, cz: f64, r: f64, x_lo: f64, x_hi: f64) -> NurbsSurface {
    use std::f64::consts::PI;
    let tau = 2.0 * PI;
    let w = std::f64::consts::FRAC_1_SQRT_2;

    let knots_v = vec![
        0.0,
        0.0,
        0.0,
        PI / 2.0,
        PI / 2.0,
        PI,
        PI,
        3.0 * PI / 2.0,
        3.0 * PI / 2.0,
        tau,
        tau,
        tau,
    ];

    // Circle in YZ plane.
    let circle_cps = [
        (r, 0.0, 1.0),
        (r, r, w),
        (0.0, r, 1.0),
        (-r, r, w),
        (-r, 0.0, 1.0),
        (-r, -r, w),
        (0.0, -r, 1.0),
        (r, -r, w),
        (r, 0.0, 1.0),
    ];

    let cps_lo: Vec<Point3> = circle_cps
        .iter()
        .map(|&(y, z, _)| Point3::new(x_lo, cy + y, cz + z))
        .collect();
    let cps_hi: Vec<Point3> = circle_cps
        .iter()
        .map(|&(y, z, _)| Point3::new(x_hi, cy + y, cz + z))
        .collect();

    let weights: Vec<f64> = circle_cps.iter().map(|&(_, _, w_)| w_).collect();

    NurbsSurface::new(
        1,
        2,
        vec![x_lo, x_lo, x_hi, x_hi],
        knots_v,
        vec![cps_lo, cps_hi],
        vec![weights.clone(), weights],
    )
    .unwrap()
}

#[test]
fn ssi_perpendicular_cylinders_two_loops() {
    // Two perpendicular cylinders of radius 1 centered at the origin:
    // cylinder A along z-axis, cylinder B along x-axis.
    // They produce two distinct closed intersection loops.
    let cyl_z = cylinder_at(0.0, 0.0, 1.0, -2.0, 2.0);
    let cyl_x = cylinder_along_x(0.0, 0.0, 1.0, -2.0, 2.0);

    let result = intersect_nurbs_nurbs(&cyl_z, &cyl_x, 20, 0.0).unwrap();

    // Should find at least 1 curve (ideally 2 for both loops).
    assert!(
        !result.is_empty(),
        "perpendicular cylinders must produce intersection curves"
    );

    // Verify all intersection points lie on both surfaces.
    for curve in &result {
        for pt in &curve.points {
            let on_cyl_z = {
                let x = pt.point.x();
                let y = pt.point.y();
                (x * x + y * y).sqrt()
            };
            let on_cyl_x = {
                let y = pt.point.y();
                let z = pt.point.z();
                (y * y + z * z).sqrt()
            };
            assert!(
                (on_cyl_z - 1.0).abs() < 0.05,
                "point should be on z-cylinder (r={on_cyl_z})"
            );
            assert!(
                (on_cyl_x - 1.0).abs() < 0.05,
                "point should be on x-cylinder (r={on_cyl_x})"
            );
        }
    }
}

#[test]
fn segment_distance_dedup_works() {
    // Verify that near_existing_segment uses segment distance,
    // not just point distance.
    let p0 = IntersectionPoint {
        point: Point3::new(0.0, 0.0, 0.0),
        param1: (0.0, 0.0),
        param2: (0.0, 0.0),
    };
    let p1 = IntersectionPoint {
        point: Point3::new(10.0, 0.0, 0.0),
        param1: (1.0, 0.0),
        param2: (1.0, 0.0),
    };
    let segment = vec![p0, p1];

    // Point near the middle of the segment (y=0.01).
    let near_mid = IntersectionPoint {
        point: Point3::new(5.0, 0.01, 0.0),
        param1: (0.5, 0.0),
        param2: (0.5, 0.0),
    };
    assert!(near_existing_segment(
        std::slice::from_ref(&segment),
        &near_mid,
        0.1
    ));

    // Point far from the segment (y=2.0).
    let far = IntersectionPoint {
        point: Point3::new(5.0, 2.0, 0.0),
        param1: (0.5, 0.0),
        param2: (0.5, 0.0),
    };
    assert!(!near_existing_segment(
        std::slice::from_ref(&segment),
        &far,
        0.1
    ));
}

#[test]
fn dual_surface_validation_passes_for_known_intersection() {
    use crate::nurbs::projection::project_point_to_surface;

    // Two transversely intersecting planar NURBS surfaces: flat (z=0) and
    // tilted (z goes from -0.5 to +0.5 across x). Their intersection is a
    // line at x=0.5 that must lie on both surfaces within tolerance.
    let s1 = flat_surface();
    let s2 = tilted_surface();

    let curves = intersect_nurbs_nurbs(&s1, &s2, 15, 0.02).unwrap();
    assert!(
        !curves.is_empty(),
        "transverse planar surfaces should produce at least one intersection curve"
    );

    let tol = 1e-3;
    for ic in &curves {
        let (t_min, t_max) = ic.curve.domain();
        for i in 0..5 {
            let t = t_min + (t_max - t_min) * i as f64 / 4.0;
            let pt = ic.curve.evaluate(t);

            // Point must be close to surface 1.
            let proj1 = project_point_to_surface(&s1, pt, tol).unwrap();
            assert!(
                proj1.distance < tol,
                "curve point at t={t:.3} deviates {:.2e} from surface 1",
                proj1.distance
            );

            // Point must be close to surface 2.
            let proj2 = project_point_to_surface(&s2, pt, tol).unwrap();
            assert!(
                proj2.distance < tol,
                "curve point at t={t:.3} deviates {:.2e} from surface 2",
                proj2.distance
            );
        }
    }
}

#[test]
fn with_context_default_matches_legacy_entry_point() {
    use crate::context::OperationContext;

    let s1 = flat_surface();
    let s2 = tilted_surface();
    let legacy = intersect_nurbs_nurbs(&s1, &s2, 15, 0.02).unwrap();
    let ctx =
        intersect_nurbs_nurbs_with_context(&s1, &s2, 15, 0.02, &OperationContext::new()).unwrap();

    assert_eq!(legacy.len(), ctx.len(), "curve counts must match");
    for (a, b) in legacy.iter().zip(&ctx) {
        assert_eq!(a.points.len(), b.points.len(), "point counts must match");
        for (pa, pb) in a.points.iter().zip(&b.points) {
            assert!(
                (pa.point - pb.point).length() == 0.0,
                "default context must reproduce the legacy result bit-for-bit"
            );
        }
    }
}

#[test]
fn caller_newton_budget_is_authoritative_for_ssi_refinement() {
    use crate::context::{OperationContext, WorkBudgets};

    let s1 = flat_surface();
    let s2 = tilted_surface();
    let full = refine_ssi_point_with_context(
        &s1,
        &s2,
        0.5263,
        0.5,
        0.5263,
        0.5,
        1e-6,
        &OperationContext::new(),
        &mut SsiScratch::new(),
    )
    .unwrap();
    assert!(full.is_some(), "default Newton budget must converge");

    let disabled =
        OperationContext::new().with_budgets(WorkBudgets::new().with_newton_iterations(0));
    let bounded = refine_ssi_point_with_context(
        &s1,
        &s2,
        0.5263,
        0.5,
        0.5263,
        0.5,
        1e-6,
        &disabled,
        &mut SsiScratch::new(),
    )
    .unwrap();
    assert!(
        bounded.is_none(),
        "a zero-iteration caller budget must perform no Newton step"
    );
}

#[test]
fn cancellation_is_polled_inside_ssi_newton_refinement() {
    use crate::MathError;
    use crate::context::{CancellationToken, OperationContext};

    let token = CancellationToken::new();
    let context = OperationContext::new().with_cancellation(token.clone());
    token.cancel();

    let result = refine_ssi_point_with_context(
        &flat_surface(),
        &tilted_surface(),
        0.5263,
        0.5,
        0.5263,
        0.5,
        1e-6,
        &context,
        &mut SsiScratch::new(),
    );
    assert!(matches!(result, Err(MathError::Cancelled)));
}

#[test]
fn tiny_march_budget_bounds_the_trace_and_terminates() {
    use crate::context::{OperationContext, WorkBudgets};

    let s1 = flat_surface();
    let s2 = tilted_surface();

    let full = intersect_nurbs_nurbs(&s1, &s2, 15, 0.02).unwrap();
    let full_points: usize = full.iter().map(|c| c.points.len()).sum();
    assert!(full_points > 10, "fixture must trace a real curve");

    let tiny = OperationContext::new().with_budgets(
        WorkBudgets::new()
            .with_march_steps(2)
            .with_segments(1)
            .with_queue_size(1),
    );
    let bounded = intersect_nurbs_nurbs_with_context(&s1, &s2, 15, 0.02, &tiny).unwrap();
    let bounded_points: usize = bounded.iter().map(|c| c.points.len()).sum();

    // The budget must actually bound the work: far fewer traced points than
    // the unbudgeted run, and the call terminates (no hang) with Ok.
    assert!(
        bounded_points < full_points,
        "tiny budget must trace fewer points ({bounded_points} vs {full_points})"
    );
}

#[test]
fn cancelled_context_refuses_ssi_with_typed_result() {
    use crate::MathError;
    use crate::context::{CancellationToken, OperationContext};

    let token = CancellationToken::new();
    let context = OperationContext::new().with_cancellation(token.clone());
    token.cancel();

    let result =
        intersect_nurbs_nurbs_with_context(&flat_surface(), &tilted_surface(), 15, 0.02, &context);
    assert!(matches!(result, Err(MathError::Cancelled)));
}

#[test]
fn plane_nurbs_open_tube_gap_is_independent_of_chart_and_rotation() {
    for swap in [false, true] {
        for rotation in [0.0_f64, 0.1, 1.7, 3.2] {
            let segments = 128;
            let end = std::f64::consts::TAU - 0.02;
            let mut knots = vec![0.0, 0.0];
            for i in 1..segments {
                knots.push(f64::from(i) / f64::from(segments));
            }
            knots.extend([1.0, 1.0]);
            let row = |z| {
                (0..=segments)
                    .map(|i| {
                        let a = end * f64::from(i) / f64::from(segments) + rotation;
                        Point3::new(a.cos(), a.sin(), z)
                    })
                    .collect::<Vec<_>>()
            };
            let bottom = row(0.0);
            let top = row(2.0);
            let axial = vec![0.0, 0.0, 2.0, 2.0];
            let surface = if swap {
                let points = bottom.iter().zip(&top).map(|(&a, &b)| vec![a, b]).collect();
                NurbsSurface::new(1, 1, knots, axial, points, vec![vec![1.0; 2]; 129])
            } else {
                NurbsSurface::new(
                    1,
                    1,
                    axial,
                    knots,
                    vec![bottom, top],
                    vec![vec![1.0; 129]; 2],
                )
            }
            .unwrap();
            let curves =
                intersect_plane_nurbs(&surface, Vec3::new(0.0, 0.0, 1.0), 1.0, 32).unwrap();
            assert_eq!(curves.len(), 1);
            let c = &curves[0].curve;
            let (a, b) = c.domain();
            let gap = (c.evaluate(a) - c.evaluate(b)).length();
            assert!(
                (gap - 2.0 * (0.01_f64).sin()).abs() < 0.001,
                "open tube closed: swap={swap}, rotation={rotation}, gap={gap}"
            );
        }
    }
}

/// Scale every control-point weight by a common positive factor: the surface
/// geometry is projectively invariant, so the traced intersection must be the
/// same curve as the unscaled pair. Guards the weight-scale normalization the
/// derivative scratch solves rely on against scale-dependent decisions in the
/// SSI pipeline.
#[test]
fn ssi_results_are_invariant_under_projective_weight_scaling() {
    use crate::context::OperationContext;
    use crate::nurbs::intersection::intersect_nurbs_nurbs_with_context;

    let scale_weights = |s: &NurbsSurface, k: f64| {
        let weights = s
            .weights()
            .iter()
            .map(|row| row.iter().map(|&w| w * k).collect())
            .collect();
        NurbsSurface::new(
            s.degree_u(),
            s.degree_v(),
            s.knots_u().to_vec(),
            s.knots_v().to_vec(),
            s.control_points().to_vec(),
            weights,
        )
        .unwrap()
    };

    let base = intersect_nurbs_nurbs_with_context(
        &saddle_surface(),
        &tilted_surface(),
        10,
        0.02,
        &OperationContext::new(),
    )
    .unwrap();
    let base_points: Vec<Point3> = base
        .iter()
        .flat_map(|c| c.points.iter().map(|p| p.point))
        .collect();
    assert!(!base_points.is_empty(), "unscaled pair intersects");

    for k in [0.37, 4.2] {
        let curves = {
            let a = scale_weights(&saddle_surface(), k);
            let b = scale_weights(&tilted_surface(), k);
            intersect_nurbs_nurbs_with_context(&a, &b, 10, 0.02, &OperationContext::new()).unwrap()
        };
        assert!(!curves.is_empty(), "scaled pair still intersects");
        let scaled_points: Vec<Point3> = curves
            .iter()
            .flat_map(|c| c.points.iter().map(|p| p.point))
            .collect();
        assert_eq!(
            scaled_points.len(),
            base_points.len(),
            "scaled pair traces the same point count (k={k})"
        );
        for p in &scaled_points {
            let nearest = base_points
                .iter()
                .map(|q| (*p - *q).length())
                .fold(f64::MAX, f64::min);
            assert!(
                nearest < 1e-6,
                "scaled point {p:?} has no unscaled witness within 1e-6 (k={k}): nearest {nearest:.3e}"
            );
        }
    }
}

// -- Marcher and seeding oracles (B19 survivor tranche, 2026-09-25 run) --
//
// Every expectation below comes from closed-form geometry: the exact contact
// line of a cylinder resting on a plane, the two ruling lines of a hyperbolic
// paraboloid cut by its tangent plane, the chord of two crossing planes, and
// the foot of a perpendicular. None of them compares against earlier output.
mod marching_oracles {
    use super::*;
    use crate::context::OperationContext;

    use super::super::surface_marching::{march_with_branches, surface_newton_step};
    use crate::nurbs::surface::DerivativeScratch;

    /// Exact rational half-cylinder of radius `r` resting on `z = 0`: axis
    /// along +y through `(0, ·, r)`, `u` sweeps the lower semicircle from
    /// `(-r, ·, r)` through the contact line `u = 0.5` to `(r, ·, r)`, `v`
    /// runs the length `len` along y.
    ///
    /// The contact point is the knot shared by two exact quarter arcs, so its
    /// point and `∂/∂u` are exact: the surface normal there is exactly `±z`.
    fn half_cylinder_on_plane(r: f64, len: f64) -> NurbsSurface {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let profile = [(-r, r), (-r, 0.0), (0.0, 0.0), (r, 0.0), (r, r)];
        let control = profile
            .iter()
            .map(|&(x, z)| vec![Point3::new(x, 0.0, z), Point3::new(x, len, z)])
            .collect();
        let weights = [1.0, w, 1.0, w, 1.0]
            .iter()
            .map(|&wi| vec![wi, wi])
            .collect();
        NurbsSurface::new(
            2,
            1,
            vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            control,
            weights,
        )
        .unwrap()
    }

    /// Bilinear patch of the plane `z = 0` over `[x0, x1] × [y0, y1]`.
    fn plane_z0(x0: f64, x1: f64, y0: f64, y1: f64) -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(x0, y0, 0.0), Point3::new(x0, y1, 0.0)],
                vec![Point3::new(x1, y0, 0.0), Point3::new(x1, y1, 0.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap()
    }

    /// The hyperbolic paraboloid `z = c·x·y` over `[-1, 1]²`. Bilinear
    /// interpolation of these four corners is exactly that quadric.
    fn saddle(c: f64) -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(-1.0, -1.0, c), Point3::new(-1.0, 1.0, -c)],
                vec![Point3::new(1.0, -1.0, -c), Point3::new(1.0, 1.0, c)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap()
    }

    /// The fixture is the cylinder it claims to be.
    fn assert_on_cylinder(s: &NurbsSurface, r: f64) {
        for i in 0..=10 {
            for j in 0..=4 {
                let p = s.evaluate(f64::from(i) / 10.0, f64::from(j) / 4.0);
                let radial = p.x().hypot(p.z() - r);
                assert!(
                    (radial - r).abs() < 1e-12,
                    "fixture off the cylinder: {p:?}"
                );
            }
        }
    }

    /// A cylinder lying on a plane meets it along a line where the two normals
    /// are exactly parallel, so `n1 × n2` vanishes at every traced point and
    /// only the singular (second-order) tangent can move the march. The trace
    /// must follow the contact line `x = z = 0` from one clamped `v` end of the
    /// cylinder to the other: length `0.998 · len` (the non-periodic 0.1%
    /// margin at each end), with no step longer than the `4 × step` cap.
    #[test]
    fn march_follows_the_exact_contact_line_of_a_resting_cylinder() {
        let (r, len) = (1.0, 4.0);
        let cyl = half_cylinder_on_plane(r, len);
        assert_on_cylinder(&cyl, r);
        let plane = plane_z0(-2.0, 2.0, -1.0, 5.0);

        let seed_point = Point3::new(0.0, 0.5 * len, 0.0);
        assert!((cyl.evaluate(0.5, 0.5) - seed_point).length() < 1e-15);
        assert!((plane.evaluate(0.5, 0.5) - seed_point).length() < 1e-15);
        let seed = IntersectionPoint {
            point: seed_point,
            param1: (0.5, 0.5),
            param2: (0.5, 0.5),
        };

        let (step, tol) = (0.05, 1e-7);
        let traced = march_intersection(&cyl, &plane, &seed, step, tol);
        assert!(traced.len() >= 3, "march stalled at the seed: {traced:?}");

        for pt in &traced {
            // On both surfaces' implicit equations: z = 0 and x² + (z-r)² = r²
            // intersect only in the line x = z = 0.
            assert!(pt.point.x().abs() < 1e-12, "off the contact line: {pt:?}");
            assert!(pt.point.z().abs() < 1e-12, "off the plane: {pt:?}");
            // Each parameter pair reproduces the point within the SSI residual
            // contract (`|S1 − S2| < tolerance`).
            assert!((cyl.evaluate(pt.param1.0, pt.param1.1) - pt.point).length() <= tol);
            assert!((plane.evaluate(pt.param2.0, pt.param2.1) - pt.point).length() <= tol);
        }

        let ys: Vec<f64> = traced.iter().map(|p| p.point.y()).collect();
        assert!(
            ys.windows(2).all(|w| w[1] > w[0]),
            "trace must run monotonically along the line: {ys:?}"
        );
        let margin = 1e-3 * len;
        assert!((ys[0] - margin).abs() < 1e-12, "start {}", ys[0]);
        assert!((ys[ys.len() - 1] - (len - margin)).abs() < 1e-12);
        let length: f64 = traced
            .windows(2)
            .map(|w| (w[1].point - w[0].point).length())
            .sum();
        assert!(
            (length - (len - 2.0 * margin)).abs() < 1e-12,
            "length {length}"
        );
        // Parameter steps never exceed `max_h = 4 · step` (v spans `len`).
        let max_gap = traced
            .windows(2)
            .map(|w| (w[1].point - w[0].point).length())
            .fold(0.0_f64, f64::max);
        assert!(
            max_gap <= 4.0 * step * len + 1e-12,
            "step cap exceeded: {max_gap}"
        );
    }

    /// On a contact line the curvature difference of the two surfaces has one
    /// zero eigenvalue, and its eigenvector is the ruling itself: `±y` here,
    /// whichever surface's parameterization the direction is built from.
    #[test]
    fn second_order_tangent_on_a_contact_line_is_the_ruling() {
        let cyl = half_cylinder_on_plane(1.0, 4.0);
        let plane = plane_z0(-2.0, 2.0, -1.0, 5.0);
        for v in [0.1, 0.3, 0.5, 0.9] {
            // Plane parameter of the contact point (0, 4v, 0).
            let v2 = (4.0 * v + 1.0) / 6.0;
            for dir in [
                second_order_tangent(&cyl, &plane, 0.5, v, 0.5, v2, &mut SsiScratch::new()),
                second_order_tangent(&plane, &cyl, 0.5, v2, 0.5, v, &mut SsiScratch::new()),
            ] {
                let dir = dir.expect("contact line has a well-defined ruling direction");
                assert!(
                    (dir.y().abs() - 1.0).abs() < 1e-12,
                    "not the ruling: {dir:?}"
                );
                assert!(dir.x().abs() < 1e-12 && dir.z().abs() < 1e-12, "{dir:?}");
            }
        }
    }

    /// The plane `z = 0` cuts `z = c·x·y` in the two lines `x = 0` and
    /// `y = 0`, crossing at the origin where the normals are parallel. A march
    /// along `x = 0` passes that crossing, and the branch it reports must lie
    /// on the other line: on both surfaces, on `y = 0`, clear of the traced
    /// branch, and on both sides of it (the transverse line continues both
    /// ways).
    #[test]
    fn march_through_a_saddle_crossing_reports_the_transverse_branch() {
        let c = 0.02;
        let s1 = saddle(c);
        let s2 = plane_z0(-1.5, 1.5, -1.5, 1.5);
        let tol = 1e-7;
        let seed = refine_ssi_point(&s1, &s2, 0.5, 0.8, 0.5, 0.7, tol).unwrap();
        assert!(seed.point.x().abs() < 1e-9 && (seed.point.y() - 0.6).abs() < 1e-9);

        let (traced, branches) = march_with_branches(
            &s1,
            &s2,
            &seed,
            0.05,
            tol,
            &OperationContext::new(),
            &mut SsiScratch::new(),
        )
        .unwrap();

        let on_both = |p: Point3| p.z().abs() <= tol && (p.z() - c * p.x() * p.y()).abs() <= tol;
        assert!(traced.iter().all(|p| on_both(p.point)));
        assert!(
            traced.iter().any(|p| p.point.y() < 0.0) && traced.iter().any(|p| p.point.y() > 0.0),
            "the trace must pass the crossing"
        );

        assert!(!branches.is_empty(), "crossing passed without a branch");
        for b in &branches {
            let p = b.point;
            assert!(on_both(p), "branch seed off the intersection: {p:?}");
            assert!(
                p.y().abs() < 1e-9,
                "branch seed not on the y = 0 line: {p:?}"
            );
            assert!(
                p.x().abs() > 10.0 * tol,
                "branch seed on the traced branch: {p:?}"
            );
            assert!((s1.evaluate(b.param1.0, b.param1.1) - p).length() <= tol);
            assert!((s2.evaluate(b.param2.0, b.param2.1) - p).length() <= tol);
        }
        assert!(branches.iter().any(|b| b.point.x() > 0.0));
        assert!(branches.iter().any(|b| b.point.x() < 0.0));
    }

    /// Ready-repro for B66. At a transversal crossing the curvature
    /// difference of the two surfaces is indefinite (here `±c` in the `xy`
    /// frame), and the branch directions are its null (asymptotic) directions,
    /// the lines `x = 0` and `y = 0`. The marcher confirms a branch only when
    /// both eigenvalues are below an absolute 0.1, so a steeper saddle's
    /// crossing goes unreported, and `second_order_tangent` returns the
    /// eigenvector of the smaller-magnitude eigenvalue: the bisector.
    #[test]
    #[ignore = "open: B66 — SSI branch points: absolute eigenvalue gate and bisector tangent"]
    fn steep_saddle_crossing_reports_its_branch_and_asymptotic_tangent() {
        let c = 0.2;
        let s1 = saddle(c);
        let s2 = plane_z0(-1.5, 1.5, -1.5, 1.5);
        let tol = 1e-7;

        let t = second_order_tangent(&s1, &s2, 0.5, 0.5, 0.5, 0.5, &mut SsiScratch::new())
            .expect("a crossing has two tangent directions");
        assert!(
            t.x().abs().min(t.y().abs()) < 1e-9,
            "tangent at the crossing must follow x = 0 or y = 0, got {t:?}"
        );

        let seed = refine_ssi_point(&s1, &s2, 0.5, 0.8, 0.5, 0.7, tol).unwrap();
        let (_, branches) = march_with_branches(
            &s1,
            &s2,
            &seed,
            0.05,
            tol,
            &OperationContext::new(),
            &mut SsiScratch::new(),
        )
        .unwrap();
        assert!(
            branches
                .iter()
                .any(|b| b.point.y().abs() < 1e-9 && b.point.x().abs() > 10.0 * tol),
            "crossing passed without a transverse branch: {branches:?}"
        );
    }

    /// Two perpendicular planes 2 cm across meet in a chord.
    /// Every grid pair is closer than the seeder's 0.1 floor, so it must refine
    /// them and return points of that chord.
    #[test]
    fn grid_seeding_finds_a_crossing_below_the_distance_floor() {
        let k = 0.01;
        let y0 = 0.3 * k;
        let a = plane_z0(-k, k, -k, k);
        // The plane y = y0 over x, z ∈ [-k, k].
        let b = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(-k, y0, -k), Point3::new(-k, y0, k)],
                vec![Point3::new(k, y0, -k), Point3::new(k, y0, k)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();

        let tol = 1e-9;
        let seeds = find_ssi_seeds_grid(&a, &b, 8, tol);
        assert!(
            !seeds.is_empty(),
            "the chord y = {y0}, z = 0 was not seeded"
        );
        for s in &seeds {
            let p = s.point;
            assert!(p.z().abs() <= tol && (p.y() - y0).abs() <= tol, "{p:?}");
            assert!(p.x().abs() <= k * (1.0 + 1e-12), "{p:?}");
        }
    }

    /// One Newton step on an affine patch lands exactly on the foot of the
    /// perpendicular: for `T = S(u*, v*) + h·n` the step from any `(u, v)` is
    /// `(u* − u, v* − v)`.
    #[test]
    fn surface_newton_step_on_an_affine_patch_hits_the_foot_exactly() {
        let o = Point3::new(1.0, -2.0, 0.5);
        let a = Vec3::new(3.0, 0.5, -1.0);
        let b = Vec3::new(-1.0, 2.0, 0.25);
        let at = |u: f64, v: f64| o + a * u + b * v;
        let patch = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![at(0.0, 0.0), at(0.0, 1.0)],
                vec![at(1.0, 0.0), at(1.0, 1.0)],
            ],
            vec![vec![1.0; 2]; 2],
        )
        .unwrap();
        let n = a.cross(b).normalize().unwrap();
        let mut scratch = DerivativeScratch::new();
        for &(us, vs, h) in &[(0.7, 0.2, 0.9), (0.15, 0.85, -2.0), (0.5, 0.5, 0.0)] {
            let target = at(us, vs) + n * h;
            for &(u0, v0) in &[(0.1, 0.1), (0.9, 0.4), (0.3, 0.95)] {
                let (du, dv) = surface_newton_step(&mut scratch, &patch, u0, v0, target);
                assert!((du - (us - u0)).abs() < 1e-12, "du {du} for {us}-{u0}");
                assert!((dv - (vs - v0)).abs() < 1e-12, "dv {dv} for {vs}-{v0}");
            }
        }
    }

    /// Iterated Newton steps on the exact cylinder converge to the closed-form
    /// closest point: the radial projection of the target onto the circle of
    /// its height. (Gauss-Newton on a curved surface converges linearly, at a
    /// rate near `|ρ − r| / r`, hence the generous iteration count.)
    #[test]
    fn surface_newton_step_iterates_to_the_cylinder_foot() {
        let (r, len) = (1.0, 4.0);
        let cyl = half_cylinder_on_plane(r, len);
        let mut scratch = DerivativeScratch::new();
        // Targets outside and inside the lower half-tube.
        for &(theta_deg, rho, y) in &[(200.0_f64, 1.3, 1.1), (300.0, 0.8, 3.2), (250.0, 1.6, 0.6)] {
            let theta = theta_deg.to_radians();
            let target = Point3::new(rho * theta.cos(), y, r + rho * theta.sin());
            let foot = Point3::new(r * theta.cos(), y, r + r * theta.sin());
            let (mut u, mut v) = (0.5, 0.5);
            for _ in 0..200 {
                let (du, dv) = surface_newton_step(&mut scratch, &cyl, u, v, target);
                u = (u + du).clamp(0.0, 1.0);
                v = (v + dv).clamp(0.0, 1.0);
            }
            let p = cyl.evaluate(u, v);
            assert!(
                (p - foot).length() < 1e-9,
                "θ={theta_deg}: {p:?} vs {foot:?}"
            );
        }
    }
}
// -- Spatial chaining equivalence (PERF-N03) --
//
// The grid + ring walk must reproduce the all-pairs + full-scan outcome
// bit-for-bit on every input shape: branches, loops, exact ties, dense
// balls, grid-aligned boundary hugs, mixed scales, non-finite points and
// degenerate thresholds. The oracle below is the verbatim pre-optimization
// algorithm; a seeded xorshift keeps the cases deterministic.

/// Verbatim pre-N03 chaining: all-pairs adjacency, `contains` endpoint
/// degrees, full-scan nearest-unused walk. Test-only oracle.
fn naive_chain(points: &[IntersectionPoint], threshold: f64) -> Vec<Vec<IntersectionPoint>> {
    if points.is_empty() {
        return Vec::new();
    }
    let n = points.len();
    let threshold_sq = threshold * threshold;
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            let d = points[i].point - points[j].point;
            if d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z())) < threshold_sq {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }
    let mut visited = vec![false; n];
    let mut components: Vec<Vec<usize>> = Vec::new();
    for start in 0..n {
        if visited[start] {
            continue;
        }
        let mut component = Vec::new();
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start);
        visited[start] = true;
        while let Some(idx) = queue.pop_front() {
            component.push(idx);
            for &neighbor in &adj[idx] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }
        components.push(component);
    }
    let mut chains = Vec::with_capacity(components.len());
    for comp in &components {
        if comp.is_empty() {
            continue;
        }
        let start_idx = comp
            .iter()
            .copied()
            .min_by_key(|&i| adj[i].iter().filter(|&&j| comp.contains(&j)).count())
            .unwrap_or(comp[0]);
        let mut chain = Vec::with_capacity(comp.len());
        let mut used = vec![false; n];
        let mut current = start_idx;
        used[current] = true;
        chain.push(points[current]);
        for _ in 1..comp.len() {
            let mut best_dist = f64::MAX;
            let mut best_idx = None;
            for &idx in comp {
                if used[idx] {
                    continue;
                }
                let d = points[current].point - points[idx].point;
                let dist_sq = d.x().mul_add(d.x(), d.y().mul_add(d.y(), d.z() * d.z()));
                if dist_sq < best_dist {
                    best_dist = dist_sq;
                    best_idx = Some(idx);
                }
            }
            if let Some(next) = best_idx {
                used[next] = true;
                chain.push(points[next]);
                current = next;
            } else {
                break;
            }
        }
        chains.push(chain);
    }
    chains
}

/// Deterministic xorshift64* uniform in [0, 1).
fn xorshift_unit(state: &mut u64) -> f64 {
    *state ^= *state >> 12;
    *state ^= *state << 25;
    *state ^= *state >> 27;
    ((*state).wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (u64::MAX >> 11) as f64
}

fn mk_point(x: f64, y: f64, z: f64, a: f64, b: f64) -> IntersectionPoint {
    IntersectionPoint {
        point: Point3::new(x, y, z),
        param1: (a, b),
        param2: (b, a),
    }
}

fn point_bits(p: &IntersectionPoint) -> (u64, u64, u64, u64, u64, u64, u64) {
    (
        p.point.x().to_bits(),
        p.point.y().to_bits(),
        p.point.z().to_bits(),
        p.param1.0.to_bits(),
        p.param1.1.to_bits(),
        p.param2.0.to_bits(),
        p.param2.1.to_bits(),
    )
}

/// Assert the shipped chaining matches the naive oracle bit-for-bit:
/// same components in the same order, same walk order, same carried
/// parameters. `-0.0` vs `0.0` and NaN payloads compare by bits.
fn check_chains_equivalent(points: &[IntersectionPoint], threshold: f64) {
    let expected = naive_chain(points, threshold);
    let actual = chain_intersection_points(points, threshold);
    assert_eq!(
        actual.len(),
        expected.len(),
        "component count differs (n={}, threshold={threshold})",
        points.len()
    );
    for (c, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
        assert_eq!(
            a.len(),
            e.len(),
            "chain {c} length differs (n={}, threshold={threshold})",
            points.len()
        );
        for (k, (pa, pe)) in a.iter().zip(e.iter()).enumerate() {
            assert_eq!(
                point_bits(pa),
                point_bits(pe),
                "chain {c} point {k} differs (n={}, threshold={threshold})",
                points.len()
            );
        }
    }
}

#[test]
fn spatial_chain_matches_oracle_on_straight_chains() {
    for n in [0usize, 1, 2, 3, 33, 200, 1500] {
        let points: Vec<IntersectionPoint> = (0..n)
            .map(|i| mk_point(i as f64, 0.0, 0.0, i as f64, 0.0))
            .collect();
        check_chains_equivalent(&points, 1.1);
    }
}

#[test]
fn spatial_chain_matches_oracle_on_loops_and_ties() {
    // Closed loop: every point has two equidistant neighbors.
    let ring: Vec<IntersectionPoint> = (0..300)
        .map(|i| {
            let a = i as f64 / 300.0 * std::f64::consts::TAU;
            mk_point(a.cos(), a.sin(), 0.0, a, 0.0)
        })
        .collect();
    check_chains_equivalent(&ring, 0.15);
    // Regular polygon around the walk start: exact distance ties between
    // symmetric unused candidates stress the first-in-component tie-break.
    let mut star = vec![mk_point(0.0, 0.0, 0.0, 0.0, 0.0)];
    for i in 0..12 {
        let a = i as f64 / 12.0 * std::f64::consts::TAU;
        star.push(mk_point(a.cos(), a.sin(), 0.0, a, 1.0));
    }
    check_chains_equivalent(&star, 1.05);
    // Dense ball: everything connects; the walk is pure ordering stress.
    let mut rng = 0x243F_6A88_85A3_08D3u64;
    let ball: Vec<IntersectionPoint> = (0..300)
        .map(|i| {
            let f = i as f64;
            mk_point(
                xorshift_unit(&mut rng) * 0.2,
                xorshift_unit(&mut rng) * 0.2,
                xorshift_unit(&mut rng) * 0.2,
                f,
                -f,
            )
        })
        .collect();
    check_chains_equivalent(&ball, 1.0);
}

#[test]
fn spatial_chain_matches_oracle_on_grids_and_clusters() {
    // Integer grid at threshold 1.1: cell boundaries (multiples of a
    // non-representable width) fall near integers, exercising boundary-hug
    // cell assignment.
    let mut grid = Vec::new();
    for x in 0..20 {
        for y in 0..20 {
            grid.push(mk_point(x as f64, y as f64, 0.0, x as f64, y as f64));
        }
    }
    check_chains_equivalent(&grid, 1.1);
    check_chains_equivalent(&grid, 1.5);
    // Sparse clusters with a thin bridge.
    let mut rng = 0xB529_7A4D_5280_4982u64;
    let mut cloud = Vec::new();
    for c in 0..10 {
        for _ in 0..30 {
            cloud.push(mk_point(
                c as f64 * 50.0 + xorshift_unit(&mut rng) * 2.0,
                xorshift_unit(&mut rng) * 2.0,
                xorshift_unit(&mut rng) * 2.0,
                c as f64,
                xorshift_unit(&mut rng),
            ));
        }
    }
    for i in 0..=40 {
        let t = i as f64 * 10.0;
        cloud.push(mk_point(t, 0.0, 0.0, t, t));
    }
    check_chains_equivalent(&cloud, 3.0);
}

#[test]
fn spatial_chain_matches_oracle_across_scales() {
    for scale in [1e-3f64, 1.0, 1e3] {
        let mut rng = 0x3333_3333_3333_3333u64 ^ scale.to_bits();
        for size in [7usize, 60, 300] {
            let points: Vec<IntersectionPoint> = (0..size)
                .map(|i| {
                    let f = i as f64;
                    mk_point(
                        xorshift_unit(&mut rng) * scale * 10.0,
                        xorshift_unit(&mut rng) * scale * 10.0,
                        xorshift_unit(&mut rng) * scale,
                        f,
                        -f,
                    )
                })
                .collect();
            // Threshold near the mean spacing, plus a sparse and a dense cut.
            check_chains_equivalent(&points, scale);
            check_chains_equivalent(&points, scale * 0.05);
            check_chains_equivalent(&points, scale * 30.0);
        }
    }
}

#[test]
fn spatial_chain_matches_oracle_on_degenerate_inputs() {
    let points: Vec<IntersectionPoint> = (0..12)
        .map(|i| mk_point(i as f64 * 0.5, 1.0, -1.0, i as f64, 0.5))
        .collect();
    // Zero, negative (squared comparison), NaN and infinite thresholds.
    check_chains_equivalent(&points, 0.0);
    check_chains_equivalent(&points, -0.0);
    check_chains_equivalent(&points, -1.0);
    check_chains_equivalent(&points, f64::NAN);
    check_chains_equivalent(&points, f64::INFINITY);
    check_chains_equivalent(&points, f64::NEG_INFINITY);
    check_chains_equivalent(&points, 1e-300);
    check_chains_equivalent(&points, 1e300);
    // Empty and singleton inputs at every degenerate threshold.
    for threshold in [0.0, 1.1, f64::NAN, f64::INFINITY] {
        check_chains_equivalent(&[], threshold);
        check_chains_equivalent(&points[..1], threshold);
    }
    // Non-finite coordinates mix with finite ones.
    let mut mixed = points;
    mixed.push(mk_point(f64::NAN, 0.0, 0.0, 0.0, 0.0));
    mixed.push(mk_point(f64::INFINITY, 1.0, 1.0, 1.0, 1.0));
    mixed.push(mk_point(0.0, 0.0, f64::NEG_INFINITY, 2.0, 2.0));
    mixed.push(mk_point(
        f64::INFINITY,
        f64::INFINITY,
        f64::INFINITY,
        3.0,
        3.0,
    ));
    check_chains_equivalent(&mixed, 1.1);
    check_chains_equivalent(&mixed, f64::INFINITY);
    // Coincident duplicates: zero-distance ties everywhere.
    let dupes: Vec<IntersectionPoint> = (0..40)
        .map(|i| mk_point(1.0, 2.0, 3.0, i as f64, 0.0))
        .collect();
    check_chains_equivalent(&dupes, 0.5);
    // Near-boundary adversarial: points a few ulps off multiples of the
    // threshold, where cell assignment could flip between neighbors.
    let cell = 1.1f64;
    let mut rng = 0xABCD_EF01_2345_6789u64;
    let hug: Vec<IntersectionPoint> = (0..300)
        .map(|i| {
            let k = (xorshift_unit(&mut rng) * 40.0).floor();
            let eps = (xorshift_unit(&mut rng) - 0.5) * 32.0 * f64::EPSILON * k.max(1.0) * cell;
            mk_point(k * cell + eps, 0.0, 0.0, i as f64, eps)
        })
        .collect();
    check_chains_equivalent(&hug, cell);
}
