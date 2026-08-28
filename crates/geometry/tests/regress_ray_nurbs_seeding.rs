//! Ray-surface seeding must scale with the sample grid, not a corner diagonal.
//!
//! `intersect_line_nurbs` finds rough candidates on an n x n sample grid and
//! refines each with Newton. A sample was counted as a candidate when its
//! distance to the ray fell under `|corner_00 - corner_11| / n`.
//!
//! That quantity is not a spacing. It is the distance between two opposite
//! corners of the parameter domain, which says nothing about how dense the
//! samples actually are -- and for a surface that CLOSES, those two corners are
//! the same point, so it collapses to zero and lands on the `.max(0.1)` floor.
//! Measured: a torus seeded at 0.1 against a real sample spacing of 0.657 and
//! missed 74% of its intersections; a cylinder seeded at 0.500 against 1.567
//! and missed 30%. A box escaped only because its diagonal happened to come out
//! larger than its spacing (0.849 vs 0.632).
//!
//! The failure is silent: a missed intersection is an empty result, not an
//! error, and point-in-solid classification then counts the wrong parity.

#![allow(clippy::unwrap_used, clippy::panic)]

use remus_geometry::convert::surface_to_nurbs::{cylinder_to_nurbs, torus_to_nurbs};
use remus_math::nurbs::intersection::intersect_line_nurbs;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

/// A ray through the axis of a closed cylinder pierces the wall exactly twice.
#[test]
fn closed_cylinder_is_pierced_twice() {
    let cyl =
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0).unwrap();
    let nurbs = cylinder_to_nurbs(&cyl, (0.0, 10.0)).unwrap();

    for (k, dir) in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.6, 0.8, 0.0),
        Vec3::new(0.573_576_436_351_046, 0.819_152_044_288_991_8, 0.0),
    ]
    .iter()
    .enumerate()
    {
        let origin = Point3::new(0.0, 0.0, 5.0) - *dir * 20.0;
        let hits = intersect_line_nurbs(&nurbs, origin, *dir, 20).unwrap();
        assert_eq!(
            hits.len(),
            2,
            "dir {k}: a ray through the axis must pierce the wall twice, got {} -- the seed \
             threshold collapses on a closed surface",
            hits.len()
        );
        for h in &hits {
            let r = h.point.x().hypot(h.point.y());
            assert!(
                (r - 5.0).abs() < 1e-6,
                "dir {k}: hit at radius {r}, expected 5.0"
            );
        }
    }
}

/// A torus closes in BOTH directions, so its two domain corners coincide and
/// the old threshold fell all the way to its 0.1 floor -- its worst case.
#[test]
fn doubly_closed_torus_is_pierced_four_times() {
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
    let nurbs = torus_to_nurbs(&torus).unwrap();

    // In the torus's own plane, straight through both sides of the tube.
    let hits = intersect_line_nurbs(
        &nurbs,
        Point3::new(-20.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        20,
    )
    .unwrap();
    assert_eq!(
        hits.len(),
        4,
        "a ray through the plane of a torus crosses the tube four times, got {}",
        hits.len()
    );
}
