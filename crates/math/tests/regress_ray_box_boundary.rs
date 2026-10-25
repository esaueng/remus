//! Closed-box ray intersections must retain face, edge, and corner hits.

use remus_math::aabb::Aabb3;
use remus_math::bvh::Bvh;
use remus_math::vec::{Point3, Vec3};

#[test]
fn rays_parallel_to_box_faces_keep_boundary_hits() {
    let bounds = Aabb3 {
        min: Point3::new(0.0, 0.0, 0.0),
        max: Point3::new(1.0, 1.0, 1.0),
    };
    for axis in 0..3 {
        for side in [0.0, 1.0] {
            for zero in [0.0, -0.0] {
                for sign in [-1.0, 1.0] {
                    let mut origin = [side; 3];
                    origin[axis] = if sign > 0.0 { -1.0 } else { 2.0 };
                    let mut direction = [zero; 3];
                    direction[axis] = sign;
                    let origin = Point3::new(origin[0], origin[1], origin[2]);
                    let direction = Vec3::new(direction[0], direction[1], direction[2]);
                    let inverse = Vec3::new(
                        1.0 / direction.x(),
                        1.0 / direction.y(),
                        1.0 / direction.z(),
                    );
                    assert!(
                        bounds.ray_intersects(origin, inverse),
                        "{origin:?} {direction:?}"
                    );
                    let bvh = Bvh::build(&[(17, bounds)]);
                    assert_eq!(bvh.query_ray(origin, direction), vec![17]);
                }
            }
        }
    }
}

#[test]
fn ray_in_a_planar_box_is_not_culled() {
    let bounds = Aabb3 {
        min: Point3::new(0.0, 0.0, 0.0),
        max: Point3::new(1.0, 1.0, 0.0),
    };
    let bvh = Bvh::build(&[(17, bounds)]);
    assert_eq!(
        bvh.query_ray(Point3::new(-1.0, 0.5, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        vec![17]
    );
}

#[test]
#[allow(clippy::float_cmp)]
fn slab_queries_match_finite_direction_oracle() {
    let bounds = Aabb3 {
        min: Point3::new(0.0, 0.0, 0.0),
        max: Point3::new(1.0, 1.0, 1.0),
    };
    // Intersect intervals using the actual directions, without reciprocal
    // infinities or NaN-sensitive min/max. Include outward rays and misses.
    for x in [-1.0, 0.0, 0.5, 1.0, 2.0] {
        for y in [-1.0, 0.0, 0.5, 1.0, 2.0] {
            for z in [-1.0, 0.0, 0.5, 1.0, 2.0] {
                let origin = Point3::new(x, y, z);
                for dx in [-1.0, -0.0, 0.0, 1.0] {
                    for dy in [-1.0, -0.0, 0.0, 1.0] {
                        for dz in [-1.0, -0.0, 0.0, 1.0] {
                            let direction = Vec3::new(dx, dy, dz);
                            let mut enter = 0.0_f64;
                            let mut leave = f64::INFINITY;
                            for axis in 0..3 {
                                if direction.0[axis] == 0.0 {
                                    if !(0.0..=1.0).contains(&origin.0[axis]) {
                                        leave = -1.0;
                                    }
                                } else {
                                    let a = -origin.0[axis] / direction.0[axis];
                                    let b = (1.0 - origin.0[axis]) / direction.0[axis];
                                    enter = enter.max(a.min(b));
                                    leave = leave.min(a.max(b));
                                }
                            }
                            let inverse = Vec3::new(1.0 / dx, 1.0 / dy, 1.0 / dz);
                            assert_eq!(
                                bounds.ray_intersects(origin, inverse),
                                leave >= enter,
                                "{origin:?} {direction:?}"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn bvh_traversal_keeps_boundary_hits_in_multiple_leaves() {
    let left = Aabb3 {
        min: Point3::new(0.0, 0.0, 0.0),
        max: Point3::new(1.0, 1.0, 1.0),
    };
    let right = Aabb3 {
        min: Point3::new(1.0, 0.0, 0.0),
        max: Point3::new(2.0, 1.0, 1.0),
    };
    let bvh = Bvh::build(&[(17, left), (23, right)]);
    let mut hits = bvh.query_ray(Point3::new(3.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0));
    hits.sort_unstable();
    assert_eq!(hits, vec![17, 23]);
}

#[test]
fn bvh_preserves_nonzero_directions_whose_reciprocals_overflow() {
    for axis in 0..3 {
        for sign in [-1.0, 1.0] {
            for transverse in [0.0, 1.0] {
                let mut min = [0.0; 3];
                let mut max = [1.0; 3];
                let mut origin = [0.5; 3];
                let mut direction = [0.0; 3];
                min[axis] = if sign > 0.0 { 0.0 } else { -1.0 };
                max[axis] = if sign > 0.0 { 1.0 } else { 0.0 };
                origin[axis] = -sign * f64::from_bits(1);
                direction[axis] = sign * 5e-309;
                direction[(axis + 1) % 3] = transverse;
                let bounds = Aabb3 {
                    min: Point3::new(min[0], min[1], min[2]),
                    max: Point3::new(max[0], max[1], max[2]),
                };
                let origin = Point3::new(origin[0], origin[1], origin[2]);
                let direction = Vec3::new(direction[0], direction[1], direction[2]);
                let bvh = Bvh::build(&[(17, bounds)]);
                assert_eq!(bvh.query_ray(origin, direction), vec![17]);
                assert!(bvh.query_ray(origin, -direction).is_empty());
                if transverse > 0.0 {
                    let mut distant = origin;
                    distant.0[axis] = -sign;
                    assert!(bvh.query_ray(distant, direction).is_empty());
                }
            }
        }
    }
}
