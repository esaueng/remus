//! Robust triangle-triangle intersection: the narrow phase shared by mesh
//! co-refinement and validation's component-overlap check.
//!
//! Split out of `mesh_boolean` so validation does not depend on the mesh
//! boolean (RFC 0007 stage 0). Behavior is unchanged.

#![allow(clippy::tuple_array_conversions)]

use remus_math::predicates::orient3d;
use remus_math::vec::{Point2, Point3};

/// An intersection segment between a triangle of mesh A and one of mesh B.
///
/// `apply_a` / `apply_b` say which mesh's split must honor the segment as a
/// triangulation constraint. Transversal intersections apply to both;
/// coplanar-contact co-refinement segments apply to one side only (the
/// segment already runs along an existing edge of the other mesh).
#[derive(Debug, Clone)]
pub struct IsectSegment {
    pub p0: Point3,
    pub p1: Point3,
    pub tri_a: usize,
    pub tri_b: usize,
    pub apply_a: bool,
    pub apply_b: bool,
}

/// Compute the intersection segments between two triangles.
///
/// Transversal case: Moller interval overlap on the intersection line of the
/// two planes, using `orient3d` for the side classification. Coplanar case:
/// mutual 2D edge clipping so each mesh conforms to the other's edges inside
/// the shared plane. Returns an empty vec if the triangles do not intersect
/// or only touch at a point.
#[allow(clippy::similar_names)]
pub fn intersect_triangles(
    a0: Point3,
    a1: Point3,
    a2: Point3,
    b0: Point3,
    b1: Point3,
    b2: Point3,
    tolerance: f64,
) -> Vec<IsectSegment> {
    // Classify vertices of B against the plane of A. orient3d scales with
    // the host triangle's area, so normalize to a distance before comparing
    // against the linear tolerance — otherwise a large host makes on-plane
    // vertices read as genuinely off-plane (skipping the grazing imprint) and
    // a tiny host makes off-plane vertices read as touching.
    let na_pre = (a1 - a0).cross(a2 - a0);
    let nb_pre = (b1 - b0).cross(b2 - b0);
    let na_len = na_pre.length().max(1e-30);
    let nb_len = nb_pre.length().max(1e-30);
    let db0 = orient3d(a0, a1, a2, b0) / na_len;
    let db1 = orient3d(a0, a1, a2, b1) / na_len;
    let db2 = orient3d(a0, a1, a2, b2) / na_len;

    if all_same_sign(db0, db1, db2, tolerance) {
        // Grazing contact: B lies on one side but touches A's plane along an
        // edge (two on-plane vertices). That edge is a region boundary for
        // the coincident-band classification — without imprinting it, A's
        // triangles straddle the boundary and the kept/dropped halves of one
        // quad leave open edges (a stacking lip's bottom annulus grazing the
        // body wall plane at z=13.3 was the lite fallback's bd=103).
        return grazing_imprint([a0, a1, a2], [b0, b1, b2], [db0, db1, db2], true, tolerance);
    }

    // Classify vertices of A against the plane of B (distance-normalized,
    // as above).
    let da0 = orient3d(b0, b1, b2, a0) / nb_len;
    let da1 = orient3d(b0, b1, b2, a1) / nb_len;
    let da2 = orient3d(b0, b1, b2, a2) / nb_len;

    if all_same_sign(da0, da1, da2, tolerance) {
        // Mirror grazing case: A touches B's plane along an edge.
        return grazing_imprint(
            [b0, b1, b2],
            [a0, a1, a2],
            [da0, da1, da2],
            false,
            tolerance,
        );
    }

    let na = (a1 - a0).cross(a2 - a0);
    let nb = (b1 - b0).cross(b2 - b0);
    let line_dir = na.cross(nb);

    // Coplanar check: when |na × nb| ≈ 0, the triangles lie in the same plane.
    let line_len_sq = line_dir.dot(line_dir);
    let na_len_sq = na.dot(na);
    let nb_len_sq = nb.dot(nb);
    // sin²(angle) threshold: tolerance² for angular comparison
    if line_len_sq < (tolerance * tolerance) * na_len_sq.max(nb_len_sq) {
        return coplanar_corefine_segments([a0, a1, a2], [b0, b1, b2], tolerance);
    }

    // Project onto the axis with the largest component for numerical stability.
    let ax = line_dir.x().abs();
    let ay = line_dir.y().abs();
    let az = line_dir.z().abs();

    let project = |p: Point3| -> f64 {
        if ax >= ay && ax >= az {
            p.x()
        } else if ay >= az {
            p.y()
        } else {
            p.z()
        }
    };

    let Some((ta_min, ta_max)) = triangle_interval(a0, a1, a2, da0, da1, da2, &project) else {
        return Vec::new();
    };
    let Some((tb_min, tb_max)) = triangle_interval(b0, b1, b2, db0, db1, db2, &project) else {
        return Vec::new();
    };

    let t_lo = ta_min.max(tb_min);
    let t_hi = ta_max.min(tb_max);

    if t_hi - t_lo < tolerance {
        return Vec::new();
    }

    let tri_data = TriPlaneData {
        v: [a0, a1, a2],
        d: [da0, da1, da2],
    };
    let p0 = point_on_intersection_line(&tri_data, t_lo, &project);
    let p1 = point_on_intersection_line(&tri_data, t_hi, &project);

    vec![IsectSegment {
        p0,
        p1,
        tri_a: 0,
        tri_b: 0,
        apply_a: true,
        apply_b: true,
    }]
}

/// Returns whether two tessellated triangles have a non-empty intersection.
pub fn triangles_intersect(a: [Point3; 3], b: [Point3; 3], tolerance: f64) -> bool {
    !intersect_triangles(a[0], a[1], a[2], b[0], b[1], b[2], tolerance).is_empty()
}

/// Imprint a grazing contact: `touching`'s edge that lies in `host`'s plane
/// (both endpoint orientations within tolerance) is clipped to `host`'s
/// triangle and emitted as a MUTUAL constraint — the host splits its
/// interior along it, and the touching mesh splits its own on-plane edge at
/// the same canonical points, so the coincident-band classification boundary
/// falls on welded triangle edges instead of cutting through interiors.
fn grazing_imprint(
    host: [Point3; 3],
    touching: [Point3; 3],
    touching_orients: [f64; 3],
    host_is_a: bool,
    tolerance: f64,
) -> Vec<IsectSegment> {
    // `touching_orients` arrive distance-normalized from the caller.
    let n = (host[1] - host[0]).cross(host[2] - host[0]);
    if n.length() < 1e-30 {
        return Vec::new();
    }
    let on_plane: Vec<usize> = (0..3)
        .filter(|&i| touching_orients[i].abs() <= tolerance)
        .collect();
    if on_plane.len() != 2 {
        return Vec::new();
    }
    let p0 = touching[on_plane[0]];
    let p1 = touching[on_plane[1]];

    // Project into the host plane's dominant axes and clip to the host
    // triangle (the same frame `coplanar_corefine_segments` uses).
    let nax = n.x().abs();
    let nay = n.y().abs();
    let naz = n.z().abs();
    let to_2d = |p: Point3| -> Point2 {
        if naz >= nax && naz >= nay {
            Point2::new(p.x(), p.y())
        } else if nay >= nax {
            Point2::new(p.x(), p.z())
        } else {
            Point2::new(p.y(), p.z())
        }
    };
    let host2d = [to_2d(host[0]), to_2d(host[1]), to_2d(host[2])];
    let Some((t0, t1)) = clip_segment_to_triangle_2d(to_2d(p0), to_2d(p1), &host2d, tolerance)
    else {
        return Vec::new();
    };
    let e = p1 - p0;
    if (t1 - t0) * e.length() < tolerance * 2.0 {
        return Vec::new();
    }
    // Mutual: the host splits its interior along the segment, and the
    // touching mesh splits its own on-plane edge at the same canonical clip
    // points (via the splitter's global on-edge point map) — one-sided
    // imprinting leaves fresh T-junctions between the host's new vertices
    // and the touching edge's own subdivision.
    let _ = host_is_a;
    vec![IsectSegment {
        p0: lerp_point(p0, p1, t0),
        p1: lerp_point(p0, p1, t1),
        tri_a: 0,
        tri_b: 0,
        apply_a: true,
        apply_b: true,
    }]
}

/// Co-refinement segments for a coplanar triangle pair.
///
/// Each edge of B clipped to the interior of A becomes a constraint for A's
/// re-triangulation (and vice versa), so both meshes end up conforming to
/// each other's edges inside the shared plane. The clip endpoints are the
/// mutual edge-edge crossing points, which lie on triangle edges of BOTH
/// meshes; the splitter's edge-point propagation carries them onto the
/// neighbors sharing those edges.
fn coplanar_corefine_segments(
    a3d: [Point3; 3],
    b3d: [Point3; 3],
    tolerance: f64,
) -> Vec<IsectSegment> {
    let na = (a3d[1] - a3d[0]).cross(a3d[2] - a3d[0]);
    let nax = na.x().abs();
    let nay = na.y().abs();
    let naz = na.z().abs();

    let to_2d = |p: Point3| -> Point2 {
        if naz >= nax && naz >= nay {
            Point2::new(p.x(), p.y())
        } else if nay >= nax {
            Point2::new(p.x(), p.z())
        } else {
            Point2::new(p.y(), p.z())
        }
    };

    let a2d = [to_2d(a3d[0]), to_2d(a3d[1]), to_2d(a3d[2])];
    let b2d = [to_2d(b3d[0]), to_2d(b3d[1]), to_2d(b3d[2])];

    let mut out = Vec::new();
    // B's edges clipped to A constrain mesh A's triangle.
    clip_edges_into(&b2d, &b3d, &a2d, tolerance, true, &mut out);
    // A's edges clipped to B constrain mesh B's triangle.
    clip_edges_into(&a2d, &a3d, &b2d, tolerance, false, &mut out);
    out
}

/// Clip each edge of `src` against the triangle `clip` (2D), emitting the
/// interior portions as constraint segments for one mesh side.
fn clip_edges_into(
    src2d: &[Point2; 3],
    src3d: &[Point3; 3],
    clip2d: &[Point2; 3],
    tolerance: f64,
    for_mesh_a: bool,
    out: &mut Vec<IsectSegment>,
) {
    for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
        let Some((t0, t1)) = clip_segment_to_triangle_2d(src2d[i], src2d[j], clip2d, tolerance)
        else {
            continue;
        };
        let e3d = src3d[j] - src3d[i];
        let len = e3d.length();
        if (t1 - t0) * len < tolerance * 2.0 {
            continue;
        }
        let p0 = lerp_point(src3d[i], src3d[j], t0);
        let p1 = lerp_point(src3d[i], src3d[j], t1);
        out.push(IsectSegment {
            p0,
            p1,
            tri_a: 0,
            tri_b: 0,
            apply_a: for_mesh_a,
            apply_b: !for_mesh_a,
        });
    }
}

/// Clip the parametric segment `e0 + t*(e1-e0)`, t in [0,1], against a 2D
/// triangle (any winding). Returns the surviving parameter interval, or
/// `None` when the segment misses the triangle.
fn clip_segment_to_triangle_2d(
    e0: Point2,
    e1: Point2,
    tri: &[Point2; 3],
    tolerance: f64,
) -> Option<(f64, f64)> {
    // Orient the triangle CCW so the inward side of each edge is cross >= 0.
    let signed2 = (tri[1].x() - tri[0].x()) * (tri[2].y() - tri[0].y())
        - (tri[1].y() - tri[0].y()) * (tri[2].x() - tri[0].x());
    if signed2.abs() < 1e-30 {
        return None;
    }
    let (o0, o1, o2) = if signed2 > 0.0 {
        (tri[0], tri[1], tri[2])
    } else {
        (tri[0], tri[2], tri[1])
    };

    let mut t_lo = 0.0_f64;
    let mut t_hi = 1.0_f64;
    for (p, q) in [(o0, o1), (o1, o2), (o2, o0)] {
        let ex = q.x() - p.x();
        let ey = q.y() - p.y();
        let elen = ex.hypot(ey);
        if elen < 1e-30 {
            return None;
        }
        // Signed distance of the segment endpoints from this clip edge
        // (positive = inside for a CCW triangle), in true distance units.
        let d0 = (ex * (e0.y() - p.y()) - ey * (e0.x() - p.x())) / elen;
        let d1 = (ex * (e1.y() - p.y()) - ey * (e1.x() - p.x())) / elen;
        let eps = tolerance;
        if d0 < -eps && d1 < -eps {
            return None;
        }
        if d0 >= -eps && d1 >= -eps {
            continue; // fully inside this half-plane
        }
        // Crossing: split at d == 0.
        let t = d0 / (d0 - d1);
        if d0 < -eps {
            t_lo = t_lo.max(t);
        } else {
            t_hi = t_hi.min(t);
        }
    }
    if t_hi <= t_lo {
        return None;
    }
    Some((t_lo, t_hi))
}

/// Check if three signed distances are all on the same side (all positive or all negative).
fn all_same_sign(d0: f64, d1: f64, d2: f64, tolerance: f64) -> bool {
    let pos = d0 > tolerance || d1 > tolerance || d2 > tolerance;
    let neg = d0 < -tolerance || d1 < -tolerance || d2 < -tolerance;
    // All non-negative or all non-positive (accounting for tolerance).
    (d0 >= -tolerance && d1 >= -tolerance && d2 >= -tolerance && pos && !neg)
        || (d0 <= tolerance && d1 <= tolerance && d2 <= tolerance && neg && !pos)
}

/// Compute the parameter interval of a triangle on the intersection line.
///
/// The three distances `d0, d1, d2` are the signed distances of each vertex
/// from the other triangle's plane. The `project` function maps a 3D point
/// to a scalar on the dominant axis of the intersection line.
fn triangle_interval(
    v0: Point3,
    v1: Point3,
    v2: Point3,
    d0: f64,
    d1: f64,
    d2: f64,
    project: &dyn Fn(Point3) -> f64,
) -> Option<(f64, f64)> {
    let p0 = project(v0);
    let p1 = project(v1);
    let p2 = project(v2);

    // Find the lone vertex (the one on the opposite side from the other two).
    // Compute the two intersection points where the triangle crosses the plane.
    let (t0, t1) = if (d0 > 0.0) != (d1 > 0.0) && (d0 > 0.0) != (d2 > 0.0) {
        // v0 is alone
        let ta = interp_param(p0, p1, d0, d1);
        let tb = interp_param(p0, p2, d0, d2);
        (ta, tb)
    } else if (d1 > 0.0) != (d0 > 0.0) && (d1 > 0.0) != (d2 > 0.0) {
        // v1 is alone
        let ta = interp_param(p1, p0, d1, d0);
        let tb = interp_param(p1, p2, d1, d2);
        (ta, tb)
    } else if (d2 > 0.0) != (d0 > 0.0) && (d2 > 0.0) != (d1 > 0.0) {
        // v2 is alone
        let ta = interp_param(p2, p0, d2, d0);
        let tb = interp_param(p2, p1, d2, d1);
        (ta, tb)
    } else {
        // Degenerate: one or more vertices are on the plane.
        // Find the two vertices that straddle or lie on the plane.
        let mut ts = Vec::new();
        if d0.abs() < 1e-15 {
            ts.push(p0);
        }
        if d1.abs() < 1e-15 {
            ts.push(p1);
        }
        if d2.abs() < 1e-15 {
            ts.push(p2);
        }
        // Also check edges that cross the plane.
        if d0 * d1 < 0.0 {
            ts.push(interp_param(p0, p1, d0, d1));
        }
        if d1 * d2 < 0.0 {
            ts.push(interp_param(p1, p2, d1, d2));
        }
        if d0 * d2 < 0.0 {
            ts.push(interp_param(p0, p2, d0, d2));
        }

        if ts.len() < 2 {
            return None;
        }

        let mut lo = ts[0];
        let mut hi = ts[0];
        for &t in &ts[1..] {
            if t < lo {
                lo = t;
            }
            if t > hi {
                hi = t;
            }
        }
        (lo, hi)
    };

    let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
    Some((lo, hi))
}

/// Interpolate to find the parameter on the intersection line where an edge
/// crosses the plane.
fn interp_param(p_a: f64, p_b: f64, d_a: f64, d_b: f64) -> f64 {
    let denom = d_a - d_b;
    if denom.abs() < 1e-30 {
        0.5 * (p_a + p_b)
    } else {
        (p_b - p_a).mul_add(d_a / denom, p_a)
    }
}

/// Triangle vertices and their signed distances from a plane.
struct TriPlaneData {
    v: [Point3; 3],
    d: [f64; 3],
}

/// Reconstruct a 3D point on the intersection line given a parameter value
/// along the dominant axis.
///
/// Finds the two edges of the triangle that cross the other triangle's plane
/// and interpolates to find the 3D point whose projection equals `t_target`.
fn point_on_intersection_line(
    tri: &TriPlaneData,
    t_target: f64,
    project: &dyn Fn(Point3) -> f64,
) -> Point3 {
    let [v0, v1, v2] = tri.v;
    let [d0, d1, d2] = tri.d;
    // Collect the 3D points where triangle edges cross the plane.
    let mut crossing_points: Vec<Point3> = Vec::with_capacity(2);
    let mut crossing_params: Vec<f64> = Vec::with_capacity(2);

    let edges = [(v0, v1, d0, d1), (v1, v2, d1, d2), (v0, v2, d0, d2)];

    for &(va, vb, da, db) in &edges {
        if da.abs() < 1e-15 && db.abs() < 1e-15 {
            // Both on the plane: add both endpoints.
            crossing_points.push(va);
            crossing_params.push(project(va));
            crossing_points.push(vb);
            crossing_params.push(project(vb));
        } else if da.abs() < 1e-15 {
            crossing_points.push(va);
            crossing_params.push(project(va));
        } else if db.abs() < 1e-15 {
            crossing_points.push(vb);
            crossing_params.push(project(vb));
        } else if da * db < 0.0 {
            let t = da / (da - db);
            let p = lerp_point(va, vb, t);
            crossing_points.push(p);
            crossing_params.push(project(p));
        }
    }

    if crossing_points.len() < 2 {
        // Fallback: return first crossing point or centroid.
        return crossing_points
            .first()
            .copied()
            .unwrap_or_else(|| triangle_centroid(v0, v1, v2));
    }

    // Interpolate between the two crossing points to hit t_target.
    let t0 = crossing_params[0];
    let t1 = crossing_params[1];
    let denom = t1 - t0;
    if denom.abs() < 1e-30 {
        crossing_points[0]
    } else {
        let s = (t_target - t0) / denom;
        lerp_point(crossing_points[0], crossing_points[1], s)
    }
}

/// Linear interpolation between two points.
pub fn lerp_point(a: Point3, b: Point3, t: f64) -> Point3 {
    Point3::new(
        (b.x() - a.x()).mul_add(t, a.x()),
        (b.y() - a.y()).mul_add(t, a.y()),
        (b.z() - a.z()).mul_add(t, a.z()),
    )
}

/// Centroid of a triangle.
pub fn triangle_centroid(v0: Point3, v1: Point3, v2: Point3) -> Point3 {
    Point3::new(
        (v0.x() + v1.x() + v2.x()) / 3.0,
        (v0.y() + v1.y() + v2.y()) / 3.0,
        (v0.z() + v1.z() + v2.z()) / 3.0,
    )
}
