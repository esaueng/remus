//! Structured fill for a trimmed sphere patch whose single boundary loop
//! winds once around the sphere's polar axis (the patch contains a pole).
//!
//! The parametric (u, v) chart cannot bound such a patch: its loop unwraps to
//! an open curve spanning a full u period, and the chord that closes it in
//! the CDT encloses the region between the curve and that chord instead of
//! the region between the curve and the pole row. That region is the
//! patch's complement within the hemisphere, wound so its shared edges run
//! the same direction as the neighbour's (B40: a cone–sphere fuse meshed the
//! retained sphere cap as the removed lens, leaving every section segment
//! one-sided at fine deflection and cancelled away by coincident-triangle
//! removal at coarse deflection).
//!
//! A loop winding once around the axis separates the two poles (Jordan curve
//! on the sphere). The patch therefore contains exactly one of them and
//! excludes the other, so stereographic projection from the excluded pole
//! maps the closed patch one-to-one onto a bounded planar region around the
//! origin. The loop is constrained verbatim in that chart. Only face-local
//! interior points pass through the inverse chart, so every shared boundary
//! vertex keeps its pool id. Chart coordinates use the unit direction of
//! each boundary sample, so a fitted section edge that sits slightly off the
//! sphere still charts to the right direction. No distance gate is applied.
//! The parametric CDT this path replaces accepts the same samples without
//! one.

use remus_math::det_hash::DetHashMap;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point2, Point3, Vec3};

use super::edge_sampling::segments_for_chord_deviation_a;
use super::nonplanar::validate_interior_grid_size;
use super::{MERGE_GRID, TriangleMesh, point_merge_key};

/// A boundary sample closer than this (radians, as `1 - cos`) to either pole
/// has no reliable azimuth, so the winding cannot be certified.
const POLE_CLEARANCE_COS: f64 = 1e-9;

/// The loop's azimuth winding must be this close to one full turn.
const WINDING_TOL: f64 = 1e-6;

/// Interior grid points closer than this fraction of the grid step to any
/// boundary chord are skipped. A point on or numerically at a constraint
/// would split the shared boundary with a face-local vertex that the
/// neighbouring face never sees.
const CLEARANCE_FRACTION: f64 = 0.25;

/// Fill a sphere patch bounded by one loop that winds once around the
/// sphere's polar axis.
///
/// `boundary` holds the loop as ordered global vertex ids (no closing
/// duplicate) in the face wire's stored traversal. That traversal keeps the
/// patch on its left as seen against the carrier's outward normal, so a
/// positive winding about `+z_axis` encloses the `+z_axis` pole, the same rule
/// the latitude-cap filler applies. Triangles are wound along the carrier's
/// outward normal; the caller flips them for a reversed face.
///
/// Returns `false` without emitting anything unless the loop certifiably
/// winds exactly once around the axis, keeps clear of both poles, and charts
/// to a simple polygon whose constrained triangulation succeeds.
#[allow(clippy::too_many_lines)]
pub(super) fn fill_sphere_pole_winding_patch(
    sphere: &SphericalSurface,
    boundary: &[u32],
    deflection: f64,
    angular_tol: f64,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> bool {
    let n = boundary.len();
    let radius = sphere.radius();
    if n < 3 || !radius.is_finite() || radius <= 0.0 {
        return false;
    }
    let center = sphere.center();
    let axis = sphere.z_axis();
    let e1 = sphere.x_axis();
    // Right-handed with `axis` by construction, whatever the frame's own
    // handedness.
    let e2 = axis.cross(e1);

    let mut directions: Vec<Vec3> = Vec::with_capacity(n);
    for &gid in boundary {
        let Some(&point) = merged.positions.get(gid as usize) else {
            return false;
        };
        let offset = point - center;
        let length = offset.length();
        if !length.is_finite() || length <= 0.0 {
            return false;
        }
        let direction = offset * (1.0 / length);
        if 1.0 - direction.dot(axis).abs() <= POLE_CLEARANCE_COS {
            return false;
        }
        directions.push(direction);
    }

    // Signed azimuth winding about `axis`. Each step must stay well under a
    // half turn, or the unwrap cannot tell which way the loop went.
    let azimuth = |d: Vec3| d.dot(e2).atan2(d.dot(e1));
    let mut total = 0.0;
    for i in 0..n {
        let a = azimuth(directions[i]);
        let b = azimuth(directions[(i + 1) % n]);
        let mut step = b - a;
        if step > std::f64::consts::PI {
            step -= std::f64::consts::TAU;
        } else if step <= -std::f64::consts::PI {
            step += std::f64::consts::TAU;
        }
        if step.abs() > std::f64::consts::FRAC_PI_2 {
            return false;
        }
        total += step;
    }
    let turns = total / std::f64::consts::TAU;
    let sign = if (turns - 1.0).abs() <= WINDING_TOL {
        1.0
    } else if (turns + 1.0).abs() <= WINDING_TOL {
        -1.0
    } else {
        return false;
    };

    // Chart centred on the enclosed pole `pole`, projected from `-pole`.
    // (c1, c2, pole) is right-handed, so a counter-clockwise chart triangle
    // maps to a triangle wound along the outward normal.
    let pole = axis * sign;
    let c1 = e1;
    let c2 = pole.cross(c1);
    let to_chart = |d: Vec3| -> Option<Point2> {
        let denominator = 1.0 + d.dot(pole);
        if !denominator.is_finite() || denominator <= 1e-12 {
            return None;
        }
        let point = Point2::new(d.dot(c1) / denominator, d.dot(c2) / denominator);
        (point.x().is_finite() && point.y().is_finite()).then_some(point)
    };
    let from_chart = |p: Point2| -> Vec3 {
        let radial_sq = p.x().mul_add(p.x(), p.y() * p.y());
        let denominator = 1.0 + radial_sq;
        c1 * (2.0 * p.x() / denominator)
            + c2 * (2.0 * p.y() / denominator)
            + pole * ((1.0 - radial_sq) / denominator)
    };

    let mut projected: Vec<Point2> = Vec::with_capacity(n);
    for &d in &directions {
        let Some(point) = to_chart(d) else {
            return false;
        };
        projected.push(point);
    }
    let (x_min, x_max, y_min, y_max) = projected.iter().fold(
        (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ),
        |(x_lo, x_hi, y_lo, y_hi), p| {
            (
                x_lo.min(p.x()),
                x_hi.max(p.x()),
                y_lo.min(p.y()),
                y_hi.max(p.y()),
            )
        },
    );
    // The enclosed pole charts to the origin, which must lie strictly inside
    // the charted loop's box.
    if !(x_min < 0.0 && x_max > 0.0 && y_min < 0.0 && y_max > 0.0) {
        return false;
    }

    // Stereographic arc length is `2 r / (1 + rho^2)` per chart unit, at most
    // `2 r` (at the pole). A chart step of half the target angle therefore
    // keeps every interior spacing at or under the target arc everywhere.
    let angular_segments =
        segments_for_chord_deviation_a(radius, std::f64::consts::PI, deflection, angular_tol, true)
            .max(2);
    #[allow(clippy::cast_precision_loss)]
    let step = std::f64::consts::PI / angular_segments as f64 * 0.5;
    if !step.is_finite() || step <= 0.0 {
        return false;
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n_x = ((x_max - x_min) / step).ceil().max(2.0) as usize;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n_y = ((y_max - y_min) / step).ceil().max(2.0) as usize;
    if validate_interior_grid_size(n_x, n_y).is_err() {
        return false;
    }

    let mut cdt = remus_math::cdt::Cdt::with_capacity(
        (
            Point2::new(x_min - step, y_min - step),
            Point2::new(x_max + step, y_max + step),
        ),
        n + n_x * n_y,
    );
    let Ok(boundary_ids) = cdt.insert_points_hilbert(&projected) else {
        return false;
    };
    let max_boundary_id = boundary_ids.iter().copied().max().unwrap_or(2);
    let mut cdt_to_global: Vec<Option<u32>> = vec![None; max_boundary_id + 1];
    for (&cdt_id, &global_id) in boundary_ids.iter().zip(boundary) {
        // Two boundary samples charting to one CDT vertex means the loop
        // touches itself in the chart: not a simple polygon.
        if cdt_to_global[cdt_id].replace(global_id).is_some() {
            return false;
        }
    }
    let boundary_pairs: Vec<(usize, usize)> = (0..n)
        .map(|i| (boundary_ids[i], boundary_ids[(i + 1) % n]))
        .collect();
    for &(a, b) in &boundary_pairs {
        if cdt.insert_constraint(a, b).is_err() {
            return false;
        }
    }

    let interior = interior_grid_points(&projected, (x_min, y_min), step, n_x, n_y);
    if !interior.is_empty() {
        let Ok(interior_ids) = cdt.insert_points_hilbert(&interior) else {
            return false;
        };
        let max_id = interior_ids.iter().copied().max().unwrap_or(0);
        if cdt_to_global.len() <= max_id {
            cdt_to_global.resize(max_id + 1, None);
        }
    }
    cdt.remove_exterior(&boundary_pairs);

    // Collect triangles before interning anything, so a decline leaves the
    // mesh untouched.
    let cdt_vertices = cdt.vertices().to_vec();
    let polygon: Vec<(f64, f64)> = projected.iter().map(|p| (p.x(), p.y())).collect();
    let mut kept: Vec<[usize; 3]> = Vec::new();
    for (a, b, c) in cdt.triangles() {
        let (pa, pb, pc) = (cdt_vertices[a], cdt_vertices[b], cdt_vertices[c]);
        let area =
            (pb.x() - pa.x()).mul_add(pc.y() - pa.y(), -(pb.y() - pa.y()) * (pc.x() - pa.x()));
        if !area.is_finite() || area.abs() < f64::MIN_POSITIVE {
            continue;
        }
        // `remove_exterior` can keep an ear across a concave run of nearly
        // cocircular constraints; the chart is bijective, so the centroid is
        // an exact inside/outside guard.
        let centroid = Point2::new(
            (pa.x() + pb.x() + pc.x()) / 3.0,
            (pa.y() + pb.y() + pc.y()) / 3.0,
        );
        if !super::nonplanar::point_in_polygon_2d(&polygon, centroid) {
            continue;
        }
        // Counter-clockwise in the chart is outward on the sphere.
        kept.push(if area > 0.0 { [a, b, c] } else { [a, c, b] });
    }
    if kept.is_empty() {
        return false;
    }

    let mut global_ids: Vec<Option<u32>> = vec![None; cdt_vertices.len()];
    let mut resolve = |index: usize,
                       merged: &mut TriangleMesh,
                       point_to_global: &mut DetHashMap<(i64, i64, i64), u32>|
     -> u32 {
        if let Some(id) = cdt_to_global.get(index).copied().flatten() {
            return id;
        }
        if let Some(id) = global_ids[index] {
            return id;
        }
        let normal = from_chart(cdt_vertices[index]);
        let point: Point3 = center + normal * radius;
        let key = point_merge_key(point, MERGE_GRID);
        let id = *point_to_global.entry(key).or_insert_with(|| {
            #[allow(clippy::cast_possible_truncation)]
            let id = merged.positions.len() as u32;
            merged.positions.push(point);
            merged.normals.push(normal);
            id
        });
        global_ids[index] = Some(id);
        id
    };
    for [a, b, c] in kept {
        let ga = resolve(a, merged, point_to_global);
        let gb = resolve(b, merged, point_to_global);
        let gc = resolve(c, merged, point_to_global);
        if ga == gb || gb == gc || ga == gc {
            continue;
        }
        merged.indices.extend_from_slice(&[ga, gb, gc]);
    }
    true
}

/// Interior grid points of the charted loop: strictly inside (even–odd
/// scanline) and at least `CLEARANCE_FRACTION * step` from every boundary
/// chord. Work is `O(rows * boundary + candidates * local chords)`, so a
/// dense rim does not multiply against a dense grid.
fn interior_grid_points(
    polygon: &[Point2],
    origin: (f64, f64),
    step: f64,
    n_x: usize,
    n_y: usize,
) -> Vec<Point2> {
    let n = polygon.len();
    let clearance = CLEARANCE_FRACTION * step;
    let cell_of = |value: f64, low: f64, count: usize| -> usize {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let cell = ((value - low) / step).floor().max(0.0) as usize;
        cell.min(count)
    };

    // Bucket every chord by the grid cells its clearance-padded box touches.
    let mut buckets: DetHashMap<(usize, usize), Vec<usize>> = DetHashMap::default();
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        let (lo_x, hi_x) = (a.x().min(b.x()) - clearance, a.x().max(b.x()) + clearance);
        let (lo_y, hi_y) = (a.y().min(b.y()) - clearance, a.y().max(b.y()) + clearance);
        for cx in cell_of(lo_x, origin.0, n_x)..=cell_of(hi_x, origin.0, n_x) {
            for cy in cell_of(lo_y, origin.1, n_y)..=cell_of(hi_y, origin.1, n_y) {
                buckets.entry((cx, cy)).or_default().push(i);
            }
        }
    }
    let near_boundary = |p: Point2, cx: usize, cy: usize| -> bool {
        buckets.get(&(cx, cy)).is_some_and(|chords| {
            chords.iter().any(|&i| {
                let (a, b) = (polygon[i], polygon[(i + 1) % n]);
                let ab = b - a;
                let length_sq = ab.dot(ab);
                let t = if length_sq > 0.0 {
                    ((p - a).dot(ab) / length_sq).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                (p - (a + ab * t)).length() < clearance
            })
        })
    };

    let mut points = Vec::new();
    let mut crossings: Vec<f64> = Vec::new();
    for iy in 1..n_y {
        #[allow(clippy::cast_precision_loss)]
        let y = origin.1 + step * iy as f64;
        crossings.clear();
        for i in 0..n {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            // Half-open rule: a vertex exactly on the scanline counts once.
            if (a.y() <= y) != (b.y() <= y) {
                let t = (y - a.y()) / (b.y() - a.y());
                crossings.push(t.mul_add(b.x() - a.x(), a.x()));
            }
        }
        crossings.sort_by(f64::total_cmp);
        for span in crossings.chunks_exact(2) {
            let (left, right) = (span[0], span[1]);
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let first = (((left - origin.0) / step).floor().max(0.0) as usize + 1).max(1);
            for ix in first..n_x {
                #[allow(clippy::cast_precision_loss)]
                let x = origin.0 + step * ix as f64;
                if x >= right {
                    break;
                }
                if x <= left {
                    continue;
                }
                let p = Point2::new(x, y);
                if !near_boundary(p, cell_of(x, origin.0, n_x), cell_of(y, origin.1, n_y)) {
                    points.push(p);
                }
            }
        }
    }
    points
}
