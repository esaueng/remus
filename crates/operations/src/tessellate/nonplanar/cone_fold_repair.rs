//! Turned-over triangles of a cone face's CDT, the interior vertices to
//! triangulate again without, and the deviation gate a rebuilt mesh must pass
//! (B289).
//!
//! The non-planar CDT triangulates a cone face in its `(u, v)` chart, with
//! `v` scaled. That chart is no development of the cone: a straight chart
//! chord between two boundary samples maps to a curve beside the straight 3D
//! chord between them. Along a rim the 3D chord runs inside the cone by the
//! circular sagitta, which is `sagitta · cos a` in slant, so near the far rim
//! of a face it cuts a lens off the chart: an interior point inside that lens
//! is inside the chart chord but past the 3D one, and the triangle the CDT
//! builds from the chord and that point turns over in space. On a band
//! narrower than the lens every interior point is inside it.
//!
//! Detection is a sign test only ([`check`]): a triangle whose normal does
//! not agree in sign with the cone's chart normal at its centroid turned
//! over. It runs on the triangle list emission reads anyway, so a face with
//! no turned-over triangle costs the scan and nothing else, and is emitted
//! exactly as the CDT made it. For each turned-over triangle the interior
//! vertex nearest its opposite edge in the chart (the one it is squashed
//! onto) is dropped and the CDT is rebuilt from scratch without it, round by
//! round (at most [`MAX_ROUNDS`]; a round that turns more over ends the
//! repair). Boundary, hole and constraint-recovery vertices are never
//! dropped, so the face's boundary, and the watertight seam with its
//! neighbours, is unchanged.
//!
//! The gate makes the repair never worse: a rebuilt mesh replaces the CDT's
//! first one only when it turns strictly fewer triangles over and every
//! triangle it does not share with the first mesh stays within the
//! deflection of the cone by [`deviation_bound`], an upper bound on the
//! triangle's distance from the cone, not a sample of it. Triangles it
//! shares are the CDT's own, kept as they were: the CDT's first mesh may
//! itself lie a little past the deflection (a thread taper's cone at 0.05
//! reads 1.02 of it on a triangle no repair touches), and that is not the
//! repair's to refuse. Dropping vertices merges triangles into larger
//! ones, which can sag far past the deflection (a narrow band left as a
//! facet across its whole span); such a repair is refused and the face keeps
//! the CDT's first mesh. Triangles made of boundary samples alone, which the
//! boundary forces, are left as they are.

use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point2, Point3, Vec3};

#[cfg(test)]
thread_local! {
    /// Set by [`without_repair`]: the repair is skipped, and a cone face
    /// keeps the CDT's first mesh (the plain mesh the tests compare with).
    static DISABLED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Runs `f` with the repair skipped on this thread (tests only).
#[cfg(test)]
pub(in crate::tessellate) fn without_repair<R>(f: impl FnOnce() -> R) -> R {
    DISABLED.with(|flag| flag.set(true));
    let result = f();
    DISABLED.with(|flag| flag.set(false));
    result
}

/// Whether the repair runs: always, except under [`without_repair`].
pub(super) fn enabled() -> bool {
    #[cfg(test)]
    {
        !DISABLED.with(std::cell::Cell::get)
    }
    #[cfg(not(test))]
    {
        true
    }
}

/// Re-triangulation rounds a face's repair may take. Each round drops at
/// least one interior vertex of every turned-over triangle that has one;
/// faces seen so far settle in one or two.
pub(super) const MAX_ROUNDS: usize = 16;

/// The cone's chart normal `S_u × S_v` direction at the point of `cone`
/// nearest `point`: the outward normal on the `v > 0` nappe, its opposite
/// below the apex (where `S_u` turns over with `v`). The CDT emits its
/// triangles counter-clockwise in the chart, so they face this way; a
/// reversed face is flipped after emission, as a whole.
fn chart_normal(cone: &ConicalSurface, point: Point3) -> Vec3 {
    // `project_point` reads a point below the apex at its own longitude;
    // the chart reaches it half a turn round, on the line through it.
    let (u, v) = cone.project_point(point);
    if v < 0.0 {
        -cone.normal(u + std::f64::consts::PI, 0.0)
    } else {
        cone.normal(u, 0.0)
    }
}

/// Whether the triangle `p` turns over against `normal` (a unit vector):
/// its normal's component along `normal` is negative, or within rounding of
/// zero. The floor is relative to the edge lengths, so the verdict does not
/// depend on the triangle's size or its distance from the origin.
pub(super) fn turns_over(p: [Point3; 3], normal: Vec3) -> bool {
    let (e1, e2) = (p[1] - p[0], p[2] - p[0]);
    let along = e1.cross(e2).dot(normal);
    // `along` sums products of edge components; each carries a few
    // roundings of `|e1| |e2|`.
    along <= 64.0 * f64::EPSILON * e1.length() * e2.length()
}

/// The chart height of `apex` over the line through `a` and `b`, squared
/// and scaled by `|b − a|²` (twice the triangle's area, squared): smallest
/// for the vertex a turned-over triangle is squashed onto.
fn squash(apex: Point2, a: Point2, b: Point2) -> f64 {
    let (ex, ey) = (b.x() - a.x(), b.y() - a.y());
    let twice_area = ex.mul_add(apex.y() - a.y(), -(ey * (apex.x() - a.x())));
    let length_sq = ex.mul_add(ex, ey * ey);
    if length_sq > 0.0 {
        twice_area * twice_area / length_sq
    } else {
        0.0
    }
}

/// Whether the triangle `p`, its vertices on `cone`, turns over against the
/// cone's chart normal at its centroid ([`turns_over`]).
pub(super) fn turned_over(cone: &ConicalSurface, p: [Point3; 3]) -> bool {
    let centroid = Point3::new(
        (p[0].x() + p[1].x() + p[2].x()) / 3.0,
        (p[0].y() + p[1].y() + p[2].y()) / 3.0,
        (p[0].z() + p[1].z() + p[2].z()) / 3.0,
    );
    turns_over(p, chart_normal(cone, centroid))
}

/// The check of one triangulation: how many of `triangles` (CDT ids into
/// `chart`) that emission keeps turned over, and the interior vertices to
/// triangulate again without. Triangles with a vertex of the CDT's super
/// triangle (ids `0..3`) are skipped; `emitted` (the hole test) is asked
/// only of turned-over ones. For every turned-over triangle with a vertex
/// `removable` answers for, that is the one nearest its opposite edge in the
/// chart, the one it is squashed onto (ties to the smaller id). Sorted and
/// unique; empty, and never allocated, when no triangle turned over.
pub(super) fn check(
    cone: &ConicalSurface,
    triangles: &[(usize, usize, usize)],
    chart: &[Point2],
    position: impl Fn(usize) -> Point3,
    removable: impl Fn(usize) -> Option<usize>,
    emitted: impl Fn((usize, usize, usize)) -> bool,
) -> (usize, Vec<usize>) {
    let mut count = 0;
    let mut drop = Vec::new();
    for &(i0, i1, i2) in triangles {
        let ids = [i0, i1, i2];
        if ids.iter().any(|&id| id < 3)
            || !turned_over(cone, ids.map(&position))
            || !emitted((i0, i1, i2))
        {
            continue;
        }
        count += 1;
        let pick = (0..3)
            .filter_map(|k| removable(ids[k]).map(|id| (k, id)))
            .map(|(k, id)| {
                let height = squash(chart[id], chart[ids[(k + 1) % 3]], chart[ids[(k + 2) % 3]]);
                (height, id)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        if let Some((_, id)) = pick {
            drop.push(id);
        }
    }
    remus_math::compact_sort::CompactSort::compact_sort_unstable(drop.as_mut_slice());
    drop.dedup();
    (count, drop)
}

/// A triangle by its vertices' position bits, in a fixed order: equal for
/// the same three points whichever vertex the CDT lists first.
pub(super) fn triangle_key(p: [Point3; 3]) -> [[u64; 3]; 3] {
    let mut key = p.map(|q| [q.x().to_bits(), q.y().to_bits(), q.z().to_bits()]);
    if key[1] < key[0] {
        key.swap(0, 1);
    }
    if key[2] < key[1] {
        key.swap(1, 2);
    }
    if key[1] < key[0] {
        key.swap(0, 1);
    }
    key
}

/// Barycentric grid divisions of a triangle in [`deviation_bound`].
pub(in crate::tessellate) const GRID: usize = 6;

/// Interior samples of each edge in [`deviation_bound`], besides the grid's.
pub(in crate::tessellate) const EDGE_SAMPLES: usize = 8;

/// An upper bound on how far the flat triangle `p`, its vertices on `cone`,
/// lies from the cone.
///
/// Let `w` be a point less the apex, `z = w · axis`, `r` its distance from
/// the axis, `a` the half-angle and `s` the sign of the nappe the
/// triangle's centroid is on. `g = r sin a − s z cos a` is the signed
/// distance from the line of the generator through the point, in its
/// meridian plane, and `|g|` bounds the distance from the cone. `r` is a
/// convex function of position (a norm of a linear map), so `g` is convex
/// on the triangle; it is zero at the vertices, so it is at most zero
/// inside, and where `r > 0` the point's generator foot lies on the
/// triangle's nappe, so `|g|` is the distance from that nappe there.
///
/// `|g|` is read at the `(GRID + 1)(GRID + 2)/2` points of a barycentric
/// grid of `GRID` divisions and at `EDGE_SAMPLES` more points along each
/// edge. Between grid points the bound adds an interpolation error: on a
/// grid sub-triangle with vertices `v_i`, `f(x) − Σ λ_i f(v_i) =
/// −½ Σ λ_i (v_i − x)ᵀ H(ξ_i) (v_i − x)` for points `ξ_i` of the triangle,
/// and the Hessian of `g` is `sin a / r` times the projection onto the
/// circumferential direction `t̂(ξ)` (the Hessian of `r`). In the frame of
/// the centroid's radial and circumferential directions `r̂₀`, `t̂₀`, and
/// with `Δ` the largest angle about the axis from the centroid to a vertex
/// (a convex triangle clear of the axis spans the angles of its vertices),
/// `|t̂(ξ) · d| ≤ |t̂₀ · d| + |r̂₀ · d| sin Δ`; a sub-triangle is the triangle
/// scaled by `1 / GRID`, so `|t̂₀ · d| ≤ T / GRID` and `|r̂₀ · d| ≤ R / GRID`
/// for the triangle's extents `T`, `R` along `t̂₀`, `r̂₀`. The term is
/// `½ (sin a / r_min) ((T + R sin Δ) / GRID)²`: small for a triangle long
/// along its generators, where the cone does not curve. `r_min` is bounded
/// below by the tangent planes of the convex `r` at the vertices (each a
/// lower bound of `r` on the whole triangle, its minimum at a vertex). A
/// triangle that may reach the axis (`r_min ≤ 0`) has no bound: infinity.
/// The rounding of the sample points adds `64 ε` of the coordinates' size.
pub(super) fn deviation_bound(cone: &ConicalSurface, p: [Point3; 3]) -> f64 {
    let (sampled, between) = deviation_parts(cone, p);
    sampled + between
}

/// [`deviation_bound`]'s sampled maximum and the allowance it adds for the
/// points between samples (curvature and rounding).
pub(super) fn deviation_parts(cone: &ConicalSurface, p: [Point3; 3]) -> (f64, f64) {
    let apex = cone.apex();
    let axis = cone.axis();
    let (sin_a, cos_a) = cone.half_angle().sin_cos();
    let centroid = p[0] + ((p[1] - p[0]) + (p[2] - p[0])) * (1.0 / 3.0);
    let nappe = if (centroid - apex).dot(axis) < 0.0 {
        -1.0
    } else {
        1.0
    };
    let split = |q: Point3| {
        let w = q - apex;
        let z = w.dot(axis);
        let radial = w - axis * z;
        (z, radial)
    };
    let g = |q: Point3| {
        let (z, radial) = split(q);
        radial.length().mul_add(sin_a, -(nappe * z * cos_a)).abs()
    };
    // Lower bound of `r` on the triangle from each vertex's tangent plane
    // `r(v) + r̂(v) · (x − v)`, a linear function whose minimum over the
    // triangle is at a vertex.
    let r_min = (0..3)
        .map(|i| {
            let (_, radial) = split(p[i]);
            let r = radial.length();
            if r <= 0.0 {
                return f64::NEG_INFINITY;
            }
            let unit = radial * (1.0 / r);
            (0..3)
                .map(|j| r + unit.dot(p[j] - p[i]))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(f64::NEG_INFINITY, f64::max);
    if r_min.is_nan() || r_min <= 0.0 {
        return (f64::INFINITY, f64::INFINITY);
    }
    // The centroid's radial and circumferential directions, the
    // triangle's extents along them, and its angular half-span from the
    // centroid.
    let (_, radial) = split(centroid);
    let Some(r0) = radial.normalize().ok() else {
        return (f64::INFINITY, f64::INFINITY);
    };
    let t0 = axis.cross(r0);
    let extent = |direction: Vec3| {
        let along = p.map(|q| (q - apex).dot(direction));
        along.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - along.iter().copied().fold(f64::INFINITY, f64::min)
    };
    let (tangential, radial_extent) = (extent(t0), extent(r0));
    let span = p
        .iter()
        .map(|&q| {
            let w = q - apex;
            w.dot(t0).atan2(w.dot(r0)).abs()
        })
        .fold(0.0, f64::max)
        .min(std::f64::consts::FRAC_PI_2);
    let h = radial_extent.mul_add(span.sin(), tangential) / GRID as f64;
    let mut sampled = 0.0_f64;
    let at = |l0: f64, l1: f64| p[0] + (p[1] - p[0]) * l1 + (p[2] - p[0]) * (1.0 - l0 - l1);
    for i in 0..=GRID {
        for j in 0..=GRID - i {
            let (l0, l1) = (i as f64 / GRID as f64, j as f64 / GRID as f64);
            sampled = sampled.max(g(at(l0, l1)));
        }
    }
    for k in 0..3 {
        let (a, b) = (p[k], p[(k + 1) % 3]);
        for step in 1..=EDGE_SAMPLES {
            let t = step as f64 / (EDGE_SAMPLES + 1) as f64;
            sampled = sampled.max(g(a + (b - a) * t));
        }
    }
    let magnitude = p
        .iter()
        .map(|q| {
            let w = *q - apex;
            w.x().abs().max(w.y().abs()).max(w.z().abs())
        })
        .fold(0.0, f64::max);
    let curvature = 0.5 * sin_a / r_min * h * h;
    (sampled, curvature + 64.0 * f64::EPSILON * magnitude)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn cone() -> ConicalSurface {
        ConicalSurface::with_ref_dir(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.5,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    /// The chart normal is `S_u × S_v` normalized, on either nappe.
    #[test]
    fn chart_normal_is_the_parametric_normal_on_both_nappes() {
        let cone = cone();
        for &(u, v) in &[(0.3, 2.0), (2.0, 0.5), (0.3, -2.0), (4.0, -0.7)] {
            let h = 1e-6;
            let su = (cone.evaluate(u + h, v) - cone.evaluate(u - h, v)) * (0.5 / h);
            let sv = (cone.evaluate(u, v + h) - cone.evaluate(u, v - h)) * (0.5 / h);
            let expected = su.cross(sv).normalize().unwrap();
            let normal = chart_normal(&cone, cone.evaluate(u, v));
            assert!(
                (normal - expected).length() < 1e-6,
                "({u}, {v}): {normal:?} against {expected:?}"
            );
        }
    }

    /// The sign test reads the same at any scale and distance from the
    /// origin, and treats a flat (degenerate) triangle as turned over.
    #[test]
    fn turned_over_is_a_scale_free_sign_test() {
        let n = Vec3::new(0.0, 0.0, 1.0);
        for scale in [1e-6, 1.0, 1e6] {
            for offset in [0.0, 1e8] {
                let o = Point3::new(offset, offset, offset);
                let tri = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| {
                    [a, b, c].map(|(x, y)| o + Vec3::new(x * scale, y * scale, 0.0))
                };
                assert!(!turns_over(tri((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)), n));
                assert!(turns_over(tri((0.0, 0.0), (0.0, 1.0), (1.0, 0.0)), n));
                assert!(turns_over(tri((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)), n));
            }
        }
    }

    /// Of a turned-over triangle, the removable vertex nearest its opposite
    /// edge is dropped; a triangle with no removable vertex drops nothing.
    #[test]
    fn the_squashed_interior_vertex_is_dropped() {
        let cone = cone();
        // Two boundary samples on the rim `v = 2` and an interior point a
        // hair inside their chart chord, which lies past their 3D chord.
        let chart = vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 2.0),
            Point2::new(1.0, 2.0),
            Point2::new(0.5, 2.0 - 1e-6),
            Point2::new(0.5, 1.0),
        ];
        let position = |id: usize| cone.evaluate(chart[id].x(), chart[id].y());
        // Counter-clockwise in the chart: the rim chord, then the point.
        let folded = [(4, 3, 5)];
        let fine = [(4, 3, 6)];
        assert!(turns_over(
            [position(4), position(3), position(5)],
            chart_normal(&cone, position(5))
        ));
        let interior = |id: usize| (id >= 5).then_some(id);
        assert_eq!(
            check(&cone, &folded, &chart, position, interior, |_| true),
            (1, vec![5])
        );
        assert_eq!(
            check(&cone, &fine, &chart, position, interior, |_| true),
            (0, vec![])
        );
        assert_eq!(
            check(&cone, &folded, &chart, position, |_| None, |_| true),
            (1, vec![])
        );
        // A turned-over triangle emission drops (inside a hole) is not
        // counted.
        assert_eq!(
            check(&cone, &folded, &chart, position, interior, |_| false),
            (0, vec![])
        );
    }

    /// The largest distance from the cone over a fine barycentric grid.
    fn brute_deviation(cone: &ConicalSurface, p: [Point3; 3]) -> f64 {
        let n = 400;
        let mut worst = 0.0_f64;
        for i in 0..=n {
            for j in 0..=n - i {
                let (l1, l2) = (i as f64 / n as f64, j as f64 / n as f64);
                let q = p[0] + (p[1] - p[0]) * l1 + (p[2] - p[0]) * l2;
                let w = q - cone.apex();
                let z = w.dot(cone.axis());
                let r = (w - cone.axis() * z).length();
                let (sin_a, cos_a) = cone.half_angle().sin_cos();
                worst = worst.max(r.mul_add(sin_a, -(z * cos_a)).abs());
            }
        }
        worst
    }

    /// The bound holds the fine-grid maximum, including one that lies off
    /// the edge midpoints and the centroid (the review's facet on a `π/4`
    /// cone, which peaks 35% along an edge), and stays within a few percent
    /// of it; a triangle that may reach the axis has no bound.
    #[test]
    fn the_deviation_bound_holds_the_triangle_maximum() {
        let quarter = ConicalSurface::with_ref_dir(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_4,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let on = |r: f64, u: f64| Point3::new(r * u.cos(), r * u.sin(), r);
        let review = [on(1.0, 0.0), on(3.0, 1.2), on(3.01, 1.19)];
        let cone = cone();
        let (sin_a, cos_a) = 0.5f64.sin_cos();
        let slant = |v: f64, u: f64| on(v * cos_a, u) + Vec3::new(0.0, 0.0, v * sin_a - v * cos_a);
        // The review's facet spans two units: its curvature term is wide.
        let bound = deviation_bound(&quarter, review);
        assert!(bound >= brute_deviation(&quarter, review) && bound >= 0.1972);
        assert!(bound <= 1.7 * 0.1972, "{bound}");
        // Triangles of a mesh's size: the bound within a quarter of the
        // maximum, and within 2.5 times it on a sliver four units long along
        // its generators and 1e-5 off it (the sub-triangles' radial extent
        // over the small angular span still counts).
        for (p, ratio) in [
            ([slant(1.0, 0.0), slant(1.0, 0.3), slant(1.2, 0.15)], 1.25),
            ([slant(2.0, 0.0), slant(2.0, 0.5), slant(2.001, 0.2)], 1.25),
            ([slant(1.0, 0.0), slant(5.0, 0.01), slant(5.0, 0.0)], 2.5),
        ] {
            let brute = brute_deviation(&cone, p);
            let bound = deviation_bound(&cone, p);
            assert!(bound >= brute, "{bound} under {brute}");
            assert!(bound <= ratio * brute, "{bound} against {brute}");
        }
        // A triangle across the axis.
        let across = [on(1.0, 0.0), on(1.0, 3.0), on(1.0, 4.0)];
        assert!(deviation_bound(&quarter, across).is_infinite());
    }
}
