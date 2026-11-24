//! Turned-over triangles of a cone face's CDT, and the interior vertices to
//! triangulate again without (B289).
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
//! Predicting which points fold (the guard this replaces) missed cases and
//! dropped points that did not fold. Here the triangulation is checked
//! instead ([`check`]): every triangle whose normal does not agree in sign
//! with the cone's chart normal at its centroid turned over, and one that
//! leans far past what its own extent allows stands on edge, a sliver
//! squashed onto a chord ([`facing`]). For each, the interior vertex nearest
//! the triangle's opposite edge in the chart (the one it is squashed onto)
//! is dropped, and the CDT is rebuilt from its constrained state without
//! them. Boundary, hole and constraint-recovery vertices are never dropped,
//! so the face's boundary, and the watertight seam with its neighbours, is
//! unchanged. A round drops at least one vertex, rounds are capped at
//! [`MAX_ROUNDS`], and a round that turns more triangles over than the one
//! before is undone, so the repair ends, is deterministic, and never leaves
//! more turned-over triangles than the CDT made. A turned-over triangle of
//! boundary vertices alone is the boundary's own (its samples force it)
//! and is left as it is.
//!
//! Dropping an interior vertex rather than inserting one: a vertex inside a
//! chord's lens folds with that chord wherever it sits, and a point placed
//! past the lens to refine the chord's triangle would be a grid point by
//! another name, while a band narrower than the lens has no room for one.
//! Dropping only merges the triangles round the vertex into triangles over
//! the same chart region with vertices from its link, all on the cone; the
//! deflection they keep is checked against the cone in closed form by the
//! tests (`tessellate::tests::cone_chord_fold`).

use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point2, Point3, Vec3};

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

/// What the check reads of one triangle of a cone face.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Facing {
    /// Faces the way the cone does, within what its own extent allows.
    Sound,
    /// Faces the cone's way but stands on edge (see [`facing`]).
    Standing,
    /// Faces against the cone, or within rounding of edge-on.
    TurnedOver,
}

/// The cosine of the lean from the cone's normal past which a triangle may
/// stand on edge: 53 degrees. Inscribed triangles of the reviewed bands lean
/// up to 37 degrees (a Schwarz lantern on a sharp cone); slivers squashed
/// onto a far chord of the steep band `1e9` out lean 60 to 74 degrees, and
/// left there they read 1.31 times its area. A wide triangle may lean
/// further (78 degrees on the thread cones, where a coarse chord spans most
/// of a radian): the second test, against the normal's turn, keeps it.
const STANDING_COS: f64 = 0.6;

/// How the triangle `p`, its vertices on `cone`, faces.
///
/// It is turned over when its normal does not agree in sign with the cone's
/// chart normal at its centroid ([`turns_over`]). It stands on edge when it
/// leans more than [`STANDING_COS`] allows and more than twice the angle the
/// cone's normal turns through between its vertices: a flat triangle
/// through points of the cone leans from the surface by about as much as
/// the surface's normal turns across it, so a wide one may lean far, but a
/// triangle squashed onto a boundary chord, its third vertex within the
/// chord's sagitta of it, follows the hair the vertex sits off the chord by
/// out to a right angle. Such slivers lie close to the cone (they pass the
/// deflection) but read up to 1.6 times a narrow band's area. The rounding
/// of positions (relative to their distance from the origin, over the
/// triangle's shortest altitude) widens the allowance.
pub(super) fn facing(cone: &ConicalSurface, p: [Point3; 3]) -> Facing {
    let centroid = Point3::new(
        (p[0].x() + p[1].x() + p[2].x()) / 3.0,
        (p[0].y() + p[1].y() + p[2].y()) / 3.0,
        (p[0].z() + p[1].z() + p[2].z()) / 3.0,
    );
    let normal = chart_normal(cone, centroid);
    if turns_over(p, normal) {
        return Facing::TurnedOver;
    }
    let cross = (p[1] - p[0]).cross(p[2] - p[0]);
    let area2 = cross.length();
    if cross.dot(normal) >= area2 * STANDING_COS {
        return Facing::Sound;
    }
    let corners = p.map(|q| chart_normal(cone, q));
    let turn = |a: Vec3, b: Vec3| 2.0 * (0.5 * (a - b).length()).min(1.0).asin();
    let spread = turn(corners[0], corners[1])
        .max(turn(corners[1], corners[2]))
        .max(turn(corners[2], corners[0]));
    let longest = (p[1] - p[0])
        .length()
        .max((p[2] - p[1]).length())
        .max((p[0] - p[2]).length());
    let magnitude = p
        .iter()
        .map(|q| q.x().abs().max(q.y().abs()).max(q.z().abs()))
        .fold(0.0, f64::max);
    // Shortest altitude `2 A / longest`; a position's rounding moves the
    // normal by its size over that.
    let noise = 64.0 * f64::EPSILON * magnitude * longest / area2;
    let allowed = 2.0f64.mul_add(spread, noise);
    if allowed < std::f64::consts::FRAC_PI_2 && cross.dot(normal) < area2 * allowed.cos() {
        Facing::Standing
    } else {
        Facing::Sound
    }
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

/// The check of one triangulation: how many of `triangles` (CDT ids into
/// `chart`) turned over, and the interior vertices to triangulate again
/// without. For every turned-over or standing triangle with a vertex
/// `removable` answers for, that is the one nearest its opposite edge in the
/// chart, the one it is squashed onto (ties to the smaller id). Sorted and
/// unique; empty when no such triangle has a removable vertex.
pub(super) fn check(
    cone: &ConicalSurface,
    triangles: &[(usize, usize, usize)],
    chart: &[Point2],
    position: impl Fn(usize) -> Point3,
    removable: impl Fn(usize) -> Option<usize>,
) -> (usize, Vec<usize>) {
    let mut turned_over = 0;
    let mut drop = Vec::new();
    for &(i0, i1, i2) in triangles {
        let ids = [i0, i1, i2];
        match facing(cone, ids.map(&position)) {
            Facing::Sound => continue,
            Facing::Standing => {}
            Facing::TurnedOver => turned_over += 1,
        }
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
    (turned_over, drop)
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
            check(&cone, &folded, &chart, position, interior),
            (1, vec![5])
        );
        assert_eq!(check(&cone, &fine, &chart, position, interior), (0, vec![]));
        assert_eq!(
            check(&cone, &folded, &chart, position, |_| None),
            (1, vec![])
        );
    }
}
