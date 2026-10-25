//! Mitre corners: two convex stripes meeting over a reflex vertex.
//!
//! At an L-bracket notch vertex whose concave edge stays sharp, the two convex
//! edges on the cap face are rounded at one radius. Their cylinders' axes lie
//! in the plane one radius under the cap and cross, so the two equal
//! cylinders meet in the plane bisecting the edges: each stripe runs past the
//! vertex to that plane and ends on the same ellipse. The cap's contact lines
//! cross there too, and both walls' contacts land one radius down the sharp
//! concave edge, which is all the corner needs: no setback, no patch.

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};

/// The mitre plane `(unit normal, n·p)` of a qualified reflex corner, or
/// `None`.
///
/// `cap` is the outward normal of the face both stripes border; `wall_a` and
/// `wall_b` those of each stripe's other face, which meet at the sharp third
/// edge. `dir_a` and `dir_b` run from `vertex` along the two stripes'
/// straight edges. Qualified: pairwise perpendicular supports, each edge on
/// its own two supports, and a reflex corner, where each edge runs out of the
/// other stripe's wall (a box corner, whose edges run into it, is convex).
pub(super) fn mitre_plane(
    cap: Vec3,
    wall_a: Vec3,
    wall_b: Vec3,
    vertex: Point3,
    dir_a: Vec3,
    dir_b: Vec3,
    tol: Tolerance,
) -> Option<(Vec3, f64)> {
    let perpendicular = |x: Vec3, y: Vec3| x.dot(y).abs() <= tol.angular;
    let lies_on = |dir: Vec3, normal: Vec3| perpendicular(dir, normal);
    let qualified = perpendicular(cap, wall_a)
        && perpendicular(cap, wall_b)
        && perpendicular(wall_a, wall_b)
        && lies_on(dir_a, cap)
        && lies_on(dir_a, wall_a)
        && lies_on(dir_b, cap)
        && lies_on(dir_b, wall_b)
        && dir_a.dot(wall_b) > tol.angular
        && dir_b.dot(wall_a) > tol.angular;
    if !qualified {
        return None;
    }
    let normal = (dir_a - dir_b).normalize().ok()?;
    Some((
        normal,
        normal.dot(Vec3::new(vertex.x(), vertex.y(), vertex.z())),
    ))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    #[test]
    fn notch_vertex_mitres_on_the_bisector() {
        // L-bracket top notch (8, 8, 20): cap +Z, base top +Y carries the +X
        // edge, wall +X carries the +Y edge.
        let vertex = Point3::new(8.0, 8.0, 20.0);
        let (normal, d) = mitre_plane(Z, Y, X, vertex, X, Y, Tolerance::new()).unwrap();
        let expected = (X - Y).normalize().unwrap();
        assert!((normal - expected).length() < 1e-12);
        assert!(d.abs() < 1e-12, "the plane x = y passes through the vertex");
    }

    #[test]
    fn box_corner_is_not_a_mitre() {
        // Box corner (0, 0, h): edges run +X on face -Y and +Y on face -X.
        let vertex = Point3::new(0.0, 0.0, 5.0);
        assert!(mitre_plane(Z, -Y, -X, vertex, X, Y, Tolerance::new()).is_none());
    }

    #[test]
    fn oblique_supports_are_refused() {
        let wall = (X + Y * 0.1).normalize().unwrap();
        let vertex = Point3::new(8.0, 8.0, 20.0);
        assert!(mitre_plane(Z, Y, wall, vertex, X, Y, Tolerance::new()).is_none());
    }
}
