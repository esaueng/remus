//! Relative planar-bound sampling never lets coarser chords reorder loops.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;

/// A loop sampled as `n` equal chords of a circle of `radius`.
fn circle_loop(radius: f64, n: usize, index: usize) -> FaceBoundLoop {
    let polygon: Vec<Point2> = (0..n)
        .map(|k| {
            #[allow(clippy::cast_precision_loss)]
            let angle = std::f64::consts::TAU * k as f64 / n as f64;
            Point2::new(radius * angle.cos(), radius * angle.sin())
        })
        .collect();
    let bounds = Aabb2::try_from_points(polygon.iter().copied()).unwrap();
    FaceBoundLoop {
        candidate_index: index,
        signed_area: polygon_signed_area(&polygon),
        perimeter: polygon_perimeter(&polygon),
        probe: Point2::new(0.0, 0.0),
        bounds,
        polygon,
        seam_flexible_u: false,
        seam_flexible_v: false,
    }
}

/// The review's thin annulus: an r 200 outer circle in one edge and an
/// r 199.99995 hole in five arcs. Sampled relative to their extents (2,048
/// and 2,560 chords) the hole's polygon is the LARGER one, so area ordering
/// would pick it as the perimeter; the classifier must see that and sample
/// both again at the absolute deflection.
#[test]
fn a_thin_annulus_is_ambiguous_at_the_relative_deflection() {
    let outer = circle_loop(200.0, 2048, 0);
    let hole = circle_loop(199.999_95, 2560, 1);
    assert!(
        hole.signed_area > outer.signed_area,
        "the reordering the check exists for"
    );
    let deflection = FACE_BOUND_RELATIVE_DEFLECTION * 400.0 * std::f64::consts::SQRT_2;
    assert!(planar_loop_areas_ambiguous(&[outer, hole], deflection));
}

#[test]
fn well_separated_loops_are_not_ambiguous() {
    let outer = circle_loop(200.0, 2048, 0);
    let hole = circle_loop(150.0, 2048, 1);
    let deflection = FACE_BOUND_RELATIVE_DEFLECTION * 400.0 * std::f64::consts::SQRT_2;
    assert!(!planar_loop_areas_ambiguous(&[outer, hole], deflection));
}
