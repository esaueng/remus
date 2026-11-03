//! Bounded classification of exact polygonal face boundaries; no winding repair.

use remus_math::predicates::orient2d;
use remus_math::vec::{Point2, Vec2};

use crate::IoError;
use crate::limits::ensure_limit;

use super::reader::malformed;

const MAX_LOOPS: usize = 128;
const MAX_WORK: usize = 8_000_000;

pub(super) struct Polygon {
    pub points: Vec<Point2>,
    pub tolerance: f64,
}

#[derive(Default)]
pub(super) struct Work(usize);

impl Work {
    fn charge(&mut self, count: usize) -> Result<(), IoError> {
        self.0 = self.0.saturating_add(count);
        ensure_limit("BREP planar boundary comparisons", self.0, MAX_WORK)
    }
}

fn turn(a: Point2, b: Point2, c: Point2) -> Result<f64, IoError> {
    let value = orient2d(a, b, c);
    if !value.is_finite() {
        return Err(malformed("non-finite planar predicate"));
    }
    Ok(value)
}

fn distance(point: Point2, a: Point2, b: Point2) -> Result<f64, IoError> {
    let d = b - a;
    let length = d.x().hypot(d.y());
    if !length.is_finite() || length == 0.0 {
        return Err(malformed("degenerate planar boundary edge"));
    }
    let unit = Vec2::new(d.x() / length, d.y() / length);
    let t = (point - a).dot(unit).clamp(0.0, length);
    let residual = point - (a + unit * t);
    let value = residual.x().hypot(residual.y());
    if !value.is_finite() {
        return Err(malformed("non-finite planar boundary distance"));
    }
    Ok(value)
}

fn in_segment_box(point: Point2, a: Point2, b: Point2) -> bool {
    point.x() >= a.x().min(b.x())
        && point.x() <= a.x().max(b.x())
        && point.y() >= a.y().min(b.y())
        && point.y() <= a.y().max(b.y())
}

fn separated(a: Point2, b: Point2, c: Point2, d: Point2, tolerance: f64) -> Result<bool, IoError> {
    let ab_c = turn(a, b, c)?;
    let ab_d = turn(a, b, d)?;
    let cd_a = turn(c, d, a)?;
    let cd_b = turn(c, d, b)?;
    // Exact incidence must not depend on a rounded point/segment distance:
    // source entity tolerances may be zero.
    if (ab_c == 0.0 && in_segment_box(c, a, b))
        || (ab_d == 0.0 && in_segment_box(d, a, b))
        || (cd_a == 0.0 && in_segment_box(a, c, d))
        || (cd_b == 0.0 && in_segment_box(b, c, d))
    {
        return Ok(false);
    }
    if ab_c != 0.0
        && ab_d != 0.0
        && cd_a != 0.0
        && cd_b != 0.0
        && ab_c.is_sign_positive() != ab_d.is_sign_positive()
        && cd_a.is_sign_positive() != cd_b.is_sign_positive()
    {
        return Ok(false);
    }
    Ok(distance(a, c, d)? > tolerance
        && distance(b, c, d)? > tolerance
        && distance(c, a, b)? > tolerance
        && distance(d, a, b)? > tolerance)
}

impl Polygon {
    fn segments(&self) -> impl Iterator<Item = (Point2, Point2)> + '_ {
        self.points
            .iter()
            .copied()
            .zip(self.points.iter().copied().cycle().skip(1))
    }

    fn signed_area(&self, work: &mut Work) -> Result<f64, IoError> {
        if self.points.len() < 3 {
            return Err(malformed("planar boundary has fewer than three vertices"));
        }
        let mut area = 0.0;
        let mut perimeter = 0.0;
        let origin = self.points[0];
        for (a, b) in self.segments() {
            area += turn(origin, a, b)?;
            perimeter += (b.x() - a.x()).hypot(b.y() - a.y());
        }
        if !area.is_finite()
            || !perimeter.is_finite()
            || area.abs() <= 2.0 * perimeter * self.tolerance
        {
            return Err(malformed("degenerate planar boundary area"));
        }
        // Adjacent segments may meet only at their common endpoint. In
        // particular, a backtracking collinear pair is not a simple loop.
        for i in 0..self.points.len() {
            work.charge(1)?;
            let a = self.points[i];
            let b = self.points[(i + 1) % self.points.len()];
            let c = self.points[(i + 2) % self.points.len()];
            if (turn(a, b, c)? == 0.0 && (in_segment_box(a, b, c) || in_segment_box(c, a, b)))
                || distance(a, b, c)? <= self.tolerance
                || distance(c, a, b)? <= self.tolerance
            {
                return Err(malformed("overlapping adjacent planar boundary edges"));
            }
        }
        for (i, (a, b)) in self.segments().enumerate() {
            for (j, (c, d)) in self.segments().enumerate().skip(i + 1) {
                if j == i + 1 || (i == 0 && j + 1 == self.points.len()) {
                    continue;
                }
                work.charge(1)?;
                if !separated(a, b, c, d, self.tolerance)? {
                    return Err(malformed("self-intersecting or touching planar boundary"));
                }
            }
        }
        Ok(area)
    }

    fn contains(&self, point: Point2, work: &mut Work) -> Result<bool, IoError> {
        work.charge(self.points.len())?;
        let mut winding = 0_i32;
        for (a, b) in self.segments() {
            if a.y() <= point.y() && b.y() > point.y() && turn(a, b, point)? > 0.0 {
                winding += 1;
            } else if a.y() > point.y() && b.y() <= point.y() && turn(a, b, point)? < 0.0 {
                winding -= 1;
            }
        }
        Ok(winding != 0)
    }
}

/// Select one enclosing perimeter, with disjoint, oppositely wound holes.
pub(super) fn outer_loop(polygons: &[Polygon], work: &mut Work) -> Result<usize, IoError> {
    ensure_limit("BREP loops per planar face", polygons.len(), MAX_LOOPS)?;
    let areas = polygons
        .iter()
        .map(|polygon| polygon.signed_area(work))
        .collect::<Result<Vec<_>, _>>()?;
    let outer = areas
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.abs().total_cmp(&b.abs()))
        .map(|(index, _)| index)
        .ok_or_else(|| malformed("planar face has no boundary"))?;
    for (i, left) in polygons.iter().enumerate() {
        for (j, right) in polygons.iter().enumerate().skip(i + 1) {
            for (a, b) in left.segments() {
                for (c, d) in right.segments() {
                    work.charge(1)?;
                    if !separated(a, b, c, d, left.tolerance.max(right.tolerance))? {
                        return Err(malformed("intersecting or touching planar face loops"));
                    }
                }
            }
            if i != outer
                && j != outer
                && (left.contains(right.points[0], work)?
                    || right.contains(left.points[0], work)?)
            {
                return Err(malformed("nested planar face holes"));
            }
        }
        if i != outer {
            if !polygons[outer].contains(left.points[0], work)? {
                return Err(malformed("disconnected planar face loops"));
            }
            if areas[i].is_sign_positive() == areas[outer].is_sign_positive() {
                return Err(malformed("planar hole winds like its perimeter"));
            }
        }
    }
    Ok(outer)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use super::*;

    fn polygon(points: &[[f64; 2]]) -> Polygon {
        Polygon {
            points: points.iter().map(|p| Point2::new(p[0], p[1])).collect(),
            tolerance: 1e-7,
        }
    }

    fn rectangle(x: f64, y: f64, size: f64, clockwise: bool) -> Polygon {
        let mut p = polygon(&[[x, y], [x + size, y], [x + size, y + size], [x, y + size]]);
        if clockwise {
            p.points.reverse();
        }
        p
    }

    #[test]
    fn perimeter_can_appear_anywhere_with_multiple_holes() {
        for index in 0..3 {
            let mut polygons = vec![rectangle(2., 2., 3., true), rectangle(12., 12., 3., true)];
            polygons.insert(index, rectangle(0., 0., 20., false));
            assert_eq!(outer_loop(&polygons, &mut Work::default()).unwrap(), index);
            for p in &mut polygons {
                p.points.reverse();
            }
            assert_eq!(outer_loop(&polygons, &mut Work::default()).unwrap(), index);
        }
    }

    #[test]
    fn invalid_nesting_winding_clearance_and_overlaps_are_errors() {
        for holes in [
            vec![rectangle(2., 2., 3., false)],  // same winding
            vec![rectangle(22., 2., 3., true)],  // separate region
            vec![rectangle(18., 2., 3., true)],  // crossing perimeter
            vec![rectangle(0., 2., 3., true)],   // touching perimeter
            vec![rectangle(1e-9, 2., 3., true)], // tolerance-ambiguous clearance
            vec![rectangle(2., 2., 3., true), rectangle(4., 4., 3., true)],
            vec![rectangle(2., 2., 3., true), rectangle(5., 2., 3., true)],
            vec![rectangle(2., 2., 10., true), rectangle(4., 4., 3., true)],
        ] {
            let mut polygons = vec![rectangle(0., 0., 20., false)];
            polygons.extend(holes);
            assert!(matches!(
                outer_loop(&polygons, &mut Work::default()),
                Err(IoError::ParseError { .. })
            ));
        }
    }

    #[test]
    fn concave_perimeter_cannot_be_certified_from_hole_vertices_alone() {
        let outer = polygon(&[
            [0., 0.],
            [12., 0.],
            [12., 12.],
            [8., 12.],
            [8., 4.],
            [4., 4.],
            [4., 12.],
            [0., 12.],
        ]);
        // Each corner is in material, but the long sides cross the notch.
        let bridge = polygon(&[[2., 6.], [2., 8.], [10., 8.], [10., 6.]]);
        assert!(matches!(
            outer_loop(&[outer, bridge], &mut Work::default()),
            Err(IoError::ParseError { .. })
        ));
    }

    #[test]
    fn crossing_backtracking_and_collapsed_boundaries_refuse() {
        for points in [
            vec![[0., 0.], [5., 5.], [0., 5.], [5., 0.]],
            vec![[0., 0.], [5., 0.], [2., 0.], [5., 5.], [0., 5.]],
            vec![[0., 0.], [5., 0.], [5., 0.], [0., 5.]],
            vec![[0., 0.], [5., 0.], [10., 0.]],
        ] {
            assert!(matches!(
                outer_loop(&[polygon(&points)], &mut Work::default()),
                Err(IoError::ParseError { .. })
            ));
        }
        let collinear = polygon(&[[0., 0.], [2., 0.], [5., 0.], [5., 5.], [0., 5.]]);
        assert_eq!(outer_loop(&[collinear], &mut Work::default()).unwrap(), 0);
    }

    #[test]
    fn exact_contact_refuses_even_with_zero_source_tolerance() {
        let mut outer = polygon(&[[0., 0.], [6., 12.], [12., 0.]]);
        let mut touching = polygon(&[[1., 2.], [3., 2.], [3., 3.]]);
        outer.tolerance = 0.;
        touching.tolerance = 0.;
        assert!(matches!(
            outer_loop(&[outer, touching], &mut Work::default()),
            Err(IoError::ParseError { .. })
        ));
    }

    #[test]
    fn classification_keeps_translation_rotation_and_small_scale() {
        for (offset, scale) in [(0., 1.), (1e9, 1.), (0., 1e-5)] {
            let mut polygons = vec![rectangle(0., 0., 20., false), rectangle(2., 2., 3., true)];
            for polygon in &mut polygons {
                polygon.tolerance *= scale;
                for point in &mut polygon.points {
                    *point = Point2::new(
                        offset + scale * (0.6 * point.x() - 0.8 * point.y()),
                        offset + scale * (0.8 * point.x() + 0.6 * point.y()),
                    );
                }
            }
            assert_eq!(outer_loop(&polygons, &mut Work::default()).unwrap(), 0);
        }
    }

    #[test]
    fn loop_count_and_import_wide_comparison_budgets_fail_closed() {
        let polygons: Vec<_> = (0..129).map(|_| rectangle(0., 0., 1., false)).collect();
        assert!(matches!(
            outer_loop(&polygons, &mut Work::default()),
            Err(IoError::LimitExceeded { .. })
        ));
        // The counter is deliberately shared between faces, rather than reset
        // by each invocation of the classifier.
        let polygons = [rectangle(0., 0., 20., false), rectangle(2., 2., 3., true)];
        let mut work = Work::default();
        outer_loop(&polygons, &mut work).unwrap();
        let first_face_work = work.0;
        work.0 = MAX_WORK - first_face_work;
        outer_loop(&polygons, &mut work).unwrap();
        assert!(matches!(
            outer_loop(&polygons, &mut work),
            Err(IoError::LimitExceeded { .. })
        ));
    }
}
