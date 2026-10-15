//! Knot-span boxes of NURBS faces and the ray prefilter they drive.
//!
//! The ray-NURBS solver behind a prepared crossing count scans a seed grid
//! and refines Newton seeds for every NURBS face whose support box the BVH
//! offers. A forward ray that stays clear of every knot-span box of the face
//! gets no hit from it, so [`PreparedSolid::count_ray_crossings`] skips the
//! face without running the solver.

use std::cmp::Ordering;

use remus_math::aabb::Aabb3;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::face::FaceSurface;

use super::PreparedSolid;

/// Relative inflation of every span box and of the ray-box test.
const BOX_ROUNDING: f64 = 1.0e-12;

fn point_norm(p: Point3) -> f64 {
    Vec3::new(p.x(), p.y(), p.z()).length()
}

/// Grow a box by its rounding allowance.
fn inflate(aabb: Aabb3) -> Aabb3 {
    let scale = [
        aabb.min.x(),
        aabb.min.y(),
        aabb.min.z(),
        aabb.max.x(),
        aabb.max.y(),
        aabb.max.z(),
    ]
    .iter()
    .fold(1.0_f64, |m, c| m.max(c.abs()));
    aabb.expanded(BOX_ROUNDING * scale)
}

/// Control-point boxes of a NURBS support's knot spans.
///
/// On the span `[u_k, u_k+1] x [v_l, v_l+1]` only the control points
/// `P[k-p..=k][l-q..=l]` have non-zero basis functions, and those are
/// non-negative and sum to one. With positive weights each surface point is
/// therefore a convex combination of exactly those points, so their box
/// contains the span's whole image (the local convex-hull property). Any
/// non-positive or non-finite weight refuses, and so does a ragged control
/// net. So does a knot vector that decreases or whose first or last span is
/// empty: evaluation clamps a parameter at the domain ends into those spans,
/// which must then be boxed.
pub(super) fn nurbs_span_boxes(surface: &NurbsSurface) -> Option<Vec<Aabb3>> {
    if !surface
        .weights()
        .iter()
        .flatten()
        .all(|&w| w.is_finite() && w > 0.0)
    {
        return None;
    }
    let (p, q) = (surface.degree_u(), surface.degree_v());
    let (knots_u, knots_v) = (surface.knots_u(), surface.knots_v());
    let points = surface.control_points();
    let (rows, cols) = (points.len(), points.first().map_or(0, Vec::len));
    if rows <= p
        || cols <= q
        || knots_u.len() != rows + p + 1
        || knots_v.len() != cols + q + 1
        || points.iter().any(|row| row.len() != cols)
    {
        return None;
    }
    let usable = |knots: &[f64], degree: usize, count: usize| {
        knots.windows(2).all(|pair| pair[0] <= pair[1])
            && knots[degree] < knots[degree + 1]
            && knots[count - 1] < knots[count]
    };
    if !usable(knots_u, p, rows) || !usable(knots_v, q, cols) {
        return None;
    }
    let mut boxes = Vec::new();
    for k in p..rows {
        if knots_u[k].partial_cmp(&knots_u[k + 1]) != Some(Ordering::Less) {
            continue;
        }
        for l in q..cols {
            if knots_v[l].partial_cmp(&knots_v[l + 1]) != Some(Ordering::Less) {
                continue;
            }
            let span = (k - p..=k).flat_map(|i| points[i][l - q..=l].iter().copied());
            let aabb = Aabb3::try_from_points(span)?;
            if ![aabb.min, aabb.max]
                .iter()
                .all(|c| c.x().is_finite() && c.y().is_finite() && c.z().is_finite())
            {
                return None;
            }
            boxes.push(inflate(aabb));
        }
    }
    (!boxes.is_empty()).then_some(boxes)
}

/// The prefilter's view of a NURBS support: its knot-span boxes and the
/// largest absolute control-point coordinate, which sets how far from a line
/// the solver can report a hit.
#[derive(Debug, Clone)]
pub(super) struct SpanBoxes {
    boxes: Vec<Aabb3>,
    magnitude: f64,
}

impl SpanBoxes {
    /// `None` when the span boxes cannot be certified ([`nurbs_span_boxes`]).
    pub(super) fn for_surface(surface: &NurbsSurface) -> Option<Self> {
        let boxes = nurbs_span_boxes(surface)?;
        let magnitude = surface
            .control_points()
            .iter()
            .flatten()
            .flat_map(|p| p.0)
            .fold(0.0_f64, |m, c| m.max(c.abs()));
        Some(Self { boxes, magnitude })
    }

    /// Whether the forward ray comes within the solver's hit reach (see
    /// [`PreparedSolid::ray_misses_nurbs_face`]) of some span box.
    fn meet(&self, origin: Point3, direction: Vec3) -> bool {
        let reach = remus_math::nurbs::intersection::line_hit_reach(self.magnitude, origin);
        self.boxes
            .iter()
            .any(|aabb| ray_meets_box(origin, direction, aabb, reach))
    }
}

/// Whether the forward ray `origin + t direction`, `t >= 0`, comes within
/// `reach` of the box (slab test against the box grown by `reach` plus the
/// rounding allowance). A non-finite input never reads as a miss.
pub(super) fn ray_meets_box(origin: Point3, direction: Vec3, aabb: &Aabb3, reach: f64) -> bool {
    let scale = 1.0 + point_norm(origin) + point_norm(aabb.min) + point_norm(aabb.max);
    let grown = aabb.expanded(reach + BOX_ROUNDING * scale);
    let (mut t0, mut t1) = (0.0_f64, f64::INFINITY);
    for k in 0..3 {
        let o = origin.0[k];
        let d = direction.0[k];
        let (lo, hi) = (grown.min.0[k], grown.max.0[k]);
        if d == 0.0 {
            if o < lo || o > hi {
                return false;
            }
            continue;
        }
        let (mut a, mut b) = ((lo - o) / d, (hi - o) / d);
        if a > b {
            std::mem::swap(&mut a, &mut b);
        }
        t0 = t0.max(a);
        t1 = t1.min(b);
        if t0 > t1 {
            return false;
        }
    }
    true
}

impl PreparedSolid<'_> {
    /// Whether a forward ray provably meets no point of a NURBS face's
    /// support that the ray-NURBS solver could report.
    ///
    /// Every hit `intersect_line_nurbs_with_grid` reports is a surface
    /// evaluation at an in-domain parameter (its seeds and Newton iterates are
    /// clamped to the domain) lying within `line_hit_reach` of the line (the
    /// seed-grid pass's `1e-5`, plus the completion pass's round-off
    /// allowance, which grows with the control points' coordinates), and
    /// every such evaluation lies in the knot-span boxes (local convex hulls,
    /// see [`nurbs_span_boxes`]). A hit is kept only ahead of the origin,
    /// where the line's closest point to it lies on the forward ray. So a ray
    /// that stays farther than that reach from every span box, for `t >= 0`,
    /// gets no hits and contributes no crossing, and the solver need not run.
    /// Only faces with a cached trim qualify, and only for a direction the
    /// solver accepts: their crossing count could not have failed, so
    /// skipping one hides no error.
    pub(super) fn ray_misses_nurbs_face(
        &self,
        index: usize,
        origin: Point3,
        direction: Vec3,
    ) -> bool {
        let prepared = &self.prepared_faces[index];
        let usable = matches!(
            direction.dot(direction).partial_cmp(&1e-20),
            Some(Ordering::Greater | Ordering::Equal)
        );
        if !prepared.nurbs || prepared.trim.is_none() || !usable {
            return false;
        }
        let boxes = prepared.span_boxes.get_or_init(|| {
            match self
                .topo
                .face(prepared.fid)
                .map(remus_topology::face::Face::surface)
            {
                Ok(FaceSurface::Nurbs(surface)) => SpanBoxes::for_surface(surface),
                Ok(
                    FaceSurface::Plane { .. }
                    | FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_),
                )
                | Err(_) => None,
            }
        });
        boxes
            .as_ref()
            .is_some_and(|boxes| !boxes.meet(origin, direction))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use remus_math::surfaces::{CylindricalSurface, SphericalSurface, ToroidalSurface};

    #[test]
    fn nurbs_span_boxes_contain_the_surface() {
        let axis = Vec3::new(0.2, 0.1, 1.0).normalize().unwrap();
        let surfaces = [
            CylindricalSurface::new(Point3::new(1.0, -2.0, 0.5), axis, 1.7)
                .unwrap()
                .to_nurbs(-1.0, 2.5)
                .unwrap(),
            SphericalSurface::new(Point3::new(0.5, 0.5, -1.0), 2.0)
                .unwrap()
                .to_nurbs()
                .unwrap(),
            ToroidalSurface::new(Point3::new(0.0, 1.0, 2.0), 3.0, 0.8)
                .unwrap()
                .to_nurbs()
                .unwrap(),
        ];
        for surface in &surfaces {
            let boxes = nurbs_span_boxes(surface).unwrap();
            let (u0, u1) = surface.domain_u();
            let (v0, v1) = surface.domain_v();
            for i in 0..=60 {
                for j in 0..=60 {
                    let u = u0 + (u1 - u0) * f64::from(i) / 60.0;
                    let v = v0 + (v1 - v0) * f64::from(j) / 60.0;
                    let point = surface.evaluate(u, v);
                    assert!(
                        boxes.iter().any(|b| b.contains_point(point)),
                        "({u}, {v}) -> {point:?} outside every span box"
                    );
                }
            }
        }
    }

    /// Forward hits of the solver, as the crossing count keeps them.
    fn forward_hits(surface: &NurbsSurface, origin: Point3, direction: Vec3) -> usize {
        remus_math::nurbs::intersection::intersect_line_nurbs(surface, origin, direction, 20)
            .unwrap()
            .iter()
            .filter(|hit| (hit.point - origin).dot(direction) / direction.dot(direction) > 1e-12)
            .count()
    }

    /// The solver can report a hit wider than its seed-grid tolerance: the
    /// completion pass accepts one within its round-off allowance, which the
    /// far end of this surface (z = 1e9) sets. The ray passes 3e-5 over the
    /// flat near span, clear of every span box by more than 1e-5, and the
    /// prefilter still keeps the face.
    #[test]
    fn prefilter_keeps_a_wide_completion_hit() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
                vec![Point3::new(2.0, 0.0, 1e9), Point3::new(2.0, 1.0, 1e9)],
            ],
            vec![vec![1.0; 2]; 3],
        )
        .unwrap();
        let origin = Point3::new(0.5, -1.0, 3e-5);
        let direction = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(forward_hits(&surface, origin, direction), 1);
        let boxes = SpanBoxes::for_surface(&surface).unwrap();
        assert!(boxes.meet(origin, direction));
        assert!(
            !boxes
                .boxes
                .iter()
                .any(|aabb| ray_meets_box(origin, direction, aabb, 1e-5))
        );
    }

    /// Deterministic low-discrepancy sample in [0, 1).
    fn halton(mut i: u32, base: u32) -> f64 {
        let (mut f, mut r) = (1.0_f64, 0.0_f64);
        while i > 0 {
            f /= f64::from(base);
            r += f * f64::from(i % base);
            i /= base;
        }
        r
    }

    /// No ray the solver gets a forward hit from is skipped: rational
    /// bicubic patches from 1e-3 to 1e9 units, rays aimed at and just past
    /// surface points (off by up to three times the solver's reach), from in
    /// front of and behind them.
    #[test]
    fn prefilter_never_skips_a_ray_the_solver_hits() {
        #![allow(clippy::cast_precision_loss)]
        let (mut hit, mut skipped) = (0, 0);
        let mut sample = 0_u32;
        let mut next = |base: u32| {
            sample += 1;
            halton(sample, base)
        };
        for scale in [1e-3, 1.0, 1e3, 1e6, 1e9] {
            let n = 6;
            let mut knots = vec![0.0; 4];
            knots.extend([0.3, 0.7]);
            knots.extend([1.0; 4]);
            let control: Vec<Vec<Point3>> = (0..n)
                .map(|i| {
                    (0..n)
                        .map(|j| {
                            let bump = next(2) - 0.5;
                            Point3::new(
                                scale * (i as f64 / 5.0 + 3.0),
                                scale * (j as f64 / 5.0 - 1.0),
                                scale * 0.4 * bump,
                            )
                        })
                        .collect()
                })
                .collect();
            let weights = (0..n)
                .map(|_| (0..n).map(|_| 0.5 + 1.5 * next(3)).collect())
                .collect();
            let surface = NurbsSurface::new(3, 3, knots.clone(), knots, control, weights).unwrap();
            let boxes = SpanBoxes::for_surface(&surface).unwrap();
            for _ in 0..120 {
                let target = surface.evaluate(next(5), next(7));
                let direction = Vec3::new(next(11) - 0.5, next(13) - 0.5, next(17) - 0.5);
                let Ok(unit) = direction.normalize() else {
                    continue;
                };
                let side = unit.cross(Vec3::new(0.3, -0.7, 0.2)).normalize().unwrap();
                let reach =
                    remus_math::nurbs::intersection::line_hit_reach(boxes.magnitude, target);
                let past = target + side * (3.0 * reach * next(19));
                let origin = past - unit * (scale * (3.0 * next(23) - 1.0));
                let hits = forward_hits(&surface, origin, unit);
                let meets = boxes.meet(origin, unit);
                assert!(hits == 0 || meets, "scale {scale}: {hits} hits skipped");
                hit += usize::from(hits > 0);
                skipped += usize::from(!meets);
            }
        }
        assert!(hit > 100 && skipped > 20, "{hit} hit, {skipped} skipped");
    }

    #[test]
    fn ray_box_test_keeps_every_ray_within_reach() {
        let aabb = Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(1.0, 1.0, 1.0),
        };
        let x = Vec3::new(1.0, 0.0, 0.0);
        // Through, grazing within reach, starting inside, and behind.
        assert!(ray_meets_box(Point3::new(-1.0, 0.5, 0.5), x, &aabb, 0.0));
        assert!(ray_meets_box(
            Point3::new(-1.0, 1.0 + 0.5e-5, 0.5),
            x,
            &aabb,
            1e-5
        ));
        assert!(ray_meets_box(Point3::new(0.5, 0.5, 0.5), -x, &aabb, 0.0));
        assert!(!ray_meets_box(Point3::new(2.0, 0.5, 0.5), x, &aabb, 1e-5));
        assert!(!ray_meets_box(
            Point3::new(-1.0, 1.0 + 2e-5, 0.5),
            x,
            &aabb,
            1e-5
        ));
        // A non-finite origin never reads as a miss.
        assert!(ray_meets_box(
            Point3::new(f64::NAN, 0.5, 0.5),
            x,
            &aabb,
            0.0
        ));
    }
}
