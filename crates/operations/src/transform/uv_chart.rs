//! Re-expressing a face's pcurves when an affine map moves its UV chart.
//!
//! [`super::transform_face_surface_recorded`] rebuilds every face carrier
//! from the mapped geometry. A plane's chart is canonical in its normal and
//! offset (the chart the strict `SameParameter` proof and the STEP reader
//! and writer share), so after a translation or rotation the old `(u, v)`
//! names a different point. A cylinder or cone is rebuilt around its image
//! axis with a canonical reference direction, which turns `u` by a phase;
//! a scale stretches their axial coordinate and a reflection reverses `u`.
//! Each of those is an affine map of the parameter plane,
//! returned as a [`FaceChartMap`] and applied to the face's stored pcurves
//! by [`apply_face_chart_map`]. A pcurve with no exact image under the map
//! is dropped, as are all pcurves of a replaced chart; the 3D edges stay
//! exact either way.
//!
//! A periodic winding counts the lifts of a pcurve's ends over the
//! principal projections of its vertices. Moving the vertices can carry a
//! seam point's principal coordinate from `0` to just below the period, so
//! windings on periodic carriers are re-derived after every transform.

use remus_math::curves2d::{Circle2D, Curve2D, Ellipse2D, Line2D, NurbsCurve2D};
use remus_math::frame::Frame3;
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::coedge::{CoedgeId, PeriodicWinding};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;

use super::{ANALYTIC_ROUNDOFF_REL, equal_within_analytic_roundoff};

/// `uv' = A·uv + b`: a point's coordinates in the source chart mapped to
/// the coordinates of its image in the transformed chart.
#[derive(Debug, Clone, Copy)]
pub struct UvAffine {
    /// Row-major linear part `A`.
    a: [[f64; 2]; 2],
    b: Vec2,
}

/// The extents of a retained planar conic edge, already transformed. A
/// conic pcurve on a plane takes them when they agree with its own mapped
/// extents within analytic roundoff, so the pcurve and the edge carrier
/// share one scale convention.
#[derive(Debug, Clone, Copy)]
pub enum EdgeExtent {
    /// A circle's radius.
    Circle(f64),
    /// An ellipse's semi-major and semi-minor axes.
    Ellipse(f64, f64),
}

/// How a transformed face's chart relates to its source chart.
#[derive(Debug, Clone, Copy)]
pub enum FaceChartMap {
    /// The carrier keeps its chart; stored pcurves stay valid as they are.
    Unchanged,
    /// The new chart is an affine image of the old one.
    Affine(UvAffine),
    /// The chart was replaced (NURBS conversion, or a reflected NURBS
    /// carrier); stored pcurves are dropped.
    Replaced,
}

/// The linear part of `matrix` applied to a direction, without
/// normalizing: its length is the map's scale along `direction`.
pub fn linear_image(matrix: &Mat4, direction: Vec3) -> Vec3 {
    let m = &matrix.0;
    Vec3::new(
        m[0][0].mul_add(
            direction.x(),
            m[0][1].mul_add(direction.y(), m[0][2] * direction.z()),
        ),
        m[1][0].mul_add(
            direction.x(),
            m[1][1].mul_add(direction.y(), m[1][2] * direction.z()),
        ),
        m[2][0].mul_add(
            direction.x(),
            m[2][1].mul_add(direction.y(), m[2][2] * direction.z()),
        ),
    )
}

/// The chart of a `Plane { normal, d }` face: origin `normal·d`, axes from
/// [`Frame3::from_normal`]. This is the chart the strict `SameParameter`
/// proof (`PlaneChart::canonical`) and the STEP reader and writer evaluate
/// plane pcurves in; all of them must agree.
fn plane_chart(normal: Vec3, d: f64) -> Result<Frame3, crate::OperationsError> {
    let origin = Point3::new(normal.x() * d, normal.y() * d, normal.z() * d);
    Ok(Frame3::from_normal(origin, normal)?)
}

impl UvAffine {
    /// The map between the charts of an axial carrier (angle `u` about its
    /// axis, `v` along it) and its image rebuilt around the image axis with
    /// a canonical reference direction: `u' = ±u + φ`, `v' = v_scale·v`.
    ///
    /// `source` and `target` are the carriers' `(x_axis, y_axis)`. `φ` is the
    /// angle of the mapped source `x_axis` in the target frame, and `u`
    /// reverses when the mapped frame is left-handed against the target (a
    /// reflection). Nothing is snapped: a tiny `φ` or a near-unit `v_scale`
    /// can be a genuine change that moves a large model's boundary.
    pub fn about_axis(
        matrix: &Mat4,
        source: (Vec3, Vec3),
        target: (Vec3, Vec3),
        v_scale: f64,
    ) -> Self {
        let x = linear_image(matrix, source.0);
        let y = linear_image(matrix, source.1);
        let (xx, xy) = (x.dot(target.0), x.dot(target.1));
        let (yx, yy) = (y.dot(target.0), y.dot(target.1));
        let sign = if xx.mul_add(yy, -(xy * yx)) < 0.0 {
            -1.0
        } else {
            1.0
        };
        Self {
            a: [[sign, 0.0], [0.0, v_scale]],
            b: Vec2::new(xy.atan2(xx), 0.0),
        }
    }

    /// `u' = −u`: a carrier whose frame is the mapped frame made
    /// right-handed again after a reflection (sphere, torus). The pole or
    /// ring axis keeps its image, so `v` is unchanged.
    pub const fn reversed_u() -> Self {
        Self {
            a: [[-1.0, 0.0], [0.0, 1.0]],
            b: Vec2::new(0.0, 0.0),
        }
    }

    /// The map between the canonical charts of a plane and its image.
    ///
    /// Exact for every affine `matrix`: the image of a planar chart is an
    /// affine chart of the image plane.
    pub fn plane(
        matrix: &Mat4,
        source: (Vec3, f64),
        target: (Vec3, f64),
    ) -> Result<Self, crate::OperationsError> {
        let source = plane_chart(source.0, source.1)?;
        let target = plane_chart(target.0, target.1)?;
        let x = linear_image(matrix, source.x);
        let y = linear_image(matrix, source.y);
        let offset = matrix.mul_point(source.origin) - target.origin;
        Ok(Self {
            a: [
                [x.dot(target.x), y.dot(target.x)],
                [x.dot(target.y), y.dot(target.y)],
            ],
            b: Vec2::new(offset.dot(target.x), offset.dot(target.y)),
        })
    }

    #[allow(clippy::float_cmp)] // Only an exact identity may skip the rewrite.
    fn is_identity(&self) -> bool {
        self.a == [[1.0, 0.0], [0.0, 1.0]] && self.b.x() == 0.0 && self.b.y() == 0.0
    }

    fn vector(&self, v: Vec2) -> Vec2 {
        Vec2::new(
            self.a[0][0].mul_add(v.x(), self.a[0][1] * v.y()),
            self.a[1][0].mul_add(v.x(), self.a[1][1] * v.y()),
        )
    }

    fn point(&self, p: Point2) -> Point2 {
        let v = self.vector(Vec2::new(p.x(), p.y()));
        Point2::new(v.x() + self.b.x(), v.y() + self.b.y())
    }

    fn determinant(&self) -> f64 {
        self.a[0][0].mul_add(self.a[1][1], -(self.a[0][1] * self.a[1][0]))
    }

    /// The uniform scale of a conformal linear part (a 2D similarity), or
    /// `None`. Conics keep their trigonometric parameter only under these.
    fn conformal_scale(&self) -> Option<f64> {
        let c0 = Vec2::new(self.a[0][0], self.a[1][0]);
        let c1 = Vec2::new(self.a[0][1], self.a[1][1]);
        let (l0, l1) = (c0.length(), c1.length());
        let orthogonal = c0.dot(c1).abs() <= ANALYTIC_ROUNDOFF_REL * l0 * l1;
        (equal_within_analytic_roundoff(l0, l1) && orthogonal).then_some(0.5 * (l0 + l1))
    }

    /// The pcurve's image: the same points, re-expressed in the new chart,
    /// with the parameter range carried so each end still lands on its
    /// vertex. `None` when no curve of the same family represents the image
    /// with a parameter affine in the old one (a conic under a non-conformal
    /// map) or the image is degenerate.
    ///
    /// On a plane, `edge` is the retained 3D circle's radius or ellipse's
    /// semi-axes. Its frame chooses the scale inside the analytic roundoff
    /// band; independently averaging UV scales can move a large conic
    /// beyond the linear tolerance. Other surfaces' UV extents are not 3D
    /// lengths.
    fn map_pcurve(&self, pcurve: &PCurve, edge: Option<EdgeExtent>) -> Option<PCurve> {
        let (t0, t1) = (pcurve.t_start(), pcurve.t_end());
        let positive = self.determinant() > 0.0;
        let (curve, start, end) = match pcurve.curve() {
            Curve2D::Line(line) => {
                // `Line2D` stores a unit direction, so its parameter is arc
                // length and scales with the direction's image.
                let direction = self.vector(line.direction());
                let scale = direction.length();
                if !(scale.is_finite() && scale > 0.0) {
                    return None;
                }
                // Normalized here: `Line2D::new` refuses directions shorter
                // than 1e-15, which a valid tiny-scale map produces.
                let unit = Vec2::new(direction.x() / scale, direction.y() / scale);
                let line = Line2D::new(self.point(line.origin()), unit).ok()?;
                (Curve2D::Line(line), t0 * scale, t1 * scale)
            }
            Curve2D::Circle(circle) => {
                let scale = self.conformal_scale()?;
                // A rotation by `phase` shifts the angle; a reflection about
                // the axis at `phase/2` also reverses it.
                let axis = self.vector(Vec2::new(1.0, 0.0));
                let phase = axis.y().atan2(axis.x());
                let mapped_radius = circle.radius() * scale;
                let radius = match edge {
                    Some(EdgeExtent::Circle(radius)) => radius,
                    _ => mapped_radius,
                };
                // Only reconcile the existing analytic roundoff convention,
                // never an unrelated disagreement between input carriers.
                if !equal_within_analytic_roundoff(radius, mapped_radius) {
                    return None;
                }
                let image = Circle2D::new(self.point(circle.center()), radius);
                let (start, end) = if positive {
                    (t0 + phase, t1 + phase)
                } else {
                    (phase - t0, phase - t1)
                };
                (Curve2D::Circle(image.ok()?), start, end)
            }
            Curve2D::Ellipse(ellipse) => {
                let scale = self.conformal_scale()?;
                let (sin, cos) = ellipse.rotation().sin_cos();
                let major = self.vector(Vec2::new(cos, sin));
                let mapped = (ellipse.semi_major() * scale, ellipse.semi_minor() * scale);
                // As for the circle. The edge keeps its axes in order, so a
                // map that swaps them (only possible at roundoff) drops the
                // pcurve rather than relabel its parameter. Axes equal within
                // roundoff leave that order ambiguous, since the edge may have
                // swapped them with a quarter-turn shift the pcurve cannot see,
                // so such a near-circular ellipse drops its pcurve too.
                let (semi_major, semi_minor) = match edge {
                    Some(EdgeExtent::Ellipse(a, b)) => {
                        if equal_within_analytic_roundoff(a, b)
                            || !(equal_within_analytic_roundoff(a, mapped.0)
                                && equal_within_analytic_roundoff(b, mapped.1))
                        {
                            return None;
                        }
                        (a, b)
                    }
                    _ => mapped,
                };
                let image = Ellipse2D::new(
                    self.point(ellipse.center()),
                    semi_major,
                    semi_minor,
                    major.y().atan2(major.x()),
                );
                let (start, end) = if positive { (t0, t1) } else { (-t0, -t1) };
                (Curve2D::Ellipse(image.ok()?), start, end)
            }
            Curve2D::Nurbs(nurbs) => {
                // Rational curves are invariant under affine maps of their
                // control points, parameter included.
                let image = NurbsCurve2D::new(
                    nurbs.degree(),
                    nurbs.knots().to_vec(),
                    nurbs
                        .control_points()
                        .iter()
                        .map(|&point| self.point(point))
                        .collect(),
                    nurbs.weights().to_vec(),
                );
                (Curve2D::Nurbs(image.ok()?), t0, t1)
            }
        };
        let image = PCurve::new(curve, start, end);
        let finite = |p: Point2| p.x().is_finite() && p.y().is_finite();
        (start.is_finite()
            && end.is_finite()
            && finite(image.evaluate(start))
            && finite(image.evaluate(end)))
        .then_some(image)
    }
}

/// Periods of a carrier's `(u, v)` coordinates, as the STEP writer reads
/// them when it checks windings.
fn periods(surface: &FaceSurface) -> (Option<f64>, Option<f64>) {
    use std::f64::consts::TAU;
    match surface {
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_) | FaceSurface::Sphere(_) => {
            (Some(TAU), None)
        }
        FaceSurface::Torus(_) => (Some(TAU), Some(TAU)),
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => (None, None),
    }
}

/// Whole periods between a pcurve end coordinate and the principal
/// coordinate of its vertex, or `None` when they are not an integral lift
/// within the STEP writer's branch tolerance.
fn lift(value: f64, principal: f64, period: f64) -> Option<i32> {
    // Same certification as `export_pcurve_winding` in the STEP writer.
    const BRANCH_TOLERANCE: f64 = 1e-10;
    let uncertainty = 8.0 * f64::EPSILON * value.abs().max(principal.abs()).max(period);
    let turns = (value - principal) / period;
    let nearest = turns.round();
    if !turns.is_finite()
        || uncertainty > BRANCH_TOLERANCE
        || (turns - nearest).abs() * period > BRANCH_TOLERANCE
        || nearest < f64::from(i32::MIN)
        || nearest > f64::from(i32::MAX)
    {
        return None;
    }
    #[allow(clippy::cast_possible_truncation)] // Range-checked above.
    Some(nearest as i32)
}

/// The winding the STEP writer derives for `pcurve` on `surface`: per
/// periodic axis, the lift shared by both ends, or zero when the ends
/// disagree. `None` when an end is not an integral lift.
fn derived_winding(
    surface: &FaceSurface,
    pcurve: &PCurve,
    start: Point3,
    end: Point3,
) -> Option<PeriodicWinding> {
    let (u_period, v_period) = periods(surface);
    let principal_start = surface.project_point(start)?;
    let principal_end = surface.project_point(end)?;
    let uv_start = pcurve.evaluate(pcurve.t_start());
    let uv_end = pcurve.evaluate(pcurve.t_end());
    let axis = |first: f64, last: f64, p_first: f64, p_last: f64, period: Option<f64>| {
        let Some(period) = period else {
            return Some(0);
        };
        let (first, last) = (lift(first, p_first, period)?, lift(last, p_last, period)?);
        Some(if first == last { first } else { 0 })
    };
    Some(PeriodicWinding::new(
        axis(
            uv_start.x(),
            uv_end.x(),
            principal_start.0,
            principal_end.0,
            u_period,
        )?,
        axis(
            uv_start.y(),
            uv_end.y(),
            principal_start.1,
            principal_end.1,
            v_period,
        )?,
    ))
}

/// Rewrites every pcurve stored on `face` for its transformed chart, then
/// re-derives the periodic windings of those that remain.
///
/// A dropped pcurve also clears its use's winding, which counts lifts of
/// that pcurve and is meaningless without it. A rewritten pcurve whose
/// winding cannot be re-derived is dropped too; on an unchanged chart such
/// a winding is left as it was.
///
/// # Errors
///
/// Returns a topology error when the face or one of its loops, coedges,
/// edges or vertices is stale.
pub fn apply_face_chart_map(
    topo: &mut Topology,
    face: FaceId,
    map: &FaceChartMap,
) -> Result<(), crate::OperationsError> {
    let rewrite = match map {
        FaceChartMap::Unchanged => None,
        FaceChartMap::Affine(affine) if affine.is_identity() => None,
        FaceChartMap::Affine(affine) => Some(Some(affine)),
        FaceChartMap::Replaced => Some(None),
    };
    let surface = topo.face(face)?.surface().clone();
    let periodic = periods(&surface) != (None, None);
    if rewrite.is_none() && !periodic {
        return Ok(());
    }
    let mut uses: Vec<CoedgeId> = Vec::new();
    for &loop_id in topo.face(face)?.boundary_loops() {
        uses.extend_from_slice(topo.face_loop(loop_id)?.coedges());
    }
    // Edges whose pcurve was dropped on this face.
    let mut dropped = Vec::new();
    for &coedge in &uses {
        let Some(pcurve) = topo.coedge(coedge)?.pcurve() else {
            continue;
        };
        let edge_id = topo.coedge(coedge)?.edge();
        let edge_extent = if surface.is_planar() {
            match topo.edge(edge_id)?.curve() {
                EdgeCurve::Circle(circle) => Some(EdgeExtent::Circle(circle.radius())),
                EdgeCurve::Ellipse(ellipse) => Some(EdgeExtent::Ellipse(
                    ellipse.semi_major(),
                    ellipse.semi_minor(),
                )),
                EdgeCurve::Line
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => None,
            }
        } else {
            None
        };
        let pcurve = match rewrite {
            None => pcurve.clone(),
            Some(target) => {
                if let Some(image) =
                    target.and_then(|affine| affine.map_pcurve(pcurve, edge_extent))
                {
                    topo.set_coedge_pcurve(coedge, image.clone())?;
                    image
                } else {
                    topo.remove_coedge_pcurve(coedge)?;
                    if topo.coedge(coedge)?.periodic_winding() != PeriodicWinding::ZERO {
                        topo.set_coedge_periodic_winding(coedge, PeriodicWinding::ZERO)?;
                    }
                    dropped.push(edge_id);
                    continue;
                }
            }
        };
        if !periodic {
            continue;
        }
        let use_data = topo.coedge(coedge)?;
        let (stored, forward) = (use_data.periodic_winding(), use_data.is_forward());
        let edge = topo.edge(use_data.edge())?;
        let (start, end) = if forward {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        let (start, end) = (topo.vertex(start)?.point(), topo.vertex(end)?.point());
        match derived_winding(&surface, &pcurve, start, end) {
            Some(winding) if winding != stored => {
                topo.set_coedge_periodic_winding(coedge, winding)?;
            }
            Some(_) => {}
            // A rewritten pcurve whose ends no longer certify as lifts (one
            // ends on a cone apex, where the principal `u` is always 0 but
            // the turned chart moved the pcurve's) would make STEP export
            // refuse the body: drop it rather than keep a stale winding.
            None if rewrite.is_some() => {
                topo.remove_coedge_pcurve(coedge)?;
                if stored != PeriodicWinding::ZERO {
                    topo.set_coedge_periodic_winding(coedge, PeriodicWinding::ZERO)?;
                }
                dropped.push(edge_id);
            }
            None => {}
        }
    }
    // A repeated edge (a seam or slit) carries a pcurve on both of its uses
    // or on neither, so dropping one branch drops its partner too.
    if !dropped.is_empty() {
        for &coedge in &uses {
            let data = topo.coedge(coedge)?;
            let (edge_id, has_pcurve, winding) = (
                data.edge(),
                data.pcurve().is_some(),
                data.periodic_winding(),
            );
            if has_pcurve && dropped.contains(&edge_id) {
                topo.remove_coedge_pcurve(coedge)?;
                if winding != PeriodicWinding::ZERO {
                    topo.set_coedge_periodic_winding(coedge, PeriodicWinding::ZERO)?;
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_topology::explorer::solid_faces;
    use remus_topology::solid::SolidId;

    fn assert_same_points(source: &PCurve, image: &PCurve, map: &UvAffine) {
        for k in 0..=16 {
            let f = f64::from(k) / 16.0;
            let expected = map
                .point(source.evaluate(source.t_start() + f * (source.t_end() - source.t_start())));
            let actual = image.evaluate(image.t_start() + f * (image.t_end() - image.t_start()));
            assert!(
                (actual - expected).length() < 1e-12,
                "sample {k}: {actual:?} != {expected:?}"
            );
        }
    }

    fn rotation(angle: f64, scale: f64, reflect: bool) -> UvAffine {
        let (sin, cos) = angle.sin_cos();
        let flip = if reflect { -1.0 } else { 1.0 };
        UvAffine {
            a: [
                [scale * cos, -scale * sin * flip],
                [scale * sin, scale * cos * flip],
            ],
            b: Vec2::new(3.0, -1.5),
        }
    }

    #[test]
    fn conformal_maps_carry_every_curve_family_at_matching_parameters() {
        let circle = PCurve::new(
            Curve2D::Circle(Circle2D::new(Point2::new(1.0, 2.0), 0.75).unwrap()),
            0.4,
            2.1,
        );
        let ellipse = PCurve::new(
            Curve2D::Ellipse(Ellipse2D::new(Point2::new(-1.0, 0.5), 2.0, 0.5, 0.3).unwrap()),
            2.0,
            -1.0,
        );
        let line = PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(0.5, 0.5), Vec2::new(0.6, 0.8)).unwrap()),
            -1.0,
            2.0,
        );
        let nurbs = PCurve::new(
            Curve2D::Nurbs(
                NurbsCurve2D::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![
                        Point2::new(1.0, 0.0),
                        Point2::new(1.0, 1.0),
                        Point2::new(0.0, 1.0),
                    ],
                    vec![1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
                )
                .unwrap(),
            ),
            0.1,
            0.9,
        );
        for map in [
            rotation(0.0, 1.0, false),
            rotation(1.3, 1.0, false),
            rotation(-2.6, 2.5, false),
            rotation(0.9, 1.0, true),
            rotation(2.2, 0.4, true),
        ] {
            for source in [&circle, &ellipse, &line, &nurbs] {
                let image = map.map_pcurve(source, None).expect("conformal image");
                assert_same_points(source, &image, &map);
            }
        }
    }

    #[test]
    fn anisotropic_maps_keep_lines_and_splines_but_refuse_conics() {
        let map = UvAffine {
            a: [[1.0, 0.0], [0.0, 2.0]],
            b: Vec2::new(0.0, 0.0),
        };
        let line = PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(0.0, 1.0), Vec2::new(0.0, -1.0)).unwrap()),
            0.0,
            4.0,
        );
        let image = map.map_pcurve(&line, None).expect("line image");
        assert_same_points(&line, &image, &map);
        assert_eq!(image.t_end().to_bits(), 8.0_f64.to_bits());
        let circle = PCurve::new(
            Curve2D::Circle(Circle2D::new(Point2::new(0.0, 0.0), 1.0).unwrap()),
            0.0,
            1.0,
        );
        assert!(map.map_pcurve(&circle, None).is_none());
        assert!(
            map.map_pcurve(&circle, Some(EdgeExtent::Circle(3.0)))
                .is_none()
        );
    }

    #[test]
    fn retained_ellipse_axes_only_reconcile_analytic_roundoff() {
        let map = rotation(0.3, 1.0, false);
        let ellipse = PCurve::new(
            Curve2D::Ellipse(Ellipse2D::new(Point2::new(1.0, -2.0), 2.0, 1.0, 0.2).unwrap()),
            0.0,
            1.0,
        );
        // The edge scaled each axis on its own, 4 ulp apart: the pcurve takes
        // the edge's extents exactly.
        let edge = (2.0 * (1.0 + 4.0 * f64::EPSILON), 1.0);
        let image = map
            .map_pcurve(&ellipse, Some(EdgeExtent::Ellipse(edge.0, edge.1)))
            .expect("roundoff-close ellipse image");
        let Curve2D::Ellipse(mapped) = image.curve() else {
            unreachable!("an ellipse maps to an ellipse");
        };
        assert_eq!(mapped.semi_major().to_bits(), edge.0.to_bits());
        assert_eq!(mapped.semi_minor().to_bits(), edge.1.to_bits());
        // Genuinely different or swapped extents never rescue the pcurve.
        assert!(
            map.map_pcurve(&ellipse, Some(EdgeExtent::Ellipse(2.1, 1.0)))
                .is_none()
        );
        assert!(
            map.map_pcurve(&ellipse, Some(EdgeExtent::Ellipse(1.0, 2.0)))
                .is_none()
        );
        // Near-circular: the edge may have swapped axes unseen by the pcurve.
        let minor = 1.0 - 2.0 * f64::EPSILON;
        let near_circle = PCurve::new(
            Curve2D::Ellipse(Ellipse2D::new(Point2::new(0.0, 0.0), 1.0, minor, 0.0).unwrap()),
            0.0,
            1.0,
        );
        assert!(
            map.map_pcurve(&near_circle, Some(EdgeExtent::Ellipse(1.0, minor)))
                .is_none()
        );
    }

    #[test]
    fn seam_pcurves_are_dropped_as_a_pair() {
        // The cylinder's seam is used twice by its lateral face. A uniform
        // scale stretches only the axial `v`, so the chart map is not
        // conformal: an ellipse branch has no image while a line branch has.
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut topo, 1.0, 2.0).unwrap();
        let lateral = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
            .unwrap();
        let mut uses = Vec::new();
        for &loop_id in topo.face(lateral).unwrap().boundary_loops() {
            uses.extend_from_slice(topo.face_loop(loop_id).unwrap().coedges());
        }
        let seam: Vec<CoedgeId> = uses
            .into_iter()
            .filter(|&coedge| {
                let edge = topo.coedge(coedge).unwrap().edge();
                matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line)
            })
            .collect();
        assert_eq!(seam.len(), 2, "cylinder seam uses");
        // The line branch lies exactly on its use, so it maps and certifies
        // on its own; only the pairing can drop it.
        let surface = topo.face(lateral).unwrap().surface().clone();
        let data = topo.coedge(seam[0]).unwrap();
        let edge = topo.edge(data.edge()).unwrap();
        let (from, to) = if data.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        let from = surface
            .project_point(topo.vertex(from).unwrap().point())
            .unwrap();
        let to = surface
            .project_point(topo.vertex(to).unwrap().point())
            .unwrap();
        let line = PCurve::new(
            Curve2D::Line(
                Line2D::new(Point2::new(from.0, from.1), Vec2::new(0.0, to.1 - from.1)).unwrap(),
            ),
            0.0,
            (to.1 - from.1).abs(),
        );
        let ellipse = PCurve::new(
            Curve2D::Ellipse(Ellipse2D::new(Point2::new(0.0, 1.0), 1.0, 0.5, 0.0).unwrap()),
            0.0,
            1.0,
        );
        topo.set_coedge_pcurve(seam[0], line).unwrap();
        topo.set_coedge_pcurve(seam[1], ellipse).unwrap();

        super::super::transform_solid(&mut topo, solid, &Mat4::scale(2.0, 2.0, 2.0)).unwrap();
        let kept: Vec<bool> = seam
            .iter()
            .map(|&coedge| topo.coedge(coedge).unwrap().pcurve().is_some())
            .collect();
        assert_eq!(kept, [false, false], "a half-populated seam is invalid");
        for &coedge in &seam {
            assert_eq!(
                topo.coedge(coedge).unwrap().periodic_winding(),
                PeriodicWinding::ZERO
            );
        }
    }

    #[test]
    fn retained_circle_radius_only_reconciles_analytic_roundoff() {
        let map = rotation(0.0, 1.0, false);
        let circle = PCurve::new(
            Curve2D::Circle(Circle2D::new(Point2::new(0.0, 0.0), 1.0).unwrap()),
            0.0,
            1.0,
        );
        // A different 3D radius must not rescue an inconsistent pcurve.
        assert!(
            map.map_pcurve(&circle, Some(EdgeExtent::Circle(1.01)))
                .is_none()
        );
        assert!(
            map.map_pcurve(&circle, Some(EdgeExtent::Circle(f64::NAN)))
                .is_none()
        );
    }

    /// Every pcurve use on the curved faces of `solid`, keyed by face
    /// position and use position, with its lifted samples.
    fn lifted_uses(topo: &Topology, solid: SolidId) -> Vec<(FaceId, CoedgeId, Vec<Point3>)> {
        let mut uses = Vec::new();
        for face in solid_faces(topo, solid).unwrap() {
            let surface = topo.face(face).unwrap().surface().clone();
            for &loop_id in topo.face(face).unwrap().boundary_loops() {
                for &coedge in topo.face_loop(loop_id).unwrap().coedges() {
                    let Some(pcurve) = topo.coedge(coedge).unwrap().pcurve() else {
                        continue;
                    };
                    let samples = (0..=16)
                        .map(|k| {
                            let f = f64::from(k) / 16.0;
                            let uv = pcurve.evaluate(
                                pcurve.t_start() + f * (pcurve.t_end() - pcurve.t_start()),
                            );
                            surface.evaluate(uv.x(), uv.y()).unwrap()
                        })
                        .collect();
                    uses.push((face, coedge, samples));
                }
            }
        }
        uses
    }

    /// Stores a fitted pcurve on every use of every curved face.
    fn attach_projected_pcurves(topo: &mut Topology, solid: SolidId) {
        let tolerance = remus_math::tolerance::Tolerance::new();
        for face in solid_faces(topo, solid).unwrap() {
            if topo.face(face).unwrap().surface().is_planar() {
                continue;
            }
            let mut uses = Vec::new();
            for &loop_id in topo.face(face).unwrap().boundary_loops() {
                uses.extend_from_slice(topo.face_loop(loop_id).unwrap().coedges());
            }
            for coedge in uses {
                let (edge, forward) = {
                    let data = topo.coedge(coedge).unwrap();
                    (data.edge(), data.is_forward())
                };
                let Ok(fit) = remus_heal::construct::project_curve::project_edge_to_pcurve(
                    topo, edge, face, 16, &tolerance,
                ) else {
                    continue;
                };
                let curve = NurbsCurve2D::new(
                    fit.degree(),
                    fit.knots().to_vec(),
                    fit.control_points()
                        .iter()
                        .map(|point| Point2::new(point.x(), point.y()))
                        .collect(),
                    fit.weights().to_vec(),
                )
                .unwrap();
                let (first, last) = (fit.knots()[0], fit.knots()[fit.knots().len() - 1]);
                let (start, end) = if forward {
                    (first, last)
                } else {
                    (last, first)
                };
                topo.set_coedge_pcurve(coedge, PCurve::new(Curve2D::Nurbs(curve), start, end))
                    .unwrap();
            }
        }
    }

    #[test]
    fn similarity_maps_keep_curved_face_pcurves_on_their_points() {
        use crate::primitives::{make_cone, make_cylinder, make_sphere, make_torus};
        type Build = fn(&mut Topology) -> SolidId;
        let bodies: [(&str, Build); 4] = [
            ("cylinder", |t| make_cylinder(t, 1.5, 4.0).unwrap()),
            ("cone", |t| make_cone(t, 2.0, 0.5, 3.0).unwrap()),
            ("sphere", |t| make_sphere(t, 2.0, 16).unwrap()),
            ("torus", |t| make_torus(t, 3.0, 1.0, 16).unwrap()),
        ];
        let matrices = [
            (
                "rigid",
                Mat4::translation(4.0, -1.0, 2.0) * Mat4::rotation_y(0.8) * Mat4::rotation_z(-0.4),
            ),
            (
                "mirror",
                Mat4::rotation_x(0.6) * Mat4::scale(1.0, -1.0, 1.0),
            ),
            ("scale", Mat4::rotation_z(2.0) * Mat4::scale(2.5, 2.5, 2.5)),
        ];
        for (body, build) in bodies {
            let mut source = Topology::new();
            let solid = build(&mut source);
            attach_projected_pcurves(&mut source, solid);
            let before = lifted_uses(&source, solid);
            assert!(before.len() >= 3, "{body}: too few pcurve uses");
            for (label, matrix) in matrices {
                let mut topo = source.clone();
                super::super::transform_solid(&mut topo, solid, &matrix).unwrap();
                let after = lifted_uses(&topo, solid);
                assert_eq!(after.len(), before.len(), "{body}/{label}: pcurves dropped");
                for ((_, coedge, old), (_, new_coedge, new)) in before.iter().zip(&after) {
                    assert_eq!(coedge, new_coedge);
                    for (k, (old, new)) in old.iter().zip(new).enumerate() {
                        let expected = matrix.mul_point(*old);
                        assert!(
                            (*new - expected).length() < 1e-9,
                            "{body}/{label} use {coedge:?} sample {k}: {new:?} != {expected:?}"
                        );
                    }
                }
                for (face, coedge, _) in &after {
                    let surface = topo.face(*face).unwrap().surface().clone();
                    let data = topo.coedge(*coedge).unwrap();
                    let edge = topo.edge(data.edge()).unwrap();
                    let (start, end) = if data.is_forward() {
                        (edge.start(), edge.end())
                    } else {
                        (edge.end(), edge.start())
                    };
                    let derived = derived_winding(
                        &surface,
                        data.pcurve().unwrap(),
                        topo.vertex(start).unwrap().point(),
                        topo.vertex(end).unwrap().point(),
                    );
                    if let Some(derived) = derived {
                        assert_eq!(data.periodic_winding(), derived, "{body}/{label}");
                    }
                }
            }
        }
    }

    #[test]
    fn apex_pcurves_that_lose_their_lift_are_dropped() {
        use std::f64::consts::TAU;
        // A pointed cone's seam runs from the base to the apex, where the
        // projection always reports `u = 0`. Turning the chart by 0.3 moves
        // the seam pcurves' apex ends to `u = 0.3`, which the STEP writer
        // cannot certify as a lift.
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cone(&mut topo, 2.0, 0.0, 3.0).unwrap();
        // `make_cone` puts its seam at `u = 3π/2`, which no apex end can
        // certify; turn it to `u = 0` (the axis points down -z) first, so the
        // seam pcurves export before the transform under test.
        super::super::transform_solid(
            &mut topo,
            solid,
            &Mat4::rotation_z(1.5 * std::f64::consts::PI),
        )
        .unwrap();
        let cone_face = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cone(_)))
            .unwrap();
        let surface = topo.face(cone_face).unwrap().surface().clone();
        let mut uses = Vec::new();
        for &loop_id in topo.face(cone_face).unwrap().boundary_loops() {
            uses.extend_from_slice(topo.face_loop(loop_id).unwrap().coedges());
        }
        let ends = |topo: &Topology, coedge: CoedgeId| {
            let data = topo.coedge(coedge).unwrap();
            let edge = topo.edge(data.edge()).unwrap();
            let (a, b) = if data.is_forward() {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            (
                topo.vertex(a).unwrap().point(),
                topo.vertex(b).unwrap().point(),
            )
        };
        let mut seams = Vec::new();
        for coedge in uses {
            let edge = topo.coedge(coedge).unwrap().edge();
            if !matches!(
                topo.edge(edge).unwrap().curve(),
                remus_topology::edge::EdgeCurve::Line
            ) {
                continue;
            }
            let (start, end) = ends(&topo, coedge);
            let (uv_start, uv_end) = (
                surface.project_point(start).unwrap(),
                surface.project_point(end).unwrap(),
            );
            let (v_start, v_end) = (uv_start.1, uv_end.1);
            // The base end's `u`; the other side of the seam one turn on.
            let base_u = if v_start > v_end {
                uv_start.0
            } else {
                uv_end.0
            };
            let u = if seams.is_empty() {
                base_u
            } else {
                base_u + TAU
            };
            let length = (v_end - v_start).abs();
            let line = Line2D::new(
                Point2::new(u, v_start),
                Vec2::new(0.0, (v_end - v_start).signum()),
            )
            .unwrap();
            let pcurve = PCurve::new(Curve2D::Line(line), 0.0, length);
            assert!(derived_winding(&surface, &pcurve, start, end).is_some());
            topo.set_coedge_pcurve(coedge, pcurve).unwrap();
            seams.push(coedge);
        }
        assert_eq!(seams.len(), 2, "pointed cone seam uses");

        super::super::transform_solid(&mut topo, solid, &Mat4::rotation_z(0.3)).unwrap();
        let surface = topo.face(cone_face).unwrap().surface().clone();
        for coedge in seams {
            let data = topo.coedge(coedge).unwrap();
            if let Some(pcurve) = data.pcurve() {
                let (start, end) = ends(&topo, coedge);
                assert!(
                    derived_winding(&surface, pcurve, start, end).is_some(),
                    "kept pcurve must stay exportable"
                );
            } else {
                assert_eq!(data.periodic_winding(), PeriodicWinding::ZERO);
            }
        }
    }

    #[test]
    fn tiny_genuine_axial_changes_are_not_snapped_away() {
        // A rotation about the carrier's own axis by `angle` shifts `u` by
        // exactly `angle`. On a radius-1e9 cylinder 3e-15 rad moves the
        // boundary ~3e-6, far beyond the linear tolerance, so it must stay.
        let angle = 3e-15;
        let cylinder = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1e9,
        )
        .unwrap();
        let frame = (cylinder.x_axis(), cylinder.y_axis());
        let v_scale = 1.0 + 4.0 * f64::EPSILON;
        let map = UvAffine::about_axis(&Mat4::rotation_z(angle), frame, frame, v_scale);
        assert!((map.b.x() - angle).abs() < 1e-20, "phase {}", map.b.x());
        assert_eq!(map.a[0][0].to_bits(), 1.0_f64.to_bits());
        assert_eq!(map.a[1][1].to_bits(), v_scale.to_bits());
        assert!(!map.is_identity());
    }

    #[test]
    fn line_pcurves_survive_maps_below_the_line_direction_floor() {
        // A valid similarity that normalizes a model of size ~1e16 to unit
        // size: the direction's image is shorter than `Line2D`'s 1e-15 floor.
        let scale = 1e-16;
        let (sin, cos) = 0.3_f64.sin_cos();
        let map = UvAffine {
            a: [[scale * cos, -scale * sin], [scale * sin, scale * cos]],
            b: Vec2::new(0.0, 0.0),
        };
        let line = PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(2.0e16, -1.0e16), Vec2::new(0.6, 0.8)).unwrap()),
            -3.0e16,
            5.0e16,
        );
        let image = map.map_pcurve(&line, None).expect("tiny-scale line image");
        for k in 0..=8 {
            let f = f64::from(k) / 8.0;
            let expected =
                map.point(line.evaluate(line.t_start() + f * (line.t_end() - line.t_start())));
            let actual = image.evaluate(image.t_start() + f * (image.t_end() - image.t_start()));
            assert!(
                (actual - expected).length() < 1e-12,
                "sample {k}: {actual:?} != {expected:?}"
            );
        }
    }

    #[test]
    fn plane_map_follows_points_through_the_canonical_charts() {
        let matrix = Mat4::translation(4.0, -1.0, 2.0)
            * Mat4::rotation_y(0.8)
            * Mat4::rotation_z(-0.4)
            * Mat4::scale(-1.5, 1.5, 1.5);
        let normal = Vec3::new(0.0, 0.6, 0.8);
        let d = 2.5;
        let image_normal = linear_image(&matrix.inverse().unwrap().transpose(), normal)
            .normalize()
            .unwrap();
        let source = plane_chart(normal, d).unwrap();
        let on_plane = |u: f64, v: f64| source.origin + source.x * u + source.y * v;
        let image_d =
            image_normal.dot(matrix.mul_point(on_plane(0.0, 0.0)) - Point3::new(0.0, 0.0, 0.0));
        let target = plane_chart(image_normal, image_d).unwrap();
        let map = UvAffine::plane(&matrix, (normal, d), (image_normal, image_d)).unwrap();
        for (u, v) in [(0.0, 0.0), (1.0, -2.0), (-3.5, 0.25)] {
            let uv = map.point(Point2::new(u, v));
            let lifted = target.origin + target.x * uv.x() + target.y * uv.y();
            assert!((lifted - matrix.mul_point(on_plane(u, v))).length() < 1e-12);
        }
        assert!(map.determinant() < 0.0, "a reflection reverses the chart");
        assert!(map.conformal_scale().is_some());
    }
}
