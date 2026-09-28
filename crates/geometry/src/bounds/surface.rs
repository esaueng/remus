//! Conservative bounds over finite surface patches.
//!
//! Whole-carrier finite boxes (sphere, torus) and full-turn slabs over a
//! finite axial range (cylinder, cone) are exact supersets of any trim they
//! clip, and the NURBS patch box is the control hull of the supporting net.
//! The critical-point helpers expose the carrier points where a world
//! coordinate can attain an interior extremum, so the check layer can
//! condition their inclusion on the face trim.

use remus_math::aabb::Aabb3;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

use super::{FiniteBound, box_of_points};

/// Conservative box of a whole sphere: `center ± radius` on every axis.
///
/// Orientation-independent and a sound superset of any spherical patch.
#[must_use]
pub fn sphere_bounds(sphere: &SphericalSurface) -> FiniteBound {
    if !anchor_is_finite(sphere.center()) || !sphere.radius().is_finite() || sphere.radius() <= 0.0
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    FiniteBound::conservative(sphere.aabb())
}

/// Conservative box of a whole torus.
///
/// Uses the carrier's exact whole-torus box: per world axis, `center ±
/// ((R + r)·hypot(X_e, Y_e) + r·|Z_e|)`.
#[must_use]
pub fn torus_bounds(torus: &ToroidalSurface) -> FiniteBound {
    if !anchor_is_finite(torus.center())
        || !torus.major_radius().is_finite()
        || !torus.minor_radius().is_finite()
        || torus.major_radius() <= 0.0
        || torus.minor_radius() <= 0.0
        || !axis_is_finite(torus.x_axis())
        || !axis_is_finite(torus.y_axis())
        || !axis_is_finite(torus.z_axis())
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    FiniteBound::conservative(torus.aabb())
}

/// Conservative box of a full-turn cylinder slab over the finite axial range
/// `[v0, v1]`.
///
/// Per world axis the radial extent `r·hypot(X_e, Y_e)` is attained
/// somewhere on every full turn, and the axial term is linear in `v`, so
/// boxing the radial hull at both axial ends contains the slab.
#[must_use]
pub fn cylinder_slab_bounds(cyl: &CylindricalSurface, v0: f64, v1: f64) -> FiniteBound {
    if !anchor_is_finite(cyl.origin())
        || !cyl.radius().is_finite()
        || cyl.radius() <= 0.0
        || !axis_is_finite(cyl.axis())
        || !axis_is_finite(cyl.x_axis())
        || !axis_is_finite(cyl.y_axis())
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    if !v0.is_finite() || !v1.is_finite() {
        return FiniteBound::infinite_unknown("non_finite_span");
    }
    let (lo, hi) = (v0.min(v1), v0.max(v1));
    let o = cyl.origin();
    let a = cyl.axis();
    let r = cyl.radius();
    let radial = [
        r * cyl.x_axis().x().hypot(cyl.y_axis().x()),
        r * cyl.x_axis().y().hypot(cyl.y_axis().y()),
        r * cyl.x_axis().z().hypot(cyl.y_axis().z()),
    ];
    let ends = [cyl.evaluate(0.0, lo), cyl.evaluate(0.0, hi)];
    if !super::all_points_finite(&ends) {
        return FiniteBound::infinite_unknown("non_finite_evaluation");
    }
    let origin = [o.x(), o.y(), o.z()];
    let axis = [a.x(), a.y(), a.z()];
    let mut min = [0.0; 3];
    let mut max = [0.0; 3];
    for e in 0..3 {
        let axial_lo = axis[e] * lo;
        let axial_hi = axis[e] * hi;
        min[e] = origin[e] - radial[e] + axial_lo.min(axial_hi);
        max[e] = origin[e] + radial[e] + axial_lo.max(axial_hi);
    }
    FiniteBound::conservative(Aabb3 {
        min: Point3::new(min[0], min[1], min[2]),
        max: Point3::new(max[0], max[1], max[2]),
    })
}

/// Conservative box of a full-turn cone slab over the finite generator range
/// `[v0, v1]` (`v` measured along the generator from the apex).
///
/// `P(u, v) = apex + v·(cos(a)·radial(u) + sin(a)·axis)`: for fixed `v` the
/// radial hull is exact as in the cylinder case, and the per-axis envelope
/// over `v` is piecewise linear with a kink only at `v = 0`, so boxing the
/// envelope at `lo`, `hi` (and at the apex when `0` lies in range) contains
/// the slab.
#[must_use]
pub fn cone_slab_bounds(cone: &ConicalSurface, v0: f64, v1: f64) -> FiniteBound {
    if !anchor_is_finite(cone.apex())
        || !cone.half_angle().is_finite()
        || cone.half_angle() <= 0.0
        || cone.half_angle() >= std::f64::consts::FRAC_PI_2
        || !axis_is_finite(cone.axis())
        || !axis_is_finite(cone.x_axis())
        || !axis_is_finite(cone.y_axis())
    {
        return FiniteBound::infinite_unknown("non_finite_carrier");
    }
    if !v0.is_finite() || !v1.is_finite() {
        return FiniteBound::infinite_unknown("non_finite_span");
    }
    let (lo, hi) = (v0.min(v1), v0.max(v1));
    let (sin_a, cos_a) = cone.half_angle().sin_cos();
    let apex = [cone.apex().x(), cone.apex().y(), cone.apex().z()];
    let axis = [cone.axis().x(), cone.axis().y(), cone.axis().z()];
    let frame = [
        (cone.x_axis().x(), cone.y_axis().x()),
        (cone.x_axis().y(), cone.y_axis().y()),
        (cone.x_axis().z(), cone.y_axis().z()),
    ];
    // Envelope samples: both axial ends, plus the apex when the range
    // crosses the kink at v = 0.
    let mut vs = vec![lo, hi];
    if lo < 0.0 && hi > 0.0 {
        vs.push(0.0);
    }
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for v in vs {
        let radial = v.abs() * cos_a;
        let axial = v * sin_a;
        for e in 0..3 {
            let hull = radial * frame[e].0.hypot(frame[e].1);
            let center = apex[e] + axial * axis[e];
            min[e] = min[e].min(center - hull);
            max[e] = max[e].max(center + hull);
        }
    }
    if min.iter().any(|v| !v.is_finite()) || max.iter().any(|v| !v.is_finite()) {
        return FiniteBound::infinite_unknown("non_finite_evaluation");
    }
    FiniteBound::conservative(Aabb3 {
        min: Point3::new(min[0], min[1], min[2]),
        max: Point3::new(max[0], max[1], max[2]),
    })
}

/// Conservative box of a NURBS surface patch.
///
/// `u_span`/`v_span` select sub-rectangles (`None` means the full domain);
/// spans are clipped to the domain and a span clipping to empty refuses with
/// `Unknown`. The box is the control hull of the net rows/columns whose
/// tensor-product basis support overlaps the clipped rectangle: with
/// positive finite weights the patch is a convex combination of those
/// points, so the hull contains it. Whole-domain requests degenerate to the
/// full control hull.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn nurbs_surface_bounds(
    surface: &NurbsSurface,
    u_span: Option<(f64, f64)>,
    v_span: Option<(f64, f64)>,
) -> FiniteBound {
    if surface.validate().is_err() {
        return FiniteBound::infinite_unknown("invalid_surface");
    }
    let cps = surface.control_points();
    let flat_valid = cps
        .iter()
        .flatten()
        .all(|p| p.x().is_finite() && p.y().is_finite() && p.z().is_finite());
    if !flat_valid {
        return FiniteBound::infinite_unknown("non_finite_control_net");
    }
    let whole_hull = match box_of_net(cps) {
        Some(hull) => hull,
        None => return FiniteBound::infinite_unknown("non_finite_control_net"),
    };
    let (du0, du1) = surface.domain_u();
    let (dv0, dv1) = surface.domain_v();
    let clip = |span: Option<(f64, f64)>, d0: f64, d1: f64| -> Option<(f64, f64)> {
        let (a, b) = span.unwrap_or((d0, d1));
        if !a.is_finite() || !b.is_finite() {
            return None;
        }
        // No overlap at all (before clamping hides it): empty span.
        if a.max(b) < d0 || a.min(b) > d1 {
            return None;
        }
        let (lo, hi) = (a.min(b).clamp(d0, d1), a.max(b).clamp(d0, d1));
        if lo > hi { None } else { Some((lo, hi)) }
    };
    let (Some((ulo, uhi)), Some((vlo, vhi))) = (clip(u_span, du0, du1), clip(v_span, dv0, dv1))
    else {
        return FiniteBound::unknown(whole_hull, "empty_span");
    };
    let rows = cps.len();
    let cols = cps.first().map_or(0, Vec::len);
    let span_lo_u = super::curve::span_index(surface.knots_u(), surface.degree_u(), rows, ulo);
    let span_hi_u = super::curve::span_index(surface.knots_u(), surface.degree_u(), rows, uhi);
    let span_lo_v = super::curve::span_index(surface.knots_v(), surface.degree_v(), cols, vlo);
    let span_hi_v = super::curve::span_index(surface.knots_v(), surface.degree_v(), cols, vhi);
    let row_first = span_lo_u.saturating_sub(surface.degree_u());
    let row_last = span_hi_u.min(rows.saturating_sub(1));
    let col_first = span_lo_v.saturating_sub(surface.degree_v());
    let col_last = span_hi_v.min(cols.saturating_sub(1));
    let mut points = Vec::new();
    for row in &cps[row_first..=row_last] {
        points.extend_from_slice(&row[col_first..=col_last]);
    }
    match box_of_points(&points) {
        Some(aabb) => FiniteBound::conservative(aabb),
        None => FiniteBound::unknown(whole_hull, "empty_span"),
    }
}

/// The six sphere poles `center ± radius·ê` for the world axes.
///
/// A world coordinate restricted to a sphere has no interior critical point
/// other than its two poles, so these are the only interior points a
/// spherical face bound can need beyond its boundary.
#[must_use]
pub fn sphere_axis_poles(sphere: &SphericalSurface) -> [Point3; 6] {
    let c = sphere.center();
    let r = sphere.radius();
    [
        Point3::new(c.x() + r, c.y(), c.z()),
        Point3::new(c.x() - r, c.y(), c.z()),
        Point3::new(c.x(), c.y() + r, c.z()),
        Point3::new(c.x(), c.y() - r, c.z()),
        Point3::new(c.x(), c.y(), c.z() + r),
        Point3::new(c.x(), c.y(), c.z() - r),
    ]
}

/// Isolated critical points of a torus coordinate along `dir`.
///
/// Solves `normal(u, v) = ±dir` restricted to `+dir`: with `s = dir·Z` and
/// `w = dir − s·Z`, non-degenerate (`w ≠ 0`) directions give exactly two
/// points `(u, v) = (atan2(w·Y, w·X) adjusted by cos-v sign, atan2(s, ±|w|))`.
/// Returns `None` when `dir` is parallel to the torus axis (the critical set
/// is a full ring, not isolated points) or degenerate, in which case the
/// caller must carry the whole axial span instead of isolated points.
#[must_use]
pub fn torus_direction_criticals(torus: &ToroidalSurface, dir: Vec3) -> Option<Vec<Point3>> {
    if !dir.x().is_finite() || !dir.y().is_finite() || !dir.z().is_finite() {
        return None;
    }
    if dir.length_squared() < 1e-30 {
        return Some(Vec::new());
    }
    let z = torus.z_axis();
    let s = dir.dot(z);
    let w = dir - z * s;
    let wlen = w.length();
    if wlen <= 1e-12 * dir.length() {
        return None;
    }
    let x = torus.x_axis();
    let y = torus.y_axis();
    let u_base = w.dot(y).atan2(w.dot(x));
    let mut points = Vec::with_capacity(2);
    for sign in [1.0, -1.0] {
        let v = s.atan2(sign * wlen);
        let u = if sign > 0.0 {
            u_base
        } else {
            u_base + std::f64::consts::PI
        };
        points.push(torus.evaluate(u, v));
    }
    Some(points)
}

/// The apex of a cone.
///
/// Generator lines all meet here, so it is the only singular point a cone
/// face bound can need beyond its boundary.
#[must_use]
pub fn cone_apex(cone: &ConicalSurface) -> Point3 {
    cone.apex()
}

/// Whether an anchor point is finite.
fn anchor_is_finite(p: Point3) -> bool {
    p.x().is_finite() && p.y().is_finite() && p.z().is_finite()
}

/// Whether a direction vector is finite.
fn axis_is_finite(v: Vec3) -> bool {
    v.x().is_finite() && v.y().is_finite() && v.z().is_finite()
}

/// Box every point of a control net, or `None` when empty.
fn box_of_net(net: &[Vec<Point3>]) -> Option<Aabb3> {
    let mut iter = net.iter().flatten();
    let first = iter.next()?;
    let mut min = *first;
    let mut max = *first;
    for &p in iter {
        min = Point3::new(min.x().min(p.x()), min.y().min(p.y()), min.z().min(p.z()));
        max = Point3::new(max.x().max(p.x()), max.y().max(p.y()), max.z().max(p.z()));
    }
    if !anchor_is_finite(min) || !anchor_is_finite(max) {
        return None;
    }
    Some(Aabb3 { min, max })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    #[test]
    fn sphere_patch_box_contains_samples() {
        let sphere = SphericalSurface::new(Point3::new(1.0, 2.0, 3.0), 2.0).unwrap();
        let bound = sphere_bounds(&sphere);
        assert!(bound.is_prunable());
        let aabb = bound.aabb();
        // Dense samples supplement the whole-carrier proof (they are not it).
        for i in 0..=20 {
            for j in 0..=10 {
                let u = f64::from(i) * std::f64::consts::TAU / 20.0;
                let v = -std::f64::consts::FRAC_PI_2 + f64::from(j) * std::f64::consts::PI / 10.0;
                let p = sphere.evaluate(u, v);
                assert!(
                    aabb.contains_point(p),
                    "whole-sphere box must contain {p:?}"
                );
            }
        }
    }

    #[test]
    fn cylinder_slab_is_tight_radially() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.5)
                .unwrap();
        let bound = cylinder_slab_bounds(&cyl, 2.0, 5.0);
        assert!(bound.is_prunable());
        let aabb = bound.aabb();
        assert!((aabb.min.x() + 1.5).abs() < 1e-12);
        assert!((aabb.max.x() - 1.5).abs() < 1e-12);
        assert!((aabb.min.z() - 2.0).abs() < 1e-12);
        assert!((aabb.max.z() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn non_finite_slab_is_unknown() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                .unwrap();
        let bound = cylinder_slab_bounds(&cyl, 0.0, f64::INFINITY);
        assert!(!bound.is_prunable());
    }

    /// Axis-aligned torus whole box is tight: ±(R + r) equatorially, ±r
    /// axially.
    #[test]
    fn torus_whole_box_is_tight() {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        let bound = torus_bounds(&torus);
        assert!(bound.is_prunable());
        let aabb = bound.aabb();
        assert!((aabb.min.x() + 4.0).abs() < 1e-12);
        assert!((aabb.max.x() - 4.0).abs() < 1e-12);
        assert!((aabb.min.y() + 4.0).abs() < 1e-12);
        assert!((aabb.max.y() - 4.0).abs() < 1e-12);
        assert!((aabb.min.z() + 1.0).abs() < 1e-12);
        assert!((aabb.max.z() - 1.0).abs() < 1e-12);
    }

    /// Tilted torus: the whole box still contains dense samples.
    #[test]
    fn tilted_torus_box_contains_samples() {
        let torus = ToroidalSurface::with_axis(
            Point3::new(5.0, -3.0, 2.0),
            4.0,
            1.0,
            Vec3::new(1.0, 1.0, 1.0),
        )
        .unwrap();
        let bound = torus_bounds(&torus);
        assert!(bound.is_prunable());
        for i in 0..=40 {
            for j in 0..=20 {
                let u = f64::from(i) * std::f64::consts::TAU / 40.0;
                let v = f64::from(j) * std::f64::consts::TAU / 20.0;
                assert!(bound.aabb().contains_point(torus.evaluate(u, v)));
            }
        }
    }

    /// Tilted cylinder slab: radial extents follow the frame, axial ends are
    /// exact.
    #[test]
    fn tilted_cylinder_slab() {
        let cyl =
            CylindricalSurface::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 1.0, 0.0), 2.0)
                .unwrap();
        let bound = cylinder_slab_bounds(&cyl, -1.0, 4.0);
        assert!(bound.is_prunable());
        for i in 0..=40 {
            let u = f64::from(i) * std::f64::consts::TAU / 40.0;
            for v in [-1.0, 1.5, 4.0] {
                assert!(bound.aabb().contains_point(cyl.evaluate(u, v)));
            }
        }
        // Axial ends are exact along the axis direction: the box corners at
        // the axis line must match origin + v·axis.
        let axis_pt = |v: f64| cyl.origin() + cyl.axis() * v;
        for v in [-1.0, 4.0] {
            let p = axis_pt(v);
            // The axis point itself need not be on the surface, but its axial
            // coordinate must lie within the box along the axis direction.
            let rel = (p - cyl.origin()).dot(cyl.axis());
            let corners = [
                bound.aabb().min,
                bound.aabb().max,
                Point3::new(
                    bound.aabb().min.x(),
                    bound.aabb().min.y(),
                    bound.aabb().max.z(),
                ),
                Point3::new(
                    bound.aabb().min.x(),
                    bound.aabb().max.y(),
                    bound.aabb().min.z(),
                ),
                Point3::new(
                    bound.aabb().max.x(),
                    bound.aabb().min.y(),
                    bound.aabb().min.z(),
                ),
            ];
            let projs: Vec<f64> = corners
                .iter()
                .map(|c| (*c - cyl.origin()).dot(cyl.axis()))
                .collect();
            let (lo, hi) = (
                projs.iter().copied().fold(f64::INFINITY, f64::min),
                projs.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            );
            assert!(rel >= lo - 1e-9 && rel <= hi + 1e-9, "v={v} rel={rel}");
        }
    }

    /// Cone slab crossing the apex: contains the apex and dense samples.
    #[test]
    fn cone_slab_across_apex() {
        let cone =
            ConicalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.3).unwrap();
        let bound = cone_slab_bounds(&cone, -1.0, 2.0);
        assert!(bound.is_prunable());
        assert!(bound.aabb().contains_point(cone.apex()));
        for i in 0..=24 {
            let u = f64::from(i) * std::f64::consts::TAU / 24.0;
            for v in [-1.0, 0.0, 2.0] {
                assert!(bound.aabb().contains_point(cone.evaluate(u, v)));
            }
        }
    }

    /// Sphere with holes-era patch relevance: poles are exactly covered.
    #[test]
    fn sphere_poles_are_exact() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0).unwrap();
        let poles = sphere_axis_poles(&sphere);
        assert!((poles[0].x() - 5.0).abs() < 1e-15);
        assert!((poles[1].x() + 5.0).abs() < 1e-15);
        assert!((poles[4].z() - 5.0).abs() < 1e-15);
        assert!((poles[5].z() + 5.0).abs() < 1e-15);
    }

    /// NURBS sub-patch uses the active control rectangle, strictly tighter
    /// than the whole hull when excluded rows/columns stick out.
    #[test]
    fn nurbs_subpatch_uses_active_rectangle() {
        // 4x4 net, degree 2 both directions, two spans per direction; the
        // last row and column stick far out.
        let mut cps = Vec::new();
        let mut ws = Vec::new();
        for i in 0..4 {
            let mut row = Vec::new();
            let mut wrow = Vec::new();
            for j in 0..4 {
                let out = if i == 3 || j == 3 { 100.0 } else { 0.0 };
                row.push(Point3::new(f64::from(i), f64::from(j), out));
                wrow.push(1.0);
            }
            cps.push(row);
            ws.push(wrow);
        }
        let knots = vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0];
        let surface = NurbsSurface::new(2, 2, knots.clone(), knots, cps, ws).unwrap();
        let (du0, du1) = surface.domain_u();
        assert!((du0 - 0.0).abs() < 1e-15 && (du1 - 1.0).abs() < 1e-15);
        // First-span sub-patch avoids row/column 3.
        let sub = nurbs_surface_bounds(&surface, Some((0.0, 0.4)), Some((0.0, 0.4)));
        assert!(sub.is_prunable());
        assert!(
            sub.aabb().max.z() < 100.0,
            "sub-patch must exclude the far net, got {}",
            sub.aabb().max.z()
        );
        for i in 0..=10 {
            for j in 0..=10 {
                let u = 0.4 * f64::from(i) / 10.0;
                let v = 0.4 * f64::from(j) / 10.0;
                assert!(sub.aabb().contains_point(surface.evaluate(u, v)));
            }
        }
    }

    /// Empty sub-rectangle (outside the domain): unknown, whole-hull box.
    #[test]
    fn nurbs_empty_subpatch_is_unknown() {
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .unwrap();
        let bound = nurbs_surface_bounds(&surface, Some((2.0, 3.0)), None);
        assert!(!bound.is_prunable());
        assert!(bound.aabb().contains_point(Point3::new(0.5, 0.5, 0.0)));
    }

    /// Torus equatorial criticals are the closed-form normal-alignment
    /// points: normal ±x at (u, v) ∈ {(0, 0)} and {(π, π)}.
    #[test]
    fn torus_direction_criticals_spot_check() {
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0, 1.0).unwrap();
        let pts = torus_direction_criticals(&torus, Vec3::new(1.0, 0.0, 0.0)).unwrap();
        assert_eq!(pts.len(), 2);
        // Outer equator point (4, 0, 0) and inner point (-2, 0, 0).
        let mut xs: Vec<f64> = pts.iter().map(|p| p.x()).collect();
        xs.sort_by(f64::total_cmp);
        assert!((xs[0] + 2.0).abs() < 1e-9, "{xs:?}");
        assert!((xs[1] - 4.0).abs() < 1e-9, "{xs:?}");
        // Axial direction rings: no isolated points.
        assert!(torus_direction_criticals(&torus, Vec3::new(0.0, 0.0, 1.0)).is_none());
        assert!(
            torus_direction_criticals(&torus, Vec3::new(0.0, 0.0, 0.0))
                .unwrap()
                .is_empty()
        );
    }
}
