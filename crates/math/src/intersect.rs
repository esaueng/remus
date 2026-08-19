//! Qualified intersection results (kernel program Milestone 3, Issue 10).
//!
//! The legacy intersection routines return bare curves and points; nothing
//! in their type says whether contact is transversal or tangential, whether
//! the answer is exact or marched, or whether part of the domain was left
//! unresolved. This module is the common result model those answers live
//! in, and the qualified entry point for surface–surface intersection.
//!
//! Every element of a result carries:
//!
//! - a **contact kind** — transversal crossing, tangential contact, whole-
//!   geometry coincidence, or (transitionally) unclassified;
//! - a **quality** — exact closed form, approximate with an error bound, or
//!   explicitly unresolved;
//! - a **source method** — which strategy produced it.
//!
//! Classification is certified only where the geometry is decided in closed
//! form. Results delegated to legacy sampling are wrapped honestly as
//! [`ContactKind::Unclassified`] + [`ResultQuality::Approximate`] — the
//! model never upgrades an uncertified answer. As pairs gain closed-form
//! or certified treatment they move out of the wrapped set; the capability
//! matrix tracks the support state per pair.
//!
//! Curve-curve and curve-surface qualification reuse this vocabulary and
//! integrate incrementally (same staging as every kernel-program slice).

use crate::MathError;
use crate::analytic_intersection::{
    AnalyticSurface, ExactIntersectionCurve, exact_plane_analytic, intersect_analytic_analytic,
};
use crate::context::OperationContext;
use crate::curves::{Circle3D, Ellipse3D};
use crate::surfaces::{CylindricalSurface, SphericalSurface};
use crate::vec::{Point3, Vec3};

/// How two geometric entities touch along one intersection element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactKind {
    /// The surfaces cross each other through the element.
    Transversal,
    /// The surfaces touch without crossing (double contact) at the element.
    Tangential,
    /// The geometries coincide over the element's whole extent.
    Coincident,
    /// Contact was not certified. Transitional: legacy-delegated results
    /// carry this until their pair gains certified classification.
    Unclassified,
}

/// How trustworthy an element's geometry is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResultQuality {
    /// Closed-form geometry; error at floating-point rounding scale.
    Exact,
    /// Sampled or fitted geometry with a reported error bound in model
    /// units. The bound is the method's declared budget, not a certificate.
    Approximate {
        /// Upper estimate of the geometric error, in model units.
        max_error: f64,
    },
    /// The method could not resolve this region within its budgets.
    Unresolved,
}

/// The strategy that produced an element.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceMethod {
    /// Direct closed-form solution for the pair.
    ClosedForm,
    /// Algebraic specialization or sampled marching from the legacy
    /// analytic-analytic path.
    LegacyAnalytic,
}

/// Geometry of one intersection curve element.
#[derive(Debug, Clone)]
pub enum CurveGeometry {
    /// An infinite straight line (origin + unit direction).
    Line {
        /// A point on the line.
        origin: Point3,
        /// Unit direction.
        direction: Vec3,
    },
    /// A full circle.
    Circle(Circle3D),
    /// A full ellipse.
    Ellipse(Ellipse3D),
    /// A sampled point chain (legacy fallback geometry).
    Sampled(Vec<Point3>),
}

/// One point element of an intersection.
#[derive(Debug, Clone)]
pub struct QualifiedPoint {
    /// The contact point.
    pub point: Point3,
    /// Contact kind at the point.
    pub kind: ContactKind,
    /// Geometry quality.
    pub quality: ResultQuality,
    /// Producing strategy.
    pub method: SourceMethod,
}

/// One curve element of an intersection.
#[derive(Debug, Clone)]
pub struct QualifiedCurve {
    /// The curve's geometry.
    pub geometry: CurveGeometry,
    /// Contact kind along the curve.
    pub kind: ContactKind,
    /// Geometry quality.
    pub quality: ResultQuality,
    /// Producing strategy.
    pub method: SourceMethod,
}

/// One element of a surface–surface intersection.
#[derive(Debug, Clone)]
pub enum IntersectionElement {
    /// An isolated contact point.
    Point(QualifiedPoint),
    /// An intersection curve.
    Curve(QualifiedCurve),
    /// The surfaces coincide as sets (same surface, possibly different
    /// parameterization). Region bounds arrive with trimmed-face
    /// qualification.
    CoincidentSurfaces,
}

/// A qualified surface–surface intersection result.
#[derive(Debug, Clone)]
pub struct SurfaceIntersection {
    /// The intersection's elements. Empty means certified-disjoint when
    /// `complete`, and "nothing found" otherwise.
    pub elements: Vec<IntersectionElement>,
    /// `true` when the whole configuration was resolved: no region was
    /// abandoned on a budget and no element is `Unresolved`. Legacy-
    /// delegated results are never marked complete.
    pub complete: bool,
}

impl SurfaceIntersection {
    fn certified(elements: Vec<IntersectionElement>) -> Self {
        Self {
            elements,
            complete: true,
        }
    }
}

/// An infinite plane `dot(normal, p) = d` as a first-class operand.
///
/// The analytic surface set ([`AnalyticSurface`]) has no plane variant —
/// legacy plane intersections take the plane as bare parameters — so the
/// qualified API models operands as plane-or-surface explicitly.
#[derive(Debug, Clone, Copy)]
pub struct PlaneOperand {
    /// Unit plane normal.
    pub normal: Vec3,
    /// Signed offset: the plane satisfies `dot(normal, p) = d`.
    pub d: f64,
}

/// One operand of a qualified surface–surface intersection.
#[derive(Clone, Copy)]
pub enum SurfaceOperand<'a> {
    /// An infinite plane.
    Plane(PlaneOperand),
    /// An analytic surface.
    Analytic(AnalyticSurface<'a>),
}

/// Intersect two surface operands, classifying contact where the pair is
/// decided in closed form.
///
/// Certified pairs (classification and geometry exact): plane–plane,
/// plane–sphere, plane–cylinder, sphere–sphere, coaxial sphere–cylinder,
/// and parallel-axis cylinder–cylinder — including their tangential and
/// coincident configurations. Every other pair (and every non-certified
/// sub-configuration) delegates to the legacy analytic path and is wrapped
/// as [`ContactKind::Unclassified`] with `complete = false`.
///
/// Tolerances come from `context.tolerance` and are scale-aware through
/// its `approx_eq`; classification of near-tangency follows the declared
/// tolerance, so the same configuration uniformly scaled with a matching
/// tolerance classifies identically.
///
/// # Errors
///
/// Returns [`MathError`] when a delegated legacy computation fails.
pub fn intersect_surfaces(
    a: SurfaceOperand<'_>,
    b: SurfaceOperand<'_>,
    context: &OperationContext,
) -> Result<SurfaceIntersection, MathError> {
    let tol = &context.tolerance;
    match (a, b) {
        (SurfaceOperand::Plane(p), SurfaceOperand::Plane(q)) => Ok(plane_plane(p, q, tol)),
        (SurfaceOperand::Plane(p), SurfaceOperand::Analytic(s))
        | (SurfaceOperand::Analytic(s), SurfaceOperand::Plane(p)) => match s {
            AnalyticSurface::Sphere(sphere) => Ok(plane_sphere(p, sphere, tol)),
            AnalyticSurface::Cylinder(cyl) => plane_cylinder(p, cyl, tol),
            AnalyticSurface::Cone(_) | AnalyticSurface::Torus(_) => legacy_plane_analytic(p, s),
        },
        (SurfaceOperand::Analytic(sa), SurfaceOperand::Analytic(sb)) => match (sa, sb) {
            (AnalyticSurface::Sphere(s1), AnalyticSurface::Sphere(s2)) => {
                Ok(sphere_sphere(s1, s2, tol))
            }
            (AnalyticSurface::Cylinder(c1), AnalyticSurface::Cylinder(c2)) => {
                cylinder_cylinder(c1, c2, sa, sb, tol)
            }
            (AnalyticSurface::Sphere(sp), AnalyticSurface::Cylinder(cy))
            | (AnalyticSurface::Cylinder(cy), AnalyticSurface::Sphere(sp)) => {
                sphere_cylinder(sp, cy, sa, sb, tol)
            }
            _ => legacy_analytic_analytic(sa, sb),
        },
    }
}

fn scale_eq(tol: &crate::tolerance::Tolerance, a: f64, b: f64) -> bool {
    tol.approx_eq(a, b)
}

fn plane_plane(
    p: PlaneOperand,
    q: PlaneOperand,
    tol: &crate::tolerance::Tolerance,
) -> SurfaceIntersection {
    let cross = p.normal.cross(q.normal);
    if cross.length() <= tol.angular.max(1e-12) {
        // Parallel: coincident when the (sign-normalized) offsets agree.
        let same_side = p.normal.dot(q.normal) > 0.0;
        let q_d = if same_side { q.d } else { -q.d };
        if scale_eq(tol, p.d, q_d) {
            return SurfaceIntersection::certified(vec![IntersectionElement::CoincidentSurfaces]);
        }
        return SurfaceIntersection::certified(vec![]);
    }
    // Transversal line: solve for a point on both planes.
    let n1 = p.normal;
    let n2 = q.normal;
    let direction = match cross.normalize() {
        Ok(d) => d,
        Err(_) => return SurfaceIntersection::certified(vec![]),
    };
    // Point on both planes: solve in the {n1, n2} span.
    let n1n2 = n1.dot(n2);
    let det = 1.0 - n1n2 * n1n2;
    let c1 = (p.d - q.d * n1n2) / det;
    let c2 = (q.d - p.d * n1n2) / det;
    let origin = Point3::new(
        c1 * n1.x() + c2 * n2.x(),
        c1 * n1.y() + c2 * n2.y(),
        c1 * n1.z() + c2 * n2.z(),
    );
    SurfaceIntersection::certified(vec![IntersectionElement::Curve(QualifiedCurve {
        geometry: CurveGeometry::Line { origin, direction },
        kind: ContactKind::Transversal,
        quality: ResultQuality::Exact,
        method: SourceMethod::ClosedForm,
    })])
}

fn plane_sphere(
    p: PlaneOperand,
    sphere: &SphericalSurface,
    tol: &crate::tolerance::Tolerance,
) -> SurfaceIntersection {
    let center = sphere.center();
    let signed = p.normal.dot(Vec3::new(center.x(), center.y(), center.z())) - p.d;
    let dist = signed.abs();
    let r = sphere.radius();
    if scale_eq(tol, dist, r) {
        // Tangent point: the foot of the center on the plane.
        let foot = center - p.normal * signed;
        return SurfaceIntersection::certified(vec![IntersectionElement::Point(QualifiedPoint {
            point: foot,
            kind: ContactKind::Tangential,
            quality: ResultQuality::Exact,
            method: SourceMethod::ClosedForm,
        })]);
    }
    if dist > r {
        return SurfaceIntersection::certified(vec![]);
    }
    let circle_r = (r * r - dist * dist).sqrt();
    let circle_center = center - p.normal * signed;
    match Circle3D::new(circle_center, p.normal, circle_r) {
        Ok(circle) => {
            SurfaceIntersection::certified(vec![IntersectionElement::Curve(QualifiedCurve {
                geometry: CurveGeometry::Circle(circle),
                kind: ContactKind::Transversal,
                quality: ResultQuality::Exact,
                method: SourceMethod::ClosedForm,
            })])
        }
        Err(_) => SurfaceIntersection::certified(vec![]),
    }
}

#[allow(clippy::many_single_char_names)]
fn plane_cylinder(
    p: PlaneOperand,
    cyl: &CylindricalSurface,
    tol: &crate::tolerance::Tolerance,
) -> Result<SurfaceIntersection, MathError> {
    let axis = cyl.axis();
    let origin = cyl.origin();
    let r = cyl.radius();
    let align = p.normal.dot(axis).abs();

    if align <= tol.angular.max(1e-12) {
        // Plane parallel to the axis: 0, 1 (tangent), or 2 lines.
        let signed = p.normal.dot(Vec3::new(origin.x(), origin.y(), origin.z())) - p.d;
        let dist = signed.abs();
        if scale_eq(tol, dist, r) {
            let touch = origin - p.normal * signed;
            return Ok(SurfaceIntersection::certified(vec![
                IntersectionElement::Curve(QualifiedCurve {
                    geometry: CurveGeometry::Line {
                        origin: touch,
                        direction: axis,
                    },
                    kind: ContactKind::Tangential,
                    quality: ResultQuality::Exact,
                    method: SourceMethod::ClosedForm,
                }),
            ]));
        }
        if dist > r {
            return Ok(SurfaceIntersection::certified(vec![]));
        }
        let half_chord = (r * r - dist * dist).sqrt();
        let foot = origin - p.normal * signed;
        let in_plane = p.normal.cross(axis).normalize()?;
        let mut elements = Vec::with_capacity(2);
        for sign in [-1.0, 1.0] {
            elements.push(IntersectionElement::Curve(QualifiedCurve {
                geometry: CurveGeometry::Line {
                    origin: foot + in_plane * (sign * half_chord),
                    direction: axis,
                },
                kind: ContactKind::Transversal,
                quality: ResultQuality::Exact,
                method: SourceMethod::ClosedForm,
            }));
        }
        return Ok(SurfaceIntersection::certified(elements));
    }

    // Oblique or perpendicular: the legacy closed form already produces the
    // exact circle/ellipse; contact is transversal in every such case.
    let curves = exact_plane_analytic(AnalyticSurface::Cylinder(cyl), p.normal, p.d)?;
    let elements: Vec<IntersectionElement> = curves
        .into_iter()
        .map(|curve| {
            IntersectionElement::Curve(match curve {
                ExactIntersectionCurve::Circle(c) => QualifiedCurve {
                    geometry: CurveGeometry::Circle(c),
                    kind: ContactKind::Transversal,
                    quality: ResultQuality::Exact,
                    method: SourceMethod::ClosedForm,
                },
                ExactIntersectionCurve::Ellipse(e) => QualifiedCurve {
                    geometry: CurveGeometry::Ellipse(e),
                    kind: ContactKind::Transversal,
                    quality: ResultQuality::Exact,
                    method: SourceMethod::ClosedForm,
                },
                ExactIntersectionCurve::Points(pts) => QualifiedCurve {
                    geometry: CurveGeometry::Sampled(pts),
                    kind: ContactKind::Unclassified,
                    quality: ResultQuality::Unresolved,
                    method: SourceMethod::LegacyAnalytic,
                },
            })
        })
        .collect();
    // The legacy closed form applies its own (coarser, 1e-10) parallel test
    // and can hand back sampled Points where the branch above judged the
    // plane oblique; an Unresolved element must never ride in a `complete`
    // result.
    let complete = elements.iter().all(|element| match element {
        IntersectionElement::Curve(c) => c.quality != ResultQuality::Unresolved,
        _ => true,
    });
    Ok(SurfaceIntersection { elements, complete })
}

fn sphere_sphere(
    s1: &SphericalSurface,
    s2: &SphericalSurface,
    tol: &crate::tolerance::Tolerance,
) -> SurfaceIntersection {
    let (c1, c2) = (s1.center(), s2.center());
    let (r1, r2) = (s1.radius(), s2.radius());
    let d = (c2 - c1).length();

    if scale_eq(tol, d, 0.0) && scale_eq(tol, r1, r2) {
        return SurfaceIntersection::certified(vec![IntersectionElement::CoincidentSurfaces]);
    }
    let tangent_external = scale_eq(tol, d, r1 + r2);
    let tangent_internal = d > tol.linear && scale_eq(tol, d, (r1 - r2).abs());
    if tangent_external || tangent_internal {
        let dir = match (c2 - c1).normalize() {
            Ok(v) => v,
            Err(_) => return SurfaceIntersection::certified(vec![]),
        };
        let sign = if tangent_external || r1 > r2 {
            1.0
        } else {
            -1.0
        };
        return SurfaceIntersection::certified(vec![IntersectionElement::Point(QualifiedPoint {
            point: c1 + dir * (sign * r1),
            kind: ContactKind::Tangential,
            quality: ResultQuality::Exact,
            method: SourceMethod::ClosedForm,
        })]);
    }
    if d > r1 + r2 || d < (r1 - r2).abs() {
        return SurfaceIntersection::certified(vec![]);
    }
    // Transversal circle on the radical plane.
    let a = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    let circle_r = (r1 * r1 - a * a).max(0.0).sqrt();
    let dir = match (c2 - c1).normalize() {
        Ok(v) => v,
        Err(_) => return SurfaceIntersection::certified(vec![]),
    };
    let center = c1 + dir * a;
    match Circle3D::new(center, dir, circle_r) {
        Ok(circle) => {
            SurfaceIntersection::certified(vec![IntersectionElement::Curve(QualifiedCurve {
                geometry: CurveGeometry::Circle(circle),
                kind: ContactKind::Transversal,
                quality: ResultQuality::Exact,
                method: SourceMethod::ClosedForm,
            })])
        }
        Err(_) => SurfaceIntersection::certified(vec![]),
    }
}

fn cylinder_cylinder(
    c1: &CylindricalSurface,
    c2: &CylindricalSurface,
    sa: AnalyticSurface<'_>,
    sb: AnalyticSurface<'_>,
    tol: &crate::tolerance::Tolerance,
) -> Result<SurfaceIntersection, MathError> {
    let parallel = c1.axis().cross(c2.axis()).length() <= tol.angular.max(1e-12);
    if !parallel {
        return legacy_analytic_analytic(sa, sb);
    }
    let (r1, r2) = (c1.radius(), c2.radius());
    // Distance between the parallel axes.
    let offset = c2.origin() - c1.origin();
    let along = c1.axis() * offset.dot(c1.axis());
    let radial = offset - along;
    let d = radial.length();

    if scale_eq(tol, d, 0.0) && scale_eq(tol, r1, r2) {
        return Ok(SurfaceIntersection::certified(vec![
            IntersectionElement::CoincidentSurfaces,
        ]));
    }
    let tangent_external = scale_eq(tol, d, r1 + r2);
    let tangent_internal = d > tol.linear && scale_eq(tol, d, (r1 - r2).abs());
    if tangent_external || tangent_internal {
        let dir = radial.normalize()?;
        let sign = if tangent_external || r1 > r2 {
            1.0
        } else {
            -1.0
        };
        return Ok(SurfaceIntersection::certified(vec![
            IntersectionElement::Curve(QualifiedCurve {
                geometry: CurveGeometry::Line {
                    origin: c1.origin() + dir * (sign * r1),
                    direction: c1.axis(),
                },
                kind: ContactKind::Tangential,
                quality: ResultQuality::Exact,
                method: SourceMethod::ClosedForm,
            }),
        ]));
    }
    if d > r1 + r2 || d < (r1 - r2).abs() {
        return Ok(SurfaceIntersection::certified(vec![]));
    }
    // Two parallel transversal lines through the circle-circle crossing in
    // the radial cross-section plane.
    let x_dir = radial.normalize()?;
    let y_dir = c1.axis().cross(x_dir).normalize()?;
    let a = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    let h = (r1 * r1 - a * a).max(0.0).sqrt();
    let mut elements = Vec::with_capacity(2);
    for sign in [-1.0, 1.0] {
        elements.push(IntersectionElement::Curve(QualifiedCurve {
            geometry: CurveGeometry::Line {
                origin: c1.origin() + x_dir * a + y_dir * (sign * h),
                direction: c1.axis(),
            },
            kind: ContactKind::Transversal,
            quality: ResultQuality::Exact,
            method: SourceMethod::ClosedForm,
        }));
    }
    Ok(SurfaceIntersection::certified(elements))
}

fn sphere_cylinder(
    sphere: &SphericalSurface,
    cyl: &CylindricalSurface,
    sa: AnalyticSurface<'_>,
    sb: AnalyticSurface<'_>,
    tol: &crate::tolerance::Tolerance,
) -> Result<SurfaceIntersection, MathError> {
    // Certified only for the coaxial case (sphere center on the cylinder
    // axis); the offset case delegates.
    let offset = sphere.center() - cyl.origin();
    let radial = offset - cyl.axis() * offset.dot(cyl.axis());
    if radial.length() > tol.linear {
        return legacy_analytic_analytic(sa, sb);
    }
    let (rs, rc) = (sphere.radius(), cyl.radius());
    if scale_eq(tol, rs, rc) {
        // The equator touches the wall tangentially all the way round.
        let center = sphere.center();
        return Ok(SurfaceIntersection::certified(vec![
            IntersectionElement::Curve(QualifiedCurve {
                geometry: CurveGeometry::Circle(Circle3D::new(center, cyl.axis(), rc)?),
                kind: ContactKind::Tangential,
                quality: ResultQuality::Exact,
                method: SourceMethod::ClosedForm,
            }),
        ]));
    }
    if rs < rc {
        return Ok(SurfaceIntersection::certified(vec![]));
    }
    let h = (rs * rs - rc * rc).sqrt();
    let mut elements = Vec::with_capacity(2);
    for sign in [-1.0, 1.0] {
        elements.push(IntersectionElement::Curve(QualifiedCurve {
            geometry: CurveGeometry::Circle(Circle3D::new(
                sphere.center() + cyl.axis() * (sign * h),
                cyl.axis(),
                rc,
            )?),
            kind: ContactKind::Transversal,
            quality: ResultQuality::Exact,
            method: SourceMethod::ClosedForm,
        }));
    }
    Ok(SurfaceIntersection::certified(elements))
}

fn legacy_plane_analytic(
    p: PlaneOperand,
    s: AnalyticSurface<'_>,
) -> Result<SurfaceIntersection, MathError> {
    let curves = exact_plane_analytic(s, p.normal, p.d)?;
    Ok(wrap_legacy_curves(
        curves
            .into_iter()
            .map(|c| match c {
                ExactIntersectionCurve::Circle(c) => CurveGeometry::Circle(c),
                ExactIntersectionCurve::Ellipse(e) => CurveGeometry::Ellipse(e),
                ExactIntersectionCurve::Points(pts) => CurveGeometry::Sampled(pts),
            })
            .collect(),
    ))
}

fn legacy_analytic_analytic(
    a: AnalyticSurface<'_>,
    b: AnalyticSurface<'_>,
) -> Result<SurfaceIntersection, MathError> {
    let curves = intersect_analytic_analytic(a, b, 24)?;
    Ok(wrap_legacy_curves(
        curves
            .into_iter()
            .map(|ic| CurveGeometry::Sampled(ic.points.into_iter().map(|p| p.point).collect()))
            .collect(),
    ))
}

/// Wraps legacy geometry without certifying it: unclassified contact, and
/// `complete = false` because the legacy paths cannot rule out missed
/// regions or report exhausted budgets.
fn wrap_legacy_curves(geometries: Vec<CurveGeometry>) -> SurfaceIntersection {
    SurfaceIntersection {
        elements: geometries
            .into_iter()
            .map(|geometry| {
                let quality = match &geometry {
                    CurveGeometry::Circle(_) | CurveGeometry::Ellipse(_) => ResultQuality::Exact,
                    _ => ResultQuality::Unresolved,
                };
                IntersectionElement::Curve(QualifiedCurve {
                    geometry,
                    kind: ContactKind::Unclassified,
                    quality,
                    method: SourceMethod::LegacyAnalytic,
                })
            })
            .collect(),
        complete: false,
    }
}
