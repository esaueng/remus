//! Matrix-generated qualification tests for the intersection result model
//! (kernel program Milestone 3, Issue 10).
//!
//! Every configuration is generated at three model scales (1e-3, 1, 1e3)
//! and must classify identically at each — the uniform-scale metamorphic
//! invariant from the kernel testing strategy. Every element the model
//! certifies as `Exact` must actually lie on both surfaces at every
//! sampled point (checked against implicit distance functions, not against
//! the routine that produced it).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use brepkit_math::analytic_intersection::AnalyticSurface;
use brepkit_math::context::OperationContext;
use brepkit_math::intersect::{
    ContactKind, CurveGeometry, IntersectionElement, PlaneOperand, ResultQuality,
    SurfaceIntersection, SurfaceOperand, intersect_surfaces,
};
use brepkit_math::surfaces::{CylindricalSurface, SphericalSurface};
use brepkit_math::traits::ParametricCurve;
use brepkit_math::vec::{Point3, Vec3};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

/// Implicit unsigned distance to an operand, independent of the
/// intersection code under test.
#[derive(Clone, Copy)]
enum Implicit {
    Plane { normal: Vec3, d: f64 },
    Sphere { center: Point3, r: f64 },
    Cylinder { origin: Point3, axis: Vec3, r: f64 },
}

impl Implicit {
    fn distance(self, p: Point3) -> f64 {
        match self {
            Self::Plane { normal, d } => (normal.dot(Vec3::new(p.x(), p.y(), p.z())) - d).abs(),
            Self::Sphere { center, r } => ((p - center).length() - r).abs(),
            Self::Cylinder { origin, axis, r } => {
                let v = p - origin;
                let radial = v - axis * v.dot(axis);
                (radial.length() - r).abs()
            }
        }
    }
}

/// One matrix cell: a builder producing the two operands and their
/// implicit forms at a given scale, plus the expected classification
/// signature.
struct Cell {
    name: &'static str,
    expected: &'static [&'static str],
    /// Whether the model must certify the whole configuration.
    complete: bool,
}

fn signature(result: &SurfaceIntersection) -> Vec<String> {
    let mut sig: Vec<String> = result
        .elements
        .iter()
        .map(|e| match e {
            IntersectionElement::Point(p) => format!("point:{:?}", p.kind),
            IntersectionElement::Curve(c) => {
                let geom = match &c.geometry {
                    CurveGeometry::Line { .. } => "line",
                    CurveGeometry::Circle(_) => "circle",
                    CurveGeometry::Ellipse(_) => "ellipse",
                    CurveGeometry::Sampled(_) => "sampled",
                };
                format!("{geom}:{:?}", c.kind)
            }
            IntersectionElement::CoincidentSurfaces => "coincident".to_string(),
        })
        .collect();
    sig.sort();
    sig
}

/// Samples an element's geometry and asserts every Exact sample lies on
/// both implicit surfaces within `tol`.
fn assert_on_both(result: &SurfaceIntersection, ia: Implicit, ib: Implicit, tol: f64, name: &str) {
    for element in &result.elements {
        let (samples, quality): (Vec<Point3>, _) = match element {
            IntersectionElement::Point(p) => (vec![p.point], p.quality),
            IntersectionElement::Curve(c) => {
                let pts = match &c.geometry {
                    CurveGeometry::Line { origin, direction } => (-4..=4)
                        .map(|k| *origin + *direction * (f64::from(k) * 0.25 * tol.max(1.0)))
                        .collect(),
                    CurveGeometry::Circle(circle) => (0..16)
                        .map(|k| circle.evaluate(f64::from(k) * std::f64::consts::TAU / 16.0))
                        .collect(),
                    CurveGeometry::Ellipse(ellipse) => (0..16)
                        .map(|k| {
                            ParametricCurve::evaluate(
                                ellipse,
                                f64::from(k) * std::f64::consts::TAU / 16.0,
                            )
                        })
                        .collect(),
                    CurveGeometry::Sampled(pts) => pts.clone(),
                };
                (pts, c.quality)
            }
            IntersectionElement::CoincidentSurfaces => continue,
        };
        if quality != ResultQuality::Exact {
            continue;
        }
        for p in samples {
            assert!(
                ia.distance(p) <= tol,
                "{name}: exact element leaves surface A by {}",
                ia.distance(p)
            );
            assert!(
                ib.distance(p) <= tol,
                "{name}: exact element leaves surface B by {}",
                ib.distance(p)
            );
        }
    }
}

/// Runs one cell at every scale and asserts an identical signature.
#[allow(clippy::too_many_lines)]
fn run_cell(cell: &Cell, build: impl Fn(f64) -> (OwnedOperand, OwnedOperand)) {
    let ctx = OperationContext::new();
    let mut signatures = Vec::new();
    for scale in SCALES {
        let (a, b) = build(scale);
        let result = intersect_surfaces(a.operand(), b.operand(), &ctx)
            .unwrap_or_else(|e| panic!("{} @ scale {scale}: {e}", cell.name));
        assert_eq!(
            result.complete, cell.complete,
            "{} @ scale {scale}: completeness",
            cell.name
        );
        // The on-surface tolerance scales with the model.
        assert_on_both(
            &result,
            a.implicit(),
            b.implicit(),
            1e-7 * scale.max(1.0),
            cell.name,
        );
        signatures.push(signature(&result));
    }
    for sig in &signatures {
        assert_eq!(
            sig, &signatures[0],
            "{}: classification must be scale-invariant",
            cell.name
        );
    }
    let expected: Vec<String> = cell.expected.iter().map(|s| (*s).to_string()).collect();
    assert_eq!(
        signatures[0], expected,
        "{}: classification signature",
        cell.name
    );
}

/// An operand owning its surface so the borrow in `SurfaceOperand` has a
/// stable home.
enum OwnedOperand {
    Plane(PlaneOperand),
    Sphere(SphericalSurface),
    Cylinder(CylindricalSurface),
}

impl OwnedOperand {
    fn operand(&self) -> SurfaceOperand<'_> {
        match self {
            Self::Plane(p) => SurfaceOperand::Plane(*p),
            Self::Sphere(s) => SurfaceOperand::Analytic(AnalyticSurface::Sphere(s)),
            Self::Cylinder(c) => SurfaceOperand::Analytic(AnalyticSurface::Cylinder(c)),
        }
    }

    fn implicit(&self) -> Implicit {
        match self {
            Self::Plane(p) => Implicit::Plane {
                normal: p.normal,
                d: p.d,
            },
            Self::Sphere(s) => Implicit::Sphere {
                center: s.center(),
                r: s.radius(),
            },
            Self::Cylinder(c) => Implicit::Cylinder {
                origin: c.origin(),
                axis: c.axis(),
                r: c.radius(),
            },
        }
    }
}

fn plane(normal: Vec3, d: f64) -> OwnedOperand {
    OwnedOperand::Plane(PlaneOperand { normal, d })
}

fn sphere(center: Point3, r: f64) -> OwnedOperand {
    OwnedOperand::Sphere(SphericalSurface::new(center, r).unwrap())
}

fn cylinder(origin: Point3, axis: Vec3, r: f64) -> OwnedOperand {
    OwnedOperand::Cylinder(CylindricalSurface::new(origin, axis, r).unwrap())
}

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);

#[test]
fn plane_plane_matrix() {
    run_cell(
        &Cell {
            name: "plane-plane crossing",
            expected: &["line:Transversal"],
            complete: true,
        },
        |s| (plane(Z, 0.0), plane(X, 0.5 * s)),
    );
    run_cell(
        &Cell {
            name: "plane-plane parallel distinct",
            expected: &[],
            complete: true,
        },
        |s| (plane(Z, 0.0), plane(Z, 2.0 * s)),
    );
    run_cell(
        &Cell {
            name: "plane-plane coincident",
            expected: &["coincident"],
            complete: true,
        },
        |s| (plane(Z, 3.0 * s), plane(Z, 3.0 * s)),
    );
    run_cell(
        &Cell {
            name: "plane-plane coincident flipped normal",
            expected: &["coincident"],
            complete: true,
        },
        |s| (plane(Z, 3.0 * s), plane(Z * -1.0, -3.0 * s)),
    );
}

#[test]
fn plane_sphere_matrix() {
    let c = |s: f64| Point3::new(0.0, 0.0, 2.0 * s);
    run_cell(
        &Cell {
            name: "plane-sphere crossing",
            expected: &["circle:Transversal"],
            complete: true,
        },
        |s| (plane(Z, 1.5 * s), sphere(c(s), s)),
    );
    run_cell(
        &Cell {
            name: "plane-sphere tangent",
            expected: &["point:Tangential"],
            complete: true,
        },
        |s| (plane(Z, s), sphere(c(s), s)),
    );
    run_cell(
        &Cell {
            name: "plane-sphere miss",
            expected: &[],
            complete: true,
        },
        |s| (plane(Z, -s), sphere(c(s), s)),
    );
}

#[test]
fn plane_cylinder_matrix() {
    let cyl = |s: f64| cylinder(Point3::new(0.0, 0.0, 0.0), Z, s);
    run_cell(
        &Cell {
            name: "plane-cylinder axis-parallel crossing",
            expected: &["line:Transversal", "line:Transversal"],
            complete: true,
        },
        |s| (plane(X, 0.5 * s), cyl(s)),
    );
    run_cell(
        &Cell {
            name: "plane-cylinder tangent line",
            expected: &["line:Tangential"],
            complete: true,
        },
        |s| (plane(X, s), cyl(s)),
    );
    run_cell(
        &Cell {
            name: "plane-cylinder tangent line opposite seam side",
            expected: &["line:Tangential"],
            complete: true,
        },
        |s| (plane(X, -s), cyl(s)),
    );
    run_cell(
        &Cell {
            name: "plane-cylinder miss",
            expected: &[],
            complete: true,
        },
        |s| (plane(X, 2.0 * s), cyl(s)),
    );
    run_cell(
        &Cell {
            name: "plane-cylinder perpendicular circle",
            expected: &["circle:Transversal"],
            complete: true,
        },
        |s| (plane(Z, 0.25 * s), cyl(s)),
    );
    run_cell(
        &Cell {
            name: "plane-cylinder oblique ellipse",
            expected: &["ellipse:Transversal"],
            complete: true,
        },
        |s| {
            let normal = (Z + X * 0.5).normalize().unwrap();
            (plane(normal, 0.0), cyl(s))
        },
    );
}

#[test]
fn sphere_sphere_matrix() {
    let o = Point3::new(0.0, 0.0, 0.0);
    run_cell(
        &Cell {
            name: "sphere-sphere crossing",
            expected: &["circle:Transversal"],
            complete: true,
        },
        |s| (sphere(o, s), sphere(Point3::new(1.5 * s, 0.0, 0.0), s)),
    );
    run_cell(
        &Cell {
            name: "sphere-sphere external tangent",
            expected: &["point:Tangential"],
            complete: true,
        },
        |s| (sphere(o, s), sphere(Point3::new(2.0 * s, 0.0, 0.0), s)),
    );
    run_cell(
        &Cell {
            name: "sphere-sphere internal tangent",
            expected: &["point:Tangential"],
            complete: true,
        },
        |s| (sphere(o, 2.0 * s), sphere(Point3::new(s, 0.0, 0.0), s)),
    );
    run_cell(
        &Cell {
            name: "sphere-sphere coincident",
            expected: &["coincident"],
            complete: true,
        },
        |s| (sphere(o, s), sphere(o, s)),
    );
    run_cell(
        &Cell {
            name: "sphere-sphere miss outside",
            expected: &[],
            complete: true,
        },
        |s| (sphere(o, s), sphere(Point3::new(3.0 * s, 0.0, 0.0), s)),
    );
    run_cell(
        &Cell {
            name: "sphere-sphere miss contained",
            expected: &[],
            complete: true,
        },
        |s| {
            (
                sphere(o, 3.0 * s),
                sphere(Point3::new(0.5 * s, 0.0, 0.0), s),
            )
        },
    );
}

#[test]
fn cylinder_cylinder_matrix() {
    let o = Point3::new(0.0, 0.0, 0.0);
    run_cell(
        &Cell {
            name: "cyl-cyl parallel crossing",
            expected: &["line:Transversal", "line:Transversal"],
            complete: true,
        },
        |s| {
            (
                cylinder(o, Z, s),
                cylinder(Point3::new(1.5 * s, 0.0, 0.0), Z, s),
            )
        },
    );
    run_cell(
        &Cell {
            name: "cyl-cyl external tangent",
            expected: &["line:Tangential"],
            complete: true,
        },
        |s| {
            (
                cylinder(o, Z, s),
                cylinder(Point3::new(2.0 * s, 0.0, 0.0), Z, s),
            )
        },
    );
    run_cell(
        &Cell {
            name: "cyl-cyl coincident",
            expected: &["coincident"],
            complete: true,
        },
        |s| (cylinder(o, Z, s), cylinder(o, Z, s)),
    );
    run_cell(
        &Cell {
            name: "cyl-cyl parallel miss",
            expected: &[],
            complete: true,
        },
        |s| {
            (
                cylinder(o, Z, s),
                cylinder(Point3::new(4.0 * s, 0.0, 0.0), Z, s),
            )
        },
    );
}

#[test]
fn sphere_cylinder_matrix() {
    let o = Point3::new(0.0, 0.0, 0.0);
    run_cell(
        &Cell {
            name: "coaxial sphere-cylinder crossing",
            expected: &["circle:Transversal", "circle:Transversal"],
            complete: true,
        },
        |s| (sphere(o, 2.0 * s), cylinder(o, Z, s)),
    );
    run_cell(
        &Cell {
            name: "coaxial sphere-cylinder tangent equator",
            expected: &["circle:Tangential"],
            complete: true,
        },
        |s| (sphere(o, s), cylinder(o, Z, s)),
    );
    run_cell(
        &Cell {
            name: "coaxial sphere-cylinder miss",
            expected: &[],
            complete: true,
        },
        |s| (sphere(o, 0.5 * s), cylinder(o, Z, s)),
    );
}

#[test]
fn uncertified_pairs_are_wrapped_honestly() {
    // Skew cylinders have no certified classification yet: the model must
    // say so — unclassified elements, and never `complete`.
    let ctx = OperationContext::new();
    let c1 = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Z, 1.0).unwrap();
    let c2 = CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), X, 1.0).unwrap();
    let result = intersect_surfaces(
        SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&c1)),
        SurfaceOperand::Analytic(AnalyticSurface::Cylinder(&c2)),
        &ctx,
    )
    .unwrap();
    assert!(
        !result.complete,
        "legacy delegation must not claim completeness"
    );
    for element in &result.elements {
        if let IntersectionElement::Curve(c) = element {
            assert_eq!(c.kind, ContactKind::Unclassified);
        }
    }
}
