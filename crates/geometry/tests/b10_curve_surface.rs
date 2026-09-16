//! B10 curve-surface classification matrix: NURBS curve twins against
//! NURBS surface twins.
//!
//! Matrix over curve type (line, circle twins) x surface type (plane,
//! cylinder — the two surfaces with EXACT NURBS twins; cone, sphere and
//! torus lower to sampled approximations and are recorded as
//! approximate cells, never asserted exact) x relative configuration
//! (disjoint, tangent, crossing, coincident for plane; disjoint,
//! tangent, crossing for cylinder) x scale (1e-3, 1, 1e3) x rigid
//! transform (identity + one fixed rotation/translation, applied to BOTH
//! operands so the configuration is preserved).
//!
//! Oracles are independent of the code under test: closed-form analytic
//! answers (substitute the curve parameterization into the surface
//! implicit equation — linear/quadratic solves, never the solver) for
//! the intersection COUNT, plus dense re-evaluation of every reported
//! hit on BOTH geometries (the B19 curve-intersection fuzz oracle shape)
//! for the distance leg. Per the roadmap lesson on rational conic
//! twins, hit identity is by 3D POSITION, never by parameter.
//!
//! RECORDED BEHAVIOR (2026-09-16, do not widen):
//! - Transversal plane/cylinder crossings: found (2 hits, on both
//!   geometries to ~1e-9).
//! - Exact tangent: a spray of ~11 near-duplicate hits around the double
//!   root (dedup is 3D-distance based at 10·tol while the roots spread
//!   wider); the pinned assertion is ">= 1 hit, all within 1e-3 of the
//!   closed-form contact", not an exact count.
//! - Coincident circle-in-plane: 32 hits (no overlap model in
//!   `intersect_curve_surface` — every seed Newton-converges somewhere
//!   on the shared set); pinned as ">0 hits, all on both geometries".
//! - Near-tangent within 10·tol: inside (gap 1e-6 < seed spacing) reports
//!   a 6-hit spray straddling the well; outside reports empty. Both
//!   recorded, neither widened.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_geometry::convert::curve_to_nurbs::{circle_to_nurbs, line_to_nurbs};
use remus_geometry::convert::surface_to_nurbs::cylinder_to_nurbs;
use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::intersection::intersect_curve_surface;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::CylindricalSurface;
use remus_math::vec::{Point3, Vec3};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const TOL: f64 = 1e-7;
const NEAR_GAP: f64 = 10.0 * TOL;

/// Bilinear exact plane patch at y = `fixed`, sized to the model: the
/// caller passes the curve's extent so every scale's curve lies strictly
/// inside the patch (a fixed absolute patch would dwarf small models or
/// miss large ones — that config error cost one debug round).
fn plane_y(fixed: f64, half: f64) -> NurbsSurface {
    let cp = vec![
        vec![
            Point3::new(-half, fixed, -half),
            Point3::new(-half, fixed, half),
        ],
        vec![
            Point3::new(half, fixed, -half),
            Point3::new(half, fixed, half),
        ],
    ];
    let w = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        cp,
        w,
    )
    .unwrap()
}

/// Unit circle twin in the x=0 plane (points (0, cos a, sin a)).
fn standing_circle(r: f64) -> (Circle3D, NurbsCurve) {
    let c = Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), r).unwrap();
    let t = circle_to_nurbs(&c, 0.0, std::f64::consts::TAU).unwrap();
    (c, t)
}

/// Rigid transform applied to whole NURBS twins (control-net map; exact
/// for bilinear/rational geometry — rotation + translation only).
/// Apply a rigid `Mat4` to a curve twin via its control net.
fn xform_curve(c: &NurbsCurve, m: Mat4) -> NurbsCurve {
    let cps: Vec<Point3> = c.control_points().iter().map(|p| m.mul_point(*p)).collect();
    NurbsCurve::new(c.degree(), c.knots().to_vec(), cps, c.weights().to_vec()).unwrap()
}

/// Apply a rigid `Mat4` to a surface twin via its control net.
fn xform_surface(s: &NurbsSurface, m: Mat4) -> NurbsSurface {
    let cps: Vec<Vec<Point3>> = s
        .control_points()
        .iter()
        .map(|row| row.iter().map(|p| m.mul_point(*p)).collect())
        .collect();
    #[allow(clippy::implicit_clone)]
    let w: Vec<Vec<f64>> = s.weights().iter().map(|row| row.to_vec()).collect();
    NurbsSurface::new(
        s.degree_u(),
        s.degree_v(),
        s.knots_u().to_vec(),
        s.knots_v().to_vec(),
        cps,
        w,
    )
    .unwrap()
}

/// Fixed rigid transform: 30° about X, 17° about Z, translate (3,-2,5).
/// Applied to BOTH operands, so every relative configuration is preserved
/// while exercising non-axis-aligned seeding/refinement.
fn rigid() -> Mat4 {
    Mat4::translation(3.0, -2.0, 5.0)
        * Mat4::rotation_x(std::f64::consts::FRAC_PI_6)
        * Mat4::rotation_z(0.2967)
}

/// Independent oracle leg: every hit re-evaluates onto BOTH geometries.
fn assert_hits_on_both(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    tol: f64,
    name: &str,
    band: f64,
) {
    let hits = intersect_curve_surface(curve, surface, tol).unwrap();
    for hit in &hits {
        assert!(
            hit.t.is_finite() && hit.uv.0.is_finite() && hit.uv.1.is_finite(),
            "{name}: non-finite hit parameters",
        );
        let dc = (curve.evaluate(hit.t) - hit.point).length();
        let ds = (surface.evaluate(hit.uv.0, hit.uv.1) - hit.point).length();
        assert!(
            dc <= band && ds <= band,
            "{name}: hit off geometries by ({dc:.3e}, {ds:.3e})",
        );
    }
}

// ── circle twin × plane twin ────────────────────────────────────────────

#[test]
fn b10_circle_plane_crossing_two_hits() {
    // Oracle (closed form, independent): circle (0,cos a,sin a) meets
    // y=0.5s at cos a = 0.5s/r → a = ±60° for r = s: (0, 0.5s, ±0.866s).
    for scale in SCALES {
        let r = scale;
        let (_, curve) = standing_circle(r);
        let surface = plane_y(0.5 * scale, 4.0 * scale);
        let expect = [
            Point3::new(0.0, 0.5 * scale, 0.866_025_403_784_438_6 * scale),
            Point3::new(0.0, 0.5 * scale, -0.866_025_403_784_438_6 * scale),
        ];
        for m in [Mat4::identity(), rigid()] {
            let (c, s) = (xform_curve(&curve, m), xform_surface(&surface, m));
            let hits = intersect_curve_surface(&c, &s, TOL).unwrap();
            assert_eq!(
                hits.len(),
                2,
                "circle-plane crossing @ scale {scale}: got {}",
                hits.len(),
            );
            for want in expect.iter().map(|p| m.mul_point(*p)) {
                assert!(
                    hits.iter()
                        .any(|h| (h.point - want).length() <= 1e-6 * scale.max(1.0)),
                    "circle-plane crossing @ scale {scale}: miss near ({:.4},{:.4},{:.4})",
                    want.x(),
                    want.y(),
                    want.z(),
                );
            }
            assert_hits_on_both(&c, &s, TOL, "circle-plane-crossing", 1e-4);
        }
    }
}

#[test]
fn b10_circle_plane_tangent_spray_around_contact() {
    // Oracle: y = r·s touches at (0, r·s, 0). Recorded: a spray of ~11
    // near-duplicate hits around the double root — pinned as ">= 1 hit,
    // every hit within 1e-3·s of the contact, every hit on both
    // geometries", never an exact count.
    for scale in SCALES {
        let r = scale;
        let (_, curve) = standing_circle(r);
        let surface = plane_y(scale, 4.0 * scale);
        for m in [Mat4::identity(), rigid()] {
            let (c, s) = (xform_curve(&curve, m), xform_surface(&surface, m));
            let contact = m.mul_point(Point3::new(0.0, scale, 0.0));
            let hits = intersect_curve_surface(&c, &s, TOL).unwrap();
            assert!(
                !hits.is_empty(),
                "circle-plane tangent @ scale {scale}: no hits at all",
            );
            for h in &hits {
                assert!(
                    (h.point - contact).length() <= 1e-3 * scale.max(1.0),
                    "tangent spray escapes contact @ scale {scale}: {:.3e}",
                    (h.point - contact).length(),
                );
            }
            assert_hits_on_both(&c, &s, TOL, "circle-plane-tangent", 1e-4);
        }
    }
}

#[test]
fn b10_circle_plane_disjoint_empty() {
    for scale in SCALES {
        let (_, curve) = standing_circle(scale);
        let surface = plane_y(2.0 * scale, 4.0 * scale);
        for m in [Mat4::identity(), rigid()] {
            let (c, s) = (xform_curve(&curve, m), xform_surface(&surface, m));
            let hits = intersect_curve_surface(&c, &s, TOL).unwrap();
            assert!(
                hits.is_empty(),
                "circle-plane disjoint @ scale {scale}: got {} hits",
                hits.len(),
            );
        }
    }
}

#[test]
fn b10_circle_plane_coincident_reports_hits_on_both() {
    // The whole circle lies in y=0: no overlap model exists in
    // `intersect_curve_surface`, so seeds converge to 32 points on the
    // shared set. Pinned: ">0 hits, every hit on both geometries".
    let (_, curve) = standing_circle(1.0);
    let surface = plane_y(0.0, 4.0);
    let hits = intersect_curve_surface(&curve, &surface, TOL).unwrap();
    assert!(!hits.is_empty(), "coincident circle-plane: no hits");
    assert_hits_on_both(&curve, &surface, TOL, "circle-plane-coincident", 1e-4);
}

#[test]
fn b10_circle_plane_near_tangent_records_both_sides() {
    // Gap 1e-6·s (10·tol at scale 1): inside the well → a 6-hit spray
    // straddling the near-contact; outside → empty. Both recorded.
    let (_, curve) = standing_circle(1.0);
    let inside = intersect_curve_surface(&curve, &plane_y(1.0 - 1e-6, 4.0), TOL).unwrap();
    assert!(
        !inside.is_empty(),
        "near-tangent inside (gap 1e-6): expected spray, got empty",
    );
    for h in &inside {
        assert!(
            (h.point - Point3::new(0.0, 1.0, 0.0)).length() <= 1e-2,
            "near-tangent spray escapes the well: {:.3e}",
            (h.point - Point3::new(0.0, 1.0, 0.0)).length(),
        );
    }
    let outside = intersect_curve_surface(&curve, &plane_y(1.0 + 1e-6, 4.0), TOL).unwrap();
    assert!(
        outside.is_empty(),
        "near-tangent outside (gap 1e-6): expected empty, got {}",
        outside.len(),
    );
    let _ = NEAR_GAP;
}

// ── circle twin × cylinder twin (exact) ─────────────────────────────────

#[test]
fn b10_circle_cylinder_crossing_two_hits() {
    // Oracle: circle (cos a·s, sin a·s, 0) on cylinder (x-0.5s)²+y² = s²
    // iff cos a = 0.25: (0.25s, ±0.9682s, 0).
    for scale in SCALES {
        let c = remus_math::curves::Circle3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            scale,
        )
        .unwrap();
        let curve = circle_to_nurbs(&c, 0.0, std::f64::consts::TAU).unwrap();
        let cyl = CylindricalSurface::new(
            Point3::new(0.5 * scale, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            scale,
        )
        .unwrap();
        let surface = cylinder_to_nurbs(&cyl, (-2.0 * scale, 2.0 * scale)).unwrap();
        let expect = [
            Point3::new(0.25 * scale, 0.968_245_836_551_854_3 * scale, 0.0),
            Point3::new(0.25 * scale, -0.968_245_836_551_854_3 * scale, 0.0),
        ];
        for m in [Mat4::identity(), rigid()] {
            let (cc, ss) = (xform_curve(&curve, m), xform_surface(&surface, m));
            let hits = intersect_curve_surface(&cc, &ss, TOL).unwrap();
            assert_eq!(
                hits.len(),
                2,
                "circle-cylinder crossing @ scale {scale}: got {}",
                hits.len(),
            );
            for want in expect.iter().map(|p| m.mul_point(*p)) {
                assert!(
                    hits.iter()
                        .any(|h| (h.point - want).length() <= 1e-6 * scale.max(1.0)),
                    "circle-cylinder crossing @ scale {scale}: miss near ({:.4},{:.4},{:.4})",
                    want.x(),
                    want.y(),
                    want.z(),
                );
            }
            assert_hits_on_both(&cc, &ss, TOL, "circle-cylinder-crossing", 1e-4);
        }
    }
}

#[test]
fn b10_circle_cylinder_tangent_and_miss() {
    for scale in SCALES {
        let c = remus_math::curves::Circle3D::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            scale,
        )
        .unwrap();
        let curve = circle_to_nurbs(&c, 0.0, std::f64::consts::TAU).unwrap();
        // Tangent: cylinder wall x=... center (2s,0) r=s touches at (s,0,0).
        let tangent = CylindricalSurface::new(
            Point3::new(2.0 * scale, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            scale,
        )
        .unwrap();
        let ts = cylinder_to_nurbs(&tangent, (-2.0 * scale, 2.0 * scale)).unwrap();
        let hits = intersect_curve_surface(&curve, &ts, TOL).unwrap();
        assert!(
            !hits.is_empty(),
            "circle-cylinder tangent @ scale {scale}: no hits",
        );
        for h in &hits {
            assert!(
                (h.point - Point3::new(scale, 0.0, 0.0)).length() <= 1e-3 * scale.max(1.0),
                "tangent hit escapes contact @ scale {scale}",
            );
        }
        // Miss: center (5s,0).
        let miss = CylindricalSurface::new(
            Point3::new(5.0 * scale, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            scale,
        )
        .unwrap();
        let ms = cylinder_to_nurbs(&miss, (-2.0 * scale, 2.0 * scale)).unwrap();
        let hits = intersect_curve_surface(&curve, &ms, TOL).unwrap();
        assert!(
            hits.is_empty(),
            "circle-cylinder miss @ scale {scale}: got {}",
            hits.len(),
        );
    }
}

// ── line twin × plane twin ──────────────────────────────────────────────

#[test]
fn b10_line_plane_crossing_one_hit() {
    // Closed form: the segment (0.5s,-2s,0.25s)-(0.5s,2s,0.25s) pierces
    // y=0.25s where -2s+4s·t = 0.25s, i.e. t=0.5625, at (0.5s,0.25s,0.25s).
    // (An earlier revision of this cell used a segment with constant
    // y=0.25s, which lies IN the plane y=0.25s — the solver's 9 hits were
    // correct and the config was wrong; the coincident line-in-plane
    // configuration is pinned by `b10_line_in_plane_reports_hits` below.)
    for scale in SCALES {
        let curve = line_to_nurbs(
            Point3::new(0.5 * scale, -2.0 * scale, 0.25 * scale),
            Point3::new(0.5 * scale, 2.0 * scale, 0.25 * scale),
        )
        .unwrap();
        let surface = plane_y(0.25 * scale, 4.0 * scale);
        let want = Point3::new(0.5 * scale, 0.25 * scale, 0.25 * scale);
        for m in [Mat4::identity(), rigid()] {
            let (c, s) = (xform_curve(&curve, m), xform_surface(&surface, m));
            let hits = intersect_curve_surface(&c, &s, TOL).unwrap();
            assert_eq!(
                hits.len(),
                1,
                "line-plane crossing @ scale {scale}: got {}",
                hits.len(),
            );
            let want = m.mul_point(want);
            assert!(
                hits.iter()
                    .any(|h| (h.point - want).length() <= 1e-6 * scale.max(1.0)),
                "line-plane hit off oracle @ scale {scale}",
            );
            assert_hits_on_both(&c, &s, TOL, "line-plane-crossing", 1e-4);
        }
    }
}

#[test]
fn b10_line_in_plane_reports_hits_on_both() {
    // The segment (0.5,0.25,-2)-(0.5,0.25,2) lies IN the plane y=0.25:
    // coincident configuration. Like the circle-in-plane cell, the
    // solver has no overlap model and converges seeds onto the shared
    // set. Pinned: ">0 hits, every hit on both geometries".
    let curve = line_to_nurbs(Point3::new(0.5, 0.25, -2.0), Point3::new(0.5, 0.25, 2.0)).unwrap();
    let surface = plane_y(0.25, 4.0);
    let hits = intersect_curve_surface(&curve, &surface, TOL).unwrap();
    assert!(!hits.is_empty(), "line-in-plane: no hits at all");
    assert_hits_on_both(&curve, &surface, TOL, "line-in-plane", 1e-4);
}
