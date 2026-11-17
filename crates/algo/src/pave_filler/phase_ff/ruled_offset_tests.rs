//! A wall ruled between two offsets of a cylinder meets it along an
//! iso-curve, and only a wall that is such a family takes that shortcut.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceSurface;

use super::exact_ruled_offset_section;

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn cylinder(x: f64, radius: f64) -> FaceSurface {
    FaceSurface::Cylinder(CylindricalSurface::new(Point3::new(x, 0.0, 0.0), Z, radius).unwrap())
}

/// A wall linear in v from `rows[i].0` to `rows[i].1`, degree one along u.
fn wall(knots_u: Vec<f64>, rows: &[(Point3, Point3)]) -> NurbsSurface {
    NurbsSurface::new(
        1,
        1,
        knots_u,
        vec![0.0, 0.0, 1.0, 1.0],
        rows.iter().map(|&(a, b)| vec![a, b]).collect(),
        vec![vec![1.0, 1.0]; rows.len()],
    )
    .unwrap()
}

#[test]
fn a_radial_offset_family_meets_the_cylinder_on_its_iso_curve() {
    // An arc of the radius-10 circle's polygon, scaled to radii 9 and 11:
    // every ruling is radial and crosses radius 10 half way.
    let rows: Vec<(Point3, Point3)> = (0..4)
        .map(|i| {
            let a = 0.2 * f64::from(i);
            let (s, c) = a.sin_cos();
            (
                Point3::new(9.0 * c, 9.0 * s, f64::from(i)),
                Point3::new(11.0 * c, 11.0 * s, f64::from(i)),
            )
        })
        .collect();
    // Control points on the circle are not the circle: the polygon's sides
    // dip inside it.
    let surface = wall(vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0], &rows);
    let tol = Tolerance::loose();
    let section = exact_ruled_offset_section(&cylinder(0.0, 10.0), &surface, tol).unwrap();
    // The polygon's chords leave radius 10 by up to 10·(1 − cos 0.1) = 0.05,
    // beyond any fit band: the shortcut refuses rather than guess.
    assert!(section.is_none());

    // The exact radial family of a straight generator: a line along Z at
    // angle 0, from radius 9 to 11, meets radius 10 on the line x = 10.
    let rows = [
        (Point3::new(9.0, 0.0, 0.0), Point3::new(11.0, 0.0, 0.0)),
        (Point3::new(9.0, 0.0, 5.0), Point3::new(11.0, 0.0, 5.0)),
    ];
    let surface = wall(vec![0.0, 0.0, 1.0, 1.0], &rows);
    let section = exact_ruled_offset_section(&cylinder(0.0, 10.0), &surface, Tolerance::new())
        .unwrap()
        .expect("a radial family takes the iso-curve");
    let EdgeCurve::NurbsCurve(curve) = &section[0].curve else {
        panic!("a NURBS iso-curve");
    };
    for p in curve.control_points() {
        assert!((p.x() - 10.0).abs() < 1e-12 && p.y().abs() < 1e-12, "{p:?}");
    }
}

#[test]
fn a_short_knot_span_off_the_family_is_never_skipped() {
    // One row of a degree-one wall, inside a span a thousandth long,
    // starts on the cylinder and ends beyond it: the crossing there is not
    // the rest of the wall's. Sampling the whole domain uniformly misses
    // it; every span is sampled, so the shortcut refuses.
    let row = |u: f64, r0: f64| (Point3::new(r0, 0.0, u), Point3::new(r0 + 2.0, 0.0, u));
    let surface = wall(
        vec![0.0, 0.0, 0.001, 0.002, 1.0, 1.0],
        &[
            row(0.0, 9.0),
            row(0.001, 10.0),
            row(0.002, 9.0),
            row(1.0, 9.0),
        ],
    );
    let section =
        exact_ruled_offset_section(&cylinder(0.0, 10.0), &surface, Tolerance::new()).unwrap();
    assert!(section.is_none());
}

#[test]
fn a_far_translated_pair_keeps_the_band_of_its_own_size() {
    // A radius-2000 cylinder far along X and a strip whose crossings slide
    // from 0.5 to 0.45 along it: not an offset family, so refused. A band
    // scaled by world coordinates (1e9) would have accepted it.
    let x = 1e9;
    let surface = wall(
        vec![0.0, 0.0, 1.0, 1.0],
        &[
            (
                Point3::new(x + 1000.0, 0.0, 0.0),
                Point3::new(x + 3000.0, 0.0, 0.0),
            ),
            (
                Point3::new(x + 1100.0, 0.0, 1.0),
                Point3::new(x + 3100.0, 0.0, 1.0),
            ),
        ],
    );
    let section =
        exact_ruled_offset_section(&cylinder(x, 2000.0), &surface, Tolerance::new()).unwrap();
    assert!(section.is_none());
}
