//! Torus cuts and classification far from the line's origin, and on very
//! short box edges.
//!
//! The line–torus quartic was formed about the caller's origin and in units
//! of the caller's direction. The boolean section trimmer passes a box edge's
//! start and raw edge vector, and the classifiers pass a query point and a
//! unit ray, so a torus thousands of tube sizes along a long edge or ray lost
//! crossings to the quartic solver's absolute filters, and an edge shorter
//! than about 3.2e-4 was solved as a cubic. The cut then came back valid but
//! uncut (the torus ignored), and grooved solids misclassified interior
//! points. Volumes here are checked against slicing integrals; points
//! against their known class.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, PreparedSolid, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_torus};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn torus_at(topo: &mut Topology, big: f64, small: f64, c: Point3) -> SolidId {
    let t = make_torus(topo, big, small, 32).unwrap();
    transform_solid(topo, t, &Mat4::translation(c.x(), c.y(), c.z())).unwrap();
    t
}

/// `(faces, torus faces)` of a solid.
fn census(topo: &Topology, s: SolidId) -> (usize, usize) {
    let faces = remus_topology::explorer::solid_faces(topo, s).unwrap();
    let torus = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .count();
    (faces.len(), torus)
}

/// Volume of the torus (axis z, centre `(_, y0, h + z0)`) inside the box's
/// `y > 0, z < h` corner, for a torus clear of the box's other faces: the
/// torus sliced across its axis into annuli, each clipped by the half-plane
/// `y > 0` in closed form, integrated over the tube angle by the midpoint
/// rule.
fn notch_volume(big: f64, small: f64, y0: f64, z0: f64) -> f64 {
    use std::f64::consts::PI;
    // Area of the disc of radius `rho` centred at `y0` with `y > 0`.
    let half_disc = |rho: f64| -> f64 {
        if y0 >= rho {
            PI * rho * rho
        } else if y0 <= -rho {
            0.0
        } else {
            (rho * rho).mul_add((-y0 / rho).acos(), y0 * rho.mul_add(rho, -y0 * y0).sqrt())
        }
    };
    // Slice at height `-small cos(th)` from the centre; `z < h` keeps
    // `th <= acos(z0 / small)`.
    let th_max = (z0 / small).clamp(-1.0, 1.0).acos();
    let n = 400_000_u32;
    let mut acc = 0.0;
    for i in 0..n {
        let th = th_max * (f64::from(i) + 0.5) / f64::from(n);
        let s = small * th.sin();
        acc += (half_disc(big + s) - half_disc(big - s)) * s;
    }
    acc * th_max / f64::from(n)
}

/// Box `[0, l] x [0, w] x [0, h]` cut by a torus `(big, small)` centred at
/// `(xc, y0, h + z0)`: the result, or the error text.
#[allow(clippy::too_many_arguments)]
fn edge_notch(
    l: f64,
    w: f64,
    h: f64,
    big: f64,
    small: f64,
    xc: f64,
    y0: f64,
    z0: f64,
) -> (Topology, Result<SolidId, String>) {
    let mut topo = Topology::new();
    let b = make_box(&mut topo, l, w, h).unwrap();
    let t = torus_at(&mut topo, big, small, Point3::new(xc, y0, h + z0));
    let result = boolean(&mut topo, BooleanOp::Cut, b, t).map_err(|e| e.to_string());
    (topo, result)
}

/// The volume removed by a successful notch cut, after checking the result
/// is a valid solid holding the one torus face.
fn removed(topo: &Topology, s: SolidId, l: f64, w: f64, h: f64, deflection: f64) -> f64 {
    assert!(validate_solid(topo, s).unwrap().is_valid());
    let (faces, torus) = census(topo, s);
    assert_eq!((faces, torus), (8, 1), "census");
    l * w * h - solid_volume(topo, s, deflection).unwrap()
}

/// The reported case: a 10000-long box notched by a torus 9440 along its top
/// edge came back valid with nothing removed; 9000 and 4500 were refused.
#[test]
fn far_torus_notch_on_a_long_box_edge_is_cut() {
    let (big, small, y0, z0) = (1.5, 0.5, 0.3, 0.2);
    let expect = notch_volume(big, small, y0, z0);
    assert!((expect - 1.053_611).abs() < 1e-6, "{expect}");
    for xc in [9_440.053_469, 9000.0, 4500.0] {
        let (topo, result) = edge_notch(10_000.0, 10.0, 10.0, big, small, xc, y0, z0);
        let s = result.unwrap_or_else(|e| panic!("xc {xc}: {e}"));
        let got = removed(&topo, s, 10_000.0, 10.0, 10.0, 5e-4);
        assert!(
            (got - expect).abs() < 1e-5 * expect,
            "xc {xc}: removed {got}, expected {expect}"
        );
    }
}

/// Torus notches spread along a long box edge: every one is either cut
/// exactly or refused, never returned as a valid solid of the wrong volume.
#[test]
fn torus_notches_along_long_box_edges_are_exact_or_refused() {
    let (big, small) = (1.5, 0.5);
    for (y0, z0) in [(0.3, 0.2), (0.0, 0.2), (-0.4, 0.15)] {
        let expect = notch_volume(big, small, y0, z0);
        for l in [3000.0, 10_000.0] {
            let (mut cut, mut refused) = (0, 0);
            for i in 0..8_u32 {
                let margin = big + small + 1.0;
                let f = (f64::from(i) * 0.618_033_988_749_895).fract();
                let xc = f.mul_add(2.0f64.mul_add(-margin, l), margin);
                let (topo, result) = edge_notch(l, 10.0, 10.0, big, small, xc, y0, z0);
                let Ok(s) = result else {
                    refused += 1;
                    continue;
                };
                let got = removed(&topo, s, l, 10.0, 10.0, 5e-4);
                assert!(
                    (got - expect).abs() < 1e-4 * expect,
                    "y0 {y0} z0 {z0} L {l} xc {xc}: removed {got}, expected {expect}"
                );
                cut += 1;
            }
            // Today every placement cuts; refusals stay allowed, wrong
            // solids do not.
            assert!(cut > 0, "y0 {y0} z0 {z0} L {l}: {refused} refused");
        }
    }
}

/// A rib whose long top edges are 3e-4 (raw edge vector `|dir|⁴ < 1e-14`)
/// notched by a small torus came back valid with the wrong volume.
#[test]
fn short_edge_rib_notch_has_the_exact_volume() {
    let (l, w, h, big, small) = (3e-4, 1e-3, 1e-3, 1e-4, 4e-5);
    let (xc, y0, z0) = (
        1.474_922_359_499_621_7e-4,
        -9.411_254_879_999_995e-6,
        1.5e-5,
    );
    // The torus spans x in [xc - R - r, xc + R + r], inside [0, l].
    assert!(xc - big - small > 0.0 && xc + big + small < l);
    let expect = notch_volume(big, small, y0, z0);
    let (topo, result) = edge_notch(l, w, h, big, small, xc, y0, z0);
    let s = result.unwrap();
    let got = removed(&topo, s, l, w, h, 1e-5 * small);
    assert!(
        (got - expect).abs() < 1e-4 * expect,
        "removed {got:e}, expected {expect:e}"
    );
}

/// Half-embedded torus groove on a face of the cube `[0, size]³` with normal
/// `axis`, its core circle through `exit`.
fn groove(topo: &mut Topology, axis: usize, exit: Point3, big: f64, small: f64) -> SolidId {
    let t = make_torus(topo, big, small, 32).unwrap();
    let rot = match axis {
        0 => Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
        1 => Mat4::rotation_x(-std::f64::consts::FRAC_PI_2),
        _ => Mat4::translation(0.0, 0.0, 0.0),
    };
    let mut c = [exit.x(), exit.y(), exit.z()];
    c[(axis + 1) % 3] -= big;
    transform_solid(topo, t, &rot).unwrap();
    transform_solid(topo, t, &Mat4::translation(c[0], c[1], c[2])).unwrap();
    t
}

/// Where the ray from `q` along `d` leaves the cube `[0, size]³`, and the
/// axis of the face it leaves through.
fn exit_of(q: Point3, d: [f64; 3], size: f64) -> (usize, Point3) {
    let qa = [q.x(), q.y(), q.z()];
    let mut best = (f64::INFINITY, 0);
    for k in 0..3 {
        let t = if d[k] > 0.0 {
            (size - qa[k]) / d[k]
        } else if d[k] < 0.0 {
            -qa[k] / d[k]
        } else {
            f64::INFINITY
        };
        if t < best.0 {
            best = (t, k);
        }
    }
    let t = best.0;
    (
        best.1,
        Point3::new(
            d[0].mul_add(t, qa[0]),
            d[1].mul_add(t, qa[1]),
            d[2].mul_add(t, qa[2]),
        ),
    )
}

/// Large cubes grooved by small tori exactly where the point classifier's
/// first two vote rays leave them: interior points were read as outside (58
/// of 400 at size 3000).
#[test]
fn grooved_large_cube_classifies_interior_points() {
    // The classifier's first two vote directions.
    let rays = [
        [
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
            0.350_889_803_483_932_2,
        ],
        [
            -0.350_889_803_483_932_2,
            0.573_576_436_351_046,
            0.740_535_693_464_567_5,
        ],
    ];
    for size in [3000.0, 5000.0] {
        let mut topo = Topology::new();
        let mut cube = make_box(&mut topo, size, size, size).unwrap();
        let q = Point3::new(0.3 * size, 0.3 * size, 0.3 * size);
        for d in rays {
            let (axis, e) = exit_of(q, d, size);
            let g = groove(&mut topo, axis, e, 1.5, 0.5);
            cube = boolean(&mut topo, BooleanOp::Cut, cube, g).unwrap();
        }
        assert!(validate_solid(&topo, cube).unwrap().is_valid());
        assert_eq!(census(&topo, cube).1, 2, "size {size}: torus faces");
        let opts = ClassifyOptions::default();
        let prepared = PreparedSolid::prepare(&topo, cube).unwrap();
        for i in 0..40_u32 {
            let k = f64::from(i);
            let p = Point3::new(
                0.01f64.mul_add(k, q.x()),
                0.007f64.mul_add(k, q.y()),
                (-0.005f64).mul_add(k, q.z()),
            );
            assert_eq!(
                classify_point(&topo, cube, p, &opts).unwrap(),
                PointClassification::Inside,
                "size {size}: {p:?}"
            );
            assert_eq!(
                prepared.classify_point(p, &opts).unwrap(),
                PointClassification::Inside,
                "size {size}: prepared {p:?}"
            );
        }
    }
}
